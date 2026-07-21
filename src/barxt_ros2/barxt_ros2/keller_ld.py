"""Keller LD I2C 압력 송신기 드라이버.

Blue Robotics KellerLD-python의 측정 순서를 따르되, 테스트에서는 실제 I2C
하드웨어 대신 mock bus를 주입할 수 있게 구성했다.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import struct
import time
from typing import Optional, Protocol, Sequence, Union


class I2CBus(Protocol):
    # smbus/smbus2 객체와 테스트용 mock 객체가 만족해야 하는 최소 인터페이스이다.
    # Protocol을 사용하면 실제 타입 상속 없이도 같은 메서드 형태만 맞추면 된다.
    def write_byte(self, address: int, value: int) -> None:
        ...

    def read_i2c_block_data(self, address: int, command: int, length: int) -> Sequence[int]:
        ...


@dataclass(frozen=True)
class KellerLDMeasurement:
    # KellerLD.read()가 반환하는 단일 측정 샘플이다.
    # 변환된 물리량과 validation/debug에 필요한 raw 값을 함께 보관한다.
    pressure_bar: float
    temperature_c: float
    raw_pressure: int
    raw_temperature: int
    status: int


class KellerLDError(RuntimeError):
    """Base class for Keller LD driver failures."""


class KellerLDStatusError(KellerLDError):
    """Raised when the status byte reports invalid data."""


class KellerLDNotInitializedError(KellerLDError):
    """Raised when read() is called before init()."""


class KellerLD:
    # BarXT/Keller LD 센서의 기본 I2C 주소이다. 현재 실제 하드웨어도 0x40에서 확인됐다.
    DEFAULT_I2C_ADDRESS = 0x40
    # 센서에 measurement conversion을 요청하는 command byte이다.
    REQUEST_MEASUREMENT = 0xAC
    # 센서 EEPROM의 scaling register에서 읽은 pressure mode id를 사람이 읽기 쉬운 이름으로 매핑한다.
    P_MODES = (
        "PR Mode, Vented Gauge",
        "PA Mode, Sealed Gauge",
        "PAA Mode, Absolute Gauge",
    )
    # mode별 기준 압력 offset이다. raw pressure 변환 후 이 값을 더해 최종 bar 값을 만든다.
    P_MODE_OFFSETS = (1.01325, 1.0, 0.0)

    def __init__(
        self,
        bus: Union[int, I2CBus] = 1,
        address: int = DEFAULT_I2C_ADDRESS,
        conversion_delay_s: float = 0.010,
    ) -> None:
        self.address = address
        # 데이터시트/레퍼런스 드라이버 기준으로 measurement command 뒤 변환 완료를 기다리는 시간이다.
        self.conversion_delay_s = conversion_delay_s
        self._bus = self._open_bus(bus)
        # init()에서 센서 EEPROM 값을 읽어 채워지는 보정/식별 정보이다.
        self.p_mode = ""
        self.p_mode_offset = 0.0
        self.p_min: Optional[float] = None
        self.p_max: Optional[float] = None
        self.year: Optional[int] = None
        self.month: Optional[int] = None
        self.day: Optional[int] = None
        self._last_measurement: Optional[KellerLDMeasurement] = None

    def _open_bus(self, bus: Union[int, I2CBus]) -> I2CBus:
        # 테스트에서는 bus 번호 대신 MockBus 객체를 직접 넘긴다.
        # 필요한 메서드를 이미 갖고 있으면 하드웨어 bus open 없이 그대로 사용한다.
        if hasattr(bus, "write_byte") and hasattr(bus, "read_i2c_block_data"):
            return bus  # type: ignore[return-value]

        # 실제 하드웨어에서는 Raspberry Pi/임베디드 Linux 환경에서 흔한 smbus를 우선 사용하고,
        # 설치 환경에 따라 smbus2도 fallback으로 허용한다.
        try:
            import smbus  # type: ignore
        except ImportError:
            try:
                import smbus2 as smbus  # type: ignore
            except ImportError as exc:
                raise KellerLDError(
                    "python3-smbus or smbus2 is required for hardware I2C access"
                ) from exc

        try:
            # bus가 7이면 /dev/i2c-7을 여는 것과 같은 의미이다.
            return smbus.SMBus(int(bus))
        except OSError as exc:
            hint = "Available busses are listed as /dev/i2c*."
            if hasattr(os, "uname") and os.uname().nodename == "raspberrypi":
                hint += " Enable the I2C interface using raspi-config."
            raise KellerLDError(f"I2C bus {bus} is not available. {hint}") from exc

    def init(self) -> bool:
        # 0x12 scaling register에는 pressure mode와 calibration date가 들어 있다.
        # 이 값은 이후 pressure offset 선택에 필요하다.
        scaling0 = self._read_register_word(0x12)
        p_mode_id = scaling0 & 0b11
        if p_mode_id >= len(self.P_MODES):
            raise KellerLDError(f"unsupported pressure mode id {p_mode_id}")
        self.p_mode = self.P_MODES[p_mode_id]
        self.set_reference_pressure(self.P_MODE_OFFSETS[p_mode_id])
        # Keller 레퍼런스 드라이버와 동일한 bit mask로 calibration date를 분리한다.
        self.year = scaling0 >> 11
        self.month = (scaling0 & 0b0000011110000000) >> 7
        self.day = (scaling0 & 0b0000000001111100) >> 2

        # p_min/p_max는 각각 32-bit float가 상위 word와 하위 word 두 register에 나뉘어 저장된다.
        # raw pressure count를 실제 bar 범위로 선형 변환할 때 반드시 필요한 보정값이다.
        p_min_raw = self._read_register_word(0x13) << 16 | self._read_register_word(0x14)
        p_max_raw = self._read_register_word(0x15) << 16 | self._read_register_word(0x16)
        self.p_min = self._uint32_to_float(p_min_raw)
        self.p_max = self._uint32_to_float(p_max_raw)
        return True

    def _read_register_word(self, register: int) -> int:
        # Keller LD는 먼저 읽고 싶은 register 주소를 1 byte write로 지정한 뒤,
        # 이어서 status byte + 16-bit register data 총 3 bytes를 읽는 방식이다.
        self._bus.write_byte(self.address, register)
        time.sleep(0.001)
        data = self._bus.read_i2c_block_data(self.address, 0, 3)
        if len(data) != 3:
            raise KellerLDError(f"expected 3 bytes from register 0x{register:02x}")
        # register read에서도 status byte를 검사해 command mode나 checksum 오류를 빠르게 잡는다.
        self._validate_status(data[0])
        return data[1] << 8 | data[2]

    @staticmethod
    def _uint32_to_float(value: int) -> float:
        # 센서 EEPROM에는 IEEE-754 float가 32-bit unsigned int 형태로 저장되어 있다.
        # struct로 같은 bit pattern을 float로 재해석한다.
        return struct.unpack("f", struct.pack("I", value))[0]

    def set_reference_pressure(self, pressure_bar: float) -> None:
        # pressure mode에 맞는 기준 압력을 offset으로 저장한다.
        # 최종 pressure_bar 계산 시 raw 변환값에 더해진다.
        self.p_mode_offset = pressure_bar

    def read(self) -> KellerLDMeasurement:
        # p_min/p_max 없이 read하면 압력 변환식의 scale을 알 수 없으므로 init()을 강제한다.
        if self.p_min is None or self.p_max is None:
            raise KellerLDNotInitializedError("call init() before read()")

        # measurement command를 보내고 conversion delay만큼 기다린 뒤,
        # status + pressure raw 2 bytes + temperature raw 2 bytes를 읽는다.
        self._bus.write_byte(self.address, self.REQUEST_MEASUREMENT)
        time.sleep(self.conversion_delay_s)
        data = self._bus.read_i2c_block_data(self.address, 0, 5)
        if len(data) != 5:
            raise KellerLDError("expected 5 bytes from measurement read")

        status = data[0]
        self._validate_status(status)
        raw_pressure = data[1] << 8 | data[2]
        raw_temperature = data[3] << 8 | data[4]
        # Keller LD pressure raw count의 nominal 범위는 16384~49152이다.
        # 32768 count 폭을 p_min~p_max 압력 범위에 선형 매핑한 뒤 mode offset을 더한다.
        pressure_bar = (
            (raw_pressure - 16384) * (self.p_max - self.p_min) / 32768
            + self.p_min
            + self.p_mode_offset
        )
        temperature_c = self.raw_temperature_to_c(raw_temperature)
        # 마지막 측정값을 캐시해 pressure()/temperature() convenience method에서 재사용한다.
        self._last_measurement = KellerLDMeasurement(
            pressure_bar=pressure_bar,
            temperature_c=temperature_c,
            raw_pressure=raw_pressure,
            raw_temperature=raw_temperature,
            status=status,
        )
        return self._last_measurement

    @staticmethod
    def raw_temperature_to_c(raw_temperature: int) -> float:
        # raw temperature의 하위 4 bit는 fractional/status 성격이라 우측 shift로 제거한다.
        # 이후 Keller 레퍼런스 식에 따라 섭씨 온도로 변환한다.
        return ((raw_temperature >> 4) - 24) * 0.05 - 50

    @staticmethod
    def _validate_status(status: int) -> None:
        # Keller LD response status must start with 0b01.  Without this check,
        # a bus held low (0x00) or an otherwise malformed response can be
        # mistaken for normal-mode sensor data.
        framing = (status >> 6) & 0b11
        if framing != 0b01:
            raise KellerLDStatusError(
                f"invalid status framing {framing:02b}, expected 01"
            )
        # status byte의 mode bit[4:3]가 0이면 normal mode이다.
        # 다른 값이면 command/reserved mode로 판단해 해당 샘플을 사용하지 않는다.
        mode = (status & (0b11 << 3)) >> 3
        if mode != 0:
            raise KellerLDStatusError(f"invalid status mode {mode}, expected 0")
        # bit 2가 set이면 센서 내부 memory checksum 오류이다.
        if status & (1 << 2):
            raise KellerLDStatusError("memory checksum error")

    def pressure(self) -> float:
        # 레퍼런스 드라이버와 유사한 convenience accessor이다.
        # read()가 한 번도 성공하지 않았다면 반환할 최신 값이 없다.
        if self._last_measurement is None:
            raise KellerLDError("call read() before pressure()")
        return self._last_measurement.pressure_bar

    def temperature(self) -> float:
        # 마지막 read()에서 계산된 온도를 반환한다.
        if self._last_measurement is None:
            raise KellerLDError("call read() before temperature()")
        return self._last_measurement.temperature_c
