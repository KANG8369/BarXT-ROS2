"""Keller LD I2C pressure transmitter driver.

This module follows the Blue Robotics KellerLD-python measurement flow while
keeping the hardware access injectable for tests.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import struct
import time
from typing import Optional, Protocol, Sequence, Union


class I2CBus(Protocol):
    def write_byte(self, address: int, value: int) -> None:
        ...

    def read_i2c_block_data(self, address: int, command: int, length: int) -> Sequence[int]:
        ...


@dataclass(frozen=True)
class KellerLDMeasurement:
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
    DEFAULT_I2C_ADDRESS = 0x40
    REQUEST_MEASUREMENT = 0xAC
    P_MODES = (
        "PR Mode, Vented Gauge",
        "PA Mode, Sealed Gauge",
        "PAA Mode, Absolute Gauge",
    )
    P_MODE_OFFSETS = (1.01325, 1.0, 0.0)

    def __init__(
        self,
        bus: Union[int, I2CBus] = 1,
        address: int = DEFAULT_I2C_ADDRESS,
        conversion_delay_s: float = 0.010,
    ) -> None:
        self.address = address
        self.conversion_delay_s = conversion_delay_s
        self._bus = self._open_bus(bus)
        self.p_mode = ""
        self.p_mode_offset = 0.0
        self.p_min: Optional[float] = None
        self.p_max: Optional[float] = None
        self.year: Optional[int] = None
        self.month: Optional[int] = None
        self.day: Optional[int] = None
        self._last_measurement: Optional[KellerLDMeasurement] = None

    def _open_bus(self, bus: Union[int, I2CBus]) -> I2CBus:
        if hasattr(bus, "write_byte") and hasattr(bus, "read_i2c_block_data"):
            return bus  # type: ignore[return-value]

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
            return smbus.SMBus(int(bus))
        except OSError as exc:
            hint = "Available busses are listed as /dev/i2c*."
            if hasattr(os, "uname") and os.uname().nodename == "raspberrypi":
                hint += " Enable the I2C interface using raspi-config."
            raise KellerLDError(f"I2C bus {bus} is not available. {hint}") from exc

    def init(self) -> bool:
        scaling0 = self._read_register_word(0x12)
        p_mode_id = scaling0 & 0b11
        self.p_mode = self.P_MODES[p_mode_id]
        self.set_reference_pressure(self.P_MODE_OFFSETS[p_mode_id])
        self.year = scaling0 >> 11
        self.month = (scaling0 & 0b0000011110000000) >> 7
        self.day = (scaling0 & 0b0000000001111100) >> 2

        p_min_raw = self._read_register_word(0x13) << 16 | self._read_register_word(0x14)
        p_max_raw = self._read_register_word(0x15) << 16 | self._read_register_word(0x16)
        self.p_min = self._uint32_to_float(p_min_raw)
        self.p_max = self._uint32_to_float(p_max_raw)
        return True

    def _read_register_word(self, register: int) -> int:
        self._bus.write_byte(self.address, register)
        time.sleep(0.001)
        data = self._bus.read_i2c_block_data(self.address, 0, 3)
        if len(data) != 3:
            raise KellerLDError(f"expected 3 bytes from register 0x{register:02x}")
        self._validate_status(data[0])
        return data[1] << 8 | data[2]

    @staticmethod
    def _uint32_to_float(value: int) -> float:
        return struct.unpack("f", struct.pack("I", value))[0]

    def set_reference_pressure(self, pressure_bar: float) -> None:
        self.p_mode_offset = pressure_bar

    def read(self) -> KellerLDMeasurement:
        if self.p_min is None or self.p_max is None:
            raise KellerLDNotInitializedError("call init() before read()")

        self._bus.write_byte(self.address, self.REQUEST_MEASUREMENT)
        time.sleep(self.conversion_delay_s)
        data = self._bus.read_i2c_block_data(self.address, 0, 5)
        if len(data) != 5:
            raise KellerLDError("expected 5 bytes from measurement read")

        status = data[0]
        self._validate_status(status)
        raw_pressure = data[1] << 8 | data[2]
        raw_temperature = data[3] << 8 | data[4]
        pressure_bar = (
            (raw_pressure - 16384) * (self.p_max - self.p_min) / 32768
            + self.p_min
            + self.p_mode_offset
        )
        temperature_c = self.raw_temperature_to_c(raw_temperature)
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
        return ((raw_temperature >> 4) - 24) * 0.05 - 50

    @staticmethod
    def _validate_status(status: int) -> None:
        mode = (status & (0b11 << 3)) >> 3
        if mode != 0:
            raise KellerLDStatusError(f"invalid status mode {mode}, expected 0")
        if status & (1 << 2):
            raise KellerLDStatusError("memory checksum error")

    def pressure(self) -> float:
        if self._last_measurement is None:
            raise KellerLDError("call read() before pressure()")
        return self._last_measurement.pressure_bar

    def temperature(self) -> float:
        if self._last_measurement is None:
            raise KellerLDError("call read() before temperature()")
        return self._last_measurement.temperature_c

