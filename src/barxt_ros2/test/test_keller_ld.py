import struct

import pytest

from barxt_ros2.keller_ld import KellerLD, KellerLDError, KellerLDStatusError


class MockBus:
    def __init__(self, reads):
        self.reads = list(reads)
        self.writes = []

    def write_byte(self, address, value):
        self.writes.append((address, value))

    def read_i2c_block_data(self, address, command, length):
        return self.reads.pop(0)


def float_register_reads(value):
    raw = struct.unpack("I", struct.pack("f", value))[0]
    ms_word = (raw >> 16) & 0xFFFF
    ls_word = raw & 0xFFFF
    return [
        [0x40, (ms_word >> 8) & 0xFF, ms_word & 0xFF],
        [0x40, (ls_word >> 8) & 0xFF, ls_word & 0xFF],
    ]


def init_reads(p_min=0.0, p_max=2.0):
    return [
        [0x40, 0, 1],
        *float_register_reads(p_min),
        *float_register_reads(p_max),
    ]


def raw_temperature_for_celsius(temperature_c):
    return int(((temperature_c + 50.0) / 0.05) + 24) << 4


def test_read_success_parses_pressure_and_temperature():
    raw_temperature = raw_temperature_for_celsius(25.0)
    bus = MockBus(
        [
            *init_reads(),
            [0x40, 0x80, 0x00, (raw_temperature >> 8) & 0xFF, raw_temperature & 0xFF],
        ]
    )
    sensor = KellerLD(bus=bus, conversion_delay_s=0.0)

    assert sensor.init()
    measurement = sensor.read()

    assert measurement.raw_pressure == 32768
    assert measurement.pressure_bar == pytest.approx(2.0)
    assert measurement.temperature_c == pytest.approx(25.0)
    assert sensor.pressure() == pytest.approx(2.0)
    assert sensor.temperature() == pytest.approx(25.0)


def test_reserved_mode_status_raises_on_register_read():
    # Mode 2 is reserved, including while the EEPROM is being read.
    bus = MockBus([[0b01010000, 0, 1]])
    sensor = KellerLD(bus=bus, conversion_delay_s=0.0)

    with pytest.raises(KellerLDStatusError, match="invalid status mode"):
        sensor.init()


def test_checksum_status_raises_on_measurement():
    raw_temperature = raw_temperature_for_celsius(25.0)
    bus = MockBus(
        [
            *init_reads(),
            [0b01000100, 0x80, 0x00, (raw_temperature >> 8) & 0xFF, raw_temperature & 0xFF],
        ]
    )
    sensor = KellerLD(bus=bus, conversion_delay_s=0.0)

    assert sensor.init()
    with pytest.raises(KellerLDStatusError, match="memory checksum error"):
        sensor.read()


def test_invalid_status_framing_raises():
    bus = MockBus([[0x00, 0, 1]])
    sensor = KellerLD(bus=bus, conversion_delay_s=0.0)

    with pytest.raises(KellerLDStatusError, match="status framing"):
        sensor.init()


def test_command_mode_status_is_allowed_on_register_read():
    reads = init_reads()
    reads[0][0] = 0b01001000  # command mode is valid for EEPROM register replies
    sensor = KellerLD(bus=MockBus(reads), conversion_delay_s=0.0)

    assert sensor.init()


def test_reserved_pressure_mode_raises():
    bus = MockBus([[0x40, 0, 3]])
    sensor = KellerLD(bus=bus, conversion_delay_s=0.0)

    with pytest.raises(KellerLDError, match="unsupported pressure mode"):
        sensor.init()
