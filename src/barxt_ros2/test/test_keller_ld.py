import struct

import pytest

from barxt_ros2.keller_ld import KellerLD, KellerLDStatusError


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
        [0, (ms_word >> 8) & 0xFF, ms_word & 0xFF],
        [0, (ls_word >> 8) & 0xFF, ls_word & 0xFF],
    ]


def init_reads(p_min=0.0, p_max=2.0):
    return [
        [0, 0, 1],
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
            [0, 0x80, 0x00, (raw_temperature >> 8) & 0xFF, raw_temperature & 0xFF],
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


def test_invalid_mode_status_raises():
    bus = MockBus([[0b00001000, 0, 1]])
    sensor = KellerLD(bus=bus, conversion_delay_s=0.0)

    with pytest.raises(KellerLDStatusError, match="invalid status mode"):
        sensor.init()


def test_checksum_status_raises_on_measurement():
    raw_temperature = raw_temperature_for_celsius(25.0)
    bus = MockBus(
        [
            *init_reads(),
            [0b00000100, 0x80, 0x00, (raw_temperature >> 8) & 0xFF, raw_temperature & 0xFF],
        ]
    )
    sensor = KellerLD(bus=bus, conversion_delay_s=0.0)

    assert sensor.init()
    with pytest.raises(KellerLDStatusError, match="memory checksum error"):
        sensor.read()
