import pytest

from barxt_ros2.keller_ld import KellerLDMeasurement
from barxt_ros2.validation import BarXTValidator, pressure_to_depth_m


def raw_temperature_for_celsius(temperature_c):
    return int(((temperature_c + 50.0) / 0.05) + 24) << 4


def measurement(
    pressure_bar=2.0,
    temperature_c=25.0,
    raw_pressure=32768,
    raw_temperature=None,
):
    if raw_temperature is None:
        raw_temperature = raw_temperature_for_celsius(temperature_c)
    return KellerLDMeasurement(
        pressure_bar=pressure_bar,
        temperature_c=temperature_c,
        raw_pressure=raw_pressure,
        raw_temperature=raw_temperature,
        status=0,
    )


def test_valid_bar3xt_sample_is_accepted():
    validator = BarXTValidator("Bar3XT")

    result = validator.validate(measurement())

    assert result.valid
    assert result.errors == []
    assert result.depth_m == pytest.approx(
        pressure_to_depth_m(2.0, surface_pressure_bar=1.01325, fluid_density_kg_m3=997.0)
    )


@pytest.mark.parametrize("raw_pressure", [16383, 49153])
def test_raw_pressure_outside_pdf_scale_is_rejected(raw_pressure):
    validator = BarXTValidator("Bar3XT")

    result = validator.validate(measurement(raw_pressure=raw_pressure))

    assert not result.valid
    assert any("raw pressure" in error for error in result.errors)


@pytest.mark.parametrize("raw_temperature", [383, 64385])
def test_raw_temperature_outside_pdf_scale_is_rejected(raw_temperature):
    validator = BarXTValidator("Bar3XT")

    result = validator.validate(measurement(raw_temperature=raw_temperature))

    assert not result.valid
    assert any("raw temperature" in error for error in result.errors)


@pytest.mark.parametrize("temperature_c", [-10.0, 80.0])
def test_operating_temperature_boundaries_are_valid(temperature_c):
    validator = BarXTValidator("Bar3XT")

    result = validator.validate(measurement(temperature_c=temperature_c))

    assert result.valid


@pytest.mark.parametrize("temperature_c", [-10.1, 80.1])
def test_temperature_outside_operating_range_is_rejected(temperature_c):
    validator = BarXTValidator("Bar3XT")

    result = validator.validate(measurement(temperature_c=temperature_c))

    assert not result.valid
    assert any("temperature" in error for error in result.errors)


@pytest.mark.parametrize("temperature_c", [-1.0, 60.0])
def test_temperature_outside_compensated_range_warns_but_publishes(temperature_c):
    validator = BarXTValidator("Bar3XT")

    result = validator.validate(measurement(temperature_c=temperature_c))

    assert result.valid
    assert any("compensated" in warning for warning in result.warnings)


@pytest.mark.parametrize("pressure_bar", [0.0, 3.0])
def test_bar3xt_pressure_boundaries_are_valid(pressure_bar):
    validator = BarXTValidator("Bar3XT")

    result = validator.validate(measurement(pressure_bar=pressure_bar))

    if pressure_bar == 3.0 and result.errors:
        assert all("depth" in error for error in result.errors)
    else:
        assert result.valid


@pytest.mark.parametrize("pressure_bar", [-0.1, 3.1])
def test_bar3xt_pressure_outside_range_is_rejected(pressure_bar):
    validator = BarXTValidator("Bar3XT")

    result = validator.validate(measurement(pressure_bar=pressure_bar))

    assert not result.valid
    assert any("pressure" in error for error in result.errors)


@pytest.mark.parametrize("pressure_bar", [1.01325, 2.96832])
def test_bar3xt_depth_boundaries_are_valid(pressure_bar):
    validator = BarXTValidator("Bar3XT")

    result = validator.validate(measurement(pressure_bar=pressure_bar))

    assert result.valid
    assert 0.0 <= result.depth_m <= 20.0


def test_depth_outside_bar3xt_range_is_rejected():
    validator = BarXTValidator("Bar3XT")

    result = validator.validate(measurement(pressure_bar=3.2))

    assert not result.valid
    assert any("depth" in error for error in result.errors)
