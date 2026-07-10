"""Validation rules for Blue Robotics BarXT sensor readings."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

from .keller_ld import KellerLDMeasurement


RAW_PRESSURE_MIN = 16384
RAW_PRESSURE_MAX = 49152
RAW_TEMPERATURE_MIN = 384
RAW_TEMPERATURE_MAX = 64384
GRAVITY_M_S2 = 9.80665
BAR_TO_PA = 100000.0


@dataclass(frozen=True)
class BarXTModelSpec:
    name: str
    pressure_min_bar: float
    pressure_max_bar: float
    depth_min_m: float
    depth_max_m: float
    temperature_min_c: float = -10.0
    temperature_max_c: float = 80.0
    compensated_temperature_min_c: float = 0.0
    compensated_temperature_max_c: float = 50.0


MODEL_SPECS: Dict[str, BarXTModelSpec] = {
    "Bar3XT": BarXTModelSpec("Bar3XT", 0.0, 3.0, 0.0, 20.0),
    "Bar10XT": BarXTModelSpec("Bar10XT", 0.0, 10.0, 0.0, 92.0),
    "Bar30XT": BarXTModelSpec("Bar30XT", 0.0, 30.0, 0.0, 296.0),
    "Bar100XT": BarXTModelSpec("Bar100XT", 0.0, 100.0, 0.0, 1009.0),
}


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    warnings: List[str]
    errors: List[str]
    depth_m: float


def pressure_to_depth_m(
    pressure_bar: float,
    surface_pressure_bar: float,
    fluid_density_kg_m3: float,
) -> float:
    gauge_pressure_pa = max(0.0, pressure_bar - surface_pressure_bar) * BAR_TO_PA
    return gauge_pressure_pa / (fluid_density_kg_m3 * GRAVITY_M_S2)


class BarXTValidator:
    def __init__(
        self,
        model: str = "Bar3XT",
        surface_pressure_bar: float = 1.01325,
        fluid_density_kg_m3: float = 997.0,
    ) -> None:
        if model not in MODEL_SPECS:
            supported = ", ".join(sorted(MODEL_SPECS))
            raise ValueError(f"unsupported BarXT model '{model}', supported: {supported}")
        if fluid_density_kg_m3 <= 0.0:
            raise ValueError("fluid_density_kg_m3 must be positive")
        self.spec = MODEL_SPECS[model]
        self.surface_pressure_bar = surface_pressure_bar
        self.fluid_density_kg_m3 = fluid_density_kg_m3

    def validate(self, measurement: KellerLDMeasurement) -> ValidationResult:
        warnings: List[str] = []
        errors: List[str] = []
        spec = self.spec
        depth_m = pressure_to_depth_m(
            measurement.pressure_bar,
            self.surface_pressure_bar,
            self.fluid_density_kg_m3,
        )

        if not RAW_PRESSURE_MIN <= measurement.raw_pressure <= RAW_PRESSURE_MAX:
            errors.append(
                f"raw pressure {measurement.raw_pressure} outside "
                f"{RAW_PRESSURE_MIN}-{RAW_PRESSURE_MAX}"
            )

        if not RAW_TEMPERATURE_MIN <= measurement.raw_temperature <= RAW_TEMPERATURE_MAX:
            errors.append(
                f"raw temperature {measurement.raw_temperature} outside "
                f"{RAW_TEMPERATURE_MIN}-{RAW_TEMPERATURE_MAX}"
            )

        if not spec.temperature_min_c <= measurement.temperature_c <= spec.temperature_max_c:
            errors.append(
                f"temperature {measurement.temperature_c:.3f} C outside "
                f"{spec.temperature_min_c:.1f}-{spec.temperature_max_c:.1f} C"
            )
        elif not (
            spec.compensated_temperature_min_c
            <= measurement.temperature_c
            <= spec.compensated_temperature_max_c
        ):
            warnings.append(
                f"temperature {measurement.temperature_c:.3f} C outside compensated "
                f"{spec.compensated_temperature_min_c:.1f}-{spec.compensated_temperature_max_c:.1f} C"
            )

        if not spec.pressure_min_bar <= measurement.pressure_bar <= spec.pressure_max_bar:
            errors.append(
                f"pressure {measurement.pressure_bar:.6f} bar outside "
                f"{spec.pressure_min_bar:.1f}-{spec.pressure_max_bar:.1f} bar"
            )

        if not spec.depth_min_m <= depth_m <= spec.depth_max_m:
            errors.append(
                f"depth {depth_m:.6f} m outside "
                f"{spec.depth_min_m:.1f}-{spec.depth_max_m:.1f} m"
            )

        return ValidationResult(
            valid=not errors,
            warnings=warnings,
            errors=errors,
            depth_m=depth_m,
        )

