"""Blue Robotics BarXT 센서 측정값 검증 규칙."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

from .keller_ld import KellerLDMeasurement


# Keller LD pressure raw count의 유효 구간이다.
# 이 구간 밖이면 센서 스케일 범위를 벗어난 값으로 보고 샘플을 drop한다.
RAW_PRESSURE_MIN = 16384
RAW_PRESSURE_MAX = 49152

# temperature raw count를 -50~150 C로 변환할 수 있는 관측 가능 구간이다.
# 이후 BarXT 운용 온도 범위는 별도로 검사한다.
RAW_TEMPERATURE_MIN = 384
RAW_TEMPERATURE_MAX = 64384

# 압력에서 수심을 계산할 때 사용하는 표준 중력가속도와 단위 변환 상수이다.
GRAVITY_M_S2 = 9.80665
BAR_TO_PA = 100000.0


@dataclass(frozen=True)
class BarXTModelSpec:
    # BarXT 모델별 하드웨어 스펙을 코드에서 일관되게 참조하기 위한 구조체이다.
    # frozen=True로 두어 런타임 중 검증 기준이 실수로 바뀌지 않게 한다.
    name: str
    pressure_min_bar: float
    pressure_max_bar: float
    depth_min_m: float
    depth_max_m: float
    temperature_min_c: float = -10.0
    temperature_max_c: float = 80.0
    compensated_temperature_min_c: float = 0.0
    compensated_temperature_max_c: float = 50.0


# 모델별 압력/수심 한계를 정의한다.
# 모델명을 launch parameter로 선택하면 BarXTValidator가 여기서 해당 스펙을 가져온다.
MODEL_SPECS: Dict[str, BarXTModelSpec] = {
    "Bar3XT": BarXTModelSpec("Bar3XT", 0.0, 3.0, 0.0, 20.0),
    "Bar10XT": BarXTModelSpec("Bar10XT", 0.0, 10.0, 0.0, 92.0),
    "Bar30XT": BarXTModelSpec("Bar30XT", 0.0, 30.0, 0.0, 296.0),
    "Bar100XT": BarXTModelSpec("Bar100XT", 0.0, 100.0, 0.0, 1009.0),
}


@dataclass(frozen=True)
class ValidationResult:
    # validate() 결과를 노드가 바로 사용할 수 있게 묶은 반환 타입이다.
    # errors가 비어 있으면 valid=True이고 publish 가능하다.
    # warnings는 publish는 가능하지만 diagnostics/log에 남겨야 하는 상태이다.
    valid: bool
    warnings: List[str]
    errors: List[str]
    depth_m: float


def pressure_to_depth_m(
    pressure_bar: float,
    surface_pressure_bar: float,
    fluid_density_kg_m3: float,
) -> float:
    # 센서 압력은 절대압 성격으로 들어오므로 수면 기준 압력을 빼서 gauge pressure를 만든다.
    # 수면보다 낮은 압력은 물리적으로 음수 수심이 되지 않도록 0으로 clamp한다.
    gauge_pressure_pa = max(0.0, pressure_bar - surface_pressure_bar) * BAR_TO_PA
    # hydrostatic relation: pressure = density * gravity * depth
    return gauge_pressure_pa / (fluid_density_kg_m3 * GRAVITY_M_S2)


class BarXTValidator:
    def __init__(
        self,
        model: str = "Bar3XT",
        surface_pressure_bar: float = 1.01325,
        fluid_density_kg_m3: float = 997.0,
    ) -> None:
        # 지원하지 않는 모델명은 시작 시점에 바로 실패시킨다.
        # 잘못된 모델로 계속 publish하는 것보다 startup failure가 안전하다.
        if model not in MODEL_SPECS:
            supported = ", ".join(sorted(MODEL_SPECS))
            raise ValueError(f"unsupported BarXT model '{model}', supported: {supported}")
        # 밀도는 depth 계산식의 분모에 들어가므로 0 이하를 허용하지 않는다.
        if fluid_density_kg_m3 <= 0.0:
            raise ValueError("fluid_density_kg_m3 must be positive")
        self.spec = MODEL_SPECS[model]
        self.surface_pressure_bar = surface_pressure_bar
        self.fluid_density_kg_m3 = fluid_density_kg_m3

    def validate(self, measurement: KellerLDMeasurement) -> ValidationResult:
        # 검증은 두 단계로 나뉜다.
        # errors: 값이 명백히 허용 범위를 벗어나 publish하면 안 되는 상태
        # warnings: 운용은 가능하지만 정확도/보상 범위를 벗어나 주의가 필요한 상태
        warnings: List[str] = []
        errors: List[str] = []
        spec = self.spec
        # depth도 publish 대상이므로 pressure와 같은 validation pass에서 계산하고 범위를 검사한다.
        depth_m = pressure_to_depth_m(
            measurement.pressure_bar,
            self.surface_pressure_bar,
            self.fluid_density_kg_m3,
        )

        # raw pressure count가 Keller LD 스케일 범위를 벗어나면 변환된 pressure도 신뢰하기 어렵다.
        if not RAW_PRESSURE_MIN <= measurement.raw_pressure <= RAW_PRESSURE_MAX:
            errors.append(
                f"raw pressure {measurement.raw_pressure} outside "
                f"{RAW_PRESSURE_MIN}-{RAW_PRESSURE_MAX}"
            )

        # raw temperature 자체의 관측 가능 범위를 먼저 검사한다.
        # 변환된 섭씨 온도 범위 검사는 아래에서 한 번 더 수행한다.
        if not RAW_TEMPERATURE_MIN <= measurement.raw_temperature <= RAW_TEMPERATURE_MAX:
            errors.append(
                f"raw temperature {measurement.raw_temperature} outside "
                f"{RAW_TEMPERATURE_MIN}-{RAW_TEMPERATURE_MAX}"
            )

        # BarXT 운용 온도(-10~80 C)를 벗어나면 오류로 보고 drop한다.
        if not spec.temperature_min_c <= measurement.temperature_c <= spec.temperature_max_c:
            errors.append(
                f"temperature {measurement.temperature_c:.3f} C outside "
                f"{spec.temperature_min_c:.1f}-{spec.temperature_max_c:.1f} C"
            )
        # 보상 온도 범위(0~50 C)를 벗어나도 운용 온도 안이면 publish는 한다.
        # 다만 보정 정확도에 영향을 줄 수 있으므로 diagnostics WARN으로 올린다.
        elif not (
            spec.compensated_temperature_min_c
            <= measurement.temperature_c
            <= spec.compensated_temperature_max_c
        ):
            warnings.append(
                f"temperature {measurement.temperature_c:.3f} C outside compensated "
                f"{spec.compensated_temperature_min_c:.1f}-{spec.compensated_temperature_max_c:.1f} C"
            )

        # 변환된 압력이 선택한 BarXT 모델의 정격 압력 범위를 벗어나면 drop한다.
        if not spec.pressure_min_bar <= measurement.pressure_bar <= spec.pressure_max_bar:
            errors.append(
                f"pressure {measurement.pressure_bar:.6f} bar outside "
                f"{spec.pressure_min_bar:.1f}-{spec.pressure_max_bar:.1f} bar"
            )

        # 압력은 정격 안이어도 surface pressure나 fluid density 설정에 따라 계산 수심이
        # 모델 수심 범위를 넘을 수 있으므로 별도로 검사한다.
        if not spec.depth_min_m <= depth_m <= spec.depth_max_m:
            errors.append(
                f"depth {depth_m:.6f} m outside "
                f"{spec.depth_min_m:.1f}-{spec.depth_max_m:.1f} m"
            )

        # 노드는 valid=False인 샘플을 publish하지 않고 diagnostics ERROR로 보고한다.
        return ValidationResult(
            valid=not errors,
            warnings=warnings,
            errors=errors,
            depth_m=depth_m,
        )
