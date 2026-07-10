"""검증된 BarXT 측정값을 ROS2 토픽으로 publish하는 노드."""

from __future__ import annotations

from typing import Callable, Optional

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Float32

from .keller_ld import KellerLD, KellerLDError, KellerLDMeasurement
from .validation import BarXTValidator


# 테스트에서 실제 KellerLD 대신 mock 센서를 주입하기 위한 factory 타입이다.
# 인자는 I2C bus 번호와 I2C address이고, KellerLD와 같은 인터페이스를 반환한다.
SensorFactory = Callable[[int, int], KellerLD]


def _as_int(value) -> int:
    # launch 파일에서 넘어온 파라미터는 문자열일 수 있다.
    # int(value, 0)을 사용하면 "64"와 "0x40" 형식을 모두 처리할 수 있다.
    if isinstance(value, str):
        return int(value, 0)
    return int(value)


def _as_float(value) -> float:
    # ROS2 launch substitution은 숫자도 문자열로 전달될 수 있으므로 float로 정규화한다.
    return float(value)


class BarXTNode(Node):
    def __init__(self, sensor_factory: Optional[SensorFactory] = None) -> None:
        super().__init__("barxt_node")

        # launch 파라미터를 노드의 공개 설정 지점으로 둔다. 같은 노드로
        # BarXT 모델 범위와 유체 밀도 설정을 바꿔 실행할 수 있다.
        # 실제 i2c-7 하드웨어 테스트도 이 파라미터만 바꿔 실행한다.
        self.declare_parameter("model", "Bar3XT")
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", KellerLD.DEFAULT_I2C_ADDRESS)
        self.declare_parameter("frame_id", "barxt_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("fluid_density_kg_m3", 997.0)
        self.declare_parameter("surface_pressure_bar", 1.01325)

        # 선언한 파라미터를 노드 내부에서 쓰기 쉬운 Python 타입으로 변환한다.
        # model과 frame_id는 문자열 그대로 사용하고, 수치 설정은 명시적으로 변환한다.
        self.model = self.get_parameter("model").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate_hz = _as_float(self.get_parameter("publish_rate_hz").value)
        i2c_bus = _as_int(self.get_parameter("i2c_bus").value)
        i2c_address = _as_int(self.get_parameter("i2c_address").value)
        fluid_density_kg_m3 = _as_float(self.get_parameter("fluid_density_kg_m3").value)
        surface_pressure_bar = _as_float(self.get_parameter("surface_pressure_bar").value)

        # BarXTValidator는 모델별 허용 범위와 수심 변환을 담당하고,
        # KellerLD는 I2C 프로토콜 처리에만 집중한다.
        self.validator = BarXTValidator(
            model=self.model,
            surface_pressure_bar=surface_pressure_bar,
            fluid_density_kg_m3=fluid_density_kg_m3,
        )
        # 테스트에서는 sensor_factory를 주입해 실제 하드웨어 없이도
        # 결정적인 측정값으로 ROS 노드를 검증할 수 있다.
        factory = sensor_factory or (lambda bus, address: KellerLD(bus=bus, address=address))
        self.sensor = factory(i2c_bus, i2c_address)
        # init()에서 센서의 pressure mode, p_min, p_max를 읽는다.
        # 이 보정값이 없으면 raw pressure를 bar 단위로 변환할 수 없다.
        self.sensor.init()

        # pressure와 temperature는 ROS 표준 sensor_msgs를 사용한다.
        # depth는 단일 실수 값이므로 std_msgs/Float32로 publish한다.
        self.pressure_pub = self.create_publisher(FluidPressure, "/barxt/pressure", 10)
        self.temperature_pub = self.create_publisher(Temperature, "/barxt/temperature", 10)
        self.depth_pub = self.create_publisher(Float32, "/barxt/depth", 10)

        # diagnostics와 로그에서 노드 상태를 추적하기 위한 누적 카운터이다.
        # total_samples는 드라이버 read가 성공한 뒤 validation까지 도달한 샘플 수이다.
        self.total_samples = 0
        self.valid_samples = 0
        self.invalid_samples = 0
        self.driver_errors = 0
        # 가장 최근 에러/경고를 저장해 diagnostics callback에서 상태를 만들 수 있게 한다.
        self.last_error = ""
        self.last_warnings = []

        # diagnostic_updater는 /diagnostics 토픽으로 노드 상태를 주기적으로 보고한다.
        # 실제 센서 운용 중에는 이 값으로 샘플 drop이나 드라이버 오류를 추적할 수 있다.
        self.diagnostics = Updater(self)
        self.diagnostics.setHardwareID(f"BarXT {self.model}")
        self.diagnostics.add("BarXT data validation", self._diagnostic_status)

        # publish_rate_hz가 0 이하이면 0으로 나누는 대신 보수적인 10 Hz
        # 주기로 동작시킨다.
        period_s = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(period_s, self._poll_sensor)

    def _poll_sensor(self) -> None:
        # 타이머 callback의 전체 흐름:
        # 1. KellerLD에서 I2C 측정값을 읽는다.
        # 2. BarXTValidator로 raw/pressure/temperature/depth 범위를 검증한다.
        # 3. 통과한 샘플만 ROS 토픽으로 publish한다.
        try:
            measurement = self.sensor.read()
        except KellerLDError as exc:
            # I2C 통신 실패, status byte 오류, 초기화 누락 등 드라이버 계층의 오류이다.
            # 이 경우 측정값 자체가 없으므로 publish하지 않고 diagnostics만 갱신한다.
            self.driver_errors += 1
            self.invalid_samples += 1
            self.last_error = str(exc)
            self.last_warnings = []
            self.get_logger().error(f"BarXT driver error: {exc}")
            self.diagnostics.update()
            return

        self.total_samples += 1
        validation = self.validator.validate(measurement)
        # 유효하지 않은 샘플은 publish하지 않는다. downstream 노드는 raw count,
        # 운용 범위, 수심 검사를 통과한 값만 받는다.
        if not validation.valid:
            self.invalid_samples += 1
            self.last_error = "; ".join(validation.errors)
            self.last_warnings = validation.warnings
            self.get_logger().warn(f"Dropping invalid BarXT sample: {self.last_error}")
            self.diagnostics.update()
            return

        self.valid_samples += 1
        self.last_error = ""
        self.last_warnings = validation.warnings
        self._publish_measurement(measurement, validation.depth_m)
        # 보상 온도 범위 이탈처럼 publish는 가능하지만 주의가 필요한 상태를 로그로 남긴다.
        if validation.warnings:
            self.get_logger().warn("; ".join(validation.warnings))
        self.diagnostics.update()

    def _publish_measurement(self, measurement: KellerLDMeasurement, depth_m: float) -> None:
        # pressure와 temperature는 Header가 있는 메시지이므로 같은 timestamp/frame_id를 쓴다.
        # depth는 Float32라 Header가 없고, 동일 poll에서 계산된 값을 바로 담는다.
        now = self.get_clock().now().to_msg()

        pressure_msg = FluidPressure()
        pressure_msg.header.stamp = now
        pressure_msg.header.frame_id = self.frame_id
        # sensor_msgs/FluidPressure는 Pa 단위를 사용하고 KellerLD는 bar로 반환한다.
        pressure_msg.fluid_pressure = measurement.pressure_bar * 100000.0
        pressure_msg.variance = 0.0
        self.pressure_pub.publish(pressure_msg)

        temperature_msg = Temperature()
        temperature_msg.header.stamp = now
        temperature_msg.header.frame_id = self.frame_id
        # sensor_msgs/Temperature의 temperature 필드는 섭씨 단위이다.
        temperature_msg.temperature = measurement.temperature_c
        temperature_msg.variance = 0.0
        self.temperature_pub.publish(temperature_msg)

        depth_msg = Float32()
        depth_msg.data = float(depth_m)
        self.depth_pub.publish(depth_msg)

    def _diagnostic_status(self, stat):
        # diagnostics는 가장 최근 poll 결과를 반영하고, 장시간 상태 확인을 위한
        # 카운터를 함께 노출한다.
        if self.last_error:
            stat.summary(DiagnosticStatus.ERROR, self.last_error)
        elif self.last_warnings:
            stat.summary(DiagnosticStatus.WARN, "; ".join(self.last_warnings))
        else:
            stat.summary(DiagnosticStatus.OK, "publishing valid BarXT samples")

        # diagnostic_msgs/KeyValue.value는 문자열 타입만 허용한다.
        # 정수를 그대로 넣으면 ROS2 Humble에서 AssertionError가 발생한다.
        stat.add("model", self.model)
        stat.add("total_samples", str(self.total_samples))
        stat.add("valid_samples", str(self.valid_samples))
        stat.add("invalid_samples", str(self.invalid_samples))
        stat.add("driver_errors", str(self.driver_errors))
        return stat


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BarXTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # launch에서 Ctrl-C로 종료할 때 traceback을 남기지 않기 위한 정상 종료 경로이다.
        pass
    finally:
        node.destroy_node()
        # launch가 이미 shutdown한 context에 대해 rclpy.shutdown()을 다시 호출하지 않는다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
