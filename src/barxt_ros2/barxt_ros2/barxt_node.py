"""ROS2 node for publishing validated BarXT measurements."""

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


SensorFactory = Callable[[int, int], KellerLD]


def _as_int(value) -> int:
    if isinstance(value, str):
        return int(value, 0)
    return int(value)


def _as_float(value) -> float:
    return float(value)


class BarXTNode(Node):
    def __init__(self, sensor_factory: Optional[SensorFactory] = None) -> None:
        super().__init__("barxt_node")

        # launch 파라미터를 노드의 공개 설정 지점으로 둔다. 같은 노드로
        # BarXT 모델 범위와 유체 밀도 설정을 바꿔 실행할 수 있다.
        self.declare_parameter("model", "Bar3XT")
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", KellerLD.DEFAULT_I2C_ADDRESS)
        self.declare_parameter("frame_id", "barxt_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("fluid_density_kg_m3", 997.0)
        self.declare_parameter("surface_pressure_bar", 1.01325)

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
        self.sensor.init()

        self.pressure_pub = self.create_publisher(FluidPressure, "/barxt/pressure", 10)
        self.temperature_pub = self.create_publisher(Temperature, "/barxt/temperature", 10)
        self.depth_pub = self.create_publisher(Float32, "/barxt/depth", 10)

        self.total_samples = 0
        self.valid_samples = 0
        self.invalid_samples = 0
        self.driver_errors = 0
        self.last_error = ""
        self.last_warnings = []

        self.diagnostics = Updater(self)
        self.diagnostics.setHardwareID(f"BarXT {self.model}")
        self.diagnostics.add("BarXT data validation", self._diagnostic_status)

        # publish_rate_hz가 0 이하이면 0으로 나누는 대신 보수적인 10 Hz
        # 주기로 동작시킨다.
        period_s = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(period_s, self._poll_sensor)

    def _poll_sensor(self) -> None:
        try:
            measurement = self.sensor.read()
        except KellerLDError as exc:
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
        if validation.warnings:
            self.get_logger().warn("; ".join(validation.warnings))
        self.diagnostics.update()

    def _publish_measurement(self, measurement: KellerLDMeasurement, depth_m: float) -> None:
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
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
