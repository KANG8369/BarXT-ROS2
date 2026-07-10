# BarXT ROS2

ROS2 Python driver package for Blue Robotics BarXT pressure/depth sensors based on the Keller 4LD I2C pressure transmitter.

The default configuration targets Bar3XT:

- I2C address: `0x40`
- Operating pressure: `0-3 bar`
- Operating depth: `0-20 m`
- Operating temperature: `-10-80 C`
- Compensated temperature band: `0-50 C`

Sources:

- Blue Robotics BarXT product specifications: https://bluerobotics.com/store/sensors-cameras/sensors/barxt-extended-submersion-depth-pressure-sensors/
- KellerLD Python reference driver: https://github.com/bluerobotics/KellerLD-python
- Keller 4LD-9LD datasheet supplied with this project request

## Topics

The `barxt_node` publishes only validated samples:

- `/barxt/pressure` (`sensor_msgs/msg/FluidPressure`)
- `/barxt/temperature` (`sensor_msgs/msg/Temperature`)
- `/barxt/depth` (`std_msgs/msg/Float32`)

Invalid samples are dropped and reported through ROS diagnostics and node logs.

## Parameters

| Name | Default | Description |
| --- | --- | --- |
| `model` | `Bar3XT` | Sensor model. Supported values: `Bar3XT`, `Bar10XT`, `Bar30XT`, `Bar100XT`. |
| `i2c_bus` | `1` | Linux I2C bus number, for example `/dev/i2c-1`. |
| `i2c_address` | `0x40` | Keller LD I2C address. |
| `frame_id` | `barxt_link` | Header frame id for pressure and temperature messages. |
| `publish_rate_hz` | `10.0` | Poll/publish rate. |
| `fluid_density_kg_m3` | `997.0` | Fluid density used for depth conversion. Freshwater default. |
| `surface_pressure_bar` | `1.01325` | Surface/reference pressure subtracted before depth conversion. |

## Validation

Runtime validation covers only fields that can be observed from I2C data:

- raw pressure count must be within `16384-49152`
- raw temperature count must map to `-50-150 C`
- decoded temperature must be within `-10-80 C`
- decoded pressure must be within the selected model pressure range
- computed depth must be within the selected model depth range
- decoded temperature outside `0-50 C` but inside `-10-80 C` is published with a diagnostics warning

Electrical and physical requirements such as supply voltage, current, connector, material, and installation dimensions cannot be verified from the sensor data stream. Confirm those during wiring and mechanical integration.

## Usage

Install the platform I2C dependency on the target computer:

```bash
sudo apt-get install python3-smbus
```

Build and source the workspace:

```bash
colcon build
source install/setup.bash
```

Run with defaults:

```bash
ros2 launch barxt_ros2 barxt.launch.py
```

Run with seawater density:

```bash
ros2 launch barxt_ros2 barxt.launch.py fluid_density_kg_m3:=1025.0
```

## Tests

Hardware-free tests use mock I2C data:

```bash
colcon test --packages-select barxt_ros2
colcon test-result --verbose
```

