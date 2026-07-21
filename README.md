# BarXT-ROS2

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

Install the platform I2C dependency in the environment that runs the node:

```bash
sudo apt-get install python3-smbus
```

When the node runs in a Docker container, install the dependency inside that
container and pass the host I2C device through when creating it:

```bash
docker run ... --device=/dev/i2c-1 ...
```

The default Raspberry Pi 40-pin I2C bus is `/dev/i2c-1`. Do not use the
example bus number below unless your platform actually exposes that device.

## Raspberry Pi I2C Wiring and Bus Check

Before starting the ROS node, confirm that the BarXT is connected to the
Raspberry Pi's default 40-pin I2C bus. With the Raspberry Pi powered off,
wire the sensor as follows:

| BarXT wire | Raspberry Pi 40-pin header |
| --- | --- |
| Red (Vin) | Physical pin 1, 3.3 V |
| Black (GND) | Physical pin 6, GND |
| White (SDA) | Physical pin 3, GPIO2 / SDA1 |
| Green (SCL) | Physical pin 5, GPIO3 / SCL1 |

After booting, check which I2C device nodes are available and scan the bus
that the sensor is physically connected to. On the default Raspberry Pi
header wiring above, the expected bus is `1` and the BarXT address is `0x40`:

```bash
ls -l /dev/i2c-*
sudo i2cdetect -y 1
```

Only run the node after `i2cdetect` shows `40`. If the sensor is deliberately
wired to a different Linux I2C bus, replace `1` with that bus number in both
the scan command and the `i2c_bus` launch parameter.

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

## Combined BarXT + MicroStrain IMU Launch

`sensors.launch.py` starts this package's BarXT node and the official
`microstrain_inertial_driver` in the same ROS 2 graph. It is intended for a
Raspberry Pi where the BarXT is on I2C bus 1 and a 3DM-GV7-INS is available
inside the container at the configured USB path.

The container must be created with both device mappings:

```bash
docker run ... --device=/dev/i2c-1 --device=/dev/ttyACM0 \
  -v /dev/serial/by-id:/dev/serial/by-id:ro ...
```

Install the MicroStrain driver in that container, then build this package:

```bash
apt update
apt install -y ros-humble-microstrain-inertial-driver
cd /ws
colcon build --packages-select barxt_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Create an IMU parameter YAML whose `port` uses the stable
`/dev/serial/by-id/...` path. For the Pi setup used in this project, the file
is `/ws/config/gv7_imu.yaml`; pass another path with `imu_params_file:=...`.
The GV7-INS configuration must use `filter_pps_source: 4` when no external PPS
is connected.

Run both sensor nodes:

```bash
ros2 launch barxt_ros2 sensors.launch.py
```

For seawater depth conversion:

```bash
ros2 launch barxt_ros2 sensors.launch.py fluid_density_kg_m3:=1025.0
```

Verify the outputs in another sourced terminal:

```bash
ros2 topic hz /imu/data
ros2 topic echo /barxt/pressure --once
ros2 topic echo /barxt/depth --once
```

## Hardware Run Guide

This guide assumes the BarXT sensor is connected to Linux I2C bus 1 and uses
the default Keller LD I2C address `0x40`.

### 1. Prepare the ROS2 environment

Source ROS2 first. Adjust the path if your ROS2 installation is different.

```bash
source /opt/ros/humble/setup.bash
```

If this workspace depends on a custom ROS2 build, source that environment
instead.

```bash
source ~/ros2_humble/install/setup.bash
```

### 2. Check I2C access

Confirm the I2C device node exists.

```bash
ls -l /dev/i2c-1
```

Expected form:

```text
crw-rw---- 1 root i2c ... /dev/i2c-1
```

If the device is owned by the `i2c` group, add the runtime user to that group
or run the hardware test with appropriate privileges.

```bash
sudo usermod -aG i2c $USER
```

Log out and back in after changing groups.


### 3. Build the package

From the workspace root:

```bash
colcon build --packages-select barxt_ros2
source install/setup.bash
```

### 4. Run the BarXT node

Run with the tested hardware settings:

```bash
ros2 launch barxt_ros2 barxt.launch.py i2c_bus:=1 i2c_address:=0x40
```

For a lower-rate smoke test:

```bash
ros2 launch barxt_ros2 barxt.launch.py i2c_bus:=1 i2c_address:=0x40 publish_rate_hz:=2.0
```

For seawater depth conversion:

```bash
ros2 launch barxt_ros2 barxt.launch.py \
  i2c_bus:=1 \
  i2c_address:=0x40 \
  fluid_density_kg_m3:=1025.0
```

### 5. Check published topics

In another terminal, source the workspace:

```bash
source install/setup.bash
```

Then check the output topics:

```bash
ros2 topic echo /barxt/pressure --once
ros2 topic echo /barxt/temperature --once
ros2 topic echo /barxt/depth --once
```

Expected topic types:

```bash
ros2 topic list -t
```

```text
/barxt/pressure [sensor_msgs/msg/FluidPressure]
/barxt/temperature [sensor_msgs/msg/Temperature]
/barxt/depth [std_msgs/msg/Float32]
```

### 6. Run tests

Hardware-free tests use mock I2C data:

```bash
colcon test --packages-select barxt_ros2
colcon test-result --verbose
```


## Tests

Hardware-free tests use mock I2C data:

```bash
colcon test --packages-select barxt_ros2
colcon test-result --verbose
```
