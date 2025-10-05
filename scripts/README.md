
This directory is intended for PlatformIO Test Runner and project tests.

Unit Testing is a software testing method by which individual units of
source code, sets of one or more MCU program modules together with associated
control data, usage procedures, and operating procedures, are tested to
determine whether they are fit for use. Unit testing finds problems early
in the development cycle.

More information about PlatformIO Unit Testing:
- https://docs.platformio.org/en/latest/advanced/unit-testing/index.html

### dependencies
- ROS2 humble
- platformio (extension in VSCode)

### usage
- build workspace (using platformio extension)
- connect esp32
- upload code to esp32

### testing 
```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

## for wifi setup

uncomment followed lines of code in file `lib/ros_communication/ros_communication.cpp` 

```cpp
IPAddress agent_ip(192, 168, 223, 64);
size_t agent_port = 8888;
 
char ssid[] = "ssid";
char psk[]= "psk";
 
set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
```

change:
- `agent_ip(192, 168, 223, 64)` - set ip of device where will be launched micro_ros agent (for example your computer)
- `agent_port` - if required 
- `ssid` and `psk` of your wifi 

comment followed lines of code in file `lib/ros_communication/ros_communication.cpp` 

```cpp
// Serial.begin(115200);
// set_microros_serial_transports(Serial);
```


uncomment followed lines of code in file `platformio.ini`

```ini
board_microros_transport = wifi
```

comment followed lines of code in file `platformio.ini`

```ini
; board_microros_transport = serial
```

After build and upload - test it
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## for serial setup

uncomment followed lines of code in file `lib/ros_communication/ros_communication.cpp` 

```cpp
Serial.begin(115200);
set_microros_serial_transports(Serial);
```

comment followed lines of code in file `lib/ros_communication/ros_communication.cpp` 

```cpp
// IPAddress agent_ip(192, 168, 223, 64);
// size_t agent_port = 8888;
 
// char ssid[] = "ssid";
// char psk[]= "psk";
 
// set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
```

uncomment followed lines of code in file `platformio.ini`

```ini
board_microros_transport = serial
```

comment followed lines of code in file `platformio.ini`

```ini
; board_microros_transport = wifi
```

After build and upload - test it
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

# driving test
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_nav

```