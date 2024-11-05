# EtherCAT Servo Motor Control Package for ROS2

This package provides a hardware interface to control a Kinco Servo motor over the EtherCAT protocol in ROS2.
---
It subscribes to velocity commands on the `/cmd_vel` topic and translates them into EtherCAT commands for motor control.

---

## Prerequisites

Before building this package, you need to install the SOEM library to enable EtherCAT communication. Follow these steps:

```bash
# Clone and install SOEM library
cd ~
git clone https://github.com/OpenEtherCATsociety/SOEM.git
cd SOEM
mkdir build
cmake ..
make
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install
```

---

## Permissions Setup

To allow the node to control network interfaces and access raw sockets (required for EtherCAT), perform the following steps:

1. **Grant Network Capabilities to the Node**  
   Assign network-related capabilities to the node executable, ex:
   ```bash
   sudo setcap cap_net_admin,cap_net_raw=eip ~/HardwareInterface_ws/install/ethercat_servo_control/lib/ethercat_servo_control/motor_control_node
   ```
   - **cap_net_admin**: Allows network management actions.
   - **cap_net_raw**: Grants permission to use raw network sockets, essential for EtherCAT.

2. **Add ROS2 Library Path**  
   If you encounter errors while running the node, set the ROS2 library path permanently:
   ```bash
   echo "/opt/ros/humble/lib" | sudo tee /etc/ld.so.conf.d/ros2.conf
   sudo ldconfig
   ```

---

### Node Structure and Key Functions
Here’s a quick overview of the primary functions in the package:

#### **MotorControlNode::MotorControlNode Constructor**

This function initializes the ROS2 node, sets parameters for the wheelbase, wheel radius, reduction ratio, and network interface, and subscribes to the `/cmd_vel` topic.  
Key variables:
- `wheel_base_`: Defines the robot's wheelbase.
- `wheel_radius_`: Radius of the wheel for speed calculation.
- `network_interface_`: Stores the network interface used for EtherCAT.

#### **MotorControlNode::initialize_motor(int slave_id)**

This function establishes the EtherCAT connection and prepares the motor for operational control.
- **EtherCAT Initialization**: Checks and configures available EtherCAT slaves.
- **Control Mode Setup**: Configures the motor for speed control.
- **State Verification**: Reads and sets the motor's state to confirm successful initialization.

#### **MotorControlNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)**

Handles `/cmd_vel` messages, adjusting motor speeds based on linear and angular velocities.
- **Speed Calculation**: Computes individual motor speeds for left and right motors.
- **Command Sending**: Sends computed speeds to the motor.

#### **MotorControlNode::send_motor_speed(int speed_rpm, int slave_id)**

Converts RPM values into the format required by the motor and sends them via EtherCAT.
- **Conversion**: RPM is converted based on the datasheet formula.
- **EtherCAT Transmission**: Sends the speed command to the motor.

#### **MotorControlNode::set_motor_acceleration(int acceleration, int slave_id)**

Sets motor acceleration and deceleration values.
- **Calculation**: Computes acceleration in RPM.
- **Command Dispatch**: Sends acceleration/deceleration commands to the motor.

#### **MotorControlNode::set_motor_kvp(int value, int slave_id)**

Sets the Kvp (velocity proportional gain) parameter, tuning motor responsiveness.
- **EtherCAT Transmission**: Configures the Kvp value on the motor.

#### **MotorControlNode::motorTestLoop(int slave_id)**

Provides a command-line loop to test motor RPM. Accepts user input for RPM, applies the command, then monitors motor response.
- **Loop Interaction**: Allows user RPM input or exits on `q`.
- **Motor Commanding**: Issues RPM commands and logs feedback.

#### **MotorControlNode::getMacAddress(std::string& mac)**

Reads a pre-stored MAC address from a file for determining network interface compatibility.
- **File Access**: Reads MAC from `ethercatdevices.txt`.
- **Validation**: Confirms file and MAC retrieval.

#### **MotorControlNode::findInterfaceByMac(const std::string& targetMac)**

Searches for the network interface based on MAC address.
- **Interface Matching**: Iterates through available network interfaces and compares MAC addresses.

---

### Final Note

 This project was originally written in ROS1 by Hasan Özcan and was migrated and enhanced in ROS2 by Saaid Jabr. 
 For more details, consult the Kinco Servo user manual, where all register addresses are documented. 

--- 


