// motor_control.hpp
#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <soem/ethercat.h>
#include <string>
#include <fstream>
#include <ifaddrs.h>
#include <netpacket/packet.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <pwd.h>
#include <iostream>
<<<<<<< HEAD
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream> 
=======
>>>>>>> 3a50146f5e52bca35aa463bec3988bebebd4e223

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode();
    
    // Function to initialize the motor (sets it to operational mode)
    bool initialize_motor(int slave_id);
    
    // Function to send motor speed commands via EtherCAT
    void send_motor_speed(int speed_rpm, int slave_id);
<<<<<<< HEAD
    
=======
>>>>>>> 3a50146f5e52bca35aa463bec3988bebebd4e223


private:
    // Callback function for processing /cmd_vel messages
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
<<<<<<< HEAD
    
=======
>>>>>>> 3a50146f5e52bca35aa463bec3988bebebd4e223

    // Helper functions for motor control
    void set_motor_acceleration(int acceleration, int slave_id);
    void set_motor_kvp(int value, int slave_id);
    void set_speed_control_mode(int slave_id);
<<<<<<< HEAD
    void motorTestLoop(int slave_id);
=======
>>>>>>> 3a50146f5e52bca35aa463bec3988bebebd4e223
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    
    // Helper functions for network interface and MAC address
    bool getMacAddress(std::string& mac);
    std::string findInterfaceByMac(const std::string& targetMac);
    
    // Parameters
<<<<<<< HEAD
    double wheel_base_;
    double wheel_radius_;
    double reduction_ratio_;
    const double encoder_resiulution = 2500*4; // 2500 PPR incremental encoder for SMC60S-0040-30AAK-3DSH Kinco Servo
    // double encoder_resiulution = 2^16;   // 16 bit absulte encoder for SMC60S-0040-30KBK-3DSH Kinco Servo
=======
    std::string network_interface_; // EtherCAT network interface name
    double wheel_base_;
    double wheel_radius_;
    double reduction_ratio_;
    int acceleration_;
>>>>>>> 3a50146f5e52bca35aa463bec3988bebebd4e223
    int control_mode_;
    int motor_initialized_;  // Tracks initialization state

    bool ec_state_;
<<<<<<< HEAD
    std::string network_interface_; // EtherCAT network interface name
    const int ethercat_timeout_ = 10000;  // EtherCAT timeout (1000us -> 1ms)
    geometry_msgs::msg::Twist past_twist_msg;
=======
    const int ethercat_timeout_ = 10000;  // EtherCAT timeout (1000us -> 1ms)
>>>>>>> 3a50146f5e52bca35aa463bec3988bebebd4e223
};

#endif  // MOTOR_CONTROL_HPP
