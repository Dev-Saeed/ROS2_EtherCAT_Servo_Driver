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

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode();
    
    // Function to initialize the motor (sets it to operational mode)
    bool initialize_motor(int slave_id);
    
    // Function to send motor speed commands via EtherCAT
    void send_motor_speed(int speed_rpm, int slave_id);


private:
    // Callback function for processing /cmd_vel messages
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Helper functions for motor control
    void set_motor_acceleration(int acceleration, int slave_id);
    void set_motor_kvp(int value, int slave_id);
    void set_speed_control_mode(int slave_id);
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    
    // Helper functions for network interface and MAC address
    bool getMacAddress(std::string& mac);
    std::string findInterfaceByMac(const std::string& targetMac);
    
    // Parameters
    std::string network_interface_; // EtherCAT network interface name
    double wheel_base_;
    double wheel_radius_;
    double reduction_ratio_;
    int acceleration_;
    int control_mode_;
    int motor_initialized_;  // Tracks initialization state

    bool ec_state_;
    const int ethercat_timeout_ = 10000;  // EtherCAT timeout (1000us -> 1ms)
};

#endif  // MOTOR_CONTROL_HPP
