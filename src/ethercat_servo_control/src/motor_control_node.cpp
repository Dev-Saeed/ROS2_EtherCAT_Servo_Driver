#include "ethercat_servo_control/motor_control_node.hpp"
#include <cmath>

// Constructor: Initializes the ROS2 node and subscription
MotorControlNode::MotorControlNode()
: Node("motor_control_node"),
  wheel_base_(0.5),  // Example: 0.5 meters
  wheel_radius_(0.2),  // Example: 0.1 meters
  reduction_ratio_(20),
  acceleration_(1000),  // Default acceleration
  control_mode_(0x03),  // Speed control mode
  motor_initialized_(false),
  ec_state_(false),
  network_interface_("enp3s0")  // Set your default network interface here
{
    // Get MAC address from file and determine the network interface
    std::string mac_address;
    if (getMacAddress(mac_address)) {
        network_interface_ = findInterfaceByMac(mac_address);
        RCLCPP_INFO(this->get_logger(), "Network interface found: %s", network_interface_.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to obtain network interface by MAC address.");
    }

    // Subscribe to the /cmd_vel topic for velocity commands
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MotorControlNode::cmd_vel_callback, this, std::placeholders::_1));
}

// Function to initialize the motor
bool MotorControlNode::initialize_motor(int slave_id)
{
    // Initialize EtherCAT on the specified network interface
    char IOmap[4096];
    
    if (ec_init(network_interface_.c_str()))  //network_interface_.c_str() // Note: LAN1-->"B4:4B:D6:5A:7B:31", LAN2-->"B4:4B:D6:5A:7B:32"
    {
        if (ec_config(FALSE, &IOmap) > 0)  // ec_config() returns the number of slaves found
        {
            RCLCPP_INFO(this->get_logger(), "EtherCAT slaves found and configured.");
        } 
        else {
            RCLCPP_ERROR(this->get_logger(), "No EtherCAT slaves found.");
            return false;
        }
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize EtherCAT on interface %s.", network_interface_.c_str());
        return false;
    }

    ec_configdc();  // EtherCAT configuration
    ec_readstate();  // Read EtherCAT state
    ec_slave[slave_id].state = EC_STATE_OPERATIONAL;  // Set to operational state
    ec_writestate(slave_id);

    RCLCPP_INFO(this->get_logger(), "Slave: %s, State: %d", ec_slave[slave_id].name, ec_slave[slave_id].state);

    int8_t control_mode = control_mode_;
    int psize=sizeof(control_mode_);

    // Set control mode (e.g., velocity mode)
    ec_SDOwrite(slave_id, 0x6060, 0x00, FALSE, psize, &control_mode, ethercat_timeout_);
    
    // Read back the state to verify seted control mode
    ec_SDOread( slave_id, 0x6060, 0x00, FALSE,&psize, &control_mode, ethercat_timeout_);
    RCLCPP_INFO(this->get_logger(), "6060: %d ", control_mode_);
    

    // Verify if the motor is operational
    uint16_t control_word = 0x06;  // Motor enable
    ec_SDOwrite(slave_id, 0x6040, 0x00, FALSE, sizeof(control_word), &control_word, ethercat_timeout_);

    control_word = 0x0F;  // Speed control enable
    ec_SDOwrite(slave_id, 0x6040, 0x00, FALSE, sizeof(control_word), &control_word, ethercat_timeout_);

    // Read back the state to verify initialization
    uint16_t read_value;
    psize = sizeof(read_value);
    ec_SDOread(slave_id, 0x6040, 0x00, FALSE, &psize, &read_value, ethercat_timeout_);

    if (read_value == 0x0F) {
        motor_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Motor %d initialized successfully!", slave_id);
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Motor %d failed to initialize!", slave_id);
        return false;
    }

    // Pre set value (This values from the old code)
    int init_speed = 0;
    int init_acc   = 1.5;
    int init_Kvp   = 90; 

    send_motor_speed(init_speed, slave_id); 
    set_motor_acceleration(init_acc, slave_id);
    set_motor_kvp(init_Kvp, slave_id);

}

// Find interface by MAC address
std::string MotorControlNode::findInterfaceByMac(const std::string& targetMac) 
{
    struct ifaddrs *ifaddr, *ifa;
    char macStr[18];

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return "";
    }

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_PACKET) {
            continue;
        }

        struct sockaddr_ll *s = (struct sockaddr_ll*)ifa->ifa_addr;
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
                 s->sll_addr[0], s->sll_addr[1], s->sll_addr[2],
                 s->sll_addr[3], s->sll_addr[4], s->sll_addr[5]);

        if (targetMac == macStr) {
            freeifaddrs(ifaddr);
            return std::string(ifa->ifa_name);
        }
    }

    freeifaddrs(ifaddr);
    return "";
}

// Read MAC address from file
bool MotorControlNode::getMacAddress(std::string& mac)
{
    uid_t uid = getuid();
    struct passwd *pw = getpwuid(uid);
    if (!pw) {
        std::cerr << "Failed to find username!" << std::endl;
        return false;
    }
    
    // You should already be created a .txt file in /home dir and had wrote mac adress in it
    std::string file_dir = "/home/" + std::string(pw->pw_name) + "/ethercatdevices.txt";
    std::ifstream file(file_dir);
    if (file.is_open()) {
        std::string line;
        if (std::getline(file, line)) {
            mac = line;
            std::cerr << "The MAC address has been read successfully.!" << std::endl;
            return true;
        } else {
            std::cerr << "File is empty!" << std::endl;
        }
        file.close();
    } else {
        std::cerr << "Failed to open the file! File directory: " << file_dir << std::endl;
    }
    return false;
}

// Function to send motor speed commands  
void MotorControlNode::send_motor_speed(int speed_rpm, int slave_id)
{
    if (!motor_initialized_) {
        RCLCPP_ERROR(this->get_logger(), "Motor %d is not initialized!", slave_id);
        return;
    }

    // Convert RPM to the value expected by the motor and send it  
    double motor_value = speed_rpm * 17896 * reduction_ratio_;
    int32_t s32val = static_cast<int32_t>(motor_value);
    ec_SDOwrite(slave_id, 0x60FF, 0x00, FALSE, sizeof(s32val), &s32val, ethercat_timeout_);
}

// Function to send motor acceleration commands  
void MotorControlNode::set_motor_acceleration(int acceleration, int slave_id)
{
    int rps=1074; // 1 rps = 1074 Dec
    acceleration=acceleration*(M_PI*2 * wheel_radius_) *reduction_ratio_; //revolution per s^2
    int32 acc_val=int(acceleration*rps); // dec
    while(ec_state_ && rclcpp::ok()) {
    rclcpp::sleep_for(std::chrono::milliseconds(5));
    }
    ec_state_ = true;
    ec_SDOwrite(slave_id, 0x6083, 0x00, FALSE, sizeof(acc_val), &acc_val, ethercat_timeout_); // Acc
    ec_SDOwrite(slave_id, 0x6084, 0x00, FALSE, sizeof(acc_val), &acc_val, ethercat_timeout_); // Dcc
    ec_state_ = false;
}

// Function to set motor Kvp value  
void MotorControlNode::set_motor_kvp(int value, int slave_id)
{
    int16_t kvp_0 = value; // Dec representation
    while (ec_state_ && rclcpp::ok()) {
        rclcpp::sleep_for(std::chrono::milliseconds(5));
    }

    ec_state_ = true;
    // Write the kvp value to the motor  
    ec_SDOwrite(slave_id, 0x60F9, 0x01, FALSE, sizeof(kvp_0), &kvp_0, ethercat_timeout_);
    ec_state_ = false;

    RCLCPP_WARN(this->get_logger(), "Motor %d kvp_0 set to %d", slave_id, kvp_0);
}

// Callback for /cmd_vel topic
void MotorControlNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Calculate the motor speeds for left and right motors based on the Twist message
    double right_motor_speed = msg->linear.x + msg->angular.z * wheel_base_ / 2;
    double left_motor_speed = msg->linear.x - msg->angular.z * wheel_base_ / 2;

    int right_rpm = static_cast<int>((right_motor_speed / (2 * M_PI * wheel_radius_)) * 60);
    int left_rpm = static_cast<int>((left_motor_speed / (2 * M_PI * wheel_radius_)) * 60);

    // Send speed commands to the motors
    send_motor_speed(right_rpm, 1);  // Send speed to right motor (slave 1)
    send_motor_speed(left_rpm, 2);   // Send speed to left motor (slave 2)
}

// Main function
int main(int argc, char *argv[])
{
 
    rclcpp::init(argc, argv);
    auto motor_control_node = std::make_shared<MotorControlNode>();

    // Initialize the motor (slave 1 and 2 can be initialized here)
    if (!motor_control_node->initialize_motor(1)) {
        RCLCPP_ERROR(motor_control_node->get_logger(), "Failed to initialize right motor ");
        return 1;
    }

    if (!motor_control_node->initialize_motor(2)) {
        RCLCPP_ERROR(motor_control_node->get_logger(), "Failed to initialize left motor ");
        return 1;
    }

    rclcpp::spin(motor_control_node);
    rclcpp::shutdown();
    return 0;
}
