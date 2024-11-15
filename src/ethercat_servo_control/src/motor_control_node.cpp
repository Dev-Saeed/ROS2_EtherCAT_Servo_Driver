/* 
Note: All registers and addresses can be referenced on the Kinco Servo user manual.

*/

#include "ethercat_servo_control/motor_control_node.hpp"
#include "ethercat_servo_control/kinco_registers.hpp"



using namespace std::chrono_literals;

// Constructor: Initializes the ROS2 node and subscription
MotorControlNode::MotorControlNode()
: Node("motor_control_node"),
  wheel_base_(0.5),  // Example: 0.5 meters
  wheel_radius_(0.1),  // Example: 0.2 meters
  reduction_ratio_(1),
  control_mode_(VELOCITY_POSE_MODE),  // Speed control mode with Velocity-Position control loops are active
  motor_initialized_(false),
  ec_state_(false),
  network_interface_("enp3s0")  // Set your default network interface here
{
    // Get MAC address from file  and determine the network interface 
    // If you want to use the predefined network_interface settled in the Constructor, just comment these lines
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

    // Create the SetAcceleration service
    acceleration_service_ = this->create_service<servo_msgs::srv::SetAcceleration>(
        "set_acceleration", 
        std::bind(&MotorControlNode::set_acceleration_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Create the SetKvp service
    kvp_service_ = this->create_service<servo_msgs::srv::SetKvp>(
        "set_kvp", 
        std::bind(&MotorControlNode::set_kvp_callback, this, std::placeholders::_1, std::placeholders::_2));
   }

// Function to initialize the motor
bool MotorControlNode::initialize_motor(int slave_id)
{
    // Declare some parameters
    this->declare_parameter("debug", 0); 
    int debug_param = this->get_parameter("debug").as_int();

    // Initialize EtherCAT on the specified network interface
    char IOmap[4096];
    if (ec_init(network_interface_.c_str()))
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
        RCLCPP_ERROR(this->get_logger(), "Ethercat devices connection failed on interface %s. Permission Denied!", network_interface_.c_str());
        return false;
    }

    ec_configdc();  // EtherCAT configuration
    ec_readstate();  // Read EtherCAT state
    ec_slave[slave_id].state = EC_STATE_OPERATIONAL;  // Set to operational state
    ec_writestate(slave_id);

    RCLCPP_INFO(this->get_logger(), "Slave: %s, State: %d", ec_slave[slave_id].name, ec_slave[slave_id].state); // state 8 == OPERATIONAL

    int8_t Operetion_mode = control_mode_;
    int8_t Operetion_read;
    int psize=sizeof(Operetion_mode);

    // Set control mode (e.g., velocity mode)
    ec_SDOwrite(slave_id, OPERATION_MODE, 0x00, FALSE, psize, &Operetion_mode, ethercat_timeout_);
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Read back the state to verify seted control mode
    ec_SDOread( slave_id, OPERATION_MODE_DISPLAY, 0x00, FALSE, &psize, &Operetion_read, ethercat_timeout_);
    RCLCPP_INFO(this->get_logger(), "Operetion Mode: %d ", Operetion_read);
    

    // Verify if the motor is operational
    uint16_t control_word = CONTROL_WORD_OFF;  // Motor enable 0x06：Power off motor
    ec_SDOwrite(slave_id, CONTROL_WORD, 0x00, FALSE, sizeof(control_word), &control_word, ethercat_timeout_);

    control_word = CONTROL_WORD_ON;  // Speed control enable  0x0F：Power on motor
    ec_SDOwrite(slave_id, CONTROL_WORD, 0x00, FALSE, sizeof(control_word), &control_word, ethercat_timeout_);

    // Read back the state to verify initialization
    uint16_t read_value;
    psize = sizeof(read_value);
    std::string state_name;

    ec_SDOread(slave_id, CONTROL_WORD, 0x00, FALSE, &psize, &read_value, ethercat_timeout_);
    std::string control_word_name = CONTROL_WORD_NAME(read_value);

    if (read_value == CONTROL_ENABLE) {
        motor_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Motor %d initialized successfully! Motor Control Word: 0x%02X (%s)", slave_id, read_value, control_word_name.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Motor %d failed to initialize! Motor Control Word: 0x%02X (%s)", slave_id, read_value, control_word_name.c_str());
        return false;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(5));
    ec_SDOread(slave_id, 0x6041, 0x00, FALSE, &psize, &read_value, ethercat_timeout_);
    std::string status_name = STATUS_WORD_NAME(read_value);
    RCLCPP_INFO(this->get_logger(), "Motor %d Status Word: 0x%02X (%s)", slave_id, read_value, status_name.c_str());



    // Pre set value (This values from the old code)
    int init_speed = 0;
    int init_acc   = 160;
    int init_Kvp   = 90; 

    rclcpp::sleep_for(std::chrono::milliseconds(5));
    send_motor_speed(init_speed, slave_id); 
    set_motor_acceleration(init_acc, slave_id);
    set_motor_kvp(init_Kvp, slave_id);

    // Check if debug is enabled
    if (debug_param == 1) {
        motorTestLoop(slave_id);
    }
    
    return true; 
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

    // Convert RPM to the dec value expected by the motor and send it  
    double motor_value = ((speed_rpm * 512 * encoder_resiulution) / 1875) * reduction_ratio_; // The equation from the datasheet
    int32_t s32val = static_cast<int32_t>(motor_value);
    ec_SDOwrite(slave_id, TARGET_SPEED, 0x00, FALSE, sizeof(s32val), &s32val, ethercat_timeout_);
}

// Function to send motor acceleration commands
void MotorControlNode::set_motor_acceleration(int acceleration, int slave_id)
{
    int acc_RPM;
    acc_RPM = acceleration*(M_PI*2 * wheel_radius_) *reduction_ratio_; //revolution per s^2
    int32 acc_dec=int((acc_RPM * 65536 * encoder_resiulution) / 4000000); // Equation from the datasheet
    
    while(ec_state_ && rclcpp::ok()) {
    rclcpp::sleep_for(std::chrono::milliseconds(5));
    }
    ec_state_ = true;
    ec_SDOwrite(slave_id, PROFILE_ACCELERATION, 0x00, FALSE, sizeof(acc_dec), &acc_dec, ethercat_timeout_); // Acc
    ec_SDOwrite(slave_id, PROFILE_DECELERATION, 0x00, FALSE, sizeof(acc_dec), &acc_dec, ethercat_timeout_); // Dcc
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
    ec_SDOwrite(slave_id, KVP, 0x01, FALSE, sizeof(kvp_0), &kvp_0, ethercat_timeout_);
    ec_state_ = false;

    RCLCPP_WARN(this->get_logger(), "Motor %d kvp_0 set to %d", slave_id, kvp_0);
}

// Callback for /cmd_vel topic
void MotorControlNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Check if the linear and angular velocities are the same as the past message
    if (past_twist_msg.linear.x == msg->linear.x && 
        past_twist_msg.angular.z == msg->angular.z)
    {
        return; // Exit if no change
    }


    // Calculate the motor speeds for left and right motors based on the Twist message
    double right_motor_speed = msg->linear.x + msg->angular.z * wheel_base_ / 2;
    // double left_motor_speed = msg->linear.x - msg->angular.z * wheel_base_ / 2;

    int right_rpm = static_cast<int>((right_motor_speed / (2 * M_PI * wheel_radius_)) * 60);
    // int left_rpm = static_cast<int>((left_motor_speed / (2 * M_PI * wheel_radius_)) * 60);

    // Send speed commands to the motors
    send_motor_speed(right_rpm, 1);  // Send speed to right motor (slave 1)
    // send_motor_speed(left_rpm, 2);   // Send speed to left motor (slave 2)
    
    // Update past_twist_msg with the current message
    past_twist_msg = *msg;
}

// Function to test the work of the motor
void MotorControlNode::motorTestLoop(int slave_id) {
    

    while (rclcpp::ok()) {
        // Prompt user for RPM input or 'q' to quit
        std::cout << "Enter desired RPM (or 'q' to quit): ";
        std::string input;
        std::cin >> input;

        // Check if the input is 'q' for quitting
        if (input == "q") {
            RCLCPP_INFO(this->get_logger(), "Exiting Motor Debuging Mode!");
            break;
        }

        // Try to convert the input to an integer RPM value
        try {
            int32_t rpm = std::stoi(input);
            if (rpm > 1000) { RCLCPP_INFO(this->get_logger(), "You Can't Set Above 1000 RPM in Test Mode!"); continue; }

            int32_t read_real;
            int psize_real = sizeof(read_real);
            uint16_t read_word;
            int psize_word = sizeof(read_word);
            int8_t read_mode;
            int psize_mode=sizeof(read_mode);

            // Run the motor test with the specified RPM
            send_motor_speed(rpm, slave_id);
            RCLCPP_INFO(this->get_logger(), "Sent Motor Target Speed:  %d RPM", rpm);
            rclcpp::sleep_for(std::chrono::seconds(1));
            ec_SDOread(slave_id, REAL_SPEED, 0x00, FALSE, &psize_real, &read_real, ethercat_timeout_);
            int64_t read_RPM = static_cast<int64_t>(read_real) * 1875 / (512 * encoder_resiulution);
            RCLCPP_WARN(this->get_logger(), "Motor %d Real Speed:       %ld RPM", slave_id, read_RPM);

            // Read Status, Control and Mode Wrods
            ec_SDOread(slave_id, STATUS_WORD, 0x00, FALSE, &psize_word, &read_word, ethercat_timeout_);
            std::string status_name = STATUS_WORD_NAME(read_word);
            RCLCPP_INFO(this->get_logger(), "Motor %d Status Word:      0x%02X (%s)", slave_id, read_word, status_name.c_str());
            ec_SDOread(slave_id, CONTROL_WORD, 0x00, FALSE, &psize_word, &read_word, ethercat_timeout_);
            std::string control_name = CONTROL_WORD_NAME(read_word);
            RCLCPP_INFO(this->get_logger(), "Motor %d Control Word:     0x%02X (%s)", slave_id, read_word, control_name.c_str());
            ec_SDOread( slave_id, OPERATION_MODE, 0x00, FALSE,&psize_mode, &read_mode, ethercat_timeout_);
            std::string mode_name = OPERATION_MODE_NAME(read_mode);
            RCLCPP_INFO(this->get_logger(), "Motor %d Mode Word:        0x%02X (%s)", slave_id, read_mode, mode_name.c_str());

            rclcpp::sleep_for(std::chrono::seconds(8));
            
            // Reset motor speed to zero
            send_motor_speed(0, slave_id);
            RCLCPP_INFO(this->get_logger(), "Sent motor Target Speed:  0 RPM");
            rclcpp::sleep_for(std::chrono::seconds(1));
            ec_SDOread(slave_id, REAL_SPEED, 0x00, FALSE, &psize_real, &read_real, ethercat_timeout_);
            RCLCPP_WARN(this->get_logger(), "Motor %d Real Speed:       %d RPM", slave_id, read_real);
        }
         
        catch (const std::invalid_argument&) {
            // If input is not a valid integer or 'q'
            std::cout << "Invalid input. Please enter a number for RPM or 'q' to quit." << std::endl;
        }
    }
}

// Service Callback Function
void MotorControlNode::set_acceleration_callback(
    const std::shared_ptr<servo_msgs::srv::SetAcceleration::Request> request,
    std::shared_ptr<servo_msgs::srv::SetAcceleration::Response> response) 
{
    MotorControlNode::set_motor_acceleration(request->acceleration, request->slave_id);
    response->success = true;
    response->message = "Acceleration set successfully.";
}

// Service Callback Function
void MotorControlNode::set_kvp_callback(
    const std::shared_ptr<servo_msgs::srv::SetKvp::Request> request,
    std::shared_ptr<servo_msgs::srv::SetKvp::Response> response) 
{
    MotorControlNode::set_motor_kvp(request->kvp, request->slave_id);
    response->success = true;
    response->message = "Kvp set successfully.";
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

    /* if (!motor_control_node->initialize_motor(2)) {
        RCLCPP_ERROR(motor_control_node->get_logger(), "Failed to initialize left motor ");
        return 1;
    } */

    rclcpp::spin(motor_control_node);
    motor_control_node->send_motor_speed(0, 1);
    // motor_control_node->send_motor_speed(0, 2);
    ec_close();
    rclcpp::shutdown();
    return 0;
}
