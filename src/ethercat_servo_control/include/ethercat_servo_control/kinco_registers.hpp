// motor_control_registers.hpp
#ifndef MOTOR_CONTROL_REGISTERS_HPP
#define MOTOR_CONTROL_REGISTERS_HPP
#include <cstdint>  // For fixed-width integer types like uint16_t



// Control and status registers
const uint16_t CONTROL_WORD = 0x6040;              // Control word register
const uint16_t STATUS_WORD = 0x6041;               // Status word register

// Operation mode state registers
const uint16_t OPERATION_MODE = 0x6060;            // Operation mode register
const uint16_t OPERATION_MODE_DISPLAY = 0x6061;    // Mode display register

const uint16_t CONTROL_DISABLE = 0x06;             // Motor disable command
const uint16_t CONTROL_ENABLE = 0x0F;              // Motor enable command

// Operation modes
const int8_t POSITION_CONTROL_MODE = 1;      // Position control mode
const int8_t VELOCITY_POSE_MODE = 3;      // Velocity control with profile acceleration/deceleration
const int8_t VELOCITY_FAST_MODE = -3;        // Direct velocity control mode
const int8_t TORQUE_CONTROL_MODE = 4;        // Torque control mode
const int8_t PULSE_TRAIN_CONTROL_MODE = -4;  // Pulse train control mode
const int8_t HOMING_MODE = 6;                // Homing mode for setting reference position
const int8_t DIFFERENTIAL_MODE = 7;          // Differential complementarity based on CANopen

// Position control registers
const uint16_t TARGET_POSITION = 0x607A;           // Target position in position mode
const uint16_t POSITION_ACTUAL = 0x6063;           // Actual position of motor
const uint16_t MAX_FOLLOWING_ERROR = 0x6065;       // Following error alarm
const uint16_t TARGET_POSITION_WINDOW = 0x6067;    // Target position reached error range

// Speed control registers
const uint16_t TARGET_SPEED = 0x60FF;              // Target speed in velocity mode
const uint16_t PROFILE_SPEED = 0x6081;             // Profile speed in position mode
const uint16_t MAX_SPEED = 0x6080;                 // Maximum speed setting
const uint16_t REAL_SPEED = 0x606C;                // Real speed reading

// Acceleration and Deceleration registers
const uint16_t PROFILE_ACCELERATION = 0x6083;      // Profile acceleration
const uint16_t PROFILE_DECELERATION = 0x6084;      // Profile deceleration

// Torque control registers
const uint16_t TARGET_TORQUE = 0x6071;             // Torque command in torque mode
const uint16_t MAX_CURRENT_COMMAND = 0x6073;       // Maximum current command in torque mode

// Homing control registers
const uint16_t HOMING_METHOD = 0x6098;             // Homing method
const uint16_t HOMING_SPEED_SWITCH = 0x6099;       // Homing speed for switch signal
const uint16_t HOMING_SPEED_ZERO = 0x609A;         // Homing speed for zero position
const uint16_t HOMING_OFFSET = 0x607C;             // Offset after homing
const uint16_t HOMING_POWER = 0x609B;            // Power level for homing operation

// Velocity loop tuning registers
const uint16_t KVP = 0x60F9;                       // Proportional gain for velocity
const uint16_t KVI = 0x60F9;                       // Integral gain for velocity loop

// Position loop tuning registers
const uint16_t KPP = 0x60FB;                       // Proportional gain for position loop
const uint16_t K_VELOCITY_FF = 0x60FB;             // Velocity feedforward for position loop
const uint16_t K_ACCELERATION_FF = 0x60FB;         // Acceleration feedforward for position loop

// Input/Output configuration registers
const uint16_t INPUT_PORT_STATUS = 0x60FD;         // Status of input port
const uint16_t OUTPUT_PORT_STATUS = 0x60FE;        // Status of output port

// Emergency and error-related registers
const uint16_t EMERGENCY_ID = 0x1014;              // Emergency message ID
const uint16_t ERROR_CODE = 0x2601;                // Error code
const uint16_t ERROR_STATE = 0x2602;             // Detailed error state information

// Network and configuration registers
const uint16_t CANOPEN_BAUD_RATE = 0x2F81;         // CANopen baud rate setting
const uint16_t NODE_GUARDING = 0x100C;             // Node guarding for CANopen communication
const uint16_t HEARTBEAT_TIME = 0x1017;            // Heartbeat time for CANopen communication


// Control and status words for the Kinco servo motor
const uint16_t CONTROL_WORD_OFF = 0x06;         // Power off motor
const uint16_t CONTROL_WORD_ON = 0x0F;          // Power on motor
const uint16_t CONTROL_WORD_QUICK_STOP = 0x0B;  // Quick stop
const uint16_t CONTROL_WORD_START = 0x2F;       // Start
const uint16_t CONTROL_WORD_POSITION_RELATIVE = 0x4F; // Start relative positioning
const uint16_t CONTROL_WORD_POSITION_ABSOLUTE = 0x103F; // Start absolute positioning with target change
const uint16_t CONTROL_WORD_HOME = 0x1F;        // Home positioning
const uint16_t CONTROL_WORD_CLEAR = 0x80;       // Clear internal shooting

// Status words for the Kinco servo motor
const uint16_t STATUS_READY_ON = 0x0001;
const uint16_t STATUS_SWITCH_ON = 0x0002;
const uint16_t STATUS_OPERATION_ENABLE = 0x0004;
const uint16_t STATUS_FAULT = 0x0008;
const uint16_t STATUS_VOLTAGE_ENABLE = 0x0010;
const uint16_t STATUS_QUICK_STOP = 0x0020;
const uint16_t STATUS_SWITCH_DISABLED = 0x0040;
const uint16_t STATUS_MANUFACTURE_0 = 0x0100;
const uint16_t STATUS_REMOTE = 0x0200;
const uint16_t STATUS_TARGET_REACHED = 0x0400;
const uint16_t STATUS_INTLIM_ACTIVE = 0x0800;
const uint16_t STATUS_SETPOINT_ACK = 0x1000;
const uint16_t STATUS_FOLLOWING_ERROR = 0x2000;
const uint16_t STATUS_COMM_FOUND = 0x4000;
const uint16_t STATUS_REFERENCE_FOUND = 0x8000;
const uint16_t STATUS_READY = 0x5437;
const uint16_t STATUS_ROTATING = 0x4437;

// Operation modes
const uint16_t OPERATION_MODE_VELOCITY = 0x03;

// Registers for setting parameters
const uint16_t REG_CONTROL_MODE = 0x6060;       // Control mode register
const uint16_t REG_MODE_DISPLAY = 0x6061;       // Mode display register
const uint16_t REG_CONTROL_WORD = 0x6040;       // Control word register
const uint16_t REG_STATUS_WORD = 0x6041;        // Status word register
const uint16_t REG_TARGET_VELOCITY = 0x60FF;    // Target velocity register
const uint16_t REG_ACCELERATION = 0x6083;       // Acceleration register
const uint16_t REG_DECELERATION = 0x6084;       // Deceleration register
const uint16_t REG_KVP = 0x60F9;                // Kvp register (0x60F9, subindex 1)

// Timeout settings
const int32_t ETHERCAT_TIMEOUT = 500; // Set this to the required EtherCAT timeout value

// Pulse Input Mode Registers
const uint16_t PULSE_CW_MODE = 0x2508;           // Pulse Control Mode (CW/CCW, A/B, etc.)
const uint16_t PULSE_FILTER = 0x2506;            // Pulse filter for signal smoothing

// Quick Stop Mode Registers
const uint16_t QUICK_STOP_MODE = 0x605A;         // Quick stop configuration


#define CONTROL_WORD_NAME(value) \
    ((value) == CONTROL_DISABLE ? "CONTROL_DISABLE" : \
    (value) == CONTROL_ENABLE ? "CONTROL_ENABLE" : \
    (value) == CONTROL_WORD_QUICK_STOP ? "CONTROL_QUICK_STOP" : \
    (value) == CONTROL_WORD_START ? "CONTROL_START" : \
    (value) == CONTROL_WORD_POSITION_RELATIVE ? "CONTROL_RELATIVE_POSITION" : \
    (value) == CONTROL_WORD_POSITION_ABSOLUTE ? "CONTROL_ABSOLUTE_POSITION" : \
    (value) == CONTROL_WORD_HOME ? "CONTROL_HOME" : \
    (value) == CONTROL_WORD_CLEAR ? "CONTROL_CLEAR_SHOOTING" : "UNKNOWN_CONTROL_WORD")

#define OPERATION_MODE_NAME(value) \
    ((value) == POSITION_CONTROL_MODE ? "POSITION_CONTROL_MODE" : \
    (value) == VELOCITY_POSE_MODE ? "VELOCITY_POSE_MODE" : \
    (value) == VELOCITY_FAST_MODE ? "VELOCITY_FAST_MODE" : \
    (value) == TORQUE_CONTROL_MODE ? "TORQUE_CONTROL_MODE" : \
    (value) == PULSE_TRAIN_CONTROL_MODE ? "PULSE_TRAIN_CONTROL_MODE" : \
    (value) == HOMING_MODE ? "HOMING_MODE" : \
    (value) == DIFFERENTIAL_MODE ? "DIFFERENTIAL_MODE" : "UNKNOWN_OPERATION_MODE")

#define STATUS_WORD_NAME(value) \
    ((value) == 0x0001 ? "READY_ON" : \
    (value) == 0x0002 ? "SWITCH_ON" : \
    (value) == 0x0004 ? "OPERATION_ENABLE" : \
    (value) == 0x0008 ? "FAULT" : \
    (value) == 0x0010 ? "VOLTAGE_ENABLE" : \
    (value) == 0x0020 ? "QUICK_STOP" : \
    (value) == 0x0040 ? "SWITCH_DISABLED" : \
    (value) == 0x0100 ? "MANUFACTURE_0" : \
    (value) == 0x0200 ? "REMOTE" : \
    (value) == 0x0400 ? "TARGET_REACHED" : \
    (value) == 0x0800 ? "INTLIM_ACTIVE" : \
    (value) == 0x1000 ? "SETPOINT_ACK" : \
    (value) == 0x2000 ? "FOLLOWING_ERROR" : \
    (value) == 0x4000 ? "COMMUNICATION_FOUND" : \
    (value) == 0x5037 ? "ROTATING" : \
    (value) == 0x4437 ? "READY" : \
    (value) == 0x4033 ? "INITIALISED" : \
    (value) == 0x8000 ? "REFERENCE_FOUND" : "UNKNOWN_STATE")

#endif // MOTOR_CONTROL_REGISTERS_HPP
