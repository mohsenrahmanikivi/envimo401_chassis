/**
  ************************************* Copyright ******************************
  *
  *                 (C) Copyright 2023,Segway-Ninebot,China,beijing
  *                            All Rights Reserved
  *
  *
  * FileName   : comm_ctrl_navigation.h
  * Version    : v1.0
  * Author     : SegwayRobotics Team
  * Date       : 2023-08-01
  * Description:
  *   1. Dynamic link library interface.
  * License:
  *     Copyright [2023] [Segway Robotics]
  *
  *		Licensed under the Apache License, Version 2.0 (the "License");
  *		you may not use this file except in compliance with the License.
  *		You may obtain a copy of the License at
  *
  *			http://www.apache.org/licenses/LICENSE-2.0
  *
  *		Unless required by applicable law or agreed to in writing, software
  *		distributed under the License is distributed on an "AS IS" BASIS,
  *		WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  *		See the License for the specific language governing permissions and
  *		limitations under the License.
  *
  * https://robotics.segway.com
  * http://www.segwayrobotics.com
  * https://www.ninebot.com
  *
  ******************************************************************************
 */


#ifdef __cplusplus
extern "C" {
#endif

#ifndef COMM_CTRL_NAVIGATION_H
#define COMM_CTRL_NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>


#define TYPE_ID_NUM                           8   // The total number of callback functions

// Data callback indices
#define Chassis_Data_Speed                    1   // Index for the speed data callback from the chassis
#define Chassis_Data_Ticks                    2   // Index for the ticks data callback from the chassis
#define Chassis_Data_Odom_Pose_xy             3   // Index for the XY position data callback from the chassis odometry
#define Chassis_Data_Odom_Euler_xy            4   // Index for the XY Euler angles data callback from the chassis odometry
#define Chassis_Data_Odom_Euler_z             5   // Index for the Z Euler angle data callback from the chassis odometry
#define Chassis_Data_Odom_Linevel_xy          6   // Index for the XY linear velocity data callback from the chassis odometry
#define Chassis_Data_Imu_Gyr                  7   // Index for the gyroscope data callback from the IMU on the chassis
#define Chassis_Data_Imu_Acc                  8   // Index for the accelerometer data callback from the IMU on the chassis

// Event identifiers
#define ChassisBootReadyEvent                 1   // Event indicating that the chassis central control boot process is complete
#define PadPowerOffEvent                      2   // Event indicating that the chassis will power off
#define OnEmergeStopEvent                     3   // Event indicating that the chassis emergency stop button has been triggered
#define OutEmergeStopEvent                    4   // Event indicating that the chassis emergency stop button has been recovered
#define OnLockedRotorProtectEvent             5   // Event indicating that the chassis motor is in a locked-rotor state
#define OutLockedRotorProtectEvent            6   // Event indicating that the chassis motor is no longer in a locked-rotor state
#define OnLostCtrlProtectEvent                7   // Event indicating that the chassis motor has lost control
#define OutLostCtrlProtectEvent               8   // Event indicating that the chassis motor has regained control
#define CalibrateGyroSuccess                  9   // Event indicating that the chassis gyroscope calibration was successful
#define CalibrateGyroFail                    10   // Event indicating that the chassis gyroscope calibration failed
#define CalibratePasheCurrentSuccess         11   // Event indicating that the chassis phase current calibration was successful
#define CalibratePasheCurrentFail            12   // Event indicating that the chassis phase current calibration failed

// Proportional coefficients for callback data
#define CHASSIS_IMU_GYR_VALUE_TRANS_SCALE   900   // Maps max int16 (32768) to gyro range (2000dps), scales by 57.3 (degrees in 1 rad)
#define CHASSIS_IMU_ACC_VALUE_TRANS_SCALE  4000   // Maps max int16 (32768) to accelerometer range (8g)

#define LINE_SPEED_TRANS_GAIN_MPS          3600   // 3600 --> 1m/s
#define ANGULAR_SPEED_TRANS_GAIN_RADPS     1000   // 1000 --> 1rad/s

// Chassis load parameters
#define NO_LOAD                              0   // Set the chassis parameters as no-load parameters, The chassis defaults to no load
#define	FULL_LOAD                            1   // Set the chassis parameters as full load parameters

// Chassis modes
#define LOCK_MODE                            0   // Lock the car
#define CTRL_MODE                            1   // Control the car
#define PUSH_MODE                            2   // push the car
#define EMERG_MODE                           3   // emergency
#define ERROR_MODE                           4   // Internal error


//-------------------Timestamp------------------------
#define MAX_BASIC_FRM_SZ 0x1F   // Define the maximum size of the basic frame

#pragma pack(1)
typedef struct StampedBasicFrame_{
    uint32_t type_id;   // Data type number
    uint64_t timestamp; // Timestamp
    char  data[MAX_BASIC_FRM_SZ]; // Chassis specific data
} StampedBasicFrame;
#pragma pack()

// Typedef for a function pointer to a function that takes a pointer to a StampedBasicFrame as an argument
typedef void (*h_aprctrl_datastamped_t)(StampedBasicFrame* frm);

// Typedef for a function pointer to a function that takes an integer as an argument
typedef void (*h_aprctrl_event_t)(int32_t event_num);

// Struct that holds a function pointer to a function that will be called when new data is available
typedef struct {
    h_aprctrl_datastamped_t on_new_data;
}s_aprctrl_datastamped_t;

// Struct that holds a function pointer to a function that will be called when an event occurs
typedef struct {
    h_aprctrl_event_t event_callback;
}s_aprctrl_event_t;


// Enumeration of board names
typedef enum {
    Host    = 1,        // Host board
    Central = 2,        // Central control board
    Motor   = 3,        // Motor board
    BMS     = 4         // Battery Management System board
} board_name_e;

// Enumeration of communication choices
typedef enum {
    comu_serial = 0,    // Serial communication
    comu_can    = 1     // CAN communication
} comu_choice_e;

// Structure that holds the speed data of the chassis
typedef struct{
    int16_t l_speed;     // Left wheel speed
    int16_t r_speed;     // Right wheel speed
    int16_t car_speed;   // Vehicle linear speed
    int16_t turn_speed;  // Turning speed
} chassis_speed_data_t;

// Structure that holds the motor ticks data
typedef struct{
    int32_t l_ticks;     // Left wheel ticks
    int32_t r_ticks;     // Right wheel ticks
} motor_ticks_t;

// Structure that holds the odometry position in XY plane
typedef struct{
    float pos_x;         // Position along the X-axis
    float pos_y;         // Position along the Y-axis
} odom_pos_xy_t;

// Structure that holds the XY Euler angles from the odometry
typedef struct{
    float euler_x;       // Euler angle along the X-axis
    float euler_y;       // Euler angle along the Y-axis
} odom_euler_xy_t;

// Structure that holds the Z Euler angle from the odometry
typedef struct{
    float euler_z;       // Euler angle along the Z-axis
} odom_euler_z_t;

// Structure that holds the XY linear velocity data from the odometry
typedef struct{
    float vel_line_x;    // Linear velocity along the X-axis
    float vel_line_y;    // Linear velocity along the Y-axis
} odom_vel_line_xy_t;

// Structure that holds the gyroscope data from the IMU on the chassis
typedef struct{
    int16_t gyr[3];      // Gyroscope data (X, Y, Z axis)
} imu_gyr_original_data_t;

// Structure that holds the accelerometer data from the IMU on the chassis
typedef struct{
    int16_t acc[3];      // Accelerometer data (X, Y, Z axis)
} imu_acc_original_data_t;


/**
* @brief   Register the callback function
* @param   f: a pointer to the callback function structure
*/
void aprctrl_datastamped_jni_register(s_aprctrl_datastamped_t* f);

/**
* @brief   Register the event callback function
* @param   f: a pointer to the event callback function structure
*/
void aprctrl_eventcallback_jni_register(s_aprctrl_event_t* f);

/**
* @brief   Get the software error status
* @param   board_name: The name of the board
* @return  The error status
*/
uint32_t get_err_state(board_name_e board_name);

/**
* @brief   Get the percentage of battery left
* @return  The percentage of battery left
*/
int16_t  get_bat_soc(void);

/**
* @brief   Get the charging status of the battery
* @return  The charging status of the battery
*/
int16_t  get_bat_charging(void);

/**
* @brief   Get battery voltage, mV
* @return  The battery voltage
*/
int32_t  get_bat_mvol(void);

/**
* @brief   Get battery current, mA
* @return  The battery current
*/
int32_t  get_bat_mcurrent(void);

/**
* @brief   Get battery temperature
* @return  The battery temperature
*/
int16_t  get_bat_temp(void);

/**
* @brief   Get the chassis working mode
* @return  The working mode of chassis
*/
int16_t  get_chassis_work_model(void);

/**
* @brief   Get whether the chassis parameters are empty or full load
* @return  0: no load, 1: full load
*/
uint8_t  get_chassis_load_state(void);

/**
* @brief   Get chassis mode
* @return  The chassis mode
*/
uint16_t get_chassis_mode(void);

/**
* @brief   Get the source of the control command
* @return  The source of control command
*/
int16_t  get_ctrl_cmd_src(void);

/**
* @brief   Get the total mileage meters of the chassis
* @return  The total mileage of chassis
*/
int32_t  get_vehicle_meter(void);

/**
* @brief   Get the version of the host board
* @return  The version of host board
*/
uint16_t get_host_version(void);

/**
* @brief   Get the version of the central control board on the chassis
* @return  The version of central control board
*/
uint16_t get_chassis_central_version(void);

/**
* @brief   Get the version of the motor board on the chassis
* @return  The version of motor board
*/
uint16_t get_chassis_motor_version(void);

/**
* @brief   Get maximum forward linear velocity feedback
* @return  Max forward linear velocity feedback
*/
int16_t  get_line_forward_max_vel_fb(void);

/**
* @brief   Get maximum backward linear velocity feedback
* @return  Max backward linear velocity feedback
*/
int16_t  get_line_backward_max_vel_fb(void);

/**
* @brief   Get maximum angular velocity feedback
* @return  Max angular velocity feedback
*/
int16_t  get_angular_max_vel_fb(void);

/**
* @brief   Get IAP total progress
* @return  IAP total progress
*/
int32_t  getIapTotalProgress(void);

/**
* @brief   Operate on the Central board: IAP
*/
void iapCentralBoard(void);

/**
* @brief   Operate on the Motor board: IAP
*/
void iapMotorBoard(void);

/**
* @brief   Query whether a single IAP ends
* @return  true if IAP ends, false otherwise
*/
bool isHostIapOver(void);

/**
* @brief   Get the result of the IAP of the host board
* @return  The result of the IAP
*/
int16_t getHostIapResult(void);

/**
* @brief   Get the error code of the IAP of the host board
* @return  The error code of the IAP
*/
int16_t getHostIapErrorCode(void);

/**
* @brief   Get chassis hang_mode
* @return  1: in Hang_mode; 0: not in Hand_mode
*/
int16_t get_chassis_hang_mode(void);

/**
* @brief   Get the status of switch for charging MOS on the central board
* @return  charging:1; no charge:0
*/
int16_t get_charge_mos_ctrl_status(void);

/**
* @brief   Get the SOC threshold of a low-power shutdown of the central board
* @return  The SOC threshold
*/
uint16_t get_low_power_shutdown_threshold(void);

/**
* @brief   Get the charge mode
* @return  0:no charge;1:line charge;2:dock charge
*/
uint16_t get_charge_mode_status(void);

/**
* @brief   Get the charge connect
* @return  0:no connect;1:charge connect
*/
uint16_t get_charge_connect_status(void);

/**
* @brief   Set the linear and angular speeds of the vehicle
* @param   linear_x: The linear speed in m/s
* @param   angular_z: The angular speed in rad/s
* @details The acceleration and deceleration parameters are as follows:
*        No Load: linear speed acc and dec is 2 m/s2, angular speed acc and dec is 4 rad/s2
*      Full Load: linear speed acc and dec is 2 m/s2, angular speed acc and dec is 2.68 rad/s2
*/
void set_cmd_vel(double linear_x,double angular_z);

/**
* @brief   Set the maximum linear velocity in the direction of advance
* @param   linear_forward_max_x: The maximum linear velocity
* @return  Status of the operation
* @details The default value is 3 m/s, and the maximum configurable value is 3 m/s.
*/
uint8_t set_line_forward_max_vel(double linear_forward_max_x);

/**
* @brief   Set the maximum linear velocity in the backward direction
* @param   linear_backward_max_x: The maximum linear velocity in the backward direction
* @return  Status of the operation
* @details The default value is 2 m/s, and the maximum configurable value is 2 m/s.
*/
uint8_t set_line_backward_max_vel(double linear_backward_max_x);

/**
* @brief   Set the maximum angular velocity
* @param   angular_max_z: The maximum angular velocity
* @return  Status of the operation
* @details The default value is 3 rad/s, and the maximum configurable value is 3 rad/s.
*/
uint8_t set_angular_max_vel(double angular_max_z);

/**
* @brief   Set chassis movement enable
* @param   enable_flag: The flag to enable or disable the movement
* @return  Status of the operation
*/
uint8_t set_enable_ctrl(uint16_t enable_flag);

/**
* @brief   Chassis initialization
* @return  Status of the initialization
*/
int init_control_ctrl(void);

/**
* @brief   Chassis software finished running
*/
void exit_control_ctrl(void);

/**
* @brief   Set the serial port name
* @param   serial_no: The name of the serial port
*/
void set_smart_car_serial(const char * serial_no);

/**
* @brief   Set communication interface
* @param   comu_choice: 'comu_serial': serial port; 'comu_can': CAN port
*/
void set_comu_interface(comu_choice_e comu_choice);

/**
* @brief   Sets whether the chassis parameters are empty or full load
* @param   newLoadSet: 0: no_load, 1: full_load
* @return  Status of the operation
*/
uint8_t set_chassis_load_state(int16_t newLoadSet);

/**
* @brief   Set two - wheel differential chassis shutdown
* @return  Status of the operation
*/
uint8_t set_chassis_poweroff(void);

/**
* @brief   An order to remove the push status
* @return  Status of the operation
*/
uint8_t set_remove_push_cmd(void);

/**
* @brief   Cancel the IAP operation of the host board
*/
void setHostIapCanceled(void);

/**
* @brief   Set chassis hang_mode
* @param   enterHand: 1: enter the Hang_mode; 0: exit the Hand_mode
*/
void set_chassis_hang_mode(int16_t enterHand);

/**
* @brief   Set the switch for charging MOS on the central board
* @param   on: charging:1, stop charging:0
* @return  Status of the operation
*/
uint8_t set_charge_mos_ctrl(bool on);

/**
* @brief   Set the wording status of the buzzer
* @param   buzzerSet: bit0:imu_calib_start; bit1:imu_calib_end;bit2:lowPower;bit3:error:bit4:chageFsm;bit5:xx;bit6:poweroff
* @return  Status of the operation
*/
uint8_t set_buzzer_work_status(uint16_t buzzerSet);

/**
* @brief   Calibrate the IMU of the chassis
* @return  ret:0:success; -1: fail to send cmd; -2: fail to calib; -3: overtime
*/
int8_t set_calib_gyro(void);

/**
* @brief   Set the SOC threshold for a low-power shutdown
* @param   low_power_shutdown_threshold: The SOC threshold
* @return  Status of the operation
*/
uint8_t set_low_power_shutdown_threshold(uint16_t low_power_shutdown_threshold);

/**
* @brief   Perform IAP on a single board
* @param   path: The path to the firmware file
* @param   boradname: The name of the board
* @param   version: The version of the firmware
* @return  The result of the operation
*/
extern int32_t IapSingerBoard(char * path,char * boradname,char* version);

#endif

#ifdef __cplusplus
}
#endif
