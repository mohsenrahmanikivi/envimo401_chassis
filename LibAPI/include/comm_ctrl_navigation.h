#ifdef __cplusplus
extern "C" {
#endif

#ifndef COMM_CTRL_NAVIGATION_H
#define COMM_CTRL_NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>

#define TYPE_ID_NUM                     12   //Number of callback functions
//--------------------------DATA CALL BACK INDEX--------------------------
#define Chassis_Data_Motors_Speed       1
#define Chassis_Data_Car_Speed       	2
#define Chassis_Data_Front_Ticks        3
#define Chassis_Data_Rear_Ticks         4
#define Chassis_Data_Odom_Pose_xy		5
#define Chassis_Data_Odom_Euler_xy		6
#define Chassis_Data_Odom_Euler_z		7
#define Chassis_Data_Odom_Linevel_xy	8
#define Chassis_Data_Imu_Gyr            9	//IMU Gyroscope data 陀螺仪数据上报
#define Chassis_Data_Imu_Acc            10	//IMU Accelerometer data 加速度计数据上�?

//-----------------------Event---------------------------------------------
#define ChassisBootReadyEvent  		1	// Chassis central control boot OK
#define PadPowerOffEvent      		2	// The chassis will power off
#define OnEmergeStopEvent     		3	// The chassis emergency stop button is triggered
#define OutEmergeStopEvent    		4	// The chassis emergency stop button recover
#define OnLockedRotorProtectEvent	5	// the chassis motor locked-rotor error for more than 15 seconds
#define OutLockedRotorProtectEvent	6   // the chassis motor no longer locked-rotor
#define OnLostCtrlProtectEvent		7	// the chassis motor lost control
#define OutLostCtrlProtectEvent		8	// the chassis motor no longer lost control
#define CalibrateGyroSuccess		9	// Chassis gyroscope calibration successful
#define CalibrateGyroFail			10  // Chassis gyroscope calibration failed
#define CalibratePasheCurrentSuccess	11	// Chassis gyroscope calibration successful
#define CalibratePasheCurrentFail		12  // Chassis gyroscope calibration failed
#define ChassisLockRotorWarning		    13  // Chassis locked rotor warning for more than 5 seconds


//---------The proportional coefficient of the callback gyro data------------------
#define CHASSIS_IMU_GYR_VALUE_TRANS_SCALE		900			//900~~32768(MAX value for int16: map table ) * 57.3(degrees for 1 rad) /2000(Gyroscope range: 2000dps)
#define CHASSIS_IMU_ACC_VALUE_TRANS_SCALE		4000		//4000~~32768(MAX value for int16: map table )/8(Accelerometer range: 8g)

#define LINE_SPEED_TRANS_GAIN_MPS		3600	//3600-->1m/s
#define ANGULAR_SPEED_TRANS_GAIN_RADPS		1000	//1000-->1rad/s

#define NO_LOAD  	0		// Set the chassis parameters as no-load parameters, The chassis defaults to no load
#define	FULL_LOAD 	1		// Set the chassis parameters as full load parameters

//-----------------------Version V0.6 and above---------------------------
#define LOCK_MODE	0		// Lock the car
#define CTRL_MODE	1		// Control the car
#define PUSH_MODE	2		// push the car
#define EMERG_MODE	3		// emergency
#define ERROR_MODE	4		// Internal error


//-------------------Timestamp------------------------
#define MAX_BASIC_FRM_SZ 0x1F

#pragma pack(1)
typedef struct StampedBasicFrame_{
    uint32_t type_id;                 //Data type number
    uint64_t timestamp;               //Linux timestamp
    char  data[MAX_BASIC_FRM_SZ];     //Chassis specific data
} StampedBasicFrame;
#pragma pack()
typedef void (*h_aprctrl_datastamped_t)(StampedBasicFrame* frm);
typedef void (*h_aprctrl_event_t)(int32_t event_num);

typedef struct {
    h_aprctrl_datastamped_t on_new_data;
}s_aprctrl_datastamped_t;

typedef struct {
    h_aprctrl_event_t event_callback;
}s_aprctrl_event_t;

typedef enum {
    Host = 1,
    Central = 2, //底盘只连中控�?
    Motor0	= 3,
    Motor1	= 4,
    Motor2	= 5,
    Motor3	= 6,
    BMS		= 7,
	Brake   = 8 //brake sticking
}board_name_e;

typedef enum {
   comu_serial = 0,
   comu_can = 1
}comu_choice_e;

typedef enum {
	motor_front = 0,
	motor_rear	= 1
}motor_index_e;

typedef struct{
	int16_t fl_speed;//front Left wheel speed
	int16_t fr_speed;//front Right wheel speed
	int16_t rl_speed;//rear Left wheel speed
	int16_t rr_speed;//rear Right wheel speed
}chassis_motors_speed_data_t;

typedef struct{
	int16_t car_speed;//Vehicle linear speed
	int16_t turn_speed;//Turning speed
}chassis_car_speed_data_t;

typedef struct{
	int32_t fl_ticks;//front Left wheel ticks
	int32_t fr_ticks;//front Right wheel ticks
}front_motors_ticks_t;

typedef struct{
	int32_t rl_ticks;//rear Left wheel ticks
	int32_t rr_ticks;//rear Right wheel ticks
}rear_motors_ticks_t;

typedef struct{
	float pos_x;
	float pos_y;
}odom_pos_xy_t;

typedef struct{
	float euler_x;
	float euler_y;
}odom_euler_xy_t;

typedef struct{
	float euler_z;
}odom_euler_z_t;

typedef struct{
	float vel_line_x;
	float vel_line_y;
}odom_vel_line_xy_t;

typedef struct{
	int16_t gyr[3];
}imu_gyr_original_data_t;

typedef struct{
	int16_t acc[3];
}imu_acc_original_data_t;


void aprctrl_datastamped_jni_register(s_aprctrl_datastamped_t* f);  //The callback registration function
void aprctrl_eventcallback_jni_register(s_aprctrl_event_t* f);     //Event callback registration function

uint32_t get_err_state(board_name_e board_name);//Gets the software error status
int16_t  get_bat_soc(void);//Gets the percentage of battery left
int16_t  get_bat_charging(void);//Gets the charging status of the battery
int32_t  get_bat_mvol(void);//Gets battery voltage, mV
int32_t  get_bat_mcurrent(void);//Gets battery current, mA
int16_t  get_bat_temp(void);// Gets battery temperature
int16_t  get_chassis_work_model(void);//Get the chassis working mode
uint8_t  get_chassis_load_state(void);//Gets whether the chassis parameters are empty or full load, 0: no_load, 1: full_load
uint16_t get_chassis_mode(void);//Gets chassis mode
int16_t  get_ctrl_cmd_src(void); // Get the source of the control command 
int32_t  get_vehicle_meter(void); //Get the total mileage meters of the chssis
uint16_t get_host_version(void);
uint16_t get_host_patch_version(void);
uint16_t get_chassis_central_version(void);
uint16_t get_chassis_motor_version(void);
uint16_t get_chassis_brake_version(void);
int16_t  get_line_forward_max_vel_fb(void);
int16_t  get_line_backward_max_vel_fb(void);
int16_t  get_angular_max_vel_fb(void);
int32_t  getAllIapProgress(void);
void 	iapCentralBoard(void);//Operate on the Central board: IAP
void 	iapMotorBoard(motor_index_e motor_index);//Operate on the Motor board: IAP, motor_index:0--front motor; 1--rear motor
void    iapBrakeBoard(void);//Operate on the Brake board: IAP
bool    isHostIapOver(void);//Query whether a single IAP ends
int16_t getHostIapResult(void);
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
