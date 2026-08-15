/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "esp32_i2c.h"
#include "fdcan.h"
#include "motor.h"
#include "pid.h"
#include "as5047p.h"
#include "bno085.h"
#include "pid_2dof.h"
#include "blf_laser_sensor.h"
#include "sensorFusion.h"
#include "omni_kinematics.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
StaticTask_t xPs4TaskTCB;
StackType_t xPs4Stack[1024];

StaticTask_t xCanTransmitTaskTCB;
StackType_t xCanTransmitStack[512];

StaticTask_t xPidTaskTCB;
StackType_t xPidStack[512];

StaticTask_t xPidStepUpTaskTCB;
StackType_t xPidStepUpStack[512];

//StaticTask_t xSwerveTaskTCB;
//StackType_t xSwerveStack[512];

StaticTask_t xBno085TaskTCB;
StackType_t xBno085Stack[256];

//StaticTask_t xMech1TaskTCB;
//StackType_t xMech1Stack[512];

StaticTask_t xCascadeMechTaskTCB;
StackType_t xCascadeMechStack[512];

StaticTask_t xAutoNavTaskTCB;
StackType_t xAutoNavStack[512];

StaticTask_t xStepUpTaskTCB;
StackType_t xStepUpStack[512];

StaticTask_t xAutoStateMachineTaskTCB;
StackType_t xAutoStateMachineStack[512];

SemaphoreHandle_t I2CMutex;
SemaphoreHandle_t xCanTxMutex;
SemaphoreHandle_t xCan3TxMutex;
SemaphoreHandle_t xSpiMutex;
SemaphoreHandle_t xPidMutex;

// Define a FreeRTOS Queue and Semaphore
QueueHandle_t EulerQueue;
SemaphoreHandle_t BNO085_Int_Sem; // interrupt semaphore

typedef struct {
    float max_vel;      // Maximum allowed speed (e.g., mm/s)
    float accel;        // Acceleration/Deceleration rate (e.g., mm/s^2)
    float target_pos;   // The final destination you want to reach
    float current_pos;  // The VIRTUAL current position
    float current_vel;  // The VIRTUAL current velocity
} MotionProfile_t;

MotionProfile_t StepUpProfiles[4] = {
    {100.0f, 250.0f, 0.0f, 0.0f, 0.0f}, // Motor 0 (e.g., Front Left)
    {100.0f, 250.0f, 0.0f, 0.0f, 0.0f}, // Motor 1 (e.g., Front Right)
    {100.0f, 250.0f, 0.0f, 0.0f, 0.0f}, // Motor 2 (e.g., Back Left)
    {100.0f, 250.0f, 0.0f, 0.0f, 0.0f}  // Motor 3 (e.g., Back Right)
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M2006_MAX_CURRENT			10000
#define M2006_INTEGRAL_LIMIT		3000.00f

#define M3508_MAX_CURRENT			16384	// +/-20A
#define M3508_INTEGRAL_LIMIT		3000.00f

#define OMEGA_DEADBAND				5.00f
#define dt 							0.01

// Heading (Yaw) BNO PID gains
#define kp_bno 						4.51515084328847 	// 3.33283139725851
#define ki_bno 						0.00f
#define kd_bno 						0.990064363482989	// 0.786257732816004
#define n_bno 						15.5535374976703	// 8.44311176008575
#define integral_limit_bno			0.00f
#define max_out_bno					50.00f
#define heading_deadband 			1.0f

// Pitch BNO PID gains
#define kp_pitch 						14.2896996
#define ki_pitch 						0.00f
#define kd_pitch 						4.1908989
#define n_pitch 						100.6181
#define integral_limit_pitch			0.00f
#define max_out_pitch					150.00f
#define pitch_deadband 					2.0f

// XY distance PID gains
#define kp_dist 					0.856582581285727 	// 0.301645673252846 // 1.12252302687043
#define ki_dist 					0.00f 				// 0.112918419077765
#define kd_dist 					0.699328668443535 	// 0.231261693650152 // 1.03859814275107
#define n_dist 						110.359275241713 	// 123.851533699934 // 80.4476756114658
#define integral_limit_dist			0.00f
#define max_out_dist				50.00f
#define dist_deadband 				50.0f

// M3508 RPM-Current gains (For Omni Wheels) -> High Weight
#define kp_omni						4.40692877852658
#define ki_omni						8.2474693225355
#define kd_omni						0.257613005073527
#define n_omni						5.5381278144111
#define b_omni						0.382608739602135
#define c_omni 						0.00243544293355455
#define integral_limit_omni			5000.00f
#define max_out_omni				6000.00f

/* ----- Not used ----- */
//#define kp_rpm_current_stepup				1.94331288
//#define ki_rpm_current_stepup				3.29879580466671
//#define kd_rpm_current_stepup				0.0137789489253414
//#define n_rpm_current_stepup				18.8940821754641
//#define b_rpm_current_stepup				0.798389591797982
//#define c_rpm_current_stepup 				0.0455857605492448
//#define integral_limit_rpm_current_stepup	3500.00f
//#define max_out_rpm_current_stepup			4000.00f

/* ----- Not used -> M3508 Angle-RPM gains (Using M3508 Encoder) ----- */
//#define kp_m3508_ang					0.0542054445276728
//#define ki_m3508_ang					0.00f
//#define kd_m3508_ang					0.00500278315
//#define n_m3508_ang						134.579548107995
//#define integral_limit_m3508_ang		0.00f
//#define max_out_m3508_ang				2000.00f
//#define encoder_deadband 				2048

// M3508 Angle-to-RPM gains (Using AS5047P)
#define kp_m3508_as5047p					5.4336151480128 				// 10.4336151480128
#define ki_m3508_as5047p					0.00f
#define kd_m3508_as5047p					0.731509829 					// 0.331509829
#define n_m3508_as5047p						4.3886332958163
#define integral_limit_m3508_as5047p		0.00f
#define max_out_m3508_as5047p				1000.00f

#define stepup_as5047p_angle_deadband 		5.0f

// M3508 Horizontal Cascade RPM-to-Current PID Gains
#define kp_m3508_HC					1.47860724873464
#define ki_m3508_HC					4.29485597969328
#define kd_m3508_HC					0.0575154736304815
#define n_m3508_HC					60.8922087595727
#define b_m3508_HC					0.797859165851829
#define c_m3508_HC					0.0170224817013682
#define integral_limit_m3508_HC		4000.00f
#define max_out_m3508_HC			5000.00f

// laser address
#define X_BLF_LASER_ADDRESS			3
#define Y_BLF_LASER_ADDRESS			1

// Servo Min and Max angle
#define SERVO_MIN_ANGLE				0
#define SERVO_MAX_ANGLE 			270
#define SERVO_MAX_ANGLE_2			180 // small servo
#define SERVO_MIN_PULSE_WIDTH		500
#define SERVO_MAX_PULSE_WIDTH		2500

// Step up Mechanism macros
#define PINION_PITCH_DIAMETER_MM 				55.50f			// Step Up Gear diameter
#define PI 										3.1415926535f
#define HORIZONTAL_CASCADE_DIAMETER_MM 			(47.36f + 2.00f) 		// horizontal diameter in mm
#define VERTICAL_CASCADE_DIAMETER_MM			45.00f			// vertical diameter in mm

#define MECH1_MAX_DISTANCE_MM 					1500.0f
#define VERTICAL_CASCADE_MAX_DISTANCE_MM 		300.0f
#define MECH2_STALL_CURRENT						7500.00f

#define ROBOT_OFFSET 				(400 + 10)		// MM (Square) -> Robot edges to the center; + 10 -> 1cm safe range
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c5;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim23;
TIM_HandleTypeDef htim24;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart7_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
Robot_CAN_Manager_Struct RobotCan;
Motor_Speed_parameters M3508_SpeedParams;

debounce_button_t L1_button = {
	.state = false,
	.prev_state = false,
	.prevTime = 0
};

debounce_button_t start_btn = {
	.state = false,
	.prev_state = false,
	.prevTime = 0
};

debounce_button_t cross_button = {
	.state = false,
	.prev_state = false,
	.prevTime = 0
};

debounce_button_t circle_button = {
	.state = false,
	.prev_state = false,
	.prevTime = 0
};

debounce_button_t share_button = {
	.state = false,
	.prev_state = false,
	.prevTime = 0
};

debounce_button_t options_button = {
	.state = false,
	.prev_state = false,
	.prevTime = 0
};

//debounce_button_t triangle_button = {
//	.state = false,
//	.prev_state = false,
//	.prevTime = 0
//};

Button_Tracker_t btn_L1 = {0};
Button_Tracker_t btn_R1 = {0};
Button_Tracker_t btn_up = {0};
Button_Tracker_t btn_down = {0};
Button_Tracker_t btn_triangle = {0};
Button_Tracker_t btn_square = {0};

uint8_t M3508_txData_CAN1_RM1[8] = {0};
uint8_t M3508_txData_CAN3_RM1[8] = {0};
uint8_t M3508_txData_CAN3_RM2[8] = {0};

float L2_R2_coeff = 0.4;

float omni_target_speeds[4] = {0};
pid_2dof_t omni_pid_rpm_current[4];

// BNO085 variables and structs
pid_1dof_t heading_pid;
bno085_quat_t current_quat;
bno085_euler_t current_euler;
bno085_euler_t robot_orientation = {0}; // Local struct to safely hold the peeked data

bool enable_success = true;

float target_robot_yaw = 0.0f; // The angle the robot is trying to hold
float target_robot_pitch = 0.0f;
int is_heading_locked = 0;     // Boolean flag for the state machine
int is_pitch_locked = 0;

// Task checking variables
uint32_t can_task = 0;
uint32_t pid_task = 0;
uint32_t ps4_task = 0;
uint32_t swerve_task = 0;
uint32_t pid_stepUp_task = 0;
uint32_t bno_task = 0;
uint32_t as5047pDiag_task = 0;

/*	 AS5047P encoders (1 CS, different SPI buses)	*/
SPI_HandleTypeDef* swerve_spis[4] = {&hspi1, &hspi3, &hspi2, &hspi4};
SPI_HandleTypeDef* mech1_spis[1] = {&hspi2}; // Only 1 SPI on this CS
SPI_HandleTypeDef* steup_spis[4] = {&hspi1, &hspi2, &hspi3, &hspi4};
SPI_HandleTypeDef* mech2_spis[1] = {&hspi4}; // Only 1 SPI on this CS

AS5047P_CS_Group_t as5047p_groups[4] = {
    {GPIOB, CS_SWERVE_Pin,  4, swerve_spis}, 	// Group 1 CS pin 1 -> 4 SPI buses
    {GPIOA, CS_MECH1_Pin, 1, mech1_spis}, 		// Group 2 CS pin 2 -> 1 SPI bus
    {GPIOC, CS_STEP_UP_Pin,  4, steup_spis}, 	// Group 3 CS pin 3 -> 4 SPI buses
    {GPIOA, CS_MECH2_Pin, 1, mech2_spis}  		// Group 4 CS pin 4 -> 1 SPI bus
};

AS5047P_SPI_Errors_t StepUp_spi_errors[4];
AS5047P_SPI_Errors_t Mech1_spi_errors[1];
AS5047P_SPI_Errors_t Mech2_spi_errors[1];

AS5047P_Diagnostics_t as5047pDiag_stepup[4];
AS5047P_Diagnostics_t as5047pDiag_mech1[1];
AS5047P_Diagnostics_t as5047pDiag_mech2[1];

uint8_t spi_stepup_error_count[4] = {0};
uint8_t spi_mech1_error_count[1] = {0};
uint8_t spi_mech2_error_count[1] = {0};

// encoder X and encoder Y variables
volatile uint32_t encoder_x_ticks = 0;
volatile uint32_t encoder_y_ticks = 0;

// blf laser displacement sensor variables
uint8_t y_laser_tx_buffer[8];
uint8_t y_laser_rx_buffer[9]; // The BLF sensor responds with 9 bytes
uint8_t x_laser_tx_buffer[8];
uint8_t x_laser_rx_buffer[9];

volatile bool new_laser_x_available = false;
volatile bool new_laser_y_available = false;
int32_t laser_y_distance = 0;
int32_t laser_x_distance = 0;

int8_t laser_x_status = 0;
int8_t laser_y_status = 0;

uint32_t x_laser_error_count;
uint32_t y_laser_error_count;

// absolute coordinate of robot after sensor fusion
float fused_robot_x = 0.0f;
float fused_robot_y = 0.0f;

bool is_homed = false;
//bool stepup_encoder_reading[4] = {true};
float stepup_continuous_angle[4] = {0};
int32_t stepup_prev_angle[4] = {0};
float stepup_target_linear_distance[4] = {0.0f};
float stepup_target_continuous_angles[4] = {0.0f};

float as5047p_stepup_current_angles[4] = {0};
float stepup_angle_offset[4] = {0};

pid_2dof_t stepup_pid_rpm_current[4];
pid_1dof_t stepup_pid_angle_rpm[4];
pid_1dof_t pitch_pid;

// Servo parameters
uint8_t pwm_channel = 0;

typedef struct {
    float current_angle;  // Where the servo physically is right now
    float target_angle;   // Where you want the servo to go
    float max_step;       // Maximum degrees to move per RTOS tick (Speed limit)
} Servo_Smooth_t;

Servo_Smooth_t Servo1 = {180.0f, 180.0f, 1.0f};
Servo_Smooth_t Servo2 = {100.0f, 100.0f, 1.0f};

float as5047p_vertCas_current_angle = 0.0f;
float as5047p_horiCas_current_angle = 0.0f;
float zero_angle_offset = 0.0f;

Unwrap_Tracker_t stepup_unwrap_trackers[4] = {
    {0.0f, 0.0f, 360.0f, true},
    {0.0f, 0.0f, 360.0f, true},
    {0.0f, 0.0f, 360.0f, true},
    {0.0f, 0.0f, 360.0f, true}
};

typedef enum {
    STEP_UP_INIT = 0,
	STEP_UP_STAGE_1,
    STEP_UP_DONE
} StepUpSequence_t;

StepUpSequence_t current_stepup_state = STEP_UP_INIT;
typedef enum {
    LEG_IN_AIR,         // Only holding the rack's weight
    LEG_BEARING_WEIGHT  // Pushing against the ground/block to lift the 15kg chassis
} LegLoadState;

LegLoadState leg_load_state[4] = {LEG_BEARING_WEIGHT, LEG_BEARING_WEIGHT, LEG_BEARING_WEIGHT, LEG_BEARING_WEIGHT};
// Gravity Feedforward Constant
const float GRAVITY_FF_CURRENT[4] = {4496.0f, -1042.0f, -1186.0f, 0.0f};

/* ------ Vertical Cascade Variables ------ */
pid_2dof_t mech1_pid_rpm_current;
pid_1dof_t mech1_pid_angle_rpm;
float vertCas_continuous_angle = 0.0f;
float vertCas_target_angle = 0.0f;
float vertCas_target_distance = 0.0f;
bool vertCas_homing_triggered = false;
float vertCas_home_offset = 0.0f;
Unwrap_Tracker_t vertCas_unwrap_tracker = {0.0f, 0.0f, 360.00f, true};

/* ------ Horizontal Cascade Variables ------ */
pid_2dof_t mech2_pid_rpm_current;
pid_1dof_t mech2_pid_angle_rpm;
float horiCas_continuous_angle = 0.0f;
float horiCas_target_angle = 0.0f;
float horiCas_target_distance = 0.0f;
bool horiCas_homing_triggered = false;
float horiCas_home_offset = 0.0f;
Unwrap_Tracker_t horiCas_unwrap_tracker = {0.0f, 0.0f, 360.00f, true};
bool recoil_triggered = false;

bool cascade_max = false;
bool cascade_min = false;

/* ------------ Robot Odemetry Variables ------------ */
float robot_coor_x = 0.0f;
float robot_coor_y = 0.0f;
float robot_velocity_x = 0.0f;
float robot_velocity_y = 0.0f;

float target_field_x = 0.00f;
float target_field_y = 0.0f;
float target_heading = 0.0f;

/* ------------- Autonomous Feature Variables ------------- */
pid_1dof_t auto_pid_x;
pid_1dof_t auto_pid_y;

typedef enum {
    AUTO_STATE_INIT = 0,
    AUTO_STATE_DRIVE_TO_TARGET,
	AUTO_STATE_EXECUTE_ACTION,
    AUTO_STATE_DONE
} AutoSequenceState_t;

typedef enum { 			// Define the action to be executed for each waypoint
    ACTION_NONE = 0,
    ACTION_GRAB_SPEAR,
	ACTION_GRAB_KFS_UP,
	ACTION_GRAB_KFS_DOWN,
	ACTION_STEP_UP,
	ACTION_STEP_DOWN,
	ACTION_STEP_DOWN_MEIHUA,
    ACTION_WAIT_2_SEC
} SequenceAction_t;

typedef enum {			// Stages in Step Up or Down
	FULL_EXTEND = 0,
	FULL_RETRACT,
	EXTEND_FRONT_STEP,
	RETRACT_FRONT_STEP,
	EXTEND_BACK_STEP,
	RETRACT_BACK_STEP,
	MOVE_TO_NEXT_BLOCK	// move for some distance before next step-up or actions
} StepUpStage_t;

typedef struct {		// Define the blueprint for a single Waypoint
    float target_x;
    float target_y;
    float target_heading;
    SequenceAction_t action_on_arrival; // What to do when we get there
} Waypoint_t;

// Massice sequence array for easy debugging and editing
const Waypoint_t autonomous_path[] = {
	{(125.0f + 250.0f + ROBOT_OFFSET + 100.0f), 950.0f, 180.0f, ACTION_NONE},
	{(125.0f + ROBOT_OFFSET), 950.0f, 180.0f, ACTION_NONE},
	{2200.0f, (2000.0f + 400.0f), 0.0f, ACTION_WAIT_2_SEC},
	{2800.0f, 2400.0f, 0.0f, ACTION_GRAB_KFS_DOWN},
	{2800.0f, 2400.0f, 0.0f, ACTION_STEP_UP},
	{2800.0f, 2400.0f, 0.0f, ACTION_STEP_UP},
	{2800.0f, 2400.0f, 0.0f, ACTION_STEP_UP},

//	{2800.0f, 2400.0f, 0.0f, ACTION_STEP_DOWN},
//	{2800.0f, 2400.0f, 180.0f, ACTION_STEP_DOWN_MEIHUA},
//    {1000.0f, 2000.0f, 0.0f, ACTION_GRAB_SPEAR},        		// 0: Go to spear, grab it
//    {1000.0f, 5000.0f, 90.0f, ACTION_NONE},             		// 1: Move to the middle of the zone 1
//	{0.0f, 0.0f, 0.0f, ACTION_NONE},							// 2: Move close to the Meihua Forest
//	{0.0f, 0.0f, 0.0f, ACTION_GRAB_KFS_UP},						// 3: Grab KFS (upwards)
//	{0.0f, 0.0f, 0.0f, ACTION_STEP_UP},							// 4: Step up
//	{0.0f, 0.0f, 0.0f, ACTION_NONE},							// 7: Move Forward

    // ... add 50 more steps here if you want!
};

//const Waypoint_t autonomous_path[] = {
//	{(125.0f + 250.0f + ROBOT_OFFSET), 950.0f, 180.0f, ACTION_NONE},
//	{(125.0f + ROBOT_OFFSET), 950.0f, 180.0f, ACTION_NONE},
//	{2800.0f, (2000.0f + 450.0f), 0.0f, ACTION_WAIT_2_SEC},
//	{(2800.0f + 2200.0f), (2000.0f + 450.0f), 0.0f, ACTION_NONE}, // Move to the side in the front of the Meihua Forest
//	{(2800.0f + 2200.0f), (12000.0f - 500.0f), 0.0f, ACTION_NONE}, // Move to the end right corner
//	{600.0f, (12000.0f - 500.0f), 90.0f, ACTION_NONE}, // Move to the tic-tac-toe rack
//	{600.0f, 2600.0f, 0.0f, ACTION_NONE}, // Move back to the start of Meihua Forest
//	{(1000.0f + 400.0f), 410.0f, 0.0f, ACTION_NONE}, // Move to the tic-tac-toe rack
//};

const int TOTAL_WAYPOINTS = sizeof(autonomous_path) / sizeof(autonomous_path[0]);
int current_waypoint_index = 0;
bool auto_state_machine_active = false;	// boolean to trigger the autonomous state machine

AutoSequenceState_t current_auto_state = AUTO_STATE_INIT; // variable to hold the current autonomous state
StepUpStage_t step_up_state = FULL_RETRACT;
StepUpStage_t step_down_state = EXTEND_BACK_STEP;
//uint32_t step_up_timer = 0;
uint32_t auto_timer_start = 0; // timer variable for WAIT_2_SEC

// ------------------------------------ Test Autonomous Variables ---------------------------------------
const Waypoint_t auto_nav[] = {
	{(125.0f + 250.0f + ROBOT_OFFSET), 950.0f, 180.0f, ACTION_NONE},
	{(125.0f + ROBOT_OFFSET), 950.0f, 180.0f, ACTION_NONE},
	{3000.0f, (2000.0f + 600.0f), 0.0f, ACTION_NONE},
	{(3000.0f + 2400.0f), (2000.0f + 600.0f), 0.0f, ACTION_NONE}, // Move to the side in the front of the Meihua Forest
	{(3000.0f + 2400.0f), (12000.0f - 500.0f), 0.0f, ACTION_NONE}, // Move to the end right corner
	{600.0f, (12000.0f - 500.0f), 90.0f, ACTION_NONE}, // Move to the tic-tac-toe rack
	{600.0f, 2600.0f, 0.0f, ACTION_NONE}, // Move back to the start of Meihua Forest
	{(1000.0f + 400.0f), 410.0f, 0.0f, ACTION_NONE}, // Move to the tic-tac-toe rack
};

const int total_AutoNav_waypoints = sizeof(auto_nav) / sizeof(auto_nav[0]);

int current_nav_index = 0;
bool auto_mode_active = false;
// ------------------------------------------------------------------------------------------------------

bool StepUpIR[4] = {false};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_I2C5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_TIM1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM23_Init(void);
static void MX_TIM24_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
enum
{
	AS5047P = 0,
	M3508_ENC
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DistanceToAngle_mapping(float* continuous_angle, float distance, float diameter, uint8_t type)
{
	switch(type)
	{
		case AS5047P:
			*continuous_angle = (distance / (PI * diameter)) * 360.00f;
			break;
		case M3508_ENC:
			*continuous_angle = (distance / (PI * diameter)) * 8192.00f * 19; // Gear Ratio of 19
			break;
	}
}

float LinearVelocity_To_RPM(float linear_vel_mm_s, float diameter_mm, float gear_ratio)
{
    // 1. Find circumference in mm
    float circumference = PI * diameter_mm;

    // 2. Convert mm/s to revolutions per second at the pinion
    float revs_per_sec = linear_vel_mm_s / circumference;

    // 3. Convert to RPM and multiply by the gearbox ratio
    return revs_per_sec * 60.0f * gear_ratio;
}

// Helper function to map values (multiply FIRST, then divide)
int value_mapping(int input, int min1, int max1, int min2, int max2)
{
	return (((input - min1) * (max2 - min2)) / (max1 - min1)) + min2;
}

void Servo_send_angle(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t angle, int min_angle, int max_angle, int min_pulse_width, int max_pulse_width)
{
	// pulse width range (500 to 2500us)
	/*
	 * 0-270 -> 500 - 2500
	 * [(angle - min_angle) / (max_angle - min_angle)] = [(pulse_width - min_pw) / (max_pw - mic_pw)]
	 * */

	// 1. Safety Clamp: Prevent out-of-bounds angles from breaking the servo
	if (angle < min_angle) {
		angle = min_angle;
	} else if (angle > max_angle) {
		angle = max_angle;
	}

	uint16_t pulse_width = (uint16_t)value_mapping((int)angle, min_angle, max_angle, min_pulse_width, max_pulse_width);

	__HAL_TIM_SET_COMPARE(htim, channel, pulse_width);
}

void Smooth_Servo_Update(Servo_Smooth_t *servo)
{
    // If we are already at the target, do nothing
    if (servo->current_angle == servo->target_angle) {
        return;
    }

    float error = servo->target_angle - servo->current_angle;

    // Move positively or negatively by the max_step limit
    if (error > servo->max_step) {
        servo->current_angle += servo->max_step;
    } else if (error < -servo->max_step) {
        servo->current_angle -= servo->max_step;
    } else {
        // If the remaining distance is smaller than a full step, just snap to the target
        servo->current_angle = servo->target_angle;
    }
}

void Servo_command(Servo_Smooth_t *servo, uint8_t pwm_channel)
{
	Smooth_Servo_Update(servo);
	switch (pwm_channel)
	{
		case 1:
			Servo_send_angle(&htim1, TIM_CHANNEL_1, (uint16_t)servo->current_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
//			HAL_GPIO_WritePin(PNEU_1_GPIO_Port, PNEU_1_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			Servo_send_angle(&htim1, TIM_CHANNEL_2, (uint16_t)servo->current_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
//			HAL_GPIO_WritePin(PNEU_2_GPIO_Port, PNEU_2_Pin, GPIO_PIN_RESET);
			break;
		case 3:
			Servo_send_angle(&htim1, TIM_CHANNEL_3, (uint16_t)servo->current_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE_2, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
//			HAL_GPIO_WritePin(PNEU_3_GPIO_Port, PNEU_3_Pin, GPIO_PIN_RESET);
			break;
		case 4:
			Servo_send_angle(&htim1, TIM_CHANNEL_4, (uint16_t)servo->current_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE_2, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
			break;
		default:
	}
}

void heading_state_machine(float *w) // state machine for heading locking
{
	if (fabsf(*w) > OMEGA_DEADBAND)
	{
	    // STATE 1: Human is actively commanding rotation
	    is_heading_locked = 0;          // Disable the PID lock

	    // Constantly update the target to the current heading so it's ready for when they let go
	    target_robot_yaw = robot_orientation.yaw;

	    // Optional: Keep resetting the PID history so it doesn't wind up while manually spinning
	    heading_pid.err[NOW] = 0;
	    heading_pid.err[LAST] = 0;
	    heading_pid.out = 0;
	    heading_pid.iout = 0;
	    heading_pid.prev_derivative = 0;
	}
	else
	{
	    // STATE 2: let go of the rotation joystick. Lock the heading!
	    if (is_heading_locked == 0) {
	        target_robot_yaw = robot_orientation.yaw; // Capture the final heading
	        is_heading_locked = 1;              // Engage the lock
	    }

	    // A. Calculate the raw heading error
	    float yaw_error = robot_orientation.yaw - target_robot_yaw;

	    // B. Wrap the error to [-180, 180] (Just like we did for the steering modules!)
	    // This ensures if the robot gets spun across the 180 boundary, it unwinds the shortest way.
	    while (yaw_error > 180.0f)  yaw_error -= 360.0f;
	    while (yaw_error < -180.0f) yaw_error += 360.0f;

	    // C. Calculate the PID correction using the Zero-Setpoint Trick
	    PID_1DOF_calc(&heading_pid, 0.0f, yaw_error, dt);

	    // D. The output of the PID becomes our automated rotational command
	    *w = heading_pid.out;
	}
}

void robotCentricToFieldCentric(float* pLX, float* pLY, bool field_centric)
{
	if(field_centric)
	{
		// Convert Degrees to Radians
		float yaw_rad = robot_orientation.yaw * (M_PI / 180.0f);

		float local_x = (*pLX * cosf(yaw_rad)) + (*pLY * sinf(yaw_rad));
		float local_y = -(*pLX * sinf(yaw_rad)) + (*pLY * cosf(yaw_rad));

		*pLX = local_x;
		*pLY = local_y;
	}
}

void Update_Trajectory(MotionProfile_t *prof, float dt_s)
{
    float error = prof->target_pos - prof->current_pos;

    // If we are practically at the target, force to 0 and exit to prevent micro-jitter
    if (fabsf(error) < 0.1f && fabsf(prof->current_vel) < 0.1f) {
        prof->current_pos = prof->target_pos;
        prof->current_vel = 0.0f;
        return;
    }

    // Calculate kinematic stopping distance at current velocity
    float stopping_dist = (prof->current_vel * prof->current_vel) / (2.0f * prof->accel);

    if (fabsf(error) <= stopping_dist) {
        // --- DECELERATE ---
        float decel_step = prof->accel * dt_s;
        if (prof->current_vel > 0) {
            prof->current_vel -= decel_step;
            if(prof->current_vel < 0) prof->current_vel = 0.0f;
        } else {
            prof->current_vel += decel_step;
            if(prof->current_vel > 0) prof->current_vel = 0.0f;
        }
    } else {
        // --- ACCELERATE / CRUISE ---
        float accel_step = prof->accel * dt_s;
        if (error > 0) {
            prof->current_vel += accel_step;
            if(prof->current_vel > prof->max_vel) prof->current_vel = prof->max_vel;
        } else {
            prof->current_vel -= accel_step;
            if(prof->current_vel < -prof->max_vel) prof->current_vel = -prof->max_vel;
        }
    }

    // Integrate velocity to get the new virtual position
    prof->current_pos += prof->current_vel * dt_s;
}

/* -------------------------------- FreeRTOS Task -------------------------------- */
void PS4task(void* pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		if(xSemaphoreTake(I2CMutex, pdMS_TO_TICKS(100)) == pdTRUE) // mutex only use for shared resource condition
		{
			PS4Read();
			DebounceButton(&start_btn.state, toggle_button[start_button], &start_btn.prev_state, &start_btn.prevTime);
			DebounceButton(&cross_button.state, toggle_button[cross], &cross_button.prev_state, &cross_button.prevTime);
			DebounceButton(&options_button.state, toggle_button[options], &options_button.prev_state, &options_button.prevTime);
			DebounceButton(&share_button.state, toggle_button[share], &share_button.prev_state, &share_button.prevTime);
//			DebounceButton(&auto_mode_active, toggle_button[circle], &circle_button.prev_state, &circle_button.prevTime);
			DebounceButton(&auto_state_machine_active, toggle_button[circle], &circle_button.prev_state, &circle_button.prevTime);

			Button_Update(&btn_L1, toggle_button[L1]);
			Button_Update(&btn_R1, toggle_button[R1]);
			Button_Update(&btn_up, toggle_button[dpad_up]);
			Button_Update(&btn_down, toggle_button[dpad_down]);

			if (btn_L1.pressed_edge || btn_R1.pressed_edge)
			{
			    // 1. ABSOLUTE GRID SNAPPING
			    // Divide by 90, round to nearest whole integer, and multiply by 90.
			    // Example A: target is 2.5 -> (2.5/90) = 0.02 -> round(0) * 90 = 0.0f
			    // Example B: target is 94.1 -> (94.1/90) = 1.04 -> round(1) * 90 = 90.0f
			    float current_snapped_yaw = roundf(target_robot_yaw / 90.0f) * 90.0f;

			    // 2. Apply the 90-degree step to the perfectly clean absolute angle
			    if (btn_L1.pressed_edge)
			    {
			        target_robot_yaw = current_snapped_yaw - 90.0f;
//			        stepup_target_linear_distance[0] = 50.0f;
//			        stepup_target_linear_distance[1] = 50.0f;
//			        stepup_target_linear_distance[2] = 50.0f;
//			        stepup_target_linear_distance[3] = 50.0f;
			    }
			    else if (btn_R1.pressed_edge)
			    {
			        target_robot_yaw = current_snapped_yaw + 90.0f;
//			        stepup_target_linear_distance[0] = 150.0f;
//			        stepup_target_linear_distance[1] = 150.0f;
//			        stepup_target_linear_distance[2] = 150.0f;
//					stepup_target_linear_distance[3] = 150.0f;
			    }

			    is_heading_locked = 1;     // Force the lock state on just in case
			    heading_pid.iout = 0.0f;   // Reset the PID integral so it snaps aggressively
			}

			// Keep the target angle strictly within [-180, 180] so the math stays clean
			while (target_robot_yaw > 180.0f)  target_robot_yaw -= 360.0f;
			while (target_robot_yaw < -180.0f) target_robot_yaw += 360.0f;

			if(btn_up.pressed_edge && (current_stepup_state ==  STEP_UP_INIT))
			{
				for(int i = 0; i < 4; i++) {
					StepUpProfiles[i].target_pos = 205.0f;
					leg_load_state[i] = LEG_BEARING_WEIGHT; // All 4 hit the ground
				}
			}
			else if(btn_down.pressed_edge && (current_stepup_state ==  STEP_UP_INIT))
			{
				for(int i = 0; i < 4; i++) {
					StepUpProfiles[i].target_pos = 50.0f; // 0: Fl, 1: FR, 2: BL, 3: BR
					leg_load_state[i] = LEG_BEARING_WEIGHT;
				}
			}

			ps4_task++;
			xSemaphoreGive(I2CMutex);
		}

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
	}
}

void CanSendTask(void* pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		if(xSemaphoreTake(xCanTxMutex, pdMS_TO_TICKS(100)) == pdTRUE) // mutex only use for shared resource condition
		{
			// Robomaster M2006
			RobotCan.Bus_Health[0] = FDCAN_Transmit(&hfdcan1, RM1, M3508_txData_CAN1_RM1, 8, FDCAN_STANDARD_ID);

			xSemaphoreGive(xCanTxMutex);
		}

		if(xSemaphoreTake(xCan3TxMutex, pdMS_TO_TICKS(100)) == pdTRUE) // mutex only use for shared resource condition
		{
			RobotCan.Bus_Health[2] = FDCAN_Transmit(&hfdcan3, RM1, M3508_txData_CAN3_RM1, 8, FDCAN_STANDARD_ID);

			if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) > 0) { // safely check the FIFO level
			    RobotCan.Bus_Health[2] = FDCAN_Transmit(&hfdcan3, RM2, M3508_txData_CAN3_RM2, 8, FDCAN_STANDARD_ID);
			}

			xSemaphoreGive(xCan3TxMutex);
		}

		can_task++;
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
	}
}

void NavPidBldcTask(void* pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		if (EulerQueue != NULL) {
			// safely read the latest bno085 angle
			xQueuePeek(EulerQueue, &robot_orientation, 0); // use xQueuePeek with a 0 block time so it never stalls the 5ms loop
		}

		if(start_btn.state)
		{
			if(xSemaphoreTake(xPidMutex, pdMS_TO_TICKS(5)) == pdTRUE)
			{
				if(!auto_state_machine_active)
				{
					float omega = ((float)R2 - (float)L2)*L2_R2_coeff;
					heading_state_machine(&omega);

					float float_LX = (float)LX;
					float float_LY = (float)LY;
					float omega_scaled = omega*(1.0);
					robotCentricToFieldCentric(&float_LX, &float_LY, share_button.state);
					omni_algorithm(float_LX, float_LY, omega_scaled, omni_target_speeds, options_button.state);
				}

				for (int i = 0; i < NUM_RM_CAN1; i++)
				{
					PID_2DOF_calc(&omni_pid_rpm_current[i], RobotCan.RM_State[i].rpm, omni_target_speeds[i], dt);
				}

				xSemaphoreGive(xPidMutex);
			}
		}
		else
		{
			for(int i = 0; i < NUM_RM_CAN1; i++)
			{
				omni_pid_rpm_current[i].out = 0;
				omni_pid_rpm_current[i].iout = 0; // Optional: Reset PID integral to prevent "windup" jump when re-enabling
			}
		}

		// 2. Map Output (Update shared array)
		if(xSemaphoreTake(xCanTxMutex, pdMS_TO_TICKS(5)) == pdTRUE) // mutex only use for shared resource condition
		{
			RM_mapping_pid_out(omni_pid_rpm_current, M3508_txData_CAN1_RM1, RobotCan.Bus_Health[0]);
			xSemaphoreGive(xCanTxMutex);
		}

		pid_task++;
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
	}
}

void StepUpTask(void* pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(10);
	float stepup_home_offset[4] = {0.0f};

	while(1)
	{
		Button_Update(&btn_triangle, toggle_button[triangle]);
		Button_Update(&btn_square, toggle_button[square]);

		if(xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS(5)) == pdTRUE)
		{
			AS5047P_Group_Read(&as5047p_groups[0], as5047p_stepup_current_angles, as5047pDiag_stepup, spi_stepup_error_count, stepup_angle_offset);
			xSemaphoreGive(xSpiMutex);
		}

		for(int i = 0; i < 4; i++) {
//			Read_And_Unwrap_Encoder(RobotCan.RM_State[i+4].angle, &stepup_encoder_reading[i], &stepup_continuous_angle[i], &stepup_prev_angle[i]);
			stepup_continuous_angle[i] = Read_And_Unwrap(as5047p_stepup_current_angles[i], &stepup_unwrap_trackers[i]);
		}

		if(cross_button.state)
		{
			// --- HOMING STATE ---
			if (!is_homed)
			{
				for(int i = 0; i < 4; i++) {
					stepup_home_offset[i] = stepup_continuous_angle[i]; // Update offset continuously until homed
					StepUpProfiles[i].current_pos = 0.0f;
					StepUpProfiles[i].target_pos = 15.0f; // lift up a bit
					StepUpProfiles[i].current_vel = 0.0f;
					stepup_pid_rpm_current[i].iout = 0.0f;
				}

				target_robot_pitch = robot_orientation.pitch;
				pitch_pid.iout = 0.0f; // Reset integral windup

				is_homed = true;
			}
			else
			{
				if(btn_triangle.pressed_edge)
				{
					switch (current_stepup_state)
					{
						case STEP_UP_INIT:
							StepUpProfiles[0].target_pos = -20.0f;
							StepUpProfiles[1].target_pos = -20.0f;
							leg_load_state[0] = LEG_IN_AIR;
							leg_load_state[1] = LEG_IN_AIR;
							current_stepup_state = STEP_UP_STAGE_1;
							break;

						case STEP_UP_STAGE_1:
							StepUpProfiles[2].target_pos = -20.0f;
							StepUpProfiles[3].target_pos = -20.0f;
							leg_load_state[2] = LEG_IN_AIR;
							leg_load_state[3] = LEG_IN_AIR;
							current_stepup_state = STEP_UP_DONE;
							break;

						case STEP_UP_DONE:
							current_stepup_state = STEP_UP_INIT;
							break;
					}
				}

				if(btn_square.pressed_edge)
				{
					switch (current_stepup_state)
					{
						case STEP_UP_INIT:
							StepUpProfiles[2].target_pos = 205.0f;
							StepUpProfiles[3].target_pos = 205.0f;
							leg_load_state[2] = LEG_IN_AIR;
							leg_load_state[3] = LEG_IN_AIR;
							current_stepup_state = STEP_UP_STAGE_1;
							break;

						case STEP_UP_STAGE_1:
							StepUpProfiles[0].target_pos = 205.0f;
							StepUpProfiles[1].target_pos = 205.0f;
							leg_load_state[0] = LEG_IN_AIR;
							leg_load_state[1] = LEG_IN_AIR;
							current_stepup_state = STEP_UP_DONE;
							break;

						case STEP_UP_DONE:
							current_stepup_state = STEP_UP_INIT;
							break;
					}
				}

				PID_1DOF_calc(&pitch_pid, robot_orientation.pitch, target_robot_pitch, dt);
				float pitch_effort = pitch_pid.out;
				// --- NORMAL OPERATION STATE ---
				for(int i = 0; i < 4; i++)
				{
					Update_Trajectory(&StepUpProfiles[i], dt);
					float relative_stepup_current_angle = (float)(stepup_continuous_angle[i] - stepup_home_offset[i]);

//					DistanceToAngle_mapping(&stepup_target_continuous_angles[i], stepup_target_linear_distance[i], PINION_PITCH_DIAMETER_MM, AS5047P);
//					PID_1DOF_calc(&stepup_pid_angle_rpm[i], relative_stepup_current_angle, stepup_target_continuous_angles[i], dt);

					float target_angle;
					DistanceToAngle_mapping(&target_angle, StepUpProfiles[i].current_pos, PINION_PITCH_DIAMETER_MM, AS5047P);

					// Calculate velocity feedforward
					float vff_rpm = LinearVelocity_To_RPM(StepUpProfiles[i].current_vel, PINION_PITCH_DIAMETER_MM, 19.0f);

					float local_vff_rpm = vff_rpm;
					if (i == 0 || i == 1) {
						local_vff_rpm += pitch_effort;
					} else if (i == 2 || i == 3) {
						local_vff_rpm -= pitch_effort;
					}

					// --- MECHANICAL INVERSION ---
					if (i == 1 || i == 2) {
						target_angle = -target_angle;
						local_vff_rpm = -local_vff_rpm; // INVERT THE VELOCITY TOO!
					}

					PID_1DOF_calc(&stepup_pid_angle_rpm[i], relative_stepup_current_angle, target_angle, dt);

					// Add the PID output to the VFF
					float final_target_rpm = stepup_pid_angle_rpm[i].out + local_vff_rpm;
					PID_2DOF_calc(&stepup_pid_rpm_current[i], RobotCan.RM_State[i+4].rpm, final_target_rpm, dt);

					float local_gff = 0.0f;

					if(leg_load_state[i] == LEG_BEARING_WEIGHT)
						local_gff = GRAVITY_FF_CURRENT[i];
					else if(leg_load_state[i] == LEG_IN_AIR)
						local_gff = 0.0f;

					stepup_pid_rpm_current[i].out += local_gff;

					if(stepup_pid_rpm_current[i].out > M3508_MAX_CURRENT) stepup_pid_rpm_current[i].out = M3508_MAX_CURRENT;
					if(stepup_pid_rpm_current[i].out < -M3508_MAX_CURRENT) stepup_pid_rpm_current[i].out = -M3508_MAX_CURRENT;
				}
			}
		}
		else
		{
			is_homed = false;
			for(int i = 0; i < 4; i++)
			{
				stepup_pid_angle_rpm[i].out = 0;
				stepup_pid_angle_rpm[i].iout = 0;
				stepup_pid_rpm_current[i].out = 0;
				stepup_pid_rpm_current[i].iout = 0;
			}
		}

		// 2. Map Output (Update shared array)
		if(xSemaphoreTake(xCan3TxMutex, pdMS_TO_TICKS(5)) == pdTRUE) // mutex only use for shared resource condition
		{
			RM_mapping_pid_out(stepup_pid_rpm_current, M3508_txData_CAN3_RM1, RobotCan.Bus_Health[2]);
			xSemaphoreGive(xCan3TxMutex);
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

void CascadeMechTask(void* pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	bool is_first_loop = true;
	uint8_t min_switch_release_counter = 0;
	while(1)
	{
    	AS5047P_Group_Read(&as5047p_groups[1], &as5047p_horiCas_current_angle, as5047pDiag_mech1, spi_mech1_error_count, &zero_angle_offset);
    	AS5047P_Group_Read(&as5047p_groups[3], &as5047p_vertCas_current_angle, as5047pDiag_mech2, spi_mech2_error_count, &zero_angle_offset);

    	vertCas_continuous_angle = Read_And_Unwrap(as5047p_vertCas_current_angle, &vertCas_unwrap_tracker);
    	horiCas_continuous_angle = Read_And_Unwrap(as5047p_horiCas_current_angle, &horiCas_unwrap_tracker);

    	if (is_first_loop) {
			vertCas_home_offset = as5047p_vertCas_current_angle;
			is_first_loop = false;
		}

		float relative_current_angle = horiCas_continuous_angle - horiCas_home_offset; // Create the working angle based on the home zero-point
		float relative_current_angle_vert = vertCas_continuous_angle - vertCas_home_offset;

		/* ------------------------- Vertical Cascade ------------------------- */
		// Prevent distance windup past limits
		if (vertCas_target_distance < 0.0f) vertCas_target_distance = 0.0f;
		if (vertCas_target_distance > VERTICAL_CASCADE_MAX_DISTANCE_MM) vertCas_target_distance = VERTICAL_CASCADE_MAX_DISTANCE_MM;

		DistanceToAngle_mapping(&vertCas_target_angle, vertCas_target_distance, VERTICAL_CASCADE_DIAMETER_MM, AS5047P);
		PID_1DOF_calc(&mech1_pid_angle_rpm, -1*relative_current_angle_vert, vertCas_target_angle, dt);
		PID_2DOF_calc(&mech1_pid_rpm_current, RobotCan.RM_State[8].rpm, mech1_pid_angle_rpm.out, dt);

		float local_gff = 1500.00f;
		int16_t motor_current_vert = (int16_t)(mech1_pid_rpm_current.out + local_gff);

		/* ------------------------- Horizontal Cascade ------------------------- */
		if(toggle_button[dpad_left])
			horiCas_target_distance -= 1.0f;
		else if(toggle_button[dpad_right])
			horiCas_target_distance += 1.0f;

		cascade_max = (HAL_GPIO_ReadPin(CASCADE_MAX_GPIO_Port, CASCADE_MAX_Pin) == GPIO_PIN_RESET);
		cascade_min = (HAL_GPIO_ReadPin(CASCADE_MIN_GPIO_Port, CASCADE_MIN_Pin) == GPIO_PIN_RESET);

		if(!horiCas_homing_triggered)
			horiCas_target_angle += 0.5f;
		else if(recoil_triggered)
		{
			horiCas_target_angle -= 0.1f;

			if(!cascade_min)
			{
				min_switch_release_counter++;

				if(min_switch_release_counter > 150)
				{
					horiCas_home_offset = horiCas_continuous_angle;
					horiCas_target_angle = 0.0f; // Target is now locked exactly at the switch
					horiCas_target_distance = 0.0f;
					mech2_pid_angle_rpm.iout = 0.0f; // Reset PID integrators to prevent sudden jerks
					mech2_pid_rpm_current.iout = 0.0f;
					recoil_triggered = false;
					min_switch_release_counter = 0; // Reset for the next homing cycle
				}
			}
			else
				min_switch_release_counter = 0;
		}
		else
			DistanceToAngle_mapping(&horiCas_target_angle, horiCas_target_distance, HORIZONTAL_CASCADE_DIAMETER_MM, AS5047P);

		PID_1DOF_calc(&mech2_pid_angle_rpm, relative_current_angle, horiCas_target_angle, dt);
		PID_2DOF_calc(&mech2_pid_rpm_current, RobotCan.RM_State[9].rpm, mech2_pid_angle_rpm.out, dt);

		int16_t motor_current = (int16_t)(mech2_pid_rpm_current.out);

		// 3. Directional Escape Logic
		if (cascade_max && motor_current < 0) {
			motor_current = 0; // Prevent driving further into the MAX limit
		}

		if (cascade_min && motor_current > 0) {
			motor_current = 0; // Prevent driving further into the MIN limit

			if(!horiCas_homing_triggered)
			{
				horiCas_homing_triggered = true;
				recoil_triggered = true;

//				// CRITICAL FIX: Capture the zero-point offset!
//				horiCas_home_offset = horiCas_continuous_angle;
				horiCas_target_angle = 0.0f; // Target is now locked exactly at the switch
//				horiCas_target_distance = 0.0f;

				mech2_pid_angle_rpm.iout = 0.0f; // Kill PID Integral Windup!
				mech2_pid_rpm_current.iout = 0.0f;
			}
		}

		if(xSemaphoreTake(xCan3TxMutex, pdMS_TO_TICKS(5)) == pdTRUE) // We do NOT transmit here. We only safely pack the array for CanSendTask.
		{
			if (RobotCan.Bus_Health[2] == 0) // HAL_OK = 0 (Bus is healthy)
			{
				// Pack fifth M3508 (0x205) with the safety-checked current
				M3508_txData_CAN3_RM2[0] = (uint8_t)(motor_current_vert >> 8);
				M3508_txData_CAN3_RM2[1] = (uint8_t)(motor_current_vert & 0xFF);

				// 0x206
				M3508_txData_CAN3_RM2[2] = (uint8_t)(motor_current >> 8);
				M3508_txData_CAN3_RM2[3] = (uint8_t)(motor_current & 0xFF);
			}
			else
			{
				// If CAN is unhealthy, send 0 current
				memset(M3508_txData_CAN3_RM2, 0, 8);
			}
			xSemaphoreGive(xCan3TxMutex);
		}

		/*
		 * Channel 1 -> lower arm (90-)
		 * Channel 4 -> Upper arm (60) - 30 degree to pickup
		 * */
		Servo_command(&Servo1, 1);
		Servo_command(&Servo2, 4);

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
	}
}

void BNO085Task(void *argument) {
    uint8_t shtp_buffer[128];
    uint16_t packet_len = 0;

    // Give the sensor time to boot up
    vTaskDelay(pdMS_TO_TICKS(100));

    // --- HARDWARE RESET SEQUENCE ---
	// Hold RST pin LOW to kill power to the IMU logic
	HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_RESET);
	vTaskDelay(pdMS_TO_TICKS(10)); // Hold in reset

	// Pull RST pin HIGH to boot the IMU in I2C mode!
	HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_SET);

	// CRITICAL FIX 1: Give the BNO085 processor time to boot up!
	vTaskDelay(pdMS_TO_TICKS(200));

	// --- THE QUEUE DRAINER ---
	// CRITICAL FIX 2: Protect the while loop with a timeout (drain_attempts)
	uint8_t drain_attempts = 0;
	while ((HAL_GPIO_ReadPin(BNO_HINT_GPIO_Port, BNO_HINT_Pin) == GPIO_PIN_RESET) && (drain_attempts < 10)) {
		// Read whatever is sitting in the buffer
		BNO085_ReadPacket(&hi2c2, shtp_buffer, sizeof(shtp_buffer), &packet_len);

		// Give the BNO085 a few milliseconds to process and release the hardware line
		vTaskDelay(pdMS_TO_TICKS(5));
		drain_attempts++;
	}

	// 3. Clear any phantom interrupt flags on the STM32 side
	__HAL_GPIO_EXTI_CLEAR_IT(BNO_HINT_Pin);
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn); // Flush the NVIC pipeline

	// CRITICAL: Manually set the RTOS-safe priority (5) BEFORE enabling!
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	// 4. NOW it is safe to turn on the interrupt!
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    // Enable the Game Rotation Vector at 100Hz (10,000 microseconds)
	enable_success = BNO085_EnableGameRotationVector(&hi2c2, 10000);

	if (enable_success == false) {
		// If we land here, the I2C transmission failed!
		// Check the Address (0x94), wiring, or do a Hard Power Cycle.
		while(1) { osDelay(1); }
	}

    while (1) {
        // Wait indefinitely for the INT pin to drop low
        if (xSemaphoreTake(BNO085_Int_Sem, portMAX_DELAY) == pdTRUE) {

            // Read the data packet over I2C
            if (BNO085_ReadPacket(&hi2c2, shtp_buffer, sizeof(shtp_buffer), &packet_len)) {

                // If it's the correct rotation vector report, parse it
                if (BNO085_ParseRotationVector(shtp_buffer, &current_quat)) {

                    // Convert Quaternions to Euler Angles
                    BNO085_QuaternionToEuler(&current_quat, &current_euler);

                    if (EulerQueue != NULL) {
                        // Send to queue (Overwrite so algorithms always get the freshest data)
                        xQueueOverwrite(EulerQueue, &current_euler);
					}

                }
            }
        }

        bno_task++;
    }
}

void AS5047P_DiagTask(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Run 1 time a second (1000ms)

    while(1)
    {
        // We need to protect SPI access because PID task uses it too!
        if(xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
			// Read all 4 Swerve Encoders simultaneously
			AS5047P_Group_GetDiagnostics(&as5047p_groups[0], as5047pDiag_stepup); // SPI Step Up mechanism uses the CS pin that the swerve used before
			AS5047P_Group_GetDiagnostics(&as5047p_groups[1], as5047pDiag_mech1);
			AS5047P_Group_GetDiagnostics(&as5047p_groups[3], as5047pDiag_mech2);

            xSemaphoreGive(xSpiMutex);
        }

        // Evaluate the results outside the Mutex to keep the SPI bus fast!
        for(int i = 0; i < 4; i++)
		{
        	if(as5047pDiag_stepup[i].MAGL ||
			   as5047pDiag_stepup[i].MAGH ||
			   as5047pDiag_stepup[i].COF  ||
			   as5047pDiag_stepup[i].ErrorFlag)
			{
				as5047pDiag_stepup[i].OK = false;
				if(as5047pDiag_stepup[i].ErrorFlag) AS5047P_Group_ReadAndClearErrors(&as5047p_groups[0], StepUp_spi_errors);
			}
			else
			{
				as5047pDiag_stepup[i].OK = true; // Allow recovery from transient spikes
			}
		}

        if(as5047pDiag_mech1[0].MAGL ||
		   as5047pDiag_mech1[0].MAGH ||
		   as5047pDiag_mech1[0].COF  ||
		   as5047pDiag_mech1[0].ErrorFlag)
		{
			as5047pDiag_mech1[0].OK = false;
			if(as5047pDiag_mech1[0].ErrorFlag) AS5047P_Group_ReadAndClearErrors(&as5047p_groups[1], Mech1_spi_errors);
		}
		else
		{
			as5047pDiag_mech1[0].OK = true; // Allow recovery from transient spikes
		}

        if(as5047pDiag_mech2[0].MAGL ||
		   as5047pDiag_mech2[0].MAGH ||
		   as5047pDiag_mech2[0].COF  ||
		   as5047pDiag_mech2[0].ErrorFlag)
		{
			as5047pDiag_mech2[0].OK = false;
			if(as5047pDiag_mech2[0].ErrorFlag) AS5047P_Group_ReadAndClearErrors(&as5047p_groups[3], Mech2_spi_errors);
		}
		else
		{
			as5047pDiag_mech2[0].OK = true; // Allow recovery from transient spikes
		}

        // Only request X if the UART is idle
		if (huart5.gState == HAL_UART_STATE_READY && huart5.RxState == HAL_UART_STATE_READY) {
			Request_Sensor_Distance_DMA(X_BLF_LASER_ADDRESS, x_laser_tx_buffer, x_laser_rx_buffer, &huart5);
		}

		// Only request Y if the UART is idle
		if (huart7.gState == HAL_UART_STATE_READY && huart7.RxState == HAL_UART_STATE_READY) {
			Request_Sensor_Distance_DMA(Y_BLF_LASER_ADDRESS, y_laser_tx_buffer, y_laser_rx_buffer, &huart7);
		}

		as5047pDiag_task++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void AutoStateMachineTask(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool is_sensor_fusion_seeded = false;

    while(1)
    {
    	/* XY Encoder update */
    	encoder_x_ticks = TIM23->CNT;
    	encoder_y_ticks = TIM24->CNT;

    	StepUpIR[0] = (HAL_GPIO_ReadPin(GPIOD, IR_3_Pin) == GPIO_PIN_RESET); // FL
    	StepUpIR[1] = (HAL_GPIO_ReadPin(GPIOF, IR_4_Pin) == GPIO_PIN_RESET); // FR
    	StepUpIR[2] = (HAL_GPIO_ReadPin(GPIOF, IR_1_Pin) == GPIO_PIN_RESET); // BL
    	StepUpIR[3] = (HAL_GPIO_ReadPin(GPIOD, IR_2_Pin) == GPIO_PIN_RESET); // BR
    	if (!is_sensor_fusion_seeded)
		{
			if (new_laser_x_available && new_laser_y_available)
			{
				// Seed the filter now that the data is real!
				Sensor_Fusion_Init(&robot_coor_x, &robot_coor_y);
				is_sensor_fusion_seeded = true;
			}
		}
		else
		{
			// Only run the continuous update AFTER it has been seeded
			Sensor_Fusion_Update(&robot_coor_x, &robot_coor_y, robot_orientation.yaw);
		}

        if (auto_state_machine_active && start_btn.state)
        {
        	// Calculate absolute errors for arrival checking
			float abs_err_x = fabsf(target_field_x - robot_coor_x);
			float abs_err_y = fabsf(target_field_y - robot_coor_y);

			float yaw_err = robot_orientation.yaw - target_heading;
			while(yaw_err > 180.0f)  yaw_err -= 360.0f;
			while(yaw_err < -180.0f) yaw_err += 360.0f;
			float abs_err_yaw = fabsf(yaw_err);

			// 1. THE BRAIN: State Machine Sequencer
			switch (current_auto_state)
			{
				case AUTO_STATE_INIT:
					current_waypoint_index = 0;
					if(autonomous_path[current_waypoint_index].action_on_arrival == ACTION_STEP_UP ||
					   autonomous_path[current_waypoint_index].action_on_arrival == ACTION_STEP_DOWN ||
					   autonomous_path[current_waypoint_index].action_on_arrival == ACTION_STEP_DOWN_MEIHUA)
					{
						target_field_x = robot_coor_x;
						target_field_y = robot_coor_y;
					}
					else
					{
						target_field_x = autonomous_path[current_waypoint_index].target_x;
						target_field_y = autonomous_path[current_waypoint_index].target_y;
					}
					target_heading = autonomous_path[current_waypoint_index].target_heading;

					if(target_field_x < ROBOT_OFFSET) target_field_x = ROBOT_OFFSET;
					if(target_field_y < ROBOT_OFFSET) target_field_y = ROBOT_OFFSET;

					PID_1DOF_Reset(&auto_pid_x);
					PID_1DOF_Reset(&auto_pid_y);
					PID_1DOF_Reset(&heading_pid);
					current_auto_state = AUTO_STATE_DRIVE_TO_TARGET;
					break;

				case AUTO_STATE_DRIVE_TO_TARGET:
					// Check if we arrived in X, Y, AND Yaw simultaneously!
					if ((abs_err_x < dist_deadband) &&
						(abs_err_y < dist_deadband) &&
						(abs_err_yaw < heading_deadband))
					{
						current_auto_state = AUTO_STATE_EXECUTE_ACTION;

						if(autonomous_path[current_waypoint_index].action_on_arrival == ACTION_STEP_UP)
							step_up_state = FULL_EXTEND;
						else if(autonomous_path[current_waypoint_index].action_on_arrival == ACTION_STEP_DOWN ||
								autonomous_path[current_waypoint_index].action_on_arrival == ACTION_STEP_DOWN_MEIHUA)
							step_down_state = EXTEND_BACK_STEP;

						auto_timer_start = HAL_GetTick(); // Start timer in case the action needs it
					}
					break;

				case AUTO_STATE_EXECUTE_ACTION:
					// Look at the array to see what mechanism to trigger
					bool action_is_finished = false;
					switch(autonomous_path[current_waypoint_index].action_on_arrival)
					{
						case ACTION_NONE:
							// Do nothing, just move on
							action_is_finished = true;
							break;

						case ACTION_GRAB_SPEAR:
							// triggers servo motor to grab the spear head
							action_is_finished = true;
							break;

						case ACTION_GRAB_KFS_UP:
							// 1. set the vertical and horizontal cascade mechanism distance
							// 2. set the gripper servo angle
							// 3. activate the pneumatic
							action_is_finished = true;
							break;

						case ACTION_GRAB_KFS_DOWN:
							// 1. set the vertical and horizontal cascade mechanism distance
							// 2. set the gripper servo angle
							// 3. activate the pneumatic
							static int grab_stage = 0;
							static bool target_set = false;

							switch(grab_stage)
							{
								case 0: // STAGE 0: Lift Vertical and Grip
									vertCas_target_distance = 200.00f; // 20cm lift

									// Fix: Check the specific float inside the struct array!
									if(fabsf(mech1_pid_angle_rpm.err[NOW]) < 10.0f)
									{
										Servo1.target_angle = 90.0f; // Grip
										grab_stage = 1; // Move to next stage
									}
									break;

								case 1: // STAGE 1: Extend Horizontal and Drive Forward
									// Fix: Ensure we only add the 100mm target EXACTLY ONCE
									if (!target_set) {
										target_field_y += 150.0f; // Drive forward 20cm
										target_set = true;
									}

									horiCas_target_distance = -200.0f; // Extend horizontal

									// Fix: Check the specific float inside the struct array!
									if(fabsf(mech2_pid_angle_rpm.err[NOW]) < 10.0f &&
										fabsf(target_field_y - robot_coor_y) < dist_deadband)
									{
										HAL_GPIO_WritePin(PNEU_BOT_GPIO_Port, PNEU_BOT_Pin, SET);
										// Action complete! Reset our static variables for the next time we grab
										grab_stage = 2;
										target_set = false;
									}
									break;

								case 2:
									Servo1.target_angle = 130.0f;
									horiCas_target_distance = 0.0f;
									vertCas_target_distance = 0.0f;

									if(fabsf(mech1_pid_angle_rpm.err[NOW]) < 10.0f && fabsf(mech2_pid_angle_rpm.err[NOW]) < 10.0f)
									{
										grab_stage = 0;
										action_is_finished = true;
									}
									break;
							}
							break;

						case ACTION_STEP_UP: // (FULL_EXTEND -> RETRACT_FRONT -> RETRACT_BACK)
							switch(step_up_state)
							{
								case FULL_EXTEND:
								{
									uint8_t legs_arrived = 0;
									for(int i=0; i<4; i++)
									{
										StepUpProfiles[i].target_pos = 205.0f;
										leg_load_state[i] = LEG_BEARING_WEIGHT;

										if(fabsf(StepUpProfiles[i].target_pos - StepUpProfiles[i].current_pos) < 5.0f) legs_arrived++; // smaller than 5mm
									}
									if(legs_arrived >= 4)
										step_up_state = RETRACT_FRONT_STEP;
									break;
								}

								case RETRACT_FRONT_STEP:
								{
									// Move until the front IR is triggered
									uint8_t legs_arrived = 0;
									if(StepUpIR[0] && StepUpIR[1])
									{
										target_field_y = robot_coor_y; // to stop the robot from moving
										// set the front step to retract
										StepUpProfiles[0].target_pos = 0.0f;
										StepUpProfiles[1].target_pos = 0.0f;

										leg_load_state[0] = LEG_IN_AIR;
										leg_load_state[1] = LEG_IN_AIR;

										for(int i=0; i<2; i++)
											if(fabsf(StepUpProfiles[i].target_pos - StepUpProfiles[i].current_pos) < 5.0f) legs_arrived++;

										if(legs_arrived >= 2)
											step_up_state = RETRACT_BACK_STEP;
									}
									else
									{	// slowly add until the IRs are triggered
										// Only push the target forward if the robot is keeping up (difference between target is 5cm)
										if ((target_field_y - robot_coor_y) < 50.0f) {
										    target_field_y += 10.0f;
										}
									}
									break;
								}

								case RETRACT_BACK_STEP:
								{
									uint8_t legs_arrived = 0;
									if(StepUpIR[2] && StepUpIR[3])
									{
										target_field_y = robot_coor_y;
										// set the front step to retract
										StepUpProfiles[2].target_pos = 0.0f;
										StepUpProfiles[3].target_pos = 0.0f;

										leg_load_state[2] = LEG_IN_AIR;
										leg_load_state[3] = LEG_IN_AIR;
										for(int i=0; i<2; i++)
											if(fabsf(StepUpProfiles[i+2].target_pos - StepUpProfiles[i+2].current_pos) < 5.0f) legs_arrived++;

										if(legs_arrived >= 2)
											step_up_state = MOVE_TO_NEXT_BLOCK;
									}
									else
										if ((target_field_y - robot_coor_y) < 50.0f) target_field_y += 10.0f;
									break;
								}

								case MOVE_TO_NEXT_BLOCK:
								{
								    // 1. Set the target ONCE
								    static bool target_set = false;
								    if (!target_set) {
								        target_field_y = robot_coor_y + 300.0f;
								        target_set = true;
								    }

								    // 2. Wait until the robot actually reaches the +300mm mark
								    if (fabsf(target_field_y - robot_coor_y) < dist_deadband) {
								        action_is_finished = true;
								        target_set = false; // Reset for the next time this action is used
								        step_up_state = FULL_EXTEND; // CRITICAL: Reset the state machine for the next run!
								    }
								    break;
								}

								case FULL_RETRACT:
									break;

								case EXTEND_FRONT_STEP:
									break;

								case EXTEND_BACK_STEP:
									break;
							}
							break;

						case ACTION_STEP_DOWN: // Step down from the back (Extend Back -> Extend Front -> Full Retract)
						case ACTION_STEP_DOWN_MEIHUA: // Step down from the back (Extend Back -> Extend Front -> Full Retract)
							static bool rotate = false;
							switch(step_down_state)
							{
								case EXTEND_BACK_STEP:
								{
									uint8_t legs_arrived = 0;
									if(!StepUpIR[0] && !StepUpIR[1])
									{
										target_field_y = robot_coor_y; // to stop the robot from moving
										target_heading = 180.0f;
										rotate = true;
									}
									else
										if ((target_field_y - robot_coor_y) < 50.0f) target_field_y += 10.0f;

									if(rotate)
									{
										if(abs_err_yaw < 5.0f)
										{
											rotate = false; // clear the flag
											target_field_y += 50.0f; // move a bit and start extend the back legs
											// set the front step to extend
											StepUpProfiles[2].target_pos = 205.0f;
											StepUpProfiles[3].target_pos = 205.0f;

											leg_load_state[2] = LEG_BEARING_WEIGHT;
											leg_load_state[3] = LEG_BEARING_WEIGHT;

											for(int i=0; i<2; i++)
												if(fabsf(StepUpProfiles[i+2].target_pos - StepUpProfiles[i+2].current_pos) < 5.0f) legs_arrived++;

											if(legs_arrived >= 2)
												step_down_state = EXTEND_FRONT_STEP;
										}
									}
									break;
								}

								case EXTEND_FRONT_STEP:
								{
									uint8_t legs_arrived = 0;
									if(StepUpIR[0] && StepUpIR[1])
									{
										target_field_y = robot_coor_y; // to stop the robot from moving
										// set the front step to retract
										StepUpProfiles[0].target_pos = 205.0f;
										StepUpProfiles[1].target_pos = 205.0f;

										leg_load_state[0] = LEG_BEARING_WEIGHT;
										leg_load_state[1] = LEG_BEARING_WEIGHT;

										for(int i=0; i<2; i++)
											if(fabsf(StepUpProfiles[i].target_pos - StepUpProfiles[i].current_pos) < 5.0f) legs_arrived++;

										if(legs_arrived >= 2)
											step_down_state = MOVE_TO_NEXT_BLOCK;
									}
									else
										if ((target_field_y - robot_coor_y) < 50.0f) target_field_y += 10.0f;
									break;
								}

								case MOVE_TO_NEXT_BLOCK:
									{
										// 1. Set the target ONCE
										static bool target_set = false;
										if (!target_set) {
											target_field_y = robot_coor_y + 200.0f;
											target_set = true;
										}

										// 2. Wait until the robot actually reaches the +300mm mark
										if (fabsf(target_field_y - robot_coor_y) < dist_deadband) {
											target_set = false; // Reset for the next time this action is used
											step_down_state = FULL_RETRACT;
										}
										break;
									}

								case FULL_RETRACT:
								{
									// retract all four
									uint8_t legs_arrived = 0;
									for(int i=0; i<4; i++)
									{
										StepUpProfiles[i].target_pos = 0.0f;
										leg_load_state[i] = LEG_IN_AIR;
										if(fabsf(StepUpProfiles[i].target_pos - StepUpProfiles[i].current_pos) < 1.0f) legs_arrived++;
									}
									if(legs_arrived >= 4)
									{
										action_is_finished = true;
										if(autonomous_path[current_waypoint_index].action_on_arrival == ACTION_STEP_DOWN_MEIHUA)
											robot_coor_y = (2000.0f + 1200.0f + 4800.0f + ROBOT_OFFSET); // update the robot_coor_y
									}
									break;
								}

								case FULL_EXTEND:
									// extend all four
									break;

								case RETRACT_FRONT_STEP:
									break;

								case RETRACT_BACK_STEP:
									break;
							}
							break;

						case ACTION_WAIT_2_SEC:
							if ((HAL_GetTick() - auto_timer_start) >= 2000) action_is_finished = true; // Stay in this state until 2s passes
							break;
					}

					// Action is complete! Move to the next waypoint in the array
					if(action_is_finished)
					{
						current_waypoint_index++;

						if (current_waypoint_index >= TOTAL_WAYPOINTS) {
							current_auto_state = AUTO_STATE_DONE; // We finished the whole arena!
						} else {
							// Load the next targets and go back to driving
							if(autonomous_path[current_waypoint_index].action_on_arrival == ACTION_STEP_UP || autonomous_path[current_waypoint_index].action_on_arrival == ACTION_STEP_DOWN)
							{
								target_field_x = robot_coor_x;
								target_field_y = robot_coor_y;
								target_heading = autonomous_path[current_waypoint_index].target_heading;
							}
							else
							{
								target_field_x = autonomous_path[current_waypoint_index].target_x;
								target_field_y = autonomous_path[current_waypoint_index].target_y;
								target_heading = autonomous_path[current_waypoint_index].target_heading;
							}

							if(target_field_x < ROBOT_OFFSET) target_field_x = ROBOT_OFFSET;
							if(target_field_y < ROBOT_OFFSET) target_field_y = ROBOT_OFFSET;

							current_auto_state = AUTO_STATE_DRIVE_TO_TARGET;
						}
					}
					break;

				case AUTO_STATE_DONE:
					// Sequence complete. The PIDs will continue to hold the robot
					break;
			}

			// PID and Kinematics (robot_coor_x and y come from your Odometry_Update() function)
            float error_x = target_field_x - robot_coor_x;
            float error_y = target_field_y - robot_coor_y;

            // Calculate Global Velocities using PID
            // The output of these PIDs is a velocity command (equivalent to joystick limits -127 to 127)
            PID_1DOF_calc(&auto_pid_x, 0.0f, error_x, dt);
            PID_1DOF_calc(&auto_pid_y, 0.0f, error_y, dt);
            PID_1DOF_calc(&heading_pid, 0.0f, yaw_err, dt);

            float v_x_global = auto_pid_x.out;
            float v_y_global = auto_pid_y.out;
            float omega_cmd  = heading_pid.out;

            // Convert Global Velocity to Local Velocity (Robot-Centric)
            float current_yaw_rad = robot_orientation.yaw * (M_PI / 180.0f); // Convert BNO085 yaw to radians for the math library

            float v_x_local =  (v_x_global * cosf(current_yaw_rad)) + (v_y_global * sinf(current_yaw_rad));
            float v_y_local = -(v_x_global * sinf(current_yaw_rad)) + (v_y_global * cosf(current_yaw_rad));

            // Send to Kinematics!
            if(xSemaphoreTake(xPidMutex, pdMS_TO_TICKS(2)) == pdTRUE) // We use the SwerveMutex to safely overwrite the variables that the SwerveAlgoTask uses
            {
                // Inject the automated local velocities directly into the algorithm
                omni_algorithm(v_x_local, v_y_local, omega_cmd, omni_target_speeds, options_button.state);

                xSemaphoreGive(xPidMutex);
            }
        }
        else
		{
			current_auto_state = AUTO_STATE_INIT;

			PID_1DOF_Reset(&auto_pid_x);
			PID_1DOF_Reset(&auto_pid_y);
			PID_1DOF_Reset(&heading_pid);
		}

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

void AutoNavTask(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool is_sensor_fusion_seeded = false;

    while(1)
    {
    	/* XY Encoder update */
    	encoder_x_ticks = TIM23->CNT;
    	encoder_y_ticks = TIM24->CNT;

    	if (!is_sensor_fusion_seeded)
		{
			if (new_laser_x_available && new_laser_y_available)
			{
				// Seed the filter now that the data is real!
				Sensor_Fusion_Init(&robot_coor_x, &robot_coor_y);
				is_sensor_fusion_seeded = true;
			}
		}
		else
		{
			// Only run the continuous update AFTER it has been seeded
			Sensor_Fusion_Update(&robot_coor_x, &robot_coor_y, robot_orientation.yaw);
		}

        if (auto_mode_active && start_btn.state)
        {
        	// Calculate absolute errors for arrival checking
			float abs_err_x = fabsf(target_field_x - robot_coor_x);
			float abs_err_y = fabsf(target_field_y - robot_coor_y);

			float yaw_err = robot_orientation.yaw - target_heading;
			while(yaw_err > 180.0f)  yaw_err -= 360.0f;
			while(yaw_err < -180.0f) yaw_err += 360.0f;
			float abs_err_yaw = fabsf(yaw_err);

			// 1. THE BRAIN: State Machine Sequencer
			switch (current_auto_state)
			{
				case AUTO_STATE_INIT:
					current_nav_index = 0;
//					target_field_x = auto_nav[current_nav_index].target_x;
					target_field_x = robot_coor_x;
					target_field_y = auto_nav[current_nav_index].target_y;
					target_heading = auto_nav[current_nav_index].target_heading;

					if(target_field_x < ROBOT_OFFSET)
						target_field_x = ROBOT_OFFSET;

					if(target_field_y < ROBOT_OFFSET)
						target_field_y = ROBOT_OFFSET;

					PID_1DOF_Reset(&auto_pid_x);
					PID_1DOF_Reset(&auto_pid_y);
					PID_1DOF_Reset(&heading_pid);
					current_auto_state = AUTO_STATE_DRIVE_TO_TARGET;
					break;

				case AUTO_STATE_DRIVE_TO_TARGET:
					// Check if we arrived in X, Y, AND Yaw simultaneously!
					if ((abs_err_x < dist_deadband) &&
						(abs_err_y < dist_deadband) &&
						(abs_err_yaw < heading_deadband))
					{
						current_auto_state = AUTO_STATE_EXECUTE_ACTION;
						auto_timer_start = HAL_GetTick(); // Start timer in case the action needs it
					}
					break;

				case AUTO_STATE_EXECUTE_ACTION:
					// Look at the array to see what mechanism to trigger
					bool action_is_finished = false;
					switch(auto_nav[current_nav_index].action_on_arrival)
					{
						case ACTION_NONE:
							// Do nothing, just move on
							action_is_finished = true;
							break;

						case ACTION_GRAB_SPEAR:
							// triggers servo motor to grab the spear head
							action_is_finished = true;
							break;

						case ACTION_GRAB_KFS_UP:
							// 1. set the vertical and horizontal cascade mechanism distance
							// 2. set the gripper servo angle
							// 3. activate the pneumatic
							action_is_finished = true;
							break;

						case ACTION_GRAB_KFS_DOWN:
							// 1. set the vertical and horizontal cascade mechanism distance
							// 2. set the gripper servo angle
							// 3. activate the pneumatic
							action_is_finished = true;
							break;

						case ACTION_STEP_UP:
							action_is_finished = true;
							break;

						case ACTION_STEP_DOWN:
						case ACTION_STEP_DOWN_MEIHUA:
							action_is_finished = true;
							break;

						case ACTION_WAIT_2_SEC:
							if ((HAL_GetTick() - auto_timer_start) >= 2000) action_is_finished = true; // Stay in this state until 2s passes
							break;
					}

					// Action is complete! Move to the next waypoint in the array
					if(action_is_finished)
					{
						current_nav_index++;

						if (current_nav_index >= total_AutoNav_waypoints) {
							current_auto_state = AUTO_STATE_DONE; // We finished the whole arena!
						} else {
							// Load the next targets and go back to driving
							target_field_x = auto_nav[current_nav_index].target_x;
							target_field_y = auto_nav[current_nav_index].target_y;
							target_heading = auto_nav[current_nav_index].target_heading;

							if(target_field_x < ROBOT_OFFSET)
								target_field_x = ROBOT_OFFSET;

							if(target_field_y < ROBOT_OFFSET)
								target_field_y = ROBOT_OFFSET;
							current_auto_state = AUTO_STATE_DRIVE_TO_TARGET;
						}
					}
					break;

				case AUTO_STATE_DONE:
					// Sequence complete. The PIDs will continue to hold the robot
					break;
			}

			// 2. THE MUSCLE: PID and Kinematics
            // robot_coor_x and y come from your Odometry_Update() function
            float error_x = target_field_x - robot_coor_x;
            float error_y = target_field_y - robot_coor_y;

            // Calculate Global Velocities using PID
            // The output of these PIDs is a velocity command (equivalent to joystick limits -127 to 127)
            PID_1DOF_calc(&auto_pid_x, 0.0f, error_x, dt);
            PID_1DOF_calc(&auto_pid_y, 0.0f, error_y, dt);
            PID_1DOF_calc(&heading_pid, 0.0f, yaw_err, dt);

            float v_x_global = auto_pid_x.out;
            float v_y_global = auto_pid_y.out;
            float omega_cmd  = heading_pid.out;

            // Convert Global Velocity to Local Velocity (Robot-Centric)
            float current_yaw_rad = robot_orientation.yaw * (M_PI / 180.0f); // Convert BNO085 yaw to radians for the math library

            float v_x_local =  (v_x_global * cosf(current_yaw_rad)) + (v_y_global * sinf(current_yaw_rad));
            float v_y_local = -(v_x_global * sinf(current_yaw_rad)) + (v_y_global * cosf(current_yaw_rad));

            // Send to Kinematics!
            if(xSemaphoreTake(xPidMutex, pdMS_TO_TICKS(2)) == pdTRUE) // We use the SwerveMutex to safely overwrite the variables that the SwerveAlgoTask uses
            {
                // Inject the automated local velocities directly into the algorithm
                omni_algorithm(v_x_local, v_y_local, omega_cmd, omni_target_speeds, options_button.state);

                xSemaphoreGive(xPidMutex);
            }
        }
        else
		{
			// If auto mode is turned off, safely reset the sequence so it
			current_auto_state = AUTO_STATE_INIT;

			PID_1DOF_Reset(&auto_pid_x);
			PID_1DOF_Reset(&auto_pid_y);
			PID_1DOF_Reset(&heading_pid);
		}

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

void Robot_Can_Init(void)
{
	// Clear the entire struct (Safety)
	memset(&RobotCan, 0, sizeof(Robot_CAN_Manager_Struct));

	// Assign RoboMaster IDs (Switch settings on ESC)
	// FDCAN1: Swerve Steering M2006 Motors (Indices 0-3)
	RobotCan.RM_IDs[0] = 0x201; // Front Right
	RobotCan.RM_IDs[1] = 0x202; // Front Left
	RobotCan.RM_IDs[2] = 0x203; // Back Left
	RobotCan.RM_IDs[3] = 0x204; // Back Right

	// FDCAN3: Step-Up M3508 Motors (Indices 4-7)
	RobotCan.RM_IDs[4] = 0x201;
	RobotCan.RM_IDs[5] = 0x202;
	RobotCan.RM_IDs[6] = 0x203;
	RobotCan.RM_IDs[7] = 0x204;

	// FDCAN3: Mechanism M3508 Motors (Indices 8-9)
	RobotCan.RM_IDs[8] = 0x205; // Mech 1
	RobotCan.RM_IDs[9] = 0x206; // Mech 2
}

void PWM_TIM1_Init(void)
{
	// PWM for servo control
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void XY_Encoder_TIM_Init(void)
{
	// Start the encoder interface on TIM23 (All channels)
	HAL_TIM_Encoder_Start(&htim23, TIM_CHANNEL_ALL);

	// Start the encoder interface on TIM24 (All channels)
	HAL_TIM_Encoder_Start(&htim24, TIM_CHANNEL_ALL);
}

void Pneumatic_Init(void)
{
	HAL_GPIO_WritePin(PNEU_BOT_GPIO_Port, PNEU_BOT_Pin, RESET);
	HAL_GPIO_WritePin(PNEU_EXTEND_GPIO_Port, PNEU_EXTEND_Pin, RESET);
	HAL_GPIO_WritePin(PNEU_TOP_GPIO_Port, PNEU_TOP_Pin, RESET);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_Delay(2000); // delay for two seconds to ensure the robot fully ready
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN3_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_I2C5_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_TIM1_Init();
  MX_FDCAN2_Init();
  MX_TIM23_Init();
  MX_TIM24_Init();
  /* USER CODE BEGIN 2 */
  Pneumatic_Init();
  Robot_Can_Init();
  FDCAN_1_Init(&hfdcan1);
  FDCAN_3_Init(&hfdcan3);
  AS5047P_SPI_Init(as5047p_groups, 4); // 4 groups (4 CS pins)

  PWM_TIM1_Init();
  XY_Encoder_TIM_Init();
  // BNO085: Force BOOT pin HIGH to enter Normal Application Mode (LOW -> Bootloader!)
  HAL_GPIO_WritePin(GPIOG, BNO_BOOT_Pin, GPIO_PIN_SET);

  RM_motor_struct_init(&M3508_SpeedParams, 100, 30, RM_M3508);

  for(int i = 0; i < 4; i++)
  {
	  PID_2DOF_Init(&omni_pid_rpm_current[i], kp_omni, ki_omni, kd_omni, n_omni, b_omni, c_omni, integral_limit_omni, max_out_omni);

	  PID_2DOF_Init(&stepup_pid_rpm_current[i], kp_omni, ki_omni, kd_omni, n_omni, b_omni, c_omni, integral_limit_omni, max_out_omni);
	  PID_1DOF_Init(&stepup_pid_angle_rpm[i], kp_m3508_as5047p, ki_m3508_as5047p, kd_m3508_as5047p, n_m3508_as5047p, integral_limit_m3508_as5047p, max_out_m3508_as5047p, POSITION_PID, dt, stepup_as5047p_angle_deadband);
//	  PID_2DOF_Init(&stepup_pid_rpm_current[i], kp_rpm_current_stepup, ki_rpm_current_stepup, kd_rpm_current_stepup, n_rpm_current_stepup, b_rpm_current_stepup, c_rpm_current_stepup, integral_limit_rpm_current_stepup, max_out_rpm_current_stepup);
//	  PID_1DOF_Init(&stepup_pid_angle_rpm[i], kp_m3508_ang, ki_m3508_ang, kd_m3508_ang, n_m3508_ang, integral_limit_m3508_ang, max_out_m3508_ang, POSITION_PID, dt, encoder_deadband);
  }

  PID_1DOF_Init(&heading_pid, kp_bno, ki_bno, kd_bno, n_bno, integral_limit_bno, max_out_bno, POSITION_PID, dt, heading_deadband);
  PID_1DOF_Init(&pitch_pid, kp_pitch, ki_pitch, kd_pitch, n_pitch, integral_limit_pitch, max_out_pitch, POSITION_PID, dt, pitch_deadband);

  PID_2DOF_Init(&mech1_pid_rpm_current, kp_omni, ki_omni, kd_omni, n_omni, b_omni, c_omni, integral_limit_omni, max_out_omni); // Mech1 -> Vertical Cascade
  PID_1DOF_Init(&mech1_pid_angle_rpm, kp_m3508_as5047p, ki_m3508_as5047p, kd_m3508_as5047p, n_m3508_as5047p, integral_limit_m3508_as5047p, max_out_m3508_as5047p, POSITION_PID, dt, stepup_as5047p_angle_deadband);

  PID_2DOF_Init(&mech2_pid_rpm_current, kp_m3508_HC, ki_m3508_HC, kd_m3508_HC, n_m3508_HC, b_m3508_HC, c_m3508_HC, integral_limit_m3508_HC, max_out_m3508_HC); // Mech2 -> Horizontal Cascade
  PID_1DOF_Init(&mech2_pid_angle_rpm, kp_m3508_as5047p, ki_m3508_as5047p, kd_m3508_as5047p, n_m3508_as5047p, integral_limit_m3508_as5047p, max_out_m3508_as5047p, POSITION_PID, dt, stepup_as5047p_angle_deadband);

  PID_1DOF_Init(&auto_pid_x, kp_dist, ki_dist, kd_dist, n_dist, integral_limit_dist, max_out_dist, POSITION_PID, dt, dist_deadband);
  PID_1DOF_Init(&auto_pid_y, kp_dist, ki_dist, kd_dist, n_dist, integral_limit_dist, max_out_dist, POSITION_PID, dt, dist_deadband);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  I2CMutex = xSemaphoreCreateMutex();
  if(I2CMutex != NULL)
	  xTaskCreateStatic(PS4task, "PS4 Task", 1024, NULL, 5, xPs4Stack, &xPs4TaskTCB);

  xCanTxMutex = xSemaphoreCreateMutex();
  if(xCanTxMutex != NULL)
	  xTaskCreateStatic(CanSendTask, "Can Transmit Task", 512, NULL, 6, xCanTransmitStack, &xCanTransmitTaskTCB);

  xPidMutex = xSemaphoreCreateMutex();
  xTaskCreateStatic(NavPidBldcTask, "PID Task", 512, NULL, 7, xPidStack, &xPidTaskTCB);
  xTaskCreateStatic(StepUpTask, "Step Up Task", 512, NULL, 8, xStepUpStack, &xStepUpTaskTCB);
  xTaskCreateStatic(CascadeMechTask, "Cascade Mechanism Task", 512, NULL, 9, xCascadeMechStack, &xCascadeMechTaskTCB);

//  xTaskCreateStatic(AutoNavTask, "Auto Navigation Task", 512, NULL, 9, xAutoNavStack, &xAutoNavTaskTCB);
  xTaskCreateStatic(AutoStateMachineTask, "Autonomous State Machine Task", 512, NULL, 9, xAutoStateMachineStack, &xAutoStateMachineTaskTCB);
  xCan3TxMutex = xSemaphoreCreateMutex();
  xSpiMutex = xSemaphoreCreateMutex();
  xTaskCreate(AS5047P_DiagTask, "AS5047_Diag", 256, NULL, 12, NULL);

  BNO085_Int_Sem = xSemaphoreCreateBinary();
  xTaskCreateStatic(BNO085Task, "BNO085 Task", 256, NULL, 10, xBno085Stack, &xBno085TaskTCB);
  EulerQueue = xQueueCreate(1, sizeof(bno085_euler_t)); // Create a queue that holds exactly 1 item of type bno085_euler_t

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 6;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 16;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 10;
  hfdcan1.Init.ExtFiltersNbr = 5;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 6;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 16;
  hfdcan2.Init.NominalTimeSeg2 = 3;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 276;
  hfdcan2.Init.StdFiltersNbr = 10;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 32;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 6;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 16;
  hfdcan3.Init.NominalTimeSeg2 = 3;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 1;
  hfdcan3.Init.DataTimeSeg2 = 1;
  hfdcan3.Init.MessageRAMOffset = 542;
  hfdcan3.Init.StdFiltersNbr = 10;
  hfdcan3.Init.ExtFiltersNbr = 5;
  hfdcan3.Init.RxFifo0ElmtsNbr = 32;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxFifo1ElmtsNbr = 0;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 0;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C5_Init(void)
{

  /* USER CODE BEGIN I2C5_Init 0 */

  /* USER CODE END I2C5_Init 0 */

  /* USER CODE BEGIN I2C5_Init 1 */

  /* USER CODE END I2C5_Init 1 */
  hi2c5.Instance = I2C5;
  hi2c5.Init.Timing = 0x307075B1;
  hi2c5.Init.OwnAddress1 = 0;
  hi2c5.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c5.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c5.Init.OwnAddress2 = 0;
  hi2c5.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c5.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c5.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c5, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c5, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C5_Init 2 */

  /* USER CODE END I2C5_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 240 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM23 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM23_Init(void)
{

  /* USER CODE BEGIN TIM23_Init 0 */

  /* USER CODE END TIM23_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM23_Init 1 */

  /* USER CODE END TIM23_Init 1 */
  htim23.Instance = TIM23;
  htim23.Init.Prescaler = 0;
  htim23.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim23.Init.Period = 4294967295;
  htim23.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim23.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim23, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim23, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM23_Init 2 */

  /* USER CODE END TIM23_Init 2 */

}

/**
  * @brief TIM24 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM24_Init(void)
{

  /* USER CODE BEGIN TIM24_Init 0 */

  /* USER CODE END TIM24_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM24_Init 1 */

  /* USER CODE END TIM24_Init 1 */
  htim24.Instance = TIM24;
  htim24.Init.Prescaler = 0;
  htim24.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim24.Init.Period = 4294967295;
  htim24.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim24.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim24, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim24, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM24_Init 2 */

  /* USER CODE END TIM24_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PNEU_TOP_GPIO_Port, PNEU_TOP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_MECH1_Pin|CS_MECH2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, BNO_RST_Pin|BNO_BOOT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_STEP_UP_GPIO_Port, CS_STEP_UP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PNEU_BOT_Pin|PNEU_EXTEND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_SWERVE_GPIO_Port, CS_SWERVE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PNEU_TOP_Pin */
  GPIO_InitStruct.Pin = PNEU_TOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PNEU_TOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CASCADE_MIN_Pin */
  GPIO_InitStruct.Pin = CASCADE_MIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CASCADE_MIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_1_Pin IR_4_Pin */
  GPIO_InitStruct.Pin = IR_1_Pin|IR_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : CASCADE_MAX_Pin */
  GPIO_InitStruct.Pin = CASCADE_MAX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CASCADE_MAX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_MECH1_Pin CS_MECH2_Pin */
  GPIO_InitStruct.Pin = CS_MECH1_Pin|CS_MECH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_3_Pin IR_2_Pin */
  GPIO_InitStruct.Pin = IR_3_Pin|IR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BNO_RST_Pin BNO_BOOT_Pin */
  GPIO_InitStruct.Pin = BNO_RST_Pin|BNO_BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO_HINT_Pin */
  GPIO_InitStruct.Pin = BNO_HINT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BNO_HINT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_STEP_UP_Pin */
  GPIO_InitStruct.Pin = CS_STEP_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CS_STEP_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PNEU_BOT_Pin PNEU_EXTEND_Pin */
  GPIO_InitStruct.Pin = PNEU_BOT_Pin|PNEU_EXTEND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_SWERVE_Pin */
  GPIO_InitStruct.Pin = CS_SWERVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CS_SWERVE_GPIO_Port, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//extern TIM_HandleTypeDef htim6;
//void TIM6_DAC_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
//
//  /* USER CODE END TIM6_DAC_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim6);
//  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
//
//  /* USER CODE END TIM6_DAC_IRQn 1 */
//}

void EXTI9_5_IRQHandler(void)
{
    // This STM32 HAL function checks the hardware flags and automatically
    // routes the signal to your HAL_GPIO_EXTI_Callback below.
    HAL_GPIO_EXTI_IRQHandler(BNO_HINT_Pin);
}

// Interrupt Service Routine (ISR Function)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {	// External Interrupt Callback (Must be mapped to the BNO085 INT pin)
    if (GPIO_Pin == BNO_HINT_Pin) {
    	// Ensure the semaphore actually exists before giving it!
		if (BNO085_Int_Sem != NULL) {
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(BNO085_Int_Sem, &xHigherPriorityTaskWoken);

			// Critical: Only yield if the RTOS scheduler is actually running!
			// If the sensor fires this interrupt during boot-up, this prevents a Hard Fault.
			if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}

		}
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    // Check if the interrupt was triggered by a new message
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        // Retrieve the message from the FIFO
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // Route to the correct processing logic based on which CAN bus fired
            if (hfdcan->Instance == FDCAN1)
            {
                // CAN1: Steering Motor M2006
                Process_RM_Msg(&RxHeader, RxData, &RobotCan, RM1_ID, 0);
            }
            else if (hfdcan->Instance == FDCAN3)
            {
                // 1. Step-Up Motors (IDs 0x201 to 0x204) -> Map to indices 4, 5, 6, 7
				if (RxHeader.Identifier >= 0x201 && RxHeader.Identifier <= 0x204)
				{
					Process_RM_Msg(&RxHeader, RxData, &RobotCan, RM1_ID, 4);
				}
				// 2. Mechanism Motors (IDs 0x205 to 0x208) -> Map to indices 8, 9, 10, 11
				else if (RxHeader.Identifier >= 0x205 && RxHeader.Identifier <= 0x208)
				{
					Process_RM_Msg(&RxHeader, RxData, &RobotCan, RM2_ID, 8);
				}
            }
        }
    }
}

// 2. The DMA Hardware calls this automatically when exactly 9 bytes are received
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART7) {
        new_laser_y_available = true;
//		laser_y_distance = Parse_Distance(y_laser_rx_buffer);
        Parse_Distance_Status(y_laser_rx_buffer, &laser_y_distance, &laser_y_status);
	}

	if (huart->Instance == UART5) {
        new_laser_x_available = true;
//        laser_x_distance = Parse_Distance(x_laser_rx_buffer);
        Parse_Distance_Status(x_laser_rx_buffer, &laser_x_distance, &laser_x_status);

        // Note: Because DMA Mode is "Normal", it stops after receiving 9 bytes.
        // We will call Request_Sensor_Distance_DMA() again when we want the next reading.
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // Check if the error occurred on either of your Laser Sensor UART lines
    if (huart->Instance == UART5 || huart->Instance == UART7)
    {
        // If the error is an Overrun (ORE), Noise (NE), or Framing (FE) error
        if (huart->ErrorCode & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE))
        {
            // Note: Your 1Hz AS5047P_DiagTask will automatically restart the
            // communication on the next cycle because the state is now READY again.
//        	HAL_UART_Abort_IT(huart);

            if (huart->Instance == UART5) x_laser_error_count++;
            if (huart->Instance == UART7) y_laser_error_count++;
        }
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
