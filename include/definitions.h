/*******************************************************************************
** I/O pin assignment
*******************************************************************************/

#define FRONT_RIGHT_ENCODER_A 39
#define FRONT_RIGHT_ENCODER_B 36
#define FRONT_LEFT_ENCODER_A 33
#define FRONT_LEFT_ENCODER_B 32

#define REAR_RIGHT_ENCODER_A 35
#define REAR_RIGHT_ENCODER_B 34
#define REAR_LEFT_ENCODER_A 26
#define REAR_LEFT_ENCODER_B 25

#define FRONT_LEFT_PWM_FORWARD  16
#define FRONT_LEFT_PWM_REVERSE  13
#define FRONT_RIGHT_PWM_FORWARD 21
#define FRONT_RIGHT_PWM_REVERSE 19

#define REAR_LEFT_PWM_FORWARD  18
#define REAR_LEFT_PWM_REVERSE  17
#define REAR_RIGHT_PWM_FORWARD 23
#define REAR_RIGHT_PWM_REVERSE 22

/*******************************************************************************
** PWM Related parameters
*******************************************************************************/
#define WHEEL_RADIUS 0.21   // m
#define WHEEL_TRACK  0.86   // m
#define WHEEL_BASE   0.75   // m
#define MAX_SPEED	 2.4	// m/s
#define MAX_PWM	 	 255

/*******************************************************************************
** Encoder related parameters
*******************************************************************************/

#define ENCODER_REDUCTION 1
#define ENCODER_PULSES_PER_ROTATION 2400	// Phase count x pulses per rotation in spec sheet

/*******************************************************************************
** PID related parameters
*******************************************************************************/

#define K_P 1
#define K_I 0.5
#define K_D 0

/*******************************************************************************
** constants for ros messages
*******************************************************************************/

#define XY_COVARIANCE  0.001
#define NON_COVARIANCE 100000
#define YAW_COVARIANCE 1000
#define IMU_COVARIANCE 0.0025


/*******************************************************************************
** status checking
*******************************************************************************/

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
