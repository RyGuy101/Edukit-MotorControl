




/*
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 *
 *    Acknowledgments to the invaluable development, support and guidance by
 *    Marco De Fazio, Giorgio Mariano, Enrico Poli, and Davide Ghezzi
 *    of STMicroelectronics
 *
 *              Motor Control Curriculum Feedback Control System
 *
 * Includes:
 * 		Stepper Motor Control interface based on the IHM01A1 and Nucleo F401RE
 * 		Optical Encoder interface supported by the Nucleo F401RE
 * 		Primary PID controller for Pendulum Angle Control
 * 		Secondary PID controller for Rotor Angle Control
 * 		User interfaces for remote access to system configuration including
 * 				Stepper Motor speed profile, current limits, and others
 * 				Control system parameters
 *
 * @author  William J. Kaiser (UCLA Electrical and Computer Engineering).
 *
 * Application based on development by STMicroelectronics as described below
 *
 * @version V1.0
 * @date    May 15th, 2019
 *
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 * @author  IPC Rennes
 * @version V1.10.0
 * @date    March 16th, 2018
 * @brief   This example shows how to use 1 IHM01A1 expansion board
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/*
 * Integrated Rotary Inverted Pendulum System Configuration
 *
 * ********** Primary Components *************************
 * Processor:			Nucleo F401RE
 * Motor Interface: 	IHM01A1 Stepper Motor Controller
 * Motor Power Supply:	12V 2A Supply
 * Encoder:				LPD3806-600BM-G5-24C 600 Pulse Per Revolution Incremental Rotary Encoder
 * Stepper Motor:		NEMA-17 17HS19-2004S  Stepper Motor
 *
 *********** Stepper Lead Assignment *********************
 *
 * Lead Color	IHM01A1 Terminal
 * 	 Blue			-A
 * 	 Red			+A
 * 	 Green			-B
 * 	 Black			+B
 *
 * Stepper Lead Extension Cable (if present) replaces Blue with White
 *
 * Caution: Please note that motors have been receieved from the vendor showing
 * reversal of Motor White and Motor Red.  Check rotor operation after assembly
 * and in initial testing.
 *
 * ********** Encoder Lead Assignment *********************
 *
 * Lead Color	Nucleo F401RE Terminal
 * 	Red 			5V
 * 	Black 			GND
 * 	White 			GPIO Dir3
 * 	Green 			GPIO Dir2
 *
 * Note: Optical Encoder is LPD3806-600BM-G5-24C This encoder requires an open collector pull up resistor.
 * Note: GPIO_PULLUP is set in HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder) of
 * stm32f4xx_hal_msp.c Line 217
 *
 * Initial Motor Speed Profiles, Torque Current, and Overcurrent Thresholds set in l6474_target_config.h
 *
 *
 *********** Control Loop Cycle Period *******************
 ***********
 * 	Note All Control Parameters are based on average cycle time values of 4 to 4.5 milliseconds
 *
 * 	Cycle time must be adjusted by CYCLE_DELAY value
 *
 * 	Note that serial data transport delay contributes to cycle delay and changes in serial data
 * 	transport must be compensated for by changing the value of CYCLE_DELAY
 *
 */




/*
 * System Configuration Parameters
 *
 * ENABLE_SUSPENDED_PENDULUM_CONTROL is set to 0 for Inverted Pendulum and
 * 									 set to 1 for Suspended Pendulum
 * ENABLE_DUAL_PID is set to 1 to enable control action
 *
 * High Speed 2 milllisecond Control Loop delay, 500 Hz Cycle System Configuration
 *
 * Motor Speed Profile Configurations are:
 *
 *						MAX_ACCEL: 3000; 	MAX_DECEL 3000
 * High Speed Mode:		MAX_SPEED: 1000; 	MIN_SPEED 500
 * Medium Speed Mode:	MAX_SPEED: 1000; 	MIN_SPEED 300
 * Low Speed Mode:		MAX_SPEED: 1000; 	MIN_SPEED 200
 * Suspended Mode: 		MAX_SPEED: 1000; 	MIN_SPEED 200
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "main.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/*
 * Timer 3, UART Transmit, and UART DMA Receive declarations
 */

TIM_HandleTypeDef htim3;

/*
  * Timer 3, UART Transmit, and UART DMA Receive declarations
  */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/*
 * UART DMA definitions
 */

#define UART_RX_BUFFER_SIZE 	(200)
#define SERIAL_MSG_MAXLEN 		(100)
#define SERIAL_MSG_EOF          '\r'

/*
 * UART Receive data structure
 */

typedef struct {
	uint32_t Len; /*!< Message length           */
	uint8_t Data[SERIAL_MSG_MAXLEN]; /*!< Message data             */
} T_Serial_Msg;

T_Serial_Msg Msg;

uint8_t RxBuffer[UART_RX_BUFFER_SIZE];
uint16_t Extract_Msg(uint8_t *CircularBuff, uint16_t StartPos, uint16_t LastPos,
		uint16_t BufMaxLen, T_Serial_Msg *Msg);

char tmp_string[256];

/*
 * Timer and UART Initialization and Error Report declarations
 */

static volatile uint16_t gLastError;
/* Private function prototypes -----------------------------------------------*/
static void MyFlagInterruptHandler(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/*
 * Control System and Motor Configuration Parameter Definitions
 */

/*
 * Sample rates defined for controller operation
 *
 * Note that these values scale derivative and integral computations
 *
 */
/* Define value for High Speed System is 500 */
#define SAMPLE_FREQUENCY 500					// Set to 500 for default
/* Define value for High Speed System is 0.002 */
#define T_SAMPLE_ROTOR 0.002					// Set to 0.002 for default
/* Define value for High Speed System is 0.002 */
#define T_SAMPLE 0.002							// Set to 0.002 for default
/* Define value for Cycle Delay for High Speed System is 1 */
#define CYCLE_DELAY 1

#define ENABLE_HIGH_SPEED_SAMPLING_MODE 0

//#define STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE 	17.778	//	Stepper response in step value command to rotation in degrees
#define STEPPER_READ_POSITION_STEPS_PER_DEGREE 		8.889	//	Stepper position read value in steps per degree
#define STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE 	STEPPER_READ_POSITION_STEPS_PER_DEGREE
#define ENCODER_READ_ANGLE_SCALE 					6.6667 // Angle Scale 6.66667 for 600 Pulse Per Rev Resolution Optical Encoder

#define ENCODER_ANGLE_POLARITY -1.0				// Note that physical system applies negative polarity to pendulum angle
												// by definition of coordinate system.

#define CYCLE_LIMIT 100000 						// Cycle limit determines run time as product of cycle limit and cycle time (typical 5 msec).
#define ENABLE_CYCLE_INFINITE 1 				// ENABLE_CYCLE_INFINITE set to 1 if continuous operation is to be enabled

#define MAX_TORQUE_CURRENT 800 					// 800 Selected Value for Integrated Rotary Inverted Pendulum System

#define OVERCURRENT_THRESHOLD 2000				// 2000 Selected Value for Integrated Rotary Inverted Pendulum System
#define SHUTDOWN_TORQUE_CURRENT 0				// 0 Selected Value for Integrated Rotary Inverted Pendulum System

/*
 * Note that Speed Profiles are set at run time during execution.
 *
 * Default speed Profiles are set in l6474_target_config.h
 */

#define MAX_SPEED_UPPER_INIT 10000						// Initialization value
#define MIN_SPEED_UPPER_INIT 10000						// Initialization value
#define MAX_SPEED_LOWER_INIT 30							// Initialization value
#define MIN_SPEED_LOWER_INIT 30							// Initialization value
#define MAX_ACCEL_UPPER_INIT 10000						// Initialization value
#define MAX_DECEL_UPPER_INIT 10000	 					// Initialization value

/*
 * Configuration of Motor Speed Profile at initialization
 */

#define MAX_SPEED 1000
#define MIN_SPEED 400
#define MAX_ACCEL 3000
#define MAX_DECEL 3000

/*
 * ONLY MODE 1 SUPPORTED
 */

#define MAX_SPEED_MODE_2 1000
#define MIN_SPEED_MODE_2 500
#define MAX_SPEED_MODE_1 1000
#define MIN_SPEED_MODE_1 400
#define MAX_SPEED_MODE_3 1000
#define MIN_SPEED_MODE_3 300
#define MAX_SPEED_MODE_4 1000
#define MIN_SPEED_MODE_4 200
#define MAX_SPEED_MODE_5 1000
#define MIN_SPEED_MODE_5 400


#define ROTOR_POSITION_MAX_DIFF 500 			// Limit of maximum permitted difference in motor command values on successive cycles

#define ENABLE_SUSPENDED_PENDULUM_CONTROL 0     // Set to 0 for Inverted Pendulum Mode - Set to 1 for Suspended Pendulum Mode

#define ENCODER_START_OFFSET 0 					// Encoder configurations may display up to 1 degree initial offset
#define ENCODER_START_OFFSET_DELAY 0			// Encoder offset delay limits application of initial offset
#define START_ANGLE 1							// Pendulum angle tolerance for system pendulum orientation at start


/*
 * Single PID, Dual PID and LQR Controllers are implemented as summation of Primary
 * and Secondary PID controller structures.  These two controller outputs are summed
 * and supplied to Rotor Control
 *
 * PID Controller parameters.  LQR Controllers are implemented with integral gain of 0.
 *
 */

#define PRIMARY_WINDUP_LIMIT 100				// Integrator wind up limits for PID controllers
#define SECONDARY_WINDUP_LIMIT 100				// Integrator wind up limits for PID controllers

#define ENABLE_LIMITER 1
#define LIMITER_THRESHOLD 2
#define LIMITER_SLOPE 8

/*
 * High Speed Mode Values: 5, 25, 30
 */
#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY 20  		// 5 - Corner frequency of low pass filter of Primary PID derivative
#define LP_CORNER_FREQ_ROTOR 50 							// 5 - Corner frequency of low pass filter of Rotor Angle
#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR 50 	// 50 Corner frequency of low pass filter of Secondary PID derivative

#define LQR_INTEGRAL_ENABLE 0							// Enables LQR mode including integral of rotor position error

#define PRIMARY_PROPORTIONAL_MODE_2 	628.7
#define PRIMARY_INTEGRAL_MODE_2     	0
#define PRIMARY_DERIVATIVE_MODE_2   	84.1

#define SECONDARY_PROPORTIONAL_MODE_2 	4.24
#define SECONDARY_INTEGRAL_MODE_2     	0
#define SECONDARY_DERIVATIVE_MODE_2   	6.38


#define PRIMARY_PROPORTIONAL_MODE_3 	842.5
#define PRIMARY_INTEGRAL_MODE_3     	0
#define PRIMARY_DERIVATIVE_MODE_3   	112.7

#define SECONDARY_PROPORTIONAL_MODE_3 	4.24
#define SECONDARY_INTEGRAL_MODE_3     	0
#define SECONDARY_DERIVATIVE_MODE_3   	8.82

#define PRIMARY_PROPORTIONAL_MODE_5 	2000
#define PRIMARY_INTEGRAL_MODE_5     	400
#define PRIMARY_DERIVATIVE_MODE_5   	300

#define SECONDARY_PROPORTIONAL_MODE_5 	4
#define SECONDARY_INTEGRAL_MODE_5     	0.2
#define SECONDARY_DERIVATIVE_MODE_5   	16

/*
 * Motor Control Configuration LQR Mode 3
 */

#define PRIMARY_PROPORTIONAL_MODE_1 	705.3
#define PRIMARY_INTEGRAL_MODE_1     	0.0
#define PRIMARY_DERIVATIVE_MODE_1   	94.4

#define SECONDARY_PROPORTIONAL_MODE_1 	4.24
#define SECONDARY_INTEGRAL_MODE_1     	0.0
#define SECONDARY_DERIVATIVE_MODE_1   	8.28

/* Gain values for Pendulum without Mass attached */

#define ROTOR_PID_PROPORTIONAL_GAIN_SINGLE_PID_MODE	4.24
#define ROTOR_PID_INTEGRAL_GAIN_SINGLE_PID_MODE 	0.0
#define ROTOR_PID_DIFFERENTIAL_GAIN_SINGLE_PID_MODE 10.15

/*
 * Suspended Mode with Motor Control Configuration 1 Mode 3
 */

#define PRIMARY_PROPORTIONAL_MODE_4 	-500.0
#define PRIMARY_INTEGRAL_MODE_4     	-90.0
#define PRIMARY_DERIVATIVE_MODE_4   	-70

#define SECONDARY_PROPORTIONAL_MODE_4 	-5
#define SECONDARY_INTEGRAL_MODE_4     	-2
#define SECONDARY_DERIVATIVE_MODE_4   	-5


/*
 * Single PID Introductory Mode 5
 */


#define DEFAULT_START_MODE	mode_1

#define ENABLE_ADAPTIVE_MODE 0
#define ADAPTIVE_THRESHOLD_LOW 30				// Default value 30
#define ADAPTIVE_THRESHOLD_HIGH 2				// Default value 2
#define ADAPTIVE_STATE 0
#define ADAPTIVE_DWELL_PERIOD 2000					// Determines dwell period during state transition

#define USER_TRANSITION_DWELL 500

/*
 * START_DEFAULT_MODE_TIME determines time delay for waiting for user input after start or reset.
 * For a time (in ticks) greater than this period, control will initiate with default mode 1 if
 * no user input appears.
 *
 * This permits system operation in default mode independent of external command from separate
 * host.
 */

#define START_DEFAULT_MODE_TIME 5000			// 5 second delay to permit user input after system start
												// If no user response then set default values
#define PENDULUM_ORIENTATION_START_DELAY 10000	// Time permitted to user to orient Pendulum vertical at start

#define INITIAL_START_DELAY 1000				// Determines time available for ensuring pendulum down and prior to user prompt
#define CONTROL_START_DELAY 3000 				// Determines time available to user after prompt for adjusting pendulum upright
#define INITIAL_PENDULUM_MOTION_TEST_DELAY 2000 // Determines delay time between successive evaluations of pendulum motion

#define ENABLE_PID 1							// Enable control operation - default value of 1 (may be disabled for test operations)
#define ENABLE_DUAL_PID 1						// Note ENABLE_DUAL_PID is set to 1 by default for summation of PID controllers
												// for either Dual PID or LQR systems
#define ENABLE_ENCODER_ANGLE_SLOPE_CORRECTION 1			// Set to 1 for PID // 0 LQR // 0 for Susp-Mode
#define LP_CORNER_FREQ_LONG_TERM 0.05					// Set to 0.05 for PID and for LQR
#define ENCODER_ANGLE_SLOPE_CORRECTION_SCALE 22.5		// Set to 22.5
#define ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT	0	// Sets limit on operation time for slope angle correction
														// If set to zero, slope correction operates at all times
														// Default set to zer
/*
 * Rotor position limits are defined to limit rotor rotation to one full rotation in clockwise or
 * counterclockwise motion.
 */

#define ROTOR_POSITION_POSITIVE_LIMIT 360		// Maximum allowed rotation in positive angle in degrees
#define ROTOR_POSITION_NEGATIVE_LIMIT -360		// Minimum allowed rotation in negative angle in degrees

/*
 * Encoder position limits are defined to detect excursions in pendulum angle corresponding to
 * departure from control and to initiate control loop exit
 */

#define ENCODER_POSITION_POSITIVE_LIMIT 40		// Maximum allowed rotation in positive angle in degrees
#define ENCODER_POSITION_NEGATIVE_LIMIT -40		// Minimum allowed rotation in negative angle in degrees

/*
 * Setting ENABLE_MOD_SIN_ROTOR_TRACKING to 1 enables a Rotor Position tracking command in the
 * form of an amplitude modulated sine wave signal
 * Frequency units and Rate are Hz
 * Amplitude units are steps
 *
 */

#define ENABLE_MOD_SIN_ROTOR_TRACKING 1		// If selected, disable all other modulation inputs
#define MOD_SIN_CARRIER_FREQ 0.003			// 0.003 default
#define MOD_SIN_START_CYCLES 10000			// 10000 default
#define MOD_SIN_AMPLITUDE 600				// 600 default
#define MOD_SIN_MODULATION_FREQ  0.001		// 0.001 default
/* Define for High Speed System */
#define MOD_SIN_SAMPLE_RATE 500.0			// 500.0 default for Low Speed
#define ENABLE_SIN_MOD 1					// 1 default

#define ENABLE_ROTOR_CHIRP 0				// If selected, disable all other modulation inputs
#define ROTOR_CHIRP_START_FREQ 0.001		// 0.001 default
#define ROTOR_CHIRP_END_FREQ 0.05			// 0.05 default
#define ROTOR_CHIRP_PERIOD 40000			// 40000 default
#define ROTOR_CHIRP_SWEEP_DELAY 10000		// 10000 default - enables control system to recover between sweeps
/* Define for High Speed System */
#define ROTOR_CHIRP_SAMPLE_RATE 500.0		// 500.0 default for Low Speed
#define ROTOR_CHIRP_START_CYCLES 0			// 0 default
#define ROTOR_CHIRP_STEP_AMPLITUDE 0.3		// 0.3 default

#define ENABLE_ROTOR_TRACK_COMB_SIGNAL 0				// If selected, disable all other modulation inputs
#define ROTOR_TRACK_COMB_SIGNAL_PERIOD 50000			// 50000 default
#define ROTOR_TRACK_DELAY 10000							// 10000 default - enables control system to recover between sweeps
/* Define for High Speed System */
#define ROTOR_TRACK_COMB_SIGNAL_SAMPLE_RATE 500.0		// 500.0 default for Low Speed
#define ROTOR_TRACK_COMB_SIGNAL_START_CYCLES 0			// 0 default
#define ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE 0.05			// 0.05 default


#define ENABLE_DISTURBANCE_REJECTION_STEP 1

/*
 * Set ENABLE_ENCODER_TEST to 1 to enable a testing of encoder response for verification
 * occurring prior to control system start.
 *
 * This will be set to 0 and disabled for normal operation
 *
 */

#define ENABLE_ENCODER_TEST 0

/*
 * Set ENABLE_ROTOR_ACTUATOR_TEST to 1 to enable a testing of encoder response for verification
 * occurring prior to control system start.
 *
 * This will be set to 0 and disabled for normal operation
 *
 */

#define ENABLE_ROTOR_ACTUATOR_TEST 0
#define ROTOR_ACTUATOR_TEST_CYCLES 1

/*
 * Setting ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE = 1 applies a Rotor Position tracking
 * command input step signal
 */

#define ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE 1			// If selected, disable all other modulation inputs
#define ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE 20		// Default 8. Amplitude of step cycle. Note: Peak-to-Peak amplitude is double this value
#define ROTOR_POSITION_STEP_RESPONSE_CYCLE_INTERVAL 8000 	// Default 8000
#define STEP_RESPONSE_AMP_LIMIT_ENABLE 1					// Enables limit of Step Response if rotor amplitude exceeds limit
															// Useful for protecting operation if summing step and sine drive
#define STEP_RESPONSE_AMP_LIMIT 350							// Angle limit for Step Response action
/*
 * Setting ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE = 1 applies a Rotor Position tracking
 * command input impulse signal
 */
#define ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE 0			// If selected, disable all other modulation inputs
#define ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE 8		// Amplitude of impulse in degrees
#define ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD 500 		// Duration of impulse in cycles
#define ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL 10000	// Interval between impulse events in cycles
/* Define for High Speed System */
#define ROTOR_IMPULSE_SAMPLE_RATE 500							// Default sample rate
/*
 * Setting ENABLE_PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE = 1 applies a Pendulum Position tracking
 * command input impulse signal
 */
#define ENABLE_PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE 0		// If selected, disable all other modulation inputs
#define PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE 20	// Amplitude of step cycle. Note: Peak-to-Peak amplitude is double this value
#define PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD 1		// Duration of impulse in cycles
#define PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL 14000	// Interval between impulse events in cycles
/* Define for High Speed System */
#define PENDULUM_IMPULSE_SAMPLE_RATE 500 						// Default sample rate

/*
 * Motor Interface Data Structure
 *
 * Note that this is not required by default since Motor Profile is included
 * from l6474_target_config.h
 *
 * Note: This application is based on usage of l6474_target_config.h header
 * file for initialization of configuration of L6474
 */

L6474_Init_t gL6474InitParams = {
		MAX_ACCEL,           	/// Acceleration rate in step/s2. Range: (0..+inf).
		MAX_DECEL,           	/// Deceleration rate in step/s2. Range: (0..+inf).
		MAX_SPEED,              /// Maximum speed in step/s. Range: (30..10000].
		MIN_SPEED,              /// Minimum speed in step/s. Range: [30..10000).
		MAX_TORQUE_CURRENT, 	/// Torque regulation current in mA. (TVAL register) Range: 31.25mA to 4000mA.
		OVERCURRENT_THRESHOLD, 	/// Overcurrent threshold (OCD_TH register). Range: 375mA to 6000mA.
		L6474_CONFIG_OC_SD_ENABLE, /// Overcurrent shutwdown (OC_SD field of CONFIG register).
		L6474_CONFIG_EN_TQREG_TVAL_USED, /// Torque regulation method (EN_TQREG field of CONFIG register).
		L6474_STEP_SEL_1_16, 	/// Step selection (STEP_SEL field of STEP_MODE register).
		L6474_SYNC_SEL_1_2, 	/// Sync selection (SYNC_SEL field of STEP_MODE register).
		L6474_FAST_STEP_12us, 	/// Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
		L6474_TOFF_FAST_8us, 	/// Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
		3,   					/// Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
		21, 					/// Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
		L6474_CONFIG_TOFF_044us, /// Target Swicthing Period (field TOFF of CONFIG register).
		L6474_CONFIG_SR_320V_us, /// Slew rate (POW_SR field of CONFIG register).
		L6474_CONFIG_INT_16MHZ, /// Clock setting (OSC_CLK_SEL field of CONFIG register).
		(L6474_ALARM_EN_OVERCURRENT | L6474_ALARM_EN_THERMAL_SHUTDOWN
				| L6474_ALARM_EN_THERMAL_WARNING | L6474_ALARM_EN_UNDERVOLTAGE
				| L6474_ALARM_EN_SW_TURN_ON | L6474_ALARM_EN_WRONG_NPERF_CMD) /// Alarm (ALARM_EN register).
		};

/*
 * PID Controller Data Structure and functions
 */

/*
 * 		Data Structure of PID Controller with low pass filter operating
 * 		on derivative component
 */

typedef struct {
	float p_gain;
	float i_gain;
	float integrator_windup_limit;
	int warn;
	float d_gain;
	float previous_error;
	float previous_derivative;
	float differential;
	float differential_filter;
	float previous_differential_filter;
	float i_error;
	float p_term, i_term, d_term;
	float control_output;
} pid_filter_control_parameters;

/// PWM period variables used by step interrupt
volatile uint32_t desired_pwm_period = 0;
volatile uint32_t current_pwm_period = 0;

/*
 * Apply acceleration
 */
#define ACCEL_CONTROL 1 // Set to 1 to enable acceleration control. Set to 0 to use position target control.
#define PWM_COUNT_SAFETY_MARGIN 4
#define MAXIMUM_ACCELERATION 20000
#define MAXIMUM_DECELERATION 20000
#define MIN_POSSIBLE_SPEED 2 // Should be at least 2, as per L6474_MIN_PWM_FREQ in l6474.c
#define MAXIMUM_SPEED 10000
void apply_acceleration(int32_t acc, int32_t* target_velocity_prescaled, uint16_t sample_freq_hz) {
	uint32_t current_pwm_period_local = current_pwm_period;
	uint32_t desired_pwm_period_local = desired_pwm_period;

	motorDir_t old_dir = *target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

	if (old_dir == FORWARD) {
		if (acc > MAXIMUM_ACCELERATION) {
			acc = MAXIMUM_ACCELERATION;
		} else if (acc < -MAXIMUM_DECELERATION) {
			acc = -MAXIMUM_DECELERATION;
		}
	} else {
		if (acc < -MAXIMUM_ACCELERATION) {
			acc = -MAXIMUM_ACCELERATION;
		} else if (acc > MAXIMUM_DECELERATION) {
			acc = MAXIMUM_DECELERATION;
		}	
	}

	*target_velocity_prescaled += L6474_Board_Pwm1PrescaleFreq(acc) / sample_freq_hz;
	motorDir_t new_dir = *target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

	uint32_t speed_prescaled;
	if (new_dir == FORWARD) {
		if (*target_velocity_prescaled < L6474_Board_Pwm1PrescaleFreq(MIN_POSSIBLE_SPEED)) {
			*target_velocity_prescaled = L6474_Board_Pwm1PrescaleFreq(MIN_POSSIBLE_SPEED);
		} else if (*target_velocity_prescaled > L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED)) {
			*target_velocity_prescaled = L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED);
		}
		speed_prescaled = *target_velocity_prescaled;
	} else {
		if (*target_velocity_prescaled > -L6474_Board_Pwm1PrescaleFreq(MIN_POSSIBLE_SPEED)) {
			*target_velocity_prescaled = -L6474_Board_Pwm1PrescaleFreq(MIN_POSSIBLE_SPEED);
		} else if (*target_velocity_prescaled < -L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED)) {
			*target_velocity_prescaled = -L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED);
		}
		speed_prescaled = *target_velocity_prescaled * -1;
	}
	uint32_t effective_pwm_period = desired_pwm_period_local;
	desired_pwm_period_local = HAL_RCC_GetSysClockFreq() / speed_prescaled;

	if (old_dir != new_dir) {
		L6474_Board_SetDirectionGpio(0, new_dir);
	}

	if (current_pwm_period_local != 0) {
		uint32_t pwm_count = L6474_Board_Pwm1GetCounter();
		uint32_t pwm_time_left = current_pwm_period_local - pwm_count;
		if (pwm_time_left > PWM_COUNT_SAFETY_MARGIN) {
			uint32_t new_pwm_time_left = pwm_time_left * desired_pwm_period_local / effective_pwm_period;
			if (new_pwm_time_left != pwm_time_left) {
				if (new_pwm_time_left < PWM_COUNT_SAFETY_MARGIN) {
					new_pwm_time_left = PWM_COUNT_SAFETY_MARGIN;
				}
				current_pwm_period_local = pwm_count + new_pwm_time_left;
				L6474_Board_Pwm1SetPeriod(current_pwm_period_local);
				current_pwm_period = current_pwm_period_local;
			}
			// Test Code:
			// uint32_t updated_pwm_count = L6474_Board_Pwm1GetCounter();
			// if (updated_pwm_count - pwm_count >= PWM_COUNT_SAFETY_MARGIN) {
			// 	char msg[100];
			// 	if (updated_pwm_count < pwm_count) {
			// 		sprintf(msg, "PWM_COUNT_SAFETY_MARGIN is too small! The next pulse already started!\r\n");
			// 	} else {
			// 		sprintf(msg, "PWM_COUNT_SAFETY_MARGIN is too small! It should be at least %ld\r\n", updated_pwm_count - pwm_count + 1);
			// 	}
			// 	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			// }
		}
	} else {
		L6474_Board_Pwm1SetPeriod(desired_pwm_period_local);
		current_pwm_period = desired_pwm_period_local;
	}

	desired_pwm_period = desired_pwm_period_local;
}

/*
 * PWM pulse (step) interrupt
 */
void Main_StepClockHandler() {
	uint32_t desired_pwm_period_local = desired_pwm_period;
	if (desired_pwm_period_local != 0) {
		L6474_Board_Pwm1SetPeriod(desired_pwm_period_local);
		current_pwm_period = desired_pwm_period_local;
	}
}

/*
 * Rotor position set
 */

void rotor_position_set(void) {
	uint32_t rotor_position_u;
	rotor_position_u = BSP_MotorControl_GetPosition(0);
	BSP_MotorControl_SetHome(0, rotor_position_u);
}

/*
 * Rotor position read (returns signed integer)
 *
 * Returns error if overflow detected
 *
 */

int rotor_position_read(int *rotor_position) {
	uint32_t rotor_position_u;
	int range_error;
	rotor_position_u = BSP_MotorControl_GetPosition(0);

	if (rotor_position_u > 2147483648) {
		*rotor_position = (int) (rotor_position_u) - 4294967296;
	} else {
		*rotor_position = (int) (rotor_position_u);
	}
	range_error = 0;
	if (*rotor_position <= -2147483648) {
		range_error = -1;
		*rotor_position = -2147483648;
	}
	if (*rotor_position >= 2147483647) {
		range_error = 1;
		*rotor_position = 2147483647;
	}
	return range_error;
}

/*
 * Encoder position read (returns signed integer)
 *
 * Returns error if overflow detected
 */

int encoder_position_read(int *encoder_position, TIM_HandleTypeDef *htim3) {
	uint32_t cnt3;
	int range_error;
	cnt3 = __HAL_TIM_GET_COUNTER(htim3);

	if (cnt3 >= 32768) {
		*encoder_position = (int) (cnt3);
		*encoder_position = *encoder_position - 65536;
	} else {
		*encoder_position = (int) (cnt3);
	}

	range_error = 0;
	if (*encoder_position <= -32768) {
		range_error = -1;
		*encoder_position = -32768;
	}
	if (*encoder_position >= 32767) {
		range_error = 1;
		*encoder_position = 327677;
	}
	return range_error;
}

/*
 * PID Controller data structure initialization
 */

void pid_filter_value_config(pid_filter_control_parameters * pid_filter) {
	pid_filter->i_error = 0;
	pid_filter->previous_error = 0;
	pid_filter->previous_derivative = 0;
	pid_filter->differential = 0;
	pid_filter->previous_differential_filter = 0;
	pid_filter->differential_filter = 0;
}

/*
 * PID Controller with low pass filter operating on derivative component
 */
void pid_filter_control_execute(pid_filter_control_parameters * pid_filter,
		float * current_error, float * sample_period, float * f_deriv_lp) {
	float fo, Wo, IWon, iir_0, iir_1, iir_2;

	/*
	* f_deriv_lp pointer to variable containing low pass filter corner frequency
	*/
	fo = *f_deriv_lp;
	Wo = 2 * 3.141592654 * fo;
	IWon = 2 / (Wo * (*sample_period));
	iir_0 = 1 / (1 + IWon);
	iir_1 = iir_0;
	iir_2 = iir_0 * (1 - IWon);

	/*
	* Accumulate integrator error
	*/

	pid_filter->i_error += (*current_error * (*sample_period));

	/*
	* Limit integrator error
	*
	* Provide warn state in pid->warn
	*
	* Warn state may be applied in system characterization and operation
	*
	*/

	pid_filter->warn = 0;
	if (pid_filter->i_error < -(pid_filter->integrator_windup_limit)) {
		pid_filter->i_error = -(pid_filter->integrator_windup_limit);
		pid_filter->warn = -1;
	} else if (pid_filter->i_error > pid_filter->integrator_windup_limit) {
		pid_filter->i_error = pid_filter->integrator_windup_limit;
		pid_filter->warn = 1;
	}
	/*
	* Compute derivative, proportional, and integral terms
	*/

	/*
	* Introduce Low Pass Filter of derivative signal
	*/

	pid_filter->differential = ((*current_error - pid_filter->previous_error)
			/ (*sample_period));
	pid_filter->differential_filter = iir_0 * pid_filter->differential
			+ iir_1 * pid_filter->previous_derivative
			- iir_2 * pid_filter->previous_differential_filter;

	pid_filter->previous_derivative = pid_filter->differential;
	pid_filter->previous_differential_filter = pid_filter->differential_filter;
	pid_filter->previous_error = *current_error;

	pid_filter->d_term = (pid_filter->d_gain * pid_filter->differential_filter);
	pid_filter->p_term = (pid_filter->p_gain * (*current_error));
	pid_filter->i_term = (pid_filter->i_gain * pid_filter->i_error);
	pid_filter->control_output = pid_filter->p_term + pid_filter->i_term
			+ pid_filter->d_term;
}

void read_float(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, float *float_return) {

	int k;

	while (1) {
		*RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		*readBytes = Extract_Msg(RxBuffer, *RxBuffer_ReadIdx, *RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

		if (*readBytes)
		{
			*RxBuffer_ReadIdx = (*RxBuffer_ReadIdx + *readBytes)
					% UART_RX_BUFFER_SIZE;
			*float_return = atof((char*) Msg.Data);
		for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
			Msg.Data[k] = 0;
		}
		*readBytes = 0;
		break;
	}
		HAL_Delay(100);
	}
}

void read_int(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, int * int_return) {

	int k;

	while (1) {
		*RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		*readBytes = Extract_Msg(RxBuffer, *RxBuffer_ReadIdx, *RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

		if (*readBytes)
		{
			*RxBuffer_ReadIdx = (*RxBuffer_ReadIdx + *readBytes)
					% UART_RX_BUFFER_SIZE;

			*int_return = atoi((char*)(Msg.Data));
		for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
			Msg.Data[k] = 0;
		}
		*readBytes = 0;
		break;
	}
		HAL_Delay(100);
	}
}

void read_char(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, char * char_return) {

	int k;

	while (1) {
		*RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		*readBytes = Extract_Msg(RxBuffer, *RxBuffer_ReadIdx, *RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

		if (*readBytes)
		{
			*RxBuffer_ReadIdx = (*RxBuffer_ReadIdx + *readBytes)
					% UART_RX_BUFFER_SIZE;
			char_return = (char*)(Msg.Data);
		for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
			Msg.Data[k] = 0;
		}
		*readBytes = 0;
		break;
	}
		HAL_Delay(100);
	}
}

int main(void) {


	char msg[192];
	char test_msg[128];
	uint32_t tick, tick_cycle_current, tick_cycle_previous, tick_cycle_start,
			tick_read_cycle, tick_read_cycle_start,tick_wait_start,tick_wait;
	uint16_t min_speed, max_speed, max_accel, max_decel;

	float Tsample, Tsample_rotor, test_time;
	float angle_scale = ENCODER_READ_ANGLE_SCALE;

	int enable_high_speed_sampling;

	uint32_t RxBuffer_ReadIdx = 0;
	uint32_t RxBuffer_WriteIdx = 0;
	uint32_t readBytes;

	int rotor_position_target = 0;
	int rotor_position_target_curr = 0;
	int rotor_position_target_prev = 0;

	int rotor_position_delta;
	int cycle_count;
	int i, j, k, m;
	int ret;
	pid_filter_control_parameters *pid_filter;
	pid_filter_control_parameters *rotor_pid;
	float windup, rotor_windup;
	float *current_error, *current_error_rotor;
	float *sample_period, *sample_period_rotor;

	int cycle_period_start;
	int cycle_period_sum;

	float *deriv_lp_corner_f;
	float *deriv_lp_corner_f_rotor;
	float proportional, rotor_p_gain;
	float integral, rotor_i_gain;
	float derivative, rotor_d_gain;
	int rotor_position;
	float rotor_position_command;
	int encoder_position = 0, encoder_position_down = 0;
	int encoder_position_curr = 0, encoder_position_prev = 0;

	float fo, Wo, IWon, iir_0, iir_1, iir_2;
	float fo_LT, Wo_LT, IWon_LT;
	float iir_LT_0, iir_LT_1, iir_LT_2;


	float rotor_position_prev, rotor_position_filter, rotor_position_filter_prev;
	float rotor_position_diff, rotor_position_diff_prev;
	float rotor_position_diff_filter, rotor_position_diff_filter_prev;

	int rotor_target_in;
	int slope;
	int slope_prev;

	float adaptive_error, adaptive_threshold_low, adaptive_threshold_high;
	float error_sum_prev, error_sum, error_sum_filter_prev, error_sum_filter;
	int adaptive_entry_tick, adaptive_dwell_period;
	int enable_adaptive_mode, adaptive_state, adaptive_state_change;
	float rotor_position_command_prev;

	float encoder_angle_slope_corr;

	int rotor_position_step_polarity;
	float impulse_start_index;

	uint32_t enable_pid;

	char *message_received;
	int mode_index = 1;
	int report_mode = 0;

	char *mode_string_stop;
	char *mode_string_mode_1;
	char *mode_string_mode_2;
	char *mode_string_mode_3;
	char *mode_string_mode_4;
	char *mode_string_mode_8;
	char *mode_string_mode_5;
	char *mode_string_inc_accel;
	char *mode_string_dec_accel;
	char *mode_string_inc_amp;
	char *mode_string_dec_amp;
	char *mode_string_mode_single_pid;
	char *mode_string_mode_test;
	char *mode_string_mode_control;
	char *mode_string_mode_high_speed_test;
	char *mode_string_mode_motor_characterization_mode;
	char *mode_string_mode_pendulum_sysid_test;
	char *mode_string_mode_load_dist;
	char *mode_string_mode_load_dist_step;
	char *mode_string_mode_noise_dist_step;
	char *mode_string_mode_plant_dist_step;
	char *mode_string_dec_pend_p;
	char *mode_string_inc_pend_p;
	char *mode_string_dec_pend_i;
	char *mode_string_inc_pend_i;
	char *mode_string_dec_pend_d;
	char *mode_string_inc_pend_d;
	char *mode_string_dec_rotor_p;
	char *mode_string_inc_rotor_p;
	char *mode_string_dec_rotor_i;
	char *mode_string_inc_rotor_i;
	char *mode_string_dec_rotor_d;
	char *mode_string_inc_rotor_d;
	char *mode_string_dec_torq_c;
	char *mode_string_inc_torq_c;
	char *mode_string_dec_max_s;
	char *mode_string_inc_max_s;
	char *mode_string_dec_min_s;
	char *mode_string_inc_min_s;
	char *mode_string_dec_max_a;
	char *mode_string_inc_max_a;
	char *mode_string_dec_max_d;
	char *mode_string_inc_max_d;
	char *mode_string_enable_step;
	char *mode_string_disable_step;
	char *mode_string_enable_load_dist;
	char *mode_string_disable_load_dist;
	char *mode_string_enable_noise_rej_step;
	char *mode_string_disable_noise_rej_step;
	char *mode_string_disable_sensitivity_fnc_step;
	char *mode_string_enable_sensitivity_fnc_step;
	char *mode_string_inc_step_size;
	char *mode_string_dec_step_size;
	char *mode_string_select_mode_5;
	char *mode_string_enable_high_speed_sampling;
	char *mode_string_disable_high_speed_sampling;

	int char_mode_select = 0;	// Flag detecting whether character mode select entered
	int mode_1 = 1;				// Enable LQR Motor Model M
	int mode_2 = 2;				// Enable LRR Motor Model H
	int mode_3 = 3;				// Enable LQR Motor Model L
	int mode_4 = 4;				// Enable Suspended Mode Motor Model M
	int mode_5 = 5;				// Enable sin drive track signal
	int mode_adaptive_off = 6;	// Disable adaptive control
	int mode_adaptive = 7;		// Enable adaptive control
	int mode_8 = 8;				// Enable custom configuration entry
	int mode_9 = 9;				// Disable sin drive track signal
	int mode_10 = 10;			// Enable Single PID Mode with Motor Model M
	int mode_11 = 11;			// Enable rotor actuator and encoder test mode
	int mode_12 = 12;			// Enable rotor actuator and encoder high speed test mode
	int mode_13 = 13;			// Enable rotor control system evaluation
	int mode_14 = 14;			// Enable pendlum system identification
	int mode_15 = 15;			// Enable interactive control of rotor actuator
	int mode_16 = 16;			// Enable load disturbance sensitivity function mode
	int mode_17 = 17;			// Enable load disturbance step function mode
	int mode_18 = 18;			// Enable noise disturbance step function mode
	int mode_19 = 19;			// Enable plant disturbance step function mode
	int mode_quit = 0;			// Initiate exit from control loop
	int mode_interactive;		// Enable continued terminal interactive user session
	int mode_index_prev, mode_index_command;
	int mode_transition_tick, mode_transition_state = 0, transition_to_adaptive_mode = 0;

	int max_speed_read, min_speed_read;

	int select_suspended_mode;
	int motor_response_model;

	int enable_rotor_actuator_test, enable_rotor_actuator_control;
	int enable_encoder_test;
	int enable_rotor_actuator_high_speed_test;
	int enable_motor_actuator_characterization_mode;
	int motor_state;
	float torq_current_val;


	int enable_rotor_chirp = 0;
	int chirp_cycle;
	int chirp_dwell_cycle;
	float chirp_time;
	float rotor_chirp_start_freq = ROTOR_CHIRP_START_FREQ;
	float rotor_chirp_end_freq = ROTOR_CHIRP_END_FREQ;
	float rotor_chirp_period = ROTOR_CHIRP_PERIOD;
	float rotor_chirp_frequency;
	float rotor_chirp_amplitude;
	int rotor_chirp_step_period;

	float pendulum_position_command;

	int enable_mod_sin_rotor_tracking = ENABLE_MOD_SIN_ROTOR_TRACKING;
	int enable_rotor_position_step_response_cycle = ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE;
	int disable_mod_sin_rotor_tracking = 0;
	int sine_drive_transition = 0;
	float mod_sin_amplitude = MOD_SIN_AMPLITUDE;
	float rotor_control_sin_amplitude = MOD_SIN_AMPLITUDE;
	float rotor_sine_drive, rotor_sine_drive_mod;
	float rotor_mod_control;
	float mod_sin_carrier_frequency;

	int enable_pendulum_position_impulse_response_cycle = 0;

	int swing_cycles, rotor_test_speed_min, rotor_test_speed_max;
	int rotor_test_acceleration_max, swing_deceleration_max;
	int start_angle_a[20], end_angle_a[20], motion_dwell_a[20];
	int abs_encoder_position_prior, abs_encoder_position_after, abs_encoder_position_max;
	uint16_t current_speed;

	int enable_pendulum_sysid_test;

	int enable_rotor_tracking_comb_signal;
	float rotor_track_comb_signal_frequency;
	float rotor_track_comb_command;
	float rotor_track_comb_amplitude;

	int enable_disturbance_rejection_step = 0;
	int enable_noise_rejection_step = 0;
	int enable_plant_rejection_step = 0;
	int enable_sensitivity_fnc_step = 0;


	float noise_rej_signal_filter, noise_rej_signal;
	float noise_rej_signal_prev, noise_rej_signal_filter_prev;

	char config_message[16];
	int config_command;
	int display_parameter;
	int step_size = 0;
	float adjust_increment = 0.5;



	/*
	* Default select_suspended_mode
	*/

	select_suspended_mode = ENABLE_SUSPENDED_PENDULUM_CONTROL;

	/* STM32xx HAL library initialization */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	//----- Initialize Motor Control Library
	/* Set the L6474 library to use 1 device */
	BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);
	/* When BSP_MotorControl_Init is called with NULL pointer,                  */
	/* the L6474 registers and parameters are set with the predefined values from file   */
	/* l6474_target_config.h, otherwise the registers are set using the   */
	/* L6474_Init_t pointer structure                */
	/* The first call to BSP_MotorControl_Init initializes the first device     */
	/* whose Id is 0.                                                           */
	/* The nth call to BSP_MotorControl_Init initializes the nth device         */
	/* whose Id is n-1.                                                         */
	/* Uncomment the call to BSP_MotorControl_Init below to initialize the      */
	/* device with the structure gL6474InitParams declared in the the main.c file */
	/* and comment the subsequent call having the NULL pointer                   */
	// BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, NULL);

	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);

	/*
	 * Initialize Timer and UART
	 */

	MX_TIM3_Init();
	MX_USART2_UART_Init();

	/*
	 * Motor Range Initialization
	 */

	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, MAX_SPEED_UPPER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, MIN_SPEED_UPPER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, MAX_SPEED_LOWER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, MIN_SPEED_LOWER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetAcceleration(0, MAX_ACCEL_UPPER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetDeceleration(0, MAX_DECEL_UPPER_INIT);
	HAL_Delay(1);

	/*
	* Default Starting Control Configuration
	*/

	max_accel = MAX_ACCEL;
	max_decel = MAX_DECEL;
	max_speed = MAX_SPEED_MODE_1;
	min_speed = MIN_SPEED_MODE_1;

	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, max_speed);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, min_speed);
	HAL_Delay(1);
	BSP_MotorControl_SetAcceleration(0, max_accel);
	HAL_Delay(1);
	BSP_MotorControl_SetDeceleration(0, max_decel);
	HAL_Delay(1);

	torq_current_val = 800;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);


	proportional = PRIMARY_PROPORTIONAL_MODE_1;
	integral = PRIMARY_INTEGRAL_MODE_1;
	derivative = PRIMARY_DERIVATIVE_MODE_1;

	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;


	/*
	* Disable adaptive_mode by default
	*/

	enable_adaptive_mode = 0;

	/*
	* Allocate user interactive command strings
	*/

	message_received = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (message_received == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_1 = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_1 == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}
	mode_string_mode_2 = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_2 == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_3 = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_3 == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_4 = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_4 == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_5 = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_5 == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_8 = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_8 == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_single_pid = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_single_pid == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_test = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_test == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_control = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_control == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_high_speed_test = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_high_speed_test == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_motor_characterization_mode = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_motor_characterization_mode == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_pendulum_sysid_test = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_pendulum_sysid_test == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_dec_accel = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_dec_accel == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_inc_accel = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_inc_accel == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_inc_amp = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_inc_amp == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_dec_amp = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_dec_amp == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_load_dist = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_load_dist == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_load_dist_step = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_load_dist_step == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_noise_dist_step = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_noise_dist_step == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_mode_plant_dist_step = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_mode_plant_dist_step == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	mode_string_stop = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	if (mode_string_stop == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}
	mode_string_dec_pend_p = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_inc_pend_p = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_dec_pend_i = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_inc_pend_i = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_dec_pend_d = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_inc_pend_d = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_dec_rotor_p = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_inc_rotor_p = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_dec_rotor_i = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_inc_rotor_i = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_dec_rotor_d = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_inc_rotor_d = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_dec_torq_c = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_inc_torq_c = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_dec_max_s = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_inc_max_s = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_dec_min_s = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_inc_min_s = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_dec_max_a = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_inc_max_a = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_dec_max_d = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_inc_max_d = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_enable_step = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_disable_step = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_enable_load_dist = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_disable_load_dist = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_enable_noise_rej_step = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_disable_noise_rej_step = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_enable_sensitivity_fnc_step = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_disable_sensitivity_fnc_step = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_inc_step_size = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_dec_step_size = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_select_mode_5 = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_enable_high_speed_sampling = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));
	mode_string_disable_high_speed_sampling = (char *)malloc(UART_RX_BUFFER_SIZE * sizeof(char));

	/*
	 * Set user interactive command string values
	 */

	sprintf(mode_string_mode_1, "1");
	sprintf(mode_string_mode_2, "2");
	sprintf(mode_string_mode_3, "3");
	sprintf(mode_string_mode_4, "4");
	sprintf(mode_string_mode_5, "m");
	sprintf(mode_string_mode_8, "g");
	sprintf(mode_string_mode_single_pid, "s");
	sprintf(mode_string_mode_test, "t");
	sprintf(mode_string_mode_control, "r");
	sprintf(mode_string_mode_high_speed_test, "z");
	sprintf(mode_string_mode_motor_characterization_mode, "c");
	sprintf(mode_string_mode_pendulum_sysid_test, "p");
	sprintf(mode_string_dec_accel, "d");
	sprintf(mode_string_inc_accel, "i");
	sprintf(mode_string_inc_amp, "j");
	sprintf(mode_string_dec_amp, "k");
	sprintf(mode_string_stop, "q");


	sprintf(mode_string_dec_pend_p, "a");
	sprintf(mode_string_inc_pend_p, "A");
	sprintf(mode_string_dec_pend_i, "b");
	sprintf(mode_string_inc_pend_i, "B");
	sprintf(mode_string_dec_pend_d, "c");
	sprintf(mode_string_inc_pend_d, "C");
	sprintf(mode_string_dec_rotor_p, "d");
	sprintf(mode_string_inc_rotor_p, "D");
	sprintf(mode_string_dec_rotor_i, "e");
	sprintf(mode_string_inc_rotor_i, "E");
	sprintf(mode_string_dec_rotor_d, "f");
	sprintf(mode_string_inc_rotor_d, "F");
	sprintf(mode_string_dec_torq_c, "t");
	sprintf(mode_string_inc_torq_c, "T");
	sprintf(mode_string_dec_max_s, "s");
	sprintf(mode_string_inc_max_s, "S");
	sprintf(mode_string_dec_min_s, "m");
	sprintf(mode_string_inc_min_s, "M");
	sprintf(mode_string_dec_max_a, "n");
	sprintf(mode_string_inc_max_a, "N");
	sprintf(mode_string_dec_max_d, "o");
	sprintf(mode_string_inc_max_d, "O");
	sprintf(mode_string_enable_step, "P");
	sprintf(mode_string_disable_step, "p");
	sprintf(mode_string_enable_load_dist, "L");
	sprintf(mode_string_disable_load_dist, "l");
	sprintf(mode_string_enable_noise_rej_step, "R");
	sprintf(mode_string_disable_noise_rej_step, "r");
	sprintf(mode_string_enable_sensitivity_fnc_step, "V");
	sprintf(mode_string_disable_sensitivity_fnc_step, "v");
	sprintf(mode_string_inc_step_size, "J");
	sprintf(mode_string_dec_step_size, "j");
	sprintf(mode_string_select_mode_5, "u");
	sprintf(mode_string_enable_high_speed_sampling, "Y");
	sprintf(mode_string_disable_high_speed_sampling, "y");


	/*
	* DMA Buffer declarations
	*/


	/* Start DMA just once because it's configured in "circular" mode */
	HAL_UART_Receive_DMA(&huart2, RxBuffer, UART_RX_BUFFER_SIZE);

	/*
	* Motor Interface and Encoder initialization
	*/

	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

	/* Attach the function Error_Handler (defined below) to the error Handler*/
	BSP_MotorControl_AttachErrorHandler(Error_Handler);

	/* Encoder init */
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	/*
	* Controller structure and variable allocation
	*/

	current_error = malloc(sizeof(float));
	if (current_error == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	current_error_rotor = malloc(sizeof(float));
	if (current_error_rotor == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	sample_period = malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	deriv_lp_corner_f = malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	deriv_lp_corner_f_rotor = malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	sample_period_rotor = malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	pid_filter = malloc(sizeof(pid_filter_control_parameters));
	if (pid_filter == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	rotor_pid = malloc(sizeof(pid_filter_control_parameters));
	if (rotor_pid == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	/*
	* Initialize Pendulum and Rotor PID Controller structures
	*/

	pid_filter_value_config(pid_filter);
	pid_filter_value_config(rotor_pid);

	/*
	*      Configure primary controller parameters
	*
	*      Note that proportional, integral, and derivative gains are set at runtime.
	*
	*      If these are to be set at compile time, then comments may be removed below along with other
	*      required code modification
	*/

	windup = PRIMARY_WINDUP_LIMIT;

	/*
	*      Configure secondary Rotor controller parameters
	*/

	rotor_windup = SECONDARY_WINDUP_LIMIT;

	/*
	*      Configure controller filter and sample time parameters
	*/

	*deriv_lp_corner_f = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
	*deriv_lp_corner_f_rotor = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	Tsample = T_SAMPLE;
	*sample_period = Tsample;
	Tsample_rotor = T_SAMPLE_ROTOR;
	*sample_period_rotor = Tsample_rotor;

	/*
	* Compute Low Pass Filter Coefficients for Rotor Position filter and Encoder Angle Slope Correction
	*/

	fo = LP_CORNER_FREQ_ROTOR;
	Wo = 2 * 3.141592654 * fo;
	IWon = 2 / (Wo * Tsample);

	iir_0 = 1 / (1 + IWon);
	iir_1 = iir_0;
	iir_2 = iir_0 * (1 - IWon);

	fo_LT = LP_CORNER_FREQ_LONG_TERM;
	Wo_LT = 2 * 3.141592654 * fo_LT;
	IWon_LT = 2 / (Wo_LT * Tsample);

	iir_LT_0 = 1 / (1 + IWon_LT);
	iir_LT_1 = iir_LT_0;
	iir_LT_2 = iir_LT_0 * (1 - IWon_LT);


	/*
	* Primary Controller Mode Configuration Loop
	*
	* Outer Loop
	* 	Acquires configuration command
	* 	Initiated control
	* Inner Loop
	* 	Control Loop
	* 	Exits to Outer Loop upon command reception
	*
	*/

	/*
	* Wait for initial start character from terminal routine
	* return value will be discarded.
	*/

	tick_read_cycle_start = HAL_GetTick();

	sprintf(msg, "\n\rSystem Starting Prepare to Enter Mode Selection... ");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);




	/*
	* Request user input for mode configuration
	*/

	enable_adaptive_mode = ENABLE_ADAPTIVE_MODE;
	adaptive_threshold_low = ADAPTIVE_THRESHOLD_LOW;
	adaptive_threshold_high = ADAPTIVE_THRESHOLD_HIGH;
	adaptive_state = ADAPTIVE_STATE;
	adaptive_state_change = 0;
	adaptive_dwell_period = ADAPTIVE_DWELL_PERIOD;


	while (1) {

	mode_interactive = 0;

	sprintf(msg, "********  System Start Mode Selections  ********\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 1 at prompt for Inverted Pendulum Control with Motor Speed Profile - Medium\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 2 at prompt for Inverted Pendulum Control with Motor Speed Profile - High\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 3 at prompt for Inverted Pendulum Control with Motor Speed Profile - Low\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 4 at prompt for Suspended Pendulum Control with Motor Speed Profile - Medium\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 's' at prompt for Single PID: With Prompts for Pendulum Controller Gains\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 'g' at prompt for General Mode: With Prompts for Both Pendulum and Rotor Controller Gains\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 't' at prompt for Test Mode: Test of Rotor Actuator and Pendulum Angle Encoder\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 'r' at prompt for Rotor Control Mode: Direct Control of Rotor Actuator\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 'c' at prompt for Motor Control Characterization Mode\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
//	sprintf(msg, "Enter 'z' at prompt for High Speed Mode: Test of Rotor Actuator and Pendulum Angle Encoder\n\r");
//	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
//	sprintf(msg, "Enter 'l' at prompt for Load Disturbance Sensitivity Function Mode\n\r");
//	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
//	sprintf(msg, "Enter 'L' at prompt for Load Disturbance Step Mode\n\r");
//	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
//	sprintf(msg, "Enter 'N' at prompt for Noise Disturbance Step Mode\n\r");
//	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
//	sprintf(msg, "Enter 'M' at prompt for Plant Disturbance Step Mode\n\r");
//	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	/*
	 * If user has responded to query for configuration, then system remains in interactive mode
	 * and default state is not automatically enabled
	 */

	if (mode_interactive == 0) {
		sprintf(msg, "\n\rEnter Mode Selection Now or System Will Start in Default Mode in 5 Seconds: ");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}

	/*
	 * If user has responded to query for configuration, then system remains in interactive mode
	 * and default state is not automatically enabled
	 */

	if (mode_interactive == 1) {
		sprintf(msg, "\n\rEnter Mode Selection Now: \n\r");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}

	tick_read_cycle_start = HAL_GetTick();
	for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
		Msg.Data[k] = 0;
	}

	enable_rotor_actuator_test = 0;
	enable_rotor_actuator_control = 0;
	enable_encoder_test = 0;
	enable_rotor_actuator_high_speed_test = 0;
	enable_motor_actuator_characterization_mode = 0;
	enable_rotor_tracking_comb_signal = 0;
	rotor_track_comb_amplitude = 0;
	enable_disturbance_rejection_step = 0;
	enable_noise_rejection_step = 0;
	enable_plant_rejection_step = 0;

		/*
		* Configuration command read loop
		*/

		while (1) {

			RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE
					- __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

			readBytes = Extract_Msg(RxBuffer, RxBuffer_ReadIdx,
					RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);


			/*
			* Exit read loop after timeout selecting default Mode 1
			*/
			tick_read_cycle = HAL_GetTick();
			if (((tick_read_cycle - tick_read_cycle_start) > START_DEFAULT_MODE_TIME) && (mode_interactive == 0)) {
				sprintf(msg, "\n\rNo Entry Detected - Now Selecting Default Inverted Pendulum Mode 1: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				select_suspended_mode = 0;
				proportional = 		PRIMARY_PROPORTIONAL_MODE_1;
				integral = 			PRIMARY_INTEGRAL_MODE_1;
				derivative = 		PRIMARY_DERIVATIVE_MODE_1;
				rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_1;
				rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_1;
				rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_1;
				max_speed = 		MAX_SPEED_MODE_1;
				min_speed = 		MIN_SPEED_MODE_1;
				enable_mod_sin_rotor_tracking = ENABLE_MOD_SIN_ROTOR_TRACKING;
				enable_rotor_position_step_response_cycle = 1;
				break;
			}

			if (readBytes) // Message found
			{
				RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;

				if (Msg.Len != 1) {
					continue;
				}

				sprintf(msg, "%s\n\r", (char *)Msg.Data);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				char_mode_select = 0;

				if (strcmp((char *)Msg.Data,mode_string_mode_single_pid)==0){
					mode_index = 10;
					char_mode_select = 1;
				}

				if (strcmp((char *)Msg.Data,mode_string_mode_test)==0){
					mode_index = 11;
					char_mode_select = 1;
					sprintf(msg, "\n\rTest Mode Selected ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}

				if (strcmp((char *)Msg.Data,mode_string_mode_control)==0){
					mode_index = 15;
					char_mode_select = 1;
					sprintf(msg, "\n\rRotor Actuator Control Mode Selected ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}

				if (strcmp((char *)Msg.Data,mode_string_mode_high_speed_test)==0){
					mode_index = 12;
					char_mode_select = 1;
				}

				if (strcmp((char *)Msg.Data,mode_string_mode_motor_characterization_mode)==0){
					mode_index = 13;
					char_mode_select = 1;
				}

				if (strcmp((char *)Msg.Data,mode_string_mode_pendulum_sysid_test)==0){
					mode_index = 14;
					char_mode_select = 1;
				}

				if (strcmp((char *)Msg.Data,mode_string_mode_load_dist)==0){
					mode_index = 16;
					char_mode_select = 1;
				}

				if (strcmp((char *)Msg.Data,mode_string_mode_load_dist_step)==0){
					mode_index = 17;
					char_mode_select = 1;
				}

				if (strcmp((char *)Msg.Data,mode_string_mode_noise_dist_step)==0){
					mode_index = 18;
					char_mode_select = 1;
				}

				if (strcmp((char *)Msg.Data,mode_string_mode_plant_dist_step)==0){
					mode_index = 18;
					char_mode_select = 1;
				}

				if (strcmp((char *)Msg.Data,mode_string_mode_8)==0){
					mode_index = 8;
					char_mode_select = 1;
				}

				if(char_mode_select == 0){
					mode_index = atoi((char *)Msg.Data);
				}


				/*
				* Set mode index according to user input
				*/

				switch (mode_index) {

				case 1:
					mode_index = mode_1;
					break;

				case 2:
					mode_index = mode_2;
					break;

				case 3:
					mode_index = mode_3;
					break;

				case 4:
					mode_index = mode_4;
					break;

				case 7:
					mode_index = mode_adaptive;
					break;

				case 8:
					mode_index = mode_8;
					mode_interactive = 1;
					break;

				case 10:
					mode_index = mode_10;
					mode_interactive = 1;
					break;

				case 11:
					mode_index = mode_11;
					mode_interactive = 1;
					break;

				case 12:
					mode_index = mode_12;
					mode_interactive = 1;
					break;

				case 13:
					mode_index = mode_13;
					mode_interactive = 1;
					break;

				case 14:
					mode_index = mode_14;
					mode_interactive = 1;
					break;

				case 15:
					mode_index = mode_15;
					mode_interactive = 1;
					break;

				case 16:
					mode_index = mode_16;
					mode_interactive = 1;
					break;

				case 17:
					mode_index = mode_17;
					mode_interactive = 1;
					break;

				case 18:
					mode_index = mode_18;
					mode_interactive = 1;
					break;

				case 19:
					mode_index = mode_19;
					mode_interactive = 1;
					break;

				default:
					mode_index = mode_1;
					break;
				}


				/*
				* Configure Motor Speed Profile and PID Controller Gains
				*/

				enable_rotor_position_step_response_cycle = 0;
				enable_mod_sin_rotor_tracking = 0;
				enable_rotor_chirp = 0;
				enable_pendulum_position_impulse_response_cycle = 0;
				enable_pendulum_sysid_test = 0;
				enable_rotor_tracking_comb_signal = 0;
				enable_disturbance_rejection_step = 0;
				enable_noise_rejection_step = 0;


				switch (mode_index) {

				case 1:
					select_suspended_mode = 0;
					proportional = 		PRIMARY_PROPORTIONAL_MODE_1;
					integral = 			PRIMARY_INTEGRAL_MODE_1;
					derivative = 		PRIMARY_DERIVATIVE_MODE_1;
					rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_1;
					rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_1;
					rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_1;
					max_speed = 		MAX_SPEED_MODE_1;
					min_speed = 		MIN_SPEED_MODE_1;

					enable_mod_sin_rotor_tracking = 0;
					enable_rotor_position_step_response_cycle = 0;
					enable_pendulum_position_impulse_response_cycle = 0;
					sprintf(msg, "\n\rMode %i Configured", mode_index);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg), HAL_MAX_DELAY);
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
					sprintf(msg, "%i", enable_rotor_chirp);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}

					if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
							&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					if (enable_rotor_tracking_comb_signal == 1){
					rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
					}
					}

					if (enable_rotor_position_step_response_cycle == 1) {
						sprintf(msg, "\n\rRotor Step Drive enabled ");
						enable_pendulum_position_impulse_response_cycle = 0;
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}

					if (enable_mod_sin_rotor_tracking == 1) {
						sprintf(msg, "\n\rRotor Sine Drive enabled ");
						enable_pendulum_position_impulse_response_cycle = 0;
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}

					if (enable_rotor_chirp == 1) {
						enable_rotor_position_step_response_cycle = 0;
						enable_pendulum_position_impulse_response_cycle = 0;
						enable_mod_sin_rotor_tracking = 0;
						sprintf(msg, "\n\rRotor Chirp Drive enabled ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}
					break;

				case 2:
					select_suspended_mode = 0;
					proportional = 		PRIMARY_PROPORTIONAL_MODE_2;
					integral = 			PRIMARY_INTEGRAL_MODE_2;
					derivative = 		PRIMARY_DERIVATIVE_MODE_2;
					rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_2;
					rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_2;
					rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_2;
					max_speed = 		MAX_SPEED_MODE_2;
					min_speed = 		MIN_SPEED_MODE_2;
					enable_mod_sin_rotor_tracking = 0;
					enable_rotor_position_step_response_cycle = 0;
					sprintf(msg, "\n\rMode %i Configured", mode_index);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg), HAL_MAX_DELAY);
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
					sprintf(msg, "%i", enable_rotor_chirp);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}

					if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
							&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					if (enable_rotor_tracking_comb_signal == 1){
					rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
					}
					}

					if (enable_rotor_position_step_response_cycle == 1) {
						sprintf(msg, "\n\rRotor Step Drive enabled ");
						enable_pendulum_position_impulse_response_cycle = 0;
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}

					if (enable_mod_sin_rotor_tracking == 1) {
						sprintf(msg, "\n\rRotor Sine Drive enabled ");
						enable_pendulum_position_impulse_response_cycle = 0;
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}

					if (enable_rotor_chirp == 1) {
						enable_rotor_position_step_response_cycle = 0;
						enable_mod_sin_rotor_tracking = 0;
						enable_pendulum_position_impulse_response_cycle = 0;
						sprintf(msg, "\n\rRotor Chirp Drive enabled ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}
					break;

				case 3:
					select_suspended_mode = 0;
					proportional = 		PRIMARY_PROPORTIONAL_MODE_3;
					integral = 			PRIMARY_INTEGRAL_MODE_3;
					derivative = 		PRIMARY_DERIVATIVE_MODE_3;
					rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_3;
					rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_3;
					rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_3;
					max_speed = 		MAX_SPEED_MODE_3;
					min_speed = 		MIN_SPEED_MODE_3;
					enable_mod_sin_rotor_tracking = 0;
					enable_rotor_position_step_response_cycle = 0;
					sprintf(msg, "\n\rMode %i Configured", mode_index);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg), HAL_MAX_DELAY);
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
					sprintf(msg, "%i", enable_rotor_chirp);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}

					if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
							&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					if (enable_rotor_tracking_comb_signal == 1){
					rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
					}
					}

					if (enable_rotor_position_step_response_cycle == 1) {
						sprintf(msg, "\n\rRotor Step Drive enabled ");
						enable_pendulum_position_impulse_response_cycle = 0;
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}

					if (enable_mod_sin_rotor_tracking == 1) {
						sprintf(msg, "\n\rRotor Sine Drive enabled ");
						enable_pendulum_position_impulse_response_cycle = 0;
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}

					if (enable_rotor_chirp == 1) {
						enable_rotor_position_step_response_cycle = 0;
						enable_mod_sin_rotor_tracking = 0;
						enable_pendulum_position_impulse_response_cycle = 0;
						sprintf(msg, "\n\rRotor Chirp Drive enabled ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}
					break;

				case 4:
					select_suspended_mode = 1;
					proportional = 		PRIMARY_PROPORTIONAL_MODE_4;
					integral = 			PRIMARY_INTEGRAL_MODE_4;
					derivative = 		PRIMARY_DERIVATIVE_MODE_4;
					rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_4;
					rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_4;
					rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_4;
					max_speed = 		MAX_SPEED_MODE_1;
					min_speed = 		MIN_SPEED_MODE_1;
					enable_mod_sin_rotor_tracking = 0;
					enable_rotor_position_step_response_cycle = 0;
					sprintf(msg, "\n\rMode %i Configured", mode_index);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg), HAL_MAX_DELAY);
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
					sprintf(msg, "%i", enable_rotor_chirp);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}

					if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
							&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					if (enable_rotor_tracking_comb_signal == 1){
					rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
					}
					}

					if (enable_rotor_position_step_response_cycle == 1) {
						sprintf(msg, "\n\rRotor Step Drive enabled ");
						enable_pendulum_position_impulse_response_cycle = 0;
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}

					if (enable_mod_sin_rotor_tracking == 1) {
						sprintf(msg, "\n\rRotor Sine Drive enabled ");
						enable_pendulum_position_impulse_response_cycle = 0;
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}

					if (enable_rotor_chirp == 1) {
						enable_rotor_position_step_response_cycle = 0;
						enable_mod_sin_rotor_tracking = 0;
						enable_pendulum_position_impulse_response_cycle = 0;
						sprintf(msg, "\n\rRotor Chirp Drive enabled ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					}
					break;


				case 7:
					enable_adaptive_mode = 1;
					select_suspended_mode = 0;
					proportional = 		PRIMARY_PROPORTIONAL_MODE_2;
					integral = 			PRIMARY_INTEGRAL_MODE_2;
					derivative = 		PRIMARY_DERIVATIVE_MODE_2;
					rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_2;
					rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_2;
					rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_2;
					max_speed = 		MAX_SPEED_MODE_2;
					min_speed = 		MIN_SPEED_MODE_2;
					enable_mod_sin_rotor_tracking = ENABLE_MOD_SIN_ROTOR_TRACKING;
					enable_rotor_position_step_response_cycle = ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE;
					sprintf(msg, "\n\rMode %i Configured", mode_index);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg), HAL_MAX_DELAY);
					break;

				case 8:

						sprintf(msg, "\n\rEnter Pendulum PID Proportional Gain: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg),
								HAL_MAX_DELAY);

						read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &proportional);
						sprintf(msg, "%0.2f", proportional);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						sprintf(msg, "\n\rEnter Pendulum PID Integral Gain: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg),
								HAL_MAX_DELAY);
						read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &integral);
						sprintf(msg, "%0.2f", integral);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						sprintf(msg, "\n\rEnter Pendulum PID Differential Gain: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg),
								HAL_MAX_DELAY);
						read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &derivative);
						sprintf(msg, "%0.2f", derivative);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						sprintf(msg, "\n\rEnter Rotor PID Proportional Gain: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg),
								HAL_MAX_DELAY);

						read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_p_gain);
						sprintf(msg, "%0.2f", rotor_p_gain);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						sprintf(msg, "\n\rEnter Rotor PID Integral Gain: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg),
								HAL_MAX_DELAY);
						read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_i_gain);
						sprintf(msg, "%0.2f", rotor_i_gain);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						sprintf(msg, "\n\rEnter Rotor PID Differential Gain: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_d_gain);
						sprintf(msg, "%0.2f", rotor_d_gain);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						sprintf(msg, "\n\rEnter 1 for Adaptive Mode - Enter 0 for Normal Mode: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_adaptive_mode);
						sprintf(msg, "%i", enable_adaptive_mode);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						if (enable_adaptive_mode == 1){
							sprintf(msg, "\n\rEnter Adaptive Low Threshold (30): ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &adaptive_threshold_low);
							sprintf(msg, "%0.2f", adaptive_threshold_low);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

							sprintf(msg, "\n\rEnter Adaptive High Threshold (2): ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &adaptive_threshold_high);
							sprintf(msg, "%0.2f", adaptive_threshold_high);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}

						select_suspended_mode = 0;

						sprintf(msg, "\n\rEnter 0 for Inverted Mode - Enter 1 for Suspended Mode: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &select_suspended_mode);
						sprintf(msg, "%i", select_suspended_mode);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						enable_rotor_position_step_response_cycle = 0;
						enable_mod_sin_rotor_tracking = 0;
						enable_rotor_chirp = 0;
						enable_rotor_position_step_response_cycle = 0;

						sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
						sprintf(msg, "%i", enable_rotor_chirp);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						if (enable_rotor_chirp == 0){
						sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
						sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
						sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}

						if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
									&& enable_mod_sin_rotor_tracking == 0){
							sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
							sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

							if (enable_rotor_tracking_comb_signal == 1){
							rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
							}
						}

						if (enable_rotor_position_step_response_cycle == 1) {
							sprintf(msg, "\n\rRotor Step Drive enabled ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}

						if (enable_mod_sin_rotor_tracking == 1) {
							sprintf(msg, "\n\rRotor Sine Drive enabled ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}

						if (enable_rotor_chirp == 1) {
							enable_rotor_position_step_response_cycle = 0;
							enable_mod_sin_rotor_tracking = 0;
							sprintf(msg, "\n\rRotor Chirp Drive enabled ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}


						sprintf(msg, "\n\rEnter 1 to Enable Disturbance Rejection Sensitivity Function Analysis; 0 to Disable: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_disturbance_rejection_step);
						sprintf(msg, "%i", enable_disturbance_rejection_step);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						if (enable_disturbance_rejection_step == 1){
							enable_rotor_position_step_response_cycle = 1;
						}

						if (enable_disturbance_rejection_step == 0){
							sprintf(msg, "\n\rEnter 1 to Enable Noise Rejection Sensitivity Function Analysis; 0 to Disable: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_noise_rejection_step);
							sprintf(msg, "%i", enable_noise_rejection_step);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}
						if (enable_noise_rejection_step == 1){
							enable_rotor_position_step_response_cycle = 1;
						}

						if (enable_noise_rejection_step == 0){
							sprintf(msg, "\n\rEnter 1 to Enable Plant Sensitivity Function Analysis; 0 to Disable: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_plant_rejection_step);
							sprintf(msg, "%i", enable_plant_rejection_step);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}
						if (enable_plant_rejection_step == 1){
							enable_rotor_position_step_response_cycle = 1;
						}

						/*
						* Reverse polarity of gain values to account for suspended mode angle configuration
						*/
						if(select_suspended_mode == 1){
							proportional = 	-proportional;
							integral = 		-integral;
							derivative = 	-derivative;
							rotor_p_gain = 	-rotor_p_gain;
							rotor_i_gain = 	-rotor_i_gain;
							rotor_d_gain = 	-rotor_d_gain;
						}


						sprintf(msg, "\n\rEnter Torque Current mA (default is 800: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &torq_current_val);
						sprintf(msg, "%0.2f", torq_current_val);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						sprintf(msg, "\n\rSelect Motor Response Model Enter 1, 2, or 3 (4 Custom Entry): ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &motor_response_model);
						sprintf(msg, "%i", motor_response_model);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						switch(motor_response_model){
						case 1:
							max_speed = 		MAX_SPEED_MODE_1;
							min_speed = 		MIN_SPEED_MODE_1;
							break;
						case 2:
							max_speed = 		MAX_SPEED_MODE_2;
							min_speed = 		MIN_SPEED_MODE_2;
							break;
						case 3:
							max_speed = 		MAX_SPEED_MODE_3;
							min_speed = 		MIN_SPEED_MODE_3;
							break;
						case 4:
							sprintf(msg, "\n\rEnter Max Speed: ");
								HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &max_speed_read);
							sprintf(msg, "%i", max_speed_read);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							sprintf(msg, "\n\rEnter Min Speed: ");
								HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &min_speed_read);
							sprintf(msg, "%i", min_speed_read);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							max_speed = (uint16_t)(max_speed_read);
							min_speed = (uint16_t)(min_speed_read);
							sprintf(msg, "\n\rEnter Max Acceleration: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_test_acceleration_max);
							sprintf(msg, "%i", rotor_test_acceleration_max);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							sprintf(msg, "\n\rEnter Max Deceleration: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &swing_deceleration_max);
							sprintf(msg, "%i", swing_deceleration_max);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							BSP_MotorControl_SetAcceleration(0,(uint16_t)(rotor_test_acceleration_max));
							BSP_MotorControl_SetDeceleration(0,(uint16_t)(swing_deceleration_max));

						}

						sprintf(msg, "\n\rPendulum PID Gains: \tP: %.02f; I: %.02f; D: %.02f", proportional, integral, derivative);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg),HAL_MAX_DELAY);
						sprintf(msg, "\n\rRotor PID Gains: \tP: %.02f; I: %.02f; D: %.02f", rotor_p_gain, rotor_i_gain, rotor_d_gain);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg),HAL_MAX_DELAY);
						if (select_suspended_mode == 1){
							sprintf(msg, "\n\rSuspended Mode gains must be negative");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg),HAL_MAX_DELAY);
						}


					break;

					/*
					* Interactive entry of Pendulum Controller gains for Single PID Inverted Mode
					*/

					case 10:

						sprintf(msg, "\n\r *** Starting Single PID Configuration Mode ***\n\r ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg),
								HAL_MAX_DELAY);

						sprintf(msg, "\n\rEnter Pendulum PID Proportional Gain: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg),
								HAL_MAX_DELAY);

						read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &proportional);
						sprintf(msg, "%0.2f", proportional);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						sprintf(msg, "\n\rEnter Pendulum PID Integral Gain: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg),
								HAL_MAX_DELAY);
						read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &integral);
						sprintf(msg, "%0.2f", integral);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						sprintf(msg, "\n\rEnter Pendulum PID Differential Gain: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg),
								HAL_MAX_DELAY);
						read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &derivative);
						sprintf(msg, "%0.2f", derivative);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						/*
						* Rotor Controller gains for Single PID Mode
						*/
						rotor_p_gain = ROTOR_PID_PROPORTIONAL_GAIN_SINGLE_PID_MODE;
						rotor_i_gain = ROTOR_PID_INTEGRAL_GAIN_SINGLE_PID_MODE;
						rotor_d_gain = ROTOR_PID_DIFFERENTIAL_GAIN_SINGLE_PID_MODE;

						/*
						 * Only inverted mode is supported in Single PID Mode
						 */

						select_suspended_mode = 0;

						enable_rotor_position_step_response_cycle = 0;
						enable_mod_sin_rotor_tracking = 0;
						enable_rotor_chirp = 0;

						sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
						sprintf(msg, "%i", enable_rotor_chirp);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						if (enable_rotor_chirp == 0){
						sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
						sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
						sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}

						sprintf(msg, "\n\rEnter 1 to Enable Pendulum Impulse; 0 to Disable: ");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_pendulum_position_impulse_response_cycle);
						sprintf(msg, "%i", enable_pendulum_position_impulse_response_cycle);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						if (enable_rotor_position_step_response_cycle == 1) {
							sprintf(msg, "\n\rRotor Step Drive enabled ");
							enable_pendulum_position_impulse_response_cycle = 0;
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}

						if (enable_mod_sin_rotor_tracking == 1) {
							sprintf(msg, "\n\rRotor Sine Drive enabled ");
							enable_pendulum_position_impulse_response_cycle = 0;
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}

						if (enable_pendulum_position_impulse_response_cycle == 1) {
							sprintf(msg, "\n\rPendulum Impulse Drive enabled ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}

						/*
						* Reverse polarity of gain values to account for suspended mode angle configuration
						*/
						if(select_suspended_mode == 1){
							proportional = 	-proportional;
							integral = 		-integral;
							derivative = 	-derivative;
							rotor_p_gain = 	-rotor_p_gain;
							rotor_i_gain = 	-rotor_i_gain;
							rotor_d_gain = 	-rotor_d_gain;
						}

						sprintf(msg, "\n\rSelect Motor Response Model Enter 1, 2, or 3 (4 Custom Entry): ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &motor_response_model);
						sprintf(msg, "%i", motor_response_model);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

						switch(motor_response_model){
						/* Medium Speed Selection */
						case 2:
							max_speed = 		MAX_SPEED_MODE_1;
							min_speed = 		MIN_SPEED_MODE_1;
							break;
						/* High Speed Selection */
						case 3:
							max_speed = 		MAX_SPEED_MODE_2;
							min_speed = 		MIN_SPEED_MODE_2;
							break;
						/* Preferred Selection for Single PID corresponding to model */
						case 1:
							max_speed = 		MAX_SPEED_MODE_3;
							min_speed = 		MIN_SPEED_MODE_3;
							break;
						case 4:
							sprintf(msg, "\n\rEnter Max Speed: ");
								HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &max_speed_read);
							sprintf(msg, "%i", max_speed_read);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							sprintf(msg, "\n\rEnter Min Speed: ");
								HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &min_speed_read);
							sprintf(msg, "%i", min_speed_read);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							max_speed = (uint16_t)(max_speed_read);
							min_speed = (uint16_t)(min_speed_read);
							break;
						}

						sprintf(msg, "\n\rPendulum PID Gains: \tP: %.02f; I: %.02f; D: %.02f", proportional, integral, derivative);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg),HAL_MAX_DELAY);
						sprintf(msg, "\n\rRotor PID Gains: \tP: %.02f; I: %.02f; D: %.02f", rotor_p_gain, rotor_i_gain, rotor_d_gain);
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg),HAL_MAX_DELAY);
						if (select_suspended_mode == 1){
							sprintf(msg, "\n\rSuspended Mode gains must be negative");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg),HAL_MAX_DELAY);
						}

					break;

						case 11:
							enable_rotor_actuator_test = 1;
							enable_encoder_test = 1;
							sprintf(msg, "\n\rTest Mode Configured");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,
									strlen(msg), HAL_MAX_DELAY);
						break;

						case 12:
							enable_rotor_actuator_high_speed_test = 1;
							sprintf(msg, "\n\rHigh Speed Test Mode Configured");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg), HAL_MAX_DELAY);
							sprintf(msg, "\n\rEnter Motor Speed Min: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_test_speed_min);
							sprintf(msg, "%i", rotor_test_speed_min);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg), HAL_MAX_DELAY);
							sprintf(msg, "\n\rEnter Motor Speed Max: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_test_speed_max);
							sprintf(msg, "%i", rotor_test_speed_max);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							sprintf(msg, "\n\rEnter Motor Acceleration Max: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_test_acceleration_max);
							sprintf(msg, "%i", rotor_test_acceleration_max);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							sprintf(msg, "\n\rEnter Motor Deceleration Max: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &swing_deceleration_max);
							sprintf(msg, "%i", swing_deceleration_max);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

							sprintf(msg, "\n\rEnter Number of Swing Cycles: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &swing_cycles);
							sprintf(msg, "%i", swing_cycles);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							m = 0;

							sprintf(msg, "\n\rEnter Start Angle %i: ", m);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &start_angle_a[m]);
							sprintf(msg, "%i", start_angle_a[m]);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							sprintf(msg, "\n\rEnter End Angle %i: ", m);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &end_angle_a[m]);
							sprintf(msg, "%i", end_angle_a[m]);
							sprintf(msg, "\n\rEnter Dwell (milliseconds) %i: ",m);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &motion_dwell_a[m]);
							sprintf(msg, "%i", motion_dwell_a[m]);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							//m = m + 1;
							//}
							break;

						case 13:
							enable_motor_actuator_characterization_mode = 1;
							sprintf(msg, "\n\rMotor Characterization Mode Configured");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg), HAL_MAX_DELAY);

							rotor_test_speed_min = 200;
							rotor_test_speed_max = 1000;
							rotor_test_acceleration_max = 3000;
							swing_deceleration_max = 3000;
							torq_current_val = 800;
							rotor_chirp_amplitude = 5;
							rotor_chirp_start_freq = 0.05;
							rotor_chirp_end_freq = 5;
							rotor_chirp_period = 40;

							break;

						case 14:
							enable_pendulum_sysid_test = 1;
							sprintf(msg, "\n\rPendulum System Identification Test Mode Configured");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							break;

						case 15:
							enable_rotor_actuator_control = 1;
							sprintf(msg, "\n\rRotor Actuator Control Mode Configured");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,
									strlen(msg), HAL_MAX_DELAY);
							break;

						case 16:
							enable_rotor_tracking_comb_signal = 1;
							rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
							sprintf(msg, "\n\rLoad Disturbance Sensitivity Spectrum Analyzer Enabled");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,
									strlen(msg), HAL_MAX_DELAY);
							break;

						case 17:
							enable_disturbance_rejection_step = 1;
							enable_rotor_position_step_response_cycle = 1;
							sprintf(msg, "\n\rLoad Disturbance Sensitivity Step Response Enabled");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,
									strlen(msg), HAL_MAX_DELAY);
							break;

						case 18:
							enable_noise_rejection_step = 1;
							enable_rotor_position_step_response_cycle = 1;
							sprintf(msg, "\n\rNoise Rejection Sensitivity Step Response Enabled");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,
									strlen(msg), HAL_MAX_DELAY);
							break;

						case 19:
							enable_plant_rejection_step = 1;
							enable_rotor_position_step_response_cycle = 1;
							sprintf(msg, "\n\rPlant Sensitivity Step Response Enabled");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg,
									strlen(msg), HAL_MAX_DELAY);
							break;


				default:

					select_suspended_mode = 0;
					proportional = 		PRIMARY_PROPORTIONAL_MODE_1;
					integral = 			PRIMARY_INTEGRAL_MODE_1;
					derivative = 		PRIMARY_DERIVATIVE_MODE_1;
					rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_1;
					rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_1;
					rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_1;
					max_speed = 		MAX_SPEED_MODE_1;
					min_speed = 		MIN_SPEED_MODE_1;
					enable_rotor_position_step_response_cycle = 0;
					enable_mod_sin_rotor_tracking = 1;
					sprintf(msg, "\n\rDefault Mode 1 Configured");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg), HAL_MAX_DELAY);
					break;
				}

				/*
				* Exit user input read loop with selected or default mode
				*/
				break;
			}
		}

		/*
		* Set Motor Speed Profile
		*/

		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);

		BSP_MotorControl_SetMaxSpeed(0, max_speed);
		BSP_MotorControl_SetMinSpeed(0, min_speed);
		BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
		BSP_MotorControl_SetDeceleration(0, MAX_DECEL);



		sprintf(msg, "\n\rMotor Profile Speeds Set at Min %u Max %u Steps per Second and Suspended Mode %i",
				min_speed, max_speed, select_suspended_mode);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg,
				strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "\n\rMotor Torque Current Set at %f",
				torq_current_val);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg,
				strlen(msg), HAL_MAX_DELAY);

		if (select_suspended_mode == 1){
		sprintf(msg, "\n\rSuspended Mode selected...");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg,
				strlen(msg), HAL_MAX_DELAY);
		}

/*
 *		Configure Primary and Secondary PID controller data structures
 */
		pid_filter->integrator_windup_limit = windup;
		pid_filter->warn = 0;
		pid_filter->p_gain = proportional;
		pid_filter->i_gain = integral;
		pid_filter->d_gain = derivative;

		rotor_pid->integrator_windup_limit = rotor_windup;
		rotor_pid->warn = 0;
		rotor_pid->p_gain = rotor_p_gain;
		rotor_pid->i_gain = rotor_i_gain;
		rotor_pid->d_gain = rotor_d_gain;

		/*
		 *  Setting enable_pid enables control loop
		 *  Note, enable_pid may be reset to zero and disabling
		 *  control loop is user action to orient pendulum vertical
		 */

		enable_pid = ENABLE_PID;

/*
 * High Speed Rotor Actuator Test
 */

		if (enable_rotor_actuator_high_speed_test == 1) {
			i = 0;

			/*
			 * Set Motor Speed Profile
			 */

			BSP_MotorControl_SetAcceleration(0,(uint16_t)(rotor_test_acceleration_max));
			BSP_MotorControl_SetDeceleration(0,(uint16_t)(swing_deceleration_max));

			sprintf(msg, "\n\rMotor Profile Speeds Min %u Max %u",
					rotor_test_speed_min, rotor_test_speed_max);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

			sprintf(msg, "\n\rMotor Profile Acceleration Max %u Deceleration Max %u",
					BSP_MotorControl_GetAcceleration(0), BSP_MotorControl_GetDeceleration(0));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);


			/*
			* Set Rotor Position Zero
			*/

			rotor_position_set();
			ret = rotor_position_read(&rotor_position);

				sprintf(msg,
						"\r\n\r\n********  Starting Rotor High Speed Test  ********\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				sprintf(msg, "\n\rMotor Profile Speeds Min %u Max %u",
						rotor_test_speed_min, rotor_test_speed_max);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

				ret = encoder_position_read(&encoder_position, &htim3);
				encoder_position_down = encoder_position;
				sprintf(msg, "\n\rInitial encoder position: %i", encoder_position_down);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

				k = 0;
				m = 0;
				tick_cycle_start = HAL_GetTick();

				while (k < swing_cycles) {

					ret = rotor_position_read(&rotor_position);
					BSP_MotorControl_GoTo(0,rotor_position + (int) (round(start_angle_a[m] * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE)));
					current_speed = BSP_MotorControl_GetCurrentSpeed(0);
					while(abs(current_speed) > 0){
						current_speed = BSP_MotorControl_GetCurrentSpeed(0);
						ret = rotor_position_read(&rotor_position);
						ret = encoder_position_read(&encoder_position, &htim3);
						tick = HAL_GetTick();
						test_time = (float) (abs(tick - tick_cycle_start)) / 1000;
						sprintf(msg, "\n\r%f\t%i\t%i", test_time, (int)(rotor_position/STEPPER_READ_POSITION_STEPS_PER_DEGREE),(int)((encoder_position - encoder_position_down)/ENCODER_READ_ANGLE_SCALE));
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
					}

					/*
					 * Motion right corresponds to rotor angle positive
					 * Pendulum motion left corresponds to angle negative
					 */

					abs_encoder_position_max = 0;
					HAL_Delay(motion_dwell_a[0]);
					while (1){
						/*
						 * Wait for pendulum angle positive indicating motion right occurring
						 */
						ret = encoder_position_read(&encoder_position, &htim3);
						tick = HAL_GetTick();
						test_time = (float) (abs(tick - tick_cycle_start)) / 1000;
						ret = rotor_position_read(&rotor_position);
						sprintf(msg, "\n\r%f\t%i\t%i", test_time, (int)(rotor_position/STEPPER_READ_POSITION_STEPS_PER_DEGREE),(int)((encoder_position - encoder_position_down)/ENCODER_READ_ANGLE_SCALE));
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
						if ((encoder_position - encoder_position_down) < 0){
							//sprintf(msg, "\n\rAngle %i", (int)((encoder_position - encoder_position_down)/ENCODER_READ_ANGLE_SCALE));
							//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
							HAL_Delay(10);
							continue;
						}
						/*
						 * Find maximum
						 */
						ret = encoder_position_read(&encoder_position, &htim3);
						abs_encoder_position_prior = abs(encoder_position - encoder_position_down);
						HAL_Delay(10);
						ret = encoder_position_read(&encoder_position, &htim3);
						abs_encoder_position_after = abs(encoder_position - encoder_position_down);
						if (abs_encoder_position_prior > abs_encoder_position_max){
							abs_encoder_position_max = abs_encoder_position_prior;
						}
						/*
						 * Add additional wait for encoder angle to drop below 90
						 * prior to next motion
						 */

						if ((abs_encoder_position_after - abs_encoder_position_prior) < 10){
								break;
						}
					}



					ret = rotor_position_read(&rotor_position);
					BSP_MotorControl_GoTo(0,rotor_position + (int) (round(-start_angle_a[m] * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE)));
					current_speed = BSP_MotorControl_GetCurrentSpeed(0);
					while(abs(current_speed) > 0){
						current_speed = BSP_MotorControl_GetCurrentSpeed(0);
						ret = rotor_position_read(&rotor_position);
						ret = encoder_position_read(&encoder_position, &htim3);
						tick = HAL_GetTick();
						test_time = (float) (abs(tick - tick_cycle_start)) / 1000;
						sprintf(msg, "\n\r%f\t%i\t%i", test_time, (int)(rotor_position/STEPPER_READ_POSITION_STEPS_PER_DEGREE),(int)((encoder_position - encoder_position_down)/ENCODER_READ_ANGLE_SCALE));
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
					}


					/*
					 * Motion left corresponds to rotor angle negative
					 * Pendulum motion right corresponds to angle positive
					 */
					abs_encoder_position_max = 0;
					HAL_Delay(motion_dwell_a[0]);
					while (1){
						/*
						 * Wait for pendulum angle negative indicating pendulum motion left
						 */
						ret = encoder_position_read(&encoder_position, &htim3);
						tick = HAL_GetTick();
						test_time = (float) (abs(tick - tick_cycle_start)) / 1000;
						ret = rotor_position_read(&rotor_position);
						sprintf(msg, "\n\r%f\t%i\t%i", test_time, (int)(rotor_position/STEPPER_READ_POSITION_STEPS_PER_DEGREE),(int)((encoder_position - encoder_position_down)/ENCODER_READ_ANGLE_SCALE));
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
						if ((encoder_position - encoder_position_down) > 0){
							//sprintf(msg, "\n\rAngle %i", (int)((encoder_position - encoder_position_down)/ENCODER_READ_ANGLE_SCALE));
							//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
							HAL_Delay(10);
							continue;
						}
						/*
						 * Find maximum
						 */
						ret = encoder_position_read(&encoder_position, &htim3);
						abs_encoder_position_prior = abs(encoder_position - encoder_position_down);
						HAL_Delay(10);
						ret = encoder_position_read(&encoder_position, &htim3);
						abs_encoder_position_after = abs(encoder_position - encoder_position_down);
						if (abs_encoder_position_prior > abs_encoder_position_max){
							abs_encoder_position_max = abs_encoder_position_prior;
						}
						/*
						 * Add additional wait for encoder angle to drop below 90
						 * prior to next motion
						 */


						if ((abs_encoder_position_after - abs_encoder_position_prior) < 10){
								break;
						}
					}
					k = k + 1;
				}

				L6474_CmdDisable(0);
				while (1) {
					sprintf(msg,
							"Test Operation Complete, System in Standby, Press Reset Button to Restart\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							HAL_MAX_DELAY);
					HAL_Delay(5000);
				}
		}


		/*
		 *
		 * Pendulum System Identification Test
		 *
		 */

		if (enable_pendulum_sysid_test == 1){

			ret = rotor_position_read(&rotor_position);
			sprintf(msg, "\r\n\r\n********  Starting Pendulum System Identification Test in 5 Seconds ********\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);


			tick_cycle_start = HAL_GetTick();
			k = 0;
			while (k < 10){
				ret = rotor_position_read(&rotor_position);
				BSP_MotorControl_GoTo(0,rotor_position + (int)(16*(STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE)));
				HAL_Delay(5000);
				ret = rotor_position_read(&rotor_position);
				BSP_MotorControl_GoTo(0,(int)(rotor_position - 16*(STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE)));
				i = 0;
				while (i < 5000){
					test_time = HAL_GetTick() - tick_cycle_start;
					ret = encoder_position_read(&encoder_position, &htim3);
					sprintf(msg, "\n\r%i\t%i\t%0.1f\t%0.1f", i, (int)((HAL_GetTick() - tick_cycle_start)),
							(float)(encoder_position/ENCODER_READ_ANGLE_SCALE),
							(float)(rotor_position/STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
					HAL_Delay(2);
					i = i + 1;
				}
				k = k + 1;
			}
			L6474_CmdDisable(0);
			while (1) {
				sprintf(msg,
						"Test Operation Complete, System in Standby, Press Reset Button to Restart\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
						HAL_MAX_DELAY);
				HAL_Delay(5000);
			}
		}

		/*
		 *
		 * Motor Control Characterization Test
		 *
		 */


		if (enable_motor_actuator_characterization_mode == 1) {

			/*
			 * Set Motor Speed Profile
			 */

			BSP_MotorControl_SetMaxSpeed(0, rotor_test_speed_max);
			BSP_MotorControl_SetMinSpeed(0, rotor_test_speed_min);

			sprintf(msg, "\n\rMotor Profile Speeds Min %u Max %u",
					rotor_test_speed_min, rotor_test_speed_max);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);

			BSP_MotorControl_SetAcceleration(0,
					(uint16_t) (rotor_test_acceleration_max));
			BSP_MotorControl_SetDeceleration(0,
					(uint16_t) (swing_deceleration_max));

			sprintf(msg,
					"\n\rMotor Profile Acceleration Max %u Deceleration Max %u",
					BSP_MotorControl_GetAcceleration(0),
					BSP_MotorControl_GetDeceleration(0));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);

			/*
			 * Set Rotor Position Zero
			 */

			rotor_position_set();
			test_time = HAL_GetTick() - tick_cycle_start;

			rotor_chirp_step_period = (int) (rotor_chirp_period * 240.0);
			tick_cycle_start = HAL_GetTick();
			mode_index_command = 1;
			mode_index = 1;

			while (1) {
				i = 0;
				while (i < rotor_chirp_step_period) {
					RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE
							- __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
					readBytes = Extract_Msg(RxBuffer, RxBuffer_ReadIdx,
							RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

					if (readBytes == 2 && Msg.Len == 1 && i % 10 == 0) {
						RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes)
								% UART_RX_BUFFER_SIZE;
						mode_transition_state = 1;
						if (strcmp((char *) Msg.Data, mode_string_stop) == 0) {
							mode_index_command = mode_quit;
						} else if (strcmp((char *) Msg.Data, mode_string_inc_accel)
								== 0) {
							mode_index_command = 17;
						} else if (strcmp((char *) Msg.Data, mode_string_dec_accel)
								== 0) {
							mode_index_command = 16;
						} else if (strcmp((char *) Msg.Data,mode_string_inc_amp)
								== 0) {
							mode_index_command = 18;
						} else if (strcmp((char *) Msg.Data,mode_string_dec_amp)
								== 0) {
							mode_index_command = 19;
						} else if (strcmp((char *) Msg.Data,
								mode_string_mode_motor_characterization_mode)
								== 0) {
							mode_index_command = 1;
						} else {
							mode_index_command = atoi((char*) Msg.Data);
						}
					}



					if (mode_index_command == mode_quit) {
						break;
					}

					if (mode_index_command == 1 && mode_transition_state == 1) {
						mode_index = 1;
						mode_transition_state = 0;
					}

					if (mode_index_command == 2 && mode_transition_state == 1) {
						mode_index = 2;
						mode_transition_state = 0;
					}

					if (mode_index_command == 3 && mode_transition_state == 1) {
						L6474_SetAnalogValue(0, L6474_TVAL, 400);
						mode_transition_state = 0;
					}

					if (mode_index_command == 4 && mode_transition_state == 1) {
						L6474_SetAnalogValue(0, L6474_TVAL, 600);
						mode_transition_state = 0;
					}
					if (mode_index_command == 5 && mode_transition_state == 1) {
						L6474_SetAnalogValue(0, L6474_TVAL, 800);
						mode_transition_state = 0;
					}

					if (mode_index_command == 6 && mode_transition_state == 1) {
						rotor_test_speed_max = rotor_test_speed_max + 100;
						if (rotor_test_speed_max > 1000) {
							rotor_test_speed_max = 1000;
						}
						BSP_MotorControl_SoftStop(0);
						BSP_MotorControl_WaitWhileActive(0);
						BSP_MotorControl_SetMaxSpeed(0, rotor_test_speed_max);
						mode_transition_state = 0;
					}

					if (mode_index_command == 7 && mode_transition_state == 1) {
						rotor_test_speed_max = rotor_test_speed_max - 100;
						if (rotor_test_speed_max < 200) {
							rotor_test_speed_max = 200;
						}
						if (rotor_test_speed_min > rotor_test_speed_max) {
							rotor_test_speed_max = rotor_test_speed_min;
						}
						BSP_MotorControl_SoftStop(0);
						BSP_MotorControl_WaitWhileActive(0);
						BSP_MotorControl_SetMaxSpeed(0, rotor_test_speed_max);
						mode_transition_state = 0;
					}

					if (mode_index_command == 8 && mode_transition_state == 1) {
						rotor_test_speed_min = rotor_test_speed_min + 100;
						if (rotor_test_speed_min > rotor_test_speed_max) {
							rotor_test_speed_min = rotor_test_speed_max;
						}
						if (rotor_test_speed_min > 1000) {
							rotor_test_speed_min = 1000;
						}
						BSP_MotorControl_SoftStop(0);
						BSP_MotorControl_WaitWhileActive(0);
						BSP_MotorControl_SetMinSpeed(0, rotor_test_speed_min);
						mode_transition_state = 0;
					}

					if (mode_index_command == 9 && mode_transition_state == 1) {
						rotor_test_speed_min = rotor_test_speed_min - 100;
						if (rotor_test_speed_min < 200) {
							rotor_test_speed_min = 200;
						}
						BSP_MotorControl_SoftStop(0);
						BSP_MotorControl_WaitWhileActive(0);
						BSP_MotorControl_SetMinSpeed(0, rotor_test_speed_min);
						mode_transition_state = 0;
					}

					if (mode_index_command == 16 && mode_transition_state == 1) {
						rotor_test_acceleration_max = rotor_test_acceleration_max - 500;
						if (rotor_test_acceleration_max < 0) {
							rotor_test_acceleration_max = 0;
						}
						swing_deceleration_max = rotor_test_acceleration_max;
						BSP_MotorControl_SoftStop(0);
						BSP_MotorControl_WaitWhileActive(0);
						BSP_MotorControl_SetAcceleration(0,
								(uint16_t) (rotor_test_acceleration_max));
						BSP_MotorControl_SetDeceleration(0,
								(uint16_t) (swing_deceleration_max));
						mode_transition_state = 0;
					}

					if (mode_index_command == 17 && mode_transition_state == 1) {
						rotor_test_acceleration_max = rotor_test_acceleration_max + 500;
						if (rotor_test_acceleration_max > 10000) {
							rotor_test_acceleration_max = 10000;
						}
						swing_deceleration_max = rotor_test_acceleration_max;
						BSP_MotorControl_SoftStop(0);
						BSP_MotorControl_WaitWhileActive(0);
						BSP_MotorControl_SetAcceleration(0,
								(uint16_t) (rotor_test_acceleration_max));
						BSP_MotorControl_SetDeceleration(0,
								(uint16_t) (swing_deceleration_max));
						mode_transition_state = 0;
					}

					if (mode_index_command == 18 && mode_transition_state == 1) {
						rotor_chirp_amplitude = rotor_chirp_amplitude + 1;
						if (rotor_chirp_amplitude > 10) {
							rotor_chirp_amplitude = 10;
						}
						mode_transition_state = 0;
					}

					if (mode_index_command == 19 && mode_transition_state == 1) {
						rotor_chirp_amplitude = rotor_chirp_amplitude - 1;
						if (rotor_chirp_amplitude < 1) {
							rotor_chirp_amplitude = 1;
						}
						mode_transition_state = 0;
					}

					if (i == 0) {
						cycle_period_start = HAL_GetTick();
						cycle_period_sum = 100 * T_SAMPLE * 1000 - 1;
					}
					if (i % 100 == 0) {
						cycle_period_sum = HAL_GetTick() - cycle_period_start;
						cycle_period_start = HAL_GetTick();
					}

					tick_cycle_previous = tick_cycle_current;
					tick_cycle_current = tick;
					chirp_time = (float) (i) / 400;
					rotor_chirp_frequency = rotor_chirp_start_freq
							+ (rotor_chirp_end_freq - rotor_chirp_start_freq)
									* (float) (i) / rotor_chirp_step_period;

					if (mode_index == 1) {
						rotor_position_command =
								rotor_chirp_amplitude
										* (float) (STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE)
										* sin(
												2.0 * 3.14159
														* rotor_chirp_frequency
														* chirp_time);
					}

					if (mode_index == 2) {
						if (sin(
								2.0 * 3.14159 * rotor_chirp_frequency
										* chirp_time) < 0) {
							k = -1;
						} else {
							k = 1;
						}
						rotor_position_command = k * rotor_chirp_amplitude
								* STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
					}

					current_speed = BSP_MotorControl_GetCurrentSpeed(0);
					BSP_MotorControl_GoTo(0, (int) (rotor_position_command));

					if (BSP_MotorControl_GetDeviceState(0) == ACCELERATING) {
						motor_state = 1;
					}
					if (BSP_MotorControl_GetDeviceState(0) == DECELERATING) {
						motor_state = -1;
					}
					if (BSP_MotorControl_GetDeviceState(0) == STEADY) {
						motor_state = -2;
					}
					if (BSP_MotorControl_GetDeviceState(0) == INACTIVE) {
						motor_state = 0;
					}
					ret = rotor_position_read(&rotor_position);
					current_speed = BSP_MotorControl_GetCurrentSpeed(0);
					sprintf(msg,
							"%i\t%i\t%i\t%i\t%i\t%f\t%i\t%i\t%i\t%i\t%i\r\n", i,
							cycle_period_sum,
							(int) (tick_cycle_current - tick_cycle_previous),
							current_speed, rotor_position,
							rotor_position_command, motor_state,
							rotor_test_speed_max, rotor_test_speed_min,
							rotor_test_acceleration_max, swing_deceleration_max);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							HAL_MAX_DELAY);
					i = i + 1;
				}
				if (mode_index_command == mode_quit) {
					break;
				}
				j = j + 1;
			}
			L6474_CmdDisable(0);
			while (1){
				HAL_Delay(5000);
				sprintf(msg, "\r\nMotor Characterization Terminated, Press Reset to Continue");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
			}
		}


		/*
		 *
		 * Interactive Digital Motor Control system
		 *
		 */

		if (enable_rotor_actuator_control == 1) {

			while (1) {

				/*
				 * Set Motor Speed Profile
				 */

				sprintf(msg, "\r\nEnter Motor Maximum Speed: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
						&rotor_test_speed_max);
				sprintf(msg, "%i", rotor_test_speed_max);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				sprintf(msg, "\r\nEnter Motor Minimum Speed: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
						&rotor_test_speed_min);
				sprintf(msg, "%i", rotor_test_speed_min);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				sprintf(msg, "\r\nEnter Motor Maximum Acceleration: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
						&rotor_test_acceleration_max);
				sprintf(msg, "%i", rotor_test_acceleration_max);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				sprintf(msg, "\r\nEnter Motor Maximum Deceleration: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
						&swing_deceleration_max);
				sprintf(msg, "%i", swing_deceleration_max);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				BSP_MotorControl_SetMaxSpeed(0, rotor_test_speed_max);
				BSP_MotorControl_SetMinSpeed(0, rotor_test_speed_min);

				sprintf(msg, "\n\rMotor Profile Speeds Minimum %u Maximum %u",
						rotor_test_speed_min, rotor_test_speed_max);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				BSP_MotorControl_SetAcceleration(0, rotor_test_acceleration_max);
				BSP_MotorControl_SetDeceleration(0, swing_deceleration_max);

				sprintf(msg,"\n\rMotor Profile Acceleration Maximum %u Deceleration Maximum %u",
						BSP_MotorControl_GetAcceleration(0), BSP_MotorControl_GetDeceleration(0));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

				j = 1;

				/*
				 * Set initial rotor position
				 */

				while (1) {

					sprintf(msg, "\r\nEnter Motor Position Target in Degrees: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &rotor_position_command);
					sprintf(msg, "%0.3f", rotor_position_command);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					BSP_MotorControl_GoTo(0, (int)(rotor_position_command*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
					BSP_MotorControl_WaitWhileActive(0);

					ret = rotor_position_read(&rotor_position);
					sprintf(msg, "Motor Position in Steps %i and Degrees %.2f\r\n",
							rotor_position, (float) ((rotor_position) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					sprintf(msg, "\r\nEnter 1 to Exit Motor Test, 0 to Continue ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
							&j);
					if (j == 1) {
						break;
					}

				}
			}
			L6474_CmdDisable(0);
			while (1){
				HAL_Delay(5000);
				sprintf(msg, "\r\nMotor Characterization Terminated, Press Reset to Continue");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
			}
		}


		/*
		 * 	Rotor Test Sequence will execute at each cycle of operation if ENABLE_ROTOR_ACTUATOR_TEST set to 1
		 *
		 *	Correct Operation is confirmed if Rotor first rotates clockwise (viewing from above) by 90
		 *	degrees, then returns to initial location, then rotates 90 degrees counterclockwise by 90
		 *	degrees and then returns to initial location
		 */

		if (enable_rotor_actuator_test == 1) {

			/*
			 * Set Motor Speed Profile
			 */

			BSP_MotorControl_SetMaxSpeed(0, MAX_SPEED_MODE_1);
			BSP_MotorControl_SetMinSpeed(0, MIN_SPEED_MODE_1);

			sprintf(msg, "\n\rMotor Profile Speeds Min %u Max %u",
					rotor_test_speed_min, rotor_test_speed_max);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

			BSP_MotorControl_SetAcceleration(0,(uint16_t)(MAX_ACCEL));
			BSP_MotorControl_SetDeceleration(0,(uint16_t)(MAX_DECEL));

			sprintf(msg, "\n\rMotor Profile Acceleration Max %u Deceleration Max %u",
					BSP_MotorControl_GetAcceleration(0), BSP_MotorControl_GetDeceleration(0));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

			j = 0;

			while (j < ROTOR_ACTUATOR_TEST_CYCLES) {

				j++;

				sprintf(msg, "\r\n\r\n********  Starting Rotor Motor Control Test  ********\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

				ret = rotor_position_read(&rotor_position);
				sprintf(msg, "Motor Position at Zero Angle: %.2f\r\n",
						(float) ((rotor_position) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

				sprintf(msg, "Next Test in 3s\r\n\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
				HAL_Delay(3000);

				rotor_position_command = -45;
				BSP_MotorControl_GoTo(0, (int)(rotor_position_command*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
				BSP_MotorControl_WaitWhileActive(0);

				ret = rotor_position_read(&rotor_position);
				sprintf(msg, "Motor Position Test to -45 Degree Angle: %.2f\r\n",
						(float) ((rotor_position) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

				sprintf(msg, "Correct motion shows rotor rotating to left\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "Next Test in 3s\r\n\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				HAL_Delay(3000);

				rotor_position_command = 0;
				BSP_MotorControl_GoTo(0, (int)(rotor_position_command*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
				BSP_MotorControl_WaitWhileActive(0);

				ret = rotor_position_read(&rotor_position);
				sprintf(msg, "Motor Position Test to Zero Angle: %.2f\r\n",
						(float) ((rotor_position) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

				sprintf(msg, "Correct motion shows rotor returning to zero angle\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

				sprintf(msg, "Next Test in 3s\r\n\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
				HAL_Delay(3000);

				rotor_position_command = 90;
				BSP_MotorControl_GoTo(0, (int)(rotor_position_command*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
				BSP_MotorControl_WaitWhileActive(0);

				ret = rotor_position_read(&rotor_position);
				sprintf(msg, "Motor Position at 90 Degree Angle: %.2f\r\n",
						(float) ((rotor_position) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "Correct motion shows rotor rotating to right\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

				sprintf(msg, "Next Test in 3s\r\n\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
				HAL_Delay(3000);

				rotor_position_command = 0;
				BSP_MotorControl_GoTo(0, (int)(rotor_position_command*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
				BSP_MotorControl_WaitWhileActive(0);

				ret = rotor_position_read(&rotor_position);
				sprintf(msg, "Motor Position at Zero Angle: %.2f\r\n",
						(float) ((rotor_position) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

				sprintf(msg, "Correct motion shows rotor rotating to zero angle\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

				sprintf(msg, "Rotor Actuator Test Cycle Complete, Next Test in 3s\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
				HAL_Delay(3000);
			}
		}

/*
 * 	Encoder Test Sequence will execute at each cycle of operation if enable_encoder_test is set to 1
 */

		if (enable_encoder_test == 1) {
			sprintf(msg, "\r\n*************  Starting Encoder Test  ***************\r\n\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);

			sprintf(msg, "Permit Pendulum to Stabilize in Vertical Down\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
			HAL_Delay(1000);
			sprintf(msg, "Angle will be measured in 3 seconds\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
			HAL_Delay(3000);

			ret = encoder_position_read(&encoder_position, &htim3);
			encoder_position_down = encoder_position;
			sprintf(msg, "Encoder Angle is: %.2f \r\n(Correct value should lie between -0.5 and 0.5 degrees))\r\n\r\n",
					(float) (encoder_position_down / angle_scale));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);

			sprintf(msg,
					"Manually Rotate Pendulum in Clock Wise Direction One Full 360 Degree Turn and Stabilize Down\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
			sprintf(msg, "Angle will be measured in 10 seconds\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
			HAL_Delay(10000);

			ret = encoder_position_read(&encoder_position, &htim3);
			sprintf(msg, "Encoder Angle is: %.2f\r\n(Correct value should lie between -359.5 and -360.5 degrees)\r\n\r\n",
					(float) ((encoder_position - encoder_position_down)
							/ angle_scale));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);

			sprintf(msg,
					"Manually Rotate Pendulum in Counter Clock Wise Direction One Full 360 Degree Turn and Stabilize Down\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
			sprintf(msg, "Angle will be measured in 10 seconds\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
			HAL_Delay(10000);

			ret = encoder_position_read(&encoder_position, &htim3);
			sprintf(msg, "Encoder Angle is: %.2f \r\n(Correct value should lie between -0.5 and 0.5 degrees) \r\n\r\n",
					(float) ((encoder_position - encoder_position_down)
							/ angle_scale));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);

			L6474_CmdDisable(0);
			while(1){
				sprintf(msg, "Test Operation Complete, System in Standby, Press Reset Button to Restart\r\n");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
									HAL_MAX_DELAY);
							HAL_Delay(5000);
			}
		}

		/*
		 * Set Motor Position Zero
		 */

		rotor_position_set();
		ret = rotor_position_read(&rotor_position);
		sprintf(msg,
				"\r\nPrepare for Control Start - Initial Rotor Position: %i\r\n",
				rotor_position);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);


		/*
		* Detect motion of Pendulum prior to measurement of down angle reference
		*
		* If motion detected, delay start until Pendulum is motionless
		*
		*/
		sprintf(msg, "Test for Pendulum at Rest - Stabilize Pendulum Now\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		ret = encoder_position_read(&encoder_position, &htim3);
		encoder_position_prev = encoder_position;
		HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
		ret = encoder_position_read(&encoder_position, &htim3);
		encoder_position_curr = encoder_position;
		while (encoder_position_curr != encoder_position_prev) {
			ret = encoder_position_read(&encoder_position, &htim3);
			encoder_position_prev = encoder_position;
			HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
			ret = encoder_position_read(&encoder_position, &htim3);
			encoder_position_curr = encoder_position;
			/*
			* Ensure stability reached with final motion test
			*/
			if (encoder_position_prev == encoder_position_curr) {
				HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
				ret = encoder_position_read(&encoder_position, &htim3);
				encoder_position_prev = encoder_position;
				HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
				ret = encoder_position_read(&encoder_position, &htim3);
				encoder_position_curr = encoder_position;
				if (encoder_position_prev == encoder_position_curr) {
				break;
				}
			}
			sprintf(msg, "Pendulum Motion Detected with angle %0.2f - Stabilize Pendulum Now\r\n",
					(float) ((encoder_position_curr - encoder_position_prev)
							/ ENCODER_READ_ANGLE_SCALE));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
		}

		sprintf(msg,
				"Pendulum Now at Rest and Measuring Pendulum Down Angle\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		/*
		* Detect encoder read fault
		*/

		ret = encoder_position_read(&encoder_position, &htim3);
		if (ret == -1) {
			sprintf(msg, "Encoder Position Under Range Error\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
		}
		if (ret == 1) {
			sprintf(msg, "Encoder Position Over Range Error\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
		}

		/*
		* Calibrate down angle
		*/
		encoder_position_down = encoder_position;

		ret = encoder_position_read(&encoder_position, &htim3);
		if (ret == -1) {
			sprintf(msg, "Encoder Position Under Range Error\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
		}
		if (ret == 1) {
			sprintf(msg, "Encoder Position Over Range Error\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
		}


		/*
		* Alert user to adjust pendulum upright by
		* counter clockwise rotation by Motor Position displacement prompt
		*
		*/
		BSP_MotorControl_GoTo(0, 30);
		BSP_MotorControl_WaitWhileActive(0);
		HAL_Delay(150);
		BSP_MotorControl_GoTo(0, -30);
		BSP_MotorControl_WaitWhileActive(0);
		HAL_Delay(150);
		BSP_MotorControl_GoTo(0, 30);
		BSP_MotorControl_WaitWhileActive(0);
		HAL_Delay(150);
		BSP_MotorControl_GoTo(0, 0);
		BSP_MotorControl_WaitWhileActive(0);

		if(select_suspended_mode == 0){
		sprintf(msg,
				"Adjust Pendulum Upright By Turning CCW Control Will Start When Vertical\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}

		/*
		 * Detect Start Condition for Pendulum Angle for Inverted Model
		 *
		 * Detect Pendulum Angle equal to vertical within tolerance of START_ANGLE
		 *
		 * Exit if no vertical orientation action detected and alert user to restart,
		 * then disable pid control and enable system restart.
		 *
		 * Permitted delay for user action is PENDULUM_ORIENTATION_START_DELAY.
		 *
		 */


		tick_wait_start = HAL_GetTick();
		if (select_suspended_mode == 0) {
			while (1){
				ret = encoder_position_read(&encoder_position, &htim3);
				if (abs(encoder_position - encoder_position_down - (int) ((round)(180 * angle_scale))) < START_ANGLE * angle_scale){
					break;
				}
				if (abs(encoder_position - encoder_position_down + (int) ((round)(180 * angle_scale))) < START_ANGLE * angle_scale){
					encoder_position_down = encoder_position_down - 2*(int) ((round)(180 * angle_scale));
					break;
				}
				tick_wait = HAL_GetTick();

				if ( (tick_wait - tick_wait_start) > PENDULUM_ORIENTATION_START_DELAY){
					sprintf(msg, "Pendulum Upright Action Not Detected - Restarting ...\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					enable_pid = 0;
					break;
				}

			}
		}


		/*
		 * For case of Suspended Mode Operation, no initial condition check is required
		 */

		if(select_suspended_mode == 1){
		sprintf(msg, "Suspended Mode Control Will Start in %i Seconds\r\n",
				(int) (CONTROL_START_DELAY / 1000));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}

		/*
		* Set Rotor Position Zero
		*/

		rotor_position_set();
		ret = rotor_position_read(&rotor_position);

		sprintf(msg, "Initial Rotor Position: %i\r\n", rotor_position);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);



		/*
		* Initialize Primary and Secondary PID controllers
		*/

		*current_error = 0;

		// Temporary fix to account for change in unit conversions:
		float p_gain = pid_filter->p_gain;
		float i_gain = pid_filter->i_gain;
		float d_gain = pid_filter->d_gain;
		if (ACCEL_CONTROL == 0) {
			pid_filter->p_gain /= STEPPER_READ_POSITION_STEPS_PER_DEGREE;
			pid_filter->i_gain /= STEPPER_READ_POSITION_STEPS_PER_DEGREE;
			pid_filter->d_gain /= STEPPER_READ_POSITION_STEPS_PER_DEGREE;
		}


		pid_filter_control_execute(pid_filter, current_error, sample_period,
				deriv_lp_corner_f);

		// Temporary fix to account for change in unit conversions:
		if (ACCEL_CONTROL == 0) {
			pid_filter->p_gain = p_gain;
			pid_filter->i_gain = i_gain;
			pid_filter->d_gain = d_gain;
		}

		*current_error_rotor = 0;
		pid_filter_control_execute(rotor_pid, current_error_rotor,
				sample_period_rotor, deriv_lp_corner_f_rotor);


		cycle_count = CYCLE_LIMIT;
		i = 0;


		rotor_position = 0;
		rotor_position_prev = 0;
		rotor_position_filter = 0;
		rotor_position_filter_prev = 0;
		rotor_position_command = 0;

		rotor_position_diff = 0;
		rotor_position_diff_prev = 0;
		rotor_position_diff_filter = 0;
		rotor_position_diff_filter_prev = 0;

		rotor_position_step_polarity = 1;

		/*
		* Default start mode is with Step Drive tracking command
		* if Sine Drive is selected at run-time, Sine Drive is retained
		* since discontinuities are otherwise encountered
		*/

		encoder_angle_slope_corr = 0;

		rotor_sine_drive = 0;
		sine_drive_transition = 0;
		rotor_mod_control = 1.0;

		for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
			Msg.Data[k] = 0;
		}

		__HAL_DMA_RESET_HANDLE_STATE(&hdma_usart2_rx);

		enable_adaptive_mode = 0;

		tick_cycle_start = HAL_GetTick();
		tick_cycle_previous = tick_cycle_start;
		tick_cycle_current =  tick_cycle_start;
		tick_cycle_current = tick_cycle_start;
		chirp_cycle = 0;
		chirp_dwell_cycle = 0;
		pendulum_position_command = 0;
		impulse_start_index = 0;
		mode_transition_state = 0;
		transition_to_adaptive_mode = 0;
        error_sum_prev = 0;
        error_sum_filter_prev = 0;
        adaptive_state = 4;
		rotor_position_target_prev = 0;
		rotor_position_command_prev = 0;
		enable_high_speed_sampling = ENABLE_HIGH_SPEED_SAMPLING_MODE;
		slope_prev = 0;
		rotor_track_comb_command = 0;

		noise_rej_signal_prev = 0;
		noise_rej_signal_filter_prev = 0;


/*
 * *************************************************************************************************
 *
 * Control Loop Start
 *
 * *************************************************************************************************
 */
		if (ACCEL_CONTROL == 1) {
			BSP_MotorControl_HardStop(0);
			L6474_CmdEnable(0);
		}
		int32_t target_velocity_prescaled = 0;

		while (enable_pid == 1) {

			/*
			*	Test for run time user input requesting change in Motor Speed Profile
			*	mode, switch to Suspended Mode, or control terminate
			*/

			RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE
					- __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
			readBytes = Extract_Msg(RxBuffer, RxBuffer_ReadIdx,
					RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

			config_command = 0;
			if (readBytes == 2 && Msg.Len == 1 && i % 10 == 0){
				RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;

				mode_index_prev = mode_index;

				strcpy(config_message, (char *) Msg.Data);
				if (strcmp((char *) Msg.Data, mode_string_inc_pend_p) == 0){
					pid_filter->p_gain = pid_filter->p_gain + adjust_increment;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_pend_p) == 0) {
					pid_filter->p_gain = pid_filter->p_gain - adjust_increment;
					if (pid_filter->p_gain <= 0) { pid_filter->p_gain = 0; }
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_pend_d) == 0) {
					pid_filter->d_gain = pid_filter->d_gain + adjust_increment;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_pend_d) == 0) {
					pid_filter->d_gain = pid_filter->d_gain - adjust_increment;
					if (pid_filter->d_gain <= 0) { pid_filter->d_gain = 0; }
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_pend_i) == 0) {
					pid_filter->i_gain = pid_filter->i_gain + adjust_increment;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_pend_i) == 0) {
					pid_filter->i_gain = pid_filter->i_gain - adjust_increment;
					if (pid_filter->i_gain <= 0) { pid_filter->i_gain = 0; }
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_rotor_p) == 0){
					rotor_pid->p_gain = rotor_pid->p_gain + adjust_increment;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_rotor_p) == 0) {
					rotor_pid->p_gain = rotor_pid->p_gain - adjust_increment;
					if (rotor_pid->p_gain <= 0) { rotor_pid->p_gain = 0; }
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_rotor_d) == 0) {
					rotor_pid->d_gain = rotor_pid->d_gain + adjust_increment;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_rotor_d) == 0) {
					rotor_pid->d_gain = rotor_pid->d_gain - adjust_increment;
					if (rotor_pid->d_gain <= 0) { rotor_pid->d_gain = 0; }
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_rotor_i) == 0) {
					rotor_pid->i_gain = rotor_pid->i_gain + adjust_increment;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_rotor_i) == 0) {
					rotor_pid->i_gain = rotor_pid->i_gain - adjust_increment;
					if (rotor_pid->i_gain <= 0) { rotor_pid->i_gain = 0; }
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_torq_c) == 0) {
					torq_current_val = L6474_GetAnalogValue(0, L6474_TVAL);
					torq_current_val = torq_current_val - adjust_increment;
					if (torq_current_val < 200){ torq_current_val = 200; }
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_torq_c) == 0) {
					torq_current_val = L6474_GetAnalogValue(0, L6474_TVAL);
					torq_current_val = torq_current_val + adjust_increment;
					if (torq_current_val > 800){ torq_current_val = 800; }
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_max_s) == 0) {
					max_speed = L6474_GetMaxSpeed(0);
					max_speed = max_speed - adjust_increment;
					if (max_speed < 100){ max_speed = 100; }
					if (max_speed < min_speed){ max_speed = min_speed;}
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					L6474_SetMaxSpeed(0, max_speed);
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_max_s) == 0) {
					max_speed = L6474_GetMaxSpeed(0);
					max_speed = max_speed + adjust_increment;
					if (max_speed > 1000){ max_speed = 1000; }
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					L6474_SetMaxSpeed(0, max_speed);
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_min_s) == 0) {
					min_speed = L6474_GetMinSpeed(0);
					min_speed = min_speed - adjust_increment;
					if (min_speed < 100){ min_speed = 100; }
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					L6474_SetMinSpeed(0, min_speed);
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_min_s) == 0) {
					min_speed = L6474_GetMinSpeed(0);
					min_speed = min_speed + adjust_increment;
					if (min_speed > 1000){ min_speed = 1000; }
					if (min_speed > max_speed){ min_speed = max_speed;}
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					L6474_SetMinSpeed(0, min_speed);
					config_command = 1;
					mode_index_command = -1;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_max_a) == 0) {
					max_accel = L6474_GetAcceleration(0);
					max_accel = max_accel - adjust_increment;
					if (max_accel <  0){ max_accel = 0;}
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					L6474_SetAcceleration(0, max_accel);
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_max_a) == 0) {
					max_accel = L6474_GetAcceleration(0);
					max_accel = max_accel + adjust_increment;
					if (max_accel >  10000){ max_accel = 10000;}
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					L6474_SetAcceleration(0, max_accel);
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_max_d) == 0) {
					max_decel = L6474_GetDeceleration(0);
					max_decel = max_decel - adjust_increment;
					if (max_decel <  0){ max_decel = 0;}
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					L6474_SetDeceleration(0, max_decel);
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_max_d) == 0) {
					max_decel = L6474_GetDeceleration(0);
					max_decel = max_decel + adjust_increment;
					if (max_decel > 10000) { max_decel = 10000; }
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					L6474_SetDeceleration(0, max_decel);
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_select_mode_5) == 0) {
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					L6474_SetDeceleration(0, MAX_DECEL);
					L6474_SetAcceleration(0, MAX_ACCEL);
					L6474_SetMinSpeed(0, MIN_SPEED_MODE_5);
					L6474_SetMaxSpeed(0, MAX_SPEED_MODE_5);
					pid_filter->p_gain = PRIMARY_PROPORTIONAL_MODE_5;
					pid_filter->i_gain = PRIMARY_INTEGRAL_MODE_5;
					pid_filter->d_gain = PRIMARY_DERIVATIVE_MODE_5;
					rotor_pid->p_gain = SECONDARY_PROPORTIONAL_MODE_5;
					rotor_pid->i_gain = SECONDARY_INTEGRAL_MODE_5;
					rotor_pid->d_gain = SECONDARY_DERIVATIVE_MODE_5;
					enable_adaptive_mode = 0;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_enable_step ) == 0 ){
					enable_rotor_position_step_response_cycle = 1;
					enable_noise_rejection_step = 0;
					enable_disturbance_rejection_step = 0;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_disable_step ) == 0 ){
					enable_rotor_position_step_response_cycle = 0;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_enable_noise_rej_step ) == 0 ){
					enable_noise_rejection_step = 1;
					enable_rotor_position_step_response_cycle = 1;
					enable_disturbance_rejection_step = 0;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_disable_noise_rej_step ) == 0 ){
					enable_noise_rejection_step = 0;
					enable_rotor_position_step_response_cycle = 1;
					enable_disturbance_rejection_step = 0;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_enable_sensitivity_fnc_step ) == 0 ){
					enable_sensitivity_fnc_step = 1;
					enable_rotor_position_step_response_cycle = 1;
					enable_disturbance_rejection_step = 0;
					enable_noise_rejection_step = 0;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_disable_sensitivity_fnc_step ) == 0 ){
					enable_sensitivity_fnc_step = 0;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_enable_load_dist ) == 0 ){
					enable_sensitivity_fnc_step = 0;
					enable_rotor_position_step_response_cycle = 1;
					enable_disturbance_rejection_step = 1;
					enable_noise_rejection_step = 0;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_disable_load_dist ) == 0 ){
					enable_disturbance_rejection_step = 0;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_step_size ) == 0 ){
					step_size = step_size + 1;
					if (step_size > 4) { step_size = 4; }
					if (step_size == 0) { adjust_increment = 0.5;}
					else if (step_size == 1) { adjust_increment = 2;}
					else if (step_size == 2) { adjust_increment = 10;}
					else if (step_size == 3) { adjust_increment = 50;}
					else if (step_size == 4) { adjust_increment = 100;}
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_step_size ) == 0 ){
					step_size = step_size - 1;
					if (step_size < 0) { step_size = 0; }
					if (step_size == 0) { adjust_increment = 0.5;}
					else if (step_size == 1) { adjust_increment = 2;}
					else if (step_size == 2) { adjust_increment = 10;}
					else if (step_size == 3) { adjust_increment = 50;}
					else if (step_size == 4) { adjust_increment = 100;}
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_enable_high_speed_sampling ) == 0 ){
					enable_high_speed_sampling = 1;
					config_command = 1;
				} else if (strcmp((char *) Msg.Data, mode_string_disable_high_speed_sampling ) == 0 ){
					enable_high_speed_sampling = 0;
					config_command = 1;
				} else {
					mode_index_command = atoi((char*) Msg.Data);
				}


				if (strcmp(config_message, "q") == 0){
					sprintf(tmp_string,
							"\n\rExit Control Loop Command Received ");
					HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string,
							strlen(tmp_string), HAL_MAX_DELAY);
					break;
				}

				/*
				* Disable sin drive tracking
				*/


				if (mode_index_command == mode_9){
					disable_mod_sin_rotor_tracking = 1;
					sine_drive_transition = 1;
					mode_index_command = -1;
				}

				if (config_command == 0){

				/*
				* Enable sin drive tracking
				*/

				if (mode_index_command == mode_5){
					disable_mod_sin_rotor_tracking = 0;
					sine_drive_transition = 1;
					mode_index_command = -1;
				}

				/*
				* Enable state change only if transition not occurring
				*/

				if (mode_transition_state == 0) {

					/*
					* Enable adaptive mode
					* Transition to adaptive mode via Mode 1 if in Mode 2
					*/

					/*
					* If user selects a Motor Model, then disable adaptive mode
					* Place adaptive state in readiness for transition to return
					* to adaptive
					*/

					if (mode_index_command == mode_1
							|| mode_index_command == mode_2
							|| mode_index_command == mode_3
							|| mode_index_command == mode_4) {
						adaptive_state = 4;
						enable_adaptive_mode = 0;
					}

					if (mode_index_command == mode_adaptive) {
						adaptive_state = 4;
						transition_to_adaptive_mode = 1;
						enable_adaptive_mode = 1;
						mode_index_command = -1;
					}

					/*
					* Disable adaptive mode and set to Motor Model M
					*/

					if (mode_index_command == mode_adaptive_off) {
						adaptive_state = 4;
						enable_adaptive_mode = 0;
						mode_index = mode_1;
						mode_index_prev = mode_1;
						mode_index_command = -1;
					}


					if (mode_index_prev == mode_3
							&& mode_index_command == mode_1) {
						adaptive_state = 4;
						enable_adaptive_mode = 0;
						mode_index = mode_1;
						mode_index_command = -1;
					}

					if (mode_index_prev == mode_2
							&& mode_index_command == mode_1) {
						adaptive_state = 4;
						enable_adaptive_mode = 0;
						mode_index = mode_1;
						mode_index_command = -1;
					}

					if (mode_index_prev == mode_1
							&& mode_index_command == mode_3) {
						adaptive_state = 4;
						enable_adaptive_mode = 0;
						mode_index = mode_3;
						mode_index_command = -1;
					}

					if (mode_index_prev == mode_1
							&& mode_index_command == mode_2) {
						adaptive_state = 4;
						enable_adaptive_mode = 0;
						mode_index = mode_2;
						mode_index_command = -1;
					}

					/*
					* Protect user mode transition from Motor Model L to Motor Model H
					* by introducing state through Motor Model M.  The following is first
					* step in transition through Motor Model M.
					*/

					if (mode_index_prev == mode_3
							&& mode_index_command == mode_2) {
						adaptive_state = 4;
						enable_adaptive_mode = 0;
						mode_index = mode_1;
						mode_transition_tick = HAL_GetTick();
						mode_transition_state = 1;
						mode_index_command = -1;
					}

					/*
					* Protect user mode transition from Motor Model H to Motor Model L
					* by introducing state through Motor Model M.  The following is first
					* step in transition through Motor Model M.
					*/


					if (mode_index_prev == mode_2
							&& mode_index_command == mode_3) {
						adaptive_state = 4;
						enable_adaptive_mode = 0;
						mode_index = mode_1;
						mode_transition_tick = HAL_GetTick();
						mode_transition_state = 1;
						mode_index_command = -1;
					}

					if (mode_index == mode_2) {
						select_suspended_mode = 0;
						proportional = PRIMARY_PROPORTIONAL_MODE_2;
						integral = PRIMARY_INTEGRAL_MODE_2;
						derivative = PRIMARY_DERIVATIVE_MODE_2;
						rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_2;
						rotor_i_gain = SECONDARY_INTEGRAL_MODE_2;
						rotor_d_gain = SECONDARY_DERIVATIVE_MODE_2;
						max_speed = MAX_SPEED_MODE_2;
						min_speed = MIN_SPEED_MODE_2;
						pid_filter->p_gain = proportional;
						pid_filter->i_gain = integral;
						pid_filter->d_gain = derivative;
						rotor_pid->p_gain = rotor_p_gain;
						rotor_pid->i_gain = rotor_i_gain;
						rotor_pid->d_gain = rotor_d_gain;
						torq_current_val = 800;
						L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
						BSP_MotorControl_SoftStop(0);
						BSP_MotorControl_WaitWhileActive(0);
						BSP_MotorControl_SetMaxSpeed(0, max_speed);
						BSP_MotorControl_SetMinSpeed(0, min_speed);
						BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
						BSP_MotorControl_SetDeceleration(0, MAX_DECEL);
					}

					if (mode_index == mode_3) {
						select_suspended_mode = 0;
						proportional = PRIMARY_PROPORTIONAL_MODE_3;
						integral = PRIMARY_INTEGRAL_MODE_3;
						derivative = PRIMARY_DERIVATIVE_MODE_3;
						rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_3;
						rotor_i_gain = SECONDARY_INTEGRAL_MODE_3;
						rotor_d_gain = SECONDARY_DERIVATIVE_MODE_3;
						max_speed = MAX_SPEED_MODE_3;
						min_speed = MIN_SPEED_MODE_3;
						pid_filter->p_gain = proportional;
						pid_filter->i_gain = integral;
						pid_filter->d_gain = derivative;
						rotor_pid->p_gain = rotor_p_gain;
						rotor_pid->i_gain = rotor_i_gain;
						rotor_pid->d_gain = rotor_d_gain;
						torq_current_val = 800;
						L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
						BSP_MotorControl_SoftStop(0);
						BSP_MotorControl_WaitWhileActive(0);
						BSP_MotorControl_SetMaxSpeed(0, max_speed);
						BSP_MotorControl_SetMinSpeed(0, min_speed);
						BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
						BSP_MotorControl_SetDeceleration(0, MAX_DECEL);
					}

					if (mode_index == mode_1) {
						select_suspended_mode = 0;
						proportional = PRIMARY_PROPORTIONAL_MODE_1;
						integral = PRIMARY_INTEGRAL_MODE_1;
						derivative = PRIMARY_DERIVATIVE_MODE_1;
						rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
						rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
						rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;
						max_speed = MAX_SPEED_MODE_1;
						min_speed = MIN_SPEED_MODE_1;
						pid_filter->p_gain = proportional;
						pid_filter->i_gain = integral;
						pid_filter->d_gain = derivative;
						rotor_pid->p_gain = rotor_p_gain;
						rotor_pid->i_gain = rotor_i_gain;
						rotor_pid->d_gain = rotor_d_gain;
						torq_current_val = 800;
						L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
						BSP_MotorControl_SoftStop(0);
						BSP_MotorControl_WaitWhileActive(0);
						BSP_MotorControl_SetMaxSpeed(0, max_speed);
						BSP_MotorControl_SetMinSpeed(0, min_speed);
						BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
						BSP_MotorControl_SetDeceleration(0, MAX_DECEL);
					}
				}
			}
			}  // End of Read Loop

			/*
			* Protect user mode transition from Motor Model H to Motor Model L
			* or Model L to Model H
			* by introducing state through Motor Model M.  The following is final
			* step in transition and includes restoration of enable_adaptive if
			* transition from adaptive off to adaptive on.
			*/

			if (mode_transition_state == 1){
				if ((HAL_GetTick() - mode_transition_tick) > USER_TRANSITION_DWELL){
					mode_transition_state = 0;
					select_suspended_mode = 0;
					if (transition_to_adaptive_mode == 1) {
						adaptive_state = 4;
						enable_adaptive_mode = 1;
						transition_to_adaptive_mode = 0;
					}

					if (mode_index_prev == mode_3){
					mode_index = 2;
					proportional = PRIMARY_PROPORTIONAL_MODE_2;
					integral = PRIMARY_INTEGRAL_MODE_2;
					derivative = PRIMARY_DERIVATIVE_MODE_2;
					rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_2;
					rotor_i_gain = SECONDARY_INTEGRAL_MODE_2;
					rotor_d_gain = SECONDARY_DERIVATIVE_MODE_2;
					max_speed = MAX_SPEED_MODE_2;
					min_speed = MIN_SPEED_MODE_2;
					pid_filter->p_gain = proportional;
					pid_filter->i_gain = integral;
					pid_filter->d_gain = derivative;
					rotor_pid->p_gain = rotor_p_gain;
					rotor_pid->i_gain = rotor_i_gain;
					rotor_pid->d_gain = rotor_d_gain;
					torq_current_val = 800;
					L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					BSP_MotorControl_SetMaxSpeed(0,max_speed);
					BSP_MotorControl_SetMinSpeed(0, min_speed);
					BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
					BSP_MotorControl_SetDeceleration(0, MAX_DECEL);
					}

					if (mode_index_prev == mode_2){
					mode_index = 3;
					proportional = PRIMARY_PROPORTIONAL_MODE_3;
					integral = PRIMARY_INTEGRAL_MODE_3;
					derivative = PRIMARY_DERIVATIVE_MODE_3;
					rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_3;
					rotor_i_gain = SECONDARY_INTEGRAL_MODE_3;
					rotor_d_gain = SECONDARY_DERIVATIVE_MODE_3;
					max_speed = MAX_SPEED_MODE_3;
					min_speed = MIN_SPEED_MODE_3;
					pid_filter->p_gain = proportional;
					pid_filter->i_gain = integral;
					pid_filter->d_gain = derivative;
					rotor_pid->p_gain = rotor_p_gain;
					rotor_pid->i_gain = rotor_i_gain;
					rotor_pid->d_gain = rotor_d_gain;
					torq_current_val = 800;
					L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					BSP_MotorControl_SetMaxSpeed(0,max_speed);
					BSP_MotorControl_SetMinSpeed(0, min_speed);
					BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
					BSP_MotorControl_SetDeceleration(0, MAX_DECEL);
					}
				}
		}

			/*
			* Exit control if cycle count limit set
			*/

			i++;
			if (i > cycle_count && ENABLE_CYCLE_INFINITE == 0) {
				break;
			}

			/*
			 * Create chirp signal for Sensitivity Function characterization
			 */


			if (enable_rotor_tracking_comb_signal > 0) {

				chirp_time = (float)((i - 1)/ROTOR_TRACK_COMB_SIGNAL_SAMPLE_RATE);
				rotor_track_comb_signal_frequency = 0.1;
				rotor_track_comb_command = ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.316;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 1.0;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 3.16;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
			}

			/*
			* Acquire encoder position and correct for initial angle value of
			* encoder measured at vertical down position at system start
			*/

			ret = encoder_position_read(&encoder_position, &htim3);
			if (select_suspended_mode == 0) {
				encoder_position = encoder_position - encoder_position_down - (int) ((round)(180 * angle_scale));
			}
			/*
			* For case of Suspended Mode Operation, no initial angle offset is required
			*/

			/*
			* Detect pendulum position excursion exceeding limits and exit
			*/

			if(select_suspended_mode == 0){
				if ((encoder_position / ENCODER_READ_ANGLE_SCALE)
						> ENCODER_POSITION_POSITIVE_LIMIT) {
					sprintf(msg, "Error Exit Encoder Position Exceeded: %i\r\n",
						encoder_position);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
						HAL_MAX_DELAY);
					break;
				}
				if ((encoder_position / ENCODER_READ_ANGLE_SCALE)
						< ENCODER_POSITION_NEGATIVE_LIMIT) {
					sprintf(msg, "Error Exit Encoder Position Exceeded: %i\r\n",
						encoder_position);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
						HAL_MAX_DELAY);
					break;
				}

			}

			/*
			* Detect rotor position excursion exceeding limits and exit
			*/

			if (rotor_position
					> (ROTOR_POSITION_POSITIVE_LIMIT
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
				sprintf(msg, "Error Exit Motor Position Exceeded: %i\r\n",
						rotor_position);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
				break;
			}

			if (rotor_position
					< (ROTOR_POSITION_NEGATIVE_LIMIT
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
				sprintf(msg, "Error Exit Motor Position Exceeded: %i\r\n",
						rotor_position);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
				break;
			}

		   /*
			* Encoder Angle Error Compensation
			*
			* Compute Proportional control of pendulum angle compensating for error due to
			* encoder offset at start time or system platform slope relative to horizontal.
			*
			* Apply optional time limit to correct for encoder error or slope.  For cycle count
			* greater than ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT, encoder angle
			* slope correction remains constant.
			*
			* If ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT = 0, then encoder angle slope
			* correction continues operation for all time
			*
			*/

			/*
			* Compute Low Pass Filtered rotor position difference
			*/

			rotor_position_diff_prev = rotor_position_diff;
			if (enable_disturbance_rejection_step == 0){
			rotor_position_diff = rotor_position_filter
					- rotor_position_command;
			}
			if (enable_disturbance_rejection_step == 1){
			rotor_position_diff = rotor_position_filter;
			}
			rotor_position_diff_filter_prev = rotor_position_diff_filter;
			rotor_position_diff_filter =
					(float) (rotor_position_diff * iir_LT_0)
							+ rotor_position_diff_prev * iir_LT_1
							- rotor_position_diff_filter_prev * iir_LT_2;

			if (ENABLE_ENCODER_ANGLE_SLOPE_CORRECTION == 1 ) {
				if ((i < ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT) || (ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT == 0)) {
					encoder_angle_slope_corr = (rotor_position_diff_filter / STEPPER_READ_POSITION_STEPS_PER_DEGREE) / ENCODER_ANGLE_SLOPE_CORRECTION_SCALE;
				}
			}

		   /*
			*  Compute current_error input for Primary Controller
			*
			*  current_error is sum of encoder angle error compensation and encoder position in degrees
			*
			*  An Encoder offset may be introduced.  The Encoder offset may remain at all times if
			*  ENCODER_OFFSET_DELAY == 0 or terminate at a time (in ticks) of ENCODER_OFFSET_DELAY
			*/

			if ((i < ENCODER_START_OFFSET_DELAY) || (ENCODER_START_OFFSET_DELAY == 0)){
				encoder_position = encoder_position - ENCODER_START_OFFSET;
			}

			*current_error = encoder_angle_slope_corr
					+ (float) ((ENCODER_ANGLE_POLARITY)
							* (encoder_position / (ENCODER_READ_ANGLE_SCALE)));

		   /*
			* Pendulum Controller execution
			*
			* Include addition of pendulum position impulse signal
			*
			*/

			*current_error = *current_error + pendulum_position_command;

			// Temporary fix to account for change in unit conversions:
			float p_gain = pid_filter->p_gain;
			float i_gain = pid_filter->i_gain;
			float d_gain = pid_filter->d_gain;
			if (ACCEL_CONTROL == 0) {
				pid_filter->p_gain /= STEPPER_READ_POSITION_STEPS_PER_DEGREE;
				pid_filter->i_gain /= STEPPER_READ_POSITION_STEPS_PER_DEGREE;
				pid_filter->d_gain /= STEPPER_READ_POSITION_STEPS_PER_DEGREE;
			}

			pid_filter_control_execute(pid_filter, current_error, sample_period,
					deriv_lp_corner_f);
			rotor_position_target = pid_filter->control_output*STEPPER_READ_POSITION_STEPS_PER_DEGREE;

			// Temporary fix to account for change in unit conversions:
			if (ACCEL_CONTROL == 0) {
				pid_filter->p_gain = p_gain;
				pid_filter->i_gain = i_gain;
				pid_filter->d_gain = d_gain;
			}

		   /*
			* Acquire Motor Position and Compute Low Pass Filtered Motor Position
			*/

			ret = rotor_position_read(&rotor_position);

			rotor_position_filter = (float) (rotor_position) * iir_0
					+ rotor_position_prev * iir_1
					- rotor_position_filter_prev * iir_2;
			rotor_position_prev = (float) (rotor_position);
			rotor_position_filter_prev = rotor_position_filter;

		   /*
			* 		Record current value of rotor_position_command tracking signal for
			* 		detection of change in tracking signal by adaptive control
			*/


			/*
			 * 		Chirp cycle count begins with index value of 0 and continues through to
			 * 		ROTOR_CHIRP_PERIOD - 1 to create a number of samples equal to ROTOR_CHIRP_PERIOD
			 */

			if (enable_rotor_chirp == 1 && enable_mod_sin_rotor_tracking == 0
							&& enable_rotor_tracking_comb_signal == 0) {

				if (i < ROTOR_CHIRP_PERIOD - 1){
					chirp_cycle = 0;
				}

				if (chirp_cycle > ROTOR_CHIRP_PERIOD - 1) {
					chirp_cycle = 0;
					chirp_dwell_cycle = ROTOR_CHIRP_SWEEP_DELAY;
				}
				if (chirp_dwell_cycle > 0){
					chirp_dwell_cycle--;
					chirp_cycle = 0;
				}
				if (chirp_dwell_cycle == 0 && i >= ROTOR_CHIRP_PERIOD - 1){
					chirp_cycle = chirp_cycle + 1;
					chirp_time = (float)((chirp_cycle - 1)/ROTOR_CHIRP_SAMPLE_RATE);
					rotor_chirp_frequency = rotor_chirp_start_freq + (rotor_chirp_end_freq - rotor_chirp_start_freq)*(chirp_cycle/rotor_chirp_period);
					rotor_position_command = ((float)(ROTOR_CHIRP_STEP_AMPLITUDE*STEPPER_READ_POSITION_STEPS_PER_DEGREE))*sin(2.0*3.14159*rotor_chirp_frequency*chirp_time/10);
				}
			}

			if (enable_rotor_chirp == 0 && enable_mod_sin_rotor_tracking == 0
											&& enable_rotor_tracking_comb_signal == 1) {
					rotor_position_command = rotor_track_comb_command;
			}

			rotor_sine_drive = 0;

			if (enable_mod_sin_rotor_tracking == 1 && ENABLE_ROTOR_CHIRP == 0) {

				if (ENABLE_ROTOR_CHIRP == 0){
					mod_sin_carrier_frequency = MOD_SIN_CARRIER_FREQ;
				}

				if (i > MOD_SIN_START_CYCLES && enable_mod_sin_rotor_tracking == 1) {
					rotor_sine_drive =
							(float) (mod_sin_amplitude
									* (1 + sin(-1.5707 + ((i - MOD_SIN_START_CYCLES)/MOD_SIN_SAMPLE_RATE) * (MOD_SIN_MODULATION_FREQ * 6.2832))));
					rotor_sine_drive_mod = sin(0 + ((i - MOD_SIN_START_CYCLES) /MOD_SIN_SAMPLE_RATE) * (mod_sin_carrier_frequency * 6.2832));
					rotor_sine_drive = rotor_sine_drive * rotor_sine_drive_mod * rotor_mod_control;
				}

				if (i > MOD_SIN_START_CYCLES && ENABLE_SIN_MOD == 0) {
					rotor_sine_drive_mod = sin(0 + ((i - MOD_SIN_START_CYCLES) /MOD_SIN_SAMPLE_RATE) * (mod_sin_carrier_frequency * 6.2832));
					rotor_sine_drive = rotor_control_sin_amplitude * rotor_sine_drive_mod * rotor_mod_control;
				}


				rotor_position_command = rotor_sine_drive;


				/*
				* Detect transition between sine drive enable and disable
				*/

				if ( abs(rotor_sine_drive_mod*MOD_SIN_AMPLITUDE) < 2 && disable_mod_sin_rotor_tracking == 1 && sine_drive_transition == 1){
					rotor_mod_control = 0.0;
					sine_drive_transition = 0;
				}

				if ( abs(rotor_sine_drive_mod*MOD_SIN_AMPLITUDE) < 2 && disable_mod_sin_rotor_tracking == 0 && sine_drive_transition == 1){
					rotor_mod_control = 1.0;
					sine_drive_transition = 0;
				}
			}

			if (ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE == 1 && i != 0) {
				if ((i % ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
					rotor_position_command =
							(float) (ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
									* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
					impulse_start_index = 0;
				}
				if (impulse_start_index
						> ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
					rotor_position_command = 0;
				}
				impulse_start_index++;
			}

			if (enable_pendulum_position_impulse_response_cycle == 1 && i != 0) {

				if ((i % PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
					pendulum_position_command =
							(float) PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE;  //TODO Units of pendulum_position_command were fixed (changed from encoder counts to degrees), but this will change behavior (it will be 6.667 times less than before)
					impulse_start_index = 0;
					chirp_cycle = 0;
				}
				if (impulse_start_index
						> PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
					pendulum_position_command = 0;
				}
				impulse_start_index++;
				chirp_cycle++;
			}


			if ((i % ROTOR_POSITION_STEP_RESPONSE_CYCLE_INTERVAL) == 0 && enable_rotor_position_step_response_cycle == 1) {
				rotor_position_step_polarity = -rotor_position_step_polarity;
				if (rotor_position_step_polarity == 1){
					chirp_cycle = 0;
				}
			}

			if (enable_rotor_position_step_response_cycle == 1) {
					if (STEP_RESPONSE_AMP_LIMIT_ENABLE == 1 && abs(rotor_sine_drive) > STEP_RESPONSE_AMP_LIMIT){
						chirp_cycle = chirp_cycle + 1;
					} else {
						rotor_position_command = rotor_sine_drive + (float) ((rotor_position_step_polarity)
											* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
											* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
						chirp_cycle = chirp_cycle + 1;
					}
			}

			if (ENABLE_DUAL_PID == 1) {

			/*
			 * Secondary Controller execution
			 */
				if (enable_disturbance_rejection_step == 0 && enable_plant_rejection_step == 0 && enable_noise_rejection_step == 0){
				*current_error_rotor = rotor_position_filter - rotor_position_command;
				}
				if (enable_disturbance_rejection_step == 1){
				*current_error_rotor = rotor_position_filter;
				}
				if (enable_noise_rejection_step == 1){
					*current_error_rotor = rotor_position_filter + rotor_position_command;
				}

				pid_filter_control_execute(rotor_pid, current_error_rotor,
						sample_period_rotor, deriv_lp_corner_f_rotor);
				rotor_position_target = pid_filter->control_output*STEPPER_READ_POSITION_STEPS_PER_DEGREE + rotor_pid->control_output;
			}


			/*
			 * Adaptive Control
			 */

            error_sum = *current_error_rotor;
            error_sum_filter = (float) (error_sum * iir_LT_0)
                            + error_sum_prev * iir_LT_1
                            - error_sum_filter_prev * iir_LT_2;
            error_sum_prev = error_sum;
            error_sum_filter_prev = error_sum_filter;
			adaptive_error = abs(error_sum_filter/ STEPPER_READ_POSITION_STEPS_PER_DEGREE);



			if (enable_adaptive_mode == 1 && i > 2) {

				if (adaptive_error < adaptive_threshold_low
						&& adaptive_state == 1) {
					adaptive_state = 2;
					adaptive_entry_tick = HAL_GetTick();
				}
				if (adaptive_error < adaptive_threshold_low
						&& adaptive_state == 2) {
					if (HAL_GetTick() - adaptive_entry_tick > adaptive_dwell_period) {
						adaptive_state = 3;
						adaptive_entry_tick = HAL_GetTick();
					}
				}

				if (adaptive_error < adaptive_threshold_low
						&& adaptive_state == 3) {
					if (HAL_GetTick() - adaptive_entry_tick > 2*adaptive_dwell_period) {
						adaptive_state = 4;
						adaptive_entry_tick = HAL_GetTick();
					}
				}

				if (fabs(rotor_position_command_prev - rotor_position_command) > adaptive_threshold_high) {
					adaptive_state = 1;
				}


				/*
				 * Ensure state change occurs only when mode state changes
				 */

				/*
				 * Final state of adaptive mode may be set to mode 1 or mode 3
				 */

				if (adaptive_state == 4 && (adaptive_state_change != adaptive_state)) {
					select_suspended_mode = 0;
					mode_index = 1;
					proportional = PRIMARY_PROPORTIONAL_MODE_3;
					integral = PRIMARY_INTEGRAL_MODE_3;
					derivative = PRIMARY_DERIVATIVE_MODE_3;
					rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_3;
					rotor_i_gain = SECONDARY_INTEGRAL_MODE_3;
					rotor_d_gain = SECONDARY_DERIVATIVE_MODE_3;
					max_speed = MAX_SPEED_MODE_3;
					min_speed = MIN_SPEED_MODE_3;
					pid_filter->p_gain = proportional;
					pid_filter->i_gain = integral;
					pid_filter->d_gain = derivative;

					rotor_pid->p_gain = rotor_p_gain;
					rotor_pid->i_gain = rotor_i_gain;
					rotor_pid->d_gain = rotor_d_gain;
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					BSP_MotorControl_SetMaxSpeed(0, max_speed);
					BSP_MotorControl_SetMinSpeed(0, min_speed);
					BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
					BSP_MotorControl_SetDeceleration(0, MAX_DECEL);
				}

				if (adaptive_state == 3 && (adaptive_state_change != adaptive_state)) {
					select_suspended_mode = 0;
					mode_index = 1;
					proportional = PRIMARY_PROPORTIONAL_MODE_1;
					integral = PRIMARY_INTEGRAL_MODE_1;
					derivative = PRIMARY_DERIVATIVE_MODE_1;
					rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
					rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
					rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;
					max_speed = MAX_SPEED_MODE_1;
					min_speed = MIN_SPEED_MODE_1;
					pid_filter->p_gain = proportional;
					pid_filter->i_gain = integral;
					pid_filter->d_gain = derivative;

					rotor_pid->p_gain = rotor_p_gain;
					rotor_pid->i_gain = rotor_i_gain;
					rotor_pid->d_gain = rotor_d_gain;
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					BSP_MotorControl_SetMaxSpeed(0, max_speed);
					BSP_MotorControl_SetMinSpeed(0, min_speed);
					BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
					BSP_MotorControl_SetDeceleration(0, MAX_DECEL);

				}
				if (adaptive_state == 1 && (adaptive_state_change != adaptive_state)) {
					select_suspended_mode = 0;
					mode_index = 2;
					proportional = PRIMARY_PROPORTIONAL_MODE_2;
					integral = PRIMARY_INTEGRAL_MODE_2;
					derivative = PRIMARY_DERIVATIVE_MODE_2;
					rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_2;
					rotor_i_gain = SECONDARY_INTEGRAL_MODE_2;
					rotor_d_gain = SECONDARY_DERIVATIVE_MODE_2;
					max_speed = MAX_SPEED_MODE_2;
					min_speed = MIN_SPEED_MODE_2;
					pid_filter->p_gain = proportional;
					pid_filter->i_gain = integral;
					pid_filter->d_gain = derivative;

					rotor_pid->p_gain = rotor_p_gain;
					rotor_pid->i_gain = rotor_i_gain;
					rotor_pid->d_gain = rotor_d_gain;
					BSP_MotorControl_SoftStop(0);
					BSP_MotorControl_WaitWhileActive(0);
					BSP_MotorControl_SetMaxSpeed(0, max_speed);
					BSP_MotorControl_SetMinSpeed(0, min_speed);
					BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
					BSP_MotorControl_SetDeceleration(0, MAX_DECEL);
				}

				adaptive_state_change = adaptive_state;
			}

			if(i == 0){
				cycle_period_start = HAL_GetTick();
				cycle_period_sum = 100*T_SAMPLE*1000 - 1;
			}
			if(i % 100 == 0){
				cycle_period_sum = HAL_GetTick() - cycle_period_start;
				cycle_period_start = HAL_GetTick();
			}
			tick = HAL_GetTick();
			tick_cycle_previous = tick_cycle_current;
			tick_cycle_current = tick;

			/*
			 * Introduce roll-over in time variable at 500 seconds to prevent
			 * excessive character count and delay in serial data
			 * transport
			 */


			/*
			 *
			 * Control system data output over USB serial interface at 115200 baud
			 * cycle_period_sum: 		    Time difference between 100 successive cycles. Used for
			 * 								computing accurate estimate of cycle period
			 * cycle_time: 					Time for current control system cycle in milliseconds
			 * encoder_position				Pendulum angle in 6.667 steps per degree
			 * rotor_position:				Rotor angle in 8.887 steps per degree
			 * pendulum_angle_controller:	Output of Pendulum Angle PID controller in 17.778 steps per degree)
			 * rotor_command:				Rotor tracking command in 8.889 steps per degree
			 * chirp_cycle:					Cycle count for chirp cycle operation
			 * rotor_position_target: 		Target angle for motor position supplied to motor
			 * 								controller (in 17.778 steps per degree) includes difference
			 * 								of rotor_position to compensate for addition of reference rotor
			 * 								angle required in motor drive interface.
			 * rotor_angle_controller:		Output of Pendulum Angle PID controller in 17.778 steps per degree)
			 *
			 * Output values for pendulum_angle_controller, rotor_position_target
			 * and rotor_angle_controller are scaled by factor of 10 to reduce
			 * characters in data payload
			 *
			 */

			/*
			 * Load Disturbance Sensitivity Function signal introduction
			 *
			 * If Load Disturbance Sensitivity Function enabled
			 */

			if (enable_disturbance_rejection_step == 1){
				rotor_position_target = rotor_position_target + rotor_position_command;
			}

			/*
			 * Output rate limiter
			 */


			rotor_target_in = rotor_position_target;

			if(ENABLE_LIMITER == 1 && ACCEL_CONTROL == 0){

				if ((rotor_position_target - rotor_position_target_prev) >= LIMITER_THRESHOLD) {
					rotor_position_target = rotor_position_target_prev
							+ (rotor_position_target - rotor_position_target_prev) / (LIMITER_SLOPE*LIMITER_THRESHOLD);
					slope = rotor_position_target - rotor_position_target_prev;
				}

				if ((rotor_position_target - rotor_position_target_prev) <= -LIMITER_THRESHOLD) {
					rotor_position_target = rotor_position_target_prev
							+ (rotor_position_target - rotor_position_target_prev) /  (LIMITER_SLOPE*LIMITER_THRESHOLD);
					slope = rotor_position_target - rotor_position_target_prev;
				}


				if (slope_prev < 0 && slope == 0) {
					rotor_position_target = rotor_position_target_prev
							- 2 * (rotor_position_target - rotor_position_target_prev) /  (LIMITER_SLOPE*LIMITER_THRESHOLD);
				}
				if (slope_prev > 0 && slope == 0) {
					rotor_position_target = rotor_position_target_prev
							- 2 * (rotor_position_target - rotor_position_target_prev) /  (LIMITER_SLOPE*LIMITER_THRESHOLD);
				}
				slope_prev = slope;
			}

			if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 1){
			sprintf(msg, "%i\t%i\t%i\t%i\r\n", cycle_period_sum,
					rotor_position, rotor_target_in,chirp_cycle);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}

			if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0){
			sprintf(msg, "%i\t%i\t%i\t%i\r\n", cycle_period_sum,
					rotor_position, rotor_target_in,100*(int)(rotor_track_comb_command));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}

			if (i % 10 != 0 && enable_high_speed_sampling == 0){
				HAL_Delay(CYCLE_DELAY);
			}

			if (enable_disturbance_rejection_step == 1) { display_parameter = rotor_position; }
			else if (enable_noise_rejection_step == 1) { noise_rej_signal = rotor_position_target/10; }
			else if (enable_sensitivity_fnc_step == 1)  { display_parameter = rotor_position_command - rotor_position; }
			else { display_parameter = rotor_position; }


			if (enable_noise_rejection_step == 1){
				noise_rej_signal_filter =  noise_rej_signal*iir_0 + noise_rej_signal_prev*iir_1 - noise_rej_signal_filter_prev*iir_2;
				noise_rej_signal_filter_prev = noise_rej_signal_filter;
				noise_rej_signal_prev = noise_rej_signal;
				display_parameter = noise_rej_signal_filter;
			}


			if (i % 10 == 0 && enable_high_speed_sampling == 0){
				report_mode++;
				/*
				 * Provide report each 10th control cycle
				 */
				if (enable_rotor_tracking_comb_signal == 0 && report_mode != 0){
				sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%.1f\t%i\t%i\t%i\r\n", cycle_period_sum,
						(int) (tick_cycle_current - tick_cycle_previous),
						encoder_position, display_parameter, (int)(pid_filter->control_output)/10,
						rotor_position_command, chirp_cycle, rotor_position_target/10,
						(int)(rotor_pid->control_output)/10);
				}

				/*
				 * Provide report each 200th cycle of system parameters
				 */

				if (enable_rotor_tracking_comb_signal == 0 && report_mode == 20){
				sprintf(msg, "%i\t%i\t%i\t%i\t%.1f\t%.1f\t%.1f\t%i\t%i\r\n", 0,
						(int)pid_filter->p_gain, (int)pid_filter->i_gain, (int)pid_filter->d_gain,
						rotor_pid->p_gain, rotor_pid->i_gain, rotor_pid->d_gain,
						max_speed, min_speed);
				}

				/*
				 * Provide report each 400th cycle of system parameters
				 */

				if (enable_rotor_tracking_comb_signal == 0 && report_mode == 40){
				sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\r\n", 1,
						(int)torq_current_val, max_accel, max_decel, enable_disturbance_rejection_step,
						enable_noise_rejection_step, enable_rotor_position_step_response_cycle,
						(int)(adjust_increment*10), enable_sensitivity_fnc_step);
						report_mode = 0;
				}
				/*
				 * If Rotor Tracking Comb Signal enabled, report at rotor_position_command
				 * is replaced with rotor_track_comb_command
				 */

				if (enable_rotor_tracking_comb_signal == 1 && enable_high_speed_sampling == 0){
				sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%.1f\t%i\t%i\t%i\r\n", cycle_period_sum,
						(int) (tick_cycle_current - tick_cycle_previous),
						encoder_position, rotor_position, (int)(pid_filter->control_output)/10,
						rotor_track_comb_command, chirp_cycle, rotor_position_target/10,
						(int)(rotor_pid->control_output)/10);
				}

			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}
			/*
			 * Limit maximum excursion in rotor angle at each cycle step
			 */

			if (ACCEL_CONTROL == 0) {
				rotor_position_delta = ROTOR_POSITION_MAX_DIFF;
				rotor_position_target_curr = rotor_position_target;
				if ((rotor_position_target_curr - rotor_position_target_prev)
						< -rotor_position_delta) {
					rotor_position_target = rotor_position_target_prev
							- rotor_position_delta;
				} else if ((rotor_position_target_curr - rotor_position_target_prev)
						> rotor_position_delta) {
					rotor_position_target = rotor_position_target_prev
							+ rotor_position_delta;
				}
			}

			rotor_position_target_prev = rotor_position_target;
			rotor_position_command_prev = rotor_position_command;

			if (ACCEL_CONTROL == 1) {
				apply_acceleration(rotor_position_target, &target_velocity_prescaled, SAMPLE_FREQUENCY);
			} else {
				BSP_MotorControl_GoTo(0, rotor_position_target/2);
			}
		}

	/*
	 * Control System Exit Loop
	 */
		if (ACCEL_CONTROL == 1) {
			desired_pwm_period = 0;
			current_pwm_period = 0;
		}

		/*
		 * Restore rotor position at low speed profile
		 */

		ret = rotor_position_read(&rotor_position);
		BSP_MotorControl_GoTo(0, 0);
		BSP_MotorControl_SoftStop(0);

		/*
		 * Terminate motor control
		 */

		ret = rotor_position_read(&rotor_position);
		sprintf(msg,"Exit Control at Rotor Angle, %.2f\r\n",
				(float) ((rotor_position)
						/ STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
}


/**
 * @brief  extract a message from a circular buffer
 * @param  CircularBuff circular buffer with data
 * @param  InitPos first byte of the message
 * @param  LastPos last added byte position
 * @param  BufMaxLen buffer length
 * @param  Msg pointer to the protocol message
 * @retval Number of bytes read if the msg is finished
 */
uint16_t Extract_Msg(uint8_t *CircularBuff, uint16_t StartPos, uint16_t LastPos,
		uint16_t BufMaxLen, T_Serial_Msg *Msg) {
	/* Number of bytes to be analyzed */
	uint16_t NumNewByte = 0;
	/* Byte to be analyzed*/
	uint8_t Data;
	/* Circular buffer index */
	uint16_t MsgIdx;
	/* Two index for ByteStuffing process  */
	uint16_t BuffIdx;

	if (LastPos >= StartPos) {
		NumNewByte = LastPos - StartPos;
	} else {
		NumNewByte = BufMaxLen + LastPos - StartPos;
	}
	BuffIdx = StartPos;

	for (MsgIdx = 0; MsgIdx < NumNewByte; MsgIdx++) {
		Data = CircularBuff[BuffIdx];
		BuffIdx++;
		if (BuffIdx >= BufMaxLen) {
			BuffIdx = 0;
		}

		/* If End of message is found, start to recompose the message */
		if (Data == SERIAL_MSG_EOF) {
			Msg->Len = MsgIdx;
			return MsgIdx + 1;
		} else {
			Msg->Data[MsgIdx] = Data;
		}
	}
	return 0;
}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler(0);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler(0);
	}

}

/* USART2 init function */

static void MX_USART2_UART_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler(0);
	}

	/* USART2 RX DMA Init */
	hdma_usart2_rx.Instance = DMA1_Stream5;
	hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) {
		Error_Handler(0);
	}
	__HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);
}

/**
 * @brief  This function is the User handler for the flag interrupt
 * @param  None
 * @retval None
 */
void MyFlagInterruptHandler(void) {
	/* Get the value of the status register via the L6474 command GET_STATUS */
	uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);

	/* Check HIZ flag: if set, power brigdes are disabled */
	if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ) {
		// HIZ state
		// Action to be customized
	}

	/* Check direction bit */
	if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR) {
		// Forward direction is set
		// Action to be customized
	} else {
		// Backward direction is set
		// Action to be customized
	}

	/* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
	/* This often occures when a command is sent to the L6474 */
	/* while it is in HIZ state */
	if ((statusRegister & L6474_STATUS_NOTPERF_CMD)
			== L6474_STATUS_NOTPERF_CMD) {
		// Command received by SPI can't be performed
		// Action to be customized
	}

	/* Check WRONG_CMD flag: if set, the command does not exist */
	if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD) {
		//command received by SPI does not exist
		// Action to be customized
	}

	/* Check UVLO flag: if not set, there is an undervoltage lock-out */
	if ((statusRegister & L6474_STATUS_UVLO) == 0) {
		//undervoltage lock-out
		// Action to be customized
	}

	/* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
	if ((statusRegister & L6474_STATUS_TH_WRN) == 0) {
		//thermal warning threshold is reached
		// Action to be customized
	}

	/* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
	if ((statusRegister & L6474_STATUS_TH_SD) == 0) {
		//thermal shut down threshold is reached
		// Action to be customized
	}

	/* Check OCD  flag: if not set, there is an overcurrent detection */
	if ((statusRegister & L6474_STATUS_OCD) == 0) {
		//overcurrent detection
		// Action to be customized
	}

}

/**
 * @brief  This function is executed in event of error occurrence.
 * @param  error number of the error event
 * @retval None
 */
void Error_Handler(uint16_t error) {
	/* Backup error number */
	gLastError = error;

	/* Infinite loop */
	while (1) {
	}
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/






