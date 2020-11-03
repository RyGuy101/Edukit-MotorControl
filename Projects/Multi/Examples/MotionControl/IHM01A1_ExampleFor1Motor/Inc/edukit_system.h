




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
 * Control System and Motor Configuration Parameter Definitions
 */

/*
 * Sample rates defined for controller operation
 *
 * Note that these values scale derivative and integral computations
 *
 */

#define ENABLE_800Hz_MODE 0
//#define ENABLE_800Hz
#ifdef ENABLE_800Hz
/* Define value for High Speed System is 833 */
#define SAMPLE_FREQUENCY 833					// Set to 500 for default
/* Define value for High Speed System is 0.00120048019 */
#define T_SAMPLE_ROTOR 0.00120048019			// Set to 0.002 for default
/* Define value for High Speed System is 0.00120048019 */
#define T_SAMPLE 0.00120048019					// Set to 0.002 for default
#endif

#ifndef ENABLE_800Hz
/* Define value for High Speed System is 500 */
#define SAMPLE_FREQUENCY 500					// Set to 500 for default
/* Define value for High Speed System is 0.002 */
#define T_SAMPLE_ROTOR 0.002					// Set to 0.002 for default
/* Define value for High Speed System is 0.002 */
#define T_SAMPLE 0.002						// Set to 0.002 for default
#endif


#define ENABLE_HIGH_SPEED_SAMPLING_MODE 0

//#define STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE 	17.778	//	Stepper response in step value command to rotation in degrees
#define STEPPER_READ_POSITION_STEPS_PER_DEGREE 		8.8889	//	Stepper position read value in steps per degree
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

#define TORQ_CURRENT_DEFAULT 800				// Default torque current


#define ROTOR_POSITION_MAX_DIFF 500 			// Limit of maximum permitted difference in motor command values on successive cycles

#define ENABLE_SUSPENDED_PENDULUM_CONTROL 0     // Set to 0 for Inverted Pendulum Mode - Set to 1 for Suspended Pendulum Mode

#define ENCODER_START_OFFSET 0 					// Encoder configurations may display up to 1 degree initial offset
#define ENCODER_START_OFFSET_DELAY 0			// Encoder offset delay limits application of initial offset
#define START_ANGLE 3							// Pendulum angle tolerance for system pendulum orientation at start


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
#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY 10  		// 10 - Corner frequency of low pass filter of Primary PID derivative
#define LP_CORNER_FREQ_ROTOR 20 						// 20 - Corner frequency of low pass filter of Rotor Angle
#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR 50 	// 50 - Corner frequency of low pass filter of Secondary PID derivative
#define LP_NOISE_REJECTION_FILTER 25					// 10 - Corner frequency of low pass filter operating on noise rejection signal


#define LQR_INTEGRAL_ENABLE 0							// Enables LQR mode including integral of rotor position error

#define PRIMARY_PROPORTIONAL_MODE_2 	39.2
#define PRIMARY_INTEGRAL_MODE_2     	0
#define PRIMARY_DERIVATIVE_MODE_2   	5.33

#define SECONDARY_PROPORTIONAL_MODE_2 	1.12
#define SECONDARY_INTEGRAL_MODE_2     	0
#define SECONDARY_DERIVATIVE_MODE_2   	1.55


#define PRIMARY_PROPORTIONAL_MODE_3 	39.2
#define PRIMARY_INTEGRAL_MODE_3     	0.0
#define PRIMARY_DERIVATIVE_MODE_3   	5.33

#define SECONDARY_PROPORTIONAL_MODE_3 	1.12
#define SECONDARY_INTEGRAL_MODE_3     	0.0
#define SECONDARY_DERIVATIVE_MODE_3   	1.55

#define PRIMARY_PROPORTIONAL_MODE_5 	39.2
#define PRIMARY_INTEGRAL_MODE_5     	0.0
#define PRIMARY_DERIVATIVE_MODE_5   	5.33

#define SECONDARY_PROPORTIONAL_MODE_5 	1.12
#define SECONDARY_INTEGRAL_MODE_5     	0.0
#define SECONDARY_DERIVATIVE_MODE_5   	1.55

/*
 * Motor Control Configuration LQR Mode 3
 */

#define PRIMARY_PROPORTIONAL_MODE_1 	39.2
#define PRIMARY_INTEGRAL_MODE_1     	0.0
#define PRIMARY_DERIVATIVE_MODE_1   	5.33

#define SECONDARY_PROPORTIONAL_MODE_1 	1.12
#define SECONDARY_INTEGRAL_MODE_1     	0.0
#define SECONDARY_DERIVATIVE_MODE_1   	1.55

/* Gain values for Pendulum without Mass attached */

#define ROTOR_PID_PROPORTIONAL_GAIN_SINGLE_PID_MODE	1.12
#define ROTOR_PID_INTEGRAL_GAIN_SINGLE_PID_MODE 	0.0
#define ROTOR_PID_DIFFERENTIAL_GAIN_SINGLE_PID_MODE 1.55

/*
 * Suspended Mode with Motor Control Configuration 1 Mode 3
 */

#define PRIMARY_PROPORTIONAL_MODE_4 	-5.36
#define PRIMARY_INTEGRAL_MODE_4     	0.0
#define PRIMARY_DERIVATIVE_MODE_4   	-0.62

#define SECONDARY_PROPORTIONAL_MODE_4 	-0.89
#define SECONDARY_INTEGRAL_MODE_4     	0.0
#define SECONDARY_DERIVATIVE_MODE_4   	-1.01


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

#define ENABLE_CONTROL_ACTION 1							// Enable control operation - default value of 1 (may be disabled for test operations)
#define ENABLE_DUAL_PID 1						// Note ENABLE_DUAL_PID is set to 1 by default for summation of PID controllers
												// for either Dual PID or LQR systems
#define ENABLE_ENCODER_ANGLE_SLOPE_CORRECTION 0			// Set to 1 for PID // 0 LQR // 0 for Susp-Mode
#define LP_CORNER_FREQ_LONG_TERM 0.05					// Set to 0.05 for PID and for LQR
#define ENCODER_ANGLE_SLOPE_CORRECTION_SCALE 200		// Set to 200
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

#define ENCODER_POSITION_POSITIVE_LIMIT  60		// Maximum allowed rotation in positive angle in steps
#define ENCODER_POSITION_NEGATIVE_LIMIT -60		// Minimum allowed rotation in negative angle in steps

/*
 * Setting ENABLE_MOD_SIN_ROTOR_TRACKING to 1 enables a Rotor Position tracking command in the
 * form of an amplitude modulated sine wave signal
 * Frequency units and Rate are Hz
 * Amplitude units are steps
 *
 */

#define ENABLE_MOD_SIN_ROTOR_TRACKING 1		// If selected, disable all other modulation inputs
#define MOD_SIN_CARRIER_FREQ 0.15			// 0.003 default
#define MOD_SIN_START_CYCLES 4000			// 10000 default
#define MOD_SIN_AMPLITUDE 600				// 600 default
#define MOD_SIN_MODULATION_FREQ  0.02		// 0.001 default
#define MOD_SIN_MODULATION_MIN 100			// Default 100
/* Define for High Speed System */
#define MOD_SIN_SAMPLE_RATE 500.0			// 500.0 default for Low Speed
#define ENABLE_SIN_MOD 1					// 1 default

#define ENABLE_ROTOR_CHIRP 0				// If selected, disable all other modulation inputs
#define ROTOR_CHIRP_START_FREQ 0.001		// 0.001 default
#define ROTOR_CHIRP_END_FREQ 20				// 5 default
#define ROTOR_CHIRP_PERIOD 20000			// 20000 default
#define ROTOR_CHIRP_SWEEP_DELAY 5000		// 5000 default - enables control system to recover between sweeps
/* Define for High Speed System */
#define ROTOR_CHIRP_SAMPLE_RATE 500.0		// 500.0 default for Low Speed
#define ROTOR_CHIRP_START_CYCLES 0			// 0 default
#define ROTOR_CHIRP_STEP_AMPLITUDE 1  	// 0.3 default

#define ENABLE_ROTOR_TRACK_COMB_SIGNAL 0				// If selected, disable all other modulation inputs
#define ROTOR_TRACK_COMB_SIGNAL_PERIOD 50000			// 50000 default
#define ROTOR_TRACK_DELAY 10000							// 10000 default - enables control system to recover between sweeps
/* Define for High Speed System */
#define ROTOR_TRACK_COMB_SIGNAL_SAMPLE_RATE 500.0		// 500.0 default for Low Speed
#define ROTOR_TRACK_COMB_SIGNAL_START_CYCLES 0			// 0 default
#define ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE 0.25				// 0.05 default

#define ENABLE_PENDULUM_DISPLACEMENT_DISTURBANCE 0
#define PENDULUM_DISPLACEMENT_DISTURBANCE_CYCLE 10000
#define PENDULUM_DISPLACEMENT_THRESHOLD_ANGLE 3


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

/* Scale rotor tracking command to be applied as disturbance when enable_disturbance_rejection_step == 1 */
#define ROTOR_DISTURBANCE_SCALE 1.0

/*
 * Setting ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE = 1 applies a Rotor Position tracking
 * command input step signal
 */

#define ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE 1			// If selected, disable all other modulation inputs
#define ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE 20		// Default 8. Amplitude of step cycle. Note: Peak-to-Peak amplitude is double this value
#define ROTOR_POSITION_STEP_RESPONSE_CYCLE_INTERVAL 10240 	// Default 10240
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
#define ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL 5000	    // Interval between impulse events in cycles
/* Define for High Speed System */
#define ROTOR_IMPULSE_SAMPLE_RATE 500							// Default sample rate
/*
 * Setting ENABLE_PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE = 1 applies a Pendulum Position tracking
 * command input impulse signal
 */
#define ENABLE_PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE 0		// If selected, disable all other modulation inputs
#define PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE 500	// Amplitude of step cycle. Note: Peak-to-Peak amplitude is double this value
#define PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD 4		// Duration of impulse in cycles
#define PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL 5000	// Interval between impulse events in cycles
/* Define for High Speed System */
#define PENDULUM_IMPULSE_SAMPLE_RATE 500 						// Default sample rate

/*
 * UART DMA definitions
 */

#define UART_RX_BUFFER_SIZE 	(200)
#define SERIAL_MSG_MAXLEN 		(100)
#define SERIAL_MSG_EOF          '\r'

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


extern void pid_filter_value_config(pid_filter_control_parameters * pid_filter);
extern void pid_filter_control_execute(pid_filter_control_parameters * pid_filter,
		float * current_error, float * sample_period, float * f_deriv_lp);

extern int encoder_position_read(int *encoder_position, TIM_HandleTypeDef *htim3);
extern int rotor_position_read(int *rotor_position);

extern void set_mode_strings(void);
extern void get_user_mode_index(char * user_string, int * char_mode_select, int * mode_index, int * mode_interactive);
extern void rotor_position_set(void);
extern void select_mode_1(void);
extern void user_configuration(void);
extern void read_int(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, int * int_return);
extern void read_float(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, float *float_return);
extern void read_char(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, char * char_return);

extern void user_prompt(void);
extern void rotor_actuator_high_speed_test(void);
extern void rotor_encoder_test(void);
extern void pendulum_system_id_test(void);
extern void motor_actuator_characterization_mode(void);
extern void interactive_rotor_actuator_control(void);

extern void user_configuration(void);
extern int mode_index_identification(char * user_config_input, int config_command_control, float *adjust_increment,
		pid_filter_control_parameters * pid_filter, pid_filter_control_parameters * rotor_pid);
extern void assign_mode_1(pid_filter_control_parameters * pid_filter,
		pid_filter_control_parameters * rotor_pid);
extern void assign_mode_2(pid_filter_control_parameters * pid_filter,
		pid_filter_control_parameters * rotor_pid);
extern void assign_mode_3(pid_filter_control_parameters * pid_filter,
		pid_filter_control_parameters * rotor_pid);

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



/* Acceleration control system variables */
volatile uint32_t apply_acc_start_time;
volatile uint32_t clock_int_time;
volatile uint32_t clock_int_tick;

/// PWM period variables used by step interrupt
volatile uint32_t desired_pwm_period;
volatile uint32_t current_pwm_period;

int32_t target_velocity_prescaled;

/* System data reporting */
char tmp_string[256];
char msg[192];
char test_msg[128];

/* System timing variables */

uint32_t tick, tick_cycle_current, tick_cycle_previous, tick_cycle_start,
tick_read_cycle, tick_read_cycle_start,tick_wait_start,tick_wait;

uint64_t tick_cycle_target;
float Tsample, Tsample_rotor, test_time;
float angle_scale;
int enable_high_speed_sampling;


/* Motor configuration */
uint16_t min_speed, max_speed, max_accel, max_decel;

/* Serial interface variables */
uint32_t RxBuffer_ReadIdx;
uint32_t RxBuffer_WriteIdx;
uint32_t readBytes;

/* Control system output signal */
int rotor_control_target_steps;
int rotor_control_target_steps_curr;
int rotor_control_target_steps_prev;

/* Control system variables */
int rotor_position_delta;
int cycle_count;
int i, j, k, m;
int ret;

/* PID control system variables */
float windup, rotor_windup;
float *current_error_steps, *current_error_rotor_steps;
float *sample_period, *sample_period_rotor;

/* Loop timing measurement variables */
int cycle_period_start;
int cycle_period_sum;

/* PID control variables */
float *deriv_lp_corner_f;
float *deriv_lp_corner_f_rotor;
float proportional, rotor_p_gain;
float integral, rotor_i_gain;
float derivative, rotor_d_gain;

/* Reference tracking command */
float reference_tracking_command;

/* Pendulum position and tracking command */

/* Rotor position and tracking command */
int rotor_position_steps;
float rotor_position_command_steps;
float rotor_position_steps_prev, rotor_position_filter_steps, rotor_position_filter_steps_prev;
float rotor_position_diff, rotor_position_diff_prev;
float rotor_position_diff_filter, rotor_position_diff_filter_prev;
int rotor_target_in_steps;

/* Encoder position variables */
int encoder_position;
int encoder_position_down;
int encoder_position_curr;
int encoder_position_prev;

/* Low pass filter variables */
float fo, Wo, IWon, iir_0, iir_1, iir_2, Wo_nr, IWon_nr, iir_0_nr, iir_1_nr, iir_2_nr;
float fo_LT, Wo_LT, IWon_LT, fo_nr;
float iir_LT_0, iir_LT_1, iir_LT_2;

/* Slope correction system variables */
int slope;
int slope_prev;
float encoder_angle_slope_corr_steps;

/* Adaptive control variables */

float adaptive_error, adaptive_threshold_low, adaptive_threshold_high;
float error_sum_prev, error_sum, error_sum_filter_prev, error_sum_filter;
int adaptive_entry_tick, adaptive_dwell_period;
int enable_adaptive_mode, adaptive_state, adaptive_state_change;
float rotor_position_command_steps_prev;

/* Rotor impulse variables */
int rotor_position_step_polarity;
int impulse_start_index;

/* User configuration variables */
uint32_t enable_control_action;
int max_speed_read, min_speed_read;
int select_suspended_mode;
int motor_response_model;
int enable_rotor_actuator_test, enable_rotor_actuator_control;
int enable_encoder_test;
int enable_rotor_actuator_high_speed_test;
int enable_motor_actuator_characterization_mode;
int motor_state;
float torq_current_val;


/* Rotor chirp system variables */
int enable_rotor_chirp;
int chirp_cycle;
int chirp_dwell_cycle;
float chirp_time;
float rotor_chirp_start_freq;
float rotor_chirp_end_freq;
float rotor_chirp_period ;
float rotor_chirp_frequency;
float rotor_chirp_amplitude;
int rotor_chirp_step_period;

float pendulum_position_command_steps;

/* Modulates sine tracking signal system variables */
int enable_mod_sin_rotor_tracking;
int enable_rotor_position_step_response_cycle;
int disable_mod_sin_rotor_tracking;
int sine_drive_transition;
float mod_sin_amplitude;
float rotor_control_sin_amplitude;
float rotor_sine_drive, rotor_sine_drive_mod;
float rotor_mod_control;
float mod_sin_carrier_frequency;

/* Pendulum impulse system variables */
int enable_pendulum_position_impulse_response_cycle;

/* Rotor hish speed test system variables */
int swing_cycles, rotor_test_speed_min, rotor_test_speed_max;
int rotor_test_acceleration_max, swing_deceleration_max;
int start_angle_a[20], end_angle_a[20], motion_dwell_a[20];
int abs_encoder_position_prior, abs_encoder_position_after, abs_encoder_position_max;
uint16_t current_speed;

/*Pendulum system ID variable */
int enable_pendulum_sysid_test;

/* Rotor comb drive system variables */
int enable_rotor_tracking_comb_signal;
float rotor_track_comb_signal_frequency;
float rotor_track_comb_command;
float rotor_track_comb_amplitude;

/* Sensitivity function system variables */
int enable_disturbance_rejection_step;
int enable_noise_rejection_step;
int enable_plant_rejection_step;
int enable_sensitivity_fnc_step;



/* Noise rejection sensitivity function low pass filter */

float noise_rej_signal_filter, noise_rej_signal;
float noise_rej_signal_prev, noise_rej_signal_filter_prev;

/*
 * Real time user input system variables
 */

char config_message[16];
int config_command;
int display_parameter;
int step_size;
float adjust_increment;
int mode_index;

/* Real time data reporting index */
int report_mode;

/*
 * User selection mode values
 */

int mode_1;				// Enable LQR Motor Model M
int mode_2;				// Enable LRR Motor Model H
int mode_3;				// Enable LQR Motor Model L
int mode_4;				// Enable Suspended Mode Motor Model M
int mode_5;				// Enable sin drive track signal
int mode_adaptive_off;	// Disable adaptive control
int mode_adaptive;		// Enable adaptive control
int mode_8;				// Enable custom configuration entry
int mode_9;				// Disable sin drive track signal
int mode_10;			// Enable Single PID Mode with Motor Model M
int mode_11;			// Enable rotor actuator and encoder test mode
int mode_12;			// Enable rotor actuator and encoder high speed test mode
int mode_13;			// Enable rotor control system evaluation
int mode_14;			// Enable pendlum system identification
int mode_15;			// Enable interactive control of rotor actuator
int mode_16;			// Enable load disturbance function step mode
int mode_17;			// Enable noise disturbance function step mode
int mode_18;			// Enable sensitivity function step mode
int mode_quit;			// Initiate exit from control loop
int mode_interactive;		// Enable continued terminal interactive user session
int mode_index_prev, mode_index_command;
int mode_transition_tick;
int mode_transition_state;
int transition_to_adaptive_mode;


/*
 * Real time user input characters
 */

char mode_string_stop[UART_RX_BUFFER_SIZE];
char mode_string_mode_1[UART_RX_BUFFER_SIZE];
char mode_string_mode_2[UART_RX_BUFFER_SIZE];
char mode_string_mode_3[UART_RX_BUFFER_SIZE];
char mode_string_mode_4[UART_RX_BUFFER_SIZE];
char mode_string_mode_8[UART_RX_BUFFER_SIZE];
char mode_string_mode_5[UART_RX_BUFFER_SIZE];
char mode_string_inc_accel[UART_RX_BUFFER_SIZE];
char mode_string_dec_accel[UART_RX_BUFFER_SIZE];
char mode_string_inc_amp[UART_RX_BUFFER_SIZE];
char mode_string_dec_amp[UART_RX_BUFFER_SIZE];
char mode_string_mode_single_pid[UART_RX_BUFFER_SIZE];
char mode_string_mode_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_control[UART_RX_BUFFER_SIZE];
char mode_string_mode_high_speed_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_motor_characterization_mode[UART_RX_BUFFER_SIZE];
char mode_string_mode_pendulum_sysid_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_load_dist[UART_RX_BUFFER_SIZE];
char mode_string_mode_load_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_noise_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_plant_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_dec_pend_p[UART_RX_BUFFER_SIZE];
char mode_string_inc_pend_p[UART_RX_BUFFER_SIZE];
char mode_string_dec_pend_i[UART_RX_BUFFER_SIZE];
char mode_string_inc_pend_i[UART_RX_BUFFER_SIZE];
char mode_string_dec_pend_d[UART_RX_BUFFER_SIZE];
char mode_string_inc_pend_d[UART_RX_BUFFER_SIZE];
char mode_string_dec_rotor_p[UART_RX_BUFFER_SIZE];
char mode_string_inc_rotor_p[UART_RX_BUFFER_SIZE];
char mode_string_dec_rotor_i[UART_RX_BUFFER_SIZE];
char mode_string_inc_rotor_i[UART_RX_BUFFER_SIZE];
char mode_string_dec_rotor_d[UART_RX_BUFFER_SIZE];
char mode_string_inc_rotor_d[UART_RX_BUFFER_SIZE];
char mode_string_dec_torq_c[UART_RX_BUFFER_SIZE];
char mode_string_inc_torq_c[UART_RX_BUFFER_SIZE];
char mode_string_dec_max_s[UART_RX_BUFFER_SIZE];
char mode_string_inc_max_s[UART_RX_BUFFER_SIZE];
char mode_string_dec_min_s[UART_RX_BUFFER_SIZE];
char mode_string_inc_min_s[UART_RX_BUFFER_SIZE];
char mode_string_dec_max_a[UART_RX_BUFFER_SIZE];
char mode_string_inc_max_a[UART_RX_BUFFER_SIZE];
char mode_string_dec_max_d[UART_RX_BUFFER_SIZE];
char mode_string_inc_max_d[UART_RX_BUFFER_SIZE];
char mode_string_enable_step[UART_RX_BUFFER_SIZE];
char mode_string_disable_step[UART_RX_BUFFER_SIZE];
char mode_string_enable_pendulum_impulse[UART_RX_BUFFER_SIZE];
char mode_string_disable_pendulum_impulse[UART_RX_BUFFER_SIZE];
char mode_string_enable_load_dist[UART_RX_BUFFER_SIZE];
char mode_string_disable_load_dist[UART_RX_BUFFER_SIZE];
char mode_string_enable_noise_rej_step[UART_RX_BUFFER_SIZE];
char mode_string_disable_noise_rej_step[UART_RX_BUFFER_SIZE];
char mode_string_disable_sensitivity_fnc_step[UART_RX_BUFFER_SIZE];
char mode_string_enable_sensitivity_fnc_step[UART_RX_BUFFER_SIZE];
char mode_string_inc_step_size[UART_RX_BUFFER_SIZE];
char mode_string_dec_step_size[UART_RX_BUFFER_SIZE];
char mode_string_select_mode_5[UART_RX_BUFFER_SIZE];
char mode_string_enable_high_speed_sampling[UART_RX_BUFFER_SIZE];
char mode_string_disable_high_speed_sampling[UART_RX_BUFFER_SIZE];

int char_mode_select;	// Flag detecting whether character mode select entered


char message_received[UART_RX_BUFFER_SIZE];
char mode_string_mode_1[UART_RX_BUFFER_SIZE];
char mode_string_mode_2[UART_RX_BUFFER_SIZE];
char mode_string_mode_3[UART_RX_BUFFER_SIZE];
char mode_string_mode_4[UART_RX_BUFFER_SIZE];
char mode_string_mode_5[UART_RX_BUFFER_SIZE];
char mode_string_mode_8[UART_RX_BUFFER_SIZE];
char mode_string_mode_single_pid[UART_RX_BUFFER_SIZE];
char mode_string_mode_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_control[UART_RX_BUFFER_SIZE];
char mode_string_mode_high_speed_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_motor_characterization_mode[UART_RX_BUFFER_SIZE];
char mode_string_mode_pendulum_sysid_test[UART_RX_BUFFER_SIZE];
char mode_string_dec_accel[UART_RX_BUFFER_SIZE];
char mode_string_inc_accel[UART_RX_BUFFER_SIZE];
char mode_string_inc_amp[UART_RX_BUFFER_SIZE];
char mode_string_dec_amp[UART_RX_BUFFER_SIZE];
char mode_string_mode_load_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_noise_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_plant_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_stop[UART_RX_BUFFER_SIZE];

