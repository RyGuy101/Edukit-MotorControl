




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
#include "edukit_system.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>


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
 * Apply acceleration
 */

#define ACCEL_CONTROL_DATA 0	// Set to 1 for display of timing data
#define ACCEL_CONTROL 1 // Set to 1 to enable acceleration control. Set to 0 to use position target control.
#define PWM_COUNT_SAFETY_MARGIN 2
#define MAXIMUM_ACCELERATION 65535
#define MAXIMUM_DECELERATION 65535
#define MIN_POSSIBLE_SPEED 1 // Should be at least 2, as per L6474_MIN_PWM_FREQ in l6474.c
#define MAXIMUM_SPEED 65535
#define DELAY_TOLERANCE 32000

static volatile uint16_t gLastError;
/* Private function prototypes -----------------------------------------------*/
static void MyFlagInterruptHandler(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
void read_float(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, float *float_return);
void Error_Handler(uint16_t error);
void read_int(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, int * int_return);
void read_char(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, char * char_return);
void select_mode_1(void);
void user_configuration(void);
int Delay_Pulse();
void Main_StepClockHandler();
void apply_acceleration(int32_t acc, int32_t* target_velocity_prescaled, uint16_t sample_freq_hz);


int Delay_Pulse(){
	if (desired_pwm_period < DELAY_TOLERANCE){
		return 0;
	} else {
		return 1;
	}
}

/*
 * PWM pulse (step) interrupt
 */
void Main_StepClockHandler() {
	/*
	 *  Stepper motor acceleration, speed, direction and position control developed by Ryan Nemiroff
	 */

	volatile uint32_t *DWT_CYCCNT   = (volatile uint32_t *)0xE0001004;
	uint32_t desired_pwm_period_local = desired_pwm_period;

	/*
	 * Add time reporting
	 */
	clock_int_time = *DWT_CYCCNT;

	if (desired_pwm_period_local != 0) {
		L6474_Board_Pwm1SetPeriod(desired_pwm_period_local);
		current_pwm_period = desired_pwm_period_local;
	}
}

/*
 *  Apply acceleration to stepper motor, with acc in steps/s^2
 */
void apply_acceleration(int32_t acc, int32_t* target_velocity_prescaled, uint16_t sample_freq_hz) {
	/*
	 *  Stepper motor acceleration, speed, direction and position control developed by Ryan Nemiroff
	 */
	volatile uint32_t *DWT_CYCCNT   = (volatile uint32_t *)0xE0001004;

	uint32_t current_pwm_period_local = current_pwm_period;
	uint32_t desired_pwm_period_local = desired_pwm_period;

	/*
	 * Add time reporting
	 */
	apply_acc_start_time = *DWT_CYCCNT;


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


int main(void) {

	static pid_filter_control_parameters *pid_filter;
	static pid_filter_control_parameters *rotor_pid;

	/* Initialize and enable cycle counter */

	volatile uint32_t *DWT_CYCCNT   = (volatile uint32_t *)0xE0001004;
	volatile uint32_t *DWT_CONTROL  = (volatile uint32_t *)0xE0001000;
	volatile uint32_t *DWT_LAR      = (volatile uint32_t *)0xE0001FB0;
	volatile uint32_t *SCB_DEMCR    = (volatile uint32_t *)0xE000EDFC;
	*DWT_LAR = 0xC5ACCE55;
	*SCB_DEMCR |= 0x01000000;
	*DWT_CONTROL |= 1 ;
	*DWT_CYCCNT = 0;
	/* initialize Integrator Mode time variables */
	apply_acc_start_time = 0;
	clock_int_time = 0;
	clock_int_tick = 0;
	/* Initialize PWM period variables used by step interrupt */
	desired_pwm_period = 0;
	current_pwm_period = 0;
	target_velocity_prescaled = 0;
	/* Initialize default start mode and reporting mode */
	mode_index = 1;
	report_mode = 0;
	/*Initialize serial read variables */
	RxBuffer_ReadIdx = 0;
	RxBuffer_WriteIdx = 0;
	readBytes = 0;
	/*Initialize encoder variables */
	encoder_position = 0;
	encoder_position_down = 0;
	encoder_position_curr = 0;
	encoder_position_prev = 0;
	angle_scale = ENCODER_READ_ANGLE_SCALE;
	/*Initialize rotor control variables */
	rotor_control_target_steps = 0;
	rotor_control_target_steps_curr = 0;
	rotor_control_target_steps_prev = 0;
	/*Initialize rotor tracking signal variables */
	enable_rotor_chirp = 0;
	rotor_chirp_start_freq = ROTOR_CHIRP_START_FREQ;
	rotor_chirp_end_freq = ROTOR_CHIRP_END_FREQ;
	rotor_chirp_period = ROTOR_CHIRP_PERIOD;
	enable_mod_sin_rotor_tracking = ENABLE_MOD_SIN_ROTOR_TRACKING;
	enable_rotor_position_step_response_cycle = ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE;
	disable_mod_sin_rotor_tracking = 0;
	sine_drive_transition = 0;
	mod_sin_amplitude = MOD_SIN_AMPLITUDE;
	rotor_control_sin_amplitude = MOD_SIN_AMPLITUDE;
	/*Initialize sensitivity function selection variables */
	enable_disturbance_rejection_step = 0;
	enable_noise_rejection_step = 0;
	enable_sensitivity_fnc_step = 0;
	enable_pendulum_position_impulse_response_cycle = 0;
	/*Initialize user adjustment variables */
	step_size = 0;
	adjust_increment = 0.5;
	/*Initialize adaptive mode state variables */
	mode_transition_state = 0;
	transition_to_adaptive_mode = 0;
	/*Initialize user interactive mode */
	char_mode_select = 0;
	/* STM32xx HAL library initialization */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Default select_suspended_mode */
	select_suspended_mode = ENABLE_SUSPENDED_PENDULUM_CONTROL;
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
	/* Initialize Timer and UART */
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	/* Motor Range Initialization */
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
	/* Default Starting Control Configuration */
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
	/* Default torque current */
	torq_current_val = 800;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
	/* Default controller gains */
	proportional = PRIMARY_PROPORTIONAL_MODE_1;
	integral = PRIMARY_INTEGRAL_MODE_1;
	derivative = PRIMARY_DERIVATIVE_MODE_1;
	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;
	/* Disable adaptive_mode by default */
	enable_adaptive_mode = 0;
	/* DMA Buffer declarations */
	/* Start DMA just once because it's configured in "circular" mode */
	HAL_UART_Receive_DMA(&huart2, RxBuffer, UART_RX_BUFFER_SIZE);
	/* Motor Interface and Encoder initialization */
	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);
	/* Attach the function Error_Handler (defined below) to the error Handler*/
	BSP_MotorControl_AttachErrorHandler(Error_Handler);
	/* Encoder inititialization */
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	/* Assign user interaction mode string values */
	set_mode_strings();
	/* Controller structure and variable allocation */

	current_error_steps = malloc(sizeof(float));
	if (current_error_steps == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	current_error_rotor_steps = malloc(sizeof(float));
	if (current_error_rotor_steps == NULL) {
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

	/* Initialize Pendulum and Rotor PID Controller structs */
	pid_filter_value_config(pid_filter);
	pid_filter_value_config(rotor_pid);
	/* Configure primary controller parameters */
	windup = PRIMARY_WINDUP_LIMIT;
	/* Configure secondary Rotor controller parameters */
	rotor_windup = SECONDARY_WINDUP_LIMIT;
	/* Configure controller filter and sample time parameters */
	*deriv_lp_corner_f = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
	*deriv_lp_corner_f_rotor = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	Tsample = T_SAMPLE;
	*sample_period = Tsample;
	Tsample_rotor = T_SAMPLE_ROTOR;
	*sample_period_rotor = Tsample_rotor;
	/* Compute Low Pass Filter Coefficients for Rotor Position filter and Encoder Angle Slope Correction */
	fo = LP_CORNER_FREQ_ROTOR;
	Wo = 2 * 3.141592654 * fo;
	IWon = 2 / (Wo * Tsample);
	iir_0 = 1 / (1 + IWon);
	iir_1 = iir_0;
	iir_2 = iir_0 * (1 - IWon);
	fo_nr = LP_NOISE_REJECTION_FILTER;
	Wo_nr = 2 * 3.141592654 * fo_nr;
	IWon_nr = 2 / (Wo_nr * Tsample);
	iir_0_nr = 1 / (1 + IWon_nr);
	iir_1_nr = iir_0_nr;
	iir_2_nr = iir_0_nr * (1 - IWon_nr);
	fo_LT = LP_CORNER_FREQ_LONG_TERM;
	Wo_LT = 2 * 3.141592654 * fo_LT;
	IWon_LT = 2 / (Wo_LT * Tsample);
	iir_LT_0 = 1 / (1 + IWon_LT);
	iir_LT_1 = iir_LT_0;
	iir_LT_2 = iir_LT_0 * (1 - IWon_LT);

	/*
	 * Primary Controller Mode Configuration Loop
	 *
	 * Outer Loop acquires configuration command
	 * Inner Loop includes control system
	 * Exits to Outer Loop upon command or exceedance
	 * of rotor or pendulum angles
	 *
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
		user_prompt();

		/*
		 * If user has responded to previous query for configuration, then system remains in interactive mode
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

		/* Flush read buffer  */
		for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }
		/* Start timer for onfiguration command read loop */
		tick_read_cycle_start = HAL_GetTick();
		/* Configuration command read loop */
		user_configuration();

		/* Set Motor Speed Profile and torque current */
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
		BSP_MotorControl_SetMaxSpeed(0, max_speed);
		BSP_MotorControl_SetMinSpeed(0, min_speed);
		BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
		BSP_MotorControl_SetDeceleration(0, MAX_DECEL);

		/* Report configuration values */
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

		/* Pendulum System Identification Test */
		if (enable_pendulum_sysid_test == 1){
			pendulum_system_id_test();
		}
		/* Motor Control Characterization Test*/
		if (enable_motor_actuator_characterization_mode == 1) {
			motor_actuator_characterization_mode();
		}
		/* Interactive digital motor control system */
		if (enable_rotor_actuator_control == 1) {
			interactive_rotor_actuator_control();
		}
		/* High speed rotor actuator test (optional)*/
		if (enable_rotor_actuator_high_speed_test == 1) {
			rotor_actuator_high_speed_test();
		}

		/*
		 * 	Rotor and Encoder Test Sequence will execute by moving rotor and reportin angle values
		 * 	as well as requesting pendulum motion followed by reporting of pendulum angles
		 *
		 * 	Agreement between actions and reported values confirms proper installation of actuator
		 * 	and pendulum encoder.
		 *
		 */

		if (enable_rotor_actuator_test == 1) {
			rotor_encoder_test();
		}

		/* Configure Primary and Secondary PID controller data structures */
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
		 * Initiate start of Control System
		 */

		/* Setting enable_control_action enables control loop */
		enable_control_action = ENABLE_CONTROL_ACTION;

		/* Set Motor Position Zero */

		rotor_position_set();
		ret = rotor_position_read(&rotor_position_steps);
		sprintf(msg,
				"\r\nPrepare for Control Start - Initial Rotor Position: %i\r\n",
				rotor_position_steps);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);


		/*
		 * Determination of vertical down orientation of the pendulum is required
		 * to establish the reference angle for the vertical upward control setpoint.
		 *
		 * This is measured when the pendulum is determined to be motionless.
		 *
		 * The user is informed to allow the pendulum to remain at rest.
		 *
		 * Motion is detected in the loop below.  Exit from the loop and
		 * initiation of control occurs next.
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
			/* Alert user of undesired motion */
			sprintf(msg, "Pendulum Motion Detected with angle %0.2f - Stabilize Pendulum Now\r\n",
					(float) ((encoder_position_curr - encoder_position_prev)
							/ ENCODER_READ_ANGLE_SCALE));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
		}

		sprintf(msg, "Pendulum Now at Rest and Measuring Pendulum Down Angle\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		/* Calibrate down angle */

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
		 * Alert user with rotor motion prompt to adjust pendulum upright by
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

		/* Request user action to bring pendulum upright */
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
		 * then disable control and enable system restart.
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
					enable_control_action = 0;
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
		ret = rotor_position_read(&rotor_position_steps);

		sprintf(msg, "Initial Rotor Position: %i\r\n", rotor_position_steps);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		/*
		 * Initialize Primary and Secondary PID controllers
		 */

		*current_error_steps = 0;

		// Temporary fix to account for change in unit conversions:
		//float p_gain = pid_filter->p_gain;
		//float i_gain = pid_filter->i_gain;
		//float d_gain = pid_filter->d_gain;
		//if (ACCEL_CONTROL == 0) {
		//	pid_filter->p_gain /= STEPPER_READ_POSITION_STEPS_PER_DEGREE;
		//	pid_filter->i_gain /= STEPPER_READ_POSITION_STEPS_PER_DEGREE;
		//	pid_filter->d_gain /= STEPPER_READ_POSITION_STEPS_PER_DEGREE;
		//}

		/* Initialize Pendulum PID control state */
		pid_filter_control_execute(pid_filter, current_error_steps, sample_period,
				deriv_lp_corner_f);

		// Temporary fix to account for change in unit conversions:
		//if (ACCEL_CONTROL == 0) {
		//	pid_filter->p_gain = p_gain;
		//	 pid_filter->i_gain = i_gain;
		//	pid_filter->d_gain = d_gain;
		//}

		/* Initialize Rotor PID control state */
		*current_error_rotor_steps = 0;
		pid_filter_control_execute(rotor_pid, current_error_rotor_steps,
				sample_period_rotor, deriv_lp_corner_f_rotor);


		/* Initialize control system variables */
		cycle_count = CYCLE_LIMIT;
		i = 0;
		rotor_position_steps = 0;
		rotor_position_steps_prev = 0;
		rotor_position_filter_steps = 0;
		rotor_position_filter_steps_prev = 0;
		rotor_position_command_steps = 0;
		rotor_position_diff = 0;
		rotor_position_diff_prev = 0;
		rotor_position_diff_filter = 0;
		rotor_position_diff_filter_prev = 0;
		rotor_position_step_polarity = 1;
		encoder_angle_slope_corr = 0;
		rotor_sine_drive = 0;
		sine_drive_transition = 0;
		rotor_mod_control = 1.0;
		enable_adaptive_mode = 0;
		tick_cycle_start = HAL_GetTick();
		tick_cycle_previous = tick_cycle_start;
		tick_cycle_current =  tick_cycle_start;
		tick_cycle_target = (uint64_t) tick_cycle_start << 10;
		chirp_cycle = 0;
		chirp_dwell_cycle = 0;
		pendulum_position_command_steps = 0;
		impulse_start_index = 0;
		mode_transition_state = 0;
		transition_to_adaptive_mode = 0;
		error_sum_prev = 0;
		error_sum_filter_prev = 0;
		adaptive_state = 4;
		rotor_control_target_steps_prev = 0;
		rotor_position_command_steps_prev = 0;
		enable_high_speed_sampling = ENABLE_HIGH_SPEED_SAMPLING_MODE;
		slope_prev = 0;
		rotor_track_comb_command = 0;
		noise_rej_signal_prev = 0;
		noise_rej_signal_filter_prev = 0;

		/* Clear read buffer */
		for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
			Msg.Data[k] = 0;
		}
		__HAL_DMA_RESET_HANDLE_STATE(&hdma_usart2_rx);
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

		while (enable_control_action == 1) {

			/*
			 *  Real time user configuration and mode assignment
			 */

			mode_index_prev = mode_index;

			RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE
					- __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
			readBytes = Extract_Msg(RxBuffer, RxBuffer_ReadIdx,
					RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

			config_command = 0;
			/* Test for message received */
			if (readBytes == 2 && Msg.Len == 1 && i % 10 == 0){
				RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;
				mode_transition_state = 1;
				/* Determine user input */
				mode_index_command = mode_index_identification((char *)Msg.Data, config_command, & adjust_increment,
						pid_filter, rotor_pid);
				strcpy(config_message, (char *) Msg.Data);
				if (strcmp(config_message, "q") == 0){
					sprintf(tmp_string,
							"\n\rExit Control Loop Command Received ");
					HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string,
							strlen(tmp_string), HAL_MAX_DELAY);
					break;
				}
			}

			/* Set mode 1 if user request detected */
			if (mode_index_command == 1 && mode_transition_state == 1) {
				mode_index = 1;
				mode_transition_state = 0;
				mode_index_command = 0;
				assign_mode_1(pid_filter, rotor_pid);
			}
			/* Set mode 3 if user request detected */
			if (mode_index_command == 2 && mode_transition_state == 1) {
				mode_index = 2;
				mode_transition_state = 0;
				mode_index_command = 0;
				assign_mode_2(pid_filter, rotor_pid);
			}
			/* Set mode 3 if user request detected */
			if (mode_index_command == 3 && mode_transition_state == 1) {
				mode_index = 3;
				mode_transition_state = 0;
				mode_index_command = 0;
				assign_mode_3(pid_filter, rotor_pid);
			}
			/* End of Real time user configuration and mode assignment read loop */



			/* Exit control if cycle count limit set */
			i++;
			if (i > cycle_count && ENABLE_CYCLE_INFINITE == 0) {
				break;
			}

			// Delay to maintain sample rate
			tick_cycle_target += (uint32_t) (T_SAMPLE * 1000 * (1<<10)); // Give tick_cycle_target precision similar to microseconds
			int cycle_delay = (int) (tick_cycle_target >> 10) - HAL_GetTick();
			if (cycle_delay >= 1) {
				HAL_Delay(cycle_delay);
			}

			/******************************
			 * BEGIN MEASUREMENT & CONTROL
			 ******************************/

			/*
			 * Acquire encoder position and correct for initial angle value of encoder measured at
			 * vertical down position at system start and 180 degree offset corresponding to
			 * vertical upwards.
			 *
			 * For case of Suspended Mode Operation, 180 degree offset is not applied
			 */

			ret = encoder_position_read(&encoder_position, &htim3);
			if (select_suspended_mode == 0) {
				encoder_position = encoder_position - encoder_position_down - (int) ((round)(180 * angle_scale));
			}
			if (select_suspended_mode == 1) {
				encoder_position = encoder_position - encoder_position_down;
			}

			/*  Detect pendulum position excursion exceeding limits and exit */

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

			/* Detect rotor position excursion exceeding limits and exit */

			if (rotor_position_steps
					> (ROTOR_POSITION_POSITIVE_LIMIT
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
				sprintf(msg, "Error Exit Motor Position Exceeded: %i\r\n",
						rotor_position_steps);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
						HAL_MAX_DELAY);
				break;
			}

			if (rotor_position_steps
					< (ROTOR_POSITION_NEGATIVE_LIMIT
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
				sprintf(msg, "Error Exit Motor Position Exceeded: %i\r\n",
						rotor_position_steps);
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
				rotor_position_diff = rotor_position_filter_steps
						- rotor_position_command_steps;
			}
			if (enable_disturbance_rejection_step == 1){
				rotor_position_diff = rotor_position_filter_steps;
			}
			rotor_position_diff_filter_prev = rotor_position_diff_filter;
			rotor_position_diff_filter =
					(float) (rotor_position_diff * iir_LT_0)
					+ rotor_position_diff_prev * iir_LT_1
					- rotor_position_diff_filter_prev * iir_LT_2;

			/* Apply slope correction */
			if (ENABLE_ENCODER_ANGLE_SLOPE_CORRECTION == 1 ) {
				if ((i < ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT) || (ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT == 0)) {
					encoder_angle_slope_corr = (rotor_position_diff_filter / STEPPER_READ_POSITION_STEPS_PER_DEGREE) / ENCODER_ANGLE_SLOPE_CORRECTION_SCALE;
				}
			}

			/*
			 *  Compute current_error_steps input for Primary Controller
			 *
			 *  current_error_steps is sum of encoder angle error compensation and encoder position in terms of stepper motor steps
			 *
			 *  An Encoder offset may be introduced.  The Encoder offset may remain at all times if
			 *  ENCODER_OFFSET_DELAY == 0 or terminate at a time (in ticks) of ENCODER_OFFSET_DELAY
			 */

			if ((i < ENCODER_START_OFFSET_DELAY) || (ENCODER_START_OFFSET_DELAY == 0)){
				encoder_position = encoder_position - ENCODER_START_OFFSET;
			}

			/*
			 * Compute error between Pendulum Angle and Pendulum Tracking Angle in units of steps
			 * Apply scale factor to match angle to step gain of rotor actuator
			 *
			 */
			*current_error_steps = encoder_angle_slope_corr
					+ (float) ((ENCODER_ANGLE_POLARITY)
							* (encoder_position / (ENCODER_READ_ANGLE_SCALE/STEPPER_READ_POSITION_STEPS_PER_DEGREE)));

			/*
			 *
			 * Pendulum Controller execution
			 *
			 * Include addition of Pendulum Angle track signal impulse signal
			 * Compute control signal, rotor position target in step units
			 *
			 * Pendulum tracking command, pendulum_position_command, also supplied in step units
			 *
			 */

			*current_error_steps = *current_error_steps + pendulum_position_command_steps; // TODO should pendulum_position_command be in steps?

			pid_filter_control_execute(pid_filter, current_error_steps, sample_period,
					deriv_lp_corner_f);
			rotor_control_target_steps = pid_filter->control_output;

			/* Acquire rotor position and compute low pass filtered rotor position */

			ret = rotor_position_read(&rotor_position_steps);

			rotor_position_filter_steps = (float) (rotor_position_steps) * iir_0
					+ rotor_position_steps_prev * iir_1
					- rotor_position_filter_steps_prev * iir_2;
			rotor_position_steps_prev = (float) (rotor_position_steps);
			rotor_position_filter_steps_prev = rotor_position_filter_steps;

			/*
			 * 		Compute rotor chirp tracking signal with chirp sweep from rotor_chirp_start_freq
			 * 		to rotor_chirp_end_freq in time period rotor_chirp_period in units of control
			 * 		loop cycle periods.  Each chirp is separated by delay of ROTOR_CHIRP_SWEEP_DELAY.
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
					rotor_chirp_frequency = rotor_chirp_start_freq + (rotor_chirp_end_freq - rotor_chirp_start_freq)*((float)(chirp_cycle/rotor_chirp_period));
					//rotor_chirp_frequency = (rotor_chirp_start_freq*rotor_chirp_end_freq*rotor_chirp_period)/((rotor_chirp_start_freq - rotor_chirp_end_freq)*((float)(chirp_cycle) + rotor_chirp_end_freq*rotor_chirp_period) );
					//rotor_chirp_frequency = rotor_chirp_end_freq * pow((rotor_chirp_start_freq/rotor_chirp_end_freq),((float)(chirp_cycle/rotor_chirp_period)));
					//rotor_chirp_frequency = rotor_chirp_start_freq - (rotor_chirp_end_freq - rotor_chirp_start_freq)*(chirp_cycle-rotor_chirp_period)*(chirp_cycle-rotor_chirp_period)/(rotor_chirp_period*rotor_chirp_period);

					rotor_position_command_steps = ((float)(ROTOR_CHIRP_STEP_AMPLITUDE*STEPPER_READ_POSITION_STEPS_PER_DEGREE))*sin(2.0*3.14159*rotor_chirp_frequency*chirp_time);
				}
			}

			/*  Create rotor track "comb" signal */
			if (enable_rotor_tracking_comb_signal > 0) {

				chirp_time = (float)((i - 1)/ROTOR_TRACK_COMB_SIGNAL_SAMPLE_RATE);
				rotor_track_comb_signal_frequency = 0.01;
				rotor_track_comb_command = ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.0178;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.0316;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.0562;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.1;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.178;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.316;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.562;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 1.00;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 1.78;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 3.16;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 5.62;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(2.0*3.14159*rotor_track_comb_signal_frequency*chirp_time);
			}

			if (enable_rotor_chirp == 0 && enable_mod_sin_rotor_tracking == 0
					&& enable_rotor_tracking_comb_signal == 1) {
				rotor_position_command_steps = rotor_track_comb_command;
			}

			/*  Create rotor track modulated sine signal */
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
					rotor_sine_drive = rotor_sine_drive + MOD_SIN_MODULATION_MIN;
					rotor_sine_drive = rotor_sine_drive * rotor_sine_drive_mod * rotor_mod_control;
				}

				if (i > MOD_SIN_START_CYCLES && ENABLE_SIN_MOD == 0) {
					rotor_sine_drive_mod = sin(0 + ((i - MOD_SIN_START_CYCLES) /MOD_SIN_SAMPLE_RATE) * (mod_sin_carrier_frequency * 6.2832));
					rotor_sine_drive = rotor_control_sin_amplitude * rotor_sine_drive_mod * rotor_mod_control;
				}
				rotor_position_command_steps = rotor_sine_drive;


				/* Detect transition between sine drive enable and disable */
				if ( abs(rotor_sine_drive_mod*MOD_SIN_AMPLITUDE) < 2 && disable_mod_sin_rotor_tracking == 1 && sine_drive_transition == 1){
					rotor_mod_control = 0.0;
					sine_drive_transition = 0;
				}
				if ( abs(rotor_sine_drive_mod*MOD_SIN_AMPLITUDE) < 2 && disable_mod_sin_rotor_tracking == 0 && sine_drive_transition == 1){
					rotor_mod_control = 1.0;
					sine_drive_transition = 0;
				}
			}

			/*  Create rotor angle track impulse signal */
			if (ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE == 1 && i != 0) {
				if ((i % ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
					rotor_position_command_steps =
							(float) (ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
									* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
					impulse_start_index = 0;
				}
				if (impulse_start_index
						> ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
					rotor_position_command_steps = 0;
				}
				impulse_start_index++;
			}

			/*
			 * Create pendulum angle track impulse signal.  Polarity of impulse alternates
			 */

			if (enable_pendulum_position_impulse_response_cycle == 1 && i != 0) {

				if ((i % PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
					pendulum_position_command_steps =
							(float) PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE;
					chirp_cycle = 0;
					impulse_start_index = 0;
				}
				if (impulse_start_index
						> PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
					pendulum_position_command_steps = 0;
				}
				impulse_start_index++;
				chirp_cycle++;
			}

			/*  Create rotor track step signal */
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
					rotor_position_command_steps = rotor_sine_drive + (float) ((rotor_position_step_polarity)
							* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
					chirp_cycle = chirp_cycle + 1;
				}
			}

			/*
			 *  Combine PID terms of Pendulum Angle Control with PID terms of Rotor Angle Control
			 *	This serves both dual PID, dual PD, and LQR control systems that include P and D
			 *	terms associated with Pendulum Angle, Pendulum Angle Time Derivative, Rotor Angle
			 *	and Rotor Angle time derivative.
			 *
			 */
			if (ENABLE_DUAL_PID == 1) {

				/*
				 * Secondary Controller execution including Sensitivity Function computation
				 */
				if (enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 0){
					*current_error_rotor_steps = rotor_position_filter_steps - rotor_position_command_steps;
				}
				if (enable_disturbance_rejection_step == 1){
					*current_error_rotor_steps = rotor_position_filter_steps;
				}
				if (enable_noise_rejection_step == 1){
					*current_error_rotor_steps = rotor_position_filter_steps + rotor_position_command_steps;
				}
				if (enable_sensitivity_fnc_step == 1){
					*current_error_rotor_steps = rotor_position_filter_steps - rotor_position_command_steps;
				}

				/*
				 * PID input supplied in units of stepper motor steps with tracking error determined
				 * as difference between rotor position tracking command and rotor position in units
				 * of stepper motor steps.
				 */


				pid_filter_control_execute(rotor_pid, current_error_rotor_steps,
						sample_period_rotor, deriv_lp_corner_f_rotor);
				rotor_control_target_steps = pid_filter->control_output + rotor_pid->control_output;


				/* Load Disturbance Sensitivity Function signal introduction */
				if (enable_disturbance_rejection_step == 1){
					rotor_control_target_steps = rotor_control_target_steps - rotor_position_command_steps;
				}
			}


			/* Output rate limiter */

			rotor_target_in_steps = rotor_control_target_steps;

			/* Limiter of control signal output for position control mode */
			if(ENABLE_LIMITER == 1 && ACCEL_CONTROL == 0){

				if ((rotor_control_target_steps - rotor_control_target_steps_prev) >= LIMITER_THRESHOLD) {
					rotor_control_target_steps = rotor_control_target_steps_prev
							+ (rotor_control_target_steps - rotor_control_target_steps_prev) / (LIMITER_SLOPE*LIMITER_THRESHOLD);
					slope = rotor_control_target_steps - rotor_control_target_steps_prev;
				}
				if ((rotor_control_target_steps - rotor_control_target_steps_prev) <= -LIMITER_THRESHOLD) {
					rotor_control_target_steps = rotor_control_target_steps_prev
							+ (rotor_control_target_steps - rotor_control_target_steps_prev) /  (LIMITER_SLOPE*LIMITER_THRESHOLD);
					slope = rotor_control_target_steps - rotor_control_target_steps_prev;
				}
				if (slope_prev < 0 && slope == 0) {
					rotor_control_target_steps = rotor_control_target_steps_prev
							- 2 * (rotor_control_target_steps - rotor_control_target_steps_prev) /  (LIMITER_SLOPE*LIMITER_THRESHOLD);
				}
				if (slope_prev > 0 && slope == 0) {
					rotor_control_target_steps = rotor_control_target_steps_prev
							- 2 * (rotor_control_target_steps - rotor_control_target_steps_prev) /  (LIMITER_SLOPE*LIMITER_THRESHOLD);
				}
				slope_prev = slope;
			}

			/* Limit maximum excursion in rotor angle at each cycle step for position control mode */
			if (ACCEL_CONTROL == 0) {
				rotor_position_delta = ROTOR_POSITION_MAX_DIFF;
				rotor_control_target_steps_curr = rotor_control_target_steps;
				if ((rotor_control_target_steps_curr - rotor_control_target_steps_prev)
						< -rotor_position_delta) {
					rotor_control_target_steps = rotor_control_target_steps_prev
							- rotor_position_delta;
				} else if ((rotor_control_target_steps_curr - rotor_control_target_steps_prev)
						> rotor_position_delta) {
					rotor_control_target_steps = rotor_control_target_steps_prev
							+ rotor_position_delta;
				}
			}

			/*
			 * Record current value of rotor_position_command tracking signal
			 * and control signal, rotor_control_target_steps for performance monitoring and adaptive control
			 */
			rotor_control_target_steps_prev = rotor_control_target_steps;
			rotor_position_command_steps_prev = rotor_position_command_steps;

			if (ACCEL_CONTROL == 1) {
				apply_acceleration(rotor_control_target_steps, &target_velocity_prescaled, SAMPLE_FREQUENCY);
			} else {
				BSP_MotorControl_GoTo(0, rotor_control_target_steps/2);
			}

			/******************************
			 * END MEASUREMENT & CONTROL
			 ******************************/

			/* Introduce pendulum displacement disturbance */
			int initial_rotor_position;
			if (ENABLE_PENDULUM_DISPLACEMENT_DISTURBANCE == 1 && i % PENDULUM_DISPLACEMENT_DISTURBANCE_CYCLE == 0 && i >= PENDULUM_DISPLACEMENT_DISTURBANCE_CYCLE){
				ret = rotor_position_read(&initial_rotor_position);
				BSP_MotorControl_GoTo(0, initial_rotor_position + 30);
				HAL_Delay(200);
			}

			/* Compute 100 cycle time average of cycle period for system performance measurement */
			if(i == 1){
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

			/* High speed sampling mode data reporting */
			if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 1 && enable_rotor_tracking_comb_signal == 0 && ACCEL_CONTROL_DATA == 0){
				sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%i\r\n", cycle_period_sum, (int)(chirp_cycle == 0),
						encoder_position, display_parameter, rotor_target_in_steps, (int)(100*rotor_position_command_steps));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}

			/* High speed sampling mode data reporting without rotor chirp signal and with comb signal */
			if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0 && enable_rotor_tracking_comb_signal == 1 && ACCEL_CONTROL_DATA == 0){
				sprintf(msg, "%i\t%i\t%i\t%i\t%i\r\n", cycle_period_sum,
						encoder_position, display_parameter, rotor_target_in_steps,(int)(100*rotor_position_command_steps));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}

			/* High speed sampling mode data reporting without rotor chirp signal and without comb signal */
			if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0 && enable_rotor_tracking_comb_signal == 0 && ACCEL_CONTROL_DATA == 0){
				if (enable_pendulum_position_impulse_response_cycle == 1) {
					reference_tracking_command = pendulum_position_command_steps;
				} else {
					reference_tracking_command = rotor_position_command_steps;
				}
				sprintf(msg, "%i\t%i\t%i\t%i\t%i\r\n", cycle_period_sum,
						encoder_position,display_parameter, rotor_target_in_steps,(int)(reference_tracking_command));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}
			/* High speed sampling mode data reporting for 800 Hz mode */
			if (ENABLE_800Hz_MODE == 1 && enable_high_speed_sampling == 1 && enable_rotor_chirp == 0 && ACCEL_CONTROL_DATA == 1){
				sprintf(msg, "%i\t%lu\r\n", rotor_position_steps, current_pwm_period);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}
			/* High speed sampling mode data reporting for 500 Hz mode with acceleration contol system data */
			if (ENABLE_800Hz_MODE == 0 && enable_high_speed_sampling == 1 && enable_rotor_chirp == 0 && ACCEL_CONTROL_DATA == 1){
				sprintf(msg, "%i\t%i\t%i\t%lu\t%lu\t%lu\r\n",
						rotor_position_steps, rotor_target_in_steps/10,(int)(rotor_position_command_steps),
						current_pwm_period, desired_pwm_period/10000,
						(clock_int_time/100000));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}

			/* Select display parameter corresponding to requested selection of Sensitivity Functions */
			if (enable_disturbance_rejection_step == 1) { display_parameter = rotor_position_steps; }
			else if (enable_noise_rejection_step == 1) { noise_rej_signal = rotor_control_target_steps; }
			else if (enable_sensitivity_fnc_step == 1)  { display_parameter = rotor_position_command_steps - rotor_position_steps; }
			else { display_parameter = rotor_position_steps; }

			if (enable_noise_rejection_step == 1 && enable_high_speed_sampling == 0){
				/*
				 * Apply antialiasing filter for noise rejection signal when reduced reporting rate applies
				 * When high speed sampling is disabled, reporting rate is 50 Hz.  Noise rejection signal filter
				 * corner frequency is 25 Hz.
				 */
				noise_rej_signal_filter =  noise_rej_signal*iir_0_nr + noise_rej_signal_prev*iir_1_nr - noise_rej_signal_filter_prev*iir_2_nr;
				noise_rej_signal_filter_prev = noise_rej_signal_filter;
				noise_rej_signal_prev = noise_rej_signal;
				display_parameter = noise_rej_signal_filter;
			}

			if (enable_noise_rejection_step == 1 && enable_high_speed_sampling == 1){
				display_parameter = noise_rej_signal;
			}

			if (i % 10 == 0 && enable_high_speed_sampling == 0){
				report_mode++;
				/* Provide report each 10th control cycle */
				if (report_mode != 0){
					sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%.1f\t%i\t%i\t%i\r\n", cycle_period_sum,
							(int) (tick_cycle_current - tick_cycle_previous),
							encoder_position, display_parameter, (int)(pid_filter->control_output)/10,
							rotor_position_command_steps, chirp_cycle, rotor_control_target_steps/10,
							(int)(rotor_pid->control_output)/10);
				}

				/* Provide report each 200th cycle of system parameters */

				if (report_mode == 20){
					sprintf(msg, "%i\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%i\t%i\r\n", 0,
							pid_filter->p_gain, pid_filter->i_gain, pid_filter->d_gain,
							rotor_pid->p_gain, rotor_pid->i_gain, rotor_pid->d_gain,
							max_speed, min_speed);
				}

				/* Provide report each 200th cycle of system parameters */

				if (report_mode == 40){
					sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\r\n", 1,
							(int)torq_current_val, max_accel, max_decel, enable_disturbance_rejection_step,
							enable_noise_rejection_step, enable_rotor_position_step_response_cycle,
							(int)(adjust_increment*10), enable_sensitivity_fnc_step);
							report_mode = 0;
				}


				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
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

		ret = rotor_position_read(&rotor_position_steps);
		BSP_MotorControl_GoTo(0, 0);
		BSP_MotorControl_SoftStop(0);

		/*
		 * Terminate motor control
		 */

		ret = rotor_position_read(&rotor_position_steps);
		sprintf(msg,"Exit Control at Rotor Angle, %.2f\r\n",
				(float) ((rotor_position_steps)
						/ STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		/*
		 * Safe exit by system software reset
		 */

		NVIC_SystemReset();

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






