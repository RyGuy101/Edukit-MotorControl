




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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "edukit_system.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>




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
		*encoder_position = 32767;
	}
	return range_error;
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
 * Single float value read
 */

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

/*
 * Single integer value read
 */

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

/*
 * Single character value read
 */

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

void assign_mode_1(pid_filter_control_parameters * pid_filter,
		pid_filter_control_parameters * rotor_pid){
	select_suspended_mode = 0;
	proportional = PRIMARY_PROPORTIONAL_MODE_1;
	integral = PRIMARY_INTEGRAL_MODE_1;
	derivative = PRIMARY_DERIVATIVE_MODE_1;
	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;
	pid_filter->p_gain = proportional;
	pid_filter->i_gain = integral;
	pid_filter->d_gain = derivative;
	rotor_pid->p_gain = rotor_p_gain;
	rotor_pid->i_gain = rotor_i_gain;
	rotor_pid->d_gain = rotor_d_gain;
	torq_current_val = 800;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
}

void assign_mode_2(pid_filter_control_parameters * pid_filter,
		pid_filter_control_parameters * rotor_pid){
	select_suspended_mode = 0;
	proportional = PRIMARY_PROPORTIONAL_MODE_2;
	integral = PRIMARY_INTEGRAL_MODE_2;
	derivative = PRIMARY_DERIVATIVE_MODE_2;
	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_2;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_2;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_2;
	pid_filter->p_gain = proportional;
	pid_filter->i_gain = integral;
	pid_filter->d_gain = derivative;
	rotor_pid->p_gain = rotor_p_gain;
	rotor_pid->i_gain = rotor_i_gain;
	rotor_pid->d_gain = rotor_d_gain;
	torq_current_val = 800;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
}

void assign_mode_3(pid_filter_control_parameters * pid_filter,
		pid_filter_control_parameters * rotor_pid){
	select_suspended_mode = 0;
	proportional = PRIMARY_PROPORTIONAL_MODE_3;
	integral = PRIMARY_INTEGRAL_MODE_3;
	derivative = PRIMARY_DERIVATIVE_MODE_3;
	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_3;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_3;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_3;
	pid_filter->p_gain = proportional;
	pid_filter->i_gain = integral;
	pid_filter->d_gain = derivative;
	rotor_pid->p_gain = rotor_p_gain;
	rotor_pid->i_gain = rotor_i_gain;
	rotor_pid->d_gain = rotor_d_gain;
	torq_current_val = 800;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
}

/*
 * Identify real time user character input and perform configuration
 *
 * Return mode value if identified
 *
 */

int mode_index_identification(char * user_config_input, int config_command_control,
		float *adjust_increment, pid_filter_control_parameters * pid_filter,
		pid_filter_control_parameters * rotor_pid){

	if (strcmp(user_config_input, mode_string_inc_pend_p) == 0){
		pid_filter->p_gain = pid_filter->p_gain + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_pend_p) == 0) {
		pid_filter->p_gain = pid_filter->p_gain - *adjust_increment;
		//if (pid_filter->p_gain <= 0) { pid_filter->p_gain = 0; }
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_pend_d) == 0) {
		pid_filter->d_gain = pid_filter->d_gain + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_pend_d) == 0) {
		pid_filter->d_gain = pid_filter->d_gain - *adjust_increment;
		//if (pid_filter->d_gain <= 0) { pid_filter->d_gain = 0; }
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_pend_i) == 0) {
		pid_filter->i_gain = pid_filter->i_gain + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_pend_i) == 0) {
		pid_filter->i_gain = pid_filter->i_gain - *adjust_increment;
		//if (pid_filter->i_gain <= 0) { pid_filter->i_gain = 0; }
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_rotor_p) == 0){
		rotor_pid->p_gain = rotor_pid->p_gain + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_rotor_p) == 0) {
		rotor_pid->p_gain = rotor_pid->p_gain - *adjust_increment;
		//if (rotor_pid->p_gain <= 0) { rotor_pid->p_gain = 0; }
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_rotor_d) == 0) {
		rotor_pid->d_gain = rotor_pid->d_gain + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_rotor_d) == 0) {
		rotor_pid->d_gain = rotor_pid->d_gain - *adjust_increment;
		//if (rotor_pid->d_gain <= 0) { rotor_pid->d_gain = 0; }
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_rotor_i) == 0) {
		rotor_pid->i_gain = rotor_pid->i_gain + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_rotor_i) == 0) {
		rotor_pid->i_gain = rotor_pid->i_gain - *adjust_increment;
		//if (rotor_pid->i_gain <= 0) { rotor_pid->i_gain = 0; }
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_torq_c) == 0) {
		torq_current_val = L6474_GetAnalogValue(0, L6474_TVAL);
		torq_current_val = torq_current_val - *adjust_increment;
		if (torq_current_val < 200){ torq_current_val = 200; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_torq_c) == 0) {
		torq_current_val = L6474_GetAnalogValue(0, L6474_TVAL);
		torq_current_val = torq_current_val + *adjust_increment;
		if (torq_current_val > 800){ torq_current_val = 800; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_max_s) == 0) {
		max_speed = L6474_GetMaxSpeed(0);
		max_speed = max_speed - *adjust_increment;
		if (max_speed < 100){ max_speed = 100; }
		if (max_speed < min_speed){ max_speed = min_speed;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetMaxSpeed(0, max_speed);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_max_s) == 0) {
		max_speed = L6474_GetMaxSpeed(0);
		max_speed = max_speed + *adjust_increment;
		if (max_speed > 1000){ max_speed = 1000; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetMaxSpeed(0, max_speed);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_min_s) == 0) {
		min_speed = L6474_GetMinSpeed(0);
		min_speed = min_speed - *adjust_increment;
		if (min_speed < 100){ min_speed = 100; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetMinSpeed(0, min_speed);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_min_s) == 0) {
		min_speed = L6474_GetMinSpeed(0);
		min_speed = min_speed + *adjust_increment;
		if (min_speed > 1000){ min_speed = 1000; }
		if (min_speed > max_speed){ min_speed = max_speed;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetMinSpeed(0, min_speed);
		config_command = 1;
		mode_index_command = -1;
	} else if (strcmp(user_config_input, mode_string_dec_max_a) == 0) {
		max_accel = L6474_GetAcceleration(0);
		max_accel = max_accel - *adjust_increment;
		if (max_accel <  0){ max_accel = 0;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAcceleration(0, max_accel);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_max_a) == 0) {
		max_accel = L6474_GetAcceleration(0);
		max_accel = max_accel + *adjust_increment;
		if (max_accel >  10000){ max_accel = 10000;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAcceleration(0, max_accel);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_max_d) == 0) {
		max_decel = L6474_GetDeceleration(0);
		max_decel = max_decel - *adjust_increment;
		if (max_decel <  0){ max_decel = 0;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetDeceleration(0, max_decel);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_max_d) == 0) {
		max_decel = L6474_GetDeceleration(0);
		max_decel = max_decel + *adjust_increment;
		if (max_decel > 10000) { max_decel = 10000; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetDeceleration(0, max_decel);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_select_mode_5) == 0) {
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
	} else if (strcmp(user_config_input, mode_string_enable_step ) == 0 ){
		enable_rotor_position_step_response_cycle = 1;
		enable_noise_rejection_step = 0;
		enable_disturbance_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_step ) == 0 ){
		enable_rotor_position_step_response_cycle = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_noise_rej_step ) == 0 ){
		enable_noise_rejection_step = 1;
		enable_rotor_position_step_response_cycle = 1;
		enable_disturbance_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_noise_rej_step ) == 0 ){
		enable_noise_rejection_step = 0;
		enable_rotor_position_step_response_cycle = 1;
		enable_disturbance_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_sensitivity_fnc_step ) == 0 ){
		enable_sensitivity_fnc_step = 1;
		enable_rotor_position_step_response_cycle = 1;
		enable_disturbance_rejection_step = 0;
		enable_noise_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_sensitivity_fnc_step ) == 0 ){
		enable_sensitivity_fnc_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_load_dist ) == 0 ){
		enable_sensitivity_fnc_step = 0;
		enable_rotor_position_step_response_cycle = 1;
		enable_disturbance_rejection_step = 1;
		enable_noise_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_load_dist ) == 0 ){
		enable_disturbance_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_step_size ) == 0 ){
		step_size = step_size + 1;
		if (step_size > 4) { step_size = 4; }
		if (step_size == 0) { *adjust_increment = 0.5;}
		else if (step_size == 1) { *adjust_increment = 2;}
		else if (step_size == 2) { *adjust_increment = 10;}
		else if (step_size == 3) { *adjust_increment = 50;}
		else if (step_size == 4) { *adjust_increment = 100;}
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_step_size ) == 0 ){
		step_size = step_size - 1;
		if (step_size < 0) { step_size = 0; }
		if (step_size == 0) { *adjust_increment = 0.5;}
		else if (step_size == 1) { *adjust_increment = 2;}
		else if (step_size == 2) { *adjust_increment = 10;}
		else if (step_size == 3) { *adjust_increment = 50;}
		else if (step_size == 4) { *adjust_increment = 100;}
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_high_speed_sampling ) == 0 ){
		enable_high_speed_sampling = 1;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_high_speed_sampling ) == 0 ){
		enable_high_speed_sampling = 0;
		config_command = 1;
	} else {
		mode_index_command = atoi((char*) Msg.Data);
	}
	/*
	 * Disable sin drive tracking
	 */
	if (mode_index_command == mode_9){
		disable_mod_sin_rotor_tracking = 1;
		sine_drive_transition = 1;
		mode_index_command = -1;
	}
	if (config_command_control == 0){
		/*
		 * Enable sin drive tracking
		 */
		if (mode_index_command == mode_5){
			disable_mod_sin_rotor_tracking = 0;
			sine_drive_transition = 1;
			mode_index_command = -1;
		}
	}
	return mode_index_command;
}


/*
 * Initialize mode identification strings
 */

void set_mode_strings(void){
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


	mode_1 = 1;				// Enable LQR Motor Model M
	mode_2 = 2;				// Enable LRR Motor Model H
	mode_3 = 3;				// Enable LQR Motor Model L
	mode_4 = 4;				// Enable Suspended Mode Motor Model M
	mode_5 = 5;				// Enable sin drive track signal
	mode_adaptive_off = 6;	// Disable adaptive control
	mode_adaptive = 7;		// Enable adaptive control
	mode_8 = 8;				// Enable custom configuration entry
	mode_9 = 9;				// Disable sin drive track signal
	mode_10 = 10;			// Enable Single PID Mode with Motor Model M
	mode_11 = 11;			// Enable rotor actuator and encoder test mode
	mode_12 = 12;			// Enable rotor actuator and encoder high speed test mode
	mode_13 = 13;			// Enable rotor control system evaluation
	mode_14 = 14;			// Enable pendlum system identification
	mode_15 = 15;			// Enable interactive control of rotor actuator
	mode_16 = 16;			// Enable load disturbance function mode
	mode_17 = 17;			// Enable noise disturbance function step mode
	mode_18 = 18;			// Enable sensitivity function step mode
	mode_quit = 0;			// Initiate exit from control loop
}

/*
 * Print user prompt for serial interface
 */

void user_prompt(void){
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
}

/*
 * Return selected user mode
 */

void get_user_mode_index(char * user_string, int * char_mode_select, int * mode_index, int * mode_interactive){

	*char_mode_select = 0;

	if (strcmp(user_string,mode_string_mode_single_pid)==0){
		*mode_index = 10;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_test)==0){
		*mode_index = 11;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_control)==0){
		*mode_index = 15;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_high_speed_test)==0){
		*mode_index = 12;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_motor_characterization_mode)==0){
		*mode_index = 13;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_pendulum_sysid_test)==0){
		*mode_index = 14;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_8)==0){
		*mode_index = 8;
		*char_mode_select = 1;
	}

	if(*char_mode_select == 0){
		*mode_index = atoi(user_string);
	}


	/*
	 * Set mode index according to user input
	 */

	switch (*mode_index) {

	case 1:
		*mode_index = mode_1;
		break;

	case 2:
		*mode_index = mode_2;
		break;

	case 3:
		*mode_index = mode_3;
		break;

	case 4:
		*mode_index = mode_4;
		break;

	case 7:
		*mode_index = mode_adaptive;
		break;

	case 8:
		*mode_index = mode_8;
		*mode_interactive = 1;
		break;

	case 10:
		*mode_index = mode_10;
		*mode_interactive = 1;
		break;

	case 11:
		*mode_index = mode_11;
		*mode_interactive = 1;
		break;

	case 12:
		*mode_index = mode_12;
		*mode_interactive = 1;
		break;

	case 13:
		*mode_index = mode_13;
		*mode_interactive = 1;
		break;

	case 14:
		*mode_index = mode_14;
		*mode_interactive = 1;
		break;

	case 15:
		*mode_index = mode_15;
		*mode_interactive = 1;
		break;

	default:
		*mode_index = mode_1;
		break;
	}

}

/*
 * Configure system based on user selection
 */

void user_configuration(void){

	enable_rotor_actuator_test = 0;
	enable_rotor_actuator_control = 0;
	enable_encoder_test = 0;
	enable_rotor_actuator_high_speed_test = 0;
	enable_motor_actuator_characterization_mode = 0;
	enable_rotor_tracking_comb_signal = 0;
	rotor_track_comb_amplitude = 0;
	enable_disturbance_rejection_step = 0;
	enable_noise_rejection_step = 0;
	enable_sensitivity_fnc_step = 0;

	while (1){
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
			enable_mod_sin_rotor_tracking = 1;
			L6474_SetAnalogValue(0, L6474_TVAL, TORQ_CURRENT_DEFAULT);
			enable_rotor_position_step_response_cycle = 1;
			break;
		}

		if (readBytes) // Message found
		{
			RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;

			if (Msg.Len != 1) {
				continue;
			}

			sprintf(msg, "%s\n\r", (char*)Msg.Data);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

			get_user_mode_index((char*)Msg.Data, &char_mode_select, &mode_index, &mode_interactive);

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
			enable_sensitivity_fnc_step = 0;


			switch (mode_index) {

			/* Mode 1 selection */

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

				if (enable_rotor_tracking_comb_signal == 1) {
					enable_rotor_position_step_response_cycle = 0;
					enable_pendulum_position_impulse_response_cycle = 0;
					enable_mod_sin_rotor_tracking = 0;
					sprintf(msg, "\n\rRotor Comb Drive enabled ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}

				break;

			/* Mode 2 selection */

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
				enable_mod_sin_rotor_tracking = ENABLE_MOD_SIN_ROTOR_TRACKING;
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

			/* Mode 3 selection */

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

			/* Mode 4 Suspended Mode selection */
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


			/* Adaptive Mode selection */
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

			/* General mode selection requiring user specification of all configurations */
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

				if (enable_rotor_tracking_comb_signal == 1) {
					enable_rotor_position_step_response_cycle = 0;
					enable_pendulum_position_impulse_response_cycle = 0;
					enable_mod_sin_rotor_tracking = 0;
					sprintf(msg, "\n\rRotor Comb Drive enabled ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}

				enable_disturbance_rejection_step = 0;
				enable_noise_rejection_step = 0;
				enable_sensitivity_fnc_step = 0;

				sprintf(msg, "\n\rEnter 1 to Enable Disturbance Rejection Sensitivity Function Analysis; 0 to Disable: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_disturbance_rejection_step);
				sprintf(msg, "%i", enable_disturbance_rejection_step);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				/*
				if (enable_disturbance_rejection_step == 1){
					if (enable_rotor_tracking_comb_signal == 1 || enable_rotor_chirp == 1){
						enable_rotor_position_step_response_cycle = 0;
					} else {
					enable_rotor_position_step_response_cycle = 1;
					}
					enable_mod_sin_rotor_tracking = 0;
				}
				*/

				if (enable_disturbance_rejection_step == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Noise Rejection Sensitivity Function Analysis; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_noise_rejection_step);
					sprintf(msg, "%i", enable_noise_rejection_step);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}
				/*
				if (enable_noise_rejection_step == 1){
					if (enable_rotor_tracking_comb_signal == 1 || enable_rotor_chirp == 1){
						enable_rotor_position_step_response_cycle = 0;
					} else {
					enable_rotor_position_step_response_cycle = 1;
					}
					enable_mod_sin_rotor_tracking = 0;
				}
				*/

				if (enable_noise_rejection_step == 0 && enable_disturbance_rejection_step == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Sensitivity Function Analysis; 0 to Disable: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_sensitivity_fnc_step);
					sprintf(msg, "%i", enable_sensitivity_fnc_step);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}

				/*

				if (enable_sensitivity_fnc_step == 1){
					if (enable_rotor_tracking_comb_signal == 1 || enable_rotor_chirp == 1){
						enable_rotor_position_step_response_cycle = 0;
					} else {
					enable_rotor_position_step_response_cycle = 1;
					}
					enable_mod_sin_rotor_tracking = 0;
				}

				*/


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

				/* Interactive entry of Pendulum Controller gains for Single PID Inverted Mode */

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

					/* Rotor actuator and encoder test mode */
					case 11:
						enable_rotor_actuator_test = 1;
						enable_encoder_test = 1;
						sprintf(msg, "\n\rTest Mode Configured");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg), HAL_MAX_DELAY);
						break;

					/* Rotor actuator high speed test mode */
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

					/* Rotor actuator characterization test mode */
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

					/* Pendulum system identification mode */
					case 14:
						enable_pendulum_sysid_test = 1;
						sprintf(msg, "\n\rPendulum System Identification Test Mode Configured");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						break;

					/* Rotor actuator control mode */
					case 15:
						enable_rotor_actuator_control = 1;
						sprintf(msg, "\n\rRotor Actuator Control Mode Configured");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg), HAL_MAX_DELAY);
						break;

					/* Rotor tracking comb signal */
					case 16:
						enable_rotor_tracking_comb_signal = 1;
						rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
						sprintf(msg, "\n\rLoad Disturbance Sensitivity Spectrum Analyzer Enabled");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg), HAL_MAX_DELAY);
						break;


					/* Default start mode */
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
						L6474_SetAnalogValue(0, L6474_TVAL, TORQ_CURRENT_DEFAULT);
						sprintf(msg, "\n\rDefault Mode 1 Configured");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg,
								strlen(msg), HAL_MAX_DELAY);
						break;
			}
			return;
		}
	}
	return;
}

/*
 * Pendulum system identification
 */

void pendulum_system_id_test(void){
	int i, k;
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
 * Rotor high speed test
 */

void rotor_actuator_high_speed_test(void){
	int k,m;

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
 * Rotor and encoder test mode
 */

void rotor_encoder_test(void){
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
}

/*
 * Rotor actuator characterization mode
 */

void motor_actuator_characterization_mode(void){
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
 * Interactive rotor control
 */

void interactive_rotor_actuator_control(void){
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
