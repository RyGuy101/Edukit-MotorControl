################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/wkaiser/Documents/STMicroelectronics_Motor_Control_Curriculum/Edukit_Resources/Edukit_Firmware/Release_6-10-2020/Project/MC_curriculum/Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.c 

OBJS += \
./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.o 

C_DEPS += \
./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.o: /Users/wkaiser/Documents/STMicroelectronics_Motor_Control_Curriculum/Edukit_Resources/Edukit_Firmware/Release_6-10-2020/Project/MC_curriculum/Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.d" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

