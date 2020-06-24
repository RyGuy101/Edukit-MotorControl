################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/wkaiser/Documents/STMicroelectronics_Motor_Control_Curriculum/Edukit_Resources/Edukit_Firmware/Release_6-10-2020/Project/MC_curriculum/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/clock_f4.c \
/Users/wkaiser/Documents/STMicroelectronics_Motor_Control_Curriculum/Edukit_Resources/Edukit_Firmware/Release_6-10-2020/Project/MC_curriculum/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c \
/Users/wkaiser/Documents/STMicroelectronics_Motor_Control_Curriculum/Edukit_Resources/Edukit_Firmware/Release_6-10-2020/Project/MC_curriculum/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f4xx_hal_msp.c \
/Users/wkaiser/Documents/STMicroelectronics_Motor_Control_Curriculum/Edukit_Resources/Edukit_Firmware/Release_6-10-2020/Project/MC_curriculum/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f4xx_it.c 

OBJS += \
./Example/User/clock_f4.o \
./Example/User/main.o \
./Example/User/stm32f4xx_hal_msp.o \
./Example/User/stm32f4xx_it.o 

C_DEPS += \
./Example/User/clock_f4.d \
./Example/User/main.d \
./Example/User/stm32f4xx_hal_msp.d \
./Example/User/stm32f4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Example/User/clock_f4.o: /Users/wkaiser/Documents/STMicroelectronics_Motor_Control_Curriculum/Edukit_Resources/Edukit_Firmware/Release_6-10-2020/Project/MC_curriculum/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/clock_f4.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Example/User/clock_f4.d" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/main.o: /Users/wkaiser/Documents/STMicroelectronics_Motor_Control_Curriculum/Edukit_Resources/Edukit_Firmware/Release_6-10-2020/Project/MC_curriculum/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Example/User/main.d" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/stm32f4xx_hal_msp.o: /Users/wkaiser/Documents/STMicroelectronics_Motor_Control_Curriculum/Edukit_Resources/Edukit_Firmware/Release_6-10-2020/Project/MC_curriculum/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f4xx_hal_msp.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Example/User/stm32f4xx_hal_msp.d" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/stm32f4xx_it.o: /Users/wkaiser/Documents/STMicroelectronics_Motor_Control_Curriculum/Edukit_Resources/Edukit_Firmware/Release_6-10-2020/Project/MC_curriculum/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f4xx_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Example/User/stm32f4xx_it.d" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

