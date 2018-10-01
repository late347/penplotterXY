################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/portable/Common/mpu_wrappers.c 

OBJS += \
./src/portable/Common/mpu_wrappers.o 

C_DEPS += \
./src/portable/Common/mpu_wrappers.d 


# Each subdirectory must supply rules for building sources it contributes
src/portable/Common/%.o: ../src/portable/Common/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M3 -D__USE_LPCOPEN -D__LPC15XX__ -D__NEWLIB__ -I"C:\Users\keijo\Documents\metropolia\kurssit\2018\s18_embedded_OS\FreeRTOS_ws\FreeRTOS\inc" -I"C:\Users\keijo\Documents\metropolia\kurssit\2018\s18_embedded_OS\FreeRTOS_ws\lpc_board_nxp_lpcxpresso_1549\inc" -I"C:\Users\keijo\Documents\metropolia\kurssit\2018\s18_embedded_OS\FreeRTOS_ws\lpc_chip_15xx\inc" -I"C:\Users\keijo\Documents\metropolia\kurssit\2018\s18_embedded_OS\FreeRTOS_ws\FreeRTOS\src\include" -I"C:\Users\keijo\Documents\metropolia\kurssit\2018\s18_embedded_OS\FreeRTOS_ws\FreeRTOS\src\portable\GCC\ARM_CM3" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -D__NEWLIB__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


