################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/RotModule/rot_secure_memory.c \
../src/RotModule/rot_timer.c 

OBJS += \
./src/RotModule/rot_secure_memory.o \
./src/RotModule/rot_timer.o 

C_DEPS += \
./src/RotModule/rot_secure_memory.d \
./src/RotModule/rot_timer.d 


# Each subdirectory must supply rules for building sources it contributes
src/RotModule/%.o: ../src/RotModule/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM v7 gcc compiler'
	arm-none-eabi-gcc -Wall -O0 -g3 -c -fmessage-length=0 -MT"$@" -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -I../../FreeRTOS_bsp/ps7_cortexa9_0/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


