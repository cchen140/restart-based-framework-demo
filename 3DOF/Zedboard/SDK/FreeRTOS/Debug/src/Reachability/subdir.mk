################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Reachability/3dof.c \
../src/Reachability/dynamics_3dof.c \
../src/Reachability/face_lift.c \
../src/Reachability/geometry.c \
../src/Reachability/simulate.c \
../src/Reachability/util.c 

OBJS += \
./src/Reachability/3dof.o \
./src/Reachability/dynamics_3dof.o \
./src/Reachability/face_lift.o \
./src/Reachability/geometry.o \
./src/Reachability/simulate.o \
./src/Reachability/util.o 

C_DEPS += \
./src/Reachability/3dof.d \
./src/Reachability/dynamics_3dof.d \
./src/Reachability/face_lift.d \
./src/Reachability/geometry.d \
./src/Reachability/simulate.d \
./src/Reachability/util.d 


# Each subdirectory must supply rules for building sources it contributes
src/Reachability/%.o: ../src/Reachability/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM v7 gcc compiler'
	arm-none-eabi-gcc -Wall -O0 -g3 -c -fmessage-length=0 -MT"$@" -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -I../../FreeRTOS_bsp/ps7_cortexa9_0/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


