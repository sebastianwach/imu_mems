################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
../Middlewares/Third_Party/FreeRTOS/Source/list.c \
../Middlewares/Third_Party/FreeRTOS/Source/queue.c \
../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
../Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
../Middlewares/Third_Party/FreeRTOS/Source/timers.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/croutine.o \
./Middlewares/Third_Party/FreeRTOS/Source/event_groups.o \
./Middlewares/Third_Party/FreeRTOS/Source/list.o \
./Middlewares/Third_Party/FreeRTOS/Source/queue.o \
./Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.o \
./Middlewares/Third_Party/FreeRTOS/Source/tasks.o \
./Middlewares/Third_Party/FreeRTOS/Source/timers.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/croutine.d \
./Middlewares/Third_Party/FreeRTOS/Source/event_groups.d \
./Middlewares/Third_Party/FreeRTOS/Source/list.d \
./Middlewares/Third_Party/FreeRTOS/Source/queue.d \
./Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.d \
./Middlewares/Third_Party/FreeRTOS/Source/tasks.d \
./Middlewares/Third_Party/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F446xx -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Core/Inc" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/CMSIS/Include" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/Third_Party/FreeRTOS/Source/include" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/X-CUBE-MEMS1/Target" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/lsm6dsl" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/lsm303agr" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/hts221" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/lps22hb" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/IKS01A2" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/Common"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


