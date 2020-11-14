################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/custom_bus.c \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/custom_bus.o \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/custom_bus.d \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F446xx -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Core/Inc" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/CMSIS/Include" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/Third_Party/FreeRTOS/Source/include" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/X-CUBE-MEMS1/Target" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/lsm6dsl" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/lsm303agr" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/hts221" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/lps22hb" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/IKS01A2" -I"D:/materialy/CPS-semestr2/Embedded systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/Common"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


