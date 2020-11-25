################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/custom/custom.c 

OBJS += \
./Drivers/BSP/custom/custom.o 

C_DEPS += \
./Drivers/BSP/custom/custom.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/custom/%.o: ../Drivers/BSP/custom/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F446xx -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Core/Inc" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/CMSIS/Include" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/Third_Party/FreeRTOS/Source/include" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/X-CUBE-MEMS1/Target" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/lsm6dsl" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/lsm303agr" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/hts221" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/lps22hb" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/IKS01A2" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Drivers/BSP/Components/Common" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/ST/STM32_MotionMC_Library/Inc" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/ST/STM32_MotionMC_Library/Lib" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/ST/STM32_MotionAC_Library/Inc" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/ST/STM32_MotionAC_Library/Lib" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/ST/STM32_MotionGC_Library/Inc" -I"D:/materialy/CPS-semestr2/Embedded_systems/Git_repo_stmf446/imu_mems/imu_mems_cube/Middlewares/ST/STM32_MotionGC_Library/Lib"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


