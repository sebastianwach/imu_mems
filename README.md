# MEMS IMU

## Hardware
### X-NUCLEO-IKS01A2 - expansion board

X-NUCLEO-IKS01A2 is a motion MEMS and environmental sensor expansion board for STM32 Nucleo board. 

Used modules:
- LSM6DSL MEMS 3D accelerometer (±2/±4/±8/±16 g) and 3D gyroscope (±125/±245/±500/±1000/±2000 dps)
- LSM303AGR MEMS 3D magnetometer (±50 gauss)

## Software
Whole project is done in STM32CubeIDE. All I/O ports, timers, connectivity, libraries generated with CubeMx.
From STM X-CUBE-MEMS1 library for IKS01A2 was used.
There was problems with STM32_MotionXC_Libraries. In the end it isn't implemented in project.

## Visulisation 

https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation


## Encountered problems
### After generating project in CubeMx
Possible error connecting with "USE_COM_LOG"
1. Go into custom_conf.h
2. change define value of USE_COM_LOG to "0U"
3. Build again.
4. Now it should work

### Problem with linking library to project
There was problem with adding STM32_MotionAC_Library, STM32_MotionGC_Library, STM32_MotionMC_Library. Solution:
Go to: Project->Properties->C/C++ Build->Settings->Tool Settings-> MCU GCC Linker in section Libraries(-l) was put file name with colon in prefix, **not with lib prefix**:
```xml
:MotionAC_CM4F_wc32_ot_hard.a
```
And in section Library search path (-L):
```xml
../Middlewares/ST/STM32_MotionAC_Library/Lib
```


### Undefined reference to MotionAC_LoadCalFromNVM and MotionAC_SaveCalInNVM

These functions are only described in header and not in static library (*.a file). There is a need to implement them by ourself. For example without any actions:
```cpp=
char MotionMC_LoadCalFromNVM (unsigned short intdataSize, unsigned int *data)
{
	return 0;
}

char MotionMC_SaveCalInNVM (unsigned short intdataSize, unsigned int *data)
{
	return 0;
}
```

### Problem with gyroscope units
Readed value from gyroscope was 100 times greater than expected. It generated unexpected behaviours and huge oscilations.

### Low frequency sampling
Sampling frequency was increased from 10Hz to 100Hz to improve Madgwick fusion algorithm

