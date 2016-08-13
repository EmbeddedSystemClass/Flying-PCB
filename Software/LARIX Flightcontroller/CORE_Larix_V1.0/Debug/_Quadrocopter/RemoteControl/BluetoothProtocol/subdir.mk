################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../_Quadrocopter/RemoteControl/BluetoothProtocol/BluetoothProtocol.c 

OBJS += \
./_Quadrocopter/RemoteControl/BluetoothProtocol/BluetoothProtocol.o 

C_DEPS += \
./_Quadrocopter/RemoteControl/BluetoothProtocol/BluetoothProtocol.d 


# Each subdirectory must supply rules for building sources it contributes
_Quadrocopter/RemoteControl/BluetoothProtocol/%.o: ../_Quadrocopter/RemoteControl/BluetoothProtocol/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM-GCC C Compiler'
	"C:\DAVE-3.1.10\ARM-GCC/bin/arm-none-eabi-gcc" -DDAVE_CE -DARM_MATH_CM4 -D__FPU_PRESENT -DUC_ID=4503003 -I"C:\DAVE-3.1.10\eclipse\/../CMSIS/Include" -I"C:\DAVE-3.1.10\eclipse\/../CMSIS/Infineon/Include" -I"C:\DAVE-3.1.10\ARM-GCC/arm-none-eabi/include" -I"C:\DAVE-3.1.10\eclipse\/../emWin/Start/GUI/inc" -I"C:\DAVE-3.1.10\eclipse\/../CMSIS/Infineon/XMC4500_series/Include" -I"E:\GitHub\Flying-PCB\Software\LARIX Flightcontroller\CORE_Larix_V1.0\Dave\Generated\inc\DAVESupport" -I"E:\GitHub\Flying-PCB\Software\LARIX Flightcontroller\CORE_Larix_V1.0\Dave\Generated\inc\LIBS" -I"E:\GitHub\Flying-PCB\Software\LARIX Flightcontroller\CORE_Larix_V1.0\Dave\Generated\inc\MOTORLIBS" -I"E:\GitHub\Flying-PCB\Software\LARIX Flightcontroller\CORE_Larix_V1.0\Dave\Generated\src\USBCDC001\Drivers\USB\Core" -I"E:\GitHub\Flying-PCB\Software\LARIX Flightcontroller\CORE_Larix_V1.0\Dave\Generated\src\USBCDC001\Drivers\USB" -I"E:\GitHub\Flying-PCB\Software\LARIX Flightcontroller\CORE_Larix_V1.0\Dave\Generated\src\USBCDC001\Drivers\USB\Class" -I"../“C:\MinGW\include”" -I"../“C:\MinGW\include”" -O0 -ffunction-sections -Wall -std=gnu99 -mfloat-abi=softfp -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $@" -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


