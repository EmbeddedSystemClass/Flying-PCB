################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Dave/Generated/src/GMM001/GMM001.c \
../Dave/Generated/src/GMM001/GMM001_Conf.c 

OBJS += \
./Dave/Generated/src/GMM001/GMM001.o \
./Dave/Generated/src/GMM001/GMM001_Conf.o 

C_DEPS += \
./Dave/Generated/src/GMM001/GMM001.d \
./Dave/Generated/src/GMM001/GMM001_Conf.d 


# Each subdirectory must supply rules for building sources it contributes
Dave/Generated/src/GMM001/%.o: ../Dave/Generated/src/GMM001/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM-GCC C Compiler'
	"C:\DAVE-3.1.10\ARM-GCC/bin/arm-none-eabi-gcc" -DDAVE_CE -DARM_MATH_CM4 -D__FPU_PRESENT -DUC_ID=4503003 -I"C:\DAVE-3.1.10\eclipse\/../CMSIS/Include" -I"C:\DAVE-3.1.10\eclipse\/../CMSIS/Infineon/Include" -I"C:\DAVE-3.1.10\ARM-GCC/arm-none-eabi/include" -I"C:\DAVE-3.1.10\eclipse\/../emWin/Start/GUI/inc" -I"C:\DAVE-3.1.10\eclipse\/../CMSIS/Infineon/XMC4500_series/Include" -I"E:\GitHub\Flying-PCB\Software\LARIX Flightcontroller\CORE_Larix_V1.0\Dave\Generated\inc\DAVESupport" -I"E:\GitHub\Flying-PCB\Software\LARIX Flightcontroller\CORE_Larix_V1.0\Dave\Generated\inc\LIBS" -I"E:\GitHub\Flying-PCB\Software\LARIX Flightcontroller\CORE_Larix_V1.0\Dave\Generated\inc\MOTORLIBS" -I"E:\GitHub\Flying-PCB\Software\LARIX Flightcontroller\CORE_Larix_V1.0\Dave\Generated\src\USBCDC001\Drivers\USB\Core" -I"E:\GitHub\Flying-PCB\Software\LARIX Flightcontroller\CORE_Larix_V1.0\Dave\Generated\src\USBCDC001\Drivers\USB" -I"E:\GitHub\Flying-PCB\Software\LARIX Flightcontroller\CORE_Larix_V1.0\Dave\Generated\src\USBCDC001\Drivers\USB\Class" -I"../“C:\MinGW\include”" -I"../“C:\MinGW\include”" -O0 -ffunction-sections -Wall -std=gnu99 -mfloat-abi=softfp -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $@" -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


