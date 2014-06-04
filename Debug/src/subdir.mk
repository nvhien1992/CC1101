################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/HA_System.cpp \
../src/MB1_Buttons.cpp \
../src/MB1_ISR.cpp \
../src/MB1_Leds.cpp \
../src/MB1_Misc.cpp \
../src/MB1_SPI.cpp \
../src/MB1_Serial_t.cpp \
../src/MB1_System.cpp \
../src/driverCC1101.cpp \
../src/hl_crc.cpp \
../src/main.cpp \
../src/variablePacketHandler.cpp 

OBJS += \
./src/HA_System.o \
./src/MB1_Buttons.o \
./src/MB1_ISR.o \
./src/MB1_Leds.o \
./src/MB1_Misc.o \
./src/MB1_SPI.o \
./src/MB1_Serial_t.o \
./src/MB1_System.o \
./src/driverCC1101.o \
./src/hl_crc.o \
./src/main.o \
./src/variablePacketHandler.o 

CPP_DEPS += \
./src/HA_System.d \
./src/MB1_Buttons.d \
./src/MB1_ISR.d \
./src/MB1_Leds.d \
./src/MB1_Misc.d \
./src/MB1_SPI.d \
./src/MB1_Serial_t.d \
./src/MB1_System.d \
./src/driverCC1101.d \
./src/hl_crc.d \
./src/main.d \
./src/variablePacketHandler.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DDEBUG -I"../include" -I"../libs/CMSIS/include" -I"../libs/StdPeriph/include" -I"../libs/misc/include" -std=gnu++11 -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


