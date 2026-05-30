################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/core_cm3.c \
../src/system_LPC17xx.c 

C_DEPS += \
./src/core_cm3.d \
./src/system_LPC17xx.d 

OBJS += \
./src/core_cm3.o \
./src/system_LPC17xx.o 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -I"C:\Users\User\Desktop\2026\electronica digital 3\DMA\LPC17xx-CMSIS-Driver-Enhancement-main\CMSISv2p00_LPC17xx\Drivers\inc" -I"../inc" -O0 -Os -g3 -gdwarf-4 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m3 -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src

clean-src:
	-$(RM) ./src/core_cm3.d ./src/core_cm3.o ./src/system_LPC17xx.d ./src/system_LPC17xx.o

.PHONY: clean-src

