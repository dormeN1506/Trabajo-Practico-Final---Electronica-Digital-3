################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/tests/src/lpc17xx_adc_tests.c \
../Drivers/tests/src/lpc17xx_dac_tests.c \
../Drivers/tests/src/lpc17xx_exti_tests.c \
../Drivers/tests/src/lpc17xx_gpdma_tests.c \
../Drivers/tests/src/lpc17xx_gpio_tests.c \
../Drivers/tests/src/lpc17xx_pinsel_tests.c \
../Drivers/tests/src/lpc17xx_pwm_tests.c \
../Drivers/tests/src/lpc17xx_systick_tests.c \
../Drivers/tests/src/lpc17xx_timer_tests.c \
../Drivers/tests/src/lpc17xx_uart_tests.c \
../Drivers/tests/src/run_tests.c 

C_DEPS += \
./Drivers/tests/src/lpc17xx_adc_tests.d \
./Drivers/tests/src/lpc17xx_dac_tests.d \
./Drivers/tests/src/lpc17xx_exti_tests.d \
./Drivers/tests/src/lpc17xx_gpdma_tests.d \
./Drivers/tests/src/lpc17xx_gpio_tests.d \
./Drivers/tests/src/lpc17xx_pinsel_tests.d \
./Drivers/tests/src/lpc17xx_pwm_tests.d \
./Drivers/tests/src/lpc17xx_systick_tests.d \
./Drivers/tests/src/lpc17xx_timer_tests.d \
./Drivers/tests/src/lpc17xx_uart_tests.d \
./Drivers/tests/src/run_tests.d 

OBJS += \
./Drivers/tests/src/lpc17xx_adc_tests.o \
./Drivers/tests/src/lpc17xx_dac_tests.o \
./Drivers/tests/src/lpc17xx_exti_tests.o \
./Drivers/tests/src/lpc17xx_gpdma_tests.o \
./Drivers/tests/src/lpc17xx_gpio_tests.o \
./Drivers/tests/src/lpc17xx_pinsel_tests.o \
./Drivers/tests/src/lpc17xx_pwm_tests.o \
./Drivers/tests/src/lpc17xx_systick_tests.o \
./Drivers/tests/src/lpc17xx_timer_tests.o \
./Drivers/tests/src/lpc17xx_uart_tests.o \
./Drivers/tests/src/run_tests.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/tests/src/%.o: ../Drivers/tests/src/%.c Drivers/tests/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -I"C:\Users\User\Desktop\2026\electronica digital 3\DMA\LPC17xx-CMSIS-Driver-Enhancement-main\CMSISv2p00_LPC17xx\Drivers\inc" -I"../inc" -O0 -Os -g3 -gdwarf-4 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m3 -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-Drivers-2f-tests-2f-src

clean-Drivers-2f-tests-2f-src:
	-$(RM) ./Drivers/tests/src/lpc17xx_adc_tests.d ./Drivers/tests/src/lpc17xx_adc_tests.o ./Drivers/tests/src/lpc17xx_dac_tests.d ./Drivers/tests/src/lpc17xx_dac_tests.o ./Drivers/tests/src/lpc17xx_exti_tests.d ./Drivers/tests/src/lpc17xx_exti_tests.o ./Drivers/tests/src/lpc17xx_gpdma_tests.d ./Drivers/tests/src/lpc17xx_gpdma_tests.o ./Drivers/tests/src/lpc17xx_gpio_tests.d ./Drivers/tests/src/lpc17xx_gpio_tests.o ./Drivers/tests/src/lpc17xx_pinsel_tests.d ./Drivers/tests/src/lpc17xx_pinsel_tests.o ./Drivers/tests/src/lpc17xx_pwm_tests.d ./Drivers/tests/src/lpc17xx_pwm_tests.o ./Drivers/tests/src/lpc17xx_systick_tests.d ./Drivers/tests/src/lpc17xx_systick_tests.o ./Drivers/tests/src/lpc17xx_timer_tests.d ./Drivers/tests/src/lpc17xx_timer_tests.o ./Drivers/tests/src/lpc17xx_uart_tests.d ./Drivers/tests/src/lpc17xx_uart_tests.o ./Drivers/tests/src/run_tests.d ./Drivers/tests/src/run_tests.o

.PHONY: clean-Drivers-2f-tests-2f-src

