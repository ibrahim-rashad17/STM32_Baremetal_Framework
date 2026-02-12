################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/common.c \
../drivers/src/stm32f407xx_gpio_driver.c \
../drivers/src/stm32f407xx_rcc_driver.c \
../drivers/src/stm32f407xx_usart_driver.c 

OBJS += \
./drivers/src/common.o \
./drivers/src/stm32f407xx_gpio_driver.o \
./drivers/src/stm32f407xx_rcc_driver.o \
./drivers/src/stm32f407xx_usart_driver.o 

C_DEPS += \
./drivers/src/common.d \
./drivers/src/stm32f407xx_gpio_driver.d \
./drivers/src/stm32f407xx_rcc_driver.d \
./drivers/src/stm32f407xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su drivers/src/%.cyclo: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/STM32CUBE_IDE/Workspace/Baremetal_Framework/drivers/includes" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/common.cyclo ./drivers/src/common.d ./drivers/src/common.o ./drivers/src/common.su ./drivers/src/stm32f407xx_gpio_driver.cyclo ./drivers/src/stm32f407xx_gpio_driver.d ./drivers/src/stm32f407xx_gpio_driver.o ./drivers/src/stm32f407xx_gpio_driver.su ./drivers/src/stm32f407xx_rcc_driver.cyclo ./drivers/src/stm32f407xx_rcc_driver.d ./drivers/src/stm32f407xx_rcc_driver.o ./drivers/src/stm32f407xx_rcc_driver.su ./drivers/src/stm32f407xx_usart_driver.cyclo ./drivers/src/stm32f407xx_usart_driver.d ./drivers/src/stm32f407xx_usart_driver.o ./drivers/src/stm32f407xx_usart_driver.su

.PHONY: clean-drivers-2f-src

