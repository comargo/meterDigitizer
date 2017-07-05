
cmake_minimum_required(VERSION 3.5)
project(meterDigitizer)
enable_language(ASM)

add_executable(${PROJECT_NAME} Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c Src/system_stm32f1xx.c Src/usbd_cdc_if.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rtc_ex.c Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c Src/usbd_conf.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c Src/usb_device.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c Src/main.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c Src/stm32f1xx_hal_msp.c Src/stm32f1xx_it.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rtc.c Src/usbd_desc.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c startup_stm32f103xb.s)
target_compile_definitions(${PROJECT_NAME} PUBLIC USE_HAL_DRIVER STM32F103xB)
target_include_directories(${PROJECT_NAME} PUBLIC Inc Drivers/STM32F1xx_HAL_Driver/Inc Drivers/STM32F1xx_HAL_Driver/Inc/Legacy Middlewares/ST/STM32_USB_Device_Library/Core/Inc Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc Drivers/CMSIS/Device/ST/STM32F1xx/Include Drivers/CMSIS/Include)
target_compile_options(${PROJECT_NAME} PUBLIC -mcpu=cortex-m3 -mthumb   -Wall -fdata-sections -ffunction-sections -std=c11)
target_link_libraries(${PROJECT_NAME} -L${CMAKE_CURRENT_SOURCE_DIR} -mcpu=cortex-m3 -mthumb   -specs=nano.specs -TSTM32F103C8Tx_FLASH.ld  -lc -lm -lnosys -Wl,-Map=./meterDigitizer.map,--cref -Wl,--gc-sections -Wl,--no-wchar-size-warning)
include(CMakeLists.local.txt OPTIONAL)

