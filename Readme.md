# STM32 HAL & FreeRTOS-based HTTP & SCPI server #

Simple HTTP & SCPI server based around FreeRTOS and libscpi libraries for Nucleo-F746ZG & Nucleo-F767ZI boards. Uses the LwIP Netconn API.

It contains two HTML pages:

  * The first page (home page) is static, it gives basic information about STM32F7 and LwIP stack.
  * The second page is dynamically refreshed (every 1 s), it shows the RTOS statistics in runtime.

If a DHCP server is available, a dynamic IP address can be allocated by enabling 
the DHCP process (#define USE_DHCP in main.h). By default DHCP is used.

Ethernet cable status is ensured by LEDs: 
  + LED1: ethernet cable is connected.
  + LED3: ethernet cable is not connected.
  
## Note ##
 
This example code was adapted from the 'LwIP HTTP Server Netconn RTOS' example provided with ST Microelectronics' [STM32F7 CMSIS package](https://github.com/STMicroelectronics/STM32CubeF7).

## Building ##

This example requires Make and the [Arm GCC-based toolchain](https://developer.arm.com/tools-and-software/embedded/arm-compiler/downloads/version-6). Building the project has been successfully tested with MSYS2 on Windows.

By default the Makefile targets the Nucleo-F746ZG board. Toggle the commented out defines in the Makefile to change the target.

With the dependencies installed, execute `make` to build the project.

## Flashing ##

To flash the project to the Nucleo board after building, OpenOCD has to be installed. After that executing `make flash` suffices to flash the binary firmware image.