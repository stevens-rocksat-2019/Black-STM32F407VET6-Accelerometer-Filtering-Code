# Black-STM32F407VET6-Accelerometer-Filtering-Code

## Dependencies

- https://github.com/stm32duino/Arduino_Core_STM32
To install add https://github.com/stm32duino/BoardManagerFiles/raw/master/STM32/package_stm_index.json to your board manager URLs in Arduino and install STM32 Cores

- https://github.com/stm32duino/STM32FreeRTOS
Install from library manager, search for `RTOS`, look for `STM32duino FreeRTOS` v10 in the list and install

- https://github.com/stm32duino/STM32SD
Install from library manager, search for `STM32SD` look for `STM32duino STM32SD` and install v1.2.0

- https://github.com/stm32duino/FatFs
Install from library manager, search for `FatFs` and install v2.0.2

## How to Upload to STM32 Board

- Plug in the STM32 board with the ST-Link

- Select upload protocol "STLink" in the "Tools" drop down menu

- For "Board" choose "Generic STM32F4 series"

- For "Board part number" Choose "Black F407VE"

- Press upload and compile

