# STM32F104-ILI9341-TFT-DRIVER
## Fast driver for ILI9341 240X320 TFT SPI display, using STM32F104 blue pill
Cannot find a good fast library one online, so forked one and improved it myself.
_________________________________________________________________________________________________
## Usage:
- If using STM32CubeIde, copy:
  - ILI9341_STM32_Driver.h, ILI9341_GFX.h, fonts.h in the project inc folder
  - ILI9341_STM32_Driver.c,ILI9341_GFX.c, fonts.c in the project src folder
- Include ILI9341_STM32_Driver.h and ILI9341_GFX.h in your main.h
- call ILI9341_Init(), then use the wanted functions.
________________________________________________________________________________________________
## Setup in ILI9341_STM32_Driver.h
- all the pins must be defined in ILI9341_STM32_Driver.h, associating the right GPIO Ports and GPIO Pins
- if using LL GPIO Driver uncomment #define USE_LL_GPIO_DRIVERS 1; else, if using HAL GPIO Drivers, comment it.
- if don't want to use CS pin, hold it to ground on the lcd, and comment out #define USE_CS_PIN 1.
  		This can save some us avoiding writing CS high and low, but decrease the 'immunity' of the lcd to
  		external noise; with CS always low the tft is always listening to commands; if possible, better to use it.
________________________________________________________________________________________________
## Hardware setup
### SPI setup (for MOSI and CLK pins)
- Mode: Transmit only master
- Frame format: Motorola
- Data Size: 8bits
- First Bit: MSB first
- Baudrate prescaler: if using 72MHz external RCC, prescaler 4 (18MBits/s) should be ok.
  If using long lines or experiences issues with missed transmissions, etc. , try to increase the
  prescaler to slow down the transmission.
- Clock Polarity (CPOL): LOW
- Clock Phase (CPHA): 1 Edge
- CRC Calculation: Disabled
- NSS Signal Type: Software
 
- Enable DMA on SPI1_TX:
- Memory to peripheral
- Mode: Normal
- Data Width: Byte
- Increment address: Memory
 
### GPIO SETUP (DC, CS, Reset)
- GPIO Output Level: Low
- Output push pull
- Max output speed: High
- Pullup/Pulldown for DC, CS, Reset:
  - DC: No pullup / pulldown
  - Reset: Pulldown (Low == not enabled)
  - CS: PullUp (High == not enabled)
 
 
________________________________________________________________________________________________ 
## Notes to use with STM32 Cube Ide:
- "main.h" should be included in ILI9341_STM32_Driver.h and in ILI9341_GFX.h
- the SPI handler must be declared extern to access it from ILI9341_STM32_Driver.h
  - in main.c: `SPI_HandleTypeDef hspi1;`
  - in main.h: `extern SPI_HandleTypeDef hspi1;`
  - in ILI9341_STM32_Driver.h: `#define HSPI_INSTANCE &hspi1`
 
- Always initialize the DMA BEFORE the others init functions in the main.
  		Cube IDE is setting things up in a "random order" when self writing the init functions;
  		The DMA one must be called first.
  		So in main.c, after HAL_Init() and SystemClock_Config() call MX_DMA_Init() in the USER CODE BEGIN SysInit,
  		before the other inits.

 

