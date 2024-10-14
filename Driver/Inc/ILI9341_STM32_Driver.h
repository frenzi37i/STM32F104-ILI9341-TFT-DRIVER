#ifndef ILI9341_STM32_DRIVER_H
#define ILI9341_STM32_DRIVER_H

/*

# STM32F104-ILI9341-TFT-DRIVER
## Fast driver for ILI9341 240X320 TFT SPI display, using STM32F104 blue pill

Cannot find a good fast library one online, so forked one and improved it myself.

Library working using DMA to increase transmission speed reducing CPU time. 

## Working principle
Almost all the SPI messages (except color burst) are sent using HAL_SPI_Transmit_DMA function. 

When transmission ends, HAL_SPI_TxCpltCallback is triggered and a tx_completed flag is setted. 
When trying to send a msg, ILI9341_SPI_Tx wait the tx_completed to be setted (is BLOCKING; 
ok if transmission speed is high).
Before calling HAL_SPI_Transmit_DMA, the DC pin is setted high if data or low if command and CS is pulled low to enable the slave. 
The CS pin is pulled high when transmission ends and HAL_SPI_TxCpltCallback is triggered. 
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
### Clock setup
- Setted to use external 72MHz oscillator

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

*/

#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_it.h"
#include "main.h"



/* User Configuration */
#define HSPI_INSTANCE &hspi1
#define LCD_CS_PORT TFT_CS_GPIO_Port
#define LCD_CS_PIN TFT_CS_Pin
#define LCD_DC_PORT TFT_DC_GPIO_Port
#define LCD_DC_PIN TFT_DC_Pin
#define LCD_RST_PORT TFT_RESET_GPIO_Port
#define LCD_RST_PIN TFT_RESET_Pin

#define USE_LL_GPIO_DRIVERS 1 // comment out to use HAL GPIO Drivers
#define USE_CS_PIN 1 // comment out to avoid CS pin (pin tied to ground)

#define ILI9341_SCREEN_HEIGHT 	240
#define ILI9341_SCREEN_WIDTH 	320

//__________________________________________________________________________

#define BURST_MAX_SIZE 			500
#define BLACK      				0x0000
#define NAVY        			0x000F
#define DARKGREEN   			0x03E0
#define DARKCYAN    			0x03EF
#define MAROON      			0x7800
#define PURPLE      			0x780F
#define OLIVE       			0x7BE0
#define LIGHTGREY   			0xC618
#define DARKGREY    			0x7BEF
#define BLUE        			0x001F
#define GREEN       			0x07E0
#define CYAN        			0x07FF
#define RED         			0xF800
#define MAGENTA     			0xF81F
#define YELLOW      			0xFFE0
#define WHITE       			0xFFFF
#define ORANGE      			0xFD20
#define GREENYELLOW 			0xAFE5
#define PINK        			0xF81F

#define SCREEN_VERTICAL_1		0
#define SCREEN_HORIZONTAL_1		1
#define SCREEN_VERTICAL_2		2
#define SCREEN_HORIZONTAL_2		3

#define IS_COMMAND 0
#define IS_DATA 1

void ILI9341_WriteCommand(uint8_t cmd);
void ILI9341_WriteData(uint8_t data);
void ILI9341_WriteBuffer(uint8_t *buffer, uint16_t len);
void ILI9341_Reset(void);
void ILI9341_Enable(void);
void ILI9341_Init();
void ILI9341_SetRotation(uint8_t rotation);
void ILI9341_SetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void ILI9341_DrawColor(uint16_t color);
void ILI9341_DrawColorBurst(uint16_t color, uint32_t size);
void ILI9341_FillScreen(uint16_t color);
void ILI9341_DrawPixel(uint16_t x,uint16_t y,uint16_t color);
void ILI9341_DrawRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void ILI9341_DrawHLine(uint16_t x, uint16_t y, uint16_t width, uint16_t color);
void ILI9341_DrawVLine(uint16_t x, uint16_t y, uint16_t height, uint16_t color);
void ILI9341_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t state);

#endif
