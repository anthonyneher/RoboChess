/*
 * LCDLib.h
 *
 *  Created on: Nov 11, 2019
 *      Author: Dave
 */

#ifndef LCDLIB_H_
#define LCDLIB_H_

#include <F28x_Project.h>
#include <stdbool.h>
#include <stdint.h>
#include "i2c.h"
#include "I2C_driver.h"

/************************************ Defines *******************************************/

/* GPIO32   - D0
 * GPIO33   - D1
 * GPIO34   - D2
 * GPIO35   - D3
 * GPIO36   - D4
 * GPIO37   - D5
 * GPIO38   - D6
 * GPIO39   - D7
 *
 * GPIO54   - LITE
 * GPIO53   - RST
 * GPIO52   - RD
 * GPIO51   - WR
 * GPIO50   - D/~C
 * GPIO49   - CS
*/

#define DEVICE_SYSCLK_FREQ 200000000

/* Screen size */
#define MAX_SCREEN_X     480
#define MAX_SCREEN_Y     320
#define MIN_SCREEN_X     0
#define MIN_SCREEN_Y     0
#define SCREEN_SIZE      153600

/* LCD MACROS */
#define CS_LOW      GpioDataRegs.GPBCLEAR.bit.GPIO49    = 1;
#define CS_HIGH     GpioDataRegs.GPBSET.bit.GPIO49      = 1;
#define RST_LOW     GpioDataRegs.GPBCLEAR.bit.GPIO53    = 1;
#define RST_HIGH    GpioDataRegs.GPBSET.bit.GPIO53      = 1;
#define C_D_LOW     GpioDataRegs.GPBCLEAR.bit.GPIO50   = 1;
#define C_D_HIGH    GpioDataRegs.GPBSET.bit.GPIO50     = 1;
#define WR_HIGH     GpioDataRegs.GPBSET.bit.GPIO51     = 1;
#define WR_LOW      GpioDataRegs.GPBCLEAR.bit.GPIO51   = 1;

/* LCD colors */
#define LCD_WHITE          0xFFFF
#define LCD_BLACK          0x0000
#define LCD_BLUE           0x0197
#define LCD_RED            0xF800
#define LCD_MAGENTA        0xF81F
#define LCD_GREEN          0x07E0
#define LCD_CYAN           0x7FFF
#define LCD_YELLOW         0xFFE0
#define LCD_GRAY           0x2104
#define LCD_PURPLE         0xF11F
#define LCD_ORANGE         0xFD20
#define LCD_PINK           0xfdba
#define LCD_OLIVE          0xdfe4

/* HX registers definition */
#define DISPLAY_OFF                         0x28
#define DISPLAY_ON                          0x29
#define WRITE_DISPLAY_BRIGHTNESS_VALUE      0x51
#define ALL_PIXEL_OFF                       0x22
#define MEMORY_WRITE                        0x2C
#define WRITE_MEMORY_CONT                   0x3C
#define PIXEL_FORMAT                        0x3A
#define MADCTL                              0x36
#define SLEEP_OUT                           0x11
#define SET_PANEL                           0xCC
#define WRITE_CONTROL_DISPLAY               0x53
#define PARTIAL_MODE_ON                     0x12
#define COLUMN_ADDR_SET                     0x2A
#define PAGE_ADDR_SET                       0x2B
#define SETCABC                             0xC9
#define NORON                               0x13
#define SETOSC                              0xB0

/* STMP TSCRN registers definitions*/
#define SLAVE_ADDRESS                       0x41
#define CHIP_ID                             0x00
#define INT_CTRL                            0x09
#define INT_EN                              0x0A
#define ADC_CTRL1                           0x20
#define ADC_CTRL2                           0x21
#define FIFO_SIZE                           0x46
#define FIFO_TH                             0x4A
#define FIFO_CTRL_STA                       0x4B    //write 1 to reset fifo
#define TSC_CTRL                            0x40
#define TSC_CFG                             0x41
#define GPIO_INT_EN                         0x0C
#define SPI_CFG                             0x08
#define SYS_CTRL2                           0x04
#define INT_STA                             0x0B
#define TSC_I_DRIVE                         0x58
#define FIFO_STA                            0x4B
#define SYS_CTRL1                           0x03
#define TSC_DATA_X                          0x4D
#define TSC_DATA_Y                          0x4F
#define WDW_TR_X                            0x42
#define WDW_TR_Y                            0x44
#define WDW_BL_X                            0x46
#define WDW_BL_Y                            0x48

/* STMPE 4-WIRE Touchscreen Controller */
#define STMPE_SYS_CTRL1             0x03
#define STMPE_SYS_CTRL1_RESET       0x02
#define STMPE_SYS_CTRL2             0x04
#define STMPE_TSC_CTRL              0x40
#define STMPE_TSC_CTRL_EN           0x01
#define STMPE_TSC_CTRL_XYZ          0x00
#define STMPE_TSC_CTRL_XY           0x01
#define STMPE_TSC_CTRL_X            0x02
#define STMPE_INT_CTRL              0x09
#define STMPE_INT_CTRL_POL_HIGH     0x04
#define STMPE_INT_CTRL_POL_LOW      0x00
#define STMPE_INT_CTRL_EDGE         0x02
#define STMPE_INT_CTRL_LEVEL        0x00
#define STMPE_INT_CTRL_ENABLE       0x01
#define STMPE_INT_CTRL_DISABLE      0x00
#define STMPE_INT_EN                0x0A
#define STMPE_INT_EN_TOUCHDET       0x01
#define STMPE_INT_EN_FIFOTH         0x02
#define STMPE_INT_EN_FIFOOF         0x04
#define STMPE_INT_EN_FIFOFULL       0x08
#define STMPE_INT_EN_FIFOEMPTY      0x10
#define STMPE_INT_EN_ADC            0x40
#define STMPE_INT_EN_GPIO           0x80
#define STMPE_INT_STA               0x0B
#define STMPE_INT_STA_TOUCHDET      0x01
#define STMPE_ADC_CTRL1             0x20
#define STMPE_ADC_CTRL1_12BIT       0x08
#define STMPE_ADC_CTRL1_10BIT       0x00
#define STMPE_ADC_CTRL2             0x21
#define STMPE_ADC_CTRL2_1_625MHZ    0x00
#define STMPE_ADC_CTRL2_3_25MHZ     0x01
#define STMPE_ADC_CTRL2_6_5MHZ      0x02
#define STMPE_TSC_CFG               0x41
#define STMPE_TSC_CFG_1SAMPLE       0x00
#define STMPE_TSC_CFG_2SAMPLE       0x40
#define STMPE_TSC_CFG_4SAMPLE       0x80
#define STMPE_TSC_CFG_8SAMPLE       0xC0
#define STMPE_TSC_CFG_DELAY_10US    0x00
#define STMPE_TSC_CFG_DELAY_50US    0x08
#define STMPE_TSC_CFG_DELAY_100US   0x10
#define STMPE_TSC_CFG_DELAY_500US   0x18
#define STMPE_TSC_CFG_DELAY_1MS     0x20
#define STMPE_TSC_CFG_DELAY_5MS     0x28
#define STMPE_TSC_CFG_DELAY_10MS    0x30
#define STMPE_TSC_CFG_DELAY_50MS    0x38
#define STMPE_TSC_CFG_SETTLE_10US   0x00
#define STMPE_TSC_CFG_SETTLE_100US  0x01
#define STMPE_TSC_CFG_SETTLE_500US  0x02
#define STMPE_TSC_CFG_SETTLE_1MS    0x03
#define STMPE_TSC_CFG_SETTLE_5MS    0x04
#define STMPE_TSC_CFG_SETTLE_10MS   0x05
#define STMPE_TSC_CFG_SETTLE_50MS   0x06
#define STMPE_TSC_CFG_SETTLE_100MS  0x07
#define STMPE_FIFO_TH               0x4A
#define STMPE_FIFO_SIZE             0x4C
#define STMPE_FIFO_STA              0x4B
#define STMPE_FIFO_STA_RESET        0x01
#define STMPE_FIFO_STA_OFLOW        0x80
#define STMPE_FIFO_STA_FULL         0x40
#define STMPE_FIFO_STA_EMPTY        0x20
#define STMPE_FIFO_STA_THTRIG       0x10
#define STMPE_TSC_I_DRIVE           0x58
#define STMPE_TSC_I_DRIVE_20MA      0x00
#define STMPE_TSC_I_DRIVE_50MA      0x01
#define STMPE_TSC_DATA_X            0x4D
#define STMPE_TSC_DATA_Y            0x4F
#define STMPE_TSC_DATA              0xD7
#define STMPE_TSC_FRACTION_Z        0x56
#define STMPE_GPIO_SET_PIN          0x10
#define STMPE_GPIO_CLR_PIN          0x11
#define STMPE_GPIO_DIR              0x13
#define STMPE_GPIO_ALT_FUNCT        0x17
#define STMPE_WDW_TR_X              0x42
#define STMPE_WDW_TR_Y              0x44
#define STMPE_WDW_BL_X              0x46
#define STMPE_WDW_BL_Y              0x48

typedef enum{
    WHITE = 1,
    BLACK = 0
}bg;

static int16_t SET_COLUMN_ADDRESS[]         = {0x00, 0x00, 0x01, 0x3F, -1};
static int16_t SET_PAGE_ADDRESS[]           = {0x00, 0x00, 0x01, 0xdf, -1};
static int16_t test_SET_COLUMN_ADDRESS[]    = {0x00, 100, 0, 200, -1};
static int16_t test_SET_PAGE_ADDRESS[]      = {0x00, 100, 0, 200, -1};
static int16_t SETCABC_PARAM[]              = {0xFD, 0xFF, -1};
static int16_t SETOSC_PARAM[]               = {0xFF, 0x01, -1};

/*  STANDARD LCD FUNCTION DECLARATIONS  */
void LCD_Init();
void LCD_initGPIO();
void LCD_DrawRectangle(uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd, uint16_t Color);
void LCD_DrawRectangleBrdr(uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd, uint16_t borderWidth, uint16_t fillColor, uint16_t borderColor);
void LCD_Clear(uint16_t Color);
inline void LCD_Write_Data_Only(uint16_t data);
void LCD_WriteIndex(uint16_t index);
inline void LCD_WriteData(uint16_t byte);
inline void LCD_WriteReg_param(uint16_t LCD_Reg, int16_t* LCD_RegValues);
inline void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue);
inline void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos );
void LCD_SetPoint(uint16_t Xpos, uint16_t Ypos, uint16_t color);
void PutChar( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint16_t fontSize, uint16_t charColor);
void LCD_Text(uint16_t Xpos, uint16_t Ypos, char *str, uint16_t fontSize, uint16_t Color);
void LCD_DrawSprite(uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd, const char* sprite);
void LCD_DrawChessSprite(uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd, const char* sprite, bg background);

/*  I2C LCD TOUCHSCREEN FUNCTIONS DECLARATIONS  */
void TCHSCRN_I2C_LCD_init();
void TCHSCRN_I2C_SendBytes(char regAddr, char *dataVals, uint16_t bytesToWrite);
void TCHSCRN_I2C_SendByte(char regAddr, char dataVal);
uint32_t TCHSCRN_I2C_ReadByte(char regAddr);
uint32_t TCHSCRN_I2C_ReadBytes(char regAddr, char *dataVals, uint16_t bytesToWrite);

/*  SPI LCD TOUCHSCREEN FUNCTIONS DECLARATIONS  */
void LCD_init_spib_hs();
uint32_t LCD_ts_rd(char regAddr, uint16_t bytesToRead);
void LCD_ts_wr(char regAddr, char data, uint16_t bytesToWrite);
void LCD_tsc_init();


#endif /* LCDLIB_H_ */
