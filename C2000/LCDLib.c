/*
 * LCDLib.c
 *
 *  Created on: Nov 11, 2019
 *      Author: Dave
 */

#include "LCDLib.h"
#include "AsciiLib.h"
#include "gpio.h"
#include "chess.h"


void LCD_initGPIO()
{
    EALLOW;
    //GPyGMUXn configurations
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);  //D7
    GPIO_setDirectionMode(38, GPIO_DIR_MODE_OUT);  //D6
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_OUT);  //D5
    GPIO_setDirectionMode(36, GPIO_DIR_MODE_OUT);  //D4
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_OUT);  //D3
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);  //D2
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);  //D1
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_OUT);  //D0

    GPIO_setDirectionMode(54, GPIO_DIR_MODE_OUT);  //LITE
    GPIO_setDirectionMode(53, GPIO_DIR_MODE_OUT);  //RST
    GPIO_setDirectionMode(52, GPIO_DIR_MODE_OUT);  //WR
    GPIO_setDirectionMode(51, GPIO_DIR_MODE_OUT);  //D/~C
    GPIO_setDirectionMode(50, GPIO_DIR_MODE_OUT);  //RD
    GPIO_setDirectionMode(49, GPIO_DIR_MODE_OUT);  //CS

    GPIO_WritePin(39, 0);  //D7
    GPIO_WritePin(38, 0);  //D6
    GPIO_WritePin(37, 0);  //D5
    GPIO_WritePin(36, 0);  //D4
    GPIO_WritePin(35, 0);  //D3
    GPIO_WritePin(34, 0);  //D2
    GPIO_WritePin(33, 0);  //D1
    GPIO_WritePin(32, 0);  //D0

    GPIO_WritePin(54, 1);  //LITE
    GPIO_WritePin(53, 1);  //RST
    GPIO_WritePin(52, 1);  //RD
    GPIO_WritePin(51, 0);  //WR
    GPIO_WritePin(50, 0);  //D/~C
    GPIO_WritePin(49, 1);  //CS
}

static void LCD_reset()
{
    RST_LOW;
    DELAY_US(1000);
    RST_HIGH;
    DELAY_US(1000);
}

void LCD_WriteData(uint16_t data)
{
    char temp = data;

    WR_LOW;
    GPIO_WritePin(32, temp & 0x01);  //D0
    temp >>= 1;
    GPIO_WritePin(33, temp & 0x01);  //D1
    temp >>= 1;
    GPIO_WritePin(34, temp & 0x01);  //D2
    temp >>= 1;
    GPIO_WritePin(35, temp & 0x01);  //D3
    temp >>= 1;
    GPIO_WritePin(36, temp & 0x01);  //D4
    temp >>= 1;
    GPIO_WritePin(37, temp & 0x01);  //D5
    temp >>= 1;
    GPIO_WritePin(38, temp & 0x01);  //D6
    temp >>= 1;
    GPIO_WritePin(39, temp & 0x01);  //D7
    WR_HIGH;
}


void LCD_DrawRectangle(uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd, uint16_t Color)
{
    yEnd--;
    xEnd--;
    //will discard rectangles with any coordinate out of bounds
    if((xStart >= MIN_SCREEN_X) && (xEnd < MAX_SCREEN_X) && (yStart >= MIN_SCREEN_Y) && (yEnd < MAX_SCREEN_Y)){
        LCD_WriteIndex(COLUMN_ADDR_SET);
        CS_LOW;
        C_D_HIGH;
        LCD_WriteData((xStart>>8));
        LCD_WriteData(  xStart);
        LCD_WriteData(  (xEnd>>8));
        LCD_WriteData(   xEnd);
        CS_HIGH;

        LCD_WriteIndex(PAGE_ADDR_SET);
        CS_LOW;
        C_D_HIGH;
        LCD_WriteData(   (yStart>>8));
        LCD_WriteData(    yStart);
        LCD_WriteData(   (yEnd>>8));
        LCD_WriteData(    yEnd);
        CS_HIGH;

        //Start data transmission
        LCD_WriteIndex(MEMORY_WRITE);
        CS_LOW;
        C_D_HIGH;
        for (int i = 0; i<xEnd-xStart+1; i++){
            for (int j = 0; j<yEnd-yStart+1; j++){
                LCD_Write_Data_Only(Color);
            }
        }
        CS_HIGH;
    }
}

void LCD_DrawRectangleBrdr(uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd, uint16_t borderWidth, uint16_t fillColor, uint16_t borderColor)
{
    LCD_DrawRectangle(xStart - borderWidth, xEnd + borderWidth,
                      yStart - borderWidth, yStart, borderColor);

    LCD_DrawRectangle(xStart - borderWidth, xEnd + borderWidth,
                      yEnd, yEnd+borderWidth, borderColor);

    LCD_DrawRectangle(xStart - borderWidth, xStart,
                      yStart, yEnd, borderColor);

    LCD_DrawRectangle(xEnd, xEnd+borderWidth,
                      yStart, yEnd, borderColor);
    yEnd--;
    xEnd--;
    //will discard rectangles with any coordinate out of bounds
    if((xStart >= MIN_SCREEN_X) && (xEnd < MAX_SCREEN_X) && (yStart >= MIN_SCREEN_Y) && (yEnd < MAX_SCREEN_Y)){
        LCD_WriteIndex(COLUMN_ADDR_SET);
        CS_LOW;
        C_D_HIGH;
        LCD_WriteData((xStart>>8));
        LCD_WriteData(  xStart);
        LCD_WriteData(  (xEnd>>8));
        LCD_WriteData(   xEnd);
        CS_HIGH;

        LCD_WriteIndex(PAGE_ADDR_SET);
        CS_LOW;
        C_D_HIGH;
        LCD_WriteData(   (yStart>>8));
        LCD_WriteData(    yStart);
        LCD_WriteData(   (yEnd>>8));
        LCD_WriteData(    yEnd);
        CS_HIGH;

        //Start data transmission
        LCD_WriteIndex(MEMORY_WRITE);
        CS_LOW;
        C_D_HIGH;
        for (int i = 0; i<xEnd-xStart+1; i++){
            for (int j = 0; j<yEnd-yStart+1; j++){
                LCD_Write_Data_Only(fillColor);
            }
        }
        CS_HIGH;
    }
}


void LCD_DrawSprite(uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd, const char* sprite)
{
    uint16_t pixel;
    yEnd--;
    xEnd--;
    //will discard rectangles with any coordinate out of bounds
    if((xStart >= MIN_SCREEN_X) && (xEnd < MAX_SCREEN_X) && (yStart >= MIN_SCREEN_Y) && (yEnd < MAX_SCREEN_Y)){
        LCD_WriteIndex(COLUMN_ADDR_SET);
        CS_LOW;
        C_D_HIGH;
        LCD_WriteData((xStart>>8));
        LCD_WriteData(  xStart);
        LCD_WriteData(  (xEnd>>8));
        LCD_WriteData(   xEnd);
        CS_HIGH;

        LCD_WriteIndex(PAGE_ADDR_SET);
        CS_LOW;
        C_D_HIGH;
        LCD_WriteData(   (yStart>>8));
        LCD_WriteData(    yStart);
        LCD_WriteData(   (yEnd>>8));
        LCD_WriteData(    yEnd);
        CS_HIGH;

        //Start data transmission
        LCD_WriteIndex(MEMORY_WRITE);
        CS_LOW;
        C_D_HIGH;
        for (int i = 0; i<xEnd-xStart+1; i++){
           for (int j = 0; j<yEnd-yStart+1; j++){
               pixel = (uint16_t)(*sprite<<8 | *(sprite+1));
               LCD_Write_Data_Only(pixel);
               sprite = sprite + 2;
           }
        }
        CS_HIGH;
    }
}

void LCD_DrawChessSprite(uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd, const char* sprite, bg background)
{
    uint16_t pixel;
    yEnd--;
    xEnd--;
    //will discard rectangles with any coordinate out of bounds
    if((xStart >= MIN_SCREEN_X) && (xEnd < MAX_SCREEN_X) && (yStart >= MIN_SCREEN_Y) && (yEnd < MAX_SCREEN_Y)){
        LCD_WriteIndex(COLUMN_ADDR_SET);
        CS_LOW;
        C_D_HIGH;
        LCD_WriteData((xStart>>8));
        LCD_WriteData(  xStart);
        LCD_WriteData(  (xEnd>>8));
        LCD_WriteData(   xEnd);
        CS_HIGH;

        LCD_WriteIndex(PAGE_ADDR_SET);
        CS_LOW;
        C_D_HIGH;
        LCD_WriteData(   (yStart>>8));
        LCD_WriteData(    yStart);
        LCD_WriteData(   (yEnd>>8));
        LCD_WriteData(    yEnd);
        CS_HIGH;

        //Start data transmission
        LCD_WriteIndex(MEMORY_WRITE);
        CS_LOW;
        C_D_HIGH;
        for (int i = 0; i<xEnd-xStart+1; i++){
           for (int j = 0; j<yEnd-yStart+1; j++){
               pixel = (uint16_t)(*sprite<<8 | *(sprite+1));
               if (pixel != 0x2104){
                   LCD_Write_Data_Only(pixel);
               }else if(background == WHITE){
                   LCD_Write_Data_Only(LCD_WHITE);
               }else{
                   LCD_Write_Data_Only(LCD_BLACK);
               }
               sprite = sprite + 2;
           }
        }
        CS_HIGH;
    }
}


void LCD_Clear(uint16_t Color)
{
    LCD_WriteIndex(PAGE_ADDR_SET);
    CS_LOW;
    C_D_HIGH;
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x01);
    LCD_WriteData(0x3F);
    CS_HIGH;

    LCD_WriteIndex(COLUMN_ADDR_SET);
    CS_LOW;
    C_D_HIGH;
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x01);
    LCD_WriteData(0xDF);
    CS_HIGH;

    LCD_WriteIndex(MEMORY_WRITE);
    CS_LOW;
    C_D_HIGH;
    for (long i = 0; i<SCREEN_SIZE; i++){
        LCD_Write_Data_Only(Color);
    }
    CS_HIGH;
}

void PutChar( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint16_t fontSize, uint16_t charColor)
{
    uint16_t refX = Xpos;
    uint16_t refY = Ypos;
    uint16_t i, j;
    uint8_t buffer[16], tmp_char;

    GetASCIICode(buffer,ASCI);  /* get font data */
    for( i=0; i<16; i++ ){
        tmp_char = buffer[i];
        for( j=0; j<8; j++ ){
            if( (tmp_char >> 7 - j) & 0x01 == 0x01 ){
                LCD_DrawRectangle(refX + j, refX + j + fontSize*2, refY + i,  refY + i + fontSize*2, charColor );
            }
            refX = refX + fontSize;
        }
        refY = refY + fontSize;
        refX = Xpos;
    }

}

void LCD_Text(uint16_t Xpos, uint16_t Ypos, char *str, uint16_t fontSize, uint16_t Color)
{
    uint8_t space = 10;
    uint8_t TempChar;
    do{
        TempChar = *str++;
        PutChar( Xpos, Ypos, TempChar, fontSize, Color);
        if( Xpos < MAX_SCREEN_X - 8*fontSize + space){
            Xpos += 8 * fontSize + space;
        }
        else if ( Ypos < 16 * fontSize + space){
            Xpos = 0;
            Ypos += 16 * fontSize + space;
        }
        else{
            Xpos = 0;
            Ypos = 0;
        }
    }while ( *str != 0 );
}


inline void LCD_WriteReg_param(uint16_t LCD_Reg, int16_t* LCD_RegValues)
{
    /* Write 16-bit Index */
    CS_LOW;
    C_D_LOW;
    LCD_WriteData(LCD_Reg);
    C_D_HIGH;
    do{
        LCD_WriteData(*LCD_RegValues);
        LCD_RegValues++;
    }while(*LCD_RegValues != -1);
    CS_HIGH;
}

inline void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
{
    /* Write 16-bit Index */
    CS_LOW;
    C_D_LOW;
    LCD_WriteData(LCD_Reg);
    C_D_HIGH;
    LCD_WriteData(LCD_RegValue);
    CS_HIGH;
}

void LCD_WriteIndex(uint16_t index)
{
    CS_LOW;
    C_D_LOW;
    LCD_WriteData(index);
    CS_HIGH;
}

inline void LCD_Write_Data_Only(uint16_t data)
{
    LCD_WriteData( 0x00FF & (data >> 8));
    LCD_WriteData( 0x00FF &  data);
}

void LCD_Init()
{
    LCD_reset();

//BACKLIGHTS
//    LCD_WriteIndex(PARTIAL_MODE_ON);
//    CS_HIGH;
    LCD_WriteIndex(NORON);
    LCD_WriteReg(WRITE_CONTROL_DISPLAY, 0x2C);
    //LCD_WriteReg(WRITE_DISPLAY_BRIGHTNESS_VALUE, 0xFF);
    LCD_WriteReg_param(SETCABC, SETCABC_PARAM);
    LCD_WriteReg(MADCTL, 0x28);
    LCD_WriteReg(SET_PANEL, 0x02);
    LCD_WriteReg(PIXEL_FORMAT, 0x05);       //65k colors
    LCD_WriteReg_param(SETOSC, SETOSC_PARAM);
    LCD_WriteIndex(SLEEP_OUT);

    DELAY_US(120000);

    LCD_Clear(LCD_BLACK);
    drawChessBoard();
    LCD_WriteIndex(DISPLAY_ON);
}

/*  I2C INTERFACE FOR TOUCH SCREEN*/
void TCHSCRN_I2C_LCD_init()
{
    I2C_Master_Init(SLAVE_ADDRESS, 200, 100);

    I2C_SendByte(SLAVE_ADDRESS, STMPE_SYS_CTRL1, STMPE_SYS_CTRL1_RESET);
    DELAY_US(100000);

    I2C_SendByte(SLAVE_ADDRESS, STMPE_SYS_CTRL2, 0x0);                  // turn on clocks
    I2C_SendByte(SLAVE_ADDRESS, STMPE_TSC_CTRL, STMPE_TSC_CTRL_XY |
                                STMPE_TSC_CTRL_EN);                     // XYZ and enable
    I2C_SendByte(SLAVE_ADDRESS, STMPE_INT_EN, STMPE_INT_EN_TOUCHDET);
    I2C_SendByte(SLAVE_ADDRESS, STMPE_ADC_CTRL1, STMPE_ADC_CTRL1_10BIT |
                                (0x5 << 4));                            // 96 clocks per conversion
    I2C_SendByte(SLAVE_ADDRESS, STMPE_ADC_CTRL2, STMPE_ADC_CTRL2_6_5MHZ);
    I2C_SendByte(SLAVE_ADDRESS, STMPE_TSC_CFG, STMPE_TSC_CFG_8SAMPLE | STMPE_TSC_CFG_DELAY_1MS |
                                 STMPE_TSC_CFG_SETTLE_5MS);
//    I2C_SendByte(SLAVE_ADDRESS, STMPE_TSC_FRACTION_Z, 0x6);/////////////////////////////

    I2C_SendByte(SLAVE_ADDRESS, STMPE_FIFO_TH, 1);
    I2C_SendByte(SLAVE_ADDRESS, STMPE_FIFO_STA, STMPE_FIFO_STA_RESET);
    I2C_SendByte(SLAVE_ADDRESS, STMPE_FIFO_STA, 0);                     // unreset
    I2C_SendByte(SLAVE_ADDRESS, STMPE_TSC_I_DRIVE, STMPE_TSC_I_DRIVE_50MA);
    I2C_SendByte(SLAVE_ADDRESS, STMPE_INT_STA, 0xFF);                   // reset all ints
    I2C_SendByte(SLAVE_ADDRESS, STMPE_INT_CTRL, STMPE_INT_CTRL_EDGE |
                                STMPE_INT_CTRL_POL_HIGH | STMPE_INT_CTRL_ENABLE);

}

void TCHSCRN_I2C_SendBytes(char regAddr, char *dataVals, uint16_t bytesToWrite)
{
    I2C_SendBytes(SLAVE_ADDRESS, regAddr, dataVals, bytesToWrite);
}

void TCHSCRN_I2C_SendByte(char regAddr, char dataVal)
{
    I2C_SendByte(SLAVE_ADDRESS, regAddr, dataVal);
}

uint32_t TCHSCRN_I2C_ReadBytes(char regAddr, char *dataVals, uint16_t bytesToWrite)
{
    return I2C_ReadBytes(SLAVE_ADDRESS, regAddr, dataVals, bytesToWrite);
}

uint32_t TCHSCRN_I2C_ReadByte(char regAddr)
{
    return I2C_ReadByte(SLAVE_ADDRESS, regAddr);
}
