/*
 * I2C_driver.c
 *
 *  Created on: Mar 10, 2020
 *      Author: drive
 */
#include <I2C_driver.h>
#include <F2837xD_Device.h>

static const Uint16 IdealModClockFrqMHz = 12;   //Ideal module clock frequency for I2C
static inline void InitI2CGpio();
static inline void SetClockDivides(float32 sysClkMHz, float32 I2CClkKHz);

/*
 *  Initializes the I2C to run in Master Mode for a One-To-One connection
 * <param="slaveAddress">Address of the slave device to write to</param>
 * <param="sysClkMhz">System Clock Frequency in Mhz</param>
 * <param="I2CClkKHz">Desired I2C Clock Frequency in KHz</param>
 */
void I2C_Master_Init(uint16_t slaveAddress, float sysClkMhz, float I2CClkKHz)
{
    // Init GPIO
    InitI2CGpio();

    EALLOW;

    CpuSysRegs.PCLKCR9.bit.I2C_B    = 1;        // Enable Clock for I2C

    I2cbRegs.I2CMDR.bit.IRS         = 0;        // Put I2C into Reset Mode

//    I2cbRegs.I2CSAR.bit.SAR = slaveAddress; // Set Slave Address

    SetClockDivides(sysClkMhz, I2CClkKHz);      // Set Clocks

    I2cbRegs.I2CMDR.bit.IRS = 1;                // Release from Reset Mode

    EDIS;
}


void I2C_SendBytes(char slaveAddr, char regAddr, char *dataVals, uint16_t bytesToWrite)
{

    I2cbRegs.I2CSAR.bit.SAR = slaveAddr;        //Set slave address

    I2cbRegs.I2CMDR.all = 0x66A0;               //Set to Master, Repeat Mode, TRX, FREE, Start

    while(I2cbRegs.I2CMDR.bit.STT);             //Wait for start condition to be cleared

    while(!I2cbRegs.I2CSTR.bit.XRDY);           //Wait if Transmit is not ready
    I2cbRegs.I2CDXR.bit.DATA = regAddr;         //Write regAddr value to I2C

    //Write dataVals to I2C
    for (Uint16 i = 0; i < bytesToWrite; i++){
        while(!I2cbRegs.I2CSTR.bit.XRDY);       //Wait if Transmit is not ready
        I2cbRegs.I2CDXR.bit.DATA = dataVals[i]; //Write dataVal value to I2C
    }

    I2cbRegs.I2CMDR.bit.STP = 1;                //Send Stop Bit
}

void I2C_SendByte(char slaveAddr, char regAddr, char dataVal)
{
    while(I2cbRegs.I2CMDR.bit.STP);         //Wait if Stop bit is not cleared
    while(I2cbRegs.I2CSTR.bit.BB);          //Wait if I2c module is busy

    I2cbRegs.I2CSAR.bit.SAR     = slaveAddr;//Set slave address

    I2cbRegs.I2CMDR.all         = 0x66A0;   //Set to Master, Repeat Mode, TRX, FREE, Start

    while(I2cbRegs.I2CMDR.bit.STT);         //Wait for start condition to be cleared

    while(!I2cbRegs.I2CSTR.bit.XRDY);       //Wait if Transmit is not ready
    I2cbRegs.I2CDXR.bit.DATA    = regAddr;  //Write regAddr value to I2C

    while(!I2cbRegs.I2CSTR.bit.XRDY);       //Wait if Transmit is not ready
    I2cbRegs.I2CDXR.bit.DATA    = dataVal;  //Write dataValue to I2C

    I2cbRegs.I2CMDR.bit.STP     = 1;        //Send Stop Bit
}

uint32_t I2C_ReadByte(char slaveAddr, char regAddr){
    uint32_t retData = 0;
    while(I2cbRegs.I2CMDR.bit.STP);         //Wait if Stop bit is not cleared
    while(I2cbRegs.I2CSTR.bit.BB);          //Wait if I2c module is busy

    I2cbRegs.I2CSAR.bit.SAR = slaveAddr;//Set slave address

    I2cbRegs.I2CMDR.all = 0x66A0;       //Set to Master, Repeat Mode, TRX, FREE, Start
    while(I2cbRegs.I2CMDR.bit.STT);     //Wait for start condition to be cleared

    while(!I2cbRegs.I2CSTR.bit.XRDY);   //Wait till transmit is ready
    I2cbRegs.I2CDXR.bit.DATA = regAddr; //Send register to be read from

    while(!I2cbRegs.I2CSTR.bit.XRDY);   //Wait till transmit is ready
    I2cbRegs.I2CMDR.bit.TRX     = 0;    //Set to Receiver
    I2cbRegs.I2CMDR.bit.NACKMOD = 1;    //Send NACK
    I2cbRegs.I2CMDR.bit.STT     = 1;    //Send start condition
    while(I2cbRegs.I2CMDR.bit.STT);     //Wait for start condition to be cleared
    while(!I2cbRegs.I2CSTR.bit.RRDY);   //Wait till read is ready

    retData = I2cbRegs.I2CDRR.bit.DATA; //Receive data

    I2cbRegs.I2CMDR.bit.STP     = 1;

    return retData;
}

uint32_t I2C_ReadBytes(char slaveAddr, char regAddr, char *dataVals, uint16_t bytesToRead)
{
    while(I2cbRegs.I2CMDR.bit.STP);     //Wait if Stop bit is not cleared
    while(I2cbRegs.I2CSTR.bit.BB);      //Wait if I2c module is busy

    I2cbRegs.I2CSAR.bit.SAR = slaveAddr;//Set slave address

    I2cbRegs.I2CMDR.all = 0x66A0;       //Set to Master, Repeat Mode, TRX, FREE, Start
    while(I2cbRegs.I2CMDR.bit.STT);     //Wait for start condition to be cleared

    while(!I2cbRegs.I2CSTR.bit.XRDY);   //Wait till transmit is ready
    I2cbRegs.I2CDXR.bit.DATA = regAddr; //Send register to be read from

    for (int i = 0; i<bytesToRead; i++){
        while(!I2cbRegs.I2CSTR.bit.XRDY);   //Wait till transmit is ready
        I2cbRegs.I2CMDR.bit.TRX     = 0;    //Set to Receiver
        I2cbRegs.I2CMDR.bit.NACKMOD = 1;    //Send NACK
        I2cbRegs.I2CMDR.bit.STT     = 1;    //Send start condition
        while(I2cbRegs.I2CMDR.bit.STT);     //Wait for start condition to be cleared
        while(!I2cbRegs.I2CSTR.bit.RRDY);   //Wait till read is ready
        dataVals[i] = I2cbRegs.I2CDRR.bit.DATA; //Receive data
    }
    I2cbRegs.I2CMDR.bit.STP     = 1;
    return 1;       //SUCCESS
}

/*
 *  Calculates and sets the ClockDivides for the I2C Module
 * <param="sysClkMhz">System Clock Frequency in Mhz</param>
 * <param="I2CClkKHz">Desired I2C Clock Frequency in KHz</param>
 */
static inline void SetClockDivides(float32 sysClkMHz, float32 I2CClkKHz)
{
    /* Calculate Module Clock Frequency - Must be between 7-12 MHz
     * Module Clock Frequency = sysClkMhz/(IPSC + 1)
     */
    Uint16 IPSC = (Uint16)(sysClkMHz/IdealModClockFrqMHz);

    /* Calculate Divide Downs for SCL
     * FreqMClk = sysClkMHz/((IPSC + 1)[(ICCL + d) + (ICCH + d)])
     *
     * Assume an even clock size -> ICCH == ICCL
     * ICCL = ICCH = sysclkMHz/(2000 * I2CClkKHz * (IPSC + 1)) - d
     */

    // Find value for d
    Uint16 d = 5;

    if (IPSC < 2)
    {
        d++;
        if (IPSC < 1)
        {
            d++;
        }
    }

    Uint16 ICCLH = (Uint16)(1000 * sysClkMHz/(2 * I2CClkKHz * (IPSC + 1)) - d);

    // Set values
    I2cbRegs.I2CPSC.all = IPSC;
    I2cbRegs.I2CCLKL = ICCLH;
    I2cbRegs.I2CCLKH = ICCLH;
}


/* Initializes the GPIO for the I2C */
static void InitI2CGpio()
{
    EALLOW;

    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;       // Enable pull-up for GPIO40 (SDAA)
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 0;       // Enable pull-up for GPIO41 (SCLA)

    GpioCtrlRegs.GPBQSEL1.bit.GPIO40 = 3;  // Asynch input GPIO40 (SDAA)
    GpioCtrlRegs.GPBQSEL1.bit.GPIO41 = 3;  // Asynch input GPIO41 (SCLA)

    GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = 1;   // Configure GPIO40 for SDAA operation
    GpioCtrlRegs.GPBGMUX1.bit.GPIO41 = 1;   // Configure GPIO41 for SCLA operation
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 2;
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 2;

    //EDIS;
}

