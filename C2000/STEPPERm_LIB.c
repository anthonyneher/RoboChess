/*
 * STEPPERm_LIB.c
 *
 *  Created on: Feb 24, 2020
 *      Author: DAVE
 */

#include "STEPPERm_LIB.h"

volatile uint16_t waitflag;

void stepper_init_spib(){
    /* Initialize SPI-B Pins */
    //GPyGMUXn configurations
    EALLOW;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 0b11;    //SPISIMOB
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 0b11;    //SPISOMIB
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 0b11;    //SPICLKB

    //GPyMUXn configurations
    EALLOW;
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0b11;     //SPISIMOB
    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0b11;     //SPISOMIB
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0b11;     //SPICLKB

    //GPyDIRn configurations
    EALLOW;
    GpioCtrlRegs.GPBDIR.bit.GPIO63  = 1;        //Configure MOSI as an output
    GpioCtrlRegs.GPCDIR.bit.GPIO64  = 0;        //Configure MISO in as an input
    GpioCtrlRegs.GPCDIR.bit.GPIO65  = 1;        //Configure SPI CLK as an output

    GPIO_setDirectionMode(STEPPER_CS_PIN, GPIO_DIR_MODE_OUT);  //CS

    EALLOW;
    GpioDataRegs.GPCSET.bit.GPIO83  = 1;        //initialize ~CS high


    /* Initialize SPI-B HS mode */
    EALLOW;
    //SPICCR configurations
    SpibRegs.SPICCR.bit.SPISWRESET      = 0;    // Set reset low before configuration changes
    SpibRegs.SPICCR.bit.CLKPOLARITY     = 0;    // Clock polarity rising
    SpibRegs.SPICCR.bit.HS_MODE         = 0;    // Turn off High_Speed mode
    SpibRegs.SPICCR.bit.SPICHAR         = 0x0F; // 16-bit character

    //SPICTL configurations
    SpibRegs.SPICTL.bit.CLK_PHASE       = 1;    // Clock phase DELAYED by 1/2 cycle
    SpibRegs.SPICTL.bit.MASTER_SLAVE    = 1;    // Enable master mode
    SpibRegs.SPICTL.bit.TALK            = 1;    // Enable transmission (Talk)
    SpibRegs.SPICTL.bit.SPIINTENA       = 0;    // SPI interrupts are disabled

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE    = 24;//19;    // Set the baud rate = LSPCLK/10

    EALLOW;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV     = 1;    // LSPCLK = 100 MHz

    SpibRegs.SPICCR.bit.SPISWRESET      = 1;    // Release the SPI from reset

    SpibRegs.SPIPRI.bit.FREE            = 1;    // Set FREE bit
                                                // Halting on a breakpoint will not halt the SPI
     /*******************************************************************/
}

void stepper_spi_wr(uint16_t regAddr, uint16_t data){
    uint16_t readData;
    STEPPER_CS_HIGH;     //Make ~CS high
    DELAY_US(1);

    SpibRegs.SPITXBUF = 0x7FFF & ((regAddr<<12) | data);   //16-bit write
    while(!SpibRegs.SPISTS.bit.INT_FLAG);
    readData = SpibRegs.SPIRXBUF;

    DELAY_US(1);
    STEPPER_CS_LOW;     //Make ~CS low
}


uint16_t stepper_spi_rd(uint16_t regAddr){
    uint16_t readData;

    STEPPER_CS_HIGH;     //Make ~CS high
    DELAY_US(1);

    SpibRegs.SPITXBUF = 0x8000 | (regAddr<<12);    //send 16-bit regAddr
    while(!SpibRegs.SPISTS.bit.INT_FLAG);
    readData = 0x0FFF & SpibRegs.SPIRXBUF;

    DELAY_US(1);
    STEPPER_CS_LOW;     //Make ~CS low
    return readData;
}

void stepper_init_settings(){
    uint16_t readReg = 0;

    /* Init PINS FOR SLEEPn, RESET, STEP, DIR */
    GPIO_setDirectionMode(SLEEP_PIN,        GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(RESET_PIN,        GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(M0_STEP_PIN,      GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(M0_DIR_PIN,       GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(M1M2_DIR_PIN,     GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(M1M2_STEP_PIN,    GPIO_DIR_MODE_OUT);
    ENABLE_OFF;
    RESET_ON;
    M1M2_nRS_ON;

    M0_STEP_OFF;
    M1M2_STEP_OFF;

    M0_DIR_FWD;
    M1M2_DIR_FWD;

    DELAY_US(100000);   //100mS

    /* START-UP phase (Manually turn off RESET and SLEEPn) */
    ENABLE_ON;          //SLEEPn off
    M1M2_nRS_OFF;
    DELAY_US(100000);   //100mS
    RESET_OFF;          //RESET off
    DELAY_US(100000);   //100mS

    M0_DIR_FWD;
    M1M2_DIR_FWD;
/*
    // MOTOR CONTROL REGISTERS
    stepper_spi_wr(TORQUE,  0x010C);
    stepper_spi_wr(OFF,     0x0032);
    stepper_spi_wr(BLANK,   0x0100);
    stepper_spi_wr(DECAY,   0x0510);
    stepper_spi_wr(STALL,   0x0A02);
    stepper_spi_wr(DRIVE,   0x0A59);
    stepper_spi_wr(CONTROL, 0x0C31);
    //    stepper_spi_wr(TORQUE,  0x0123);   //isgain = 5 | Rsense = 0.05Ohms | Ifs = 1.5 A (?)
    //    stepper_spi_wr(DRIVE,   0x0000);

    PREVENT POTENTIAL INITIAL OVERCURRENT FAUL
    DELAY_US(100000);           //1mS
    readReg = stepper_spi_rd(STATUS);
    if (readReg == 6 || readReg == 2 || readReg == 4){
        stepper_spi_wr(STATUS, 0);
    }*/
    DELAY_US(100000);           //1mS
}

uint16_t stepper_move(uint32_t *cntVar, uint32_t numOfSteps, dir direction, volatile bool *mutex){
    if (*mutex == false && *cntVar == 0){
        *mutex = true;
        *cntVar = numOfSteps*8;//64;
        if(direction == FWD){
            M0_DIR_FWD;
        }else{
            M0_DIR_BWD;
        }
        CPUTimer_startTimer(CPUTIMER0_BASE);                //Start TIMER0
        return 1;
    }else{
        return 0;
    }
}

uint16_t stepper_moveA4988(uint32_t *cntVar, uint32_t numOfSteps, dir direction, volatile bool *mutex){
    if (*mutex == false && *cntVar == 0){
            *mutex = true;
            *cntVar = numOfSteps*8;
            if(direction == FWD){
                M1M2_DIR_FWD;
            }else{
                M1M2_DIR_BWD;
            }
            CPUTimer_startTimer(CPUTIMER1_BASE);                //Start TIMER0
            return 1;
        }else{
            return 0;
        }
}

void stepper_mutex_wait(volatile bool *mutex){
    while(*mutex);
}

void stepper_mutex_delay(volatile bool *mutex){
    while(*mutex);
    waitflag = 1;
    CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);
    CPUTimer_startTimer(CPUTIMER2_BASE);
    while(waitflag);
    CPUTimer_stopTimer(CPUTIMER2_BASE);

//    DELAY_US(500000);
}
