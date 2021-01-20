/*
 * PlungerControl.c
 *
 *  Created on: Jun 22, 2020
 *      Author: anthonyneher
 */

#include "PlungerControl.h"
#include "F28x_Project.h"


//
// InitEPwm1Example - Initialize EPWM1 values
//
float calibration = 1.182;


void InitEPwm1Example()
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;//enable ePWM
    //
    // Setup TBCLK
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;       // Set timer period
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0x7;   // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = 0x04;//TB_DIV2;

    //
    // Setup shadow register load on ZERO
    //
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
    EPwm1Regs.CMPA.bit.CMPA = 675;
    EPwm1Regs.CMPB.bit.CMPB = EPWM1_MIN_CMPB;     // Set Compare B value

    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A,
                                                 // up count

    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B,
                                                 // up count

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

}

void InitMagnetGpio(void){
    EALLOW;
    GpioCtrlRegs.GPCDIR.bit.GPIO68 = 1;
}

void Magnet_On(void){
    GpioDataRegs.GPCDAT.bit.GPIO68 = 1;
}

void Magnet_Off(void){
    GpioDataRegs.GPCDAT.bit.GPIO68 = 0;
}

void Start_PWM(void){
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;//enable ePWM
}

void Stop_PWM(void){
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1=0;//enable ePWM
}

void Down(uint16_t length){
    EPwm1Regs.CMPA.bit.CMPA = 725;//down
    DELAY_US(length*10000);
    EPwm1Regs.CMPA.bit.CMPA = 675;
}

void Up(uint16_t length){
    EPwm1Regs.CMPA.bit.CMPA = 625;
    DELAY_US(length*10000);
    EPwm1Regs.CMPA.bit.CMPA = 675;
}

/*
 * Pickup method that takes in piece name and calls necessary methods to
 * pick pieces up and put them down
 */
void PickUp(uint16_t piece){
    switch(piece){
        case Pawn:
            Down(rDown1 * calibration);
            Magnet_On();
            Up(rUp1 * calibration);
            break;

        case Rook:
            Down(rDown1 * calibration);
            Magnet_On();
            Up(rUp1 * calibration);
            break;

        case Knight:
            Down(hDown1 * calibration);
            Magnet_On();
            Up(hUp1 * calibration);
            break;

        case Bishop:
            Down(bDown1 * calibration);
            Magnet_On();
            Up(bUp1 * calibration);
            break;

        case Queen:
            Down(qDown1 * calibration);
            Magnet_On();
            Up(qUp1 * calibration);
            break;

        case King:
            Down(kDown1 * calibration);
            Magnet_On();
            Up(kUp1 * calibration);
            break;
    }
}

/*
 * Pickup method that takes in piece name and calls necessary methods to
 * pick pieces up and put them down
 */
void Place(uint16_t piece){
    switch(piece){
        case Pawn:
            Down(rDown2 * calibration);
            Magnet_Off();
            Up(rUp2 * calibration);
            break;

        case Rook:
            Down(rDown2 * calibration);
            Magnet_Off();
            Up(rUp2 * calibration);
            break;

        case Knight:
            Down(hDown2 * calibration);
            Magnet_Off();
            Up(hUp2 * calibration);
            break;

        case Bishop:
            Down(bDown2 * calibration);
            Magnet_Off();
            Up(bUp2 * calibration);
            break;

        case Queen:
            Down(qDown2 * calibration);
            Magnet_Off();
            Up(qUp2 * calibration);
            break;

        case King:
            Down(kDown2 * calibration);
            Magnet_Off();
            Up(kUp2 * calibration);
            break;
    }
}
