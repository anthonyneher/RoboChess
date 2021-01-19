/* Authors: David Rivera & Anthony Neher
 * Set arm to default position at (1, 9), in between moves
 * Uses <setOrigin()> and <movePieceSimult(uint16_t nextX, uint16_t nextY)> to control arm
 * All other movement functions include more safety contingencies, but are unneeded
 * Flesh out draw clock and touch screen interrupt for menu control
*/

#include <F28x_Project.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "gpio.h"
#include "interrupt.h"
#include "cputimer.h"
#include "LCDLib.h"
#include "STEPPERm_LIB.h"
#include "chess.h"
#include "PlungerControl.h"

#define DEVICE_SYSCLK_FREQ  200000000

__interrupt void RX_MCBSPB_isr();
__interrupt void TIMER0_isr(void);
__interrupt void TIMER1_isr(void);
__interrupt void TIMER2_isr(void);
__interrupt void XINT1_isr();
__interrupt void XINT2_isr();
__interrupt void XINT3_isr();
__interrupt void XINT4_isr();
__interrupt void XINT5_isr();

volatile char received_board[8][8];
volatile uint16_t  move_check = 3;
__interrupt void SPIB_RX_isr(void);
void init_spiB(void);
volatile uint16_t from = 0;
volatile uint16_t to = 0;
volatile bool capturing;
volatile uint16_t pickup;
volatile uint16_t place;
uint16_t dump_position;

volatile uint16_t index = 0;
volatile uint16_t start = 0;
volatile uint16_t move = 0;
volatile uint16_t done_yet = 0;
volatile uint16_t any_new = 0;
volatile uint16_t update_board_display = 0;
volatile uint16_t received = 0;
volatile uint16_t current = 0;
volatile uint16_t displayed = 0;
volatile uint16_t reset_game = 0;
volatile uint16_t difficulty = 1;
volatile bool updatedif = false;
volatile bool receiving = 0;
volatile bool castling = false;
volatile bool castle = false;
volatile bool kingside = false;
volatile int xx = 0;
volatile int yy = 0;


void setOrigin();   //return plunger to origin

uint16_t movePiece(uint16_t currX, uint16_t currY, uint16_t nextX, uint16_t nextY);
uint16_t movePieceSimult(uint16_t nextX, uint16_t nextY);

void movePlungerFromTo(uint16_t *currPosPlunger, uint16_t newPosPlunger, volatile bool *mutex);
uint8_t movePlungerTurns(uint16_t turns, dir direction, volatile bool *mutex);
void moveArmFromTo(uint16_t *currPosArm, uint16_t newPosArm, volatile bool *mutex);
uint8_t moveArmTurns(uint16_t turns, dir direction, volatile bool *mutex);

void goToOriginPlunger();           //sends plunger to its origin (left limit switch)
void OriginPlunger_RESET();         //gets plunger off the left limit switch
void goToOriginPlunger_RESET();     //sends plunger to its origin AND gets arm off the left limit switch
void goToLimitPlunger_RESET();      //sends plunger to right limit switch AND gets plunger off the front limit switch

void goToOriginArm();               //sends arm to its origin (back limit switch)
void OriginArm_RESET();             //gets arm off the back limit switch
void goToOriginArm_RESET();         //sends arm to its origin AND gets arm off the back limit switch
void goToLimitArm_RESET();          //sends arm to front limit switch AND gets arm off the front limit switch

void drawClock(uint16_t Color);                     //Draw the clock on the LCD
void pause_start_game(bool *pause_start_toggle);    //Pause and start the game

/* STEPPER GLOBAL VARIABLES */
volatile bool initializing = false;
volatile bool mutex_M0;
volatile bool mutex_M1M2;
bool back_sw_pressed;
bool forward_sw_pressed;
bool left_sw_pressed;
bool right_sw_pressed;
uint32_t numOfSteps             = 0;
uint32_t numOfStepsA4988        = 0;
uint16_t currPosArm             = 0;
uint16_t currPosPlunger         = 0;
uint16_t readReg                = 0;
uint16_t xint1CNT               = 0;
uint16_t xint2CNT               = 0;
uint16_t xint3CNT               = 0;
uint16_t xint4CNT               = 0;
//taking pieces
bool taking = false;
bool posttake = false;
uint16_t dropspot = 0;

/* LCD & TOUCHSCREEN GLOBAL VARIABLES*/
char readByte;
char chipId[2]      = {0x00, 0x00};
char dataXY[4]      = {0x00, 0x00, 0x00, 0x00};

uint32_t tchScrnCnt = 0;
uint16_t posXY      = 0;
uint16_t X          = 0;
uint16_t Y          = 0;
uint16_t prevX      = 0;
uint16_t prevY      = 0;
uint16_t rawX       = 0;
uint16_t rawY       = 0;

uint16_t prevMinutes    = 0;
uint16_t prevSeconds    = 0;
uint16_t minutes    = 0;
uint16_t seconds    = 0;


char chessPieces[8][8];

chessPiece chessStates;
bool chessStateTog;

bool debug = true;

int main(void)
{

    EALLOW;
    InitSysCtrl();
    EALLOW;
    DELAY_US(100);
    init_spiB();
//    InitSPIA();                                         //Initialize SPIA to send data to Codec
//    InitMcBSPb();                                       //Initialize McBSP system
//    InitAIC23(false);                                   //Configure Codec



/*
     DINT; //disable CPU interrupts

     InitPieCtrl();//initialize PIE regs

     //clear all interrupt flags
     IER = 0x0000;
     IFR = 0x0000;
     */

 //    //init pie vector table with pointers to shell interrupts
 //    InitPieVectTable();

 //    EALLOW; // This is needed to write to EALLOW protected registers
 //    PieVectTable.EPWM1_INT = &epwm1_isr;
 //
 //    EDIS;   // This is needed to disable write to EALLOW protected registers

    InitEPwm1Gpio();
    InitMagnetGpio();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;
    InitEPwm1Example();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
    IER=0x20;                              // Enable CPU INT6
    EINT;                                  // Enable Global Interrupts

    ERTM;

    /* INITIALIZE STEPPER MOTOR*/
    //stepper_init_spib();
    stepper_init_settings();

    /* Initialize Touchscreen */
    LCD_initGPIO();
    LCD_Init();
    //drawClock(LCD_WHITE);
    initChessPieces(&chessPieces);
    drawPieces(&chessPieces);
    //drawClock(LCD_BLACK);
    TCHSCRN_I2C_LCD_init();

    Interrupt_initModule();                             //Enable interrupt module
    Interrupt_initVectorTable();                        //Enable interrupt vector table
/*
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;
    InitEPwm1Example();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
    InitEPwm1Gpio();
    InitMagnetGpio();
*/

    //Initialize MCBSPB Interrupt
//    Interrupt_register(INT_MCBSPB_RX, &RX_MCBSPB_isr);  //Location of INT_MCBSPB_RX isr loaded

    //Initialize TIMER0 Interrupt for DRV8771
    Interrupt_register(INT_TIMER0, &TIMER0_isr);
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_setPeriod(CPUTIMER0_BASE, (uint32_t)(DEVICE_SYSCLK_FREQ * (1/14000.0)));
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);   //no prescaler
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);

    /* Initialize TIMER1 Interrupt for A4988 */
    Interrupt_register(INT_TIMER1, &TIMER1_isr);
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_setPeriod(CPUTIMER1_BASE, (uint32_t)(DEVICE_SYSCLK_FREQ * STEPPER_LO_PERIOD));
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);   //no prescaler
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_setEmulationMode(CPUTIMER1_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);

    /* Initialize TIMER2 Interrupt for clock */
    Interrupt_register(INT_TIMER2, &TIMER2_isr);
    CPUTimer_stopTimer(CPUTIMER2_BASE);
    CPUTimer_setPeriod(CPUTIMER2_BASE, (uint32_t)(DEVICE_SYSCLK_FREQ * 0.5));
    CPUTimer_setPreScaler(CPUTIMER2_BASE, 0);                           //no prescaler
    CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);
    CPUTimer_setEmulationMode(CPUTIMER2_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(CPUTIMER2_BASE);

    /* Initialize XINT1 for back switch */
    Interrupt_register(INT_XINT1, &XINT1_isr);
    GPIO_setDirectionMode(LIMSW_XINT1_PIN, GPIO_DIR_MODE_IN);            // input
    GPIO_setQualificationMode(LIMSW_XINT1_PIN, GPIO_QUAL_SYNC);
    GPIO_setQualificationPeriod(3,510);
    GPIO_setInterruptPin(LIMSW_XINT1_PIN, GPIO_INT_XINT1);
    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_enableInterrupt(GPIO_INT_XINT1);         // Enable XINT1

    /* Initialize XINT2 for front switch */
    Interrupt_register(INT_XINT2, &XINT2_isr);
    GPIO_setDirectionMode(LIMSW_XINT2_PIN, GPIO_DIR_MODE_IN);            // input
    GPIO_setQualificationMode(LIMSW_XINT2_PIN, GPIO_QUAL_SYNC);
    GPIO_setQualificationPeriod(3,510);
    GPIO_setInterruptPin(LIMSW_XINT2_PIN, GPIO_INT_XINT2);
    GPIO_setInterruptType(GPIO_INT_XINT2, GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_enableInterrupt(GPIO_INT_XINT2);         // Enable XINT1

    /* Initialize XINT3 for left switch */
    Interrupt_register(INT_XINT3, &XINT3_isr);
    GPIO_setDirectionMode(LIMSW_XINT3_PIN, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(LIMSW_XINT3_PIN, GPIO_QUAL_SYNC);
    GPIO_setQualificationPeriod(3,510);
    GPIO_setInterruptPin(LIMSW_XINT3_PIN, GPIO_INT_XINT3);
    GPIO_setInterruptType(GPIO_INT_XINT3, GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_enableInterrupt(GPIO_INT_XINT3);

    /* Initialize XINT4 for right switch */
    Interrupt_register(INT_XINT4, &XINT4_isr);
    GPIO_setDirectionMode(LIMSW_XINT4_PIN, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(LIMSW_XINT4_PIN, GPIO_QUAL_SYNC);
    GPIO_setQualificationPeriod(3,510);
    GPIO_setInterruptPin(LIMSW_XINT4_PIN, GPIO_INT_XINT4);
    GPIO_setInterruptType(GPIO_INT_XINT4, GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_enableInterrupt(GPIO_INT_XINT4);

    /* Initialize XINT5 */
    GPIO_setDirectionMode(59, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(59, GPIO_QUAL_SYNC);
    GPIO_setQualificationPeriod(59,510);
    GPIO_setInterruptPin(59, GPIO_INT_XINT5);
    GPIO_setInterruptType(GPIO_INT_XINT5, GPIO_INT_TYPE_RISING_EDGE);
    GPIO_enableInterrupt(GPIO_INT_XINT5);// Enable XINT5
    Interrupt_register(INT_XINT5, &XINT5_isr);
    Interrupt_register(INT_SPIB_RX, &SPIB_RX_isr);

    /* Enable Interrupts */
    Interrupt_enable(INT_TIMER0);
    Interrupt_enable(INT_TIMER1);
    Interrupt_enable(INT_TIMER2);
    Interrupt_enable(INT_XINT1);
    Interrupt_enable(INT_XINT2);
    Interrupt_enable(INT_XINT3);
    Interrupt_enable(INT_XINT4);
    Interrupt_enable(INT_XINT5);
    Interrupt_enable(INT_SPIB_RX);
    Interrupt_enableMaster();
//    Interrupt_enable(INT_MCBSPB_RX);

    setOrigin();
    current = 0;
    dump_position = 1;


    //CPUTimer_startTimer(CPUTIMER2_BASE);
    //movePieceSimult(1,9);
    /*
    Down(rDown2*calibration);
    Magnet_Off();
    DELAY_US(50000);
    Up(rUp2*calibration);
    movePieceSimult(1,8);*/
    while(start == 0);//wait until raspberry pi boots and runs game program
    int x;
    int y;
//    while(1){
//        movePieceSimult(1,8);
//        PickUp(Pawn);
//        Place(Pawn);
//
//        movePieceSimult(2,8);
//        PickUp(Knight);
//        Place(Knight);
//
//        movePieceSimult(3,8);
//        PickUp(Bishop);
//        Place(Bishop);
//
//        movePieceSimult(4,8);
//        PickUp(Queen);
//        Place(Queen);
//
//        movePieceSimult(5,8);
//        PickUp(King);
//        Place(King);
//    }
    while(1){
        if(updatedif){
            if(difficulty == 1){
                LCD_DrawRectangleBrdr(BUTTON_OFFSET_X, BUTTON_OFFSET_X + BUTTON_LENGTH + 3,
                                      Y8, Y8+BUTTON_WIDTH, 3, LCD_GREEN, LCD_BLACK);
                LCD_Text(BUTTON_OFFSET_X+15, Y8, "EASY", 2, LCD_BLACK);
            }else if(difficulty == 2){
                LCD_DrawRectangleBrdr(BUTTON_OFFSET_X, BUTTON_OFFSET_X + BUTTON_LENGTH + 3,
                                      Y8, Y8+BUTTON_WIDTH, 3, LCD_YELLOW, LCD_BLACK);
                LCD_Text(BUTTON_OFFSET_X+15, Y8, "FAIR", 2, LCD_BLACK);
            }else{
                LCD_DrawRectangleBrdr(BUTTON_OFFSET_X, BUTTON_OFFSET_X + BUTTON_LENGTH + 3,
                                      Y8, Y8+BUTTON_WIDTH, 3, LCD_RED, LCD_BLACK);
                LCD_Text(BUTTON_OFFSET_X+15, Y8, "HARD", 2, LCD_BLACK);
            }
            updatedif = false;
        }
        if(update_board_display){
            //new board display has been received from PI
            updatePieces(chessPieces, received_board);
            for(int y = 0; y<8; y++){
                for(int x = 0; x < 8; x++){
                    chessPieces[y][x] = received_board[y][x];
                }
            }
            update_board_display = 0;
        }
        if(capturing){
            //move has been received from Pi
            if(current == displayed){
                displayed += 2;
                current += 2;
            }else current += 2;
            done_yet = 0;

            if(castle){
                if(kingside){
                    movePieceSimult(5,8);
                    PickUp(King);
                    movePieceSimult(7,8);
                    Place(King);
                    movePieceSimult(8,8);
                    PickUp(Rook);
                    movePieceSimult(6,8);
                    Place(Rook);
                }else{
                    movePieceSimult(5,8);
                    PickUp(King);
                    movePieceSimult(3,8);
                    Place(King);
                    movePieceSimult(1,8);
                    PickUp(Rook);
                    movePieceSimult(4,8);
                    Place(Rook);
                }
                movePieceSimult(1,9);
                castle = false;
            }else{
                if(!move){
                    capturing = 0;
                    x = (to%8) + 1;
                    y = (to/8) + 1;
                    movePieceSimult(x,y);
                    PickUp(pickup);
                    taking = true;
                    movePieceSimult(0, 0);
                    Place(pickup);
                }
                x = (from%8)+1;
                y = (from/8)+1;
                movePieceSimult(x,y);
                PickUp(place);
                x = (to%8)+1;
                y = (to/8)+1;
                movePieceSimult(x,y);
                Place(place);
                movePieceSimult(1,9);
            }


            done_yet = 1;
            capturing = false;
        }
    }
}

void init_spiB(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR8.bit.SPI_B = 1;//enable spi B
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0; // sets the clock to 200MHz

    EALLOW;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 3;//SPIBMOSI
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 3;//SPIBCLK
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 3;//SPIBMISO

    GpioCtrlRegs.GPAGMUX2.bit.GPIO23 = 1;//SPISTEB

    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3;//SPIBMOSI
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;//SPIBCLK
    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;//SPIBMISO

    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 2;//SPISTEB


    EALLOW;
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;     // Enable PIE Group 6 Channel 1
    IER=0x20;                              // Enable CPU INT6
    EDIS;

    SpibRegs.SPICCR.bit.SPISWRESET = 0;//software reset false
    SpibRegs.SPISTS.bit.OVERRUN_FLAG = 1;
    SpibRegs.SPICCR.bit.HS_MODE = 0;
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpibRegs.SPICCR.bit.SPICHAR = 0x7;
    SpibRegs.SPICTL.bit.CLK_PHASE = 0;
    SpibRegs.SPICTL.bit.TALK = 1;
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 0;
    SpibRegs.SPICTL.bit.SPIINTENA = 1;//enable SPI interrupt
    SpibRegs.SPIBRR.all = 0x04;
    SpibRegs.SPICTL.bit.OVERRUNINTENA = 1;//enable overrun interrupt
    SpibRegs.SPICCR.bit.SPISWRESET = 1;//reset off
    SpibRegs.SPIPRI.bit.FREE = 1;

    Uint16 boiii = SpibRegs.SPIRXBUF;
}

//SPIA RX Interrrupt
__interrupt void SPIB_RX_isr(void)
{
    uint16_t garbage;
    received = (SpibRegs.SPIRXBUF & 0xFF);
    if(capturing){
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
        return;
    }
    uint16_t view = received;
    if(received == 0x0000){
        start = 1;
    }
    else if(received == 0x0001){//move
        while(!SpibRegs.SPISTS.bit.INT_FLAG);
        from = (SpibRegs.SPIRXBUF & 0xFF);
        while(!SpibRegs.SPISTS.bit.INT_FLAG);
        to = (SpibRegs.SPIRXBUF & 0xFF);
        while(!SpibRegs.SPISTS.bit.INT_FLAG);
        place = SpibRegs.SPIRXBUF & 0xFF;

        capturing = true;
        move = 1;
        DELAY_US(1000);
    }
    else if(received == 0x0005){
        while(!SpibRegs.SPISTS.bit.INT_FLAG);
        from = (SpibRegs.SPIRXBUF & 0xFF);
        while(!SpibRegs.SPISTS.bit.INT_FLAG);
        to = (SpibRegs.SPIRXBUF & 0xFF);
        while(!SpibRegs.SPISTS.bit.INT_FLAG);
        place = SpibRegs.SPIRXBUF & 0xFF;
        while(!SpibRegs.SPISTS.bit.INT_FLAG);
        pickup = SpibRegs.SPIRXBUF & 0xFF;


        capturing = true;
        move = 0;
        DELAY_US(1000);
    }
    else if(received == 0x0002){//done?
        SpibRegs.SPITXBUF = (done_yet<<8);
        while(!SpibRegs.SPISTS.bit.INT_FLAG);
        garbage = SpibRegs.SPIRXBUF;
    }
    else if(received == 0x0003){//any new?
        SpibRegs.SPITXBUF = (displayed<<8);
        while(!SpibRegs.SPISTS.bit.INT_FLAG);
        garbage = SpibRegs.SPIRXBUF;

//        SpibRegs.SPITXBUF = (reset_game<<8);
//        while(!SpibRegs.SPISTS.bit.INT_FLAG);
//        garbage = SpibRegs.SPIRXBUF;
//
//        SpibRegs.SPITXBUF = (difficulty<<8);
//        while(!SpibRegs.SPISTS.bit.INT_FLAG);
//        garbage = SpibRegs.SPIRXBUF;

        any_new = 1;
    }
    else if(received == 0x0004){//board display
        for(int x = 0;x<64;x++){
            while(!SpibRegs.SPISTS.bit.INT_FLAG);
            received_board[(x/8)][x%8] = (char)(SpibRegs.SPIRXBUF & 0xFF);
        }
        update_board_display = 1;
    }
    else if(received == 0x0006){
        SpibRegs.SPITXBUF = (reset_game<<8);
        while(!SpibRegs.SPISTS.bit.INT_FLAG);
        garbage = SpibRegs.SPIRXBUF;
        reset_game = 0;
    }
    else if(received == 0x0008){
        SpibRegs.SPITXBUF = (difficulty<<8);
        while(!SpibRegs.SPISTS.bit.INT_FLAG);
        garbage = SpibRegs.SPIRXBUF;
    }else if(received == 0x0009){
        while(!SpibRegs.SPISTS.bit.INT_FLAG);
        kingside = (SpibRegs.SPIRXBUF & 0xFF);
        castle = true;
        capturing = true;
    }
    //SpibRegs.SPITXBUF = 0x7400;
    // Acknowledge this interrupt to receive more interrupts from group 6
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

__interrupt void RX_MCBSPB_isr(){
    McbspbRegs.DXR2.all = McbspbRegs.DRR2.all;
    McbspbRegs.DXR1.all = McbspbRegs.DRR1.all;
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);
}

__interrupt void XINT1_isr(){
    GPIO_disableInterrupt(GPIO_INT_XINT1);
    //DISABLE MOTOR HERE
    xint1CNT++;
    DELAY_US(80000);            //80ms debounce wait

    if(!GpioDataRegs.GPADAT.bit.GPIO10){
        CPUTimer_stopTimer(CPUTIMER1_BASE);
        numOfStepsA4988     = 0;
        mutex_M1M2          = false;

        back_sw_pressed     = true;
    }

    DELAY_US(80000);            //80ms debounce wait
    GPIO_enableInterrupt(GPIO_INT_XINT1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void XINT2_isr(){
    GPIO_disableInterrupt(GPIO_INT_XINT2);
    //DISABLE MOTOR HERE
    xint2CNT++;
    DELAY_US(80000);                    //80ms debounce wait

    if(!GpioDataRegs.GPADAT.bit.GPIO11){
        CPUTimer_stopTimer(CPUTIMER1_BASE);
        numOfStepsA4988     = 0;
        mutex_M1M2          = false;

        forward_sw_pressed  = true;
    }

    DELAY_US(80000);            //80ms debounce wait
    GPIO_enableInterrupt(GPIO_INT_XINT2);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void XINT3_isr(){
    GPIO_disableInterrupt(GPIO_INT_XINT3);
    //DISABLE MOTOR HERE
    DELAY_US(80000);            //80ms debounce wait

    if(!GpioDataRegs.GPCDAT.bit.GPIO75){
        xint3CNT++;
        CPUTimer_stopTimer(CPUTIMER0_BASE);
        numOfSteps          = 0;
        mutex_M0            = false;
        left_sw_pressed     = true;
    }

    DELAY_US(80000);            //80ms debounce wait
    GPIO_enableInterrupt(GPIO_INT_XINT3);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP12);
}

__interrupt void XINT4_isr(){
    GPIO_disableInterrupt(GPIO_INT_XINT4);
    //DISABLE MOTOR HERE
    DELAY_US(80000);                    //80ms debounce wait

    if(!GpioDataRegs.GPCDAT.bit.GPIO76){
        xint4CNT++;
        CPUTimer_stopTimer(CPUTIMER0_BASE);
        numOfSteps          = 0;
        mutex_M0            = false;
        right_sw_pressed    = true;
    }

    DELAY_US(80000);            //80ms debounce wait
    GPIO_enableInterrupt(GPIO_INT_XINT4);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP12);
}

__interrupt void XINT5_isr(){
    Interrupt_disable(INT_XINT5);
    readByte        = TCHSCRN_I2C_ReadByte(STMPE_INT_STA);
    if(readByte & 0x01){
        tchScrnCnt++;
        readByte        = TCHSCRN_I2C_ReadByte(STMPE_FIFO_STA);
        while(!((readByte>>5) & 0x01)){             //read data until FIFO empty
            posXY       = TCHSCRN_I2C_ReadBytes(STMPE_TSC_DATA, dataXY, sizeof(dataXY));
            readByte    = TCHSCRN_I2C_ReadByte(STMPE_FIFO_STA);
        }
        X = (uint16_t)((((dataXY[1] & 0x0F)<<8) | dataXY[2]) * 0.09);
        Y = (uint16_t)(((dataXY[0] << 4) | (dataXY[1]  >> 4)) * 0.09);

        if(((prevX != X) || (prevY != Y))){
            if((X >= 40 && X <= 120) && (Y >= 90 && Y <= 145)){//next button pressed
                if (displayed != current){
                    displayed++;
                }
            }else if((X >= 40 && X <= 120) && (Y >= 175 && Y <= 230)){//previous button pressed
                if (displayed != 0){
                    displayed--;
                }
            }else if((X >= 40 && X <= 120) && (Y >= 275 && Y <= 320)){//reset game
                reset_game = 1;
                dump_position = 0;
                displayed = 0;
            }else if((X >= 40 && X <= 120) && (Y >= 0 && Y <= 75)){//change difficulty
                if(difficulty == 3) difficulty = 1;
                else difficulty++;
                updatedif = true;
            }
        }
        prevX = X;
        prevY = Y;
        TCHSCRN_I2C_SendByte(STMPE_INT_STA, 0xFF);        //Clear status flags

        Interrupt_enable(INT_XINT5);
    }else{
        Interrupt_enable(INT_XINT5);
    }
//    DELAY_US(100000);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP12);
}

__interrupt void TIMER0_isr(void){
    if (numOfSteps > 0){
        M0_STEP;
        numOfSteps--;
    }else{
        CPUTimer_stopTimer(CPUTIMER0_BASE);
        mutex_M0 = false;
    }
//    M0_STEP;
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void TIMER1_isr(void){
    if (numOfStepsA4988 > 0){
        M1M2_STEP;
        numOfStepsA4988--;
    }else{
        CPUTimer_stopTimer(CPUTIMER1_BASE);
        mutex_M1M2 = false;
    }
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void TIMER2_isr(void){
    waitflag = 0;
}


void goToOriginArm_RESET(){
    while(back_sw_pressed == false){
       while(!stepper_moveA4988(&numOfStepsA4988, SIXTEENTH_TURN, BWD, &mutex_M1M2));
    }


    /* get off switch */

    while(!GpioDataRegs.GPADAT.bit.GPIO10){
       DELAY_US(80000);
       while(!stepper_moveA4988(&numOfStepsA4988, EIGTH_TURN, FWD, &mutex_M1M2));
    }
    //DELAY_US(800000);

    forward_sw_pressed = false;
    back_sw_pressed = false;
}

void goToLimitArm_RESET(){
    while(forward_sw_pressed == false){
       stepper_moveA4988(&numOfStepsA4988, SIXTEENTH_TURN, FWD, &mutex_M1M2);
    }
    DELAY_US(80000);
    /* get off switch */
    while(!GpioDataRegs.GPADAT.bit.GPIO11){
       DELAY_US(80000);
       stepper_moveA4988(&numOfStepsA4988, EIGTH_TURN, BWD, &mutex_M1M2);
    }
    DELAY_US(800000);
    forward_sw_pressed = false;
    back_sw_pressed = false;
}


void goToOriginArm(){
    while(back_sw_pressed == false){
       while(!stepper_moveA4988(&numOfStepsA4988, SIXTEENTH_TURN, BWD, &mutex_M1M2));
    }
    DELAY_US(80000);
    forward_sw_pressed = false;
}

void OriginArm_RESET(){
    /* get off switch */
    while(!GpioDataRegs.GPADAT.bit.GPIO10){
       DELAY_US(80000);
       while(!stepper_moveA4988(&numOfStepsA4988, EIGTH_TURN, FWD, &mutex_M1M2));
    }
    DELAY_US(800000);
    back_sw_pressed = false;
}

void moveArmFromTo(uint16_t *currPosArm, uint16_t newPosArm, volatile bool *mutex){
    int16_t posDiff = 0;                //position difference
    if(newPosArm > 10 || newPosArm < 1){
        return;
    }else{
        posDiff = newPosArm - *currPosArm;
        if(posDiff > 0){
            if(!moveArmTurns(posDiff*SQUARE_LENGTH, BWD, mutex)){
                return;
            }
        }else if(posDiff < 0){
            posDiff = -1*posDiff;
           if(!moveArmTurns(posDiff*SQUARE_LENGTH, FWD, mutex)){
               return;
           }
        }
        *currPosArm = newPosArm;
    }
}

uint8_t moveArmTurns(uint16_t turns, dir direction, volatile bool *mutex){
    uint16_t numberOfHalfTurns = turns;

    stepper_mutex_delay(mutex);
    while(numberOfHalfTurns > 0){
        if(forward_sw_pressed){
            stepper_mutex_delay(mutex);
            goToOriginArm();
        }else if(back_sw_pressed){
            stepper_mutex_wait(mutex);
            OriginArm_RESET();
            numberOfHalfTurns = 0;
            return 0;
        }else{
            stepper_mutex_wait(mutex);
            stepper_moveA4988(&numOfStepsA4988, HALF_TURN, direction, mutex);
            numberOfHalfTurns--;
        }
    }
    return 1;
}

void movePlungerFromTo(uint16_t *currPosPlunger, uint16_t newPosPlunger, volatile bool *mutex){
    int16_t posDiff = 0;                //position difference
    if(newPosPlunger > 8 || newPosPlunger < 1){
        return;
    }else{
        posDiff = newPosPlunger - *currPosPlunger;
        if(posDiff > 0){
            if(!movePlungerTurns(posDiff*SQUARE_LENGTH, FWD, mutex)){
                return;
            }
        }else if(posDiff < 0){
            posDiff = -1*posDiff;
            if(!movePlungerTurns(posDiff*SQUARE_LENGTH, BWD, mutex)){   //quit if reached end ERROR!
                return;
            }
        }
        *currPosPlunger = newPosPlunger;    //reassign new position if successful
    }
}

uint8_t movePlungerTurns(uint16_t turns, dir direction, volatile bool *mutex){
    uint16_t numberOfHalfTurns = turns;
    stepper_mutex_delay(mutex);
    while(numberOfHalfTurns > 0){
        if(right_sw_pressed && !initializing){
            stepper_mutex_delay(mutex);
            goToOriginPlunger();
        }else if(left_sw_pressed && !initializing){
            stepper_mutex_wait(mutex);
            OriginPlunger_RESET();
            numberOfHalfTurns = 0;
            return 0;
        }else{
            stepper_mutex_wait(mutex);
            stepper_move(&numOfSteps, HALF_TURN, direction, mutex);
            numberOfHalfTurns--;
        }
    }
    return 1;
}

void goToOriginPlunger(){
    while(left_sw_pressed == false){
       while(!stepper_move(&numOfSteps, SIXTEENTH_TURN, BWD, &mutex_M0));
    }
    DELAY_US(80000);
    right_sw_pressed = false;
}

void OriginPlunger_RESET(){
    /* get off switch */
    while(!GpioDataRegs.GPCDAT.bit.GPIO75){
       DELAY_US(80000);
       while(!stepper_move(&numOfSteps, EIGTH_TURN, FWD, &mutex_M0));
    }
    DELAY_US(800000);
    left_sw_pressed = false;
}

void goToOriginPlunger_RESET(){
    while(left_sw_pressed == false){
       while(!stepper_move(&numOfSteps, SIXTEENTH_TURN, BWD, &mutex_M0));
    }
    //DELAY_US(80000);
    /* get off switch */
    while(!GpioDataRegs.GPCDAT.bit.GPIO75){
       DELAY_US(80000);
       while(!stepper_move(&numOfSteps, EIGTH_TURN, FWD, &mutex_M0));
    }
    //DELAY_US(800000);
    right_sw_pressed = false;
    left_sw_pressed = false;
}

void goToLimitPlunger_RESET(){
    while(right_sw_pressed == false){
       stepper_move(&numOfSteps, SIXTEENTH_TURN, FWD, &mutex_M0);
    }
    DELAY_US(80000);
    /* get off switch */
    while(!GpioDataRegs.GPCDAT.bit.GPIO76){
       DELAY_US(80000);
       stepper_move(&numOfSteps, EIGTH_TURN, BWD, &mutex_M0);
    }
    DELAY_US(800000);
    right_sw_pressed = false;
    left_sw_pressed = false;
}

void setOrigin(){
    goToOriginArm_RESET();

    moveArmTurns(BOARD_ORIGIN_ARM, FWD, &mutex_M1M2);
    goToOriginPlunger_RESET();

    movePlungerTurns(BOARD_ORIGIN_PLUNGER, FWD, &mutex_M0);
    currPosArm      = 9;    //arm is in first square
    currPosPlunger  = 1;    //plunger to first square
    ENABLE_OFF;
}

/*
 * NICE AND SLOW
 * X relates to the plunger
 * Y relates to the arm
*/
uint16_t movePiece(uint16_t currX, uint16_t currY, uint16_t nextX, uint16_t nextY){
    moveArmFromTo(&currPosArm, currY, &mutex_M1M2);
    movePlungerFromTo(&currPosPlunger, currX, &mutex_M0);

    /* pick up piece here */
    DELAY_US(3000000);
    moveArmFromTo(&currPosArm, nextY, &mutex_M1M2);
    movePlungerFromTo(&currPosPlunger, nextX, &mutex_M0);

    return 1;
}



uint16_t movePieceSimult(uint16_t nextX, uint16_t nextY){
    ENABLE_ON;
    stepper_mutex_delay(&mutex_M0);
    stepper_mutex_delay(&mutex_M1M2);
    int32_t armPositionDiff = 0;                //position difference
    int32_t plungerPositionDiff = 0;            //position difference


    if(taking){//unplanned workaround for more efficient dropping system during piece take
        //X
        plungerPositionDiff = (int32_t)(HALF_TURN*SQUARE_LENGTH*(currPosPlunger-1))-((int32_t)dropspot)*DROP_X*HALF_TURN;
        if(plungerPositionDiff<0){
            plungerPositionDiff*=-1;
            stepper_move(&numOfSteps, plungerPositionDiff, FWD, &mutex_M0);
        }else{
            stepper_move(&numOfSteps, plungerPositionDiff, BWD, &mutex_M0);
        }
        //Y
        armPositionDiff = (int32_t)(HALF_TURN*SQUARE_LENGTH*(9-currPosArm));
        if(dropspot%2 == 0){//inline with Y = 9
            stepper_moveA4988(&numOfStepsA4988, armPositionDiff, BWD, &mutex_M1M2);
        }else{//offset from Y = 9
            stepper_moveA4988(&numOfStepsA4988, (armPositionDiff+DROP_Y*HALF_TURN), BWD, &mutex_M1M2);
        }
        taking = false;
        posttake = true;

    }else if(posttake){
        //X
        plungerPositionDiff = ((int32_t)dropspot)*DROP_X*HALF_TURN - (int32_t)(HALF_TURN*SQUARE_LENGTH*(nextX-1));
        if(plungerPositionDiff<0){
            plungerPositionDiff*=-1;
            stepper_move(&numOfSteps, plungerPositionDiff, FWD, &mutex_M0);
        }else{
            stepper_move(&numOfSteps, plungerPositionDiff, BWD, &mutex_M0);
        }

        //Y
        armPositionDiff = (int32_t)(HALF_TURN*SQUARE_LENGTH*(9-nextY));
        if(dropspot%2 == 0){//inline with Y = 9
            stepper_moveA4988(&numOfStepsA4988, armPositionDiff, FWD, &mutex_M1M2);
        }else{//offset from Y = 9
            stepper_moveA4988(&numOfStepsA4988, (armPositionDiff+DROP_Y*HALF_TURN), FWD, &mutex_M1M2);
        }
        dropspot++;
        posttake = false;
        currPosPlunger  = nextX;
        currPosArm      = nextY;
    }else{
        if(nextX > 8 || nextX < 1){
            return 0;
        }
        if(nextY > 9 || nextY < 1){
            return 0;
        }

        armPositionDiff     = (int32_t)currPosArm - (int32_t)nextY;
        plungerPositionDiff = (int32_t)currPosPlunger - (int32_t)nextX;

        if(armPositionDiff >= 0 && plungerPositionDiff >= 0){
            stepper_move(&numOfSteps, HALF_TURN*SQUARE_LENGTH*plungerPositionDiff, BWD, &mutex_M0);
            stepper_moveA4988(&numOfStepsA4988, HALF_TURN*SQUARE_LENGTH*armPositionDiff, FWD, &mutex_M1M2);
        }else if(armPositionDiff >= 0 && plungerPositionDiff < 0){
            plungerPositionDiff = -1 * plungerPositionDiff;
            stepper_move(&numOfSteps, HALF_TURN*SQUARE_LENGTH*plungerPositionDiff, FWD, &mutex_M0);
            stepper_moveA4988(&numOfStepsA4988, HALF_TURN*SQUARE_LENGTH*armPositionDiff, FWD, &mutex_M1M2);
        }else if(armPositionDiff < 0 && plungerPositionDiff >= 0){
            armPositionDiff     = -1*armPositionDiff;
            stepper_move(&numOfSteps, HALF_TURN*SQUARE_LENGTH*plungerPositionDiff, BWD, &mutex_M0);
            stepper_moveA4988(&numOfStepsA4988, HALF_TURN*SQUARE_LENGTH*armPositionDiff, BWD, &mutex_M1M2);
        }else{
            armPositionDiff     = -1*armPositionDiff;
            plungerPositionDiff = -1 * plungerPositionDiff;
            stepper_move(&numOfSteps, HALF_TURN*SQUARE_LENGTH*plungerPositionDiff, FWD, &mutex_M0);
            stepper_moveA4988(&numOfStepsA4988, HALF_TURN*SQUARE_LENGTH*armPositionDiff, BWD, &mutex_M1M2);
        }
        currPosPlunger  = nextX;
        currPosArm      = nextY;
    }
    stepper_mutex_delay(&mutex_M0);
    stepper_mutex_delay(&mutex_M1M2);

    ENABLE_OFF;//disable when not in use
    return 1;
}
//uint16_t movePieceSimult(uint16_t nextX, uint16_t nextY){
//    ENABLE_ON;
//    stepper_mutex_delay(&mutex_M0);
//    stepper_mutex_delay(&mutex_M1M2);
//    int32_t armPositionDiff = 0;                //position difference
//    int32_t plungerPositionDiff = 0;            //position difference
//
//    if(nextX > 8 || nextX < 1){
//        return 0;
//    }
//    if(nextY > 9 || nextY < 1){
//        return 0;
//    }
//
//    armPositionDiff     = (int32_t)currPosArm - (int32_t)nextY;
//    plungerPositionDiff = (int32_t)currPosPlunger - (int32_t)nextX;
//
//    if(armPositionDiff >= 0 && plungerPositionDiff >= 0){
//        stepper_move(&numOfSteps, HALF_TURN*SQUARE_LENGTH*plungerPositionDiff, BWD, &mutex_M0);
//        stepper_moveA4988(&numOfStepsA4988, HALF_TURN*SQUARE_LENGTH*armPositionDiff, FWD, &mutex_M1M2);
//    }else if(armPositionDiff >= 0 && plungerPositionDiff < 0){
//        plungerPositionDiff = -1 * plungerPositionDiff;
//        stepper_move(&numOfSteps, HALF_TURN*SQUARE_LENGTH*plungerPositionDiff, FWD, &mutex_M0);
//        stepper_moveA4988(&numOfStepsA4988, HALF_TURN*SQUARE_LENGTH*armPositionDiff, FWD, &mutex_M1M2);
//    }else if(armPositionDiff < 0 && plungerPositionDiff >= 0){
//        armPositionDiff     = -1*armPositionDiff;
//        stepper_move(&numOfSteps, HALF_TURN*SQUARE_LENGTH*plungerPositionDiff, BWD, &mutex_M0);
//        stepper_moveA4988(&numOfStepsA4988, HALF_TURN*SQUARE_LENGTH*armPositionDiff, BWD, &mutex_M1M2);
//    }else{
//        armPositionDiff     = -1*armPositionDiff;
//        plungerPositionDiff = -1 * plungerPositionDiff;
//        stepper_move(&numOfSteps, HALF_TURN*SQUARE_LENGTH*plungerPositionDiff, FWD, &mutex_M0);
//        stepper_moveA4988(&numOfStepsA4988, HALF_TURN*SQUARE_LENGTH*armPositionDiff, BWD, &mutex_M1M2);
//    }
//    currPosPlunger  = nextX;
//    currPosArm      = nextY;
//    stepper_mutex_delay(&mutex_M0);
//    stepper_mutex_delay(&mutex_M1M2);
//
//    ENABLE_OFF;//disable when not in use
//    return 1;
//}
