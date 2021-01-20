#ifndef PLUNGERCONTOL_H_
#define PLUNGERCONTROL_H_
#include "F28x_Project.h"

#define EPWM1_TIMER_TBPRD  8896  // Period register
#define EPWM1_MAX_CMPA     1950
#define EPWM1_MIN_CMPA       50
#define EPWM1_MAX_CMPB     1950
#define EPWM1_MIN_CMPB       50

//defines used to pick up board pieces
#define rDown1              275
#define rUp1                324
#define rDown2              271
#define rUp2                324

#define hDown1              259
#define hUp1                308
#define hDown2              257
#define hUp2                307

#define bDown1              243
#define bUp1                295
#define bDown2              241
#define bUp2                293

#define qDown1              223
#define qUp1                269
#define qDown2              220
#define qUp2                267

#define kDown1              194
#define kUp1                235
#define kDown2              197
#define kUp2                232

#define EPWM_CMP_UP           1
#define EPWM_CMP_DOWN         0

#define Pawn                  1
#define Rook                  2
#define Knight                3
#define Bishop                4
#define Queen                 5
#define King                  6


void InitEPwm1Example(void);
void InitEPwm1Gpio(void);
void InitMagnetGpio(void);
void Magnet_On(void);
void Magnet_Off(void);
void Start_PWM(void);
void Stop_PWM(void);
void Up(uint16_t length);
void Down(uint16_t length);
void PickUp(uint16_t piece);
void Place(uint16_t piece);



#endif
