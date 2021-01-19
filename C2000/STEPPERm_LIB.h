/*
 * STEPPERm_LIB.h
 *
 *  Created on: Feb 24, 2020
 *      Author: DAVE
 */

#ifndef STEPPERM_LIB_H_
#define STEPPERM_LIB_H_

#include <F28x_Project.h>
#include <stdbool.h>
#include <stdint.h>
#include "cputimer.h"
#include "gpio.h"

extern volatile uint16_t waitflag;


typedef enum{
    FWD = 1,
    BWD = 0
}dir;

#define STEPPER_HI_FREQ     90000
#define STEPPER_MD_FREQ     120000
#define STEPPER_LO_FREQ     14000//16000
#define STEPPER_HI_PERIOD   1/STEPPER_HI_FREQ
#define STEPPER_MD_PERIOD   1/STEPPER_MD_FREQ
#define STEPPER_LO_PERIOD   1/STEPPER_LO_FREQ

/* M0 PINS */
#define RESET_PIN           87
#define SLEEP_PIN           88
#define STEPPER_CS_PIN      83
#define M0_DIR_PIN          89
#define M0_STEP_PIN         90

/* M1M2 PINS */
#define M1M2_nRS_PIN        85
#define M1M2_DIR_PIN        91
#define M1M2_STEP_PIN       92

/*LIMIT SWITCHES PINS*/
#define LIMSW_XINT1_PIN     10
#define LIMSW_XINT2_PIN     11
#define LIMSW_XINT3_PIN     75
#define LIMSW_XINT4_PIN     76

/* M0 MACROS */
#define RESET_OFF           GPIO_WritePin(RESET_PIN, 1);
#define RESET_ON            GPIO_WritePin(RESET_PIN, 0);

#define ENABLE_OFF           GPIO_WritePin(SLEEP_PIN, 1);
#define ENABLE_ON            GPIO_WritePin(SLEEP_PIN, 0);

#define STEPPER_CS_LOW      GPIO_WritePin(STEPPER_CS_PIN, 0);
#define STEPPER_CS_HIGH     GPIO_WritePin(STEPPER_CS_PIN, 1);

#define M0_DIR_FWD          GPIO_WritePin(M0_DIR_PIN, 0);       //set dir BWD
#define M0_DIR_BWD         GPIO_WritePin(M0_DIR_PIN, 1);      //set dir FRWD
#define M0_STEP             GPIO_togglePin(M0_STEP_PIN);
#define M0_STEP_OFF         GPIO_writePin(M0_STEP_PIN, 0);

/*  M1M2 MACROS */
#define M1M2_nRS_OFF        GPIO_writePin(M1M2_nRS_PIN, 1);
#define M1M2_nRS_ON         GPIO_writePin(M1M2_nRS_PIN, 0);
#define M1M2_DIR_FWD        GPIO_writePin(M1M2_DIR_PIN, 0);
#define M1M2_DIR_BWD       GPIO_writePin(M1M2_DIR_PIN, 1);
#define M1M2_STEP           GPIO_togglePin(M1M2_STEP_PIN);
#define M1M2_STEP_OFF       GPIO_writePin(M1M2_STEP_PIN, 0)


/*  REGISTER DEFINITIONS  */
#define CONTROL     0x00
#define TORQUE      0x01
#define OFF         0x02
#define BLANK       0x03
#define DECAY       0x04
#define STALL       0x05
#define DRIVE       0x06
#define STATUS      0x07

/*  MOVEMENT DEFINITIONS*/
#define SIXTEENTH_TURN  25
#define EIGTH_TURN      50
#define QUARTER_TURN    100
#define HALF_TURN       200
//#define HALF_TURN       200
#define FULL_TURN       400

#define BOARD_ORIGIN_ARM        11
#define BOARD_ORIGIN_PLUNGER    10
#define SQUARE_LENGTH           23
#define DROP_X                  16
#define DROP_Y                  12
#define CAPTURED_PIECE          9

/*  Functions  */
void stepper_init_spib();
void stepper_init_settings();
void stepper_spi_wr(uint16_t regAddr, uint16_t data);
uint16_t stepper_spi_rd(uint16_t regAddr);
uint16_t stepper_move(uint32_t *cntVar, uint32_t numOfSteps, dir direction, volatile bool *mutex);
uint16_t stepper_moveA4988(uint32_t *cntVar, uint32_t numOfSteps, dir direction, volatile bool *mutex);
void stepper_mutex_wait(volatile bool *mutex);
void stepper_mutex_delay(volatile bool *mutex);

#endif /* STEPPERM_LIB_H_ */
