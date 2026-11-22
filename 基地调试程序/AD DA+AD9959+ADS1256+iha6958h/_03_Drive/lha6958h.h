#ifndef __LHA6958H_H
#define __LHA6958H_H

/* STM32F407ZG parallel-interface allocation using the exposed expansion headers.
 * DB0..DB6: PG0..PG6; DB7..DB14: PG8..PG15; DB15: PC0.
 * FRSTDATA/BUSY: PC6/PC7; CS/SCLK(RD)/WR/CONVST: PC8..PC11;
 * OS0/OS1/OS2: PC12/PC13/PA8; RESET (active high): PA9.
 * PAR/SER SEL must be tied low for 16-bit parallel mode. The module's SCLK
 * silkscreen is the chip's multiplexed RD/SCLK pin; it functions as RD here. */
#include "stm32f4xx.h"
#include <stdint.h>

typedef enum { LHA6958H_OK = 0, LHA6958H_ERROR, LHA6958H_TIMEOUT,
               LHA6958H_BUSY, LHA6958H_FRSTDATA_ERROR, LHA6958H_VERIFY_ERROR } LHA6958H_Status;
typedef struct { int16_t code[8]; } LHA6958H_Frame;

LHA6958H_Status LHA6958H_Init(void);
LHA6958H_Status LHA6958H_Sample(LHA6958H_Frame *frame, uint32_t timeout_us);
LHA6958H_Status LHA6958H_StartConversion(void);
LHA6958H_Status LHA6958H_WaitReady(uint32_t timeout_us);
LHA6958H_Status LHA6958H_ReadFrame(LHA6958H_Frame *frame);
LHA6958H_Status LHA6958H_ReadRegister(uint8_t address, uint8_t *value);
LHA6958H_Status LHA6958H_WriteRegister(uint8_t address, uint8_t value);
float LHA6958H_CodeToVoltage(uint8_t channel, int16_t code);

#endif
