#ifndef PID_SERIAL_H
#define PID_SERIAL_H

#include <stdint.h>

/* Initializes the line-oriented USART1 PID tuning protocol (PA9/PA10). */
void PidSerial_Init(void);

/* Polls a bounded number of RX bytes; call from the instrument task. */
void PidSerial_Process(void);

#endif /* PID_SERIAL_H */
