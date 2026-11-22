#ifndef _DRIVE_AD9959_H_
#define _DRIVE_AD9959_H_

#include "sys.h"
#include "stdint.h"

void Init_AD9959(void);
void Write_frequence(u8 Channel, u32 Freq);
void Write_frequence_no_update(u8 Channel, u32 Freq);
void Write_Amplitude(u8 Channel, u16 Ampli);
void Write_Amplitude_no_update(u8 Channel, u16 Ampli);
void Write_Phase(u8 Channel, u16 Phase);
void Write_Phase_no_update(u8 Channel, u16 Phase);
void IntReset(void);
void IO_Update(void);
void Intserve(void);
void AD9959_SyncPhase_All(void);

#endif
