#include "AD9959.h"
#include "Drive_Communication.h"

static u32 s_frequency_hz = 1000U;
static u16 s_amplitude_mv = 0U;
static u16 s_phase_deg = 0U;

void Init_AD9959(void)
{
    DDSDataInit();
}

void Write_frequence(u8 Channel, u32 Freq)
{
    (void)Channel;
    s_frequency_hz = Freq;
    (void)AD9959_SetOutput(s_frequency_hz, (float)s_amplitude_mv,
                           s_phase_deg, 1U);
}

void Write_frequence_no_update(u8 Channel, u32 Freq)
{
    Write_frequence(Channel, Freq);
}

void Write_Amplitude(u8 Channel, u16 Ampli)
{
    (void)Channel;
    s_amplitude_mv = Ampli;
    (void)AD9959_SetOutput(s_frequency_hz, (float)s_amplitude_mv,
                           s_phase_deg, 1U);
}

void Write_Amplitude_no_update(u8 Channel, u16 Ampli)
{
    Write_Amplitude(Channel, Ampli);
}

void Write_Phase(u8 Channel, u16 Phase)
{
    (void)Channel;
    s_phase_deg = Phase;
    (void)AD9959_SetOutput(s_frequency_hz, (float)s_amplitude_mv,
                           s_phase_deg, 1U);
}

void Write_Phase_no_update(u8 Channel, u16 Phase)
{
    Write_Phase(Channel, Phase);
}

void IntReset(void)
{
}

void IO_Update(void)
{
}

void Intserve(void)
{
}

void AD9959_SyncPhase_All(void)
{
}
