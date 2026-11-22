#include "Drive_PGA4311.h"
#include "User_SPI.h"

static uint8_t PGA4311_GainToCode(float gain)
{
	if( gain <= -95.5f )
		return 1U;
	if( gain >= 31.5f )
		return 255U;

	return (uint8_t)(192.0f + 2.0f * gain);
}

void PGA4311_Init(void)
{
	SPI_GPIO_Init(1);
	PGA4311_SetAV(0.0f, 0.0f, 0.0f, 0.0f);
}

void PGA4311_SetAV(float A1, float A2, float A3, float A4)
{
	uint8_t D1, D2, D3, D4;
	uint32_t data;

	D1 = PGA4311_GainToCode(A1);
	D2 = PGA4311_GainToCode(A2);
	D3 = PGA4311_GainToCode(A3);
	D4 = PGA4311_GainToCode(A4);

	/* The first byte shifted in is channel 4; channel 1 is shifted in last. */
	data = ((uint32_t)D4 << 24)
	     | ((uint32_t)D3 << 16)
	     | ((uint32_t)D2 << 8)
	     | (uint32_t)D1;
	User_SPI_SendData(data, 32U, 1U);
}
