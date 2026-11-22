/*
*********************************************************************************************************
                                               _04_OS
    File             : main.c
*********************************************************************************************************
*/
#include "User_header.h"
#include "User.h"

unsigned int TASK_0_STK[5120];
unsigned int TASK_1_STK[512];
unsigned int TASK_5_STK[64];

void OS_Init(void)
{
    System_init();
    LED_Init();
    PS2_Keyboard_Init();
    OS_LCD_Init();
}

int main(void)
{
    Task_Create(User_main, &TASK_0_STK[5120 - 1], 2);
    Task_Create(MyPs2KeyScan, &TASK_1_STK[512 - 1], 0);
    Task_Create(LED_main, &TASK_5_STK[64 - 1], 3);

    OS_Init();
    OS_Start();
}
