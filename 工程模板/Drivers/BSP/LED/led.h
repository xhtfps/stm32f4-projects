#ifndef __LED_H
#define __LED_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* 引脚 定义 */

#define LED1_GPIO_PORT                  GPIOB
#define LED1_GPIO_PIN                   GPIO_PIN_7
#define LED1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)             /* PB口时钟使能 */

#define LED2_GPIO_PORT                  GPIOB
#define LED2_GPIO_PIN                   GPIO_PIN_8
#define LED2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)             /* PB口时钟使能 */

#define LED3_GPIO_PORT                  GPIOD
#define LED3_GPIO_PIN                   GPIO_PIN_2
#define LED3_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)             /* PD口时钟使能 */

#define LED4_GPIO_PORT                  GPIOB
#define LED4_GPIO_PIN                   GPIO_PIN_1
#define LED4_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)             /* PB口时钟使能 */

/******************************************************************************************/

/* LED端口定义 */
#define LED1(x)   do{ x ? \
                      HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)       /* LED1 = RED */

#define LED2(x)   do{ x ? \
                      HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)       /* LED2 = GREEN */

#define LED3(x)   do{ x ? \
                      HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)       /* LED3 = BLUE */

#define LED4(x)   do{ x ? \
                      HAL_GPIO_WritePin(LED4_GPIO_PORT, LED4_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LED4_GPIO_PORT, LED4_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)       /* LED4 = YELLOW */

/* LED取反定义 */
#define LED1_TOGGLE()    do{ HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN); }while(0)       /* LED1 = !LED1 */
#define LED2_TOGGLE()    do{ HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_GPIO_PIN); }while(0)       /* LED2 = !LED2 */
#define LED3_TOGGLE()    do{ HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_GPIO_PIN); }while(0)       /* LED3 = !LED3 */
#define LED4_TOGGLE()    do{ HAL_GPIO_TogglePin(LED4_GPIO_PORT, LED4_GPIO_PIN); }while(0)       /* LED4 = !LED4 */

/******************************************************************************************/
/* 外部接口函数*/
void led_init(void);                                                                           /* 初始化 */

#endif
