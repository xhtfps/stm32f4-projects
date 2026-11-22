/* ****************************
 * Project description:
 *
 * A Project empty template head file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/09/05
 *
 * Update date: 2021/11/03
 * ****************************/
#ifndef USER_H
#define USER_H

/* ***************************** Include & Define Part     	*****************************
 * 头文件声明及宏定义区
 * */
#include "User_header.h"

#define TitleLength 4
#define TitleStr "测试程序"
#define ModelVerStr "MD:6-2021/10/29"
#define UserVerStr "User:1.0-2026/6/5"

#define MenuChoiceNum 4
#define Menu1Choice1 "AD DA"
#define Menu1Choice2 "AD9959"
#define Menu1Choice3 "ADS1256"
#define Menu1Choice4 "PGA4311"
#define Menu1Choice5 "模板选项5"

/* ***************************** Function Declaration Part  *****************************
 * 函数声明区
 * */
void User_main(void);

void Init_All(void);

void Disp_Main(void);
void Show_Val( uint8_t location , float value , char *str );
void Change_Menu( uint8_t menu_sign );

void MenuHaddler_1(void);
void MenuHaddler_2(void);
void MenuHaddler_3(void);
void MenuHaddler_4(void);
void MenuHaddler_5(void);

float PS2_ReadNum( float num );

/* ***************************** Variable definition Part   *****************************/







#endif






