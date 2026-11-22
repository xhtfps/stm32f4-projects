#ifndef USER_H
#define USER_H

/**
 * @file    user.h
 * @brief   工程模板 UI 头文件
 *
 * 提供 LCD 菜单框架常量、数据类型和公共接口声明。
 * 开发新项目时，在此添加自定义的页面宏、数据结构及函数声明。
 */

#include <stdbool.h>
#include <stdint.h>

#include "./BSP/PS2/ps2.h"
#include "./BSP/TFT_LCD/TFT_LCD.h"

/* ================================================================
 * 菜单与页面宏定义
 * ================================================================ */

/** 系统标题字符串（GBK 编码） */
#define TitleStr      "\xB9\xA4\xB3\xCC\xC4\xA3\xB0\xE5"  /* 工程模板 */
/** 硬件版本字符串 */
#define ModelVerStr   "STM32F4"
/** 软件版本字符串 */
#define UserVerStr    "V1.0"
/** 菜单项数量 */
#define MenuChoiceNum 5U
/** 菜单项文本宏 */
#define Menu1Choice1  "1 MPU6050"
#define Menu1Choice2  "2 AD7606 TEST"
#define Menu1Choice3  "3 PAGE 3"
#define Menu1Choice4  "4 PAGE 4"
#define Menu1Choice5  "5 PAGE 5"
/** 菜单项数量（与 MenuChoiceNum 一致） */
#define UI_MENU_COUNT 5U

/* ================================================================
 * LCD 显示区域常量
 * ================================================================ */

/** 右侧内容区起始 X 坐标 */
#define CONTENT_LEFT    314U
/** 数据数值显示 X 坐标 */
#define DATA_VALUE_X    564U
/** 左侧菜单字号 */
#define UI_MENU_SIZE    24U
/** 正文字号 */
#define UI_PRIMARY_SIZE 24U
/** 提示小字号 */
#define UI_HELP_SIZE    16U
/** 页面数量 */
#define UI_PAGE_COUNT   5U

/* ================================================================
 * UI 命令类型
 * ================================================================ */

/** UI 层按键触发的底层执行命令类型 */
typedef enum
{
    UI_COMMAND_NONE = 0,               /**< 无效命令 */
    UI_COMMAND_TEMPLATE_1,             /**< 模板命令 1 */
    UI_COMMAND_TEMPLATE_2,             /**< 模板命令 2 */
} UI_CommandType_t;

/**
 * @brief UI 命令结构体
 *
 * 封装 UI 层按键触发的参数和执行命令。
 */
typedef struct
{
    UI_CommandType_t type;    /**< 命令类型 */
    uint32_t param1;          /**< 参数 1（预留） */
    float param2;             /**< 参数 2（预留） */
} UI_Command_t;

/* ================================================================
 * LCD 显示工具函数
 *
 * 这些函数提供了底层的字符、字符串、数字显示能力。
 * ================================================================ */

/**
 * @brief 设置中文字库是否可用
 * @param available 1=可用（GBK显示），0=不可用（纯英文回退）
 */
void UI_SetChineseFontAvailable(uint8_t available);

/**
 * @brief 初始化 UI，绘制主界面
 */
void UI_Init(void);

/**
 * @brief 显示主页面（全屏重绘）
 */
void Disp_Main(void);

/**
 * @brief 用户主循环（按键扫描 + UI 更新）
 */
void User_main(void);

/**
 * @brief 全初始化（兼容旧版接口）
 */
void Init_All(void);

/**
 * @brief 在指定位置显示格式化数值
 * @param location 位置编号 (1~20)
 * @param value    待显示数值
 * @param format   printf 格式字符串
 */
void Show_Val(uint8_t location, float value, char *format);

/**
 * @brief 外部切换菜单页面
 * @param menu_sign 页面编号 (1~MenuChoiceNum)
 */
void Change_Menu(uint8_t menu_sign);

/**
 * @brief 获取当前页面编号
 * @return 当前页面编号 (0=主页, 1~5=菜单页)
 */
uint8_t UI_GetCurrentPage(void);

/**
 * @brief 刷新 MPU6050 数据（在菜单 1 页面时更新显示数值）
 */
void UI_Mpu6050_UpdateData(void);

/** 刷新 AD7606 八通道采样数据（菜单 2 页面）。 */
void UI_Ad7606_UpdateData(void);

/* 旧版菜单切换函数（兼容保留） */
void MenuHaddler_1(void);
void MenuHaddler_2(void);
void MenuHaddler_3(void);
void MenuHaddler_4(void);
void MenuHaddler_5(void);

/**
 * @brief 预置数值读取接口（兼容旧版）
 */
float PS2_ReadNum(float num);

/**
 * @brief 键盘事件处理
 * @param key     按键值
 * @param command 输出参数：触发的命令
 * @return true=产生有效命令, false=仅页面操作
 */
bool UI_HandleKey(PS2_KeyValue_t key, UI_Command_t *command);

/**
 * @brief 显示 FreeRTOS 演示页面（按键切换）
 * @param page 页面编号 (0~3)
 */
void UI_ShowDemoPage(uint8_t page);

/* ================================================================
 * 用户自定义区
 *
 * 开发新项目时，在此添加：
 *   - 自定义页面宏 (#define PAGE_xxx  n)
 *   - 自定义数据结构 (typedef struct ...)
 *   - 自定义函数声明
 * ================================================================ */

#endif /* USER_H */
