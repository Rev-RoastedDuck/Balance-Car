/*
 * tool.h
 *
 *  Created on: 2024年7月31日
 *      Author: Rev_RoastDuck
 */

#ifndef COMMON_TOOL_H_
#define COMMON_TOOL_H_

/******************************************************************************/
/*---------------------------------头文件引入---------------------------------*/
/******************************************************************************/
#include "stm32f10x.h"

/******************************************************************************/
/*---------------------------------全局宏定义---------------------------------*/
/******************************************************************************/
#define ARRAY_LENGTH(arr) (sizeof(arr) / sizeof((arr)[0]))
#define ABS(num) 					((num) < 0 ? -(num) : (num))



#endif
