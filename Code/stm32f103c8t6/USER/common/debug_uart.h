/*
 * debug_uart.h
 *
 *  Created on: 2024年7月29日
 *      Author: Rev_RoastDuck
 */

#ifndef COMMON_DEBUG_UART_H_
#define COMMON_DEBUG_UART_H_

/******************************************************************************/
/*---------------------------------头文件引入---------------------------------*/
/******************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include "Driver_USART.h"


/******************************************************************************/
/*---------------------------------内部宏定义---------------------------------*/
/******************************************************************************/
#define DEBUG_PRINT_1(fmt, ...) \
    do { \
        printf("[%s:%d] " fmt, __FILE__, __LINE__, ##__VA_ARGS__); \
    } while (0)

#define DEBUG_PRINT_2(pass, fmt, ...) \
    do { \
        debug_uart_print_handle(pass,fmt, __FILE__, __LINE__, ##__VA_ARGS__); \
    } while (0)

/******************************************************************************/
/*----------------------------------内部函数----------------------------------*/
/******************************************************************************/
/**
 * @brief 		发送debug信息的处理函数
 * @param[in] pass 控制是否打印的标志
 * @param[in] fmt 格式化字符串
 * @param[in] file 文件名
 * @param[in] line 行号
 */
void debug_uart_print_handle(const uint8_t pass ,const char *fmt, const char *file, int line, ...);
		
		
/******************************************************************************/
/*----------------------------------全局函数----------------------------------*/
/******************************************************************************/
/**
 * @brief 发送debug信息
 */
#define debug_uart_printf_1(fmt, ...) 				DEBUG_PRINT_1(fmt, ##__VA_ARGS__)
#define debug_uart_printf_2(pass, fmt, ...) 	DEBUG_PRINT_2(pass, fmt, ##__VA_ARGS__)
		
/**
 * @brief		初始化debug设备
 * @return	0 on success, or -1 on error.
 */
int8_t debug_uart_init(void);


#endif
