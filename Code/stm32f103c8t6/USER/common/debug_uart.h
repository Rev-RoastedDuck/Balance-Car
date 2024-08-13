/*
 * debug_uart.h
 *
 *  Created on: 2024��7��29��
 *      Author: Rev_RoastDuck
 */

#ifndef COMMON_DEBUG_UART_H_
#define COMMON_DEBUG_UART_H_

/******************************************************************************/
/*---------------------------------ͷ�ļ�����---------------------------------*/
/******************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "Driver_USART.h"
#include "stm32f10x.h" 

/******************************************************************************/
/*---------------------------------�ڲ��궨��---------------------------------*/
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
/*----------------------------------�ڲ�����----------------------------------*/
/******************************************************************************/
/**
 * @brief 		����debug��Ϣ�Ĵ�����
 * @param[in] pass �����Ƿ��ӡ�ı�־
 * @param[in] fmt ��ʽ���ַ���
 * @param[in] file �ļ���
 * @param[in] line �к�
 */
void debug_uart_print_handle(const uint8_t pass ,const char *fmt, const char *file, int line, ...);
		
		
/******************************************************************************/
/*----------------------------------ȫ�ֺ���----------------------------------*/
/******************************************************************************/
/**
 * @brief ����debug��Ϣ
 * @note	���������ж���ʹ��
 * @note	���������ж���ʹ��
 * @note	���������ж���ʹ��
 */
#define debug_uart_printf_1(fmt, ...) 				DEBUG_PRINT_1(fmt, ##__VA_ARGS__)
#define debug_uart_printf_2(pass, fmt, ...) 	DEBUG_PRINT_2(pass, fmt, ##__VA_ARGS__)
		
/**
 * @brief		��ʼ��debug�豸
 * @return	0 on success, or -1 on error.
 */
int8_t debug_uart_init(void);

/**
 * @brief  			���͸�ʽ���ַ���
 * @param[in] 	fmt ��ʽ���ַ���
 * @return			None
 * @note				variable argument list string printf return num
 * @note				�������ж���ʹ��
 * @note				�������ж���ʹ��
 * @note				�������ж���ʹ��
 */
void debug_uart_send(const char *fmt, ...);
#endif
