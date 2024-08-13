/*
 * oled.h
 *	
 *  Created on: 2024��8��3��
 *      Author: Rev_RoastDuck
 */


#ifndef DEVICES_OLED_H_
#define DEVICES_OLED_H_

/******************************************************************************/
/*---------------------------------ͷ�ļ�����---------------------------------*/
/******************************************************************************/
#include "stm32f10x.h"
#include "soft_i2c.h"


/******************************************************************************/
/*-----------------------------------���ò�-----------------------------------*/
/******************************************************************************/
#define OLED_IIC_PORT            (GPIOB)                                   // ��� IIC GPIO
#define OLED_SCL_PIN             (GPIO_Pin_9)                              // ��� IIC SCL ���� 
#define OLED_SDA_PIN             (GPIO_Pin_8)   

/******************************************************************************/
/*------------------------------------�ṹ��----------------------------------*/
/******************************************************************************/
typedef struct _RRD_DEVICE_OLED {
    void (*oled_init)							(void);
    void (*oled_clear)						(void);
    void (*oled_show_char)				(uint8_t line, uint8_t column, char ch);
    void (*oled_show_string)			(uint8_t line, uint8_t column, char *string);
    void (*oled_show_num)					(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
    void (*oled_show_signed_num)	(uint8_t line, uint8_t column, int32_t number, uint8_t length);
    void (*oled_show_hex_num)			(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
    void (*oled_show_float)				(uint8_t line, uint8_t column, float number, uint8_t precision);
		void (*oled_show_bin_num)			(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
} RRD_DEVICE_OLED;

/******************************************************************************/
/*-----------------------------------�ⲿ�ӿ�---------------------------------*/
/******************************************************************************/
extern RRD_DEVICE_OLED OLED_DRIVER;

/******************************************************************************/
/*-----------------------------------DEBUG�ӿ�--------------------------------*/
/******************************************************************************/
#define OPEN_OLED_TEST		0
#if OPEN_OLED_TEST
	void oled_test_start(void);
#endif

#endif /* \} ifndef DEVICES_OLED_H_ */
