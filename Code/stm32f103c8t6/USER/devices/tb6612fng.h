/*
 * tb6612fng.h
 *	
 *  Created on: 2024��8��9��
 *      Author: Rev_RoastDuck
 */

#ifndef DEVICE_TB6612FNG_H_
#define DEVICE_TB6612FNG_H_

/******************************************************************************/
/*---------------------------------ͷ�ļ�����---------------------------------*/
/******************************************************************************/
#include "stm32f10x.h"
#include <stdlib.h>
#include "debug_uart.h"

/******************************************************************************/
/*------------------------------------�ṹ��----------------------------------*/
/******************************************************************************/
typedef struct _RRD_DEVICE_TB6612FNG {
	GPIO_TypeDef* in1_gpio_x;
	uint16_t 			in1_gpio_pin;
	
	GPIO_TypeDef* in2_gpio_x;
	uint16_t 			in2_gpio_pin;

	int8_t 	(*init) 		(struct _RRD_DEVICE_TB6612FNG* driver,GPIO_TypeDef* in1_gpio_x, const uint16_t in1_gpio_pin,GPIO_TypeDef* in2_gpio_x, const uint16_t in2_gpio_pin);
	void 		(*forward)	(struct _RRD_DEVICE_TB6612FNG* driver);
	void 		(*backward)	(struct _RRD_DEVICE_TB6612FNG* driver);
	void 		(*stop)			(struct _RRD_DEVICE_TB6612FNG* driver);

} RRD_DEVICE_TB6612FNG;

/******************************************************************************/
/*----------------------------------�ṹ�巽��--------------------------------*/
/******************************************************************************/
RRD_DEVICE_TB6612FNG* tb6612fng_device_new(void);

#endif




