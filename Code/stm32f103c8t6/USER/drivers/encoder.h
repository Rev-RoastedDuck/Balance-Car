/*
 * encoder.h
 *	
 *  Created on: 2024年8月1日
 *      Author: Rev_RoastDuck
 */

#ifndef DRIVERS_ENCODER_H_
#define DRIVERS_ENCODER_H_

/******************************************************************************/
/*---------------------------------头文件引入---------------------------------*/
/******************************************************************************/
#include "stm32f10x.h"
#include "debug_uart.h"
#include <stdlib.h>

/******************************************************************************/
/*------------------------------------结构体----------------------------------*/
/******************************************************************************/
typedef struct _RRD_DRIVER_ENCODER {
	TIM_TypeDef*  tim_x;
	
	void 					(*encoder_driver_del)  (struct _RRD_DRIVER_ENCODER* self);
	void 					(*encoder_config)			 (struct _RRD_DRIVER_ENCODER *self,
																				GPIO_TypeDef* ic1_gpio_x, const uint16_t	ic1_gpio_pin,
																			  GPIO_TypeDef* ic2_gpio_x, const uint16_t		ic2_gpio_pin,
																			  TIM_TypeDef*  tim_x);
	int32_t 			(*encoder_get_count)	 (struct _RRD_DRIVER_ENCODER *self);
	void 					(*encoder_clear_count) (struct _RRD_DRIVER_ENCODER *self);
}RRD_DRIVER_ENCODER;


/******************************************************************************/
/*----------------------------------结构体方法--------------------------------*/
/******************************************************************************/
RRD_DRIVER_ENCODER* encoder_driver_new(void);

/******************************************************************************/
/*-----------------------------------DEBUG接口--------------------------------*/
/******************************************************************************/
#define OPEN_ENCODER_TEST		1

#if OPEN_ENCODER_TEST
	void encoder_test_start(void);
#endif

#endif

