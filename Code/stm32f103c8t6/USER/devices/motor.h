/*
 * motor.h
 *	
 *  Created on: 2024年8月9日
 *      Author: Rev_RoastDuck
 */

#ifndef DEVICE_MOTOR_H_
#define DEVICE_MOTOR_H_

/******************************************************************************/
/*---------------------------------头文件引入---------------------------------*/
/******************************************************************************/
#include "pwm.h"


/******************************************************************************/
/*------------------------------------结构体----------------------------------*/
/******************************************************************************/
typedef struct _RRD_DEVICE_MOTOR {
	uint16_t compare;
	uint16_t compare_range[2];
	
	TIM_TypeDef* tim_x;
	TIM_CHANNEL tim_channel;
	
	int8_t		(*init) 								(struct _RRD_DEVICE_MOTOR* motor_driver,GPIO_TypeDef* gpio_x, const uint16_t gpio_pin, TIM_TypeDef* tim_x,const TIM_CHANNEL tim_channel,const uint16_t	tim_period,const uint16_t	tim_prescaler,const uint16_t	tim_clock_div);
	void 			(*set_compare_range) 		(struct _RRD_DEVICE_MOTOR* motor_driver,const uint16_t min,const uint16_t max);
	void 			(*set_compare) 					(struct _RRD_DEVICE_MOTOR* motor_driver,const uint16_t compare);
	uint16_t 	(*get_compare)					(struct _RRD_DEVICE_MOTOR* motor_driver);
	uint16_t 	(*get_tim_period)				(struct _RRD_DEVICE_MOTOR* motor_driver);

} RRD_DEVICE_MOTOR;

/******************************************************************************/
/*----------------------------------结构体方法--------------------------------*/
/******************************************************************************/
RRD_DEVICE_MOTOR* motor_driver_new(void);

#endif




