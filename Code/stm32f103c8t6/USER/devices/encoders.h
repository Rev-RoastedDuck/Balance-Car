/*
 * encoders.h
 *	
 *  Created on: 2024年8月11日
 *      Author: Rev_RoastDuck
 */


#ifndef DEVICES_ENCODERS_H_
#define DEVICES_ENCODERS_H_

/******************************************************************************/
/*---------------------------------头文件引入---------------------------------*/
/******************************************************************************/
#include "stm32f10x.h"
#include "encoder.h"

/******************************************************************************/
/*-----------------------------------配置层-----------------------------------*/
/******************************************************************************/



/******************************************************************************/
/*------------------------------------结构体----------------------------------*/
/******************************************************************************/
typedef struct _RRD_DEVICE_ENCODERS {
	RRD_DRIVER_ENCODER* encoder_driver_left;
	RRD_DRIVER_ENCODER* encoder_driver_right;
	
	void 		(*init)							(void);
	void 		(*get_counts)						(int16_t *encoder_left,int16_t *encoder_right);
}RRD_DEVICE_ENCODERS;

/******************************************************************************/
/*-----------------------------------外部接口---------------------------------*/
/******************************************************************************/
extern RRD_DEVICE_ENCODERS ENCODERS_DEVICE;
#endif

