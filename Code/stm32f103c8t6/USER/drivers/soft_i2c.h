/*
 * soft_i2c.h
 *	
 *  Created on: 2024年7月30日
 *      Author: Rev_RoastDuck
 */

#ifndef DRIVERS_SOFT_I2C_H_
#define DRIVERS_SOFT_I2C_H_

/******************************************************************************/
/*---------------------------------头文件引入---------------------------------*/
/******************************************************************************/
#include "stm32f10x.h"
#include "stddef.h"
#include "debug_uart.h"

/******************************************************************************/
/*------------------------------------结构体----------------------------------*/
/******************************************************************************/
typedef struct _RRD_DRIVER_IIC {
	GPIO_TypeDef* iic_gpio_port;
	uint16_t 			iic_scl_pin;
	uint16_t 			iic_sda_pin;
	
	void 		(*i2c_stop)					(struct _RRD_DRIVER_IIC *self);             
	void 		(*i2c_start)				(struct _RRD_DRIVER_IIC *self);             

	uint8_t (*i2c_read_byte) 		(struct _RRD_DRIVER_IIC *self, uint8_t ack);   		
	void 		(*i2c_send_byte) 		(struct _RRD_DRIVER_IIC *self, uint8_t byte); 
	
	uint8_t (*i2c_wait_ack)			(struct _RRD_DRIVER_IIC *self);           	
	void 		(*i2c_ack)					(struct _RRD_DRIVER_IIC *self);             
	void 		(*i2c_nack)					(struct _RRD_DRIVER_IIC *self);             
	
	void 		(*i2c_gpio_config)	(struct _RRD_DRIVER_IIC *self);           
	uint8_t (*i2c_check_device)	(struct _RRD_DRIVER_IIC *self, uint8_t _Address); 	
}RRD_DRIVER_IIC;


/******************************************************************************/
/*----------------------------------结构体方法--------------------------------*/
/******************************************************************************/
int8_t iic_driver_initialize(RRD_DRIVER_IIC *driver);

#endif

