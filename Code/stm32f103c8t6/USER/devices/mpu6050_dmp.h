/*
 * mpu6050_dmp.h
 *	
 *  Created on: 2024年8月2日
 *      Author: Rev_RoastDuck
 */

#ifndef DEVICE_MPU6050_DMP_H_
#define DEVICE_MPU6050_DMP_H_

/******************************************************************************/
/*-----------------------------------DEBUG接口--------------------------------*/
/******************************************************************************/
#define OPEN_MPU6050_DMP_TEST		1
#if OPEN_MPU6050_DMP_TEST
#include "mpu6050.h"
#include "inv_mpu.h"
void mpu6050_dmp_test(void);
#endif   /* #if OPEN_MPU6050_DMP_TEST */

#endif

