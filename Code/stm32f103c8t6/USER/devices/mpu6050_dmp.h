/*
 * mpu6050_dmp.h
 *	
 *  Created on: 2024��8��2��
 *      Author: Rev_RoastDuck
 */

#ifndef DEVICE_MPU6050_DMP_H_
#define DEVICE_MPU6050_DMP_H_

/******************************************************************************/
/*-----------------------------------DEBUG�ӿ�--------------------------------*/
/******************************************************************************/
#define OPEN_MPU6050_DMP_TEST		1
#if OPEN_MPU6050_DMP_TEST
#include "mpu6050.h"
#include "inv_mpu.h"
void mpu6050_dmp_test(void);
#endif   /* #if OPEN_MPU6050_DMP_TEST */

#endif

