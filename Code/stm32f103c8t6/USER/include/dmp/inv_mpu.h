/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu.h
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 *                  MPU6500
 *                  MPU9150 (or MPU6050 w/ AK8975 on the auxiliary bus)
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */

#ifndef _INV_MPU_H_
#define _INV_MPU_H_
#include "stm32f10x.h"

//��������ٶ�
#define DEFAULT_MPU_HZ  (100)		//100Hz

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

//��ֲ�ٷ�MSP430 DMP��������
struct int_param_s {
//#if defined EMPL_TARGET_MSP430 || defined MOTION_DRIVER_TARGET_MSP430
    void (*cb)(void);
    unsigned short pin;
    unsigned char lp_exit;
    unsigned char active_low;
//#elif defined EMPL_TARGET_UC3L0
//    unsigned long pin;
//    void (*cb)(volatile void*);
//    void *arg;
//#endif
};

#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DMP              (0x0002)
#define MPU_INT_STATUS_PLL_READY        (0x0004)
#define MPU_INT_STATUS_I2C_MST          (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW    (0x0010)
#define MPU_INT_STATUS_ZMOT             (0x0020)
#define MPU_INT_STATUS_MOT              (0x0040)
#define MPU_INT_STATUS_FREE_FALL        (0x0080)
#define MPU_INT_STATUS_DMP_0            (0x0100)
#define MPU_INT_STATUS_DMP_1            (0x0200)
#define MPU_INT_STATUS_DMP_2            (0x0400)
#define MPU_INT_STATUS_DMP_3            (0x0800)
#define MPU_INT_STATUS_DMP_4            (0x1000)
#define MPU_INT_STATUS_DMP_5            (0x2000)

/* Set up APIs */
int mpu_init(void);
int mpu_init_slave(void);
int mpu_set_bypass(unsigned char bypass_on);

/* Configuration APIs */
int mpu_lp_accel_mode(unsigned char rate);
int mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time,
    unsigned char lpa_freq);
int mpu_set_int_level(unsigned char active_low);
int mpu_set_int_latched(unsigned char enable);

int mpu_set_dmp_state(unsigned char enable);
int mpu_get_dmp_state(unsigned char *enabled);

int mpu_get_lpf(unsigned short *lpf);
int mpu_set_lpf(unsigned short lpf);

int mpu_get_gyro_fsr(unsigned short *fsr);
int mpu_set_gyro_fsr(unsigned short fsr);

int mpu_get_accel_fsr(unsigned char *fsr);
int mpu_set_accel_fsr(unsigned char fsr);

int mpu_get_compass_fsr(unsigned short *fsr);

int mpu_get_gyro_sens(float *sens);
int mpu_get_accel_sens(unsigned short *sens);

int mpu_get_sample_rate(unsigned short *rate);
int mpu_set_sample_rate(unsigned short rate);
int mpu_get_compass_sample_rate(unsigned short *rate);
int mpu_set_compass_sample_rate(unsigned short rate);

int mpu_get_fifo_config(unsigned char *sensors);
int mpu_configure_fifo(unsigned char sensors);

int mpu_get_power_state(unsigned char *power_on);
int mpu_set_sensors(unsigned char sensors);

int mpu_set_accel_bias(const long *accel_bias);

/* Data getter/setter APIs */
int mpu_get_int_status(short *status);
int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
    unsigned char *sensors, unsigned char *more);
int mpu_read_fifo_stream(unsigned short length, unsigned char *data,
    unsigned char *more);
int mpu_reset_fifo(void);

int mpu_write_mem(unsigned short mem_addr, unsigned short length,
    unsigned char *data);
int mpu_read_mem(unsigned short mem_addr, unsigned short length,
    unsigned char *data);
int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
    unsigned short start_addr, unsigned short sample_rate);

int mpu_reg_dump(void);
int mpu_read_reg(unsigned char reg, unsigned char *data);
int mpu_run_self_test(long *gyro, long *accel);
int mpu_register_tap_cb(void (*func)(unsigned char, unsigned char));


/******************************************************************************/
/*-----------------------------------�ⲿ����---------------------------------*/
/******************************************************************************/
/**
  * @brief   			��ȡ��ǰϵͳʱ�䣨���룩
  * @param[out]  	time  ָ�� `unsigned long` ���͵�ָ�룬���ڴ洢��ǰʱ�䣨���룩��
  * @return  			��
  * @details			�˺�����ϵͳ��ǰʱ���Ժ���Ϊ��λ�洢�� `time` ָ��ı����С�
  * @note   			�ú���ͨ������ʱ�����¼����ʱ���㡣
  */
void mget_ms(unsigned long *time);

/**
  * @brief   			����������һ��ת��Ϊ����
  * @param[in]   	row   ָ���������һ�е������ָ�롣����Ӧ�������� `signed char` ���͵�ֵ����ʾ��������һ�С�
  * @return      	����һ���޷��Ŷ�����ֵ����ʾת����ı���ֵ��
  * @details
  * ���������е�����Ԫ�ص�ֵ��ȷ������ֵ `b`������ֵ�ļ���������£�
  * - �����һ��Ԫ�ش����㣬����ֵ `b` Ϊ 0��
  * - �����һ��Ԫ��С���㣬����ֵ `b` Ϊ 4��
  * - �����һ��Ԫ�ص����㣬�ҵڶ���Ԫ�ش����㣬����ֵ `b` Ϊ 1��
  * - ����ڶ���Ԫ��С���㣬����ֵ `b` Ϊ 5��
  * - ����ڶ���Ԫ�ص����㣬�ҵ�����Ԫ�ش����㣬����ֵ `b` Ϊ 2��
  * - ���������Ԫ��С���㣬����ֵ `b` Ϊ 6��
  * - �����������Ԫ�ؾ�Ϊ�㣬����ֵ `b` Ϊ 7����ʾ����
  * 
  * @retval 0 �� 7 ֮����޷��Ŷ�����ֵ����ʾת����ı���ֵ��
  */
unsigned short inv_row_2_scale(const signed char *row);

/**
  * @brief   		���������ת��Ϊ����ֵ
  * @param[in]   mtx   ָ����� 3x3 �������������ָ�롣����Ӧ�����Ÿ� `signed char` ���͵�ֵ����ʾ�������� 3x3 ����
  * @return     ����һ���޷��Ŷ�����ֵ����ʾת����ı���ֵ
  * @details		�˺�������������ÿһ��ת��Ϊ����ֵ��������Щ����ֵ��ϳ�һ�����ֵ������ֵ�ļ����� `inv_row_2_scale` ����ʵ��
  * @retval  		����һ���޷��Ŷ�����ֵ����ʾ�������ת����ı���ֵ
  */
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);

/**
  * @brief   		ִ�� MPU �Լ�
  * @param[in]  ��
  * @return     ����һ�� `u8` ���͵�ֵ����ʾ�Լ���
  * @details		�˺���ִ�� MPU ���������Լ���̣���ȷ�ϴ������Ĺ��ܺ�׼ȷ�ԡ�
  * @retval  		0  ��ʾ�Լ�ͨ��
  */
u8 run_self_test(void);

/**
  * @brief   		 	��ʼ�� MPU DMP�������˶���������
  * @param[in]   	��
  * @return      	����һ�� `u8` ���͵�ֵ����ʾ��ʼ�������
  * @details			�˺�����ʼ�� MPU �� DMP ���ܣ��Ա�������ݴ���ͼ��㡣
  * @retval  			0  ��ʾ��ʼ���ɹ�
  */
u8 mpu_dmp_init(void);

/**
  * @brief   			��ȡ DMP ���������̬����
  * @param[out]  	pitch  ָ�� `float` ���͵�ָ�룬���ڴ洢�����ǣ�Pitch����
  * @param[out]  	roll   ָ�� `float` ���͵�ָ�룬���ڴ洢��ת�ǣ�Roll����
  * @param[out]  	yaw    ָ�� `float` ���͵�ָ�룬���ڴ洢ƫ���ǣ�Yaw����
  * @return       ����һ�� `u8` ���͵�ֵ����ʾ��ȡ���ݵĽ����
  * @details		  �˺����� MPU �� DMP �л�ȡ���������̬���ݣ������ǡ���ת�Ǻ�ƫ���ǣ���
  * 
  * @retval 			 0  ��ʾ�ɹ���ȡ����
  */
u8 mpu_dmp_get_data(float *pitch, float *roll, float *yaw);

/**
  * @brief   		 ��ȡ����Ƶ�ʷ�Ƶֵ
  * @param[out]		smplrt_div	
  * @return  		 0 �ɹ���ȡ
  */
int mpu_get_smplrt_div(u8 *smplrt_div);


/**
 *  @brief      Read raw gyro data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_gyro_reg(short *data, unsigned long *timestamp);

/**
 *  @brief      Read raw accel data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_accel_reg(short *data, unsigned long *timestamp);

/**
 *  @brief      Read raw compass data.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_compass_reg(short *data, unsigned long *timestamp);

/**
 *  @brief      Read temperature data directly from the registers.
 *  @param[out] data        Data in q16 format.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_temperature(long *data, unsigned long *timestamp);
#endif  /* #ifndef _INV_MPU_H_ */

