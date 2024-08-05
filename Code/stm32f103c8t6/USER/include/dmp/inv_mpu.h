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

//定义输出速度
#define DEFAULT_MPU_HZ  (100)		//100Hz

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

//移植官方MSP430 DMP驱动过来
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
/*-----------------------------------外部函数---------------------------------*/
/******************************************************************************/
/**
  * @brief   			获取当前系统时间（毫秒）
  * @param[out]  	time  指向 `unsigned long` 类型的指针，用于存储当前时间（毫秒）。
  * @return  			无
  * @details			此函数将系统当前时间以毫秒为单位存储到 `time` 指向的变量中。
  * @note   			该函数通常用于时间戳记录或延时计算。
  */
void mget_ms(unsigned long *time);

/**
  * @brief   			将方向矩阵的一行转换为标量
  * @param[in]   	row   指向包含矩阵一行的数组的指针。数组应包含三个 `signed char` 类型的值，表示方向矩阵的一行。
  * @return      	返回一个无符号短整型值，表示转换后的标量值。
  * @details
  * 根据输入行的三个元素的值来确定标量值 `b`。标量值的计算规则如下：
  * - 如果第一个元素大于零，标量值 `b` 为 0。
  * - 如果第一个元素小于零，标量值 `b` 为 4。
  * - 如果第一个元素等于零，且第二个元素大于零，标量值 `b` 为 1。
  * - 如果第二个元素小于零，标量值 `b` 为 5。
  * - 如果第二个元素等于零，且第三个元素大于零，标量值 `b` 为 2。
  * - 如果第三个元素小于零，标量值 `b` 为 6。
  * - 如果所有三个元素均为零，标量值 `b` 为 7，表示错误。
  * 
  * @retval 0 到 7 之间的无符号短整型值，表示转换后的标量值。
  */
unsigned short inv_row_2_scale(const signed char *row);

/**
  * @brief   		将方向矩阵转换为标量值
  * @param[in]   mtx   指向包含 3x3 方向矩阵的数组的指针。数组应包含九个 `signed char` 类型的值，表示方向矩阵的 3x3 行列
  * @return     返回一个无符号短整型值，表示转换后的标量值
  * @details		此函数将方向矩阵的每一行转换为标量值，并将这些标量值组合成一个结果值。标量值的计算由 `inv_row_2_scale` 函数实现
  * @retval  		返回一个无符号短整型值，表示方向矩阵转换后的标量值
  */
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);

/**
  * @brief   		执行 MPU 自检
  * @param[in]  无
  * @return     返回一个 `u8` 类型的值，表示自检结果
  * @details		此函数执行 MPU 传感器的自检过程，以确认传感器的功能和准确性。
  * @retval  		0  表示自检通过
  */
u8 run_self_test(void);

/**
  * @brief   		 	初始化 MPU DMP（数字运动处理器）
  * @param[in]   	无
  * @return      	返回一个 `u8` 类型的值，表示初始化结果。
  * @details			此函数初始化 MPU 的 DMP 功能，以便进行数据处理和计算。
  * @retval  			0  表示初始化成功
  */
u8 mpu_dmp_init(void);

/**
  * @brief   			获取 DMP 计算出的姿态数据
  * @param[out]  	pitch  指向 `float` 类型的指针，用于存储俯仰角（Pitch）。
  * @param[out]  	roll   指向 `float` 类型的指针，用于存储滚转角（Roll）。
  * @param[out]  	yaw    指向 `float` 类型的指针，用于存储偏航角（Yaw）。
  * @return       返回一个 `u8` 类型的值，表示获取数据的结果。
  * @details		  此函数从 MPU 的 DMP 中获取计算出的姿态数据（俯仰角、滚转角和偏航角）。
  * 
  * @retval 			 0  表示成功获取数据
  */
u8 mpu_dmp_get_data(float *pitch, float *roll, float *yaw);

/**
  * @brief   		 获取采样频率分频值
  * @param[out]		smplrt_div	
  * @return  		 0 成功获取
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

