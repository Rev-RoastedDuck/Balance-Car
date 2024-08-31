/*
 * pid_rrd.h
 *
 *  Created on: 2024年1月28日
 *      Author: intl4419
 */

#ifndef ALGORITHM_PID_RRD_H_
#define ALGORITHM_PID_RRD_H_

/******************************************************************************/
/*---------------------------------头文件引入---------------------------------*/
/******************************************************************************/
#include "typedef.h"
#include "stm32f10x.h"


/** \addtogroup PWM
 * \{ */
/******************************************************************************/
/*-----------------------------------结构体-----------------------------------*/
/******************************************************************************/

// 2024/06/20 修改 pwm_curr_duty 类型为 sint32
typedef struct{
    int32_t pwm_curr_duty;          // 当前的pwm占空比
    uint32_t pwm_count_period;       // pwm的计数周期周期
}PWM_Data;

/** \} */



/** \addtogroup 浮点数PID
 * \{ */



/******************************************************************************/
/*-----------------------------------结构体------------------------------------*/
/******************************************************************************/
typedef struct{
    volatile float En_0;            // 本次偏差
    volatile float En_1;            // 上次偏差
    volatile float En_2;            // 上上次偏差

    volatile float Kp;              // 比例系数:       P
    volatile float Ti;              // 积分时间常数:   I
    volatile float Td;              // 微分时间常数:   D

    volatile float current_value;   // 当前传感器的值
    volatile float desired_value;   // 设定传感器的值


    volatile float calc_result;     // 增量PID计算本次应该输出的增量值--本次计算的结果
    volatile float Tsam;            // 采样周期---控制周期，每隔Tsam控制器输出一次PID运算结果

    PWM_Data *pwm_data;
}PID_Inc_Info;


typedef struct{
    volatile float En_0;            // 本次偏差
    volatile float En_1;            // 上次偏差
    volatile float SEk ;            // 历史偏差值

    volatile float Kp;              // 比例系数:       P
    volatile float Ti;              // 积分时间常数:   I
    volatile float Td;              // 微分时间常数:   D

    volatile float current_value;   // 当前传感器的值
    volatile float desired_value;   // 设定传感器的值

    volatile float calc_result;     // 增量PID计算本次应该输出的增量值--本次计算的结果
    volatile float Tsam;            // 采样周期---控制周期，每隔Tsam控制器输出一次PID运算结果

    PWM_Data *pwm_data;
}PID_Loc_Info;

/******************************************************************************/
/*----------------------------------全局函数-----------------------------------*/
/******************************************************************************/

/**
 * @brief  PID结构体参数初始化
 * @param  pid:    PID
 * @return None
 */
void pid_inc_parament_init_rrd(PID_Inc_Info *pid);
void pid_loc_parament_init_rrd(PID_Loc_Info *pid);

/**
 * @brief  清除偏差值
 * @param  pid:    PID
 * @return None
 */
void pic_loc_clear_error(PID_Loc_Info *pid);
void pic_inc_clear_error(PID_Inc_Info *pid);



/**
 * @brief  PID运算
 * @param  pid:    PID
 * @return None
 */
void pid_inc_calc_rrd(PID_Inc_Info *pid);
void pid_loc_calc_rrd(PID_Loc_Info *pid);
/** \} */

#if 0

/** \addtogroup 整数PID
 * \{ */
/******************************************************************************/
/*-----------------------------------结构体------------------------------------*/
/******************************************************************************/
typedef struct{
    uint32 En_0;            // 本次偏差
    uint32 En_1;            // 上次偏差
    uint32 En_2;            // 上上次偏差

    uint32 Kp;              // 比例系数:       P
    uint32 Ti;              // 积分时间常数:   I
    uint32 Td;              // 微分时间常数:   D

    uint32 current_value;   // 当前传感器的值
    uint32 desired_value;   // 设定传感器的值


    uint32 calc_result;     // 增量PID计算本次应该输出的增量值--本次计算的结果
    uint32 Tsam;            // 采样周期---控制周期，每隔Tsam控制器输出一次PID运算结果

    uint16 multiplier;      // 倍率。过小会导致数值失真，过大会导致数值溢出。10/100/1000/10000

    PWM_Data *pwm_data;
}PID_Int;


/******************************************************************************/
/*----------------------------------全局函数-----------------------------------*/
/******************************************************************************/
/**
 * @brief  PID运算
 * @param  pid:    PID
 * @return None
 */
void pid_int_calc_rrd(PID_Int *pid);
/** \} */
#endif

#endif /* CODE_ALGORITHM_RRD_PID_RRD_H_ */
