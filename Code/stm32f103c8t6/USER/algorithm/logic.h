/*
 * logic.h
 *
 *  Created on: 2024年2月4日
 *      Author: intl4419
 */

#ifndef ALGORITHM_LOGIC_H_
#define ALGORITHM_LOGIC_H_

#include "debug_uart.h"
#include "pid_rrd.h"

/**
 * @brief   		计算pid位置环
 * @param[in]   pid  位置式pid
 * @param[in]   current_angle  当前角度
 * @param[in]   gyro_y  			 y轴速度
 * @return  		pid计算结果
 */
float logic_calc_pid_balance(PID_Loc_Info *pid,float current_angle,float gyro_y);

/**
 * @brief   		计算pid速度环
 * @param[in]   pid  位置式pid
 * @param[in]   encoder_value_left   左编码器数值
 * @param[in]   encoder_value_right  右编码器数值
 * @return  		pid计算结果
 */
float logic_calc_pid_speed(PID_Loc_Info *pid,int32_t encoder_value_left,int32_t encoder_value_right);

/**
 * @brief   		计算pid转向环(yaw)
 * @param[in]   pid  位置式pid
 * @return  		pid计算结果
 */
float logic_calc_pid_turn(PID_Loc_Info *pid);

#endif /* CODE_LOGIC_H_ */
