/*
 * pid_rrd.c
 *
 *  Created on: 2024年1月28日
 *      Author: intl4419
 */

#include "pid_rrd.h"

/** \addtogroup 过滤器
 * \{ */
/******************************************************************************/
/*----------------------------------静态函数-----------------------------------*/
/******************************************************************************/

/**
 * @brief  预处理过滤器
 * @param  pid:    PID
 * @return None
 */
static boolean __preprocessing(void *pid,boolean is_pid_int)
{
    boolean will_continue = TRUE;

    // 死区控制

    // 数值滤波

    return will_continue;
}

/**
 * @brief  后处理过滤器
 * @param  pid:    PID
 * @return None
 */
static void __postprocessing(void *pid,boolean is_pid_inc)
{

}
/** \} */



/** \addtogroup 浮点数PID
 * \{ */
/******************************************************************************/
/*----------------------------------静态函数-----------------------------------*/
/******************************************************************************/
/**
 * @brief  增量式pid计算
 * @param  pid:    增量式PID
 * @return None
 */
static void __pid_inc_calc_rrd(PID_Inc_Info *pid)
{
   float dk1;
   float dk2;
   float t1,t2,t3;

    pid->En_0 = pid->desired_value - pid->current_value;                     // 计算本次误差
    dk1 = pid->En_0 - pid->En_1;                                             // 计算本次偏差与上次偏差之差
    dk2 = pid->En_0 - 2 * pid->En_1 + pid->En_2;

    t1 = pid->Kp * dk1;
    t2 = pid->Ti == 0? 0 : ((pid->Kp * pid->Tsam * pid->En_0) / pid->Ti);    // 判断是否需要积分I输出
    t3 = pid->Td == 0? 0 : ((pid->Td * pid->Kp * dk2) / pid->Tsam);

    pid->calc_result = t1 + t2 + t3;                                         // 本次PWM应该输出的增量值
//    2024/3/22
//    车库元素需要倒车，方向由外部根据偏差值的正负绝对，
//    等等，好像车库开环控制就可以了

    // 2024/06/20
//    pid->pwm_data->pwm_curr_duty += (uint32)(pid->calc_result);                        // 计算本次PWM应该输出的最终值

    pid->En_2 = pid->En_1;                                                   // 保存误差值
    pid->En_1 = pid->En_0;

//    printf("t1:%f \r\n",t1);
//    printf("t2:%f \r\n",t2);
//    printf("t3:%f \r\n",t3);
//    printf("calc_result:%f\r\n",pid->calc_result);
////    printf("pwm_curr_duty:%ld\r\n",pid->pwm_data->pwm_curr_duty);
//    printf("=========\r\n");


    /**\NOTE 浮点数PID
     *  考虑到motor.h中，满速占空比控制函数
     *   void motor_set_speed_full_rrd(Motor_Channel motor_channel ,uint16 speed,Motor_Direction direction);
     *   所以，解除以下阈值控制
     */
//    pid->calc_result = t1 + t2 + t3;                                         // 本次PWM应该输出的增量值
//    pid->pwm_data->pwm_curr_duty += pid->calc_result;                        // 本次PWM应该输出的最终值
//
//    pid->En_2 = pid->En_1;                                                   // 保存误差值
//    pid->En_1 = pid->En_0;
//    if(pid->pwm_data->pwm_curr_duty<0)
//    {
//        pid->pwm_data->pwm_curr_duty=0;
//        return;
//    }
//
//    if(pid->pwm_data->pwm_curr_duty>pid->pwm_data->pwm_count_period)
//    {
//        pid->pwm_data->pwm_curr_duty=pid->pwm_data->pwm_count_period;
//        return;
//    }

}

/**
 * @brief  位置式 pid计算
 * @param  pid:    PID
 * @return None
 */
static void __pid_loc_calc_rrd(PID_Loc_Info *pid)
{
    float td;
    float t1,t2,t3;

//    pid->En_0 = pid->desired_value - pid->current_value;                         // 得到当前的偏差值
    pid->SEk += pid->En_0;                                                       // 历史偏差总和

    t1 = pid->Kp * pid->En_0;                                                    // 比例输出 P
    t2 = pid->Ti == 0? 0:((pid->Tsam/pid->Ti)*pid->Kp)*pid->SEk*pid->Kp;         // 积分输出 I

    td = pid->Td / pid->Tsam;                                                    // 微分输出 D
    t3 = td * pid->Kp * (pid->En_0 - pid->En_1);                                 // (最近两次偏差之差)

    pid->calc_result = t1 + t2 + t3;
//    pid->pwm_data->pwm_curr_duty = pid->calc_result;                             // pid输出结果
    pid->En_1 = pid->En_0;                                                       //更新偏差

//    printf("t1:%f\r\n",t1);
//    printf("t2:%f\r\n",t2);
//    printf("t3:%f\r\n",t3);
//    printf("pid->calc_result:%f\r\n",pid->calc_result);
//    printf("pid->pwm_data->pwm_curr_duty:%ld\r\n",pid->pwm_data->pwm_curr_duty);

//    舵机pwm设置处已有范围限制
//    // 范围控制
//    if(pid->pwm_data->pwm_curr_duty<0)
//    {
//        pid->pwm_data->pwm_curr_duty=0;
//        return;
//    }
//
//    if(pid->pwm_data->pwm_curr_duty>pid->pwm_data->pwm_count_period)
//    {
//        pid->pwm_data->pwm_curr_duty=pid->pwm_data->pwm_count_period;
//        return;
//    }
}

/******************************************************************************/
/*----------------------------------全局函数-----------------------------------*/
/******************************************************************************/
/**
 * @brief  pid初始化
 * @param  pid:    PID
 * @return None
 */
void pid_parament_init_rrd(PID_Inc_Info *pid)
{
    pid->Kp=0;
    pid->Td=0;
    pid->Ti=0;

    pid->En_0 = 0;
    pid->En_1 = 0;
    pid->En_2 = 0;

    pid->desired_value = 0;
    pid->current_value = 0;

    pid->desired_value =0;
    pid->pwm_data->pwm_curr_duty=0;
    pid->pwm_data->pwm_count_period=0;
}


void pid_Inc_calc_rrd(PID_Inc_Info *pid)
{
    __preprocessing(pid,FALSE);
    __pid_inc_calc_rrd(pid);
    __postprocessing(pid,TRUE);
}

void pid_loc_calc_rrd(PID_Loc_Info *pid)
{
    __preprocessing(pid,FALSE);
    __pid_loc_calc_rrd(pid);
    __postprocessing(pid,FALSE);
}

/** \} */

# if 0
/** \addtogroup 整数PID
 * \{ */
/******************************************************************************/
/*----------------------------------静态函数-----------------------------------*/
/******************************************************************************/

void __pid_int_calc_rrd(PID_Int *pid)
{
    sint32 dk1;
    sint32 dk2;
    sint32 t1, t2, t3;

    pid->En_0 = (pid->desired_value - pid->current_value);                   // 计算本次误差
    dk1 = pid->En_0 - pid->En_1;                                             // 计算本次偏差与上次偏差之差
    dk2 = pid->En_0 - 2 * pid->En_1 + pid->En_2;

    t1 = pid->Kp * dk1;
    t2 = ((pid->Kp * pid->Tsam * pid->En_0) / pid->Ti);
    t3 = ((pid->Kp * pid->Td * dk2) / pid->Tsam);

    pid->calc_result = t1 + t2 + t3;                                         // 本次PWM应该输出的增量值
    pid->pwm_data->pwm_curr_duty += (uint16)(pid->calc_result / pid->multiplier);      // 本次PWM应该输出的最终值

    pid->En_2 = pid->En_1;                                                   // 保存误差值
    pid->En_1 = pid->En_0;

    // 范围控制
    if (pid->pwm_data->pwm_curr_duty < 0) {
        pid->pwm_data->pwm_curr_duty = 0;
        return;
    }

    if (pid->pwm_data->pwm_curr_duty > pid->pwm_data->pwm_count_period) {
        pid->pwm_data->pwm_curr_duty = pid->pwm_data->pwm_count_period;
        return;
    }
}

/******************************************************************************/
/*----------------------------------全局函数-----------------------------------*/
/******************************************************************************/
void pid_int_parament_init_rrd(PID_Int *pid)
{
    pid->Kp=0;
    pid->Td=0;
    pid->Ti=0;

    pid->En_0 = 0;
    pid->En_1 = 0;
    pid->En_2 = 0;

    pid->multiplier = 1000;

    pid->desired_value = 0;
    pid->current_value = 0;

    pid->desired_value =0;
    pid->pwm_data->pwm_curr_duty=0;
    pid->pwm_data->pwm_count_period=0;
}

void pid_int_calc_rrd(PID_Int *pid)
{
    __preprocessing(pid,TRUE);
    __pid_int_calc_rrd(pid);
    __postprocessing(pid,TRUE);
}
/** \} */
#endif
