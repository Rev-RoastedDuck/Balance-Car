/*
 * pid_rrd.h
 *
 *  Created on: 2024��1��28��
 *      Author: intl4419
 */

#ifndef ALGORITHM_PID_RRD_H_
#define ALGORITHM_PID_RRD_H_

/******************************************************************************/
/*---------------------------------ͷ�ļ�����---------------------------------*/
/******************************************************************************/
#include "typedef.h"
#include "stm32f10x.h"


/** \addtogroup PWM
 * \{ */
/******************************************************************************/
/*-----------------------------------�ṹ��-----------------------------------*/
/******************************************************************************/

// 2024/06/20 �޸� pwm_curr_duty ����Ϊ sint32
typedef struct{
    int32_t pwm_curr_duty;          // ��ǰ��pwmռ�ձ�
    uint32_t pwm_count_period;       // pwm�ļ�����������
}PWM_Data;

/** \} */



/** \addtogroup ������PID
 * \{ */



/******************************************************************************/
/*-----------------------------------�ṹ��------------------------------------*/
/******************************************************************************/
typedef struct{
    volatile float En_0;            // ����ƫ��
    volatile float En_1;            // �ϴ�ƫ��
    volatile float En_2;            // ���ϴ�ƫ��

    volatile float Kp;              // ����ϵ��:       P
    volatile float Ti;              // ����ʱ�䳣��:   I
    volatile float Td;              // ΢��ʱ�䳣��:   D

    volatile float current_value;   // ��ǰ��������ֵ
    volatile float desired_value;   // �趨��������ֵ


    volatile float calc_result;     // ����PID���㱾��Ӧ�����������ֵ--���μ���Ľ��
    volatile float Tsam;            // ��������---�������ڣ�ÿ��Tsam���������һ��PID������

    PWM_Data *pwm_data;
}PID_Inc_Info;


typedef struct{
    volatile float En_0;            // ����ƫ��
    volatile float En_1;            // �ϴ�ƫ��
    volatile float SEk ;            // ��ʷƫ��ֵ

    volatile float Kp;              // ����ϵ��:       P
    volatile float Ti;              // ����ʱ�䳣��:   I
    volatile float Td;              // ΢��ʱ�䳣��:   D

    volatile float current_value;   // ��ǰ��������ֵ
    volatile float desired_value;   // �趨��������ֵ

    volatile float calc_result;     // ����PID���㱾��Ӧ�����������ֵ--���μ���Ľ��
    volatile float Tsam;            // ��������---�������ڣ�ÿ��Tsam���������һ��PID������

    PWM_Data *pwm_data;
}PID_Loc_Info;

/******************************************************************************/
/*----------------------------------ȫ�ֺ���-----------------------------------*/
/******************************************************************************/

/**
 * @brief  PID�ṹ�������ʼ��
 * @param  pid:    PID
 * @return None
 */
void pid_inc_parament_init_rrd(PID_Inc_Info *pid);
void pid_loc_parament_init_rrd(PID_Loc_Info *pid);

/**
 * @brief  ���ƫ��ֵ
 * @param  pid:    PID
 * @return None
 */
void pic_loc_clear_error(PID_Loc_Info *pid);
void pic_inc_clear_error(PID_Inc_Info *pid);



/**
 * @brief  PID����
 * @param  pid:    PID
 * @return None
 */
void pid_inc_calc_rrd(PID_Inc_Info *pid);
void pid_loc_calc_rrd(PID_Loc_Info *pid);
/** \} */

#if 0

/** \addtogroup ����PID
 * \{ */
/******************************************************************************/
/*-----------------------------------�ṹ��------------------------------------*/
/******************************************************************************/
typedef struct{
    uint32 En_0;            // ����ƫ��
    uint32 En_1;            // �ϴ�ƫ��
    uint32 En_2;            // ���ϴ�ƫ��

    uint32 Kp;              // ����ϵ��:       P
    uint32 Ti;              // ����ʱ�䳣��:   I
    uint32 Td;              // ΢��ʱ�䳣��:   D

    uint32 current_value;   // ��ǰ��������ֵ
    uint32 desired_value;   // �趨��������ֵ


    uint32 calc_result;     // ����PID���㱾��Ӧ�����������ֵ--���μ���Ľ��
    uint32 Tsam;            // ��������---�������ڣ�ÿ��Tsam���������һ��PID������

    uint16 multiplier;      // ���ʡ���С�ᵼ����ֵʧ�棬����ᵼ����ֵ�����10/100/1000/10000

    PWM_Data *pwm_data;
}PID_Int;


/******************************************************************************/
/*----------------------------------ȫ�ֺ���-----------------------------------*/
/******************************************************************************/
/**
 * @brief  PID����
 * @param  pid:    PID
 * @return None
 */
void pid_int_calc_rrd(PID_Int *pid);
/** \} */
#endif

#endif /* CODE_ALGORITHM_RRD_PID_RRD_H_ */
