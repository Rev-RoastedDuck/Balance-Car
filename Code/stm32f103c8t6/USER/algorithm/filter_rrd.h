/*
 * filter.h
 *
 *  Created on: 2024��1��29��
 *      Author: intl4419
 */

#ifndef ALGORITHM_FILTER_RRD_H_
#define ALGORITHM_FILTER_RRD_H_

/******************************************************************************/
/*---------------------------------ͷ�ļ�����---------------------------------*/
/******************************************************************************/
#include "typedef.h"
#include "stm32f10x.h"


/******************************************************************************/
/*----------------------------------���ò���----------------------------------*/
/******************************************************************************/
#define QUEUE_SIZE 8


/** \addtogroup ����
 * \{ */

/******************************************************************************/
/*-----------------------------------�ṹ��------------------------------------*/
/******************************************************************************/

/**
 * @brief  ���нṹ��
 */
typedef struct {
    uint8_t   head;               // ����ͷ��ָ��
    uint8_t   tail;               // ����β��ָ�룬ָ����һ��Ҫ���޸ĵ�λ�ã������ǵ�ǰ�����޸ĵ�λ�ã���ǰ�����޸ĵ�λ��ֵΪtail-1
    uint8_t   count;              // �����������Ԫ�ص�����
    float sum;                // �ܺ�
    float data[QUEUE_SIZE];   // ����
} Circular_Queue_Filter;

/******************************************************************************/
/*----------------------------------ȫ�ֺ���-----------------------------------*/
/******************************************************************************/
/**
 * @brief  ���
 * @param  queue:    ����
 * @parma  value:    ��ӵ���ֵ
 * @return None
 */
void queue_enqueue(Circular_Queue_Filter* queue, float value);

/**
 * @brief  ��ӣ�ͬʱ��������ܺ�
 * @param  queue:    ����
 * @parma  value:    ��ӵ���ֵ
 * @return None
 */
void queue_enqueue_sum(Circular_Queue_Filter* queue, float value);

/** \} */



/** \addtogroup ������
 * \{ */

/******************************************************************************/
/*-----------------------------------�ṹ��------------------------------------*/
/******************************************************************************/

/**
 * @brief  һ���˲��ṹ��
 * @note    aԽ���²ɼ���ֵռ��Ȩ��Խ���㷨Խ��������ƽ˳�Բ
 * @note    aԽС���²ɼ���ֵռ��Ȩ��ԽС���㷨�����Ȳ��ƽ˳�Ժá�
 */
typedef struct {
    float k;                    // ��Χ: 0~1
    float last_value;           // ��һ����ֵ
    uint8_t is_first;             // �ж��Ƿ��ǵ�һ���˲�
} First_Order_Filter_Info;

/**
 * @brief  �������˲��ṹ��
 */
typedef struct {
    float filterValue;          //�˲����ֵ
    float kalmanGain;           //Kalamn���棬������ϵ����1.Ȩ��۲�ģ�ͺ�Ԥ��ģ��
    float A;                    //״̬���󣬶�Ӧ�����ٶ�
    float H;                    //�۲���󣬱�ʾ�۲�ֵ��ʵ��ֵ�����Թ�ϵ
    float Q;                    //״̬����ķ��Ԥ��ģ�ʹ�����ƫ��(����)
    float R;                    //�۲����ķ���۲�ʱ������ƫ��(����)
    float P;                    //Ԥ�����
    float B;
    float u;
    //    z;                    //�۲�ֵ
}Kalman_Filter_Info;

/******************************************************************************/
/*----------------------------------ȫ�ֺ���-----------------------------------*/
/******************************************************************************/
/**
 * @brief  ƽ��ֵ������
 * @param  average_queue:    ƽ������������
 * @parma  curr_data:        ��ǰ��Ҫ�����˵���ֵ
 * @return None
 * @note   ��ֵ��ƽ��
 */
float filter_average_filter(Circular_Queue_Filter* average_queue,float curr_data);
/**
 * @brief  ��ֵ�˲���
 * @param  middle_queue:    ��ֵ����������
 * @parma  curr_data:       ��ǰ��Ҫ�����˵���ֵ
 * @return None
 * @note   ȥ�����ֵ����Сֵ����ƽ��
 */
float middle_filter(Circular_Queue_Filter* middle_queue,float curr_data);
/**
 * @brief  һ���˲���
 * @param  info:        һ���˲��ṹ��
 * @parma  curr_data:    ��ǰ��Ҫ�����˵���ֵ
 * @return None
 * @note   ʹ����������������
 */
float first_order_filter(First_Order_Filter_Info* info,float curr_data);


/**
 * @brief  �������˲�����ʼ��
 * @param  info:         �������˲��ṹ��
 * @parma  curr_data:    ��ǰ��Ҫ�����˵���ֵ
 * @return None
 */
void kalman_filter_init(Kalman_Filter_Info* info);

/**
 * @brief  �������˲���
 * @param  info:         �������˲��ṹ��
 * @parma  curr_data:    ��ǰ��Ҫ�����˵���ֵ
 * @return None
 * @todo   �˽⿨�����˲��ĸ�����������
 */
float kalman_filter(Kalman_Filter_Info* info, float curr_data);

/** \} */
#endif /* CODE_ALGORITHM_RRD_FILTER_RRD_H_ */
