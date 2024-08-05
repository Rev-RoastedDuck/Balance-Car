/*
 * filter_rrd.c
 *
 *  Created on: 2024��1��29��
 *      Author: intl4419
 */
#include "filter_rrd.h"


/** \addtogroup ����
 * \{ */
/******************************************************************************/
/*----------------------------------��̬����-----------------------------------*/
/******************************************************************************/
/**
 * @brief  �ҳ���������ֵ����Сֵ
 * @param  arr:      ����
 * @param  size:     ���鳤��
 * @return min_val:  �����Сֵ�����ĵ�ַ
 * @return max_val:  ������ֵ�����ĵ�ַ
 */
void __list_find_min_max(float *arr, uint8_t size, float* min_val, float* max_val)
{
    *min_val = arr[0];
    *max_val = arr[0];

    for (uint8_t i = 1; i < size; i++) {
        if (arr[i] < *min_val) {
            *min_val = arr[i];
        }
        if (arr[i] > *max_val) {
            *max_val = arr[i];
        }
    }
}
/** \} */






/** \addtogroup ����
 * Note:    2024/06/20 ����ֻ����Ϊ�������ݵ��������ƺ�������Ҫ����
 * \{ */
/******************************************************************************/
/*----------------------------------ȫ�ֺ���-----------------------------------*/
/******************************************************************************/
/**
 * @brief  ���
 * @param  queue:    ����
 * @parma  value:    ��ӵ���ֵ
 * @return None
 */
void queue_enqueue(Circular_Queue_Filter* queue, float value) {
    queue->data[queue->tail] = value;

    if (queue->count != QUEUE_SIZE) {
        queue->count += 1;
    }
    queue->tail = (queue->tail + 1) % QUEUE_SIZE;
    queue->head = queue->count > QUEUE_SIZE ? queue->tail : 0;
}

/**
 * @brief  ��ӣ�ͬʱ��������ܺ�
 * @param  queue:    ����
 * @parma  value:    ��ӵ���ֵ
 * @return None
 */
void queue_enqueue_sum(Circular_Queue_Filter* queue, float value) {
    queue->sum -= queue->data[queue->tail];
    queue->sum += value;

    queue->data[queue->tail] = value;

    if (queue->count != QUEUE_SIZE) {
        queue->count += 1;
    }
    queue->tail = (queue->tail + 1) % QUEUE_SIZE;
    queue->head = queue->count > QUEUE_SIZE ? queue->tail : 0;
}
/** \} */





/** \addtogroup ������
 * \{ */
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
float filter_average_filter(Circular_Queue_Filter* average_queue,float curr_data) {
    queue_enqueue_sum(average_queue, curr_data);
    return(average_queue->sum / average_queue->count);
}

/**
 * @brief  ��ֵ�˲���
 * @param  middle_queue:    ��ֵ����������
 * @parma  curr_data:       ��ǰ��Ҫ�����˵���ֵ
 * @return None
 * @note   ȥ�����ֵ����Сֵ����ƽ��
 */
float middle_filter(Circular_Queue_Filter* middle_queue,float curr_data) {
    queue_enqueue_sum(middle_queue,curr_data);

    // ������ֵ����С��2ʱ���˲�������ƽ��ֵ
    if (middle_queue->count <= 2) {
        return (middle_queue->sum) / (middle_queue->count);
    }

    float max_value = 0;
    float min_value = 0;
    __list_find_min_max(middle_queue->data, middle_queue->count, &max_value, &min_value);
    return (middle_queue->sum - max_value - min_value) / (middle_queue->count - 2);
}

/**
 * @brief  һ���˲���
 * @param  info:        һ���˲��ṹ��
 * @parma  curr_data:    ��ǰ��Ҫ�����˵���ֵ
 * @return None
 * @note   ���µĲ���ֵ���ϴε��˲��������һ����Ȩƽ��ֵ
 */
float first_order_filter(First_Order_Filter_Info* info,float curr_data) {
    // ������һ���˲�
    if (info->is_first) {
        info->is_first = 0;
        info->last_value = curr_data;
        return info->last_value;
    }

    info->last_value = info->k * curr_data + (1 - info->k) * info->last_value;
    return info->last_value;
}

/**
 * @brief  �������˲���
 * @param  info:         �������˲��ṹ��
 * @parma  curr_data:    ��ǰ��Ҫ�����˵���ֵ
 * @return info->filterValue: �˲����ֵ
 * @todo   �˽⿨�����˲��ĸ�����������
 */
float kalman_filter(Kalman_Filter_Info* info, float curr_data) {
    /*
    float predictValue = info->A * info->filterValue + info->B * info->u;             // ����Ԥ��ֵ
    info->P = info->A * info->A * info->P + info->Q;                                    // ��Э����
    info->kalmanGain = info->P * info->H / (info->P * info->H * info->H + info->R);     // ���㿨��������
    info->filterValue = predictValue + (curr_data - predictValue) * info->kalmanGain;   // ���������ֵ
    info->P = (1 - info->kalmanGain * info->H) * info->P;                               // ����Э����
    return info->filterValue;
    */

    float predictValue = info->A * info->filterValue + info->B * info->u;             // ����Ԥ��ֵ
    info->P = info->A * info->A * info->P + info->Q;                                    // ��Э�����Ԥ��ģ�͵Ĳ�ȷ����
    float P_H_temp = info->P * info->H;                                               // P*H,�����м�������򻯼���
    info->kalmanGain = P_H_temp / (P_H_temp * info->H + info->R);                       // ���㿨��������
    info->filterValue = predictValue + (curr_data - predictValue) * info->kalmanGain;   // ���������ֵ
    info->P = info->P - info->kalmanGain * P_H_temp;                                    // ����Э����

    return info->filterValue;
}

/**
 * @brief  �������˲�����ʼ��
 * @param  info:         �������˲��ṹ��
 * @return None
 */
void kalman_filter_init(Kalman_Filter_Info* info) {
    info->A = 1;
    info->H = 1;

    info->P = 0.07;
    info->Q = 0.04;
    info->R = 0.8;

    info->B = 0;
    info->u = 0;
    info->filterValue = 0;
}
/** \} */
