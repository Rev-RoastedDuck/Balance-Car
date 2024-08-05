/*
 * filter_rrd.c
 *
 *  Created on: 2024年1月29日
 *      Author: intl4419
 */
#include "filter_rrd.h"


/** \addtogroup 数组
 * \{ */
/******************************************************************************/
/*----------------------------------静态函数-----------------------------------*/
/******************************************************************************/
/**
 * @brief  找出数组的最大值和最小值
 * @param  arr:      数组
 * @param  size:     数组长度
 * @return min_val:  存放最小值变量的地址
 * @return max_val:  存放最大值变量的地址
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






/** \addtogroup 队列
 * Note:    2024/06/20 这里只是作为储存数据的容器，似乎并不需要队列
 * \{ */
/******************************************************************************/
/*----------------------------------全局函数-----------------------------------*/
/******************************************************************************/
/**
 * @brief  入队
 * @param  queue:    队列
 * @parma  value:    入队的数值
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
 * @brief  入队，同时计算队列总和
 * @param  queue:    队列
 * @parma  value:    入队的数值
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





/** \addtogroup 过滤器
 * \{ */
/******************************************************************************/
/*----------------------------------全局函数-----------------------------------*/
/******************************************************************************/

/**
 * @brief  平均值过滤器
 * @param  average_queue:    平均过滤器队列
 * @parma  curr_data:        当前需要被过滤的数值
 * @return None
 * @note   多值求平均
 */
float filter_average_filter(Circular_Queue_Filter* average_queue,float curr_data) {
    queue_enqueue_sum(average_queue, curr_data);
    return(average_queue->sum / average_queue->count);
}

/**
 * @brief  中值滤波器
 * @param  middle_queue:    中值过滤器队列
 * @parma  curr_data:       当前需要被过滤的数值
 * @return None
 * @note   去掉最大值和最小值，求平均
 */
float middle_filter(Circular_Queue_Filter* middle_queue,float curr_data) {
    queue_enqueue_sum(middle_queue,curr_data);

    // 队列数值个数小于2时不滤波，返回平均值
    if (middle_queue->count <= 2) {
        return (middle_queue->sum) / (middle_queue->count);
    }

    float max_value = 0;
    float min_value = 0;
    __list_find_min_max(middle_queue->data, middle_queue->count, &max_value, &min_value);
    return (middle_queue->sum - max_value - min_value) / (middle_queue->count - 2);
}

/**
 * @brief  一阶滤波器
 * @param  info:        一阶滤波结构体
 * @parma  curr_data:    当前需要被过滤的数值
 * @return None
 * @note   将新的采样值与上次的滤波结果计算一个加权平均值
 */
float first_order_filter(First_Order_Filter_Info* info,float curr_data) {
    // 跳过第一次滤波
    if (info->is_first) {
        info->is_first = 0;
        info->last_value = curr_data;
        return info->last_value;
    }

    info->last_value = info->k * curr_data + (1 - info->k) * info->last_value;
    return info->last_value;
}

/**
 * @brief  卡尔曼滤波器
 * @param  info:         卡尔曼滤波结构体
 * @parma  curr_data:    当前需要被过滤的数值
 * @return info->filterValue: 滤波后的值
 * @todo   了解卡尔曼滤波的各个参数含义
 */
float kalman_filter(Kalman_Filter_Info* info, float curr_data) {
    /*
    float predictValue = info->A * info->filterValue + info->B * info->u;             // 计算预测值
    info->P = info->A * info->A * info->P + info->Q;                                    // 求协方差
    info->kalmanGain = info->P * info->H / (info->P * info->H * info->H + info->R);     // 计算卡尔曼增益
    info->filterValue = predictValue + (curr_data - predictValue) * info->kalmanGain;   // 计算输出的值
    info->P = (1 - info->kalmanGain * info->H) * info->P;                               // 更新协方差
    return info->filterValue;
    */

    float predictValue = info->A * info->filterValue + info->B * info->u;             // 计算预测值
    info->P = info->A * info->A * info->P + info->Q;                                    // 求协方差，求预测模型的不确定性
    float P_H_temp = info->P * info->H;                                               // P*H,储存中间变量，简化计算
    info->kalmanGain = P_H_temp / (P_H_temp * info->H + info->R);                       // 计算卡尔曼增益
    info->filterValue = predictValue + (curr_data - predictValue) * info->kalmanGain;   // 计算输出的值
    info->P = info->P - info->kalmanGain * P_H_temp;                                    // 更新协方差

    return info->filterValue;
}

/**
 * @brief  卡尔曼滤波器初始化
 * @param  info:         卡尔曼滤波结构体
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
