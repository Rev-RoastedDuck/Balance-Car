/*
 * filter.h
 *
 *  Created on: 2024年1月29日
 *      Author: intl4419
 */

#ifndef ALGORITHM_FILTER_RRD_H_
#define ALGORITHM_FILTER_RRD_H_

/******************************************************************************/
/*---------------------------------头文件引入---------------------------------*/
/******************************************************************************/
#include "typedef.h"
#include "stm32f10x.h"


/******************************************************************************/
/*----------------------------------配置参数----------------------------------*/
/******************************************************************************/
#define QUEUE_SIZE 8


/** \addtogroup 队列
 * \{ */

/******************************************************************************/
/*-----------------------------------结构体------------------------------------*/
/******************************************************************************/

/**
 * @brief  队列结构体
 */
typedef struct {
    uint8_t   head;               // 队列头部指针
    uint8_t   tail;               // 队列尾部指针，指向下一个要被修改的位置，而不是当前最新修改的位置，当前最新修改的位置值为tail-1
    uint8_t   count;              // 队列中已添加元素的数量
    float sum;                // 总和
    float data[QUEUE_SIZE];   // 数据
} Circular_Queue_Filter;

/******************************************************************************/
/*----------------------------------全局函数-----------------------------------*/
/******************************************************************************/
/**
 * @brief  入队
 * @param  queue:    队列
 * @parma  value:    入队的数值
 * @return None
 */
void queue_enqueue(Circular_Queue_Filter* queue, float value);

/**
 * @brief  入队，同时计算队列总和
 * @param  queue:    队列
 * @parma  value:    入队的数值
 * @return None
 */
void queue_enqueue_sum(Circular_Queue_Filter* queue, float value);

/** \} */



/** \addtogroup 过滤器
 * \{ */

/******************************************************************************/
/*-----------------------------------结构体------------------------------------*/
/******************************************************************************/

/**
 * @brief  一阶滤波结构体
 * @note    a越大，新采集的值占的权重越大，算法越灵敏，但平顺性差；
 * @note    a越小，新采集的值占的权重越小，算法灵敏度差，但平顺性好。
 */
typedef struct {
    float k;                    // 范围: 0~1
    float last_value;           // 上一个数值
    uint8_t is_first;             // 判断是否是第一次滤波
} First_Order_Filter_Info;

/**
 * @brief  卡尔曼滤波结构体
 */
typedef struct {
    float filterValue;          //滤波后的值
    float kalmanGain;           //Kalamn增益，卡尔曼系数，1.权衡观察模型和预测模型
    float A;                    //状态矩阵，对应的是速度
    float H;                    //观测矩阵，表示观察值和实际值的线性关系
    float Q;                    //状态矩阵的方差，预测模型带来的偏差(噪声)
    float R;                    //观测矩阵的方差，观测时带来的偏差(噪声)
    float P;                    //预测误差
    float B;
    float u;
    //    z;                    //观测值
}Kalman_Filter_Info;

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
float filter_average_filter(Circular_Queue_Filter* average_queue,float curr_data);
/**
 * @brief  中值滤波器
 * @param  middle_queue:    中值过滤器队列
 * @parma  curr_data:       当前需要被过滤的数值
 * @return None
 * @note   去掉最大值和最小值，求平均
 */
float middle_filter(Circular_Queue_Filter* middle_queue,float curr_data);
/**
 * @brief  一阶滤波器
 * @param  info:        一阶滤波结构体
 * @parma  curr_data:    当前需要被过滤的数值
 * @return None
 * @note   使用正比例函数过滤
 */
float first_order_filter(First_Order_Filter_Info* info,float curr_data);


/**
 * @brief  卡尔曼滤波器初始化
 * @param  info:         卡尔曼滤波结构体
 * @parma  curr_data:    当前需要被过滤的数值
 * @return None
 */
void kalman_filter_init(Kalman_Filter_Info* info);

/**
 * @brief  卡尔曼滤波器
 * @param  info:         卡尔曼滤波结构体
 * @parma  curr_data:    当前需要被过滤的数值
 * @return None
 * @todo   了解卡尔曼滤波的各个参数含义
 */
float kalman_filter(Kalman_Filter_Info* info, float curr_data);

/** \} */
#endif /* CODE_ALGORITHM_RRD_FILTER_RRD_H_ */
