/*
 * protocol.c
 *
 *  Created on: 2024年2月14日
 *      Author: intl4419
 */

#include "protocol.h"

/******************************************************************************/
/*----------------------------------配置参数-----------------------------------*/
/******************************************************************************/
#define g_PACKET_HEADER_LEN    (2)
static const uint8_t g_PACKET_HEADER[g_PACKET_HEADER_LEN]={0xAA,0xAB};    // 定义数据包头

#define g_PACKET_TAIL_LEN    (2)
static const uint8_t g_PACKET_TAIL[g_PACKET_TAIL_LEN]={0xFA,0xFB};        // 定义数据包尾


/******************************************************************************/
/*----------------------------------全局变量-----------------------------------*/
/******************************************************************************/
static Receive_Data  g_RX_DATA;
#if Only_Receive_Data
static Transmit_Data g_TX_DATA;
#endif

/******************************************************************************/
/*----------------------------------函数声明-----------------------------------*/
/******************************************************************************/
uint8_t buile_packet(const uint8_t *buff, uint8_t len,Server_Type server,uint8_t *packet);


/******************************************************************************/
/*------------------------------------接口-------------------------------------*/
/******************************************************************************/
/**
 * @brief  一个发送数据的接口
 * @param  data 数据包主体
 * @return None
 */
void transmit_interface(const uint8_t *buff, uint8_t len,Server_Type server)
{
//    uint8_t packet[60];
//    uint8_t packet_len = 0;

//    packet_len = buile_packet(buff,len,server,packet);

//    printf("data: ");
//    for(uint8 index = 0;index < packet_len;index++){
//        printf("%d",packet[index]);
//        printf(" ");
//    }
//    printf("\r\n");

    // eg.
    // bluetooth_send_buffer_rrd(packet,packet_len);
//     uart_write_buffer(DEBUG_UART_INDEX, packet, packet_len);
}


/******************************************************************************/
/*----------------------------------服务函数-----------------------------------*/
/******************************************************************************/
/**
 * @brief  一个设置参数的服务函数 直立环 速度环 转向环
 * @param  data 数据包
 * @return None
 */
void set_config_params_handler(const Receive_Data* data)
{

}


/**
 * @brief  一个设置控制状态的服务函数 转向 运动距离 速度
 * @param  data 数据包
 * @return None
 */
void set_movemont_params_handle(const Receive_Data* data)
{

}


/**
 * @brief  一个解析手柄数据的服务函数
 * @param  data 数据包
 * @return None
 */
void parse_handle_params_handle(const Receive_Data* data)
{

}

/**
 * @brief  选择服务函数
 * @param  数据包
 * @return None
 * @todo   改成回调函数形式
 */
void select_server_handler(Server_Type server)
{
    switch(server)
    {		
				case set_config_params:
					set_config_params_handler(&g_RX_DATA);
					break;
				case set_movement_params:
					set_movemont_params_handle(&g_RX_DATA);
					break;
				case parse_handle_params:
					parse_handle_params_handle(&g_RX_DATA);
					break;
        case none_server:
            break;
        default:
            break;
    }
}

/******************************************************************************/
/*----------------------------------校验函数-----------------------------------*/
/******************************************************************************/

/**
 * @brief  校验数据头
 * @param  数据包
 * @return None
 */
boolean check_packet_header(uint8_t data)
{
    static uint8_t is_header_count = 0;         // 记录数据头(0xAA 0xAB)校验通过次数

    // 1.判断下标位is_header_count的数据包头是不是对应的数据包头，是则加一，否则清零
    is_header_count = g_PACKET_HEADER[is_header_count]==data?++is_header_count:0;

    // 2.如果校验次数位数据包头数组长度，则返回TRUE
    if(is_header_count==g_PACKET_HEADER_LEN)
    {
        is_header_count = 0;
        return TRUE;
    }

    // 3.没有校验完成，返回FALSE
    return FALSE;
}

/**
 * @brief  校验数据尾
 * @param  数据包
 * @return None
 */
boolean check_packet_tail(uint8_t data)
{
    static uint8_t is_tail_count = 0;         // 记录数据尾(0xFA 0xFB)校验通过次数

    // 1.判断下标位is_header_count的数据包头是不是对应的数据包头，是则加一，否则清零
    is_tail_count = g_PACKET_TAIL[is_tail_count]==data?++is_tail_count:0;
//    printf("is_tail_count:%d",is_tail_count);
    // 2.如果校验次数位数据包头数组长度，则返回TRUE
    if(is_tail_count==g_PACKET_TAIL_LEN)
    {
        is_tail_count = 0;
        return TRUE;
    }


    // 3.没有校验完成，返回FALSE
    return FALSE;
}

/**
 * @brief  校验数据长度
 * @param  数据包
 * @return None
 */
boolean check_data_length(uint8_t len,uint8_t desire_len)
{
    return len==desire_len?TRUE:FALSE;
}


/**
 * @brief  计算数据包大小
 * @param  数据包
 * @return None
 */
uint8_t cacul_packet_size(const uint8_t*data_list,uint8_t len)
{
    uint8_t index = 0;
    uint8_t calcu_size = 0;

    for(index = 0;index < len;index++){
        calcu_size += data_list[index];
    }
    return calcu_size;
}

/**
 * @brief  校验数据包大小
 * @param  数据包
 * @return None
 * @note   用uint8计算长度
 */
boolean check_packet_size(uint8_t*data_list,uint8_t len,uint8_t desire_size)
{
    uint8_t calcu_size = cacul_packet_size(data_list,len);
//    printf("calcu_size:%d\r\n",calcu_size);
//    printf("desire_size:%d\r\n",desire_size);

    // 2.判断计算结果是否等于预期大小
    if(calcu_size==desire_size){
        return TRUE;
    }
    return FALSE;
}

/**
 * @brief  判断是不是服务枚举类中的其中一个
 * @param  数据包
 * @return None
 */
boolean check_packet_server_type(uint8_t data)
{
    switch (data) {
        case set_config_params:
        case set_movement_params:
				case parse_handle_params:
            return TRUE;
        default:
            return FALSE;
    }
}

/******************************************************************************/
/*------------------------------------接收------------------------------------*/
/******************************************************************************/

/**
 * @brief  解析数据包
 * @param  data  一字节数据
 * @return None
 * @todo
 */

static uint8_t step = 0;
static uint8_t data_pack_len = 0;
static uint8_t data_pack_size = 0;        // 用uint8没有问题
static uint8_t rx_data_buff_index = 0;
static Server_Type server = (Server_Type)0;

void parse_packet(uint8_t data)
{
    // 0. 校验数据包头
    if (0 == step)
    {
        server = (Server_Type)0;
        data_pack_len = 0;
        data_pack_size = 0;
        rx_data_buff_index = 0;
        step = check_packet_header(data) ? ++step : 0;
        return;
    }

    // 1. 保存数据包长度
    if (1 == step)
    {
        step++;
        data_pack_len = data;
        return;
    }

    // 2. 保存数据包大小
    if (2 == step)
    {
        step++;
        data_pack_size = data;
        return;
    }

    // 3.保存数据包服务类型
    if (3 == step)
    {
        if (check_packet_server_type(data))
        {
            step++;
            server = (Server_Type)data;
        }else{
            step = 0;
        }
        return;
    }

    // 4.保存数据体
    if (4 == step)
    {
        // 加一个长度限制
        if (data != g_PACKET_TAIL[1])
        {
            g_RX_DATA.rx_data_buff[rx_data_buff_index++] = data;
            return;
        }
        step++;
        rx_data_buff_index--;                               // 当检测到第一个数据包尾时，
                                                            // rx_data_buff_index会加一，所以这里需要减掉.
    }
    // 5.校验数据长度
    if (5 == step)
    {
        // 服务编号也是数据体的一部分，但rx_data_buff未包含,所以去掉server的位数
        if (!check_data_length(rx_data_buff_index, data_pack_len-1))
        {
            step = 0;
            return;
        }
        step++;
    }

    // 6.校验数据包大小
    if (6 == step)
    {
        // 服务编号也是数据体的一部分，但rx_data_buff未包含，所以去掉server的大小
        if (!check_packet_size(g_RX_DATA.rx_data_buff, rx_data_buff_index, data_pack_size-server))
        {
            step = 0;
            return;
        }
        step++;
    }

    // 7.执行服务函数
    if (7 == step)
    {
        select_server_handler(server);
    }

    step = 0;
}


/******************************************************************************/
/*------------------------------------发送------------------------------------*/
/******************************************************************************/
/**
 * @brief  构建数据包
 * @param  buff     数据包主体
 * @param  len      数据包主体长度
 * @param  server   服务类型
 * @param  server   服务类型
 * @return packet   完整的数据包
 * @todo
 */
uint8_t buile_packet(const uint8_t *buff, uint8_t len,Server_Type server,uint8_t *packet){
    uint8_t index = 0;

    // 1.填充数据包头
    for(uint8_t i=0;i<g_PACKET_HEADER_LEN;i++)
    {
        packet[index++] = g_PACKET_HEADER[i];
    }

    // 2.填充数据长度,加上功能号
    packet[index++] = len + 1;

    // 3.填充数据大小，加上功能号的大小
    packet[index++] = (uint8_t)(cacul_packet_size(buff,len) + server);

    // 4.填充功能
    packet[index++] = server;

    // 5.填充数据体
    for(uint8_t i=0;i<len;i++)
    {
        packet[index++] = buff[i];
    }

    // 6.填充数据尾
    for(uint8_t i=0;i<g_PACKET_TAIL_LEN;i++)
    {
        packet[index++] = g_PACKET_TAIL[i];
    }

    return index;
}


/**
 * @brief  发送数据包
 * @param  server   服务类型
 * @return None
 * @todo
 */
void transmit_data(Server_Type server)
{
    select_server_handler(server);
}




















