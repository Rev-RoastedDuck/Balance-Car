/*
 * protocol.c
 *
 *  Created on: 2024��2��14��
 *      Author: intl4419
 */

#include "protocol.h"

/******************************************************************************/
/*----------------------------------���ò���-----------------------------------*/
/******************************************************************************/
#define g_PACKET_HEADER_LEN    (2)
static const uint8_t g_PACKET_HEADER[g_PACKET_HEADER_LEN]={0xAA,0xAB};    // �������ݰ�ͷ

#define g_PACKET_TAIL_LEN    (2)
static const uint8_t g_PACKET_TAIL[g_PACKET_TAIL_LEN]={0xFA,0xFB};        // �������ݰ�β


/******************************************************************************/
/*----------------------------------ȫ�ֱ���-----------------------------------*/
/******************************************************************************/
static Receive_Data  g_RX_DATA;
#if Only_Receive_Data
static Transmit_Data g_TX_DATA;
#endif

/******************************************************************************/
/*----------------------------------��������-----------------------------------*/
/******************************************************************************/
uint8_t buile_packet(const uint8_t *buff, uint8_t len,Server_Type server,uint8_t *packet);


/******************************************************************************/
/*------------------------------------�ӿ�-------------------------------------*/
/******************************************************************************/
/**
 * @brief  һ���������ݵĽӿ�
 * @param  data ���ݰ�����
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
/*----------------------------------������-----------------------------------*/
/******************************************************************************/
/**
 * @brief  һ�����ò����ķ����� ֱ���� �ٶȻ� ת��
 * @param  data ���ݰ�
 * @return None
 */
void set_config_params_handler(const Receive_Data* data)
{

}


/**
 * @brief  һ�����ÿ���״̬�ķ����� ת�� �˶����� �ٶ�
 * @param  data ���ݰ�
 * @return None
 */
void set_movemont_params_handle(const Receive_Data* data)
{

}


/**
 * @brief  һ�������ֱ����ݵķ�����
 * @param  data ���ݰ�
 * @return None
 */
void parse_handle_params_handle(const Receive_Data* data)
{

}

/**
 * @brief  ѡ�������
 * @param  ���ݰ�
 * @return None
 * @todo   �ĳɻص�������ʽ
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
/*----------------------------------У�麯��-----------------------------------*/
/******************************************************************************/

/**
 * @brief  У������ͷ
 * @param  ���ݰ�
 * @return None
 */
boolean check_packet_header(uint8_t data)
{
    static uint8_t is_header_count = 0;         // ��¼����ͷ(0xAA 0xAB)У��ͨ������

    // 1.�ж��±�λis_header_count�����ݰ�ͷ�ǲ��Ƕ�Ӧ�����ݰ�ͷ�������һ����������
    is_header_count = g_PACKET_HEADER[is_header_count]==data?++is_header_count:0;

    // 2.���У�����λ���ݰ�ͷ���鳤�ȣ��򷵻�TRUE
    if(is_header_count==g_PACKET_HEADER_LEN)
    {
        is_header_count = 0;
        return TRUE;
    }

    // 3.û��У����ɣ�����FALSE
    return FALSE;
}

/**
 * @brief  У������β
 * @param  ���ݰ�
 * @return None
 */
boolean check_packet_tail(uint8_t data)
{
    static uint8_t is_tail_count = 0;         // ��¼����β(0xFA 0xFB)У��ͨ������

    // 1.�ж��±�λis_header_count�����ݰ�ͷ�ǲ��Ƕ�Ӧ�����ݰ�ͷ�������һ����������
    is_tail_count = g_PACKET_TAIL[is_tail_count]==data?++is_tail_count:0;
//    printf("is_tail_count:%d",is_tail_count);
    // 2.���У�����λ���ݰ�ͷ���鳤�ȣ��򷵻�TRUE
    if(is_tail_count==g_PACKET_TAIL_LEN)
    {
        is_tail_count = 0;
        return TRUE;
    }


    // 3.û��У����ɣ�����FALSE
    return FALSE;
}

/**
 * @brief  У�����ݳ���
 * @param  ���ݰ�
 * @return None
 */
boolean check_data_length(uint8_t len,uint8_t desire_len)
{
    return len==desire_len?TRUE:FALSE;
}


/**
 * @brief  �������ݰ���С
 * @param  ���ݰ�
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
 * @brief  У�����ݰ���С
 * @param  ���ݰ�
 * @return None
 * @note   ��uint8���㳤��
 */
boolean check_packet_size(uint8_t*data_list,uint8_t len,uint8_t desire_size)
{
    uint8_t calcu_size = cacul_packet_size(data_list,len);
//    printf("calcu_size:%d\r\n",calcu_size);
//    printf("desire_size:%d\r\n",desire_size);

    // 2.�жϼ������Ƿ����Ԥ�ڴ�С
    if(calcu_size==desire_size){
        return TRUE;
    }
    return FALSE;
}

/**
 * @brief  �ж��ǲ��Ƿ���ö�����е�����һ��
 * @param  ���ݰ�
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
/*------------------------------------����------------------------------------*/
/******************************************************************************/

/**
 * @brief  �������ݰ�
 * @param  data  һ�ֽ�����
 * @return None
 * @todo
 */

static uint8_t step = 0;
static uint8_t data_pack_len = 0;
static uint8_t data_pack_size = 0;        // ��uint8û������
static uint8_t rx_data_buff_index = 0;
static Server_Type server = (Server_Type)0;

void parse_packet(uint8_t data)
{
    // 0. У�����ݰ�ͷ
    if (0 == step)
    {
        server = (Server_Type)0;
        data_pack_len = 0;
        data_pack_size = 0;
        rx_data_buff_index = 0;
        step = check_packet_header(data) ? ++step : 0;
        return;
    }

    // 1. �������ݰ�����
    if (1 == step)
    {
        step++;
        data_pack_len = data;
        return;
    }

    // 2. �������ݰ���С
    if (2 == step)
    {
        step++;
        data_pack_size = data;
        return;
    }

    // 3.�������ݰ���������
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

    // 4.����������
    if (4 == step)
    {
        // ��һ����������
        if (data != g_PACKET_TAIL[1])
        {
            g_RX_DATA.rx_data_buff[rx_data_buff_index++] = data;
            return;
        }
        step++;
        rx_data_buff_index--;                               // ����⵽��һ�����ݰ�βʱ��
                                                            // rx_data_buff_index���һ������������Ҫ����.
    }
    // 5.У�����ݳ���
    if (5 == step)
    {
        // ������Ҳ���������һ���֣���rx_data_buffδ����,����ȥ��server��λ��
        if (!check_data_length(rx_data_buff_index, data_pack_len-1))
        {
            step = 0;
            return;
        }
        step++;
    }

    // 6.У�����ݰ���С
    if (6 == step)
    {
        // ������Ҳ���������һ���֣���rx_data_buffδ����������ȥ��server�Ĵ�С
        if (!check_packet_size(g_RX_DATA.rx_data_buff, rx_data_buff_index, data_pack_size-server))
        {
            step = 0;
            return;
        }
        step++;
    }

    // 7.ִ�з�����
    if (7 == step)
    {
        select_server_handler(server);
    }

    step = 0;
}


/******************************************************************************/
/*------------------------------------����------------------------------------*/
/******************************************************************************/
/**
 * @brief  �������ݰ�
 * @param  buff     ���ݰ�����
 * @param  len      ���ݰ����峤��
 * @param  server   ��������
 * @param  server   ��������
 * @return packet   ���������ݰ�
 * @todo
 */
uint8_t buile_packet(const uint8_t *buff, uint8_t len,Server_Type server,uint8_t *packet){
    uint8_t index = 0;

    // 1.������ݰ�ͷ
    for(uint8_t i=0;i<g_PACKET_HEADER_LEN;i++)
    {
        packet[index++] = g_PACKET_HEADER[i];
    }

    // 2.������ݳ���,���Ϲ��ܺ�
    packet[index++] = len + 1;

    // 3.������ݴ�С�����Ϲ��ܺŵĴ�С
    packet[index++] = (uint8_t)(cacul_packet_size(buff,len) + server);

    // 4.��书��
    packet[index++] = server;

    // 5.���������
    for(uint8_t i=0;i<len;i++)
    {
        packet[index++] = buff[i];
    }

    // 6.�������β
    for(uint8_t i=0;i<g_PACKET_TAIL_LEN;i++)
    {
        packet[index++] = g_PACKET_TAIL[i];
    }

    return index;
}


/**
 * @brief  �������ݰ�
 * @param  server   ��������
 * @return None
 * @todo
 */
void transmit_data(Server_Type server)
{
    select_server_handler(server);
}




















