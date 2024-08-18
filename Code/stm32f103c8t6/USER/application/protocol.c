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
static const uint8_t g_PACKET_HEADER[g_PACKET_HEADER_LEN]={0xAA,0xAB};    							// �������ݰ�ͷ

#define g_PACKET_TAIL_LEN    (2)
static const uint8_t g_PACKET_TAIL[g_PACKET_TAIL_LEN]={0xFA,0xFB};        							// �������ݰ�β

#define g_SIMPLY_PACKET_HEADER_LEN    (2)
static const uint8_t g_SIMPLY_PACKET_HEADER[g_SIMPLY_PACKET_HEADER_LEN]={0xBA,0xBB};    // �������ݰ�ͷ

#define g_SIMPLY_PACKET_TAIL_LEN    (2)
static const uint8_t g_SIMPLY_PACKET_TAIL[g_SIMPLY_PACKET_TAIL_LEN]={0xEA,0xEB};        // �������ݰ�β


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
	g_BALANCE_CAR_INFO.pid_control.open_loop = data->config_params.need_open_loop_control? TRUE :
																																													FALSE;
	
	// ������
	pic_loc_clear_error(&g_BALANCE_CAR_INFO.pid_data.balance_pid);
	pic_loc_clear_error(&g_BALANCE_CAR_INFO.pid_data.speed_pid);
	pic_loc_clear_error(&g_BALANCE_CAR_INFO.pid_data.turn_pid);
	
	g_BALANCE_CAR_INFO.pid_data.balance_pid.Kp = data->config_params.balance_pid[0];	// ֱ���� P D
	g_BALANCE_CAR_INFO.pid_data.balance_pid.Td = data->config_params.balance_pid[1];	
	
	g_BALANCE_CAR_INFO.pid_data.speed_pid.Kp = data->config_params.speed_pid[0];	// �ٶȻ� P I
	g_BALANCE_CAR_INFO.pid_data.speed_pid.Ti = data->config_params.speed_pid[1];
	
	g_BALANCE_CAR_INFO.pid_data.turn_pid.Kp = data->config_params.turn_pid[0]; 	// ת�� P D 
	g_BALANCE_CAR_INFO.pid_data.turn_pid.Td = data->config_params.turn_pid[1];
	
	g_BALANCE_CAR_INFO.pid_data.balance_pid.desired_value = data->config_params.desired_pitch_angle;	// ��е��ֵ
	
	g_BALANCE_CAR_INFO.encoder_date.encoder_left_filter.k = data->config_params.encoder_l_filter[0];	// ��������˲������� һ���˲� K
	g_BALANCE_CAR_INFO.encoder_date.encoder_right_filter.k = data->config_params.encoder_r_filter[0];	// �ұ������˲������� һ���˲� K

	g_BALANCE_CAR_INFO.function_option.need_dmp_reset = data->config_params.function_options[0];

	if(g_BALANCE_CAR_INFO.function_option.need_oled_show != data->config_params.function_options[1]){
		g_BALANCE_CAR_INFO.function_option.need_oled_clear = TRUE;
		g_BALANCE_CAR_INFO.function_option.need_oled_show = data->config_params.function_options[1];
	}

	
	
	debug_uart_send_2(1,"P: %0.2f D:%0.2f T: %0.2f \r\n",g_BALANCE_CAR_INFO.pid_data.balance_pid.Kp,
																												g_BALANCE_CAR_INFO.pid_data.balance_pid.Td,
																												g_BALANCE_CAR_INFO.pid_data.balance_pid.Tsam);
	
	debug_uart_send_2(1,"P: %0.2f D: %0.2f T: %0.2f \r\n",g_BALANCE_CAR_INFO.pid_data.speed_pid.Kp,
																												g_BALANCE_CAR_INFO.pid_data.speed_pid.Td,
																												g_BALANCE_CAR_INFO.pid_data.speed_pid.Tsam);

	debug_uart_send_2(1,"P: %0.2f D: %0.2f T: %0.2f \r\n",g_BALANCE_CAR_INFO.pid_data.turn_pid.Kp,
																												g_BALANCE_CAR_INFO.pid_data.turn_pid.Td,
																												g_BALANCE_CAR_INFO.pid_data.turn_pid.Tsam);
	
}


/**
 * @brief  һ�����ÿ���״̬�ķ����� ת�� �˶����� �ٶ�
 * @param  data ���ݰ�
 * @return None
 */
void set_movemont_params_handle(const Receive_Data* data)
{
	debug_uart_send_2(1,"set_movemont_params_handle \r\n");
	
	g_BALANCE_CAR_INFO.pid_data.turn_pid.desired_value = data->movement_params.turn_angle;
	
	// ����ʽpidʱ���������ٶ�
	if(g_BALANCE_CAR_INFO.pid_control.open_loop){
		g_BALANCE_CAR_INFO.pid_open_loop_data.motor_left_pwm_reload_value = (int16_t)data->movement_params.move_speed[0];
		g_BALANCE_CAR_INFO.pid_open_loop_data.motor_right_pwm_reload_value = (int16_t)data->movement_params.move_speed[1];
	}
	else{
		// speed_pid(����ʽpid) Ŀ���ٶ� 
		// g_BALANCE_CAR_INFO.pid_data.speed_pid.desired_value = data->movement_params.move_speed[0];
		
		// speed_pid(λ��ʽpid) Ŀ���ƶ�����
		g_BALANCE_CAR_INFO.pid_data.speed_pid.desired_value = (uint16_t)data->movement_params.move_speed[1];
	}

	
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
				
        default:
        case none_server:
            break;
    }
}

/******************************************************************************/
/*----------------------------------��������----------------------------------*/
/******************************************************************************/
/** \addtogroup ���У�����ݰ�����
 *  \{ */
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
					
						if(server){
						
						}
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
/** \group ���У�����ݰ�����
 **  \} */



/** \addtogroup �����ݰ�����
 *  \{ */
/**
 * @brief      	�����ݰ�����
 * @param[in]  	data ��������ַ
 * @param[in]  	start_index ���ݰ���ʼƫ����
 * @param[in]  	end_index ���ݰ�ĩβƫ����
 * @return 			None
 * @note				Ĭ�ϵ�����λ���Ƿ������ѡ��
 */
void parse_packet_simple(uint8_t* data,uint8_t start_index,uint8_t end_index){		
		// 1.У�鳤��
		if(end_index - start_index <= 4){
			debug_uart_send_2(0,"simple pack len: %d \r\n",end_index - start_index);
			return;
		}
		
		// 2.У���ͷ
		if(!(data[start_index] == g_SIMPLY_PACKET_HEADER[0]
			&& data[start_index + 1] == g_SIMPLY_PACKET_HEADER[1])){
			debug_uart_send_2(0,"simple pack header: %x  %x \r\n",data[start_index],data[start_index + 1]);
			return;
		}
		
		// 3.У���β
		if(!(data[end_index] != g_SIMPLY_PACKET_TAIL[1]
			&& data[end_index - 1] != g_SIMPLY_PACKET_TAIL[0])){
			debug_uart_send_2(0,"simple pack tail: %x  %x \r\n",data[end_index - 1],data[end_index]);
			return;
		}
			
		// 4.ȷ�Ϸ���
		if(!check_packet_server_type(data[start_index + 2])){
			debug_uart_send_2(0,"simple pack server: %x \r\n",data[start_index + 2]);
			return;
		}
		
		// 5.ת������
		uint8_t pack_lenth = end_index - start_index - 5;
		for(uint8_t index = 0;index < pack_lenth;index++){
			g_RX_DATA.rx_data_buff[index] = data[start_index + 3 + index];
			debug_uart_send_2(1,"%x ",g_RX_DATA.rx_data_buff[index]);
		}
		
		// 6.ִ�з���
		Server_Type server_simple_packet = (Server_Type)data[start_index + 2];
		select_server_handler(server_simple_packet);
		debug_uart_send_2(0,"simple pack parse success \r\n");
}

/** \group �����ݰ�����
 **  \} */



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




















