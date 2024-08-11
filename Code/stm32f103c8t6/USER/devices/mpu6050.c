#include "mpu6050.h"
#include "mpu6050_map.h"


/******************************************************************************/
/*-----------------------------------�����-----------------------------------*/
/******************************************************************************/
static RRD_DRIVER_IIC g_MPU6050_IIC_DEVICE;

#define i2c_Stop()  					g_MPU6050_IIC_DEVICE.i2c_stop(&g_MPU6050_IIC_DEVICE)
#define i2c_Start() 					g_MPU6050_IIC_DEVICE.i2c_start(&g_MPU6050_IIC_DEVICE)
//#define i2c_WaitAck() 				g_MPU6050_IIC_DEVICE.i2c_wait_ack(&g_MPU6050_IIC_DEVICE)
#define i2c_ReadByte(ack) 		g_MPU6050_IIC_DEVICE.i2c_read_byte(&g_MPU6050_IIC_DEVICE,ack)
#define i2c_SendByte(_ucByte) g_MPU6050_IIC_DEVICE.i2c_send_byte(&g_MPU6050_IIC_DEVICE,_ucByte)

#define i2c_WaitAck() \
				do{ \
						if(g_MPU6050_IIC_DEVICE.i2c_wait_ack(&g_MPU6050_IIC_DEVICE)){	\
							g_MPU6050_IIC_DEVICE.i2c_stop(&g_MPU6050_IIC_DEVICE); \
							return 1; \
						} \
				}while(0)

/******************************************************************************/
/*-----------------------------------�ڲ�����---------------------------------*/
/******************************************************************************/
/**
 * @brief   		д���ݵ� MPU6050 �Ĵ���
 * @param[in]   reg_add Ŀ��Ĵ�����ַ
 * @param[in]   reg_dat Ҫд��Ĵ���������
 * @return  		0 �ɹ�  1 ʧ��
*/
static uint8_t mpu6050_write_reg(uint8_t reg_add,
															uint8_t reg_dat){
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);
	i2c_WaitAck();
	i2c_SendByte(reg_add);
	i2c_WaitAck();
	i2c_SendByte(reg_dat);
	i2c_WaitAck();
	i2c_Stop();
	return 0;
}

															
/******************************************************************************/
/*-----------------------------------�ӿ�ʵ��---------------------------------*/
/******************************************************************************/

/**
 * @brief   		��MPU6050�Ĵ�����ȡ����
 * @param[in]  reg_add 	Ŀ��Ĵ�����ַ
 * @param[in]  data_list	��Ŷ�ȡ���ݵĻ�����
 * @param[in]  num 			Ҫ��ȡ�������ֽ���
 * @return  	 0 �ɹ�  1 ʧ��
 */
static uint8_t mpu6050_read_data(const uint8_t reg_add,
																uint8_t* data_list,
																const uint8_t num){
	unsigned char i;
	
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);
	i2c_WaitAck();
	i2c_SendByte(reg_add);
	i2c_WaitAck();
	
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS+1);
	i2c_WaitAck();
	
	for(i=0;i<(num-1);i++){
		*data_list=i2c_ReadByte(1);
		data_list++;
	}
	*data_list=i2c_ReadByte(0);
	i2c_Stop();
	
	return 0;
}

/**
 * @brief   		��MPU6050�Ĵ�����ȡ����
 * @param[in]  reg_add 	Ŀ��Ĵ�����ַ
 * @param[in]  data_list	��Ŷ�ȡ���ݵĻ�����
 * @param[in]  num 			Ҫ��ȡ�������ֽ���
 * @return  
 */
static uint8_t mpu6050_write_data(const uint8_t reg_add,
																	uint8_t* data_list,
																	const uint8_t num){
	i2c_Start(); 
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);
	i2c_WaitAck();
																
	i2c_SendByte(reg_add);
	i2c_WaitAck();
	
	for(u8 i = 0;i < num;i++){
		i2c_SendByte(data_list[i]);
		i2c_WaitAck();
	}    
	i2c_Stop();
	return 0;
}

/**
 * @brief   ��ʼ��MPU6050оƬ
 * @param   
 * @return  
 */
static void mpu6050_init(void){
	iic_driver_initialize(&g_MPU6050_IIC_DEVICE);
	g_MPU6050_IIC_DEVICE.iic_scl_pin = MPU6050_SCL_PIN;
	g_MPU6050_IIC_DEVICE.iic_sda_pin = MPU6050_SDA_PIN;
	g_MPU6050_IIC_DEVICE.iic_gpio_port = MPU6050_IIC_PORT;

	g_MPU6050_IIC_DEVICE.i2c_gpio_config(&g_MPU6050_IIC_DEVICE);
	
	delay_ms_soft(100);																									// �ϵ���ʱ
	
	mpu6050_write_reg(MPU6050_RA_PWR_MGMT_1, 0x00);	    								// �������״̬
	mpu6050_write_reg(MPU6050_RA_SMPLRT_DIV , MPU6050_SMPLRT_DIV_SET);	// �����ǲ����ʣ�1KHz
	mpu6050_write_reg(MPU6050_RA_CONFIG , MPU6050_LPF);	        				// ��ͨ�˲��������ã���ֹƵ����1K��������5K
	mpu6050_write_reg(MPU6050_RA_ACCEL_CONFIG , MPU6050_ACC_SAMPLE);	  // ���ٶȼ�����
	mpu6050_write_reg(MPU6050_RA_GYRO_CONFIG, MPU6050_GYR_SAMPLE);     	// ����������
}

#if	0
/**
 * @brief   		����MPU6050�ĵ�ͨ�˲�ֵ
 * @param[in]   lpf ��ֹƵ��
 * @return
 */
static void mpu6050_set_lpf(uint16_t lpf)
{
	u8 data=0;
	if(lpf >= 188)		 data = 1;
	else if(lpf >= 98 ) data = 2;
	else if(lpf >= 42 ) data = 3;
	else if(lpf >= 20 ) data = 4;
	else if(lpf >= 10 ) data = 5;
	else 							 data = 6; 
	mpu6050_write_reg(MPU6050_RA_CONFIG,data);
}

 
/**
 * @brief   		����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
 * @param[in]   rate 4~1000(Hz)
 * @return
 */
static void mpu6050_set_rate(uint16_t rate)
{
	uint8_t data;
	if(rate > 1000) rate = 1000;
	if(rate < 4)	  rate = 4;
	data = 1000 / rate - 1;
	mpu6050_write_reg(MPU6050_RA_SMPLRT_DIV,data);	//�������ֵ�ͨ�˲���
// 	mpu6050_set_lpf(rate / 2);											//�Զ�����LPFΪ�����ʵ�һ��
}

#endif

/**
 * @brief   		��ȡMPU6050��ID
 * @param[in]   
 * @return  		MPU6050 ID
 */
static uint8_t mpu6050_get_id(void){
	uint8_t re = 0;
	mpu6050_read_data(MPU6050_RA_WHO_AM_I,&re,1);
	if(re != 0x68){
		debug_uart_printf_1("MPU6050 dectected error! %d \r\n",re);
		return 0;
	}
	debug_uart_printf_1("MPU6050 ID = %d\r\n",re);
	return 1;
}

/**
 * @brief   		 ��ȡMPU6050�ļ��ٶ�����
 * @param[out]  accData  ���ռ��ٶȵ�����
 * @return  
 */
static void mpu6050_get_acc(int16_t *acc_list){
	uint8_t buf[6];
	mpu6050_read_data(MPU6050_ACC_OUT, buf, 6);
	acc_list[0] = (buf[0] << 8) | buf[1];
	acc_list[1] = (buf[2] << 8) | buf[3];
	acc_list[2] = (buf[4] << 8) | buf[5];
}

/**
 * @brief   		 ��ȡMPU6050�ĽǼ��ٶ�����
 * @param[out]  gyroData  ���սǼ��ٶȵ�����
 * @return  
 */
static void mpu6050_get_gyro(int16_t *gyro_list){
	uint8_t buf[6];
	mpu6050_read_data(MPU6050_GYRO_OUT,buf,6);
	gyro_list[0] = (buf[0] << 8) | buf[1];
	gyro_list[1] = (buf[2] << 8) | buf[3];
	gyro_list[2] = (buf[4] << 8) | buf[5];
}


/**
 * @brief   		 ��ȡMPU6050��ԭʼ�¶�����
 * @param[out]  tempData  �¶ȱ���
 * @return  
 */
static void mpu6050_get_temp(int16_t *temp_data){
	uint8_t buf[2];
	mpu6050_read_data(MPU6050_RA_TEMP_OUT_H,buf,2);
	*temp_data = (buf[0] << 8) | buf[1];
}

/**
 * @brief   		�� MPU6050 ���ٶȼ�����ת��Ϊʵ����������
 * @param[int]  gyro_value		������ļ��ٶȼ�����
 * @return  		void
 * @example			float data = mpu6050_acc_transition(imu660ra_acc_x);  //��λΪ g(m/s^2)
 */
static float mpu6050_acc_transition (int16_t acc_value){
	float acc_data = 0;
	switch(MPU6050_ACC_SAMPLE)
	{
		case 0x00: acc_data = (float)acc_value / 16384; break;      // 0x00 ���ٶȼ�����Ϊ:��2g   ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
		case 0x08: acc_data = (float)acc_value / 8192;  break;      // 0x08 ���ٶȼ�����Ϊ:��4g   ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
		case 0x10: acc_data = (float)acc_value / 4096;  break;      // 0x10 ���ٶȼ�����Ϊ:��8g   ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
		case 0x18: acc_data = (float)acc_value / 2048;  break;      // 0x18 ���ٶȼ�����Ϊ:��16g  ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
		default: break;
	}
	return acc_data;
}


/**
 * @brief   		�� MPU6050 ����������ת��Ϊʵ����������
 * @param[int]  gyro_value		�����������������
 * @return  		void
 * @example			float data = mpu6050_gyro_transition(imu660ra_gyro_x);  // ��λΪ��/s
 */
static float mpu6050_gyro_transition (int16_t gyro_value){
	float gyro_data = 0;
	switch(MPU6050_GYR_SAMPLE)
	{
		case 0x00: gyro_data = (float)gyro_value / 131.0f;  break;  // 0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ��� 131     ����ת��Ϊ������λ�����ݣ���λΪ����/s
		case 0x08: gyro_data = (float)gyro_value / 65.5f;   break;  // 0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ��� 65.5    ����ת��Ϊ������λ�����ݣ���λΪ����/s
		case 0x10: gyro_data = (float)gyro_value / 32.8f;   break;  // 0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8     ����ת��Ϊ������λ�����ݣ���λΪ����/s
		case 0x18: gyro_data = (float)gyro_value / 16.4f;   break;  // 0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4     ����ת��Ϊ������λ�����ݣ���λΪ����/s
		default: break;
	}
	return gyro_data;
}

/**
  * @brief   		 ��ȡMPU6050���¶����ݣ�ת�������϶�
  * @param[out]  temperature  �¶ȱ��� 
  * @return  
  */
static void mpu6050_temp_transition(float *temperature){
	uint8_t buf[2];
	mpu6050_read_data(MPU6050_RA_TEMP_OUT_H,buf,2);
  int16_t temp3= (buf[0] << 8) | buf[1];	
	*temperature=((double) temp3/340.0)+36.53;
}

/******************************************************************************/
/*-----------------------------------�ⲿ�ӿ�---------------------------------*/
/******************************************************************************/
RRD_DEVICE_MPU6050 MPU6050_DEVICE = {
	.mpu6050_init = mpu6050_init,
	.mpu6050_read_data = mpu6050_read_data,
	.mpu6050_write_data = mpu6050_write_data,
	.mpu6050_get_id = mpu6050_get_id,
	.mpu6050_get_acc = mpu6050_get_acc,
	.mpu6050_get_temp = mpu6050_get_temp,
	.mpu6050_get_gyro = mpu6050_get_gyro,
	.mpu6050_acc_transition = mpu6050_acc_transition,
	.mpu6050_gyro_transition = mpu6050_gyro_transition,
	.mpu6050_temp_transition = mpu6050_temp_transition
};

/******************************************************************************/
/*-----------------------------------DEBUG�ӿ�--------------------------------*/
/******************************************************************************/
#if OPEN_MPU6050_TEST
void mpu6050_test(void){
	
	float Temp;
	int16_t Gyro[3];	float Gyro_F[3];
	int16_t Accel[3];	float Accel_F[3];

	debug_uart_init();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	MPU6050_DEVICE.mpu6050_init();
	
	printf("init now!!! \r\n");
	if (!MPU6050_DEVICE.mpu6050_get_id()){
    printf("\r\nû�м�⵽MPU6050��������\r\n");
		while(1);
	}
  
  while(1)
  {		
		MPU6050_DEVICE.mpu6050_get_acc(Accel);
		MPU6050_DEVICE.mpu6050_get_gyro(Gyro);
		MPU6050_DEVICE.mpu6050_temp_transition(&Temp);
		
//		printf("\r\n���ٶȣ� %8d%8d%8d    ",Accel[0],Accel[1],Accel[2]);
//		printf("�����ǣ� %8d%8d%8d    ",Gyro[0],Gyro[1],Gyro[2]);
//		printf("�¶ȣ� %8.2f",Temp);
	
		Accel_F[0] = MPU6050_DEVICE.mpu6050_acc_transition(Accel[0]);
		Accel_F[1] = MPU6050_DEVICE.mpu6050_acc_transition(Accel[1]);
		Accel_F[2] = MPU6050_DEVICE.mpu6050_acc_transition(Accel[2]);
	
		Gyro_F[0] = MPU6050_DEVICE.mpu6050_gyro_transition(Gyro[0]);
		Gyro_F[1] = MPU6050_DEVICE.mpu6050_gyro_transition(Gyro[1]);
		Gyro_F[2] = MPU6050_DEVICE.mpu6050_gyro_transition(Gyro[2]);
	
		printf("\r\n���ٶȣ� %8.2f%8.2f%8.2f    ",Accel_F[0],Accel_F[1],Accel_F[2]);
		printf("�����ǣ� %8.2f%8.2f%8.2f    ",    Gyro_F[0], Gyro_F[1], Gyro_F[2]);

		delay_ms_soft(10);
	}
}
#endif
