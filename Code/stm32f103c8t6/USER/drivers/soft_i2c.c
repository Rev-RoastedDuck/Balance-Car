#include "soft_i2c.h"

/*
	Ӧ��˵����
	�ڷ���I2C�豸ǰ�����ȵ��� i2c_check_device() ���I2C�豸�Ƿ��������ú���������GPIO
*/

#if 1
    #include "stm32f10x_gpio.h"

    /* ʹ��GPIO�⺯��ʵ�� */
    static inline void I2C_SCL_1(RRD_DRIVER_IIC *driver) {
        GPIO_SetBits(driver->iic_gpio_port, driver->iic_scl_pin);
    }

    static inline void I2C_SCL_0(RRD_DRIVER_IIC *driver) {
        GPIO_ResetBits(driver->iic_gpio_port, driver->iic_scl_pin);
    }

    static inline void I2C_SDA_1(RRD_DRIVER_IIC *driver) {
        GPIO_SetBits(driver->iic_gpio_port, driver->iic_sda_pin);
    }

    static inline void I2C_SDA_0(RRD_DRIVER_IIC *driver) {
        GPIO_ResetBits(driver->iic_gpio_port, driver->iic_sda_pin);
    }

    static inline uint8_t I2C_SDA_READ(RRD_DRIVER_IIC *driver) {
        return GPIO_ReadInputDataBit(driver->iic_gpio_port, driver->iic_sda_pin);	/* ��SDA����״̬ */
    }

#else
    /* ֱ�ӼĴ�������ʵ�� */
		/*��ע�⣺����д������IAR��߼����Ż�ʱ���ᱻ�����������Ż� */
    static inline void I2C_SCL_1(RRD_DRIVER_IIC *driver) {
        driver->iic_gpio_port->BSRR = driver->iic_scl_pin;
    }

    static inline void I2C_SCL_0(RRD_DRIVER_IIC *driver) {
        driver->iic_gpio_port->BRR = driver->iic_scl_pin;
    }

    static inline void I2C_SDA_1(RRD_DRIVER_IIC *driver) {
        driver->iic_gpio_port->BSRR = driver->iic_sda_pin;
    }

    static inline void I2C_SDA_0(RRD_DRIVER_IIC *driver) {
        driver->iic_gpio_port->BRR = driver->iic_sda_pin;
    }

    static inline uint8_t I2C_SDA_READ(RRD_DRIVER_IIC *driver) {
        return (driver->iic_gpio_port->IDR & driver->iic_sda_pin) != 0;	/* ��SDA����״̬ */
    }
#endif




/******************************************************************************/
/*----------------------------------�ڲ��궨��--------------------------------*/
/******************************************************************************/
#define I2C_WR	0		/* д����bit */
#define I2C_RD	1		/* ������bit */


/******************************************************************************/
/*-----------------------------------�ڲ�����---------------------------------*/
/******************************************************************************/
/**
 * @brief  			I2C����λ�ӳ٣����400KHz
 * @note			 �����ʱ����ͨ��������AX-Pro�߼������ǲ��Եõ��ġ�
							  CPU��Ƶ72MHzʱ�����ڲ�Flash����, MDK���̲��Ż�
							 ѭ������Ϊ10ʱ��SCLƵ�� = 205KHz 
							 ѭ������Ϊ7ʱ��SCLƵ�� = 347KHz�� SCL�ߵ�ƽʱ��1.5us��SCL�͵�ƽʱ��2.87us 
							 ѭ������Ϊ5ʱ��SCLƵ�� = 421KHz�� SCL�ߵ�ƽʱ��1.25us��SCL�͵�ƽʱ��2.375us 
							 IAR���̱���Ч�ʸߣ���������Ϊ7
 * @param[in]   
 * @return      
 */
static inline void i2c_delay(void)
{
	// 10
	uint8_t i;
	for (i = 0; i < 8; i++);
}

/**
 * @brief  			ѡ��gpioʱ����
 * @note				��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź�
 * @param[in]   
 * @return
 */
static int8_t select_RCC_APB2Periph(GPIO_TypeDef* iic_gpio,uint32_t* periph){
	if(GPIOA == iic_gpio){
		*periph = RCC_APB2Periph_GPIOA;
		return 0;
	}
	else if(GPIOB == iic_gpio){
		*periph = RCC_APB2Periph_GPIOB;
		return 0;
	}
	else if(GPIOC == iic_gpio){
		*periph = RCC_APB2Periph_GPIOC;
		return 0;
	}
	else if(GPIOD == iic_gpio){
		*periph = RCC_APB2Periph_GPIOD;
		return 0;
	}
	else if(GPIOE == iic_gpio){
		*periph = RCC_APB2Periph_GPIOE;
		return 0;
	}
	
	return -1;
}


/******************************************************************************/
/*-----------------------------------�ӿ�ʵ��---------------------------------*/
/******************************************************************************/
/**
 * @brief  			CPU����I2C���������ź�
 * @note				��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź�
 * @param[in]   
 * @return
 */
static void i2c_start(RRD_DRIVER_IIC *driver)
{
	I2C_SDA_1(driver);
	I2C_SCL_1(driver);
	i2c_delay();
	I2C_SDA_0(driver);
	i2c_delay();
	I2C_SCL_0(driver);
	i2c_delay();
}

/**
 * @brief  			CPU����I2C����ֹͣ�ź�
 * @note				��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź�
 * @param[in]   
 * @return
 */
static void i2c_stop(RRD_DRIVER_IIC *driver)
{
	I2C_SDA_0(driver);
	I2C_SCL_1(driver);
	i2c_delay();
	I2C_SDA_1(driver);
}

/**
 * @brief  			CPU����һ��ACK�ź�
 * @param[in]   
 * @return
 */
static void i2c_ack(RRD_DRIVER_IIC *driver)
{
	I2C_SDA_0(driver);	// CPU����SDA = 0
	i2c_delay();
	I2C_SCL_1(driver);	// CPU����1��ʱ��
	i2c_delay();
	I2C_SCL_0(driver);
	i2c_delay();
	I2C_SDA_1(driver);	// CPU�ͷ�SDA����
}

/**
 * @brief  			CPU����1��NACK�ź�
 * @param[in]   
 * @return
 */
static void i2c_nack(RRD_DRIVER_IIC *driver)
{
	I2C_SDA_1(driver);	// CPU����SDA = 1
	i2c_delay();
	I2C_SCL_1(driver);	// CPU����1��ʱ��
	i2c_delay();
	I2C_SCL_0(driver);
	i2c_delay();	
}

/**
 * @brief  			CPU��I2C�����豸����8bit����
 * @param[in]   _ucByte  �ȴ����͵��ֽ�
 * @return
 */
static void i2c_send_byte(RRD_DRIVER_IIC *driver, uint8_t _ucByte)
{
	uint8_t i;

	// �ȷ����ֽڵĸ�λbit7
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
		{
			I2C_SDA_1(driver);
		}
		else
		{
			I2C_SDA_0(driver);
		}
		i2c_delay();
		I2C_SCL_1(driver);
		i2c_delay();	
		I2C_SCL_0(driver);
		if (i == 7)
		{
			 I2C_SDA_1(driver); // �ͷ�����
		}
		_ucByte <<= 1;	// ����һ��bit
		i2c_delay();
	}
}


/**
 * @brief  			CPU��I2C�����豸��ȡ8bit����	
 * @param[in]   
 * @return			����������
 */
static uint8_t i2c_read_byte(RRD_DRIVER_IIC *driver, u8 ack)
{
	uint8_t i;
	uint8_t value;

	// ������1��bitΪ���ݵ�bit7
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1(driver);
		i2c_delay();
		if (I2C_SDA_READ(driver))
		{
			value++;
		}
		I2C_SCL_0(driver);
		i2c_delay();
	}
	if(ack==0)
		i2c_nack(driver);
	else
		i2c_ack(driver);
	return value;
}

/**
 * @brief  			CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
 * @param[in]   
 * @return			0:��ȷӦ��1:��������Ӧ
 */
static uint8_t i2c_wait_ack(RRD_DRIVER_IIC *driver)
{
	uint8_t re;

	I2C_SDA_1(driver);	// CPU�ͷ�SDA����
	i2c_delay();
	I2C_SCL_1(driver);	// CPU����SCL = 1, ��ʱ�����᷵��ACKӦ��
	i2c_delay();
	if (I2C_SDA_READ(driver))	// CPU��ȡSDA����״̬
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0(driver);
	i2c_delay();
	return re;
}


/**
 * @brief  			����I2C���ߵ�GPIO������ģ��IO�ķ�ʽʵ��
 * @param[in]   
 * @return
 */
static void i2c_gpio_config(RRD_DRIVER_IIC *driver)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint32_t periph;
	select_RCC_APB2Periph(driver->iic_gpio_port,&periph);
	RCC_APB2PeriphClockCmd(periph, ENABLE);									// ��GPIOʱ��

	GPIO_InitStructure.GPIO_Pin = driver->iic_scl_pin | driver->iic_sda_pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  			// ��©���
	GPIO_Init(driver->iic_gpio_port, &GPIO_InitStructure);

	i2c_stop(driver);			// ��һ��ֹͣ�ź�, ��λI2C�����ϵ������豸������ģʽ 
}


/**
 * @brief  			���I2C�����豸��CPU�����豸��ַ��Ȼ���ȡ�豸Ӧ�����жϸ��豸�Ƿ����
 * @param[in]   _Address �豸��I2C���ߵ�ַ
 * @return			0:��ȷ�� 1:δ̽�⵽
 */
static uint8_t i2c_check_device(RRD_DRIVER_IIC *driver,uint8_t _Address)
{
	uint8_t ucAck;

	i2c_gpio_config(driver);								// ����GPIO
	i2c_start(driver);											// ���������ź�
	
	i2c_send_byte(driver,_Address|I2C_WR);		// �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ�
	ucAck = i2c_wait_ack(driver);						// ����豸��ACKӦ��

	i2c_stop(driver);												// ����ֹͣ�ź�

	return ucAck;
}

/******************************************************************************/
/*----------------------------------�ӿڳ�ʼ��--------------------------------*/
/******************************************************************************/
/**
 * @brief  			��ʼ��iic�豸
 * @param[in]   driver iic�豸ָ��
 * @return			0:��ʼ���ɹ�  -1:δ��ʼ���ɹ�
 */
int8_t iic_driver_initialize(RRD_DRIVER_IIC *driver) {
	if (driver == NULL){ return -1;}

	driver->iic_gpio_port = GPIOB;
	driver->iic_scl_pin = GPIO_Pin_6;
	driver->iic_sda_pin = GPIO_Pin_7;
	
	driver->i2c_stop = i2c_stop;
	driver->i2c_start = i2c_start;
	
	driver->i2c_read_byte = i2c_read_byte;
	driver->i2c_send_byte = i2c_send_byte;

	driver->i2c_ack = i2c_ack;
	driver->i2c_nack = i2c_nack;
	driver->i2c_wait_ack = i2c_wait_ack;
	
	driver->i2c_gpio_config = i2c_gpio_config;
	driver->i2c_check_device = i2c_check_device;
	
	return 0;
}
