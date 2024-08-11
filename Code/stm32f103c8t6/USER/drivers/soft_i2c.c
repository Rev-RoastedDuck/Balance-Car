#include "soft_i2c.h"

/*
	应用说明：
	在访问I2C设备前，请先调用 i2c_check_device() 检测I2C设备是否正常，该函数会配置GPIO
*/

#if 1
    #include "stm32f10x_gpio.h"

    /* 使用GPIO库函数实现 */
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
        return GPIO_ReadInputDataBit(driver->iic_gpio_port, driver->iic_sda_pin);	/* 读SDA口线状态 */
    }

#else
    /* 直接寄存器操作实现 */
		/*　注意：如下写法，在IAR最高级别优化时，会被编译器错误优化 */
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
        return (driver->iic_gpio_port->IDR & driver->iic_sda_pin) != 0;	/* 读SDA口线状态 */
    }
#endif




/******************************************************************************/
/*----------------------------------内部宏定义--------------------------------*/
/******************************************************************************/
#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */


/******************************************************************************/
/*-----------------------------------内部函数---------------------------------*/
/******************************************************************************/
/**
 * @brief  			I2C总线位延迟，最快400KHz
 * @note			 下面的时间是通过安富莱AX-Pro逻辑分析仪测试得到的。
							  CPU主频72MHz时，在内部Flash运行, MDK工程不优化
							 循环次数为10时，SCL频率 = 205KHz 
							 循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
							 循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
							 IAR工程编译效率高，不能设置为7
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
 * @brief  			选择gpio时钟线
 * @note				当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号
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
/*-----------------------------------接口实现---------------------------------*/
/******************************************************************************/
/**
 * @brief  			CPU发起I2C总线启动信号
 * @note				当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号
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
 * @brief  			CPU发起I2C总线停止信号
 * @note				当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号
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
 * @brief  			CPU产生一个ACK信号
 * @param[in]   
 * @return
 */
static void i2c_ack(RRD_DRIVER_IIC *driver)
{
	I2C_SDA_0(driver);	// CPU驱动SDA = 0
	i2c_delay();
	I2C_SCL_1(driver);	// CPU产生1个时钟
	i2c_delay();
	I2C_SCL_0(driver);
	i2c_delay();
	I2C_SDA_1(driver);	// CPU释放SDA总线
}

/**
 * @brief  			CPU产生1个NACK信号
 * @param[in]   
 * @return
 */
static void i2c_nack(RRD_DRIVER_IIC *driver)
{
	I2C_SDA_1(driver);	// CPU驱动SDA = 1
	i2c_delay();
	I2C_SCL_1(driver);	// CPU产生1个时钟
	i2c_delay();
	I2C_SCL_0(driver);
	i2c_delay();	
}

/**
 * @brief  			CPU向I2C总线设备发送8bit数据
 * @param[in]   _ucByte  等待发送的字节
 * @return
 */
static void i2c_send_byte(RRD_DRIVER_IIC *driver, uint8_t _ucByte)
{
	uint8_t i;

	// 先发送字节的高位bit7
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
			 I2C_SDA_1(driver); // 释放总线
		}
		_ucByte <<= 1;	// 左移一个bit
		i2c_delay();
	}
}


/**
 * @brief  			CPU从I2C总线设备读取8bit数据	
 * @param[in]   
 * @return			读到的数据
 */
static uint8_t i2c_read_byte(RRD_DRIVER_IIC *driver, u8 ack)
{
	uint8_t i;
	uint8_t value;

	// 读到第1个bit为数据的bit7
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
 * @brief  			CPU产生一个时钟，并读取器件的ACK应答信号
 * @param[in]   
 * @return			0:正确应答，1:无器件响应
 */
static uint8_t i2c_wait_ack(RRD_DRIVER_IIC *driver)
{
	uint8_t re;

	I2C_SDA_1(driver);	// CPU释放SDA总线
	i2c_delay();
	I2C_SCL_1(driver);	// CPU驱动SCL = 1, 此时器件会返回ACK应答
	i2c_delay();
	if (I2C_SDA_READ(driver))	// CPU读取SDA口线状态
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
 * @brief  			配置I2C总线的GPIO，采用模拟IO的方式实现
 * @param[in]   
 * @return
 */
static void i2c_gpio_config(RRD_DRIVER_IIC *driver)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint32_t periph;
	select_RCC_APB2Periph(driver->iic_gpio_port,&periph);
	RCC_APB2PeriphClockCmd(periph, ENABLE);									// 打开GPIO时钟

	GPIO_InitStructure.GPIO_Pin = driver->iic_scl_pin | driver->iic_sda_pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  			// 开漏输出
	GPIO_Init(driver->iic_gpio_port, &GPIO_InitStructure);

	i2c_stop(driver);			// 给一个停止信号, 复位I2C总线上的所有设备到待机模式 
}


/**
 * @brief  			检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
 * @param[in]   _Address 设备的I2C总线地址
 * @return			0:正确， 1:未探测到
 */
static uint8_t i2c_check_device(RRD_DRIVER_IIC *driver,uint8_t _Address)
{
	uint8_t ucAck;

	i2c_gpio_config(driver);								// 配置GPIO
	i2c_start(driver);											// 发送启动信号
	
	i2c_send_byte(driver,_Address|I2C_WR);		// 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传
	ucAck = i2c_wait_ack(driver);						// 检测设备的ACK应答

	i2c_stop(driver);												// 发送停止信号

	return ucAck;
}

/******************************************************************************/
/*----------------------------------接口初始化--------------------------------*/
/******************************************************************************/
/**
 * @brief  			初始化iic设备
 * @param[in]   driver iic设备指针
 * @return			0:初始化成功  -1:未初始化成功
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
