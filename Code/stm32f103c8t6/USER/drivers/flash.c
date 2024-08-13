#include "flash.h"

/**
 * @brief  			�жϵ�ǰҳ��û������
 * @param[in]   
 * @return      
 */
boolean flash_page_has_data(uint32_t address){
	if(address > FLASH_TAIL_ADDRESS
		|| address < FLASH_BASE_ADDRESS){
		debug_uart_printf_2(0,"flash address no in range");
		return FALSE;
	}

	uint32_t page_address = GET_PAGE_START_ADDRESS(address);

	// 4�ֽڶ���
	uint32_t num = 0;
	debug_uart_printf_2(0,"\r\n");
	for(num = 0; num < FLASH_PAGE_SIZE && *(uint32_t *)(page_address + num  * 4) == 0; num ++);
	debug_uart_printf_2(0,"\r\n");
	return num == FLASH_PAGE_SIZE ? FALSE : TRUE;
}

/**
 * @brief  			��������flash����
 * @param[in]   
 * @return      
 */
static void flash_erase_all_pages(void){
	FLASH_Unlock();
	FLASH_EraseAllPages();
	FLASH_Lock();
}

/**
 * @brief  			����ָ����ַ������
 * @param[in]   address  ��ַ
 * @return      
 */
static void flash_erase_page(uint32_t address){
	if ((address % FLASH_PAGE_SIZE) != 0) {
		// û���ֽڶ���
		debug_uart_printf_2(0,"unaligned byte access. \r\n");
		return;
	}
	if(address > FLASH_TAIL_ADDRESS
		|| address < FLASH_BASE_ADDRESS){
		debug_uart_printf_2(0,"flash address no in range");
		return;
	}
	FLASH_Unlock();
	FLASH_ErasePage(address);
	FLASH_Lock();
}

/**
 * @brief  			��flashָ����ַд��uint32����
 * @param[in]   address  ��ַ
 * @param[in]   data  	 Ҫд���uint32����
 * @return      
 */
static void flash_write_32bits_data(uint32_t address, uint32_t data){
	if(address > FLASH_TAIL_ADDRESS
		|| address < FLASH_BASE_ADDRESS){
		debug_uart_printf_2(0,"flash address no in range");
		return;
	}
	FLASH_Unlock();
	FLASH_ProgramWord(address, data);
	FLASH_Lock();
}

/**
 * @brief  			��ȡuint32����
 * @param[in]   address  ��ַ
 * @return      
 */
static uint32_t flash_read_32bits_data(uint32_t address){
	if(address > FLASH_TAIL_ADDRESS
		|| address < FLASH_BASE_ADDRESS){
		debug_uart_printf_2(0,"flash address no in range");
		return 0;
	}
	return *((__IO uint32_t *)(address));
}

/**
 * @brief  			��flashָ����ַд��uint16����
 * @param[in]   address  ��ַ
 * @param[in]   data  	 Ҫд���uint16����
 * @return      
 */
static void flash_write_16bits_data(uint32_t address, uint16_t data){
	if(address > FLASH_TAIL_ADDRESS
		|| address < FLASH_BASE_ADDRESS){
		debug_uart_printf_2(0,"flash address no in range");
		return;
	}
	FLASH_Unlock();
	FLASH_ProgramHalfWord(address, data);	
	FLASH_Lock();
}

/**
 * @brief  			��ȡuint32����
 * @param[in]   address  ��ַ
 * @return      
 */
static uint16_t flash_read_16bits_data(uint32_t address){
	if(address > FLASH_TAIL_ADDRESS
		|| address < FLASH_BASE_ADDRESS){
		debug_uart_printf_2(0,"flash address no in range");
		return 0;
	}
	return *((__IO uint16_t *)(address));
}

/**
 * @brief  			��flashָ����ַд������
 * @param[in]   address  		��ַ
 * @param[in]   data  	 		Ҫд�������ָ��
 * @param[in]		vector_size ������С
 * @return 			����ĩβ�ĵ�ַ
 * @note				���������ݲ�ֳ�8bits�洢 һ��8bits�浽һ��32bits��λ��
 * @note				��������ڴ���С����������8bits�������
 */
static uint32_t flash_write_vector_data_8bits(uint32_t address, const void* data, uint16_t vector_size){
	if(address > FLASH_TAIL_ADDRESS
		|| address < FLASH_BASE_ADDRESS){
		debug_uart_printf_2(0,"flash address no in range");
		return 0;
	}
	const uint8_t* uint8_ptr = (const uint8_t*)data;
	
	FLASH_Unlock();
	uint16_t len = (uint16_t)(vector_size / 1);
	for (uint16_t i = 0; i < len; i += 4) {
			FLASH_ProgramWord(address + i, *((uint32_t *)(uint8_ptr + i)));
	}
	FLASH_Lock();
	
	return address + len;
}


/**
 * @brief  			��ȡ���ݵ�����
 * @param[in]   address  		��ַ
 * @param[in]   data  	 		Ҫ��ȡ������ָ��
 * @param[in]		vector_size ������С
 * @return 
 * @note				��������ڴ���С����������8bits�������
 */
static void flash_read_vector_data_8bits(uint32_t address, void* data, uint16_t vector_size){
	if(address > FLASH_TAIL_ADDRESS
		|| address < FLASH_BASE_ADDRESS){
		debug_uart_printf_2(0,"flash address no in range");
		return;
	}
	uint8_t* uint8_ptr = (uint8_t*)data;
	for (uint16_t i = 0; i < vector_size; i += 4) {
			*((uint32_t *)(uint8_ptr + i)) = FLASH_READ_DATA(address + i,uint32_t);
	}
}

#if FLASH_NO_USE_SECTION
/**
 * @brief  			��flashָ����ַд������
 * @param[in]   address  		��ַ
 * @param[in]   data  	 		Ҫд�������ָ��
 * @param[in]		vector_size ������С
 * @return 
 * @note				���������ݲ�ֳ�8bits�洢 һ��8bits�浽һ��32bits��λ��
 * @note				��������ڴ���С����������16bits�������
 */
static uint32_t flash_write_vector_data_16bits(uint32_t address, const void* data, uint16_t vector_size){
	if(address > FLASH_TAIL_ADDRESS
		|| address < FLASH_BASE_ADDRESS){
		debug_uart_printf_2(0,"flash address no in range");
		return 0;
	}
	const uint16_t* uint16_ptr = (const uint16_t*)data;
	
	FLASH_Unlock();
	uint16_t len = (uint16_t)(vector_size / 2);
	for (uint16_t i = 0; i < len; i += 1) {
			FLASH_ProgramHalfWord(address + (i * 2), *(uint16_ptr + i));
	}
	FLASH_Lock();
	return address + len;
}

/**
 * @brief  			��flashָ����ַд������
 * @param[in]   address  		��ַ
 * @param[in]   data  	 		Ҫд�������ָ��
 * @param[in]		vector_size ������С
 * @return 
 * @note				���������ݲ�ֳ�8bits�洢 һ��8bits�浽һ��32bits��λ��
 * @note				��������ڴ���С����������32bits�������
 */
static uint32_t flash_write_vector_data_32bits(uint32_t address, const void* data, uint16_t vector_size){
	if(address > FLASH_TAIL_ADDRESS
		|| address < FLASH_BASE_ADDRESS){
		debug_uart_printf_2(0,"flash address no in range");
		return 0;
	}
	const uint32_t* uint32_ptr = (const uint32_t*)data;
	
	FLASH_Unlock();
	uint16_t len = (uint16_t)(vector_size / 4);
	for (uint16_t i = 0; i < len; i += 1) {
			FLASH_ProgramWord(address + (i * 4), *(uint32_ptr + i));
	}
	FLASH_Lock();
	return address + len;
}

/**
 * @brief  			��ȡ���ݵ�����
 * @param[in]   address  		��ַ
 * @param[in]   data  	 		Ҫ��ȡ������ָ��
 * @param[in]		vector_size ������С
 * @return 
 * @note				��������ڴ���С����������16bits�������
 */
static void flash_read_vector_data_16bits(uint32_t address, void* data, uint16_t vector_size){
	if(address > FLASH_TAIL_ADDRESS
		|| address < FLASH_BASE_ADDRESS){
		debug_uart_printf_2(0,"flash address no in range");
		return;
	}
	uint16_t* uint16_ptr = (uint16_t*)data;
	uint16_t len = (uint16_t)(vector_size / 2);
	for (uint16_t i = 0; i < len; i += 1) {
			*(uint16_ptr + i) = FLASH_READ_DATA(address + i * 2,uint16_t);
	}
}

/**
 * @brief  			��ȡ���ݵ�����
 * @param[in]   address  		��ַ
 * @param[in]   data  	 		Ҫ��ȡ������ָ��
 * @param[in]		vector_size ������С
 * @return 
 * @note				��������ڴ���С����������32bits�������
 */
static void flash_read_vector_data_32bits(uint32_t address, void* data, uint16_t vector_size){
	if(address > FLASH_TAIL_ADDRESS
		|| address < FLASH_BASE_ADDRESS){
		debug_uart_printf_2(0,"flash address no in range");
		return;
	}
	uint32_t* uint32_ptr = (uint32_t*)data;
	uint16_t len = (uint16_t)(vector_size / 4);
	for (uint16_t i = 0; i < len; i += 1) {
			*(uint32_ptr + i) = FLASH_READ_DATA(address + i * 4,uint32_t);
	}
}
#endif

/******************************************************************************/
/*-----------------------------------�ⲿ�ӿ�---------------------------------*/
/******************************************************************************/
RRD_DRIVER_FLASH FLASH_DRIVER = {
	.flash_page_has_data = flash_page_has_data,
	
	.flash_erase_page = flash_erase_page,
	.flash_erase_all_pages = flash_erase_all_pages,
	
	.flash_read_16bits_data = flash_read_16bits_data,
	.flash_read_32bits_data = flash_read_32bits_data,
	
	.flash_write_16bits_data = flash_write_16bits_data,
	.flash_write_32bits_data = flash_write_32bits_data,
	
	.flash_read_vector_data = flash_read_vector_data_8bits,
	.flash_write_vector_data = flash_write_vector_data_8bits
};




/******************************************************************************/
/*-----------------------------------DEBUG�ӿ�--------------------------------*/
/******************************************************************************/
#if OPEN_FLASH_TEST

#include "delay.h"
#include "debug_uart.h"

typedef union{
	float float_data;
	uint32_t uint32_data;
	uint8_t list[4];
} TEST_UNION;

typedef struct {
    uint32_t 	field1;
    uint16_t 	field2;
    uint8_t 	field3;
    uint8_t 	field4;
} TEST_STRUCT;

typedef struct {
    uint32_t 	field1;
    uint16_t 	field2;
    uint16_t 	field3;
    uint16_t 	field4;
} TEST_STRUCT_1;

typedef struct {
    uint32_t 	field1;
    uint32_t 	field2;
    uint32_t 	field3;
    uint32_t 	field4;
} TEST_STRUCT_2;

#define ADDR   ADDR_FLASH_SECTOR(127) + 9

void flash_test(void){
	debug_uart_init();
	
	// ��ȡҳ��ַ
//	uint32_t page_add = GET_PAGE_START_ADDRESS(ADDR);
//	printf("page_add : %x \r\n",page_add);
	
//	// ����ת��
//	float float_data = 3.21;
//	flash_erase_page(ADDR);
//	flash_write_32bits_data(ADDR,FLOAT_TO_UINT32(float_data));
//	float_data = UINT32_TO_FLOAT(FLASH_READ_DATA(ADDR,uint32_t));
//	printf("float_data : %f \r\n",float_data);
//	
//	// ������
//	TEST_UNION data;
//	data.float_data = 90.21;
//	flash_erase_page(ADDR);
//	flash_write_32bits_data(ADDR,data.uint32_data);
//	data.uint32_data = FLASH_READ_DATA(ADDR,uint32_t);
//	debug_uart_send("float_data :  %f \r\n",data.float_data);
//	
//	// uint8
//	uint8_t uint8_data = 10;
//	flash_erase_page(ADDR);
//	flash_write_32bits_data(ADDR,(uint32_t)uint8_data);
//	uint8_data = FLASH_READ_DATA(ADDR,uint8_t);
//	debug_uart_send("uint8_data :  %d \r\n",uint8_data);
//	
//	// int8
//	int8_t int8_data = -11;
//	flash_erase_page(ADDR);
//	flash_write_32bits_data(ADDR,(uint32_t)int8_data);
//	int8_data = FLASH_READ_DATA(ADDR,int8_t);
//	debug_uart_send("int8_data :  %d \r\n",int8_data);
	
//	// �ṹ��
//	TEST_STRUCT test_struct;
//	test_struct.field1 = 300;
//	test_struct.field2 = 280;
//	test_struct.field3 = 10;
//	test_struct.field4 = 8;
//	
//	flash_erase_page(ADDR);
//	flash_write_vector_data_8bits(ADDR,&test_struct,sizeof(test_struct));
//	
//	TEST_STRUCT test_struct_1;
//	flash_read_vector_data_8bits(ADDR,&test_struct_1,sizeof(test_struct_1));
//	
//	printf(" %4d %4d %4d %4d \r\n",test_struct_1.field1,
//																	test_struct_1.field2,
//																	test_struct_1.field3,
//																	test_struct_1.field4);

//	// �ṹ��
//	TEST_STRUCT_1 test_struct;
//	test_struct.field1 = 300;
//	test_struct.field2 = 666;
//	test_struct.field3 = 555;
//	test_struct.field4 = 43334;
//	
//	flash_erase_page(ADDR);
//	printf("1 \r\n");
//	flash_write_vector_data_16bits(ADDR,&test_struct,sizeof(test_struct));
//	printf("2 \r\n");
//	TEST_STRUCT_1 test_struct_1;
//	flash_read_vector_data_16bits(ADDR,&test_struct_1,sizeof(test_struct_1));
//	
//	printf(" %d %d %d %d \r\n",test_struct_1.field1,
//																	test_struct_1.field2,
//																	test_struct_1.field3,
//																	test_struct_1.field4);

	// �ṹ��
//	TEST_STRUCT_2 test_struct;
//	test_struct.field1 = 222222222;
//	test_struct.field2 = 1111111;
//	test_struct.field3 = 12345678;
//	test_struct.field4 = 43334;
//	
//	flash_erase_page(ADDR);
//	flash_write_vector_data_32bits(ADDR,&test_struct,sizeof(TEST_STRUCT_2));
//	
//	TEST_STRUCT_2 test_struct_1;
//	flash_read_vector_data_32bits(ADDR,&test_struct_1,sizeof(TEST_STRUCT_2));
//	
//	printf(" %d %d %d %d \r\n",test_struct_1.field1,
//																	test_struct_1.field2,
//																	test_struct_1.field3,
//																	test_struct_1.field4);

	while(1){
		delay_ms_soft(10);
		printf("hello \r\n");
	}
}
#endif
