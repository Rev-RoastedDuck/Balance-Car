/*
 * flash.h
 *
 *  Created on: 2024��8��11��
 *      Author: intl4419
 */

#ifndef DRIVER_FLASH_H_
#define DRIVER_FLASH_H_

/******************************************************************************/
/*---------------------------------ͷ�ļ�����---------------------------------*/
/******************************************************************************/
#include "stm32f10x.h"
#include "typedef.h"
#include "debug_uart.h"

/******************************************************************************/
/*-----------------------------------�궨��-----------------------------------*/
/******************************************************************************/
#define FLASH_BASE_ADDRESS    							((uint32_t)0x08000000)									  // flash�׵�ַ
#define FLASH_TAIL_ADDRESS    							((uint32_t)0x0801FFFF)									  // flashĩ��ַ
#define FLASH_PAGE_SIZE           					((uint32_t)0x400)  											  // һҳ�Ĵ�СΪ1k
#define FLASH_PAGE_NUM											127

/******************************************************************************/
/*------------------------------------�ṹ��----------------------------------*/
/******************************************************************************/
typedef struct _RRD_DRIVER_FLASH {
	boolean 	(*flash_page_has_data)			(uint32_t address);
	
	void 			(*flash_erase_all_pages)		(void);
	void 			(*flash_erase_page)					(uint32_t address);
	
	uint16_t 	(*flash_read_16bits_data)		(uint32_t address);
	uint32_t 	(*flash_read_32bits_data)		(uint32_t address);
	
	void 			(*flash_write_16bits_data)	(uint32_t address, uint16_t data);
	void 			(*flash_write_32bits_data)	(uint32_t address, uint32_t data);
	
	void 			(*flash_read_vector_data)		(uint32_t address, void* data, uint16_t vector_size);
	uint32_t 	(*flash_write_vector_data)	(uint32_t address, const void* data, uint16_t vector_size);
}RRD_DRIVER_FLASH;
/******************************************************************************/
/*-----------------------------------�ⲿ�ӿ�---------------------------------*/
/******************************************************************************/
#define FLOAT_TO_UINT32(float_data)					*((uint32_t*)&float_data)													// floatתuin32�洢
#define UINT32_TO_FLOAT(uint32_data)				*((float*)&uint32_data)														// uint32תfloat�洢
#define ADDR_FLASH_SECTOR(n)  							(FLASH_BASE_ADDRESS + (n) * FLASH_PAGE_SIZE)			// ��ȡ��Nҳ�ĵ�ַ
#define GET_PAGE_START_ADDRESS(address)			((address / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE)		// ��ȡ��ַΪaddress��ҳ��ַ
#define FLASH_READ_DATA(address,data_type)  (*((__IO data_type *)(address)))									// ��ȡ��ַΪaddress ��������Ϊdata_type������

extern RRD_DRIVER_FLASH FLASH_DRIVER;

/******************************************************************************/
/*-----------------------------------DEBUG�ӿ�--------------------------------*/
/******************************************************************************/
#define OPEN_FLASH_TEST 0
#if OPEN_FLASH_TEST
	void flash_test(void);
#endif
#endif
