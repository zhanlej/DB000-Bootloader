#ifndef __IAP_H__
#define __IAP_H__
#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//IAP ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/24
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////	
typedef  void (*iapfun)(void);				//����һ���������͵Ĳ���.

//////////////////////////////////////////////////////////////////////////////////
//#define IAP_DEBUG 1

#define FLASH_APP1_ADDR		0x08005000  	//��һ��Ӧ�ó�����ʼ��ַ(�����FLASH)
											//����0X08000000~0X08004FFF�Ŀռ�ΪIAPʹ��
#define FLASH_APP1_SIZE 44*1024
#define APP_TEMP_LEN 1024
#define FIRMWARE_PACK_SIZE 1024
////////////////////////////////////////////////////////////////////////////
#define W25X_NEW_FIRMWARE_ADDR 0
#define W25X_OLD_FIRMWARE_ADDR 0x100000
////////////////////////////////////////////////////////////////////////////

extern u8 firmware_packet[FIRMWARE_PACK_SIZE];	//����ÿ���̼�������

void iap_load_app(u32 appxaddr);			//ִ��flash�����app����
void iap_load_appsram(u32 appxaddr);		//ִ��sram�����app����
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 applen);	//��ָ����ַ��ʼ,д��bin
void iap_read_appbin(u32 appxaddr,u8 *appbuf,u32 appsize);	//��ָ����ַ��ʼ,��ȡ����
void Jump_to_App(void);
int APP_W25qxx_to_Flash(u32 w25_addr, u32 app_addr, u32 app_sum_len);
int APP_Flash_to_W25qxx(u32 app_addr, u32 w25qxx_addr, u32 app_sum_len);
#endif







































