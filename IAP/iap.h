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

#define FLASH_APP1_ADDR		0x08005000  	//��һ��Ӧ�ó�����ʼ��ַ(�����FLASH)
											//����0X08000000~0X08004FFF�Ŀռ�ΪIAPʹ��

void iap_load_app(u32 appxaddr);			//ִ��flash�����app����
void iap_load_appsram(u32 appxaddr);		//ִ��sram�����app����
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 applen);	//��ָ����ַ��ʼ,д��bin
void iap_read_appbin(u32 appxaddr,u8 *appbuf,u32 appsize);	//��ָ����ַ��ʼ,��ȡ����
void Jump_to_App(void);
int APP_W25qxx_to_Flash(u32 ftp_sum_len);
#endif







































