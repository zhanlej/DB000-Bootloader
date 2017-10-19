#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//STM32 FLASH ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////
//�û������Լ�����Ҫ����
#define STM32_FLASH_SIZE 64 	 		//��ѡSTM32��FLASH������С(��λΪK)
#define STM32_FLASH_WREN 1              //ʹ��FLASHд��(0��������;1��ʹ��)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH����ʼ��ַ
//FLASH������ֵ

//����һЩ�����ĵ�ַ
#define FLASH_SAVE_ADDR  0X0800FC00		//����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)
//��ʱʱ�䱣����FLASH�еĵ�ַ
#define FLASH_PARAM_SIZE 4
#define FLASH_TIMEOUT_FLAG FLASH_SAVE_ADDR //timeout_flag�ĵ�ַ
#define FLASH_TIMEOUT FLASH_TIMEOUT_FLAG+FLASH_PARAM_SIZE //timeout�ĵ�ַ
//FTP����������FLASH�еĵ�ַ
#define FTP_PARAM_SIZE 32
#define FLASH_FIRMWARE_FLAG FLASH_TIMEOUT+FLASH_PARAM_SIZE //firmware_flag�ĵ�ַ
#define FLASH_FTP_SERVER FLASH_FIRMWARE_FLAG+FLASH_PARAM_SIZE //ftp������IP��ַ
#define FLASH_FTP_USERNAME FLASH_FTP_SERVER+FTP_PARAM_SIZE //ftp�û���
#define FLASH_FTP_PASSWD FLASH_FTP_USERNAME+FTP_PARAM_SIZE //ftp����
#define FLASH_FTP_FILENAME FLASH_FTP_PASSWD+FTP_PARAM_SIZE //ftp�ļ���
#define FLASH_FTP_PATH FLASH_FTP_FILENAME+FTP_PARAM_SIZE //ftp·��
 

u16 STMFLASH_ReadHalfWord(u32 faddr);		  //��������  
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//ָ����ַ��ʼд��ָ�����ȵ�����
u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//ָ����ַ��ʼ��ȡָ����������
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����

//����д��
void Test_Write(u32 WriteAddr,u16 WriteData);								   
void Flash_Write_Number(u32 timeout_count, u32 wirte_addr);
u32 Flash_Read_Number(u32 wirte_addr);
void Flash_Write_Str(u32 flash_addr,u8 *write_buf,u32 size);
void Flash_Read_Str(u32 flash_addr,u8 *read_buf,u32 size);

#endif

















