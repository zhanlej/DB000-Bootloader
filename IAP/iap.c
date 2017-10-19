#include "string.h"
#include "sys.h"
#include "delay.h"
#include "uart.h"
#include "stmflash.h"
#include "iap.h"
#include "w25qxx.h"
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

//ȫ�ֱ���
u8 app_temp[APP_TEMP_LEN]; //for write to app
u8 firmware_packet[FIRMWARE_PACK_SIZE];	//����ÿ���̼�������

iapfun jump2app; 
u16 iapbuf[1024];   
//appxaddr:Ӧ�ó������ʼ��ַ
//appbuf:Ӧ�ó���CODE.
//appsize:Ӧ�ó����С(�ֽ�).
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 appsize)
{
	u16 t;
	u16 i=0;
	u16 temp;
	u32 fwaddr=appxaddr;//��ǰд��ĵ�ַ
	u8 *dfu=appbuf;
	for(t=0;t<appsize;t+=2)
	{						    
		temp=(u16)dfu[1]<<8;
		temp+=(u16)dfu[0];	  
		dfu+=2;//ƫ��2���ֽ�
		iapbuf[i++]=temp;	    
		if(i==1024)
		{
			i=0;
			STMFLASH_Write(fwaddr,iapbuf,1024);	
			fwaddr+=2048;//ƫ��2048  16=2*8.����Ҫ����2.
		}
	}
	if(i)STMFLASH_Write(fwaddr,iapbuf,i);//������һЩ�����ֽ�д��ȥ.  
}

void iap_read_appbin(u32 appxaddr,u8 *appbuf,u32 appsize)
{
	STMFLASH_Read(appxaddr,(u16*)appbuf,appsize/2);
}

//��ת��Ӧ�ó����
//appxaddr:�û�������ʼ��ַ.
void iap_load_app(u32 appxaddr)
{
	if(((*(vu32*)appxaddr)&0x2FFE0000)==0x20000000)	//���ջ����ַ�Ƿ�Ϸ�.
	{ 
		jump2app=(iapfun)*(vu32*)(appxaddr+4);		//�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)		
		MSR_MSP(*(vu32*)appxaddr);					//��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)
		jump2app();									//��ת��APP.
	}
}		 

void Jump_to_App(void)
{
	printf("��ʼִ��FLASH�û�����!!\r\n");
	if(((*(vu32*)(FLASH_APP1_ADDR+4))&0xFF000000)==0x08000000)//�ж��Ƿ�Ϊ0X08XXXXXX.
	{	 
		__disable_irq();
		USART_ITConfig(USART1 , USART_IT_RXNE , DISABLE);//ʧ�ܽ����ж�
		USART_Cmd(USART1 , DISABLE);//�رմ���
		USART_ITConfig(USART2 , USART_IT_RXNE , DISABLE);//ʧ�ܽ����ж�
		USART_Cmd(USART2 , DISABLE);//�رմ���
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM2, DISABLE);  //������ʧ��
		SPI_Cmd(SPI2, DISABLE); //ʧ��SPI����
		
		iap_load_app(FLASH_APP1_ADDR);//ִ��FLASH APP����
	}else 
	{
		printf("��FLASHӦ�ó���,�޷�ִ��!\r\n");	   
	}		
}

int APP_W25qxx_to_Flash(u32 w25qxx_addr, u32 app_addr, u32 app_sum_len)
{
u8 appcheck[4];				//��������յ��������Ƿ���app����
u32 appremain = 0;
u32 app_off = 0;
u32 app_temp_len = 0;
	
#ifdef IAP_DEBUG
int i=0, j=0;
#endif

	/*��ʼ��Ƭ��FLASHдAPP����*/
	if(app_sum_len)
	{
		appremain = app_sum_len;
		printf("��ʼ���¹̼�...\r\n");	
		printf("w25qxx_addr = 0x%08x, app_addr = 0x%08x, app_sum_len = %d\r\n", w25qxx_addr, app_addr, app_sum_len);
		W25QXX_Read(appcheck,w25qxx_addr+4,4);					//�ӵ�4����ַ����ʼ,����4���ֽ�
		printf("appcheck = %02x%02x%02x%02x\r\n",appcheck[3],appcheck[2],appcheck[1],appcheck[0]);

		if(appcheck[3] == 0x08)//�ж��Ƿ�Ϊ0X08XXXXXX.
		//if(1)
		{
			
			while(appremain)
			{
				if(appremain <= APP_TEMP_LEN)
				{
					app_temp_len = appremain;
					appremain = 0;
				}
				else
				{
					app_temp_len = APP_TEMP_LEN;
					appremain -= APP_TEMP_LEN;
				}
				
				printf("Start Read W25Q128.... %d", (app_off/APP_TEMP_LEN)+1);
				memset(app_temp, 0, APP_TEMP_LEN);
				W25QXX_Read(app_temp, w25qxx_addr + app_off, app_temp_len);
				
#ifdef IAP_DEBUG
				//�������FLASH�е������Ƿ���ȷ
				printf("\r\nW25XX check\r\n");
				for(i = 0; i < app_temp_len; i+=16)
				{
					printf("%07x: ", (app_off/APP_TEMP_LEN)+i);
					for(j = 0; j < 16; j++)
					{
						printf("%02x", app_temp[i+j]);
						if((j+1)%2 == 0) printf(" ");
					}
					printf("\r\n");
				}
#endif
				
				printf("app_off = %d, app_temp_len = %d\r\n", app_off, app_temp_len);
				iap_write_appbin(app_addr + app_off, app_temp, APP_TEMP_LEN);//����FLASH����   
				
#ifdef IAP_DEBUG
				//�������FLASH�е������Ƿ���ȷ
				iap_read_appbin(app_addr + app_off, app_temp, APP_TEMP_LEN);
				printf("\r\nƬ��FLASH check\r\n");
				for(i = 0; i < APP_TEMP_LEN; i+=16)
				{
					printf("%07x: ", (app_off/APP_TEMP_LEN)+i);
					for(j = 0; j < 16; j++)
					{
						printf("%02x", app_temp[i+j]);
						if((j+1)%2 == 0) printf(" ");
					}
					printf("\r\n");
				}
				memset(app_temp, 0, APP_TEMP_LEN);
#endif
				
				app_off += APP_TEMP_LEN;
			}
			printf("�̼��������!\r\n");	
		}else 
		{	   
			printf("��FLASHӦ�ó���!\r\n");
			return 0;
		}
	}else 
	{
		printf("û�п��Ը��µĹ̼�!\r\n");
		return 0;
	}
	
	return 1;
}

int APP_Flash_to_W25qxx(u32 app_addr, u32 w25qxx_addr, u32 app_sum_len)
{
	u8 appcheck[4];				//��������յ��������Ƿ���app����
	u32 appremain = 0;
	u32 app_off = 0;
	u32 app_temp_len = 0;
	
#ifdef IAP_DEBUG
int i=0, j=0;
#endif

	/*��ʼ��Ƭ��FLASHдAPP����*/
	if(app_sum_len)
	{
		appremain = app_sum_len;
		printf("��ʼ���ݹ̼�...\r\n");	
		printf("w25qxx_addr = 0x%08x, app_addr = 0x%08x, app_sum_len = %d\r\n", w25qxx_addr, app_addr, app_sum_len);
		//W25QXX_Read(appcheck,w25qxx_addr+4,4);					//�ӵ�4����ַ����ʼ,����4���ֽ�
		iap_read_appbin(app_addr+4, appcheck, 4);
		printf("appcheck = %02x%02x%02x%02x\r\n",appcheck[3],appcheck[2],appcheck[1],appcheck[0]);

		if(appcheck[3] == 0x08)//�ж��Ƿ�Ϊ0X08XXXXXX.
		{
			
			while(appremain)
			{
				if(appremain <= APP_TEMP_LEN)
				{
					app_temp_len = appremain;
					appremain = 0;
				}
				else
				{
					app_temp_len = APP_TEMP_LEN;
					appremain -= APP_TEMP_LEN;
				}
				
				printf("Start Read FLASH.... %d", (app_off/APP_TEMP_LEN)+1);
				memset(app_temp, 0, APP_TEMP_LEN);
				//W25QXX_Read(app_temp, w25qxx_addr + app_off, app_temp_len);
				iap_read_appbin(app_addr + app_off, app_temp, app_temp_len);
				
#ifdef IAP_DEBUG
				//�������FLASH�е������Ƿ���ȷ
				printf("\r\nFLASH check\r\n");
				for(i = 0; i < app_temp_len; i+=16)
				{
					printf("%07x: ", (app_off/APP_TEMP_LEN)+i);
					for(j = 0; j < 16; j++)
					{
						printf("%02x", app_temp[i+j]);
						if((j+1)%2 == 0) printf(" ");
					}
					printf("\r\n");
				}
#endif
				
				printf("app_off = %d, app_temp_len = %d\r\n", app_off, app_temp_len);
				//iap_write_appbin(app_addr + app_off, app_temp, APP_TEMP_LEN);//����FLASH����   
				W25QXX_Write(app_temp, w25qxx_addr + app_off, APP_TEMP_LEN);//���ݵ�w25qxx��
				
#ifdef IAP_DEBUG
				//�������FLASH�е������Ƿ���ȷ
				//iap_read_appbin(app_addr + app_off, app_temp, APP_TEMP_LEN);
				W25QXX_Read(app_temp, w25qxx_addr + app_off, APP_TEMP_LEN);
				printf("\r\nW25QXX check\r\n");
				for(i = 0; i < APP_TEMP_LEN; i+=16)
				{
					printf("%07x: ", (app_off/APP_TEMP_LEN)+i);
					for(j = 0; j < 16; j++)
					{
						printf("%02x", app_temp[i+j]);
						if((j+1)%2 == 0) printf(" ");
					}
					printf("\r\n");
				}
				memset(app_temp, 0, APP_TEMP_LEN);
#endif
				
				app_off += APP_TEMP_LEN;
			}
			printf("�̼��������!\r\n");	
		}else 
		{	   
			printf("��FLASHӦ�ó���!\r\n");
			return 0;
		}
	}else 
	{
		printf("û�п��Ա��ݵĹ̼�!\r\n");
		return 0;
	}
	
	return 1;
}








