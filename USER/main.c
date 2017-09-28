#include "stdio.h"
#include "string.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
//#include "lcd.h"
//#include "usart.h"
#include "stmflash.h"
#include "iap.h"
#include "uart.h"
//����flash
#include "w25qxx.h"	
 
 
/************************************************
 ALIENTEKս��STM32������ʵ��47
 IAPʵ�� Bootloader V1.0 ���� 
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

#define APP_TEMP_LEN 5120
u8 app_temp[APP_TEMP_LEN];
int app_temp_len = 0;

int main(void)
{		
	u8 t;
	u8 key;
	u16 oldcount=0;				//�ϵĴ��ڽ�������ֵ
	u16 applenth=0;				//���յ���app���볤��
	u8 clearflag=0;  
	u8 appcheck[4];				//��������յ��������Ƿ���app����
	int appremain = 0;				//��¼ʣ��������ݻ�û
	int app_off = 0;
	int i = 0;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	//uart_init(115200);	//���ڳ�ʼ��Ϊ115200
	USART1Conf(115200, 0, 0);
	USART2Conf(115200, 0, 1);
	delay_init();	   	 	//��ʱ��ʼ�� 
 	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();					//��ʼ������
	W25QXX_Init();			//W25QXX��ʼ��
	
	printf("test");
	while(W25QXX_ReadID()!=W25Q128)								//��ⲻ��W25Q128
	{
		printf("W25Q128 Check Failed!");
		delay_ms(500);
		printf("Please Check!");
		delay_ms(500);
		LED0=!LED0;//DS0��˸
	}
 
	while(1)
	{
	 	if(USART_RX_CNT)
		{
			if(oldcount==USART_RX_CNT)//��������,û���յ��κ�����,��Ϊ�������ݽ������.
			{
				applenth=USART_RX_CNT;
				oldcount=0;
				USART_RX_CNT=0;
				printf("�û�����������!\r\n");
				printf("���볤��:%dBytes\r\n",applenth);
				printf("Start Write W25Q128....\r\n"); 
				W25QXX_Write((u8*)USART_RX_BUF, 0, applenth);			//�ӵ�0����ַ����ʼд������
				printf("W25Q128 Write Finished!\r\n");	//��ʾ�������
				memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
			}else oldcount=USART_RX_CNT;			
		}
		t++;
		delay_ms(10);
		if(t==30)
		{
			LED0=!LED0;
			t=0;
			if(clearflag)
			{
				clearflag--;
			}
		}	  	 
		key=KEY_Scan(0);
		if(key==WKUP_PRES)
		{
			if(applenth)
			{
				appremain = applenth;
				printf("��ʼ���¹̼�...\r\n");	
				W25QXX_Read(appcheck,4,4);					//�ӵ�4����ַ����ʼ,����4���ֽ�
				printf("appcheck = %02x%02x%02x%02x\r\n",appcheck[3],appcheck[2],appcheck[1],appcheck[0]);

				//printf("(*(vu32*)(0X20001000+4)) = %x\n", (*(vu32*)(0X20001000+4)));
 				//if(((*(vu32*)(0X20001000+4))&0xFF000000)==0x08000000)//�ж��Ƿ�Ϊ0X08XXXXXX.
				if(appcheck[3] == 0x08)//�ж��Ƿ�Ϊ0X08XXXXXX.
				{
					i = 0;
					app_off = i*APP_TEMP_LEN;
					
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
						
						printf("Start Read W25Q128.... %d\r\n", i+1);
						W25QXX_Read(app_temp, app_off, app_temp_len);
						//printf("W25Q128 Read Finished!\r\n");
						iap_write_appbin(FLASH_APP1_ADDR+app_off, app_temp, app_temp_len);//����FLASH����   
						memset(app_temp, 0, APP_TEMP_LEN);
						
						i++;
					}
					printf("�̼��������!\r\n");	
				}else 
				{	   
					printf("��FLASHӦ�ó���!\r\n");
				}
 			}else 
			{
				printf("û�п��Ը��µĹ̼�!\r\n");
			}
			clearflag=7;//��־��������ʾ,��������7*300ms�������ʾ									 
		}
		if(key==KEY2_PRES)
		{
			if(applenth)
			{																	 
				printf("�̼�������!\r\n");    
				applenth=0;
			}else 
			{
				printf("û�п�������Ĺ̼�!\r\n");
			}
			clearflag=7;//��־��������ʾ,��������7*300ms�������ʾ									 
		}
		if(key==KEY1_PRES)
		{
			printf("��ʼִ��FLASH�û�����!!\r\n");
			if(((*(vu32*)(FLASH_APP1_ADDR+4))&0xFF000000)==0x08000000)//�ж��Ƿ�Ϊ0X08XXXXXX.
			{	 
				iap_load_app(FLASH_APP1_ADDR);//ִ��FLASH APP����
			}else 
			{
				printf("��FLASHӦ�ó���,�޷�ִ��!\r\n");	   
			}									 
			clearflag=7;//��־��������ʾ,��������7*300ms�������ʾ	  
		}
		if(key==KEY0_PRES)
		{
			printf("��ʼִ��SRAM�û�����!!\r\n");
			if(((*(vu32*)(0X20001000+4))&0xFF000000)==0x20000000)//�ж��Ƿ�Ϊ0X20XXXXXX.
			{	 
				iap_load_app(0X20001000);//SRAM��ַ
			}else 
			{
				printf("��SRAMӦ�ó���,�޷�ִ��!\r\n");	   
			}									 
			clearflag=7;//��־��������ʾ,��������7*300ms�������ʾ	 
		}				   
		 
	}   	   
}













