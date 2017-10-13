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
//GPRSģ��
#include "interface.h"
#include "serialportAPI.h"
#include "sim800C.h"
 
 
/************************************************
 ALIENTEKս��STM32������ʵ��47
 IAPʵ�� Bootloader V1.0 ���� 
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/


char http_buf[2];	//GPRSģ��ͨ��httpЭ���ȡ������

//debug value
char flash_buf[10];


void NVIC_Configuration(void)  //�ж����ȼ�NVIC����
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�0��
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�3��
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
  NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

#ifndef SWIO_DEBUG	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	// �ı�ָ���ܽŵ�ӳ�� GPIO_Remap_SWJ_Disable SWJ ��ȫ���ã�JTAG+SW-DP��
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	// �ı�ָ���ܽŵ�ӳ�� GPIO_Remap_SWJ_JTAGDisable ��JTAG-DP ���� + SW-DP ʹ��
#endif
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;     //�źŵ�--PB3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;     //������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //GPIO_SetBits(GPIOB, GPIO_Pin_14);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;     //LED1����--PB5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_5);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;     //LED2����--PB6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_6);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;     //LED3����--PB7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_7);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;     //LED4����--PB8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_8);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			//LED5����--PB1
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//�������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_9);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;     //2����������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;     //������
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  GPIO_SetBits(GPIOB, GPIO_Pin_13);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;     //������
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //��������
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  //GPIO_SetBits(GPIOB, GPIO_Pin_14);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;     //A0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;     //A4
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;     //A5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;     //A6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;     //A7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;     //GPRSģ��POWERKEY
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_0); //PB0�ϵ�͵�ƽ
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;     //GPRSģ��VBAT
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, GPIO_Pin_1); //PA1�ϵ�͵�ƽ

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;   //USART1 TX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;   //USART1 RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //USART2 TX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;     //A2
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //��������
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;      //USART2 RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//USART3 TX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//��������ٶ�50MHz
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//���츴�����
  GPIO_Init(GPIOB, &GPIO_InitStructure); //����ʼ���õĽṹ��װ��Ĵ���

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//USART3 RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIOģʽ��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);//����ʼ���õĽṹ��װ��Ĵ���

}

void GPRS_USART(u32 baudRate)
{
	USART1Conf(baudRate, 0, 0);
}

#define APP_TEMP_LEN 1024
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
	int app_temp_len = 0;
	u8 app_temp[APP_TEMP_LEN];
	int i = 0;

  NVIC_Configuration();
	//GPIO_Configuration();
	//uart_init(115200);	//���ڳ�ʼ��Ϊ115200
	UartBegin(115200, &GPRS_USART, &U1_PutChar);				//����1����
	USART2Conf(115200, 0, 1);
	TIM2_Init();					//ÿ1ms�ж�һ�εĶ�ʱ����������¼ʱ��
	delay_init();	   	 	//��ʱ��ʼ�� 
 	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();					//��ʼ������
	W25QXX_Init();			//W25QXX��ʼ��
	GPIO_Configuration();
	
	delay(1000);
	
	printf("test\r\n");
	while((W25QXX_ReadID())!=W25Q32)								//��ⲻ��W25Q32
	{
		printf("W25Q32 Check Failed!\r\n");
		delay_ms(500);
		printf("Please Check!\r\n");
		delay_ms(500);
		LED0=!LED0;//DS0��˸
	}
	
	while(0 == GSMInit(HOST_NAME, HOST_PORT, http_buf)) GSM_restart();
	
					
 
	while(1)
	{
		//printf("USART_RX_CNT = %d\r\n", USART_RX_CNT);
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
				W25QXX_Read((u8 *)flash_buf, 0, 10);					//�ӵ�0����ַ����ʼ,����10���ֽ�
				printf("flash_buf = %s\r\n",flash_buf);
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
		//key=KEY_Scan(0);
		key=111;
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















