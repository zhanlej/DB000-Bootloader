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
//������
#include "buzzer.h"
 
 
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
}

void GPRS_USART(u32 baudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;     //GPRSģ��POWERKEY
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_0); //PB0�ϵ�͵�ƽ
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;     //GPRSģ��VBAT
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //�������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, GPIO_Pin_1); //PA1�ϵ�͵�ƽ
	
	USART1Conf(baudRate, 0, 0);
}

int main(void)
{		
	u32 firmware_flag = 0;
	int download_times = 0;
	
	//for debug

  NVIC_Configuration();
	delay_init();	   	 	//��ʱ��ʼ�� 
#ifndef ACTIVE_BEEP
	TIM1_Int_Init();			//�򿪶�ʱ��TIM1��������Դ��������PWM
#endif
	TIM2_Init();					//ÿ1ms�ж�һ�εĶ�ʱ����������¼ʱ��
	UartBegin(115200, &GPRS_USART, &U1_PutChar);				//����1����
	USART2Conf(9600, 0, 1);
	W25QXX_Init();			//W25QXX��ʼ��
	
	delay(1000);
	
	printf("IAP TEST\r\n");
	printf("\r\n########### ��¼����: "__DATE__" - "__TIME__"\r\n");
	
	//for debug
	//while(1);
	
	while((W25QXX_ReadID())!=W25Q32)								//��ⲻ��W25Q32
	{
		printf("W25Q32 Check Failed!\r\n");
		printf("Please Check!\r\n");
		while(1) beep_on(1000);
	}
	
	//����firmware_flag��ֵ�ж��Ƿ���Ҫ���¹̼���
	firmware_flag = Flash_Read_Number(FLASH_FIRMWARE_FLAG);
	printf("firmware_flag = %d\r\n", firmware_flag);
	if(firmware_flag == 1) printf("ֱ������APP\r\n");
	else if(firmware_flag == 2) //����2˵��������;ʧ�ܣ������ݵĹ̼���д��FLASH��
	{
		APP_W25qxx_to_Flash(W25X_OLD_FIRMWARE_ADDR, FLASH_APP1_ADDR, FLASH_APP1_SIZE);
	}
	else  //����׼�����£��Ƚ�ԭ���Ĺ̼����б��ݣ�Ȼ��firmware_flag����Ϊ2
	{
		APP_Flash_to_W25qxx(FLASH_APP1_ADDR, W25X_OLD_FIRMWARE_ADDR, FLASH_APP1_SIZE);
		Flash_Write_Number(2, FLASH_FIRMWARE_FLAG);
		while(0 == GSMInit(HOST_NAME, HOST_PORT, http_buf)) 
		{
			if(download_times >= 3)	
			{
				printf("download_times = %d\r\n", download_times);
				//����MCU
				__disable_fault_irq();   
				NVIC_SystemReset();
				while(1);
			}
			else download_times++;
			
			GSM_restart();
		}
	}
	
	Jump_to_App();		
	   
}















