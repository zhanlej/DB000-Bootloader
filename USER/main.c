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

int main(void)
{		
	u32 firmware_flag = 0;
	
	//add for debug

  NVIC_Configuration();
	//GPIO_Configuration();
	UartBegin(115200, &GPRS_USART, &U1_PutChar);				//����1����
	USART2Conf(9600, 0, 1);
	TIM2_Init();					//ÿ1ms�ж�һ�εĶ�ʱ����������¼ʱ��
	delay_init();	   	 	//��ʱ��ʼ�� 
	W25QXX_Init();			//W25QXX��ʼ��
	GPIO_Configuration();
	
	delay(1000);
	
	printf("IAP TEST\r\n");
	
	//����firmware_flag��ֵ�ж��Ƿ���Ҫ���¹̼���
	firmware_flag = Flash_Read_Number(FLASH_FIRMWARE_FLAG);
	printf("firmware_flag = %d\r\n", firmware_flag);
	if(firmware_flag == 1) Jump_to_App();
	Flash_Write_Number(0, FLASH_FIRMWARE_FLAG);
	firmware_flag = Flash_Read_Number(FLASH_FIRMWARE_FLAG);
	printf("firmware_flag = %d\r\n", firmware_flag);
	
	while((W25QXX_ReadID())!=W25Q32)								//��ⲻ��W25Q32
	{
		printf("W25Q32 Check Failed!\r\n");
		delay_ms(500);
		printf("Please Check!\r\n");
		delay_ms(500);
		LED0=!LED0;//DS0��˸
	}
	
	while(0 == GSMInit(HOST_NAME, HOST_PORT, http_buf)) GSM_restart();
	
	Jump_to_App();		
	   
}















