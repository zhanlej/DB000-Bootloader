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
//外置flash
#include "w25qxx.h"	
//GPRS模块
#include "interface.h"
#include "serialportAPI.h"
#include "sim800C.h"
//蜂鸣器
#include "buzzer.h"
 
 
/************************************************
 ALIENTEK战舰STM32开发板实验47
 IAP实验 Bootloader V1.0 代码 
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/


char http_buf[2];	//GPRS模块通过http协议获取的数据

//debug value

void NVIC_Configuration(void)  //中断优先级NVIC设置
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
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;     //GPRS模块POWERKEY
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_0); //PB0上电低电平
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;     //GPRS模块VBAT
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, GPIO_Pin_1); //PA1上电低电平
	
	USART1Conf(baudRate, 0, 0);
}

int main(void)
{		
	u32 firmware_flag = 0;
	int download_times = 0;
	
	//for debug

  NVIC_Configuration();
	delay_init();	   	 	//延时初始化 
#ifndef ACTIVE_BEEP
	TIM1_Int_Init();			//打开定时器TIM1，产生无源蜂鸣器的PWM
#endif
	TIM2_Init();					//每1ms中断一次的定时器，用来记录时间
	UartBegin(115200, &GPRS_USART, &U1_PutChar);				//串口1配置
	USART2Conf(9600, 0, 1);
	W25QXX_Init();			//W25QXX初始化
	
	delay(1000);
	
	printf("IAP TEST\r\n");
	printf("\r\n########### 烧录日期: "__DATE__" - "__TIME__"\r\n");
	
	//for debug
	//while(1);
	
	while((W25QXX_ReadID())!=W25Q32)								//检测不到W25Q32
	{
		printf("W25Q32 Check Failed!\r\n");
		printf("Please Check!\r\n");
		while(1) beep_on(1000);
	}
	
	//根据firmware_flag的值判断是否需要更新固件。
	firmware_flag = Flash_Read_Number(FLASH_FIRMWARE_FLAG);
	printf("firmware_flag = %d\r\n", firmware_flag);
	if(firmware_flag == 1) printf("直接跳入APP\r\n");
	else if(firmware_flag == 2) //等于2说明更新中途失败，将备份的固件烧写到FLASH中
	{
		APP_W25qxx_to_Flash(W25X_OLD_FIRMWARE_ADDR, FLASH_APP1_ADDR, FLASH_APP1_SIZE);
	}
	else  //否则准备更新，先将原来的固件进行备份，然后将firmware_flag设置为2
	{
		APP_Flash_to_W25qxx(FLASH_APP1_ADDR, W25X_OLD_FIRMWARE_ADDR, FLASH_APP1_SIZE);
		Flash_Write_Number(2, FLASH_FIRMWARE_FLAG);
		while(0 == GSMInit(HOST_NAME, HOST_PORT, http_buf)) 
		{
			if(download_times >= 3)	
			{
				printf("download_times = %d\r\n", download_times);
				//重启MCU
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















