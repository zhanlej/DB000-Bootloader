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
char flash_buf[10];


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

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

#ifndef SWIO_DEBUG	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	// 改变指定管脚的映射 GPIO_Remap_SWJ_Disable SWJ 完全禁用（JTAG+SW-DP）
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	// 改变指定管脚的映射 GPIO_Remap_SWJ_JTAGDisable ，JTAG-DP 禁用 + SW-DP 使能
#endif
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;     //信号灯--PB3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;     //物理按键
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //上拉输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //GPIO_SetBits(GPIOB, GPIO_Pin_14);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;     //LED1控制--PB5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_5);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;     //LED2控制--PB6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_6);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;     //LED3控制--PB7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_7);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;     //LED4控制--PB8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_8);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			//LED5控制--PB1
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//复用输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_9);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;     //2空气质量灯
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;     //蜂鸣器
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  GPIO_SetBits(GPIOB, GPIO_Pin_13);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;     //物理按键
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //上拉输入
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  //GPIO_SetBits(GPIOB, GPIO_Pin_14);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;     //A0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //上拉输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;     //A4
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;     //A5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;     //A6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;     //A7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;     //GPRS模块POWERKEY
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_0); //PB0上电低电平
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;     //GPRS模块VBAT
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, GPIO_Pin_1); //PA1上电低电平

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;   //USART1 TX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;   //USART1 RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //USART2 TX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;     //A2
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //上拉输入
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;      //USART2 RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//USART3 TX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//设置最高速度50MHz
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//推挽复用输出
  GPIO_Init(GPIOB, &GPIO_InitStructure); //将初始化好的结构体装入寄存器

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//USART3 RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO模式悬浮输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);//将初始化好的结构体装入寄存器

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
	UartBegin(115200, &GPRS_USART, &U1_PutChar);				//串口1配置
	USART2Conf(9600, 0, 1);
	TIM2_Init();					//每1ms中断一次的定时器，用来记录时间
	delay_init();	   	 	//延时初始化 
	W25QXX_Init();			//W25QXX初始化
	GPIO_Configuration();
	
	delay(1000);
	
	printf("IAP TEST\r\n");
	printf("\r\n########### 烧录日期: "__DATE__" - "__TIME__"\r\n");
	
	//for debug
	//while(1);
	
	while((W25QXX_ReadID())!=W25Q32)								//检测不到W25Q32
	{
		printf("W25Q32 Check Failed!\r\n");
		printf("Please Check!\r\n");
		while(1);
	}
	
	//根据firmware_flag的值判断是否需要更新固件。
	firmware_flag = Flash_Read_Number(FLASH_FIRMWARE_FLAG);
	printf("firmware_flag = %d\r\n", firmware_flag);
	if(firmware_flag == 1) printf("直接跳入APP\r\n");
	else if(firmware_flag == 2) //等于2说明更新中途失败，将备份的固件烧写到FLASH中
	{
		APP_W25qxx_to_Flash(W25X_OLD_FIRMWARE_ADDR, FLASH_APP1_ADDR, FLASH_APP1_SIZE);
		Jump_to_App();
	}
	else  //否则准备更新，先将原来的固件进行备份，然后将firmware_flag设置为2
	{
		APP_Flash_to_W25qxx(FLASH_APP1_ADDR, W25X_OLD_FIRMWARE_ADDR, FLASH_APP1_SIZE);
		Flash_Write_Number(2, FLASH_FIRMWARE_FLAG);
		while(0 == GSMInit(HOST_NAME, HOST_PORT, http_buf)) GSM_restart();
	}
	
	Jump_to_App();		
	   
}















