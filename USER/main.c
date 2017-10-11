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

#define APP_TEMP_LEN 5120
u8 app_temp[APP_TEMP_LEN];
int app_temp_len = 0;
char http_buf[512];	//GPRS模块通过http协议获取的数据


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

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
  NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
}

void GPRS_USART(u32 baudRate)
{
	USART1Conf(baudRate, 0, 0);
}

int main(void)
{		
	u8 t;
	u8 key;
	u16 oldcount=0;				//老的串口接收数据值
	u16 applenth=0;				//接收到的app代码长度
	u8 clearflag=0;  
	u8 appcheck[4];				//用来检查收到的数据是否是app数据
	int appremain = 0;				//记录剩余多少数据还没
	int app_off = 0;
	int i = 0;

  NVIC_Configuration();
	//uart_init(115200);	//串口初始化为115200
	UartBegin(115200, &GPRS_USART, &U1_PutChar);				//串口1配置
	USART2Conf(115200, 0, 1);
	TIM2_Init();					//每1ms中断一次的定时器，用来记录时间
	delay_init();	   	 	//延时初始化 
 	LED_Init();		  			//初始化与LED连接的硬件接口
	KEY_Init();					//初始化按键
	W25QXX_Init();			//W25QXX初始化
	
	printf("test\r\n");
	while(0 == GSMInit(HOST_NAME, HOST_PORT, http_buf)) GSM_restart();
	while(W25QXX_ReadID()!=W25Q128)								//检测不到W25Q128
	{
		printf("W25Q128 Check Failed!\r\n");
		delay_ms(500);
		printf("Please Check!\r\n");
		delay_ms(500);
		LED0=!LED0;//DS0闪烁
	}
 
	while(1)
	{
	 	if(USART_RX_CNT)
		{
			if(oldcount==USART_RX_CNT)//新周期内,没有收到任何数据,认为本次数据接收完成.
			{
				applenth=USART_RX_CNT;
				oldcount=0;
				USART_RX_CNT=0;
				printf("用户程序接收完成!\r\n");
				printf("代码长度:%dBytes\r\n",applenth);
				printf("Start Write W25Q128....\r\n"); 
				W25QXX_Write((u8*)USART_RX_BUF, 0, applenth);			//从第0个地址处开始写入数据
				printf("W25Q128 Write Finished!\r\n");	//提示传送完成
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
				printf("开始更新固件...\r\n");	
				W25QXX_Read(appcheck,4,4);					//从第4个地址处开始,读出4个字节
				printf("appcheck = %02x%02x%02x%02x\r\n",appcheck[3],appcheck[2],appcheck[1],appcheck[0]);

				//printf("(*(vu32*)(0X20001000+4)) = %x\n", (*(vu32*)(0X20001000+4)));
 				//if(((*(vu32*)(0X20001000+4))&0xFF000000)==0x08000000)//判断是否为0X08XXXXXX.
				if(appcheck[3] == 0x08)//判断是否为0X08XXXXXX.
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
						iap_write_appbin(FLASH_APP1_ADDR+app_off, app_temp, app_temp_len);//更新FLASH代码   
						memset(app_temp, 0, APP_TEMP_LEN);
						
						i++;
					}
					printf("固件更新完成!\r\n");	
				}else 
				{	   
					printf("非FLASH应用程序!\r\n");
				}
 			}else 
			{
				printf("没有可以更新的固件!\r\n");
			}
			clearflag=7;//标志更新了显示,并且设置7*300ms后清除显示									 
		}
		if(key==KEY2_PRES)
		{
			if(applenth)
			{																	 
				printf("固件清除完成!\r\n");    
				applenth=0;
			}else 
			{
				printf("没有可以清除的固件!\r\n");
			}
			clearflag=7;//标志更新了显示,并且设置7*300ms后清除显示									 
		}
		if(key==KEY1_PRES)
		{
			printf("开始执行FLASH用户代码!!\r\n");
			if(((*(vu32*)(FLASH_APP1_ADDR+4))&0xFF000000)==0x08000000)//判断是否为0X08XXXXXX.
			{	 
				iap_load_app(FLASH_APP1_ADDR);//执行FLASH APP代码
			}else 
			{
				printf("非FLASH应用程序,无法执行!\r\n");	   
			}									 
			clearflag=7;//标志更新了显示,并且设置7*300ms后清除显示	  
		}
		if(key==KEY0_PRES)
		{
			printf("开始执行SRAM用户代码!!\r\n");
			if(((*(vu32*)(0X20001000+4))&0xFF000000)==0x20000000)//判断是否为0X20XXXXXX.
			{	 
				iap_load_app(0X20001000);//SRAM地址
			}else 
			{
				printf("非SRAM应用程序,无法执行!\r\n");	   
			}									 
			clearflag=7;//标志更新了显示,并且设置7*300ms后清除显示	 
		}				   
		 
	}   	   
}















