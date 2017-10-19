#include "string.h"
#include "sys.h"
#include "delay.h"
#include "uart.h"
#include "stmflash.h"
#include "iap.h"
#include "w25qxx.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//IAP 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/24
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////	

//全局变量
u8 app_temp[APP_TEMP_LEN]; //for write to app
u8 firmware_packet[FIRMWARE_PACK_SIZE];	//保存每包固件的内容

iapfun jump2app; 
u16 iapbuf[1024];   
//appxaddr:应用程序的起始地址
//appbuf:应用程序CODE.
//appsize:应用程序大小(字节).
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 appsize)
{
	u16 t;
	u16 i=0;
	u16 temp;
	u32 fwaddr=appxaddr;//当前写入的地址
	u8 *dfu=appbuf;
	for(t=0;t<appsize;t+=2)
	{						    
		temp=(u16)dfu[1]<<8;
		temp+=(u16)dfu[0];	  
		dfu+=2;//偏移2个字节
		iapbuf[i++]=temp;	    
		if(i==1024)
		{
			i=0;
			STMFLASH_Write(fwaddr,iapbuf,1024);	
			fwaddr+=2048;//偏移2048  16=2*8.所以要乘以2.
		}
	}
	if(i)STMFLASH_Write(fwaddr,iapbuf,i);//将最后的一些内容字节写进去.  
}

void iap_read_appbin(u32 appxaddr,u8 *appbuf,u32 appsize)
{
	STMFLASH_Read(appxaddr,(u16*)appbuf,appsize/2);
}

//跳转到应用程序段
//appxaddr:用户代码起始地址.
void iap_load_app(u32 appxaddr)
{
	if(((*(vu32*)appxaddr)&0x2FFE0000)==0x20000000)	//检查栈顶地址是否合法.
	{ 
		jump2app=(iapfun)*(vu32*)(appxaddr+4);		//用户代码区第二个字为程序开始地址(复位地址)		
		MSR_MSP(*(vu32*)appxaddr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		jump2app();									//跳转到APP.
	}
}		 

void Jump_to_App(void)
{
	printf("开始执行FLASH用户代码!!\r\n");
	if(((*(vu32*)(FLASH_APP1_ADDR+4))&0xFF000000)==0x08000000)//判断是否为0X08XXXXXX.
	{	 
		__disable_irq();
		USART_ITConfig(USART1 , USART_IT_RXNE , DISABLE);//失能接收中断
		USART_Cmd(USART1 , DISABLE);//关闭串口
		USART_ITConfig(USART2 , USART_IT_RXNE , DISABLE);//失能接收中断
		USART_Cmd(USART2 , DISABLE);//关闭串口
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM2, DISABLE);  //计数器失能
		SPI_Cmd(SPI2, DISABLE); //失能SPI外设
		
		iap_load_app(FLASH_APP1_ADDR);//执行FLASH APP代码
	}else 
	{
		printf("非FLASH应用程序,无法执行!\r\n");	   
	}		
}

int APP_W25qxx_to_Flash(u32 w25qxx_addr, u32 app_addr, u32 app_sum_len)
{
u8 appcheck[4];				//用来检查收到的数据是否是app数据
u32 appremain = 0;
u32 app_off = 0;
u32 app_temp_len = 0;
	
#ifdef IAP_DEBUG
int i=0, j=0;
#endif

	/*开始往片内FLASH写APP代码*/
	if(app_sum_len)
	{
		appremain = app_sum_len;
		printf("开始更新固件...\r\n");	
		printf("w25qxx_addr = 0x%08x, app_addr = 0x%08x, app_sum_len = %d\r\n", w25qxx_addr, app_addr, app_sum_len);
		W25QXX_Read(appcheck,w25qxx_addr+4,4);					//从第4个地址处开始,读出4个字节
		printf("appcheck = %02x%02x%02x%02x\r\n",appcheck[3],appcheck[2],appcheck[1],appcheck[0]);

		if(appcheck[3] == 0x08)//判断是否为0X08XXXXXX.
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
				//检查外置FLASH中的数据是否正确
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
				iap_write_appbin(app_addr + app_off, app_temp, APP_TEMP_LEN);//更新FLASH代码   
				
#ifdef IAP_DEBUG
				//检查外内FLASH中的数据是否正确
				iap_read_appbin(app_addr + app_off, app_temp, APP_TEMP_LEN);
				printf("\r\n片内FLASH check\r\n");
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
			printf("固件更新完成!\r\n");	
		}else 
		{	   
			printf("非FLASH应用程序!\r\n");
			return 0;
		}
	}else 
	{
		printf("没有可以更新的固件!\r\n");
		return 0;
	}
	
	return 1;
}

int APP_Flash_to_W25qxx(u32 app_addr, u32 w25qxx_addr, u32 app_sum_len)
{
	u8 appcheck[4];				//用来检查收到的数据是否是app数据
	u32 appremain = 0;
	u32 app_off = 0;
	u32 app_temp_len = 0;
	
#ifdef IAP_DEBUG
int i=0, j=0;
#endif

	/*开始往片内FLASH写APP代码*/
	if(app_sum_len)
	{
		appremain = app_sum_len;
		printf("开始备份固件...\r\n");	
		printf("w25qxx_addr = 0x%08x, app_addr = 0x%08x, app_sum_len = %d\r\n", w25qxx_addr, app_addr, app_sum_len);
		//W25QXX_Read(appcheck,w25qxx_addr+4,4);					//从第4个地址处开始,读出4个字节
		iap_read_appbin(app_addr+4, appcheck, 4);
		printf("appcheck = %02x%02x%02x%02x\r\n",appcheck[3],appcheck[2],appcheck[1],appcheck[0]);

		if(appcheck[3] == 0x08)//判断是否为0X08XXXXXX.
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
				//检查外置FLASH中的数据是否正确
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
				//iap_write_appbin(app_addr + app_off, app_temp, APP_TEMP_LEN);//更新FLASH代码   
				W25QXX_Write(app_temp, w25qxx_addr + app_off, APP_TEMP_LEN);//备份到w25qxx中
				
#ifdef IAP_DEBUG
				//检查外内FLASH中的数据是否正确
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
			printf("固件备份完成!\r\n");	
		}else 
		{	   
			printf("非FLASH应用程序!\r\n");
			return 0;
		}
	}else 
	{
		printf("没有可以备份的固件!\r\n");
		return 0;
	}
	
	return 1;
}








