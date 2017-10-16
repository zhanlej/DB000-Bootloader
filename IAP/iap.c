#include "sys.h"
#include "delay.h"
#include "uart.h"
#include "stmflash.h"
#include "iap.h"
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

void Jump_to_App()
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
		
		Flash_Write_Number(1, FLASH_FIRMWARE_FLAG);
		iap_load_app(FLASH_APP1_ADDR);//执行FLASH APP代码
	}else 
	{
		printf("非FLASH应用程序,无法执行!\r\n");	   
	}		
}












