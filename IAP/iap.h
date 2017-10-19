#ifndef __IAP_H__
#define __IAP_H__
#include "sys.h"  
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
typedef  void (*iapfun)(void);				//定义一个函数类型的参数.

//////////////////////////////////////////////////////////////////////////////////
//#define IAP_DEBUG 1

#define FLASH_APP1_ADDR		0x08005000  	//第一个应用程序起始地址(存放在FLASH)
											//保留0X08000000~0X08004FFF的空间为IAP使用
#define FLASH_APP1_SIZE 44*1024
#define APP_TEMP_LEN 1024
#define FIRMWARE_PACK_SIZE 1024
////////////////////////////////////////////////////////////////////////////
#define W25X_NEW_FIRMWARE_ADDR 0
#define W25X_OLD_FIRMWARE_ADDR 0x100000
////////////////////////////////////////////////////////////////////////////

extern u8 firmware_packet[FIRMWARE_PACK_SIZE];	//保存每包固件的内容

void iap_load_app(u32 appxaddr);			//执行flash里面的app程序
void iap_load_appsram(u32 appxaddr);		//执行sram里面的app程序
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 applen);	//在指定地址开始,写入bin
void iap_read_appbin(u32 appxaddr,u8 *appbuf,u32 appsize);	//在指定地址开始,读取内容
void Jump_to_App(void);
int APP_W25qxx_to_Flash(u32 w25_addr, u32 app_addr, u32 app_sum_len);
int APP_Flash_to_W25qxx(u32 app_addr, u32 w25qxx_addr, u32 app_sum_len);
#endif







































