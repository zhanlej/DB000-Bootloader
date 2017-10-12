#ifndef __UART_H_
#define __UART_H_

#include "stm32f10x.h"
#include "stdio.h"

#define USART_REC_LEN  			1*1024 //�����������ֽ��� 55K
#define DBG_USART2 1

extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
extern u16 USART_RX_CNT;				//���յ��ֽ���	 

void USART1Conf(u32 baudRate, u32 nvicPre, u32 nvicSub);
void U1_PutChar(u8 Data);
void U1_PutNChar(u8 *buf , u16 size);
void U1_PutStr(char *str);
void USART2Conf(u32 baudRate, u32 nvicPre, u32 nvicSub);
void U2_PutChar(u8 Data);
void U2_PutNChar(u8 *buf , u16 size);
void U2_PutStr(char *str);
void USART3Conf(u32 baudRate, u32 nvicPre, u32 nvicSub);
void U3_PutChar(u8 Data);
void U3_PutNChar(u8 *buf , u16 size);
void U3_PutStr(char *str);
void U2_PutDbgStrln(char *str);
void U3_PutDbgStrln(char *str);
#endif
