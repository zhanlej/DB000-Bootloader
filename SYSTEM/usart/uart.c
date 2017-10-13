#include "uart.h"


u8 USART_RX_BUF[USART_REC_LEN];//���ջ���,���USART_REC_LEN���ֽ�,��ʼ��ַΪ0X20001000.    
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       	//����״̬���	  
u16 USART_RX_CNT=0;			//���յ��ֽ���	   

#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
#ifdef DBG_USART1
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
#elif DBG_USART2
	while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
	USART2->DR = (u8) ch;  
#elif DBG_USART3
	while((USART3->SR&0X40)==0);//ѭ������,ֱ���������   
	USART3->DR = (u8) ch;  
#endif
	return ch;
}
#endif 

//UART1 function
//UART1 TxD GPIOA9   RxD GPIOA10
void USART1Conf(u32 baudRate, u32 nvicPre, u32 nvicSub)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //ʹ��USART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//GPIOAʱ��
	
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10 

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=nvicPre ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = nvicSub;		//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	//USART1 Configure	
	USART_InitStructure.USART_BaudRate = baudRate;//������19200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//���ݿ��8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//ʹ�ܷ��������
	USART_Init(USART1 , &USART_InitStructure);//����ʼ���õĽṹ��װ��Ĵ���	
	
	//USART1_INT Configure
	USART_ITConfig(USART1 , USART_IT_RXNE , ENABLE);//ʹ�ܽ����ж�
	USART_Cmd(USART1 , ENABLE);//�򿪴���
	USART_ClearFlag(USART1 , USART_FLAG_TC);//�����һ�����ݷ���ʧ�ܵ�����
}

void U1_PutChar(u8 Data)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);//�ȴ��������
	USART_SendData(USART1 , Data);
}
void U1_PutStr(char *str)//����һ���ַ���
{
	while(*str != '\0')
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);//�ȴ��������
		USART_SendData(USART1 , *str++);
	}
}

void U1_PutNChar(u8 *buf , u16 size)
{
  u8 i;
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); //��ֹ��һ�ֽڶ�ʧ
	for(i=0;i<size;i++)
	{
		 USART_SendData(USART1 , buf[i]);
		 while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);//�ȴ��������
	}
}

//UART2 function
//UART2 TxD GPIOA2   RxD GPIOA3
void USART2Conf(u32 baudRate, u32 nvicPre, u32 nvicSub)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //ʹ��USART2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//GPIOAʱ��
	
	//USART2_TX   GPIOA.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.2
   
  //USART2_RX	  GPIOA.3��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.3  

  //Usart2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=nvicPre ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = nvicSub;		//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	//USART2 Configure	
	USART_InitStructure.USART_BaudRate = baudRate;//������19200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//���ݿ��8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//ʹ�ܷ��������
	USART_Init(USART2 , &USART_InitStructure);//����ʼ���õĽṹ��װ��Ĵ���	
	
	//USART2_INT Configure
	USART_ITConfig(USART2 , USART_IT_RXNE , ENABLE);//ʹ�ܽ����ж�
	USART_Cmd(USART2 , ENABLE);//�򿪴���
	USART_ClearFlag(USART2 , USART_FLAG_TC);//�����һ�����ݷ���ʧ�ܵ�����
}

void U2_PutChar(u8 Data)
{
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//�ȴ��������
	USART_SendData(USART2 , Data);
}
void U2_PutStr(char *str)//����һ���ַ���
{
	while(*str != '\0')
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//�ȴ��������
		USART_SendData(USART2 , *str++);
	}
}

void U2_PutNChar(u8 *buf , u16 size)
{
  u8 i;
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); //��ֹ��һ�ֽڶ�ʧ
	for(i=0;i<size;i++)
	{
		 USART_SendData(USART2 , buf[i]);
		 while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//�ȴ��������
	}
}

//UART1 function
//UART1 TxD GPIOB10   RxD GPIOB11
void USART3Conf(u32 baudRate, u32 nvicPre, u32 nvicSub)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //ʹ��USART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//GPIOAʱ��
	
	//USART3_TX   GPIOB.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PA.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOA.10
   
  //USART3_RX	  GPIOB.11��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOA.10 

  //Usart3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=nvicPre ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = nvicSub;		//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	//USART3 Configure	
	USART_InitStructure.USART_BaudRate = baudRate;//������19200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//���ݿ��8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//ʹ�ܷ��������
	USART_Init(USART3 , &USART_InitStructure);//����ʼ���õĽṹ��װ��Ĵ���	
	
	//USART3_INT Configure
	USART_ITConfig(USART3 , USART_IT_RXNE , ENABLE);//ʹ�ܽ����ж�
	USART_Cmd(USART3 , ENABLE);//�򿪴���
	USART_ClearFlag(USART3 , USART_FLAG_TC);//�����һ�����ݷ���ʧ�ܵ�����
}

void U3_PutChar(u8 Data)
{
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);//�ȴ��������
	USART_SendData(USART3 , Data);
}
void U3_PutStr(char *str)//����һ���ַ���
{
	while(*str != '\0')
	{
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);//�ȴ��������
		USART_SendData(USART3 , *str++);
	}
}

void U3_PutNChar(u8 *buf , u16 size)
{
  u8 i;
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET); //��ֹ��һ�ֽڶ�ʧ
	for(i=0;i<size;i++)
	{
		 USART_SendData(USART3 , buf[i]);
		 while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);//�ȴ��������
	}
}

void U2_PutDbgStrln(char *str)//����һ���ַ���������
{
	while(*str != '\0')
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//�ȴ��������
		USART_SendData(USART2 , *str++);
	}		
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//�ȴ��������
	USART_SendData(USART2 , '\r');
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//�ȴ��������
	USART_SendData(USART2 , '\n');
}

void U3_PutDbgStrln(char *str)//����һ���ַ���������
{
	while(*str != '\0')
	{
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);//�ȴ��������
		USART_SendData(USART3 , *str++);
	}		
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);//�ȴ��������
	USART_SendData(USART3 , '\r');
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);//�ȴ��������
	USART_SendData(USART3 , '\n');
}


