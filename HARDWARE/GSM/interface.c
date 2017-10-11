#include "interface.h"

//void delay_init(void)
//{
//   SysTick->CTRL&=0xfffffffb;//控制寄存器，选择外部时钟即系统时钟的八分之一（HCLK/8；72M/8=9M）
//}

//1us 延时函数
void Delay_us(u32 Nus)   
{   
SysTick->LOAD=Nus*9;          //时间加载    72M主频     
SysTick->CTRL|=0x01;             //开始倒数      
while(!(SysTick->CTRL&(1<<16))); //等待时间到达   
SysTick->CTRL=0X00000000;        //关闭计数器   
SysTick->VAL=0X00000000;         //清空计数器        
} 

void UserLEDInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG , ENABLE);
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//配置GPIO端口速度
	GPIO_Init(LED_GPIO , &GPIO_InitStructure);
	
	LED_SET;	
}

/**-------------------------------------------------------
  * @函数名 NVIC_TIM5Configuration
  * @功能   配置TIM5中断向量参数函数
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
//static void NVIC_TIM2Configuration(void)
//{ 
//    NVIC_InitTypeDef NVIC_InitStructure;

//    /* Set the Vector Table base address at 0x08000000 */
//    //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
//}


void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* Time base configuration */
    //这个就是自动装载的计数值，由于计数是从0开始的，周期为1000us
    TIM_TimeBaseStructure.TIM_Period = (1000 - 1);//1kHz
    // 这个就是预分频系数，当由于为0时表示不分频所以要减1
    TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);//1MHz
    // 高级应用本次不涉及。定义在定时器时钟(CK_INT)频率与数字滤波器(ETR,TIx)
    // 使用的采样频率之间的分频比例
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    //向上计数
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //初始化定时器5
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* Clear TIM5 update pending flag[清除TIM2溢出中断标志] */
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    /* TIM IT enable */ //打开溢出中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* TIM5 enable counter */
    TIM_Cmd(TIM2, ENABLE);  //计数器使能，开始工作

    /* 中断参数配置 */
    //NVIC_TIM2Configuration();
}

void LEDToggle(uint16_t Led)
{
    /* 指定管脚输出异或 1，实现对应的LED指示灯状态取反目的 */
	LED_GPIO->ODR ^= Led;
	//若要提高效率，建议直接调用 LEDnOBB = !LEDnOBB;
}


