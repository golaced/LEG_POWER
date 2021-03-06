
#include "pwm_in.h"
#include "pwm_out.h"
#include "include.h"
#include "my_math.h"

//21分频到 84000000/21 = 4M   0.25us
u32 Rc_Pwm_Inr_mine[8];
u32 Rc_Pwm_In_mine[8],Rc_Pwm_Out_mine[8]={1500,1500,1500,1500,1500,1500,1500,1500};
PWMIN pwmin;
#define INIT_DUTY 4000 //u16(1000/0.25)
#define ACCURACY 10000 //u16(2500/0.25) //accuracy
#define PWM_RADIO 4//(8000 - 4000)/1000.0

u8 PWM_Out_Init(uint16_t hz)//400hz
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t PrescalerValue = 0;
	u32 hz_set = ACCURACY*hz;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
	
	hz_set = LIMIT (hz_set,1,84000000);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE, ENABLE);

/////////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIO_Pin_0 | GPIO_Pin_1 | 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
// TIM4 ->>云台控制
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel3 */
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC3Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel4 */
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC4Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);//--------------------------云台 舵机
/////////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  //TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);	
	////////////////////////////////////////////////////////////////////////////////////

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//  GPIO_Init(GPIOC, &GPIO_InitStructure); 

//  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
//  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
//	
//	/* Compute the prescaler value */
//  PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
//  /* Time base configuration */
//  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
//  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);


//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

//  /* PWM1 Mode configuration: Channel3 */
//  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC3Init(TIM8, &TIM_OCInitStructure);
//  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel4 */
//  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC4Init(TIM8, &TIM_OCInitStructure);
//  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	
//	TIM_CtrlPWMOutputs(TIM8, ENABLE);
//  TIM_ARRPreloadConfig(TIM8, ENABLE);
//  TIM_Cmd(TIM8, ENABLE);


	if( hz_set > 84000000 )
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

u8 CH_out_Mapping[MAXMOTORS] = {0,1,2,3};

void CH_out_Mapping_Fun(u16 *out,u16 *mapped )
{
	u8 i;
	for( i = 0 ; i < MAXMOTORS ; i++ )
	{
		*( mapped + i ) = *( out + CH_out_Mapping[i] );
	}
}
u8 sel=0;
s32 pwm_tem_view[8];
#include "filter.h"
void SetPwm(u32 pwm[8],int off[8],u32 min,u32 max)
{
	u8 i;
	s32 pwm_tem[8];
	u32 pwmr[8];
		for(i=0;i<8;i++)
	{  
			pwmr[i] = pwm[i]+off[i];
	}
	
	

	for(i=0;i<8;i++)
	{   
			pwm_tem[i] = Moving_Median(i+2,3,pwmr[i]);
		 if(i!=4||i!=5)
			pwm_tem_view[i]=pwm_tem[i] = LIMIT(pwm_tem[i],min,max);
	}
	

//	TIM1->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[0]] ) + INIT_DUTY;				//1	
//	TIM1->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[1]] ) + INIT_DUTY;				//2
//	TIM1->CCR2 = PWM_RADIO *( pwm_tem[CH_out_Mapping[2]] ) + INIT_DUTY;				//3	
//	TIM1->CCR1 = PWM_RADIO *( pwm_tem[CH_out_Mapping[3]] ) + INIT_DUTY;				//4

// 	TIM4->CCR2 = PWM_RADIO *( pwm_tem[CH_out_Mapping[0]] ) + INIT_DUTY;				//5	
// 	TIM4->CCR1 = PWM_RADIO *( pwm_tem[CH_out_Mapping[1]] ) + INIT_DUTY;				//6	
//  TIM8->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[2]] ) + INIT_DUTY;				//7	
//  TIM8->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[3]] ) + INIT_DUTY;				//8	
	TIM3->CCR1 = ( pwm_tem[0] ) ;//+ INIT_DUTY;				//1	
	TIM3->CCR2 = ( pwm_tem[1] ) ;//+ INIT_DUTY;				//2
	TIM3->CCR3 = ( pwm_tem[2] ) ;//+ INIT_DUTY;				//3	
	TIM3->CCR4 = ( pwm_tem[3] ) ;//+ INIT_DUTY;				//4

 	TIM4->CCR1 = ( pwm_tem[4] ) ;//+ INIT_DUTY;				//5	
 	TIM4->CCR2 = ( pwm_tem[5] ) ;//+ INIT_DUTY;				//6	
//  TIM8->CCR4 = ( pwm_tem[6] ) ;//+ INIT_DUTY;				//7	
//  TIM8->CCR3 = ( pwm_tem[7] ) ;//+ INIT_DUTY;				//8	
}


void LEG_POWER(u8 sel)
{
if(sel)
GPIO_SetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_2);//out5678 force up
else
GPIO_ResetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_2);//out5678 force up
}
void SEL_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;   
	GPIO_InitStructure.GPIO_Pin   =GPIO_Pin_2|GPIO_Pin_0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}




u8 PWM_Out_Init_FOR_CAL(uint16_t hz,uint16_t min,uint16_t max)//400hz
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t PrescalerValue = 0;
	u32 hz_set = ACCURACY*hz;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
	
	hz_set = LIMIT (hz_set,1,84000000);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

/////////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIO_Pin_0 | GPIO_Pin_1 | 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	
	/* Compute the prescaler value */
  PrescalerValue = 83;//(uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = hz;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel3 */
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC3Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel4 */
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC4Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
/////////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	
	/* Compute the prescaler value */
  PrescalerValue = 167;//(uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = hz;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  //TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);	
	////////////////////////////////////////////////////////////////////////////////////

//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9; 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
//  GPIO_Init(GPIOC, &GPIO_InitStructure); 
//  
//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8; 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
//  GPIO_Init(GPIOC, &GPIO_InitStructure); 
//  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
//  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
//	
//	/* Compute the prescaler value */
//  //PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
//  /* Time base configuration */
//  TIM_TimeBaseStructure.TIM_Period = hz;									
//  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);


//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

//  /* PWM1 Mode configuration: Channel3 */
//  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = min;
//  TIM_OC3Init(TIM8, &TIM_OCInitStructure);
//  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel4 */
//  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = min;
//  TIM_OC4Init(TIM8, &TIM_OCInitStructure);
//  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	
//	TIM_CtrlPWMOutputs(TIM8, ENABLE);
//  TIM_ARRPreloadConfig(TIM8, ENABLE);
//  TIM_Cmd(TIM8, ENABLE);


	if( hz_set > 84000000 )
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
