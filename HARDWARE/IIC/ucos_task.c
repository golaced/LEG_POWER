#include "include.h" 
#include "iic_soft.h"
#include "hml_sample.h"
#include "ms5611.h"
#include "ms5611_2.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "flash.h"
#include "led_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "pwm_in.h"
#include "flow.h"
#include "circle.h"
#include "eso.h"
#include "gps.h"
#include "m100.h"

float leg_dt[5];
OS_STK LEG1_TASK_STK[LEG_STK_SIZE];
void leg1_task(void *pdata)
{
 u8 i;
 static u8 init;	
	leg_init(&leg[1],1);
 	while(1)
	{
	leg_dt[0] = Get_Cycle_T(GET_T_LEG1); 						//获取内环准确的执行周期
  leg_drive(&leg[1],0.02);//leg_dt[0]);
  Send_LEG(1);

	delay_ms(20);
	}
}		

OS_STK LEG2_TASK_STK[LEG_STK_SIZE];
void leg2_task(void *pdata)
{
 u8 i;
 static u8 init;	
	leg_init(&leg[2],2);
 	while(1)
	{
	leg_dt[1] = Get_Cycle_T(GET_T_LEG2); 						//获取内环准确的执行周期
leg_drive(&leg[2],0.02);//leg_dt[1]);
  Send_LEG(2);

	delay_ms(20);
	}
}		
OS_STK LEG3_TASK_STK[LEG_STK_SIZE];
void leg3_task(void *pdata)
{
 u8 i;
 static u8 init;	
	leg_init(&leg[3],3);
 	while(1)
	{
	leg_dt[2] = Get_Cycle_T(GET_T_LEG3); 						//获取内环准确的执行周期
  leg_drive(&leg[3],0.02);//leg_dt[2]);
  Send_LEG(3);

	delay_ms(20);
	}
}		
OS_STK LEG4_TASK_STK[LEG_STK_SIZE];
void leg4_task(void *pdata)
{
 u8 i;
 static u8 init;	
	leg_init(&leg[4],4);
 	while(1)
	{
	leg_dt[3] = Get_Cycle_T(GET_T_LEG4); 						//获取内环准确的执行周期
  leg_drive(&leg[4],0.02);//leg_dt[3]);
  Send_LEG(4);

	delay_ms(20);
	}
}		

//========================外环  任务函数============================路径规划
OS_STK BRAIN_TASK_STK[BRAIN_STK_SIZE];
float test[5]={1,1,4};
void brain_task(void *pdata)
{	static u8 cnt,cnt1,cnt2,init;						  
 	while(1)
	{	
	leg_dt[4] = Get_Cycle_T(GET_T_BRAIN);								//获取外环准确的执行周期

		
	//test[2]=inTrig(test[0],test[1],0,0,0,5,5,0);
  //test[4]=inTrig2(test[0],test[1],5,5,5,0,0,5,0,0);
	//find_closet_point(test_id,test[0],0, 3,0, 5,0, 2,0 ,-1,0,test[2]);	
  //find_leg_need_move(0,0,0);		

	leg_task(0.02);
		
	center_control();	
	cal_deng_from_spd(&brain);		
	delay_ms(20);
	}
}		


//=======================串口 任务函数===========================
OS_STK  UART_TASK_STK[UART_STK_SIZE];
u8 UART_UP_LOAD_SEL=4;//<------------------------------UART UPLOAD DATA SEL
u8 state_v_test=0;
u8 num_need_to_check;
void uart_task(void *pdata)
{	static u8 cnt[4];	
  static u8 sd_sel;	
 	while(1)
	{
			
				
				//UPLOAD			
					
//					if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
//							{ 	
//							DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
//							clear_leg_uart();		
//							//GOL_LINK_TASK_DMA();	
//							USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
//							MYDMA_Enable(DMA2_Stream7,leg_uart_cnt+2);     //开始一次DMA传输！	  
//							}	
						
	
											
		delay_ms(20);  
	}
}	


//------------------------------软件定时器--------------------------------//
OS_TMR   * tmr1;			//软件定时器1
OS_TMR   * tmr2;			//软件定时器2
OS_TMR   * tmr3;			//软件定时器3

//软件定时器1的回调函数	
//每100ms执行一次,用于显示CPU使用率和内存使用率		
 u16 cpuusage=0;
void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	static u8 tcnt=0;	    

	if(tcnt==5)
	{
 		
		cpuusage=0;
		tcnt=0; 
	}
	cpuusage+=OSCPUUsage;
	tcnt++;				    
}
 #include "circle.h"
//软件定时器2的回调函数				  50ms	 
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
u8 i;	
static u16 cnt_1,cnt_2;	
static u8 cnt;
  for(i=0;i<5;i++)
	 if(leg[i].sys.leg_loss_cnt++>2/0.05)leg[i].leg_connect=0;
	 if(brain.sys.leg_loss_cnt++>2/0.05)brain.leg_connect=0;
	
	
}
//软件定时器3的回调函数				  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg) 
{	
 
} 


//