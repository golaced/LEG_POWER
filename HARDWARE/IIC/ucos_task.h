#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"


extern OS_TMR   * tmr1;			//�����ʱ��1
extern OS_TMR   * tmr2;			//�����ʱ��2
extern OS_TMR   * tmr3;			//�����ʱ��3
void tmr1_callback(OS_TMR *ptmr,void *p_arg); 		  	   
void tmr2_callback(OS_TMR *ptmr,void *p_arg); 	  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg); 
	
//�����������ȼ� 0->Highest     
//10->��ʼ��������ȼ�����Ϊ���


#define LEG1_TASK_PRIO       			  1 //MEMS
#define LEG2_TASK_PRIO       			  2 //MEMS
#define LEG3_TASK_PRIO       			  3 //MEMS
#define LEG4_TASK_PRIO       			  4 //MEMS

#define BRAIN_TASK_PRIO       			6 //MEMS

#define UART_TASK_PRIO       			  9 //MEMS



//-----------------------LEG�����߳�
//���������ջ��С
#define LEG_STK_SIZE  					64*4
//�����ջ	
extern OS_STK LEG1_TASK_STK[LEG_STK_SIZE];
//������
void leg1_task(void *pdata);


//�����ջ	
extern OS_STK LEG2_TASK_STK[LEG_STK_SIZE];
//������
void leg2_task(void *pdata);

//�����ջ	
extern OS_STK LEG3_TASK_STK[LEG_STK_SIZE];
//������
void leg3_task(void *pdata);

//�����ջ	
extern OS_STK LEG4_TASK_STK[LEG_STK_SIZE];
//������
void leg4_task(void *pdata);

//-----------------------LEG�����߳�
//���������ջ��С
#define BRAIN_STK_SIZE  					64*16
//�����ջ	
extern OS_STK BRAIN_TASK_STK[BRAIN_STK_SIZE];
//������
void brain_task(void *pdata);

//------------------------UART�߳�
//���������ջ��С
#define UART_STK_SIZE  					64*4
//�����ջ	
extern OS_STK UART_TASK_STK[UART_STK_SIZE];
//������
void uart_task(void *pdata);
//


#endif

