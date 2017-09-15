#include "include.h" 
LEG_STRUCT leg[5];
BRAIN_STRUCT brain;
//% leg ending positon calculate'
//%         _                side view            ----  ------>x  back view
//%         |  l1                               3  ||   |
//%      __ O                o---> y               OO   |
//%     1  //                |                     ||   \/ Z 
//%       //     l2          |                     ||
//%      O                 Z \/                    OO
//%    /  \\                                       ||
//%   / 2  \\    l3                                ||
//%          O                                     OO
void READ_LEG_ID(LEG_STRUCT *in)
{
in->sys.id=0;
}
float off_local[2]={0.86,0};	
float k_z=0.92;
u16 SET_PWM3_OFF=0;
void leg_init( LEG_STRUCT *in,u8 id)
{
	
in->sys.id=id;
in->leg_ground=1;
in->sys.init_mode=0;	
in->sys.l1=4.9;
in->sys.l2=6.3;
in->sys.l3=7.8;	

switch(in->sys.id){
case 1:	
in->sys.leg_set_invert=1;
in->sys.PWM_OFF[0]=1850;//570;	
in->sys.PWM_OFF[1]=620;//1870;	
in->sys.PWM_OFF[2]=1400+SET_PWM3_OFF;//1600		
in->sys.PWM_OFF[3]=1380;
in->sys.sita_flag[0]=-1;
in->sys.sita_flag[1]=1;	
in->sys.sita_flag[2]=1;
in->sys.pwm_id[0]=9;
in->sys.pwm_id[1]=8;
in->sys.pwm_id[2]=10;
in->sys.pwm_id[3]=11;
break;
case 2:	
in->sys.leg_set_invert=0;
in->sys.PWM_OFF[0]=700;	
in->sys.PWM_OFF[1]=1960;	
in->sys.PWM_OFF[2]=1500+SET_PWM3_OFF;	
in->sys.PWM_OFF[3]=1380;
in->sys.sita_flag[0]=1;
in->sys.sita_flag[1]=-1;	
in->sys.sita_flag[2]=1;	
in->sys.pwm_id[0]=15;
in->sys.pwm_id[1]=14;
in->sys.pwm_id[2]=13;
in->sys.pwm_id[3]=12;
break;
case 3:	
in->sys.leg_set_invert=1;	
in->sys.PWM_OFF[0]=1060;	
in->sys.PWM_OFF[1]=1810;//1540;	
in->sys.PWM_OFF[2]=1480-SET_PWM3_OFF;	
in->sys.PWM_OFF[3]=1425;
in->sys.sita_flag[0]=1;
in->sys.sita_flag[1]=-1;	
in->sys.sita_flag[2]=1;
in->sys.pwm_id[0]=6;
in->sys.pwm_id[1]=7;
in->sys.pwm_id[2]=5;
in->sys.pwm_id[3]=4;
break;
case 4:	
in->sys.leg_set_invert=0;	
in->sys.PWM_OFF[0]=1530;	
in->sys.PWM_OFF[1]=660;	
in->sys.PWM_OFF[2]=1400-SET_PWM3_OFF;	
in->sys.PWM_OFF[3]=1388;
in->sys.sita_flag[0]=-1;
in->sys.sita_flag[1]=1;	
in->sys.sita_flag[2]=1;
in->sys.pwm_id[0]=1;
in->sys.pwm_id[1]=0;
in->sys.pwm_id[2]=2;
in->sys.pwm_id[3]=3;
break;
}
int flag[2]={1,1};
if(in->sys.id==3||in->sys.id==4)
	flag[0]=-1;
if(in->sys.id==2||in->sys.id==4)
	flag[1]=-1;
in->sys.init_end_pos.x=in->pos_tar[2].x=in->sys.pos_tar_trig_test[2].x=off_local[0]*flag[0];	
in->sys.init_end_pos.y=in->pos_tar[2].y=in->sys.pos_tar_trig_test[2].y=off_local[1]*flag[1];		
in->sys.init_end_pos.z=in->pos_tar[2].z=in->sys.pos_tar_trig_test[2].z=(in->sys.l1+in->sys.l2+in->sys.l3)*0.88*k_z;
	
in->pos_tar_trig[2].z=in->sys.init_end_pos.z;
in->sys.limit.x=(in->sys.l1+in->sys.l2+in->sys.l3)*0.98*0.25;	
in->sys.limit.y=(in->sys.l1+in->sys.l2+in->sys.l3)*0.98*0.25;	
in->sys.limit.z=(in->sys.l1+in->sys.l2+in->sys.l3)*0.88;	
	
in->sys.limit_min.z=(in->sys.l3-(in->sys.l2-in->sys.l1))*1.05;

in->sys.desire_time=0.4;

int DL=88;
in->sys.PWM_MIN[0]=500+DL;	
in->sys.PWM_MIN[1]=500+DL;
in->sys.PWM_MIN[2]=500+DL;

in->sys.PWM_MAX[0]=2500-DL;
in->sys.PWM_MAX[1]=2500-DL;	
in->sys.PWM_MAX[2]=2500-DL;	

in->sys.PWM_PER_DEGREE=9.34;//7.8;//9.1;		

in->sys.en_pwm_out=1;

in->sys.leg_up_high=3;

in->leg_ground=1;

}	
//λ�ó������ƶ���Χ���
u8 pos_range_check(LEG_STRUCT * in,float x,float y,float z)
{
 float r=(in->sys.l1+in->sys.l2+in->sys.l3)*1;
 float r_pos=sqrt(x*x+y*y+z*z);
 if(r_pos>r||r_pos<(in->sys.l3-(in->sys.l2-in->sys.l1))*1)
	 return 0;
 else
	 return 1;
}

//��λ�ý���ؽڽǶ� 
void cal_sita_from_pos(LEG_STRUCT * in,float x_i,float y_i,float z_i,u8 out)
{ 
u8 id=in->sys.id;	
u8 ero;	
float x=LIMIT(x_i,-in->sys.limit.x,in->sys.limit.x);	
float y=LIMIT(y_i,-in->sys.limit.y,in->sys.limit.y);
float z=LIMIT(z_i,-in->sys.limit.z,in->sys.limit.z);	
	if(pos_range_check(in,x,y,z))	
	{
	in->err=0;
	}		
	else
	in->err=1;
	in->sita[2]=fast_atan2(x,z)*RtA;
	float l1=cos(in->sita[2]*AtR)*in->sys.l1;
	float l2=cos(in->sita[2]*AtR)*in->sys.l2;
	float l3=cos(in->sita[2]*AtR)*in->sys.l3;
	float temp=z-l1;
	float l4=sqrt(y*y+temp*temp);
	temp=(l2*l2+l3*l3-l4*l4)/(2*l2*l3);
	float sita5=acos(LIMIT(temp,-1,1))*RtA;
	if(isnan(sita5))
		ero=1;
	in->sita[1]=(180-sita5);             
	temp=(l2*l2+l4*l4-l3*l3)/(2*l2*l4);
	in->sita[0]=180-acos(y/l4)*RtA-acos(LIMIT(temp,-1,1))*RtA;//*cos(in->sita[2]*AtR);
	//recal_pos 
	if(out){//��Ϊ��������¼��㵱ǰ�Ƕȶ�Ӧ��������ĩ��λ��
	in->pos_now[0].x=l1*sin(in->sita[2]*AtR);in->pos_now[0].y=0;in->pos_now[0].z=cos(in->sita[2]*AtR)*l1;
	float h1=sin(in->sita[0]*AtR)*l2;
	float h2=sin((180-in->sita[0]-in->sita[1])*AtR)*l3;
	float d1=cos(in->sita[2]*AtR)*l2;
	float d2=cos(in->sita[2]*AtR)*l3;
	in->pos_now[1].x=(l1+h1)*sin(in->sita[2]*AtR);in->pos_now[1].y=-cos(in->sita[0]*AtR)*d1;in->pos_now[1].z=cos(in->sita[2]*AtR)*(l1+h1);
	in->pos_now[2].x=(l1+h1+h2)*sin(in->sita[2]*AtR);in->pos_now[2].y=-cos(in->sita[0]*AtR)*d1+cos((180-in->sita[0]-in->sita[1])*AtR)*d2;in->pos_now[2].z=cos(in->sita[2]*AtR)*(l1+h1+h2);	

	if(in->sys.leg_set_invert)
		for(u8 i=0;i<3;i++)
	   {
		 in->pos_now[i].x*=-1;
		 in->pos_now[i].y*=-1;
		 }
  if(isnan(in->sita[1])||isnan(in->sita[0]))
		ero=1;
  }
}	

//�ӹؽڽǶȼ���λ��
void cal_pos_from_sita(LEG_STRUCT * in,float sita1,float sita2,float sita3)
{                                                                                                                                                                                                            

float l1=cos(sita3*AtR)*in->sys.l1;
float l2=cos(sita3*AtR)*in->sys.l2;
float l3=cos(sita3*AtR)*in->sys.l3;
//recal_pos 
in->pos_now[0].x=l1*sin(sita3*AtR);in->pos_now[0].y=0;in->pos_now[0].z=cos(sita3*AtR)*l1;
float h1=sin(sita1*AtR)*l2;
float h2=sin((180-sita1-sita2)*AtR)*l3;
float d1=cos(sita3*AtR)*l2;
float d2=cos(sita3*AtR)*l3;
in->pos_now[1].x=(l1+h1)*sin(sita3*AtR);in->pos_now[1].y=-cos(sita1*AtR)*d1;in->pos_now[1].z=cos(sita3*AtR)*(l1+h1);
in->pos_now[2].x=(l1+h1+h2)*sin(sita3*AtR);in->pos_now[2].y=-cos(sita1*AtR)*d1+cos((180-sita1-sita2)*AtR)*d2;in->pos_now[2].z=cos(sita3*AtR)*(l1+h1+h2);	

}	

//�ӹؽڽǶȼ�����PWM
void cal_pwm_from_sita(LEG_STRUCT * in)
{ u8 i=0;
	in->sys.PWM_OUT[i]=LIMIT(in->sys.PWM_OFF[i]+0*in->sys.sita_flag[i]*in->sys.PWM_PER_DEGREE
	+in->sys.sita_flag[i]*in->sita[i]*in->sys.PWM_PER_DEGREE,in->sys.PWM_MIN[i],in->sys.PWM_MAX[i]);
	for(i=1;i<3;i++)
	in->sys.PWM_OUT[i]=LIMIT(in->sys.PWM_OFF[i]+in->sys.sita_flag[i]*in->sita[i]*in->sys.PWM_PER_DEGREE,in->sys.PWM_MIN[i],in->sys.PWM_MAX[i]);
	in->sys.PWM_OUT[3]=LIMIT(in->sys.PWM_OFF[3],500,2500);
}	

//���������������ά����
float curve_cal(float c0,float c3,float c4,float c5,float c6,float t)
{
float temp;
temp=c0+c3*pow(t,3)+c4*pow(t,4)+c5*pow(t,5)+c6*pow(t,6);
return temp;
}

//�����������ϵ��
float c0[3]; //x y z
float c3[3];
float c4[3];
float c5[3];
float c6[3];
void cal_curve_from_pos(LEG_STRUCT * in,float desire_time)
{
float pos_now[3];
float pos_tar[3];
pos_now[Xs]=in->pos_now[2].x;
pos_now[Ys]=in->pos_now[2].y;
pos_now[Zs]=in->pos_now[2].z;
pos_tar[Xs]=in->pos_tar_trig[2].x;
pos_tar[Ys]=in->pos_tar_trig[2].y;
pos_tar[Zs]=in->pos_tar_trig[2].z;
	
		float t1=0;t1=desire_time/2; //middle 0.5s
    float t2=0;t2=desire_time; //end 1s
    float p0[3];
    float p1[3];
    float p2[3];
//----------start
    p0[0]=pos_now[Xs];
   	p0[1]=pos_now[Ys];
   	p0[2]=pos_now[Zs];
//--------------end
	  p2[0]=pos_tar[Xs];//x
   	p2[1]=pos_tar[Ys];//y
   	p2[2]=pos_tar[Zs];//z
//-------------middle
   	p1[0]=(p0[0]+p2[0])/2;
   	p1[1]=(p0[1]+p2[1])/2;
   	p1[2]=LIMIT((p0[2]+p2[2])/2-in->sys.leg_up_high,in->sys.limit_min.z,30);//wait
		
  float p1_p0[3];
	float p0_p2[3];
	int i;
	for(i=0;i<3;i++)
	{
		c0[i]=p0[i];
		p1_p0[i]=p1[i]-p0[i];
		p0_p2[i]=p0[i]-p2[i];
	}

	float t1_3=pow(t1,3);
	float t1_4=pow(t1,4);
	float t1_5=pow(t1,5);
	float t1_6=pow(t1,6);

	float t2_2=pow(t2,2);
	float t2_3=pow(t2,3);
	float t2_5=pow(t2,5);
	float t2_6=pow(t2,6);

	float temp1=0;temp1=1/(t1_3*pow((t1-t2),3)*t2_3);
	for(i=0;i<3;i++)
	{	c3[i]=-1*temp1*(t2_6*(p1_p0[i])+5*t1_4*t2_2*3*(p0_p2[i])
		+2*t1_6*5*(p0_p2[i])-3*t1_5*t2*8*(p0_p2[i]));

    c4[i]=temp1/t2*(3*t2_6*(p1_p0[i])+15*t1_3*t2_3*(p0_p2[i])
    		-27*t1_5*t2*(p0_p2[i])+t1_6*15*(p0_p2[i]));

	c5[i]=-temp1/t2_2*3*(
			t2_6*(p1_p0[i])
			+2*t1_6*(p0_p2[i])
			+t1_3*t2_3*8*(p0_p2[i])
			-t1_4*t2_2*9*(p0_p2[i]));

	c6[i]=temp1/t2_2*
	(t2_5*(p1_p0[i])
			+6*t1_5*(p0_p2[i])
			+10*t1_3*t2_2*(p0_p2[i])
			-t1_4*t2*15*(p0_p2[i]));}
}

//�����滮���߸�������άλ��
void  cal_pos_tar_from_curve(LEG_STRUCT * in,float time_now)
{
float cal_curve[3];	
u8 i;
for(i=0;i<3;i++)
cal_curve[i]=curve_cal(c0[i],c3[i],c4[i],c5[i],c6[i],time_now);
	
in->pos_tar[2].x=cal_curve[Xs];
in->pos_tar[2].y=cal_curve[Ys];
in->pos_tar[2].z=cal_curve[Zs];
}	

float rate_delay_kuai=0.1;
float delay_time_kuai=0.66;
//����
void leg_follow_curve(LEG_STRUCT * in,float desire_time,u8 *en,float dt)
{
u8 id=in->sys.id;
static u16 ground_mask[5];	
static u8 state[5];
static float time[5],delay[5];	
//�ж��Ƿ��غϵ�
	
switch(state[id])
{
case 0:
if(*en){//���ŵص�滮��ǰ�켣
cal_curve_from_pos(in,desire_time);
state[id]=1;	
ground_mask[id]=time[id]=delay[id]=0;	
//	if(!in->sys.use_ground_check)
in->leg_ground=0;
}
break;
case 1://��ʱ��͹켣����ÿһʱ�����������
if(*en){
cal_pos_tar_from_curve(in,time[id]);
time[id]+=dt;
if(time[id]>desire_time*rate_delay_kuai)	
{state[id]=2;}
}
break;
case 2:
if(*en){
delay[id]+=dt;
if(delay[id]>delay_time_kuai)	
{state[id]=3;}
}
break;
case 3:
if(*en){
cal_pos_tar_from_curve(in,time[id]);
time[id]+=dt;
if(time[id]>desire_time)	
{state[id]=4;}
}
break;
case 4:
	in->leg_ground=1;
  state[id]=0;
  if(in->rst_leg)
		in->rst_leg=0;
  *en=0;
break;
}	
}

//����
void  cal_pos_tar_for_deng(LEG_STRUCT * in,float spdx,float spdy,float dt)
{
u8 id=in->sys.id;
static u8 state;
static float time;	
//	if(!in->sys.use_ground_check)
//   in->leg_ground=1;	
//�ж��Ƿ��غϵ�
	if(!in->err&&in->leg_ground){
	 if(in->sys.leg_set_invert){	
	in->pos_tar[2].x+=-spdx*dt;
	in->pos_tar[2].y+=-spdy*dt;
	 }else{
	in->pos_tar[2].x+=-spdx*dt;
	in->pos_tar[2].y+=-spdy*dt;
	 }
	in->pos_tar[2].x=LIMIT(in->pos_tar[2].x,-in->sys.limit.x,in->sys.limit.x);
	in->pos_tar[2].y=LIMIT(in->pos_tar[2].y,-in->sys.limit.y,in->sys.limit.y);
	}
//in->pos_tar[2].z=cal_curve[Zs];
}	

//�ŵؼ��
void  leg_ground_check(LEG_STRUCT * in)
{
u8 id=in->sys.id;
static u8 state;
static float time;	
//�ж��Ƿ��غϵ�
 if(!in->sys.use_ground_check&&in->sys.leg_ground_force==0)
 {
//   if(in->pos_now[2].z>=in->sys.init_end_pos.z*0.96)
//     in->leg_ground=1;
//	 else
//     in->leg_ground=0;
 }
   if(in->sys.leg_ground_force==2)
   in->leg_ground=1;
	 else if(in->sys.leg_ground_force==1)
	 in->leg_ground=0;	 
}	



float sita_test[3]={90,0,0};
u8 line_test[4];
u8 force_test_mode;
void leg_publish(LEG_STRUCT * in)
{
float x_temp,y_temp,z_temp;
static u16 cnt[5];	
	if(in->sys.id==1||in->sys.id==3){
	in->sys.off_all.x=brain.sys.center_off.x+center_control_out[Xr]*0;
	in->sys.off_all.y=brain.sys.center_off.y+center_control_out[Yr]*0;
	in->sys.off_all.z=brain.sys.center_off.z;
	}
	else
	{
	in->sys.off_all.x=brain.sys.center_off1.x+center_control_out[Xr]*0;
	in->sys.off_all.y=brain.sys.center_off1.y+center_control_out[Yr]*0;
	in->sys.off_all.z=brain.sys.center_off1.z;
	}
	in->sys.leg_up_high=brain.sys.leg_h;
	in->sys.desire_time=brain.sys.desire_time;
	if(brain.power_all)//&&in->sys.id!=1)
		in->leg_power=1;
	if(brain.control_mode)
		in->control_mode=1;
	
	static u8 rst_all;
	static u8 rst_flag=1;
	switch(rst_all){
		case 0:
			if(brain.rst_all){
			brain.spd=0;
			leg[1].rst_leg=1;
		  rst_all=1;
			}
		break;
		case 1:
			if(leg[rst_flag].rst_leg==0)
			{rst_flag++;leg[rst_flag].rst_leg=1;}
		if(rst_flag==5)
		{rst_flag=1;brain.rst_all=0;rst_all=0;}
		break;	
	}

	
	
	#if USE_BUS_DJ
	if(in->leg_power==0){	
		if(cnt[in->sys.id]++>1/0.02){cnt[in->sys.id]=0;
		LEG_POWER_OFF(in->sys.id);}
	}
	#endif
  if(in->control_mode||force_test_mode)
	{	
	
  if(in->sys.leg_set_invert)//<---------------------use normal mode 
	{	
	x_temp=-(in->pos_tar[2].x);
	y_temp=-(in->pos_tar[2].y);
	z_temp=in->pos_tar[2].z;
	}else{
	x_temp=in->pos_tar[2].x;
	y_temp=in->pos_tar[2].y;
	z_temp=in->pos_tar[2].z;
	}		
	//���������Ƕ�
	cal_sita_from_pos(in,x_temp+in->sys.off_local.x+in->sys.off_all.x,y_temp+in->sys.off_local.y+in->sys.off_all.y,
	z_temp+in->sys.off_local.z+in->sys.off_all.z,1);
	 if(line_test[3]){//ǿ�ƽǶȲ���
	in->sita[0]=sita_test[0];in->sita[1]=sita_test[1];in->sita[2]=sita_test[2];}
  cal_pwm_from_sita(in);//����PWM�ɽǶ�	
	cal_sita_from_pos(in,x_temp,y_temp,z_temp,0);//�ӽǶȷ���λ��	
	}
	
	else{//----------------------not on control mode ǿ�Ʋ�������λ��
		in->pos_tar[2].x=0;in->pos_tar[2].y=0;in->pos_tar[2].z=in->sys.init_end_pos.z;
		//in->leg_ground=1;
		 if(in->sys.leg_set_invert)
	{	
	x_temp=-(in->sys.init_end_pos.x-in->sys.off_all.x);
	y_temp=-(in->sys.init_end_pos.y-in->sys.off_all.y);
	z_temp=in->sys.init_end_pos.z+in->sys.off_all.z;
	}else{
	x_temp=in->sys.init_end_pos.x+in->sys.off_all.x;
	y_temp=in->sys.init_end_pos.y+in->sys.off_all.y;
	z_temp=in->sys.init_end_pos.z+in->sys.off_all.z;
	}		
	//���������Ƕ�
	cal_sita_from_pos(in,x_temp+in->sys.off_local.x+in->sys.off_all.x,y_temp+in->sys.off_local.y+in->sys.off_all.y,
	z_temp+in->sys.off_local.z+in->sys.off_all.z,1);
	 if(line_test[3]){//ǿ�ƽǶȲ���
	in->sita[0]=sita_test[0];in->sita[1]=sita_test[1];in->sita[2]=sita_test[2];}
  cal_pwm_from_sita(in);//����PWM�ɽǶ�	
	cal_sita_from_pos(in,x_temp,y_temp,z_temp,0);//�ӽǶȷ���λ��	
	}
 
	
}



void leg_drive(LEG_STRUCT * in,float dt)
{  
 u8 id=in->sys.id; 	
 static u8 init[5];	
	if(!init[id])
	{
	init[id]=1;	
	in->sys.pos_tar_reg[0]=in->pos_tar_trig[2].x;
	in->sys.pos_tar_reg[1]=in->pos_tar_trig[2].y;
	in->sys.pos_tar_reg[2]=in->pos_tar_trig[2].z;	
	}
   
	  if(in->rst_leg)
		{
		in->pos_tar_trig[2].x=in->sys.init_end_pos.x;
		in->pos_tar_trig[2].y=in->sys.init_end_pos.y;
		in->pos_tar_trig[2].z=in->sys.init_end_pos.z+(float)RNG_Get_RandomRange(-1000,1000)/100000.;
		}
		
	
		if((in->pos_tar_trig[2].x!=in->sys.pos_tar_reg[0]||
			 in->pos_tar_trig[2].y!=in->sys.pos_tar_reg[1]||
			 in->pos_tar_trig[2].z!=in->sys.pos_tar_reg[2])&&!in->curve_trig)	
			{in->curve_trig=1;}//trig ʹ��
    //��
		if(in->curve_trig&&in->sys.desire_time>0)
		leg_follow_curve(in,in->sys.desire_time,& in->curve_trig,dt);
		//�ŵ�
		leg_ground_check(in);
		//��
		if(!in->curve_trig&&(fabs(in->deng[1])>0||fabs(in->deng[0])>0))
		cal_pos_tar_for_deng(in,in->deng[0],in->deng[1],dt);	
		//���	
	  leg_publish(in);
		
		in->sys.pos_tar_reg[0]=in->pos_tar_trig[2].x;
		in->sys.pos_tar_reg[1]=in->pos_tar_trig[2].y;
		in->sys.pos_tar_reg[2]=in->pos_tar_trig[2].z;
		//
}


