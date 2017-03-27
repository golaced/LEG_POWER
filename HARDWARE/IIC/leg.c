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

void leg_init( LEG_STRUCT *in,u8 id)
{
	
in->sys.id=id;
in->sys.init_mode=0;	
in->sys.l1=6.5;
in->sys.l2=7.2;
in->sys.l3=9.3;	

in->sys.init_end_pos.x=in->pos_tar[2].x=in->sys.pos_tar_trig_test[2].x=0;	
in->sys.init_end_pos.y=in->pos_tar[2].y=in->sys.pos_tar_trig_test[2].y=0;	
in->sys.init_end_pos.z=in->pos_tar[2].z=in->sys.pos_tar_trig_test[2].z=19;	
	
in->sys.limit.x=(in->sys.l1+in->sys.l2+in->sys.l3)*0.98;	
in->sys.limit.y=(in->sys.l1+in->sys.l2+in->sys.l3)*0.98;	
in->sys.limit.z=(in->sys.l1+in->sys.l2+in->sys.l3)*0.98;	
	
in->sys.limit_min.z=(in->sys.l3-(in->sys.l2-in->sys.l1))*1.05;

in->sys.desire_time=0.25;
switch(in->sys.id){
case 1:	
in->sys.leg_set_invert=0;
in->sys.PWM_OFF[0]=570;	
in->sys.PWM_OFF[1]=1870;	
in->sys.PWM_OFF[2]=1500;		
in->sys.sita_flag[0]=1;
in->sys.sita_flag[1]=-1;	
in->sys.sita_flag[2]=-1;
break;
case 2:	
in->sys.leg_set_invert=1;
in->sys.PWM_OFF[0]=520;	
in->sys.PWM_OFF[1]=2200;	
in->sys.PWM_OFF[2]=1650;	
in->sys.sita_flag[0]=1;
in->sys.sita_flag[1]=-1;	
in->sys.sita_flag[2]=-1;		
break;
case 3:	
in->sys.leg_set_invert=0;	
in->sys.PWM_OFF[0]=1925;	
in->sys.PWM_OFF[1]=625;	
in->sys.PWM_OFF[2]=1450;	
in->sys.sita_flag[0]=-1;
in->sys.sita_flag[1]=1;	
in->sys.sita_flag[2]=-1;	
break;
case 4:	
in->sys.leg_set_invert=1;	
in->sys.PWM_OFF[0]=2025;	
in->sys.PWM_OFF[1]=760;	
in->sys.PWM_OFF[2]=1750;	
in->sys.sita_flag[0]=-1;
in->sys.sita_flag[1]=1;	
in->sys.sita_flag[2]=-1;	
break;
}

in->sys.PWM_MIN[0]=500;	
in->sys.PWM_MIN[1]=500;	
in->sys.PWM_MIN[2]=500;	

in->sys.PWM_MAX[0]=2500;	
in->sys.PWM_MAX[1]=2500;	
in->sys.PWM_MAX[2]=2500;	

in->sys.PWM_PER_DEGREE=9;//9.1;		

in->sys.en_pwm_out=1;

in->sys.leg_up_high=6.6;

in->leg_ground=1;

}	

u8 pos_range_check(LEG_STRUCT * in,float x,float y,float z)
{
 float r=(in->sys.l1+in->sys.l2+in->sys.l3)*1;
 float r_pos=sqrt(x*x+y*y+z*z);
 if(r_pos>r||r_pos<(in->sys.l3-(in->sys.l2-in->sys.l1))*1)
	 return 0;
 else
	 return 1;
}


void cal_sita_from_pos(LEG_STRUCT * in,float x_i,float y_i,float z_i,u8 out)
{  
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
	if(out){
	in->pos_now[0].x=l1*sin(in->sita[2]*AtR);in->pos_now[0].y=0;in->pos_now[0].z=cos(in->sita[2]*AtR)*l1;
	float h1=sin(in->sita[0]*AtR)*l2;
	float h2=sin((180-in->sita[0]-in->sita[1])*AtR)*l3;
	float d1=cos(in->sita[2]*AtR)*l2;
	float d2=cos(in->sita[2]*AtR)*l3;
	in->pos_now[1].x=(l1+h1)*sin(in->sita[2]*AtR);in->pos_now[1].y=-cos(in->sita[0]*AtR)*d1;in->pos_now[1].z=cos(in->sita[2]*AtR)*(l1+h1);
	in->pos_now[2].x=(l1+h1+h2)*sin(in->sita[2]*AtR);in->pos_now[2].y=-cos(in->sita[0]*AtR)*d1+cos((180-in->sita[0]-in->sita[1])*AtR)*d2;in->pos_now[2].z=cos(in->sita[2]*AtR)*(l1+h1+h2);	
  if(isnan(in->sita[1])||isnan(in->sita[0]))
		ero=1;
  }
	//}
	
}	


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

void cal_pwm_from_sita(LEG_STRUCT * in)
{ u8 i;
	for(i=0;i<3;i++)
	in->sys.PWM_OUT[i]=LIMIT(in->sys.PWM_OFF[i]+in->sys.sita_flag[i]*in->sita[i]*in->sys.PWM_PER_DEGREE,in->sys.PWM_MIN[i],in->sys.PWM_MAX[i]);
	
}	

float curve_cal(float c0,float c3,float c4,float c5,float c6,float t)
{
float temp;
temp=c0+c3*pow(t,3)+c4*pow(t,4)+c5*pow(t,5)+c6*pow(t,6);
return temp;
}

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
	

//void cal_curve(int ,float x0,float y0,float z0,float T1,float T2)
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




void leg_follow_curve(LEG_STRUCT * in,float desire_time,u8 *en,float dt)
{
static u16 ground_mask;	
static u8 state;
static float time;	
//判断是否重合等
	
switch(state)
{
case 0:
if(*en){
cal_curve_from_pos(in,desire_time);
state=1;	
ground_mask=time=0;	
	if(!in->sys.use_ground_check)
   in->leg_ground=0;
}
break;
case 1:
if(in->leg_ground&&ground_mask++>100)	
state=*en=0;
if(*en){
cal_pos_tar_from_curve(in,time);
time+=dt;
if(time>desire_time)	
	state=*en=0;
}
else
{state=0;}	

break;

}
	
}


void  cal_pos_tar_for_deng(LEG_STRUCT * in,float spdx,float spdy,float dt)
{
static u8 state;
static float time;	
	if(!in->sys.use_ground_check)
   in->leg_ground=1;	
//判断是否重合等
if(!in->err&&in->leg_ground){
in->pos_tar[2].x+=-spdx*dt;
in->pos_tar[2].y+=-spdy*dt;
	
in->pos_tar[2].x=LIMIT(in->pos_tar[2].x,-in->sys.limit.x,in->sys.limit.x);
in->pos_tar[2].y=LIMIT(in->pos_tar[2].y,-in->sys.limit.x,in->sys.limit.y);
}


//in->pos_tar[2].z=cal_curve[Zs];
}	


void  leg_ground_check(LEG_STRUCT * in)
{

static u8 state;
static float time;	
//判断是否重合等
 if(!in->sys.use_ground_check)
 {
   if(in->pos_now[2].z>=in->sys.init_end_pos.z*0.98)
     in->leg_ground=1;
	 else
     in->leg_ground=0;
 }
 
}	



float sita_test[3];
u8 line_test[4];
u8 force_test_mode;
void leg_publish(LEG_STRUCT * in)
{
float x_temp,y_temp,z_temp;
  if(in->control_mode||force_test_mode)
	{	
	
  if(in->sys.leg_set_invert)
	{	
	x_temp=-(in->pos_tar[2].x);
	y_temp=-(in->pos_tar[2].y);
	z_temp=in->pos_tar[2].z;
	}else{
	x_temp=in->pos_tar[2].x;
	y_temp=in->pos_tar[2].y;
	z_temp=in->pos_tar[2].z;
	}		
	cal_sita_from_pos(in,x_temp+in->sys.off_local.x+in->sys.off_all.x,y_temp+in->sys.off_local.y+in->sys.off_all.y,
	z_temp+in->sys.off_local.z+in->sys.off_all.z,1);
  cal_pwm_from_sita(in);		
	cal_sita_from_pos(in,x_temp,y_temp,z_temp,0);	
	}
	else{
		in->pos_tar[2].x=0;in->pos_tar[2].y=0;in->pos_tar[2].z=19;in->leg_ground=1;
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
	cal_sita_from_pos(in,x_temp+in->sys.off_local.x,y_temp+in->sys.off_local.y,z_temp+in->sys.off_local.z,1);	
	}
  if(line_test[3]){
	in->sita[0]=sita_test[0];in->sita[1]=sita_test[1];in->sita[2]=sita_test[2];}
	
}



void leg_drive(LEG_STRUCT * in,float dt)
{    
    if(in->sys.pos_tar_trig_test[2].z!=19||in->sys.pos_tar_trig_test[2].y!=0||in->sys.pos_tar_trig_test[2].x!=0)		
		{
		in->pos_tar_trig[2].x=in->sys.pos_tar_trig_test[2].x;
		in->pos_tar_trig[2].y=in->sys.pos_tar_trig_test[2].y;
		in->pos_tar_trig[2].z=in->sys.pos_tar_trig_test[2].z;
		}
		if((in->pos_tar_trig[2].x!=in->sys.pos_tar_reg[0]||
			 in->pos_tar_trig[2].y!=in->sys.pos_tar_reg[1]||
			 in->pos_tar_trig[2].z!=in->sys.pos_tar_reg[2])&&!in->curve_trig)	
			{in->curve_trig=1;}

		if(in->curve_trig&&in->sys.desire_time>0)
		leg_follow_curve(in,in->sys.desire_time,& in->curve_trig,dt);
		if(!in->curve_trig&&(fabs(in->deng[1])>0||fabs(in->deng[0])>0))
		cal_pos_tar_for_deng(in,in->deng[0],in->deng[1],dt);	
					
		in->sys.pos_tar_reg[0]=in->pos_tar_trig[2].x;
		in->sys.pos_tar_reg[1]=in->pos_tar_trig[2].y;
		in->sys.pos_tar_reg[2]=in->pos_tar_trig[2].z;
		//
	  leg_publish(in);
}


