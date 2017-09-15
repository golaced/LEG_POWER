#include "include.h" 

float cal_area_trig(float x1,float y1,float x2,float y2,float x3,float y3)
{
float S;	
S=(1/2)*(x1*y2+x2*y3+x3*y1-x1*y3-x2*y1-x3*y2);
return S;
}	

//一个点在三角形内部
u8 inTrig(float x, float y,float x1,float y1,float x2,float y2,float x3,float y3) {
  POS a,b,c,p;
  p.x=x;p.y=y;
	a.x=x1;a.y=y1;
	b.x=x2;b.y=y2;
	c.x=x3;c.y=y3;
	
  float signOfTrig = (b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x);
  float signOfAB = (b.x - a.x)*(p.y - a.y) - (b.y - a.y)*(p.x - a.x);
  float signOfCA = (a.x - c.x)*(p.y - c.y) - (a.y - c.y)*(p.x - c.x);
  float signOfBC = (c.x - b.x)*(p.y - b.y) - (c.y - b.y)*(p.x - b.x);

  u8 d1 = (signOfAB * signOfTrig > 0);
  u8 d2 = (signOfCA * signOfTrig > 0);
  u8 d3 = (signOfBC * signOfTrig > 0);

  return d1 && d2 && d3;
}
//一个点在四边形内部
//			  y
//	d----------b          /\
// 	     |                |
//			 O                L
//			 |                |
//	a----------c   x			\/
u8 segmentsIntr(POS b,POS c,POS d,POS a,float *x,float *y){  
  
/** 1 解线性方程组, 求线段交点. **/  
// 如果分母为0 则平行或共线, 不相交  
    float denominator = (b.y - a.y)*(d.x - c.x) - (a.x - b.x)*(c.y - d.y);  
    if (denominator==0) {  
        return 0;  
    }  
   
// 线段所在直线的交点坐标 (x , y)      
     *x = ( (b.x - a.x) * (d.x - c.x) * (c.y - a.y)   
                + (b.y - a.y) * (d.x - c.x) * a.x   
                - (d.y - c.y) * (b.x - a.x) * c.x ) / denominator ;  
     *y = -( (b.y - a.y) * (d.y - c.y) * (c.x - a.x)   
                + (b.x - a.x) * (d.y - c.y) * a.y   
                - (d.x - c.x) * (b.y - a.y) * c.y ) / denominator;  
  
/** 2 判断交点是否在两条线段上 **/  
    if (  
        // 交点在线段1上  
        (*x - a.x) * (*x - b.x) <= 0 && (*y - a.y) * (*y - b.y) <= 0  
        // 且交点也在线段2上  
         && (*x - c.x) * (*x - d.x) <= 0 && (*y - c.y) * (*y - d.y) <= 0  
        ){  
  
        // 返回交点p  
        return 1;
    }  
    //否则不相交  
    return 0;  
  
}  
//一个点在四边形内部
u8 inTrig2(float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4) {
	u8 in_tri1=0,in_tri2=0,in_line_t12=0,in_tri3=0,in_tri4=0;
	in_tri1=inTrig(x,y, x1, y1, x2, y2, x3, y3);
  in_tri2=inTrig(x,y, x2, y2, x3, y3, x4, y4);
  in_tri3=inTrig(x,y, x1, y1, x3, y3, x4, y4);
  in_tri4=inTrig(x,y, x2, y2, x1, y1, x4, y4);
	POS b, c, d, a;
	b.x=x1;b.y=y1;
	c.x=x2;c.y=y2;
	d.x=x3;d.y=y3;
	a.x=x4;a.y=y4;
	float x_o,y_o;
	u8 cross;
	cross=segmentsIntr( b, c, d, a, &x_o, &y_o); 
	if(x_o==x&&y_o==y&&cross)
	in_line_t12=1;
  return in_tri1||in_tri2||in_tri3||in_tri4||in_line_t12;
}

static void swap(float *a, float *b)  
{  
    int     c;  
     c = *a;  
    *a = *b;  
    *b =  c;  
}  
//找到离xy最近的点
void find_closet_point(u8*min_id,float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4,u8 num) {
	u8 i,j;
	float dis[4],dis_id[4]={0,1,2,3};
	  dis[0]=sqrt(pow(x-x1,2)+pow(y-y1,2));
	  dis[1]=sqrt(pow(x-x2,2)+pow(y-y2,2));
	  dis[2]=sqrt(pow(x-x3,2)+pow(y-y3,2));
	  dis[3]=sqrt(pow(x-x4,2)+pow(y-y4,2));
    for (i = 0; i < num; i++)  
    {  
        //每一次由底至上地上升  
        for (j = num-1; j > i; j--)  
        {  
            if (dis[j] < dis[j-1])  
            {  
                swap(&dis[i], &dis[j]); 
								swap(&dis_id[i], &dis_id[j]);  
            }  
        }  
    }  
   if(num==3)
	 {
	  min_id[0]=dis_id[0];
		min_id[1]=dis_id[1]; 
	 }
	 else 
	 {
	  min_id[0]=dis_id[0];
		min_id[1]=dis_id[1]; 
	 } 
}


//------------------------------------------------------------brain---------------------------------------------------------------------
//       
//<-----W-------->
//			  y
//	3----------1          /\
// 	     |                |
//			 O                L
//			 |                |
//	4----------2   x			\/
 
void barin_init(BRAIN_STRUCT *in)
{
float W=5.7*2;//cm	
float L=18.5;
in->sys.leg_local[1].x=W/2;
in->sys.leg_local[1].y=L/2;
in->sys.leg_local[1].z=0;	
	
in->sys.leg_local[2].x=W/2;
in->sys.leg_local[2].y=-L/2;
in->sys.leg_local[2].z=0;	
	
in->sys.leg_local[3].x=-W/2;
in->sys.leg_local[3].y=L/2;
in->sys.leg_local[3].z=0;	
	
in->sys.leg_local[4].x=-W/2;
in->sys.leg_local[4].y=-L/2;
in->sys.leg_local[4].z=0;	

in->area_of_leg[1]=
cal_area_trig(in->sys.leg_local[1].x,in->sys.leg_local[1].y,
in->sys.leg_local[2].x,in->sys.leg_local[2].y,
in->sys.leg_local[3].x,in->sys.leg_local[3].y)/2+
cal_area_trig(in->sys.leg_local[4].x,in->sys.leg_local[4].y,
in->sys.leg_local[2].x,in->sys.leg_local[2].y,
in->sys.leg_local[3].x,in->sys.leg_local[3].y)/2;
	

in->leg_move_range[Yr]=leg[1].sys.limit.y*0.98;//cm		
in->leg_move_range[Xr]=in->leg_move_range[Yr]*0.68;//cm	

in->sys.k_spd_to_range=3.68;//0.88;
in->sys.center_off.x=0;
in->sys.center_off.y=0;
in->sys.center_off1.x=0;
in->sys.center_off1.y=0;

in->center_off_when_move[Xr]=-0.2;
in->center_off_when_move[Yr]=-0.88;
in->sys.leg_t=0.5;
in->sys.leg_h=4.68;
in->sys.desire_time=0.56;//0.76;
leg[1].sys.id=1;
leg[2].sys.id=2;
leg[3].sys.id=3;
leg[4].sys.id=4;
//leg[1].pos_tar[2].x=0;	
//leg[1].pos_tar[2].y=0;	
//leg[1].pos_tar[2].z=leg[1].sys.init_end_pos.z;	
//leg[2].pos_tar[2].x=0;	
//leg[2].pos_tar[2].y=0;	
//leg[2].pos_tar[2].z=leg[2].sys.init_end_pos.z;	
//leg[3].pos_tar[2].x=0;	
//leg[3].pos_tar[2].y=0;	
//leg[3].pos_tar[2].z=leg[3].sys.init_end_pos.z;	
//leg[4].pos_tar[2].x=0;	
//leg[4].pos_tar[2].y=0;	
//leg[4].pos_tar[2].z=leg[4].sys.init_end_pos.z;	

}	

//转换腿局部坐标系到全局机体坐标系
void conver_legpos_to_barin(BRAIN_STRUCT *in,LEG_STRUCT * inl,u8 id)
{ u8 i;
	for(i=0;i<3;i++){
	inl->pos_now_brain[i].x=inl->pos_now[i].x+in->sys.leg_local[id].x;
  inl->pos_now_brain[i].y=inl->pos_now[i].y+in->sys.leg_local[id].y;
	inl->pos_now_brain[i].z=inl->pos_now[i].z+in->sys.leg_local[id].z;
	}
}

//估计机器人中心
void estimate_center(BRAIN_STRUCT *in,float att[3],float spd_body[3],float acc_body[3],float spd_tar[3],float w_tar)//估计机体重心
{ u8 i,j;
	float high_robot,temp,k_off_z;
	for(i=1;i<5;i++)
		if(leg[i].leg_ground)
		{temp+=leg[i].pos_now[2].z;j++;}
		if(j>0){
		high_robot=temp/j;
		k_off_z=LIMIT(high_robot/15.5*2.68,0.2,2.2);
		}else
		k_off_z=1;
		
	in->center.x=0-leg[1].leg_ground*in->center_off_when_move[Xr]-leg[2].leg_ground*in->center_off_when_move[Xr]
	+leg[3].leg_ground*in->center_off_when_move[Xr]+leg[4].leg_ground*in->center_off_when_move[Xr];
	
	in->center.y=0-leg[1].leg_ground*in->center_off_when_move[Yr]-leg[3].leg_ground*in->center_off_when_move[Yr]
	+leg[2].leg_ground*in->center_off_when_move[Yr]+leg[4].leg_ground*in->center_off_when_move[Yr];
	in->center.x*=k_off_z;	
	in->center.y*=k_off_z;	
	in->center.z=0;

}

//选择需要跨腿 
float test1[3];
void find_leg_need_move(float spd_tar[3],float str[4],float end[4]) {
float yaw_spd,yaw_line,yaw_ero;
	if(fabs(spd_tar[Xr])>0||fabs(spd_tar[Yr])>0)
	{
			yaw_spd=fast_atan2(spd_tar[Xr],spd_tar[Yr])*57.3+180;
			float x_point_s,y_point_s,x_point_e,y_point_e; 
			x_point_s=str[Xr];  
		  y_point_s=str[Yr];
			x_point_e=end[Xr];  
		  y_point_e=end[Yr];
			
				float y_se= y_point_e-y_point_s;
				float x_se= x_point_e-x_point_s;
				if (x_se==0 && y_se>0)
						yaw_line = 360;
				if (x_se==0 && y_se<0)
						yaw_line = 180;
				if (y_se==0 && x_se>0)
						yaw_line = 90;
				if (y_se==0 && x_se<0)
						yaw_line = 270;
				float temp=fast_atan2(x_se,y_se)*57.3;
				if (x_se>0 && y_se>0)
					 yaw_line = temp;
				else if( x_se<0 && y_se>0)
					 yaw_line = 360 + temp;
				else if (x_se<0 && y_se<0)
					 yaw_line = 360 +temp;
				else if (x_se>0 && y_se<0)
					 yaw_line = temp;
			
			
			yaw_ero=fabs(To_180_degrees(yaw_line-yaw_spd));
			if(yaw_ero==90)//垂直
			{
			float random=RNG_Get_RandomRange(0,9);//获取[0,9]区间的随机数
			  if(random>4.5)
				{
				leg[(u8)end[3]].need_move+=2;
		  	//leg[(u8)end[3]].leg_move_pass_cnt=0;
        //leg[(u8)str[3]].leg_move_pass_cnt++;	
				}else{
				leg[(u8)str[3]].need_move+=2;
			  //leg[(u8)str[3]].leg_move_pass_cnt=0;
        //leg[(u8)end[3]].leg_move_pass_cnt++;		
					
				}
				
			}	
			else if(yaw_ero>90)//钝角
			{
			leg[(u8)end[3]].need_move+=2;
      //leg[(u8)end[3]].leg_move_pass_cnt++;				
			}	
			else//锐角
			{
			leg[(u8)str[3]].need_move+=2;
      //leg[(u8)str[3]].leg_move_pass_cnt++;	
			}	
	}
}

//判断机器人需要跨腿的ID
void check_leg_need_move(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt)
{ u8 i;
	u8 cnt;
	u8 min_id[2];
	static u8 lose_center_flag;
	float end_pos[4][4];
	float temp_x,temp_y;
  in->force_stop=0;
	for(i=1;i<5;i++)//存储临时变量
		if(leg[i].leg_ground){
      leg[i].need_move=0;			
			end_pos[in->ground_leg_num][Xr]=leg[i].pos_now_brain[2].x;
			end_pos[in->ground_leg_num][Yr]=leg[i].pos_now_brain[2].y;
			end_pos[in->ground_leg_num][Zr]=leg[i].pos_now_brain[2].z;
			end_pos[in->ground_leg_num][3]=i;	
		}
		
 //超出移动范围需要跨腿
 u8 out_cnt=0;
 for(i=1;i<5;i++){//in->sys.center_off.x+
	//check out range
	 float length,tar_x,tar_y,spd;
	 tar_x=leg[i].pos_now[2].x+dt*LIMIT(spd_tar[Xr],-2,2)*6;
	 tar_y=leg[i].pos_now[2].y+dt*LIMIT(spd_tar[Yr],-2,2)*6;
	 spd=my_sqrt(pow(spd_tar[Xr],2)+pow(spd_tar[Yr],2));	 
	 length=my_sqrt(pow(tar_x-0,2)+pow(tar_y-0,2));	 
	 float b=in->leg_move_range[Xr];
	 float a=in->leg_move_range[Yr];
	 float temp=pow(tar_x/b,2)+pow(tar_y/a,2);
	 float temp1=pow(leg[i].pos_now[2].x/b,2)+pow(leg[i].pos_now[2].y/a,2);
	 if(temp<1&&temp1>1&&spd>0)//&&length>in->leg_move_range[Xr])
	 { leg[i].need_move+=1;
		 in->force_stop=1;
	 }
	 if(temp1>1)
		  out_cnt++;
 } 

 //重心丢失需要跨腿
	//check loss center
 if(0){ 
 switch(in->ground_leg_num) 
 {	 
 case 3://三条腿
 temp_x=dt*LIMIT(spd_tar[Xr],-2,2)+in->center.x+0;
 temp_y=dt*LIMIT(spd_tar[Yr],-2,2)+in->center.y+0;
 if(inTrig(temp_x,temp_y,end_pos[0][Xr],end_pos[0][Yr],end_pos[1][Xr],end_pos[1][Yr],end_pos[2][Xr],end_pos[2][Yr]))
	 brain.loss_center=0;//
 else
	 brain.loss_center=1;//
 
 if(brain.loss_center&&!lose_center_flag)
 {
 find_closet_point(min_id,temp_x,temp_y,end_pos[0][Xr],end_pos[0][Yr],end_pos[1][Xr],end_pos[1][Yr],end_pos[2][Xr],end_pos[2][Yr],0,0,3); 

 find_leg_need_move(spd_tar,end_pos[min_id[0]],end_pos[min_id[1]]);	 
 }
 lose_center_flag=brain.loss_center;
 break;
 case 4://四条腿
 temp_x=dt*LIMIT(spd_tar[Xr],-2,2)+in->center.x+0;
 temp_y=dt*LIMIT(spd_tar[Yr],-2,2)+in->center.y+0;
 brain.loss_center=!inTrig2(temp_x,temp_y,end_pos[0][Xr],end_pos[0][Yr],end_pos[1][Xr],end_pos[1][Yr],end_pos[2][Xr],end_pos[2][Yr],end_pos[3][Xr],end_pos[3][Yr]);
 if(brain.loss_center&&!lose_center_flag)
 {
	find_closet_point(min_id,temp_x,temp_y,end_pos[0][Xr],end_pos[0][Yr],end_pos[1][Xr],end_pos[1][Yr],end_pos[2][Xr],end_pos[2][Yr],end_pos[3][Xr],end_pos[3][Yr],4);  
  
  find_leg_need_move(spd_tar,end_pos[min_id[0]],end_pos[min_id[1]]);
 }
 break;
 }}	 
 
 for(i=1;i<5;i++)//赋值给brain
 in->leg_move[i]=leg[i].need_move;
}
//计算着地脚重心
void cal_center_of_leg_ground(BRAIN_STRUCT *in)
{
u8 i=0,j=0;	
double x[5]={-5,0,4,0},y[5]={0,5,0,0};
float temp[2][3];
in->ground_leg_num=0;
  for(i=1;i<5;i++){
		if(leg[i].leg_ground){
		x[j]=leg[i].pos_now_brain[2].x;
		y[j]=leg[i].pos_now_brain[2].y;
		j++;
		}
  }
in->ground_leg_num=j;
  
switch(in->ground_leg_num) 
 {	 
 case 3:
   in->leg_ground_center[Xr]=(float)(x[0]+x[1]+x[2])/3.;
   in->leg_ground_center[Yr]=(float)(y[0]+y[1]+y[2])/3.;
   in->area_of_leg[0]=cal_area_trig( x[0],y[0], x[1],y[1], x[2],y[2]);
 break;
 case 4:
    temp[0][Xr]=(float)(x[0]+x[1]+x[2])/3.;
    temp[0][Yr]=(float)(y[0]+y[1]+y[2])/3.;
    temp[1][Xr]=(float)(x[1]+x[2]+x[3])/3.;
    temp[1][Yr]=(float)(y[1]+y[2]+y[3])/3.;
    in->leg_ground_center[Xr]=(float)(temp[0][Xr]+temp[1][Xr])/2.;
    in->leg_ground_center[Yr]=(float)(temp[0][Yr]+temp[1][Yr])/2.;
    in->area_of_leg[0]=cal_area_trig( x[0],y[0], x[1],y[1], x[2],y[2])/2+cal_area_trig( x[3],y[3], x[1],y[1], x[2],y[2])/2;
 break;
 default:
	  in->area_of_leg[0]=0;
 break;
 }	 
}

//重心控制
float center_control_out[2];
float k_center_c[2]={6.6,6.6};
float flt=0.86;
void center_control(float dt)//中心控制PID
{
  
	float ero[2];
	 ero[Xr]=-my_deathzoom(brain.center.x-brain.leg_ground_center[Xr],0.006);
   ero[Yr]=-my_deathzoom(brain.center.y-brain.leg_ground_center[Yr],0.006);
   if(fabs(ero[Xr])<0.15&&fabs(ero[Yr])<0.15)
	 brain.center_stable=1;
	 else
	 brain.center_stable=0;
	 if(brain.ground_leg_num<4){
   center_control_out[Xr]=ero[Xr]*k_center_c[Xr];
	 center_control_out[Yr]=ero[Yr]*k_center_c[Yr];
	 }else
	 {
	  center_control_out[Xr]= center_control_out[Yr]=0;
	 }
}

//需要跨腿的目标调用该函数
void leg_tar_est(BRAIN_STRUCT *in,LEG_STRUCT *leg,float spd_body[3],float spd_tar[3],float w_tar,u8 need_move,float dt)
{
u8 id=leg->sys.id;
float tar_yaw,spd;
		tar_yaw=in->spd_yaw;
	  spd=in->spd;
float tar_x,tar_y,tar_z;	
switch(need_move){//超出范围
	case 1 :
		tar_x=LIMIT(sin(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->leg_move_range[Xr]*0.98,in->leg_move_range[Xr]*0.98);
	  tar_y=LIMIT(cos(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->leg_move_range[Yr]*0.98,in->leg_move_range[Yr]*0.98);
	  leg->pos_tar_trig[2].x=leg->sys.init_end_pos.x*1+tar_x+(float)RNG_Get_RandomRange(-1000,1000)/100000.+in->sys.center_off.x;
	  leg->pos_tar_trig[2].y=leg->sys.init_end_pos.y*1+tar_y+(float)RNG_Get_RandomRange(-1000,1000)/100000.+in->sys.center_off.y;
	  leg->pos_tar_trig[2].z=leg->sys.init_end_pos.z*0.98;
		  in->sys.pos_tar_trig[leg->sys.id].x=leg->pos_tar_trig[2].x;
		  in->sys.pos_tar_trig[leg->sys.id].y=leg->pos_tar_trig[2].y;
		  in->sys.pos_tar_trig[leg->sys.id].z=leg->pos_tar_trig[2].z;

	break;
	case 2 ://重心丢失
		tar_x=LIMIT(sin(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->leg_move_range[Xr]*0.98,in->leg_move_range[Xr]*0.98);
	  tar_y=LIMIT(cos(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->leg_move_range[Yr]*0.98,in->leg_move_range[Yr]*0.98);
	  leg->pos_tar[2].x=tar_x+(float)RNG_Get_RandomRange(-1000,1000)/100000.;
		leg->pos_tar[2].y=tar_y+(float)RNG_Get_RandomRange(-1000,1000)/100000.;	
		  in->sys.pos_tar_trig[leg->sys.id].x=leg->pos_tar[2].x;
		  in->sys.pos_tar_trig[leg->sys.id].y=leg->pos_tar[2].y;
		  in->sys.pos_tar_trig[leg->sys.id].z=leg->pos_tar[2].z;

	break;
}
}

//触发跨脚
u8 trig_list[5]=		 {0,1,4,3,2};
u8 trig_list_ero[5]= {0,2,3,4,1};
u8 trig_list_ero2[5]={0,0,0,0,0};
u8 test112=15;
float move_off[2]={0};
u16 set11=30;
 u8 last_move_leg;	
void get_leg_tar_trig(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt)//mmmm
{
static float dt_leg_min_trig_cnt;
u8 i,cnt,id,id_star,leg_move_pass_cnt;
u8 leg_out_cnt[3]={0};//flag of state robot	
u8 out_range_id[4];
u8 loss_center_id[4];	
	for(i=1;i<5;i++){//cnt_leg_situation
	if(leg[i].need_move==1)//out range leg
	{
	 	leg[i].sys.leg_move_pass_cnt++;
	 leg_out_cnt[0]++;	
	}
	else if(leg[i].need_move==2)//loss center
	{leg[i].sys.leg_move_pass_cnt++;
	 leg_out_cnt[1]++;
	}
	else if(leg[i].need_move==3)//out range leg + loss center
	{leg[i].sys.leg_move_pass_cnt++;
	 leg_out_cnt[2]++;
	}
 }
	
 static u8 can_move_leg;
 if(dt_leg_min_trig_cnt++>2*brain.sys.desire_time/dt)
	 can_move_leg=1;
 
 u8 id_need_to_move=RNG_Get_RandomRange(1,4);
 for(i=1;i<5;i++)
   if(leg[i].sys.leg_move_pass_cnt>leg[id_need_to_move].sys.leg_move_pass_cnt)
		  id_need_to_move=i;
	 
 if(can_move_leg&&leg_out_cnt[0]>0){
 leg_tar_est(in,&leg[id_need_to_move],0,0,0,leg[id_need_to_move].need_move,dt);
 dt_leg_min_trig_cnt=0;
 can_move_leg=0;
 leg[id_need_to_move].sys.leg_move_pass_cnt=0;	 
 }
}

//由机体速度换算蹬腿速度
void cal_deng_from_spd(BRAIN_STRUCT *in)
{
u8 i;	
float spdx,spdy,spdz;	
float spd=in->spd;
float yaw=in->spd_yaw;
	
	
  if(in->force_stop)
		spd=0;
	float k_move=1;
	if(brain.ground_leg_num<4)k_move=0.68;
  in->sys.tar_spd[Xr]=spdx=sin(yaw*0.0173)*spd*k_move;
	in->sys.tar_spd[Yr]=spdy=cos(yaw*0.0173)*spd*k_move;
	
	
		for(i=1;i<5;i++){
		if((leg[i].control_mode||in->control_mode)&&leg[i].leg_ground&&!leg[i].sys.leg_ground_force){	
		leg[i].deng[Xr]=LIMIT(spdx+1*center_control_out[Xr],-8.8,8.8);
		leg[i].deng[Yr]=LIMIT(spdy+1*center_control_out[Yr],-8.8,8.8);	}
		}
}

void leg_task(float dt)
{
static u16 cnt_init;	
static u8 init;
u8 i;
if(!init&&cnt_init++>3/0.02){init=1;
barin_init(&brain);}

if(init){
	    for(i=1;i<5;i++) 	  
	    conver_legpos_to_barin(&brain,&leg[i],i);      
	
			estimate_center(&brain,brain.att,brain.now_spd,brain.now_acc,brain.sys.tar_spd,brain.tar_w);//估计机体重心

			cal_center_of_leg_ground(&brain);//估计着地多边形

			check_leg_need_move(&brain,brain.now_spd,brain.sys.tar_spd,brain.tar_w,dt);//判断需要跨的脚
			 
			get_leg_tar_trig(&brain,brain.now_spd,brain.sys.tar_spd,brain.tar_w, dt);//跨腿目标规划
	
	    center_control(dt);	
	
	    cal_deng_from_spd(&brain);	
}

}