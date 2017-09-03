#include "include.h" 

//------------------------------------------------------------barin---------------------------------------------------------------------
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
float L=16.7;
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

in->leg_move_range[Xr]=3;//cm	
in->leg_move_range[Yr]=3;//cm		
	
in->sys.k_spd_to_range=0.36;
in->sys.center_off.x=0;
in->sys.center_off.y=0;
in->sys.center_off1.x=0;
in->sys.center_off1.y=0;


in->sys.leg_t=0.5;
in->sys.leg_h=3;
in->sys.desire_time=0.3588;
leg[1].sys.id=1;
leg[2].sys.id=2;
leg[3].sys.id=3;
leg[4].sys.id=4;
leg[1].pos_tar[2].x=0;	
leg[1].pos_tar[2].y=0;	
leg[1].pos_tar[2].z=leg[1].sys.init_end_pos.z;	
leg[2].pos_tar[2].x=0;	
leg[2].pos_tar[2].y=0;	
leg[2].pos_tar[2].z=leg[2].sys.init_end_pos.z;	
leg[3].pos_tar[2].x=0;	
leg[3].pos_tar[2].y=0;	
leg[3].pos_tar[2].z=leg[3].sys.init_end_pos.z;	
leg[4].pos_tar[2].x=0;	
leg[4].pos_tar[2].y=0;	
leg[4].pos_tar[2].z=leg[4].sys.init_end_pos.z;	

}	

void conver_legpos_to_barin(BRAIN_STRUCT *in,LEG_STRUCT * inl,u8 id)
{u8 i;
	for(i=0;i<3;i++){
	inl->pos_now_brain[i].x=inl->pos_now[i].x+in->sys.leg_local[id].x;
  inl->pos_now_brain[i].y=inl->pos_now[i].y+in->sys.leg_local[id].y;
	inl->pos_now_brain[i].z=inl->pos_now[i].z+in->sys.leg_local[id].z;
	}
}

void estimate_center(BRAIN_STRUCT *in,float att[3],float spd_body[3],float acc_body[3],float spd_tar[3],float w_tar)//估计机体重心
{

	in->center.x=0;
	in->center.y=0;
	in->center.z=0;
}



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

void check_leg_need_move(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt)//判断需要跨的脚
{ u8 i;
	u8 cnt;
	u8 min_id[2];
	static u8 lose_center_flag;
	float end_pos[4][4];
	float temp_x,temp_y;
	  in->ground_leg_num=0;
	for(i=1;i<5;i++)
		if(leg[i].leg_ground){	
			end_pos[in->ground_leg_num][Xr]=leg[i].pos_now_brain[2].x;
			end_pos[in->ground_leg_num][Yr]=leg[i].pos_now_brain[2].y;
			end_pos[in->ground_leg_num][Zr]=leg[i].pos_now_brain[2].z;
			end_pos[in->ground_leg_num][3]=i;	
			in->ground_leg_num++;
		}
	
 for(i=1;i<5;i++){
	leg[i].need_move=0;
	//check out range
	 if((leg[i].pos_now[2].x+dt*spd_tar[Xr])>(in->sys.center_off.x+in->leg_move_range[Xr])&&spd_tar[Xr]<=0)
		 leg[i].need_move+=1;
	 else if((leg[i].pos_now[2].x+dt*spd_tar[Xr])<(in->sys.center_off.x-in->leg_move_range[Xr])&&spd_tar[Xr]>=0)
		 leg[i].need_move+=1;
	 else if((leg[i].pos_now[2].y+dt*spd_tar[Yr])>(in->sys.center_off.y+in->leg_move_range[Yr])&&spd_tar[Yr]<=0)
		 leg[i].need_move+=1;
	 else if((leg[i].pos_now[2].y+dt*spd_tar[Yr])<(in->sys.center_off.y-in->leg_move_range[Yr])&&spd_tar[Yr]>=0)
		 leg[i].need_move+=1;
 } 
 u8 loss_center_temp=0;
 
	//check loss center 	 
 switch(in->ground_leg_num) 
 {	 
 case 3:

 temp_x=dt*spd_tar[Xr]+in->center.x+0;
 temp_y=dt*spd_tar[Yr]+in->center.y+0;
 if(inTrig(temp_x,temp_y,end_pos[0][Xr],end_pos[0][Yr],end_pos[1][Xr],end_pos[1][Yr],end_pos[2][Xr],end_pos[2][Yr]))
	 brain.loss_center=0;//
 else
	 brain.loss_center=1;//
 lose_center_flag=brain.loss_center;
 if(brain.loss_center&&!lose_center_flag)
 {
 find_closet_point(min_id,temp_x,temp_y,end_pos[0][Xr],end_pos[0][Yr],end_pos[1][Xr],end_pos[1][Yr],end_pos[2][Xr],end_pos[2][Yr],0,0,3); 

 find_leg_need_move(spd_tar,end_pos[min_id[0]],end_pos[min_id[1]]);	 
 }
 break;
 case 4:
 temp_x=dt*spd_tar[Xr]+in->center.x+0;
 temp_y=dt*spd_tar[Yr]+in->center.y+0;
 brain.loss_center=!inTrig2(temp_x,temp_y,end_pos[0][Xr],end_pos[0][Yr],end_pos[1][Xr],end_pos[1][Yr],end_pos[2][Xr],end_pos[2][Yr],end_pos[3][Xr],end_pos[3][Yr]);
 if(brain.loss_center&&!lose_center_flag)
 {
	find_closet_point(min_id,temp_x,temp_y,end_pos[0][Xr],end_pos[0][Yr],end_pos[1][Xr],end_pos[1][Yr],end_pos[2][Xr],end_pos[2][Yr],end_pos[3][Xr],end_pos[3][Yr],4);  
  
  find_leg_need_move(spd_tar,end_pos[min_id[0]],end_pos[min_id[1]]);
 }
 break;
 }	 

 	for(i=1;i<5;i++)
 in->leg_move[i]=leg[i].need_move;
}

void cal_center_of_leg_ground(BRAIN_STRUCT *in)//计算着地脚重心
{
u8 i;	
double x[4]={-5,0,4,0},y[4]={0,5,0,0};
float temp[2][3];
		x[0]=leg[1].pos_now_brain[2].x;
		y[0]=leg[1].pos_now_brain[2].y;
		x[1]=leg[2].pos_now_brain[2].x;
		y[1]=leg[2].pos_now_brain[2].y;
		x[2]=leg[3].pos_now_brain[2].x;
		y[2]=leg[3].pos_now_brain[2].y;
		x[3]=leg[4].pos_now_brain[2].x;
		y[3]=leg[4].pos_now_brain[2].y;
  in->ground_leg_num=0;
	for(i=1;i<5;i++)
		if(leg[i].leg_ground){	
			in->ground_leg_num++;
		}
switch(in->ground_leg_num) 
 {	 
 case 3:
   in->leg_ground_center[Xr]=(float)(x[0]*leg[1].leg_ground+x[1]*leg[2].leg_ground+x[2]*leg[3].leg_ground+x[3]*leg[4].leg_ground)/3.;
   in->leg_ground_center[Yr]=(float)(y[0]*leg[1].leg_ground+y[1]*leg[2].leg_ground+y[2]*leg[3].leg_ground+y[3]*leg[4].leg_ground)/3.;
 break;
 case 4:
    temp[0][Xr]=(float)(x[0]+x[1]+x[2])/3.;
    temp[0][Yr]=(float)(y[0]+y[1]+y[2])/3.;
    temp[1][Xr]=(float)(x[1]+x[2]+x[3])/3.;
    temp[1][Yr]=(float)(y[1]+y[2]+y[3])/3.;
    in->leg_ground_center[Xr]=(float)(temp[0][Xr]+temp[1][Xr])/2.;
    in->leg_ground_center[Yr]=(float)(temp[0][Yr]+temp[1][Yr])/2.;
 break;
 }	 
}

float center_control_out[2];
float k_center_c[2]={2,0};
float flt=0.2;
void center_control(void)//PID
{
  
	float ero[2];
//	 ero[Xr]=brain.center.x-brain.leg_ground_center[Xr];
//   ero[Yr]=brain.center.y-brain.leg_ground_center[Yr];
	 if(last_move_leg==1||last_move_leg==2)
		ero[Xr]=1; 
	 else if(last_move_leg==4||last_move_leg==3)
		ero[Xr]=-1; 
	 else
		ero[Xr]=0;  
   center_control_out[Xr]=flt*ero[Xr]*k_center_c[Xr]+(1-flt)*center_control_out[Xr];
	 center_control_out[Yr]=flt*ero[Yr]*k_center_c[Yr]+(1-flt)*center_control_out[Yr];
}

void leg_tar_est(BRAIN_STRUCT *in,LEG_STRUCT *leg,float spd_body[3],float spd_tar[3],float w_tar,u8 need_move,float dt)
{
u8 id=leg->sys.id;
float tar_yaw,spd;
		tar_yaw=in->spd_yaw;//(spd_tar[Xr],spd_tar[Yr])*57.3;
	  spd=in->spd;//sqrt(pow(spd_tar[0],2)+pow(spd_tar[1],2));
float tar_x,tar_y,tar_z;	
switch(need_move){
	case 1 :
		tar_x=LIMIT(sin(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->leg_move_range[Xr]*0.98,in->leg_move_range[Xr]*0.98);
	  tar_y=LIMIT(cos(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->leg_move_range[Yr]*0.98,in->leg_move_range[Yr]*0.98);
	  leg->pos_tar_trig[2].x=tar_x+(float)RNG_Get_RandomRange(-1000,1000)/100000.+in->sys.center_off.x;
	  leg->pos_tar_trig[2].y=tar_y+(float)RNG_Get_RandomRange(-1000,1000)/100000.+in->sys.center_off.y;
	  leg->pos_tar_trig[2].z=leg->sys.init_end_pos.z;
		  in->sys.pos_tar_trig[leg->sys.id].x=leg->pos_tar_trig[2].x;
		  in->sys.pos_tar_trig[leg->sys.id].y=leg->pos_tar_trig[2].y;
		  in->sys.pos_tar_trig[leg->sys.id].z=leg->pos_tar_trig[2].z;
		  leg->sys.leg_move_pass_cnt=0;
	break;
	case 2 :
		tar_x=LIMIT(sin(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->leg_move_range[Xr]*0.98,in->leg_move_range[Xr]*0.98);
	  tar_y=LIMIT(cos(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->leg_move_range[Yr]*0.98,in->leg_move_range[Yr]*0.98);
	  leg->pos_tar[2].x=tar_x+(float)RNG_Get_RandomRange(-1000,1000)/100000.;
	   if(leg->sys.id==2)
	  leg->pos_tar[2].y=tar_y+(float)RNG_Get_RandomRange(-1000,1000)/100000.+0.5;
		else
		leg->pos_tar[2].y=tar_y+(float)RNG_Get_RandomRange(-1000,1000)/100000.;	
		  in->sys.pos_tar_trig[leg->sys.id].x=leg->pos_tar[2].x;
		  in->sys.pos_tar_trig[leg->sys.id].y=leg->pos_tar[2].y;
		  in->sys.pos_tar_trig[leg->sys.id].z=leg->pos_tar[2].z;
		leg->sys.leg_move_pass_cnt=0;
	break;
}


}	
u8 trig_list[5]=		 {0,1,4,3,2};
u8 trig_list_ero[5]= {0,2,3,4,1};
u8 trig_list_ero2[5]={0,0,0,0,0};
u8 test112=15;
float move_off[2]={0};
u16 set11=30;
 u8 last_move_leg;	
void get_leg_tar_trig(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt)//mmmm
{

u8 i,cnt,id,id_star,leg_move_pass_cnt;
u8 leg_out_cnt[3]={0};	
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
	else if(leg[i].need_move==3)//
	{leg[i].sys.leg_move_pass_cnt++;
	 leg_out_cnt[1]++;
	}
 }
static u8 leg_up_falg;	 
// if(in->ground_leg_num==4&&!leg_up_falg)
//	  in->force_stop=0;
// if(in->ground_leg_num<4||leg_out_cnt[1]>=2||leg_out_cnt[0]>=2||
//	 	 (fabs(leg[i].pos_now[2].x+dt*spd_tar[Xr])>in->leg_move_range[Xr])||
//    (fabs(leg[i].pos_now[2].y+dt*spd_tar[Yr])>in->leg_move_range[Yr]))
//		in->force_stop=1;
//	

static u8 cnt_delay;	
//if(brain.ground_leg_num>3){cnt_delay=0;
//	cnt=0;
//if(leg_out_cnt[1]>2){
//		for(i=1;i<5;i++)
//		 { if(leg[i].need_move>=2)
//			  {id_star=i;leg_move_pass_cnt=leg[i].leg_move_pass_cnt;break;}
//		 } 
//		 
//			for(i=id_star;i<5;i++)
//		 { 
//			 if(leg[trig_list[i]].leg_move_pass_cnt>=leg_move_pass_cnt&&leg[trig_list[i]].need_move>=2&&last_move_leg!=trig_list_ero[i]&&last_move_leg!=trig_list_ero2[i])
//			 {id=trig_list[i];leg_move_pass_cnt=leg[trig_list[i]].leg_move_pass_cnt;}
//		 }
//	 
//	 leg_tar_est(in, &leg[id],spd_body,spd_tar, w_tar, 1, dt);last_move_leg=id;	leg[id].leg_move_pass_cnt=0 ;
//		 
// }
//else if(leg_out_cnt[1]>0)
//{
//		for(i=1;i<5;i++)
//		 { if(leg[trig_list[i]].need_move>=2&&last_move_leg!=trig_list_ero[i]&&last_move_leg!=trig_list_ero2[i])
//			  { leg_tar_est(in, &leg[trig_list[i]],spd_body,spd_tar, w_tar, 1, dt);	 
//				  last_move_leg=id;leg[trig_list[i]].leg_move_pass_cnt=0 ;
//				break;}
//		 } 
////-------------------------------------------------------------------------------------
//}else if(leg_out_cnt[0]>2){//--------------out range

//		for(i=1;i<5;i++)
//		 { if(leg[i].need_move>=1)
//			  {id_star=i;leg_move_pass_cnt=leg[i].leg_move_pass_cnt;break;}
//		 } 
//		 
//			for(i=id_star;i<5;i++)
//		 { 
//			 if(leg[trig_list[i]].leg_move_pass_cnt>=leg_move_pass_cnt&&leg[trig_list[i]].need_move>=1&&last_move_leg!=trig_list_ero[i]&&last_move_leg!=trig_list_ero2[i])
//			 {id=trig_list[i];leg_move_pass_cnt=leg[trig_list[i]].leg_move_pass_cnt;break;}
//		 }
//	 leg_tar_est(in, &leg[id],spd_body,spd_tar, w_tar, 1, dt);last_move_leg=id;	 leg[id].leg_move_pass_cnt=0 ;
//		 
// }
//else if(leg_out_cnt[0]>0)
//{
//		for(i=1;i<5;i++)
//		 { if(leg[trig_list[i]].need_move>=1&&last_move_leg!=trig_list_ero[i]&&last_move_leg!=trig_list_ero2[i])
//			  { leg_tar_est(in, &leg[trig_list[i]],spd_body,spd_tar, w_tar, 1, dt);last_move_leg=id;leg[trig_list[i]].leg_move_pass_cnt=0 ;
//				break;}
//		 } 

//}
//}


static u8 cnt_all_ground=0;
static u8 state,state_cnt=0;
static u16 cnt1;
switch(state)
{
case 0:
if(brain.ground_leg_num>3&&leg_out_cnt[0]>0&&leg[1].need_move>0)
{state++;in->force_stop=1;last_move_leg=1;cnt1=0;}
break;
case 1:
center_control();	
if(cnt1++>set11){cnt1=0;state++;}
break;
case 2:
//添加要移动前倾斜
leg_tar_est(in, &leg[1],spd_body,spd_tar, w_tar, 1, dt);state++;
break;
case 3:
if(brain.ground_leg_num==4&&cnt_all_ground++>test112)
{state++;cnt_all_ground=0;in->force_stop=0;}
break;	

case 4:
if(brain.ground_leg_num>3&&leg_out_cnt[0]>0&&leg[4].need_move>0)
{state++;in->force_stop=1;last_move_leg=4;}
break;
case 5:
center_control();	
if(cnt1++>set11){cnt1=0;state++;}
break;
case 6:
//添加要移动前倾斜
leg_tar_est(in, &leg[4],spd_body,spd_tar, w_tar, 1, dt);state++;
break;
case 7:
if(brain.ground_leg_num==4&&cnt_all_ground++>test112)
{state++;cnt_all_ground=0;in->force_stop=0;}
break;	

case 8:
if(brain.ground_leg_num>3&&leg_out_cnt[0]>0&&leg[3].need_move>0)
{state++;in->force_stop=1;last_move_leg=3;}
break;
case 9:
center_control();	
if(cnt1++>set11){cnt1=0;state++;}
break;
case 10:
//添加要移动前倾斜
leg_tar_est(in, &leg[3],spd_body,spd_tar, w_tar, 1, dt);state++;
break;
case 11:
if(brain.ground_leg_num==4&&cnt_all_ground++>test112)
{state++;cnt_all_ground=0;in->force_stop=0;}
break;	

case 12:
if(brain.ground_leg_num>3&&leg_out_cnt[0]>0&&leg[2].need_move>0)
{state++;in->force_stop=1;last_move_leg=2;}
break;
case 13:
center_control();	
if(cnt1++>set11){cnt1=0;state++;}
break;
case 14:
//添加要移动前倾斜
leg_tar_est(in, &leg[2],spd_body,spd_tar, w_tar, 1, dt);state++;
break;
case 15:
if(brain.ground_leg_num==4&&cnt_all_ground++>test112)
{state=0;cnt_all_ground=0;in->force_stop=0;}
break;	
	
}
 
}

void cal_deng_from_spd(BRAIN_STRUCT *in)
{
u8 i;	
float spdx,spdy,spdz;	
float spd=in->spd;
float yaw=in->spd_yaw;
	
	
  if(in->force_stop)
		spd=0;
  in->sys.tar_spd[Xr]=spdx=sin(yaw*0.0173)*spd;
	in->sys.tar_spd[Yr]=spdy=cos(yaw*0.0173)*spd;
		for(i=1;i<5;i++){
		if(leg[i].control_mode||in->control_mode){	
		leg[i].deng[Xr]=spdx+0*center_control_out[Xr];
		leg[i].deng[Yr]=spdy+0*center_control_out[Yr];	}
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

			cal_center_of_leg_ground(&brain);

			check_leg_need_move(&brain,brain.now_spd,brain.sys.tar_spd,brain.tar_w,dt);//判断需要跨的脚
			// 
			get_leg_tar_trig(&brain,brain.now_spd,brain.sys.tar_spd,brain.tar_w, dt);

}

}