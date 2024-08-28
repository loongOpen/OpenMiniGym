#include "include.h"
#include "eso.h"
#include "gait_math.h"
#if !RUN_WEBOTS
#include "arm_math.h"
#endif
#define MIN_F_DIS 0.00001
/*
X
|
|
|
|-------Y
*/
_GAIT_WALK walk_gait;
void line_function_from_two_point_n(float x1, float y1, float x2, float y2, float *a, float *b, float *c)
{
	*a = (x2 - x1);
	*b = (y1 - y2);
	*c = (y2*x1 - y1 * x2);
}

void line_function_from_two_point(float x1, float y1, float x2, float y2, float *k, float *b)
{
	float k_temp = 0;
	*k = k_temp = (x1 - x2) / (y1 - y2 + 0.000001);
	*b = x1 - k_temp * y1;
}

void line_function_from_arrow_n(float x, float y, float yaw, float *a, float *b, float *c)
{
	float r = 0.1;
	float t_x = x + cosd(yaw)*r;
	float t_y = y + sind(yaw)*r;
	line_function_from_two_point_n(x, y, t_x, t_y, a, b, c);
}

void line_function_from_arrow(float x, float y, float yaw, float *k, float *b)
{
	float tyaw = 90 - yaw;
	float k_temp = 0;
	if (ABS(tyaw) < 0.1)
		tyaw = 0.1;
	*k = k_temp = tand(tyaw);
	*b = x - k_temp * y;
}

void line_function90_from_arrow(float x, float y, float yaw, float *k, float *b)//g直线方程 x=y*k+b 
{
	float tyaw = 90 - yaw;
	float k_temp = 0;
	if (ABS(tyaw) < 0.1)
		tyaw = 0.1;
	*k = k_temp = -1 / tand(tyaw);
	*b = x - k_temp * y;
}

u8 cross_point_of_lines(float k1, float b1, float k2, float b2, float *x, float *y)//g
{
	float y_temp;
	if (ABS(k1 - k2) < 0.001)
	{
		*x = *y = 0;
		return 0;
	}

	*y = y_temp = (b1 - b2) / (k2 - k1 + 0.00001);
	*x = k1 * y_temp + b1;
	return 1;
}

u8 cross_point_of_lines_n(float a1, float b1, float c1, float a2, float b2, float c2, float *x, float *y)//g
{
	float m = a1 * b2 - a2 * b1;
	if (m == 0)
		return 0;

	*x = (c1*a2 - c2 * a1) / m;
	*y = (c2*b1 - c1 * b2) / m;

	return 1;
}

void swap(float *a, float *b)  //g
{
	float   c;
	c = *a;
	*a = *b;
	*b = c;
}

float cal_area_trig(END_POS p1, END_POS p2, END_POS p3)//float x1,float y1,float x2,float y2,float x3,float y3)
{
	float x1 = p1.y;
	float y1 = p1.x;
	float x2 = p2.y;
	float y2 = p2.x;
	float x3 = p3.y;
	float y3 = p3.x;
	float a = sqrtf((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
	float b = sqrtf((x3 - x2)*(x3 - x2) + (y3 - y2)*(y3 - y2));
	float c = sqrtf((x3 - x1)*(x3 - x1) + (y3 - y1)*(y3 - y1));
	float p = (a + b + c) / 2;
	float Ss = sqrtf(p*(p - a)*(p - b)*(p - c));
	return Ss;
}

//一个点在三角形内部
u8 inTrig(END_POS point, END_POS p1, END_POS p2, END_POS p3) {//g
	END_POS a, b, c, p;
	float x = point.x;
	float y = point.y;
	float x1 = p1.x;
	float y1 = p1.y;
	float x2 = p2.x;
	float y2 = p2.y;
	float x3 = p3.x;
	float y3 = p3.y;
	p.y = x; p.x = y;
	a.y = x1; a.x = y1;
	b.y = x2; b.x = y2;
	c.y = x3; c.x = y3;

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
u8 segmentsIntr(END_POS b, END_POS c, END_POS d, END_POS a, float *x, float *y) {  //g

/** 1 解线性方程组, 求线段交点. **/
// 如果分母为0 则平行或共线, 不相交  
	float denominator = (b.x - a.x)*(d.y - c.y) - (a.y - b.y)*(c.x - d.x);
	if (denominator == 0) {
		return 0;
	}

	// 线段所在直线的交点坐标 (x , y)      
	*y = ((b.y - a.y) * (d.y - c.y) * (c.x - a.x)
		+ (b.x - a.x) * (d.y - c.y) * a.x
		- (d.x - c.x) * (b.y - a.y) * c.x) / denominator;
	*x = -((b.x - a.x) * (d.x - c.x) * (c.y - a.y)
		+ (b.y - a.y) * (d.x - c.x) * a.x
		- (d.y - c.y) * (b.x - a.x) * c.x) / denominator;

	/** 2 判断交点是否在两条线段上 **/
	if (
		// 交点在线段1上  
		(*y - a.y) * (*y - b.y) <= 0 && (*x - a.x) * (*x - b.x) <= 0
		// 且交点也在线段2上  
		&& (*y - c.y) * (*y - d.y) <= 0 && (*x - c.x) * (*x - d.x) <= 0
		) {

		// 返回交点p  
		return 1;
	}
	//否则不相交  
	return 0;

}
//一个点在四边形内部
u8 inTrig2(END_POS point, END_POS p1, END_POS p2, END_POS p3, END_POS p4) {//g
	u8 in_tri1 = 0, in_tri2 = 0, in_line_t12 = 0, in_tri3 = 0, in_tri4 = 0;
	float x = point.x;
	float y = point.y;
	float x1 = p1.x;
	float y1 = p1.y;
	float x2 = p2.x;
	float y2 = p2.y;
	float x3 = p3.x;
	float y3 = p3.y;
	float x4 = p4.x;
	float y4 = p4.y;
	in_tri1 = inTrig(point, p1, p2, p3);
	in_tri2 = inTrig(point, p2, p3, p4);
	in_tri3 = inTrig(point, p2, p4, p1);
	in_tri4 = inTrig(point, p4, p1, p2);
	END_POS b, c, d, a;
	b = p1;
	c = p2;
	d = p3;
	a = p4;
	float x_o, y_o;
	u8 cross;
	cross = segmentsIntr(b, c, d, a, &x_o, &y_o);
	if (x_o == x && y_o == y && cross)
		in_line_t12 = 1;
	return in_tri1 || in_tri2 || in_tri3 || in_tri4 || in_line_t12;
}

//找到离xy最近的点
void find_closet_point(u8*min_id, float x, float y, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4, u8 num) {
	u8 i, j;
	float dis[4], dis_id[4] = { 0,1,2,3 };
	dis[0] = sqrtf(pow(x - x1, 2) + pow(y - y1, 2));
	dis[1] = sqrtf(pow(x - x2, 2) + pow(y - y2, 2));
	dis[2] = sqrtf(pow(x - x3, 2) + pow(y - y3, 2));
	dis[3] = sqrtf(pow(x - x4, 2) + pow(y - y4, 2));
	for (i = 0; i < num; i++)
	{
		//每一次由底至上地上升  
		for (j = num - 1; j > i; j--)
		{
			if (dis[j] < dis[j - 1])
			{
				swap(&dis[i], &dis[j]);
				swap(&dis_id[i], &dis_id[j]);
			}
		}
	}
	if (num == 3)
	{
		min_id[0] = dis_id[0];
		min_id[1] = dis_id[1];
	}
	else
	{
		min_id[0] = dis_id[0];
		min_id[1] = dis_id[1];
	}
}

float cal_bear(float x2, float  y2, float  x1, float  y1)//计算1 到 2的朝向
{
	float angle = 0;
	float y_se = y1 - y2;
	float x_se = x1 - x2;
	if (x_se == 0 && y_se > 0)
		angle = 360;
	if (x_se == 0 && y_se < 0)
		angle = 180;
	if (y_se == 0 && x_se > 0)
		angle = 90;
	if (y_se == 0 && x_se < 0)
		angle = 270;
	if (x_se > 0 && y_se > 0)
		angle = atan(x_se / y_se)*57.3;
	else if (x_se < 0 && y_se>0)
		angle = 360 + atan(x_se / y_se)*57.3;
	else if (x_se < 0 && y_se < 0)
		angle = 180 + atan(x_se / y_se)*57.3;
	else if (x_se > 0 && y_se < 0)
		angle = 180 + atan(x_se / y_se)*57.3;
	return angle;
}


END_POS cal_quad_cog(END_POS a, END_POS b, END_POS c, END_POS d)//g
{
	END_POS cog1, cog2, cog;
	cog1.x = (a.x + b.x + c.x) / 3;
	cog1.y = (a.y + b.y + c.y) / 3;
	cog1.z = (a.z + b.z + c.z) / 3;

	cog2.x = (b.x + c.x + d.x) / 3;
	cog2.y = (b.y + c.y + d.y) / 3;
	cog2.z = (b.z + c.z + d.z) / 3;

	cog.x = (cog1.x + cog2.x) / 2;
	cog.y = (cog1.y + cog2.y) / 2;
	cog.z = (cog1.z + cog2.z) / 2;
	return cog;
}


END_POS cal_tri_cog(END_POS a, END_POS b, END_POS c)//g
{
	END_POS cog;
	cog.x = (a.x + b.x + c.x) / 3;
	cog.y = (a.y + b.y + c.y) / 3;
	cog.z = (a.z + b.z + c.z) / 3;
	return cog;
}

END_POS cal_quad_cog1(END_POS a, END_POS b, END_POS c, END_POS d)//g
{
	END_POS cog1, cog2, cog;
	cog1 = cal_tri_cog1(a, b, c);
	cog2 = cal_tri_cog1(b, c, d);

	cog.x = (cog1.x + cog2.x) / 2;
	cog.y = (cog1.y + cog2.y) / 2;
	cog.z = (cog1.z + cog2.z) / 2;
	return cog;
}


END_POS cal_tri_cog1(END_POS a, END_POS b, END_POS c)//g
{
	END_POS cog;
	float m0 = a.y;
	float n0 = a.x;
	float m1 = b.y;
	float n1 = b.x;
	float m2 = c.y;
	float n2 = c.x;
	int dax = 0;
	int day = 0;

	int dbx = 0;
	int dby = 0;

	float absA = 0.0f;
	float absB = 0.0f;
	float temp = 0;

	dax = m0 - m1;
	day = n0 - n1;

	dbx = m2 - m1;
	dby = n2 - n1;

	temp = dax * dax + day * day * 1.0f;
	absA = sqrtf(temp);
	temp = dbx * dbx + dby * dby * 1.0f;
	absB = sqrtf(temp);

	float a1 = 0.0f;
	float b1 = 0.0f;

	a1 = (absB * day - absA * dby);
	b1 = (absA * dbx - absB * dax);


	dax = m0 - m2;
	day = n0 - n2;

	dbx = m1 - m2;
	dby = n1 - n2;

	temp = dax * dax + day * day * 1.0f;
	absA = sqrtf(temp);
	temp = dbx * dbx + dby * dby * 1.0f;
	absB = sqrtf(temp);

	float c1 = 0.0f;
	float d1 = 0.0f;

	c1 = (absB * day - absA * dby);
	d1 = (absA * dbx - absB * dax);

	float PointX = 0.0f;
	float PointY = 0.0f;


	if (a1 != 0)
	{
		PointX = (c1 * b1 * m1 + n2 * a1 * c1 - n1 * a1 * c1 - a1 * d1 * m2) / (c1 * b1 - a1 * d1 + 0.000001);
		PointY = b1 * (PointX - m1) / (a1 + 0.000001) + n1;

	}
	else
	{
		PointX = m1;
		PointY = d1 * (m1 - m2) / (c1 + 0.000001) + n2;
	}

	float intersectionX = 0.0f;
	float intersectionY = 0.0f;

	if (dax != 0)
	{
		intersectionX = (day * day * m2 - day * dax * n2 + day * dax * PointY + dax * dax * PointX) / (dax * dax + day * day + 0.000001);
		intersectionY = day * (intersectionX - m2) / (dax + 0.000001) + n2;

	}
	else
	{
		intersectionX = m2;
		intersectionY = dax * (intersectionX - PointX) / (-day + 0.000001) + PointY;
	}

	//    *px = PointX;
	//    *py = PointY;
	cog.y = PointX;
	cog.x = PointY;
	cog.z = (a.z + b.z + c.z) / 3;
	// float temp1 = (intersectionX - PointX) * (intersectionX - PointX) + (intersectionY - PointY) * (intersectionY - PointY);
	// *pr = sqrtf((intersectionX - PointX) * (intersectionX - PointX) + (intersectionY - PointY) * (intersectionY - PointY));
	return cog;
}

char point_is_right_line_with_arrow(END_POS point, float yaw_line, float k, float b)//点在直线右侧
{
	float ya = 0;
	float xa = k * ya + b;

	float yb = ya + sind(yaw_line);
	float xb = k * yb + b;

	float f = (yb - ya)*(point.x - xa) - (point.y - ya)*(xb - xa);

	if (f < 0)
		return -1;
	else if (f > 0)
		return 1;//在右侧
	else
		return 0;
}

char point_is_front_line_with_arrow(END_POS point, float yaw_90, float k, float b)//点在直线前方
{
	if (point_is_right_line_with_arrow(point, yaw_90 - 90, k, b) == 1)
		return 1;
	else
		return 0;
}

float cal_dis_point_line(END_POS point, float k, float b)//点到直线的距离
{
	return fabs(k*point.y - point.x + b) / sqrtf(k*k + 1);
}

char check_leg_out_space(char leg, float chech_band, float *band_dis)//g
{
	char i, j;
	END_POS now_pos = vmc[leg].epos;
	float dis[2];
	if (vmc[leg].epos.x > 0)
	{
		dis[0] = (MAX_X*chech_band - now_pos.x);
	}
	else
		dis[0] = -(MIN_X*chech_band - now_pos.x);

	if (vmc[leg].epos.y > 0)
	{
		dis[1] = (MAX_Y*chech_band - now_pos.y);
	}
	else
		dis[1] = -(MIN_Y*chech_band - now_pos.y);

	for (i = 0; i < 2; i++)  //距离从小到大排序
	{
		for (j = 2 - 1; j > i; j--)
		{
			if (dis[j] < dis[j - 1])
			{
				swap(&dis[i], &dis[j]);
			}
		}
	}
	*band_dis = dis[0];

	if (dis[0] < 0 || dis[1] < 0)
		return 1;
	else
		return 0;
}


float cal_yaw_xy(float x, float y)//坐标系前向为x  顺时针为正
{
	return  90 - fast_atan2(x, y)*57.3;
}


float cal_stable_area(void)//计算稳定区域g 全局
{
	float st_value = 0;
	char i = 0, j = 0;
	END_POS leg_g3[3];
	END_POS leg_g4[4];

	if (walk_gait.ground_num == 4)//4腿支撑
	{
		leg_g4[0] = vmc[0].epos_nn;
		leg_g4[1] = vmc[1].epos_nn;
		leg_g4[2] = vmc[2].epos_nn;
		leg_g4[3] = vmc[3].epos_nn;
		if (inTrig2(walk_gait.now_zmp_nn, leg_g4[0], leg_g4[1], leg_g4[2], leg_g4[3]))
		{
			st_value = cal_steady_s4(walk_gait.now_zmp_nn, leg_g4[0], leg_g4[1], leg_g4[2], leg_g4[3]);
		}
	}
	else if (walk_gait.ground_num == 3)//3腿支撑
	{
		for (i = 0; i < 4; i++)
		{
			if (vmc[i].ground)
			{
				leg_g3[j++] = vmc[i].epos_nn;
			}
		}
		if (inTrig(walk_gait.now_zmp_nn, leg_g3[0], leg_g3[1], leg_g3[2]))
		{
			st_value = cal_steady_s3(walk_gait.now_zmp_nn, leg_g3[0], leg_g3[1], leg_g3[2]);
		}

	}
	else
		st_value = 0;


	return st_value;
}


//------------------------------------------------------------------
//点在直线上
u8 check_point_on_line(END_POS point, float k, float b, float err)
{
	float temp = k * point.y + b - point.x;
	if (ABS(temp) < err)
		return 1;
	else
		return 0;
}

//点在点矢量方向前
u8 check_point_front_arrow(END_POS point, END_POS vector_p, float yaw)//g
{
	float tyaw = 90 - yaw + 0.000011;
	float kc_90 = -1 / tand(tyaw);
	float bc_90 = vector_p.x - kc_90 * vector_p.y;
	float cy_t = vector_p.y + sind(yaw) * 1, cx_t = vector_p.x + cosd(yaw) * 1;
	int flag[2];
	flag[0] = kc_90 * cy_t + bc_90 - cx_t;
	flag[1] = kc_90 * point.y + bc_90 - point.x;
	if ((flag[0] > 0 && flag[1] > 0) || (flag[0] < 0 && flag[1] < 0))
		return 1;
	else
		return 0;
}

u8 check_point_front_arrow_n(END_POS point, END_POS vector_p, float yaw)//g
{
	float a90, b90, c90;
	line_function_from_arrow_n(vector_p.x, vector_p.y, yaw + 90, &a90, &b90, &c90);

	float a, b, c;
	line_function_from_arrow_n(point.x, point.y, yaw, &a, &b, &c);

	float cx = 0, cy = 0;
	char temp = cross_point_of_lines_n(a90, b90, c90, a, b, c, &cx, &cy);//g

	if (!temp)
		return 99;

	float r = MIN_F_DIS;
	float fx = point.x + cosd(yaw)*MIN_F_DIS;
	float fy = point.y + sind(yaw)*MIN_F_DIS;

	float disf = cal_dis_of_points(fx, fy, cx, cy);
	float dis = cal_dis_of_points(point.x, point.y, cx, cy);

	if (disf >= dis)
		return 1;
	else
		return 0;
}

//判断两点在线同一侧
u8 check_points_same_side(float x1, float y1, float x2, float y2, float k, float b)
{
	int flag[2];
	flag[0] = k * x1 + b - y1;
	flag[1] = k * x2 + b - y2;
	if (flag[0] * flag[1] > 0)
		return 1;
	else
		return 0;
}


////点矢量与直线交点
//u8 check_cross_arrow_line(float cx,float cy,float yaw,float k,float b,float *x,float *y)
//{ 
//  float tyaw=90-yaw+0.000011;
//	float kc=tand(tyaw);
//	float bc=cy-kc*cx;
//	float cro_x,cro_y;
//	u8 flag;	
//	flag=cross_point_of_lines(k,b,kc,bc,&cro_x,&cro_y);
//	*x=cro_x;
//	*y=cro_y;
//	
//	if(flag==0||fabs(cro_y)>100||fabs(cro_x)>100)
//		return 0;
//	//有交点且在前方
//	if(check_point_front_arrow(cro_x,cro_y, cx, cy, yaw))	
//	return 1;
//	else{
//	*x=*y=0;	
//	return 0;
//	}
//}	

//点矢量垂线与直线交点
u8 check_cross_arrow90_line(float cx, float cy, float yaw, float k, float b, float *x, float *y)
{
	float tyaw = 90 - yaw + 0.000011;
	float kc = tand(tyaw);
	float kc_90 = -1 / (kc + 0.00001);
	float bc_90 = cy - kc_90 * cx;
	float cro_x, cro_y;

	u8 flag;
	flag = cross_point_of_lines(k, b, kc_90, bc_90, &cro_x, &cro_y);
	*x = cro_x;
	*y = cro_y;

	if (flag == 0) {
		*x = *y = 0;
		return 0;
	}
	else
		return 1;
}

//计算两点距离
float cal_dis_of_points(float x1, float y1, float x2, float y2)//g
{
	return sqrtf(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

//判断一个点在椭圆内部
u8 in_circle(END_POS center, END_POS point, float d_short, float d_long)
{
	float temp = pow(point.y - center.y, 2) / pow(d_short, 2) + pow(point.x - center.x, 2) / pow(d_long, 2);
	if (temp > 1)//外面
		return 0;
	else
		return 1;
}

//点到直线距离
float dis_point_to_line(END_POS point, float k, float b)//g
{
	float k_90 = -1 / (k + 0.000001);
	float b_90 = point.x - k_90 * point.y;
	float cx, cy;
	cross_point_of_lines(k, b, k_90, b_90, &cx, &cy);

	return cal_dis_of_points(point.x, point.y, cx, cy);
}



////点矢量与四边形交点
u8 check_point_to_tangle(float x, float y, float yaw, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4
	, float *jiao1_x, float *jiao1_y, float *jiao2_x, float *jiao2_y)
{
	u8 i, j = 0;
	float k[4], b[4];
	float kc, bc;
	line_function_from_two_point(x1, y1, x2, y2, &k[0], &b[0]);
	line_function_from_two_point(x2, y2, x3, y3, &k[1], &b[1]);
	line_function_from_two_point(x3, y3, x4, y4, &k[2], &b[2]);
	line_function_from_two_point(x4, y4, x1, y1, &k[3], &b[3]);
	line_function_from_arrow(x, y, yaw, &kc, &bc);
	u8 flag[4];
	float cro_x[4], cro_y[4];
	flag[0] = cross_point_of_lines(kc, bc, k[0], b[0], &cro_x[0], &cro_y[0]);
	flag[1] = cross_point_of_lines(kc, bc, k[1], b[1], &cro_x[1], &cro_y[1]);
	flag[2] = cross_point_of_lines(kc, bc, k[2], b[2], &cro_x[2], &cro_y[2]);
	flag[3] = cross_point_of_lines(kc, bc, k[3], b[3], &cro_x[3], &cro_y[3]);


	if (flag[0] || flag[1] || flag[2] || flag[3] == 1)
	{
		float jiaodiao1[2][2];
		for (i = 0; i < 4; i++)
		{
			if (flag[i] && (fabs(cro_x[i]) < fabs(x2) + fabs(x1)) && (fabs(cro_y[i]) < fabs(y2) + fabs(y3)))
			{
				jiaodiao1[j][Xr] = cro_x[i]; jiaodiao1[j][Yr] = cro_y[i];
				j++;
				if (j > 2)
					break;
			}
		}

		if (j < 1)
			return 0;

		u8 flag2;
		END_POS point, arrow;
		point.x = jiaodiao1[0][Xr];
		point.y = jiaodiao1[0][Yr];
		arrow.x = x;
		arrow.y = y;
		flag2 = check_point_front_arrow(point, arrow, yaw);

		if (flag2) {
			*jiao1_x = jiaodiao1[0][Xr];
			*jiao1_y = jiaodiao1[0][Yr];
			*jiao2_x = jiaodiao1[1][Xr];
			*jiao2_y = jiaodiao1[1][Yr];
		}
		else
		{
			*jiao1_x = jiaodiao1[1][Xr];
			*jiao1_y = jiaodiao1[1][Yr];
			*jiao2_x = jiaodiao1[0][Xr];
			*jiao2_y = jiaodiao1[0][Yr];
		}
		return 1;
	}
	else
		return 0;
}


////点矢量与四边形交点
u8 check_point_to_tangle_n(float x, float y, float yaw, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4
	, float *jiao1_x, float *jiao1_y, float *jiao2_x, float *jiao2_y)
{
	u8 i, j = 0;
	float a[4], b[4], c[4];
	float ac, bc, cc;
	float jiaodiao1[2][2];
	line_function_from_two_point_n(x1, y1, x2, y2, &a[0], &b[0], &c[0]);
	line_function_from_two_point_n(x2, y2, x3, y3, &a[1], &b[1], &c[1]);
	line_function_from_two_point_n(x3, y3, x4, y4, &a[2], &b[2], &c[2]);
	line_function_from_two_point_n(x4, y4, x1, y1, &a[3], &b[3], &c[3]);
	line_function_from_arrow_n(x, y, yaw, &ac, &bc, &cc);

	u8 flag[4];
	float cro_x[4] = { 0 }, cro_y[4] = { 0 };
	flag[0] = cross_point_of_lines_n(ac, bc, cc, a[0], b[0], c[0], &cro_x[0], &cro_y[0]);
	flag[1] = cross_point_of_lines_n(ac, bc, cc, a[1], b[1], c[1], &cro_x[1], &cro_y[1]);
	flag[2] = cross_point_of_lines_n(ac, bc, cc, a[2], b[2], c[2], &cro_x[2], &cro_y[2]);
	flag[3] = cross_point_of_lines_n(ac, bc, cc, a[3], b[3], c[3], &cro_x[3], &cro_y[3]);


	if (flag[0] || flag[1] || flag[2] || flag[3] == 1)
	{
		j = 0;
		for (i = 0; i < 4; i++)
		{
			if (flag[i] && (fabs(cro_x[i]) < fabs(x2) + fabs(x1)) && (fabs(cro_y[i]) < fabs(y2) + fabs(y3)))//简单保护
			{
				jiaodiao1[j][Xr] = cro_x[i]; jiaodiao1[j][Yr] = cro_y[i]; j++;
				if (j > 2)//交点过多
					return 0;
			}
		}

		if (j < 1)//没有交点
			return 0;

		u8 flag2;
		END_POS point, arrow;
		point.x = jiaodiao1[0][Xr];
		point.y = jiaodiao1[0][Yr];
		arrow.x = x;
		arrow.y = y;
		flag2 = check_point_front_arrow_n(point, arrow, yaw);

		if (flag2) {
			*jiao1_x = jiaodiao1[0][Xr];
			*jiao1_y = jiaodiao1[0][Yr];
			*jiao2_x = jiaodiao1[1][Xr];
			*jiao2_y = jiaodiao1[1][Yr];
		}
		else
		{
			*jiao1_x = jiaodiao1[1][Xr];
			*jiao1_y = jiaodiao1[1][Yr];
			*jiao2_x = jiaodiao1[0][Xr];
			*jiao2_y = jiaodiao1[0][Yr];
		}
		return 1;
	}
	else
		return 0;
}

////点矢量与三角形交点
//u8 check_point_to_trig(float x,float y,float yaw,float x1,float y1,float x2,float y2,float x3,float y3
//	,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
//{
//	u8 i,j=0;
//  float k[3],b[3];
//  float kc,bc;
//	line_function_from_two_point(x1,y1,x2,y2,&k[0],&b[0]);
//	line_function_from_two_point(x2,y2,x3,y3,&k[1],&b[1]);
//	line_function_from_two_point(x3,y3,x1,y1,&k[2],&b[2]);
//  line_function_from_arrow(x,y,yaw,&kc,&bc);
//	u8 flag[3];
//	float cro_x[3],cro_y[3];
//  flag[0]=cross_point_of_lines(kc,bc,k[0],b[0],&cro_x[0],&cro_y[0]);
//	flag[1]=cross_point_of_lines(kc,bc,k[1],b[1],&cro_x[1],&cro_y[1]);
//	flag[2]=cross_point_of_lines(kc,bc,k[2],b[2],&cro_x[2],&cro_y[2]);
//  
//	float dis[3];
//	dis[0]=cal_dis_of_points(x1,y1,x,y);
//	dis[1]=cal_dis_of_points(x2,y2,x,y);
//	dis[2]=cal_dis_of_points(x3,y3,x,y);
//	
//	
//	if(flag[0]||flag[1]||flag[2])
//	{	
//	float jiaodiao1[2][2],temp;
//	for(i=0;i<3;i++)
//		{
//			temp=cal_dis_of_points(cro_x[i],cro_y[i],x,y);
//		  if(flag[i]&&(temp<dis[0]||temp<dis[1]||temp<dis[2]))
//			{ jiaodiao1[j][Xr]=cro_x[i];jiaodiao1[j][Yr]=cro_y[i];
//        j++;
//				if(j>2)
//					break;
//      }				
//		}
//	
//	if(j<1)
//		return 0;
//	
//  u8 flag2;	
//	flag2=check_point_front_arrow(jiaodiao1[0][Xr],jiaodiao1[0][Yr],x,y,yaw);
//		
//	 if(flag2){
//	 *jiao1_x=jiaodiao1[0][Xr];
//	 *jiao1_y=jiaodiao1[0][Yr];
//	 *jiao2_x=jiaodiao1[1][Xr];
//	 *jiao2_y=jiaodiao1[1][Yr];}
//	 else
//		{
//	 *jiao1_x=jiaodiao1[1][Xr];
//	 *jiao1_y=jiaodiao1[1][Yr];
//	 *jiao2_x=jiaodiao1[0][Xr];
//	 *jiao2_y=jiaodiao1[0][Yr];} 
//	return 1;
//	}else 
//	return 0;
//}

//计算椭圆与矢量的两个交点
void cal_jiao_of_tuo_and_line(float cx, float cy, float d_short, float d_long, float yaw, float *jiao1_x, float *jiao1_y, float *jiao2_x, float *jiao2_y)
{
	float tyaw = 90 - yaw + 0.000011;
	float tan_yaw = tand(tyaw);
	float jiaodiao[2][2] = { 0 };
	//计算速度直线与椭圆交点
	float temp = sqrtf(pow(d_short, 2) / (1 + pow(d_short*tan_yaw / d_long, 2)));
	//判断速度方向交点符号
	if (yaw + 0.000011 > 0 && yaw + 0.000011 < 180)
	{
		jiaodiao[0][Xr] = temp; jiaodiao[1][Xr] = -temp;
	}
	else
	{
		jiaodiao[0][Xr] = -temp; jiaodiao[1][Xr] = temp;
	}

	jiaodiao[0][Yr] = tan_yaw * jiaodiao[0][Xr];
	jiaodiao[1][Yr] = tan_yaw * jiaodiao[1][Xr];

	jiaodiao[0][Xr] += cx; jiaodiao[1][Xr] += cx;
	jiaodiao[0][Yr] += cy; jiaodiao[1][Yr] += cy;

	*jiao1_x = jiaodiao[0][Xr];
	*jiao1_y = jiaodiao[0][Yr];
	*jiao2_x = jiaodiao[1][Xr];
	*jiao2_y = jiaodiao[1][Yr];
}


//计算当前稳态余量
float cal_steady_s3(END_POS point, END_POS p1, END_POS p2, END_POS p3)//g
{
	float ST;
	float k[3], b[3];
	float Dd[3], temp;
	u8 i, intrig;
	intrig = inTrig(point, p1, p2, p3);
	if (intrig) {
		if (p2.y == p1.y)
			p2.y += 0.001;
		if (p3.y == p1.y)
			p1.y += 0.001;

		line_function_from_two_point(p1.x, p1.y, p2.x, p2.y, &k[0], &b[0]);
		line_function_from_two_point(p3.x, p3.y, p2.x, p2.y, &k[1], &b[1]);
		line_function_from_two_point(p1.x, p1.y, p3.x, p3.y, &k[2], &b[2]);

		for (i = 0; i < 3; i++)
			Dd[i] = dis_point_to_line(point, k[i], b[i]);

		ST = Dd[0];
		for (i = 1; i < 3; i++)
			if (Dd[i] < ST)
				ST = Dd[i];

		if (ST > 0)
			return ST;
		else
			return 0;
	}
	else
		return 0;
}

//计算当前稳态余量4leg
float cal_steady_s4(END_POS point, END_POS p1, END_POS p2, END_POS p3, END_POS p4)//g
{
	float ST;
	float k[4], b[4];
	float Dd[4], temp;
	u8 i, intrig;

	line_function_from_two_point(p1.x, p1.y, p2.x, p2.y, &k[0], &b[0]);
	line_function_from_two_point(p2.x, p2.y, p4.x, p4.y, &k[1], &b[1]);
	line_function_from_two_point(p4.x, p4.y, p3.x, p3.y, &k[2], &b[2]);
	line_function_from_two_point(p3.x, p3.y, p1.x, p1.y, &k[3], &b[3]);
	for (i = 0; i < 4; i++)
		Dd[i] = dis_point_to_line(point, k[i], b[i]);

	ST = Dd[0];
	for (i = 1; i < 4; i++)
		if (Dd[i] < ST)
			ST = Dd[i];

	if (ST > 0)
		return ST;
	else
		return 0;
}

void cal_tri_scale_point(END_POS p1, END_POS p2, END_POS p3, END_POS* p11, END_POS* p22, END_POS* p33, float scale)//计算三角形缩放顶点
{



}
