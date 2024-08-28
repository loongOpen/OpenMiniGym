//#include "include.h"
//#include "gait_math.h"
//#include "force_dis_8.h"
//#include "locomotion_header.h"
//#include <iostream>
//#include <fstream>
//#include <string>
//#include "draw_local_map_good_location_cube.h"
//using namespace std;
////webots下绘制函数
//WEBTOS_DRAWING webots_draw, webots_draw_n;
//WEBTOS_DRAWING_RX webots_draw_rx;

//_HEIGHT_MAP fake_map[10], guass_map,cube_map[CUBE_NUM];
//_GLOBAL_MAP global_map;
///*
//X+  H
//|
//o--Y+ W
//*/
//#define SIZE_D 50.f
//void find_grid_on_map_n(END_POS point, int* grid_x, int* grid_y) {//全局地图中点的网格 由于数组只能0开始因此移动半个地图
//#if 1
//	int grid_int[2] = { (int)(point.x * SIZE_D),(int)(point.y * SIZE_D) };
//	int size_int = (int)(MAP_SIZE * SIZE_D);
//	*grid_x = grid_int[Xr] / size_int + grid_int[Xr] % size_int + MAP_H_N / 2;
//	*grid_y = grid_int[Yr] / size_int + grid_int[Yr] % size_int + MAP_W_N / 2;
//#else
//	float grid_int[2] = { point.x ,point.y };
//	float size_int = MAP_SIZE;
//	*grid_x = (int)(grid_int[Xr] / size_int) + fmod(grid_int[Xr], size_int) + MAP_H_N / 2;
//	*grid_y = (int)(grid_int[Yr] / size_int) + fmod(grid_int[Yr], size_int) + MAP_W_N / 2;
//#endif

//	*grid_x = LIMIT(*grid_x, 0, MAP_H_N);
//	*grid_y = LIMIT(*grid_y, 0, MAP_W_N);
//}

//void find_grid_on_map_local(END_POS point, int* grid_x, int* grid_y) {
//#if 1
//	int grid_int[2] = { point.x * SIZE_D,point.y * SIZE_D };
//	int size_int = MAP_SIZE * SIZE_D;
//	*grid_x = grid_int[Xr] / size_int + grid_int[Xr] % size_int + MAP_H / 2;
//	*grid_y = grid_int[Yr] / size_int + grid_int[Yr] % size_int + MAP_W / 2;
//#else
//	float grid_int[2] = { point.x ,point.y };
//	float size_int = MAP_SIZE;
//	*grid_x = (int)(grid_int[Xr] / size_int) + fmod(grid_int[Xr], size_int) + MAP_H / 2;
//	*grid_y = (int)(grid_int[Yr] / size_int) + fmod(grid_int[Yr], size_int) + MAP_W / 2;
//#endif

//	*grid_x = LIMIT(*grid_x, 0, MAP_H);
//	*grid_y = LIMIT(*grid_y, 0, MAP_W);
//}

//#include "foot_sel_on_map.h"
//#include "foot_sel_on_map_emxutil.h"
//#include "foot_good_foot_found.h"

//float cog_off_ground_real = 0;
//int location_select_on_map(char sel, END_POS slip_n, char mode, END_POS *location) {
//	END_POS leg_n, ankle_n;
//	int i, j, k;
//	float spd_yaw;
//	int spd_flag = 1;
//	float k1, b1;

//	float location_map[MAP_H*MAP_W];
//	float map_edge_out[MAP_H*MAP_W];
//	float map_local[MAP_H*MAP_W];
//	float map_local_foot[3];
//	float robot_param[3] = { Hw,Www, MAX_X };
//	float map_param[5] = { MAP_H,MAP_W,MAP_H_N,MAP_W_N,1 };

//	double check_param[3] = {2,-0.07,0.22 };//bind =1

	
//	if (_hu_model.fake_map_guass == 2) {
//		check_param[0] = 0;
//	}

//	for (i = 0; i < MAP_H; i++) {//局部地图转换matlab格式
//		for (j = 0; j < MAP_W; j++) {
//			map_local[MAP_H*i + j] = global_map.h_grid_local[i][j] - cog_off_ground_real;
//		}
//	}
//	//printf("cog_off_ground_real=%f\n", cog_off_ground_real);
//	foot_sel_on_map(
//		map_local,
//		map_param,
//		check_param,
//		location_map,
//		map_edge_out
//	);

//	//输出到共享内存
//	for (i = 0; i < MAP_H; i++) {
//		for (j = 0; j < MAP_W; j++) {

//			global_map.h_grid_local_good[i][j] = location_map[MAP_H*i + j];
//			global_map.h_grid_local_edge[i][j] = map_edge_out[MAP_H*i + j];

//		}
//	}
//#if 0//采用matlab导出
//	double now_foot[3] = { vmc[sel].epos_n.x,vmc[sel].epos_n.y,vmc[sel].epos_n.z };
//	double slip_foot[3] = { slip_n.x,slip_n.y,slip_n.z };
//	double gfoot[3] = { 0 };
//	double is_found = 0;
//	foot_good_foot_found(
//		sel,
//		vmc_all.param.tar_spd_use_rc.x,
//		now_foot,
//		slip_foot,
//		location_map,
//		map_param,
//		check_param,
//		robot_param,
//		gfoot,
//		&is_found);
//	int grid_vision[2];
//	END_POS pos_vision;

//	pos_vision.x = location->x = gfoot[0];
//	pos_vision.y = location->y = gfoot[1];
//	find_grid_on_map_local(pos_vision, &grid_vision[Xr], &grid_vision[Yr]);
//	location->z = global_map.h_grid_local_good[grid_vision[Xr]][grid_vision[Yr]];
//	if (is_found)
//		return 1;
//	else
//		return 0;
//#else
//	if ((sel == 1 || sel == 3) && 0)
//		return 0;


 
//#if EN_SHOW_MAP1||0
//	static int cnt_p = 99;
//	if (cnt_p++ > 20) {
//		cnt_p = 0;
//		printf("centre_cell_inner[%f]:---------------------------------------------------------------------------------------------------------------------------------------\n", vmc_all.pos_n.z);
//		for (int i = 0; i < MAP_H ; i++)
//		{
//			for (int j = 0; j < MAP_W; j++) {
//				//printf("%f ",global_map.h_grid_local_edge[i][j]);
//#if EN_CUBE
//				if (global_map.h_grid_local_good[MAP_H - 1 - i][j] > 0.03f)
//#else
//				if (global_map.h_grid_local_good[MAP_H - 1 - i][j] > -0.08f)
//#endif
//					printf("* ");
//				//             else if(global_map.h_grid_local_edge[MAP_H-1-i][j]>1)
//				//                printf("x ");
//				else
//					printf("  ");
//			}
//			printf("\n");
//		}
//		printf("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n", vmc_all.pos_n.z);
//	}
//#endif

//	//识别梅花桩中心
//#if EN_CUBE_CENTER||1

//	int centroids[40] = { 0 };
//	int id_cube[2] = { 0 };
//	END_POS cube_center[20];
//	int num = 0;
//	draw_local_map_good_location_cube(location_map, check_param, centroids, &num);
//	//printf("num=%d\n", num);
//	if (num) {
//		for (int i = 0; i < num; i++) {
//			id_cube[Yr] = centroids[i];
//			id_cube[Xr] = centroids[i + 20];
//			//printf("id=%d %d %d\n", i, id_cube[Xr], id_cube[Yr]);
//			cube_center[i].x = id_cube[Xr] * MAP_SIZE - MAP_H / 2 * MAP_SIZE;
//			cube_center[i].y = id_cube[Yr] * MAP_SIZE - MAP_W / 2 * MAP_SIZE;
//			//printf("id=%d %f %f\n", i, cube_center[i].x, cube_center[i].y);
//		}
//	}

//	if (vmc[sel].cube_lock == 1)//first
//		vmc[sel].cube_lock = 2;
//	else if (vmc[sel].cube_lock == 2) {//check cube in this swing phase
//		if (num)//have cube
//			vmc[sel].cube_lock = 3;
//		else//no cube
//			vmc[sel].cube_lock = 0;
//	}
//#endif
 
//	int good_location_flag[3] = { 0 };
//	END_POS nominal_pos;
//	END_POS lift_pos;
//	int grid_ankle[2];
//	int grid_nominal[2];
//	lift_pos = vmc[sel].epos_lf_n;//未设定
//	leg_n = vmc[sel].epos_n;//input
//	ankle_n.x = vmc_all.H*vmc[sel].flag_fb / 2;
//	ankle_n.y = vmc_all.W*vmc[sel].flag_rl / 2;
//	nominal_pos = slip_n;
//	//if (vmc[sel].err)
//	//	printf("[%d] %f %f | %f %f\n", sel, slip_n.x, slip_n.y, pos[0], pos[1]);
// //

//	//cube center min to slip
//	int id_min_to_slip = 0;
//	int cube_good = 0;
//	float dis_cube = 99;
//	for (j = 0; j < num; j++) {
//		float temp = sqrtf(powf(cube_center[j].x - nominal_pos.x, 2) + powf(cube_center[j].y - nominal_pos.y, 2));
//		if (temp < dis_cube) {
//			dis_cube = temp;
//			id_min_to_slip = j;
//			cube_good = 1;
//		}
//	}

//	float zmp_check_x = vmc_all.H / 2 *0.3f;//0.6
//	float zmp_check_y = vmc_all.W / 2 *0.15f;
//	int cube_flag[3] = { 0 };
//	//check cube
//	if ((cube_center[id_min_to_slip].x < zmp_check_x && (sel == 0 || sel == 2))
//		|| (cube_center[id_min_to_slip].x > -zmp_check_x && (sel == 1 || sel == 3))
//		)//ZMP约束
//		cube_flag[1] = 1;//找到但机体缺少ZMP了

//        if ((cube_center[id_min_to_slip].y < zmp_check_y  && (sel == 0 || sel == 1))
//            || (cube_center[id_min_to_slip].y > -zmp_check_y && (sel == 2 || sel == 3)))
//            cube_flag[2] = 1;

//	if (cube_flag[1] || cube_flag[2])//不满足ZMP约束向后选择
//		cube_good = 0;

//	find_grid_on_map_local(ankle_n, &grid_ankle[Xr], &grid_ankle[Yr]);
//	find_grid_on_map_local(nominal_pos, &grid_nominal[Xr], &grid_nominal[Yr]);//SLIP的位置

//	int search_wide[2] = { 16,16 };//搜索的半径
//	int star_x = 0;
//	int star_y = 0;
//	int end_x = 0;
//	int end_y = 0;
//	star_x = LIMIT(grid_nominal[Xr] - search_wide[Xr], 0, MAP_H);
//	end_x = LIMIT(grid_nominal[Xr] + search_wide[Xr], 0, MAP_H);
//	star_y = LIMIT(grid_nominal[Yr] - search_wide[Yr], 0, MAP_W);
//	end_y = LIMIT(grid_nominal[Yr] + search_wide[Yr], 0, MAP_W);
//	//slip一定范围内的点
//	END_POS grid_good1[1600];
//	END_POS pos_good1[1600];
//	int cnt_good1 = 0;

//	for (int i = star_x; i < end_x; i++) {
//		for (int j = star_y; j < end_y; j++) {
//			if (global_map.h_grid_local_good[i][j] >= check_param[1]
//				&& global_map.h_grid_local_good[i][j] <= check_param[2]
//				)//寻找满足高度限制
//			{
//				grid_good1[cnt_good1].x = i;
//				grid_good1[cnt_good1].y = j;

//				pos_good1[cnt_good1].x = i * MAP_SIZE - MAP_H / 2 * MAP_SIZE;
//				pos_good1[cnt_good1].y = j * MAP_SIZE - MAP_W / 2 * MAP_SIZE;
//				cnt_good1++;
//			}
//		}
//	}

//	END_POS grid_good2[1600];
//	END_POS pos_good2[1600];
//	int cnt_good2 = 0;
//	char cross_mid_line = 0;//过中线与碰撞保护
//	char zmp_limit = 0;
//	//避碰与ZMP约束   硬约束
//	if (cnt_good1 > 0) {
//		for (int i = 0; i < cnt_good1; i++)
//		{
//			END_POS diag_pos;
//			cross_mid_line = zmp_limit = good_location_flag[1] = good_location_flag[2] = 0;

//			//避开碰撞
//			if (sel == 0)
//				diag_pos = vmc[2].epos_n;
//			if (sel == 2)
//				diag_pos = vmc[0].epos_n;
//			if (sel == 1)
//				diag_pos = vmc[3].epos_n;
//			if (sel == 3)
//				diag_pos = vmc[1].epos_n;

//			float dis = sqrt(powf(diag_pos.x - pos_good1[i].x, 2) + powf(diag_pos.y - pos_good1[i].y, 2));
//#if 1
//			if ((sel == 0 || sel == 1) && 1) {//过中线
//				if (pos_good1[i].y < vmc_all.W / 2.f *0.25f
//					)
//					cross_mid_line = 1;
//			}
//			else {
//				if (pos_good1[i].y > -vmc_all.W / 2.f *0.25f
//					)
//					cross_mid_line = 1;
//			}
//#endif

//#if 1
//			if ((pos_good1[i].x < zmp_check_x && (sel == 0 || sel == 2))
//				|| (pos_good1[i].x > -zmp_check_x && (sel == 1 || sel == 3))
//				)//ZMP约束
//				good_location_flag[1] = 1;//找到但机体缺少ZMP了

//			if ((pos_good1[i].y < zmp_check_y && (sel == 0 || sel == 1))
//				|| (pos_good1[i].y > -zmp_check_y && (sel == 2 || sel == 3)))
//				good_location_flag[2] = 1;

//			if (good_location_flag[1] || good_location_flag[2])//不满足ZMP约束向后选择
//				zmp_limit = 1;
//#endif
//			if (dis < 0.1f || cross_mid_line || zmp_limit) {
//				//太近
//			}
//			else
//			{
//				grid_good2[cnt_good2].x = grid_good1[i].x;
//				grid_good2[cnt_good2].y = grid_good1[i].y;

//				pos_good2[cnt_good2].x = pos_good1[i].x;
//				pos_good2[cnt_good2].y = pos_good1[i].y;
//				cnt_good2++;
//			}
//		}
//	}

//	float score_grid[1600] = { 0 };//网格得分  软约束
//	float score_gain[6] = { -0.25, -1, 0, -0.15 };///距离  slip   0.25            越小 权重越大  不能比slip大
//	int grid_id[1600] = { 0 };
//	END_POS pos_neig;
//	//权重zZ
//	if (cnt_good2 > 0) {
//		for (int i = 0; i < cnt_good2; i++)
//		{
//			//基础得分 分数高 优势大
//			score_grid[i] = 50;

//			//最小跨步长度权重
//			float dis1 = sqrtf(powf(lift_pos.x - pos_good2[i].x, 2) + powf(lift_pos.y - pos_good2[i].y, 2));

//			score_grid[i] += LIMIT(score_gain[0] * dis1, -15, 15);
//			//最小与SLIP点的距离
//			float dis2 = sqrtf(powf(nominal_pos.x - pos_good2[i].x, 2) + powf(nominal_pos.y - pos_good2[i].y, 2));
//			score_grid[i] += LIMIT(score_gain[1] * dis2, -25, 25);

//			//cube center
//#if EN_CUBE_CENTER||1
//			if (num&&vmc[sel].cube_lock==3) {
//				float dis_cube = sqrtf(powf(cube_center[id_min_to_slip].x - pos_good2[i].x, 2)
//					+ powf(cube_center[id_min_to_slip].y - pos_good2[i].y, 2));
//				if (dis_cube <= MAP_SIZE * 2.f) {
//					//printf("ss");
//					score_grid[i] += 5;
//				}
//			}
//#endif
//			//离开对角腿的距离  不加好一些  增大支撑区域目标 w2
//			switch (sel) {
//			case 0:pos_neig.x = vmc[1].epos_lf_n.x; pos_neig.y = vmc[1].epos_lf_n.y; break;
//			case 1:pos_neig.x = vmc[0].epos_lf_n.x; pos_neig.y = vmc[0].epos_lf_n.y; break;
//			case 2:pos_neig.x = vmc[3].epos_lf_n.x; pos_neig.y = vmc[3].epos_lf_n.y; break;
//			case 3:pos_neig.x = vmc[2].epos_lf_n.x; pos_neig.y = vmc[2].epos_lf_n.y; break;
//			default:pos_neig = pos_good2[i]; break;
//			}
//			float dis3 = sqrtf(powf(pos_neig.x - pos_good2[i].x, 2) + powf(pos_neig.y - pos_good2[i].y, 2));
//			score_grid[i] += LIMIT(score_gain[2] * dis3, -15, 15);//距离约大权重越大

//			//最小上一次落足距离 w3
//			float dis4 = sqrtf(powf(vmc[sel].opt_epos_n.x - pos_good2[i].x, 2) + powf(vmc[sel].opt_epos_n.y - pos_good2[i].y, 2));
//			score_grid[i] += LIMIT(score_gain[3] * dis4, -15, 15);

//			//final score
//			score_grid[i] = LIMIT(score_grid[i], 0, 100);
//			grid_id[i] = i;
//		}
//	}

//	// 越小 权重越大
//	if (cnt_good2 > 0) {
//		good_location_flag[0] = 1;//有满足的点
//		bubble_sort(score_grid, grid_id, cnt_good2);//得分排序从大到小
//		//printf("%f %f %f\n", score_grid[0], score_grid[1], score_grid[2]);
//		//printf("%d %d %d %d\n", grid_id[0], grid_id[1], grid_id[2], max_score_id);
//	}
//	else
//		good_location_flag[0] = 0;


//	int good_st_id = 0;//除去最高分
//	//if (cnt_good2 < 1)good_st_id = 0;

//	if (good_location_flag[0]) {
//		float flt_cube = 0.8;
//		if (num&&cube_good
//			&&vmc[sel].cube_lock == 3
//			&& 0) {//have cube can use cube pos to fix
//			location->x = cube_center[id_min_to_slip].x*flt_cube + (1 - flt_cube)*pos_good2[grid_id[good_st_id]].x;
//			location->y = cube_center[id_min_to_slip].y*flt_cube + (1 - flt_cube)*pos_good2[grid_id[good_st_id]].y;
//		}
//		else {
//			location->x = pos_good2[grid_id[good_st_id]].x;
//			location->y = pos_good2[grid_id[good_st_id]].y;
//		}
//		location->z = global_map.h_grid_local_good[(int)grid_good2[grid_id[good_st_id]].x][(int)grid_good2[grid_id[good_st_id]].y];
//		vmc[sel].opt_epos_n.x = location->x;
//		vmc[sel].opt_epos_n.y = location->y;
//		vmc[sel].opt_epos_n.z = location->z;
//		//if (sel == 1||1)
//		//{
//		//	printf("[%d]found foot! %d %d %d||%d %d %d\n",
//		//		sel,
//		//		good_location_flag[0],
//		//		good_location_flag[1],
//		//		good_location_flag[2],
//		//		cnt_good1, cnt_good2,
//		//		0);
//		//}
//		return 1;
//	}
//	else {
 

//		//if (sel == 1 || 1)
//		//{
//		//	printf("[%d]not found foot! %d %d %d||%d %d %d\n",
//		//		sel,
//		//		good_location_flag[0],
//		//		good_location_flag[1],
//		//		good_location_flag[2],
//		//		cnt_good1,
//		//		cnt_good2,
//		//		0);
//		//}
//		return 0;
//	}
//#if 0

//	//X 轴
//	if (vmc_all.spd_ng.x >= 0) {
//		spd_flag = 1;
//		spd_yaw = 0;
//	}
//	else {
//		spd_flag = -1;
//		spd_yaw = 180;
//	}
//	leg_n = vmc[sel].epos_n;
//	ankle_n.x = vmc_all.H*vmc[sel].flag_fb / 2;
//	ankle_n.y = vmc_all.W*vmc[sel].flag_rl / 2;
//	END_POS nominal_pos;
//	nominal_pos = slip_n;// leg_n;
//	//nominal_pos = leg_n;//采用当前落足位置()判断后续 否则使用关节位置//-------------------------------
//	int grid_ankle[2];
//	int grid_nominal[2];
//	int grid_slip[2];
//	//line_function_from_arrow(leg_n.x, leg_n.y, spd_yaw, &k1, &b1);//速度矢量直线
//	find_grid_on_map_local(ankle_n, &grid_ankle[Xr], &grid_ankle[Yr]);
//	find_grid_on_map_local(nominal_pos, &grid_nominal[Xr], &grid_nominal[Yr]);
//	find_grid_on_map_local(slip_n, &grid_slip[Xr], &grid_slip[Yr]);
//	//找直线上所有满足条件的点
//	int grid_x_on_vector1[MAP_H];
//	int cnt_grid1 = 0;

//	int grid_x_on_vector2[MAP_H];
//	int cnt_grid2 = 0;
//	int grid_x_on_vector22[MAP_H];
//	int cnt_grid22 = 0;
//	int grid_x_on_vector3[MAP_H];
//	int cnt_grid3 = 0;
//	int x_id_closet_to_slip = 99;
//	float x_closet_dis = 99;
//	int good_location_flag[10] = { 0 };
//	int safe_bind = 1;
//	int safe_bind_slip = 1;

//	if (_hu_model.fake_map_guass == 2) {
//		safe_bind = 0;
//	}

//	if (mode == 0)//haogou
//	{
//		for (i = 0; i < MAP_H; i++)
//		{
//			if (global_map.h_grid_local_good[i][grid_nominal[Yr]] >= check_param[1]
//				&& global_map.h_grid_local_good[i][grid_nominal[Yr]] <= check_param[2]
//				)//寻找在速度矢量上的X坐标 满足高度限制
//			{
//				grid_x_on_vector1[cnt_grid1++] = i;
//			}
//		}


//		for (i = 0; i < cnt_grid1; i++)//寻找在速度前方的X坐标
//		{
//			if ((grid_x_on_vector1[i] >= (grid_nominal[Xr] + safe_bind) && spd_flag >= 0) ||
//				(grid_x_on_vector1[i] <= (grid_nominal[Xr] - safe_bind) && spd_flag < 0)
//				) {//寻找在速度方向上的X坐标
//				grid_x_on_vector2[cnt_grid2++] = grid_x_on_vector1[i];//正方向
//			}
//			if ((grid_x_on_vector1[i] <= (grid_nominal[Xr] - safe_bind) && spd_flag >= 0) ||
//				(grid_x_on_vector1[i] >= (grid_nominal[Xr] + safe_bind) && spd_flag < 0)) {
//				grid_x_on_vector22[cnt_grid22++] = grid_x_on_vector1[i];//反方向
//			}
//		}

//		for (i = 0; i < cnt_grid2; i++)//在速度前方点寻找运动空间内的点
//		{
//			END_POS pos_grid;
//			pos_grid.x = grid_x_on_vector2[i] * MAP_SIZE - MAP_H / 2 * MAP_SIZE;
//			if (fabs(pos_grid.x - ankle_n.x) < robot_param[2]) {//寻找在速度方向上的X坐标
//				grid_x_on_vector3[cnt_grid3++] = grid_x_on_vector2[i];//正方向
//			}
//		}

//		x_id_closet_to_slip = 99;
//		x_closet_dis = 99;
//		for (i = 0; i < cnt_grid3; i++)//寻找里SLIP结果最近的点
//		{
//			END_POS pos_grid;
//			pos_grid.x = grid_x_on_vector3[i] * MAP_SIZE - MAP_H / 2 * MAP_SIZE;
//			float dis = fabs(slip_n.x - pos_grid.x);

//			if (dis < x_closet_dis) {//寻找在速度方向上的X坐标
//				x_id_closet_to_slip = grid_x_on_vector3[i];
//				x_closet_dis = dis;
//			}
//		}


//		if (x_id_closet_to_slip != 99) {//找到前方合适落足
//			location->x = x_id_closet_to_slip * MAP_SIZE - MAP_H / 2 * MAP_SIZE;
//			if (sel == 2 || sel == 0)
//				location->x += MAP_SIZE / 2;
//			else
//				location->x -= MAP_SIZE / 2;
//			location->y = grid_nominal[Yr] * MAP_SIZE - MAP_W / 2 * MAP_SIZE;
//			location->z = global_map.h_grid_local_good[x_id_closet_to_slip][grid_nominal[Yr]];
			
//			float zmp_check = vmc_all.H / 2 * cosd(vmc_all.att[PITr]) *0.7;

//			if ((location->x< zmp_check&&location->x>0)
//				|| (location->x >-zmp_check &&location->x < 0)
//				)
//				//ZMP约束
//				good_location_flag[0] = 0;//找到但机体缺少ZMP了
//			else
//				good_location_flag[0] = 1;
//			//printf("Find location-F slip_x=%f visual=%f %f\n", slip_n.x, location->x, location->z);//
//		}


//		//----------------------------------------------------------
//		if (!good_location_flag[0]) {//没再前方找到合适的落足位置
//			//从后方寻找最近点
//			x_id_closet_to_slip = 99;
//			x_closet_dis = 99;
//			for (i = 0; i < cnt_grid22; i++)//寻找里SLIP结果最近的点
//			{
//				END_POS pos_grid;
//				pos_grid.x = grid_x_on_vector22[i] * MAP_SIZE - MAP_H / 2 * MAP_SIZE;
//				float dis = fabs(slip_n.x - pos_grid.x);

//				if (dis < x_closet_dis) {//寻找在速度方向上的X坐标
//					x_id_closet_to_slip = grid_x_on_vector22[i];
//					x_closet_dis = dis;
//				}
//			}

//			if (x_id_closet_to_slip != 99) {//找到后方落足位置
//				good_location_flag[2] = 1;
//				location->x = x_id_closet_to_slip * MAP_SIZE - MAP_H / 2 * MAP_SIZE;
//				if (sel == 2 || sel == 0)
//					location->x += MAP_SIZE / 2;
//				else
//					location->x -= MAP_SIZE / 2;
//				location->y = grid_nominal[Yr] * MAP_SIZE - MAP_W / 2 * MAP_SIZE;
//				location->z = global_map.h_grid_local_good[x_id_closet_to_slip][grid_nominal[Yr]];
//				//printf("Find location-B slip_x=%f visual=%f %f\n", slip_n.x, location->x, location->z);//
//				//return 1;
//			}
//		}

//		//判断如果SLIP点在合适的落足位置上则还是采用SLIP点
//		good_location_flag[1] = 0;

//		for (j = 0; j < safe_bind_slip; j++) {
//			if (global_map.h_grid_local_good[grid_slip[Xr] + j][grid_slip[Yr]] >= check_param[1]
//				&& global_map.h_grid_local_good[grid_slip[Xr] + j][grid_slip[Yr]] <= check_param[2]
//				&& global_map.h_grid_local_good[grid_slip[Xr] - j][grid_slip[Yr]] >= check_param[1]
//				&& global_map.h_grid_local_good[grid_slip[Xr] - j][grid_slip[Yr]] <= check_param[2]
//				)
//				good_location_flag[1] = 1;
//		}

//		//good_location_flag[2] = 1;
//		//落足稳定阈值判断

//		if ((good_location_flag[0] || good_location_flag[2]) && !good_location_flag[1])
//			return 1;
//		else
//			return 0;
//	}
//	else//taojie
//	{



//	}
//#endif
//#endif
//}

//void move_robot_on_map_n(float dt) {//移动全局地图

//	END_POS robot_vel_n;
//	if (global_map.init_odom) {
//		printf("Vision::Odom map reset!\n");
//		global_map.init_odom = 0;//复位里程计
//		global_map.robot_cog.x = global_map.robot_cog.y = 0;
//	}

//	global_map.robot_att[YAWr] = vmc_all.att[YAWr];
//	//逆时针Yaw为+  顺时针为-  左y为-  右为+
//	rotate_vect3_with_yaw(vmc_all.spd_ng, &robot_vel_n, -global_map.robot_att[YAWr]);//计算地图机器人的运动速度
//	global_map.robot_vel = robot_vel_n;
	
//#if POS_USE_SIMU
//	//消除初始化偏差后的机器人位置
//	global_map.robot_cog.x = -webots_draw_rx.cog_real.z - global_map.robot_cog_off_real.x;
//	global_map.robot_cog.y =  webots_draw_rx.cog_real.x - global_map.robot_cog_off_real.y;
//	global_map.robot_cog.z =  webots_draw_rx.cog_real.y - global_map.robot_cog_off_real.z;

//	for (int i = 0; i < 4; i++) {
//		global_map.leg_pos[i].x = -webots_draw_rx.leg_end_real[i].z - global_map.robot_cog_off_real.x;
//		global_map.leg_pos[i].y =  webots_draw_rx.leg_end_real[i].x - global_map.robot_cog_off_real.y;
//		global_map.leg_pos[i].z =  webots_draw_rx.leg_end_real[i].y - global_map.robot_cog_off_real.z;
//	}

//	END_POS Leg_z_smallest;
//	char leg_lowest_id = 0;
//	Leg_z_smallest = global_map.leg_pos[0];//计算当前最低足底的位置
//	for (int i = 0; i < 4; i++) {
//		if (vmc[i].ground)
//		{
//			Leg_z_smallest = global_map.leg_pos[i];
//			leg_lowest_id = i;
//			break;
//		}
//	}

//	for (int i = 0; i < 4; i++) {
//		if (global_map.leg_pos[i].z < Leg_z_smallest.z)
//		{
//			Leg_z_smallest = global_map.leg_pos[i];
//			leg_lowest_id = i;
//		}
//	}

//	int grid_cog_on_map[2];
//	find_grid_on_map_n(global_map.robot_cog, &grid_cog_on_map[Xr], &grid_cog_on_map[Yr]);
//	cog_off_ground_real = global_map.h_grid[grid_cog_on_map[Xr]][grid_cog_on_map[Yr]];//用于落足点计算的高程基准
//	if (cog_off_ground_real < 0)
//		cog_off_ground_real = 0;
	
//	int grid_leg_on_map[2];
//	find_grid_on_map_n(Leg_z_smallest, &grid_leg_on_map[Xr], &grid_leg_on_map[Yr]);
//	webots_draw_n.grid_m_base_z = global_map.h_grid[grid_leg_on_map[Xr]][grid_leg_on_map[Yr]] - cog_off_ground_real * (cog_off_ground_real > 0);
	
//    if (_hu_model.fake_map_guass == 2  ) {//动觉智能高程图
//		cog_off_ground_real = 0;
//	    webots_draw_n.grid_m_base_z = 0;
//	}
//#else
//	global_map.robot_cog.x += global_map.robot_vel.x*dt;//计算全局位置
//	global_map.robot_cog.y += global_map.robot_vel.y*dt;
//#endif
//	/*printf(" %f %f %f %f global_map.robot_att[YAWr]=%f\n", robot_vel_n.x, robot_vel_n.y,
//		global_map.robot_cog.x, global_map.robot_cog.y, global_map.robot_att[YAWr]);*/
//	//以机器人COG为中心  计算每个网格点的坐标
//	END_POS grid_cog_local[MAP_H][MAP_W];

//	for (int i = 0; i < MAP_H; i++)
//	{
//		for (int j = 0; j < MAP_W; j++)
//		{
//			grid_cog_local[i][j].x = i * MAP_SIZE - MAP_H * MAP_SIZE / 2 + MAP_SIZE / 2;
//			grid_cog_local[i][j].y = j * MAP_SIZE - MAP_W * MAP_SIZE / 2 + MAP_SIZE / 2;
//		}
//	}

//	//在全局地图中抠出局部地图
//	END_POS grid_cog_local_on_map[MAP_H][MAP_W];
//	int grid_local_on_map[2];
//	//printf(" global_map.robot_att[YAWr]=%f\n", global_map.robot_att[YAWr]);
//	for (int i = 0; i < MAP_H; i++)
//	{
//		for (int j = 0; j < MAP_W; j++)
//		{
//			//将机器人局部地图转换到全局
//			rotate_vect3_with_yaw(grid_cog_local[i][j], &grid_cog_local_on_map[i][j], -global_map.robot_att[YAWr]);
//			grid_cog_local_on_map[i][j].x += global_map.robot_cog.x;
//			grid_cog_local_on_map[i][j].y += global_map.robot_cog.y;

//			find_grid_on_map_n(grid_cog_local_on_map[i][j], &grid_local_on_map[Xr], &grid_local_on_map[Yr]);
//			if (grid_local_on_map[Xr] >= 0 && grid_local_on_map[Xr] < MAP_H_N&&grid_local_on_map[Yr] >= 0 && grid_local_on_map[Yr] < MAP_W_N) {
//				//获取局部地图在全局地图上的高度
//				global_map.h_grid_local[i][j] = global_map.h_grid[grid_local_on_map[Xr]][grid_local_on_map[Yr]];//全局地图高度
//				global_map.grid_pos_local[i][j] = grid_cog_local_on_map[i][j];//局部地图点在全局地图位置
//			}
//			else {
//				printf("MAP::grid outer global map updating!!\n");
//			}
//		}

//	}
//	//printf("Map update:: cog=%f %f\n", global_map.robot_cog.x, global_map.robot_cog.y);
//#if 0//地图打印
//	for (int i = 0; i < MAP_H / 2; i++)
//	{
//		for (int j = 0; j < MAP_W / 2; j++)
//		{

//			//printf("%.2f ", global_map.h_grid_local[i][j]);
//			printf("%.2f ", global_map.h_grid_local_good[i][j]);
//		}
//		printf("\n");
//	}
//	//printf("\t");
//#endif
//	//printf("ss\n");
//}

//static FILE *fp_map;//读取离线地图 真实机器人
//int fake_map_init(_HEIGHT_MAP* map_fake, float x_off_n, float y_off_n, float rotate_yaw, char map_sel)
//{
//	int i, j;
//	ifstream myfile(".\\Map\\g_map_t_15cm.txt");
//	if (myfile.is_open()) {
//		printf("---------------------------------Guass Map Ready---------------------------------\n");
//	}
//	else {
//		printf("---------------------------------Guass Map Fail！！---------------------------------\n");
//		return 0;
//	}

//	float map_read[GMAP_H][GMAP_W] = { 0 };
//	global_map.robot_att[YAWr] = vmc_all.att[YAWr];

//	for (int i = 0; i < GMAP_H; i++)
//	{
//		for (int j = 0; j < GMAP_W; j++)
//		{
//			if (_hu_model.guass_map_type == 0) {//15cm台阶
//				myfile >> map_read[i][j];
//				guass_map.map[i][j] = map_read[i][j];
//			}
//			guass_map.grid_pos[i][j].x = i * MAP_SIZE - GMAP_H * MAP_SIZE / 2 * 1 - MAP_SIZE / 2;//计算地图网格的坐标
//			guass_map.grid_pos[i][j].y = j * MAP_SIZE - GMAP_W * MAP_SIZE / 2 * 1 - MAP_SIZE / 2;//将W移动到了中间
//		}
//	}

//	printf("Guass Map init at x=%f y=%f yaw=%f yaw_now=%f\n", x_off_n, y_off_n, rotate_yaw, global_map.robot_att[YAWr]);
//	END_POS grid_pos_n[GMAP_H][GMAP_W];
//	int grid_pos[2];
//	global_map.robot_cog.x = 0;//以机器人为中心
//	global_map.robot_cog.y = 0;
//	global_map.robot_cog.z = 0;

//	global_map.robot_cog_off_real.x = -webots_draw_rx.cog_real.z;//记录webots里位置作为偏差
//	global_map.robot_cog_off_real.y = webots_draw_rx.cog_real.x;
//	global_map.robot_cog_off_real.z = webots_draw_rx.cog_real.y;
//	for (i = 0; i < GMAP_H; i++)
//	{
//		for (j = 0; j < GMAP_W; j++)
//		{
//			rotate_vect3_with_yaw(guass_map.grid_pos[i][j], &grid_pos_n[i][j], rotate_yaw + global_map.robot_att[YAWr]);//选择补偿地图偏差
//			grid_pos_n[i][j].x += x_off_n + global_map.robot_cog.x;
//			grid_pos_n[i][j].y += y_off_n + global_map.robot_cog.y;
//			find_grid_on_map_n(grid_pos_n[i][j], &grid_pos[Xr], &grid_pos[Yr]);
//			if (grid_pos[Xr] > 0 && grid_pos[Xr] < MAP_H_N&&grid_pos[Yr]>0 && grid_pos[Yr] < MAP_W_N) {
//				//将人工地图高度赋值在全局地图上
//				global_map.h_grid[grid_pos[Xr]][grid_pos[Yr]] = guass_map.map[i][j];//全局地图高度
//				global_map.h_grid_pos[i][j] = grid_pos_n[i][j];//全局地图位置
//			}
//			else {
//				printf("Guass MAP::grid outer global map init!!\n");
//			}

//		}
//	}

//	return 1;
//}

//void toEulerAngle(const double x, const double y, const double z, const double w, double& roll, double& pitch, double& yaw)
//{
//	// roll (x-axis rotation)
//	double sinr_cosp = +2.0 * (w * x + y * z);
//	double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
//	roll = atan2(sinr_cosp, cosr_cosp);

//	// pitch (y-axis rotation)
//	double sinp = +2.0 * (w * y - z * x);
//	if (fabs(sinp) >= 1)
//		pitch = copysign(3.1415926 / 2, sinp); // use 90 degrees if out of range
//	else
//		pitch = asin(sinp);

//	// yaw (z-axis rotation)
//	double siny_cosp = +2.0 * (w * z + x * y);
//	double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
//	yaw = atan2(siny_cosp, cosy_cosp);
//	//    return yaw;
//}

//#include <vector>;
//int cube_map_init(void)//梅花桩地图
//{
//	ofstream ofs;
//	ofs.open(".\\Map\\cube_cog.txt", ios::out);
//	ofstream ofs1;
//	ofs1.open(".\\Map\\way_point.txt", ios::out);
//	int i, j;
//	printf("---------------------------------Cube Map  Ready---------------------------------\n");
//#if 1
//	for (int i = 0; i < MAP_H_N; i++)//初始化全局地图
//	{
//		for (int j = 0; j < MAP_W_N; j++)
//		{
//			global_map.h_grid[i][j] = -0.1;
//		}
//	}
//#endif

//	END_POS cog_cube[CUBE_NUM];
//	for (int i = 0; i < CUBE_NUM; i++)//获取梅花桩中心
//	{
//		cog_cube[i].x = -webots_draw_rx.cube_cog[i].z;
//		cog_cube[i].y =  webots_draw_rx.cube_cog[i].x;
//		cog_cube[i].z =  webots_draw_rx.cube_cog[i].y;
//	}

//	//向全局地图同步
//	if (1) {
//		global_map.robot_cog.x = 0;//以机器人为中心
//		global_map.robot_cog.y = 0;
//		global_map.robot_cog.z = 0;

//		global_map.robot_cog_off_real.x = -webots_draw_rx.cog_real.z;//记录webots里位置作为偏差
//		global_map.robot_cog_off_real.y =  webots_draw_rx.cog_real.x;
//		global_map.robot_cog_off_real.z =  webots_draw_rx.cog_real.y;

//		END_POS fake_map_n_off;//地图相对机器人的偏差
//		fake_map_n_off.x = global_map.robot_cog_off_real.x;
//		fake_map_n_off.y = global_map.robot_cog_off_real.y;
//		fake_map_n_off.z = global_map.robot_cog_off_real.z;

//		//路点
//		for (int i = 0; i < WAY_POINT_NUM; i++) {
//			global_map.way_point[i].x = -webots_draw_rx.way_point_pos[i].z - fake_map_n_off.x;//计算地图中心在全局地图中的位置
//			global_map.way_point[i].y =  webots_draw_rx.way_point_pos[i].x - fake_map_n_off.y;
//			global_map.way_point[i].z =  webots_draw_rx.way_point_pos[i].y - webots_draw_rx.way_point_pos[0].y;
//			printf("way_point[%d] x=%f y=%f z=%f\n", i, global_map.way_point[i].x, global_map.way_point[i].y, global_map.way_point[i].z);
//			ofs1 << i << " " << global_map.way_point[i].x << " " << global_map.way_point[i].y << endl;  //endl用于换行

//		}

//		for (int sel = 0; sel < CUBE_NUM; sel++)
//		{
//			END_POS fake_map_n;
//			fake_map_n.x = cog_cube[sel].x - fake_map_n_off.x;//计算地图中心在全局地图中的位置
//			fake_map_n.y = cog_cube[sel].y - fake_map_n_off.y;
//			//地图高度以第一个地图为偏差
//			fake_map_n.z = cog_cube[sel].z - cog_cube[0].z;
//			printf("cube[%d] x=%f y=%f z=%f\n", sel, fake_map_n.x, fake_map_n.y, fake_map_n.z);
//			ofs << sel << " " << fake_map_n.x << " " << fake_map_n.y << endl;  //endl用于换行
//			END_POS grid_pos_n[FMAP_H][FMAP_W];
//			int h_len = (int)(CUBE_W / MAP_SIZE);
//			int w_len = (int)(CUBE_W / MAP_SIZE);
			
//			//以质心生成梅花桩
//			for (i = 0; i < h_len; i++)
//			{
//				for (j = 0; j < w_len; j++)
//				{
//					if (sqrt((i - h_len / 2)*(i - h_len / 2) + (j - w_len / 2)*(j - w_len / 2)) <= w_len / 2) {
//						grid_pos_n[i][j].x = i * MAP_SIZE - h_len / 2 * MAP_SIZE;
//						grid_pos_n[i][j].y = j * MAP_SIZE - w_len / 2 * MAP_SIZE;
//						grid_pos_n[i][j].z = fake_map_n.z;
//					}
//				}
//			}

//			double roll = 0, pitch = 0, yaw = 0;//转换地图朝向角度
//			toEulerAngle(
//				webots_draw_rx.cube_ori[sel][0],
//				webots_draw_rx.cube_ori[sel][1],
//				webots_draw_rx.cube_ori[sel][2],
//				webots_draw_rx.cube_ori[sel][3],
//				yaw, roll, pitch);

//			float rotate_yaw = -To_180_degreesw(180 + yaw * 57.3);//地图朝向与x轴相反+180

//			int grid_pos[2];
//			for (i = 0; i < h_len; i++)
//			{
//				for (j = 0; j < w_len; j++)
//				{
//					END_POS grid_pos_center;//地图读取时已经移动到中心，Webot地图的坐标系原点在右下角 需要继续移动
//					rotate_vect3_with_yaw(grid_pos_n[i][j], &grid_pos_center, rotate_yaw);

//					grid_pos_center.x += fake_map_n.x;
//					grid_pos_center.y += fake_map_n.y;

//					//找到全局地图中的栅格位置
//					find_grid_on_map_n(grid_pos_center, &grid_pos[Xr], &grid_pos[Yr]);

//					if (grid_pos[Xr] >= 0 && grid_pos[Xr] <= MAP_H_N && grid_pos[Yr] >= 0 && grid_pos[Yr] <= MAP_W_N) {
//						//将人工地图高度赋值在全局地图上
//						global_map.h_grid[grid_pos[Xr]][grid_pos[Yr]] = grid_pos_n[i][j].z;//全局地图高度

//						global_map.h_grid_pos[i][j] = grid_pos_center;// grid_pos_n[i][j];//全局地图位置 XY
//					}
//					else {
//						printf("Cube MAP::grid outer global map init!!\n");
//					}
//				}
//			}
//		}
//	}
//#if 1
//	//读操作
//	ifstream istr;
//	string temp;
//	vector<string> veci;

//	istr.open(".\\Map\\cube_cog.txt");
//	//以换行符作为结束
//	while (istr >> temp)
//	{
//		veci.push_back(temp);
//	}

//	for (auto ite = veci.begin(); ite < veci.end(); ++ite)
//	{
//		float temp= std::stof(*ite);
 
//		cout << temp << endl;
//	}
//	istr.close();
//#endif



//	return 1;

//}

//static FILE *fp_map_link[10];//读取连续离线地图
//int fake_map_link_init(void)
//{
//	static int init_map_read = 0;
//	int i, j;
//	ifstream myfile0(".\\Map\\fake_map0.txt");
//	ifstream myfile1(".\\Map\\fake_map1.txt");
//	ifstream myfile2(".\\Map\\fake_map2.txt");

//	if (myfile0.is_open() && !init_map_read) {
//		init_map_read = 1;
//		printf("---------------------------------Fake Map Link Ready---------------------------------\n");
//	}
//	else
//		return 0;
//#if 0
//	for (int i = 0; i < MAP_H_N; i++)//初始化全局地图
//	{
//		for (int j = 0; j < MAP_W_N; j++)
//		{
//			global_map.h_grid[i][j] = 0;
//		}
//	}
//#endif
//	int map_id = 0;//计算各地图网格中心的坐标
//	float map_read[10][FMAP_H][FMAP_W] = { 0 };
//	for (int map_sel = 0; map_sel < FMAP_NUM; map_sel++)
//	{
//		for (int i = 0; i < FMAP_H; i++)
//		{
//			for (int j = 0; j < FMAP_W; j++)
//			{
//				myfile0 >> map_read[0][i][j];
//				myfile1 >> map_read[1][i][j];
//				myfile2 >> map_read[2][i][j];

//				fake_map[map_sel].map[i][j] = map_read[map_sel][i][j];

//				fake_map[map_sel].grid_pos[i][j].x = i * MAP_SIZE - FMAP_H * MAP_SIZE / 2 * 1 - MAP_SIZE / 2;//计算地图网格的坐标
//				fake_map[map_sel].grid_pos[i][j].y = j * MAP_SIZE - FMAP_W * MAP_SIZE / 2 * 1 - MAP_SIZE / 2;//将W移动到了中间
//			}
//		}
//	}

//	//向全局地图同步
//	static int init_mems = 0;
//	//global_map.stand_map_z_off = webots_draw_rx.cog_real.y - webots_draw_rx.fake_map_cog[0].y;//以0地图为基准定义地图z轴

//	if (webots_draw_rx.fake_map_cog[0].x != 0 && webots_draw_rx.cog_real.z != 0 && !init_mems) {
//		init_mems = 1;
//		global_map.robot_cog.x = 0;//以机器人为中心
//		global_map.robot_cog.y = 0;
//		global_map.robot_cog.z = 0;

//		global_map.robot_cog_off_real.x = -webots_draw_rx.cog_real.z;//记录webots里位置作为偏差
//		global_map.robot_cog_off_real.y =  webots_draw_rx.cog_real.x;
//		global_map.robot_cog_off_real.z =  webots_draw_rx.cog_real.y;

//		END_POS fake_map_n_off;//地图相对机器人的偏差
//		fake_map_n_off.x = global_map.robot_cog_off_real.x;// +FMAP_H * MAP_SIZE / 2 * 1;
//		fake_map_n_off.y = global_map.robot_cog_off_real.y;// -FMAP_W * MAP_SIZE / 2 * 1;
//		fake_map_n_off.z = global_map.robot_cog_off_real.z;
//#if HU_TASK==HU_TASK4
//		//梅花桩
//		cube_map_init();
//#endif
//		//地形标志
//		for (int i = 0; i < 12; i++) {
//			global_map.flag_n[i].x =  (-webots_draw_rx.flag_pos[i].z) - fake_map_n_off.x;//计算地图中心在全局地图中的位置
//			global_map.flag_n[i].y =   (webots_draw_rx.flag_pos[i].x) - fake_map_n_off.y;

//			//printf("flag[%d] x=%f y=%f\n", i, global_map.flag_n[i].x, global_map.flag_n[i].y);
//		}

//		for (int map_sel = 0; map_sel < FMAP_NUM; map_sel++)
//		{
//			END_POS fake_map_n;
//			fake_map_n.x = -webots_draw_rx.fake_map_cog[map_sel].z - fake_map_n_off.x;//计算地图中心在全局地图中的位置
//			fake_map_n.y =  webots_draw_rx.fake_map_cog[map_sel].x - fake_map_n_off.y;
//			//地图高度以第一个地图为偏差
//			fake_map_n.z = webots_draw_rx.fake_map_cog[map_sel].y - webots_draw_rx.fake_map_cog[0].y;

//			double roll=0, pitch=0, yaw=0;//转换地图朝向角度
//			toEulerAngle(
//				webots_draw_rx.fake_map_ori[map_sel][0],
//				webots_draw_rx.fake_map_ori[map_sel][1],
//				webots_draw_rx.fake_map_ori[map_sel][2],
//				webots_draw_rx.fake_map_ori[map_sel][3],
//				yaw, roll, pitch);
	
//			float yaw_robot = -To_180_degreesw(180 + vmc_all.att[YAWr]);
//			float rotate_yaw = -To_180_degreesw(180 + yaw*57.3);//地图朝向与x轴相反+180

//			printf("sel=%d %f %f fake_map_n.z=%f map_yaw=%f yaw_robot=%f robot_yaw=%f\n", map_sel, fake_map_n.x, fake_map_n.y, fake_map_n.z,
//			webots_draw_rx.fake_map_ori[map_sel][3] * 57.3, yaw_robot, rotate_yaw);
//			END_POS grid_pos_n[FMAP_H][FMAP_W];
//			int grid_pos[2];
//			for (i = 0; i < FMAP_H; i++)
//			{
//				for (j = 0; j < FMAP_W; j++)
//				{
//					//将人工地图进行旋转
//					END_POS grid_pos_center;//地图读取时已经移动到中心，Webot地图的坐标系原点在右下角 需要继续移动
//					grid_pos_center.x = fake_map[map_sel].grid_pos[i][j].x +FMAP_H * MAP_SIZE / 2 ;
//					grid_pos_center.y = fake_map[map_sel].grid_pos[i][j].y -FMAP_W * MAP_SIZE / 2 ;
//					grid_pos_center.z = fake_map[map_sel].grid_pos[i][j].z;
//					rotate_vect3_with_yaw(grid_pos_center, &grid_pos_n[i][j], rotate_yaw);
//					//移动地图坐标系使用Webot下的偏差
//					grid_pos_n[i][j].x += fake_map_n.x;
//					grid_pos_n[i][j].y += fake_map_n.y;
//					//找到全局地图中的栅格位置
//					find_grid_on_map_n(grid_pos_n[i][j], &grid_pos[Xr], &grid_pos[Yr]);

//					if (grid_pos[Xr] >= 0 && grid_pos[Xr] <= MAP_H_N && grid_pos[Yr] >= 0 && grid_pos[Yr] <= MAP_W_N) {
//						//将人工地图高度赋值在全局地图上
//						global_map.h_grid[grid_pos[Xr]][grid_pos[Yr]] = fake_map[map_sel].map[i][j];//全局地图高度
//						global_map.h_grid[grid_pos[Xr]][grid_pos[Yr]] += fake_map_n.z;//补偿地图基准偏差

//						global_map.h_grid_pos[i][j] = grid_pos_n[i][j];//全局地图位置 XY
//					}
//					else {
//						printf("Link MAP::grid outer global map init!!\n");
//					}
//				}
//			}
//		}
//	}
//	return 1;
//}

//void convert_n_t_Webots_n(END_POS point, END_POS* point_n_webots)//主要转换足端轨迹和落足点
//{
//	END_POS point_n, point_c;
//	float yaw = -To_180_degreesw(180 + vmc_all.att[YAWr] - 90*0);
//	point_n.x = point.x * cosd(yaw) - point.y * sind(yaw);
//	point_n.y = point.x * sind(yaw) + point.y * cosd(yaw);
//	point_n.z = point.z;

//	point_c.x = -point_n.y;//转换到Webots坐标系
//	point_c.y = point_n.z;//高度-
//	point_c.z = point_n.x; //前

//	point_n_webots->x = point_c.x;
//	point_n_webots->y = point_c.y;
//	point_n_webots->z = point_c.z;
//}

//void convert_n_t_Webots_n_now_yaw(END_POS point, END_POS* point_n_webots)//主要转换足端轨迹和落足点
//{
//	point_n_webots->x = -point.y;
//	point_n_webots->y = point.z;
//	point_n_webots->z = point.x;
//}
////webots下绘制函数
////#include <windows.h>
////#include<stdio.h>
////#include<stdlib.h>
////#define  SIZE_MEM 4096*2000
////LPVOID lpdata = NULL;//指针标识首地址
////HANDLE hmap;
////void win_mem_tx()//写入地图落足等到显示线程
////{
////	static int init = 0;
////	if (lpdata != NULL && !init)
////	{
////		puts("mem exist\n");
////	}

////	if (!init) {
////		init = 1;
////		hmap = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL,
////			PAGE_READWRITE | SEC_COMMIT, 0, SIZE_MEM, "mem_control_t_draw");
////	}

////	if (hmap == NULL)
////	{
////		puts("creat fail\n");
////	}
////	else
////	{
////		//映射文件到指针
////		lpdata = MapViewOfFile(hmap, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, 0);

////		memcpy(lpdata, (struct WEBTOS_DRAWING*)&webots_draw_n, sizeof(webots_draw_n));
////	}
////}

////HANDLE hmapfile_rx;
////int win_mem_rx()//从仿真器读取真实数据
////{
////	static int init = 0;
////	if (!init) {
////		init = 1;
////		hmapfile_rx = OpenFileMappingA(FILE_MAP_READ, FALSE, "mem_control_r_draw");
////	}
////	if (hmapfile_rx == NULL)
////	{
////		init = 0;
////		//printf("robot::open fail\n");
////		return 0;
////	}
////	//创建指针，指向这片内存
////	LPVOID lpbase = MapViewOfFile(hmapfile_rx, FILE_MAP_READ, 0, 0, 0);
////	if (lpbase == NULL)
////	{
////		printf("robot::meme open fail\n");
////	}

////	float  *p = (float *)lpbase;
////	struct WEBTOS_DRAWING_RX* temp = (struct WEBTOS_DRAWING_RX *)lpbase;
////	memcpy((struct WEBTOS_DRAWING_RX*)&webots_draw_rx, temp, sizeof(webots_draw_rx));
////	//printf("%f %f %f\n", webots_draw_rx.fake_map_cog.x, webots_draw_rx.fake_map_cog.y, webots_draw_rx.fake_map_cog.z);
////	//printf("%f %f %f\n", webots_draw_rx.cog_real.x, webots_draw_rx.cog_real.y, webots_draw_rx.cog_real.z);
////	return 1;
////}


////void share_memory_drawing(void)
////{
////	win_mem_tx();
////	win_mem_rx();
////	END_POS pos_n_webots_temp;
////	for (int i = 0; i < 4; i++) {
////		convert_n_t_Webots_n(webots_draw.leg_sw_tar[i], &webots_draw_n.leg_sw_tar[i]);
////		convert_n_t_Webots_n(webots_draw.leg_sw_slip[i], &webots_draw_n.leg_sw_slip[i]);
////		convert_n_t_Webots_n(webots_draw.tar_grf[i], &webots_draw_n.tar_grf[i]);
////		convert_n_t_Webots_n(webots_draw.mu_limit[i], &webots_draw_n.mu_limit[i]);
////#if 1
////		convert_n_t_Webots_n(webots_draw.zmp_pos, &webots_draw_n.zmp_pos);
////		convert_n_t_Webots_n(webots_draw.tar_zmp_pos, &webots_draw_n.tar_zmp_pos);
////#else
////		convert_n_t_Webots_n_now_yaw(webots_draw.zmp_pos, &webots_draw_n.zmp_pos);
////		convert_n_t_Webots_n_now_yaw(webots_draw.tar_zmp_pos, &webots_draw_n.tar_zmp_pos);
////#endif
////		webots_draw_n.ground[i] = webots_draw.ground[i];
////		webots_draw_n.touch[i] = webots_draw.touch[i];
////	}

////	convert_n_t_Webots_n(vmc_all.pos_n, &pos_n_webots_temp);
////	convert_n_t_Webots_n(vmc_all.pos_n, &webots_draw_n.cog_pos);

////	for (int i = 0; i < 4; i++) {
////		for (int j = 0; j < 30; j++) {
////			END_POS zero;
////			zero.x = zero.y = zero.z = 0;
////			convert_n_t_Webots_n(webots_draw.sw_tar_traj[i][j], &pos_n_webots_temp);

////			if (vmc[i].ground) {

////				convert_n_t_Webots_n(zero, &webots_draw_n.sw_tar_traj[i][j]);
////			}
////			else {

////				convert_n_t_Webots_n(webots_draw.sw_tar_traj[i][j], &webots_draw_n.sw_tar_traj[i][j]);
////			}
////		}

////	}
////	for (int sel = 0; sel < FMAP_NUM; sel++) {
////		for (int i = 0; i < FMAP_H; i++) {
////			for (int j = 0; j < FMAP_W; j++) {
////				webots_draw_n.fgrid_m[sel][i][j] = fake_map[sel].map[i][j];
////			}
////		}
////	}
////	//地图测试
////#if 0
////	static int pcnt[2] = { 0 };
////	static int cnt_time = 0;
////	if (cnt_time++ > 2) {
////		cnt_time = 0;
////		pcnt[0]++;
////		if (pcnt[0] > 40) {
////			pcnt[0] = 0;
////			pcnt[1]++;
////			for (int i = 0; i < 40; i++)
////				webots_draw_n.grid_m[pcnt[1]][i] = 0.1;
////		}
////	}
////#else
////	for (int i = 0; i < MAP_H; i++) {
////		for (int j = 0; j < MAP_W; j++) {
////			webots_draw_n.grid_m[i][j] = global_map.h_grid_local[i][j];
////			webots_draw_n.grid_m_good[i][j] = global_map.h_grid_local_good[i][j];
////			webots_draw_n.grid_m_edge[i][j] = global_map.h_grid_local_edge[i][j];
////		}
////	}
////#endif
////	webots_draw_n.cog_att[2] = To_180_degreesw(180 + vmc_all.att[YAWr]);
////	webots_draw_n.en_draw_map = EN_H_MAP_UPDATE;


////	//--human
////	webots_draw_n.grid_wide = 0.04;
////}
