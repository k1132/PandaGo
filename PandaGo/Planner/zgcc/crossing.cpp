#include "./trace_road.h"
#include "./crossing.h"
#include "./Cmorphin.h"
#include <math.h>
#include <string.h>
using namespace std;

int g_cross_road_type = 0;

int g_cross_und_trace_cur_lane_counter;
COOR2 g_cross_smooth_start_pt;
COOR2 g_cross_smooth_end_pt;
int g_finish_cross = 0;//[用来表示刚结束路口，减速缓行]
int g_cross_flag = 0;						//[初始化标志]
int g_cross_exit_type = 0;					//[路口出口点类型，0  有出口点  1  有估算出口点  2  没有出口点]
int g_cross_finish_signal = 0;				//[根据融合道路稳定，给出结束路口状态]
int g_subgoal_adjust_flag = 0;				//[当子目标点是最后一个时，进行修正]
int g_cross_start_yaw = 0;					//[初始航向角]
int g_cross_cur_yaw = 0;					//[当前航向角]
int g_cross_goal_yaw = 0;
int g_cross_turn_yaw = 0;					//[需要旋转的航向角]
int g_cross_reach_goal_yaw = 0;				//[航向角满足约束的标志]
int g_cross_left_yaw_t = 0;
int g_cross_right_yaw_t = 0;
COOR2 g_cross_subgoal_pts[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];		//[交叉口子目标点]
int g_cross_subgoal_pts_num = 0;				//[子目标点数目]
int g_cross_cur_subgoal = 0;					//[当前子目标点索引]
int g_cross_reach_goal = 0;					//[到达最终目标点标志]
int g_cross_travel_dist = 0;					//[前方可通行距离，用来控制速度（有没有效目前没测试过）]
int g_cross_quit_dist = 0;
COOR2 g_cross_quit_start_pt;
int g_cross_quit_dist_sum = 0;
//double g_cross_travel_rate = 0;					//[前方可通行比率，用来控制速度（有没有效目前没测试过）]

//[路口执行时，切换的角度阈值]
int g_straight_alpha = 0;
int g_left_turn_alpha = 0;
int g_right_turn_alpha = 0;
int g_left_turn_pt_adjust = 0;
int g_left_turn_pt2x_adjust = 0;
int g_left_turn_pt2y_adjust = 0;
int g_right_turn_pt_adjust = 0;
int g_right_turn_pt2x_adjust = 0;
int g_right_turn_pt2y_adjust = 0;
int g_uturn_alpha1 = 0;
int g_uturn_alpha2 = 0;
int g_uturn_alpha3 = 0;
int g_uturn_alpha4 = 0;


//[通过路口速度]
int g_cross_straight_speed;
int g_cross_left_turn_speed;
int g_cross_right_turn_speed;
int g_cross_uturn_speed;
int g_uturn_sub_pt_adjust;

COOR2 g_cross_slow_pt;
int g_cross_und_slow_down_dist = 0;  //[路口理解第二次降速距离阈值]
int g_cross_und_slow_down_speed = 0;  //[路口理解第二次降速]
int g_cross_und_slow_down_flag = 0;  //[路口理解第二次降速标志]
int g_cross_adjust_count = 0;
int g_cross_adjust_flag = 0;
int g_uturn_stop_once = 0;
int g_uturn_stop_once_counter = 0;

COOR2 g_last_frame_pt;//[补丁参数，结构化道路由于分割不好，导致两车道合并，有时规划偏右，下一帧左换道，看上去跳变了]

//***********************************************************************************************
//                                zgccmax 2012.May.29
//void reset_cross_flag()
//param:
//return:
//discribe: 清除路口状态变量
//***********************************************************************************************
void reset_cross_flag()
{
	g_cross_reach_goal = 0;
	g_cross_reach_goal_yaw = 0;
	g_cross_flag = 0;
	g_subgoal_adjust_flag = 0;
	memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));
}

/*==================================================================
 * 函数名  ：	int cross_left_turn(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data)
 * 功能    ：	执行路口左拐弯
 * 输入参数：	PL_FUNC_INPUT *pl_input			环境数据输入
				PL_CS_DATA *pl_cs_data			规划输出
				PL_LOCAL_DATA *pl_local_data	规划触发事件
 * 输出参数：	
 * 返回值  ：	int  0   有可执行路径  -1   无可执行路径
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int cross_left_turn(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data)
{
	int ret = 0;

	int i;
	COOR2 start_pt;
	COOR2 goal_pt;
	POSE_INFO gps_info;

	gps_info = pl_input->state.pos;

	if (g_cross_flag == 0)
	{
		start_pt = pl_input->state.pos.com_coord;
		goal_pt = pl_input->fu_pl_data.cross.exit_point;

		if (pl_input->fu_pl_data.cross.valid_num < 2)
		{
			if (goal_pt.x == 0 && goal_pt.y == 0)
			{//[无出口点]
				g_cross_exit_type = 2;
			}
			else
			{//[仅使用出口点]
				g_cross_exit_type = 1;
			}
		}
		else
		{
			//[有采集路口引导点]
			g_cross_exit_type = 0;
		}

#ifdef MBUG_OPEN_
		MBUG("g_cross_exit_type : %d\n", g_cross_exit_type);
#endif
		COOR2 pts1[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
		int pts1_num;
		COOR2 pts2[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
		int pts2_num;

		if (g_cross_exit_type != 2)
		{
			//[交叉口初始]
			memset(pts1, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			memset(pts2, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			pts1_num = 0;
			pts2_num = 0;

			diff_coor2_e2v(&gps_info, &start_pt, &pts1[0]);
			diff_coor2_e2v(&gps_info, &goal_pt, &pts1[5]);
			if (g_cross_exit_type == 1)
			{
				pts1[1].x = pts1[0].x;
				pts1[1].y = (INT32)(pts1[5].y * 2.0 / 3);
				pts1[2].x = pts1[0].x;
				pts1[2].y = pts1[5].y;
				pts1[3].x = (INT32)((pts1[5].x - pts1[2].x) * 1.0 / 3 + pts1[2].x);
				pts1[3].y = (INT32)((pts1[5].y - pts1[2].y) * 1.0 / 3 + pts1[2].y);
				pts1[4].x = (INT32)((pts1[5].x - pts1[2].x) * 2.0 / 3 + pts1[2].x);
				pts1[4].y = (INT32)((pts1[5].y - pts1[2].y) * 2.0 / 3 + pts1[2].y);
				//pts1[5] = goal_pt;
			}
			else if (g_cross_exit_type == 0)
			{
				diff_coor2_e2v(&gps_info, &pl_input->fu_pl_data.cross.middle_point[0], &pts1[2]);
				diff_coor2_e2v(&gps_info, &pl_input->fu_pl_data.cross.middle_point[1], &pts1[4]);
				//diff_coor2_e2v(&gps_info, &goal_pt, &pts1[5]);
				pts1[1].x = (INT32)((pts1[2].x - pts1[0].x) * 2.0 / 3 + pts1[0].x);
				pts1[1].y = (INT32)((pts1[2].y - pts1[0].y) * 2.0 / 3 + pts1[0].y);
				pts1[3].x = (INT32)((pts1[4].x - pts1[2].x) * 1.0 / 2 + pts1[2].x);
				pts1[3].y = (INT32)((pts1[4].y - pts1[2].y) * 1.0 / 2 + pts1[2].y);
			}

			pts1_num = 6;

			//[由控制点生成贝塞尔曲线]
			pts2_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			get_bezier_line(pts1, pts1_num, pts2, pts2_num);


#ifdef MBUG_OPEN_
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
				MBUG("pts2: (%d, %d)\n", pts2[i].x, pts2[i].y);
#endif


			//[转换到惯导系统GPS的坐标系下]
			for (i = 0; i < pts2_num; i++)
				diff_coor2_v2e(&gps_info, &pts2[i], &g_cross_subgoal_pts[i]);

			g_cross_subgoal_pts_num = pts2_num;
		}

		g_cross_cur_subgoal = 0;
		g_cross_reach_goal = 0;
		g_cross_start_yaw = g_yaw;


		CONVERT_YAW(g_cross_start_yaw);
		g_cross_reach_goal_yaw = 0;

		if (g_cross_exit_type == 0)
		{//[有采集过路口，那么计算所需转向角]
			COOR2 vec_goal;//[计算初始航向角和出口航向角之间的角度]
			vec_goal.x = pts1[pts1_num - 1].x - pts1[pts1_num - 2].x;
			vec_goal.y = pts1[pts1_num - 1].y - pts1[pts1_num - 2].y;
			g_cross_goal_yaw = (INT32)(atan2(vec_goal.y, vec_goal.x) * 180 / PI - 90);

			g_cross_turn_yaw = g_cross_goal_yaw - g_cross_start_yaw;
			CHECK_YAW(g_cross_turn_yaw);
		}
		else
		{
			g_cross_turn_yaw = 90;//[左转90]
		}

#ifdef MBUG_OPEN_
		MBUG("g_cross_turn_yaw: %d\n", g_cross_turn_yaw);
#endif


		for (i = 0; i < CROSS_AVG_NUM; i++)
			g_cross_avg_angle[i] = 90.0;

		g_cross_avg_angle_num = 0;
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_cross_travel_rate = 0;
		g_cross_avg_index = 0;
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_subgoal_adjust_flag = 0;
		g_cross_finish_signal = 0;
		g_cross_flag = 1;

		g_cross_quit_dist_sum = 0;
		memset(&g_cross_quit_start_pt, 0, sizeof(COOR2));

		g_finish_cross = 0;
		memset(&g_cross_smooth_start_pt, 0, sizeof(COOR2));
		memset(&g_cross_smooth_end_pt, 0, sizeof(COOR2));
	}


	//	if (g_cross_reach_goal == 1 && g_cross_exit_type != 2)
	if (g_cross_cur_subgoal > NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1 && g_cross_exit_type != 2)
	{//[点已经跟随完毕，检测退出条件]
		g_cross_cur_yaw = g_yaw;
		CONVERT_YAW(g_cross_cur_yaw);

#ifdef MBUG_OPEN_
		MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif

		//[计算航向角偏移]
		double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
		CHECK_YAW(det_yaw);

		if (fabs(det_yaw - g_cross_turn_yaw) < g_left_turn_alpha)
		{
			if (g_multi_lane.ok_times[1] == 2 && g_multi_lane.ok_times[2] == 2)
			{//[道路稳定了]
				g_cross_finish_signal = 1;
			}
		}
	}
	else if (g_cross_exit_type == 2)
	{//[无目标点，就凭角度]
		g_cross_cur_yaw = g_yaw;
		CONVERT_YAW(g_cross_cur_yaw);

#ifdef MBUG_OPEN_
		MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif

		//[计算航向角偏移]
		double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
		CHECK_YAW(det_yaw);

		if (fabs(det_yaw - g_cross_turn_yaw) < g_left_turn_alpha)
		{
			g_cross_finish_signal = 1;
		}
	}

	if (g_cross_finish_signal == 1)
	{
#ifdef MBUG_OPEN_
		MBUG("g_cross_finish_signal : %d\n", g_cross_finish_signal);
#endif
		g_cross_reach_goal = 0;
		g_cross_reach_goal_yaw = 0;
		g_cross_flag = 0;
		g_subgoal_adjust_flag = 0;
		g_cross_finish_signal = 0;
		g_cross_exit_type = 0;
		//[@@发送事件，切换全局状态！！！]
		//ret = -1;
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			g_mid_line[i].x = 0;
			g_mid_line[i].y = i * 100;
		}
		g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
		//g_fidelity = 1;
		ret = 1;
		pl_local_data->event = (EVENT)E_CROSS_OVER;
		memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));

#ifdef MBUG_OPEN_
		MBUG("E_CROSS_OVER has send!\n");
#endif

		g_finish_cross = 10;
		g_cross_smooth_start_pt.x = pl_input->state.pos.ins_coord.x;
		g_cross_smooth_start_pt.y = pl_input->state.pos.ins_coord.y;
		return ret;
	}

	if (g_cross_exit_type == 2)
	{//[无出口点，向偏左一点方向漫游]
		COOR2 pt;

		g_cross_cur_yaw = g_yaw;
		CONVERT_YAW(g_cross_cur_yaw);

#ifdef MBUG_OPEN_
		MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif

		//[计算航向角偏移]
		double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
		CHECK_YAW(det_yaw);

		if (g_cross_reach_goal_yaw == 0)
		{
			if (fabs(det_yaw - g_cross_turn_yaw) < g_left_turn_alpha)
			{
				g_cross_reach_goal_yaw = 1;
			}
		}
		
		if (g_cross_reach_goal_yaw == 0)
		{//[没有出点]
			if (g_road_type == 0)
			{
				if (fabs(det_yaw - (90.0)) > 70.0)
				{
					pt.x = -80;
					pt.y = 800;
				}
				else if (fabs(det_yaw - (90.0)) > 50.0)
				{
					pt.x = -150;
					pt.y = 800;

				}
				else if (fabs(det_yaw - (90.0)) > 40.0)
				{
					pt.x = -250;
					pt.y = 800;
				}
				else
				{
					pt.x = -50;
					pt.y = 800;
				}
			}
			else
			{
				pt.x = -720;
				pt.y = 0;
			}
		}
		else
		{
			double adjust_yaw = 0;
			g_cross_cur_yaw = g_yaw;
			CONVERT_YAW(g_cross_cur_yaw);

#ifdef MBUG_OPEN_
			MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif

			//[计算航向角偏移]
			det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);

			//[设置一个虚拟目标点]
			adjust_yaw = det_yaw - g_cross_turn_yaw;
			CHECK_YAW(adjust_yaw);

#ifdef MBUG_OPEN_
			MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif

			adjust_yaw = adjust_yaw * PI / 180;
			double theta = get_road_direction(pl_input, 3, adjust_yaw);
			theta = theta * PI / 180;
			adjust_yaw = 0.3 * adjust_yaw + 0.7 * theta;

			pt.x = (INT32)(sin(adjust_yaw) * 2000);
			pt.y = (INT32)(cos(adjust_yaw) * 2000);
		}
		

		double out_rate = 0;
		ret = get_best_morphin_line6(pt, out_rate);
		if (ret >= 0)
			g_cross_travel_rate = out_rate;
		else
			g_cross_travel_rate = 0;

		if (ret == -1)
		{
#ifdef MBUG_OPEN_
			MBUG("morphin no way to go\n");
#endif
		}
		else
		{
			if (g_cross_avg_angle_num != CROSS_AVG_NUM)
			{
				double angle = g_morphin_angle[ret];
				g_cross_avg_angle[g_cross_avg_index] = angle;
				g_cross_avg_index++;
				if (g_cross_avg_index == CROSS_AVG_NUM)
				{
					g_cross_avg_index = 0;
				}

				g_cross_avg_angle_num++;

				double avg = 0;
				for (i=0; i<g_cross_avg_angle_num; i++)
				{
					avg += g_cross_avg_angle[i];
				}
				avg /= g_cross_avg_angle_num;
				get_avg_mid_line(avg);
			}//[if]
			else
			{
				double angle = g_morphin_angle[ret];
				g_cross_avg_angle[g_cross_avg_index] = angle;
				g_cross_avg_index++;

				if (g_cross_avg_index == CROSS_AVG_NUM)
				{
					g_cross_avg_index = 0;
				}

				double avg = 0;
				for (i=0; i<g_cross_avg_angle_num; i++)
				{
					avg += g_cross_avg_angle[i];
				}
				avg /= g_cross_avg_angle_num;
				get_avg_mid_line(avg);
			}//[else]

			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			//g_fidelity = 1;
		}//[else]
	}
	else
	{
		for (i = 0; i< NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[i], &subgoal_show[i]);

		//[检测是否到达子目标点，取出下一个子目标点]
		COOR2 temp_cur_subgoal;

#ifdef MBUG_OPEN_
		MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
		
		diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
		double xx, yy;
		xx = temp_cur_subgoal.x;
		yy = temp_cur_subgoal.y - 400;

#ifdef MBUG_OPEN_
		MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif

		double dist = sqrt(xx * xx + yy * yy);

#ifdef MBUG_OPEN_
		MBUG("dist to the cur sub goal pt: %f\n", dist);
#endif

		while (dist < 400 || yy < 0)//[条件之内认为到达子目标点]
		{
			g_cross_cur_subgoal++;

#ifdef MBUG_OPEN_
			MBUG("change to the next sub goal pt \n");
#endif

			if (g_cross_cur_subgoal == NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
			{
				g_cross_reach_goal = 1;
				g_cross_quit_start_pt = pl_input->state.pos.ins_coord;

#ifdef MBUG_OPEN_
				MBUG("reach the final sub goal pt\n");
#endif
				break;
			}

#ifdef MBUG_OPEN_
			MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif

			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
			xx = temp_cur_subgoal.x;
			yy = temp_cur_subgoal.y - 400;
			dist = sqrt(xx * xx + yy * yy);

#ifdef MBUG_OPEN_
			MBUG("dist to the cur sub goal pt: %f\n", dist);
			MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
		}

		//[到达目标点了]
		//[@@状态切换的问题]
		if (g_cross_reach_goal == 1)
		{
#ifdef MBUG_OPEN_
			MBUG("reach the goal\n");
#endif

			g_cross_cur_yaw = g_yaw;
			CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
			MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif

			//[计算航向角偏移]
			double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);

			if (fabs(det_yaw - g_cross_turn_yaw) < g_left_turn_alpha)
			{
#ifdef MBUG_OPEN_
				MBUG("reach the yaw\n");
#endif
				g_cross_reach_goal_yaw = 1;
			}


			if (g_multi_lane.ok_times[1] == 2 && g_multi_lane.ok_times[2] == 2)
			{//[道路稳定了，提前切出]
				g_cross_reach_goal_yaw = 1;
			}

			if (dist_point(&pl_input->state.pos.ins_coord, &g_cross_quit_start_pt) > g_cross_quit_dist)
			{//[超距]
				g_cross_reach_goal_yaw = 1;
			}

			if (g_cross_reach_goal_yaw == 0)
			{//[尚未到达目标航向角]
#ifdef MBUG_OPEN_
				MBUG("haven't reach the yaw\n");
#endif
				//[设置一个虚拟目标点]
				double adjust_yaw = 0;
				adjust_yaw = det_yaw - g_cross_turn_yaw;
				CHECK_YAW(adjust_yaw);
#ifdef MBUG_OPEN_
				MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
				adjust_yaw = adjust_yaw / 180 * PI;

				COOR2 virtual_pt;
				virtual_pt.x = (INT32)(sin(adjust_yaw) * 2000);
				virtual_pt.y = (INT32)(cos(adjust_yaw) * 2000);
#ifdef MBUG_OPEN_
				MBUG("virtual_pt (%d, %d)\n", virtual_pt.x, virtual_pt.y);
#endif
				//[路口规划]
				double out_rate = 0;
				ret = get_best_morphin_line6(virtual_pt, out_rate);
				if (ret >= 0)
					g_cross_travel_rate = out_rate;
				else
					g_cross_travel_rate = 0;

//				MORPHIN2 morphin = g_morhpin2[0];

				if (ret == -1)
				{
#ifdef MBUG_OPEN_
					MBUG("morphin no way to go\n");
#endif
				}
				else
				{
					if (g_cross_avg_angle_num != CROSS_AVG_NUM)
					{
						double angle = g_morphin_angle[ret];
						g_cross_avg_angle[g_cross_avg_index] = angle;
						g_cross_avg_index++;
						if (g_cross_avg_index == CROSS_AVG_NUM)
						{
							g_cross_avg_index = 0;
						}

						g_cross_avg_angle_num++;

						double avg = 0;
						for (i=0; i<g_cross_avg_angle_num; i++)
						{
							avg += g_cross_avg_angle[i];
						}
						avg /= g_cross_avg_angle_num;
						get_avg_mid_line(avg);
					}//[if]
					else
					{
						double angle = g_morphin_angle[ret];
						g_cross_avg_angle[g_cross_avg_index] = angle;
						g_cross_avg_index++;

						if (g_cross_avg_index == CROSS_AVG_NUM)
						{
							g_cross_avg_index = 0;
						}

						double avg = 0;
						for (i=0; i<g_cross_avg_angle_num; i++)
						{
							avg += g_cross_avg_angle[i];
						}
						avg /= g_cross_avg_angle_num;
						get_avg_mid_line(avg);
					}//[else]

					g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					//g_fidelity = 1;
				}
			}//[end if (g_cross_reach_goal_yaw == 0)]
			else
			{//[满足目标点和航向角，状态变量清零，状态切换]
				g_cross_reach_goal = 0;
				g_cross_reach_goal_yaw = 0;
				g_cross_flag = 0;
				g_subgoal_adjust_flag = 0;
				//[@@发送事件，切换全局状态！！！]
				//ret = -1;
				for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
				{
					g_mid_line[i].x = 0;
					g_mid_line[i].y = i * 100;
				}
				g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
				//g_fidelity = 1;
				ret = 1;
				pl_local_data->event = (EVENT)E_CROSS_OVER;
				memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));
#ifdef MBUG_OPEN_
				MBUG("E_CROSS_OVER has send!\n");
#endif
				g_finish_cross = 10;
				g_cross_smooth_start_pt.x = pl_input->state.pos.ins_coord.x;
				g_cross_smooth_start_pt.y = pl_input->state.pos.ins_coord.y;
				return ret;
			}
		}
		else
		{
			//[路口规划]
			double out_rate = 0;
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);

			ret = get_best_morphin_line6(temp_cur_subgoal, out_rate);

			if (ret >= 0)
				g_cross_travel_rate = out_rate;
			else
				g_cross_travel_rate = 0;

			if (ret == -1)
			{
#ifdef MBUG_OPEN_
				MBUG("morphin no way to go\n");
#endif
			}
			else
			{
				if (g_cross_avg_angle_num != CROSS_AVG_NUM)
				{
					double angle = g_morphin_angle[ret];
					g_cross_avg_angle[g_cross_avg_index] = angle;
					g_cross_avg_index++;
					if (g_cross_avg_index == CROSS_AVG_NUM)
					{
						g_cross_avg_index = 0;
					}

					g_cross_avg_angle_num++;

					double avg = 0;
					for (i=0; i<g_cross_avg_angle_num; i++)
					{
						avg += g_cross_avg_angle[i];
					}
					avg /= g_cross_avg_angle_num;
					get_avg_mid_line(avg);
				}//[if]
				else
				{
					double angle = g_morphin_angle[ret];
					g_cross_avg_angle[g_cross_avg_index] = angle;
					g_cross_avg_index++;

					if (g_cross_avg_index == CROSS_AVG_NUM)
					{
						g_cross_avg_index = 0;
					}

					double avg = 0;
					for (i=0; i<g_cross_avg_angle_num; i++)
					{
						avg += g_cross_avg_angle[i];
					}
					avg /= g_cross_avg_angle_num;
					get_avg_mid_line(avg);
				}//[else]

				g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
				//g_fidelity = 1;
			}//[else]
		}//[else]
	}

	return ret;
}

int cross_right_turn(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data)
{
	int ret = 0;

	int i;
	COOR2 start_pt;
	COOR2 goal_pt;
	POSE_INFO gps_info;

	gps_info = pl_input->state.pos;

	if (g_cross_flag == 0)
	{
		start_pt = pl_input->state.pos.com_coord;
		goal_pt = pl_input->fu_pl_data.cross.exit_point;

		if (pl_input->fu_pl_data.cross.valid_num < 2)
		{
			if (goal_pt.x == 0 && goal_pt.y == 0)
			{//[无出口点]
				g_cross_exit_type = 2;
			}
			else
			{//[仅使用出口点]
				g_cross_exit_type = 1;
			}
	}
		else
		{
			//[有采集路口引导点]
			g_cross_exit_type = 0;
		}

#ifdef MBUG_OPEN_
		MBUG("g_cross_exit_type : %d\n", g_cross_exit_type);
#endif
		COOR2 pts1[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
		int pts1_num;
		COOR2 pts2[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
		int pts2_num;

		if (g_cross_exit_type != 2)
		{
			//[交叉口初始]
			memset(pts1, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			memset(pts2, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			pts1_num = 0;
			pts2_num = 0;

			diff_coor2_e2v(&gps_info, &start_pt, &pts1[0]);
			diff_coor2_e2v(&gps_info, &goal_pt, &pts1[5]);
			if (g_cross_exit_type == 1)
			{
				pts1[1].x = pts1[0].x;
				pts1[1].y = (INT32)(pts1[5].y * 2.0 / 3);
				pts1[2].x = pts1[0].x;
				pts1[2].y = pts1[5].y;
				pts1[3].x = (INT32)((pts1[5].x - pts1[2].x) * 1.0 / 3 + pts1[2].x);
				pts1[3].y = (INT32)((pts1[5].y - pts1[2].y) * 1.0 / 3 + pts1[2].y);
				pts1[4].x = (INT32)((pts1[5].x - pts1[2].x) * 2.0 / 3 + pts1[2].x);
				pts1[4].y = (INT32)((pts1[5].y - pts1[2].y) * 2.0 / 3 + pts1[2].y);
				//pts1[5] = goal_pt;
			}
			else if (g_cross_exit_type == 0)
			{
				diff_coor2_e2v(&gps_info, &pl_input->fu_pl_data.cross.middle_point[0], &pts1[2]);
				diff_coor2_e2v(&gps_info, &pl_input->fu_pl_data.cross.middle_point[1], &pts1[4]);
				//diff_coor2_e2v(&gps_info, &goal_pt, &pts1[5]);
				pts1[1].x = (INT32)((pts1[2].x - pts1[0].x) * 2.0 / 3 + pts1[0].x);
				pts1[1].y = (INT32)((pts1[2].y - pts1[0].y) * 2.0 / 3 + pts1[0].y);
				pts1[3].x = (INT32)((pts1[4].x - pts1[2].x) * 1.0 / 2 + pts1[2].x);
				pts1[3].y = (INT32)((pts1[4].y - pts1[2].y) * 1.0 / 2 + pts1[2].y);
			}

			pts1_num = 6;

			//[由控制点生成贝塞尔曲线]
			pts2_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			get_bezier_line(pts1, pts1_num, pts2, pts2_num);


#ifdef MBUG_OPEN_
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
				MBUG("pts2: (%d, %d)\n", pts2[i].x, pts2[i].y);
#endif


			//[转换到惯导系统GPS的坐标系下]
			for (i = 0; i < pts2_num; i++)
				diff_coor2_v2e(&gps_info, &pts2[i], &g_cross_subgoal_pts[i]);

			g_cross_subgoal_pts_num = pts2_num;
		}

		g_cross_cur_subgoal = 0;
		g_cross_reach_goal = 0;
		g_cross_start_yaw = g_yaw;


		CONVERT_YAW(g_cross_start_yaw);
		g_cross_reach_goal_yaw = 0;

		if (g_cross_exit_type == 0)
		{//[有采集过路口，那么计算所需转向角]
			COOR2 vec_goal;//[计算初始航向角和出口航向角之间的角度]
			vec_goal.x = pts1[pts1_num - 1].x - pts1[pts1_num - 2].x;
			vec_goal.y = pts1[pts1_num - 1].y - pts1[pts1_num - 2].y;
			g_cross_goal_yaw = (INT32)(atan2(vec_goal.y, vec_goal.x) * 180 / PI - 90);

			g_cross_turn_yaw = g_cross_goal_yaw - g_cross_start_yaw;
			CHECK_YAW(g_cross_turn_yaw);
		}
		else
		{
			g_cross_turn_yaw = -90;//[右转90]
		}

#ifdef MBUG_OPEN_
		MBUG("g_cross_turn_yaw: %d\n", g_cross_turn_yaw);
#endif


		for (i = 0; i < CROSS_AVG_NUM; i++)
			g_cross_avg_angle[i] = 90.0;

		g_cross_avg_angle_num = 0;
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_cross_travel_rate = 0;
		g_cross_avg_index = 0;
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_subgoal_adjust_flag = 0;
		g_cross_finish_signal = 0;
		g_cross_flag = 1;

		g_cross_quit_dist_sum = 0;
		memset(&g_cross_quit_start_pt, 0, sizeof(COOR2));

		g_finish_cross = 0;
		memset(&g_cross_smooth_start_pt, 0, sizeof(COOR2));
		memset(&g_cross_smooth_end_pt, 0, sizeof(COOR2));
}


	//	if (g_cross_reach_goal == 1 && g_cross_exit_type != 2)
	if (g_cross_cur_subgoal > NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1 && g_cross_exit_type != 2)
	{//[点已经跟随完毕，检测退出条件]
		g_cross_cur_yaw = g_yaw;
		CONVERT_YAW(g_cross_cur_yaw);

#ifdef MBUG_OPEN_
		MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif

		//[计算航向角偏移]
		double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
		CHECK_YAW(det_yaw);

		if (fabs(det_yaw - (g_cross_turn_yaw)) < g_right_turn_alpha)
		{
			if (g_multi_lane.ok_times[1] == 2 && g_multi_lane.ok_times[2] == 2)
			{//[道路稳定了]
				g_cross_finish_signal = 1;
			}
		}
	}
	else if (g_cross_exit_type == 2)
	{//[无目标点，就凭角度]
		g_cross_cur_yaw = g_yaw;
		CONVERT_YAW(g_cross_cur_yaw);

#ifdef MBUG_OPEN_
		MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif

		//[计算航向角偏移]
		double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
		CHECK_YAW(det_yaw);

		if (fabs(det_yaw - (g_cross_turn_yaw)) < g_right_turn_alpha)
		{
			g_cross_finish_signal = 1;
		}
	}

	if (g_cross_finish_signal == 1)
	{
#ifdef MBUG_OPEN_
		MBUG("g_cross_finish_signal : %d\n", g_cross_finish_signal);
#endif
		g_cross_reach_goal = 0;
		g_cross_reach_goal_yaw = 0;
		g_cross_flag = 0;
		g_subgoal_adjust_flag = 0;
		g_cross_finish_signal = 0;
		g_cross_exit_type = 0;
		//[@@发送事件，切换全局状态！！！]
		//ret = -1;
		for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			g_mid_line[i].x = 0;
			g_mid_line[i].y = i * 100;
		}
		g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
		//g_fidelity = 1;
		ret = 1;
		pl_local_data->event = (EVENT)E_CROSS_OVER;
		memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));

#ifdef MBUG_OPEN_
		MBUG("E_CROSS_OVER has send!\n");
#endif

		g_finish_cross = 10;
		g_cross_smooth_start_pt.x = pl_input->state.pos.ins_coord.x;
		g_cross_smooth_start_pt.y = pl_input->state.pos.ins_coord.y;
		return ret;
	}

	if (g_cross_exit_type == 2)
	{//[无出口点，向偏右一点方向漫游]
		COOR2 pt;

		g_cross_cur_yaw = g_yaw;
		CONVERT_YAW(g_cross_cur_yaw);

#ifdef MBUG_OPEN_
		MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif

		//[计算航向角偏移]
		double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
		CHECK_YAW(det_yaw);

		if (g_cross_reach_goal_yaw == 0)
		{
			if (fabs(det_yaw - g_cross_turn_yaw) < g_right_turn_alpha)
			{
				g_cross_reach_goal_yaw = 1;
			}
		}

		if (g_cross_reach_goal_yaw == 0)
		{//[没有出点]
			if (g_road_type == 0)
			{
				if (fabs(det_yaw - (-90.0)) > 70.0)
				{
					pt.x = 110;
					pt.y = 800;
				}
				else if (fabs(det_yaw - (-90.0)) > 50.0)
				{
					pt.x = 200;
					pt.y = 800;

				}
				else if (fabs(det_yaw - (-90.0)) > 40.0)
				{
					pt.x = 250;
					pt.y = 800;
				}
				else
				{
					pt.x = 110;
					pt.y = 800;
				}
			}
			else
			{
				pt.x = 720;
				pt.y = 0;
			}

		}
		else
		{
			double adjust_yaw = 0;
			g_cross_cur_yaw = g_yaw;
			CONVERT_YAW(g_cross_cur_yaw);

#ifdef MBUG_OPEN_
			MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif

			//[计算航向角偏移]
			det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);

			//[设置一个虚拟目标点]
			adjust_yaw = det_yaw - g_cross_turn_yaw;
			CHECK_YAW(adjust_yaw);

#ifdef MBUG_OPEN_
			MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif

			adjust_yaw = adjust_yaw * PI / 180;
			double theta = get_road_direction(pl_input, 3, adjust_yaw);
			theta = theta * PI / 180;
			adjust_yaw = 0.3 * adjust_yaw + 0.7 * theta;

			pt.x = (INT32)(sin(adjust_yaw) * 2000);
			pt.y = (INT32)(cos(adjust_yaw) * 2000);
		}


		double out_rate = 0;
		if (g_cross_road_type == 0)
		{
			ret = get_best_morphin_line6(pt, out_rate);
		}
		else
		{
			ret = get_best_morphin_line55(pt, out_rate);
		}

		if (ret >= 0)
			g_cross_travel_rate = out_rate;
		else
			g_cross_travel_rate = 0;

		if (ret == -1)
		{
#ifdef MBUG_OPEN_
			MBUG("morphin no way to go\n");
#endif
		}
		else
		{
			if (g_cross_avg_angle_num != CROSS_AVG_NUM)
			{
				double angle = g_morphin_angle[ret];
				g_cross_avg_angle[g_cross_avg_index] = angle;
				g_cross_avg_index++;
				if (g_cross_avg_index == CROSS_AVG_NUM)
				{
					g_cross_avg_index = 0;
				}

				g_cross_avg_angle_num++;

				double avg = 0;
				for (i = 0; i < g_cross_avg_angle_num; i++)
				{
					avg += g_cross_avg_angle[i];
				}
				avg /= g_cross_avg_angle_num;
				get_avg_mid_line(avg);
			}//[if]
			else
			{
				double angle = g_morphin_angle[ret];
				g_cross_avg_angle[g_cross_avg_index] = angle;
				g_cross_avg_index++;

				if (g_cross_avg_index == CROSS_AVG_NUM)
				{
					g_cross_avg_index = 0;
				}

				double avg = 0;
				for (i = 0; i < g_cross_avg_angle_num; i++)
				{
					avg += g_cross_avg_angle[i];
				}
				avg /= g_cross_avg_angle_num;
				get_avg_mid_line(avg);
			}//[else]

			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			//g_fidelity = 1;
		}//[else]
	}
	else
	{
		for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[i], &subgoal_show[i]);

		//[检测是否到达子目标点，取出下一个子目标点]
		COOR2 temp_cur_subgoal;

#ifdef MBUG_OPEN_
		MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif

		diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
		double xx, yy;
		xx = temp_cur_subgoal.x;
		yy = temp_cur_subgoal.y - 400;

#ifdef MBUG_OPEN_
		MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif

		double dist = sqrt(xx * xx + yy * yy);

#ifdef MBUG_OPEN_
		MBUG("dist to the cur sub goal pt: %f\n", dist);
#endif

		while (dist < 400 || yy < 0)//[条件之内认为到达子目标点]
		{
			g_cross_cur_subgoal++;

#ifdef MBUG_OPEN_
			MBUG("change to the next sub goal pt \n");
#endif

			if (g_cross_cur_subgoal == NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
			{
				g_cross_reach_goal = 1;
				g_cross_quit_start_pt = pl_input->state.pos.ins_coord;

#ifdef MBUG_OPEN_
				MBUG("reach the final sub goal pt\n");
#endif
				break;
			}

#ifdef MBUG_OPEN_
			MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif

			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
			xx = temp_cur_subgoal.x;
			yy = temp_cur_subgoal.y - 400;
			dist = sqrt(xx * xx + yy * yy);

#ifdef MBUG_OPEN_
			MBUG("dist to the cur sub goal pt: %f\n", dist);
			MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
		}

		//[到达目标点了]
		//[@@状态切换的问题]
		if (g_cross_reach_goal == 1)
		{
#ifdef MBUG_OPEN_
			MBUG("reach the goal\n");
#endif

			g_cross_cur_yaw = g_yaw;
			CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
			MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif

			//[计算航向角偏移]
			double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);

			if (fabs(det_yaw - g_cross_turn_yaw) < g_right_turn_alpha)
			{
#ifdef MBUG_OPEN_
				MBUG("reach the yaw\n");
#endif
				g_cross_reach_goal_yaw = 1;
			}


			if (g_multi_lane.ok_times[1] == 2 && g_multi_lane.ok_times[2] == 2)
			{//[道路稳定了，提前退出]
				g_cross_reach_goal_yaw = 1;
			}

			if (dist_point(&pl_input->state.pos.ins_coord, &g_cross_quit_start_pt) > g_cross_quit_dist)
			{//[超距]
				g_cross_reach_goal_yaw = 1;
			}

			if (g_cross_reach_goal_yaw == 0)
			{//[尚未到达目标航向角]
#ifdef MBUG_OPEN_
				MBUG("haven't reach the yaw\n");
#endif
				//[设置一个虚拟目标点]
				double adjust_yaw = 0;
				adjust_yaw = det_yaw - g_cross_turn_yaw;
				CHECK_YAW(adjust_yaw);
#ifdef MBUG_OPEN_
				MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
				adjust_yaw = adjust_yaw / 180 * PI;

				COOR2 virtual_pt;
				virtual_pt.x = (INT32)(sin(adjust_yaw) * 2000);
				virtual_pt.y = (INT32)(cos(adjust_yaw) * 2000);
#ifdef MBUG_OPEN_
				MBUG("virtual_pt (%d, %d)\n", virtual_pt.x, virtual_pt.y);
#endif
				//[路口规划]
				double out_rate = 0;
				if (g_cross_road_type == 0)
				{
					ret = get_best_morphin_line6(virtual_pt, out_rate);
				}
				else
				{
					ret = get_best_morphin_line55(virtual_pt, out_rate);
				}

				if (ret >= 0)
					g_cross_travel_rate = out_rate;
				else
					g_cross_travel_rate = 0;

				//				MORPHIN2 morphin = g_morhpin2[0];

				if (ret == -1)
				{
#ifdef MBUG_OPEN_
					MBUG("morphin no way to go\n");
#endif
				}
				else
				{
					if (g_cross_avg_angle_num != CROSS_AVG_NUM)
					{
						double angle = g_morphin_angle[ret];
						g_cross_avg_angle[g_cross_avg_index] = angle;
						g_cross_avg_index++;
						if (g_cross_avg_index == CROSS_AVG_NUM)
						{
							g_cross_avg_index = 0;
						}

						g_cross_avg_angle_num++;

						double avg = 0;
						for (i = 0; i < g_cross_avg_angle_num; i++)
						{
							avg += g_cross_avg_angle[i];
						}
						avg /= g_cross_avg_angle_num;
						get_avg_mid_line(avg);
					}//[if]
					else
					{
						double angle = g_morphin_angle[ret];
						g_cross_avg_angle[g_cross_avg_index] = angle;
						g_cross_avg_index++;

						if (g_cross_avg_index == CROSS_AVG_NUM)
						{
							g_cross_avg_index = 0;
						}

						double avg = 0;
						for (i = 0; i < g_cross_avg_angle_num; i++)
						{
							avg += g_cross_avg_angle[i];
						}
						avg /= g_cross_avg_angle_num;
						get_avg_mid_line(avg);
					}//[else]

					g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					//g_fidelity = 1;
				}
			}//[end if (g_cross_reach_goal_yaw == 0)]
			else
			{//[满足目标点和航向角，状态变量清零，状态切换]
				g_cross_reach_goal = 0;
				g_cross_reach_goal_yaw = 0;
				g_cross_flag = 0;
				g_subgoal_adjust_flag = 0;
				//[@@发送事件，切换全局状态！！！]
				for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
				{
					g_mid_line[i].x = 0;
					g_mid_line[i].y = i * 100;
				}
				g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
				//g_fidelity = 1;
				ret = 1;
				pl_local_data->event = (EVENT)E_CROSS_OVER;
				memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));
#ifdef MBUG_OPEN_
				MBUG("E_CROSS_OVER has send!\n");
#endif
				g_finish_cross = 10;
				g_cross_smooth_start_pt.x = pl_input->state.pos.ins_coord.x;
				g_cross_smooth_start_pt.y = pl_input->state.pos.ins_coord.y;
				return ret;
			}
		}
		else
		{
			//[路口规划]
			double out_rate = 0;
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);

			if (g_cross_road_type == 0)
			{
				ret = get_best_morphin_line6(temp_cur_subgoal, out_rate);
			}
			else
			{
				ret = get_best_morphin_line55(temp_cur_subgoal, out_rate);
			}

			if (ret >= 0)
				g_cross_travel_rate = out_rate;
			else
				g_cross_travel_rate = 0;

			if (ret == -1)
			{
#ifdef MBUG_OPEN_
				MBUG("morphin no way to go\n");
#endif
			}
			else
			{
				if (g_cross_avg_angle_num != CROSS_AVG_NUM)
				{
					double angle = g_morphin_angle[ret];
					g_cross_avg_angle[g_cross_avg_index] = angle;
					g_cross_avg_index++;
					if (g_cross_avg_index == CROSS_AVG_NUM)
					{
						g_cross_avg_index = 0;
					}

					g_cross_avg_angle_num++;

					double avg = 0;
					for (i = 0; i < g_cross_avg_angle_num; i++)
					{
						avg += g_cross_avg_angle[i];
					}
					avg /= g_cross_avg_angle_num;
					get_avg_mid_line(avg);
				}//[if]
				else
				{
					double angle = g_morphin_angle[ret];
					g_cross_avg_angle[g_cross_avg_index] = angle;
					g_cross_avg_index++;

					if (g_cross_avg_index == CROSS_AVG_NUM)
					{
						g_cross_avg_index = 0;
					}

					double avg = 0;
					for (i = 0; i < g_cross_avg_angle_num; i++)
					{
						avg += g_cross_avg_angle[i];
					}
					avg /= g_cross_avg_angle_num;
					get_avg_mid_line(avg);
				}//[else]

				g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
				//g_fidelity = 1;
			}//[else]
		}//[else]
	}

	return ret;
}

int cross_straight(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data)
{
	int ret = 0;

	int i;
	COOR2 start_pt;
	COOR2 mid_pt;
	COOR2 goal_pt;
	POSE_INFO gps_info;

	gps_info = pl_input->state.pos;

	if (g_cross_flag == 0)
	{
		start_pt = pl_input->state.pos.com_coord;
		goal_pt = pl_input->fu_pl_data.cross.exit_point;

#ifdef DEBUG_MODE
		goal_pt.x = 10938338;
		goal_pt.y = -6767650;
#endif

		//[根据出口点判断是何种类型]
		if (goal_pt.x == 0 && goal_pt.y == 0)
		{//[无出口点]
			g_cross_exit_type = 2;
		}
		else if (fabs((double)goal_pt.x) < 20000 && fabs((double)goal_pt.y) < 20000)
		{//[估算的出口点不会超出这个范围]
			g_cross_exit_type = 1;
		}
		else
		{//[有出口点]
			g_cross_exit_type = 0;
		}

#ifdef MBUG_OPEN_
		MBUG("start_pt : (%d, %d)\n", start_pt.x, start_pt.y);
		MBUG("goal_pt : (%d, %d)\n", goal_pt.x, goal_pt.y);
#endif
		if (g_cross_exit_type != 2)
		{
			//[交叉口初始]
			COOR2 pts1[3];
			//int pts1_num;
			COOR2 pts2[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
			int pts2_num;

			diff_coor2_e2v(&gps_info, &start_pt, &pts1[0]);
			if (g_cross_exit_type == 1)
			{
				pts1[2] = goal_pt;
			}
			else if (g_cross_exit_type == 0)
			{
				diff_coor2_e2v(&gps_info, &goal_pt, &pts1[2]);
			}

			mid_pt.x = (pts1[0].x + pts1[2].x) / 2;
			mid_pt.y = (pts1[0].y + pts1[2].y) / 2;

			pts1[1] = mid_pt;
			//pts1_num = 3;

#ifdef MBUG_OPEN_
			for (i=0; i<3; i++)
				MBUG("pts1: (%d, %d)\n", pts1[i].x, pts1[i].y);
#endif
			//[其实可以不用贝塞尔曲线，随意改]
// 			pts2_num = SUB_GOAL_PT_NUM;
// 			get_bezier_line(pts1, pts1_num, pts2, pts2_num);
			pts2_num = 2;
			pts2[0] = pts1[0];
			pts2[1] = pts1[2];
			//memcpy(pts2, pts1, sizeof(COOR2) * pts2_num);

#ifdef MBUG_OPEN_
			for (i = 0; i<pts2_num; i++)
				MBUG("pts2: (%d, %d)\n", pts2[i].x, pts2[i].y);
#endif
			//[转换到惯导系统GPS的坐标系下]
			for (i=0; i<pts2_num; i++)
				diff_coor2_v2e(&gps_info, &pts2[i], &g_cross_subgoal_pts[i]);

			g_cross_subgoal_pts_num = pts2_num;
		}

		g_cross_cur_subgoal = 0;
		g_cross_reach_goal = 0;
		g_cross_start_yaw = g_yaw;
		CONVERT_YAW(g_cross_start_yaw);
		g_cross_reach_goal_yaw = 0;

		for (i=0; i<CROSS_AVG_NUM; i++)
		{
			g_cross_avg_angle[i] = 90.0;
		}
		g_cross_avg_angle_num = 0;
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_cross_travel_rate = 0;
		g_cross_avg_index = 0;
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_subgoal_adjust_flag = 0;
		g_cross_finish_signal = 0;
		g_cross_flag = 1;

		g_cross_quit_dist_sum = 0;
		memset(&g_cross_quit_start_pt, 0, sizeof(COOR2));

		g_finish_cross = 0;
	}

	//	if (g_cross_reach_goal == 1 && g_cross_exit_type != 2)
	if (g_cross_cur_subgoal == 1 && g_cross_exit_type != 2)
	{
		g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
		MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
		CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
		MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif

		if (g_road_type == 0)
		{
			if (g_multi_lane.ok_times[1] == 2 && g_multi_lane.ok_times[2] == 2)
			{//[道路稳定了]
				g_cross_finish_signal = 1;
			}
		}
		else
		{
			if (g_natural_boundary.l_nums >= 2 && g_natural_boundary.r_nums >= 2)
				g_cross_finish_signal = 1;
		}
	}
	else if (g_cross_exit_type == 2)
	{
		g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
		MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
		CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
		MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif

		if (g_road_type == 0)
		{
			if (g_multi_lane.ok_times[1] == 2 && g_multi_lane.ok_times[2] == 2)
			{//[道路稳定了]
				g_cross_finish_signal = 1;
			}
		}
		else
		{
			if (g_natural_boundary.l_nums >= 2 && g_natural_boundary.r_nums >= 2)
				g_cross_finish_signal = 1;
		}

	}


	if (g_cross_finish_signal == 1)
	{
#ifdef MBUG_OPEN_
		MBUG("g_cross_finish_signal : %d\n", g_cross_finish_signal);
#endif
		g_cross_reach_goal = 0;
		g_cross_reach_goal_yaw = 0;
		g_cross_flag = 0;
		g_cross_finish_signal = 0;
		g_cross_exit_type = 0;
		//[@@发送事件，切换全局状态！！！]
		//ret = -1;
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			g_mid_line[i].x = 0;
			g_mid_line[i].y = i * 100;
		}
		g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
//		g_fidelity = 1;
		ret = 1;
		pl_local_data->event = (EVENT)E_CROSS_OVER;
		memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));
#ifdef MBUG_OPEN_
		MBUG("E_CROSS_OVER has send!\n");
#endif
		return ret;
	}

	if (g_cross_exit_type == 2)
	{//[无出口点，向前方漫游]
		COOR2 pt;
		g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
		MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
		CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
		MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
		//[计算航向角偏移]
		double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
		CHECK_YAW(det_yaw);

		if (fabs(det_yaw - 0.0) < g_straight_alpha)
		{
			pt.x = 0;
			pt.y = 800;
		}
		else
		{

			//[设置一个虚拟目标点]
			double adjust_yaw = 0;
			adjust_yaw = det_yaw - 0;
#ifdef MBUG_OPEN_
			MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
			adjust_yaw = adjust_yaw / 180 * PI;

			pt.x = (INT32)(sin(adjust_yaw) * 2000);
			pt.y = (INT32)(cos(adjust_yaw) * 2000);
#ifdef MBUG_OPEN_
			MBUG("virtual_pt (%d, %d)\n", pt.x, pt.y);
#endif
		}

		double out_rate = 0;
		ret = get_best_morphin_line6(pt, out_rate);
		if (ret >= 0)
			g_cross_travel_rate = out_rate;
		else
			g_cross_travel_rate = 0;

		if (ret == -1)
		{
#ifdef MBUG_OPEN_
			MBUG("morphin no way to go\n");
#endif
		}
		else
		{
			if (g_cross_avg_angle_num != CROSS_AVG_NUM / 2)
			{
				double angle = g_morphin_angle[ret];
				g_cross_avg_angle[g_cross_avg_index] = angle;
				g_cross_avg_index++;
				if (g_cross_avg_index == CROSS_AVG_NUM / 2)
				{
					g_cross_avg_index = 0;
				}

				g_cross_avg_angle_num++;

				double avg = 0;
				for (i=0; i<g_cross_avg_angle_num; i++)
				{
					avg += g_cross_avg_angle[i];
				}
				avg /= g_cross_avg_angle_num;
				get_avg_mid_line(avg);
			}//[if]
			else
			{
				double angle = g_morphin_angle[ret];
				g_cross_avg_angle[g_cross_avg_index] = angle;
				g_cross_avg_index++;

				if (g_cross_avg_index == CROSS_AVG_NUM / 2)
				{
					g_cross_avg_index = 0;
				}

				double avg = 0;
				for (i=0; i<g_cross_avg_angle_num; i++)
				{
					avg += g_cross_avg_angle[i];
				}
				avg /= g_cross_avg_angle_num;
				get_avg_mid_line(avg);
			}//[else]

			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
//			g_fidelity = 1;
		}//[else]
	}
	else
	{
		for (i = 0; i< 2; i++)
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[i], &subgoal_show[i]);

		//[检测是否到达子目标点，取出下一个子目标点]
		COOR2 temp_cur_subgoal;
#ifdef MBUG_OPEN_
		MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
		diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
		double xx, yy;
		xx = temp_cur_subgoal.x;
		yy = temp_cur_subgoal.y - 400;
#ifdef MBUG_OPEN_
		MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
		double dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
		MBUG("dist to the cur sub goal pt: %f\n", dist);
#endif
		while (dist < 400 || yy < 0)//[条件之内认为到达子目标点]
		{
			g_cross_cur_subgoal++;
#ifdef MBUG_OPEN_
			MBUG("change to the next sub goal pt \n");
#endif
			if (g_cross_cur_subgoal == 2)
			{
				g_cross_reach_goal = 1;
#ifdef MBUG_OPEN_
				MBUG("reach the final sub goal pt\n");
#endif

				g_cross_quit_start_pt = pl_input->state.pos.ins_coord;
				break;
			}
#ifdef MBUG_OPEN_
			MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
			xx = temp_cur_subgoal.x;
			yy = temp_cur_subgoal.y - 400;
			dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
			MBUG("dist to the cur sub goal pt: %f\n", dist);
			MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
		}

		//[到达目标点了]
		//[@@状态切换的问题]
		if (g_cross_reach_goal == 1)
		{
#ifdef MBUG_OPEN_
			MBUG("reach the goal\n");
#endif
			g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
			MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
			MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			//[计算航向角偏移]
			double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);
			
			g_cross_reach_goal_yaw = 1;

			if (g_cross_reach_goal_yaw == 0)
			{//[尚未到达目标航向角]
#ifdef MBUG_OPEN_
				MBUG("haven't reach the yaw\n");
#endif
				//[设置一个虚拟目标点]
				double adjust_yaw = 0;
				adjust_yaw = det_yaw - 0;
#ifdef MBUG_OPEN_
				MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
				adjust_yaw = adjust_yaw / 180 * PI;

				COOR2 virtual_pt;
				virtual_pt.x = (INT32)(sin(adjust_yaw) * 2000);
				virtual_pt.y = (INT32)(cos(adjust_yaw) * 2000);
#ifdef MBUG_OPEN_
				MBUG("virtual_pt (%d, %d)\n", virtual_pt.x, virtual_pt.y);
#endif
				//[路口规划]
				double out_rate = 0;
				ret = get_best_morphin_line6(virtual_pt, out_rate);
				if (ret >= 0)
					g_cross_travel_rate = out_rate;
				else
					g_cross_travel_rate = 0;

//				MORPHIN2 morphin = g_morhpin2[0];

				if (ret == -1)
				{
#ifdef MBUG_OPEN_
					MBUG("morphin no way to go\n");
#endif
				}
				else
				{
					if (g_cross_avg_angle_num != CROSS_AVG_NUM / 2)
					{
						double angle = g_morphin_angle[ret];
						g_cross_avg_angle[g_cross_avg_index] = angle;
						g_cross_avg_index++;
						if (g_cross_avg_index == CROSS_AVG_NUM / 2)
						{
							g_cross_avg_index = 0;
						}

						g_cross_avg_angle_num++;

						double avg = 0;
						for (i=0; i<g_cross_avg_angle_num; i++)
						{
							avg += g_cross_avg_angle[i];
						}
						avg /= g_cross_avg_angle_num;
						get_avg_mid_line(avg);
					}//[if]
					else
					{
						double angle = g_morphin_angle[ret];
						g_cross_avg_angle[g_cross_avg_index] = angle;
						g_cross_avg_index++;

						if (g_cross_avg_index == CROSS_AVG_NUM / 2)
						{
							g_cross_avg_index = 0;
						}

						double avg = 0;
						for (i=0; i<g_cross_avg_angle_num; i++)
						{
							avg += g_cross_avg_angle[i];
						}
						avg /= g_cross_avg_angle_num;
						get_avg_mid_line(avg);
					}//[else]

					g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
//					g_fidelity = 1;
				}
			}//[end if (g_cross_reach_goal_yaw == 0)]
			else
			{//[满足目标点和航向角，状态变量清零，状态切换]
				g_cross_reach_goal = 0;
				g_cross_reach_goal_yaw = 0;
				g_cross_flag = 0;
				g_subgoal_adjust_flag = 0;
				//[@@发送事件，切换全局状态！！！]
				for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
				{
					g_mid_line[i].x = 0;
					g_mid_line[i].y = i * 100;
				}
				g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
//				g_fidelity = 1;
				ret = 1;
				pl_local_data->event = (EVENT)E_CROSS_OVER;
				memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));
#ifdef MBUG_OPEN_
				MBUG("E_CROSS_OVER has send!\n");
#endif
				return ret;
			}
		}
		else
		{
			//[路口规划]
			double out_rate = 0;
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
			ret = get_best_morphin_line6(temp_cur_subgoal, out_rate);
			if (ret >= 0)
				g_cross_travel_rate = out_rate;
			else
				g_cross_travel_rate = 0;

			if (ret == -1)
			{
#ifdef MBUG_OPEN_
				MBUG("morphin no way to go\n");
#endif
			}
			else
			{
				if (g_cross_avg_angle_num != CROSS_AVG_NUM / 2)
				{
					double angle = g_morphin_angle[ret];
					g_cross_avg_angle[g_cross_avg_index] = angle;
					g_cross_avg_index++;
					if (g_cross_avg_index == CROSS_AVG_NUM / 2)
					{
						g_cross_avg_index = 0;
					}

					g_cross_avg_angle_num++;

					double avg = 0;
					for (i=0; i<g_cross_avg_angle_num; i++)
					{
						avg += g_cross_avg_angle[i];
					}
					avg /= g_cross_avg_angle_num;
					get_avg_mid_line(avg);
				}//[if]
				else
				{
					double angle = g_morphin_angle[ret];
					g_cross_avg_angle[g_cross_avg_index] = angle;
					g_cross_avg_index++;

					if (g_cross_avg_index == CROSS_AVG_NUM / 2)
					{
						g_cross_avg_index = 0;
					}

					double avg = 0;
					for (i=0; i<g_cross_avg_angle_num; i++)
					{
						avg += g_cross_avg_angle[i];
					}
					avg /= g_cross_avg_angle_num;
					get_avg_mid_line(avg);
				}//[else]

				g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
//				g_fidelity = 1;
			}//[else]
		}//[else]
	}
	return ret;
}

int cross_u_turn_new(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data)
{
	int ret = 0;

	int i;
	POSE_INFO gps_info;

	gps_info = pl_input->state.pos;

	if (g_cross_flag == 0)
	{
		COOR2 pts[3];
		
		pts[0] = pl_input->fu_pl_data.cross.entrance_point;
		pts[1] = pl_input->fu_pl_data.cross.cross_quad.coor2[0];//[取出子目标点]
		pts[2] = pl_input->fu_pl_data.cross.exit_point;

		COOR2 v_pt[3];
		coor2_e2v(&pl_input->state.pos, &pts[0], &v_pt[0]);
		coor2_e2v(&pl_input->state.pos, &pts[2], &v_pt[2]);
		pts[1].x = (v_pt[0].x + v_pt[2].x) / 2;
		pts[1].y = (v_pt[0].y + v_pt[2].y) / 2;

#ifdef DEBUG_MODE
		// 		pts[0].x = 10700450;
		// 		pts[0].y = -6914289;`
		// 		pts[1].x = 10700450;
		// 		pts[1].y = -6914289;
		pts[0].x = 0;
		pts[0].y = 0;
		pts[1].x = 0;
		pts[1].y = 0;
#endif

		if (pts[1].x == 0 && pts[1].y == 0)
		{
			//[没有子目标点]
			g_cross_exit_type = 1;

			diff_coor2_v2e(&gps_info, &pts[0], &g_cross_subgoal_pts[0]);
			diff_coor2_v2e(&gps_info, &pts[2], &g_cross_subgoal_pts[2]);
			g_cross_subgoal_pts_num = 2;
		}
		else
		{
			//[有子目标点]
			g_cross_exit_type = 0;

			//diff_coor2_v2e(&gps_info, &pts[0], &g_cross_subgoal_pts[0]);
			g_cross_subgoal_pts[0] = pts[0];
			diff_coor2_v2e(&gps_info, &pts[1], &g_cross_subgoal_pts[1]);
			g_cross_subgoal_pts[2] = pts[2];
			//diff_coor2_v2e(&gps_info, &pts[2], &g_cross_subgoal_pts[2]);
			g_cross_subgoal_pts_num = 3;
		}

		g_cross_cur_subgoal = 0;
		g_cross_reach_goal = 0;

		//#ifdef DEBUG_MODE
		g_cross_start_yaw = g_yaw;
		//#endif

		CONVERT_YAW(g_cross_start_yaw);
		g_cross_reach_goal_yaw = 0;

		for (i = 0; i<CROSS_AVG_NUM; i++)
		{
			g_cross_avg_angle[i] = 90.0;
		}
		g_cross_avg_angle_num = 0;
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_cross_travel_rate = 0;
		g_cross_avg_index = 0;
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_subgoal_adjust_flag = 0;
		g_cross_finish_signal = 0;
		g_cross_flag = 1;
		g_finish_cross = 0;

		g_uturn_stop_once = g_uturn_stop_once_counter;
		memset(&g_cross_smooth_start_pt, 0, sizeof(COOR2));
		memset(&g_cross_smooth_end_pt, 0, sizeof(COOR2));
	}

	if (g_cross_exit_type == 0)
	{//[有子目标点]
		if (g_cross_cur_subgoal == 2)
		{//[出点]
			g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
			MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
			MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			//[计算航向角偏移]
			double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);

			if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha1)
			{
				if (g_multi_lane.ok_times[1] == 2 && g_multi_lane.ok_times[2] == 2)
				{//[道路稳定了]
					g_cross_finish_signal = 1;
				}
			}
		}

		if (g_cross_finish_signal == 1)
		{
#ifdef MBUG_OPEN_
			MBUG("g_cross_finish_signal : %d\n", g_cross_finish_signal);
#endif
			g_cross_reach_goal = 0;
			g_cross_reach_goal_yaw = 0;
			g_cross_flag = 0;
			g_subgoal_adjust_flag = 0;
			g_cross_finish_signal = 0;
			g_cross_exit_type = 0;
			//[@@发送事件，切换全局状态！！！]
			//ret = -1;
			int step_x = g_last_frame_pt.x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
			int step_y = (g_last_frame_pt.y - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 0;

			for (i = 1; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1; i++)
			{
				g_mid_line[i].x = step_x * i;
				g_mid_line[i].y = step_y * i + 400;
			}
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			//g_fidelity = 1;
			ret = 1;
			pl_local_data->event = (EVENT)E_CROSS_OVER;
			memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));
#ifdef MBUG_OPEN_
			MBUG("E_CROSS_OVER has send!\n");
#endif
			g_finish_cross = 10;
			g_cross_smooth_start_pt.x = pl_input->state.pos.ins_coord.x;
			g_cross_smooth_start_pt.y = pl_input->state.pos.ins_coord.y;
			return ret;
		}//[if (g_cross_finish_signal == 1)]

		//[检测是否到达目标点]
		COOR2 temp_cur_subgoal;
		double xx, yy;
		double dist;
#ifdef MBUG_OPEN_
		MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
		//[********目标点的判断********]
		if (g_cross_cur_subgoal == 0 || g_cross_cur_subgoal == 1)
		{//[子目标点规划]
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);

			xx = temp_cur_subgoal.x;
			yy = temp_cur_subgoal.y - 400;
#ifdef MBUG_OPEN_
			MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
			dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
			MBUG("dist to the cur sub goal pt: %f\n", dist);
#endif
			if (dist < 200 || yy < 0)//[通过子目标点]
			{
				g_cross_cur_subgoal++;
#ifdef MBUG_OPEN_
				MBUG("change to the next sub goal pt \n");
				MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
				diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
				xx = temp_cur_subgoal.x;
				yy = temp_cur_subgoal.y - 400;
				dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
				MBUG("dist to the cur sub goal pt: %f\n", dist);
				MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
			}
		}//[if (g_cross_cur_subgoal == 0)]

		if (g_cross_cur_subgoal == 2)
		{
#ifdef MBUG_OPEN_
			MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
			xx = temp_cur_subgoal.x;
			yy = temp_cur_subgoal.y - 400;

#ifdef MBUG_OPEN_
			MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
			dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
			MBUG("dist to the cur sub goal pt: %f\n", dist);
#endif
			if (g_cross_reach_goal_yaw == 1)
			{
				if (dist < 200 || yy < 200)//[通过第二个目标点]
				{
					g_cross_reach_goal = 1;
#ifdef MBUG_OPEN_
					MBUG("reach the final sub goal pt\n");
#endif
				}
			}
		}//[if (g_cross_cur_subgoal == 1)]
		//[********目标点的判断********]

		if (g_cross_cur_subgoal == 0 || g_cross_cur_subgoal == 1)
		{
			//[子目标点的路口规划]
			double out_rate = 0;
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
			ret = get_best_morphin_line5(temp_cur_subgoal, out_rate);
			if (ret >= 0)
				g_cross_travel_rate = out_rate;
			else
				g_cross_travel_rate = 0;
		}
		else
		{
			double det_yaw;
			double adjust_yaw = 0;
			//[第二个目标点先满足转向角度条件]
			if (g_cross_reach_goal_yaw == 0)
			{
#ifdef MBUG_OPEN_
				MBUG("reach the goal\n");
#endif
				g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
				MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
				MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
				CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
				MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
				MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
				//[计算航向角偏移]
				det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
				CHECK_YAW(det_yaw);
#ifdef MBUG_OPEN_
				MBUG("fabs(fabs(det_yaw) - 180.0) : %lf\n", fabs(fabs(det_yaw) - 180.0));
#endif
				if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha1)
				{//[达到阈值切换状态，以目标点为主]

					g_cross_reach_goal_yaw = 1;
#ifdef MBUG_OPEN_
					MBUG("reach the yaw 1\n");
					MBUG("g_cross_reach_goal_yaw : %d\n", g_cross_reach_goal_yaw);
#endif
				}
			}

			if (g_cross_reach_goal_yaw == 0)
			{//[尚未到达目标航向角]
#ifdef MBUG_OPEN_
				MBUG("haven't reach the yaw\n");
#endif
				//[设置一个虚拟目标点]
				if (det_yaw <= 180 && det_yaw >= 0)
				{
					adjust_yaw = -(180 - det_yaw);
				}
				else
				{
					adjust_yaw = 180 + det_yaw;
				}
#ifdef MBUG_OPEN_
				MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
				adjust_yaw = adjust_yaw / 180 * PI;


				COOR2 virtual_pt;
				virtual_pt.x = (INT32)(sin(adjust_yaw) * 2000);
				virtual_pt.y = (INT32)(cos(adjust_yaw) * 2000);

				//[@@若是车头往右偏了很可能导致车从右边拐，所以强制设置一个左侧虚拟点]
				virtual_pt.x = -700;
				virtual_pt.y = 0;
#ifdef MBUG_OPEN_
				MBUG("virtual_pt (%d, %d)\n", virtual_pt.x, virtual_pt.y);
#endif
				//[路口规划]
				double out_rate = 0;
				ret = get_best_morphin_line5(virtual_pt, out_rate);
				if (ret >= 0)
					g_cross_travel_rate = out_rate;
				else
					g_cross_travel_rate = 0;

			}//[end if (g_cross_reach_goal_yaw == 0)]
			else
			{//[角度满足]
				if (g_cross_reach_goal == 1)
				{//[往180度方向驶去，等待发现道路]
					double theta = 0;
					double adjust_yaw = 0;
					g_cross_cur_yaw = g_yaw;
					CONVERT_YAW(g_cross_cur_yaw);
					det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
					CHECK_YAW(det_yaw);

					if (det_yaw <= 180 && det_yaw >= 0)
					{
						adjust_yaw = -(180 - det_yaw);
					}
					else
					{
						adjust_yaw = 180 + det_yaw;
					}
#ifdef MBUG_OPEN_
					MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
					adjust_yaw = adjust_yaw * PI / 180;

					// 					double s_theta = (adjust_yaw * 180 / PI - 10) * PI / 180;
					// 					double e_theta = (adjust_yaw * 180 / PI + 10) * PI / 180;
					// 					theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

					theta = adjust_yaw;// .3 * adjust_yaw + 0.7 * theta;
					COOR2 pt;
					pt.y = 2000;
					pt.x = (INT32)(tan(theta) * pt.y);

					double out_rate = 0;
					ret = get_best_morphin_line5(pt, out_rate);
					if (ret >= 0)
						g_cross_travel_rate = out_rate;
					else
						g_cross_travel_rate = 0;

				}
				else
				{//[目标点尚未满足]
					//[路口规划]
					double theta = 0;

					g_cross_cur_yaw = g_yaw;
					CONVERT_YAW(g_cross_cur_yaw);
					det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
					CHECK_YAW(det_yaw);

					if (det_yaw <= 180 && det_yaw >= 0)
					{
						adjust_yaw = -(180 - det_yaw);
					}
					else
					{
						adjust_yaw = 180 + det_yaw;
					}
#ifdef MBUG_OPEN_
					MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
					adjust_yaw = adjust_yaw * PI / 180;

					// 					double s_theta = (adjust_yaw * 180 / PI - 10) * PI / 180;
					// 					double e_theta = (adjust_yaw * 180 / PI + 10) * PI / 180;
					// 					theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

					theta = adjust_yaw;// 0.3 * adjust_yaw + 0.7 * theta;
					COOR2 pt;
					pt.y = 2000;
					pt.x = (INT32)(tan(theta) * pt.y);

					double out_rate = 0;
					// 					diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
					// 					ret = get_best_morphin_line5(temp_cur_subgoal, out_rate);
					ret = get_best_morphin_line5(pt, out_rate);
					if (ret >= 0)
						g_cross_travel_rate = out_rate;
					else
						g_cross_travel_rate = 0;
				}//[else]
			}//[else]
		}//[else]
	}//[if (g_cross_exit_type == 0)]

	else if (g_cross_exit_type == 1)
	{//[没有目标点]
		if (g_cross_cur_subgoal == 2)
		{//[出点规划]
			g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
			MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
			MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			//[计算航向角偏移]
			double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);

			if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha3)
			{
				//get_road_direction(pl_input, 2, fabs(det_yaw) - 180.0);
				// 				if (g_multi_lane.ok_times[1] == 2 && g_multi_lane.ok_times[2] == 2)
				// 				{//[道路稳定了]
				g_cross_finish_signal = 1;
				//				}
			}
		}

		if (g_cross_finish_signal == 1)
		{
#ifdef MBUG_OPEN_
			MBUG("g_cross_finish_signal : %d\n", g_cross_finish_signal);
#endif
			g_cross_reach_goal = 0;
			g_cross_reach_goal_yaw = 0;
			g_cross_flag = 0;
			g_subgoal_adjust_flag = 0;
			g_cross_finish_signal = 0;
			g_cross_exit_type = 0;
			//[@@发送事件，切换全局状态！！！]
			//ret = -1;
			int step_x = g_last_frame_pt.x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
			int step_y = (g_last_frame_pt.y - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 0;

			for (i = 1; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1; i++)
			{
				g_mid_line[i].x = step_x * i;
				g_mid_line[i].y = step_y * i + 400;
			}
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			//			g_fidelity = 1;
			ret = 1;
			pl_local_data->event = (EVENT)E_CROSS_OVER;
			memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));
#ifdef MBUG_OPEN_
			MBUG("E_CROSS_OVER has send!\n");
#endif
			g_finish_cross = 10;
			g_cross_smooth_start_pt.x = pl_input->state.pos.ins_coord.x;
			g_cross_smooth_start_pt.y = pl_input->state.pos.ins_coord.y;
			return ret;
		}//[if (g_cross_finish_signal == 1)]

		//[检测是否到达目标点]
		COOR2 temp_cur_subgoal;
		double xx, yy;
		double dist;
#ifdef MBUG_OPEN_
		MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
		//[********目标点的判断********]
		if (g_cross_cur_subgoal == 0)
		{//[没有子目标点规划]
			g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
			MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
			MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			//[计算航向角偏移]
			double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);

			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);

			xx = temp_cur_subgoal.x;
			yy = temp_cur_subgoal.y - 400;
#ifdef MBUG_OPEN_
			MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
			dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
			MBUG("dist to the cur sub goal pt: %f\n", dist);
#endif
			if (dist < 200 || yy < 0)
				g_cross_cur_subgoal = 2;

			COOR2 pt;
			if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha2)
			{
				double theta = 0;
				double adjust_yaw = 0;
				g_cross_cur_yaw = g_yaw;
				CONVERT_YAW(g_cross_cur_yaw);
				det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
				CHECK_YAW(det_yaw);

				if (det_yaw <= 180 && det_yaw >= 0)
				{
					adjust_yaw = -(180 - det_yaw);
				}
				else
				{
					adjust_yaw = 180 + det_yaw;
				}
#ifdef MBUG_OPEN_
				MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
				adjust_yaw = adjust_yaw * PI / 180;

				// 				double s_theta = (fabs(det_yaw) - 180.0 - 10) * PI / 180;
				// 				double e_theta = (fabs(det_yaw) - 180.0 + 10) * PI / 180;
				// 				theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

				theta = adjust_yaw;// 0.3 * adjust_yaw + 0.7 * theta;
				COOR2 pt;
				pt.y = 2000;
				pt.x = (INT32)(tan(theta) * pt.y);
			}
			else
			{
				pt.x = -720;
				pt.y = 0;
			}

			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &pt);
			double out_rate = 0;
			ret = get_best_morphin_line5(pt, out_rate);
			if (ret >= 0)
				g_cross_travel_rate = out_rate;
			else
				g_cross_travel_rate = 0;
		}//[if (g_cross_cur_subgoal == 0)]
		else if (g_cross_cur_subgoal == 2)
		{//[出点规划]
#ifdef MBUG_OPEN_
			MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
			xx = temp_cur_subgoal.x;
			yy = temp_cur_subgoal.y - 400;
#ifdef MBUG_OPEN_
			MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
			dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
			MBUG("dist to the cur sub goal pt: %f\n", dist);
#endif
			if (g_cross_reach_goal_yaw == 1)
			{
				if (dist < 200 || yy < 200)//[通过子目标点]
				{
					g_cross_reach_goal = 1;
#ifdef MBUG_OPEN_
					MBUG("reach the final sub goal pt\n");
#endif
				}
			}

			double det_yaw;
			double adjust_yaw = 0;
			//[第二个目标点先满足转向角度条件]
			if (g_cross_reach_goal_yaw == 0)
			{
#ifdef MBUG_OPEN_
				MBUG("reach the goal\n");
#endif
				g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
				MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
				MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
				CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
				MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
				MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
				//[计算航向角偏移]
				det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
				CHECK_YAW(det_yaw);
#ifdef MBUG_OPEN_
				MBUG("fabs(fabs(det_yaw) - 180.0) : %lf\n", fabs(fabs(det_yaw) - 180.0));
#endif
				if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha3)
				{//[达到阈值切换状态，以目标点为主]
#ifdef MBUG_OPEN_
					MBUG("reach the yaw 1\n");
#endif
					g_cross_reach_goal_yaw = 1;
#ifdef MBUG_OPEN_
					MBUG("g_cross_reach_goal_yaw : %d\n", g_cross_reach_goal_yaw);
#endif
				}
			}

			if (g_cross_reach_goal_yaw == 0)
			{//[尚未到达目标航向角]
#ifdef MBUG_OPEN_
				MBUG("haven't reach the yaw\n");
#endif
				//[设置一个虚拟目标点]
				if (det_yaw <= 180 && det_yaw >= 0)
				{
					adjust_yaw = -(180 - det_yaw);
				}
				else
				{
					adjust_yaw = 180 + det_yaw;
				}
#ifdef MBUG_OPEN_
				MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
				adjust_yaw = adjust_yaw / 180 * PI;


				COOR2 virtual_pt;
				virtual_pt.x = (INT32)(sin(adjust_yaw) * 2000);
				virtual_pt.y = (INT32)(cos(adjust_yaw) * 2000);

				//[@@若是车头往右偏了很可能导致车从右边拐，所以强制设置一个左侧虚拟点]
				virtual_pt.x = -700;
				virtual_pt.y = 0;
#ifdef MBUG_OPEN_
				MBUG("virtual_pt (%d, %d)\n", virtual_pt.x, virtual_pt.y);
#endif
				//[路口规划]
				double out_rate = 0;
				ret = get_best_morphin_line5(virtual_pt, out_rate);
				if (ret >= 0)
					g_cross_travel_rate = out_rate;
				else
					g_cross_travel_rate = 0;

				diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
				if (fabs((double)temp_cur_subgoal.x) <= fabs((double)temp_cur_subgoal.y))
				{
					g_cross_reach_goal_yaw = 1;
				}

			}//[end if (g_cross_reach_goal_yaw == 0)]
			else
			{//[角度满足]
				if (g_cross_reach_goal == 1)
				{//[往180度方向驶去，等待发现道路]
					double theta = 0;
					double adjust_yaw = 0;
					g_cross_cur_yaw = g_yaw;
					CONVERT_YAW(g_cross_cur_yaw);
					det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
					CHECK_YAW(det_yaw);

					if (det_yaw <= 180 && det_yaw >= 0)
					{
						adjust_yaw = -(180 - det_yaw);
					}
					else
					{
						adjust_yaw = 180 + det_yaw;
					}
#ifdef MBUG_OPEN_
					MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
					adjust_yaw = adjust_yaw * PI / 180;

					// 					double s_theta = (adjust_yaw * 180 / PI - 10) * PI / 180;
					// 					double e_theta = (adjust_yaw * 180 / PI + 10) * PI / 180;
					// 					theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

					theta = adjust_yaw;// 0.3 * adjust_yaw + 0.7 * theta;
					COOR2 pt;
					pt.y = 2000;
					pt.x = (INT32)(tan(theta) * pt.y);

					double out_rate = 0;
					ret = get_best_morphin_line5(pt, out_rate);
					if (ret >= 0)
						g_cross_travel_rate = out_rate;
					else
						g_cross_travel_rate = 0;
				}
				else
				{//[目标点尚未满足]
					//[路口规划]
					double theta = 0;
					double adjust_yaw = 0;
					g_cross_cur_yaw = g_yaw;
					CONVERT_YAW(g_cross_cur_yaw);
					det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
					CHECK_YAW(det_yaw);

					if (det_yaw <= 180 && det_yaw >= 0)
					{
						adjust_yaw = -(180 - det_yaw);
					}
					else
					{
						adjust_yaw = 180 + det_yaw;
					}
#ifdef MBUG_OPEN_
					MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
					adjust_yaw = adjust_yaw * PI / 180;

					// 					double s_theta = (fabs(det_yaw) - 180.0 - 10) * PI / 180;
					// 					double e_theta = (fabs(det_yaw) - 180.0 + 10) * PI / 180;
					// 					theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

					theta = adjust_yaw;//0.3 * adjust_yaw + 0.7 * theta;
					COOR2 pt;
					pt.y = 2000;
					pt.x = (INT32)(tan(theta) * pt.y);

					double out_rate = 0;
					// 					diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
					// 					ret = get_best_morphin_line5(temp_cur_subgoal, out_rate);
					ret = get_best_morphin_line5(pt, out_rate);
					if (ret >= 0)
						g_cross_travel_rate = out_rate;
					else
						g_cross_travel_rate = 0;
				}//[else]
			}//[else]
		}//[else if (g_cross_cur_subgoal == 1)]
	}//[else if (g_cross_exit_type == 1)]

	//[以下是为了回放输出目标点]
	for (i = 0; i< g_cross_subgoal_pts_num; i++)
		diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[i], &subgoal_show[i]);

	if (ret == -1)
	{
#ifdef MBUG_OPEN_
		MBUG("morphin no way to go\n");
#endif
	}
	else
	{
		if (g_cross_avg_angle_num != CROSS_AVG_NUM)
		{
			double angle = g_morphin_angle[ret];
			g_cross_avg_angle[g_cross_avg_index] = angle;
			g_cross_avg_index++;
			if (g_cross_avg_index == CROSS_AVG_NUM)
			{
				g_cross_avg_index = 0;
			}

			g_cross_avg_angle_num++;

			double avg = 0;
			for (i = 0; i<g_cross_avg_angle_num; i++)
			{
				avg += g_cross_avg_angle[i];
			}
			avg /= g_cross_avg_angle_num;
			get_avg_mid_line(avg);
		}//[if]
		else
		{
			double angle = g_morphin_angle[ret];
			g_cross_avg_angle[g_cross_avg_index] = angle;
			g_cross_avg_index++;

			if (g_cross_avg_index == CROSS_AVG_NUM)
			{
				g_cross_avg_index = 0;
			}

			double avg = 0;
			for (i = 0; i<g_cross_avg_angle_num; i++)
			{
				avg += g_cross_avg_angle[i];
			}
			avg /= g_cross_avg_angle_num;
			get_avg_mid_line(avg);
		}//[else]

		// 		double angle = g_morphin_angle[ret];
		// 		get_avg_mid_line(angle);
		g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
		//		g_fidelity = 1;
	}

	return ret;
}

//[增加一个右侧子目标点版本]
int cross_u_turn(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data)
{
	int ret = 0;

	int i;
	POSE_INFO gps_info;

	gps_info = pl_input->state.pos;

	if (g_cross_flag == 0)
	{
		COOR2 pts[3];
		COOR2 temp_pts[3];
		pts[1] = pl_input->fu_pl_data.cross.cross_quad.coor2[0];//[取出子目标点]
		//pts[0] = pl_input->fu_pl_data.cross.entrance_point;//[从融合得到的是车体坐标系的点]
		pts[2] = pl_input->fu_pl_data.cross.exit_point;

#ifdef DEBUG_MODE
		// 		pts[0].x = 10700450;
		// 		pts[0].y = -6914289;`
		// 		pts[1].x = 10700450;
		// 		pts[1].y = -6914289;
		pts[0].x = 0;
		pts[0].y = 0;
		pts[1].x = 0;
		pts[1].y = 0;
#endif

		if (pts[1].x == 0 && pts[1].y == 0)
		{
			if (pts[2].x == 0 && pts[2].y == 0)
			{//[出口点和子目标点都没有]
				g_cross_exit_type = 3;
			}
			else
			{//[有出口点，没有子目标点]
				g_cross_exit_type = 2;
			}
		}
		else
		{
			if (pts[2].x == 0 && pts[2].y == 0)
			{//[没有出口点，有子目标点]
				g_cross_exit_type = 1;
			}
			else
			{//[出口点和子目标点都有]
				g_cross_exit_type = 0;
			}
		}
		
		//[补出第一个子目标点]
		//pts[1].y += 400;
		pts[0].x = g_uturn_sub_pt_adjust;
		pts[0].y = pts[1].y * 2 / 3;

#ifdef MBUG_OPEN_
		MBUG("pts[0] : (%d, %d)\n", pts[0].x, pts[0].y);
		MBUG("pts[1] : (%d, %d)\n", pts[1].x, pts[1].y);
		MBUG("pts[0] : (%d, %d)\n", pts[2].x, pts[2].y);
#endif
		memcpy(&temp_pts[0], &pts[0], sizeof(COOR2));
		memcpy(&temp_pts[1], &pts[1], sizeof(COOR2));

		if (g_cross_exit_type == 0 || g_cross_exit_type == 2)
		{
			if (fabs((double)pts[2].x) < 20000 && fabs((double)pts[2].y) < 20000)
				temp_pts[2] = pts[2];
			else
				diff_coor2_e2v(&gps_info, &pts[2], &temp_pts[2]);
		}
#ifdef MBUG_OPEN_
		MBUG("v pts[0] : (%d, %d)\n", temp_pts[0].x, temp_pts[0].y);
		MBUG("v pts[1] : (%d, %d)\n", temp_pts[1].x, temp_pts[1].y);
		MBUG("v pts[2] : (%d, %d)\n", temp_pts[2].x, temp_pts[2].y);
#endif
		diff_coor2_v2e(&gps_info, &temp_pts[0], &g_cross_subgoal_pts[0]);
		diff_coor2_v2e(&gps_info, &temp_pts[1], &g_cross_subgoal_pts[1]);
		diff_coor2_v2e(&gps_info, &temp_pts[2], &g_cross_subgoal_pts[2]);
		g_cross_subgoal_pts_num = 3;

		g_cross_cur_subgoal = 0;
		g_cross_reach_goal = 0;

//#ifdef DEBUG_MODE
		g_cross_start_yaw = g_yaw;
//#endif
		
		CONVERT_YAW(g_cross_start_yaw);
		g_cross_reach_goal_yaw = 0;

		for (i=0; i<CROSS_AVG_NUM; i++)
		{
			g_cross_avg_angle[i] = 90.0;
		}
		g_cross_avg_angle_num = 0;
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_cross_travel_rate = 0;
		g_cross_avg_index = 0;
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_subgoal_adjust_flag = 0;
		g_cross_finish_signal = 0;
		g_cross_flag = 1;
		g_finish_cross = 0;

		g_uturn_stop_once = g_uturn_stop_once_counter;
		memset(&g_cross_smooth_start_pt, 0, sizeof(COOR2));
		memset(&g_cross_smooth_end_pt, 0, sizeof(COOR2));
	}

	if (g_cross_exit_type == 0)
	{//[出点和子目标点都有]
		if (g_cross_cur_subgoal == 2)
		{//[出点]
			g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
			MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
			MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			//[计算航向角偏移]
			double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);

			if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha1)
			{
				if (g_multi_lane.ok_times[1] == 2 && g_multi_lane.ok_times[2] == 2)
				{//[道路稳定了]
					g_cross_finish_signal = 1;
				}
			}
		}

		if (g_cross_finish_signal == 1)
		{
#ifdef MBUG_OPEN_
			MBUG("g_cross_finish_signal : %d\n", g_cross_finish_signal);
#endif
			g_cross_reach_goal = 0;
			g_cross_reach_goal_yaw = 0;
			g_cross_flag = 0;
			g_subgoal_adjust_flag = 0;
			g_cross_finish_signal = 0;
			g_cross_exit_type = 0;
			//[@@发送事件，切换全局状态！！！]
			//ret = -1;
			int step_x = g_last_frame_pt.x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
			int step_y = (g_last_frame_pt.y - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 0;

			for (i=1; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1; i++)
			{
				g_mid_line[i].x = step_x * i;
				g_mid_line[i].y = step_y * i + 400;
			}
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			//g_fidelity = 1;
			ret = 1;
			pl_local_data->event = (EVENT)E_CROSS_OVER;
			memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));
#ifdef MBUG_OPEN_
			MBUG("E_CROSS_OVER has send!\n");
#endif
			g_finish_cross = 10;
			g_cross_smooth_start_pt.x = pl_input->state.pos.ins_coord.x;
			g_cross_smooth_start_pt.y = pl_input->state.pos.ins_coord.y;
			return ret;
		}//[if (g_cross_finish_signal == 1)]

		//[检测是否到达目标点]
		COOR2 temp_cur_subgoal;
		double xx, yy;
		double dist;
#ifdef MBUG_OPEN_
		MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
		//[********目标点的判断********]
		if (g_cross_cur_subgoal == 0 || g_cross_cur_subgoal == 1)
		{//[子目标点规划]
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);

			xx = temp_cur_subgoal.x;
			yy = temp_cur_subgoal.y - 400;
#ifdef MBUG_OPEN_
			MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
			dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
			MBUG("dist to the cur sub goal pt: %f\n", dist);
#endif
			if (dist < 200 || yy < 0)//[通过子目标点]
			{
				g_cross_cur_subgoal++;
#ifdef MBUG_OPEN_
				MBUG("change to the next sub goal pt \n");
				MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
				diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
				xx = temp_cur_subgoal.x;
				yy = temp_cur_subgoal.y - 400;
				dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
				MBUG("dist to the cur sub goal pt: %f\n", dist);
				MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
			}
		}//[if (g_cross_cur_subgoal == 0)]

		if (g_cross_cur_subgoal == 2)
		{
#ifdef MBUG_OPEN_
			MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
			xx = temp_cur_subgoal.x;
			yy = temp_cur_subgoal.y - 400;

#ifdef MBUG_OPEN_
			MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
			dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
			MBUG("dist to the cur sub goal pt: %f\n", dist);
#endif
			if (g_cross_reach_goal_yaw == 1)
			{
				if (dist < 200 || yy < 200)//[通过第二个目标点]
				{
					g_cross_reach_goal = 1;
#ifdef MBUG_OPEN_
					MBUG("reach the final sub goal pt\n");
#endif
				}
			}
		}//[if (g_cross_cur_subgoal == 1)]
		//[********目标点的判断********]

		if (g_cross_cur_subgoal == 0 || g_cross_cur_subgoal == 1)
		{
			//[子目标点的路口规划]
			double out_rate = 0;
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
			ret = get_best_morphin_line5(temp_cur_subgoal, out_rate);
			if (ret >= 0)
				g_cross_travel_rate = out_rate;
			else
				g_cross_travel_rate = 0;
		}
		else
		{
			double det_yaw;
			double adjust_yaw = 0;
			//[第二个目标点先满足转向角度条件]
			if (g_cross_reach_goal_yaw == 0)
			{
#ifdef MBUG_OPEN_
				MBUG("reach the goal\n");
#endif
				g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
				MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
				MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
				CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
				MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
				MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
				//[计算航向角偏移]
				det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
				CHECK_YAW(det_yaw);
#ifdef MBUG_OPEN_
				MBUG("fabs(fabs(det_yaw) - 180.0) : %lf\n", fabs(fabs(det_yaw) - 180.0));
#endif
				if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha1)
				{//[达到阈值切换状态，以目标点为主]

					g_cross_reach_goal_yaw = 1;
#ifdef MBUG_OPEN_
					MBUG("reach the yaw 1\n");
					MBUG("g_cross_reach_goal_yaw : %d\n", g_cross_reach_goal_yaw);
#endif
				}
			}

			if (g_cross_reach_goal_yaw == 0)
			{//[尚未到达目标航向角]
#ifdef MBUG_OPEN_
				MBUG("haven't reach the yaw\n");
#endif
				//[设置一个虚拟目标点]
				if (det_yaw <= 180 && det_yaw >=0)
				{
					adjust_yaw = -(180 - det_yaw);
				}
				else
				{
					adjust_yaw = 180 + det_yaw;
				}
#ifdef MBUG_OPEN_
				MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
				adjust_yaw = adjust_yaw / 180 * PI;


				COOR2 virtual_pt;
				virtual_pt.x = (INT32)(sin(adjust_yaw) * 2000);
				virtual_pt.y = (INT32)(cos(adjust_yaw) * 2000);

				//[@@若是车头往右偏了很可能导致车从右边拐，所以强制设置一个左侧虚拟点]
				virtual_pt.x = -700;
				virtual_pt.y = 0;
#ifdef MBUG_OPEN_
				MBUG("virtual_pt (%d, %d)\n", virtual_pt.x, virtual_pt.y);
#endif
				//[路口规划]
				double out_rate = 0;
				ret = get_best_morphin_line5(virtual_pt, out_rate);
				if (ret >= 0)
					g_cross_travel_rate = out_rate;
				else
					g_cross_travel_rate = 0;

			}//[end if (g_cross_reach_goal_yaw == 0)]
			else 
			{//[角度满足]
				if (g_cross_reach_goal == 1)
				{//[往180度方向驶去，等待发现道路]
					double theta = 0;
					double adjust_yaw = 0;
					g_cross_cur_yaw = g_yaw;
					CONVERT_YAW(g_cross_cur_yaw);
					det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
					CHECK_YAW(det_yaw);

					if (det_yaw <= 180 && det_yaw >=0)
					{
						adjust_yaw = -(180 - det_yaw);
					}
					else
					{
						adjust_yaw = 180 + det_yaw;
					}
#ifdef MBUG_OPEN_
					MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
					adjust_yaw = adjust_yaw * PI / 180;

// 					double s_theta = (adjust_yaw * 180 / PI - 10) * PI / 180;
// 					double e_theta = (adjust_yaw * 180 / PI + 10) * PI / 180;
// 					theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

					theta = adjust_yaw;// .3 * adjust_yaw + 0.7 * theta;
					COOR2 pt;
					pt.y = 2000;
					pt.x = (INT32)(tan(theta) * pt.y);

					double out_rate = 0;
					ret = get_best_morphin_line5(pt, out_rate);
					if (ret >= 0)
						g_cross_travel_rate = out_rate;
					else
						g_cross_travel_rate = 0;

				}
				else
				{//[目标点尚未满足]
					//[路口规划]
					double theta = 0;

					g_cross_cur_yaw = g_yaw;
					CONVERT_YAW(g_cross_cur_yaw);
					det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
					CHECK_YAW(det_yaw);

					if (det_yaw <= 180 && det_yaw >=0)
					{
						adjust_yaw = -(180 - det_yaw);
					}
					else
					{
						adjust_yaw = 180 + det_yaw;
					}
#ifdef MBUG_OPEN_
					MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
					adjust_yaw = adjust_yaw * PI / 180;

// 					double s_theta = (adjust_yaw * 180 / PI - 10) * PI / 180;
// 					double e_theta = (adjust_yaw * 180 / PI + 10) * PI / 180;
// 					theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

					theta = adjust_yaw;// 0.3 * adjust_yaw + 0.7 * theta;
					COOR2 pt;
					pt.y = 2000;
					pt.x = (INT32)(tan(theta) * pt.y);

					double out_rate = 0;
// 					diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
// 					ret = get_best_morphin_line5(temp_cur_subgoal, out_rate);
					ret = get_best_morphin_line5(pt, out_rate);
					if (ret >= 0)
						g_cross_travel_rate = out_rate;
					else
						g_cross_travel_rate = 0;
				}//[else]
			}//[else]
		}//[else]
	}//[if (g_cross_exit_type == 0)]

	else if (g_cross_exit_type == 1)
	{//[没有出口点，有子目标点]
		if (g_cross_cur_subgoal == 2)
		{//[无出口点的规划]
			g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
			MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
			MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			//[计算航向角偏移]
			double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);

			if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha2)
			{
// 				if (g_multi_lane.ok_times[1] == 2 && g_multi_lane.ok_times[2] == 2)
// 				{//[道路稳定了]
					g_cross_finish_signal = 1;
//				}

				if (g_cross_reach_goal_yaw == 0)
				{
#ifdef MBUG_OPEN_
					MBUG("reach the yaw 1\n");
#endif
					g_cross_reach_goal_yaw = 1;
#ifdef MBUG_OPEN_
					MBUG("g_cross_reach_goal_yaw : %d\n", g_cross_reach_goal_yaw);
#endif
				}
			}

			if (g_cross_finish_signal == 1)
			{
#ifdef MBUG_OPEN_
				MBUG("g_cross_finish_signal : %d\n", g_cross_finish_signal);
#endif
				g_cross_reach_goal = 0;
				g_cross_reach_goal_yaw = 0;
				g_cross_flag = 0;
				g_subgoal_adjust_flag = 0;
				g_cross_finish_signal = 0;
				g_cross_exit_type = 0;
				//[@@发送事件，切换全局状态！！！]
				//ret = -1;
				int step_x = g_last_frame_pt.x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
				int step_y = (g_last_frame_pt.y - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
				g_mid_line[0].x = 0;
				g_mid_line[0].y = 0;

				for (i=1; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1; i++)
				{
					g_mid_line[i].x = step_x * i;
					g_mid_line[i].y = step_y * i + 400;
				}
				g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
//				g_fidelity = 1;
				ret = 1;
				pl_local_data->event = (EVENT)E_CROSS_OVER;
				memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));
#ifdef MBUG_OPEN_
				MBUG("E_CROSS_OVER has send!\n");
#endif
				g_finish_cross = 10;
				g_cross_smooth_start_pt.x = pl_input->state.pos.ins_coord.x;
				g_cross_smooth_start_pt.y = pl_input->state.pos.ins_coord.y;
				return ret;
			}

			if (g_cross_reach_goal_yaw == 0)
			{
				COOR2 pt;
				if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha2)
				{//[角度满足]
					double theta = 0;
					double adjust_yaw = 0;
					g_cross_cur_yaw = g_yaw;
					CONVERT_YAW(g_cross_cur_yaw);
					det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
					CHECK_YAW(det_yaw);

					if (det_yaw <= 180 && det_yaw >=0)
					{
						adjust_yaw = -(180 - det_yaw);
					}
					else
					{
						adjust_yaw = 180 + det_yaw;
					}
#ifdef MBUG_OPEN_
					MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
					adjust_yaw = adjust_yaw * PI / 180;

// 					double s_theta = (fabs(det_yaw) - 180.0 - 10) * PI / 180;
// 					double e_theta = (fabs(det_yaw) - 180.0 + 10) * PI / 180;
// 					theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

					theta = adjust_yaw;// 0.3 * adjust_yaw + 0.7 * theta;
					COOR2 pt;
					pt.y = 2000;
					pt.x = (INT32)(tan(theta) * pt.y);
				}
				else
				{
					pt.x = -720;
					pt.y = 0;
				}

				double out_rate = 0;
				ret = get_best_morphin_line5(pt, out_rate);
				if (ret >= 0)
					g_cross_travel_rate = out_rate;
				else
					g_cross_travel_rate = 0;
			}
			else
			{
				/*
				double adjust_yaw = 0;
				g_cross_cur_yaw = g_yaw;
				MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
				MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
				CONVERT_YAW(g_cross_cur_yaw);
				MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
				MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);

				
				//[计算航向角偏移]
				det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
				CHECK_YAW(det_yaw);

				//[设置一个虚拟目标点]
				if (det_yaw <= 180 && det_yaw >=0)
				{
					adjust_yaw = -(180 - det_yaw);
				}
				else
				{
					adjust_yaw = 180 + det_yaw;
				}
				MBUG("adjust_yaw :%lf\n", adjust_yaw);
				adjust_yaw = adjust_yaw / 180 * PI;
				*/

				double theta = 0;
				double adjust_yaw = 0;
				g_cross_cur_yaw = g_yaw;
				CONVERT_YAW(g_cross_cur_yaw);
				det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
				CHECK_YAW(det_yaw);

				if (det_yaw <= 180 && det_yaw >=0)
				{
					adjust_yaw = -(180 - det_yaw);
				}
				else
				{
					adjust_yaw = 180 + det_yaw;
				}
#ifdef MBUG_OPEN_
				MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
				adjust_yaw = adjust_yaw * PI / 180;

// 				double s_theta = (adjust_yaw * 180 / PI - 10) * PI / 180;
// 				double e_theta = (adjust_yaw * 180 / PI + 10) * PI / 180;
// 				theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

				theta = adjust_yaw;//0.3 * adjust_yaw + 0.7 * theta;
				COOR2 pt;
				pt.y = 2000;
				pt.x = (INT32)(tan(theta) * pt.y);


// 				COOR2 virtual_pt;
// 				virtual_pt.x = (INT32)(sin(adjust_yaw) * 2000);
// 				virtual_pt.y = (INT32)(cos(adjust_yaw) * 2000);

				double out_rate = 0;
				ret = get_best_morphin_line5(pt, out_rate);
				if (ret >= 0)
					g_cross_travel_rate = out_rate;
				else
					g_cross_travel_rate = 0;

			}
			
		}//[if (g_cross_cur_subgoal == 1)]
		else
		{//[子目标的规划]
			//[检测是否到达子目标点，取出下一个子目标点]
			COOR2 temp_cur_subgoal;
			double xx, yy;
			double dist;
#ifdef MBUG_OPEN_
			MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);

			xx = temp_cur_subgoal.x;
			yy = temp_cur_subgoal.y - 400;
#ifdef MBUG_OPEN_
			MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
			dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
			MBUG("dist to the cur sub goal pt: %f\n", dist);
#endif
			if (dist < 200 || yy < 0)//[通过第一个目标点]
			{
				g_cross_cur_subgoal++;
#ifdef MBUG_OPEN_
				MBUG("change to the next sub goal pt \n");
				MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
				diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
				xx = temp_cur_subgoal.x;
				yy = temp_cur_subgoal.y - 400;
				dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
				MBUG("dist to the cur sub goal pt: %f\n", dist);
				MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
			}

			if (g_cross_cur_subgoal == 0 || g_cross_cur_subgoal == 1)
			{
				double out_rate = 0;
				diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
				ret = get_best_morphin_line5(temp_cur_subgoal, out_rate);
				if (ret >= 0)
					g_cross_travel_rate = out_rate;
				else
					g_cross_travel_rate = 0;
			}
			else
			{
				g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
				MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
				MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
				CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
				MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
				MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
				//[计算航向角偏移]
				double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
				CHECK_YAW(det_yaw);

				COOR2 pt;
				if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha2)
				{
					pt.x = -100;
					pt.y = 800;
				}
				else
				{
					pt.x = -720;
					pt.y = 0;
				}

				double out_rate = 0;
				ret = get_best_morphin_line5(pt, out_rate);
				if (ret >= 0)
					g_cross_travel_rate = out_rate;
				else
					g_cross_travel_rate = 0;
			}//[else]
		}//[else]
	}//[else if (g_cross_exit_type == 1)]

	else if (g_cross_exit_type == 2)
	{//[有出口点，没有目标点]
		if (g_cross_cur_subgoal == 2)
		{//[出点规划]
			g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
			MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
			MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			//[计算航向角偏移]
			double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);

			if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha3)
			{
				//get_road_direction(pl_input, 2, fabs(det_yaw) - 180.0);
// 				if (g_multi_lane.ok_times[1] == 2 && g_multi_lane.ok_times[2] == 2)
// 				{//[道路稳定了]
					g_cross_finish_signal = 1;
//				}
			}
		}

		if (g_cross_finish_signal == 1)
		{
#ifdef MBUG_OPEN_
			MBUG("g_cross_finish_signal : %d\n", g_cross_finish_signal);
#endif
			g_cross_reach_goal = 0;
			g_cross_reach_goal_yaw = 0;
			g_cross_flag = 0;
			g_subgoal_adjust_flag = 0;
			g_cross_finish_signal = 0;
			g_cross_exit_type = 0;
			//[@@发送事件，切换全局状态！！！]
			//ret = -1;
			int step_x = g_last_frame_pt.x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
			int step_y = (g_last_frame_pt.y - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 0;

			for (i=1; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1; i++)
			{
				g_mid_line[i].x = step_x * i;
				g_mid_line[i].y = step_y * i + 400;
			}
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
//			g_fidelity = 1;
			ret = 1;
			pl_local_data->event = (EVENT)E_CROSS_OVER;
			memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));
#ifdef MBUG_OPEN_
			MBUG("E_CROSS_OVER has send!\n");
#endif
			g_finish_cross = 10;
			g_cross_smooth_start_pt.x = pl_input->state.pos.ins_coord.x;
			g_cross_smooth_start_pt.y = pl_input->state.pos.ins_coord.y;
			return ret;
		}//[if (g_cross_finish_signal == 1)]

		//[检测是否到达目标点]
		COOR2 temp_cur_subgoal;
		double xx, yy;
		double dist;
#ifdef MBUG_OPEN_
		MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
		//[********目标点的判断********]
		if (g_cross_cur_subgoal == 0 || g_cross_cur_subgoal == 1)
		{//[没有子目标点规划]
			g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
			MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
			MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
			MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
			//[计算航向角偏移]
			double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);

			if (fabs(fabs(det_yaw) - 180.0) > 60.0)
				g_cross_cur_subgoal++;

			COOR2 pt;
			if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha2)
			{
				double theta = 0;
				double adjust_yaw = 0;
				g_cross_cur_yaw = g_yaw;
				CONVERT_YAW(g_cross_cur_yaw);
				det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
				CHECK_YAW(det_yaw);

				if (det_yaw <= 180 && det_yaw >=0)
				{
					adjust_yaw = -(180 - det_yaw);
				}
				else
				{
					adjust_yaw = 180 + det_yaw;
				}
#ifdef MBUG_OPEN_
				MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
				adjust_yaw = adjust_yaw * PI / 180;

// 				double s_theta = (fabs(det_yaw) - 180.0 - 10) * PI / 180;
// 				double e_theta = (fabs(det_yaw) - 180.0 + 10) * PI / 180;
// 				theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

				theta = adjust_yaw;// 0.3 * adjust_yaw + 0.7 * theta;
				COOR2 pt;
				pt.y = 2000;
				pt.x = (INT32)(tan(theta) * pt.y);
			}
			else
			{
				pt.x = -720;
				pt.y = 0;
			}

			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &pt);
			double out_rate = 0;
			ret = get_best_morphin_line6(pt, out_rate);
			if (ret >= 0)
				g_cross_travel_rate = out_rate;
			else
				g_cross_travel_rate = 0;
		}//[if (g_cross_cur_subgoal == 0)]
		else if (g_cross_cur_subgoal == 2)
		{//[出点规划]
#ifdef MBUG_OPEN_
			MBUG("cur sub goal is %d\n", g_cross_cur_subgoal);
#endif
			diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
			xx = temp_cur_subgoal.x;
			yy = temp_cur_subgoal.y - 400;
#ifdef MBUG_OPEN_
			MBUG("(%d, %d) \n", temp_cur_subgoal.x, temp_cur_subgoal.y);
#endif
			dist = sqrt(xx * xx + yy * yy);
#ifdef MBUG_OPEN_
			MBUG("dist to the cur sub goal pt: %f\n", dist);
#endif
			if (g_cross_reach_goal_yaw == 1)
			{
				if (dist < 200 || yy < 200)//[通过子目标点]
				{
					g_cross_reach_goal = 1;
#ifdef MBUG_OPEN_
					MBUG("reach the final sub goal pt\n");
#endif
				}
			}

			double det_yaw;
			double adjust_yaw = 0;
			//[第二个目标点先满足转向角度条件]
			if (g_cross_reach_goal_yaw == 0)
			{
#ifdef MBUG_OPEN_
				MBUG("reach the goal\n");
#endif
				g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
				MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
				MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
				CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
				MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
				MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
				//[计算航向角偏移]
				det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
				CHECK_YAW(det_yaw);
#ifdef MBUG_OPEN_
				MBUG("fabs(fabs(det_yaw) - 180.0) : %lf\n", fabs(fabs(det_yaw) - 180.0));
#endif
				if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha3)
				{//[达到阈值切换状态，以目标点为主]
#ifdef MBUG_OPEN_
					MBUG("reach the yaw 1\n");
#endif
					g_cross_reach_goal_yaw = 1;
#ifdef MBUG_OPEN_
					MBUG("g_cross_reach_goal_yaw : %d\n", g_cross_reach_goal_yaw);
#endif
				}
			}

			if (g_cross_reach_goal_yaw == 0)
			{//[尚未到达目标航向角]
#ifdef MBUG_OPEN_
				MBUG("haven't reach the yaw\n");
#endif
				//[设置一个虚拟目标点]
				if (det_yaw <= 180 && det_yaw >=0)
				{
					adjust_yaw = -(180 - det_yaw);
				}
				else
				{
					adjust_yaw = 180 + det_yaw;
				}
#ifdef MBUG_OPEN_
				MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
				adjust_yaw = adjust_yaw / 180 * PI;


				COOR2 virtual_pt;
				virtual_pt.x = (INT32)(sin(adjust_yaw) * 2000);
				virtual_pt.y = (INT32)(cos(adjust_yaw) * 2000);

				//[@@若是车头往右偏了很可能导致车从右边拐，所以强制设置一个左侧虚拟点]
				virtual_pt.x = -700;
				virtual_pt.y = 0;
#ifdef MBUG_OPEN_
				MBUG("virtual_pt (%d, %d)\n", virtual_pt.x, virtual_pt.y);
#endif
				//[路口规划]
				double out_rate = 0;
				ret = get_best_morphin_line5(virtual_pt, out_rate);
				if (ret >= 0)
					g_cross_travel_rate = out_rate;
				else
					g_cross_travel_rate = 0;

				diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
				if (fabs((double)temp_cur_subgoal.x) <= fabs((double)temp_cur_subgoal.y))
				{
					g_cross_reach_goal_yaw = 1;
				}

			}//[end if (g_cross_reach_goal_yaw == 0)]
			else 
			{//[角度满足]
				if (g_cross_reach_goal == 1)
				{//[往180度方向驶去，等待发现道路]
					double theta = 0;
					double adjust_yaw = 0;
					g_cross_cur_yaw = g_yaw;
					CONVERT_YAW(g_cross_cur_yaw);
					det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
					CHECK_YAW(det_yaw);

					if (det_yaw <= 180 && det_yaw >=0)
					{
						adjust_yaw = -(180 - det_yaw);
					}
					else
					{
						adjust_yaw = 180 + det_yaw;
					}
#ifdef MBUG_OPEN_
					MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
					adjust_yaw = adjust_yaw * PI / 180;

// 					double s_theta = (adjust_yaw * 180 / PI - 10) * PI / 180;
// 					double e_theta = (adjust_yaw * 180 / PI + 10) * PI / 180;
// 					theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

					theta = adjust_yaw;// 0.3 * adjust_yaw + 0.7 * theta;
					COOR2 pt;
					pt.y = 2000;
					pt.x = (INT32)(tan(theta) * pt.y);

					double out_rate = 0;
					ret = get_best_morphin_line5(pt, out_rate);
					if (ret >= 0)
						g_cross_travel_rate = out_rate;
					else
						g_cross_travel_rate = 0;
				}
				else
				{//[目标点尚未满足]
					//[路口规划]
					double theta = 0;
					double adjust_yaw = 0;
					g_cross_cur_yaw = g_yaw;
					CONVERT_YAW(g_cross_cur_yaw);
					det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
					CHECK_YAW(det_yaw);

					if (det_yaw <= 180 && det_yaw >=0)
					{
						adjust_yaw = -(180 - det_yaw);
					}
					else
					{
						adjust_yaw = 180 + det_yaw;
					}
#ifdef MBUG_OPEN_
					MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
					adjust_yaw = adjust_yaw * PI / 180;

// 					double s_theta = (fabs(det_yaw) - 180.0 - 10) * PI / 180;
// 					double e_theta = (fabs(det_yaw) - 180.0 + 10) * PI / 180;
// 					theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

					theta = adjust_yaw;//0.3 * adjust_yaw + 0.7 * theta;
					COOR2 pt;
					pt.y = 2000;
					pt.x = (INT32)(tan(theta) * pt.y);

					double out_rate = 0;
// 					diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[g_cross_cur_subgoal], &temp_cur_subgoal);
// 					ret = get_best_morphin_line5(temp_cur_subgoal, out_rate);
					ret = get_best_morphin_line5(pt, out_rate);
					if (ret >= 0)
						g_cross_travel_rate = out_rate;
					else
						g_cross_travel_rate = 0;
				}//[else]
			}//[else]
		}//[else if (g_cross_cur_subgoal == 1)]
	}//[else if (g_cross_exit_type == 2)]

	else if (g_cross_exit_type == 3)
	{//[没有出口点和子目标点]
		g_cross_cur_yaw = g_yaw;
#ifdef MBUG_OPEN_
		MBUG("g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
		CONVERT_YAW(g_cross_cur_yaw);
#ifdef MBUG_OPEN_
		MBUG("CONVERT_YAW g_cross_start_yaw : %d\n", g_cross_start_yaw);
		MBUG("CONVERT_YAW g_cross_cur_yaw : %d\n", g_cross_cur_yaw);
#endif
		//[计算航向角偏移]
		double det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
		CHECK_YAW(det_yaw);

		if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha4)
		{
// 			if (g_multi_lane.ok_times[1] == 2 && g_multi_lane.ok_times[2] == 2)
// 			{//[道路稳定了]
				g_cross_finish_signal = 1;
	//		}

			if (g_cross_reach_goal_yaw == 0)
			{
#ifdef MBUG_OPEN_
				MBUG("reach the yaw 1\n");
#endif
				g_cross_reach_goal_yaw = 1;
#ifdef MBUG_OPEN_
				MBUG("g_cross_reach_goal_yaw : %d\n", g_cross_reach_goal_yaw);
#endif
			}
		}

		if (g_cross_finish_signal == 1)
		{
#ifdef MBUG_OPEN_
			MBUG("g_cross_finish_signal : %d\n", g_cross_finish_signal);
#endif
			g_cross_reach_goal = 0;
			g_cross_reach_goal_yaw = 0;
			g_cross_flag = 0;
			g_subgoal_adjust_flag = 0;
			g_cross_finish_signal = 0;
			g_cross_exit_type = 0;
			//[@@发送事件，切换全局状态！！！]
			//ret = -1;
			int step_x = g_last_frame_pt.x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
			int step_y = (g_last_frame_pt.y - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 0;

			for (i=1; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1; i++)
			{
				g_mid_line[i].x = step_x * i;
				g_mid_line[i].y = step_y * i + 400;
			}
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
//			g_fidelity = 1;
			ret = 1;
			pl_local_data->event = (EVENT)E_CROSS_OVER;
			memset(subgoal_show, 0, SUB_GOAL_PT_NUM * sizeof(COOR2));
#ifdef MBUG_OPEN_
			MBUG("E_CROSS_OVER has send!\n");
#endif
			g_finish_cross = 10;
			g_cross_smooth_start_pt.x = pl_input->state.pos.ins_coord.x;
			g_cross_smooth_start_pt.y = pl_input->state.pos.ins_coord.y;
			return ret;
		}

		if (g_cross_reach_goal_yaw == 0)
		{
			COOR2 pt;
			if (fabs(fabs(det_yaw) - 180.0) < g_uturn_alpha2)
			{
				double theta = 0;
				double adjust_yaw = 0;
				g_cross_cur_yaw = g_yaw;
				CONVERT_YAW(g_cross_cur_yaw);
				det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
				CHECK_YAW(det_yaw);

				if (det_yaw <= 180 && det_yaw >=0)
				{
					adjust_yaw = -(180 - det_yaw);
				}
				else
				{
					adjust_yaw = 180 + det_yaw;
				}
#ifdef MBUG_OPEN_
				MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
				adjust_yaw = adjust_yaw * PI / 180;

// 				double s_theta = (adjust_yaw * 180 / PI - 10) * PI / 180;
// 				double e_theta = (adjust_yaw * 180 / PI + 10) * PI / 180;
// 				theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

				theta = adjust_yaw;// 0.3 * adjust_yaw + 0.7 * theta;
				COOR2 pt;
				pt.y = 2000;
				pt.x = (INT32)(tan(theta) * pt.y);
			}
			else
			{
				pt.x = -720;
				pt.y = 0;

				double out_rate = 0;
				ret = get_best_morphin_line5(pt, out_rate);
				if (ret >= 0)
					g_cross_travel_rate = out_rate;
				else
					g_cross_travel_rate = 0;
			}
		}
		else
		{
			double theta = 0;
			double adjust_yaw = 0;
			g_cross_cur_yaw = g_yaw;
			CONVERT_YAW(g_cross_cur_yaw);
			det_yaw = (double)(g_cross_cur_yaw - g_cross_start_yaw);
			CHECK_YAW(det_yaw);

			if (det_yaw <= 180 && det_yaw >=0)
			{
				adjust_yaw = -(180 - det_yaw);
			}
			else
			{
				adjust_yaw = 180 + det_yaw;
			}
#ifdef MBUG_OPEN_
			MBUG("adjust_yaw :%lf\n", adjust_yaw);
#endif
			adjust_yaw = adjust_yaw * PI / 180;

// 			double s_theta = (fabs(det_yaw) - 180.0 - 10) * PI / 180;
// 			double e_theta = (fabs(det_yaw) - 180.0 + 10) * PI / 180;
// 			theta = get_angle(s_theta, e_theta, &pl_input->fu_pl_data.gridmap);

			theta = adjust_yaw;//0.3 * adjust_yaw + 0.7 * theta;
			COOR2 pt;
			pt.y = 2000;
			pt.x = (INT32)(tan(theta) * pt.y);

			double out_rate = 0;
			ret = get_best_morphin_line5(pt, out_rate);
			if (ret >= 0)
				g_cross_travel_rate = out_rate;
			else
				g_cross_travel_rate = 0;

		}
	}//[else if (g_cross_exit_type == 3)]


	//[以下是为了回放输出目标点]
	for (i = 0; i< g_cross_subgoal_pts_num; i++)
		diff_coor2_e2v(&gps_info, &g_cross_subgoal_pts[i], &subgoal_show[i]);

	if (ret == -1)
	{
#ifdef MBUG_OPEN_
		MBUG("morphin no way to go\n");
#endif
	}
	else
	{
		if (g_cross_avg_angle_num != CROSS_AVG_NUM)
		{
			double angle = g_morphin_angle[ret];
			g_cross_avg_angle[g_cross_avg_index] = angle;
			g_cross_avg_index++;
			if (g_cross_avg_index == CROSS_AVG_NUM)
			{
				g_cross_avg_index = 0;
			}

			g_cross_avg_angle_num++;

			double avg = 0;
			for (i=0; i<g_cross_avg_angle_num; i++)
			{
				avg += g_cross_avg_angle[i];
			}
			avg /= g_cross_avg_angle_num;
			get_avg_mid_line(avg);
		}//[if]
		else
		{
			double angle = g_morphin_angle[ret];
			g_cross_avg_angle[g_cross_avg_index] = angle;
			g_cross_avg_index++;

			if (g_cross_avg_index == CROSS_AVG_NUM)
			{
				g_cross_avg_index = 0;
			}

			double avg = 0;
			for (i=0; i<g_cross_avg_angle_num; i++)
			{
				avg += g_cross_avg_angle[i];
			}
			avg /= g_cross_avg_angle_num;
			get_avg_mid_line(avg);
		}//[else]

		// 		double angle = g_morphin_angle[ret];
		// 		get_avg_mid_line(angle);
		g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
//		g_fidelity = 1;
	}

	return ret;
}

/*==================================================================
 * 函数名  ：	int cross_intersection(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data)
 * 功能    ：	交叉路口执行
 * 输入参数：	PL_FUNC_INPUT *pl_input			环境数据输入
				PL_CS_DATA *pl_cs_data			规划输出
				PL_LOCAL_DATA *pl_local_data	规划触发事件
 * 输出参数：	
 * 返回值  ：	int  0   有可执行路径  -1   无可执行路径
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int cross_intersection(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data)
{
	int ret = 0;

	switch (g_stat)
	{
	case S_LEFT:
		ret = cross_left_turn(pl_input, pl_cs_data, pl_local_data);
		break;
	case S_RIGHT:
		ret = cross_right_turn(pl_input, pl_cs_data, pl_local_data);
		break;
	case S_STRAIGHT:
		ret = cross_straight(pl_input, pl_cs_data, pl_local_data);
		break;
	case S_UTURN:
		ret = cross_u_turn_new(pl_input, pl_cs_data, pl_local_data);
		//ret = cross_u_turn(pl_input, pl_cs_data, pl_local_data);
		break;
	}

	return ret;
}