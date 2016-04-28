#ifndef CROSSING_H_
#define CROSSING_H_

#include "../robix4/protocols/app_pl.h"
#include "./basicfunction.h"

#define SUB_GOAL_PT_NUM 5							//[子目标点数目 宏定义]

extern int g_cross_road_type;
extern int g_cross_und_trace_cur_lane_counter;
extern COOR2 g_cross_smooth_start_pt;
extern COOR2 g_cross_smooth_end_pt;
extern int g_finish_cross;//[用来表示刚结束路口，减速缓行]
extern int g_cross_flag;						//[初始化标志]
// int g_cross_exit_type = 0;					//[路口出口点类型，0  有出口点  1  有估算出口点  2  没有出口点]
// int g_cross_finish_signal = 0;				//[根据融合道路稳定，给出结束路口状态]
extern int g_subgoal_adjust_flag;				//[当子目标点是最后一个时，进行修正]
extern int g_cross_start_yaw;					//[初始航向角]
extern int g_cross_cur_yaw;						//[当前航向角]
extern int g_cross_reach_goal_yaw;				//[航向角满足约束的标志]
// int g_cross_left_yaw_t = 0;
// int g_cross_right_yaw_t = 0;
extern COOR2 g_cross_subgoal_pts[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];		//[交叉口子目标点]
extern int g_cross_subgoal_pts_num;				//[子目标点数目]
extern int g_cross_cur_subgoal;					//[当前子目标点索引]
extern int g_cross_reach_goal;					//[到达最终目标点标志]
extern int g_cross_travel_dist;					//[前方可通行距离，用来控制速度（有没有效目前没测试过）]
extern int g_cross_quit_dist;					//[超距退出距离]
extern COOR2 g_cross_quit_start_pt;				//[超距退出起始点]
extern int g_cross_quit_dist_sum;				//[超距退出累计距离]
// double g_cross_travel_rate = 0;


//[路口执行时，切换的角度阈值]
extern int g_straight_alpha;
extern int g_left_turn_alpha;
extern int g_right_turn_alpha;

extern int g_left_turn_pt_adjust;
extern int g_left_turn_pt2x_adjust;
extern int g_left_turn_pt2y_adjust;

extern int g_right_turn_pt_adjust;
extern int g_right_turn_pt2x_adjust;
extern int g_right_turn_pt2y_adjust;

extern int g_uturn_alpha1;
extern int g_uturn_alpha2;
extern int g_uturn_alpha3;
extern int g_uturn_alpha4;


extern int g_cross_straight_speed;
extern int g_cross_left_turn_speed;
extern int g_cross_right_turn_speed;
extern int g_cross_uturn_speed;
extern COOR2 g_cross_slow_pt;
extern int g_cross_und_slow_down_dist;
extern int g_cross_und_slow_down_speed;
extern int g_cross_und_slow_down_flag;
extern int g_uturn_sub_pt_adjust;
extern int g_cross_adjust_count;
extern int g_cross_adjust_flag;
extern int g_uturn_stop_once;
extern int g_uturn_stop_once_counter;

extern COOR2 g_last_frame_pt;

void reset_cross_flag();
int cross_intersection(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data);

#endif /*CROSSING_H_*/