#ifndef CROSSING_H_
#define CROSSING_H_

#include "../robix4/protocols/app_pl.h"
#include "./basicfunction.h"

#define SUB_GOAL_PT_NUM 5							//[��Ŀ�����Ŀ �궨��]

extern int g_cross_road_type;
extern int g_cross_und_trace_cur_lane_counter;
extern COOR2 g_cross_smooth_start_pt;
extern COOR2 g_cross_smooth_end_pt;
extern int g_finish_cross;//[������ʾ�ս���·�ڣ����ٻ���]
extern int g_cross_flag;						//[��ʼ����־]
// int g_cross_exit_type = 0;					//[·�ڳ��ڵ����ͣ�0  �г��ڵ�  1  �й�����ڵ�  2  û�г��ڵ�]
// int g_cross_finish_signal = 0;				//[�����ںϵ�·�ȶ�����������·��״̬]
extern int g_subgoal_adjust_flag;				//[����Ŀ��������һ��ʱ����������]
extern int g_cross_start_yaw;					//[��ʼ�����]
extern int g_cross_cur_yaw;						//[��ǰ�����]
extern int g_cross_reach_goal_yaw;				//[���������Լ���ı�־]
// int g_cross_left_yaw_t = 0;
// int g_cross_right_yaw_t = 0;
extern COOR2 g_cross_subgoal_pts[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];		//[�������Ŀ���]
extern int g_cross_subgoal_pts_num;				//[��Ŀ�����Ŀ]
extern int g_cross_cur_subgoal;					//[��ǰ��Ŀ�������]
extern int g_cross_reach_goal;					//[��������Ŀ����־]
extern int g_cross_travel_dist;					//[ǰ����ͨ�о��룬���������ٶȣ���û��ЧĿǰû���Թ���]
extern int g_cross_quit_dist;					//[�����˳�����]
extern COOR2 g_cross_quit_start_pt;				//[�����˳���ʼ��]
extern int g_cross_quit_dist_sum;				//[�����˳��ۼƾ���]
// double g_cross_travel_rate = 0;


//[·��ִ��ʱ���л��ĽǶ���ֵ]
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