/**
* @file localplan.cpp
* @author 诸葛程晨 zgccmax@163.com
* @date 2011年
* @brief 局部路径规划
* @version v1.0
*/


// #include <cstdio>
// #include <cstdlib>
#include <cstring>
#include <cmath>
//#include <ctime>
#include <vector>
// #include <map>
// #include <deque>
// #include <algorithm>
// #include <fstream>
// 
// #include <iostream>
// #include <sstream>

using namespace std;

#include "./Cmorphin.h"
#include "./basicfunction.h"
#include "./trace_road.h"
#include "./crossing.h"
#include "./reverse.h"
#include "./TurnRound.h"

#define TOYOTA	0
#define BYD		1
#define ALV3	3
static int g_car_identity = 0;//[0 TOYOTA  1 BYD]

//COOR2 g_last_frame_pt;//[补丁参数，结构化道路由于分割不好，导致两车道合并，有时规划偏右，下一帧左换道，看上去跳变了]

//[超车相关  take_over.h]
int take_over(int cur_spd, int gridmap[][GRID_WIDTH], DYN_OBS* dyn_obs, int dyn_obs_num, MULTI_LANE* multi_lane, int mode, COOR2 *plan_line, int *status );

//[急弯降速]
int g_sharp_turn_flag;				//[急弯降速标志]
COOR2 g_sharp_turn_last_pt;			//[上一个降速GPS点]
COOR2 g_sharp_turn_slow_down_s_pt;	//[降速起始GPS点]
COOR2 g_sharp_turn_slow_down_e_pt;	//[降速结束GPS点]
int g_sharp_turn_in_dist;			//[进入急弯降速状态的预警距离]
int g_sharp_turn_out_dist;			//[退出急弯降速状态的缓冲距离]
int g_sharp_turn_speed;				//[急弯降速设置的速度]

//[2013年常熟环湖道路，视觉检测弯道]
static int g_one_curve_level_speed = 0;
static int g_two_curve_level_speed = 0;
static int g_three_curve_level_speed = 0;
static int g_four_curve_level_speed = 0;

//[弯道等级计数器]
static int g_one_curve_leve_counter = 0;
static int g_two_curve_level_counter = 0;
static int g_three_curve_level_counter = 0;
static int g_four_curve_level_counter = 0;
//[当前弯道等级]
static int g_curent_curve_level = 0;
static unsigned int g_frame_id = 0;		//[规划全局帧号]
static int g_fu_frame_id = 0;			//[融合帧号]
static int odo_init_flag = 0;

static int g_fidelity;					//[当前规划可信度，目前使用1和0]
int g_stat;								//[全局状态]
static int g_last_navi_state = TRACE_LANE;	//[上一帧子任务状态]

//[全局GPS相关]
static int g_pitch;				//[俯仰角]
static int g_roll;				//[横滚角]
int g_yaw;						//[航向角]
COOR2 g_vehicle_org_pt;			//[原点，方便计算]

//[路点速度]
int g_road_type;
int g_wp_speed;			//[路点速度，现在用来作为每次速度规划的基础速度]
int g_max_speed;		//[最高速度，由规划配置文件读取，用来试验用，不用时配置文件设置为0]
int g_cur_speed;		//[当前速度]
int g_real_speed;		//[实际速度]
int g_cross_speed;

int g_speed_limit_sign = 0;				//[识别限速标志牌后限速]
int g_speed_limi_pl = 0;					//[限速值]

//[路口理解状态速度]
int g_cross_und_speed;					//[进入路口理解状态时的速度]
int g_cross_und_straight_speed;			//[路口直行速度]
int g_cross_und_left_speed;				//[路口左拐速度]
int g_cross_und_right_speed;			//[路口右拐速度]
int g_cross_und_uturn_speed;			//[路口Uturn速度]
int g_cross_und_traffic_sign_speed;		//[当初怕限速牌不能正确识别，所以设置一旦发现就限速]

//[不同速度，换道角度控制]
double g_10km_change_steer;				//[10km/h  换道预瞄角度阈值]
double g_20km_change_steer;				//[20km/h  换道预瞄角度阈值]
double g_30km_change_steer;				//[30km/h  换道预瞄角度阈值]
double g_40km_change_steer;				//[40km/h  换道预瞄角度阈值]
double g_50km_change_steer;				//[50km/h  换道预瞄角度阈值]
double g_60km_change_steer;				//[60km/h  换道预瞄角度阈值]

int g_vibration_count;	//[振荡计数器，控制振荡]
int g_vibration_flag;	//[振荡标志]
int g_slow_down_dist;	//[有狭窄障碍物减速]
int g_follow_car_speed;	//[前方跟随车辆速度]
int g_last_speed;		//[上次规划速度]
int g_unsafe_dist;		//[不可通行处的距离]
int g_unsafe_dist_for_change_lane;			//[换道时危险所处距离]
int g_far_obs_warning_flag = 0;				//[远处有障碍物的预警标志]

int g_natural_look_dist;					//[非结构化道路关注的距离]

//[用来记录路口通过时使用的子目标点，然后传递给显示窗口]
COOR2 subgoal_show[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
int subgoal_show_num;

static int g_morphin_index_test = -1;				//[用来测试规划弧线与实际方向盘转角用]
int g_morphin_search_dist = -1;						//[生成的搜索弧线长度]
int g_roving_switch = 0;						//[漫游状态开关]
int g_roving_speed =  0;						//[漫游速度]
int g_nature_road_switch = 0;				//[使用自然边界的开关]
int g_nature_road_to_obs_switch = 0;			//[自然道边投影到栅格]

//[全局任务区域处理]
static int g_global_task_pt_valid_dist = 3000;		//[全局任务点有效距离]

static int g_emergency_counter = 0;					//[陷入困境计时]
static int g_emergency_roving_timer = 0;			//[摆脱困境计时]
static int emergency_roving_timer = 0;				//[读取配置文件的参数]
static int g_cur_search_index = -1;					//[当前帧使用的索引]


//[乡村道路相关]
#define LINE_AVG_NUM 10							//[10帧平滑]
static COOR2 g_line_avg[LINE_AVG_NUM][NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
static int g_line_avg_num;
static int g_line_avg_index;						//[索引]
static int g_line_avg_last_index;

//[道边，规划线]
static COOR2 g_left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];		//[当前车道的左道边]
static int g_left_line_num = 0;
static COOR2 g_right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];		//[当前车道的右道边]
static int g_right_line_num = 0;
COOR2 g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];				//[规划线，经过重采样后，传递给控制的点]
static int g_on_the_line_flag = 0;									//[车子是否压线的标志]
int g_long_line_num;				//[从杜师兄版本沿用下来描述规划线的点个数]


int g_64lidar_switch = 1;					//[64线雷达开关]
COOR2 g_grid_center;						//[栅格图中所处中心位置]
int g_grid_dist;							//[栅格能够规划的距离]
int g_grid_map[GRID_HEIGHT][GRID_WIDTH];			//[栅格地图]
int g_closed_grid_map[GRID_HEIGHT][GRID_WIDTH];		//[闭运算后的栅格地图]
int g_hd_grid_map[GRID_HEIGHT_HD][GRID_WIDTH_HD];
COOR2 g_hd_grid_center;						//[栅格图中所处中心位置]
int g_hd_closed_grid_map[GRID_HEIGHT_HD][GRID_WIDTH_HD];

//[障碍物栅格描述]
typedef struct
{
	int x;
	int y;
	QUAD quad;
	int property;//[属性]
	int dist;
}Grid_Obs;
vector <Grid_Obs>grid_obs;
static int g_has_mid_obs = 0;					//[道路中间障碍物标志]
static int g_left_has_big_obs = 0;				//[道路左方有大障碍]
static int g_left_big_obs_dist = MAX_VALUE;		//[左侧大障碍到车的距离]
static int g_right_has_big_obs = 0;				//[道路右方有大障碍]
static int g_right_big_obs_dist = MAX_VALUE;	//[右侧大障碍到车的距离]

static int g_narrow_dist = 0;		//[狭小区域到车的距离]

DYN_OBS g_dyn_obs[MAX_DYN_AREA];	//[动态障碍物描述]
int g_dyn_obs_num;					//[动态障碍物数目]
DYN_OBS g_lateral_dyn_obs[MAX_DYN_AREA];
int g_lateral_dyn_obs_num;


int g_safe_dist_to_follow_car = 0;	//[跟随前方车辆的安全距离阈值]
int g_car_ahead_dist = 0;			//[前方车辆的距离]
int g_take_over_flag = 0;			//[是否进行超车的标志]
static int g_dyn_obs_switch = 0;	//[是否考虑动态障碍物的标志]

//[超车参数]
static int g_stop_dist = 0;					//[紧急停车阈值]
static int g_10km_dist = 0;					//[10km/h速度安全距离]
static int g_20km_dist = 0;					//[20km/h速度安全距离]
static int g_30km_dist = 0;					//[30km/h速度安全距离]
static int g_40km_dist = 0;					//[40km/h速度安全距离]
static int g_take_over_threshold = 0;		//[所超目标车速度阈值]
static int g_take_over_speed = 0;			//[超车时所用速度]
static int g_change_lane_speed = 0;			//[换道时使用速度]


//[杜师兄版本使用，大部分不再使用]
static int car_len;			//[车长]
static int car_wid;			//[车宽]
static int mini_pass;		//[最小安全冗余]

//[鄂尔多斯比赛相关]
int g_is_stopping;//[停止线的时候用来在远距离控制道路跟踪策略]

//[内部状态]
NAVI_STATE g_navi_state = TRACE_LANE;					//[内部子任务]
CHANGE_LANE_STATE g_change_lane_state = NORMAL_CHANGE;	//[换道子任务]
static int g_change_lane_count = 0;						//[确认换道成功的计数器]
static int g_change_fade_timer = 0;						//[换道时用来维持规划贴住道边，使得路线平滑的计时器]
static int g_change_mode = -1;							//[0 左换道， 1 右换道]
static double g_change_steer_limit = 0;					//[换道时不同速度下，限制换道规划线的角度]
static int g_take_over_speed_up_time = 0;				//[获取加速时间的参数值]
static int g_take_over_speed_up_timer = 0;				//[加速计时器，现在用作超车结束后稳定速度的标志]
static int g_take_over_back_lane_flag = -1;				//[超车回道标志  -1  正常巡航  0  左回道  1  右回道]
static int g_yellow_line_flag = 0;						//[0  左右都可以换道  1  左车道线是黄线  2  黄线在右侧]
static int g_change_lane_direct_keep = -1;				//[0 保持左换道  1 保持右换道]
static int g_change_lane_search_dist = MAX_VALUE;		//[换道时候搜索距离]
static int g_change_lane_search_flag = 0;				//[换道搜索标志]

MULTI_LANE g_multi_lane;								//[维持的多车道信息]

//[用来表示换道时，跟踪的当前车道的实际意义]
//[0 当前为起始车道 -1 当前为左车道 1 当前为右车道]
static int g_change_lane_see = 0;
static int g_change_lane_unsafe_dist = MAX_VALUE;

//[自然道边]
NATURAL_BOUNDARY g_natural_boundary;
int g_natural_road_search_dist = MAX_VALUE;


CROSS_UND_STATE g_cross_und_state;						//[路口理解的子状态]

//[车辆启动没有反应的定时器]
static int g_speed_up_timer = 0;

//[越野]
static COOR2 g_cross_country_cur_pt;
static COOR2 g_cross_country_cur_ins_pt;
static int g_cross_country_reach_dist_threshold;
COOR2 g_local_pt;

//[道路理解的距离]
static int g_fusion_structure_road_length = 0;
//[@@2013常熟城市比赛弯道检测]
static int g_fusion_road_length = 0;

//[倒车数据显示]
CReverse reversePlan;				//[倒车规划]
deque<COOR2> g_reverse_pts_show;
static int g_reverse_start_timer_value;
static int g_reverse_start_timer;


//[侧方位停车]
#define LATERAL_PRAKING_NULL -1
#define LATERAL_PARKING_PULL_INTO_PARKING 0
#define LATERAL_PARKING_PARKING	1
#define LATERAL_PARKING_WAITING 2
#define LATERAL_PARKING_PULL_OUT_PARKING 3
#define LATERAL_PARKING_OVER 4
static int g_lateral_parking_state = 0;
//static int g_lateral_parking_speed = 0;
static int g_lateral_parking_init_flag = 0;
static int g_lateral_parking_start_yaw = 0;
static int g_lateral_parking_waiting_flag = 0;
static int g_lateral_parking_waiting_timer = 0;
static int g_lateral_parking_geo_switch = 0;
static int g_lateral_parking_waiting_timer_value = 0;
static COOR2 g_lateral_parking_entrance_pt;
static COOR2 g_lateral_parking_parking_pt;
static COOR2 g_lateral_parking_exit_pt;

//[跟踪全局任务点]
static int g_trace_global_task_pt_flag = 0;
static int g_trace_global_task_pt_valid_dist = 0;

//[急转弯标记]
static int g_fast_turn_flag = 0;
//[速度异常检测]
static int g_spd_error_detect_switch = 1;

//[S弯相关]
static int g_s_obs_detect_switch = 0;
int check_s_obs(int gridmap[][GRID_WIDTH], COOR2* plan_line, int* line_num);
int hd_check_s_obs(int gridmap[][GRID_WIDTH_HD], COOR2* plan_line, int* line_num);
double g_s_obs_avg_angle[S_OBS_AVG_NUM];		//[窗口平滑角度]
int g_s_obs_avg_angle_num;
int g_s_obs_avg_index;
static COOR2 g_s_obs_plan_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];		//[S弯规划线描述点]
static int g_s_obs_plan_line_num = 0;									//[S弯规划线描述点个数]
//[S弯相关参数]
int g_is_s_obs = 0;							//[是否是S弯的标志]
static int g_is_s_obs_confirm_time = 0;				//[S弯确认次数]
static int g_is_not_s_obs_confirm_time = 0;			//[不是S弯确认次数]
static int g_s_obs_quit = 0;						//[是否退出S弯的标志]
static int g_s_obs_quit_timer = 0;					//[退出S弯预警次数]
int g_s_obs_speed_down = 0;							//[S弯降速标志]
int g_s_obs_last_angle = -1;						//[上一帧规划角度]
static int g_s_obs_crash_flag = 0;
//[退出S弯时，先维持状态一段距离]
static COOR2 g_s_quit_pt;							//[维持的起点]
static COOR2 g_e_quit_pt;							//[维持的终点]

//***********************************************************************************************
//                                zgccmax 2013.Mar.2
//void fix_long_short(COOR2 *left_line, int *left_line_num, COOR2 *right_line, int *right_line_num)
//param:    COOR2 *left_line		左边
//			int *left_line_num		左边点个数
//			COOR2 *right_line		右边
//			int *right_line_num		右边点个数
//return:
//discribe:将道路两个边界修正成一样长短，以短边为依据。
//***********************************************************************************************
void fix_long_short(COOR2 *left_line, int *left_line_num, COOR2 *right_line, int *right_line_num)
{
	int i;
	int x;

	COOR2 temp_line[200];
	int temp_line_num = 0;

	int step;

#ifdef MBUG_OPEN_
	MBUG("fix_long_short \n");
	for (i=0;i<*left_line_num;i++)
	{
		MBUG("(%d, %d) ", left_line[i].x, left_line[i].y);
	}
	MBUG("\n");
	for (i=0;i<*right_line_num;i++)
	{
		MBUG("(%d, %d) ", right_line[i].x, right_line[i].y);
	}
	MBUG("\n");
#endif

	if (left_line[*left_line_num - 1].y < right_line[*right_line_num - 1].y)
	{//[右道路边长]
		get_x_coord(left_line[*left_line_num - 1].y, right_line, *right_line_num, &x);

		i = 0;
		while (right_line[i].y < left_line[*left_line_num - 1].y)
			i++;

		right_line[i].x = x;
		right_line[i].y = left_line[*left_line_num - 1].y;

		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;
		step = (right_line[i].y - right_line[0].y) / NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;

		line_fitting(right_line, i + 1, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(right_line, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		*right_line_num = temp_line_num;


		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;
		step = (left_line[*left_line_num - 1].y - left_line[0].y) / NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;

		line_fitting(left_line, *left_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(left_line, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		*left_line_num = temp_line_num;
	}
	else
	{
		get_x_coord(right_line[*right_line_num - 1].y, left_line, *left_line_num, &x);

		i = 0;
		while (left_line[i].y < right_line[*right_line_num - 1].y)
			i++;

		left_line[i].x = x;
		left_line[i].y = right_line[*right_line_num - 1].y;

		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;
		step = (left_line[i].y - left_line[0].y) / NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;

		line_fitting(left_line, i + 1, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(left_line, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		*left_line_num = temp_line_num;

		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;
		step = (right_line[*right_line_num - 1].y - right_line[0].y) / NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;

		line_fitting(right_line, *right_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(right_line, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		*right_line_num = temp_line_num;
	}
}

//***********************************************************************************************
//                                zgccmax 2013.Mar.2
//void set_obs_to_line(QUAD *box4, COOR2 *line, int *line_num, int mode)
//param:    QUAD *box4				四边形的障碍物描述
//			COOR2 *line				道路边
//			int *line_num			描述边的点个数
//			int mode				0  左边  1  右边
//return:
//discribe:将障碍物归边，形成一条将障碍物绕过的边
//***********************************************************************************************
void set_obs_to_line(QUAD *box4, COOR2 *line, int *line_num, int mode)
{
	int i, j, k, i1, i2;
	COOR2 tmp_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE + 4];
	COOR2 tmp_line1[4];
	COOR2 tmp_line3[4];
	COOR2 tmp_line2[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE + 4];
	COOR2 l_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int l_line_num;
	int cnt;//, cnt1;
	int miny, maxy, midy;
	int minx, maxx;
	int x0, x1;//yy, xl, xx,
	//	int l_pl_dist;

	int left_expand_dist;
	int right_expand_dist;

	memset(tmp_line, 0, sizeof(tmp_line));
	memset(tmp_line1, 0, sizeof(tmp_line1));
	memset(tmp_line2, 0, sizeof(tmp_line2));

	l_line_num = *line_num;
	memcpy(l_line, line, sizeof(COOR2) * l_line_num);

	//[得到最大最小x和y以及y的中值]
	get_mid_y(box4, &miny, &maxy, &midy);
	get_mid_x(box4, &minx, &maxx, NULL);

	//[以下的参数值的含义是可以不考虑的伸进路中障碍物的宽度]
	if (g_navi_state != TRACE_IN_NATURAL_ROAD)
	{
		left_expand_dist = 40;
		right_expand_dist = 40;
	}
	else
	{
		if (g_roving_switch == 0)
		{
			left_expand_dist = 40;
			right_expand_dist = 40;
		}
		else
		{
			left_expand_dist = 40;
			right_expand_dist = 40;
		}
		
	}
	//[将栅格障碍膨胀]
	for (i = 0; i < 4; i ++)
	{
		if (box4->coor2[i].x == minx)
		{
			box4->coor2[i].x -= left_expand_dist;
		}
		else if (box4->coor2[i].x == maxx)
		{
			box4->coor2[i].x += right_expand_dist;
		}

		if (box4->coor2[i].y == miny)
		{
			box4->coor2[i].y -= 100;
		}
		else if (box4->coor2[i].y == maxy)
		{
			box4->coor2[i].y += 100;
		}
	}

	miny -= 100;
	maxy += 100;

	minx -= 20;
	maxx += 20;

	if (minx == 130 && miny ==250)
	{
		minx = minx;
	}

	/* 此处写得很啰嗦...出现在障碍下界为负的情况 */
	if (miny < 0)
	{
#ifdef MBUG_OPEN_
		MBUG("miny < 0\n");
#endif
		for (i = 0; i < 4; i ++)
		{
			if (box4->coor2[i].y == miny)
			{
				box4->coor2[i].y = 1;
				miny = 1;

			}
		}
	}

	//[这里是将y超出道路的部分抹平，由于其他地方的改动，这里注释掉了]
	// 	if (maxy > l_line[l_line_num - 1].y)
	// 	{
	// 		for (i = 0; i < 4; i ++)
	// 		{
	// 			if (box4->coor2[i].y == maxy)
	// 			{
	// 				maxy = l_line[l_line_num - 1].y;
	// 				box4->coor2[i].y = maxy;
	// 			}
	// 		}
	// 	}

	midy = (miny + maxy) / 2;

	if (mode == 0)
	{
		k = -1;

		//[最下方 且靠右的点]
		for (j = 0; j < 4; j ++)
		{
			if (box4->coor2[j].y == miny)
			{
				if (k == -1)
					k = j;
				else
				{
					if (box4->coor2[k].x < box4->coor2[j].x)
					{
						k = j;
					}
				}
			}
		}

		i1 = k;

		k = -1;

		//[最上方 且靠右的点]
		for (j = 0; j < 4; j ++)
		{
			if (box4->coor2[j].y == maxy)
			{
				if (k == -1)
					k = j;
				else
				{
					if (box4->coor2[k].x < box4->coor2[j].x)
					{
						k = j;
					}
				}
			}
		}

		i2 = k;

		memset(tmp_line3, 0, sizeof(tmp_line3));
		memcpy(tmp_line3, box4->coor2 + i1, sizeof(COOR2));
		memcpy(tmp_line3 + 1, box4->coor2 + i2, sizeof(COOR2));

		//[将障碍物的点投影到一条直线上去]
		memset(tmp_line, 0, sizeof(tmp_line));
		memcpy(tmp_line, l_line, l_line_num * sizeof(COOR2));
		memcpy(tmp_line + l_line_num, tmp_line3, sizeof(tmp_line3));

		cnt = l_line_num + 2;

		//[按y轴排序]
		for (i = 0; i < cnt - 1; i ++)
		{
			for (j = i + 1; j < cnt; j ++)
			{
				if (tmp_line[i].y > tmp_line[j].y)
				{
					swap_point(tmp_line + i, tmp_line + j);
				}
			}
		}

		//[y值相同取x大的]
		for (i = 0; i < cnt - 1; i ++)
		{
			if (tmp_line[i].y == tmp_line[i + 1].y)
			{
				if (tmp_line[i].x > tmp_line[i + 1].x)
				{
					tmp_line[i + 1].x = tmp_line[i].x;
				}

				memmove(tmp_line + i, tmp_line + i + 1, (cnt - i - 1) * sizeof(COOR2));
				cnt --;
				i --;
			}
		}

		//[以下对所有点进行排序]
		k = 0;
		for (i = 0; i < cnt; i ++)
		{
			if (tmp_line[i].y < box4->coor2[i1].y)
			{
				memcpy(tmp_line2 + k, tmp_line + i, sizeof(COOR2));
				k ++;
			}
			else if (tmp_line[i].y > box4->coor2[i2].y)
			{
				memcpy(tmp_line2 + k, tmp_line + i, sizeof(COOR2) * (cnt - i));
				k += (cnt - i);
				break;
			}
			else
			{
				get_x_coord(tmp_line[i].y, l_line, l_line_num, &x0);
				get_x_coord(tmp_line[i].y, tmp_line3, 2, &x1);

				tmp_line[i].x = (x0 > x1 ? x0 : x1);

				memcpy(tmp_line2 + k, tmp_line + i, sizeof(COOR2));
				k ++;
			}
		}

		for (i=1;i<k-1; i++)
		{
			if (tmp_line2[i].x == tmp_line2[i+1].x && tmp_line2[i-1].x == tmp_line2[i].x && (tmp_line2[i+1].y - tmp_line2[i-1].y) < 300)
			{
				memmove(tmp_line2 + i, tmp_line2 + i + 1, sizeof(COOR2) * (k - i - 1));
				k--;
			}
		}

		if (k > NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		{
			COOR2 pts[200];
			int pts_num = 0;
			memset(pts, 0, sizeof(COOR2) * 200);

			int step;
			step = (int)(tmp_line2[k - 1].y - tmp_line2[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

			line_fitting(tmp_line2, k, pts, pts_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			memcpy(tmp_line2, pts, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			k = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
		}

		*line_num = k;

		memset(line, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(line, tmp_line2, k * sizeof(COOR2));

		for (i = 1; i < k - 1; i ++)
		{
			for (j = i + 1; j < k; j ++)
			{
				if (line[i].y > line[j].y)
				{
					swap_point(line + i, line + j);
				}
			}
		}
	}
	else
	{//[内容同if]
		k = -1;

		for (j = 0; j < 4; j ++)
		{
			if (box4->coor2[j].y == miny)
			{
				if (k == -1)
					k = j;
				else
				{
					if (box4->coor2[k].x > box4->coor2[j].x)
					{
						k = j;
					}
				}
			}
		}

		i1 = k;

		k = -1;

		for (j = 0; j < 4; j ++)
		{
			if (box4->coor2[j].y == maxy)
			{
				if (k == -1)
					k = j;
				else
				{
					if (box4->coor2[k].x > box4->coor2[j].x)
					{
						k = j;
					}
				}
			}
		}

		i2 = k;

		memset(tmp_line3, 0, sizeof(tmp_line3));
		memcpy(tmp_line3, box4->coor2 + i1, sizeof(COOR2));
		memcpy(tmp_line3 + 1, box4->coor2 + i2, sizeof(COOR2));

		memset(tmp_line, 0, sizeof(tmp_line));
		memcpy(tmp_line, l_line, l_line_num * sizeof(COOR2));
		memcpy(tmp_line + l_line_num, tmp_line3, sizeof(tmp_line3));

		cnt = l_line_num + 2;

		for (i = 0; i < cnt - 1; i ++)
		{
			for (j = i + 1; j < cnt; j ++)
			{
				if (tmp_line[i].y > tmp_line[j].y)
				{
					swap_point(tmp_line + i, tmp_line + j);
				}
			}
		}

		for (i = 0; i < cnt - 1; i ++)
		{
			if (tmp_line[i].y == tmp_line[i + 1].y)
			{
				if (tmp_line[i].x < tmp_line[i + 1].x)
				{
					tmp_line[i + 1].x = tmp_line[i].x;
				}

				memmove(tmp_line + i, tmp_line + i + 1, (cnt - i - 1) * sizeof(COOR2));
				cnt--;
				i --;
			}
		}

		//[以下对所有点进行排序]
		k = 0;
		for (i = 0; i < cnt; i ++)
		{
			if (tmp_line[i].y < box4->coor2[i1].y)
			{
				memcpy(tmp_line2 + k, tmp_line + i, sizeof(COOR2));
				k ++;
			}
			else if (tmp_line[i].y > box4->coor2[i2].y)
			{
				memcpy(tmp_line2 + k, tmp_line + i, sizeof(COOR2) * (cnt - i));
				k += (cnt - i);
				break;
			}
			else
			{
				get_x_coord(tmp_line[i].y, l_line, l_line_num, &x0);
				get_x_coord(tmp_line[i].y, tmp_line3, 2, &x1);

				tmp_line[i].x = (x0 < x1 ? x0 : x1);

				memcpy(tmp_line2 + k, tmp_line + i, sizeof(COOR2));
				k ++;
			}
		}

		for (i=1;i<k-1; i++)
		{
			if (tmp_line2[i].x == tmp_line2[i+1].x && tmp_line2[i-1].x == tmp_line2[i].x && (tmp_line2[i+1].y - tmp_line2[i-1].y) < 300)
			{
				memmove(tmp_line2 + i, tmp_line2 + i + 1, sizeof(COOR2) * (k - i - 1));
				k--;
			}
		}

		if (k > NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		{
			COOR2 pts[200];
			int pts_num = 0;
			memset(pts, 0, sizeof(COOR2) * 200);

			int step;
			step = (int)(tmp_line2[k - 1].y - tmp_line2[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

			line_fitting(tmp_line2, k, pts, pts_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			memcpy(tmp_line2, pts, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			k = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
		}

		*line_num = k;

		memset(line, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(line, tmp_line2, k * sizeof(COOR2));

		for (i = 1; i < k - 1; i ++)
		{
			for (j = i + 1; j < k; j ++)
			{
				if (line[i].y > line[j].y)
				{
					swap_point(line + i, line + j);
				}
			}
		}


	}

	//[后面的代码不用看了]
	return;

	if (k > NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		k = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;

	memset(line, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memcpy(line, tmp_line2, k * sizeof(COOR2));
	*line_num = k;

	for (i = 1; i < k - 1; i ++)
	{
		for (j = i + 1; j < k; j ++)
		{
			if (line[i].y > line[j].y)
			{
				swap_point(line + i, line + j);
			}
		}
	}
}

//[膨胀2m的圆形模板]
#define  CIRCLE_TEMPLET8 197

#define  CIRCLE_TEMPLET 197
static int circle[197][3] = {{-8,0,0},{-7,-3,0},{-7,-2,1},{-7,-1,1},{-7,0,1},{-7,1,1},{-7,2,1},{-7,3,0},{-6,-5,0},{-6,-4,1},{-6,-3,1},{-6,-2,2},{-6,-1,2},{-6,0,2},{-6,1,2},{-6,2,2},{-6,3,1},{-6,4,1},{-6,5,0},{-5,-6,0},{-5,-5,1},{-5,-4,2},{-5,-3,2},{-5,-2,3},{-5,-1,3},{-5,0,3},{-5,1,3},{-5,2,3},{-5,3,2},{-5,4,2},{-5,5,1},{-5,6,0},{-4,-6,1},{-4,-5,2},{-4,-4,2},{-4,-3,3},{-4,-2,4},{-4,-1,4},{-4,0,4},{-4,1,4},{-4,2,4},{-4,3,3},{-4,4,2},{-4,5,2},{-4,6,1},{-3,-7,0},{-3,-6,1},{-3,-5,2},{-3,-4,3},{-3,-3,4},{-3,-2,4},{-3,-1,5},{-3,0,5},{-3,1,5},{-3,2,4},{-3,3,4},{-3,4,3},{-3,5,2},{-3,6,1},{-3,7,0},{-2,-7,1},{-2,-6,2},{-2,-5,3},{-2,-4,4},{-2,-3,4},{-2,-2,5},{-2,-1,6},{-2,0,6},{-2,1,6},{-2,2,5},{-2,3,4},{-2,4,4},{-2,5,3},{-2,6,2},{-2,7,1},{-1,-7,1},{-1,-6,2},{-1,-5,3},{-1,-4,4},{-1,-3,5},{-1,-2,6},{-1,-1,7},{-1,0,7},{-1,1,7},{-1,2,6},{-1,3,5},{-1,4,4},{-1,5,3},{-1,6,2},{-1,7,1},{0,-8,0},{0,-7,1},{0,-6,2},{0,-5,3},{0,-4,4},{0,-3,5},{0,-2,6},{0,-1,7},{0,0,8},{0,1,7},{0,2,6},{0,3,5},{0,4,4},{0,5,3},{0,6,2},{0,7,1},{0,8,0},{1,-7,1},{1,-6,2},{1,-5,3},{1,-4,4},{1,-3,5},{1,-2,6},{1,-1,7},{1,0,7},{1,1,7},{1,2,6},{1,3,5},{1,4,4},{1,5,3},{1,6,2},{1,7,1},{2,-7,1},{2,-6,2},{2,-5,3},{2,-4,4},{2,-3,4},{2,-2,5},{2,-1,6},{2,0,6},{2,1,6},{2,2,5},{2,3,4},{2,4,4},{2,5,3},{2,6,2},{2,7,1},{3,-7,0},{3,-6,1},{3,-5,2},{3,-4,3},{3,-3,4},{3,-2,4},{3,-1,5},{3,0,5},{3,1,5},{3,2,4},{3,3,4},{3,4,3},{3,5,2},{3,6,1},{3,7,0},{4,-6,1},{4,-5,2},{4,-4,2},{4,-3,3},{4,-2,4},{4,-1,4},{4,0,4},{4,1,4},{4,2,4},{4,3,3},{4,4,2},{4,5,2},{4,6,1},{5,-6,0},{5,-5,1},{5,-4,2},{5,-3,2},{5,-2,3},{5,-1,3},{5,0,3},{5,1,3},{5,2,3},{5,3,2},{5,4,2},{5,5,1},{5,6,0},{6,-5,0},{6,-4,1},{6,-3,1},{6,-2,2},{6,-1,2},{6,0,2},{6,1,2},{6,2,2},{6,3,1},{6,4,1},{6,5,0},{7,-3,0},{7,-2,1},{7,-1,1},{7,0,1},{7,1,1},{7,2,1},{7,3,0},{8,0,0}};

//[x,y,weight]
#define CIRCLE_TEMPLET_BIG 253
//static int circle_big[253][3] = {{-9,0,0},{-8,-4,0},{-8,-3,0},{-8,-2,1},{-8,-1,1},{-8,0,1},{-8,1,1},{-8,2,1},{-8,3,0},{-8,4,0},{-7,-5,0},{-7,-4,1},{-7,-3,1},{-7,-2,2},{-7,-1,2},{-7,0,2},{-7,1,2},{-7,2,2},{-7,3,1},{-7,4,1},{-7,5,0},{-6,-6,1},{-6,-5,1},{-6,-4,2},{-6,-3,2},{-6,-2,3},{-6,-1,3},{-6,0,3},{-6,1,3},{-6,2,3},{-6,3,2},{-6,4,2},{-6,5,1},{-6,6,1},{-5,-7,0},{-5,-6,1},{-5,-5,2},{-5,-4,3},{-5,-3,3},{-5,-2,4},{-5,-1,4},{-5,0,4},{-5,1,4},{-5,2,4},{-5,3,3},{-5,4,3},{-5,5,2},{-5,6,1},{-5,7,0},{-4,-8,0},{-4,-7,1},{-4,-6,2},{-4,-5,3},{-4,-4,3},{-4,-3,4},{-4,-2,5},{-4,-1,5},{-4,0,5},{-4,1,5},{-4,2,5},{-4,3,4},{-4,4,3},{-4,5,3},{-4,6,2},{-4,7,1},{-4,8,0},{-3,-8,0},{-3,-7,1},{-3,-6,2},{-3,-5,3},{-3,-4,4},{-3,-3,5},{-3,-2,5},{-3,-1,6},{-3,0,6},{-3,1,6},{-3,2,5},{-3,3,5},{-3,4,4},{-3,5,3},{-3,6,2},{-3,7,1},{-3,8,0},{-2,-8,1},{-2,-7,2},{-2,-6,3},{-2,-5,4},{-2,-4,5},{-2,-3,5},{-2,-2,6},{-2,-1,7},{-2,0,7},{-2,1,7},{-2,2,6},{-2,3,5},{-2,4,5},{-2,5,4},{-2,6,3},{-2,7,2},{-2,8,1},{-1,-8,1},{-1,-7,2},{-1,-6,3},{-1,-5,4},{-1,-4,5},{-1,-3,6},{-1,-2,7},{-1,-1,8},{-1,0,8},{-1,1,8},{-1,2,7},{-1,3,6},{-1,4,5},{-1,5,4},{-1,6,3},{-1,7,2},{-1,8,1},{0,-9,0},{0,-8,1},{0,-7,2},{0,-6,3},{0,-5,4},{0,-4,5},{0,-3,6},{0,-2,7},{0,-1,8},{0,0,9},{0,1,8},{0,2,7},{0,3,6},{0,4,5},{0,5,4},{0,6,3},{0,7,2},{0,8,1},{0,9,0},{1,-8,1},{1,-7,2},{1,-6,3},{1,-5,4},{1,-4,5},{1,-3,6},{1,-2,7},{1,-1,8},{1,0,8},{1,1,8},{1,2,7},{1,3,6},{1,4,5},{1,5,4},{1,6,3},{1,7,2},{1,8,1},{2,-8,1},{2,-7,2},{2,-6,3},{2,-5,4},{2,-4,5},{2,-3,5},{2,-2,6},{2,-1,7},{2,0,7},{2,1,7},{2,2,6},{2,3,5},{2,4,5},{2,5,4},{2,6,3},{2,7,2},{2,8,1},{3,-8,0},{3,-7,1},{3,-6,2},{3,-5,3},{3,-4,4},{3,-3,5},{3,-2,5},{3,-1,6},{3,0,6},{3,1,6},{3,2,5},{3,3,5},{3,4,4},{3,5,3},{3,6,2},{3,7,1},{3,8,0},{4,-8,0},{4,-7,1},{4,-6,2},{4,-5,3},{4,-4,3},{4,-3,4},{4,-2,5},{4,-1,5},{4,0,5},{4,1,5},{4,2,5},{4,3,4},{4,4,3},{4,5,3},{4,6,2},{4,7,1},{4,8,0},{5,-7,0},{5,-6,1},{5,-5,2},{5,-4,3},{5,-3,3},{5,-2,4},{5,-1,4},{5,0,4},{5,1,4},{5,2,4},{5,3,3},{5,4,3},{5,5,2},{5,6,1},{5,7,0},{6,-6,1},{6,-5,1},{6,-4,2},{6,-3,2},{6,-2,3},{6,-1,3},{6,0,3},{6,1,3},{6,2,3},{6,3,2},{6,4,2},{6,5,1},{6,6,1},{7,-5,0},{7,-4,1},{7,-3,1},{7,-2,2},{7,-1,2},{7,0,2},{7,1,2},{7,2,2},{7,3,1},{7,4,1},{7,5,0},{8,-4,0},{8,-3,0},{8,-2,1},{8,-1,1},{8,0,1},{8,1,1},{8,2,1},{8,3,0},{8,4,0},{9,0,0}};
int g_grid_dd_map[GRID_HEIGHT][GRID_WIDTH];//[膨胀腐蚀图]
int g_grid_dd_flag = 0;
/*==================================================================
 * 函数名  ：	void grid_operation(int grid[][GRID_WIDTH], int res_grid[][GRID_WIDTH], int win)
 * 功能    ：	对左右12m，前方35m的区域栅格进行膨胀
 * 输入参数：	int grid[][GRID_WIDTH]		输入栅格
				int res_grid[][GRID_WIDTH]	输出栅格
				int win						窗口大小
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void grid_operation(int grid[][GRID_WIDTH], int res_grid[][GRID_WIDTH], int win)
{
	int x, y;
	int i, j, m, n;
	int max;
	memset(res_grid, 0, GRID_HEIGHT * GRID_WIDTH * sizeof(int));

	//[膨胀]
	for (y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for (x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			max = 0;
			for (i = -win; i <= win; i++)
			{
				for (j = -(win); j <= (win); j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (grid[m][n] > 0)
					{
						max = 8;
						break;
					}
				}
			}
			res_grid[y][x] = max;
		}
	}
}

/*==================================================================
 * 函数名  ：	void grid_close_operation(int grid[][GRID_WIDTH], int win)
 * 功能    ：	对左右12m，前方35m的区域栅格进行膨胀腐蚀运算
 * 输入参数：	int grid[][GRID_WIDTH]		输入栅格
				int win						窗口大小
 * 输出参数：	g_grid_dd_map				保存到全局变量
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void grid_close_operation(int grid[][GRID_WIDTH], int win)
{
	int x, y;
	int i, j, m, n;
	int max;
	int res_grid[GRID_HEIGHT][GRID_WIDTH];
	memset(res_grid, 0, GRID_HEIGHT * GRID_WIDTH * sizeof(int));

	//[膨胀]
	for (y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for (x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			max = 0;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (grid[m][n] > 0)
					{
						max = 1;
						break;
					}
				}
			}
			res_grid[y][x] = max;
		}
	}

	//[腐蚀]
	g_grid_dd_flag = 1;
	memcpy(g_grid_dd_map, res_grid, GRID_HEIGHT * GRID_WIDTH * sizeof(int));
	int min = 0;

	for (y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for (x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			min = 1;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (res_grid[m][n] < 1)
					{
						min = 0;
						break;
					}
				}
			}
			g_grid_dd_map[y][x] = min;
		}
	}
}

void grid_close_operation(int grid[][GRID_WIDTH], int win, int win_2)
{
	int x, y;
	int i, j, m, n;
	int max;
	int res_grid[GRID_HEIGHT][GRID_WIDTH];
	memset(res_grid, 0, GRID_HEIGHT * GRID_WIDTH * sizeof(int));

	//[膨胀]
	for (y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for (x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			max = 0;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (grid[m][n] > 0)
					{
						max = 8;
						break;
					}
				}
			}
			res_grid[y][x] = max;
		}
	}

	//[腐蚀]
	memcpy(g_closed_grid_map, res_grid, GRID_HEIGHT * GRID_WIDTH * sizeof(int));
	int min = 0;

	if (win_2 != -1 && win_2 <= win)
		win = win_2;

	for (y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for (x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			min = 8;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (res_grid[m][n] < 8)
					{
						min = 0;
						break;
					}
				}
			}
			g_closed_grid_map[y][x] = min;
		}
	}
}

void grid_close_operation(int grid[][GRID_WIDTH], int win, int win_2, int res_grid[][GRID_WIDTH])
{
	int x, y;
	int i, j, m, n;
	int max;
	int temp_grid[GRID_HEIGHT][GRID_WIDTH];
	memset(temp_grid, 0, GRID_HEIGHT * GRID_WIDTH * sizeof(int));

	memcpy(temp_grid, grid, GRID_HEIGHT * GRID_WIDTH * sizeof(int));
	//[膨胀]
	for (y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for (x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			max = 0;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (grid[m][n] > 0)
					{
						max = 8;
						break;
					}
				}
			}
			temp_grid[y][x] = max;
		}
	}

	//[腐蚀]
	memcpy(res_grid, temp_grid, GRID_HEIGHT * GRID_WIDTH * sizeof(int));
	int min = 0;

	if (win_2 != -1 && win_2 <= win)
		win = win_2;

	for (y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for (x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			min = 8;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (temp_grid[m][n] < 8)
					{
						min = 0;
						break;
					}
				}
			}
			res_grid[y][x] = min;
		}
	}
}

/*==================================================================
 * 函数名  ：	void grid_operation(int grid[][GRID_WIDTH], int width, int height, int res_grid[][GRID_WIDTH])
 * 功能    ：	2014年比赛之前的栅格操作，有重名函数
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void grid_operation(int grid[][GRID_WIDTH], int width, int height, int res_grid[][GRID_WIDTH])
{
	int x, y;
	int i, j, m, n;
	int max, min;
	int temp_grid[GRID_HEIGHT][GRID_WIDTH];
	int win = 4;
	memset(res_grid, 0, GRID_HEIGHT * GRID_WIDTH * sizeof(int));

	//[所有当成静态障碍物避障]
	for (i = 0; i < GRID_HEIGHT; i++)
	{
		for (j = 0; j < GRID_WIDTH; j++)
		{
			if (grid[i][j] > 0)
				grid[i][j] = 8;
		}
	}

	memcpy(temp_grid, grid, width * height * sizeof(int));

	//膨胀
	for (y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for (x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			max = 0;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (grid[m][n] > 0)
					{
						max = 8;
						break;
					}
				}
			}
			temp_grid[y][x] = max;
		}
	}

	memcpy(grid, temp_grid, width * height * sizeof(int));

	for (y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for (x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			min = 8;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (temp_grid[m][n] < 8)
					{
						min = 0;
						break;
					}
				}
			}
			grid[y][x] = min;
		}
	}

	memcpy(res_grid, grid, width * height * sizeof(int));

	//[@@目前实验，先return]
	// 	if (g_navi_state == ROVING || g_cross_und_state == CROSS_ROVING || g_is_s_obs == 1)
	// 	{
	// 		return;
	// 	}

// 	if (g_cross_und_state == CROSS_ROVING || g_is_s_obs == 1)
// 	{
// 		return;
// 	}

	//[@@目前实验]

	for (y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for (x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			if (grid[y][x] == 8)
			{
				for (i = 0; i < CIRCLE_TEMPLET; i++)
				{
					int elevel;
					n = x + circle[i][0];
					m = y + circle[i][1];
					elevel = circle[i][2];

					if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (elevel > res_grid[m][n])
					{
						res_grid[m][n] = elevel;
					}
				}
			}
		}
	}
}

/*==================================================================
 * 函数名  ：	void grid_operation_include_tail(int grid[][GRID_WIDTH], int res_grid[][GRID_WIDTH])
 * 功能    ：	进行膨胀，涵盖车后方栅格
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void grid_operation_include_tail(int grid[][GRID_WIDTH], int res_grid[][GRID_WIDTH])
{
	int x, y;
	int i, j, m, n;
	int max;
	int temp_grid[GRID_HEIGHT][GRID_WIDTH];
	int win = 2;
	memset(res_grid, 0, GRID_HEIGHT * GRID_WIDTH * sizeof(int));

	//[所有当成静态障碍物避障]
	for (i = 0; i < GRID_HEIGHT; i++)
		for (j = 0; j < GRID_WIDTH; j++)
			if (grid[i][j] > 0)
				grid[i][j] = 1;

	memcpy(temp_grid, grid, GRID_HEIGHT * GRID_WIDTH * sizeof(int));

	//[1.膨胀连通，前方35m，左右各12m，后方12m]
	for (y = g_grid_center.y - 48; y < g_grid_center.y + 140; y++)
	{
		for (x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			max = 0;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_grid_center.y - 48 || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (grid[m][n] > 0)
					{
						max = 1;
						break;
					}
				}
			}
			res_grid[y][x] = max;
		}
	}
}

#define  CIRCLE_TEMPLET4_S 45
static int circle4_s[CIRCLE_TEMPLET4_S][3] = { { -3, -2, 1 }, { -3, -1, 2 }, { -3, 0, 2 }, { -3, 1, 2 }, { -3, 2, 1 }, { -2, -3, 1 }, { -2, -2, 2 }, { -2, -1, 3 }, { -2, 0, 3 }, { -2, 1, 3 }, { -2, 2, 2 }, { -2, 3, 1 }, { -1, -3, 2 }, { -1, -2, 3 }, { -1, -1, 4 }, { -1, 0, 4 }, { -1, 1, 4 }, { -1, 2, 3 }, { -1, 3, 2 }, { 0, -3, 2 }, { 0, -2, 3 }, { 0, -1, 4 }, { 0, 0, 5 }, { 0, 1, 4 }, { 0, 2, 3 }, { 0, 3, 2 }, { 1, -3, 2 }, { 1, -2, 3 }, { 1, -1, 4 }, { 1, 0, 4 }, { 1, 1, 4 }, { 1, 2, 3 }, { 1, 3, 2 }, { 2, -3, 1 }, { 2, -2, 2 }, { 2, -1, 3 }, { 2, 0, 3 }, { 2, 1, 3 }, { 2, 2, 2 }, { 2, 3, 1 }, { 3, -2, 1 }, { 3, -1, 2 }, { 3, 0, 2 }, { 3, 1, 2 }, { 3, 2, 1 } };
#define CIRCLE_TEMPLET6_S 109
static int circle6_s[CIRCLE_TEMPLET6_S][3] = { { -5, -3, -1 }, { -5, -2, 0 }, { -5, -1, 0 }, { -5, 0, 0 }, { -5, 1, 0 }, { -5, 2, 0 }, { -5, 3, -1 }, { -4, -4, -1 }, { -4, -3, 0 }, { -4, -2, 1 }, { -4, -1, 1 }, { -4, 0, 1 }, { -4, 1, 1 }, { -4, 2, 1 }, { -4, 3, 0 }, { -4, 4, -1 }, { -3, -5, -1 }, { -3, -4, 0 }, { -3, -3, 1 }, { -3, -2, 1 }, { -3, -1, 2 }, { -3, 0, 2 }, { -3, 1, 2 }, { -3, 2, 1 }, { -3, 3, 1 }, { -3, 4, 0 }, { -3, 5, -1 }, { -2, -5, 0 }, { -2, -4, 1 }, { -2, -3, 1 }, { -2, -2, 2 }, { -2, -1, 3 }, { -2, 0, 3 }, { -2, 1, 3 }, { -2, 2, 2 }, { -2, 3, 1 }, { -2, 4, 1 }, { -2, 5, 0 }, { -1, -5, 0 }, { -1, -4, 1 }, { -1, -3, 2 }, { -1, -2, 3 }, { -1, -1, 4 }, { -1, 0, 4 }, { -1, 1, 4 }, { -1, 2, 3 }, { -1, 3, 2 }, { -1, 4, 1 }, { -1, 5, 0 }, { 0, -5, 0 }, { 0, -4, 1 }, { 0, -3, 2 }, { 0, -2, 3 }, { 0, -1, 4 }, { 0, 0, 5 }, { 0, 1, 4 }, { 0, 2, 3 }, { 0, 3, 2 }, { 0, 4, 1 }, { 0, 5, 0 }, { 1, -5, 0 }, { 1, -4, 1 }, { 1, -3, 2 }, { 1, -2, 3 }, { 1, -1, 4 }, { 1, 0, 4 }, { 1, 1, 4 }, { 1, 2, 3 }, { 1, 3, 2 }, { 1, 4, 1 }, { 1, 5, 0 }, { 2, -5, 0 }, { 2, -4, 1 }, { 2, -3, 1 }, { 2, -2, 2 }, { 2, -1, 3 }, { 2, 0, 3 }, { 2, 1, 3 }, { 2, 2, 2 }, { 2, 3, 1 }, { 2, 4, 1 }, { 2, 5, 0 }, { 3, -5, -1 }, { 3, -4, 0 }, { 3, -3, 1 }, { 3, -2, 1 }, { 3, -1, 2 }, { 3, 0, 2 }, { 3, 1, 2 }, { 3, 2, 1 }, { 3, 3, 1 }, { 3, 4, 0 }, { 3, 5, -1 }, { 4, -4, -1 }, { 4, -3, 0 }, { 4, -2, 1 }, { 4, -1, 1 }, { 4, 0, 1 }, { 4, 1, 1 }, { 4, 2, 1 }, { 4, 3, 0 }, { 4, 4, -1 }, { 5, -3, -1 }, { 5, -2, 0 }, { 5, -1, 0 }, { 5, 0, 0 }, { 5, 1, 0 }, { 5, 2, 0 }, { 5, 3, -1 } };
#define  CIRCLE_TEMPLET8_S 197
static int circle8_s[197][3] = { { -8, 0, 0 }, { -7, -3, 0 }, { -7, -2, 1 }, { -7, -1, 1 }, { -7, 0, 1 }, { -7, 1, 1 }, { -7, 2, 1 }, { -7, 3, 0 }, { -6, -5, 0 }, { -6, -4, 1 }, { -6, -3, 1 }, { -6, -2, 2 }, { -6, -1, 2 }, { -6, 0, 2 }, { -6, 1, 2 }, { -6, 2, 2 }, { -6, 3, 1 }, { -6, 4, 1 }, { -6, 5, 0 }, { -5, -6, 0 }, { -5, -5, 1 }, { -5, -4, 2 }, { -5, -3, 2 }, { -5, -2, 3 }, { -5, -1, 3 }, { -5, 0, 3 }, { -5, 1, 3 }, { -5, 2, 3 }, { -5, 3, 2 }, { -5, 4, 2 }, { -5, 5, 1 }, { -5, 6, 0 }, { -4, -6, 1 }, { -4, -5, 2 }, { -4, -4, 2 }, { -4, -3, 3 }, { -4, -2, 4 }, { -4, -1, 4 }, { -4, 0, 4 }, { -4, 1, 4 }, { -4, 2, 4 }, { -4, 3, 3 }, { -4, 4, 2 }, { -4, 5, 2 }, { -4, 6, 1 }, { -3, -7, 0 }, { -3, -6, 1 }, { -3, -5, 2 }, { -3, -4, 3 }, { -3, -3, 4 }, { -3, -2, 4 }, { -3, -1, 5 }, { -3, 0, 5 }, { -3, 1, 5 }, { -3, 2, 4 }, { -3, 3, 4 }, { -3, 4, 3 }, { -3, 5, 2 }, { -3, 6, 1 }, { -3, 7, 0 }, { -2, -7, 1 }, { -2, -6, 2 }, { -2, -5, 3 }, { -2, -4, 4 }, { -2, -3, 4 }, { -2, -2, 5 }, { -2, -1, 6 }, { -2, 0, 6 }, { -2, 1, 6 }, { -2, 2, 5 }, { -2, 3, 4 }, { -2, 4, 4 }, { -2, 5, 3 }, { -2, 6, 2 }, { -2, 7, 1 }, { -1, -7, 1 }, { -1, -6, 2 }, { -1, -5, 3 }, { -1, -4, 4 }, { -1, -3, 5 }, { -1, -2, 6 }, { -1, -1, 7 }, { -1, 0, 7 }, { -1, 1, 7 }, { -1, 2, 6 }, { -1, 3, 5 }, { -1, 4, 4 }, { -1, 5, 3 }, { -1, 6, 2 }, { -1, 7, 1 }, { 0, -8, 0 }, { 0, -7, 1 }, { 0, -6, 2 }, { 0, -5, 3 }, { 0, -4, 4 }, { 0, -3, 5 }, { 0, -2, 6 }, { 0, -1, 7 }, { 0, 0, 8 }, { 0, 1, 7 }, { 0, 2, 6 }, { 0, 3, 5 }, { 0, 4, 4 }, { 0, 5, 3 }, { 0, 6, 2 }, { 0, 7, 1 }, { 0, 8, 0 }, { 1, -7, 1 }, { 1, -6, 2 }, { 1, -5, 3 }, { 1, -4, 4 }, { 1, -3, 5 }, { 1, -2, 6 }, { 1, -1, 7 }, { 1, 0, 7 }, { 1, 1, 7 }, { 1, 2, 6 }, { 1, 3, 5 }, { 1, 4, 4 }, { 1, 5, 3 }, { 1, 6, 2 }, { 1, 7, 1 }, { 2, -7, 1 }, { 2, -6, 2 }, { 2, -5, 3 }, { 2, -4, 4 }, { 2, -3, 4 }, { 2, -2, 5 }, { 2, -1, 6 }, { 2, 0, 6 }, { 2, 1, 6 }, { 2, 2, 5 }, { 2, 3, 4 }, { 2, 4, 4 }, { 2, 5, 3 }, { 2, 6, 2 }, { 2, 7, 1 }, { 3, -7, 0 }, { 3, -6, 1 }, { 3, -5, 2 }, { 3, -4, 3 }, { 3, -3, 4 }, { 3, -2, 4 }, { 3, -1, 5 }, { 3, 0, 5 }, { 3, 1, 5 }, { 3, 2, 4 }, { 3, 3, 4 }, { 3, 4, 3 }, { 3, 5, 2 }, { 3, 6, 1 }, { 3, 7, 0 }, { 4, -6, 1 }, { 4, -5, 2 }, { 4, -4, 2 }, { 4, -3, 3 }, { 4, -2, 4 }, { 4, -1, 4 }, { 4, 0, 4 }, { 4, 1, 4 }, { 4, 2, 4 }, { 4, 3, 3 }, { 4, 4, 2 }, { 4, 5, 2 }, { 4, 6, 1 }, { 5, -6, 0 }, { 5, -5, 1 }, { 5, -4, 2 }, { 5, -3, 2 }, { 5, -2, 3 }, { 5, -1, 3 }, { 5, 0, 3 }, { 5, 1, 3 }, { 5, 2, 3 }, { 5, 3, 2 }, { 5, 4, 2 }, { 5, 5, 1 }, { 5, 6, 0 }, { 6, -5, 0 }, { 6, -4, 1 }, { 6, -3, 1 }, { 6, -2, 2 }, { 6, -1, 2 }, { 6, 0, 2 }, { 6, 1, 2 }, { 6, 2, 2 }, { 6, 3, 1 }, { 6, 4, 1 }, { 6, 5, 0 }, { 7, -3, 0 }, { 7, -2, 1 }, { 7, -1, 1 }, { 7, 0, 1 }, { 7, 1, 1 }, { 7, 2, 1 }, { 7, 3, 0 }, { 8, 0, 0 } };

/*==================================================================
 * 函数名  ：	void grid_operation_circle6(int grid[][GRID_WIDTH], int width, int height, int res_grid[][GRID_WIDTH])
 * 功能    ：	半径为6栅格的圆进行膨胀
 * 输入参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void grid_operation_circle(int grid[][GRID_WIDTH], int width, int height, int res_grid[][GRID_WIDTH], int template_win)
{
	int x, y;
	int i, j, m, n;
	int max, min;
	int temp_grid[GRID_HEIGHT][GRID_WIDTH];
	int win = 4;
	memset(res_grid, 0, GRID_HEIGHT * GRID_WIDTH * sizeof(int));

	if (g_roving_switch == 1)
	{
		win = 6;
	}
	else
	{
		win = 2;
	}

	if (template_win  == 3)
	{
		win = 3;
	}


	//[所有当成静态障碍物避障]
	for (i=0;i<GRID_HEIGHT;i++)
	{
		for (j=0;j<GRID_WIDTH;j++)
		{
			if (grid[i][j] > 0)
				grid[i][j] = 1;
		}
	}

	memcpy(temp_grid, grid, width * height * sizeof(int));

	//[1.膨胀连通，前方35m，左右各12m]
	for(y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for(x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			max = 0;
			for (i=-win; i<=win; i++)
			{
				for (j=-win; j<=win; j++)
				{
					n = x+i;
					m = y+j;
					//[越界判断]
					if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (grid[m][n] > 0)
					{
						max = 1;
						break;
					}
				}
			}
			temp_grid[y][x] = max;
		}
	}

	if (template_win == 3)
	{
		memcpy(res_grid, temp_grid, width * height * sizeof(int));
		return;
	}

	memcpy(grid, temp_grid, width * height * sizeof(int));

	//[2.腐蚀去噪]
	for(y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for(x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			min = 1;
			for (i=-win; i<=win; i++)
			{
				for (j=-win; j<=win; j++)
				{
					n = x+i;
					m = y+j;
					//[越界判断]
					if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
						continue;

					if (temp_grid[m][n] < 1)
					{
						min = 0;
						break;
					}
				}
			}
			grid[y][x] = min;
		}
	}

	//[3.套用圆形模板]
	for(y = g_grid_center.y; y < g_grid_center.y + 140; y++)
	{
		for(x = g_grid_center.x - 48; x < g_grid_center.x + 48; x++)
		{
			if (grid[y][x] == 1)
			{
				switch (template_win)
				{
				case 4:
					for(i=0; i<CIRCLE_TEMPLET4_S; i++)
					{
						n = x + circle4_s[i][0];
						m = y + circle4_s[i][1];

						if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
							continue;

						if (res_grid[m][n] < 1)
							res_grid[m][n] = 1;
					}
					break;
				case 6:
					for(i=0; i<CIRCLE_TEMPLET6_S; i++)
					{
						n = x + circle6_s[i][0];
						m = y + circle6_s[i][1];

						if (m < g_grid_center.y || m >= g_grid_center.y + 140 || n < g_grid_center.x - 48 || n >= g_grid_center.x + 48)
							continue;

						if (res_grid[m][n] < 1)
							res_grid[m][n] = 1;
					}
					break;
				}

			}
		}
	}
}

void hd_grid_close_operation(int grid[][GRID_WIDTH_HD], int win, int win_2, int res_grid[][GRID_WIDTH_HD])
{
	int x, y;
	int i, j, m, n;
	int max;
	memset(res_grid, 0, GRID_HEIGHT_HD * GRID_WIDTH_HD * sizeof(int));

	int temp_grid[GRID_HEIGHT_HD][GRID_WIDTH_HD];
	memcpy(temp_grid, grid, GRID_HEIGHT_HD * GRID_WIDTH_HD * sizeof(int));

	//[膨胀]
	for (y = g_hd_grid_center.y; y < g_hd_grid_center.y + 140; y++)
	{
		for (x = g_hd_grid_center.x - 48; x < g_hd_grid_center.x + 48; x++)
		{
			max = 0;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_hd_grid_center.y || \
						m >= g_hd_grid_center.y + 140 || \
						n < g_hd_grid_center.x - 48 || \
						n >= g_hd_grid_center.x + 48)
						continue;

					if (grid[m][n] > 0)
					{
						max = 1;
						break;
					}
				}
			}
			temp_grid[y][x] = max;
		}
	}

	//[腐蚀]
	memcpy(grid, temp_grid, GRID_HEIGHT_HD * GRID_WIDTH_HD * sizeof(int));
	int min = 0;

	if (win_2 != -1 && win_2 <= win)
		win = win_2;

	for (y = g_hd_grid_center.y; y < g_hd_grid_center.y + 140; y++)
	{
		for (x = g_hd_grid_center.x - 48; x < g_hd_grid_center.x + 48; x++)
		{
			min = 1;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_hd_grid_center.y || \
						m >= g_hd_grid_center.y + 140 || \
						n < g_hd_grid_center.x - 48 || \
						n >= g_hd_grid_center.x + 48)
						continue;

					if (grid[m][n] < 1)
					{
						min = 0;
						break;
					}
				}
			}
			res_grid[y][x] = min;
		}
	}
}

void hd_grid_close_operation_circle(int grid[][GRID_WIDTH_HD], int win, int win_2, int res_grid[][GRID_WIDTH_HD])
{
	int x, y;
	int i, j, m, n;
	int max, min;
	int temp_grid[GRID_HEIGHT_HD][GRID_WIDTH_HD];
	memset(res_grid, 0, GRID_HEIGHT_HD * GRID_WIDTH_HD * sizeof(int));
	memset(temp_grid, 0, GRID_HEIGHT_HD * GRID_WIDTH_HD * sizeof(int));

	//[1.膨胀连通，前方35m，左右各12m]
	for (y = g_hd_grid_center.y; y < g_hd_grid_center.y + 140; y++)
	{
		for (x = g_hd_grid_center.x - 48; x < g_hd_grid_center.x + 48; x++)
		{
			max = 0;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_hd_grid_center.y || \
						m >= g_hd_grid_center.y + 140 || \
						n < g_hd_grid_center.x - 48 || \
						n >= g_hd_grid_center.x + 48)
						continue;

					if (grid[m][n] > 0)
					{
						max = 8;
						break;
					}
				}
			}
			temp_grid[y][x] = max;
		}
	}

	memcpy(res_grid, temp_grid, GRID_HEIGHT_HD * GRID_WIDTH_HD * sizeof(int));

	//[2.腐蚀去噪]
	if (win_2 != -1 && win_2 <= win)
		win = win_2;

	for (y = g_hd_grid_center.y; y < g_hd_grid_center.y + 140; y++)
	{
		for (x = g_hd_grid_center.x - 48; x < g_hd_grid_center.x + 48; x++)
		{
			min = 8;
			for (i = -win; i <= win; i++)
			{
				for (j = -win; j <= win; j++)
				{
					n = x + i;
					m = y + j;
					//[越界判断]
					if (m < g_hd_grid_center.y || \
						m >= g_hd_grid_center.y + 140 || \
						n < g_hd_grid_center.x - 48 || \
						n >= g_hd_grid_center.x + 48)
						continue;

					if (temp_grid[m][n] < 8)
					{
						min = 0;
						break;
					}
				}
			}
			res_grid[y][x] = min;
		}
	}

	//[3.套用圆形模板]
	int template_win = 8;
	memcpy(temp_grid, res_grid, GRID_HEIGHT_HD * GRID_WIDTH_HD * sizeof(int));
	for (y = g_hd_grid_center.y; y < g_hd_grid_center.y + 140; y++)
	{
		for (x = g_hd_grid_center.x - 48; x < g_hd_grid_center.x + 48; x++)
		{
			if (temp_grid[y][x] == 8)
			{
				switch (template_win)
				{
				case 4:
					for (i = 0; i < CIRCLE_TEMPLET4_S; i++)
					{
						n = x + circle4_s[i][0];
						m = y + circle4_s[i][1];

						if (m < g_hd_grid_center.y || \
							m >= g_hd_grid_center.y + 140 || \
							n < g_hd_grid_center.x - 48 || \
							n >= g_hd_grid_center.x + 48)
							continue;

						if (res_grid[m][n] < 1)
							res_grid[m][n] = 1;
					}
					break;
				case 6:
					for (i = 0; i < CIRCLE_TEMPLET6_S; i++)
					{
						n = x + circle6_s[i][0];
						m = y + circle6_s[i][1];

						if (m < g_hd_grid_center.y || \
							m >= g_hd_grid_center.y + 140 || \
							n < g_hd_grid_center.x - 48 || \
							n >= g_hd_grid_center.x + 48)
							continue;

						if (res_grid[m][n] < 1)
							res_grid[m][n] = 1;
					}
					break;
				case 8:
					if (res_grid[y][x] == 8)
					{
						for (i = 0; i < CIRCLE_TEMPLET8_S; i++)
						{
							int elevel;
							n = x + circle8_s[i][0];
							m = y + circle8_s[i][1];
							elevel = circle8_s[i][2];

							if (m < g_hd_grid_center.y || \
								m >= g_hd_grid_center.y + 140 || \
								n < g_hd_grid_center.x - 48 || \
								n >= g_hd_grid_center.x + 48)
								continue;

							if (elevel > res_grid[m][n])
							{
								res_grid[m][n] = elevel;
							}
						}
					}
					break;
				}

			}
		}
	}
}

//***********************************************************************************************
//                                zgccmax 2013.Mar.2
//void fill_edge_to_grid_map(COOR2 *edge, int edge_num, int grid[][GRID_WIDTH])
//param:    COOR2 *edge					道路边
//			int edge_num				道路边描述的点个数
//			int grid[][GRID_WIDTH]		栅格
//return:
//discribe:将道路边填入栅格数据
//***********************************************************************************************
void fill_edge_to_grid_map(COOR2 *edge, int edge_num, int grid[][GRID_WIDTH])
{
	int i;
	int x;
	int y;
	for (i=0; i<edge_num; i++)
	{
		x = edge[i].x / GRID_LEN_PER_CELL + g_grid_center.x;
		y = edge[i].y / GRID_LEN_PER_CELL + g_grid_center.y;
		if (x < 0 || x > GRID_WIDTH - 1 || y < 0 || y > GRID_HEIGHT - 1)
			continue;

		grid[y][x] = 1;
	}
}

/*==================================================================
 * 函数名  ：	void get_gridmap(PL_FUNC_INPUT *pl_input)
 * 功能    ：	读取雷达栅格数据，并做预处理。输入数据意义参见协议，在规划模块中，0表示可通行，8表示静态障碍物，9表示动态障碍物
 * 输入参数：	PL_FUNC_INPUT *pl_input		环境数据输入
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void get_gridmap(PL_FUNC_INPUT *pl_input)
{
	memset(g_grid_map, 0, GRID_WIDTH * GRID_HEIGHT * sizeof(int));
	int i,j;
	int cell, tmp;

	g_grid_center.x = pl_input->fu_pl_data.gridmap.center.x;
	g_grid_center.y = pl_input->fu_pl_data.gridmap.center.y;

	//[@@等雷达模块填上这个值后，这里以后要注释掉]
	g_grid_center.x = 80;
	g_grid_center.y = 80;
	//[@@这里以后要注释掉]

	g_grid_dist = (GRID_HEIGHT - g_grid_center.y) * GRID_LEN_PER_CELL;


	for (i = 0; i < H_GRID_MAP_64; i++)
	{
		for (j = 0; j < W_GRID_MAP_64; j++)
		{
			cell = pl_input->fu_pl_data.gridmap.grid[i][j];

			tmp = (cell & 0xF0) >> 4;
			if (tmp == UNOCCUPIED)
			{
				g_grid_map[i][j * 2] = 0;
			}
			else if (tmp == OCCUPIEDDYNAMIC)
			{
				g_grid_map[i][j * 2] = 9;
			}
			else
			{
				g_grid_map[i][j * 2] = 8;
			}

			tmp = (cell & 0x0F);
			if (tmp == UNOCCUPIED)
			{
				g_grid_map[i][j * 2 + 1] = 0;
			}
			else if (tmp == OCCUPIEDDYNAMIC)
			{
				g_grid_map[i][j * 2 + 1] = 9;
			}
			else
			{
				g_grid_map[i][j * 2 + 1] = 8;
			}
		}
	}

	// 	if (g_stat == S_LEFT || 
	// 		g_stat == S_RIGHT || 
	// 		g_stat == S_STRAIGHT || 
	// 		g_stat == S_UTURN|| 
	// 		g_navi_state == ROVING)
	// 	{
	// 		if (g_navi_state == ROVING)
	// 		{
	// 			fill_edge_to_grid_map(g_natural_boundary.l_boundary, g_natural_boundary.l_nums, g_grid_map);
	// 			fill_edge_to_grid_map(g_natural_boundary.r_boundary, g_natural_boundary.r_nums, g_grid_map);
	// 		}
	// 
	// 		grid_operation(g_grid_map, GRID_WIDTH, GRID_HEIGHT, g_closed_grid_map);
	// 	}

	// 	fstream out;
	// 	out.open("E:\\grid.txt", ios::out);
	// 	for (i = GRID_HEIGHT - 1; i >=0; i--)
	// 	{
	// 		for (j = 0; j < GRID_WIDTH; j++)
	// 		{
	// 			out << g_closed_grid_map[i][j];
	// 		}
	// 		out << "\n";
	// 	}
	// 	out.close();

	//[输出栅格到文本的测试]
	// 	int temp_grid[GRID_HEIGHT][GRID_WIDTH];
	// 	memcpy(temp_grid, g_grid_map, GRID_WIDTH * GRID_HEIGHT * sizeof(int));
	// 	grid_close_operation(temp_grid, GRID_WIDTH, GRID_HEIGHT, 0, g_grid_map);

}

void get_hd_gridmap(PL_FUNC_INPUT *pl_input)
{
	memset(g_hd_grid_map, 0, GRID_WIDTH_HD * GRID_HEIGHT_HD * sizeof(int));
	int i, j;
	int cell, tmp;

	g_hd_grid_center.x = pl_input->fu_pl_data.hdmap.center.x;
	g_hd_grid_center.y = pl_input->fu_pl_data.hdmap.center.y;

	//[@@等雷达模块填上这个值后，这里以后要注释掉]
	g_hd_grid_center.x = 120;
	g_hd_grid_center.y = 0;
	//[@@这里以后要注释掉]

	for (i = 0; i < H_GRID_MAP_HD; i++)
	{
		for (j = 0; j < W_GRID_MAP_HD; j++)
		{
			cell = pl_input->fu_pl_data.hdmap.grid[i][j];

			tmp = (cell & 0xF0) >> 4;
			if (tmp == UNOCCUPIED)
			{
				g_hd_grid_map[i][j * 2] = 0;
			}
			else
			{
				g_hd_grid_map[i][j * 2] = 1;
			}

			tmp = (cell & 0x0F);
			if (tmp == UNOCCUPIED)
			{
				g_hd_grid_map[i][j * 2 + 1] = 0;
			}
			else
			{
				g_hd_grid_map[i][j * 2 + 1] = 1;
			}
		}
	}

	// 	if (g_stat == S_LEFT || 
	// 		g_stat == S_RIGHT || 
	// 		g_stat == S_STRAIGHT || 
	// 		g_stat == S_UTURN|| 
	// 		g_navi_state == ROVING)
	// 	{
	// 		if (g_navi_state == ROVING)
	// 		{
	// 			fill_edge_to_grid_map(g_natural_boundary.l_boundary, g_natural_boundary.l_nums, g_grid_map);
	// 			fill_edge_to_grid_map(g_natural_boundary.r_boundary, g_natural_boundary.r_nums, g_grid_map);
	// 		}
	// 
	// 		grid_operation(g_grid_map, GRID_WIDTH, GRID_HEIGHT, g_closed_grid_map);
	// 	}

	// 	fstream out;
	// 	out.open("E:\\grid.txt", ios::out);
	// 	for (i = GRID_HEIGHT - 1; i >=0; i--)
	// 	{
	// 		for (j = 0; j < GRID_WIDTH; j++)
	// 		{
	// 			out << g_closed_grid_map[i][j];
	// 		}
	// 		out << "\n";
	// 	}
	// 	out.close();

	//[输出栅格到文本的测试]
	// 	int temp_grid[GRID_HEIGHT][GRID_WIDTH];
	// 	memcpy(temp_grid, g_grid_map, GRID_WIDTH * GRID_HEIGHT * sizeof(int));
	// 	grid_close_operation(temp_grid, GRID_WIDTH, GRID_HEIGHT, 0, g_grid_map);

}

//[障碍物到左右两边的距离]
typedef struct
{
	int l_dist;
	int r_dist;
}L_R_DIST;

//[处理剩下的栅格障碍]
//[0 从左边通行 1 从右边通行]
//***********************************************************************************************
//                                zgccmax 2013.Mar.02
//int process_rest_grid_obs(COOR2 *left_line, int left_line_num, COOR2 *right_line, int right_line_num)
//param:    COOR2 *left_line		左边
//          int left_line_num		左边描述点个数
//          COOR2 *right_line		右边
//          int right_line_num		右边描述点个数
//return:   int  0  左边  1  右边
//discribe:对无法归边的障碍物做判断，判断从左边或右边通过
//***********************************************************************************************
int process_rest_grid_obs(COOR2 *left_line, int left_line_num, COOR2 *right_line, int right_line_num)
{
	int ret = 0;
	//	int i;
	int x0, x1;
	double l_sum, r_sum, l_avg, r_avg, l_var, r_var, l_min, r_min;

	vector<Grid_Obs>::iterator it;
	vector<L_R_DIST> l_r_dist;
	vector<L_R_DIST>::iterator _it;

	l_min = 10000;
	r_min = 10000;
	l_sum = 0;
	r_sum = 0;

	//[统计所剩下障碍物左边和右边的空余距离总和]
	for (it = grid_obs.begin(); it < grid_obs.end(); it++)
	{
		QUAD temp_quad;
		temp_quad = (*it).quad;
		get_min_max(&temp_quad, left_line, left_line_num, &x0, NULL, car_len);
		get_min_max(&temp_quad, right_line, right_line_num, NULL, &x1, car_len);
		x1 = -x1;

		L_R_DIST temp;
		temp.l_dist = x0;
		temp.r_dist = x1;
		l_r_dist.push_back(temp);

		if (l_min > x0)
			l_min = x0;

		if (r_min > x1)
			r_min = x1;

		l_sum += x0;
		r_sum += x1;
	}

	//[计算平均值和方差，进行判断]
	l_avg = l_sum / grid_obs.size();
	r_avg = r_sum / grid_obs.size();

	l_sum = 0;
	r_sum = 0;
	for (_it = l_r_dist.begin(); _it < l_r_dist.end(); _it++)
	{
		L_R_DIST temp;
		temp = *_it;

		l_sum += (temp.l_dist - l_avg) * (temp.l_dist - l_avg);
		r_sum += (temp.r_dist - r_avg) * (temp.r_dist - r_avg);
	}
	l_var = l_sum / (l_r_dist.size() - 1);
	r_var = r_sum / (l_r_dist.size() - 1);

	if (l_min > r_min)
	{
		ret = 0;
	}
	else if (l_min < r_min)
	{
		ret = 1;
	}
	else if (l_avg > r_avg)
	{
		ret = 0;
	}
	else if (l_avg < r_avg)
	{
		ret = 1;
	}
	else if (l_var <= r_var)
	{
		ret = 0;
	}
	else
	{
		ret = 1;
	}

	return ret;
}

//***********************************************************************************************
//                                zgccmax 2012.Aug.11
//int get_road_wide(COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE], int &left_line_num, 
//					COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE], int &right_line_num)
//param:    COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE]		道路左边
//			int &left_line_num										道路左边数目
//			COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE]		道路右边
//			int &right_line_num										道路右边数目
//return:   void
//discribe: 得到道路宽度
//***********************************************************************************************
int get_road_wide(COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE], int &left_line_num, 
				  COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE], int &right_line_num)
{
	int i = 0;
	int wide = 0;
	double sum = 0;
	double avg = 0;
	COOR2 temp_left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int temp_left_line_num = 0;
	COOR2 temp_right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int temp_right_line_num = 0;

#ifdef MBUG_OPEN_
	MBUG("get road wide : 2\n");
	MBUG("left_line_num : %d\n", left_line_num);
	for (i=0;i<left_line_num;i++)
	{
		MBUG("(%d, %d) ", left_line[i].x, left_line[i].y);
	}
	MBUG("\n");
	MBUG("right_line_num : %d\n", right_line_num);
	for (i=0;i<right_line_num;i++)
	{
		MBUG("(%d, %d) ", right_line[i].x, right_line[i].y);
	}
	MBUG("\n");
#endif

	if (left_line_num < 2 || right_line_num < 2)
	{
		return 0;
	}

	if (left_line[0].x == left_line[1].x && \
		left_line[0].y == left_line[1].y && \
		right_line[0].x == right_line[1].x && \
		right_line[0].y == right_line[1].y)
	{
		return 0;
	}

	memset(temp_left_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	memset(temp_right_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	memcpy(temp_left_line, left_line, left_line_num * sizeof(COOR2));
	memcpy(temp_right_line, right_line, right_line_num * sizeof(COOR2));
	temp_left_line_num = left_line_num;
	temp_right_line_num = right_line_num;

//	fflush(NULL);
	fix_long_short(temp_left_line, &temp_left_line_num, temp_right_line, &temp_right_line_num);

#ifdef MBUG_OPEN_
	MBUG("wide sum temp_left_line_num : %d\n", temp_left_line_num);
#endif
	for (i=0; i<temp_left_line_num; i++)
	{
		sum += fabs(temp_left_line[i].x - temp_right_line[i].x + 0.0);
	}
	avg = sum / temp_left_line_num;

	wide = (int)avg;
	return wide;
}

//***********************************************************************************************
//                                zgccmax 2012.Aug.11
//void cut_the_long_edge(COOR2 *left_line, int *left_line_num, COOR2 *right_line, int *right_line_num)
//param:    COOR2 *left_line
//			int *left_line_num
//			COOR2 *right_line
//			int *right_line_num
//return:   void
//discribe: 修剪两条道路边至一样长
//***********************************************************************************************
void cut_the_long_edge(COOR2 *left_line, int *left_line_num, COOR2 *right_line, int *right_line_num)
{
	int i;
	if (left_line[*left_line_num - 1].y > right_line[*right_line_num - 1].y)
	{
		i = 0;
		while (left_line[i].y < right_line[*right_line_num - 1].y)
			i++;

		left_line[i].x = left_line[i - 1].x;
		left_line[i].y = right_line[*right_line_num - 1].y;
		*left_line_num = i + 1;
	}
	else if (left_line[*left_line_num - 1].y < right_line[*right_line_num - 1].y)
	{
		i = 0;
		while (right_line[i].y < left_line[*left_line_num - 1].y)
			i++;

		right_line[i].x = right_line[i - 1].x;
		right_line[i].y = left_line[*left_line_num - 1].y;
		*right_line_num = i + 1;
	}

}

//[补丁函数：作用是修正规划线从近处预瞄到远处一个安全的点]
/*==================================================================
 * 函数名  ：	void fix_the_mid_line()
 * 功能    ：	使取中生成的规划线预瞄到远处一个无碰点
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void fix_the_mid_line()
{
	int i, j;
	int x0;
	int x1;
	int sign;

	COOR2 temp_line[200];
	int temp_line_num = 0;
	COOR2 temp_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int temp_mid_line_num = 0;
	int step;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;

	for (i = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1; i>1; i--)
	{
		memset(temp_mid_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		temp_mid_line_num = 0;
		temp_mid_line[0] = g_mid_line[0];
		temp_mid_line[1] = g_mid_line[1];
		memcpy(temp_mid_line + 2, g_mid_line + i, (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - i) * sizeof(COOR2));
		temp_mid_line_num = 2 + (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - i);
		step = (int)(temp_mid_line[temp_mid_line_num - 1].y - temp_mid_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
		line_fitting(temp_mid_line, temp_mid_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(temp_mid_line, temp_line, temp_line_num * sizeof(COOR2));
		temp_mid_line_num = temp_line_num;

		sign = 1;

		for (j=0; j<i; j++)
		{
			get_x_coord(temp_mid_line[j].y, g_left_line, g_left_line_num, &x0);
			get_x_coord(temp_mid_line[j].y, g_right_line, g_right_line_num, &x1);

			//[由于膨胀了75cm，那么还有一定冗余，这里放开40cm]
			if (g_navi_state != TRACE_IN_NATURAL_ROAD)
			{
				if (temp_mid_line[j].x - x0 < (car_wid / 2) || \
					x1 - temp_mid_line[j].x < (car_wid / 2))
				{
					sign = 0;
					break;
				}
			}
			else
			{
				if (temp_mid_line[j].y > 900)
				{
					if (temp_mid_line[j].x - x0 < car_wid / 2 + 10 || \
						x1 - temp_mid_line[j].x < car_wid / 2 + 10)
					{
						sign = 0;
						break;
					}
				}
				else
				{
					if (temp_mid_line[j].x - x0 < car_wid / 2 - 20 || \
						x1 - temp_mid_line[j].x < car_wid / 2 - 20)
					{
						sign = 0;
						break;
					}
				}
			}
		}

		if (sign == 1)
		{
			memcpy(g_mid_line, temp_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			break;
		}
	}
}

//[此处用于最后输出。寻找最紧急的距离，进行减速判定]
/*==================================================================
 * 函数名  ：	void fix_the_mid_line_2()
 * 功能    ：	此处用于最后输出。寻找最紧急的距离，进行减速判定
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void fix_the_mid_line_2()
{
	int i, j;
	int x0;
	int x1;
	int sign;

	COOR2 temp_line[200];
	int temp_line_num = 0;
	COOR2 temp_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int temp_mid_line_num = 0;
	int step;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;

	int width = MAX_VALUE;
	g_slow_down_dist = MAX_VALUE;

	for (i = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1; i>4; i--)
	{
		memset(temp_mid_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		temp_mid_line_num = 0;

		//[0-400cm直接复制，只处理后面的数据]
		temp_mid_line[0] = g_mid_line[0];
		temp_mid_line[1] = g_mid_line[1];
		temp_mid_line[2] = g_mid_line[2];
		temp_mid_line[3] = g_mid_line[3];
		temp_mid_line[4] = g_mid_line[4];

		memcpy(temp_mid_line + 5, g_mid_line + i, (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - i) * sizeof(COOR2));
		temp_mid_line_num = 5 + (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - i);
		step = (int)(temp_mid_line[temp_mid_line_num - 1].y - temp_mid_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
		line_fitting(temp_mid_line, temp_mid_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(temp_mid_line, temp_line, temp_line_num * sizeof(COOR2));
		temp_mid_line_num = temp_line_num;

		sign = 1;

		for (j=0; j<i; j++)
		{
			get_x_coord(temp_mid_line[j].y, g_left_line, g_left_line_num, &x0);
			get_x_coord(temp_mid_line[j].y, g_right_line, g_right_line_num, &x1);

			width = x1 - x0;
			//[由于膨胀了75cm，那么还有一定冗余，这里放开40cm]
			if (g_navi_state != TRACE_IN_NATURAL_ROAD)
			{
				if (temp_mid_line[j].x - x0 < (car_wid / 2) || \
					x1 - temp_mid_line[j].x < (car_wid / 2))
				{
					sign = 0;
					break;
				}
			}
			else
			{
				if (temp_mid_line[j].y > 900)
				{
					if (temp_mid_line[j].x - x0 < car_wid / 2 + 10 || \
						x1 - temp_mid_line[j].x < car_wid / 2 + 10)
					{
						sign = 0;
						break;
					}
				}
				else
				{
					if (temp_mid_line[j].x - x0 < car_wid / 2 - 20 || \
						x1 - temp_mid_line[j].x < car_wid / 2 - 20)
					{
						sign = 0;
						break;
					}
				}
			}
		}

		if (sign == 1)
		{
			if (width <= 300)
			{
				g_slow_down_dist = temp_mid_line[i].y;
			}
			memcpy(g_mid_line, temp_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			break;
		}
	}

	//[求取最紧急的地方]
	for (i=0; i<g_left_line_num; i++)
	{
		width = g_right_line[i].x - g_left_line[i].x;

		if (width <= 300)
		{
			g_slow_down_dist = temp_mid_line[i].y;
			break;
		}
	}
}

//[用来修补压线时，规划线]
/*==================================================================
 * 函数名  ：	void fix_the_mid_line_3()
 * 功能    ：	在车子处于车道线上方，用于修正取中规划线导致行驶角度过大的问题
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void fix_the_mid_line_3()
{
	int i, j;
	COOR2 temp_line[200];
	int temp_line_num = 0;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;

	g_mid_line[0].x = 0;
	g_mid_line[0].y = 400;
	g_mid_line[1].x = 0;
	g_mid_line[1].y = 500;

	i=0;
	while (g_left_line[i].y <= 1000)
	{//[从车头600cm处开始规划]
		i++;
	}

	j=i;//[规划线从第三个点给起]
	for (; i<g_left_line_num; i++)
	{
		g_mid_line[j].x = (g_left_line[i].x + g_right_line[i].x) / 2;
		g_mid_line[j++].y = g_left_line[i].y;

		if (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE == j)
		{
			break;
		}
	}

	for (i=2;i<j; i++)
	{//[从第三个规划点开始]

		double res = abs(g_mid_line[i].x) / (abs(g_mid_line[i].y - 400) + 0.0);
		//[控制最大的偏角小于8.5度，这个可以当做经验参数，根据实验情况进行调节]
		//if (res < 0.149)
		if (res < g_change_steer_limit)
			break;
	}
	if (i==j)
	{
		memmove(g_mid_line + 2, g_mid_line + j - 1, sizeof(COOR2));
		j = 3;
	}
	else
	{
		memmove(g_mid_line + 2, g_mid_line + i, (j - i) * sizeof(COOR2));
		j = j - (i - 2);
	}

	g_long_line_num = j;
	
	if (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y < 2000)
	{
		int step = (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y - g_mid_line[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
		line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		get_bezier_line(temp_line, temp_line_num, g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	}
	else
	{
		line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, 100, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		get_bezier_line(temp_line, temp_line_num, g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	}

	g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
	g_fidelity = 1;
}

//[用来修补超车后，规划线]
/*==================================================================
 * 函数名  ：	void fix_the_mid_line_4()
 * 功能    ：	用来平滑超车后规划线
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void fix_the_mid_line_4()
{
	COOR2 temp_line[200];
	int temp_line_num = 0;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;

	g_mid_line[0].x = 0;
	g_mid_line[0].y = 400;
	g_mid_line[1].x = 0;
	g_mid_line[1].y = 500;

	g_mid_line[2].x = g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x;
	g_mid_line[2].y = g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y;

	g_long_line_num = 3;
	
	if (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y < 2000)
	{
		int step = (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y - g_mid_line[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
		line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	}
	else
	{
		line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, 100, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	}

	get_bezier_line(temp_line, temp_line_num, g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
	g_fidelity = 1;
}

//[用来修补换道，规划线]
/*==================================================================
 * 函数名  ：	void fix_the_mid_line_5()
 * 功能    ：	用来平滑换道后的规划线
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void fix_the_mid_line_5()
{
	int i, j;
	int x0;
	int x1;
	int sign;
	int index;

	COOR2 temp_line[200];
	int temp_line_num = 0;
	COOR2 temp_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int temp_mid_line_num = 0;
	int step;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;


		if (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y < 2000)
	{
		int step = (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y - g_mid_line[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
		line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	}
	else
	{
		line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, 100, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	}

	memcpy(g_mid_line, temp_line, temp_line_num * sizeof(COOR2));


	index = -1;
	//[1.找到预瞄角度大于限制的点]
	for (i=4;i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;i++)
	{
		double res = abs(g_mid_line[i].x / (g_mid_line[i].y - 400.0));
		if (res < g_change_steer_limit)
		{
			index = i;
			break;
		}
	}

	if (index == -1)
	{
		index = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1;
	}
	//[2.遍历并找到无碰撞预瞄点]
	for (i = index; i>3; i--)
	{
		memset(temp_mid_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		temp_mid_line_num = 0;

		temp_mid_line[0] = g_mid_line[0];
		temp_mid_line[1] = g_mid_line[1];
		temp_mid_line[2] = g_mid_line[2];
		temp_mid_line[3] = g_mid_line[3];

		memcpy(temp_mid_line + 4, g_mid_line + i, (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - i) * sizeof(COOR2));
		temp_mid_line_num = 4 + (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - i);
		step = (int)(temp_mid_line[temp_mid_line_num - 1].y - temp_mid_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
		line_fitting(temp_mid_line, temp_mid_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(temp_mid_line, temp_line, temp_line_num * sizeof(COOR2));
		temp_mid_line_num = temp_line_num;

		sign = 1;

		for (j=2; j<i; j++)
		{
			get_x_coord(temp_mid_line[j].y, g_left_line, g_left_line_num, &x0);
			get_x_coord(temp_mid_line[j].y, g_right_line, g_right_line_num, &x1);

			//[由于膨胀了75cm，那么还有一定冗余，这里放开40cm]
			if (g_navi_state != TRACE_IN_NATURAL_ROAD)
			{
				if (temp_mid_line[j].x - x0 < (car_wid / 2) || \
					x1 - temp_mid_line[j].x < (car_wid / 2))
				{
					sign = 0;
					break;
				}
			}
			else
			{
				if (temp_mid_line[j].x - x0 < car_wid / 2 + 50|| \
					x1 - temp_mid_line[j].x < car_wid / 2 + 50)
				{
					sign = 0;
					break;
				}
			}
		}

		if (sign == 1)
		{
			memcpy(g_mid_line, temp_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			break;
		}
	}
}

//[使用长边进行修补换道规划线]
/*==================================================================
 * 函数名  ：	void fix_the_mid_line_6()
 * 功能    ：	用来平滑换道后的规划线
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void fix_the_mid_line_6()
{
	int i, j;
	int x0;
	int x1;
	int sign;
	int index;

	COOR2 temp_line[200];
	int temp_line_num = 0;
	COOR2 temp_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int temp_mid_line_num = 0;
	int step;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;

// 	line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, 100, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
// 	memcpy(g_mid_line, temp_line, temp_line_num * sizeof(COOR2));


	index = -1;
	for (i=4;i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;i++)
	{
		double res = abs(g_mid_line[i].x / (g_mid_line[i].y - 400.0));
		if (res < g_change_steer_limit)
		{
			index = i;
			break;
		}
	}

	if (index == -1)
	{
		index = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1;
	}
	for (i = index; i>3; i--)
	{
		memset(temp_mid_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		temp_mid_line_num = 0;

		temp_mid_line[0] = g_mid_line[0];
		temp_mid_line[1] = g_mid_line[1];

		memcpy(temp_mid_line + 2, g_mid_line + i, (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - i) * sizeof(COOR2));
		temp_mid_line_num = 2 + (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - i);
		step = (int)(temp_mid_line[temp_mid_line_num - 1].y - temp_mid_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
		line_fitting(temp_mid_line, temp_mid_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(temp_mid_line, temp_line, temp_line_num * sizeof(COOR2));
		temp_mid_line_num = temp_line_num;

		sign = 1;

		for (j=2; j<i; j++)
		{
			//get_x_coord(temp_mid_line[j].y, g_left_line, g_left_line_num, &x0);
			//get_x_coord(temp_mid_line[j].y, g_right_line, g_right_line_num, &x1);
			get_x_coord(g_left_line[j].y, temp_mid_line, temp_mid_line_num, &x0);
			get_x_coord(g_right_line[j].y, temp_mid_line, temp_mid_line_num, &x1);

			if (g_navi_state == CHANGE_TO_LEFT_LANE || g_navi_state == CHANGE_TO_RIGHT_LANE)
			{
				if (x0 - g_left_line[j].x < car_wid / 2 + 50|| \
					g_right_line[j].x - x1 < car_wid / 2 + 50)
				{
					sign = 0;
					break;
				}
			}
		}

		if (sign == 1)
		{
			memcpy(g_mid_line, temp_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			break;
		}
	}
}

//[让换道规划线预瞄角度大点]
void fix_the_mid_line_7()
{
	int sign = 0;
	int i = 1;
	int xx = 0;

	COOR2 temp_line[200];
	int temp_line_num = 0;
	COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];

	memset(temp_line, 0, 200 * sizeof(COOR2));
	memset(left_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	memset(right_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));

	while (sign != 2 && i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
	{
		xx = g_mid_line[i].x - g_mid_line[i - 1].x;
		if (xx < 0)
		{
			if (sign == 0)
			{
				sign = -1;
			}
			else if (sign == -1)
			{
				sign = -1;
			}
			else
			{
				sign = 2;
			}
		}
		else if (xx > 0)
		{
			if (sign == 0)
			{
				sign = 1;
			}
			else if (sign == 1)
			{
				sign = 1;
			}
			else
			{
				sign = 2;
			}
		}
		else
			;

		i++;
	}

	if (sign != 0)
	{
		g_long_line_num = i;

		if (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y < 2000)
		{
			int step = (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y - g_mid_line[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
			line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		}
		else
		{
			line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, 100, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		}
		get_bezier_line(temp_line, temp_line_num, g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
	}
}

/*==================================================================
 * 函数名  ：	int check_the_lane(COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE], int &left_line_num,
									 COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE], int &right_line_num)
 * 功能    ：	将障碍物进行归边，并检测道路可通行性。根据不同的子任务和全局状态，检测策略不同。
 * 输入参数：	COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE]		道路左边
				int &left_line_num										道路左边数目
				COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE]		道路右边
				int &right_line_num										道路右边数目
 * 输出参数：	
 * 返回值  ：	int		0 clear    -1 block    -2 no edge
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int check_the_lane(COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE], int &left_line_num, 
				   COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE], int &right_line_num)
{
	int i = 0;
	int j = 0;
	int l_index = 0;
	int r_index = 0;
	int x = 0;
	int l_grid_map[GRID_HEIGHT][GRID_WIDTH];
	int ret = 0;
	int wide = 0;

	if (left_line_num < 2 || right_line_num < 2)
	{
		ret = -2;
		return ret;
	}

#ifdef MBUG_OPEN_
	MBUG("get road wide :1\n");
#endif
	wide = get_road_wide(left_line, left_line_num, right_line, right_line_num);


	COOR2 temp_left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	COOR2 temp_right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];

	memset(temp_left_line, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memset(temp_right_line, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memcpy(temp_left_line, left_line, left_line_num * sizeof(COOR2));
	memcpy(temp_right_line, right_line, right_line_num * sizeof(COOR2));

	COOR2 org_left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int org_left_line_num;
	COOR2 org_right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int org_right_line_num;
	memcpy(org_left_line, temp_left_line, left_line_num * sizeof(COOR2));
	memcpy(org_right_line, temp_right_line, right_line_num * sizeof(COOR2));
	org_left_line_num = left_line_num;
	org_right_line_num = right_line_num;


	memset(l_grid_map, 0, GRID_WIDTH * GRID_HEIGHT * sizeof(int));
	memcpy(l_grid_map, g_grid_map, GRID_WIDTH * GRID_HEIGHT * sizeof(int));

	grid_obs.clear();


	//[只处理到g_grid_dits]
	if (g_navi_state == TRACE_IN_NATURAL_ROAD)
	{//[跟随自然道边使用自然道边长度参数]
		g_grid_dist = g_natural_look_dist;
	}
	else if (g_stat == S_CROSS_UND)
	{//[跟随结构化道路的情况下]
		if (g_road_type == 0)
		{
			g_grid_dist = 4000;
		}
		else
		{
			if (g_fusion_structure_road_length > 0)
				g_grid_dist = g_fusion_structure_road_length;
			else
				g_grid_dist = 4000;
		}
	}
	else
	{
		if (g_road_type == 0)
		{
			g_grid_dist = FU_ROAD_LENGTH;
		}
		else
		{
			g_grid_dist = g_fusion_structure_road_length;
		}
	}

	i = 0;
	while (temp_left_line[i].y < g_grid_dist)
	{
		i++;
		if (i >= NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
			break;
	}

	int top_index =  temp_left_line[i - 1].y / GRID_LEN_PER_CELL + g_grid_center.y;


	//[由近及远提取道路中间栅格障碍物，注意栅格中心位置]
	for (i = g_grid_center.y + 16; i < top_index; i++)
	{
		get_x_coord((i - g_grid_center.y) * GRID_LEN_PER_CELL, temp_left_line, left_line_num, &x);

		l_index = x / GRID_LEN_PER_CELL + g_grid_center.x;

		get_x_coord((i - g_grid_center.y) * GRID_LEN_PER_CELL, temp_right_line, right_line_num, &x);

		r_index = x / GRID_LEN_PER_CELL + g_grid_center.x;

		for (j = l_index; j <= r_index; j++)
		{
			if (g_grid_map[i][j] == 1 || g_grid_map[i][j] == 8)//[因为从ROVING状态切换过来障碍物是8]
			{
				g_grid_map[i][j] = 0;
				Grid_Obs temp_obs;

				temp_obs.x = j;
				temp_obs.y = i;

				//[障碍物由栅格转到车体局部坐标系]
				if (temp_obs.x <= g_grid_center.x)
				{
					temp_obs.quad.coor2[0].x = (temp_obs.x - g_grid_center.x) * GRID_LEN_PER_CELL - GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[1].x = (temp_obs.x - g_grid_center.x) * GRID_LEN_PER_CELL - GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[2].x = (temp_obs.x - g_grid_center.x) * GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[3].x = (temp_obs.x - g_grid_center.x) * GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[0].y = (temp_obs.y - g_grid_center.y) * GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[1].y = (temp_obs.y - g_grid_center.y) * GRID_LEN_PER_CELL + GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[2].y = (temp_obs.y - g_grid_center.y) * GRID_LEN_PER_CELL + GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[3].y = (temp_obs.y - g_grid_center.y) * GRID_LEN_PER_CELL;
				}
				else
				{
					temp_obs.quad.coor2[0].x = (temp_obs.x - g_grid_center.x) * GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[1].x = (temp_obs.x - g_grid_center.x) * GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[2].x = (temp_obs.x - g_grid_center.x) * GRID_LEN_PER_CELL + GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[3].x = (temp_obs.x - g_grid_center.x) * GRID_LEN_PER_CELL + GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[0].y = (temp_obs.y - g_grid_center.y) * GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[1].y = (temp_obs.y - g_grid_center.y) * GRID_LEN_PER_CELL + GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[2].y = (temp_obs.y - g_grid_center.y) * GRID_LEN_PER_CELL + GRID_LEN_PER_CELL;
					temp_obs.quad.coor2[3].y = (temp_obs.y - g_grid_center.y) * GRID_LEN_PER_CELL;
				}

				grid_obs.push_back(temp_obs);
			}//[end if]
		}//[end for j]
	}//[end for i]

	int last_num = 0;
	vector<Grid_Obs>::iterator it;

	int x0, x1;
	int xx;
#ifdef MBUG_OPEN_
	MBUG("total grid obs num = %d\n", grid_obs.size());
#endif
	//[遍历所有栅格障碍进行左右归边]
	g_has_mid_obs = 0;
	g_left_has_big_obs = 0;
	g_left_big_obs_dist = MAX_VALUE;
	g_right_has_big_obs = 0;
	g_right_big_obs_dist = MAX_VALUE;
	do 
	{
		last_num = (int)grid_obs.size();

		for (it=grid_obs.begin(); it<grid_obs.end();it++)
		{

			QUAD temp_quad;
			temp_quad = (*it).quad;

			get_min_max(&temp_quad, temp_left_line, left_line_num, &x0, NULL, car_len);
			get_min_max(&temp_quad, temp_right_line, right_line_num, NULL, &x1, car_len);
			x1 = -x1;
			xx = x0 - x1;

			if (xx < 0)
			{
				//[归左边]
				if (x0 < car_wid)
				{
					int xx = 0;
					get_x_coord(temp_quad.coor2[0].y, org_left_line, org_left_line_num, &xx);
					if ((temp_quad.coor2[0].x - xx) > 120 && temp_quad.coor2[0].y <= 2500)
					{
						g_has_mid_obs = 1;
						g_left_has_big_obs = 1;
						if (g_left_big_obs_dist > temp_quad.coor2[0].y)
							g_left_big_obs_dist = temp_quad.coor2[0].y;
					}

					set_obs_to_line(&temp_quad, temp_left_line, &left_line_num, 0);
					grid_obs.erase(it);
					break;
				}
			}
			else
			{
				//[归右边]
				if (x1 < car_wid)
				{
					int xx = 0;
					get_x_coord(temp_quad.coor2[1].y, org_right_line, org_right_line_num, &xx);
					if ((xx - temp_quad.coor2[0].x) > 120 && temp_quad.coor2[0].y <= 2500)
					{
						g_has_mid_obs = 1;
						g_right_has_big_obs = 1;
						if (g_right_big_obs_dist > temp_quad.coor2[0].y)
							g_right_big_obs_dist = temp_quad.coor2[0].y;
					}

					set_obs_to_line(&temp_quad, temp_right_line, &right_line_num, 1);
					grid_obs.erase(it);
					break;
				}
			}
		}
	} while ((int)grid_obs.size() < last_num);
#ifdef MBUG_OPEN_
	MBUG("left_line_num : %d\n", left_line_num);
	MBUG("right_line_num : %d\n", right_line_num);
	MBUG("the rest obs num = %d\n", last_num);
#endif
	if (last_num > 0)
	{
		ret = process_rest_grid_obs(temp_left_line, left_line_num, temp_right_line, right_line_num);
		if (ret == 0)
		{
			//[全部归到右边]
			for (it = grid_obs.begin(); it < grid_obs.end(); it++)
			{
				QUAD temp_quad;
				temp_quad = (*it).quad;
				set_obs_to_line(&temp_quad, temp_right_line, &right_line_num, 1);
			}
		}
		else
		{
			//[全部归到左边]
			for (it=grid_obs.begin();it<grid_obs.end();it++)
			{
				QUAD temp_quad;
				temp_quad = (*it).quad;
				set_obs_to_line(&temp_quad, temp_left_line, &left_line_num, 0);
			}
		}
	}	

	memcpy(g_grid_map, l_grid_map, GRID_WIDTH * GRID_HEIGHT * sizeof(int));

	ret = 0;
#ifdef MBUG_OPEN_
	MBUG("the edge after grid_obs process\n");
	MBUG("left_line_num : %d\n", left_line_num);
	for (i=0;i<left_line_num;i++)
	{
		MBUG("(%d,%d) ", temp_left_line[i].x, temp_left_line[i].y);
	}
	MBUG("\n");
	MBUG("right_line_num : %d\n", right_line_num);
	for (i=0;i<right_line_num;i++)
	{
		MBUG("(%d,%d) ", temp_right_line[i].x, temp_right_line[i].y);
	}
	MBUG("\n");
#endif
	cut_the_long_edge(temp_left_line, &left_line_num, temp_right_line, &right_line_num);

	COOR2 temp_line[200];
	int temp_line_num = 0;
	int step;

	//[将归边后的左右边重新拟合]
	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;
	step = (int)(temp_left_line[left_line_num - 1].y - temp_left_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
	line_fitting(temp_left_line, left_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memcpy(temp_left_line, temp_line, temp_line_num * sizeof(COOR2));
	left_line_num = temp_line_num;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;
	step = (int)(temp_right_line[right_line_num - 1].y - temp_right_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
	line_fitting(temp_right_line, right_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memcpy(temp_right_line, temp_line, temp_line_num * sizeof(COOR2));
	right_line_num = temp_line_num;

#ifdef MBUG_OPEN_
	MBUG("the edge after fitting\n");
	MBUG("left_line_num : %d\n", left_line_num);
	for (i=0;i<left_line_num;i++)
	{
		MBUG("(%d,%d) ", temp_left_line[i].x, temp_left_line[i].y);
	}
	MBUG("\n");
	MBUG("right_line_num : %d\n", right_line_num);
	for (i=0;i<right_line_num;i++)
	{
		MBUG("(%d,%d) ", temp_right_line[i].x, temp_right_line[i].y);
	}
	MBUG("\n");
#endif

	//[道路边多长，碰撞检测距离就到多远]
	g_grid_dist = temp_right_line[right_line_num - 1].y;
	

	int temp_mid_line_num = 0;
	step = (g_grid_dist - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

	//[根据不同的环境生成规划线]
	for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
	{
		g_mid_line[i].y = i * step + 400;

		get_x_coord(g_mid_line[i].y, temp_left_line, left_line_num, &x0);
		get_x_coord(g_mid_line[i].y, temp_right_line, right_line_num, &x1);

		xx = x1 - x0;

		if (g_navi_state == TRACE_IN_NATURAL_ROAD)
		{
			if (g_stat == S_CROSS_UND || g_stat == S_TASK_OVER)
			{
				if (xx >= car_wid * 2 + mini_pass * 4)
				{
					g_mid_line[i].x = (x0 + x1) / 2;

					xx = x1 - g_mid_line[i].x;
					if (xx < car_wid / 2)
					{
						g_mid_line[i].x -= (car_wid / 2 - xx);
					}
				}
				else if (xx >= car_wid + mini_pass * 2)
				{
					g_mid_line[i].x = (x0 + x1) / 2;
				}
				else
				{
					g_mid_line[i].x = (x0 + x1) / 2;
				}
			}
			else
			{

			if (xx >= car_wid * 2 + mini_pass * 4)
			{
				g_mid_line[i].x = (x0 + x1) / 2;// +120;

				xx = x1 - g_mid_line[i].x;
				if (xx < car_wid / 2)
				{
					g_mid_line[i].x -= (car_wid / 2 - xx);
				}
			}
			else if (xx >= car_wid + mini_pass * 2)
			{
				g_mid_line[i].x = (x0 + x1) / 2;// +50;
			}
			else
			{
				g_mid_line[i].x = (x0 + x1) / 2;
			}
			}
		}
		else if (g_navi_state == CHANGE_TO_LEFT_LANE)
		{
			if (xx >= car_wid * 2 + mini_pass * 4)
			{
				g_mid_line[i].x = (x0 * 3 + 3) / 4 + (x1 + 3) / 4;

				xx = x1 - g_mid_line[i].x;
				if (xx < car_wid / 2)
				{
					g_mid_line[i].x -= (car_wid / 2 - xx);
				}
			}
			else if (xx >= car_wid + mini_pass * 2)
			{
				g_mid_line[i].x = (x0 + x1) / 2;
			}
			else
			{
				g_mid_line[i].x = (x0 + x1) / 2;
			}
		}
		else if (g_navi_state == CHANGE_TO_RIGHT_LANE)
		{
			if (xx >= car_wid * 2 + mini_pass * 4)
			{
				g_mid_line[i].x = (x0 + 3) / 4 + (x1 * 3 + 3) / 4;

				xx = x1 - g_mid_line[i].x;
				if (xx < car_wid / 2)
				{
					g_mid_line[i].x -= (car_wid / 2 - xx);
				}
			}
			else if (xx >= car_wid + mini_pass * 2)
			{
				g_mid_line[i].x = (x0 + x1) / 2;
			}
			else
			{
				g_mid_line[i].x = (x0 + x1) / 2;
			}
		}
		else
		{
			if (wide < 500)
			{
				g_mid_line[i].x = (x0 + x1) / 2;
			}
			else
			{
				g_mid_line[i].x = (x0 * 6 + 3) / 20 + (x1 * 14 + 3) / 20;
			}
		}
	}
	temp_mid_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;
	step = (int)(g_mid_line[temp_mid_line_num - 1].y - g_mid_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
	line_fitting(g_mid_line, temp_mid_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memcpy(g_mid_line, temp_line, temp_line_num * sizeof(COOR2));
	temp_mid_line_num = temp_line_num;

#ifdef MBUG_OPEN_
	MBUG("first g_mid_line :\n");
	MBUG("temp_mid_line_num : %d\n", temp_mid_line_num);
	for (i=0; i<temp_mid_line_num; i++)
	{
		MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
	}
	MBUG("\n ");
#endif
	//[处理好的路边传到全局变量，若车道通畅，则为后面进行规划提供便利]
	memcpy(g_left_line, temp_left_line, left_line_num * sizeof(COOR2));
	g_left_line_num = left_line_num;
	memcpy(g_right_line, temp_right_line, right_line_num * sizeof(COOR2));
	g_right_line_num = right_line_num;

	//[必须写在此处，上面有边的赋值，在左右边有的情况下调用]
	fix_the_mid_line();
#ifdef MBUG_OPEN_
	MBUG("fix_the_mid_line :\n");
	MBUG("temp_mid_line_num : %d\n", temp_mid_line_num);
	for (i=0; i<temp_mid_line_num; i++)
	{
		MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
	}
	MBUG("\n ");
#endif
	//[检测规划线是否可通行]
	int safe_dist = 0;
	if (g_navi_state == TRACE_IN_NATURAL_ROAD)
	{
		safe_dist = g_natural_look_dist;
		g_natural_road_search_dist = temp_left_line[temp_line_num - 1].y;//[自然道边的长度，用来控制速度]
	}
	else if (g_change_lane_search_flag == 1)
	{//[此处是多车道连续避障的检测距离]
		g_grid_dist = g_change_lane_search_dist;
	}
	else if (g_stat == S_CROSS_UND)
	{
		safe_dist = 4000;
	}

	for (i=0; i<temp_mid_line_num; i++)
	{
		get_x_coord(g_mid_line[i].y, temp_left_line, left_line_num, &x0);
		get_x_coord(g_mid_line[i].y, temp_right_line, right_line_num, &x1);

		xx = x1 - x0;

		if (g_mid_line[i].y < g_grid_dist)
		{
			//[找到最近的危险点]
			if (xx < car_wid)
			{
				safe_dist = g_mid_line[i - 1].y;
				ret = -1;
				break;
			}
		}
	}

	//[根据当前速度判断当前的最危险点是否足够远]
	if (g_navi_state == TRACE_LANE)
	{
		if (g_road_type == 0)
		{
			g_far_obs_warning_flag = 0;

			if (g_real_speed < (int)(40 * CM))
			{
				if (safe_dist > 4000 && safe_dist <= FU_ROAD_LENGTH)
				{//[速度相对较低，且障碍物所处距离较远，因此认为安全，但是要进行减速]
					g_unsafe_dist = MAX_VALUE;
					ret = 0;
					g_far_obs_warning_flag = 1;
				}
			}
		}
		else
		{
			g_far_obs_warning_flag = 0;

			if (g_real_speed < (int)(25 * CM))
			{
				if (safe_dist >= 2500 && safe_dist <= g_natural_look_dist)
				{//[速度相对较低，且障碍物所处距离较远，因此认为安全，但是要进行减速]
					//g_unsafe_dist = MAX_VALUE;
					//ret = 0;
					g_far_obs_warning_flag = 1;
				}
			}
		}

	}

	if (g_navi_state == TRACE_IN_NATURAL_ROAD && safe_dist >= 800)
	{//[乡村道路下，规划距离在10m-g_natural_look_dist间，小于8m堵塞（车头前4m），就停车]
		g_grid_dist = safe_dist;
		ret = 0;

		for (i=0; i<temp_mid_line_num; i++)
		{
			get_x_coord(g_mid_line[i].y, temp_left_line, left_line_num, &x0);
			get_x_coord(g_mid_line[i].y, temp_right_line, right_line_num, &x1);

			xx = x1 - x0;

			if (g_mid_line[i].y < g_grid_dist)
			{
				if (xx < car_wid)
				{
					safe_dist = g_mid_line[i].y;
					ret = -1;
					break;
				}
			}
		}

		i = 0;
		while (g_mid_line[i].y < g_grid_dist)
		{
			i++;
			if (i>=NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
				break;
		}
		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;
		step = (int)(g_mid_line[i - 1].y - g_mid_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
		line_fitting(g_mid_line, temp_mid_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(g_mid_line, temp_line, temp_line_num * sizeof(COOR2));
		temp_mid_line_num = temp_line_num;
	}
	else if ((g_stat == S_CROSS_UND || g_stat == S_TASK_OVER) && safe_dist >= 1000)
	{
		g_grid_dist = safe_dist;
		ret = 0;

		for (i=0; i<temp_mid_line_num; i++)
		{
			get_x_coord(g_mid_line[i].y, temp_left_line, left_line_num, &x0);
			get_x_coord(g_mid_line[i].y, temp_right_line, right_line_num, &x1);

			xx = x1 - x0;

			if (g_mid_line[i].y < g_grid_dist)
			{
				if (xx < car_wid)
				{
					safe_dist = g_mid_line[i].y;
					ret = -1;
					break;
				}
			}
		}
	}

	if (ret == 0)
	{//[可以在安全距离内安全通行]
		g_unsafe_dist = MAX_VALUE;
	}
	else
	{//[会有危险的距离]
		g_unsafe_dist = g_mid_line[i].y;
	}


	if ((temp_left_line[0].x > 150 || \
		temp_left_line[1].x > 150 || \
		temp_left_line[2].x > 150 || \
		temp_right_line[0].x < -150 || \
		temp_right_line[1].x < -150 || \
		temp_right_line[2].x < -150) && g_take_over_flag == 0)
	{//[防止边界跑到外侧，然而要超车的时候看旁边车道不用这样考虑]
		ret = -1;
	}


	g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;

	return ret;
}


//***********************************************************************************************
//                                zgccmax 2013.Mar.03
//void keep_trace_in_the_mid_lane()
//param:
//return:
//discribe:检测到多车道，尽量在中间车道行驶
//***********************************************************************************************
void keep_trace_in_the_mid_lane()
{
	//[两个车道以下，包括两车道，不换道]
	if (g_natural_boundary.l_nums >= 2 && g_natural_boundary.r_nums >= 2 \
		&& g_natural_boundary.l_fidelity >= 90 && g_natural_boundary.r_fidelity >= 90)
	{
		int width = g_natural_boundary.r_boundary[0].x - g_natural_boundary.l_boundary[0].x;
		//[使用自然道边作为约束，估计有几个车道]
		if (width <= 850)
		{
			g_take_over_back_lane_flag = -1;
			return;
		}
	}

	//[以下为有三车道以上情况，包括三车道]
	if (g_multi_lane.lane_line[1].line_color == YELLOW)
	{//[左边线是黄线]
		if (g_multi_lane.lane_line[2].valid_num_points >= 2 && g_multi_lane.lane_line[3].valid_num_points >= 2)
		{//[有右边车道]
			if (g_multi_lane.lane_line[3].line[0].x < 700)
			{//[保证右侧车道只有一个车道宽]
				if (g_natural_boundary.r_nums >= 2 && g_natural_boundary.r_fidelity >= 90)
				{//[在自然道边可信的情况下处理]
					if (g_natural_boundary.r_boundary[0].x - g_multi_lane.lane_line[3].line[0].x > 80)
					{//[右侧车道离自然道边有50cm距离]
						g_take_over_back_lane_flag = 1;
						return;
					}
				}
			}
		}
	}
	//[不符合情况清除]
	g_take_over_back_lane_flag = -1;
}

//[@@]
/*==================================================================
 * 函数名  ：	void keep_trace_in_right_lane()
 * 功能    ：	2013年常熟比赛保持靠右车道
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void keep_trace_in_right_lane()
{
	if (g_navi_state == TRACE_LANE || \
		g_navi_state == CHANGE_TO_LEFT_LANE || \
		g_navi_state == CHANGE_TO_RIGHT_LANE)
	{
		if (g_multi_lane.lane_line[2].valid_num_points > 2 && g_multi_lane.lane_line[3].valid_num_points > 2 && g_multi_lane.lane_line[0].valid_num_points == 0)
		{//[环湖路只有两车道，有左右两车道时，不换道]
			g_take_over_back_lane_flag = 1;
		}
		else
		{
			g_take_over_back_lane_flag = -1;
		}
	}
}

int check_collision_for_lane_merge()
{
	int ret = 0;

	int x = g_mid_line[0].x;
	int y = g_mid_line[0].y;
	int num = y / GRID_LEN_PER_CELL;
	double x_step = x * 1.0 / num;
	for (int i = 0; i < num;i++)
	{
		x = (int)((x_step * i) / GRID_LEN_PER_CELL + g_grid_center.x);
		y = i + g_grid_center.y;

		if (g_grid_map[y][x] > 0)
		{
			ret = -1;
			return ret;
		}
	}

	return ret;
}

/*==================================================================
 * 函数名  ：	int trace_lane(PL_FUNC_INPUT *pl_input, PL_LOCAL_DATA *pl_local_data)
 * 功能    ：	道路跟踪。规划内部的状态切换
 * 输入参数：	PL_FUNC_INPUT *pl_input			融合输入
				PL_LOCAL_DATA *pl_local_data	用来向总控发送事件
 * 输出参数：	
 * 返回值  ：	int 0  可以执行  -1  不可执行
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int trace_lane(PL_FUNC_INPUT *pl_input)
{
	int ret = 0;
	int i;
	COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int left_line_num, right_line_num;
	//	int step;

	//[从全局变量中获得当前道边]
	//[@@首先要确定道路是否能用]
	memset(left_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	memset(right_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	left_line_num = 0;
	right_line_num = 0;


	//[只有在正常巡航时保持居中]
	if (g_stat == S_ROAD_NAV)
	{
		//keep_trace_in_the_mid_lane();
// 		if (g_road_type == 1)
// 		{
// 			keep_trace_in_right_lane();
// 		}

	}
	else
	{
		g_take_over_back_lane_flag = -1;
	}

	//[根据黄线信息判断是否回道]
// 	if (g_stat == S_ROAD_NAV || g_stat == S_CROSS_UND)
// 	{
// 		if (g_multi_lane.lane_line[0].line_color == YELLOW)
// 		{
// 			g_yellow_line_flag = 0;
// 			g_take_over_back_lane_flag = -1;
// 		}
// 		else if (g_multi_lane.lane_line[1].line_color == YELLOW)
// 		{
// 			g_yellow_line_flag = 1;
// 			g_take_over_back_lane_flag = -1;
// 		}
// 		else if (g_multi_lane.lane_line[2].line_color == YELLOW || g_multi_lane.lane_line[3].line_color == YELLOW)
// 		{
// 			g_yellow_line_flag = 2;
// 			if (g_stat != S_CROSS_UND)
// 			{
// 				g_take_over_back_lane_flag = 1;//[开启右回道标志]
// 			}
// 		}
// 		else
// 		{
// 			g_yellow_line_flag = 0;
// 		}
// 	}
// 	else
// 	{
// 		g_yellow_line_flag = 0;
// 	}


	//[只用1、2车道线，1、2车道线描述当前车道两边界]
	if (g_multi_lane.lane_line[1].valid_num_points > 1 && g_multi_lane.lane_line[2].valid_num_points > 1 \
		&& g_multi_lane.lane_line[1].line[0].x > -300 && g_multi_lane.lane_line[1].line[0].x < 100 \
		&& g_multi_lane.lane_line[2].line[0].x > -100 && g_multi_lane.lane_line[2].line[0].x < 300)
	{//[最好的情况，左右车道比较稳定。设置的判定参数可以修改，并没有特定的说法，经验参数]
		memcpy(left_line, g_multi_lane.lane_line[1].line, g_multi_lane.lane_line[1].valid_num_points * sizeof(COOR2));
		left_line_num = g_multi_lane.lane_line[1].valid_num_points;

		memcpy(right_line, g_multi_lane.lane_line[2].line, g_multi_lane.lane_line[2].valid_num_points * sizeof(COOR2));
		right_line_num = g_multi_lane.lane_line[2].valid_num_points;
#ifdef MBUG_OPEN_
		MBUG("trace lane 1 2:\n");
		for (i = 0; i < left_line_num; i++)
		{
			MBUG("(%d, %d) ", left_line[i].x, left_line[i].y);
		}
		MBUG("\n");
		for (i = 0; i < right_line_num; i++)
		{
			MBUG("(%d, %d) ", right_line[i].x, right_line[i].y);
		}
		MBUG("\n");
#endif
		g_navi_state = TRACE_LANE;
	}
	else if (g_multi_lane.lane_line[1].valid_num_points > 1 \
		&& g_multi_lane.lane_line[1].line[0].x > -300 && g_multi_lane.lane_line[1].line[0].x < 100)
	{//[左道边稳定，补出右道边]
		memcpy(left_line, g_multi_lane.lane_line[1].line, g_multi_lane.lane_line[1].valid_num_points * sizeof(COOR2));
		left_line_num = g_multi_lane.lane_line[1].valid_num_points;

		for (i = 0; i < g_multi_lane.lane_line[1].valid_num_points; i++)
		{
			right_line[i].x = g_multi_lane.lane_line[1].line[i].x + 375;
			right_line[i].y = g_multi_lane.lane_line[1].line[i].y;
		}
		right_line_num = g_multi_lane.lane_line[1].valid_num_points;
#ifdef MBUG_OPEN_
		MBUG("trace lane 1 (1 + 3 -> 2):\n");
		for (i = 0; i < left_line_num; i++)
		{
			MBUG("(%d, %d) ", left_line[i].x, left_line[i].y);
		}
		MBUG("\n");
		for (i = 0; i < right_line_num; i++)
		{
			MBUG("(%d, %d) ", right_line[i].x, right_line[i].y);
		}
		MBUG("\n");
#endif
		g_navi_state = TRACE_LANE;
	}
	else if (g_multi_lane.lane_line[2].valid_num_points > 1 \
		&& g_multi_lane.lane_line[2].line[0].x > -100 && g_multi_lane.lane_line[2].line[0].x < 300)
	{//[右道边稳定，补出左道边]
		for (i = 0; i < g_multi_lane.lane_line[2].valid_num_points; i++)
		{
			left_line[i].x = g_multi_lane.lane_line[2].line[i].x - 375;
			left_line[i].y = g_multi_lane.lane_line[2].line[i].y;
		}
		left_line_num = g_multi_lane.lane_line[2].valid_num_points;

		memcpy(right_line, g_multi_lane.lane_line[2].line, g_multi_lane.lane_line[2].valid_num_points * sizeof(COOR2));
		right_line_num = g_multi_lane.lane_line[2].valid_num_points;
#ifdef MBUG_OPEN_
		MBUG("trace lane (0 + 2 -> 1) 2:\n");
		for (i = 0; i < left_line_num; i++)
		{
			MBUG("(%d, %d) ", left_line[i].x, left_line[i].y);
		}
		MBUG("\n");
		for (i = 0; i < right_line_num; i++)
		{
			MBUG("(%d, %d) ", right_line[i].x, right_line[i].y);
		}
		MBUG("\n");
#endif
		g_navi_state = TRACE_LANE;
	}
	else
	{//[没有可用行车线车道，检查有无自然道边]
		if (g_stat == S_ROAD_NAV || g_stat == S_CROSS_UND || g_stat == S_TASK_OVER)
		{
			if (g_road_type == 0)
			{
				double x1, y1;
				double x2, y2;
				double x;
				int num = 0;
				int points_num = 0;
				x1 = y1 = x2 = y2 = 0;
				double theta = 0;
				double x_step = 0;
				double y_step = 0;

				//[根据结构化道路以及自然道边的方向补出道路]
				for (i = 0; i < LANE_LINE_NUM; i++)
				{
					if (g_multi_lane.lane_line[i].valid_num_points >= 2)
					{
						num++;
						points_num = g_multi_lane.lane_line[i].valid_num_points;
						x1 += g_multi_lane.lane_line[i].line[0].x;
						y1 += g_multi_lane.lane_line[i].line[0].y;

						x2 += g_multi_lane.lane_line[i].line[points_num - 1].x;
						y2 += g_multi_lane.lane_line[i].line[points_num - 1].y;
					}
				}

				if (num > 0)
				{//[有结构化道路]
					x1 = x1 / num;
					y1 = y1 / num;
					x2 = x2 / num;
					y2 = y2 / num;

					theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出道路角度]

					//[利用375补出左右道路边]
					if (fabs(theta) > 0.5 && fabs(theta) < 3)
					{//[大于两度使用道路方向]
						x = (x1 - x2 + 0.0) / (y1 - y2) * (FU_ROAD_LENGTH - 400) - 187;
						x_step = (x - (-187) + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							left_line[i].x = (INT32)(-187 + i * x_step);
							left_line[i].y = (INT32)(400 + i * y_step);
						}

						x = (x1 - x2 + 0.0) / (y1 - y2) * (FU_ROAD_LENGTH - 400) + 187;
						x_step = (x - 187 + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							right_line[i].x = (INT32)(187 + i * x_step);
							right_line[i].y = (INT32)(400 + i * y_step);
						}

						left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					}
					else if (fabs(theta) <= 0.5)
					{
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							left_line[i].x = -187;
							left_line[i].y = (INT32)(400 + i * y_step);
							right_line[i].x = 187;
							right_line[i].y = (INT32)(400 + i * y_step);
						}

						left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					}
					else
					{
						if (theta < 0)
						{
							theta = -3;
						}
						else
						{
							theta = 3;
						}
						x = tan(theta * PI / 180) * (FU_ROAD_LENGTH - 400) - 187;
						x_step = (x - (-187) + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							left_line[i].x = (INT32)(-187 + i * x_step);
							left_line[i].y = (INT32)(400 + i * y_step);
						}

						x = tan(theta * PI / 180) * (FU_ROAD_LENGTH - 400) + 187;
						x_step = (x - 187 + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							right_line[i].x = (INT32)(187 + i * x_step);
							right_line[i].y = (INT32)(400 + i * y_step);
						}

						left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					}
				}
				else if (g_natural_boundary.l_nums >= 2 || g_natural_boundary.r_nums >= 2)
				{//[没有结构化道路，使用自然道边]

					double l_theta = 0;
					double r_theta = 0;
					//[自然道边使用斜率最小的一个最为参考]
					if (g_natural_boundary.l_nums >= 2 && g_natural_boundary.r_nums >= 2)
					{
						x1 = g_natural_boundary.l_boundary[0].x;
						y1 = g_natural_boundary.l_boundary[0].y;
						x2 = g_natural_boundary.l_boundary[g_natural_boundary.l_nums - 1].x;
						y2 = g_natural_boundary.l_boundary[g_natural_boundary.l_nums - 1].y;
						l_theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出左道路边角度]

						x1 = g_natural_boundary.r_boundary[0].x;
						y1 = g_natural_boundary.r_boundary[0].y;
						x2 = g_natural_boundary.r_boundary[g_natural_boundary.r_nums - 1].x;
						y2 = g_natural_boundary.r_boundary[g_natural_boundary.r_nums - 1].y;
						r_theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出右道路边角度]

						theta = fabs(l_theta) < fabs(r_theta) ? l_theta : r_theta;
					}
					else if (g_natural_boundary.l_nums >= 2)
					{
						x1 = g_natural_boundary.l_boundary[0].x;
						y1 = g_natural_boundary.l_boundary[0].y;
						x2 = g_natural_boundary.l_boundary[g_natural_boundary.l_nums - 1].x;
						y2 = g_natural_boundary.l_boundary[g_natural_boundary.l_nums - 1].y;
						theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出道路角度]
					}
					else
					{
						x1 = g_natural_boundary.r_boundary[0].x;
						y1 = g_natural_boundary.r_boundary[0].y;
						x2 = g_natural_boundary.r_boundary[g_natural_boundary.r_nums - 1].x;
						y2 = g_natural_boundary.r_boundary[g_natural_boundary.r_nums - 1].y;
						theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出道路角度]
					}

					//[利用400cm补出左右道路边]
					if (fabs(theta) > 0.5 && fabs(theta) < 3)
					{//[0.5-2度内]
						x = (x1 - x2 + 0.0) / (y1 - y2) * (FU_ROAD_LENGTH - 400) - 200;
						x_step = (x - (-200) + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							left_line[i].x = (INT32)(-200 + i * x_step);
							left_line[i].y = (INT32)(400 + i * y_step);
						}

						x = (x1 - x2 + 0.0) / (y1 - y2) * (FU_ROAD_LENGTH - 400) + 200;
						x_step = (x - 200 + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							right_line[i].x = (INT32)(200 + i * x_step);
							right_line[i].y = (INT32)(400 + i * y_step);
						}

						left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					}
					else if (fabs(theta) <= 0.5)
					{
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							left_line[i].x = -200;
							left_line[i].y = (INT32)(400 + i * y_step);
							right_line[i].x = 200;
							right_line[i].y = (INT32)(400 + i * y_step);
						}

						left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					}
					else
					{//[3度以上]
						if (theta < 0)
							theta = -3;
						else
							theta = 3;
						x = tan(theta * PI / 180) * (FU_ROAD_LENGTH - 400) - 200;
						x_step = (x - (-200) + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							left_line[i].x = (INT32)(-200 + i * x_step);
							left_line[i].y = (INT32)(400 + i * y_step);
						}

						x = tan(theta * PI / 180) * (FU_ROAD_LENGTH - 400) + 200;
						x_step = (x - 200 + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							right_line[i].x = (INT32)(200 + i * x_step);
							right_line[i].y = (INT32)(400 + i * y_step);
						}

						left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					}
				}
				else
				{//[没有任何道路信息，使用上一帧的方向]
					//theta = atan((g_last_frame_pt.x + 0.0) / (g_last_frame_pt.y - 400)) * 180 / PI;//[求出道路角度]

					//[沿车行驶方向补出道边]
					theta = 0;
					x = tan(theta * PI / 180) * (FU_ROAD_LENGTH - 400) - 200;
					x_step = (x - (-200) + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
					y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

					for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
					{
						left_line[i].x = (INT32)(-200 + i * x_step);
						left_line[i].y = (INT32)(400 + i * y_step);
					}

					x = tan(theta * PI / 180) * (FU_ROAD_LENGTH - 400) + 200;
					x_step = (x - 200 + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
					y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

					for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
					{
						right_line[i].x = (INT32)(200 + i * x_step);
						right_line[i].y = (INT32)(400 + i * y_step);
					}

					left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
				}
			}//[if (g_road_type == 0)]
			else
			{
				{//[使用自然道边]
					g_navi_state = TRACE_IN_NATURAL_ROAD;

					if (g_natural_boundary.l_nums >= 2 && g_natural_boundary.r_nums >= 2 && g_emergency_roving_timer == 0)
					{
					}
					else
					{
						g_navi_state = ROVING;

						if (g_emergency_roving_timer > 0)
						{
							//[进行漫游计时器的调整]
							if (abs(g_cur_search_index - MORPHIN_MID_INDEX) < 5)//[基本直行]
							{
								g_emergency_roving_timer -= 1;
								if (g_emergency_roving_timer < 0)
								{
									g_emergency_roving_timer = 0;
								}
							}
							else if (g_navi_state == TRACE_LANE)
							{
								g_emergency_roving_timer -= 1;
								if (g_emergency_roving_timer < 0)
								{
									g_emergency_roving_timer = 0;
								}
							}
							else
							{
								g_emergency_roving_timer--;
								if (g_emergency_roving_timer < 0)
								{
									g_emergency_roving_timer = 0;
								}
							}
						}
						else
						{
							g_emergency_roving_timer = 0;
							g_emergency_counter = 0;
						}

						return ret;
					}
				}//[使用自然道边]
			}
		}
	}

	int l_dyn_obs_index = -1;//[记录当前车道的动态目标]

	if (g_stat == S_ROAD_NAV && g_navi_state != TRACE_IN_NATURAL_ROAD)
	{//[检查当前车道可通行性]

		if (left_line[0].x > -100 || right_line[0].x < 100)
		{//[压线]
			g_on_the_line_flag = 1;
		}
		else
		{
			g_on_the_line_flag = 0;
		}

		//[上一帧若是Roving，这里重新读取栅格使得动态障碍物描述恢复]
		get_gridmap(pl_input);

		//[进行道路可通行性检测，记录危险距离]
		g_unsafe_dist_for_change_lane = MAX_VALUE;
		ret = check_the_lane(left_line, left_line_num, right_line, right_line_num);

		//[对动态障碍物进行处理]
		if (g_stat == S_ROAD_NAV && \
			(g_navi_state == TRACE_LANE || \
			g_navi_state == FOLLOW_THE_CAR || \
			g_navi_state == D_EMERGENCY_STOP || \
			g_navi_state == S_EMERGENCY_STOP || \
			g_navi_state == ROVING))
		{
			int min_y = MAX_VALUE;
			int min_index = MAX_VALUE;
			DYN_OBS temp_dyn_obs;
			for (i = 0; i < g_dyn_obs_num; i++)
			{
				int l_x = 0;
				int r_x = 0;
				temp_dyn_obs = g_dyn_obs[i];
				get_x_coord(temp_dyn_obs.center.y, left_line, left_line_num, &l_x);
				get_x_coord(temp_dyn_obs.center.y, right_line, right_line_num, &r_x);
				if (temp_dyn_obs.center.x > l_x - 30 && temp_dyn_obs.center.x < r_x + 30)
				{//[找到在当前车道，且离车最近的动态障碍物]
					if (temp_dyn_obs.center.y > 400 && temp_dyn_obs.center.y < min_y)
					{//[在车后面的动态障碍物不考虑，这里使用>400考虑到不会有和车体重合的障碍物，而且后方车辆会避免与我方车辆碰撞]
						min_y = temp_dyn_obs.center.y;
						min_index = i;
					}
				}
			}

			g_car_ahead_dist = MAX_VALUE;

			if (min_index != MAX_VALUE)
			{//[有影响车辆行驶的动态障碍物存在]
				temp_dyn_obs = g_dyn_obs[min_index];
				l_dyn_obs_index = min_index;

				//[@@由于融合给出的动态障碍物栅格清除不干净，这里做个处理，]
				//[认为动态障碍物下方的区域静态为空，所以不对静态障碍物判断]
				if (temp_dyn_obs.nearest_pt.y - 400 < g_unsafe_dist)
				{//[若是动态障碍物比危险点近，那么变为处理动态障碍物(减去4m是因为有的动障碍物会有拖尾巴)]
					ret = 0;
				}

				//[注意：下面if条件的补集在else里面没有可能考虑完全]
				if (temp_dyn_obs.speed.y > g_wp_speed && g_real_speed < g_wp_speed)//[大于设置最高速度]
				{
					if (ret == 0)
					{//[没有静态障碍物堵塞，则跟随]
#ifdef MBUG_OPEN_
						MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
						MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
						MBUG("follow the dyn obs\n");
#endif
						g_take_over_flag = 0;
						g_navi_state = FOLLOW_THE_CAR;
						g_follow_car_speed = g_wp_speed;
					}
				}//[if (temp_dyn_obs.speed.y > g_wp_speed)//[大于设置最高速度]]
				else
				{
					if (ret == 0)
					{//[没有静态障碍物堵塞]
						if (temp_dyn_obs.speed.y > (g_real_speed + (int)(3 * CM)))//[加3km防止前面车辆测的速度虚高]
						{//[前方车辆速度大于当前实时速度，以前方车辆为最高速度跟随]
#ifdef MBUG_OPEN_
							MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
							MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
							MBUG("follow the dyn obs\n");
#endif
							g_take_over_flag = 0;
							g_navi_state = FOLLOW_THE_CAR;
							//[使用速度低的]
							g_follow_car_speed = temp_dyn_obs.speed.y > g_wp_speed ? g_wp_speed : temp_dyn_obs.speed.y;
						}
						else
						{//[前方车辆速度小于当前实时速度，根据距离进行速度设置]
							if (temp_dyn_obs.speed.y < (int)(g_take_over_threshold * CM))
							{//[目标车辆小于超车阈值，进行超车]
								if (g_real_speed <= (int)(15 * CM))
								{
									if (temp_dyn_obs.nearest_pt.y > g_10km_dist)
									{
#ifdef MBUG_OPEN_
										MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
										MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
										MBUG("take over the dyn obs\n");
#endif
										g_navi_state = TRACE_LANE;//[切换到可以行进的状态]
										g_take_over_flag = 1;
										g_follow_car_speed = temp_dyn_obs.speed.y;
										g_car_ahead_dist = temp_dyn_obs.nearest_pt.y;
										ret = -1;
									}
									else
									{
#ifdef MBUG_OPEN_
										MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
										MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
										MBUG("emergency stop\n");
#endif
										g_take_over_flag = 0;
										g_navi_state = D_EMERGENCY_STOP;
										g_car_ahead_dist = temp_dyn_obs.nearest_pt.y;
										ret = -1;
									}
								}
								else if (g_real_speed > (int)(15 * CM) && g_real_speed <= (int)(20 * CM))
								{
									if (temp_dyn_obs.nearest_pt.y > g_20km_dist - 500)
									{
#ifdef MBUG_OPEN_
										MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
										MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
										MBUG("take over the dyn obs\n");
#endif
										g_navi_state = TRACE_LANE;//[切换到可以行进的状态]
										g_take_over_flag = 1;
										g_follow_car_speed = temp_dyn_obs.speed.y;
										g_car_ahead_dist = temp_dyn_obs.nearest_pt.y;
										ret = -1;
									}
									else
									{
#ifdef MBUG_OPEN_
										MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
										MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
										MBUG("emergency stop\n");
#endif
										g_take_over_flag = 0;
										g_navi_state = D_EMERGENCY_STOP;
										g_car_ahead_dist = temp_dyn_obs.nearest_pt.y;
										ret = -1;
									}
								}
								else if (g_real_speed > (int)(20 * CM) && g_real_speed <= (int)(30 * CM))
								{
									if (temp_dyn_obs.nearest_pt.y > g_30km_dist - 1000)
									{
#ifdef MBUG_OPEN_
										MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
										MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
										MBUG("take over the dyn obs\n");
#endif
										g_navi_state = TRACE_LANE;//[切换到可以行进的状态]
										g_take_over_flag = 1;
										g_follow_car_speed = temp_dyn_obs.speed.y;
										g_car_ahead_dist = temp_dyn_obs.nearest_pt.y;
										ret = -1;
									}
									else
									{
#ifdef MBUG_OPEN_
										MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
										MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
										MBUG("emergency stop\n");
#endif
										g_take_over_flag = 0;
										g_navi_state = D_EMERGENCY_STOP;
										g_car_ahead_dist = temp_dyn_obs.nearest_pt.y;
										ret = -1;
									}
								}
								else if (g_real_speed > (int)(30 * CM) && g_real_speed <= (int)(40 * CM))
								{
									if (temp_dyn_obs.nearest_pt.y > g_40km_dist - 1000)
									{
#ifdef MBUG_OPEN_
										MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
										MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
										MBUG("take over the dyn obs\n");
#endif
										g_navi_state = TRACE_LANE;//[切换到可以行进的状态]
										g_take_over_flag = 1;
										g_follow_car_speed = temp_dyn_obs.speed.y;
										g_car_ahead_dist = temp_dyn_obs.nearest_pt.y;
										ret = -1;
									}
									else
									{
#ifdef MBUG_OPEN_
										MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
										MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
										MBUG("emergency stop\n");
#endif
										g_take_over_flag = 0;
										g_navi_state = D_EMERGENCY_STOP;
										g_car_ahead_dist = temp_dyn_obs.nearest_pt.y;
										ret = -1;
									}
								}
								else
								{//[实际速度高于40km]
									if (temp_dyn_obs.nearest_pt.y > g_40km_dist)
									{
#ifdef MBUG_OPEN_
										MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
										MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
										MBUG("take over the dyn obs\n");
#endif
										g_navi_state = TRACE_LANE;//[切换到可以行进的状态]
										g_take_over_flag = 1;
										g_follow_car_speed = temp_dyn_obs.speed.y;
										g_car_ahead_dist = temp_dyn_obs.nearest_pt.y;
										ret = -1;
									}
									else
									{
#ifdef MBUG_OPEN_
										MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
										MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
										MBUG("emergency stop\n");
#endif
										g_take_over_flag = 0;
										g_navi_state = D_EMERGENCY_STOP;
										g_car_ahead_dist = temp_dyn_obs.nearest_pt.y;
										ret = -1;
									}
								}
							}//[if (temp_dyn_obs.speed.y < (int)(g_take_over_threshold * CM))]
							else if (temp_dyn_obs.nearest_pt.y > g_40km_dist)
							{
#ifdef MBUG_OPEN_
								MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
								MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
								MBUG("follow the dyn obs\n");
#endif
								g_take_over_flag = 0;
								g_navi_state = FOLLOW_THE_CAR;
								g_follow_car_speed = temp_dyn_obs.speed.y > g_wp_speed ? g_wp_speed : temp_dyn_obs.speed.y;
							}
							else if (temp_dyn_obs.nearest_pt.y > g_30km_dist && temp_dyn_obs.nearest_pt.y <= g_40km_dist)
							{
#ifdef MBUG_OPEN_
								MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
								MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
								MBUG("follow the dyn obs\n");
#endif
								g_take_over_flag = 0;
								g_navi_state = FOLLOW_THE_CAR;
								g_follow_car_speed = (temp_dyn_obs.speed.y - (int)(3 * CM)) > g_wp_speed ? g_wp_speed : (temp_dyn_obs.speed.y - (int)(3 * CM));
							}
							else if (temp_dyn_obs.nearest_pt.y > g_20km_dist && temp_dyn_obs.nearest_pt.y <= g_30km_dist)
							{
#ifdef MBUG_OPEN_
								MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
								MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
								MBUG("follow the dyn obs\n");
#endif
								g_take_over_flag = 0;
								g_navi_state = FOLLOW_THE_CAR;
								g_follow_car_speed = (temp_dyn_obs.speed.y - (int)(4 * CM)) > g_wp_speed ? g_wp_speed : (temp_dyn_obs.speed.y - (int)(4 * CM));
							}
							else if (temp_dyn_obs.nearest_pt.y > g_10km_dist && temp_dyn_obs.nearest_pt.y <= g_20km_dist)
							{
#ifdef MBUG_OPEN_
								MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
								MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
								MBUG("follow the dyn obs\n");
#endif
								g_take_over_flag = 0;
								g_navi_state = FOLLOW_THE_CAR;
								g_follow_car_speed = (temp_dyn_obs.speed.y - (int)(5 * CM)) > g_wp_speed ? g_wp_speed : (temp_dyn_obs.speed.y - (int)(5 * CM));
							}
							else if (temp_dyn_obs.nearest_pt.y > g_stop_dist && temp_dyn_obs.nearest_pt.y <= g_10km_dist)
							{
#ifdef MBUG_OPEN_
								MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
								MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
								MBUG("follow the dyn obs\n");
#endif
								g_take_over_flag = 0;
								g_navi_state = FOLLOW_THE_CAR;
								g_follow_car_speed = (temp_dyn_obs.speed.y - (int)(6 * CM)) > g_wp_speed ? g_wp_speed : (temp_dyn_obs.speed.y - (int)(6 * CM));
							}
							else
							{
#ifdef MBUG_OPEN_
								MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
								MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
								MBUG("emergency stop\n");
#endif
								g_take_over_flag = 0;
								g_navi_state = D_EMERGENCY_STOP;
								g_car_ahead_dist = temp_dyn_obs.center.y;
								ret = -1;
							}
						}//[else]
					}//[if (ret == 0)]
				}//[else]
			}//[if (min_index != MAX_VALUE)]
			else
			{//[没有影响车辆行驶的动态障碍物存在]
				if (ret == 0)
				{//[静态障碍物没有堵塞道路]
					g_take_over_flag = 0;
					g_change_lane_direct_keep = -1;
					if (g_road_type == 0)
					{
						g_navi_state = TRACE_LANE;
					}
					else
					{
						g_navi_state = TRACE_IN_NATURAL_ROAD;
					}
				}
				else
				{
					//[无动态障碍物，静态障碍物堵塞道路]
					g_take_over_flag = 0;
					if (g_road_type == 0)
					{
						g_navi_state = S_EMERGENCY_STOP;
					}
					else
					{
						g_navi_state = ROVING;
						return ret;
					}
					
				}
			}
		}//[if (g_stat == S_ROAD_NAV && (g_navi_state == TRACE_LANE || g_navi_state == FOLLOW_THE_CAR || g_navi_state == EMERGENCY_STOP))]
		else if (g_stat == S_ROAD_NAV && g_navi_state == ROVING)
		{
			if (ret == 0 && g_emergency_roving_timer == 0)
			{//[从漫游状态返回道路跟踪]
				g_navi_state = TRACE_LANE;
			}
		}
	}//[if (g_stat == S_ROAD_NAV && g_navi_state != TRACE_IN_NATURAL_ROAD)]
	else if ((g_stat == S_ROAD_NAV && g_navi_state == TRACE_IN_NATURAL_ROAD) || \
		(g_stat == S_TASK_OVER && g_navi_state == TRACE_IN_NATURAL_ROAD) || \
		(g_stat == S_CROSS_UND && g_navi_state == TRACE_IN_NATURAL_ROAD))
	{//[没检测到结构化道路]
		ret = check_the_lane(g_natural_boundary.l_boundary, g_natural_boundary.l_nums, \
			g_natural_boundary.r_boundary, g_natural_boundary.r_nums);

		int min_y = MAX_VALUE;
		int min_index = MAX_VALUE;
		DYN_OBS temp_dyn_obs;
		for (i = 0; i < g_dyn_obs_num; i++)
		{
			temp_dyn_obs = g_dyn_obs[i];
			if (temp_dyn_obs.center.x > -200 && temp_dyn_obs.center.x < 200)
			{//[找到在当前车道，且离车最近的动态障碍物]
				if (temp_dyn_obs.center.y > 400 && temp_dyn_obs.center.y < min_y)
				{//[在车后面的动态障碍物不考虑，这里使用>400考虑到不会有和车体重合的障碍物，而且后方车辆会避免与我方车辆碰撞]
					min_y = temp_dyn_obs.center.y;
					min_index = i;
				}
			}
		}

		g_car_ahead_dist = MAX_VALUE;

		if (min_index != MAX_VALUE)
		{//[有影响车辆行驶的动态障碍物存在]
			temp_dyn_obs = g_dyn_obs[min_index];
			l_dyn_obs_index = min_index;

			//[@@由于融合给出的动态障碍物栅格清除不干净，这里做个处理，]
			//[认为动态障碍物下方的区域静态为空，所以不对静态障碍物判断]
			if (temp_dyn_obs.nearest_pt.y - 200 < g_unsafe_dist)
			{//[若是动态障碍物比危险点近，那么变为处理动态障碍物(减去4m是因为有的动障碍物会有拖尾巴)]
				ret = 0;
			}

			if (ret == 0)
			{//[没有静态障碍物堵塞，则跟随]
#ifdef MBUG_OPEN_
				MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
				MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
				MBUG("follow the dyn obs\n");
#endif
				g_take_over_flag = 0;
				g_navi_state = FOLLOW_THE_CAR;
				g_follow_car_speed = temp_dyn_obs.speed.y;
			}
		}//[if (min_index != MAX_VALUE)]
		else
		{
			if (ret == -1 && g_navi_state == TRACE_IN_NATURAL_ROAD)
				g_navi_state = ROVING;
		}
		return ret;
	}
	else
	{//[此处为S_CROSS_UND，道路跟踪]
#ifdef MBUG_OPEN_
		MBUG("S_CROSS_UND  check_the_lane\n");
		for (i = 0; i < left_line_num; i++)
		{
			MBUG("(%d, %d) ", left_line[i].x, left_line[i].y);
		}
		MBUG("\n");
		for (i = 0; i < right_line_num; i++)
		{
			MBUG("(%d, %d) ", right_line[i].x, right_line[i].y);
		}
		MBUG("\n");
#endif
		ret = check_the_lane(left_line, left_line_num, right_line, right_line_num);
	}

	if (ret == -1 && g_stat == S_ROAD_NAV && g_navi_state != D_EMERGENCY_STOP && g_navi_state != ROVING)
	{//[道路巡航状态下堵塞]
#ifdef MBUG_OPEN_
		MBUG("current lane is blocked!!\n");
#endif
		g_take_over_back_lane_flag = -1;//[清除回道标志]

		if (g_unsafe_dist <= 1000)
		{//[距离不够换道]
			g_navi_state = S_EMERGENCY_STOP;
		}
		else
		{
			//[记录下最危险距离]
			g_unsafe_dist_for_change_lane = g_unsafe_dist;
			int num1 = 0;
			int num2 = 0;

			if (g_last_frame_pt.x > 600)
			{//[上一帧向右方行驶，所以先检测右车道，这样做是为了保持规划稳定性]
				//[@@检测左车道和右车道，进行换道]
				//[右车道]
				if (g_multi_lane.ok_times[3] == 2 && g_change_lane_direct_keep != 0)
				{
					if (g_take_over_flag == 0)
					{
						g_change_lane_search_flag = 1;
						//[两个车道内向危险之处后面看10m，用于检测是否可通行]
						g_change_lane_search_dist = g_unsafe_dist + 1000;
						if (g_change_lane_search_dist > 5000)
						{
							g_change_lane_search_dist = 5000;
						}

						num1 = g_multi_lane.lane_line[1].valid_num_points;
						num2 = g_multi_lane.lane_line[3].valid_num_points;
						ret = check_the_lane(g_multi_lane.lane_line[1].line, num1,
							g_multi_lane.lane_line[3].line, num2);

						if (ret == 0)
							ret = check_collision_for_lane_merge();

						g_change_lane_search_flag = 0;
					}
					else
					{
						num1 = g_multi_lane.lane_line[2].valid_num_points;
						num2 = g_multi_lane.lane_line[3].valid_num_points;
						ret = check_the_lane(g_multi_lane.lane_line[2].line, num1,
							g_multi_lane.lane_line[3].line, num2);

						if (ret == 0)
						{
							num1 = g_multi_lane.lane_line[1].valid_num_points;
							num2 = g_multi_lane.lane_line[3].valid_num_points;
							ret = check_the_lane(g_multi_lane.lane_line[1].line, num1,
								g_multi_lane.lane_line[3].line, num2);

							if (ret == 0)
								ret = check_collision_for_lane_merge();
						}
					}

					//[@@增加动态障碍物的判断]
					DYN_OBS temp_dyn_obs;
					for (i = 0; i < g_dyn_obs_num; i++)
					{
						int l_x = 0;
						int r_x = 0;
						temp_dyn_obs = g_dyn_obs[i];
						get_x_coord(temp_dyn_obs.center.y, g_multi_lane.lane_line[2].line, (int)g_multi_lane.lane_line[2].valid_num_points, &l_x);
						get_x_coord(temp_dyn_obs.center.y, g_multi_lane.lane_line[3].line, (int)g_multi_lane.lane_line[3].valid_num_points, &r_x);
						if (temp_dyn_obs.center.x > l_x && temp_dyn_obs.center.x < r_x)
						{//[右车道有动态障碍物]
							ret = -1;
							break;
						}
					}

					if (ret == 0)
					{
						//[@@右换道]
#ifdef MBUG_OPEN_
						MBUG("------------- Change to right lane --------------\n");
#endif
						if (g_take_over_flag == 0)
							g_navi_state = CHANGE_TO_RIGHT_LANE;
						else
							g_navi_state = TAKEOVER_FROM_RIGHT;

						g_change_lane_state = NORMAL_CHANGE;
						g_change_lane_count = 0;
						g_change_lane_see = 1;
						g_change_fade_timer = 0;
						g_change_lane_direct_keep = 1;

						return ret;
					}
				}

				//[左车道]
				if (g_multi_lane.ok_times[0] == 2 && g_yellow_line_flag == 0 && g_change_lane_direct_keep != 1)
				{
					if (g_take_over_flag == 0)
					{
						g_change_lane_search_flag = 1;
						//[两个车道内向危险之处后面看10m，用于检测是否可通行]
						g_change_lane_search_dist = g_unsafe_dist + 1000;
						if (g_change_lane_search_dist > 5000)
						{
							g_change_lane_search_dist = 5000;
						}
						num1 = g_multi_lane.lane_line[0].valid_num_points;
						num2 = g_multi_lane.lane_line[2].valid_num_points;
						ret = check_the_lane(g_multi_lane.lane_line[0].line, num1,
							g_multi_lane.lane_line[2].line, num2);

						if (ret == 0)
							ret = check_collision_for_lane_merge();

						g_change_lane_search_flag = 0;
					}
					else
					{
						num1 = g_multi_lane.lane_line[0].valid_num_points;
						num2 = g_multi_lane.lane_line[1].valid_num_points;
						ret = check_the_lane(g_multi_lane.lane_line[0].line, num1,
							g_multi_lane.lane_line[1].line, num2);
						if (ret == 0)
						{
							num1 = g_multi_lane.lane_line[0].valid_num_points;
							num2 = g_multi_lane.lane_line[2].valid_num_points;
							ret = check_the_lane(g_multi_lane.lane_line[0].line, num1,
								g_multi_lane.lane_line[2].line, num2);

							if (ret == 0)
								ret = check_collision_for_lane_merge();

							g_change_lane_search_flag = 0;
						}
					}

					//[@@增加动态障碍物的判断]
					DYN_OBS temp_dyn_obs;
					for (i = 0; i < g_dyn_obs_num; i++)
					{
						int l_x = 0;
						int r_x = 0;
						temp_dyn_obs = g_dyn_obs[i];
						get_x_coord(temp_dyn_obs.center.y, g_multi_lane.lane_line[0].line, (int)g_multi_lane.lane_line[0].valid_num_points, &l_x);
						get_x_coord(temp_dyn_obs.center.y, g_multi_lane.lane_line[1].line, (int)g_multi_lane.lane_line[1].valid_num_points, &r_x);
						if (temp_dyn_obs.center.x > l_x && temp_dyn_obs.center.x < r_x)
						{//[左车道是否有动态障碍物]
							ret = -1;
							break;
						}
					}

					if (ret == 0)
					{
						//[@@左换道]
#ifdef MBUG_OPEN_
						MBUG("------------- Start to change to left lane --------------\n");
#endif
						if (g_take_over_flag == 0)
							g_navi_state = CHANGE_TO_LEFT_LANE;
						else
							g_navi_state = TAKEOVER_FROM_LEFT;

						g_change_lane_state = NORMAL_CHANGE;
						g_change_lane_count = 0;
						g_change_lane_see = -1;
						g_change_fade_timer = 0;
						g_change_lane_direct_keep = 0;

						return ret;
					}
				}
			}//[if (g_last_frame_pt.x > 600)]
			else
			{//[默认先检测左车道]
				//[@@2015年比赛，采集路点靠中间车道，因此改成先检测右车道，防止越过黄线]

				//[右车道]
				if (g_multi_lane.ok_times[3] == 2 && g_change_lane_direct_keep != 0)
				{
					if (g_take_over_flag == 0)
					{
						g_change_lane_search_flag = 1;
						//[两个车道内向危险之处后面看10m，用于检测是否可通行]
						g_change_lane_search_dist = g_unsafe_dist + 1000;
						if (g_change_lane_search_dist > 5000)
						{
							g_change_lane_search_dist = 5000;
						}

						num1 = g_multi_lane.lane_line[1].valid_num_points;
						num2 = g_multi_lane.lane_line[3].valid_num_points;
						ret = check_the_lane(g_multi_lane.lane_line[1].line, num1,
							g_multi_lane.lane_line[3].line, num2);

						if (ret == 0)
							ret = check_collision_for_lane_merge();

						g_change_lane_search_flag = 0;
					}
					else
					{
						num1 = g_multi_lane.lane_line[2].valid_num_points;
						num2 = g_multi_lane.lane_line[3].valid_num_points;
						ret = check_the_lane(g_multi_lane.lane_line[2].line, num1,
							g_multi_lane.lane_line[3].line, num2);

						if (ret == 0)
						{
							num1 = g_multi_lane.lane_line[1].valid_num_points;
							num2 = g_multi_lane.lane_line[3].valid_num_points;
							ret = check_the_lane(g_multi_lane.lane_line[1].line, num1,
								g_multi_lane.lane_line[3].line, num2);

							if (ret == 0)
								ret = check_collision_for_lane_merge();
						}
					}

					//[@@增加动态障碍物的判断]
					DYN_OBS temp_dyn_obs;
					for (i = 0; i < g_dyn_obs_num; i++)
					{
						int l_x = 0;
						int r_x = 0;
						temp_dyn_obs = g_dyn_obs[i];
						get_x_coord(temp_dyn_obs.center.y, g_multi_lane.lane_line[2].line, (int)g_multi_lane.lane_line[2].valid_num_points, &l_x);
						get_x_coord(temp_dyn_obs.center.y, g_multi_lane.lane_line[3].line, (int)g_multi_lane.lane_line[3].valid_num_points, &r_x);
						if (temp_dyn_obs.center.x > l_x && temp_dyn_obs.center.x < r_x)
						{//[右车道有动态障碍物]
							ret = -1;
							break;
						}
					}

					if (ret == 0)
					{
						//[@@右换道]
#ifdef MBUG_OPEN_
						MBUG("------------- Change to right lane --------------\n");
#endif
						if (g_take_over_flag == 0)
							g_navi_state = CHANGE_TO_RIGHT_LANE;
						else
							g_navi_state = TAKEOVER_FROM_RIGHT;

						g_change_lane_state = NORMAL_CHANGE;
						g_change_lane_count = 0;
						g_change_lane_see = 1;
						g_change_fade_timer = 0;
						g_change_lane_direct_keep = 1;

						return ret;
					}
				}

				//[@@检测左车道和右车道，进行换道]
				//[左车道]
				if (g_multi_lane.ok_times[0] == 2 && g_yellow_line_flag == 0 && g_change_lane_direct_keep != 1)
				{
					if (g_take_over_flag == 0)
					{
						g_change_lane_search_flag = 1;
						//[两个车道内向危险之处后面看10m，用于检测是否可通行]
						g_change_lane_search_dist = g_unsafe_dist + 1000;
						if (g_change_lane_search_dist > 5000)
						{
							g_change_lane_search_dist = 5000;
						}
						num1 = g_multi_lane.lane_line[0].valid_num_points;
						num2 = g_multi_lane.lane_line[2].valid_num_points;
						ret = check_the_lane(g_multi_lane.lane_line[0].line, num1,
							g_multi_lane.lane_line[2].line, num2);

						if (ret == 0)
							ret = check_collision_for_lane_merge();

						g_change_lane_search_flag = 0;
					}
					else
					{
						num1 = g_multi_lane.lane_line[0].valid_num_points;
						num2 = g_multi_lane.lane_line[1].valid_num_points;
						ret = check_the_lane(g_multi_lane.lane_line[0].line, num1,
							g_multi_lane.lane_line[1].line, num2);
						if (ret == 0)
						{
							num1 = g_multi_lane.lane_line[0].valid_num_points;
							num2 = g_multi_lane.lane_line[2].valid_num_points;
							ret = check_the_lane(g_multi_lane.lane_line[0].line, num1,
								g_multi_lane.lane_line[2].line, num2);
							if (ret == 0)
								ret = check_collision_for_lane_merge();
							
							g_change_lane_search_flag = 0;
						}
					}

					//[@@增加动态障碍物的判断]
					DYN_OBS temp_dyn_obs;
					for (i = 0; i < g_dyn_obs_num; i++)
					{
						int l_x = 0;
						int r_x = 0;
						temp_dyn_obs = g_dyn_obs[i];
						get_x_coord(temp_dyn_obs.center.y, g_multi_lane.lane_line[0].line, (int)g_multi_lane.lane_line[0].valid_num_points, &l_x);
						get_x_coord(temp_dyn_obs.center.y, g_multi_lane.lane_line[1].line, (int)g_multi_lane.lane_line[1].valid_num_points, &r_x);
						if (temp_dyn_obs.nearest_pt.y - 500 < g_unsafe_dist && temp_dyn_obs.speed.y > g_cur_speed)
						{
							continue;
						}
						if (temp_dyn_obs.center.x > l_x && temp_dyn_obs.center.x < r_x)
						{//[左车道有动态障碍物]
							ret = -1;
							break;
						}
					}

					if (ret == 0)
					{
						//[@@左换道]
#ifdef MBUG_OPEN_
						MBUG("------------- Start to change to left lane --------------\n");
#endif
						if (g_take_over_flag == 0)
							g_navi_state = CHANGE_TO_LEFT_LANE;
						else
							g_navi_state = TAKEOVER_FROM_LEFT;

						g_change_lane_state = NORMAL_CHANGE;
						g_change_lane_count = 0;
						g_change_lane_see = -1;
						g_change_fade_timer = 0;
						g_change_lane_direct_keep = 0;

						return ret;
					}
				}


			}


			if (g_navi_state == TRACE_LANE)
			{//[本车道有动态障碍物，但不能换道]
				DYN_OBS temp_dyn_obs;
				temp_dyn_obs = g_dyn_obs[l_dyn_obs_index];

				if (temp_dyn_obs.nearest_pt.y > g_40km_dist)
				{
#ifdef MBUG_OPEN_
					MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
					MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
					MBUG("follow the dyn obs\n");
#endif
					g_take_over_flag = 0;
					g_navi_state = FOLLOW_THE_CAR;
					g_follow_car_speed = (int)(35 * CM);
				}
				else if (temp_dyn_obs.nearest_pt.y > g_30km_dist && temp_dyn_obs.nearest_pt.y <= g_40km_dist)
				{
#ifdef MBUG_OPEN_
					MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
					MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
					MBUG("follow the dyn obs\n");
#endif
					g_take_over_flag = 0;
					g_navi_state = FOLLOW_THE_CAR;
					g_follow_car_speed = (int)(30 * CM);
				}
				else if (temp_dyn_obs.nearest_pt.y > g_20km_dist && temp_dyn_obs.nearest_pt.y <= g_30km_dist)
				{
#ifdef MBUG_OPEN_
					MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
					MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
					MBUG("follow the dyn obs\n");
#endif
					g_take_over_flag = 0;
					g_navi_state = FOLLOW_THE_CAR;
					g_follow_car_speed = (int)(20 * CM);
				}
				else if (temp_dyn_obs.nearest_pt.y > g_10km_dist && temp_dyn_obs.nearest_pt.y <= g_20km_dist)
				{
#ifdef MBUG_OPEN_
					MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
					MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
					MBUG("follow the dyn obs\n");
#endif
					g_take_over_flag = 0;
					g_navi_state = FOLLOW_THE_CAR;
					g_follow_car_speed = (int)(15 * CM);
				}
				else if (temp_dyn_obs.nearest_pt.y > g_stop_dist && temp_dyn_obs.nearest_pt.y <= g_10km_dist)
				{
#ifdef MBUG_OPEN_
					MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
					MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
					MBUG("follow the dyn obs\n");
#endif
					g_take_over_flag = 0;
					g_navi_state = FOLLOW_THE_CAR;
					g_follow_car_speed = (int)(10 * CM);
				}
				else
				{
#ifdef MBUG_OPEN_
					MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
					MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
					MBUG("emergency stop\n");
#endif
					g_take_over_flag = 0;
					g_navi_state = S_EMERGENCY_STOP;
					g_car_ahead_dist = temp_dyn_obs.center.y;
					ret = -1;
				}
			}
			else
			{
				ret = check_the_lane(left_line, left_line_num, right_line, right_line_num);
				g_navi_state = S_EMERGENCY_STOP;
			}
		}//[else]
	}
	else if (ret == -1 && g_stat == S_ROAD_NAV && g_navi_state == D_EMERGENCY_STOP)
	{//[堵塞]
		return ret;
	}
	else if (ret == -1 && g_stat == S_CROSS_UND)
	{//[路口检测状态下堵塞，直接停车]
#ifdef MBUG_OPEN_
		MBUG("In S_CROSS_UND current lane is blocked!!\n");
#endif
		return ret;
	}
	else if (ret == -1 && g_stat == S_TASK_OVER)
	{//[路口检测状态下堵塞，直接停车]
#ifdef MBUG_OPEN_
		MBUG("In S_TASK_OVER current lane is blocked!!\n");
#endif
		return ret;
	}
	else if (ret == -2)
	{
		//[没有道路边界]
	}
	else
	{
		//[按照正常规划进行]

		//[若有回道标志，进行回道]
		if (g_take_over_back_lane_flag != -1)
		{
			COOR2 temp_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
			memcpy(temp_mid_line, g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));

			if (g_take_over_back_lane_flag == 0)
			{
				//[左回道]
				if (g_multi_lane.ok_times[0] == 2 && g_yellow_line_flag == 0 && g_change_lane_direct_keep != 1)
				{
					g_change_lane_search_flag = 1;
					g_change_lane_search_dist = g_unsafe_dist + 1000;//[向后看10m]
					if (g_change_lane_search_dist > 5000)
					{
						g_change_lane_search_dist = 5000;
					}
					int num1 = g_multi_lane.lane_line[0].valid_num_points;
					int num2 = g_multi_lane.lane_line[2].valid_num_points;
					ret = check_the_lane(g_multi_lane.lane_line[0].line, num1,
						g_multi_lane.lane_line[2].line, num2);
					g_change_lane_search_flag = 0;

					//[@@增加动态障碍物的判断]
					DYN_OBS temp_dyn_obs;
					for (i = 0; i < g_dyn_obs_num; i++)
					{
						int l_x = 0;
						int r_x = 0;
						temp_dyn_obs = g_dyn_obs[i];
						get_x_coord(temp_dyn_obs.center.y, g_multi_lane.lane_line[0].line, (int)g_multi_lane.lane_line[0].valid_num_points, &l_x);
						get_x_coord(temp_dyn_obs.center.y, g_multi_lane.lane_line[1].line, (int)g_multi_lane.lane_line[1].valid_num_points, &r_x);
						if (temp_dyn_obs.center.x > l_x && temp_dyn_obs.center.x < r_x)
						{//[左车道有动态障碍物]
							ret = -1;
							break;
						}
					}

					if (ret == 0)
					{//[可以回道]
#ifdef MBUG_OPEN_
						MBUG("------------- back to left lane --------------\n");
#endif
						g_navi_state = CHANGE_TO_LEFT_LANE;

						g_change_lane_state = NORMAL_CHANGE;
						g_change_lane_count = 0;
						g_change_lane_see = -1;
						g_change_fade_timer = 0;
						g_change_lane_direct_keep = 0;

						return ret;
					}
					else
					{//[不可以回道]
						ret = 0;
						g_unsafe_dist = MAX_VALUE;
						g_navi_state = TRACE_LANE;
						g_take_over_back_lane_flag = -1;
						g_change_lane_direct_keep = -1;
						//[重新使用当前车道的规划]
						memcpy(g_mid_line, temp_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
						return ret;
					}
				}
			}//[if (g_take_over_back_lane_flag == 0)]

			else if (g_take_over_back_lane_flag == 1)
			{
				//[右回道]
				if (g_multi_lane.ok_times[3] == 2 && g_change_lane_direct_keep != 0)
				{
					g_change_lane_search_flag = 1;
					g_change_lane_search_dist = g_unsafe_dist + 1000;//[向后看10m]
					if (g_change_lane_search_dist > 5000)
					{
						g_change_lane_search_dist = 5000;
					}
					int num1 = g_multi_lane.lane_line[1].valid_num_points;
					int num2 = g_multi_lane.lane_line[3].valid_num_points;
					ret = check_the_lane(g_multi_lane.lane_line[1].line, num1,
						g_multi_lane.lane_line[3].line, num2);
					g_change_lane_search_flag = 0;

					//[@@增加动态障碍物的判断]
					DYN_OBS temp_dyn_obs;
					for (i = 0; i < g_dyn_obs_num; i++)
					{
						int l_x = 0;
						int r_x = 0;
						temp_dyn_obs = g_dyn_obs[i];
						get_x_coord(temp_dyn_obs.center.y, g_multi_lane.lane_line[2].line, (int)g_multi_lane.lane_line[2].valid_num_points, &l_x);
						get_x_coord(temp_dyn_obs.center.y, g_multi_lane.lane_line[3].line, (int)g_multi_lane.lane_line[3].valid_num_points, &r_x);
						if (temp_dyn_obs.center.x > l_x && temp_dyn_obs.center.x < r_x)
						{//[右车道有动态障碍物]
							ret = -1;
							break;
						}
					}

					if (ret == 0)
					{//[可以回道]
#ifdef MBUG_OPEN_
						MBUG("------------- back to right lane --------------\n");
#endif
						g_navi_state = CHANGE_TO_RIGHT_LANE;

						g_change_lane_state = NORMAL_CHANGE;
						g_change_lane_count = 0;
						g_change_lane_see = 1;
						g_change_fade_timer = 0;
						g_change_lane_direct_keep = 1;

						return ret;
					}
					else
					{//[不可以回道]
						ret = 0;
						g_unsafe_dist = MAX_VALUE;
						g_take_over_back_lane_flag = -1;
						g_navi_state = TRACE_LANE;
						g_change_lane_direct_keep = -1;
						//[重新使用当前车道的规划]
						memcpy(g_mid_line, temp_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
						return ret;
					}
				}//[if (g_multi_lane.ok_times[3] == 2)]
			}//[else if (g_take_over_back_lane_flag == 1)]
		}//[if (g_take_over_back_lane_flag != -1)]
	}//[else]


	//[以下是脱困计时器，由于某些原因目前计时器不启动]
	if (g_navi_state == S_EMERGENCY_STOP || g_navi_state == D_EMERGENCY_STOP)
	{
		if (g_emergency_roving_timer == 0)
		{//[高速下面不会进入脱困状态]
			g_emergency_counter++;

			if (g_emergency_counter > 0 && g_real_speed < (int)(10 * CM))
			{
				g_emergency_roving_timer = emergency_roving_timer;
				g_emergency_counter = 0;
				g_navi_state = ROVING;
			}
		}
		else
		{
			//[进行漫游计时器的调整]
			g_navi_state = ROVING;
			if (abs(g_cur_search_index - MORPHIN_MID_INDEX) < 5)//[基本直行]
			{
				g_emergency_roving_timer -= 1;
				if (g_emergency_roving_timer <= 0)
				{//[没有道路计时器重置]
					g_emergency_roving_timer = emergency_roving_timer;
				}
			}
			else
			{
				g_emergency_roving_timer--;
				if (g_emergency_roving_timer <= 0)
				{//[没有道路计时器重置]
					g_emergency_roving_timer = emergency_roving_timer;
				}
			}
		}
	}
	else
	{
		//[执行到这里说明 ret == 0]
		if (g_multi_lane.lane_line[1].fidelity >= 70 && g_multi_lane.lane_line[2].fidelity >= 70)
		{//[有道路，进行跟踪]
			g_emergency_roving_timer = 0;
		}

		if (g_emergency_roving_timer > 0)
		{
			//[进行漫游计时器的调整]
			//g_navi_state = ROVING;
			if (abs(g_cur_search_index - MORPHIN_MID_INDEX) < 5)//[基本直行]
			{
				g_emergency_roving_timer -= 1;
				if (g_emergency_roving_timer < 0)
				{
					g_emergency_roving_timer = 0;
				}
			}
			else if (g_navi_state == TRACE_LANE)
			{
				g_emergency_roving_timer -= 1;
				if (g_emergency_roving_timer < 0)
				{
					g_emergency_roving_timer = 0;
				}
			}
			else
			{
				g_emergency_roving_timer--;
				if (g_emergency_roving_timer < 0)
				{
					g_emergency_roving_timer = 0;
				}
			}
			g_navi_state = ROVING;
		}
		else
		{//[恢复成道路跟踪]
			if (g_navi_state != FOLLOW_THE_CAR)
			{
				if (g_road_type == 0)
				{
					g_navi_state = TRACE_LANE;
				}
				else
					g_navi_state = TRACE_IN_NATURAL_ROAD;
			}

			g_emergency_roving_timer = 0;
			g_emergency_counter = 0;
		}
	}

	return ret;
}

/*==================================================================
 * 函数名  ：	int change_lane(PL_FUNC_INPUT *pl_input, int mode)
 * 功能    ：	换道规划，给出换道用的道路。有相应的状态切换逻辑
 * 输入参数：	PL_FUNC_INPUT *pl_input		环境数据输入
				int mode					0  左换道    1  右换道
 * 输出参数：	0  可以执行  -1  不可执行
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int change_lane(PL_FUNC_INPUT *pl_input, int mode)
{
#ifdef MBUG_OPEN_
	MBUG("change_lane \n");
#endif
	int ret = 0;

	int i;
	COOR2 temp_left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	COOR2 temp_right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int temp_left_line_num = 0;
	int temp_right_line_num = 0;

	memset(temp_left_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	memset(temp_right_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));

	COOR2 temp_line[200];
	int temp_line_num = 0;
	int step;

// 	if (g_stat == S_ROAD_NAV)
// 	{
// 		//keep_trace_in_the_mid_lane();
// 		keep_trace_in_right_lane();
// 	}
// 	else
// 	{
// 		g_take_over_back_lane_flag = -1;
// 	}

	//[根据黄线信息进行判断换道]
	/*
	if (g_stat == S_ROAD_NAV || g_stat == S_CROSS_UND)
	{
		if (g_multi_lane.lane_line[0].line_color == YELLOW)
		{
			g_yellow_line_flag = 0;
			g_take_over_back_lane_flag = -1;
		}
		else if (g_multi_lane.lane_line[1].line_color == YELLOW)
		{
			g_yellow_line_flag = 1;
			g_take_over_back_lane_flag = -1;
		}
		else if (g_multi_lane.lane_line[2].line_color == YELLOW || g_multi_lane.lane_line[3].line_color == YELLOW)
		{
			g_yellow_line_flag = 2;
			if (g_stat != S_CROSS_UND)
			{
				g_take_over_back_lane_flag = 1;//[开启右回道标志]
			}
		}
		else
		{
			g_yellow_line_flag = 0;
		}
	}
	else
	{
		g_yellow_line_flag = 0;
	}
	*/

	if (g_change_lane_count == 0)
	{//[还在初始车道]
		//[从融合输入取出当前车道的左道边]
		for (i=0; i<pl_input->fu_pl_data.lines.l_edges[0].valid_num_points; i++)
		{
			temp_left_line[i].x = pl_input->fu_pl_data.lines.l_edges[0].line[i].x;
			temp_left_line[i].y = pl_input->fu_pl_data.lines.l_edges[0].line[i].y;
		}
		temp_left_line_num = pl_input->fu_pl_data.lines.l_edges[0].valid_num_points;

		memset(temp_line, 0, 200 * sizeof(COOR2));
		step = (int)((temp_left_line[temp_left_line_num - 1].y - temp_left_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1));

		line_fitting(temp_left_line, temp_left_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(temp_left_line, temp_line, temp_line_num * sizeof(COOR2));
		temp_left_line_num = temp_line_num;

		//[从融合输入取出当前车道的右道边]
		for (i=0; i<pl_input->fu_pl_data.lines.r_edges[0].valid_num_points; i++)
		{
			temp_right_line[i].x = pl_input->fu_pl_data.lines.r_edges[0].line[i].x;
			temp_right_line[i].y = pl_input->fu_pl_data.lines.r_edges[0].line[i].y;
		}
		temp_right_line_num = pl_input->fu_pl_data.lines.r_edges[0].valid_num_points;

		memset(temp_line, 0, 200 * sizeof(COOR2));
		step = (int)((temp_right_line[temp_right_line_num - 1].y - temp_right_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1));

		line_fitting(temp_right_line, temp_right_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(temp_right_line, temp_line, temp_line_num * sizeof(COOR2));
		temp_right_line_num = temp_line_num;



		//[上面一段会出现没有边的情况]
		temp_left_line_num = g_multi_lane.lane_line[1].valid_num_points;
		memcpy(temp_left_line, g_multi_lane.lane_line[1].line, temp_left_line_num * sizeof(COOR2));
		memset(temp_line, 0, 200 * sizeof(COOR2));
		step = (int)((temp_left_line[temp_left_line_num - 1].y - temp_left_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1));
		line_fitting(temp_left_line, temp_left_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(temp_left_line, temp_line, temp_line_num * sizeof(COOR2));
		temp_left_line_num = temp_line_num;

		//[**************************]
		temp_right_line_num = g_multi_lane.lane_line[2].valid_num_points;
		memcpy(temp_right_line, g_multi_lane.lane_line[2].line, temp_right_line_num * sizeof(COOR2));
		memset(temp_line, 0, 200 * sizeof(COOR2));
		step = (int)((temp_right_line[temp_right_line_num - 1].y - temp_right_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1));
		line_fitting(temp_right_line, temp_right_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(temp_right_line, temp_line, temp_line_num * sizeof(COOR2));
		temp_right_line_num = temp_line_num;


		ret = check_the_lane(temp_left_line, temp_left_line_num, temp_right_line, temp_right_line_num);

		//[@@这个条件会不会进入，有待测试和考虑]
		if (ret == -2)
		{
			g_navi_state = S_EMERGENCY_STOP;
			g_take_over_back_lane_flag = -1;
			return ret;
		}

		//[换道过程中，初始车道只要有动态障碍物就继续保持换道]
		DYN_OBS temp_dyn_obs;
		for (i=0; i<g_dyn_obs_num; i++)
		{
			int l_x = 0;
			int r_x = 0;
			temp_dyn_obs = g_dyn_obs[i];
			if (temp_dyn_obs.center.y <= 400)
			{
				continue;
			}
			if (temp_dyn_obs.speed.y > g_cur_speed && temp_dyn_obs.center.y - 500 < g_unsafe_dist)
			{
				ret = 0;
				continue;
			}
			get_x_coord(temp_dyn_obs.center.y, temp_left_line, temp_left_line_num, &l_x);
			get_x_coord(temp_dyn_obs.center.y, temp_right_line, temp_right_line_num, &r_x);
			if (temp_dyn_obs.center.x > l_x && temp_dyn_obs.center.x < r_x)
			{
				if (mode == 0)
				{
					g_navi_state = CHANGE_TO_LEFT_LANE;
				}
				else
				{
					g_navi_state = CHANGE_TO_RIGHT_LANE;
				}
				//
				ret = -1;
				g_take_over_back_lane_flag = -1;
				break;
			}
		}

		if (ret == 0 && (g_take_over_back_lane_flag == -1 \
			|| (g_take_over_back_lane_flag == 0 && mode == 1) \
			|| (g_take_over_back_lane_flag == 1 && mode == 0)))
		{//[原来道路可以通行，且没有回道标志，则恢复道路跟踪]

			//[更新历史数据，恢复正常的道路跟踪]
			memcpy(g_multi_lane.lane_line[1].line, temp_left_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			memcpy(g_multi_lane.lane_line[2].line, temp_right_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			g_multi_lane.ok_times[1] = 2;
			g_multi_lane.ok_times[2] = 2;
			memcpy(&g_multi_lane.gps_info, &pl_input->state.pos, sizeof(POSE_INFO));

			g_take_over_flag = 0;
			g_navi_state = TRACE_LANE;
			g_change_lane_count = 0;
			g_change_lane_search_dist = MAX_VALUE;
			g_unsafe_dist_for_change_lane = MAX_VALUE;

			g_change_lane_direct_keep = -1;

			if (g_change_lane_see == -1)
			{//[停止左换道]
				g_change_lane_see = 0;
#ifdef MBUG_OPEN_
				MBUG("Stop changing to left......continue to trace cur lane\n");
#endif
				return ret;
			}

			else if (g_change_lane_see == 1)
			{//[停止右换道]
				g_change_lane_see = 0;
#ifdef MBUG_OPEN_
				MBUG("Stop changing to right......continue to trace cur lane\n");
#endif
				return ret;
			}
		}//[if (ret == 0 && g_take_over_back_lane_flag == -1)]
		else
		{//[当前车道危险点的距离]
			g_unsafe_dist_for_change_lane = g_unsafe_dist;
		}
	}//[if (g_change_lane_count == 0)]
	else
	{//[已经在目标车道，检测初始车道障碍物位置]
		if (mode == 0)
		{//[左换道，取2、3，道边不稳定，根据已知的补出]
			if (g_multi_lane.lane_line[2].valid_num_points >= 2 && g_multi_lane.lane_line[3].valid_num_points >= 2)
			{
				for (i = 0; i < g_multi_lane.lane_line[2].valid_num_points; i++)
				{
					temp_left_line[i].x = g_multi_lane.lane_line[2].line[i].x;
					temp_left_line[i].y = g_multi_lane.lane_line[2].line[i].y;
				}
				temp_left_line_num = g_multi_lane.lane_line[2].valid_num_points;

				for (i=0; i < g_multi_lane.lane_line[3].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[3].line[i].x;
					temp_right_line[i].y = g_multi_lane.lane_line[3].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[3].valid_num_points;
			}
			else if (g_multi_lane.lane_line[2].valid_num_points >= 2 && g_multi_lane.lane_line[3].valid_num_points < 2)
			{
				for (i = 0; i < g_multi_lane.lane_line[2].valid_num_points; i++)
				{
					temp_left_line[i].x = g_multi_lane.lane_line[2].line[i].x;
					temp_left_line[i].y = g_multi_lane.lane_line[2].line[i].y;
				}
				temp_left_line_num = g_multi_lane.lane_line[2].valid_num_points;

				for (i=0; i < temp_left_line_num; i++)
				{
					temp_right_line[i].x = temp_left_line[i].x + 375;
					temp_right_line[i].y = temp_left_line[i].y;
				}
				temp_right_line_num = temp_left_line_num;
			}
			else if (g_multi_lane.lane_line[2].valid_num_points < 2 && g_multi_lane.lane_line[3].valid_num_points >= 2)
			{
				for (i=0; i < g_multi_lane.lane_line[3].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[3].line[i].x;
					temp_right_line[i].y = g_multi_lane.lane_line[3].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[3].valid_num_points;

				for (i = 0; i < temp_right_line_num; i++)
				{
					temp_left_line[i].x = temp_right_line[i].x - 375;
					temp_left_line[i].y = temp_right_line[i].y;
				}
				temp_left_line_num = temp_right_line_num;
			}

			ret = check_the_lane(temp_left_line, temp_left_line_num, temp_right_line, temp_right_line_num);
			//[检测危险点后面10m]
			g_change_lane_search_dist = g_unsafe_dist + 1000;
			if (g_change_lane_search_dist > 5000)
			{
				g_change_lane_search_dist = 5000;
			}

			if (g_change_lane_search_dist < 1500 || g_change_lane_search_dist == 5000)
			{//[车辆已经基本越过目标障碍物，清空状态]
				g_change_lane_see = 0;
				g_change_lane_count = 0;
				g_take_over_flag = 0;
				g_navi_state = TRACE_LANE;
				g_take_over_back_lane_flag = -1;
				g_change_fade_timer = 10;
				g_change_mode = 0;
				g_change_lane_direct_keep = -1;
				return 0;
			}
		}//[if (mode == 0)]
		else
		{//[右换道，取0、1，道边不稳定，根据已知的补出]
			if (g_multi_lane.lane_line[0].valid_num_points >= 2 && g_multi_lane.lane_line[1].valid_num_points >= 2)
			{
				for (i = 0; i < g_multi_lane.lane_line[0].valid_num_points; i++)
				{
					temp_left_line[i].x = g_multi_lane.lane_line[0].line[i].x;
					temp_left_line[i].y = g_multi_lane.lane_line[0].line[i].y;
				}
				temp_left_line_num = g_multi_lane.lane_line[0].valid_num_points;

				for (i=0; i < g_multi_lane.lane_line[1].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[1].line[i].x;
					temp_right_line[i].y = g_multi_lane.lane_line[1].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[1].valid_num_points;
			}
			else if (g_multi_lane.lane_line[0].valid_num_points >= 2 && g_multi_lane.lane_line[1].valid_num_points < 2)
			{
				for (i = 0; i < g_multi_lane.lane_line[1].valid_num_points; i++)
				{
					temp_left_line[i].x = g_multi_lane.lane_line[0].line[i].x;
					temp_left_line[i].y = g_multi_lane.lane_line[0].line[i].y;
				}
				temp_left_line_num = g_multi_lane.lane_line[0].valid_num_points;

				for (i=0; i < temp_left_line_num; i++)
				{
					temp_right_line[i].x = temp_left_line[i].x + 375;
					temp_right_line[i].y = temp_left_line[i].y;
				}
				temp_right_line_num = temp_left_line_num;
			}
			else if (g_multi_lane.lane_line[0].valid_num_points < 2 && g_multi_lane.lane_line[1].valid_num_points >= 2)
			{
				for (i=0; i < g_multi_lane.lane_line[1].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[1].line[i].x;
					temp_right_line[i].y = g_multi_lane.lane_line[1].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[1].valid_num_points;

				for (i = 0; i < temp_right_line_num; i++)
				{
					temp_left_line[i].x = temp_right_line[i].x - 375;
					temp_left_line[i].y = temp_right_line[i].y;
				}
				temp_left_line_num = temp_right_line_num;
			}
			else if (g_multi_lane.lane_line[2].valid_num_points > 0)
			{
				for (i=0; i < g_multi_lane.lane_line[2].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[2].line[i].x - 375;
					temp_right_line[i].y = g_multi_lane.lane_line[2].line[i].y;

					temp_left_line[i].x = g_multi_lane.lane_line[2].line[i].x - 750;
					temp_left_line[i].y = g_multi_lane.lane_line[2].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[2].valid_num_points;
				temp_left_line_num = g_multi_lane.lane_line[2].valid_num_points;

			}

			ret = check_the_lane(temp_left_line, temp_left_line_num, temp_right_line, temp_right_line_num);
			//[检测危险点后面10m]
			g_change_lane_search_dist = g_unsafe_dist + 1000;
			if (g_change_lane_search_dist > 5000)
			{
				g_change_lane_search_dist = 5000;
			}

			if (g_change_lane_search_dist < 1500 || g_change_lane_search_dist == 5000)
			{//[车辆已经基本越过目标障碍物，清空状态]
				g_change_lane_see = 0;
				g_change_lane_count = 0;
				g_take_over_flag = 0;
				g_navi_state = TRACE_LANE;
				g_take_over_back_lane_flag = -1;
				g_change_fade_timer = 10;
				g_change_mode = 0;
				g_change_lane_direct_keep = -1;
				return 0;
			}
		}//[else]
	}//[else]

	//[为了连续避障，上面检测出最近危险点，下面将两个道路合并成一条道路，检测距离加长到危险点之后10m，]
	//[在这段道路上检测可通行性。]
	if (0 == mode)
	{//[左换道]
		if (g_change_lane_see == 0 && g_change_lane_count > 0)
		{
			//[已经在左方车道，现在的1、2、3对应换道前0、1、2]
			if (g_multi_lane.lane_line[1].valid_num_points >= 2 && g_multi_lane.lane_line[3].valid_num_points >= 2)
			{
				for (i = 0; i < g_multi_lane.lane_line[1].valid_num_points; i++)
				{
					temp_left_line[i].x = g_multi_lane.lane_line[1].line[i].x;
					temp_left_line[i].y = g_multi_lane.lane_line[1].line[i].y;
				}
				temp_left_line_num = g_multi_lane.lane_line[1].valid_num_points;

				for (i=0; i < g_multi_lane.lane_line[3].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[3].line[i].x;
					temp_right_line[i].y = g_multi_lane.lane_line[3].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[3].valid_num_points;
			}
			else if (g_multi_lane.lane_line[1].valid_num_points >= 2 && g_multi_lane.lane_line[3].valid_num_points < 2)
			{
				for (i = 0; i < g_multi_lane.lane_line[1].valid_num_points; i++)
				{
					temp_left_line[i].x = g_multi_lane.lane_line[1].line[i].x;
					temp_left_line[i].y = g_multi_lane.lane_line[1].line[i].y;
				}
				temp_left_line_num = g_multi_lane.lane_line[1].valid_num_points;

				for (i=0; i < temp_left_line_num; i++)
				{
					temp_right_line[i].x = temp_left_line[i].x + 700;
					temp_right_line[i].y = temp_left_line[i].y;
				}
				temp_right_line_num = temp_left_line_num;
			}
			else if (g_multi_lane.lane_line[1].valid_num_points < 2 && g_multi_lane.lane_line[3].valid_num_points >= 2)
			{
				for (i=0; i < g_multi_lane.lane_line[3].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[3].line[i].x;
					temp_right_line[i].y = g_multi_lane.lane_line[3].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[3].valid_num_points;

				for (i = 0; i < temp_right_line_num; i++)
				{
					temp_left_line[i].x = temp_right_line[i].x - 700;
					temp_left_line[i].y = temp_right_line[i].y;
				}
				temp_left_line_num = temp_right_line_num;
			}
		}//[if (g_change_lane_see == 0 && g_change_lane_count > 0)]
		else
		{//[还在初始车道]

			if (g_multi_lane.lane_line[0].valid_num_points >= 2 && g_multi_lane.lane_line[2].valid_num_points >= 2)
			{
				for (i = 0; i < g_multi_lane.lane_line[0].valid_num_points; i++)
				{
					temp_left_line[i].x = g_multi_lane.lane_line[0].line[i].x;
					temp_left_line[i].y = g_multi_lane.lane_line[0].line[i].y;
				}
				temp_left_line_num = g_multi_lane.lane_line[0].valid_num_points;

				for (i=0; i < g_multi_lane.lane_line[2].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[2].line[i].x;
					temp_right_line[i].y = g_multi_lane.lane_line[2].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[2].valid_num_points;
			}
			else if (g_multi_lane.lane_line[0].valid_num_points >= 2 && g_multi_lane.lane_line[2].valid_num_points < 2)
			{
				for (i = 0; i < g_multi_lane.lane_line[0].valid_num_points; i++)
				{
					temp_left_line[i].x = g_multi_lane.lane_line[0].line[i].x;
					temp_left_line[i].y = g_multi_lane.lane_line[0].line[i].y;
				}
				temp_left_line_num = g_multi_lane.lane_line[0].valid_num_points;

				for (i=0; i < temp_left_line_num; i++)
				{
					temp_right_line[i].x = temp_left_line[i].x + 700;
					temp_right_line[i].y = temp_left_line[i].y;
				}
				temp_right_line_num = temp_left_line_num;
			}
			else if (g_multi_lane.lane_line[0].valid_num_points < 2 && g_multi_lane.lane_line[2].valid_num_points >= 2)
			{
				for (i=0; i < g_multi_lane.lane_line[2].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[2].line[i].x;
					temp_right_line[i].y = g_multi_lane.lane_line[2].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[2].valid_num_points;

				for (i = 0; i < temp_right_line_num; i++)
				{
					temp_left_line[i].x = temp_right_line[i].x - 700;
					temp_left_line[i].y = temp_right_line[i].y;
				}
				temp_left_line_num = temp_right_line_num;
			}
		}//[if (-1 == g_change_lane_see)]
	
#ifdef MBUG_OPEN_
		MBUG("mode : %d,  g_change_lane_see ; %d  \n", mode, g_change_lane_see);
		MBUG("get change lane to left\n");
		for (i=0; i<temp_left_line_num; i++)
		{
			MBUG("(%d, %d) ", temp_left_line[i].x, temp_left_line[i].y);
		}
		MBUG("\n");
		for (i=0; i<temp_right_line_num; i++)
		{
			MBUG("(%d, %d) ", temp_right_line[i].x, temp_right_line[i].y);
		}
		MBUG("\n");
#endif
	}//[if (0 == mode)]
	else
	{//[右换道]

		if (g_change_lane_see == 0 && g_change_lane_count > 0)
		{//[已经在右方车道，现在的0、1、2对应换道前1、2、3]
			if (g_multi_lane.lane_line[0].valid_num_points >= 2 && g_multi_lane.lane_line[2].valid_num_points >= 2)
			{
				for (i = 0; i < g_multi_lane.lane_line[0].valid_num_points; i++)
				{
					temp_left_line[i].x = g_multi_lane.lane_line[0].line[i].x;
					temp_left_line[i].y = g_multi_lane.lane_line[0].line[i].y;
				}
				temp_left_line_num = g_multi_lane.lane_line[0].valid_num_points;

				for (i=0; i < g_multi_lane.lane_line[2].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[2].line[i].x;
					temp_right_line[i].y = g_multi_lane.lane_line[2].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[2].valid_num_points;
			}
			else if (g_multi_lane.lane_line[0].valid_num_points >= 2 && g_multi_lane.lane_line[2].valid_num_points < 2)
			{
				for (i = 0; i < g_multi_lane.lane_line[1].valid_num_points; i++)
				{
					temp_left_line[i].x = g_multi_lane.lane_line[0].line[i].x;
					temp_left_line[i].y = g_multi_lane.lane_line[0].line[i].y;
				}
				temp_left_line_num = g_multi_lane.lane_line[0].valid_num_points;

				for (i=0; i < temp_left_line_num; i++)
				{
					temp_right_line[i].x = temp_left_line[i].x + 700;
					temp_right_line[i].y = temp_left_line[i].y;
				}
				temp_right_line_num = temp_left_line_num;
			}
			else if (g_multi_lane.lane_line[0].valid_num_points < 2 && g_multi_lane.lane_line[2].valid_num_points >= 2)
			{
				for (i=0; i < g_multi_lane.lane_line[2].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[2].line[i].x;
					temp_right_line[i].y = g_multi_lane.lane_line[2].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[2].valid_num_points;

				for (i = 0; i < temp_right_line_num; i++)
				{
					temp_left_line[i].x = temp_right_line[i].x - 700;
					temp_left_line[i].y = temp_right_line[i].y;
				}
				temp_left_line_num = temp_right_line_num;
			}
		}//[if (g_change_lane_see == 0 && g_change_lane_count > 0)]

		else
		{//[还在初始车道]

			if (g_multi_lane.lane_line[1].valid_num_points >= 2 && g_multi_lane.lane_line[3].valid_num_points >= 2)
			{
				for (i = 0; i < g_multi_lane.lane_line[1].valid_num_points; i++)
				{
					temp_left_line[i].x = g_multi_lane.lane_line[1].line[i].x;
					temp_left_line[i].y = g_multi_lane.lane_line[1].line[i].y;
				}
				temp_left_line_num = g_multi_lane.lane_line[1].valid_num_points;

				for (i=0; i < g_multi_lane.lane_line[3].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[3].line[i].x;
					temp_right_line[i].y = g_multi_lane.lane_line[3].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[3].valid_num_points;
			}
			else if (g_multi_lane.lane_line[1].valid_num_points >= 2 && g_multi_lane.lane_line[3].valid_num_points < 2)
			{
				for (i = 0; i < g_multi_lane.lane_line[1].valid_num_points; i++)
				{
					temp_left_line[i].x = g_multi_lane.lane_line[1].line[i].x;
					temp_left_line[i].y = g_multi_lane.lane_line[1].line[i].y;
				}
				temp_left_line_num = g_multi_lane.lane_line[1].valid_num_points;

				for (i=0; i < temp_left_line_num; i++)
				{
					temp_right_line[i].x = temp_left_line[i].x + 700;
					temp_right_line[i].y = temp_left_line[i].y;
				}
				temp_right_line_num = temp_left_line_num;
			}
			else if (g_multi_lane.lane_line[1].valid_num_points < 2 && g_multi_lane.lane_line[3].valid_num_points >= 2)
			{
				for (i=0; i < g_multi_lane.lane_line[3].valid_num_points; i++)
				{
					temp_right_line[i].x = g_multi_lane.lane_line[3].line[i].x;
					temp_right_line[i].y = g_multi_lane.lane_line[3].line[i].y;
				}
				temp_right_line_num = g_multi_lane.lane_line[3].valid_num_points;

				for (i = 0; i < temp_right_line_num; i++)
				{
					temp_left_line[i].x = temp_right_line[i].x - 700;
					temp_left_line[i].y = temp_right_line[i].y;
				}
				temp_left_line_num = temp_right_line_num;
			}

		}//[if (1 == g_change_lane_see)]
#ifdef MBUG_OPEN_
		MBUG("mode : %d,  g_change_lane_see ; %d  \n", mode, g_change_lane_see);
		MBUG("get change lane to right\n");
		for (i=0; i<temp_left_line_num; i++)
		{
			MBUG("(%d, %d) ", temp_left_line[i].x, temp_left_line[i].y);
		}
		MBUG("\n");
		for (i=0; i<temp_right_line_num; i++)
		{
			MBUG("(%d, %d) ", temp_right_line[i].x, temp_right_line[i].y);
		}
		MBUG("\n");
#endif
	}//[else]

	g_change_lane_search_flag = 1;
	g_change_lane_search_dist = g_unsafe_dist + 1000;
	if (g_change_lane_search_dist > 5000)
	{
		g_change_lane_search_dist = 5000;
	}
	ret = check_the_lane(temp_left_line, temp_left_line_num, temp_right_line, temp_right_line_num);
	g_change_lane_search_flag = 0;

	if (ret != 0)
	{
		g_navi_state = ROVING;
		g_take_over_back_lane_flag = -1;
	}

	return ret;
}

/*==================================================================
 * 函数名  ：	int trace_in_natural_road()
 * 功能    ：	非结构化道路跟踪
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	0  可以执行  -1  不可执行
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int trace_in_natural_road()
{
	int ret = 0;

	int i;
	COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int left_line_num = 0;
	int right_line_num = 0;

	memset(left_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	memset(right_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));

	if (g_road_type == 0)
	{

		//[从全局变量中获得当前道边]
		//[@@首先要确定道路是否能用]
		//[先检测是否有可以使用的结构化道路]
		//[0、1、2、3车道线]
		if (g_multi_lane.lane_line[1].valid_num_points > 1 && g_multi_lane.lane_line[2].valid_num_points > 1)
		{//[1、2能用]
			memcpy(left_line, g_multi_lane.lane_line[1].line, g_multi_lane.lane_line[1].valid_num_points * sizeof(COOR2));
			left_line_num = g_multi_lane.lane_line[1].valid_num_points;

			memcpy(right_line, g_multi_lane.lane_line[2].line, g_multi_lane.lane_line[2].valid_num_points * sizeof(COOR2));
			right_line_num = g_multi_lane.lane_line[2].valid_num_points;
#ifdef MBUG_OPEN_
			MBUG("trace lane 1 2:\n");
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", left_line[i].x, left_line[i].y);
			}
			MBUG("\n");
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", right_line[i].x, right_line[i].y);
			}
			MBUG("\n");
#endif
			ret = check_the_lane(left_line, left_line_num, right_line, right_line_num);

			if (ret == -1)
			{
				g_navi_state = S_EMERGENCY_STOP;
			}
			else
			{
				g_navi_state = TRACE_LANE;
			}

			return ret;
		}
		else if (g_multi_lane.lane_line[0].valid_num_points > 1 && g_multi_lane.lane_line[2].valid_num_points > 1)
		{//[0、2能用]
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				left_line[i].x = (g_multi_lane.lane_line[0].line[i].x + g_multi_lane.lane_line[2].line[i].x) / 2;
				left_line[i].y = g_multi_lane.lane_line[2].line[i].y;
				left_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			}

			memcpy(right_line, g_multi_lane.lane_line[2].line, g_multi_lane.lane_line[2].valid_num_points * sizeof(COOR2));
			right_line_num = g_multi_lane.lane_line[2].valid_num_points;
#ifdef MBUG_OPEN_
			MBUG("trace lane (0 + 2 -> 1) 2:\n");
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", left_line[i].x, left_line[i].y);
			}
			MBUG("\n");
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", right_line[i].x, right_line[i].y);
			}
			MBUG("\n");
#endif
			ret = check_the_lane(left_line, left_line_num, right_line, right_line_num);

			if (ret == -1)
			{
				g_navi_state = S_EMERGENCY_STOP;
			}
			else
			{
				g_navi_state = TRACE_LANE;
			}
			return ret;
		}
		else if (g_multi_lane.lane_line[1].valid_num_points > 1 && g_multi_lane.lane_line[3].valid_num_points > 1)
		{//[1、3能用]
			memcpy(left_line, g_multi_lane.lane_line[1].line, g_multi_lane.lane_line[1].valid_num_points * sizeof(COOR2));
			left_line_num = g_multi_lane.lane_line[1].valid_num_points;

			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				right_line[i].x = (g_multi_lane.lane_line[1].line[i].x + g_multi_lane.lane_line[3].line[i].x) / 2;
				right_line[i].y = g_multi_lane.lane_line[1].line[i].y;
				right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			}
#ifdef MBUG_OPEN_
			MBUG("trace lane 1 (1 + 3 -> 2):\n");
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", left_line[i].x, left_line[i].y);
			}
			MBUG("\n");
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", right_line[i].x, right_line[i].y);
			}
			MBUG("\n");
#endif
			ret = check_the_lane(left_line, left_line_num, right_line, right_line_num);

			if (ret == -1)
			{
				g_navi_state = S_EMERGENCY_STOP;
			}
			else
			{
				g_navi_state = TRACE_LANE;
			}
			return ret;
		}
		else if (g_multi_lane.lane_line[1].valid_num_points > 1)
		{//[1能用，补出最低限度能走的车道375cm]
			memcpy(left_line, g_multi_lane.lane_line[1].line, g_multi_lane.lane_line[1].valid_num_points * sizeof(COOR2));
			left_line_num = g_multi_lane.lane_line[1].valid_num_points;

			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				right_line[i].x = g_multi_lane.lane_line[1].line[i].x + 375;
				right_line[i].y = g_multi_lane.lane_line[1].line[i].y;
				right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			}
#ifdef MBUG_OPEN_
			MBUG("trace lane 1 (1 + 375 -> 2):\n");
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", left_line[i].x, left_line[i].y);
			}
			MBUG("\n");
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", right_line[i].x, right_line[i].y);
			}
			MBUG("\n");
#endif
			ret = check_the_lane(left_line, left_line_num, right_line, right_line_num);

			if (ret == -1)
			{
				g_navi_state = S_EMERGENCY_STOP;
			}
			else
			{
				g_navi_state = TRACE_LANE;
			}
			return ret;
		}
		else if (g_multi_lane.lane_line[2].valid_num_points > 1)
		{//[2能用，补出最低限度能走的车道375cm]
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				left_line[i].x = g_multi_lane.lane_line[2].line[i].x - 375;
				left_line[i].y = g_multi_lane.lane_line[2].line[i].y;
				left_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			}

			memcpy(right_line, g_multi_lane.lane_line[2].line, g_multi_lane.lane_line[2].valid_num_points * sizeof(COOR2));
			right_line_num = g_multi_lane.lane_line[2].valid_num_points;
#ifdef MBUG_OPEN_
			MBUG("trace lane (2 - 375 -> 1) 2:\n");
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", left_line[i].x, left_line[i].y);
			}
			MBUG("\n");
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", right_line[i].x, right_line[i].y);
			}
			MBUG("\n");
#endif

			ret = check_the_lane(left_line, left_line_num, right_line, right_line_num);

			if (ret == -1)
			{
				g_navi_state = S_EMERGENCY_STOP;
			}
			else
			{
				g_navi_state = TRACE_LANE;
			}
			return ret;
		}
		else
		{//[没有可用行车线车道，检查有无自然道边]
			if (g_natural_boundary.l_nums > 2 && g_natural_boundary.r_nums > 2)
			{
				g_navi_state = TRACE_IN_NATURAL_ROAD;
			}
			else
			{
				g_navi_state = ROVING;
				return ret;
			}
		}
	}

	ret = check_the_lane(g_natural_boundary.l_boundary, g_natural_boundary.l_nums, \
		g_natural_boundary.r_boundary, g_natural_boundary.r_nums);

	int l_dyn_obs_index = -1;//[记录当前车道的动态目标]
	int min_y = MAX_VALUE;
	int min_index = MAX_VALUE;
	DYN_OBS temp_dyn_obs;
	for (i = 0; i < g_dyn_obs_num; i++)
	{
		temp_dyn_obs = g_dyn_obs[i];
		if (temp_dyn_obs.center.x > -200 && temp_dyn_obs.center.x < 200)
		{//[找到在当前车道，且离车最近的动态障碍物]
			if (temp_dyn_obs.center.y > 400 && temp_dyn_obs.center.y < min_y)
			{//[在车后面的动态障碍物不考虑，这里使用>400考虑到不会有和车体重合的障碍物，而且后方车辆会避免与我方车辆碰撞]
				min_y = temp_dyn_obs.center.y;
				min_index = i;
			}
		}
	}

	g_car_ahead_dist = MAX_VALUE;

	if (min_index != MAX_VALUE)
	{//[有影响车辆行驶的动态障碍物存在]
		temp_dyn_obs = g_dyn_obs[min_index];
		l_dyn_obs_index = min_index;

		//[@@由于融合给出的动态障碍物栅格清除不干净，这里做个处理，]
		//[认为动态障碍物下方的区域静态为空，所以不对静态障碍物判断]
		if (temp_dyn_obs.nearest_pt.y - 200 < g_unsafe_dist)
		{//[若是动态障碍物比危险点近，那么变为处理动态障碍物(减去4m是因为有的动障碍物会有拖尾巴)]
			ret = 0;
		}

		if (ret == 0)
		{//[没有静态障碍物堵塞，则跟随]
#ifdef MBUG_OPEN_
			MBUG("dyn obs dist %d \n", temp_dyn_obs.center.y);
			MBUG("dyn obs speed %d \n", temp_dyn_obs.speed.y);
			MBUG("follow the dyn obs\n");
#endif
			g_take_over_flag = 0;
			g_navi_state = FOLLOW_THE_CAR;
			g_follow_car_speed = temp_dyn_obs.speed.y;
		}
	}//[if (min_index != MAX_VALUE)]
	else
	{
		if (ret == -1 || ret == -2)
		{//[非结构化道路不可通行，启动紧急定时器]
			if (g_emergency_roving_timer == 0)
			{
				g_emergency_counter++;

				if (g_emergency_counter > 0)
				{
					g_emergency_roving_timer = emergency_roving_timer;
					g_emergency_counter = 0;
					g_navi_state = ROVING;
				}
			}
			else
			{
				//[进行漫游计时器的调整]
				g_navi_state = ROVING;
				if (abs(g_cur_search_index - MORPHIN_MID_INDEX) < 5)//[基本直行]
				{
					g_emergency_roving_timer -= 1;
					if (g_emergency_roving_timer < 0)
					{
						g_emergency_roving_timer = 0;
					}
				}
				else
				{
					g_emergency_roving_timer--;
					if (g_emergency_roving_timer < 0)
					{
						g_emergency_roving_timer = 0;
					}
				}
			}
		}//[if (ret == -1)]
		else
		{
			if (g_emergency_roving_timer > 0)
			{
				//[进行漫游计时器的调整]
				//g_navi_state = ROVING;
				if (abs(g_cur_search_index - MORPHIN_MID_INDEX) < 5)//[基本直行]
				{
					g_emergency_roving_timer -= 1;
					if (g_emergency_roving_timer < 0)
					{
						g_emergency_roving_timer = 0;
					}
				}
				else if (g_navi_state == TRACE_LANE)
				{
					g_emergency_roving_timer -= 1;
					if (g_emergency_roving_timer < 0)
					{
						g_emergency_roving_timer = 0;
					}
				}
				else
				{
					g_emergency_roving_timer--;
					if (g_emergency_roving_timer < 0)
					{
						g_emergency_roving_timer = 0;
					}
				}
			}
			else
			{
				g_emergency_roving_timer = 0;
				g_emergency_counter = 0;
			}
		}//[else]
	}



	return ret;
}

//***********************************************************************************************
//                                zgccmax 2012.Feb.17
//void reset_for_wait()
//param:    void
//return:   void
//discribe: 等待状态下清除所有变量，所有状态变为初始状态。@@以后增加新的状态注意在此处增加清除代码。
//***********************************************************************************************
void reset_for_wait()
{
	int i = 0;
	odo_init_flag = 0;

	g_sharp_turn_flag = 0;
	memset(&g_sharp_turn_last_pt, 0, sizeof(COOR2));
	memset(&g_sharp_turn_slow_down_s_pt, 0, sizeof(COOR2));
	memset(&g_sharp_turn_slow_down_e_pt, 0, sizeof(COOR2));

	g_cur_speed = 0;
	g_last_speed = 0;
	g_real_speed = 0;
	g_vibration_count = 0;
	g_vibration_flag = 0;
	g_slow_down_dist = MAX_VALUE;
	g_unsafe_dist = MAX_VALUE;
	g_unsafe_dist_for_change_lane = MAX_VALUE;

	g_natural_road_search_dist = MAX_VALUE;

	g_speed_limit_sign = 0;
	g_speed_limi_pl = 0;

	g_cross_adjust_count = 0;
	g_cross_adjust_flag = 0;

	//[道路跟踪]
	memset(g_line_avg, 0, LINE_AVG_NUM * sizeof(COOR2));
	g_line_avg_num = 0;
	g_line_avg_index = 0;
	g_line_avg_last_index = 0;


	memset(&g_multi_lane, 0, sizeof(MULTI_LANE));
	if (g_road_type == 0)
	{
		g_navi_state = TRACE_LANE;
	}
	else
	{
		g_navi_state = TRACE_IN_NATURAL_ROAD;
	}
	
	g_change_lane_state = NORMAL_CHANGE;
	g_change_lane_count = 0;
	g_change_lane_see = 0;
	g_change_fade_timer = 0;
	g_change_mode = -1;
	g_change_steer_limit = 0;
	g_change_lane_unsafe_dist = MAX_VALUE;
	g_change_lane_direct_keep = -1;

	g_change_lane_search_dist = MAX_VALUE;
	g_change_lane_search_flag = 0;

	g_take_over_flag = 0;
	g_take_over_speed_up_timer = 0;
	g_take_over_back_lane_flag = -1;

	g_car_ahead_dist = MAX_VALUE;

	g_emergency_counter = 0;					//[陷入困境计时]
	g_emergency_roving_timer = 0;			//[摆脱困境计时]
	g_cur_search_index = -1;

	g_is_s_obs = 0;
	g_is_s_obs_confirm_time = 0;
	g_is_not_s_obs_confirm_time = 0;
	g_s_obs_quit = 0;
	g_s_obs_quit_timer = 0;
	g_s_obs_speed_down = 0;
	g_s_obs_last_angle = -1;
	memset(&g_s_quit_pt, 0, sizeof(COOR2));
	memset(&g_e_quit_pt, 0, sizeof(COOR2));

	//[交叉路口]
	memset(subgoal_show, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	//	g_morphin_index_test = -1;
	g_cross_flag = 0;
	g_subgoal_adjust_flag = 0;
	g_cross_start_yaw = 0;
	g_cross_cur_yaw = 0;
	g_cross_reach_goal_yaw = 0;
	memset(g_cross_subgoal_pts, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	g_cross_subgoal_pts_num = 0;
	g_cross_cur_subgoal = 0;
	g_cross_reach_goal = 0;
	g_finish_cross = 0;
	memset(&g_cross_smooth_start_pt, 0, sizeof(COOR2));
	memset(&g_cross_smooth_end_pt, 0, sizeof(COOR2));

	// 	memset(g_cross_avg_angle, 0, CROSS_AVG_NUM * sizeof(double));
	// 	g_cross_avg_angle_num = 0;
	for (i=0; i<CROSS_AVG_NUM; i++)
	{
		g_cross_avg_angle[i] = 90.0;
	}
	g_cross_avg_angle_num = 0;
	g_cross_avg_index = 0;
	g_cross_last_index = MORPHIN_MID_INDEX;


	for (i=0; i<S_OBS_AVG_NUM; i++)
	{
		g_s_obs_avg_angle[i] = 90.0;
	}
	g_s_obs_avg_angle_num = 0;
	g_s_obs_avg_index = 0;



	//	memset(g_cross_avg_path, 0, CROSS_AVG_NUM * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	//	g_cross_avg_path_num = 0;
	g_cross_avg_index = 0;
	g_cross_travel_rate = 0;
	g_cross_travel_dist = 2000;

	//[倒车]
// 	g_reverse_pts.clear();
// 	g_reverse_flag = 0;
// 	memset(&g_reverse_start_pt, 0, sizeof(COOR2));
// 	g_reverse_forward_sum_dist = 0;
// 	g_reverse_back_sum_dist = 0;
// 	memset(&g_reverse_last_pt, 0, sizeof(COOR2));
// 	memset(&g_reverse_back_last_pt, 0, sizeof(COOR2));
}

/*==================================================================
 * 函数名  ：	void keep_multi_lane(PL_FUNC_INPUT *pl_input)
 * 功能    ：	维持规划用的多车道数据，用来换道
 * 输入参数：	PL_FUNC_INPUT *pl_input		环境数据输入
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void keep_multi_lane(PL_FUNC_INPUT *pl_input)
{
	int i, j;
	COOR2 temp_line[200];
	int temp_line_num = 0;
	PL_EDGE lane[LANE_LINE_NUM];
	POSE_INFO gps_info;
	int step;

	COOR2 old_pt;
	COOR2 cur_pt;

	memset(lane, 0, LANE_LINE_NUM * sizeof(PL_EDGE));

	memcpy(&gps_info, &pl_input->state.pos, sizeof(POSE_INFO));

	//[0 1 2 3 四条边，1 为当前车道左侧边 2 为当前车道的右侧边]
	//[只取左中右三个车道共四条边]
	if (pl_input->fu_pl_data.lines.valid_num_in_l_wing >= 2)
	{
		lane[0].dir_type_hor = pl_input->fu_pl_data.lines.l_edges[1].dir_type_hor;
		lane[0].fidelity = pl_input->fu_pl_data.lines.l_edges[1].fidelity;
		lane[0].line_color = pl_input->fu_pl_data.lines.l_edges[1].line_color;
		lane[0].line_type = pl_input->fu_pl_data.lines.l_edges[1].line_type;
		for (i = 0; i < pl_input->fu_pl_data.lines.l_edges[1].valid_num_points; i++)
		{
			lane[0].line[i].x = pl_input->fu_pl_data.lines.l_edges[1].line[i].x;
			lane[0].line[i].y = pl_input->fu_pl_data.lines.l_edges[1].line[i].y;
		}
		lane[0].valid_num_points = pl_input->fu_pl_data.lines.l_edges[1].valid_num_points;

		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;

		step = (int)(lane[0].line[lane[0].valid_num_points - 1].y - lane[0].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(lane[0].line, lane[0].valid_num_points, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(lane[0].line, temp_line, temp_line_num * sizeof(COOR2));
		lane[0].valid_num_points = temp_line_num;
	}

	//[*****************************************************************************************]
	lane[1].dir_type_hor = pl_input->fu_pl_data.lines.l_edges[0].dir_type_hor;
	lane[1].fidelity = pl_input->fu_pl_data.lines.l_edges[0].fidelity;
	lane[1].line_color = pl_input->fu_pl_data.lines.l_edges[0].line_color;
	lane[1].line_type = pl_input->fu_pl_data.lines.l_edges[0].line_type;
	for (i = 0; i < pl_input->fu_pl_data.lines.l_edges[0].valid_num_points; i++)
	{
		lane[1].line[i].x = pl_input->fu_pl_data.lines.l_edges[0].line[i].x;
		lane[1].line[i].y = pl_input->fu_pl_data.lines.l_edges[0].line[i].y;
	}
	lane[1].valid_num_points = pl_input->fu_pl_data.lines.l_edges[0].valid_num_points;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;

	step = (int)(lane[1].line[lane[1].valid_num_points - 1].y - lane[1].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

	line_fitting(lane[1].line, lane[1].valid_num_points, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memcpy(lane[1].line, temp_line, temp_line_num * sizeof(COOR2));
	lane[1].valid_num_points = temp_line_num;

	//[*****************************************************************************************]
	if (pl_input->fu_pl_data.lines.valid_num_in_r_wing >= 2)
	{
		lane[3].dir_type_hor = pl_input->fu_pl_data.lines.r_edges[1].dir_type_hor;
		lane[3].fidelity = pl_input->fu_pl_data.lines.r_edges[1].fidelity;
		lane[3].line_color = pl_input->fu_pl_data.lines.r_edges[1].line_color;
		lane[3].line_type = pl_input->fu_pl_data.lines.r_edges[1].line_type;
		for (i = 0; i < pl_input->fu_pl_data.lines.r_edges[1].valid_num_points; i++)
		{
			lane[3].line[i].x = pl_input->fu_pl_data.lines.r_edges[1].line[i].x;
			lane[3].line[i].y = pl_input->fu_pl_data.lines.r_edges[1].line[i].y;
		}
		lane[3].valid_num_points = pl_input->fu_pl_data.lines.r_edges[1].valid_num_points;

		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;

		step = (int)(lane[3].line[lane[3].valid_num_points - 1].y - lane[3].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(lane[3].line, lane[3].valid_num_points, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(lane[3].line, temp_line, temp_line_num * sizeof(COOR2));
		lane[3].valid_num_points = temp_line_num;
	}

	//[*****************************************************************************************]
	lane[2].dir_type_hor = pl_input->fu_pl_data.lines.r_edges[0].dir_type_hor;
	lane[2].fidelity = pl_input->fu_pl_data.lines.r_edges[0].fidelity;
	lane[2].line_color = pl_input->fu_pl_data.lines.r_edges[0].line_color;
	lane[2].line_type = pl_input->fu_pl_data.lines.r_edges[0].line_type;
	for (i = 0; i < pl_input->fu_pl_data.lines.r_edges[0].valid_num_points; i++)
	{
		lane[2].line[i].x = pl_input->fu_pl_data.lines.r_edges[0].line[i].x;
		lane[2].line[i].y = pl_input->fu_pl_data.lines.r_edges[0].line[i].y;
	}
	lane[2].valid_num_points = pl_input->fu_pl_data.lines.r_edges[0].valid_num_points;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;

	step = (int)(lane[2].line[lane[2].valid_num_points - 1].y - lane[2].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

	line_fitting(lane[2].line, lane[2].valid_num_points, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memcpy(lane[2].line, temp_line, temp_line_num * sizeof(COOR2));
	lane[2].valid_num_points = temp_line_num;


	//[@@依据最近的一点进行匹配，匹配到的ok_times + 1 没有匹配到ok_times - 1]
	//[匹配的时候车不会变道]
	for (i = 0; i < LANE_LINE_NUM; i++)
	{
		if (lane[i].valid_num_points > 1)
		{//[此车道数目有效]

			if (g_multi_lane.ok_times[i] == 0)
			{
				//[以前未确认，直接赋值]
				g_multi_lane.lane_line[i] = lane[i];
				g_multi_lane.ok_times[i] = 1;
			}
			else
			{
				//[以前有确认，现在进行匹配]
				old_pt = coordinate_transform(gps_info, g_multi_lane.gps_info, g_multi_lane.lane_line[i].line[0]);
				cur_pt = lane[i].line[0];

				//[前后帧之间步长]
				//int dist = (int)(35 * CM / 10) + 50;
				//if (dist_point(&old_pt, &cur_pt) <= dist)//[一般来说距离产生于y值]
				if (abs(old_pt.x - cur_pt.x) < 80)// && abs(old_pt.y - cur_pt.y) < dist)
				{//[匹配上]
					g_multi_lane.ok_times[i]++;
					if (g_multi_lane.ok_times[i] > 2)
						g_multi_lane.ok_times[i] = 2;

					g_multi_lane.lane_line[i] = lane[i];
				}
				else
				{//[未匹配上]
					g_multi_lane.ok_times[i]--;
					if (g_multi_lane.ok_times[i] < 0)
						g_multi_lane.ok_times[i] = 0;

					if (g_multi_lane.ok_times[i] == 0)
					{//[删除道路边的点信息，其他不变]
						memset(&g_multi_lane.lane_line[i].line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
					}

					//[把历史的点转化到当前坐标系下，截取保存]
					for (j=0; j<g_multi_lane.lane_line[i].valid_num_points; j++)
					{
						cur_pt = coordinate_transform(gps_info, g_multi_lane.gps_info, g_multi_lane.lane_line[i].line[j]);
						g_multi_lane.lane_line[i].line[j] = cur_pt;
					}

					//[修正历史的点，从y=400给起]
					for (j=0; j<g_multi_lane.lane_line[i].valid_num_points; j++)
					{
						if (g_multi_lane.lane_line[i].line[j].y >= 400)
							break;
					}

					if (j>0)
					{
						COOR2 pt1;
						COOR2 pt2;
						COOR2 pt3;
						int pt_num = 0;
						pt1 = g_multi_lane.lane_line[i].line[j - 1];
						pt2 = g_multi_lane.lane_line[i].line[j];

						if (pt2.y == 400)
						{
							memmove(g_multi_lane.lane_line[i].line, 
								g_multi_lane.lane_line[i].line + j, 
								sizeof(COOR2) * (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - j));

							pt_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - j;
						}
						else
						{
							if (pt1.x == pt2.x)
							{
								pt3.x = pt1.x;
								pt3.y = 400;
							}
							else
							{
								pt3.y = 400;
								pt3.x = (INT32)(pt1.x - (pt1.y - 400.0) * (pt1.x - pt2.x) / (pt1.y - pt2.y));
							}

							g_multi_lane.lane_line[i].line[0] = pt3;
							memmove(g_multi_lane.lane_line[i].line + 1, 
								g_multi_lane.lane_line[i].line + j, 
								sizeof(COOR2) * (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - j));

							pt_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - j + 1;
						}

						memset(temp_line, 0, 200 * sizeof(COOR2));
						temp_line_num = 0;

						step = (int)(g_multi_lane.lane_line[i].line[pt_num - 1].y - g_multi_lane.lane_line[i].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						line_fitting(g_multi_lane.lane_line[i].line, pt_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
						memcpy(g_multi_lane.lane_line[i].line, temp_line, temp_line_num * sizeof(COOR2));
						g_multi_lane.lane_line[i].valid_num_points = temp_line_num;
					}//[if (j>0)]
				}//[else]
			}//[else]
		}//[end if (lane[i].valid_num_points > 1)]
		else
		{//[此车道数目无效]

			g_multi_lane.ok_times[i]--;
			if (g_multi_lane.ok_times[i] < 0)
				g_multi_lane.ok_times[i] = 0;

			if (g_multi_lane.ok_times[i] == 0)
			{
				memset(&g_multi_lane.lane_line[i].line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			}

			if (g_multi_lane.lane_line[i].valid_num_points > 1)
			{//[此车道历史数目有效]

				//[把历史的点转化到当前坐标系下，截取保存]
				for (j=0; j<g_multi_lane.lane_line[i].valid_num_points; j++)
				{
					cur_pt = coordinate_transform(gps_info, g_multi_lane.gps_info, g_multi_lane.lane_line[i].line[j]);
					g_multi_lane.lane_line[i].line[j] = cur_pt;
				}

				//[修正历史的点，从y=400给起]
				for (j=0; j<g_multi_lane.lane_line[i].valid_num_points; j++)
				{
					if (g_multi_lane.lane_line[i].line[j].y >= 400)
						break;
				}

				if (j>0)
				{
					COOR2 pt1;
					COOR2 pt2;
					COOR2 pt3;
					int pt_num = 0;
					pt1 = g_multi_lane.lane_line[i].line[j - 1];
					pt2 = g_multi_lane.lane_line[i].line[j];

					if (pt2.y == 400)
					{
						memmove(g_multi_lane.lane_line[i].line, 
							g_multi_lane.lane_line[i].line + j, 
							sizeof(COOR2) * (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - j));

						pt_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - j;
					}
					else
					{
						if (pt1.x == pt2.x)
						{
							pt3.x = pt1.x;
							pt3.y = 400;
						}
						else
						{
							pt3.y = 400;
							pt3.x = (INT32)(pt1.x - (pt1.y - 400.0) * (pt1.x - pt2.x) / (pt1.y - pt2.y));
						}

						g_multi_lane.lane_line[i].line[0] = pt3;
						memmove(g_multi_lane.lane_line[i].line + 1, 
							g_multi_lane.lane_line[i].line + j, 
							sizeof(COOR2) * (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - j));

						pt_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - j + 1;
					}

					memset(temp_line, 0, 200 * sizeof(COOR2));
					temp_line_num = 0;

					step = (int)(g_multi_lane.lane_line[i].line[pt_num - 1].y - g_multi_lane.lane_line[i].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

					line_fitting(g_multi_lane.lane_line[i].line, pt_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
					memcpy(g_multi_lane.lane_line[i].line, temp_line, temp_line_num * sizeof(COOR2));
					g_multi_lane.lane_line[i].valid_num_points = temp_line_num;
				}//[if (j>0)]
			}//[if (g_multi_lane.lane_line[i].valid_num_points > 1)]
		}//[else]
	}//[end for i]

	if (g_multi_lane.lane_line[1].valid_num_points == 0 && \
		g_multi_lane.lane_line[2].valid_num_points > 0  && \
		g_multi_lane.lane_line[2].fidelity  >= 70)
	{
		for (i=0;i<g_multi_lane.lane_line[2].valid_num_points; i++)
		{
			g_multi_lane.lane_line[1].line[i].x = g_multi_lane.lane_line[2].line[i].x - 375;
			g_multi_lane.lane_line[1].line[i].y = g_multi_lane.lane_line[2].line[i].y;
		}
		g_multi_lane.lane_line[1].fidelity = 70;
		g_multi_lane.lane_line[1].valid_num_points = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
		g_multi_lane.ok_times[1] = 1;
	}
	else if (g_multi_lane.lane_line[2].valid_num_points == 0 && \
		g_multi_lane.lane_line[1].valid_num_points > 0 \
		&& g_multi_lane.lane_line[1].fidelity  >= 70)
	{
		for (i=0;i<g_multi_lane.lane_line[1].valid_num_points; i++)
		{
			g_multi_lane.lane_line[2].line[i].x = g_multi_lane.lane_line[1].line[i].x + 375;
			g_multi_lane.lane_line[2].line[i].y = g_multi_lane.lane_line[1].line[i].y;
		}
		g_multi_lane.lane_line[2].fidelity = 70;
		g_multi_lane.lane_line[2].valid_num_points = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
		g_multi_lane.ok_times[2] = 1;
	}

	memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));
}

//***********************************************************************************************
//                                zgccmax 2012.May.28
//void keep_multi_lane_in_change_lane(PL_FUNC_INPUT *pl_input, int mode)
//param:    PL_FUNC_INPUT *pl_input			融合输入
//			int mode						0  换左道  1  换右道
//return:   void
//discribe: 匹配并给出预瞄的车道，融合输入的当前车道 l 和 r 与历史的车道去匹配。
//			g_change_lane_see = -1    还在初始车道，预瞄左车道
//			g_change_lane_see = 0     已经在目标车道，预瞄当前目标车道
//			g_change_lane_see = 1     还在初始车道，预瞄右车道
//***********************************************************************************************
void keep_multi_lane_in_change_lane(PL_FUNC_INPUT *pl_input, int mode)
{
	int i;
	COOR2 l_old_pt;
	COOR2 l_cur_pt;
	COOR2 r_old_pt;
	COOR2 r_cur_pt;

	COOR2 temp_line[200];
	int temp_line_num = 0;
	PL_EDGE lane[LANE_LINE_NUM];
	POSE_INFO gps_info;
	int step;

	memset(lane, 0, LANE_LINE_NUM * sizeof(PL_EDGE));

	memcpy(&gps_info, &pl_input->state.pos, sizeof(POSE_INFO));

	if (pl_input->fu_pl_data.lines.valid_num_in_l_wing > 1)
	{
		lane[0].dir_type_hor = pl_input->fu_pl_data.lines.l_edges[1].dir_type_hor;
		lane[0].fidelity = pl_input->fu_pl_data.lines.l_edges[1].fidelity;
		lane[0].line_color = pl_input->fu_pl_data.lines.l_edges[1].line_color;
		lane[0].line_type = pl_input->fu_pl_data.lines.l_edges[1].line_type;
		for (i = 0; i < pl_input->fu_pl_data.lines.l_edges[1].valid_num_points; i++)
		{
			lane[0].line[i].x = pl_input->fu_pl_data.lines.l_edges[1].line[i].x;
			lane[0].line[i].y = pl_input->fu_pl_data.lines.l_edges[1].line[i].y;
		}
		lane[0].valid_num_points = pl_input->fu_pl_data.lines.l_edges[1].valid_num_points;
		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;

		step = (int)(lane[0].line[lane[0].valid_num_points - 1].y - lane[0].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(lane[0].line, lane[0].valid_num_points, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(lane[0].line, temp_line, temp_line_num * sizeof(COOR2));
		lane[0].valid_num_points = temp_line_num;
	}

	//[只使用1、2作为输入，即使用融合的当前车道去匹配历史]
	if (pl_input->fu_pl_data.lines.l_edges[0].valid_num_points >= 2)
	{
		lane[1].dir_type_hor = pl_input->fu_pl_data.lines.l_edges[0].dir_type_hor;
		lane[1].fidelity = pl_input->fu_pl_data.lines.l_edges[0].fidelity;
		lane[1].line_color = pl_input->fu_pl_data.lines.l_edges[0].line_color;
		lane[1].line_type = pl_input->fu_pl_data.lines.l_edges[0].line_type;
		for (i = 0; i < pl_input->fu_pl_data.lines.l_edges[0].valid_num_points; i++)
		{
			lane[1].line[i].x = pl_input->fu_pl_data.lines.l_edges[0].line[i].x;
			lane[1].line[i].y = pl_input->fu_pl_data.lines.l_edges[0].line[i].y;
		}
		lane[1].valid_num_points = pl_input->fu_pl_data.lines.l_edges[0].valid_num_points;
		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;

		step = (int)(lane[1].line[lane[1].valid_num_points - 1].y - lane[1].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(lane[1].line, lane[1].valid_num_points, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(lane[1].line, temp_line, temp_line_num * sizeof(COOR2));
		lane[1].valid_num_points = temp_line_num;
	}

	if (pl_input->fu_pl_data.lines.r_edges[0].valid_num_points >= 2)
	{
		lane[2].dir_type_hor = pl_input->fu_pl_data.lines.r_edges[0].dir_type_hor;
		lane[2].fidelity = pl_input->fu_pl_data.lines.r_edges[0].fidelity;
		lane[2].line_color = pl_input->fu_pl_data.lines.r_edges[0].line_color;
		lane[2].line_type = pl_input->fu_pl_data.lines.r_edges[0].line_type;
		for (i = 0; i < pl_input->fu_pl_data.lines.r_edges[0].valid_num_points; i++)
		{
			lane[2].line[i].x = pl_input->fu_pl_data.lines.r_edges[0].line[i].x;
			lane[2].line[i].y = pl_input->fu_pl_data.lines.r_edges[0].line[i].y;
		}
		lane[2].valid_num_points = pl_input->fu_pl_data.lines.r_edges[0].valid_num_points;
		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;

		step = (int)(lane[2].line[lane[2].valid_num_points - 1].y - lane[2].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(lane[2].line, lane[2].valid_num_points, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(lane[2].line, temp_line, temp_line_num * sizeof(COOR2));
		lane[2].valid_num_points = temp_line_num;
	}

	if (pl_input->fu_pl_data.lines.valid_num_in_r_wing > 1)
	{
		lane[3].dir_type_hor = pl_input->fu_pl_data.lines.r_edges[1].dir_type_hor;
		lane[3].fidelity = pl_input->fu_pl_data.lines.r_edges[1].fidelity;
		lane[3].line_color = pl_input->fu_pl_data.lines.r_edges[1].line_color;
		lane[3].line_type = pl_input->fu_pl_data.lines.r_edges[1].line_type;
		for (i = 0; i < pl_input->fu_pl_data.lines.r_edges[1].valid_num_points; i++)
		{
			lane[3].line[i].x = pl_input->fu_pl_data.lines.r_edges[1].line[i].x;
			lane[3].line[i].y = pl_input->fu_pl_data.lines.r_edges[1].line[i].y;
		}
		lane[3].valid_num_points = pl_input->fu_pl_data.lines.r_edges[1].valid_num_points;
		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;

		step = (int)(lane[3].line[lane[3].valid_num_points - 1].y - lane[3].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(lane[3].line, lane[3].valid_num_points, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(lane[3].line, temp_line, temp_line_num * sizeof(COOR2));
		lane[3].valid_num_points = temp_line_num;
	}

	if (lane[1].valid_num_points < 2 || lane[2].valid_num_points < 2)
	{
		pl_input->fu_pl_data.lines.l_edges[0].valid_num_points = 0;
		memset(&g_multi_lane, 0, sizeof(MULTI_LANE));
		memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));
		return;
	}

	//[匹配点使用最近的点]
	l_old_pt = coordinate_transform(gps_info, g_multi_lane.gps_info, g_multi_lane.lane_line[1].line[0]);
	l_cur_pt = lane[1].line[0];

	r_old_pt = coordinate_transform(gps_info, g_multi_lane.gps_info, g_multi_lane.lane_line[2].line[0]);
	r_cur_pt = lane[2].line[0];

	//[匹配参数为估计的经验值，可以根据实验情况更改]
	if (0 == mode)
	{//[左换道]

		if (abs(l_old_pt.x - l_cur_pt.x) < 100 || abs(r_old_pt.x - r_cur_pt.x) < 100)
		{//[左右点的x值误差在1m内，匹配上了，还在初始车道]

			//[更新历史数据]
			memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

			g_change_lane_see = -1;
#ifdef MBUG_OPEN_
			MBUG("Changing to left......still in the initial lane\n");
			MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
			MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
			MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
			MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
		}
		else
		{//[没有匹配上]

			if (l_old_pt.x > 50 && abs(l_old_pt.x - r_cur_pt.x) < 100)
			{//[原先的左侧边已经到了右边，且与现在右侧边x值误差1m内，]
				//[说明初始车道已经在车右侧，正在进入左侧道路]

				//[更新历史数据]
				memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
				//[注意这里的确认值，只有1、2填了，根据情况修改]
				g_multi_lane.ok_times[0] = 0;
				g_multi_lane.ok_times[1] = 2;
				g_multi_lane.ok_times[2] = 2;
				g_multi_lane.ok_times[3] = 0;
				memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

#ifdef MBUG_OPEN_
				MBUG("Changing to left......near the middle of the left lane\n");
				MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
				MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
				MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
				MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
				return;
			}

			if (l_old_pt.x - l_cur_pt.x > 200)
			{//[原来的左侧边比现在的左侧边的x值大了2m，则说明当前车道已经认为是左车道]

				//[更新历史数据]
				memcpy(&g_multi_lane.lane_line[0], &lane[1], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[1], &lane[2], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

				g_change_lane_see = 0;
				//[换道确认计数器++]
				g_change_lane_count++;
#ifdef MBUG_OPEN_
				MBUG("g_change_lane_count : %d\n", g_change_lane_count);
#endif
				if (g_change_lane_count >= 3)
				{//[3帧确认换道完成，根据实验情况修改参数]
					//[更新历史数据，恢复正常的道路跟踪]
					memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
					memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
					memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
					memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
					g_multi_lane.ok_times[0] = 0;
					g_multi_lane.ok_times[1] = 2;
					g_multi_lane.ok_times[2] = 2;
					g_multi_lane.ok_times[3] = 0;
					memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

#ifdef MBUG_OPEN_
					MBUG("Changing to left......finish the change\n");
					MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
					MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
					MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
					MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
					return;
				}
#ifdef MBUG_OPEN_
				MBUG("Changing to left......drive into the left lane\n");
				MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
				MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
				MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
				MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
				return;
			}//[if (l_old_pt.x - l_cur_pt.x > 200)]

			//[不符合匹配条件，默认行驶于初始车道，且不更新历史数据]
			memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
			for (i=0; i<LANE_LINE_NUM; i++)
			{
				if (g_multi_lane.lane_line[i].valid_num_points >= 2)
				{
					g_multi_lane.ok_times[i] = 1;
				}
				else
				{
					g_multi_lane.ok_times[i] = 0;
				}
			}
			memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

			g_change_lane_see = -1;
#ifdef MBUG_OPEN_
			MBUG("Changing to left......default strategy\n");
			MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
			MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
			MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
			MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
		}//[else]
	}//[if (0 == mode)]
	else
	{//[右换道]

		if (abs(l_old_pt.x - l_cur_pt.x) < 100 || abs(r_old_pt.x - r_cur_pt.x) < 100)
		{//[左右点的x值误差在1m内，匹配上了，还在初始车道]

			//[更新历史数据]
			memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

			g_change_lane_see = 1;
#ifdef MBUG_OPEN_
			MBUG("Changing to right......still in the initial lane\n");
			MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
			MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
			MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
			MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
		}
		else
		{//[没有匹配上]

			if (r_old_pt.x < -50 && abs(r_old_pt.x - l_cur_pt.x) < 100)
			{//[原先的右侧边已经到了左边，且与现在左侧边x值误差1m内，]
				//[说明初始车道已经在车左侧，正在进入右侧道路]

				//[更新历史数据]
				memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
				//[注意这里的确认值，只有1、2填了，根据情况修改]
				g_multi_lane.ok_times[0] = 0;
				g_multi_lane.ok_times[1] = 2;
				g_multi_lane.ok_times[2] = 2;
				g_multi_lane.ok_times[3] = 0;
				memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

#ifdef MBUG_OPEN_
				MBUG("Changing to right......near the middle of the right lane\n");
				MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
				MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
				MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
				MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
				return;
			}

			if (r_cur_pt.x - r_old_pt.x > 200)
			{//[现在的右侧边比原来的右侧边的x值大了2m，则说明当前车道已经认为是右车道]

				//[更新历史数据]
				memcpy(&g_multi_lane.lane_line[2], &lane[1], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[3], &lane[2], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

				g_change_lane_see = 0;
				//[换道确认计数器++]
				g_change_lane_count++;
#ifdef MBUG_OPEN_
				MBUG("g_change_lane_count : %d\n", g_change_lane_count);
#endif
				if (g_change_lane_count >= 3)
				{//[3帧确认换道完成，根据实验情况修改参数]
					//[更新历史数据，恢复正常的道路跟踪]
					memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
					memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
					memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
					memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
					g_multi_lane.ok_times[0] = 0;
					g_multi_lane.ok_times[1] = 2;
					g_multi_lane.ok_times[2] = 2;
					g_multi_lane.ok_times[3] = 0;
					memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

#ifdef MBUG_OPEN_
					MBUG("Changing to right......finish the change\n");
					MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
					MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
					MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
					MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
					return;
				}
#ifdef MBUG_OPEN_
				MBUG("Changing to right......drive into the right lane\n");
				MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
				MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
				MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
				MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
				return;
			}//[if (r_cur_pt.x - r_old_pt.x > 200)]

			//[不符合匹配条件，默认行驶于初始车道，且不更新历史数据]
			memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
			for (i=0; i<LANE_LINE_NUM; i++)
			{
				if (g_multi_lane.lane_line[i].valid_num_points >= 2)
				{
					g_multi_lane.ok_times[i] = 1;
				}
				else
				{
					g_multi_lane.ok_times[i] = 0;
				}
			}
			memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));
			g_change_lane_see = 1;
#ifdef MBUG_OPEN_
			MBUG("Changing to right......default strategy\n");
			MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
			MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
			MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
			MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
		}//[else]
	}//[else]
}

/*==================================================================
 * 函数名  ：	void keep_multi_lane_while_finish(PL_EDGE lane[LANE_LINE_NUM], POSE_INFO gps_info)
 * 功能    ：	在越过车道线驶入目标车道过程中对车道线进行匹配
 * 输入参数：	PL_EDGE lane[LANE_LINE_NUM]			当前融合输入车道
				POSE_INFO gps_info					当前GPS信息
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void keep_multi_lane_while_finish(PL_EDGE lane[LANE_LINE_NUM], POSE_INFO gps_info)
{
	int i, j;
	COOR2 old_pt;
	COOR2 cur_pt;

	int flag = 0;//[是否匹配上的标志]

	if (g_change_lane_count == 0)
	{//[已经进入目标车道]
		memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
		memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
		memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
		memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
		for (i=0; i<LANE_LINE_NUM; i++)
		{
			if (g_multi_lane.lane_line[i].valid_num_points >= 2)
			{
				g_multi_lane.ok_times[i] = 1;
			}
			else
			{
				g_multi_lane.ok_times[i] = 0;
			}
		}
		memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));
	}
	else
	{//[匹配车道，对匹配上的车道进行更新]
		for (i=0; i<LANE_LINE_NUM; i++)
		{
			if (g_multi_lane.lane_line[i].valid_num_points > 0)
			{
				flag = 0;
				for (j=0; j<LANE_LINE_NUM; j++)
				{
					if (lane[j].valid_num_points > 0)
					{
						old_pt = coordinate_transform(gps_info, g_multi_lane.gps_info, g_multi_lane.lane_line[i].line[0]);
						cur_pt = lane[j].line[0];

						if (abs(old_pt.x - cur_pt.x) < 100)
						{
							memcpy(&g_multi_lane.lane_line[i], &lane[j], sizeof(PL_EDGE));
							g_multi_lane.ok_times[i] = 2;
							memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));
							flag = 1;
						}
					}
				}
				if (flag == 0)
				{
					g_multi_lane.ok_times[i]--;
					if (g_multi_lane.ok_times[i] <= 0)
					{
						g_multi_lane.ok_times[i] = 0;
						memset(&g_multi_lane.lane_line[i], 0, sizeof(PL_EDGE));
					}
				}
			}
		}
	}
	
}

/*==================================================================
 * 函数名  ：	void keep_multi_lane_in_take_over(PL_FUNC_INPUT *pl_input, int mode)
 * 功能    ：	匹配并给出预瞄的车道，融合输入的当前车道 l 和 r 与历史的车道去匹配。
				g_change_lane_see = -1    还在初始车道，预瞄左车道
				g_change_lane_see = 0     已经在目标车道，预瞄当前目标车道
				g_change_lane_see = 1     还在初始车道，预瞄右车道
 * 输入参数：	PL_FUNC_INPUT *pl_input	  环境数据输入
				int mode				  0  左超车  1  右超车
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void keep_multi_lane_in_take_over(PL_FUNC_INPUT *pl_input, int mode)
{
	int i;
	COOR2 l_old_pt;
	COOR2 l_cur_pt;
	COOR2 r_old_pt;
	COOR2 r_cur_pt;

	COOR2 temp_line[200];
	int temp_line_num = 0;
	PL_EDGE lane[LANE_LINE_NUM];
	POSE_INFO gps_info;
	int step;

	memset(lane, 0, LANE_LINE_NUM * sizeof(PL_EDGE));

	memcpy(&gps_info, &pl_input->state.pos, sizeof(POSE_INFO));

	if (pl_input->fu_pl_data.lines.valid_num_in_l_wing > 1)
	{
		lane[0].dir_type_hor = pl_input->fu_pl_data.lines.l_edges[1].dir_type_hor;
		lane[0].fidelity = pl_input->fu_pl_data.lines.l_edges[1].fidelity;
		lane[0].line_color = pl_input->fu_pl_data.lines.l_edges[1].line_color;
		lane[0].line_type = pl_input->fu_pl_data.lines.l_edges[1].line_type;
		for (i = 0; i < pl_input->fu_pl_data.lines.l_edges[1].valid_num_points; i++)
		{
			lane[0].line[i].x = pl_input->fu_pl_data.lines.l_edges[1].line[i].x;
			lane[0].line[i].y = pl_input->fu_pl_data.lines.l_edges[1].line[i].y;
		}
		lane[0].valid_num_points = pl_input->fu_pl_data.lines.l_edges[1].valid_num_points;
		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;

		step = (int)(lane[0].line[lane[0].valid_num_points - 1].y - lane[0].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(lane[0].line, lane[0].valid_num_points, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(lane[0].line, temp_line, temp_line_num * sizeof(COOR2));
		lane[0].valid_num_points = temp_line_num;
	}

	//[只使用1、2作为输入，即使用融合的当前车道去匹配历史]
	if (pl_input->fu_pl_data.lines.l_edges[0].valid_num_points > 0)
	{
		lane[1].dir_type_hor = pl_input->fu_pl_data.lines.l_edges[0].dir_type_hor;
		lane[1].fidelity = pl_input->fu_pl_data.lines.l_edges[0].fidelity;
		lane[1].line_color = pl_input->fu_pl_data.lines.l_edges[0].line_color;
		lane[1].line_type = pl_input->fu_pl_data.lines.l_edges[0].line_type;
		for (i = 0; i < pl_input->fu_pl_data.lines.l_edges[0].valid_num_points; i++)
		{
			lane[1].line[i].x = pl_input->fu_pl_data.lines.l_edges[0].line[i].x;
			lane[1].line[i].y = pl_input->fu_pl_data.lines.l_edges[0].line[i].y;
		}
		lane[1].valid_num_points = pl_input->fu_pl_data.lines.l_edges[0].valid_num_points;
		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;

		step = (int)(lane[1].line[lane[1].valid_num_points - 1].y - lane[1].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(lane[1].line, lane[1].valid_num_points, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(lane[1].line, temp_line, temp_line_num * sizeof(COOR2));
		lane[1].valid_num_points = temp_line_num;
	}
	
	if (pl_input->fu_pl_data.lines.r_edges[0].valid_num_points > 0)
	{
		lane[2].dir_type_hor = pl_input->fu_pl_data.lines.r_edges[0].dir_type_hor;
		lane[2].fidelity = pl_input->fu_pl_data.lines.r_edges[0].fidelity;
		lane[2].line_color = pl_input->fu_pl_data.lines.r_edges[0].line_color;
		lane[2].line_type = pl_input->fu_pl_data.lines.r_edges[0].line_type;
		for (i = 0; i < pl_input->fu_pl_data.lines.r_edges[0].valid_num_points; i++)
		{
			lane[2].line[i].x = pl_input->fu_pl_data.lines.r_edges[0].line[i].x;
			lane[2].line[i].y = pl_input->fu_pl_data.lines.r_edges[0].line[i].y;
		}
		lane[2].valid_num_points = pl_input->fu_pl_data.lines.r_edges[0].valid_num_points;
		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;

		step = (int)(lane[2].line[lane[2].valid_num_points - 1].y - lane[2].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(lane[2].line, lane[2].valid_num_points, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(lane[2].line, temp_line, temp_line_num * sizeof(COOR2));
		lane[2].valid_num_points = temp_line_num;
	}

	if (pl_input->fu_pl_data.lines.valid_num_in_r_wing > 1)
	{
		lane[3].dir_type_hor = pl_input->fu_pl_data.lines.r_edges[1].dir_type_hor;
		lane[3].fidelity = pl_input->fu_pl_data.lines.r_edges[1].fidelity;
		lane[3].line_color = pl_input->fu_pl_data.lines.r_edges[1].line_color;
		lane[3].line_type = pl_input->fu_pl_data.lines.r_edges[1].line_type;
		for (i = 0; i < pl_input->fu_pl_data.lines.r_edges[1].valid_num_points; i++)
		{
			lane[3].line[i].x = pl_input->fu_pl_data.lines.r_edges[1].line[i].x;
			lane[3].line[i].y = pl_input->fu_pl_data.lines.r_edges[1].line[i].y;
		}
		lane[3].valid_num_points = pl_input->fu_pl_data.lines.r_edges[1].valid_num_points;
		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;

		step = (int)(lane[3].line[lane[3].valid_num_points - 1].y - lane[3].line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(lane[3].line, lane[3].valid_num_points, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(lane[3].line, temp_line, temp_line_num * sizeof(COOR2));
		lane[3].valid_num_points = temp_line_num;
	}

	//[对车道边进行处理]
	if (lane[1].valid_num_points < 2 && lane[2].valid_num_points < 2)
	{
		memset(&g_multi_lane, 0, sizeof(MULTI_LANE));
		memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));
		return;
	}
	else if (lane[1].valid_num_points < 2)
	{
		for (i=0; i<lane[2].valid_num_points; i++)
		{
			lane[1].line[i].x = lane[2].line[i].x - 375;
			lane[1].line[i].y = lane[2].line[i].y;
		}
		lane[1].valid_num_points = lane[2].valid_num_points;
	}
	else if (lane[2].valid_num_points < 2)
	{
		for (i=0; i<lane[1].valid_num_points; i++)
		{
			lane[2].line[i].x = lane[1].line[i].x + 375;
			lane[2].line[i].y = lane[1].line[i].y;
		}
		lane[2].valid_num_points = lane[1].valid_num_points;
	}

	//[将历史车道转到当前坐标系下，进行匹配]
	l_old_pt = coordinate_transform(gps_info, g_multi_lane.gps_info, g_multi_lane.lane_line[1].line[0]);
	l_cur_pt = lane[1].line[0];

	r_old_pt = coordinate_transform(gps_info, g_multi_lane.gps_info, g_multi_lane.lane_line[2].line[0]);
	r_cur_pt = lane[2].line[0];

	//[匹配参数估计经验值]
	if (0 == mode)
	{//[左换道]

		if (g_change_lane_count > 0)
		{
			keep_multi_lane_while_finish(lane, gps_info);

			g_change_lane_see = 0;
			g_change_lane_count++;
#ifdef MBUG_OPEN_
			MBUG("g_change_lane_count : %d\n", g_change_lane_count);
#endif
			return;
		}

		if (abs(l_old_pt.x - l_cur_pt.x) < 100 && abs(r_old_pt.x - r_cur_pt.x) < 100 && g_change_lane_see == -1)
		{//[匹配上了，还在初始车道]

			//[更新历史数据]
			memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
			for (i=0; i<LANE_LINE_NUM; i++)
			{
				if (g_multi_lane.lane_line[i].valid_num_points >= 2)
				{
					g_multi_lane.ok_times[i] = 1;
				}
				else
				{
					g_multi_lane.ok_times[i] = 0;
				}
			}
			memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

			g_change_lane_see = -1;
#ifdef MBUG_OPEN_
			MBUG("Take over to left......still in the initial lane\n");
			MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
			MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
			MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
			MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
		}
		else
		{//[没有匹配上]

			//[@@可能会没有机会进入下面的判断***********]
			if (l_old_pt.x > 50 && abs(l_old_pt.x - r_cur_pt.x) < 100)
			{//[初始车道已经在车右侧，进入左侧道路]

				//[更新历史数据]
				//memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[0], &lane[1], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[1], &lane[2], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[2], &lane[3], sizeof(PL_EDGE));
				g_multi_lane.ok_times[0] = 2;
				g_multi_lane.ok_times[1] = 2;
				g_multi_lane.ok_times[2] = 2;
				g_multi_lane.ok_times[3] = 0;
				memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

				g_change_lane_see = 0;
				g_change_lane_count = 0;
				g_change_lane_count++;
#ifdef MBUG_OPEN_
				MBUG("Take over to left......near the middle of the left lane\n");
				MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
				MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
				MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
				MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
				return;
			}
			//[@@可能会没有机会进入上面的判断***********]

			if (l_old_pt.x - l_cur_pt.x > 200)
			{//[当前车道已经认为是左车道]

				//[更新历史数据]
				keep_multi_lane_while_finish(lane, gps_info);

				g_change_lane_see = 0;
				g_change_lane_count++;
#ifdef MBUG_OPEN_
				MBUG("g_change_lane_count : %d\n", g_change_lane_count);
				MBUG("Take over to to left......drive into the left lane\n");
				MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
				MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
				MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
				MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
				return;
			}

			//[默认行驶于初始车道，且不更新历史数据]
			memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
			for (i=0; i<LANE_LINE_NUM; i++)
			{
				if (g_multi_lane.lane_line[i].valid_num_points >= 2)
				{
					g_multi_lane.ok_times[i] = 1;
				}
				else
				{
					g_multi_lane.ok_times[i] = 0;
				}
			}
			g_change_lane_see = -1;
#ifdef MBUG_OPEN_
			MBUG("Take over to left......default strategy\n");
			MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
			MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
			MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
			MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
		}
	}
	else
	{//[右换道]

		if (g_change_lane_count > 0)
		{
			//[已经在右方车道，等待超车状态切换]
			keep_multi_lane_while_finish(lane, gps_info);

			g_change_lane_see = 0;
			g_change_lane_count++;
#ifdef MBUG_OPEN_
			MBUG("g_change_lane_count : %d\n", g_change_lane_count);
#endif
			return;
		}

		if (abs(l_old_pt.x - l_cur_pt.x) < 100 && abs(r_old_pt.x - r_cur_pt.x) < 100 && g_change_lane_see == 1)
		{//[匹配上了，还在初始车道]

			//[更新历史数据]
			memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
			for (i=0; i<LANE_LINE_NUM; i++)
			{
				if (g_multi_lane.lane_line[i].valid_num_points >= 2)
				{
					g_multi_lane.ok_times[i] = 1;
				}
				else
				{
					g_multi_lane.ok_times[i] = 0;
				}
			}
			memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

			g_change_lane_see = 1;
#ifdef MBUG_OPEN_
			MBUG("Take over to right......still in the initial lane\n");
			MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
			MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
			MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
			MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
		}
		else
		{//[没有匹配上]

			//[@@可能会没有机会进入下面的判断***********]
			if (r_old_pt.x < -50 && abs(r_old_pt.x - l_cur_pt.x) < 100)
			{//[初始车道已经在车左侧，进入右侧道路]

				//[更新历史数据]
				memcpy(&g_multi_lane.lane_line[1], &lane[0], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[2], &lane[1], sizeof(PL_EDGE));
				memcpy(&g_multi_lane.lane_line[3], &lane[2], sizeof(PL_EDGE));
				//memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
				g_multi_lane.ok_times[0] = 0;
				g_multi_lane.ok_times[1] = 2;
				g_multi_lane.ok_times[2] = 2;
				g_multi_lane.ok_times[3] = 2;
				memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

				g_change_lane_see = 0;
				g_change_lane_count = 0;
				g_change_lane_count++;
#ifdef MBUG_OPEN_
				MBUG("Take over to right......near the middle of the right lane\n");
				MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
				MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
				MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
				MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
				return;
			}
			//[@@可能会没有机会进入上面的判断***********]

			if (r_cur_pt.x - r_old_pt.x > 200)
			{//[当前车道已经认为是右车道]

				//[更新历史数据]
				keep_multi_lane_while_finish(lane, gps_info);

				g_change_lane_see = 0;
				g_change_lane_count++;
#ifdef MBUG_OPEN_
				MBUG("g_change_lane_count : %d\n", g_change_lane_count);
				MBUG("Take over to right......drive into the right lane\n");
				MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
				MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
				MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
				MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
				return;
			}

			//[默认行驶于初始车道，且不更新历史数据]
			memcpy(&g_multi_lane.lane_line[0], &lane[0], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[1], &lane[1], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[2], &lane[2], sizeof(PL_EDGE));
			memcpy(&g_multi_lane.lane_line[3], &lane[3], sizeof(PL_EDGE));
			for (i=0; i<LANE_LINE_NUM; i++)
			{
				if (g_multi_lane.lane_line[i].valid_num_points >= 2)
				{
					g_multi_lane.ok_times[i] = 1;
				}
				else
				{
					g_multi_lane.ok_times[i] = 0;
				}
			}
			memcpy(&g_multi_lane.gps_info, &gps_info, sizeof(POSE_INFO));

			g_change_lane_see = 1;
#ifdef MBUG_OPEN_
			MBUG("Take over to right......default strategy\n");
			MBUG("l_old_pt : (%d, %d)\n", l_old_pt.x, l_old_pt.y);
			MBUG("l_cur_pt : (%d, %d)\n", l_cur_pt.x, l_cur_pt.y);
			MBUG("r_old_pt : (%d, %d)\n", r_old_pt.x, r_old_pt.y);
			MBUG("r_cur_pt : (%d, %d)\n", r_cur_pt.x, r_cur_pt.y);
#endif
		}
	}
}

/*==================================================================
 * 函数名  ：	int get_change_lane_path(int mode)
 * 功能    ：	生成换道规划线
 * 输入参数：	int mode	0  左换道		1  右换道
 * 输出参数：	
 * 返回值  ：	int		0  可以安全行驶		-1  不可以安全行驶
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int get_change_lane_path(int mode)
{
	int ret = 0;

	int i;
	COOR2 temp_line[200];
	int temp_line_num = 0;
	COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];

	memset(temp_line, 0, 200 * sizeof(COOR2));
	memset(left_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	memset(right_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));

	if (mode == 0)
	{
		if (g_navi_state == TRACE_LANE)
		{//[刚恢复成道路跟踪的第一帧，下一帧转移至get_trace_lane_path函数]
			int num1 = g_multi_lane.lane_line[1].valid_num_points;
			int num2 = g_multi_lane.lane_line[2].valid_num_points;
			ret = check_the_lane(g_multi_lane.lane_line[1].line, num1, \
				g_multi_lane.lane_line[2].line, num2);
		}

		//[检测最危险点]
		for (i=0; i<g_right_line_num; i++)
		{
			if (g_multi_lane.lane_line[2].line[i].x - g_right_line[i].x > 150)
			{//[原始道边x减去障碍物归边之后的x，判断是否是障碍物堵塞道路]
				g_unsafe_dist_for_change_lane = g_right_line[i].y;
				break;
			}
		}

		if (g_unsafe_dist_for_change_lane < 800)
		{
			ret = -1;
			g_navi_state = S_EMERGENCY_STOP;
			g_take_over_back_lane_flag = -1;
			g_take_over_flag = 0;
			g_change_lane_count = 0;
			g_change_lane_see = 0;
		}

		if (ret == 0)
		{
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 0;
			g_mid_line[1].x = 0;
			g_mid_line[1].y = 400;

			fix_the_mid_line_6();
#ifdef MBUG_OPEN_
			MBUG("get_trace_lane_path : \n");
			for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
			}
			MBUG("\n");
#endif

			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			if (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y < 2000)
			{
				int step = (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y - g_mid_line[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
				line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			}
			else
			{
				line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, 100, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			}
			get_bezier_line(temp_line, temp_line_num, g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;

			fix_the_mid_line_7();

			g_fidelity = 1;
			ret = 0;
		}
		else
		{
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 0;
			g_mid_line[1].x = 0;
			g_mid_line[1].y = 400;
			ret = -1;
			g_navi_state = S_EMERGENCY_STOP;
			g_take_over_back_lane_flag = -1;
			g_take_over_flag = 0;
			g_change_lane_count = 0;
			g_change_lane_see = 0;
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			g_fidelity = 1;
		}
		
	}
	else
	{
		if (g_navi_state == TRACE_LANE)
		{//[刚恢复成道路跟踪的第一帧，下一帧转移至get_trace_lane_path函数]
			int num1 = g_multi_lane.lane_line[1].valid_num_points;
			int num2 = g_multi_lane.lane_line[2].valid_num_points;
			ret = check_the_lane(g_multi_lane.lane_line[1].line, num1, \
				g_multi_lane.lane_line[2].line, num2);
		}

		//[检测最危险点]
		for (i=0; i<g_left_line_num; i++)
		{//[原始道边x减去障碍物归边之后的x，判断是否是障碍物堵塞道路]
			if (g_left_line[i].x - g_multi_lane.lane_line[1].line[i].x > 150)
			{
				g_unsafe_dist_for_change_lane = g_left_line[i].y;
				break;
			}
		}

		if (g_unsafe_dist_for_change_lane < 800)
		{
			ret = -1;
			g_navi_state = S_EMERGENCY_STOP;
			g_take_over_back_lane_flag = -1;
			g_take_over_flag = 0;
			g_change_lane_count = 0;
			g_change_lane_see = 0;
		}

		if (ret == 0)
		{
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 0;
			g_mid_line[1].x = 0;
			g_mid_line[1].y = 400;
#ifdef MBUG_OPEN_
			MBUG("g_left_line : \n");
			for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", g_left_line[i].x, g_left_line[i].y);
			}
			MBUG("\n");
			MBUG("g_right_line : \n");
			for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", g_right_line[i].x, g_right_line[i].y);
			}
			MBUG("\n");
#endif
			fix_the_mid_line_6();
#ifdef MBUG_OPEN_
			MBUG("get_trace_lane_path : \n");
			for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
			}
			MBUG("\n");
#endif
			fix_the_mid_line();

			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			if (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y < 2000)
			{
				int step = (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y - g_mid_line[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
				line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			}
			else
			{
				line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line, temp_line_num, 100, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			}
			get_bezier_line(temp_line, temp_line_num, g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;

			fix_the_mid_line_7();

			g_fidelity = 1;
			ret = 0;
		}
		else
		{
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 0;
			g_mid_line[1].x = 0;
			g_mid_line[1].y = 400;
			ret = -1;
			g_navi_state = S_EMERGENCY_STOP;
			g_take_over_back_lane_flag = -1;
			g_take_over_flag = 0;
			g_change_lane_count = 0;
			g_change_lane_see = 0;
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			g_fidelity = 1;
		}
	}

	return ret;
}

//***********************************************************************************************
//                                zgccmax 2012.Aug.11
//int check_path_for_big_angle()
//param:	void
//return: 0 角度正常 -1 角度偏大  -2振荡
//discribe: 计算规划路径是否角度偏大
//***********************************************************************************************
int check_path_for_big_angle(double &rate)
{
	int ret = 0;
	int i = 0;

	double angle = 0;
	double max_angle = -1;

//	int index = -1;

	for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
	{
		if (g_mid_line[i].x == 0 || g_mid_line[i].y <= 400)
		{
			continue;
		}

		if (g_mid_line[i].y > 2000)
			break;

		angle = atan((g_mid_line[i].x + 0.0) / (g_mid_line[i].y - 400)) * 180 / PI;

		if (fabs(angle) > max_angle)
		{
			max_angle = fabs(angle);
//			index = i;
		}
	}

	if (max_angle >= 10)
	{
		g_vibration_count++;
		if (g_vibration_count > 10)
		{
			g_vibration_flag = 1;
			g_vibration_count = 10;
		}
	}
	else
	{
		g_vibration_count--;
		if (g_vibration_count < 0)
		{
			g_vibration_count = 0;
			g_vibration_flag = 0;
		}	
	}

	if (g_vibration_flag == 1)
	{
		ret = -2;
	}
	else
	{
		if (max_angle < 2)
		{
			rate = 1.0;
		}
		else if (max_angle >= 2 && max_angle <= 5)
		{//[速度控制在区间50%-100%，公式自行推导]
			rate = (5 - max_angle) / 6.0 + 0.5;

			if (max_angle >= 3.5)
			{
				ret = -1;
			}
		}
		else
		{
			rate = 0.5;

			ret = -1;
		}
	}

	return ret;
}

/*==================================================================
 * 函数名  ：	int get_trace_lane_path(int mode)
 * 功能    ：	道路跟踪生成规划线，对生成的曲线进行优化
 * 输入参数：	int mode	0  正常巡航		1  减速停车
 * 输出参数：	
 * 返回值  ：	可以执行 -1 不可执行
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int get_trace_lane_path(int mode)
{
	int ret = 0;

	int i, j;
	COOR2 temp_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];

	if (g_change_fade_timer == 0)
	{
		if (0 == mode)
		{
			//[以下的赋值是为了使bezier拟合的车头前方线平缓点，补丁而已……]
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 400;
			g_mid_line[1].x = 0;
			g_mid_line[1].y = 500;
#ifdef MBUG_OPEN_
			MBUG("get_trace_lane_path : \n");
			for (i=0; i<g_long_line_num; i++)
			{
				MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
			}
			MBUG("\n");
#endif
			memmove(g_mid_line + 1, g_mid_line, (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1) * sizeof(COOR2));
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 0;

			COOR2 temp_line2[200];
			int temp_line_num2 = 0;
			memset(temp_line2, 0, sizeof(COOR2) * 200);

int step = (int)((g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y - g_mid_line[0].y + 0.0) / 19);
			line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line2, temp_line_num2, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			memcpy(g_mid_line, temp_line2, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);

			fix_the_mid_line_2();

			if (g_line_avg_num != 10)
			{
				memcpy(g_line_avg[g_line_avg_index], g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));

				g_line_avg_last_index = g_line_avg_index;
				g_line_avg_index++;
				if (g_line_avg_index == LINE_AVG_NUM)
				{
					g_line_avg_index = 0;
				}
				g_line_avg_num++;
			}
			else
			{
				memcpy(g_line_avg[g_line_avg_index], g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));

				g_line_avg_last_index = g_line_avg_index;
				g_line_avg_index++;
				if (g_line_avg_index == LINE_AVG_NUM)
				{
					g_line_avg_index = 0;
				}
			}

			double rate = 0;
			ret = check_path_for_big_angle(rate);
			if (ret == -1)
			{
				int xx;
				int yy;

				for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
				{
					xx = 0;
					yy = 0;
					int weight = 0;
					for (j=0; j<g_line_avg_num; j++)
					{
						if (j == g_line_avg_last_index)
						{
							xx += g_line_avg[j][i].x * 60;
							yy += g_line_avg[j][i].y * 60;

							weight += 60;
						}
						else
						{
							xx += g_line_avg[j][i].x * 4;
							yy += g_line_avg[j][i].y * 4;

							weight += 4;
						}
					}
					g_mid_line[i].x = xx / weight;
					g_mid_line[i].y = yy / weight;
				}
			}

			fix_the_mid_line_2();
			//[车子压线处理]
			if (g_on_the_line_flag == 1 && g_navi_state != S_EMERGENCY_STOP && g_change_fade_timer == 0)
			{
				fix_the_mid_line_3();
				memmove(g_mid_line + 1, g_mid_line, (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1) * sizeof(COOR2));
				g_mid_line[0].x = 0;
				g_mid_line[0].y = 0;
			}
			else if (g_take_over_speed_up_timer == 1)
			{
				fix_the_mid_line_4();
				memmove(g_mid_line + 1, g_mid_line, (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1) * sizeof(COOR2));
				g_mid_line[0].x = 0;
				g_mid_line[0].y = 0;
			}

			get_bezier_line(g_mid_line, g_long_line_num, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			memcpy(g_mid_line, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));

			memset(temp_line2, 0, sizeof(COOR2) * 200);

			if (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y < 2000)
			{
				int step = (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y - g_mid_line[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
				line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line2, temp_line_num2, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			}
			else
			{
				line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line2, temp_line_num2, 100, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			}
			memcpy(g_mid_line, temp_line2, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);

			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
#ifdef MBUG_OPEN_
			MBUG("debug 555 : \n");
			for (i = 0; i < g_long_line_num; i++)
			{
				MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
			}
			MBUG("\n");
#endif
			g_fidelity = 1;
			ret = 0;
		}
		else
		{
			COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
			COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
			int left_line_num, right_line_num;

			memset(left_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			memset(right_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));

			left_line_num = g_multi_lane.lane_line[1].valid_num_points;
			memcpy(left_line, g_multi_lane.lane_line[1].line, left_line_num * sizeof(COOR2));

			right_line_num = g_multi_lane.lane_line[2].valid_num_points;
			memcpy(right_line, g_multi_lane.lane_line[2].line, right_line_num * sizeof(COOR2));

			ret = check_the_lane(left_line, left_line_num, right_line, right_line_num);

			g_mid_line[0].x = 0;
			g_mid_line[0].y = 400;
			g_mid_line[1].x = 0;
			g_mid_line[1].y = 500;
#ifdef MBUG_OPEN_
			MBUG("get_trace_lane_path : \n");
			for (i=0; i<g_long_line_num; i++)
			{
				MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
			}
			MBUG("\n");
#endif
			get_bezier_line(g_mid_line, g_long_line_num, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			memcpy(g_mid_line, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));

			memmove(g_mid_line + 1, g_mid_line, (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1) * sizeof(COOR2));
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 0;

			COOR2 temp_line2[200];
			int temp_line_num2 = 0;
			memset(temp_line2, 0, sizeof(COOR2) * 200);

			if (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y < 2000)
			{
				int step = (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y - g_mid_line[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
				line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line2, temp_line_num2, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
				memcpy(g_mid_line, temp_line2, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			}
			else
			{
				line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line2, temp_line_num2, 100, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
				memcpy(g_mid_line, temp_line2, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			}

			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			g_fidelity = 0;
		}
	}
	else
	{
		ret = get_change_lane_path(g_change_mode);
		
		int i;
		double x1, y1;
		double x2, y2;
		int num = 0;
		int points_num = 0;
		x1 = y1 = x2 = y2 = 0;
		double theta = 0;

		//[根据结构化道路以及自然道边的方向补出道路]
		for (i=0; i<LANE_LINE_NUM; i++)
		{
			if (g_multi_lane.lane_line[i].valid_num_points >= 2)
			{
				num++;
				points_num = g_multi_lane.lane_line[i].valid_num_points;
				x1 += g_multi_lane.lane_line[i].line[0].x;
				y1 += g_multi_lane.lane_line[i].line[0].y;

				x2 += g_multi_lane.lane_line[i].line[points_num - 1].x;
				y2 += g_multi_lane.lane_line[i].line[points_num - 1].y;
			}
		}

		if (num > 0)
		{//[有结构化道路]
			x1 = x1 / num;
			y1 = y1 / num;
			x2 = x2 / num;
			y2 = y2 / num;

			theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出道路角度]

			if (fabs(theta) < 4)
			{
				g_change_fade_timer--;
				if (g_change_fade_timer < 0)
				{
					g_change_fade_timer = 0;
				}
				g_change_mode = -1;
			}
		}
	}

	return ret;
}

/*==================================================================
 * 函数名  ：	int get_trace_lane_in_natural_path()
 * 功能    ：	生成自然道路跟踪规划线
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	int		0  可以执行  -1  不可以执行
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int get_trace_lane_in_natural_path()
{
	int ret = 0;

	int i, j;
	COOR2 temp_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	//[以下的赋值是为了使bezier拟合的车头前方线平缓点，补丁而已……]
	g_mid_line[0].x = 0;
	g_mid_line[0].y = 400;
	if (g_mid_line[1].y > 500)
	{
		g_mid_line[1].x = 0;
		g_mid_line[1].y = 500;
	}

#ifdef MBUG_OPEN_
	MBUG("get_trace_lane_in_natural_path : \n");
	for (i=0; i<g_long_line_num; i++)
	{
		MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
	}
	MBUG("\n");

	MBUG("g_left_line : \n");
	for (i=0; i<g_left_line_num; i++)
	{
		MBUG("(%d, %d) ", g_left_line[i].x, g_left_line[i].y);
	}
	MBUG("\n");
	MBUG("g_right_line : \n");
	for (i=0; i<g_right_line_num; i++)
	{
		MBUG("(%d, %d) ", g_right_line[i].x, g_right_line[i].y);
	}
	MBUG("\n");
#endif


	fix_the_mid_line();
#ifdef MBUG_OPEN_
	MBUG("fix_the_mid_line : \n");
	for (i=0; i<g_long_line_num; i++)
	{
		MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
	}
	MBUG("\n");
#endif

	get_bezier_line(g_mid_line, g_long_line_num, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memcpy(g_mid_line, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
#ifdef MBUG_OPEN_
	MBUG("get_bezier_line : \n");
	for (i=0; i<g_long_line_num; i++)
	{
		MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
	}
	MBUG("\n");
#endif

	memmove(g_mid_line + 1, g_mid_line, (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1) * sizeof(COOR2));
	g_mid_line[0].x = 0;
	g_mid_line[0].y = 0;

	COOR2 temp_line2[200];
	int temp_line_num2 = 0;
	int step = 0;
	memset(temp_line2, 0, sizeof(COOR2) * 200);

	if (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y < 2000)
	{
		step = (g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y - g_mid_line[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
		line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line2, temp_line_num2, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(g_mid_line, temp_line2, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	}
	else
	{
		line_fitting(g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE, temp_line2, temp_line_num2, 100, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(g_mid_line, temp_line2, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	}
#ifdef MBUG_OPEN_
	MBUG("line_fitting : \n");
	for (i=0; i<g_long_line_num; i++)
	{
		MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
	}
	MBUG("\n");
#endif
	fix_the_mid_line_2();
#ifdef MBUG_OPEN_
	MBUG("get_trace_lane_path : \n");
	for (i=0; i<g_long_line_num; i++)
	{
		MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
	}
	MBUG("\n");
#endif

	//[平滑]
	if (g_line_avg_num != 10)
	{
		memcpy(g_line_avg[g_line_avg_index], g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		g_line_avg_index++;
		if (g_line_avg_index == LINE_AVG_NUM)
		{
			g_line_avg_index = 0;
		}
		g_line_avg_num++;
	}
	else
	{
		memcpy(g_line_avg[g_line_avg_index], g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		g_line_avg_index++;
		if (g_line_avg_index == LINE_AVG_NUM)
		{
			g_line_avg_index = 0;
		}
	}

	double rate = 0;
	ret = check_path_for_big_angle(rate);
	if (ret == -1)
	{
		int xx;
		int yy;

		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			xx = 0;
			yy = 0;
			int weight = 0;
			for (j=0; j<g_line_avg_num; j++)
			{
				if (j == g_line_avg_last_index)
				{
					xx += g_line_avg[j][i].x * 90;
					yy += g_line_avg[j][i].y * 90;

					weight += 90;
				}
				else
				{
					xx += g_line_avg[j][i].x * 1;
					yy += g_line_avg[j][i].y * 1;

					weight += 1;
				}
			}
			g_mid_line[i].x = xx / weight;
			g_mid_line[i].y = yy / weight;
		}
	}

	fix_the_mid_line_2();
#ifdef MBUG_OPEN_
	MBUG("get_trace_lane_path : \n");
	for (i=0; i<g_long_line_num; i++)
	{
		MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
	}
	MBUG("\n");
#endif

	g_narrow_dist = MAX_VALUE;
	//[计算有效点数，用于控制底层向需要行驶方向进行预瞄，防止底层预瞄远处导致关键点不看]
	int narrow_dist = g_right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y;
	int width;
	int pt_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
	int x0, x1;
	for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
	{
		if (g_mid_line[i].y <= 400)
			continue;

		get_x_coord(g_mid_line[i].y, g_left_line, g_left_line_num, &x0);
		get_x_coord(g_mid_line[i].y, g_right_line, g_right_line_num, &x1);

		width = x1 - x0;

		if (width <= 450)
		{
			narrow_dist = g_mid_line[i].y;
			break;
		}
	}
	g_narrow_dist = narrow_dist;

	if (narrow_dist <= 1200)
	{
		pt_num = 0;
		for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			if (g_mid_line[i].y <= 800)
			{
				pt_num++;
				continue;
			}

			get_x_coord(g_mid_line[i].y, g_left_line, g_left_line_num, &x0);
			get_x_coord(g_mid_line[i].y, g_right_line, g_right_line_num, &x1);

			width = x1 - x0;

			if (g_mid_line[i].y == narrow_dist)
			{
				g_mid_line[i].x = (x1 - x0) / 2 + x0;
				pt_num++;
			}
			else if (g_mid_line[i].y < narrow_dist)
			{
				pt_num++;
			}
			else
				break;
		}

		get_bezier_line(g_mid_line, pt_num, temp_line, pt_num);
		memset(g_mid_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		memcpy(g_mid_line, temp_line, pt_num * sizeof(COOR2));
	}
	else
	{
		for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			if (g_mid_line[i].y > narrow_dist)
			{
				pt_num = i;
				if (pt_num > NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
					pt_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
				break;
			}
		}
	}

	ret = 0;
	g_long_line_num = pt_num;
	g_fidelity = 1;

	return ret;
}

/*==================================================================
 * 函数名  ：	void plan_init()
 * 功能    ：	读取配置文件，进行相关参数初始化
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void plan_init()
{
	FILE *fp = NULL;
	int i = 0;

	char name[256] = "";
	int val = 0;

	//[相关初始化]
	g_vehicle_org_pt.x = 0;
	g_vehicle_org_pt.y = 0;

	g_sharp_turn_flag = 0;
	memset(&g_sharp_turn_last_pt, 0, sizeof(COOR2));
	memset(&g_sharp_turn_slow_down_s_pt, 0, sizeof(COOR2));
	memset(&g_sharp_turn_slow_down_e_pt, 0, sizeof(COOR2));

	g_navi_state = TRACE_LANE;
	g_natural_road_search_dist = MAX_VALUE;

	g_cross_adjust_count = 0;
	g_cross_adjust_flag = 0;


	g_speed_limit_sign = 0;
	g_speed_limi_pl = 0;

	car_len = 420;
	car_wid = 230;

	g_64lidar_switch = 1;

	mini_pass = 0;
	g_max_speed = 0;
	g_real_speed = 0;
	g_vibration_count = 0;
	g_vibration_flag = 0;
	g_slow_down_dist = MAX_VALUE;
	g_unsafe_dist = MAX_VALUE;
	g_unsafe_dist_for_change_lane = MAX_VALUE;
	g_change_fade_timer = 0;
	g_change_mode = -1;
	g_change_steer_limit = 0;
	g_take_over_flag = 0;
	g_take_over_speed_up_timer = 0;
	g_take_over_back_lane_flag = -1;
	g_change_lane_unsafe_dist = MAX_VALUE;
	g_change_lane_direct_keep = -1;

	g_change_lane_search_dist = MAX_VALUE;
	g_change_lane_search_flag = 0;

	g_car_ahead_dist = MAX_VALUE;

	g_emergency_counter = 0;					//[陷入困境计时]
	g_emergency_roving_timer = 0;			//[摆脱困境计时]
	g_cur_search_index = -1;

	memset(g_line_avg, 0, LINE_AVG_NUM * sizeof(COOR2));
	g_line_avg_num = 0;
	g_line_avg_index = 0;
	g_line_avg_last_index = 0;

	g_cross_travel_dist = 2000;
	g_morphin_dist = 2000;

	morphin3();
	collision_arc();

	for (i=0; i<CROSS_AVG_NUM; i++)
	{
		g_cross_avg_angle[i] = 90.0;
	}
	g_cross_avg_angle_num = 0;
	g_cross_avg_index = 0;
	g_cross_last_index = MORPHIN_MID_INDEX;

	g_sharp_turn_in_dist = 3000;
	g_sharp_turn_out_dist = 10000;
	g_sharp_turn_speed = 15;//[15km/h]

	int reverse_speed;
	int reverse_plan_dist;
	int reverse_plan_pts_step;
	int reverse_step;
	int reverse_pt_num_limit;
	int reverse_back_stop_dist;
	int reverse_test_start_dist;
	int reverse_test_stop_dist;

#ifdef __GNUC__
	if ((fp = fopen("/home/alv/ugv/bin/config/localplan.ini", "r")) == NULL)
	{
#ifdef MBUG_OPEN_
		MBUG("localplan.ini can't open\n");
#endif
		return;
	}
#else
	if ((fp = fopen(".\\Planner\\zgcc\\localplan.ini", "r")) == NULL)
	{
		return;
	}
#endif

	while ((fscanf(fp, "%s = %d", name, &val)) != EOF)
	{
		if (strcmp(name, "car_identity") == 0)
		{
			g_car_identity = val;
		}
		if (strcmp(name, "car_len") == 0)
		{
			car_len = val;
		}
		if (strcmp(name, "car_wid") == 0)
		{
			car_wid = val;
		}
		if (strcmp(name, "max_speed") == 0)
		{
			g_max_speed = (int)(val * CM);
		}
		if (strcmp(name, "cross_und_slow_down_dist") == 0)
		{
			g_cross_und_slow_down_dist = val;
		}
		if (strcmp(name, "cross_und_slow_down_speed") == 0)
		{
			g_cross_und_slow_down_speed = val;
		}
		if (strcmp(name, "cross_und_speed") == 0)
		{
			g_cross_und_speed = val;
		}
		if (strcmp(name, "cross_und_straight_speed") == 0)
		{
			g_cross_und_straight_speed = val;
		}
		if (strcmp(name, "cross_und_left_speed") == 0)
		{
			g_cross_und_left_speed = val;
		}
		if (strcmp(name, "cross_und_right_speed") == 0)
		{
			g_cross_und_right_speed = val;
		}
		if (strcmp(name, "cross_und_uturn_speed") == 0)
		{
			g_cross_und_uturn_speed = val;
		}
		if (strcmp(name, "cross_und_traffic_sign_speed") == 0)
		{
			g_cross_und_traffic_sign_speed = val;
		}
		if (strcmp(name, "cross_straight_speed") == 0)
		{
			g_cross_straight_speed = val;
		}
		if (strcmp(name, "cross_left_turn_speed") == 0)
		{
			g_cross_left_turn_speed = val;
		}
		if (strcmp(name, "cross_right_turn_speed") == 0)
		{
			g_cross_right_turn_speed = val;
		}
		if (strcmp(name, "cross_uturn_speed") == 0)
		{
			g_cross_uturn_speed = val;
		}
		if (strcmp(name, "cross_quit_dist") == 0)
		{
			g_cross_quit_dist = val;
		}
		if (strcmp(name, "morphin_index") == 0)
		{
			g_morphin_index_test = val;
		}
		if (strcmp(name, "morphin_search_dist") == 0)
		{
			g_morphin_search_dist = val;
		}
		if (strcmp(name, "dyn_obs_switch") == 0)
		{
			g_dyn_obs_switch = val;
		}
		if (strcmp(name, "take_over_speed_up_time") == 0)
		{
			g_take_over_speed_up_time = val;
		}
		if (strcmp(name, "roving_switch") == 0)
		{
			g_roving_switch  = val;
		}
		if (strcmp(name, "roving_speed") == 0)
		{
			g_roving_speed = val;
		}
// 		if (strcmp(name, "roving_pilot_switch") == 0)
// 		{
// 			g_roving_pilot_switch = val;
// 		}
		if (strcmp(name, "nature_road_switch") == 0)
		{
			g_nature_road_switch = val;
		}
		if (strcmp(name, "nature_road_to_obs_switch") == 0)
		{
			g_nature_road_to_obs_switch = val;
		}
		if (strcmp(name, "64lidar_switch") == 0)
		{
			g_64lidar_switch = val;
		}
		if (strcmp(name, "global_task_pt_valid_dist") == 0)
		{
			g_global_task_pt_valid_dist = val;
		}
// 		if (strcmp(name, "global_task_pt_area_dist") == 0)
// 		{
// 			g_global_task_pt_area_dist = val;
// 		}
// 		if (strcmp(name, "global_task_adjust_dist") == 0)
// 		{
// 			g_global_task_adjust_dist = val;
// 		}
// 		if (strcmp(name, "global_task_adjust_threshold1") == 0)
// 		{
// 			g_global_task_adjust_threshold1 = val;
// 		}
// 		if (strcmp(name, "global_task_adjust_threshold2") == 0)
// 		{
// 			g_global_task_adjust_threshold2 = val;
// 		}
// 		if (strcmp(name, "global_task_adjust_threshold3") == 0)
// 		{
// 			g_global_task_adjust_threshold3 = val;
// 		}

		if (strcmp(name, "emergency_roving_timer") == 0)
		{
			emergency_roving_timer = val;
		}
		if (strcmp(name, "country_road") == 0)
		{
			//[0  城市道路  1  乡村道路]
			g_road_type = 0;
		}
		if (strcmp(name, "stop_dist") == 0)
		{
			g_stop_dist = val;
		}
		if (strcmp(name, "10km_dist") == 0)
		{
			g_10km_dist = val;
		}
		if (strcmp(name, "20km_dist") == 0)
		{
			g_20km_dist = val;
		}
		if (strcmp(name, "30km_dist") == 0)
		{
			g_30km_dist = val;
		}
		if (strcmp(name, "40km_dist") == 0)
		{
			g_40km_dist = val;
		}
		if (strcmp(name, "take_over_speed") == 0)
		{
			g_take_over_speed = val;
		}
		if (strcmp(name, "take_over_threshold") == 0)
		{
			g_take_over_threshold = val;
		}
		if (strcmp(name, "change_lane_speed") == 0)
		{
			g_change_lane_speed = val;
		}
		if (strcmp(name, "10km_change_steer") == 0)
		{
			g_10km_change_steer = val / 1000.0;
		}
		if (strcmp(name, "20km_change_steer") == 0)
		{
			g_20km_change_steer = val / 1000.0;
		}
		if (strcmp(name, "30km_change_steer") == 0)
		{
			g_30km_change_steer = val / 1000.0;
		}
		if (strcmp(name, "40km_change_steer") == 0)
		{
			g_40km_change_steer = val / 1000.0;
		}
		if (strcmp(name, "50km_change_steer") == 0)
		{
			g_50km_change_steer = val / 1000.0;
		}
		if (strcmp(name, "60km_change_steer") == 0)
		{
			g_60km_change_steer = val / 1000.0;
		}
		if (strcmp(name, "straight_alpha") == 0)
		{
			g_straight_alpha = val;
		}
		if (strcmp(name, "left_turn_alpha") == 0)
		{
			g_left_turn_alpha = val;
		}
		if (strcmp(name, "right_turn_alpha") == 0)
		{
			g_right_turn_alpha = val;
		}
		if (strcmp(name, "left_turn_pt_adjust") == 0)
		{
			g_left_turn_pt_adjust = val;
		}
		if (strcmp(name, "left_turn_pt2x_adjust") == 0)
		{
			g_left_turn_pt2x_adjust = val;
		}
		if (strcmp(name, "left_turn_pt2y_adjust") == 0)
		{
			g_left_turn_pt2y_adjust = val;
		}

		if (strcmp(name, "right_turn_pt_adjust") == 0)
		{
			g_right_turn_pt_adjust = val;
		}
		if (strcmp(name, "right_turn_pt2x_adjust") == 0)
		{
			g_right_turn_pt2x_adjust = val;
		}
		if (strcmp(name, "right_turn_pt2y_adjust") == 0)
		{
			g_right_turn_pt2y_adjust = val;
		}
		if (strcmp(name, "uturn_alpha1") == 0)
		{
			g_uturn_alpha1 = val;
		}
		if (strcmp(name, "uturn_alpha2") == 0)
		{
			g_uturn_alpha2 = val;
		}
		if (strcmp(name, "uturn_alpha3") == 0)
		{
			g_uturn_alpha3 = val;
		}
		if (strcmp(name, "uturn_alpha4") == 0)
		{
			g_uturn_alpha4 = val;
		}
		if (strcmp(name, "uturn_sub_pt_adjust") == 0)
		{
			g_uturn_sub_pt_adjust = val;
		}
		if (strcmp(name, "sharp_turn_in_dist") == 0)
		{
			g_sharp_turn_in_dist = val;
		}
		if (strcmp(name, "sharp_turn_out_dist") == 0)
		{
			g_sharp_turn_out_dist = val;
		}
		if (strcmp(name, "sharp_turn_speed") == 0)
		{
			g_sharp_turn_speed = val;
		}
		if (strcmp(name, "natural_look_dist") == 0)
		{
			g_natural_look_dist = val;
		}
		if (strcmp(name, "speed_up_timer") == 0)
		{
			g_speed_up_timer = val;
		}
		if (strcmp(name, "cross_country_reach_dist_threshold") == 0)
		{
			g_cross_country_reach_dist_threshold = val;
		}
		if (strcmp(name, "trace_global_task_pt_valid_dist") == 0)
		{
			g_trace_global_task_pt_valid_dist = val;
		}
		if (strcmp(name, "spd_error_detect_switch") == 0)
		{
			g_spd_error_detect_switch = val;
		}
		if (strcmp(name, "one_curve_level_speed") == 0)
		{
			g_one_curve_level_speed  = val;
		}
		if (strcmp(name, "two_curve_level_speed") == 0)
		{
			g_two_curve_level_speed  = val;
		}
		if (strcmp(name, "three_curve_level_speed") == 0)
		{
			g_three_curve_level_speed  = val;
		}
		if (strcmp(name, "four_curve_level_speed") == 0)
		{
			g_four_curve_level_speed  = val;
		}
		if (strcmp(name, "uturn_stop_once_counter") == 0)
		{
			g_uturn_stop_once_counter = val;
		}
		if (strcmp(name, "reverse_start_timer_value") == 0)
		{
			g_reverse_start_timer_value = val;
		}
		if (strcmp(name, "reverse_speed") == 0)
		{
			reverse_speed = val;
		}
		if (strcmp(name, "reverse_plan_dist") == 0)
		{
			reverse_plan_dist = val;
		}
		if (strcmp(name, "reverse_plan_pts_step") == 0)
		{
			reverse_plan_pts_step = val;
		}
		if (strcmp(name, "reverse_step") == 0)
		{
			reverse_step = val;
		}
		if (strcmp(name, "reverse_pt_num_limit") == 0)
		{
			reverse_pt_num_limit = val;
		}
		if (strcmp(name, "reverse_back_stop_dist") == 0)
		{
			reverse_back_stop_dist = val;
		}
		if (strcmp(name, "reverse_test_start_dist") == 0)
		{
			reverse_test_start_dist = val;
		}
		if (strcmp(name, "reverse_test_stop_dist") == 0)
		{
			reverse_test_stop_dist = val;
		}
		if (strcmp(name, "lateral_parking_waiting_timer_value") == 0)
		{
			g_lateral_parking_waiting_timer_value = val;
		}
		if (strcmp(name, "lateral_parking_geo_switch") == 0)
		{
			g_lateral_parking_geo_switch = val;
		}
		if (strcmp(name, "trace_global_task_pt_valid_dist") == 0)
		{
			g_trace_global_task_pt_valid_dist = val;
		}
		if (strcmp(name, "s_obs_detect_switch") == 0)
		{
			g_s_obs_detect_switch = val;
		}
	}

	reversePlan.ReverseInit(reverse_speed, 
		reverse_plan_dist, 
		reverse_plan_pts_step, 
		reverse_step, 
		reverse_pt_num_limit, 
		reverse_back_stop_dist,
		reverse_test_start_dist, 
		reverse_test_stop_dist);
	fclose(fp);
}

/*==================================================================
 * 函数名  ：	void fill_dyn_obs_to_grid()
 * 功能    ：	把动态障碍物填入栅格
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void fill_dyn_obs_to_grid()
{
	int i, j, k;
	COOR2 l_b_pt;//[左下角]
	COOR2 r_u_pt;//[右上角]

	for (i=0; i<g_dyn_obs_num; i++)
	{
		//[规整，即使旋转了，扩展不会太大]
		//[所谓规整，用横平竖直的外接矩形去描述]
		l_b_pt.x = g_dyn_obs[i].coor2[0].x;
		l_b_pt.y = g_dyn_obs[i].coor2[0].y;
		r_u_pt.x = g_dyn_obs[i].coor2[0].x;
		r_u_pt.y = g_dyn_obs[i].coor2[0].y;
		for (j=0; j<4; j++)
		{
			if (l_b_pt.x > g_dyn_obs[i].coor2[j].x)
			{
				l_b_pt.x = g_dyn_obs[i].coor2[j].x;
			}

			if (l_b_pt.y > g_dyn_obs[i].coor2[j].y)
			{
				l_b_pt.y = g_dyn_obs[i].coor2[j].y;
			}

			if (r_u_pt.x < g_dyn_obs[i].coor2[j].x)
			{
				r_u_pt.x = g_dyn_obs[i].coor2[j].x;
			}

			if (r_u_pt.y < g_dyn_obs[i].coor2[j].y)
			{
				r_u_pt.y = g_dyn_obs[i].coor2[j].y;
			}
		}

		if (l_b_pt.x < 0)
		{
			l_b_pt.x = l_b_pt.x / 25 + (g_grid_center.x - 1);
			if (l_b_pt.x < 0)
			{
				l_b_pt.x = 0;
			}
		}
		else
		{
			l_b_pt.x = l_b_pt.x / 25 + (g_grid_center.x + 1);
			if (l_b_pt.x >= GRID_WIDTH)
			{
				l_b_pt.x = GRID_WIDTH - 1;
			}
		}

		l_b_pt.y = l_b_pt.y / 25 + g_grid_center.y;
		if (l_b_pt.y < 0)
		{
			l_b_pt.y = 0;
		}
		if (l_b_pt.y > GRID_HEIGHT)
		{
			l_b_pt.y = GRID_HEIGHT - 1;
		}




		if (r_u_pt.x < 0)
		{
			r_u_pt.x = r_u_pt.x / 25 + (g_grid_center.x - 1);
			if (r_u_pt.x < 0)
			{
				r_u_pt.x = 0;
			}
		}
		else
		{
			r_u_pt.x = r_u_pt.x / 25 + (g_grid_center.x + 1);
			if (r_u_pt.x >= GRID_WIDTH)
			{
				r_u_pt.x = GRID_WIDTH - 1;
			}
		}

		r_u_pt.y = r_u_pt.y / 25 + g_grid_center.y;
		if (r_u_pt.y < 0)
		{
			r_u_pt.y = 0;
		}
		if (r_u_pt.y > GRID_HEIGHT)
		{
			r_u_pt.y = GRID_HEIGHT - 1;
		}

		for (j = l_b_pt.y; j <= r_u_pt.y; j++)
		{
			for (k = l_b_pt.x; k <= r_u_pt.x; k++)
			{
				g_grid_map[j][k] = 9;
			}
		}
	}

	for (i = 0; i<g_lateral_dyn_obs_num; i++)
	{
		//[规整，即使旋转了，扩展不会太大]
		//[所谓规整，用横平竖直的外接矩形去描述]
		l_b_pt.x = g_lateral_dyn_obs[i].coor2[0].x;
		l_b_pt.y = g_lateral_dyn_obs[i].coor2[0].y;
		r_u_pt.x = g_lateral_dyn_obs[i].coor2[0].x;
		r_u_pt.y = g_lateral_dyn_obs[i].coor2[0].y;
		for (j = 0; j<4; j++)
		{
			if (l_b_pt.x > g_lateral_dyn_obs[i].coor2[j].x)
			{
				l_b_pt.x = g_lateral_dyn_obs[i].coor2[j].x;
			}

			if (l_b_pt.y > g_lateral_dyn_obs[i].coor2[j].y)
			{
				l_b_pt.y = g_lateral_dyn_obs[i].coor2[j].y;
			}

			if (r_u_pt.x < g_lateral_dyn_obs[i].coor2[j].x)
			{
				r_u_pt.x = g_lateral_dyn_obs[i].coor2[j].x;
			}

			if (r_u_pt.y < g_lateral_dyn_obs[i].coor2[j].y)
			{
				r_u_pt.y = g_lateral_dyn_obs[i].coor2[j].y;
			}
		}

		if (l_b_pt.x < 0)
		{
			l_b_pt.x = l_b_pt.x / 25 + (g_grid_center.x - 1);
			if (l_b_pt.x < 0)
			{
				l_b_pt.x = 0;
			}
		}
		else
		{
			l_b_pt.x = l_b_pt.x / 25 + (g_grid_center.x + 1);
			if (l_b_pt.x >= GRID_WIDTH)
			{
				l_b_pt.x = GRID_WIDTH - 1;
			}
		}

		l_b_pt.y = l_b_pt.y / 25 + g_grid_center.y;
		if (l_b_pt.y < 0)
		{
			l_b_pt.y = 0;
		}
		if (l_b_pt.y > GRID_HEIGHT)
		{
			l_b_pt.y = GRID_HEIGHT - 1;
		}

		if (r_u_pt.x < 0)
		{
			r_u_pt.x = r_u_pt.x / 25 + (g_grid_center.x - 1);
			if (r_u_pt.x < 0)
			{
				r_u_pt.x = 0;
			}
		}
		else
		{
			r_u_pt.x = r_u_pt.x / 25 + (g_grid_center.x + 1);
			if (r_u_pt.x >= GRID_WIDTH)
			{
				r_u_pt.x = GRID_WIDTH - 1;
			}
		}

		r_u_pt.y = r_u_pt.y / 25 + g_grid_center.y;
		if (r_u_pt.y < 0)
		{
			r_u_pt.y = 0;
		}
		if (r_u_pt.y > GRID_HEIGHT)
		{
			r_u_pt.y = GRID_HEIGHT - 1;
		}

		for (j = l_b_pt.y; j <= r_u_pt.y; j++)
		{
			for (k = l_b_pt.x; k <= r_u_pt.x; k++)
			{
				g_grid_map[j][k] = 9;
			}
		}
	}
}

/*==================================================================
 * 函数名  ：	void keep_dyn_obs(PL_FUNC_INPUT *pl_input)
 * 功能    ：	更新规划内部的动态障碍物集合，目前不进行匹配。暂不跟踪目标，只对车道是否合适进行判断
 * 输入参数：	PL_FUNC_INPUT *pl_input		环境数据输入
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void keep_dyn_obs(PL_FUNC_INPUT *pl_input)
{
	int i, j;

	DYN_OBSTACLE_AREA dyn_obstacle_area;
	COOR2 center;
	SPD2D speed;

	g_dyn_obs_num = 0;
	memset(g_dyn_obs, 0, MAX_DYN_AREA * sizeof(DYN_OBS));

	g_lateral_dyn_obs_num = 0;
	memset(g_lateral_dyn_obs, 0, MAX_DYN_AREA * sizeof(DYN_OBS));

	if (g_dyn_obs_switch == 0)
		return;

	//[1、提取横向动态障碍物]
	for (i = 0; i < pl_input->fu_pl_data.gridmap.d_areas.valid_area_num; i++)
	{
		memcpy(&dyn_obstacle_area, &pl_input->fu_pl_data.gridmap.d_areas.areas[i], sizeof(DYN_OBSTACLE_AREA));

		center.x = (dyn_obstacle_area.mask.coor2[0].x + dyn_obstacle_area.mask.coor2[2].x) / 2;
		center.y = (dyn_obstacle_area.mask.coor2[0].y + dyn_obstacle_area.mask.coor2[2].y) / 2;
		speed = dyn_obstacle_area.speed;

		//[横向速度高且横向速度低于5km/h，判定为横向移动行人]
		if (abs(speed.y) < abs(speed.x) && abs(speed.x) <= (5 * CM))
		{
			//[远距离以及在身后的横向移动障碍物不处理]
			if (center.y > 3000)
				continue;
			else
			{
				if (center.y <= 400)
					continue;
			}

			g_lateral_dyn_obs[g_lateral_dyn_obs_num].center = center;
			memcpy(g_lateral_dyn_obs[g_lateral_dyn_obs_num].coor2, pl_input->fu_pl_data.gridmap.d_areas.areas[i].mask.coor2, 4 * sizeof(COOR2));
			g_lateral_dyn_obs[g_lateral_dyn_obs_num].speed = speed;

			double dist = MAX_VALUE;
			int index = -1;
			COOR2 temp_pt;
			temp_pt.x = 0;
			temp_pt.y = 0;
			for (j = 0; j < 4; j++)
			{
				if (dist_point(&g_lateral_dyn_obs[g_lateral_dyn_obs_num].coor2[j], &temp_pt) < dist)
				{
					dist = dist_point(&g_lateral_dyn_obs[g_lateral_dyn_obs_num].coor2[j], &temp_pt);
					index = j;
				}
			}
			g_lateral_dyn_obs[g_lateral_dyn_obs_num].nearest_pt = g_lateral_dyn_obs[g_lateral_dyn_obs_num].coor2[index];
			g_lateral_dyn_obs_num++;
		}
	}

	//[提取纵向移动障碍物]
	COOR2 left_edge[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int left_edge_num = 0;
	COOR2 right_edge[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int right_edge_num = 0;
	int l_index = 0;
	int r_index = 0;
	COOR2 temp_line[200];
	int temp_line_num = 0;
	int step = 0;

	l_index = pl_input->fu_pl_data.lines.valid_num_in_l_wing;
	if (l_index > 0)
	{
		if (l_index == 3)
		{//[只取到左侧车道]
			l_index = 2;
		}
		for (i = 0; i < pl_input->fu_pl_data.lines.l_edges[l_index - 1].valid_num_points; i++)
		{
			left_edge[i].x = pl_input->fu_pl_data.lines.l_edges[l_index - 1].line[i].x;
			left_edge[i].y = pl_input->fu_pl_data.lines.l_edges[l_index - 1].line[i].y;
		}
		left_edge_num = pl_input->fu_pl_data.lines.l_edges[l_index - 1].valid_num_points;

		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;

		step = (int)(left_edge[left_edge_num - 1].y - left_edge[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(left_edge, left_edge_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(left_edge, temp_line, temp_line_num * sizeof(COOR2));
		left_edge_num = temp_line_num;
	}

	r_index = pl_input->fu_pl_data.lines.valid_num_in_r_wing;
	if (r_index > 0)
	{
		if (r_index == 3)
		{//[只取到右侧车道]
			r_index = 2;
		}
		for (i = 0; i < pl_input->fu_pl_data.lines.r_edges[r_index - 1].valid_num_points; i++)
		{
			right_edge[i].x = pl_input->fu_pl_data.lines.r_edges[r_index - 1].line[i].x;
			right_edge[i].y = pl_input->fu_pl_data.lines.r_edges[r_index - 1].line[i].y;
		}
		right_edge_num = pl_input->fu_pl_data.lines.r_edges[r_index - 1].valid_num_points;

		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;

		step = (int)(right_edge[right_edge_num - 1].y - right_edge[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(right_edge, right_edge_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(right_edge, temp_line, temp_line_num * sizeof(COOR2));
		right_edge_num = temp_line_num;
	}


	if (pl_input->fu_pl_data.gridmap.d_areas.valid_area_num > MAX_DYN_AREA)
	{
		pl_input->fu_pl_data.gridmap.d_areas.valid_area_num = MAX_DYN_AREA;
	}
			
	for (i = 0; i<pl_input->fu_pl_data.gridmap.d_areas.valid_area_num; i++)
	{
		memcpy(&dyn_obstacle_area, &pl_input->fu_pl_data.gridmap.d_areas.areas[i], sizeof(DYN_OBSTACLE_AREA));

		center.x = (dyn_obstacle_area.mask.coor2[0].x + dyn_obstacle_area.mask.coor2[2].x) / 2;
		center.y = (dyn_obstacle_area.mask.coor2[0].y + dyn_obstacle_area.mask.coor2[2].y) / 2;
		speed = dyn_obstacle_area.speed;

		// 		if (speed.y <= 277)
		// 		{//[10km/h以下当静态避障]
		// 			continue;
		// 		}

		if (g_road_type == 0)
		{//[结构化道路]
			if (g_stat == S_CROSS_UND)
			{//[道路理解剔除不需要理睬的动态障碍]
				if (center.y > 6000 || center.y < -1500)
				{//[60m以外的不管]
					continue;
				}

				if (left_edge_num >= 2 && right_edge_num >= 2)
				{
					int l_x = 0;
					int r_x = 0;
					get_x_coord(center.y, left_edge, left_edge_num, &l_x);
					get_x_coord(center.y, right_edge, right_edge_num, &r_x);
					if (center.x < l_x || center.x > r_x)
					{//[只看道路内的]
						continue;
					}

					if ((center.y > 1000 && speed.y > g_real_speed) || (center.y < -1500 && speed.y < g_real_speed))
					{//[10m之外，车速比我方高不管， 15m之后，比我方速度小不管]
						continue;;
					}
				}
			}
			else
			{
				if (center.y > 6000)
				{//[60m以外的不管]
					continue;
				}

				if (left_edge_num >= 2 && right_edge_num >= 2)
				{
					int l_x = 0;
					int r_x = 0;
					get_x_coord(center.y, left_edge, left_edge_num, &l_x);
					get_x_coord(center.y, right_edge, right_edge_num, &r_x);
					if (center.x < l_x || center.x > r_x)
					{//[结构化道路稳定的情况下，只看道路内的]
						continue;
					}
				}
			}
		}
		else
		{//[乡村道路]
			if (g_stat == S_CROSS_UND)
			{//[道路理解剔除不需要理睬的动态障碍]
				if (center.y > 6000 || center.y < -1500)
				{//[60m以外的不管]
					continue;
				}

				if (left_edge_num >= 2 && right_edge_num >= 2)
				{
					int l_x = 0;
					int r_x = 0;
					get_x_coord(center.y, left_edge, left_edge_num, &l_x);
					get_x_coord(center.y, right_edge, right_edge_num, &r_x);
					if (center.x < l_x || center.x > r_x)
					{//[只看道路内的]
						continue;
					}

					if ((center.y > 1000 && speed.y > g_real_speed) || (center.y < -1500 && speed.y < g_real_speed))
					{//[10m之外，车速比我方高不管， 15m之后，比我方速度小不管]
						continue;;
					}
				}
			}
			else
			{
				if (center.y > 6000 || center.y < -1500 || \
					center.x < -500 || center.x > 500)
				{//[上述条件下动态障碍物不管]
					continue;
				}

			}
		}


		g_dyn_obs[g_dyn_obs_num].center = center;
		memcpy(g_dyn_obs[g_dyn_obs_num].coor2, pl_input->fu_pl_data.gridmap.d_areas.areas[i].mask.coor2, 4 * sizeof(COOR2));
		g_dyn_obs[g_dyn_obs_num].speed = speed;

		double dist = MAX_VALUE;
		int index = -1;
		COOR2 temp_pt;
		temp_pt.x = 0;
		temp_pt.y = 0;
		for (j=0; j<4; j++)
		{
			if (dist_point(&g_dyn_obs[g_dyn_obs_num].coor2[j], &temp_pt) < dist)
			{
				dist = dist_point(&g_dyn_obs[g_dyn_obs_num].coor2[j], &temp_pt);
				index = j;
			}
		}
		g_dyn_obs[g_dyn_obs_num].nearest_pt = g_dyn_obs[g_dyn_obs_num].coor2[index];
		g_dyn_obs_num++;
	}
}

/*==================================================================
* 函数名  ：int trace_in_natural_road_histogram_dist()
* 功能    ：	防止因为道路两侧障碍物伸入道路过多，检测成窄点，进行是否能够直行的检测
* 输入参数：
* 输出参数：
* 返回值  ：int		0  可以顺畅通行		-1  有窄点
* 作者    ：
* 日期    ：
* 修改记录：
*==================================================================*/
int g_histogram_dist[HISTOGRAM_DIST_SIZE];
#define LOOK_DIST 2400
int trace_in_natural_road_histogram_dist()
{
	int ret = -1;
	memset(g_histogram_dist, 0, sizeof(int) * HISTOGRAM_DIST_SIZE);

	int look_dist_cell = LOOK_DIST / GRID_LEN_PER_CELL;
	for (int i = 0; i < HISTOGRAM_DIST_SIZE; i++)
	{
		for (int j = g_grid_center.y; j < g_grid_center.y + look_dist_cell; j++)
		{
			int x = i + g_grid_center.x - (HISTOGRAM_DIST_SIZE / 2);
			if (g_grid_map[j][x] == 0)
				g_histogram_dist[i]++;
			else
				break;
		}
	}

	int count1 = 0;


	//[6.正前方有很近的障碍物]
	count1 = 0;
	for (int i = 3; i < 7; i++)
	{
		if (g_histogram_dist[i] <= (500 / GRID_LEN_PER_CELL))
		{
			ret = -4;
			return ret;
		}
	}

	//[1.全部安全]
	count1 = 0;
	for (int i = 0; i < HISTOGRAM_DIST_SIZE; i++)
		if (g_histogram_dist[i] == look_dist_cell)
			count1++;
	if (count1 == HISTOGRAM_DIST_SIZE)
	{
		ret = 0;
		return ret;
	}

	//[2.边缘有一点]
	count1 = 0;
	for (int i = 1; i < HISTOGRAM_DIST_SIZE - 1; i++)
		if (g_histogram_dist[i] == look_dist_cell)
			count1++;

	if (count1 == HISTOGRAM_DIST_SIZE - 2)
	{
		if (g_histogram_dist[0] >(1800 / GRID_LEN_PER_CELL) && \
			g_histogram_dist[HISTOGRAM_DIST_SIZE - 1] > (1800 / GRID_LEN_PER_CELL))
		{
			ret = 0;
			return ret;
		}
	}

	//[3.右侧有一点]
	count1 = 0;
	for (int i = 0; i < HISTOGRAM_DIST_SIZE - 3; i++)
		if (g_histogram_dist[i] == look_dist_cell)
			count1++;

	if (count1 == HISTOGRAM_DIST_SIZE - 3)
	{
		if (g_histogram_dist[HISTOGRAM_DIST_SIZE - 1] >(2000 / GRID_LEN_PER_CELL) && \
			g_histogram_dist[HISTOGRAM_DIST_SIZE - 2] > (2000 / GRID_LEN_PER_CELL) && \
			g_histogram_dist[HISTOGRAM_DIST_SIZE - 3] > (2000 / GRID_LEN_PER_CELL))
		{
			ret = 0;
			return ret;
		}
	}

	//[4.左侧有一点]
	count1 = 0;
	for (int i = 3; i < HISTOGRAM_DIST_SIZE; i++)
		if (g_histogram_dist[i] == look_dist_cell)
			count1++;

	if (count1 == HISTOGRAM_DIST_SIZE - 3)
	{
		if (g_histogram_dist[0] >(2000 / GRID_LEN_PER_CELL) && \
			g_histogram_dist[1] > (2000 / GRID_LEN_PER_CELL) && \
			g_histogram_dist[2] > (2000 / GRID_LEN_PER_CELL))
		{
			ret = 0;
			return ret;
		}
	}

	//[5.边缘有紧急障碍]
	if (g_histogram_dist[0] <= (800 / GRID_LEN_PER_CELL))
	{
		ret = -2;
		return ret;
	}
	else if (g_histogram_dist[HISTOGRAM_DIST_SIZE - 1] <= (800 / GRID_LEN_PER_CELL))
	{
		ret = -3;
		return ret;
	}

	return ret;
}

//***********************************************************************************************
//                                zgccmax 2012.May.30
//void get_pl_speed()
//param:    void
//return:   void
//discribe: 依据情况进行速度规划
//***********************************************************************************************
void get_pl_speed(PL_FUNC_INPUT *pl_input)
{
	double acc;
	double time;
	int ret = 0;

	if (g_stat == S_ROAD_NAV)
	{
		if (g_navi_state == TRACE_LANE)
		{
			double rate = 0;
			ret = check_path_for_big_angle(rate);

			if (g_take_over_speed_up_timer > 0)
			{
				if (ret == 0)
				{
					g_take_over_speed_up_timer = 0;
				}
			}
			else
			{
				g_wp_speed = (int)(g_wp_speed * rate);
			}

			if (ret == -2)
			{//[振荡]
				g_wp_speed = (int)(3 * CM);
			}

			if (g_unsafe_dist == MAX_VALUE)
			{//[正常启动加速]
				if (g_real_speed < (int)(10 * CM))
				{
					g_cur_speed = g_wp_speed > (int)(17 * CM) ? (int)(17 * CM) : g_wp_speed;
				}
				else if (g_real_speed < (int)(15 * CM))
				{
					g_cur_speed = g_wp_speed > (int)(22 * CM) ? (int)(22 * CM) : g_wp_speed;
				}
				else if (g_real_speed < (int)(20 * CM))
				{
					g_cur_speed = g_wp_speed > (int)(27 * CM) ? (int)(27 * CM) : g_wp_speed;
				}
				else if (g_real_speed < (int)(25 * CM))
				{
					g_cur_speed = g_wp_speed > (int)(32 * CM) ? (int)(32 * CM) : g_wp_speed;
				}
				else if (g_real_speed < (int)(30 * CM))
				{
					g_cur_speed = g_wp_speed > (int)(37 * CM) ? (int)(37 * CM) : g_wp_speed;
				}
				else if (g_real_speed < (int)(35 * CM))
				{
					g_cur_speed = g_wp_speed > (int)(42 * CM) ? (int)(42 * CM) : g_wp_speed;
				}
				else
				{
					g_cur_speed = g_wp_speed;
				}

				if (g_road_type == 0)
				{
					if (g_change_fade_timer > 0)
					{//[刚换完道，保持低速]
						g_cur_speed = (int)(12 * CM);
					}

					if (g_finish_cross > 0)
					{//[结束路口低速]
						g_cross_smooth_end_pt.x = pl_input->state.pos.ins_coord.x;
						g_cross_smooth_end_pt.y = pl_input->state.pos.ins_coord.y;

						g_cur_speed = (int)(12 * CM);
						double dist = dist_point(&g_cross_smooth_start_pt, &g_cross_smooth_end_pt);
						if (dist > 1500)
						{//[平滑15m]
							g_finish_cross = 0;
						}
					}

					if (g_far_obs_warning_flag == 1 && g_change_fade_timer <= 0 && g_finish_cross != 1)
					{//[低速优先]
						if (g_real_speed > (int)(20 * CM))
						{
							g_cur_speed = (int)(15 * CM);
						}
						else if (g_real_speed <= (int)(20 * CM) && g_real_speed >= (int)(15 * CM))
						{
							g_cur_speed = g_real_speed;
						}
						else
						{
							if (g_real_speed < (int)(6 * CM))
							{
								g_cur_speed = g_wp_speed > (int)(6 * CM) ? (int)(6 * CM) : g_wp_speed;
							}
							else if (g_real_speed < (int)(10 * CM))
							{
								g_cur_speed = g_wp_speed > (int)(10 * CM) ? (int)(10 * CM) : g_wp_speed;
							}
							else if (g_real_speed < (int)(15 * CM))
							{
								g_cur_speed = g_wp_speed > (int)(15 * CM) ? (int)(15 * CM) : g_wp_speed;
							}
						}
					}
				}//[if (g_road_type == 0)]
				else
				{//[乡村道路速度策略]
					if (g_change_fade_timer > 0)
					{//[刚换完道，保持低速]
						g_cur_speed = (int)(12 * CM);
					}

					if (g_finish_cross > 0)
					{//[结束路口低速]
						g_cross_smooth_end_pt.x = pl_input->state.pos.ins_coord.x;
						g_cross_smooth_end_pt.y = pl_input->state.pos.ins_coord.y;

						g_cur_speed = (int)(12 * CM);
						double dist = dist_point(&g_cross_smooth_start_pt, &g_cross_smooth_end_pt);
						if (dist > 1500)
						{//[平滑15m]
							g_finish_cross = 0;
						}
					}

					if (g_far_obs_warning_flag == 1 && g_change_fade_timer <= 0 && g_finish_cross != 1)
					{//[低速优先]
						if (g_real_speed > (int)(20 * CM))
						{
							g_cur_speed = (int)(15 * CM);
						}
						else if (g_real_speed <= (int)(20 * CM) && g_real_speed >= (int)(15 * CM))
						{
							g_cur_speed = g_real_speed;
						}
						else
						{
							if (g_real_speed < (int)(6 * CM))
							{
								g_cur_speed = g_wp_speed > (int)(6 * CM) ? (int)(6 * CM) : g_wp_speed;
							}
							else if (g_real_speed < (int)(10 * CM))
							{
								g_cur_speed = g_wp_speed > (int)(10 * CM) ? (int)(10 * CM) : g_wp_speed;
							}
							else if (g_real_speed < (int)(15 * CM))
							{
								g_cur_speed = g_wp_speed > (int)(15 * CM) ? (int)(15 * CM) : g_wp_speed;
							}
						}
					}

					if (g_is_stopping == 1)
					{
						g_cur_speed = g_real_speed - (int)(5 * CM);
						if (g_cur_speed < (int)(20 * CM))
						{
							g_cur_speed = (int)(20 * CM);
						}
					}
				}
			}
			else
			{//[有障碍]
				if (g_unsafe_dist <= 600)
				{
					g_cur_speed = 0;
				}
				else if (g_unsafe_dist > 600 && g_unsafe_dist <= 1000)
				{
					g_cur_speed = (int)(4 * CM);
				}
				else if (g_unsafe_dist > 1000 && g_unsafe_dist <= 2000)
				{
					g_cur_speed = (int)(15 * CM);
				}
				else if (g_unsafe_dist > 2000 && g_unsafe_dist <= 3000)
				{
					g_cur_speed = (int)(20 * CM);
				}
				else if (g_unsafe_dist > 3000 && g_unsafe_dist <= 4000)
				{
					g_cur_speed = (int)(25 * CM);
				}
				else
				{
					g_cur_speed = (int)(30 * CM);
				}
			}

		}//[if (g_navi_state == TRACE_LANE)]
		else if (g_navi_state == FOLLOW_THE_CAR)
		{
			if (g_wp_speed < g_follow_car_speed)
			{
				//[小于前方车辆速度，保持]
			}
			else
				g_wp_speed = g_follow_car_speed;

			if (g_real_speed < (int)(10 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(12 * CM) ? (int)(12 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(15 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(17 * CM) ? (int)(17 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(20 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(22 * CM) ? (int)(22 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(25 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(27 * CM) ? (int)(27 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(30 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(32 * CM) ? (int)(32 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(35 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(37 * CM) ? (int)(37 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(40 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(42 * CM) ? (int)(42 * CM) : g_wp_speed;
			}
			else
			{
				g_cur_speed = g_wp_speed;
			}

		}
		else if (g_navi_state == TAKEOVER_FROM_LEFT || g_navi_state == TAKEOVER_FROM_RIGHT)
		{
			g_wp_speed = (int)(g_take_over_speed * CM);

			//g_cur_speed = g_real_speed > g_wp_speed ? g_wp_speed : (g_real_speed + (int)(3 * CM));
			if (g_real_speed < (int)(10 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(12 * CM) ? (int)(12 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(15 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(17 * CM) ? (int)(17 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(20 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(22 * CM) ? (int)(22 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(25 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(27 * CM) ? (int)(27 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(30 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(32 * CM) ? (int)(32 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(35 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(37 * CM) ? (int)(37 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(40 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(42 * CM) ? (int)(42 * CM) : g_wp_speed;
			}
			else
			{
				g_cur_speed = g_wp_speed;
			}
		}
		else if (g_navi_state == CHANGE_TO_LEFT_LANE || g_navi_state == CHANGE_TO_RIGHT_LANE)
		{
			if (g_unsafe_dist_for_change_lane < 600)
			{
				g_change_lane_speed = 0;
				g_wp_speed = g_change_lane_speed;
				g_cur_speed = g_wp_speed;
			}
			else if (g_unsafe_dist_for_change_lane >= 600 && g_unsafe_dist_for_change_lane < 1000)
			{
				g_change_lane_speed = (int)(4 * CM);
				g_wp_speed = g_change_lane_speed;
				g_cur_speed = g_wp_speed;
			}
			else if (g_unsafe_dist_for_change_lane >= 1000 && g_unsafe_dist_for_change_lane < 2000)
			{
				g_change_lane_speed = (int)(10 * CM);
				g_wp_speed = g_change_lane_speed;
				g_cur_speed = g_wp_speed;
			}
			else if (g_unsafe_dist_for_change_lane >= 2000 && g_unsafe_dist_for_change_lane < 3000)
			{
				g_change_lane_speed = (int)(15 * CM);
				g_wp_speed = g_change_lane_speed;
				g_cur_speed = g_wp_speed;
			}
			else
			{
				g_change_lane_speed = (int)(15 * CM);
				g_wp_speed = g_change_lane_speed;
				g_cur_speed = g_wp_speed;
			}
		}
		else if (g_navi_state == TRACE_IN_NATURAL_ROAD)
		{
			//[20m-36m速度线性控制]
			if (g_car_identity == ALV3)
			{
				if (g_natural_road_search_dist < 2000)
				{
					g_wp_speed = (int)(15 * CM);
				}
				else
				{
					if (g_natural_road_search_dist > 3600)
					{
						g_natural_road_search_dist = 3600;
					}
					g_wp_speed = (int)((g_natural_road_search_dist - 2000.0) / (3600 - 2000) * \
						(g_wp_speed - (int)(15 * CM)) + (int)(15 * CM));
				}
			}
			else
			{
				if (g_natural_road_search_dist < 2000)
				{
					g_wp_speed = (int)(20 * CM);
				}
				else
				{
					if (g_natural_road_search_dist > 3600)
					{
						g_natural_road_search_dist = 3600;
					}
					g_wp_speed = (int)((g_natural_road_search_dist - 2000.0) / (3600 - 2000) * \
						(g_wp_speed - (int)(20 * CM)) + (int)(20 * CM));
				}
			}


			if (g_sharp_turn_flag == 1)
			{//[急弯减速]
				g_wp_speed = (int)(g_sharp_turn_speed * CM);
			}

			double rate = 0;
			ret = check_path_for_big_angle(rate);

			if (g_unsafe_dist == MAX_VALUE)
			{
				if (g_real_speed < (int)(10 * CM))
				{
					g_cur_speed = g_wp_speed > (int)(12 * CM) ? (int)(12 * CM) : g_wp_speed;
				}
				else if (g_real_speed < (int)(15 * CM))
				{
					g_cur_speed = g_wp_speed > (int)(17 * CM) ? (int)(17 * CM) : g_wp_speed;
				}
				else if (g_real_speed < (int)(20 * CM))
				{
					g_cur_speed = g_wp_speed > (int)(22 * CM) ? (int)(22 * CM) : g_wp_speed;
				}
				else if (g_real_speed < (int)(25 * CM))
				{
					g_cur_speed = g_wp_speed > (int)(27 * CM) ? (int)(27 * CM) : g_wp_speed;
				}
				else if (g_real_speed < (int)(30 * CM))
				{
					g_cur_speed = g_wp_speed > (int)(32 * CM) ? (int)(32 * CM) : g_wp_speed;
				}
				else
				{
					g_cur_speed = g_wp_speed;
				}

				int result = trace_in_natural_road_histogram_dist();

				if (result != 0 || g_has_mid_obs)
				{
					if (g_slow_down_dist > 2500)
					{
						g_cur_speed = g_cur_speed;
					}
					if (g_slow_down_dist >= 2000 && g_slow_down_dist <= 2500)
					{
						g_cur_speed = g_cur_speed > (int)(16 * CM) ? (int)(16 * CM) : g_cur_speed;
					}
					else if (g_slow_down_dist < 2000 && g_slow_down_dist >= 1600)
					{
						g_cur_speed = g_cur_speed >(int)(13 * CM) ? (int)(13 * CM) : g_cur_speed;
					}
					else if (g_slow_down_dist < 1600 && g_slow_down_dist >= 1200)
					{
						g_cur_speed = g_cur_speed >(int)(10 * CM) ? (int)(10 * CM) : g_cur_speed;
					}
					else if (g_slow_down_dist < 1200 && g_slow_down_dist >= 800)
					{
						g_cur_speed = g_cur_speed >(int)(7 * CM) ? (int)(7 * CM) : g_cur_speed;
					}
					else if (g_slow_down_dist < 800)
					{
						g_cur_speed = g_cur_speed >(int)(4.5 * CM) ? (int)(4.5 * CM) : g_cur_speed;
					}


					if (g_narrow_dist <= 2000 && g_narrow_dist > 1600)
					{
						g_cur_speed = g_cur_speed > (int)(15 * CM) ? (int)(15 * CM) : g_cur_speed;
					}
					else if (g_narrow_dist <= 1600 && g_narrow_dist > 1200)
					{
						g_cur_speed = g_cur_speed > (int)(10 * CM) ? (int)(10 * CM) : g_cur_speed;
					}
					else if (g_narrow_dist <= 1200)
					{
						g_cur_speed = g_cur_speed > (int)(5 * CM) ? (int)(5 * CM) : g_cur_speed;
					}

					if (result == -4)
					{
						g_cur_speed = 0;
					}
				}
				
				if (ret == -2)
				{//[大角度]
					if (g_cur_speed > (int)(5 * CM))
					{
						g_cur_speed = (int)(g_cur_speed * 3.0 / 4);
					}
				}
			}
			else
			{
				//[当前方道路有障碍物，速度减小]
				if (g_unsafe_dist <= 1000)
				{//[此时急停]
					g_cur_speed = 0;
				}
				else
				{
					time = ((g_unsafe_dist - 1000) * 2 + 0.0) / g_wp_speed;
					acc = (g_wp_speed - time) / 10;//[每一帧的加速度]
					if (acc < 24)
					{//[设置最低加速度]
						acc = 36;
					}
					g_cur_speed = (int)(g_cur_speed - acc);
					if (g_cur_speed < 0)
					{
						g_cur_speed = 0;
					}
				}
			}
		}
		else if (g_navi_state == ROVING)
		{
			//[2014年越野]
			if (g_roving_switch == 1)
			{
				g_cur_speed = g_cross_speed;
			}
			else
			{
				g_cur_speed = g_cross_speed;//(int)(5 * CM);
			}
		}
		else if (g_navi_state == D_EMERGENCY_STOP)
		{
			//[降速]
			g_wp_speed = (int)(g_real_speed - 5 * CM);
			if (g_wp_speed < 0)
			{
				g_wp_speed = 0;
			}

			if (g_car_ahead_dist > 5000)
			{
				g_cur_speed = g_wp_speed > (int)(15 * CM) ? (int)(15 * CM) : g_wp_speed;
			}
			else if (g_car_ahead_dist <= 5000 && g_car_ahead_dist > 4000)
			{
				g_cur_speed = g_wp_speed > (int)(12 * CM) ? (int)(12 * CM) : g_wp_speed;
			}
			else if (g_car_ahead_dist <= 4000 && g_car_ahead_dist > 3000)
			{
				g_cur_speed = g_wp_speed > (int)(9 * CM) ? (int)(9 * CM) : g_wp_speed;
			}
			else if (g_car_ahead_dist <= 3000 && g_car_ahead_dist > 2000)
			{
				g_cur_speed = g_wp_speed > (int)(6 * CM) ? (int)(6 * CM) : g_wp_speed;
			}
			else if (g_car_ahead_dist <= 2000 && g_car_ahead_dist > 1000)
			{
				g_cur_speed = g_wp_speed > (int)(3 * CM) ? (int)(3 * CM) : g_wp_speed;
			}
			else
			{
				g_cur_speed = 0;
			}
		}
		else if (g_navi_state == S_EMERGENCY_STOP)
		{
			//[降速]
			g_wp_speed = (int)(g_real_speed - 5 * CM);
			if (g_wp_speed < 0)
			{
				g_wp_speed = 0;
			}
			if (g_unsafe_dist > 5000)
			{
				g_cur_speed = g_wp_speed > (int)(40 * CM) ? (int)(40 * CM) : g_wp_speed;
			}
			else if (g_unsafe_dist <= 5000 && g_unsafe_dist > 4000)
			{
				g_cur_speed = g_wp_speed > (int)(30 * CM) ? (int)(30 * CM) : g_wp_speed;
			}
			else if (g_unsafe_dist <= 4000 && g_unsafe_dist > 3000)
			{
				g_cur_speed = g_wp_speed > (int)(20 * CM) ? (int)(20 * CM) : g_wp_speed;
			}
			else if (g_unsafe_dist <= 3000 && g_unsafe_dist > 2000)
			{
				g_cur_speed = g_wp_speed > (int)(10 * CM) ? (int)(10 * CM) : g_wp_speed;
			}
			else if (g_unsafe_dist <= 2000 && g_unsafe_dist > 1000)
			{
				g_cur_speed = g_wp_speed > (int)(6 * CM) ? (int)(6 * CM) : g_wp_speed;
			}
			else
			{
				g_cur_speed = 0;
			}
		}


		if (g_navi_state == TRACE_LANE || g_navi_state == TRACE_IN_NATURAL_ROAD)
		{
			if (g_road_type == 1)
			{//[@@2013年常熟比赛，结构非结构混合道路下弯道降速策略]
				//[根据道路弯曲等级来再次调整速度]
				if (pl_input->fu_pl_data.stop_lines.stop_lines[0].start.x == 1 && \
					pl_input->fu_pl_data.stop_lines.stop_lines[0].start.y == 1 && \
					pl_input->fu_pl_data.stop_lines.stop_lines[0].end.x == 1 && \
					pl_input->fu_pl_data.stop_lines.stop_lines[0].end.y == 1)
				{//[直道]
					g_one_curve_leve_counter++;

					if (g_one_curve_leve_counter >= 3)
					{//[确认3帧]
						g_one_curve_leve_counter = 3;
						g_two_curve_level_counter = 0;
						g_three_curve_level_counter = 0;
						g_four_curve_level_counter = 0;
						g_curent_curve_level = 1;
#ifdef MBUG_OPEN_
						MBUG("Road Curve Level : %d\n", 0);
#endif
					}
				}
				else if(pl_input->fu_pl_data.stop_lines.stop_lines[0].start.x ==2 && \
					pl_input->fu_pl_data.stop_lines.stop_lines[0].start.y == 2 && \
					pl_input->fu_pl_data.stop_lines.stop_lines[0].end.x == 2 && \
					pl_input->fu_pl_data.stop_lines.stop_lines[0].end.y == 2)
				{//[缓弯]
					g_two_curve_level_counter++;

					if (g_two_curve_level_counter >= 3)
					{//[确认3帧]
						g_one_curve_leve_counter = 0;
						g_two_curve_level_counter = 3;
						g_three_curve_level_counter = 0;
						g_four_curve_level_counter = 0;
						g_curent_curve_level = 2;
#ifdef MBUG_OPEN_
						MBUG("Road Curve Level : %d\n", 1);
#endif
					}
				}
				else if(pl_input->fu_pl_data.stop_lines.stop_lines[0].start.x ==3 && \
					pl_input->fu_pl_data.stop_lines.stop_lines[0].start.y == 3 && \
					pl_input->fu_pl_data.stop_lines.stop_lines[0].end.x == 3 && \
					pl_input->fu_pl_data.stop_lines.stop_lines[0].end.y == 3)
				{//[大弯]
					g_three_curve_level_counter++;

					if (g_three_curve_level_counter >= 3)
					{//[确认3帧]
						g_one_curve_leve_counter = 0;
						g_two_curve_level_counter = 0;
						g_three_curve_level_counter = 3;
						g_four_curve_level_counter = 0;
						g_curent_curve_level = 3;
#ifdef MBUG_OPEN_
						MBUG("Road Curve Level : %d\n", 2);
#endif
					}
				}
				else if(pl_input->fu_pl_data.stop_lines.stop_lines[0].start.x ==4 && \
					pl_input->fu_pl_data.stop_lines.stop_lines[0].start.y == 4 && \
					pl_input->fu_pl_data.stop_lines.stop_lines[0].end.x == 4 && \
					pl_input->fu_pl_data.stop_lines.stop_lines[0].end.y == 4)
				{//[急弯]
					g_four_curve_level_counter++;

					if (g_four_curve_level_counter >= 3)
					{//[确认3帧]
						g_one_curve_leve_counter = 0;
						g_two_curve_level_counter = 0;
						g_three_curve_level_counter = 0;
						g_four_curve_level_counter = 3;
						g_curent_curve_level = 4;
#ifdef MBUG_OPEN_
						MBUG("Road Curve Level : %d\n", 3);
#endif
					}
				}

				switch (g_curent_curve_level)
				{
				case 1:
					//[直道不减速]
					break;
				case 2:
					//[小弯]
					g_cur_speed = g_cur_speed > (int)(g_two_curve_level_speed * CM) \
						? (int)(g_two_curve_level_speed * CM) : g_cur_speed;
					break;
				case 3:
					//[急弯]
					g_cur_speed = g_cur_speed > (int)(g_three_curve_level_speed * CM) \
						? (int)(g_three_curve_level_speed * CM) : g_cur_speed;
					break;
				case 4:
					g_cur_speed = g_cur_speed > (int)(g_four_curve_level_speed * CM) \
						? (int)(g_four_curve_level_speed * CM) : g_cur_speed;
					break;
				default:
					break;
				}
			}
		}

	}
	else if (g_stat == S_CROSS_UND)
	{
		g_cur_speed = g_wp_speed;// * g_grid_dist / 4000.0;

		if (g_road_type == 1)
		{//[乡村道路理解减速到15km]
			g_cur_speed = g_cur_speed > (int)(15 * CM) ? (int)(15 * CM) : g_cur_speed;
		}
	}
	else if (g_stat == S_LEFT ||
		g_stat == S_RIGHT ||
		g_stat == S_STRAIGHT ||
		g_stat == S_UTURN)
	{
		if (g_cross_travel_rate < 0)
		{
			g_cur_speed = 0;
		}
		else
		{
			if (g_stat == S_STRAIGHT)
			{
				if (g_road_type == 1)
				{//[乡村道路下使用低速]
					g_wp_speed = (int)(15 * CM);
				}
			}
			//[此处参见工作笔记2012.07.10]
			g_cross_travel_rate = 1;
			double a = (1 - g_cross_travel_rate) * log(1999.0);
			double b = 2 / (1 + exp(a));

			g_wp_speed = (int)(g_wp_speed * b);

			//[直行根据角度控制速度]
			if (g_stat == S_STRAIGHT)
			{
				if (g_cross_last_index == -1)
				{
					g_wp_speed = 0;
				}
				else
				{
					//double angle = MORPHIN_MID_INDEX - abs(MORPHIN_MID_INDEX - g_cross_last_index);
					//g_wp_speed = (int)(angle / MORPHIN_MID_INDEX * ((int)(20 * CM) - (int)(5 * CM)) + (int)(5 * CM));
				}

			}

// 			if (g_real_speed < (int)(10 * CM))
// 			{
// 				g_cur_speed = g_wp_speed > (int)(12 * CM) ? (int)(12 * CM) : g_wp_speed;
// 			}
// 			else if (g_real_speed < (int)(15 * CM))
// 			{
// 				g_cur_speed = g_wp_speed > (int)(17 * CM) ? (int)(17 * CM) : g_wp_speed;
// 			}
// 			else if (g_real_speed < (int)(20 * CM))
// 			{
// 				g_cur_speed = g_wp_speed > (int)(22 * CM) ? (int)(22 * CM) : g_wp_speed;
// 			}
// 			else if (g_real_speed < (int)(25 * CM))
// 			{
// 				g_cur_speed = g_wp_speed > (int)(27 * CM) ? (int)(27 * CM) : g_wp_speed;
// 			}
// 			else if (g_real_speed < (int)(30 * CM))
// 			{
// 				g_cur_speed = g_wp_speed > (int)(32 * CM) ? (int)(32 * CM) : g_wp_speed;
// 			}
// 			else if (g_real_speed < (int)(35 * CM))
// 			{
// 				g_cur_speed = g_wp_speed > (int)(37 * CM) ? (int)(37 * CM) : g_wp_speed;
// 			}
// 			else if (g_real_speed < (int)(40 * CM))
// 			{
// 				g_cur_speed = g_wp_speed > (int)(42 * CM) ? (int)(42 * CM) : g_wp_speed;
// 			}
// 			else
// 			{
				g_cur_speed = g_wp_speed;
	//		}
		}

		g_last_speed = g_cur_speed;

	}
	else if (g_stat == S_TASK_OVER)
	{
		double rate = 0;
		ret = check_path_for_big_angle(rate);
		rate = 1;
		g_wp_speed = (int)(g_wp_speed * rate);

		g_cur_speed = g_wp_speed;

		if (g_unsafe_dist == MAX_VALUE)
		{
			if (g_real_speed < (int)(25 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(32 * CM) ? (int)(32 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(30 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(37 * CM) ? (int)(37 * CM) : g_wp_speed;
			}
			else if (g_real_speed < (int)(35 * CM))
			{
				g_cur_speed = g_wp_speed > (int)(42 * CM) ? (int)(42 * CM) : g_wp_speed;
			}
			else
			{
				g_cur_speed = g_wp_speed;
			}
		}
		else
		{
			//[当前方道路有障碍物，速度减小]
			if (g_unsafe_dist > 5000)
			{
				g_cur_speed = g_wp_speed > (int)(20 * CM) ? (int)(20 * CM) : g_wp_speed;
			}
			else if (g_unsafe_dist <= 5000 && g_unsafe_dist > 4000)
			{
				g_cur_speed = g_wp_speed > (int)(16 * CM) ? (int)(16 * CM) : g_wp_speed;
			}
			else if (g_unsafe_dist <= 4000 && g_unsafe_dist > 3000)
			{
				g_cur_speed = g_wp_speed > (int)(12 * CM) ? (int)(12 * CM) : g_wp_speed;
			}
			else if (g_unsafe_dist <= 3000 && g_unsafe_dist > 2000)
			{
				g_cur_speed = g_wp_speed > (int)(8 * CM) ? (int)(8 * CM) : g_wp_speed;
			}
			else if (g_unsafe_dist <= 2000 && g_unsafe_dist > 1000)
			{
				g_cur_speed = g_wp_speed > (int)(4 * CM) ? (int)(4 * CM) : g_wp_speed;
			}
			else
			{
				g_cur_speed = 0;
			}
		}
	}
	else if (g_stat == S_PARKING)
	{
		g_cur_speed = g_cross_speed;
	}
	else
	{
		g_cur_speed = 0;
	}

	//[限速]
	if (g_speed_limit_sign == 1)
	{
		if (g_stat == S_ROAD_NAV || \
			g_stat == S_CROSS_UND || \
			g_stat == S_STRAIGHT)
		{
			g_wp_speed = g_speed_limi_pl;
			g_cur_speed = g_cur_speed > g_wp_speed ? g_wp_speed : g_cur_speed;
		}
	}

	if (g_fusion_road_length < 3200 && g_fusion_road_length > 2400)
	{//[@@2013常熟城市比赛弯道减速]
		g_cur_speed = g_cur_speed > (int)(15 * CM) ? (int)(15 * CM) : g_cur_speed;
	}

	if (g_is_s_obs == 1)
	{//[S弯低速]
		//g_cur_speed = (int)(3 * CM);
		if (g_s_obs_speed_down == 0)
		{
			g_cur_speed = (int)(6 * CM);
		}
		else if (g_s_obs_speed_down == 1)
		{
			g_cur_speed = (int)(5 * CM);
		}
		else
		{
			g_cur_speed = (int)(4 * CM);
		}

		if (g_s_obs_crash_flag == 1)
		{
			g_cur_speed = (int)(3 * CM);
		}
	}

	if (g_is_stopping == 1)
	{
		g_cur_speed = 0;
	}
}

static int g_left_light_counter = 0;
static int g_right_light_counter = 0;
//***********************************************************************************************
//                                zgccmax 2012.Mar.05
//void set_pl_result(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data)
//param:    PL_FUNC_INPUT *pl_input			规划输入
//          PL_CS_DATA *pl_cs_data			底层控制输出
//return:   void
//discribe: 填写规划输出结果
//***********************************************************************************************
void set_pl_result(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data)
{
	memset(pl_cs_data, 0, sizeof(PL_CS_DATA));


	//[灯光]
	if (g_stat == S_ROAD_NAV)
	{
		if (g_navi_state == TAKEOVER_FROM_LEFT || g_navi_state == CHANGE_TO_LEFT_LANE)
		{
			pl_cs_data->sys_command = 0x04;
			g_left_light_counter = 20;
			g_right_light_counter = 0;
		}
		else if (g_navi_state == TAKEOVER_FROM_RIGHT || g_navi_state == CHANGE_TO_RIGHT_LANE)
		{
			pl_cs_data->sys_command = 0x08;
			g_left_light_counter = 0;
			g_right_light_counter = 20;
		}
		else
		{
			if (g_left_light_counter > 0)
			{
				pl_cs_data->sys_command = 0x04;
				g_left_light_counter--;
			}
			else if (g_right_light_counter > 0)
			{
				pl_cs_data->sys_command = 0x08;
				g_right_light_counter--;
			}
			else
			{
				pl_cs_data->sys_command = 0x00;
			}
		}
	}
	else if (g_stat == S_LEFT || g_stat == S_UTURN)
	{
		pl_cs_data->sys_command = 0x04;
		g_left_light_counter = 0;
		g_right_light_counter = 0;
	}
	else if (g_stat == S_RIGHT)
	{
		pl_cs_data->sys_command = 0x08;
		g_left_light_counter = 0;
		g_right_light_counter = 0;
	}
	else
	{
		pl_cs_data->sys_command = 0x00;
		g_left_light_counter = 0;
		g_right_light_counter = 0;
	}

	if (g_is_s_obs == 1)
	{
		pl_cs_data->sys_command = 0x00;
	}
	
	if (g_cur_speed == 0 && g_uturn_stop_once == 0)
	{//[速度为零，停车]
		pl_cs_data->id = g_frame_id;
		pl_cs_data->isok = 1;
		pl_cs_data->number_of_effective_points = g_long_line_num;
		memset(g_mid_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		memcpy(pl_cs_data->path, g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		pl_cs_data->speed = g_cur_speed;
		memcpy(&pl_cs_data->state, &pl_input->state, sizeof(STATE));
		pl_cs_data->sys_command = 0;
	}
	else
	{
		if (g_fidelity == 1)
		{
			pl_cs_data->id = g_frame_id;
			pl_cs_data->isok = 1;
			pl_cs_data->number_of_effective_points = g_long_line_num;
			memcpy(pl_cs_data->path, g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			pl_cs_data->speed = g_cur_speed;
			memcpy(&pl_cs_data->state, &pl_input->state, sizeof(STATE));
		}
		else
		{
			pl_cs_data->id = g_frame_id;
			pl_cs_data->isok = 1;
			pl_cs_data->number_of_effective_points = g_long_line_num;
			memset(g_mid_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			memcpy(pl_cs_data->path, g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			pl_cs_data->speed = 0;
			memcpy(&pl_cs_data->state, &pl_input->state, sizeof(STATE));
			pl_cs_data->sys_command = 0;
		}
	}
}

/*==================================================================
 * 函数名  ：	void keep_natural_road(PL_FUNC_INPUT *pl_input)
 * 功能    ：	 获取自然道边数据，保存到规划内部结构
 * 输入参数：	PL_FUNC_INPUT *pl_input		环境数据输入
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void keep_natural_road(PL_FUNC_INPUT *pl_input)
{
	int i = 0;
	int j = 0;
	memset(&g_natural_boundary, 0, sizeof(NATURAL_BOUNDARY));

	if (g_nature_road_switch == 0)
	{
		return;
	}

	COOR2 temp_line[200];
	int temp_line_num = 0;
	double step;

	if (pl_input->fu_pl_data.boundary.l_fidelity >= 0)
	{
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			g_natural_boundary.l_boundary[i].x = pl_input->fu_pl_data.boundary.l_boundary[i].x;
			g_natural_boundary.l_boundary[i].y = pl_input->fu_pl_data.boundary.l_boundary[i].y;
		}

		g_natural_boundary.l_nums = get_effective_points_num(g_natural_boundary.l_boundary);

		if (g_natural_boundary.l_nums >= 2)
		{
			for (i=0;i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;i++)
			{
				if (g_natural_boundary.l_boundary[i].y >= 400)
					break;
			}
			if (g_roving_switch == 1)
			{//[开启越野模式只使用到20m]
				for (j=g_natural_boundary.l_nums - 1;j>=0;j--)
				{
					if (g_natural_boundary.l_boundary[j].y <= 2000)
						break;
				}
			}
			else
				j = g_natural_boundary.l_nums - 1;

			if (i>=j)
			{
				memset(g_natural_boundary.l_boundary, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
				g_natural_boundary.l_nums = 0;
				g_natural_boundary.l_fidelity = 0;
			}
			else
			{
				memmove(g_natural_boundary.l_boundary, g_natural_boundary.l_boundary + i, (g_natural_boundary.l_nums - i) * sizeof(COOR2));
				g_natural_boundary.l_nums = g_natural_boundary.l_nums - i - (g_natural_boundary.l_nums - 1 - j);

				memset(temp_line, 0, 200 * sizeof(COOR2));
				step = (g_natural_boundary.l_boundary[g_natural_boundary.l_nums - 1].y - g_natural_boundary.l_boundary[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
				line_fitting(g_natural_boundary.l_boundary, g_natural_boundary.l_nums, temp_line, temp_line_num, (int)step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
				memcpy(g_natural_boundary.l_boundary, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
				g_natural_boundary.l_nums = temp_line_num;

				g_natural_boundary.l_fidelity = pl_input->fu_pl_data.boundary.l_fidelity;
			}
			
		}
		else
		{
			memset(g_natural_boundary.l_boundary, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			g_natural_boundary.l_nums = 0;
			g_natural_boundary.l_fidelity = 0;
		}
	}
	
	if (pl_input->fu_pl_data.boundary.r_fidelity >= 0)
	{
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			g_natural_boundary.r_boundary[i].x = pl_input->fu_pl_data.boundary.r_boundary[i].x;
			g_natural_boundary.r_boundary[i].y = pl_input->fu_pl_data.boundary.r_boundary[i].y;
		}

		g_natural_boundary.r_nums = get_effective_points_num(g_natural_boundary.r_boundary);

		if (g_natural_boundary.r_nums >= 2)
		{
			for (i=0;i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;i++)
			{
				if (g_natural_boundary.r_boundary[i].y >= 400)
					break;
			}
			if (g_roving_switch == 1)
			{//[越野模式下只使用到20m]
				for (j=g_natural_boundary.r_nums - 1;j>=0;j--)
				{
					if (g_natural_boundary.r_boundary[j].y <= 2000)
						break;
				}
			}
			else
				j = g_natural_boundary.r_nums - 1;

			if (i>=j)
			{
				memset(g_natural_boundary.r_boundary, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
				g_natural_boundary.r_nums = 0;
				g_natural_boundary.r_fidelity = 0;
			}
			else
			{
				memmove(g_natural_boundary.r_boundary, g_natural_boundary.r_boundary + i, (g_natural_boundary.r_nums - i) * sizeof(COOR2));
				g_natural_boundary.r_nums = g_natural_boundary.r_nums - i - (g_natural_boundary.r_nums - 1 - j);

				memset(temp_line, 0, 200 * sizeof(COOR2));
				step = (g_natural_boundary.r_boundary[g_natural_boundary.r_nums - 1].y - g_natural_boundary.r_boundary[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
				line_fitting(g_natural_boundary.r_boundary, g_natural_boundary.r_nums, temp_line, temp_line_num, (int)step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
				memcpy(g_natural_boundary.r_boundary, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
				g_natural_boundary.r_nums = temp_line_num;

				g_natural_boundary.r_fidelity = pl_input->fu_pl_data.boundary.r_fidelity;
			}

		}
		else
		{
			memset(g_natural_boundary.r_boundary, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			g_natural_boundary.r_nums = 0;
			g_natural_boundary.r_fidelity = 0;
		}
	}

#ifdef MBUG_OPEN_
	MBUG("left natural boundary : ");
	for (i=0;i<g_natural_boundary.l_nums;i++)
		MBUG("(%d, %d), ", g_natural_boundary.l_boundary[i].x, g_natural_boundary.l_boundary[i].y);
	MBUG("\n");
	
	MBUG("right natural boundary : ");
	for (i=0;i<g_natural_boundary.r_nums;i++)
		MBUG("(%d, %d), ", g_natural_boundary.r_boundary[i].x, g_natural_boundary.r_boundary[i].y);
	MBUG("\n");
#endif
}

/*==================================================================
 * 函数名  ：	int get_emergency_line(PL_FUNC_INPUT *pl_input)
 * 功能    ：	紧急停车状态下检测可通行性。
 * 输入参数：	PL_FUNC_INPUT *pl_input			环境数据输入
 * 输出参数：	
 * 返回值  ：	int   0  恢复可通行  -1  仍然不可通行
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int get_emergency_line(PL_FUNC_INPUT *pl_input)
{
	int ret = 0;

	int i;
	COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int left_line_num, right_line_num;
	left_line_num = 0;
	right_line_num = 0;

	memset(left_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	memset(right_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));


	if (g_multi_lane.lane_line[1].valid_num_points >= 2 && 
		g_multi_lane.lane_line[2].valid_num_points >= 2)
	{
		memcpy(left_line, g_multi_lane.lane_line[1].line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		memcpy(right_line, g_multi_lane.lane_line[2].line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		left_line_num = g_multi_lane.lane_line[1].valid_num_points;
		right_line_num = g_multi_lane.lane_line[2].valid_num_points;
	}
	else if (g_multi_lane.lane_line[1].valid_num_points >= 2)
	{
		memcpy(left_line, g_multi_lane.lane_line[1].line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		left_line_num = g_multi_lane.lane_line[1].valid_num_points;
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			right_line[i].x = left_line[i].x + 375;
			right_line[i].y = left_line[i].y;
		}
		right_line_num = left_line_num;
	}
	else if (g_multi_lane.lane_line[2].valid_num_points >= 2)
	{
		memcpy(right_line, g_multi_lane.lane_line[2].line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		right_line_num = g_multi_lane.lane_line[2].valid_num_points;
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			left_line[i].x = right_line[i].x - 375;
			left_line[i].y = right_line[i].y;
		}
		left_line_num = right_line_num;
	}
	else
	{
		int y_step = 0;

		y_step = (int)((FU_ROAD_LENGTH - 400.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1));
		for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			left_line[i].x = -187;
			left_line[i].y = 400 + i * y_step;
		}
		left_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;

		y_step = (int)((FU_ROAD_LENGTH - 400.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1));
		for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			right_line[i].x = 187;
			right_line[i].y = 400 + i * y_step;
		}
		right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
	}

//	int l_unsafe_dist = MAX_VALUE;
//	l_unsafe_dist = g_unsafe_dist;
	

	if (left_line_num > 1 && right_line_num > 1)
	{
		ret = check_the_lane(left_line, left_line_num, right_line, right_line_num);
	}
	else
		ret = -1;

	DYN_OBSTACLE_AREA dyn_obstacle_area;
	COOR2 center;
	SPD2D speed;
	//[检测动态障碍物]
	for (i = 0; i<pl_input->fu_pl_data.gridmap.d_areas.valid_area_num; i++)
	{
		memcpy(&dyn_obstacle_area, &pl_input->fu_pl_data.gridmap.d_areas.areas[i], sizeof(DYN_OBSTACLE_AREA));

		center.x = (dyn_obstacle_area.mask.coor2[0].x + dyn_obstacle_area.mask.coor2[2].x) / 2;
		center.y = (dyn_obstacle_area.mask.coor2[0].y + dyn_obstacle_area.mask.coor2[2].y) / 2;
		speed = dyn_obstacle_area.speed;

		if (center.y > 6000 || center.y < -1500)
		{//[60m以外的不管]
			continue;
		}

		if (left_line_num >= 2 && right_line_num >= 2)
		{
			int l_x = 0;
			int r_x = 0;
			get_x_coord(center.y, left_line, left_line_num, &l_x);
			get_x_coord(center.y, right_line, right_line_num, &r_x);
			if (center.x < l_x || center.x > r_x)
			{//[只看道路内的]
				continue;
			}

			if ((center.y > 1000 && speed.y > g_real_speed) || (center.y < -1500 && speed.y < g_real_speed))
			{//[10m之外，车速比我方高不管， 15m之后，比我方速度小不管]
				continue;;
			}
			else
			{//[有动态障碍物，急停]
//				l_unsafe_dist = 0;
			}
		}
	}
	return ret;
}

//***********************************************************************************************
//                                zgccmax 2012.Mar.05
//void fix_the_lost_structual_line(PL_FUNC_INPUT *pl_input)
//param:    PL_FUNC_INPUT *pl_input			规划输入
//return:   void
//discribe: 补出结构化道路。由于当前车道可能检测不稳定，只能检测到外面的车道，根据道路宽约束补出
//			当前车道。
//***********************************************************************************************
void fix_the_lost_structual_line(PL_FUNC_INPUT *pl_input)
{
	int i;
	COOR2 temp_line[200];
	int temp_line_num = 0;
	COOR2 mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int left_line_num = 0;
	COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int right_line_num = 0;
	int step;

	memset(left_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	memset(right_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	left_line_num = 0;
	right_line_num = 0;
	for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
	{
		left_line[i].x = pl_input->fu_pl_data.lines.l_edges[0].line[i].x;
		left_line[i].y = pl_input->fu_pl_data.lines.l_edges[0].line[i].y;
	}
	left_line_num = pl_input->fu_pl_data.lines.l_edges[0].valid_num_points;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;

	step = (int)(left_line[left_line_num - 1].y - left_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

	line_fitting(left_line, left_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memcpy(left_line, temp_line, temp_line_num * sizeof(COOR2));
	left_line_num = temp_line_num;

	for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
	{
		right_line[i].x = pl_input->fu_pl_data.lines.r_edges[0].line[i].x;
		right_line[i].y = pl_input->fu_pl_data.lines.r_edges[0].line[i].y;
	}
	right_line_num = pl_input->fu_pl_data.lines.r_edges[0].valid_num_points;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;

	step = (int)(right_line[right_line_num - 1].y - right_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

	line_fitting(right_line, right_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memcpy(right_line, temp_line, temp_line_num * sizeof(COOR2));
	right_line_num = temp_line_num;
#ifdef MBUG_OPEN_
	MBUG("get road wide : 3\n");
	for (i=0;i<left_line_num;i++)
	{
		MBUG("(%d, %d) ", left_line[i].x, left_line[i].y);
	}
	MBUG("\n");
	for (i=0;i<right_line_num;i++)
	{
		MBUG("(%d, %d) ", right_line[i].x, right_line[i].y);
	}
	MBUG("\n");
#endif

	int width = get_road_wide(left_line, left_line_num, right_line, right_line_num);
	if (width > 450)
	{
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			mid_line[i].x = (left_line[i].x + right_line[i].x) / 2;
			mid_line[i].y = (left_line[i].y + right_line[i].y) / 2;
		}

		if (mid_line[0].x <= 0)
		{
			memcpy(pl_input->fu_pl_data.lines.l_edges + 1, pl_input->fu_pl_data.lines.l_edges, 2 * sizeof(EDGE));
			if (pl_input->fu_pl_data.lines.valid_num_in_l_wing <= 2)
				pl_input->fu_pl_data.lines.valid_num_in_l_wing++;

			for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				pl_input->fu_pl_data.lines.l_edges[0].line[i].x = right_line[i].x - 375;
				pl_input->fu_pl_data.lines.l_edges[0].line[i].y = right_line[i].y;
			}
			pl_input->fu_pl_data.lines.l_edges[0].valid_num_points = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
		}
		else
		{
			memcpy(pl_input->fu_pl_data.lines.r_edges + 1, pl_input->fu_pl_data.lines.r_edges, 2 * sizeof(EDGE));
			if (pl_input->fu_pl_data.lines.valid_num_in_r_wing <= 2)
				pl_input->fu_pl_data.lines.valid_num_in_r_wing++;

			for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				pl_input->fu_pl_data.lines.r_edges[0].line[i].x = left_line[i].x + 375;
				pl_input->fu_pl_data.lines.r_edges[0].line[i].y = left_line[i].y;
			}
			pl_input->fu_pl_data.lines.r_edges[0].valid_num_points = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
		}
	}
}


//[得到融合结构道路的长度]
void get_fusion_structure_road_length()
{
	int i = 0;
	int max_length = -1;
	for (i=0; i<4; i++)
	{
		int pts_num = g_multi_lane.lane_line[i].valid_num_points;
		if (pts_num > 0)
		{
			if (g_multi_lane.lane_line[i].line[pts_num - 1].y > max_length)
			{
				max_length = g_multi_lane.lane_line[i].line[pts_num - 1].y;
			}
		}
	}

	if (max_length != -1)
	{
		g_fusion_structure_road_length = max_length;
	}
	else if (max_length < 2000)
	{
		g_fusion_structure_road_length = 0;

	}
}

//[与get_fusion_structure_road_length()进行区别，分别是乡村道路和结构化道路使用]
void get_fusion_road_length()
{
	int i = 0;
	int max_length = -1;
	for (i=0; i<4; i++)
	{
		int pts_num = g_multi_lane.lane_line[i].valid_num_points;
		if (pts_num > 0)
		{
			if (g_multi_lane.lane_line[i].line[pts_num - 1].y > max_length)
			{
				max_length = g_multi_lane.lane_line[i].line[pts_num - 1].y;
			}
		}
	}

	if (max_length != -1)
	{
		g_fusion_road_length = max_length;
	}
	else if (max_length < 2000)
	{
		g_fusion_road_length = 0;

	}
}

COOR2 g_cur_road_pt;
int g_cross_und_speed_flag = 0;//[路口路点读取一次，防止总控撞点后切到下一个路点]


typedef struct
{
	int lwheel_speedmeter;//[左轮里程计]
	int rwheel_speedmeter;//[右轮里程计]
}SPEED_METER;

SPEED_METER g_last_speedmeter;
SPEED_METER g_cur_speedmeter;
int means_speed[5];

//[掉头测试]
bool bTurnRoundFlag = false;
CTurnRound *turnround = NULL;

/*==================================================================
 * 函数名  ：	void set_stop_plan(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, int mode)
 * 功能    ：	设置停车指令
 * 输入参数：	int mode		0  普通停车		1  开启双跳停车
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void set_stop_plan(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, int mode)
{
	for (int ii=0;ii<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;ii++)
	{
		pl_cs_data->path[ii].x = 0;
		pl_cs_data->path[ii].y = ii * 100;
	}
	pl_cs_data->id = g_frame_id;
	pl_cs_data->isok = 1;
	pl_cs_data->number_of_effective_points = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
	switch (mode)
	{
	case 0:
		pl_cs_data->sys_command = 0;
		break;
	case 1:
		pl_cs_data->sys_command = SYS_COMMAND_LEFT | SYS_COMMAND_RIGHT;
		break;
	}
	
	pl_cs_data->speed = (int)(0 * CM);
	memcpy(&pl_cs_data->state, &pl_input->state, sizeof(STATE));
}

/*==================================================================
 * 函数名  ：	void get_speed_from_odo(PL_FUNC_INPUT *pl_input)
 * 功能    ：	使用里程计数据计算实时速度
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int get_speed_from_odo(PL_FUNC_INPUT *pl_input)
{
	int ret = 0;
	//[因为2015年将原来的signed short 改成了unsigned short 为了方便改代码代码这里做转换]
	INT16 left_odo = pl_input->odo_info.left;
	INT16 right_odo = pl_input->odo_info.right;
	//[通过里程计计算速度]
	if (odo_init_flag == 0)
	{
		g_last_speedmeter.lwheel_speedmeter = abs(left_odo);//pl_input->state.pos.acc.x;
		g_last_speedmeter.rwheel_speedmeter = abs(right_odo);//pl_input->state.pos.acc.y;
		g_cur_speedmeter.lwheel_speedmeter = abs(left_odo);//pl_input->state.pos.acc.x;
		g_cur_speedmeter.rwheel_speedmeter = abs(right_odo);//pl_input->state.pos.acc.y;
		memset(means_speed, 0, sizeof(int) * 5);
		odo_init_flag = 1;
	}

	g_cur_speedmeter.lwheel_speedmeter = abs(left_odo);
	g_cur_speedmeter.rwheel_speedmeter = abs(right_odo);

	//[猎豹参数 0.05028  巡洋舰参数 0.0513149  每单位脉冲的距离 单位m]
	int cal_spd = (int)(((abs(g_cur_speedmeter.rwheel_speedmeter - g_last_speedmeter.rwheel_speedmeter) +\
		+ 0.0 + abs(g_cur_speedmeter.lwheel_speedmeter - g_last_speedmeter.lwheel_speedmeter)) / 2) * 0.0513149 * 100 * 10);

	memmove(means_speed+1, means_speed, sizeof(int) * 4);
	means_speed[0] = cal_spd;

	//[中值滤波]
	int means_speed2[5];
	memcpy(means_speed2, means_speed, sizeof(int)*5);
	int tmp;
	for (int i=0;i<4; i++)
	{
		for (int j=i+1;j<5;j++)
		{
			if (means_speed2[i] > means_speed2[j])
			{
				tmp = means_speed2[i];
				means_speed2[i] = means_speed2[j];
				means_speed2[j] = tmp;
			}
		}
	}

	if (means_speed2[2] > 555)//[20km/h]
	{
		if (abs(abs(pl_input->state.pos.spd) - abs(means_speed2[2])) > 277)
		{
#ifdef MBUG_OPEN_
			MBUG("pl_input->state.pos.spd : %d\n", pl_input->state.pos.spd);
			MBUG("means_speed2[2] : %d\n", means_speed2[2]);
#endif
			ret = -1;
			return ret;
		}
	}
	g_real_speed = means_speed2[2];


	g_last_speedmeter.lwheel_speedmeter = g_cur_speedmeter.lwheel_speedmeter;
	g_last_speedmeter.rwheel_speedmeter = g_cur_speedmeter.rwheel_speedmeter;

	return ret;
}


//[生成在全局任务点附近的点]
/*==================================================================
 * 函数名  ：	COOR2 get_global_task_area_pt(PL_FUNC_INPUT *pl_input, COOR2 cur_road_pt)
 * 功能    ：	根据状态生成相应的局部目标点
 * 输入参数：	COOR2 cur_road_pt		当前路点
 * 输出参数：	COOR2		生成的局部目标点
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
/*
COOR2 get_global_task_area_pt(PL_FUNC_INPUT *pl_input, COOR2 cur_road_pt)
{
	COOR2 global_task_area_pt;
	global_task_area_pt.x = 0;
	global_task_area_pt.y = 0;

	if ((g_global_task_forking_adjust_pt_flag > 0 || \
		pl_input->gp_info.tps[0].type == 3) && \
		g_global_task_adjust_area_flag == TRACE_LOCAL_PT)
	{//[若不是岔路口状态，且是跟踪局部目标点情况下，直接返回当前路点]
		global_task_area_pt = cur_road_pt;
		return global_task_area_pt;
	}
	else if (g_global_task_adjust_area_flag == GO_TO_GLOBAL_TASK_PT)
	{//[全局路点有效范围之内，有效距离参数g_global_task_pt_valid_dist]
		
		global_task_area_pt = cur_road_pt;

		if (g_global_task_forking_adjust_pt_flag == 1 || \
			g_global_task_forking_adjust_pt_flag == 3 || \
			g_global_task_forking_adjust_pt_flag == 5)
		{//[岔路口，虚拟点]
			return global_task_area_pt;
		}

		if (pl_input->gp_info.tps[1].type == 3)
			return global_task_area_pt;

		//[判断车方向和全局任务点的距离]
		if (abs(cur_road_pt.x) <= g_global_task_pt_area_dist)
		{
			global_task_area_pt.x = 0;
			global_task_area_pt.y = cur_road_pt.y;
		}
		else
		{//[预瞄以全局路点的为中心的一个圆区域]
			if (cur_road_pt.x < 0)
			{
				global_task_area_pt.x = cur_road_pt.x + g_global_task_pt_area_dist;
				global_task_area_pt.y = cur_road_pt.y;
			}
			else
			{
				global_task_area_pt.x = cur_road_pt.x - g_global_task_pt_area_dist;
				global_task_area_pt.y = cur_road_pt.y;
			}
		}
		
	}
	else if (g_global_task_adjust_area_flag == TURN_TO_NEXT_GLOBAL_TASK_PT)
 	{//[离当前全局目标点一个合适的距离，判断与下一个全局目标点的角度，若角度偏大，生成一个虚拟点进行方向调整]
		int cur_yaw = g_yaw;
		int delta_yaw = cur_yaw - g_global_task_adjust_goal_yaw;

		if (delta_yaw <= -180)
			delta_yaw = delta_yaw + 360;
		else if (delta_yaw >= 180)
			delta_yaw = 360 - delta_yaw;

		if (delta_yaw < 0)
		{//[下一个全局目标点在左侧]
			global_task_area_pt.x = -800;
			global_task_area_pt.y = 800;
		}
		else if (delta_yaw > 0)
		{//[下一个全局目标点在右侧]
			global_task_area_pt.x = 800;
			global_task_area_pt.y = 800;
		}
	}

	return global_task_area_pt;
}
*/

//[用滑动窗口对局部点进行平滑]
#define LOCAL_PT_FILTER_WIN 15
static COOR2 local_pt_filter_win[LOCAL_PT_FILTER_WIN];
static int local_pt_filter_num = 0;
static int local_pt_filter_cur_idx = 0;
static COOR2 last_local_pt = {0, 0};
/*==================================================================
 * 函数名  ：	COOR2 get_cross_country_local_pt(PL_FUNC_INPUT *pl_input, COOR2 cur_road_pt)
 * 功能    ：	越野环境下局部目标点生成
 * 输入参数：	PL_FUNC_INPUT *pl_input		规划输入
				COOR2 cur_road_pt			当前全局路点(已转到惯导坐标系)
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
COOR2 get_cross_country_local_pt(PL_FUNC_INPUT *pl_input, COOR2 cur_road_pt)
{//[输入保证cur_road_pt.y != 0]
	COOR2 cur_pt;
	COOR2 org_pt;
	org_pt.x = 0;
	org_pt.y = 0;

	cur_pt = cur_road_pt;
	if (dist_point(&org_pt, &cur_road_pt) <= 1500)
	{//[直奔目标点，状态清零]
		memset(local_pt_filter_win, 0, sizeof(COOR2) * LOCAL_PT_FILTER_WIN);
		local_pt_filter_num = 0;
		local_pt_filter_cur_idx = 0;
		memset(&last_local_pt, 0, sizeof(COOR2));

		return cur_road_pt;
	}
	else
	{
// 		if (g_geo_guide_flag == 1)
// 		{
// 			COOR2 temp_pt;
// 			temp_pt.x = g_geo_guide_gls[g_geo_guide_cur_index].x;
// 			temp_pt.y = g_geo_guide_gls[g_geo_guide_cur_index].y;
// 			coor2_e2v(&pl_input->state.pos, &temp_pt, &cur_pt);
// 		}
	}

	return cur_pt;
}





















int avoid_lateral_moving_obs_speed_plan(int speed)
{
	if (g_lateral_dyn_obs_num == 0)
		return speed;

	for (int i = 0; i < g_lateral_dyn_obs_num; i++)
	{//[保证给定一个障碍物]
		if (g_lateral_dyn_obs[i].nearest_pt.y >= 800)
			speed = speed >(int)(5 * CM) ? (int)(5 * CM) : speed;
		else
		{
			if (g_lateral_dyn_obs[i].coor2[2].x >= -200)
				speed = 0;		
		}
	}

	return speed;
}










































































void set_roving_effective_pt_num()
{
	for (int i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;i++)
	{
		if (g_mid_line[i].y > 2000)
		{
			g_long_line_num = i;
			break;
		}


		int x = g_mid_line[i].x / GRID_LEN_PER_CELL + g_grid_center.x;
		int y = g_mid_line[i].y / GRID_LEN_PER_CELL + g_grid_center.y;
		if (g_closed_grid_map[y][x] > 0)
		{
			g_long_line_num = i;
			break;
		}
	}
}


//[打包数据，发送给比亚迪车子]
union UN_PL_CS_DATA
{
	PL_CS_DATA name;
	unsigned char c[260];
} un_pl_cs_data;

void pack_pl_cs_data(PL_CS_DATA *pl_cs_data)
{
	unsigned char buff[260];//[sizeof(PL_CS_DATA)]
	memset(buff, 0, sizeof(PL_CS_DATA));

	memcpy(&un_pl_cs_data.name, pl_cs_data, sizeof(PL_CS_DATA));
	
	unsigned char sum = 0;
	for (int i = 0; i < 244; i++)
		sum += un_pl_cs_data.c[i];

	un_pl_cs_data.c[257] = sum;
	un_pl_cs_data.c[258] = 0x0a;
	un_pl_cs_data.c[259] = 0x24;

	memcpy(pl_cs_data, &un_pl_cs_data, sizeof(PL_CS_DATA));
}


void process_pl_cs_data(PL_CS_DATA *pl_cs_data)
{
	switch (g_car_identity)
	{
	case BYD:
		pack_pl_cs_data(pl_cs_data);
		break;
	default:
		break;
	}
}














































/*
void global_task_pt_analyze(PL_FUNC_INPUT *pl_input)
{
	g_trace_global_task_pt_flag = 0;
	COOR2 pt1;
	COOR2 pt2;
	COOR2 v_pt1;
	COOR2 v_pt2;

	//[1、两个任务点过近，继续保持任务点跟踪]
	if (pl_input->gp_info.tps[0].x != 0 || pl_input->gp_info.tps[0].y != 0)
	{
		pt1.x = pl_input->gp_info.tps[0].x;
		pt1.y = pl_input->gp_info.tps[0].y;
		pt2.x = pl_input->gp_info.tps[1].x;
		pt2.y = pl_input->gp_info.tps[1].y;

		coor2_e2v(&pl_input->state.pos, &pt1, &v_pt1);
		coor2_e2v(&pl_input->state.pos, &pt2, &v_pt2);
		double dist = dist_point(&v_pt1, &v_pt2);
		if (dist < g_trace_global_task_pt_valid_dist)
		{
			g_trace_global_task_pt_flag = 1;
			return;
		}
	}

	//[2、进入下一个任务点范围，进行跟踪]
	if (pl_input->gp_info.tps[2].x != 0 || pl_input->gp_info.tps[2].y != 0)
	{
		pt1.x = pl_input->gp_info.tps[1].x;
		pt1.y = pl_input->gp_info.tps[1].y;
		pt2.x = pl_input->gp_info.tps[2].x;
		pt2.y = pl_input->gp_info.tps[2].y;

		double dist;
		coor2_e2v(&pl_input->state.pos, &pt1, &v_pt1);
		dist = dist_point(&g_vehicle_org_pt, &v_pt1);
		if (dist < g_trace_global_task_pt_valid_dist)
		{
			coor2_e2v(&pl_input->state.pos, &pt2, &v_pt2);
			double dist = dist_point(&v_pt1, &v_pt2);
			if (dist < g_trace_global_task_pt_valid_dist)
			{
				g_trace_global_task_pt_flag = 1;
				return;
			}
		}
		
	}
}
*/
//[mode 0 默认90度方向  1  使用local_goal_pt的方向]
static double g_org_LMS_last_angle = 90;
void global_task_pt_analyze(PL_FUNC_INPUT *pl_input)
{
	if (g_stat == S_ROAD_NAV && \
		(g_navi_state == TRACE_LANE || \
		g_navi_state == ROVING))
	{
		//[1、计算航向偏角]
		COOR2 pt1;
		COOR2 pt2;
		pt1.x = pl_input->gp_info.tps[1].x;
		pt1.y = pl_input->gp_info.tps[1].y;
		pt2.x = pl_input->state.pos.com_coord.x;
		pt2.y = pl_input->state.pos.com_coord.y;
		pt1.x = pt1.x - pt2.x;
		pt1.y = pt1.y - pt2.y;

		int lead_yaw = (int)(atan2(pt1.y, pt1.x) * 180 / PI);
		//[2、转换到正北系下]
		lead_yaw -= 90;
		if (lead_yaw < -180)
			lead_yaw = 360 + lead_yaw;
		//[3、计算偏差]
		int cur_yaw = g_yaw;
		int delta_yaw = cur_yaw - lead_yaw;

		if (delta_yaw <= -180)
			delta_yaw = delta_yaw + 360;
		else if (delta_yaw >= 180)
			delta_yaw = 360 - delta_yaw;
		if (g_trace_global_task_pt_flag == 0)
		{
			if (abs(delta_yaw) >= 80)
			{
				pt1.x = pl_input->gp_info.tps[1].x;
				pt1.y = pl_input->gp_info.tps[1].y;
				coor2_e2v(&pl_input->state.pos, &pt1, &pt2);
				if (pt2.x > 0)
					g_trace_global_task_pt_flag = 1;//[右转]
				else
					g_trace_global_task_pt_flag = 2;//[左转]
			}
			else if (g_navi_state == TRACE_LANE)
			{
				g_org_LMS_last_angle = 90;
			}
		}
		else
		{
			if (abs(delta_yaw) <= 30)
			{
				g_org_LMS_last_angle = 90;
				g_trace_global_task_pt_flag = 0;
			}
		}
	}
	else
	{
		g_trace_global_task_pt_flag = 0;
		g_org_LMS_last_angle = 90;
	}

#ifdef MBUG_OPEN_
	MBUG("trace to global task pt : %d\n", 1);
#endif
}

//[底层异常标志]
static int g_speed_feedback_error_flag = 0;


void set_road_type(int val)
{
	g_road_type = val;
	switch (g_road_type)
	{
	case 0:
		g_navi_state = TRACE_LANE;
		break;
	case 1:
		g_navi_state = TRACE_IN_NATURAL_ROAD;
		break;
	default:
		break;
	}
	
}

void set_db_flag(PL_CS_DATA *pl_cs_data)
{
	pl_cs_data->ref_pts[0].x = 404;
	pl_cs_data->ref_pts[0].y = 404;
}

//[用栅格地图模拟单线]
#define LMS_ANGLE_NUM 361
#define LMS_POINT_NUM 200
#define LMS_MODIFY_Y 0
#define LMS_CV 3
#define LMS_CV_TURNING 7

#define LMS_LEFT_ANGLE_1 140 * 2
#define LMS_LEFT_ANGLE_2 105 * 2
#define LMS_RIGHT_ANGLE_1 40 * 2
#define LMS_RIGHT_ANGLE_2 75 * 2
#define LMS_LATERAL_Y 270		//[车辆侧方检测并行障碍物所看到的Y方向距离 单位cm]
#define LMS_LATERAL_X 130
#define LMS_DIST_THRESHOLD 60	//[点聚类阈值]
#define LMS_THROUGHT_DIST_THRESHOLD 90	//[区域聚类阈值]

/****/
static int ORG_VLMS[LMS_ANGLE_NUM];
static int ORG_VLMS_TURNING[LMS_ANGLE_NUM];
static COOR2 ORG_LMS_POINT[LMS_ANGLE_NUM][LMS_POINT_NUM];
LMS_AREA g_org_lms_area[LMS_AREA_NUM];
int g_org_lms_area_num = 0;
static double g_org_lms_xarray[LMS_ANGLE_NUM];
static int g_org_lms_xarray_init_flag = 0;
static int g_org_lms_search_s = LMS_LEFT_ANGLE_1;
static int g_org_lms_search_e = LMS_RIGHT_ANGLE_1;
/****/

static int VLMS_TURNING[LMS_ANGLE_NUM];
static int VLMS[LMS_ANGLE_NUM];
static COOR2 LMS_POINT[LMS_ANGLE_NUM][LMS_POINT_NUM];
LMS_AREA g_lms_area[LMS_AREA_NUM];
int g_lms_area_num = 0;

static int g_lms_search_s = LMS_LEFT_ANGLE_1;
static int g_lms_search_e = LMS_RIGHT_ANGLE_1;

#define LMS_THROUGH_AREA_NUM 60
static LMS_AREA g_lms_through_area[LMS_THROUGH_AREA_NUM][LMS_AREA_NUM];
static int g_lms_through_area_num;
static LMS_SEGMENT_AREA_DATE g_lms_segment_area_date[LMS_THROUGH_AREA_NUM];
//static int g_lms_segment_area_num[LMS_THROUGH_AREA_NUM];

static double g_lms_xarray[LMS_ANGLE_NUM];
static int g_lms_xarray_init_flag = 0;
static int g_lms_car_len = 400;
static int g_lms_car_wid = 200;
//[left     x <= 0;  y <= 400; y > 0; 400x - (xarray + 100)y + 40000 >= 0]
//[right    x >= 0;  y <= 400; y > 0; 400x - (xarray - 100)y - 40000 <= 0]
void org_check_turning_angle()
{
	if (g_org_lms_xarray_init_flag == 0)
	{
		memset(g_org_lms_xarray, 0, sizeof(double) * LMS_ANGLE_NUM);
		//[左转]
		for (int i = LMS_LEFT_ANGLE_1; i > 90 * 2; i--)
		{
			double theta = ((i / 2.0) - 90) * PI / 180;
			double a = g_lms_car_len / tan(theta);
			double b = g_lms_car_wid / 2;
			double c = g_lms_car_len;
			double d = (a - b) * (a - b);
			double e = c * c;
			g_org_lms_xarray[i] = -a + sqrt(d - e);
		}

		for (int i = LMS_RIGHT_ANGLE_1; i < 90 * 2; i++)
		{
			double theta = (90 - (i / 2.0)) * PI / 180;
			double a = g_lms_car_len / tan(theta);
			double b = g_lms_car_wid / 2;
			double c = g_lms_car_len;
			double d = (a - b) * (a - b);
			double e = c * c;
			g_org_lms_xarray[i] = a - sqrt(d - e);
		}

		g_org_lms_xarray_init_flag = 1;
	}

	//[检测转弯是否会碰撞]
	int collision_flag = 0;
	g_org_lms_search_s = LMS_LEFT_ANGLE_1;
	for (int i = LMS_LEFT_ANGLE_1; i > 90 * 2; i--)
	{//[左转]
		collision_flag = 0;
		double xarray = g_org_lms_xarray[i];

		for (int j = LMS_ANGLE_NUM - 1; j > 90 * 2; j--)
		{
			double theta = (j + 0.0) / 2;
			double x = ORG_VLMS_TURNING[j] * cos(theta * PI / 180);
			double y = ORG_VLMS_TURNING[j] * sin(theta * PI / 180);

			double val = 400 * x - (xarray + 100) * y + 40000;
			if (y >= 200 && y <= 400 && val >= 0 && x <= 0)
			{//[处于碰撞区域]
				collision_flag = 1;
				break;
			}
		}

		if (collision_flag == 1)
			g_org_lms_search_s--;
		else
			break;
	}

	g_org_lms_search_e = LMS_RIGHT_ANGLE_1;
	for (int i = LMS_RIGHT_ANGLE_1; i < 90 * 2; i++)
	{//[左转]
		collision_flag = 0;
		double xarray = g_org_lms_xarray[i];

		for (int j = 0; j < 90 * 2; j++)
		{
			double theta = (j + 0.0) / 2;
			double x = ORG_VLMS_TURNING[j] * cos(theta * PI / 180);
			double y = ORG_VLMS_TURNING[j] * sin(theta * PI / 180);

			double val = 400 * x - (xarray - 100) * y - 40000;
			if (y >= 200 && y <= 400 && val <= 0 && x >= 0)
			{//[处于碰撞区域]
				collision_flag = 1;
				break;
			}
		}

		if (collision_flag == 1)
			g_org_lms_search_e++;
		else
			break;
	}

	int a = 0;
	a = a;
}

void hd_check_turning_angle()
{
	if (g_lms_xarray_init_flag == 0)
	{
		memset(g_lms_xarray, 0, sizeof(double) * LMS_ANGLE_NUM);
		//[左转]
		for (int i = LMS_LEFT_ANGLE_1; i > 90 * 2; i--)
		{
			double theta = ((i / 2.0) - 90) * PI / 180;
			double a = g_lms_car_len / tan(theta);
			double b = g_lms_car_wid / 2;
			double c = g_lms_car_len;
			double d = (a - b) * (a - b);
			double e = c * c;
			g_lms_xarray[i] = -a + sqrt(d - e);
		}

		for (int i = LMS_RIGHT_ANGLE_1; i < 90 * 2; i++)
		{
			double theta = (90 - (i / 2.0)) * PI / 180;
			double a = g_lms_car_len / tan(theta);
			double b = g_lms_car_wid / 2;
			double c = g_lms_car_len;
			double d = (a - b) * (a - b);
			double e = c * c;
			g_lms_xarray[i] = a - sqrt(d - e);
		}

		g_lms_xarray_init_flag = 1;
	}

	//[检测转弯是否会碰撞]
	int collision_flag = 0;
	g_lms_search_s = LMS_LEFT_ANGLE_1;
	for (int i = LMS_LEFT_ANGLE_1; i > 90 * 2; i--)
	{//[左转]
		collision_flag = 0;
		double xarray = g_lms_xarray[i];

		for (int j = LMS_ANGLE_NUM - 1; j > 90 * 2;j--)
		{
			double theta = (j + 0.0) / 2;
			double x = VLMS_TURNING[j] * cos(theta * PI / 180);
			double y = VLMS_TURNING[j] * sin(theta * PI / 180);

			double val = 400 * x - (xarray + 100) * y + 40000;
			if (y >= 200 && y <= 400 && val >= 0 && x <= 0)
			{//[处于碰撞区域]
				collision_flag = 1;
				break;
			}
		}

		if (collision_flag == 1)
			g_lms_search_s--;
		else
			break;
	}

	g_lms_search_e = LMS_RIGHT_ANGLE_1;
	for (int i = LMS_RIGHT_ANGLE_1; i < 90 * 2; i++)
	{//[左转]
		collision_flag = 0;
		double xarray = g_lms_xarray[i];

		for (int j = 0; j < 90 * 2; j++)
		{
			double theta = (j + 0.0) / 2;
			double x = VLMS_TURNING[j] * cos(theta * PI / 180);
			double y = VLMS_TURNING[j] * sin(theta * PI / 180);

			double val = 400 * x - (xarray - 100) * y - 40000;
			if (y >= 200 && y <= 400 && val <= 0 && x >= 0)
			{//[处于碰撞区域]
				collision_flag = 1;
				break;
			}
		}

		if (collision_flag == 1)
			g_lms_search_e++;
		else
			break;
	}

// 	int a = 0;
// 	a = a;
}

void org_grid2LMS()
{
	memset(ORG_VLMS, 0, sizeof(int) * LMS_ANGLE_NUM);
	memset(ORG_LMS_POINT, 0, sizeof(COOR2) * LMS_ANGLE_NUM);
	int sensor_range = 2000;

	for (int i = 0; i < LMS_ANGLE_NUM; i++)
		ORG_VLMS[i] = 2000;

	for (int i = 0; i < LMS_ANGLE_NUM; i++)
		ORG_VLMS_TURNING[i] = 2000;

	for (int i = 0; i < LMS_ANGLE_NUM; i++)
	{
		double theta = (i + 0.0) / 2;

		int num = 0;
		double range_x = sensor_range * cos(theta * PI / 180);
		double range_y = sensor_range * sin(theta * PI / 180) + LMS_MODIFY_Y;
		double x = 0;
		double y = LMS_MODIFY_Y;
		int grid_x = 0;
		int grid_y = 0;
		int collison_flag = 0;
		if (theta >= 0 && theta < 45)
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				x <= range_x && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL) + g_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL) + g_grid_center.y;

				if (g_closed_grid_map[grid_y][grid_x] >= LMS_CV)
				{
					collison_flag = 1;
					ORG_VLMS[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					VLMS_TURNING[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				x += GRID_LEN_PER_CELL;
				y = range_y - (range_y - LMS_MODIFY_Y) / range_x * (range_x - x);
			}
		}
		else if (theta >= 45 && theta < 90)
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				x <= range_x && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL) + g_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL) + g_grid_center.y;

				if (g_closed_grid_map[grid_y][grid_x] >= LMS_CV)
				{
					collison_flag = 1;
					ORG_VLMS[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					VLMS_TURNING[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				y += GRID_LEN_PER_CELL;
				x = range_x - (range_y - y) * range_x / (range_y - LMS_MODIFY_Y);
			}
		}
		else if (theta == 90)
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL) + g_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL) + g_grid_center.y;

				if (g_closed_grid_map[grid_y][grid_x] >= LMS_CV)
				{
					collison_flag = 1;
					ORG_VLMS[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					VLMS_TURNING[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				x = 0;
				y += GRID_LEN_PER_CELL;
			}
		}
		else if (theta > 90 && theta <= 135)
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				x <= -range_x && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL) + g_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL) + g_grid_center.y;

				if (g_closed_grid_map[grid_y][grid_x] >= LMS_CV)
				{
					collison_flag = 1;
					ORG_VLMS[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					VLMS_TURNING[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				y += GRID_LEN_PER_CELL;
				x = range_x - (range_y - y) * range_x / (range_y - LMS_MODIFY_Y);
			}
		}
		else
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				x <= -range_x && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL) + g_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL) + g_grid_center.y;

				if (g_closed_grid_map[grid_y][grid_x] >= LMS_CV)
				{
					collison_flag = 1;
					ORG_VLMS[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					VLMS_TURNING[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				x -= GRID_LEN_PER_CELL;
				y = range_y - (range_y - LMS_MODIFY_Y) / range_x * (range_x - x);
			}
		}
	}


	org_check_turning_angle();

// 	int mid_win[5];
// 	for (int i = 2; i < LMS_ANGLE_NUM - 2; i++)
// 	{
// 		memcpy(mid_win, ORG_VLMS + i, sizeof(int) * 5);
// 		int temp;
// 		for (int j = 0; j < 4; j++)
// 		{
// 			for (int k = j + 1; k < 5; k++)
// 			{
// 				if (mid_win[j] > mid_win[k])
// 				{
// 					temp = mid_win[j];
// 					mid_win[j] = mid_win[k];
// 					mid_win[k] = temp;
// 				}
// 			}
// 		}
// 		ORG_VLMS[i] = mid_win[2];
// 	}

	int a = 0;
	a = a;
}

void org_cluster_LMS()
{
	int ret = 0;
	int as, ae, am;
	int search_s, search_e;

	search_s = LMS_LEFT_ANGLE_1;
	search_e = LMS_RIGHT_ANGLE_1;
	int flag;
	memset(g_org_lms_area, 0, sizeof(LMS_AREA) * LMS_AREA_NUM);
	g_org_lms_area_num = 0;
	as = ae = search_s;
	flag = 0;
	for (int i = search_s; i >= search_e; i--)
	{
		double theta = (i + 0.0) / 2;
		double x = ORG_VLMS[i] * cos(theta * PI / 180);
		double y = ORG_VLMS[i] * sin(theta * PI / 180);

		if (y > 0)
		{
			if (flag == 0)
			{
				flag = 1;
				as = ae = i;

				if (i == search_e)
				{
					am = (as - ae + 1) / 2 + ae;
					g_org_lms_area[g_org_lms_area_num].as = as;
					g_org_lms_area[g_org_lms_area_num].ae = ae;
					g_org_lms_area[g_org_lms_area_num].am = am;
					g_org_lms_area[g_org_lms_area_num].num = as - ae + 1;
					g_org_lms_area[g_org_lms_area_num].as_dist = ORG_VLMS[as];
					g_org_lms_area[g_org_lms_area_num].ae_dist = ORG_VLMS[ae];
					g_org_lms_area_num++;
					break;
				}
			}
			else
			{
				ae = i;
				if (i == search_e)
				{

					am = (as - ae + 1) / 2 + ae;
					g_org_lms_area[g_org_lms_area_num].as = as;
					g_org_lms_area[g_org_lms_area_num].ae = ae;
					g_org_lms_area[g_org_lms_area_num].am = am;
					g_org_lms_area[g_org_lms_area_num].num = as - ae + 1;
					g_org_lms_area[g_org_lms_area_num].as_dist = ORG_VLMS[as];
					g_org_lms_area[g_org_lms_area_num].ae_dist = ORG_VLMS[ae];
					g_org_lms_area_num++;
					break;
				}

				double pre_x = ORG_VLMS[i + 1] * cos(theta * PI / 180);
				double pre_y = ORG_VLMS[i + 1] * sin(theta * PI / 180);
				//[]
				double dx = x - pre_x;
				double dy = y - pre_y;
				double dist = sqrt(dx * dx + dy * dy);//sqrt(y * y + pre_y * pre_y - 2 * y * pre_y * cos(0.5 * PI / 180));
				//if (abs(pre_y - y) > LMS_DIST_THRESHOLD)
				if (dist > LMS_DIST_THRESHOLD)
				{
					ae++;
					am = (as - ae + 1) / 2 + ae;
					g_org_lms_area[g_org_lms_area_num].as = as;
					g_org_lms_area[g_org_lms_area_num].ae = ae;
					g_org_lms_area[g_org_lms_area_num].am = am;
					g_org_lms_area[g_org_lms_area_num].num = as - ae + 1;
					g_org_lms_area[g_org_lms_area_num].as_dist = ORG_VLMS[as];
					g_org_lms_area[g_org_lms_area_num].ae_dist = ORG_VLMS[ae];
					g_org_lms_area_num++;
					as = ae = i;
				}
			}
		}
		else
		{
			if (flag == 0)
			{
				as = ae = i;
			}
			else
			{
				am = (as - ae + 1) / 2 + ae;
				g_org_lms_area[g_org_lms_area_num].as = as;
				g_org_lms_area[g_org_lms_area_num].ae = ae;
				g_org_lms_area[g_org_lms_area_num].am = am;
				g_org_lms_area[g_org_lms_area_num].num = as - ae + 1;
				g_org_lms_area[g_org_lms_area_num].as_dist = ORG_VLMS[as];
				g_org_lms_area[g_org_lms_area_num].ae_dist = ORG_VLMS[ae];
				g_org_lms_area_num++;
				as = ae = i;
				flag = 0;
			}
		}
	}

	int a = 0;
	a = a;
}


COOR2 get_LMS_goal_pt(double local_goal_angle, int mode)
{
	COOR2 goal_pt;

	int max_dist = -1;
	int max_dist_idx = -1;
	for (int i = 0; i < g_org_lms_area_num; i++)
	{
		max_dist = -1;
		for (int j = g_org_lms_area[i].as; j >= g_org_lms_area[i].ae; j--)
		{
			if (ORG_VLMS[j] > max_dist)
			{
				max_dist = ORG_VLMS[j];
			}
		}
		g_org_lms_area[i].max_dist = max_dist;
	}

	//[正前方优先]
	max_dist = 0;
	max_dist_idx = -1;
	int min_angle_idx = MAX_VALUE;
	double min_angle = MAX_VALUE;

	if (g_org_LMS_last_angle == -1)
	{
		g_org_LMS_last_angle = 90;
	}

		if (mode == 1)
		g_org_LMS_last_angle = local_goal_angle;
	
	for (int i = 0; i < g_org_lms_area_num; i++)
	{
		if (g_org_lms_area[i].max_dist > 0 && g_org_lms_area[i].max_dist > max_dist)
		{
			double angle = g_org_lms_area[i].am;
			max_dist = g_org_lms_area[i].max_dist;
			max_dist_idx = i;
			min_angle = abs(angle / 2 - g_org_LMS_last_angle);
			min_angle_idx = (int)angle;
		}
		else if (g_org_lms_area[i].max_dist > 0 && g_org_lms_area[i].max_dist == max_dist)
		{
			double angle = g_org_lms_area[i].am;
			if (abs(angle / 2 - g_org_LMS_last_angle) < min_angle)
			{
				max_dist = g_org_lms_area[i].max_dist;
				max_dist_idx = i;
				min_angle = abs(angle / 2 - g_org_LMS_last_angle);
				min_angle_idx = (int)angle;
			}
		}
	}

	if (g_org_lms_area[max_dist_idx].ae / 2 < g_org_LMS_last_angle && \
		g_org_lms_area[max_dist_idx].as / 2 > g_org_LMS_last_angle)
	{//[如果期望角度落在区间之内，偏向期望角度]
		min_angle_idx = (int)(g_org_LMS_last_angle * 2);
		if (min_angle_idx > 360)
		{
			min_angle_idx = 360;
		}
		else if (min_angle_idx < 0)
			min_angle_idx = 0;
		max_dist = ORG_VLMS[min_angle_idx];
	}
	else
	{
		max_dist = ORG_VLMS[min_angle_idx];
		g_org_LMS_last_angle = min_angle_idx / 2;
	}

	goal_pt.x = (INT32)((max_dist - 50) * cos(min_angle_idx / 2 * PI / 180));
	goal_pt.y = (INT32)((max_dist - 50) * sin(min_angle_idx / 2 * PI / 180));
	return goal_pt;
}

void hd_grid2LMS()
{
	memset(VLMS, 0, sizeof(int) * LMS_ANGLE_NUM);
	memset(LMS_POINT, 0, sizeof(COOR2) * LMS_ANGLE_NUM);
	int sensor_range = 2000;

	for (int i = 0; i < LMS_ANGLE_NUM; i++)
		VLMS[i] = 2000;

	for (int i = 0; i < LMS_ANGLE_NUM; i++)
	{
		double theta = (i + 0.0) / 2;

		int num = 0;
		double range_x = sensor_range * cos(theta * PI / 180);
		double range_y = sensor_range * sin(theta * PI / 180) + LMS_MODIFY_Y;
		double x = 0;
		double y = LMS_MODIFY_Y;
		int grid_x = 0;
		int grid_y = 0;
		int collison_flag = 0;
		if (theta >= 0 && theta < 45)
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				x <= range_x && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y;

				if (g_hd_closed_grid_map[grid_y][grid_x] >= LMS_CV)
				{
					collison_flag = 1;
					VLMS[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				x += GRID_LEN_PER_CELL_HD;
				y = range_y - (range_y - LMS_MODIFY_Y) / range_x * (range_x - x);
			}
		}
		else if (theta >= 45 && theta < 90)
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				x <= range_x && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y;

				if (g_hd_closed_grid_map[grid_y][grid_x] >= LMS_CV)
				{
					collison_flag = 1;
					VLMS[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				y += GRID_LEN_PER_CELL_HD;
				x = range_x - (range_y - y) * range_x / (range_y - LMS_MODIFY_Y);
			}
		}
		else if (theta == 90)
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y;

				if (g_hd_closed_grid_map[grid_y][grid_x] >= LMS_CV)
				{
					collison_flag = 1;
					VLMS[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				x = 0;
				y += GRID_LEN_PER_CELL_HD;
			}
		}
		else if (theta > 90 && theta <= 135)
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				x <= -range_x && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y;

				if (g_hd_closed_grid_map[grid_y][grid_x] >= LMS_CV)
				{
					collison_flag = 1;
					VLMS[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				y += GRID_LEN_PER_CELL_HD;
				x = range_x - (range_y - y) * range_x / (range_y - LMS_MODIFY_Y);
			}
		}
		else
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				x <= -range_x && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y;

				if (g_hd_closed_grid_map[grid_y][grid_x] >= LMS_CV)
				{
					collison_flag = 1;
					VLMS[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				x -= GRID_LEN_PER_CELL_HD;
				y = range_y - (range_y - LMS_MODIFY_Y) / range_x * (range_x - x);
			}
		}
	}

	for (int i = 0; i < LMS_ANGLE_NUM; i++)
		VLMS_TURNING[i] = 2000;

	for (int i = 0; i < LMS_ANGLE_NUM; i++)
	{
		double theta = (i + 0.0) / 2;

		int num = 0;
		double range_x = sensor_range * cos(theta * PI / 180);
		double range_y = sensor_range * sin(theta * PI / 180) + LMS_MODIFY_Y;
		double x = 0;
		double y = LMS_MODIFY_Y;
		int grid_x = 0;
		int grid_y = 0;
		int collison_flag = 0;
		if (theta >= 0 && theta < 45)
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				x <= range_x && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y;

				if (g_hd_closed_grid_map[grid_y][grid_x] >= LMS_CV_TURNING)
				{
					collison_flag = 1;
					VLMS_TURNING[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				x += GRID_LEN_PER_CELL_HD;
				y = range_y - (range_y - LMS_MODIFY_Y) / range_x * (range_x - x);
			}
		}
		else if (theta >= 45 && theta < 90)
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				x <= range_x && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y;

				if (g_hd_closed_grid_map[grid_y][grid_x] >= LMS_CV_TURNING)
				{
					collison_flag = 1;
					VLMS_TURNING[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				y += GRID_LEN_PER_CELL_HD;
				x = range_x - (range_y - y) * range_x / (range_y - LMS_MODIFY_Y);
			}
		}
		else if (theta == 90)
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y;

				if (g_hd_closed_grid_map[grid_y][grid_x] >= LMS_CV_TURNING)
				{
					collison_flag = 1;
					VLMS_TURNING[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				x = 0;
				y += GRID_LEN_PER_CELL_HD;
			}
		}
		else if (theta > 90 && theta <= 135)
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				x <= -range_x && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y;

				if (g_hd_closed_grid_map[grid_y][grid_x] >= LMS_CV_TURNING)
				{
					collison_flag = 1;
					VLMS_TURNING[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				y += GRID_LEN_PER_CELL_HD;
				x = range_x - (range_y - y) * range_x / (range_y - LMS_MODIFY_Y);
			}
		}
		else
		{
			x = 0;
			y = LMS_MODIFY_Y;
			grid_x = 0;
			grid_y = 0;
			collison_flag = 0;

			while (num < LMS_POINT_NUM && \
				x <= -range_x && \
				y <= range_y && \
				collison_flag == 0)
			{
				grid_x = (int)(x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x;
				grid_y = (int)(y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y;

				if (g_hd_closed_grid_map[grid_y][grid_x] >= LMS_CV_TURNING)
				{
					collison_flag = 1;
					VLMS_TURNING[i] = (int)sqrt(x * x + (y - LMS_MODIFY_Y) * (y - LMS_MODIFY_Y));
					break;
				}

				x -= GRID_LEN_PER_CELL_HD;
				y = range_y - (range_y - LMS_MODIFY_Y) / range_x * (range_x - x);
			}
		}
	}

	hd_check_turning_angle();
// 	g_lms_search_s = LMS_LEFT_ANGLE_1;
// 	g_lms_search_e = LMS_RIGHT_ANGLE_1;
	/*
	for (int i = LMS_ANGLE_NUM - 1; i >= 0; i--)
	{
		double theta = (i + 0.0) / 2;
		double x = VLMS[i] * cos(theta * PI / 180);
		double y = VLMS[i] * sin(theta * PI / 180);
		if (y > 200 && y < 400 && x >= -105 && x <= 105)
		{
			VLMS[i] = 0;
			if (x < 0)
			{
				//g_lms_search_s = LMS_LEFT_ANGLE_2;
			}

			if (x > 0)
			{
				//g_lms_search_e = LMS_RIGHT_ANGLE_2;
			}
		}
	}
	*/

	int mid_win[5];
	for (int i = 2; i < LMS_ANGLE_NUM - 2; i++)
	{
		memcpy(mid_win, VLMS + i, sizeof(int) * 5);
		int temp;
		for (int j = 0; j < 4; j++)
		{
			for (int k = j + 1; k < 5; k++)
			{
				if (mid_win[j] > mid_win[k])
				{
					temp = mid_win[j];
					mid_win[j] = mid_win[k];
					mid_win[k] = temp;
				}
			}
		}
		VLMS[i] = mid_win[2];
	}

	int a = 0;
	a = a;
}

double hd_LMS_through_dist(int area_idx, int cur_idx, int left_dist, int right_dist)
{
	double through_dist = 0;
	int as, ae;
	int num = 0;
	as = g_lms_through_area[area_idx][cur_idx].as;
	ae = g_lms_through_area[area_idx][cur_idx].ae;
	int idx;
	if (left_dist < right_dist)
	{
		idx = ae;
		for (int i = ae; i <= as;i++)
		{
			if (VLMS[i] > left_dist)
			{
				idx = i;
				break;
			}
		}
		num = as - idx + 1;

		through_dist = left_dist * (num * 0.5 * PI / 180);
	}
	else
	{
		idx = as;
		for (int i = as; i >= ae; i--)
		{
			if (VLMS[i] > right_dist)
			{
				idx = i;
				break;
			}
		}
		num = idx - ae + 1;

		through_dist = right_dist * (num * 0.5 * PI / 180);
	}

	return through_dist;
}

void hd_LMS_merge_area()
{
	//[标记凹障碍]
	int concave_num = 0;
	for (int i = 1; i < g_lms_area_num - 1; i++)
	{
		if (g_lms_area[i].as_dist < g_lms_area[i - 1].ae_dist && \
			g_lms_area[i].ae_dist < g_lms_area[i + 1].as_dist)
		{
			g_lms_area[i].concave = 1;
			concave_num++;
		}
	}

	//[提取被分割的可通行区域]
	g_lms_through_area_num = 0;
	memset(g_lms_through_area, 0, sizeof(LMS_AREA) * LMS_THROUGH_AREA_NUM * LMS_AREA_NUM);
	memset(g_lms_segment_area_date, 0, sizeof(LMS_SEGMENT_AREA_DATE) * LMS_THROUGH_AREA_NUM);
	int idx;
	int flag = 0;
	for (int i = 0; i < g_lms_area_num;i++)
	{
		if (g_lms_area[i].concave == 0)
		{
			if (flag == 0)
			{
				flag = 1;
				idx = 0;
				g_lms_through_area[g_lms_through_area_num][idx] = g_lms_area[i];

				if (i == g_lms_area_num - 1)
				{
					g_lms_segment_area_date[g_lms_through_area_num].num = idx + 1;
					g_lms_through_area_num++;
				}
			}
			else
			{
				idx++;
				g_lms_through_area[g_lms_through_area_num][idx] = g_lms_area[i];

				if (i == g_lms_area_num - 1)
				{
					g_lms_segment_area_date[g_lms_through_area_num].num = idx + 1;
					g_lms_through_area_num++;
				}
			}
		}
		else
		{
			if (flag == 0)
			{
			}
			else
			{
				g_lms_segment_area_date[g_lms_through_area_num].num = idx + 1;
				g_lms_through_area_num++;
				flag = 0;
			}
		}
	}
	int a = 0;
	a = a;

	//[计算每个通行区域的可通行范围]
	for (int i = 0; i < g_lms_through_area_num;i++)
	{
	
// 		int is = 0;
// 		int ie = g_lms_segment_area_date[i].num - 1;

		//[先找最远的区域]
		int max_dist = -1;
		int max_dist_idx = -1;
		for (int j = 0; j < g_lms_segment_area_date[i].num; j++)
		{
			if (g_lms_through_area[i][j].max_dist > max_dist)
			{
				max_dist = g_lms_through_area[i][j].max_dist;
				max_dist_idx = j;
			}
		}

		//[!!LMS角度索引值0为右侧的方向]
		int cur_left_idx;
		int cur_right_idx;
		int cur_idx;
		int left_min_dist;
		int right_min_dist;
		//int check_min_dist;
		double through_dist;
		//int angle_num;
		cur_idx = max_dist_idx;
		if (cur_idx == 0)
		{
			left_min_dist = g_lms_through_area[i][cur_idx].as_dist;
			cur_left_idx = 0;
		}
		else
		{
			left_min_dist = g_lms_through_area[i][cur_idx].as_dist < g_lms_through_area[i][cur_idx - 1].ae_dist ? \
				g_lms_through_area[i][cur_idx].as_dist : g_lms_through_area[i][cur_idx - 1].ae_dist;
			cur_left_idx = cur_idx - 1;
		}

		if (cur_idx == g_lms_segment_area_date[i].num - 1)
		{
			right_min_dist = g_lms_through_area[i][cur_idx].ae_dist;
			cur_right_idx = g_lms_segment_area_date[i].num - 1;
		}
		else
		{
			right_min_dist = g_lms_through_area[i][cur_idx].ae_dist < g_lms_through_area[i][cur_idx + 1].as_dist ? \
				g_lms_through_area[i][cur_idx].ae_dist : g_lms_through_area[i][cur_idx + 1].as_dist;
			cur_right_idx = cur_idx + 1;
		}

		through_dist = hd_LMS_through_dist(i, cur_idx, left_min_dist, right_min_dist);

		int normal_exit = 1;
		int left_dist;
		int right_dist;
		while (through_dist < LMS_THROUGHT_DIST_THRESHOLD)
		{//[循环合并]
			if (cur_idx != 0 && cur_idx != g_lms_segment_area_date[i].num - 1)
			{
				left_dist = g_lms_through_area[i][cur_left_idx].ae_dist;
				right_dist = g_lms_through_area[i][cur_right_idx].as_dist;

				if (left_dist < right_dist)
				{//[合并右侧区域]
					g_lms_through_area[i][cur_idx].ae = g_lms_through_area[i][cur_right_idx].ae;
					g_lms_through_area[i][cur_idx].num += g_lms_through_area[i][cur_right_idx].num;
					g_lms_through_area[i][cur_idx].ae_dist = g_lms_through_area[i][cur_right_idx].ae_dist;
					g_lms_through_area[i][cur_idx].am = g_lms_through_area[i][cur_idx].num / 2 + g_lms_through_area[i][cur_idx].ae;
					for (int j = cur_right_idx + 1; j < g_lms_segment_area_date[i].num; j++)
						g_lms_through_area[i][j - 1] = g_lms_through_area[i][j];
					g_lms_segment_area_date[i].num--;
					cur_right_idx = cur_idx + 1;
					if (cur_right_idx > g_lms_segment_area_date[i].num - 1)
						cur_right_idx = g_lms_segment_area_date[i].num - 1;
				}
				else
				{//[合并左侧区域]
					g_lms_through_area[i][cur_idx].as = g_lms_through_area[i][cur_left_idx].as;
					g_lms_through_area[i][cur_idx].num += g_lms_through_area[i][cur_left_idx].num;
					g_lms_through_area[i][cur_idx].as_dist = g_lms_through_area[i][cur_left_idx].as_dist;
					g_lms_through_area[i][cur_idx].am = g_lms_through_area[i][cur_idx].num / 2 + g_lms_through_area[i][cur_idx].ae;
					for (int j = cur_left_idx + 1; j < g_lms_segment_area_date[i].num; j++)
						g_lms_through_area[i][j - 1] = g_lms_through_area[i][j];
					g_lms_segment_area_date[i].num--;
					cur_idx--;
					cur_left_idx = cur_idx - 1;
					cur_right_idx = cur_idx + 1;
					if (cur_left_idx < 0)
						cur_left_idx = 0;
					if (cur_right_idx > g_lms_segment_area_date[i].num - 1)
						cur_right_idx = g_lms_segment_area_date[i].num - 1;
				}

				//[可通行计算]
				if (cur_left_idx == cur_idx)
				{
					left_min_dist = g_lms_through_area[i][cur_idx].as_dist;
				}
				else
				{
					left_min_dist = g_lms_through_area[i][cur_idx].as_dist < g_lms_through_area[i][cur_idx - 1].ae_dist ? \
						g_lms_through_area[i][cur_idx].as_dist : g_lms_through_area[i][cur_idx - 1].ae_dist;
				}
				if (cur_right_idx == cur_idx)
				{
					right_min_dist = g_lms_through_area[i][cur_idx].ae_dist;
				}
				else
				{
					right_min_dist = g_lms_through_area[i][cur_idx].ae_dist < g_lms_through_area[i][cur_idx + 1].as_dist ? \
						g_lms_through_area[i][cur_idx].ae_dist : g_lms_through_area[i][cur_idx + 1].as_dist;
				}

				through_dist = hd_LMS_through_dist(i, cur_idx, left_min_dist, right_min_dist);
// 				check_min_dist = left_min_dist < right_min_dist ? left_min_dist : right_min_dist;
// 				angle_num = g_lms_through_area[i][cur_idx].num;
// 				through_dist = check_min_dist * (angle_num * 0.5 * PI / 180);
			}
			else if (cur_idx == 0 && cur_idx != g_lms_segment_area_date[i].num - 1)
			{//[合并右侧区域]
				g_lms_through_area[i][cur_idx].ae = g_lms_through_area[i][cur_right_idx].ae;
				g_lms_through_area[i][cur_idx].num += g_lms_through_area[i][cur_right_idx].num;
				g_lms_through_area[i][cur_idx].ae_dist = g_lms_through_area[i][cur_right_idx].ae_dist;
				g_lms_through_area[i][cur_idx].am = g_lms_through_area[i][cur_idx].num / 2 + g_lms_through_area[i][cur_idx].ae;
				for (int j = cur_right_idx + 1; j < g_lms_segment_area_date[i].num; j++)
					g_lms_through_area[i][j - 1] = g_lms_through_area[i][j];
				g_lms_segment_area_date[i].num--;
				cur_right_idx = cur_idx + 1;
				if (cur_right_idx > g_lms_segment_area_date[i].num - 1)
					cur_right_idx = g_lms_segment_area_date[i].num - 1;

				//[可通行计算]
				left_min_dist = g_lms_through_area[i][cur_idx].as_dist;
				if (cur_right_idx == cur_idx)
				{
					right_min_dist = g_lms_through_area[i][cur_idx].ae_dist;
				}
				else
				{
					right_min_dist = g_lms_through_area[i][cur_idx].ae_dist < g_lms_through_area[i][cur_idx + 1].as_dist ? \
						g_lms_through_area[i][cur_idx].ae_dist : g_lms_through_area[i][cur_idx + 1].as_dist;
				}

				through_dist = hd_LMS_through_dist(i, cur_idx, left_min_dist, right_min_dist);
// 				check_min_dist = left_min_dist < right_min_dist ? left_min_dist : right_min_dist;
// 				angle_num = g_lms_through_area[i][cur_idx].num;
// 				through_dist = check_min_dist * (angle_num * 0.5 * PI / 180);
			}
			else if (cur_idx != 0 && cur_idx == g_lms_segment_area_date[i].num - 1)
			{//[合并左侧区域]
				g_lms_through_area[i][cur_idx].as = g_lms_through_area[i][cur_left_idx].as;
				g_lms_through_area[i][cur_idx].num += g_lms_through_area[i][cur_left_idx].num;
				g_lms_through_area[i][cur_idx].as_dist = g_lms_through_area[i][cur_left_idx].as_dist;
				g_lms_through_area[i][cur_idx].am = g_lms_through_area[i][cur_idx].num / 2 + g_lms_through_area[i][cur_idx].ae;
				for (int j = cur_left_idx + 1; j < g_lms_segment_area_date[i].num; j++)
					g_lms_through_area[i][j - 1] = g_lms_through_area[i][j];
				g_lms_segment_area_date[i].num--;
				cur_idx--;
				cur_left_idx = cur_idx - 1;
				cur_right_idx = cur_idx;
				if (cur_left_idx < 0)
					cur_left_idx = 0;

				//[可通行计算]
				if (cur_left_idx == cur_idx)
				{
					left_min_dist = g_lms_through_area[i][cur_idx].as_dist;
				}
				else
				{
					left_min_dist = g_lms_through_area[i][cur_idx].as_dist < g_lms_through_area[i][cur_idx - 1].ae_dist ? \
						g_lms_through_area[i][cur_idx].as_dist : g_lms_through_area[i][cur_idx - 1].ae_dist;
				}
				right_min_dist = g_lms_through_area[i][cur_idx].ae_dist;

				through_dist = hd_LMS_through_dist(i, cur_idx, left_min_dist, right_min_dist);
// 				check_min_dist = left_min_dist < right_min_dist ? left_min_dist : right_min_dist;
// 				angle_num = g_lms_through_area[i][cur_idx].num;
// 				through_dist = check_min_dist * (angle_num * 0.5 * PI / 180);
			}

			if (g_lms_segment_area_date[i].num == 1)
			{
				if (through_dist < LMS_THROUGHT_DIST_THRESHOLD)
				{
					g_lms_segment_area_date[i].max_gap = 0;
					g_lms_segment_area_date[i].max_gap_idx = -1;
					normal_exit = 0;
					break;
				}
			}
		}

		if (normal_exit == 1)
		{
			g_lms_segment_area_date[i].max_gap = through_dist;
			g_lms_segment_area_date[i].max_gap_idx = cur_idx;
		}
	}
	/*
	int is, ie;
	g_lms_through_area_num = 0;

	memset(g_lms_through_area, 0, sizeof(LMS_AREA) * LMS_THROUGH_AREA_NUM * LMS_AREA_NUM);
	is = ie = 0;
	for (int i = 0; i < g_lms_area_num; i++)
	{
		if (g_lms_area[i].concave == 0)
		{
			if (flag == 0)
			{
				flag = 1;
				is = ie = i;

				if (i == g_lms_area_num - 1)
				{
					g_lms_through_area[g_lms_through_area_num].is = is;
					g_lms_through_area[g_lms_through_area_num].ie = ie;
					g_lms_through_area_num++;
					break;
				}
			}
			else
			{
				ie = i;
				if (i == g_lms_area_num - 1)
				{
					g_lms_through_area[g_lms_through_area_num].is = is;
					g_lms_through_area[g_lms_through_area_num].ie = ie;
					g_lms_through_area_num++;
					break;
				}
			}
		}
		else
		{
			if (flag == 0)
			{
				is = ie = i;
			}
			else
			{
				g_lms_through_area[g_lms_through_area_num].is = is;
				g_lms_through_area[g_lms_through_area_num].ie = ie;
				g_lms_through_area_num++;
				is = ie = i;
				flag = 0;
			}
		}
	}

	for (int i = 0; i < g_lms_through_area_num; i++)
	{
		int is = g_lms_through_area[i].is;
		int ie = g_lms_through_area[i].ie;

		//[搜索合并可通行区域]
		int max_dist = -1;
		int max_dist_idx = -1;
		for (int j = is; j <= ie; j++)
		{
			if (g_lms_area[j].max_dist > max_dist)
			{
				max_dist = g_lms_area[j].max_dist;
				max_dist_idx = j;
			}
		}

		//[!!LMS角度索引值0为右侧的方向]
		int cur_left_idx;
		int cur_right_idx;
		int cur_idx;
		int left_min_dist;
		int right_min_dist;
		int check_min_dist;
		double through_dist;
		int angle_num;
		cur_idx = max_dist_idx;
		if (cur_idx == is)
		{
			left_min_dist = g_lms_area[cur_idx].as_dist;
			cur_left_idx = is;
		}
		else
		{
			left_min_dist = g_lms_area[cur_idx].as_dist < g_lms_area[cur_idx - 1].ae_dist ? \
				g_lms_area[cur_idx].as_dist : g_lms_area[cur_idx - 1].ae_dist;
			cur_left_idx = cur_idx - 1;
		}

		if (cur_idx == ie)
		{
			right_min_dist = g_lms_area[cur_idx].ae_dist;
			cur_right_idx = cur_idx;
		}
		else
		{
			right_min_dist = g_lms_area[cur_idx].ae_dist < g_lms_area[cur_idx + 1].as_dist ? \
				g_lms_area[cur_idx].ae_dist : g_lms_area[cur_idx + 1].as_dist;
			cur_right_idx = cur_idx + 1;
		}

		check_min_dist = left_min_dist < right_min_dist ? left_min_dist : right_min_dist;
		angle_num = g_lms_area[cur_idx].num;
		through_dist = check_min_dist * (angle_num * 0.5 * PI / 180);

		int normal_exit = 1;
		int left_dist;
		int right_dist;
		while (through_dist < LMS_THROUGHT_DIST_THRESHOLD)
		{//[循环合并]
			if (cur_idx != is && cur_idx != ie)
			{
				left_dist = g_lms_area[cur_left_idx].ae_dist;
				right_dist = g_lms_area[cur_right_idx].as_dist;

				if (left_dist < right_dist)
				{//[合并右侧区域]
					g_lms_area[cur_idx].ae = g_lms_area[cur_right_idx].ae;
					g_lms_area[cur_idx].num += g_lms_area[cur_right_idx].num;
					g_lms_area[cur_idx].ae_dist = g_lms_area[cur_right_idx].ae_dist;
					g_lms_area[cur_idx].am = g_lms_area[cur_idx].num / 2 + g_lms_area[cur_idx].ae;
					for (int i = cur_right_idx + 1; i < g_lms_area_num; i++)
						g_lms_area[i - 1] = g_lms_area[i];
					g_lms_area_num--;
				}
				else
				{//[合并左侧区域]
					g_lms_area[cur_idx].as = g_lms_area[cur_left_idx].as;
					g_lms_area[cur_idx].num += g_lms_area[cur_left_idx].num;
					g_lms_area[cur_idx].as_dist = g_lms_area[cur_left_idx].as_dist;
					g_lms_area[cur_idx].am = g_lms_area[cur_idx].num / 2 + g_lms_area[cur_idx].ae;
					for (int i = cur_left_idx + 1; i < g_lms_area_num; i++)
						g_lms_area[i - 1] = g_lms_area[i];
					g_lms_area_num--;
					cur_right_idx--;
					cur_left_idx--;
					cur_idx--;
				}

				//[可通行计算]
				left_min_dist = g_lms_area[cur_idx].as_dist < g_lms_area[cur_idx - 1].ae_dist ? \
					g_lms_area[cur_idx].as_dist : g_lms_area[cur_idx - 1].ae_dist;
				right_min_dist = g_lms_area[cur_idx].ae_dist < g_lms_area[cur_idx + 1].as_dist ? \
					g_lms_area[cur_idx].ae_dist : g_lms_area[cur_idx + 1].as_dist;
				check_min_dist = left_min_dist < right_min_dist ? left_min_dist : right_min_dist;
				angle_num = g_lms_area[cur_idx].num;
				through_dist = check_min_dist * (angle_num * 0.5 * PI / 180);
			}
			else if (cur_idx == 0 && cur_idx != g_lms_area_num - 1)
			{//[合并右侧区域]
				g_lms_area[cur_idx].ae = g_lms_area[cur_right_idx].ae;
				g_lms_area[cur_idx].num += g_lms_area[cur_right_idx].num;
				g_lms_area[cur_idx].ae_dist = g_lms_area[cur_right_idx].ae_dist;
				g_lms_area[cur_idx].am = g_lms_area[cur_idx].num / 2 + g_lms_area[cur_idx].ae;
				for (int i = cur_right_idx + 1; i < g_lms_area_num; i++)
					g_lms_area[i - 1] = g_lms_area[i];
				g_lms_area_num--;

				//[可通行计算]
				left_min_dist = g_lms_area[cur_idx].as_dist;
				right_min_dist = g_lms_area[cur_idx].ae_dist < g_lms_area[cur_idx + 1].as_dist ? \
					g_lms_area[cur_idx].ae_dist : g_lms_area[cur_idx + 1].as_dist;
				check_min_dist = left_min_dist < right_min_dist ? left_min_dist : right_min_dist;
				angle_num = g_lms_area[cur_idx].num;
				through_dist = check_min_dist * (angle_num * 0.5 * PI / 180);
			}
			else if (cur_idx != 0 && cur_idx == g_lms_area_num - 1)
			{//[合并左侧区域]
				g_lms_area[cur_idx].as = g_lms_area[cur_left_idx].as;
				g_lms_area[cur_idx].num += g_lms_area[cur_left_idx].num;
				g_lms_area[cur_idx].as_dist = g_lms_area[cur_left_idx].as_dist;
				g_lms_area[cur_idx].am = g_lms_area[cur_idx].num / 2 + g_lms_area[cur_idx].ae;
				for (int i = cur_left_idx + 1; i < g_lms_area_num; i++)
					g_lms_area[i - 1] = g_lms_area[i];
				g_lms_area_num--;
				cur_right_idx--;
				cur_left_idx--;
				cur_idx--;

				//[可通行计算]
				left_min_dist = g_lms_area[cur_idx].as_dist < g_lms_area[cur_idx - 1].ae_dist ? \
					g_lms_area[cur_idx].as_dist : g_lms_area[cur_idx - 1].ae_dist;
				right_min_dist = g_lms_area[cur_idx].ae_dist;
				angle_num = g_lms_area[cur_idx].num;
				through_dist = check_min_dist * (angle_num * 0.5 * PI / 180);
			}

			if (g_lms_area_num == 1)
			{
				if (through_dist < LMS_THROUGHT_DIST_THRESHOLD)
				{
					normal_exit = 0;
					break;
				}
			}
		}
// 
// 		if (normal_exit == 0)
// 		{
// 			return ret = -1;
// 		}
	}
	*/
}

int hd_LMS_mid_plan()
{
	int ret = 0;
	int as, ae, am;
	int search_s, search_e;
// 	search_s = g_lms_search_s;
// 	search_e = g_lms_search_e;

	search_s = LMS_LEFT_ANGLE_1;
	search_e = LMS_RIGHT_ANGLE_1;
	int flag;
	memset(g_lms_area, 0, sizeof(LMS_AREA) * LMS_AREA_NUM);
	g_lms_area_num = 0;
	as = ae = search_s;
	flag = 0;
	for (int i = search_s; i >= search_e; i--)
	{
		double theta = (i + 0.0) / 2;
		double y = VLMS[i] * sin(theta * PI / 180);

		if (y > 0)
		{
			if (flag == 0)
			{
				flag = 1;
				as = ae = i;

				if (i == search_e)
				{
					am = (as - ae + 1) / 2 + ae;
					g_lms_area[g_lms_area_num].as = as;
					g_lms_area[g_lms_area_num].ae = ae;
					g_lms_area[g_lms_area_num].am = am;
					g_lms_area[g_lms_area_num].num = as - ae + 1;
					g_lms_area[g_lms_area_num].as_dist = VLMS[as];
					g_lms_area[g_lms_area_num].ae_dist = VLMS[ae];
					g_lms_area_num++;
					break;
				}
			}
			else
			{
				ae = i;
				if (i == search_e)
				{

					am = (as - ae + 1) / 2 + ae;
					g_lms_area[g_lms_area_num].as = as;
					g_lms_area[g_lms_area_num].ae = ae;
					g_lms_area[g_lms_area_num].am = am;
					g_lms_area[g_lms_area_num].num = as - ae + 1;
					g_lms_area[g_lms_area_num].as_dist = VLMS[as];
					g_lms_area[g_lms_area_num].ae_dist = VLMS[ae];
					g_lms_area_num++;
					break;
				}

				double pre_y = VLMS[i + 1] * sin(theta * PI / 180);
				double dist = sqrt(y * y + pre_y * pre_y - 2 * y * pre_y * cos(0.5 * PI / 180));
				//if (abs(pre_y - y) > LMS_DIST_THRESHOLD)
				if (dist > LMS_DIST_THRESHOLD)
				{
					ae++;
					am = (as - ae + 1) / 2 + ae;
					g_lms_area[g_lms_area_num].as = as;
					g_lms_area[g_lms_area_num].ae = ae;
					g_lms_area[g_lms_area_num].am = am;
					g_lms_area[g_lms_area_num].num = as - ae + 1;
					g_lms_area[g_lms_area_num].as_dist = VLMS[as];
					g_lms_area[g_lms_area_num].ae_dist = VLMS[ae];
					g_lms_area_num++;
					as = ae = i;
				}
			}
		}
		else
		{
			if (flag == 0)
			{
				as = ae = i;
			}
			else
			{
				am = (as - ae + 1) / 2 + ae;
				g_lms_area[g_lms_area_num].as = as;
				g_lms_area[g_lms_area_num].ae = ae;
				g_lms_area[g_lms_area_num].am = am;
				g_lms_area[g_lms_area_num].num = as - ae + 1;
				g_lms_area[g_lms_area_num].as_dist = VLMS[as];
				g_lms_area[g_lms_area_num].ae_dist = VLMS[ae];
				g_lms_area_num++;
				as = ae = i;
				flag = 0;
			}
		}
	}

	int max_dist = -1;
	int max_dist_idx = -1;
	for (int i = 0; i < g_lms_area_num;i++)
	{
		max_dist = -1;
		for (int j = g_lms_area[i].as; j >= g_lms_area[i].ae;j--)
		{
			if (VLMS[j] > max_dist)
			{
				max_dist = VLMS[j];
			}
		}
		g_lms_area[i].max_dist = max_dist;
	}

	hd_LMS_merge_area();

	//[正前方优先]
	double max_gap = 0;
	int max_segment_idx = -1;
	int max_gap_idx = -1;
	int min_angle_idx = MAX_VALUE;
	double min_angle = MAX_VALUE;
	for (int i = 0; i < g_lms_through_area_num;i++)
	{
		if (g_lms_segment_area_date[i].max_gap > 0)
		{
			int idx = g_lms_segment_area_date[i].max_gap_idx;
			double angle = g_lms_through_area[i][idx].am;
			if (abs(angle / 2 - 90) < min_angle)
			{
				max_segment_idx = i;
				max_gap_idx = idx;
				min_angle = abs(angle / 2 - 90);
				min_angle_idx = (int)angle;
			}
// 			max_gap = g_lms_segment_area_date[i].max_gap_idx;
// 			max_segment_idx = i;
		}
	}

	if (min_angle_idx == MAX_VALUE)
	{
		return ret = -1;
	}
	else
	{
		int left_dist = g_lms_through_area[max_segment_idx][max_gap_idx].as_dist;
		int right_dist = g_lms_through_area[max_segment_idx][max_gap_idx].ae_dist;
		int is = g_lms_through_area[max_segment_idx][max_gap_idx].as;
		int ie = g_lms_through_area[max_segment_idx][max_gap_idx].ae;
		if (left_dist > right_dist)
		{
			int angle_num = 1;
			double through_dist;
			for (int i = is; i >= ie;i--)
			{
				through_dist = right_dist * (angle_num * 0.5 * PI / 180);
				if (through_dist > LMS_THROUGHT_DIST_THRESHOLD)
				{
					min_angle_idx = is - (is - i + 1) / 2;
					break;
				}
				angle_num++;
			}
		}
		else if (left_dist < right_dist)
		{
			int angle_num = 1;
			double through_dist;
			for (int i = ie; i <= is; i++)
			{
				through_dist = right_dist * (angle_num * 0.5 * PI / 180);
				if (through_dist > LMS_THROUGHT_DIST_THRESHOLD)
				{
					min_angle_idx = (i - ie + 1) / 2 + ie;
					break;
				}
				angle_num++;
			}
		}
		else
		{
			min_angle_idx = g_lms_through_area[max_segment_idx][max_gap_idx].am;
		}
	

		min_angle_idx = g_lms_through_area[max_segment_idx][max_gap_idx].am;
// 		int max_gap_idx = g_lms_segment_area_date[max_segment_idx].max_gap_idx;
// 		int am = g_lms_through_area[max_segment_idx][max_gap_idx].am;


// 		if (g_lms_search_s == LMS_LEFT_ANGLE_2)
// 		{
// 			if (am > 90 * 2)
// 			{
// 				am = 90 * 2;
// 			}
// 		}
// 		if (g_lms_search_e == LMS_RIGHT_ANGLE_2)
// 		{
// 			if (am < 90 * 2)
// 			{
// 				am = 90 * 2;
// 			}
// 		}

// 		if (g_lms_search_s == LMS_LEFT_ANGLE_2)
// 		{
// 			if (min_angle_idx > 90 * 2)
// 			{
// 				min_angle_idx = 90 * 2;
// 			}
// 		}
// 		if (g_lms_search_e == LMS_RIGHT_ANGLE_2)
// 		{
// 			if (min_angle_idx < 90 * 2)
// 			{
// 				min_angle_idx = 90 * 2;
// 			}
// 		}

		if (min_angle_idx > g_lms_search_s)
		{
			min_angle_idx = g_lms_search_s;
		}
		else if (min_angle_idx < g_lms_search_e)
		{
			min_angle_idx = g_lms_search_e;
		}

		double theta = min_angle_idx / 2;
		double x = 1000 * cos(theta * PI / 180);
		double y = 1000 * sin(theta * PI / 180);
		double x_step;
		double y_step;
		x_step = x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
		y_step = y / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
		g_mid_line[0].x = 0;
		g_mid_line[0].y = 0;
		for (int i = 1; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			g_mid_line[i].x = (INT32)(i * x_step);
			g_mid_line[i].y = (INT32)(i * y_step);
		}


		//[截取规划线]
		int cut_index = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1;
		int num = 0;
		for (int i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			int xx = (int)((g_mid_line[i].x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x);
			int yy = (int)((g_mid_line[i].y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y);
			if (g_mid_line[i].y > y || g_hd_closed_grid_map[yy][xx] >= 4)
			{
				cut_index = i;
				break;
			}
		}

		if (g_mid_line[cut_index].y < 500)
		{
			return ret = -1;
		}

		COOR2 temp_line[200];
		int temp_line_num = 0;
		double step;
		memset(temp_line, 0, 200 * sizeof(COOR2));
		temp_line_num = 0;
		step = (g_mid_line[cut_index].y - g_mid_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

		line_fitting(g_mid_line, cut_index + 1, temp_line, temp_line_num, (int)step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(g_mid_line, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		g_long_line_num = temp_line_num;


		return ret;
	}
/*
	double theta = g_lms_area[cur_idx].am / 2;
	double x = 1000 * cos(theta * PI / 180);
	double y = 1000 * sin(theta * PI / 180);
	double x_step;
	double y_step;
	x_step = x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
	y_step = y / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
	g_mid_line[0].x = 0;
	g_mid_line[0].y = 0;
	for (int i = 1; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
	{
	g_mid_line[i].x = (INT32)(i * x_step);
	g_mid_line[i].y = (INT32)(i * y_step);
	}
	*/
	
}

int hd_mid_plan()
{
	int ret = 0;
	int as, ae, am;
	int search_s, search_e;
	// 	search_s = g_lms_search_s;
	// 	search_e = g_lms_search_e;

	search_s = LMS_LEFT_ANGLE_1;
	search_e = LMS_RIGHT_ANGLE_1;
	int flag;
	memset(g_lms_area, 0, sizeof(LMS_AREA) * LMS_AREA_NUM);
	g_lms_area_num = 0;
	as = ae = search_s;
	flag = 0;
	for (int i = search_s; i >= search_e; i--)
	{
		double theta = (i + 0.0) / 2;
		double y = VLMS[i] * sin(theta * PI / 180);

		if (y > 0)
		{
			if (flag == 0)
			{
				flag = 1;
				as = ae = i;

				if (i == search_e)
				{
					am = (as - ae + 1) / 2 + ae;
					g_lms_area[g_lms_area_num].as = as;
					g_lms_area[g_lms_area_num].ae = ae;
					g_lms_area[g_lms_area_num].am = am;
					g_lms_area[g_lms_area_num].num = as - ae + 1;
					g_lms_area[g_lms_area_num].as_dist = VLMS[as];
					g_lms_area[g_lms_area_num].ae_dist = VLMS[ae];
					g_lms_area_num++;
					break;
				}
			}
			else
			{
				ae = i;
				if (i == search_e)
				{

					am = (as - ae + 1) / 2 + ae;
					g_lms_area[g_lms_area_num].as = as;
					g_lms_area[g_lms_area_num].ae = ae;
					g_lms_area[g_lms_area_num].am = am;
					g_lms_area[g_lms_area_num].num = as - ae + 1;
					g_lms_area[g_lms_area_num].as_dist = VLMS[as];
					g_lms_area[g_lms_area_num].ae_dist = VLMS[ae];
					g_lms_area_num++;
					break;
				}

				double pre_y = VLMS[i + 1] * sin(theta * PI / 180);
				double dist = sqrt(y * y + pre_y * pre_y - 2 * y * pre_y * cos(0.5 * PI / 180));
				//if (abs(pre_y - y) > LMS_DIST_THRESHOLD)
				if (dist > LMS_DIST_THRESHOLD)
				{
					ae++;
					am = (as - ae + 1) / 2 + ae;
					g_lms_area[g_lms_area_num].as = as;
					g_lms_area[g_lms_area_num].ae = ae;
					g_lms_area[g_lms_area_num].am = am;
					g_lms_area[g_lms_area_num].num = as - ae + 1;
					g_lms_area[g_lms_area_num].as_dist = VLMS[as];
					g_lms_area[g_lms_area_num].ae_dist = VLMS[ae];
					g_lms_area_num++;
					as = ae = i;
				}
			}
		}
		else
		{
			if (flag == 0)
			{
				as = ae = i;
			}
			else
			{
				am = (as - ae + 1) / 2 + ae;
				g_lms_area[g_lms_area_num].as = as;
				g_lms_area[g_lms_area_num].ae = ae;
				g_lms_area[g_lms_area_num].am = am;
				g_lms_area[g_lms_area_num].num = as - ae + 1;
				g_lms_area[g_lms_area_num].as_dist = VLMS[as];
				g_lms_area[g_lms_area_num].ae_dist = VLMS[ae];
				g_lms_area_num++;
				as = ae = i;
				flag = 0;
			}
		}
	}

	int max_dist = -1;
	int max_dist_idx = -1;
	for (int i = 0; i < g_lms_area_num; i++)
	{
		max_dist = -1;
		for (int j = g_lms_area[i].as; j >= g_lms_area[i].ae; j--)
		{
			if (VLMS[j] > max_dist)
			{
				max_dist = VLMS[j];
			}
		}
		g_lms_area[i].max_dist = max_dist;
	}

	//[正前方优先]
	max_dist = 0;
	max_dist_idx = -1;
	int min_angle_idx = MAX_VALUE;
	double min_angle = MAX_VALUE;


	for (int i = 0; i < g_lms_area_num; i++)
	{
		if (g_lms_area[i].max_dist > 0 && g_lms_area[i].max_dist > max_dist)
		{
			double angle = g_lms_area[i].am;
			max_dist = g_lms_area[i].max_dist;
			max_dist_idx = i;
			min_angle = abs(angle / 2 - 90);
			min_angle_idx = (int)angle;
		}
		else if (g_lms_area[i].max_dist > 0 && g_lms_area[i].max_dist == max_dist)
		{
			double angle = g_lms_area[i].am;
			if (abs(angle / 2 - 90) < min_angle)
			{
				max_dist = g_lms_area[i].max_dist;
				max_dist_idx = i;
				min_angle = abs(angle / 2 - 90);
				min_angle_idx = (int)angle;
			}
		}
	}

	if (min_angle_idx == MAX_VALUE)
	{
		return ret = -1;
	}
	else
	{
		if (min_angle_idx > g_lms_search_s)
		{
			min_angle_idx = g_lms_search_s;
		}
		else if (min_angle_idx < g_lms_search_e)
		{
			min_angle_idx = g_lms_search_e;
		}

		double dist = max_dist;
		if (dist < 500)
		{
			return ret = -1;
		}


		double theta = min_angle_idx / 2;
		double x = dist * cos(theta * PI / 180);
		double y = dist * sin(theta * PI / 180);
// 
// 		if (y > 650)
// 		{
// 			y = 650;
// 		}
// 		if (y - 100 > 630)
// 			y = y - 100;
		double last_x = x;
		double last_y = y;
		int clear_flag = 0;
		int collision_flag = 0;

		while (!clear_flag)
		{
			double radian;
			double radius;
			double min_radius = 100000000;
			double D = 0;
			double car_len = 400;
			int goal_index;
			//[确定目标点所在轨迹线下标]
			if (fabs(x) <= 10)//|| xx < 10)//fabs(yy / xx) > 60)//[对应89.5度以上的tan值]
			{
				//[目标点在前方，直行]
				goal_index = MORPHIN_MID_INDEX;
				radian = 90;
			}
			else
			{
				//[目标点不在车正前方-45度到45度之间使用最小转弯半径的虚拟点]
				if (x < 0 && y < -x)
				{
					x = -720;
					y = 0;
				}
				else if (x > 0 && y < x)
				{
					x = 720;
					y = 0;
				}

				if (x < 0)
				{
					D = -(x / 2.0 + (y * y - car_len * car_len) / (2 * x));
					radius = -sqrt(D * D + car_len * car_len);
					radian = atan(car_len / D);

					radian = 90 + radian / PI * 180;
				}
				else if (x > 0)
				{
					D = x / 2.0 + (y * y - car_len * car_len) / (2 * x);
					radius = sqrt(D * D + car_len * car_len);
					radian = atan(car_len / D);

					radian = 90 - (radian / PI * 180);
				}

			}

			//hd_get_avg_mid_line(radian, dist);

			double x_step;
			double y_step;
			x_step = x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
// 			if (y - 100 > 630)
// 				y = y - 100;
			y_step = y / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
			g_mid_line[0].x = 0;
			g_mid_line[0].y = 0;
			for (int i = 1; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1; i++)
			{
				g_mid_line[i].x = roundf2i(i * x_step);
				g_mid_line[i].y = roundf2i(i * y_step);
			}
			g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x = roundf2i(x);
			g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y = roundf2i(y);

			//[碰撞检测]
			//int cut_index = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1;
			collision_flag = 0;
			int num = 0;
			int grid_x;
			int grid_y;
			for (int i = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1; i >= 0; i--)
			{
				grid_x = roundf2i((g_mid_line[i].x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x);
				grid_y = roundf2i((g_mid_line[i].y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y);

				if (g_mid_line[i].y <= 400)
				{
					continue;
				}
				if (g_hd_closed_grid_map[grid_y][grid_x] > 0)
				{
					x = g_mid_line[i].x;
					y = g_mid_line[i].y;
					collision_flag = 1;
					break;
				}
				else
				{
					int x1, x2;
					for (x1 = grid_x; x1 >= grid_x - 44; x1--)//[550cm]
					{
						if (g_hd_closed_grid_map[grid_y][x1] >= 1)
						{
							x1++;
							break;
						}
					}
					for (x2 = grid_x; x2 <= grid_x + 44; x2++)//[550cm]
					{
						if (g_hd_closed_grid_map[grid_y][x2] >= 1)
						{
							x2--;
							break;
						}
					}

// 					if ((x2 - x1 + 1 <= 20) && g_mid_line[i].y > 630)//[狭小，居中]
// 					{
// 						if (abs((grid_x - x1) - (x2 - grid_x)) >= 2)
// 						{
// 							x = g_mid_line[i].x;
// 							y = g_mid_line[i].y;
// 							collision_flag = 2;
// 							break;
// 						}
// 					}
				}
			}

			if (collision_flag)
			{
				//[搜索目标点]
				int left_max_num_x = -1;
				int left_max_num = 0;
				int right_max_num_x = -1;
				int right_max_num = 0;
				//int xs, xe, xnum;
				
				grid_x = roundf2i((x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x);
				grid_y = roundf2i((y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y);


				if (g_hd_closed_grid_map[grid_y][grid_x] == 0)
				{
					int left_flag = 1;
					int right_flag = 1;
					left_max_num_x = right_max_num_x = grid_x;

					for (int i = 0; i <= 24;i++)
					{
						if (left_flag)
						{
							if (g_hd_closed_grid_map[grid_y][grid_x - i] > 0)
							{
								left_flag = 0;
							}
							else
							{
								left_max_num++;
							}
						}
						if (right_flag)
						{
							if (g_hd_closed_grid_map[grid_y][grid_x + i] > 0)
							{
								right_flag = 0;
							}
							else
							{
								right_max_num++;
							}
						}
					}
				}
				else
				{
					int left_flag = 0;
					int right_flag = 0;
					int flag = -1;
					for (int i = 0; i <= 24;i++)
					{
						if (left_flag == 0)
						{
							if (g_hd_closed_grid_map[grid_y][grid_x - i] > 0)
							{
								left_flag = 0;
							}
							else
							{
								left_flag = 1;
							}
						}

						if (right_flag == 0)
						{
							if (g_hd_closed_grid_map[grid_y][grid_x + i] > 0)
							{
								right_flag = 0;
							}
							else
							{
								right_flag = 1;
							}
						}

						if (flag == -1)
						{
							if (left_flag == 1 && right_flag == 1)
							{
								left_max_num_x = grid_x - i;
								right_max_num_x = grid_x + i;

								if (abs(left_max_num_x - g_hd_grid_center.x) <= abs(right_max_num_x - g_hd_grid_center.x))
								{
									flag = 1;
								}
								else
								{
									flag = 2;
								}
								
							}
							else if (left_flag == 1 && right_flag == 0)
							{
								left_max_num_x = grid_x - i;
								flag = 1;
							}
							else if (left_flag == 0 && right_flag == 1)
							{
								right_max_num_x = grid_x + i;
								flag = 2;
							}
						}
						else
						{
							if (flag == 1)
							{
								if (g_hd_closed_grid_map[grid_y][grid_x - i] > 0)
								{
									break;
								}
								else
								{
									left_max_num++;
								}
							}
							else if (flag == 2)
							{
								if (g_hd_closed_grid_map[grid_y][grid_x + i] > 0)
								{
									right_flag = 0;
								}
								else
								{
									right_max_num++;
								}
							}
						}
					}

					if (flag == -1)
					{
						return ret = -1;
					}
				}
				/*
				left_max_num = 0;
				flag = 0;
				for (int i = grid_x; i >= grid_x - 24;i--)
				{
					if (g_hd_closed_grid_map[grid_y][i] == 0)
					{
						if (flag == 0)
						{
							flag = 1;
							xs = xe = i;

							if (i == grid_x - 24)
							{
								xnum = (xs - xe + 1);
								if (xnum > left_max_num)
								{
									left_max_num = xnum;
									left_max_num_x = xs;
								}
							}
						}
						else
						{
							xe = i;
							if (i == grid_x - 24)
							{
								xnum = (xs - xe + 1);
								if (xnum > left_max_num)
								{
									left_max_num = xnum;
									left_max_num_x = xs;
								}
							}
						}
					}
					else
					{
						if (flag == 0)
						{
							xs = xe = i;
						}
						else
						{
							xnum = (xs - xe + 1);
							if (xnum > left_max_num)
							{
								left_max_num = xnum;
								left_max_num_x = xs;
							}
							xs = xe = i;
							flag = 0;
						}
					}
				}

				right_max_num = 0;
				flag = 0;
				for (int i = grid_x; i <= grid_x + 24; i++)
				{
					if (g_hd_closed_grid_map[grid_y][i] == 0)
					{
						if (flag == 0)
						{
							flag = 1;
							xs = xe = i;

							if (i == grid_x + 24)
							{
								xnum = (xe - xs + 1);
								if (xnum > right_max_num)
								{
									right_max_num = xnum;
									right_max_num_x = xs;
								}
							}
						}
						else
						{
							xe = i;
							if (i == grid_x + 24)
							{
								xnum = (xe - xs + 1);
								if (xnum > right_max_num)
								{
									right_max_num = xnum;
									right_max_num_x = xs;
								}
							}
						}
					}
					else
					{
						if (flag == 0)
						{
							xs = xe = i;
						}
						else
						{
							xnum = (xe - xs + 1);
							if (xnum > right_max_num)
							{
								right_max_num = xnum;
								right_max_num_x = xs;
							}
							xs = xe = i;
							flag = 0;
						}
					}
				}
				*/

				//[确定目标点]
				if (left_max_num_x == right_max_num_x)
				{
					if (left_max_num + right_max_num - 1 <= 22)
					{
						if (left_max_num > right_max_num)
						{
							grid_x = grid_x - (left_max_num - right_max_num) / 2;
						}
						else
						{
							grid_x = grid_x + (right_max_num - left_max_num) / 2;
						}
					}
					else
					{
						if (left_max_num > right_max_num)
						{
							if (right_max_num < 6)
							{
								grid_x = grid_x - (6 - right_max_num);
							}
						}
						else
						{
							if (left_max_num < 6)
							{
								grid_x = grid_x + (6 - left_max_num);
							}
						}
					}
				}
				else
				{
					if (left_max_num > right_max_num)
					{
						if (left_max_num <= 20)
						{
							grid_x = left_max_num_x - left_max_num / 2;
						}
						else
						{
							grid_x = left_max_num_x - 5;
						}
					}
					else
					{
						if (right_max_num <= 20)
						{
							grid_x = right_max_num_x + right_max_num / 2;
						}
						else
						{
							grid_x = right_max_num_x + 5;
						}
					}
				}
				x = (grid_x - g_hd_grid_center.x) * GRID_LEN_PER_CELL_HD;
				y = (grid_y - g_hd_grid_center.y) * GRID_LEN_PER_CELL_HD;

				if (collision_flag == 2)
				{
					if ((abs(x - last_x) < 10 && abs(y - last_y) < 10) || y < 650)
					{
						clear_flag = 1;
					}
					else
					{
						last_x = x;
						last_y = y;
						dist = sqrt(x * x + y * y);
					}
				}
				else
				{
					if ((abs(x - last_x) < 10 && abs(y - last_y) < 10) || y < 600)
					{
						return ret = -1;
					}
					else
					{
						last_x = x;
						last_y = y;
						dist = sqrt(x * x + y * y);
					}
				}
			}
			else
			{
				clear_flag = 1;
			}

		}//[while]





// 		double x_step;
// 		double y_step;
// 		x_step = x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
// 		y_step = y / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
// 		g_mid_line[0].x = 0;
// 		g_mid_line[0].y = 0;
// 		for (int i = 1; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
// 		{
// 			g_mid_line[i].x = (INT32)(i * x_step);
// 			g_mid_line[i].y = (INT32)(i * y_step);
// 		}
// 
// 
// 		//[截取规划线]
// 		int cut_index = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1;
// 		int num = 0;
// 		for (int i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
// 		{
// 			int xx = (int)((g_mid_line[i].x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x);
// 			int yy = (int)((g_mid_line[i].y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y);
// 			if (g_mid_line[i].y > y || g_hd_closed_grid_map[yy][xx] >= 4)
// 			{
// 				cut_index = i;
// 				break;
// 			}
// 		}
// 
// 		if (g_mid_line[cut_index].y < 500)
// 		{
// 			return ret = -1;
// 		}

// 		COOR2 temp_line[200];
// 		int temp_line_num = 0;
// 		double step;
// 		memset(temp_line, 0, 200 * sizeof(COOR2));
// 		temp_line_num = 0;
// 		step = (g_mid_line[cut_index].y - g_mid_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
// 
// 		line_fitting(g_mid_line, cut_index + 1, temp_line, temp_line_num, (int)step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
// 		memcpy(g_mid_line, temp_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
// 		g_long_line_num = temp_line_num;


		return ret;
	}
	/*
	double theta = g_lms_area[cur_idx].am / 2;
	double x = 1000 * cos(theta * PI / 180);
	double y = 1000 * sin(theta * PI / 180);
	double x_step;
	double y_step;
	x_step = x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
	y_step = y / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
	g_mid_line[0].x = 0;
	g_mid_line[0].y = 0;
	for (int i = 1; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
	{
	g_mid_line[i].x = (INT32)(i * x_step);
	g_mid_line[i].y = (INT32)(i * y_step);
	}
	*/

}

//[返回值 0 进入狭窄活口  1 交通锥引导  2 处于狭窄活口]
static COOR2 mode2_mid_pt;
int hd_s_obs_plan_mode()
{
	int ret = 0;

	int area[100];
	int area_grid_y[100];
	int area_num = 0;
	memset(area, 0, 100 * sizeof(int));
	memset(area_grid_y, 0, 100 * sizeof(int));

	for (int y = g_hd_grid_center.y + 28; y <= g_hd_grid_center.y + 44;y++)
	{
		int x = g_hd_grid_center.x;
		if (g_hd_closed_grid_map[y][x] >= 4)
		{
			break;
		}

		int x1, x2;
		for (x1 = x; x1 >= x - 44;x1--)//[550cm]
		{
			if (g_hd_closed_grid_map[y][x1] >= 4)
			{
				x1++;
				break;
			}
		}
		for (x2 = x; x2 <= x + 44; x2++)//[550cm]
		{
			if (g_hd_closed_grid_map[y][x2] >= 4)
			{
				x2--;
				break;
			}
		}

		if (x2 - x1 + 1 >= 12)
		{
			area[area_num] = x2 - x1 + 1;
			area_grid_y[area_num] = y;
			area_num++;
		}
	}

	//[寻找狭窄豁口的方向]
	COOR2 narrow_line[100];
	int narrow_line_num = 0;
	memset(narrow_line, 0, 100 * sizeof(COOR2));
	for (int y = g_hd_grid_center.y; y <= g_hd_grid_center.y + 44; y++)
	{
		int x = g_hd_grid_center.x;
		if (g_hd_closed_grid_map[y][x] >= 4)
		{
			break;
		}

		int x1, x2;
		for (x1 = x; x1 >= x - 44; x1--)//[550cm]
		{
			if (g_hd_closed_grid_map[y][x1] >= 4)
			{
				x1++;
				break;
			}
		}
		for (x2 = x; x2 <= x + 44; x2++)//[550cm]
		{
			if (g_hd_closed_grid_map[y][x2] >= 4)
			{
				x2--;
				break;
			}
		}

		if (x2 - x1 + 1 >= 12 && x2 - x1 + 1 <= 23)
		{
			narrow_line[narrow_line_num].x = (INT32)(((x1 + (x2 - x1 + 1) / 2) - g_hd_grid_center.x) * GRID_LEN_PER_CELL_HD);
			narrow_line[narrow_line_num].y = (INT32)((y - g_hd_grid_center.y) * GRID_LEN_PER_CELL_HD);
			narrow_line_num++;
		}
	}

	if (area_num >= 10)
	{
		double count = 0;
		for (int i = 0; i < area_num;i++)
		{
			if (area[i] <= 23)
			{
				count++;
			}
		}

		double percent = count / area_num;
		if (percent > 0.50)
		{
			ret = 1;
		}
		else if (percent > 0)
		{
			int idx = 0;
			for (int i = area_num-1; i >=0;i--)
			{
				if (area[i] <= 23)
				{
					idx = i;
					break;
				}
			}
			int x = g_hd_grid_center.x;
			int y = area_grid_y[idx];
			int x1, x2;
			int num1, num2;
			num1 = 0;
			num2 = 0;
			for (x1 = x; x1 >= x - 44; x1--)//[550cm]
			{
				if (g_hd_closed_grid_map[y][x1] >= 4)
				{
					x1++;
					break;
				}
				else
				{
					num1++;
				}
			}
			for (x2 = x; x2 <= x + 44; x2++)//[550cm]
			{
				if (g_hd_closed_grid_map[y][x2] >= 4)
				{
					x2--;
					break;
				}
				else
				{
					num2++;
				}
			}

			if (narrow_line_num < 6)
			{
				if (num1 > num2)
				{
					mode2_mid_pt.x = x - (num1 - num2) / 2;
					mode2_mid_pt.y = y;
				}
				else
				{
					mode2_mid_pt.x = x + (num2 - num1) / 2;
					mode2_mid_pt.y = y;
				}
			}
			else
			{
				double k = 0;
				double b = 0;
				int result = PcaProcess(narrow_line, narrow_line_num, k, b);
				if (result == -1 && k == 0)
				{
					mode2_mid_pt.x = g_hd_grid_center.x;
					mode2_mid_pt.y = (INT32)(630 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.y);
				}
				else
				{
					mode2_mid_pt.y = (INT32)(630 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.y);
					mode2_mid_pt.x = (INT32)(630 / k / GRID_LEN_PER_CELL_HD + g_hd_grid_center.x);
				}
			}

			ret = 2;
		}
	}

	return ret;
}

int hd_narrow_area_mid_plan()
{
	int ret = 0;

	double x_step;
	double y_step;
	double x = (mode2_mid_pt.x - g_hd_grid_center.x) * GRID_LEN_PER_CELL_HD;
	double y = (mode2_mid_pt.y - g_hd_grid_center.y) * GRID_LEN_PER_CELL_HD;


	if (y < 625)
	{
		if (x == 0)
		{
			y = 625;
		}
		else
		{
			x = (625 - y) * x / y + x;
			y = 625;
		}
	}

	x_step = x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
	y_step = y / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
	g_mid_line[0].x = 0;
	g_mid_line[0].y = 0;
	for (int i = 1; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
	{
		g_mid_line[i].x = roundf2i(i * x_step);
		g_mid_line[i].y = roundf2i(i * y_step);
	}


	return ret;
}

int get_astar_mid_path(COOR2 *astar_path, int &astar_path_num)
{
	int ret = 0;
	int pt_index = astar_path_num - 1;
	COOR2 cur_pt;
	int l_dist, r_dist;
	int grid_x, grid_y;

	//[搜索线居中处理]
	for (int i = 0; i < astar_path_num;i++)
	{
		l_dist = 0;
		r_dist = 0;
		cur_pt = astar_path[i];
		grid_y = cur_pt.y;
		if (cur_pt.y == g_grid_center.y + 40)
			cur_pt.y = cur_pt.y;
		for (int j = 0; j <= 8; j++)//[向左搜索9栅格]
		{
			grid_x = cur_pt.x - j;
			if (g_closed_grid_map[grid_y][grid_x] > 0)
				break;
			else
				l_dist++;
		}
		for (int j = 0; j <= 8; j++)//[向右搜索9栅格]
		{
			grid_x = cur_pt.x + j;
			if (g_closed_grid_map[grid_y][grid_x] > 0)
				break;
			else
				r_dist++;
		}

		//[中间栅格被重复运算所以减1]
		//[栅格被膨胀了1m，因此为了保证车通过，预留0.5m的空间，即2个栅格，再放宽一个栅格即3个]
		if (l_dist + r_dist - 1 < 3)
		{//[说明该栅格不可通行]
			if (cur_pt.y < g_grid_center.y + 24)//[小于6m]不可走
			{
				ret = -1;
				return ret;
			}
			else
			{
				astar_path_num = i + 1;
				return ret;
			}
		}
		else
		{//[可通行，则调整至居中]
			if (l_dist + r_dist - 1 < 13)//[小于5m宽度保持居中]
			{
				if (l_dist > r_dist)
					cur_pt.x = cur_pt.x - (l_dist - r_dist) / 2;
				else
					cur_pt.x = cur_pt.x + (r_dist - l_dist) / 2;
			}
			else
			{//[可通行区域太宽了，保证离障碍物1.5m]
				//[不可能同时小于1]
				int offset = 0;
				if (l_dist < 2)
				{
					offset = 2 - l_dist;
					cur_pt.x = cur_pt.x + offset;
				}
				else if (r_dist < 2)
				{
					offset = 2 - r_dist;
					cur_pt.x = cur_pt.x - offset;
				}
			}
			astar_path[i] = cur_pt;
		}
	}

	return ret;
}

int get_optimize_path(COOR2 *astar_path, int &astar_path_num)
{
	int ret = 0;
	//[降采样]
	if (astar_path_num > NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
	{
		int step = astar_path_num / 19;
		for (int i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			if (i * step >= astar_path_num)
			{
				g_mid_line[i].x = (astar_path[astar_path_num - 1].x - g_grid_center.x) * GRID_LEN_PER_CELL;
				g_mid_line[i].y = (astar_path[astar_path_num - 1].y - g_grid_center.y) * GRID_LEN_PER_CELL;
				break;
			}
			g_mid_line[i].x = (astar_path[i * step].x - g_grid_center.x) * GRID_LEN_PER_CELL;
			g_mid_line[i].y = (astar_path[i * step].y - g_grid_center.y) * GRID_LEN_PER_CELL;
		}
		g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
	}
	else
	{
		for (int i = 0; i < astar_path_num; i++)
		{
			g_mid_line[i].x = (astar_path[i].x - g_grid_center.x) * GRID_LEN_PER_CELL;
			g_mid_line[i].y = (astar_path[i].y - g_grid_center.y) * GRID_LEN_PER_CELL;
		}
		g_long_line_num = astar_path_num;
	}

	
	//[进行优化]
	int sign;

	COOR2 temp_line[200];
	int temp_line_num = 0;
	COOR2 temp_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int temp_mid_line_num = 0;

	memset(temp_line, 0, 200 * sizeof(COOR2));
	temp_line_num = 0;

	g_mid_line[0].x = 0;
	g_mid_line[0].y = 0;
	for (int i = g_long_line_num - 1; i > 0; i--)
	{
		memset(temp_mid_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		temp_mid_line_num = 0;
		temp_mid_line[0] = g_mid_line[0];
		memcpy(temp_mid_line + 1, g_mid_line + i, (g_long_line_num - i) * sizeof(COOR2));
		temp_mid_line_num = 1 + (g_long_line_num - i);
// 		memcpy(temp_mid_line + 1, g_mid_line + i, sizeof(COOR2));
// 		temp_mid_line_num = 2;
		int step = (int)(temp_mid_line[temp_mid_line_num - 1].y - temp_mid_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
		line_fitting(temp_mid_line, temp_mid_line_num, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memcpy(temp_mid_line, temp_line, temp_line_num * sizeof(COOR2));
		temp_mid_line_num = temp_line_num;

		sign = 1;
		int id;
		for (int j = 0; j < i; j++)
		{
			id = j;
			int grid_x = temp_mid_line[j].x / GRID_LEN_PER_CELL + g_grid_center.x;
			int grid_y = temp_mid_line[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
			if (g_closed_grid_map[grid_y][grid_x] > 0 || \
				g_closed_grid_map[grid_y][grid_x - 1] > 0 || \
				g_closed_grid_map[grid_y][grid_x - 2] > 0 || \
				g_closed_grid_map[grid_y][grid_x + 1] > 0 || \
				g_closed_grid_map[grid_y][grid_x + 2] > 0)
			{
				sign = 0;
				break;
			}
		}

		if (sign == 1)
		{
			//memcpy(g_mid_line, temp_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			//memcpy(g_mid_line, temp_mid_line, i * sizeof(COOR2));
			step = (int)(temp_mid_line[id].y - temp_mid_line[0].y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
			line_fitting(temp_mid_line, id + 1, temp_line, temp_line_num, step, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			memcpy(g_mid_line, temp_line, temp_line_num * sizeof(COOR2));
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			break;
		}
	}

	return ret;
}



//[设定状态]
static int g_status_var = 0;
void set_zgcc_plan_status(int var)
{
	g_status_var = var;
}
int get_zgcc_plan_status()
{
	ZG_PLAN_STATUS zg_plan_status;

	if (g_stat == S_INVALID || \
		g_stat == S_WAIT || \
		g_stat == S_WAIT_LIGHT || \
		g_status_var == 1)
	{
		zg_plan_status = ZG_STOP;
	}
	else if (g_navi_state == CHANGE_TO_LEFT_LANE || \
		g_navi_state == CHANGE_TO_RIGHT_LANE)
	{
		zg_plan_status = ZG_CHANGE_LANE;
	}
	else if (g_navi_state == TAKEOVER_FROM_LEFT || \
		g_navi_state == TAKEOVER_FROM_RIGHT)
	{
		zg_plan_status = ZG_OVERTAKE;
	}
	else if (g_is_s_obs == 1)
	{
		zg_plan_status = ZG_SOBS;
	}
	else if (g_status_var == 2)
	{
		zg_plan_status = ZG_REVERSE;
	}
	else
	{
		zg_plan_status = ZG_NORMAL;
	}

	return zg_plan_status;
}



#include <ctime>
int pl_road_trace_interface(int frame_id, PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data)
{
	int ret = 0;

	g_frame_id = frame_id;
#ifdef MBUG_OPEN_
	MBUG("                 PL ROAD TRACE INTERFACE  ID   [%d] :                 \n", g_frame_id);
	MBUG("fu_id : %d\n", g_fu_frame_id);
	MBUG("GPS spd : %d\n", pl_input->state.pos.spd);
	MBUG("fu input GPS g_x = %d g_y = %d\n", pl_input->fu_pl_data.state.pos.ins_coord.x, pl_input->fu_pl_data.state.pos.ins_coord.y);
	MBUG("fu input GPS x = %d y = %d\n", pl_input->fu_pl_data.state.pos.com_coord.x, pl_input->fu_pl_data.state.pos.com_coord.y);
	MBUG("pl input GPS g_x = %d g_y = %d\n", pl_input->state.pos.ins_coord.x, pl_input->state.pos.ins_coord.y);
	MBUG("pl input GPS x = %d y = %d\n", pl_input->state.pos.com_coord.x, pl_input->state.pos.com_coord.y);
	MBUG("pl input left speedmeter = %d\n", pl_input->odo_info.left);
	MBUG("pl input right speedmeter = %d\n", pl_input->odo_info.right);
	MBUG("pl input spd = %d\n", pl_input->state.pos.spd);
#endif



	g_fu_frame_id = pl_input->fu_pl_data.header.id;

	if (g_car_identity == TOYOTA)
	{
		//[通过里程计计算速度]
		ret = get_speed_from_odo(pl_input);

		if (g_spd_error_detect_switch == 0)
		{
			ret = 0;
		}

		if (ret == -1)
			g_speed_feedback_error_flag = 1;
		if (g_speed_feedback_error_flag == 1)
		{
			for (int ii = 0; ii < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; ii++)
			{
				pl_cs_data->path[ii].x = 0;
				pl_cs_data->path[ii].y = ii * 100;
			}
			pl_cs_data->id = g_frame_id;
			pl_cs_data->isok = 1;
			pl_cs_data->number_of_effective_points = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			pl_cs_data->sys_command = 0x10 | 0x20 | 0x04 | 0x08;

			pl_cs_data->speed = 0;
			memcpy(&pl_cs_data->state, &pl_input->state, sizeof(STATE));
			process_pl_cs_data(pl_cs_data);
#ifdef MBUG_OPEN_
			MBUG("ODD Data is Error!!!\n");
			MBUG("PL ROAD TRACE INTERFACE END ---------------\n");
#endif

			set_zgcc_plan_status(1);
			return 0;
		}

		//g_real_speed = pl_input->state.pos.spd;//[获取实时速度]

	}
	else if (g_car_identity == ALV3)
	{
		pl_input->state.pos.spd = pl_input->cs_pl_data.state.pos.spd;

		g_real_speed = pl_input->state.pos.spd;//[获取实时速度]
	}

	//[****TEST****]
	COOR2 pt;
	pt.x = pl_input->gp_info.tps[1].x;
	pt.y = pl_input->gp_info.tps[1].y;
	coor2_e2v(&pl_input->state.pos, &pt, &g_cur_road_pt);//[用于显示当前路点]
	//[****TEST****]

	STATUS status = (STATUS)pl_input->state.status;
	g_stat = getStatus(&status);
#ifdef MBUG_OPEN_
	MBUG("g_stat = %d\n", g_stat);
#endif

	//[这两个状态下，停车]
	if (g_stat == S_INVALID || \
		g_stat == S_WAIT || \
		g_stat == S_WAIT_LIGHT)
	{
		set_stop_plan(pl_input, pl_cs_data, 0);
		process_pl_cs_data(pl_cs_data);

#ifdef MBUG_OPEN_
		MBUG("PL ROAD TRACE INTERFACE END ---------------\n");
#endif
		set_zgcc_plan_status(1);
		return 0;
	}


	ret = -1;
	g_fidelity = 0;
	g_pitch = (int)((double)pl_input->state.pos.pitch / 100000000.0 * 180 / PI + 0.5f);
	g_roll = (int)((double)pl_input->state.pos.roll / 100000000.0 * 180 / PI + 0.5f);
	g_yaw = (int)((double)pl_input->state.pos.yaw / 100000000.0 * 180 / PI + 0.5f);
	pl_cs_data->sys_command = 0;


	//[补出残缺的结构化道路]
	//fix_the_lost_structual_line(pl_input);

	//[读取栅格数据]
	get_gridmap(pl_input);
	get_hd_gridmap(pl_input);

	if (g_64lidar_switch == 0)
	{
#ifdef MBUG_OPEN_
		MBUG("g_64lidar_switch : Turn Off !!!!\n");
#endif
		memset(g_grid_map, 0, sizeof(int) * GRID_WIDTH * GRID_HEIGHT);
		memset(g_grid_map, 0, sizeof(int) * GRID_WIDTH_HD * GRID_HEIGHT_HD);
	}

	//[融合有BUG超出20个点]
	for (int i = 0; i < pl_input->fu_pl_data.lines.valid_num_in_l_wing; i++)
	{
		if (pl_input->fu_pl_data.lines.l_edges[i].valid_num_points > NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		{
			pl_input->fu_pl_data.lines.l_edges[i].valid_num_points = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
		}
	}
	for (int i = 0; i < pl_input->fu_pl_data.lines.valid_num_in_r_wing; i++)
	{
		if (pl_input->fu_pl_data.lines.r_edges[i].valid_num_points > NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		{
			pl_input->fu_pl_data.lines.r_edges[i].valid_num_points = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
		}
	}
	//[读取动态障碍物]
	keep_dyn_obs(pl_input);

	//[动态障碍物填写到栅格]
	fill_dyn_obs_to_grid();

	//[读取自然道路]
	keep_natural_road(pl_input);

	if (g_road_type == 1)
	{//[乡村模式下检测结构化道路长度]
		get_fusion_structure_road_length();
	}
	else
	{
		get_fusion_road_length();
	}

	//[倒车规划]
	reversePlan.MaintainPts(pl_input->state.pos);
	reversePlan.ReverseTest(pl_input->state.pos);
 	g_reverse_pts_show.clear();
	for (int i=0; i<(int)reversePlan.m_reverse_pts.size(); i++)
	{
		COOR2 l_reverse_pt;
		coor2_e2v(&pl_input->state.pos, &reversePlan.m_reverse_pts[i], &l_reverse_pt);
		g_reverse_pts_show.push_back(l_reverse_pt);
	}
	if (reversePlan.GetReverseFlag())
	{
		//grid_operation_circle(g_grid_map, GRID_WIDTH, GRID_HEIGHT, g_closed_grid_map, 4);
		g_navi_state = REVERSE_DIRECT_BACK;
		grid_operation_include_tail(g_grid_map, g_closed_grid_map);
		ret = reversePlan.ReversePlan_DirectBack(pl_input->state.pos,
										pl_input->state, g_closed_grid_map, 
										g_grid_center,
										g_frame_id, 
										pl_cs_data);
		if (ret == -1)
		{
			g_navi_state = TRACE_LANE;
			reversePlan.SetReverseFlag(false);
		}
#ifdef MBUG_OPEN_
		MBUG("Can't Recerse  !!!!!!!!*******************\r\rn");
		MBUG("the Process_Plan finish ---------------\n\n\n\n\n");
#endif

		process_pl_cs_data(pl_cs_data);
		set_zgcc_plan_status(2);
		return 0;
	}


	if (g_roving_switch == 1)
	{//[开启漫游后，强行设置状态以及速度]
		g_stat = S_ROAD_NAV;
		g_navi_state = ROVING;
		g_max_speed = (int)(g_roving_speed * CM);
	}


	ret = -1;
	//int res_grid[GRID_HEIGHT][GRID_WIDTH];
	//grid_close_operation(g_grid_map, 2, 2, res_grid);

	if (g_s_obs_detect_switch == 0)
	{
		ret = -1;
	}
	else
	{
		hd_grid_close_operation_circle(g_hd_grid_map, 7, 7, g_hd_closed_grid_map);
		ret = hd_check_s_obs(g_hd_closed_grid_map, g_s_obs_plan_line, &g_s_obs_plan_line_num);
	}

	if (g_navi_state == TAKEOVER_FROM_LEFT || \
		g_navi_state == TAKEOVER_FROM_RIGHT)
	{
		ret = -1;
	}


//	ret = -1;
// 	if (ret == -1)
// 		fprintf(stderr, "S OBS OFF\n");
// 	else
// 		fprintf(stderr, "S OBS ON\n");

	if (g_is_s_obs == 0)
	{
		if (ret == 0)
		{
			g_is_s_obs_confirm_time++;
		}
		else
		{
			g_is_s_obs_confirm_time = 0;
		}

		if (g_is_s_obs_confirm_time >= 3)
		{
			fprintf(stderr, "-----  S OBS ON -----\n");
			//g_s_obs_quit_timer = 45;
			g_s_quit_pt.x = pl_input->state.pos.ins_coord.x;
			g_s_quit_pt.y = pl_input->state.pos.ins_coord.y;
			g_is_s_obs = 1;
			g_is_s_obs_confirm_time = 0;
			g_s_obs_last_angle = -1;
			g_s_obs_speed_down = 0;
		}
	}
	else
	{
		if (ret == 0)
		{
			//g_s_obs_quit_timer = 45;
			g_s_quit_pt.x = pl_input->state.pos.ins_coord.x;
			g_s_quit_pt.y = pl_input->state.pos.ins_coord.y;
			g_is_s_obs_confirm_time = 0;
			g_s_obs_quit = 0;
		}
		else
		{
			g_is_s_obs_confirm_time--;
		}

		if (g_is_s_obs_confirm_time <= -3)
		{
			//g_is_s_obs = 0;
			g_is_s_obs_confirm_time = 0;
			g_s_obs_quit = 1;
		}
	}

	if (g_s_obs_quit == 1)
	{
		//g_s_obs_quit_timer--;
		g_e_quit_pt.x = pl_input->state.pos.ins_coord.x;
		g_e_quit_pt.y = pl_input->state.pos.ins_coord.y;

		if (dist_point(&g_s_quit_pt, &g_e_quit_pt) > 800)
		{
			g_s_obs_quit = 0;
			g_is_s_obs = 0;
			g_s_obs_last_angle = -1;
			g_s_obs_speed_down = 0;
		}
	}

	if (g_is_s_obs != 1)
	{
		memset(g_s_obs_avg_angle, 0, S_OBS_AVG_NUM * sizeof(double));
		g_s_obs_avg_angle_num = 0;
		g_s_obs_avg_index = 0;
		g_s_obs_crash_flag = 0;
	}

	if (g_stat == S_LEFT || \
		g_stat == S_RIGHT || \
		g_stat == S_STRAIGHT || \
		g_stat == S_UTURN || \
		g_stat == S_CROSS_UND ||\
		g_stat == S_TASK_OVER)
	{//[S弯不可能出现在这些状态]
		g_is_s_obs = 0;
		g_is_s_obs_confirm_time = 0;
		g_is_not_s_obs_confirm_time = 0;
		g_s_obs_quit = 0;
		g_s_obs_quit_timer = 0;
		g_s_obs_speed_down = 0;
		g_s_obs_last_angle = -1;
		memset(&g_s_quit_pt, 0, sizeof(COOR2));
		memset(&g_e_quit_pt, 0, sizeof(COOR2));
	}


	//[该状态下检测停止线]
	if (g_stat == S_ROAD_NAV || \
		g_stat == S_CROSS_UND || \
		g_stat == S_TASK_OVER || \
		g_stat == S_PARKING || \
		g_stat == S_LEFT || \
		g_stat == S_RIGHT || \
		g_stat == S_STRAIGHT)//[乡村道路下，检测到限速，融合以停止线信息发过来进行减速]
	{
		if (pl_input->fu_pl_data.stop_lines.stop_lines[0].start.x == 0 && \
			pl_input->fu_pl_data.stop_lines.stop_lines[0].start.y == 500 && \
			pl_input->fu_pl_data.stop_lines.stop_lines[0].end.x == 0 && \
			pl_input->fu_pl_data.stop_lines.stop_lines[0].end.y == 500)
		{
			//[和融合约定，此情况到达停止线]
#ifdef MBUG_OPEN_
			MBUG("------------- Stop Line Done --------------\n");
#endif
			g_is_stopping = 1;
		}
		else
		{
#ifdef MBUG_OPEN_
			MBUG("------------- Stop Line Tracing --------------\n");
#endif
			if (g_stat != S_TASK_OVER)
				g_is_stopping = 0;
		}
	}
	else
	{
		g_is_stopping = 0;
	}

	if (g_stat == S_CROSS_UND || \
		g_stat == S_ROAD_NAV)
	{//[路口理解下，会获得限速]
		int a, b, c, d;
		a = pl_input->fu_pl_data.stop_lines.stop_lines[0].start.x;
		b = pl_input->fu_pl_data.stop_lines.stop_lines[0].start.y;
		c = pl_input->fu_pl_data.stop_lines.stop_lines[0].end.x;
		d = pl_input->fu_pl_data.stop_lines.stop_lines[0].end.y;
		if (a > 0 && a == b && b == c && c == d && \
			pl_input->fu_pl_data.stop_lines.valid_line_num == 1)
		{
#ifdef MBUG_OPEN_
			MBUG("------------- limit speed ON--------------\n");
#endif
			g_speed_limit_sign = 1;
			g_speed_limi_pl = a;
		}
	}
	
	int a, b, c, d;
	a = pl_input->fu_pl_data.stop_lines.stop_lines[0].start.x;
	b = pl_input->fu_pl_data.stop_lines.stop_lines[0].start.y;
	c = pl_input->fu_pl_data.stop_lines.stop_lines[0].end.x;
	d = pl_input->fu_pl_data.stop_lines.stop_lines[0].end.y;
	if (a < 0 && a == b && b == c && c == d && \
		pl_input->fu_pl_data.stop_lines.valid_line_num == 1)
	{//[四个同样的负值解除限速]
#ifdef MBUG_OPEN_
		MBUG("------------- limit speed OFF --------------\n");
#endif
		g_speed_limit_sign = 0;
	}

	if (g_stat != S_CROSS_UND)
	{
		g_cross_und_speed_flag = 0;
	}
	//[根据状态初次设置速度]
#ifdef MBUG_OPEN_
	MBUG("--------------- Set the speed ----------------------\n");
#endif

	if (g_stat == S_ROAD_NAV ||
		g_stat == S_CROSS_UND ||
		g_stat == S_STRAIGHT ||
		g_stat == S_LEFT ||
		g_stat == S_RIGHT ||
		g_stat == S_UTURN ||
		g_stat == S_TASK_OVER)
	{
		g_roving_speed = 20;

		if (g_stat == S_ROAD_NAV)
		{
			if (g_max_speed > 0)
			{
				g_wp_speed = g_max_speed;
			}
			else
			{
				g_wp_speed = (int)(25 * CM);
			}

			if (g_navi_state == D_EMERGENCY_STOP)
			{//[使用低速，等待探索]
				g_wp_speed = (int)(10 * CM);
			}
			else if (g_navi_state == TRACE_IN_NATURAL_ROAD)
			{
				if (g_car_identity == ALV3)
				{
					g_wp_speed = (int)(21 * CM);
				}
				else
				{
					g_wp_speed = g_max_speed;
				}
				
			}

			if (g_is_stopping == 1)
			{
				g_wp_speed = 0;
			}
		}
		else if (g_stat == S_CROSS_UND)
		{
			if (g_is_stopping == 1)
			{
				g_wp_speed = 0;
			}
			else
			{
				if (g_cross_und_speed_flag == 0)
				{
					/*g_wp_speed = (int)(g_cross_und_speed * CM);*/
					switch (pl_input->gp_info.tps[1].direction)//pl_input->rbx_taskpoint.taskpoints[1].direction)
					{//[由于总控可能撞点，所以不能继续读取路点属性]
					case 0://[Unknow]
					case 1://[直行];
						g_wp_speed = (int)(g_cross_und_straight_speed * CM);
						break;
					case 2://[右拐]
						g_wp_speed = (int)(g_cross_und_right_speed * CM);
						break;
					case 3://[左拐]
						g_wp_speed = (int)(g_cross_und_left_speed * CM);
						break;
					case 4://[Uturn]
						g_wp_speed = (int)(g_cross_und_uturn_speed * CM);
						break;
					case 5:
						g_wp_speed = (int)(g_cross_und_traffic_sign_speed * CM);
					default:
						break;
					}

					g_cross_und_speed = g_wp_speed;//[路口理解取出速度，为以后使用]
					g_cross_und_speed_flag = 1;
				}
				else
				{
					g_wp_speed = g_cross_und_speed;
					if (g_cross_und_slow_down_flag)
					{
						g_wp_speed = (int)(g_cross_und_slow_down_speed * CM);
					}
				}
				
			}
		}
		else if (g_stat == S_STRAIGHT ||
			g_stat == S_LEFT ||
			g_stat == S_RIGHT ||
			g_stat == S_UTURN)
		{
			if (g_is_stopping == 1)
			{
				g_wp_speed = 0;
			}
			else
			{
				if (g_stat == S_STRAIGHT)
					g_wp_speed = (int)(g_cross_straight_speed * CM);
				else if (g_stat == S_LEFT)
					g_wp_speed = (int)(g_cross_left_turn_speed * CM);
				else if (g_stat == S_RIGHT)
					g_wp_speed = (int)(g_cross_right_turn_speed * CM);
				else if (g_stat == S_UTURN)
					g_wp_speed = (int)(g_cross_uturn_speed * CM);
			}
			//g_wp_speed = (int)(8 * CM);
		}
		else if (g_stat == S_TASK_OVER)
		{
			COOR2 pt1;
			COOR2 pt2;
			pt1.x = pl_input->gp_info.tps[1].x;//pl_input->rbx_taskpoint.taskpoints[1].x;
			pt1.y = pl_input->gp_info.tps[1].y;//pl_input->rbx_taskpoint.taskpoints[1].y;
			pt2.x = pl_input->state.pos.com_coord.x;
			pt2.y = pl_input->state.pos.com_coord.y;

			//[根据到最后目标点的距离去不同速度]
			double dist = dist_point(&pt1, &pt2);
			if (dist < 10000 && dist >= 6000)
			{
				g_wp_speed = (int)(20 * CM);
			}
			else if (dist > 4000 && dist < 6000)
			{
				g_wp_speed = (int)(10 * CM);
			}
			else
			{
				g_wp_speed = (int)(5 * CM);
			}
		}
	}

#ifdef MBUG_OPEN_
	MBUG("g_wp_speed : %f\n", g_wp_speed * 3.6 / 100);
	MBUG("g_max_speed : %f\n", g_max_speed * 3.6 / 100);
	MBUG("g_cur_speed : %f\n", g_cur_speed * 3.6 / 100);
#endif
	//[根据速度设置换道的预瞄角度以及安全距离]
	int speed = (int)(g_real_speed * 3.6 / 100);
	if (speed <= 10)
	{
		g_change_steer_limit = g_10km_change_steer;
		g_safe_dist_to_follow_car = 1000;
	}
	if (speed > 10 && speed <= 20)
	{
		g_change_steer_limit = g_20km_change_steer;
		g_safe_dist_to_follow_car = 2000;
	}
	else if (speed > 20 && speed <= 30)
	{
		g_change_steer_limit = g_30km_change_steer;
		g_safe_dist_to_follow_car = 3000;
	}
	else if (speed > 30 && speed <= 40)
	{
		g_change_steer_limit = g_40km_change_steer;
		g_safe_dist_to_follow_car = 4000;
	}
	else if (speed > 40 && speed <= 50)
	{
		g_change_steer_limit = g_50km_change_steer;
		g_safe_dist_to_follow_car = 5000;
	}
	else if (speed > 50)
	{
		g_change_steer_limit = g_60km_change_steer;
		g_safe_dist_to_follow_car = 6000;
	}

#ifdef MBUG_OPEN_
	MBUG("limit - speed : %d\n", speed);
#endif

	//[规划核心部分]
	int i;// , j, k;
	memset(g_mid_line, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);

	if (g_stat != S_LEFT && 
		g_stat != S_RIGHT && 
		g_stat != S_UTURN && 
		g_stat != S_STRAIGHT)
	{
		reset_cross_flag();
	}

	if (g_stat != S_CROSS_UND)
	{
		g_cross_adjust_count = 0;
		g_cross_adjust_flag = 0;
		g_cross_und_state = CROSS_TRACE;
		g_cross_und_slow_down_flag = 0;
	}

	if (g_stat != S_ROAD_NAV)
	{
		g_navi_state = TRACE_LANE;

		g_reverse_start_timer = 0;


		g_emergency_roving_timer = 0;
		g_emergency_counter = 0;

		//[超车状态清零]
		g_take_over_flag = 0;
		g_take_over_speed_up_timer = 0;
		g_take_over_back_lane_flag = -1;

		//[换道状态清零]
		g_change_lane_count = 0;
		g_change_lane_see = 0;
		g_change_fade_timer = 0;
		g_change_mode = -1;
		g_change_lane_unsafe_dist = MAX_VALUE;
		g_change_lane_direct_keep = -1;
		g_change_lane_search_flag = 0;
		g_change_lane_search_dist = MAX_VALUE;

		g_vibration_count = 0;
		g_vibration_flag = 0;
		g_slow_down_dist = MAX_VALUE;
	}

	//[清除平均]
	if ((g_stat != S_ROAD_NAV && g_stat != S_CROSS_UND && g_stat != S_TASK_OVER) \
		|| g_change_fade_timer > 0 \
		|| g_navi_state == D_EMERGENCY_STOP \
		|| g_navi_state == S_EMERGENCY_STOP \
		|| g_navi_state == TRACE_IN_NATURAL_ROAD)
	{
		memset(g_line_avg, 0, LINE_AVG_NUM * sizeof(COOR2));
		g_line_avg_num = 0;
		g_line_avg_index = 0;
		g_line_avg_last_index = 0;
	}

	if (g_navi_state != ROVING)
	{
		reset_cross_country_paragram();
	}


	if (g_stat != S_LEFT && \
		g_stat != S_RIGHT && \
		g_stat != S_UTURN && \
		g_stat != S_STRAIGHT && \
		g_navi_state != ROVING && \
		g_is_s_obs != 1)
	{
		memset(g_cross_avg_angle, 0, CROSS_AVG_NUM * sizeof(double));
		g_cross_avg_index = 0;
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_cross_avg_angle_num = 0;

		memset(g_s_obs_avg_angle, 0, S_OBS_AVG_NUM * sizeof(double));
		g_s_obs_avg_angle_num = 0;
		g_s_obs_avg_index = 0;
	}

	//[此处调用用于清除状态]
	global_task_pt_analyze(pl_input);

	//[规划核心部分]
	if (g_stat == S_ROAD_NAV)
	{
		//[********S_ROAD_NAV状态下，规划内部状态转换********]
		if (g_navi_state == TRACE_LANE)
		{
			keep_multi_lane(pl_input);
			ret = trace_lane(pl_input);
		}
		else if (g_navi_state == FOLLOW_THE_CAR)
		{
			keep_multi_lane(pl_input);
			ret = trace_lane(pl_input);
		}
		else if (g_navi_state == CHANGE_TO_LEFT_LANE)
		{//[左换道]
			keep_multi_lane_in_take_over(pl_input, 0);
			ret = change_lane(pl_input, 0);
		}
		else if (g_navi_state == CHANGE_TO_RIGHT_LANE)
		{//[右换道]
			keep_multi_lane_in_take_over(pl_input, 1);
			ret = change_lane(pl_input, 1);
		}
		else if (g_navi_state == D_EMERGENCY_STOP || g_navi_state == S_EMERGENCY_STOP)
		{
			keep_multi_lane(pl_input);
			ret = trace_lane(pl_input);
		}
		else if (g_navi_state == TRACE_IN_NATURAL_ROAD)
		{
			keep_multi_lane(pl_input);
			ret = trace_in_natural_road();
		}
		else if (g_navi_state == ROVING)
		{
			keep_multi_lane(pl_input);
			ret = trace_lane(pl_input);
		}

		//[********S_ROAD_NAV状态下，规划内部状态转换********]



		//[@@以下会针对特殊要求进行规划内部状态的强制设定]
		if (g_roving_switch == 1)
		{//[开启漫游后，强行设置状态以及速度]
			g_navi_state = ROVING;
		}

		global_task_pt_analyze(pl_input);

		if (g_trace_global_task_pt_flag > 0)
			g_navi_state = ROVING;

		//[根据不同状态生成规划线]
		if (g_navi_state == TRACE_LANE)
		{
			if (ret == 0)
			{
				get_trace_lane_path(0);
			}
			else
			{
				g_fidelity = 0;
#ifdef MBUG_OPEN_
				MBUG("trace lane g_fidelity : %d\n", g_fidelity);
#endif
				get_trace_lane_path(1);
			}
		}
		else if (g_navi_state == FOLLOW_THE_CAR)
		{
			int i;
			COOR2 left_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
			COOR2 right_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
			int left_line_num, right_line_num;
			left_line_num = 0;
			right_line_num = 0;
			//[只用1、2车道线]

			double x1, y1;
			double x2, y2;
			double x;
			int num = 0;
			int points_num = 0;
			x1 = y1 = x2 = y2 = 0;
			double theta = 0;
			double x_step = 0;
			double y_step = 0;

			if (g_multi_lane.lane_line[1].valid_num_points >= 2 && g_multi_lane.lane_line[2].valid_num_points >= 2)
			{
				for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
				{
					left_line[i].x = g_multi_lane.lane_line[1].line[i].x;
					left_line[i].y = g_multi_lane.lane_line[1].line[i].y;
					right_line[i].x = g_multi_lane.lane_line[2].line[i].x;
					right_line[i].y = g_multi_lane.lane_line[2].line[i].y;
				}
				left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			}
			else
			{
				//[根据结构化道路以及自然道边的方向补出道路]
				for (i = 0; i < LANE_LINE_NUM; i++)
				{
					if (g_multi_lane.lane_line[i].valid_num_points >= 2)
					{
						num++;
						points_num = g_multi_lane.lane_line[i].valid_num_points;
						x1 += g_multi_lane.lane_line[i].line[0].x;
						y1 += g_multi_lane.lane_line[i].line[0].y;

						x2 += g_multi_lane.lane_line[i].line[points_num - 1].x;
						y2 += g_multi_lane.lane_line[i].line[points_num - 1].y;
					}
				}

				if (num > 0)
				{//[有结构化道路]
					x1 = x1 / num;
					y1 = y1 / num;
					x2 = x2 / num;
					y2 = y2 / num;

					theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出道路角度]

					//[利用375补出左右道路边]
					if (fabs(theta) > 0.5 && fabs(theta) < 3)
					{//[大于两度使用道路方向]
						x = (x1 - x2 + 0.0) / (y1 - y2) * (FU_ROAD_LENGTH - 400) - 187;
						x_step = (x - (-187) + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							left_line[i].x = (INT32)(-187 + i * x_step);
							left_line[i].y = (INT32)(400 + i * y_step);
						}

						x = (x1 - x2 + 0.0) / (y1 - y2) * (FU_ROAD_LENGTH - 400) + 187;
						x_step = (x - 187 + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							right_line[i].x = (INT32)(187 + i * x_step);
							right_line[i].y = (INT32)(400 + i * y_step);
						}

						left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					}
					else if (fabs(theta) <= 0.5)
					{
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							left_line[i].x = -187;
							left_line[i].y = (INT32)(400 + i * y_step);
							right_line[i].x = 187;
							right_line[i].y = (INT32)(400 + i * y_step);
						}

						left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					}
					else
					{
						if (theta < 0)
						{
							theta = -3;
						}
						else
						{
							theta = 3;
						}
						x = tan(theta * PI / 180) * (FU_ROAD_LENGTH - 400) - 187;
						x_step = (x - (-187) + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							left_line[i].x = (INT32)(-187 + i * x_step);
							left_line[i].y = (INT32)(400 + i * y_step);
						}

						x = tan(theta * PI / 180) * (FU_ROAD_LENGTH - 400) + 187;
						x_step = (x - 187 + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							right_line[i].x = (INT32)(187 + i * x_step);
							right_line[i].y = (INT32)(400 + i * y_step);
						}

						left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					}
				}
				else if (g_natural_boundary.l_nums >= 2 || g_natural_boundary.r_nums >= 2)
				{//[没有结构化道路，使用自然道边]

					double l_theta = 0;
					double r_theta = 0;
					//[自然道边使用斜率最小的一个最为参考]
					if (g_natural_boundary.l_nums >= 2 && g_natural_boundary.r_nums >= 2)
					{
						x1 = g_natural_boundary.l_boundary[0].x;
						y1 = g_natural_boundary.l_boundary[0].y;
						x2 = g_natural_boundary.l_boundary[g_natural_boundary.l_nums - 1].x;
						y2 = g_natural_boundary.l_boundary[g_natural_boundary.l_nums - 1].y;
						l_theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出道路角度]

						x1 = g_natural_boundary.r_boundary[0].x;
						y1 = g_natural_boundary.r_boundary[0].y;
						x2 = g_natural_boundary.r_boundary[g_natural_boundary.r_nums - 1].x;
						y2 = g_natural_boundary.r_boundary[g_natural_boundary.r_nums - 1].y;
						r_theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出道路角度]

						if (fabs(l_theta) < fabs(r_theta))
						{
							x1 = g_natural_boundary.l_boundary[0].x;
							y1 = g_natural_boundary.l_boundary[0].y;
							x2 = g_natural_boundary.l_boundary[g_natural_boundary.l_nums - 1].x;
							y2 = g_natural_boundary.l_boundary[g_natural_boundary.l_nums - 1].y;
							theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出道路角度]
						}
						else
						{
							x1 = g_natural_boundary.r_boundary[0].x;
							y1 = g_natural_boundary.r_boundary[0].y;
							x2 = g_natural_boundary.r_boundary[g_natural_boundary.r_nums - 1].x;
							y2 = g_natural_boundary.r_boundary[g_natural_boundary.r_nums - 1].y;
							theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出道路角度]
						}
					}
					else if (g_natural_boundary.l_nums >= 2)
					{
						x1 = g_natural_boundary.l_boundary[0].x;
						y1 = g_natural_boundary.l_boundary[0].y;
						x2 = g_natural_boundary.l_boundary[g_natural_boundary.l_nums - 1].x;
						y2 = g_natural_boundary.l_boundary[g_natural_boundary.l_nums - 1].y;
						theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出道路角度]
					}
					else
					{
						x1 = g_natural_boundary.r_boundary[0].x;
						y1 = g_natural_boundary.r_boundary[0].y;
						x2 = g_natural_boundary.r_boundary[g_natural_boundary.r_nums - 1].x;
						y2 = g_natural_boundary.r_boundary[g_natural_boundary.r_nums - 1].y;
						theta = atan((x1 - x2 + 0.0) / (y1 - y2)) * 180 / PI;//[求出道路角度]
					}

					//[利用375补出左右道路边]
					if (fabs(theta) > 0.5 && fabs(theta) < 3)
					{//[0.5-2度内]
						x = (x1 - x2 + 0.0) / (y1 - y2) * (FU_ROAD_LENGTH - 400) - 187;
						x_step = (x - (-187) + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							left_line[i].x = (INT32)(-187 + i * x_step);
							left_line[i].y = (INT32)(400 + i * y_step);
						}

						x = (x1 - x2 + 0.0) / (y1 - y2) * (FU_ROAD_LENGTH - 400) + 187;
						x_step = (x - 187 + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							right_line[i].x = (INT32)(187 + i * x_step);
							right_line[i].y = (INT32)(400 + i * y_step);
						}

						left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					}
					else if (fabs(theta) <= 0.5)
					{
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							left_line[i].x = -187;
							left_line[i].y = (INT32)(400 + i * y_step);
							right_line[i].x = 187;
							right_line[i].y = (INT32)(400 + i * y_step);
						}

						left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					}
					else
					{//[3度以上，以两度变换]
						if (theta < 0)
						{
							theta = -3;
						}
						else
						{
							theta = 3;
						}
						x = tan(theta * PI / 180) * (FU_ROAD_LENGTH - 400) - 187;
						x_step = (x - (-187) + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							left_line[i].x = (INT32)(-187 + i * x_step);
							left_line[i].y = (INT32)(400 + i * y_step);
						}

						x = tan(theta * PI / 180) * (FU_ROAD_LENGTH - 400) + 187;
						x_step = (x - 187 + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
						y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

						for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
						{
							right_line[i].x = (INT32)(187 + i * x_step);
							right_line[i].y = (INT32)(400 + i * y_step);
						}

						left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					}
				}
				else
				{//[没有任何道路信息，使用上一帧的方向]
					//theta = atan((g_last_frame_pt.x + 0.0) / (g_last_frame_pt.y - 400)) * 180 / PI;//[求出道路角度]
					theta = 0;

					x = tan(theta * PI / 180) * (FU_ROAD_LENGTH - 400) - 187;
					x_step = (x - (-187) + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
					y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

					for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
					{
						left_line[i].x = (INT32)(-187 + i * x_step);
						left_line[i].y = (INT32)(400 + i * y_step);
					}

					x = tan(theta * PI / 180) * (FU_ROAD_LENGTH - 400) + 187;
					x_step = (x - 187 + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
					y_step = (FU_ROAD_LENGTH - 400) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);

					for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
					{
						right_line[i].x = (INT32)(187 + i * x_step);
						right_line[i].y = (INT32)(400 + i * y_step);
					}

					left_line_num = right_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
				}
			}

			ret = check_the_lane(left_line, left_line_num, right_line, right_line_num);
			get_trace_lane_path(0);
		}
		else if (g_navi_state == TAKEOVER_FROM_LEFT)
		{//[状态切换以及规划生成在一个函数里面]
			//keep_multi_lane(pl_input);
			keep_multi_lane_in_take_over(pl_input, 0);
			MULTI_LANE l_multi_lane;
			memset(&l_multi_lane, 0, sizeof(MULTI_LANE));
			if (g_change_lane_see == 0 && g_change_lane_count > 0)
			{//[已经在左方车道，现在的1、2、3对应换道前0、1、2]
				for (i = 1; i < 4; i++)
				{
					memcpy(&l_multi_lane.lane_line[i - 1], &g_multi_lane.lane_line[i], sizeof(PL_EDGE));
					l_multi_lane.ok_times[i - 1] = g_multi_lane.ok_times[i];
				}
			}
			else
			{//[换道过程中，主要在初始车道，对于车道的认知还不需要改变]
				for (i = 0; i < 3; i++)
				{
					memcpy(&l_multi_lane.lane_line[i], &g_multi_lane.lane_line[i], sizeof(PL_EDGE));
					l_multi_lane.ok_times[i] = g_multi_lane.ok_times[i];
				}
			}
			int l_status = TRACE_LANE;
			take_over(g_real_speed, g_grid_map, g_dyn_obs, g_dyn_obs_num, &l_multi_lane, 0, g_mid_line, &l_status);
			g_navi_state = (NAVI_STATE)l_status;

			if (g_navi_state == TAKEOVER_FROM_LEFT && g_change_lane_count > 4 && g_multi_lane.lane_line[2].line[0].x > 130)
			{
				g_navi_state = TRACE_LANE;
			}

			if (g_navi_state != TAKEOVER_FROM_LEFT)
			{//[清空一个用于换道保持的标志]
				g_change_lane_count = 0;
				g_change_lane_direct_keep = -1;
			}

			if (g_navi_state != D_EMERGENCY_STOP)
			{
				g_fidelity = 1;
			}
			else
			{
				g_fidelity = 0;
			}

			if (g_navi_state == TRACE_LANE)
			{//[恢复道路跟踪]
				g_take_over_speed_up_timer = g_take_over_speed_up_time;
				//g_take_over_back_lane_flag = 1;//[开启右回道标志]
			}
			else
			{
				g_take_over_back_lane_flag = -1;
			}
		}
		else if (g_navi_state == TAKEOVER_FROM_RIGHT)
		{
			//keep_multi_lane(pl_input);
			keep_multi_lane_in_take_over(pl_input, 1);
			MULTI_LANE l_multi_lane;
			memset(&l_multi_lane, 0, sizeof(MULTI_LANE));
			if (g_change_lane_see == 0 && g_change_lane_count > 0)
			{//[已经在右方车道，现在的0、1、2对应换道前1、2、3]
				for (i = 0; i < 3; i++)
				{
					memcpy(&l_multi_lane.lane_line[i + 1], &g_multi_lane.lane_line[i], sizeof(PL_EDGE));
					l_multi_lane.ok_times[i + 1] = g_multi_lane.ok_times[i];
				}
			}
			else
			{//[换道过程中，主要在初始车道，对于车道的认知还不需要改变]
				for (i = 1; i < 4; i++)
				{
					memcpy(&l_multi_lane.lane_line[i], &g_multi_lane.lane_line[i], sizeof(PL_EDGE));
					l_multi_lane.ok_times[i] = g_multi_lane.ok_times[i];
				}
			}
			int l_status = TRACE_LANE;
			take_over(g_real_speed, g_grid_map, g_dyn_obs, g_dyn_obs_num, &l_multi_lane, 1, g_mid_line, &l_status);

			g_navi_state = (NAVI_STATE)l_status;

			if (g_navi_state == TAKEOVER_FROM_RIGHT && g_change_lane_count > 4 && g_multi_lane.lane_line[1].line[0].x < -130)
			{
				g_navi_state = TRACE_LANE;
			}

			if (g_navi_state != TAKEOVER_FROM_RIGHT)
			{//[清空一个用于换道保持的标志]
				g_change_lane_count = 0;
				g_change_lane_direct_keep = -1;
			}

			if (g_navi_state != D_EMERGENCY_STOP)
			{
				g_fidelity = 1;
				COOR2 temp_pts[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
				int temp_pts_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
				memcpy(temp_pts, g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
				get_bezier_line(temp_pts, temp_pts_num, g_mid_line, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
			}
			else
			{
				g_fidelity = 0;
			}

			if (g_navi_state == TRACE_LANE)
			{//[恢复道路跟踪]
				g_take_over_speed_up_timer = g_take_over_speed_up_time;
				//g_take_over_back_lane_flag = 0;//[开启左回道标志]
			}
			else
			{
				g_take_over_back_lane_flag = -1;
			}
		}
		else if (g_navi_state == CHANGE_TO_LEFT_LANE)
		{
			//[超车道未被堵塞]
			if (ret == 0)
			{
				ret = get_change_lane_path(0);
			}
			else
			{
				memset(g_mid_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
				g_long_line_num = 0;
				g_fidelity = 0;
			}
		}
		else if (g_navi_state == CHANGE_TO_RIGHT_LANE)
		{
			//[超车道未被堵塞]
			if (ret == 0)
			{
				ret = get_change_lane_path(1);
			}
			else
			{
				memset(g_mid_line, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
				g_long_line_num = 0;
				g_fidelity = 0;
			}
		}
		else if (g_navi_state == TRACE_IN_NATURAL_ROAD)
		{
			get_trace_lane_in_natural_path();
			g_fidelity = 1;
		}
		else if (g_navi_state == ROVING)
		{
			//[膨胀1m]
			grid_operation(g_grid_map, g_closed_grid_map, 4);
			//[栅格虚拟单线雷达]
			org_grid2LMS();
			//[虚拟单线雷达点数据聚类]
			org_cluster_LMS();

			double local_goal_angle = 90;
			if (g_trace_global_task_pt_flag > 0)
			{
				if (g_trace_global_task_pt_flag == 1)
					local_goal_angle = 45;
				else if (g_trace_global_task_pt_flag == 2)
					local_goal_angle = 135;

				//[寻找目标点]
				g_local_pt = get_LMS_goal_pt(local_goal_angle, 1);
			}
			else
			{//[全局任务点在车前方，使用全局任务点]
				COOR2 pt1, pt2;
				pt1.x = pl_input->gp_info.tps[1].x;
				pt1.y = pl_input->gp_info.tps[1].y;
				coor2_e2v(&pl_input->state.pos, &pt1, &pt2);
				pt1.x = 0;
				pt1.y = 0;
				double dist = dist_point(&pt1, &pt2);
				double theta = atan2(pt2.y, pt2.x) * 180 / PI;
				if (theta > 55 &&\
					theta < 125 && \
					dist < 10000)
				{
					local_goal_angle = theta;
					g_local_pt = get_LMS_goal_pt(local_goal_angle, 1);
				}
				else
					g_local_pt = get_LMS_goal_pt(local_goal_angle, 0);
			}		



			int *pathBank = NULL;
			int pathLength = 0;
			int x_target = g_local_pt.x / GRID_LEN_PER_CELL + g_grid_center.x;
			int y_target = g_local_pt.y / GRID_LEN_PER_CELL + g_grid_center.y;
			//[A*寻找路径]
			pathBank = AStarFindPath(g_grid_center.x, g_grid_center.y + 1, x_target, y_target, g_closed_grid_map, g_grid_center, pathLength);
			if (pathBank != NULL)
			{
				COOR2 astar_path[240];
				int astar_path_num;
				memset(astar_path, 0, sizeof(COOR2) * 240);
				for (int i = 0; i < pathLength; i++)
				{
					astar_path[i].x = pathBank[i * 2 + 1];
					astar_path[i].y = pathBank[i * 2];
				}
				astar_path_num = pathLength;
				free(pathBank);
				pathBank = NULL;
				//[得到居中路径]
				get_astar_mid_path(astar_path, astar_path_num);
				//[优化路径]
				get_optimize_path(astar_path, astar_path_num);

				//[计算速度]
				double x = g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x;
				double y = g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y;
				double dist = sqrt(x * x + y * y);
				if (dist <= 600)
					g_cross_speed = (int)(5 * CM);
				else
					g_cross_speed = (int)(((dist - 600) / (2000 - 600) * 10 + 5) * CM);

				g_fidelity = 1;
			}
			else
				g_fidelity = 0;
		}
		else if (g_navi_state == D_EMERGENCY_STOP || g_navi_state == S_EMERGENCY_STOP)
		{
			ret = get_emergency_line(pl_input);		//[@@作用遗忘了。。。]
			ret = get_trace_lane_path(0);
			g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			g_fidelity = 1;
		}
	}//[if (g_stat == S_ROAD_NAV)]
	else if (g_stat == S_CROSS_UND)
	{
		if (g_road_type == 0 || g_road_type == 1)
		{//[城市化道路]
			//[@@2013年常熟比赛，城乡结合道路]
			keep_multi_lane(pl_input);
			g_cross_und_state = CROSS_TRACE;


			if (g_is_stopping == 1)
			{
				g_fidelity = 0;
#ifdef MBUG_OPEN_
				MBUG("S_CROSS_UND STOP LINE : %d\n", g_fidelity);
#endif
				get_trace_lane_path(1);
			}
			else
			{
			
					COOR2 cur_goal_pt;
					COOR2 temp_pt;
					temp_pt.x = pl_input->gp_info.tps[1].x;
					temp_pt.y = pl_input->gp_info.tps[1].y;
					coor2_e2v(&pl_input->state.pos, &temp_pt, &cur_goal_pt);

					COOR2 pt1;
					COOR2 pt2;
					pt1.x = pl_input->gp_info.tps[1].x;
					pt1.y = pl_input->gp_info.tps[1].y;
					pt2.x = pl_input->state.pos.com_coord.x;
					pt2.y = pl_input->state.pos.com_coord.y;
					pt1.x = pt1.x - pt2.x;
					pt1.y = pt1.y - pt2.y;
					//coor2_e2v(&pl_input->state.pos, &pt1, &pt2);
					int lead_yaw = (int)(atan2(pt1.y, pt1.x) * 180 / PI);
					//[2、转换到正北系下]
					lead_yaw -= 90;
					if (lead_yaw < -180)
						lead_yaw = 360 + lead_yaw;
					//[3、计算偏差]
					int cur_yaw = g_yaw;
					int delta_yaw = cur_yaw - lead_yaw;

					if (delta_yaw <= -180)
						delta_yaw = delta_yaw + 360;
					else if (delta_yaw >= 180)
						delta_yaw = 360 - delta_yaw;


					double dist = dist_point(&cur_goal_pt, &g_vehicle_org_pt);
					//if ((dist < g_global_task_pt_valid_dist && 
					if ((dist < 3000 && \
						dist > 1500) && \
						delta_yaw > 60 && \
						pl_input->gp_info.tps[1].direction != 5 && \
						pl_input->gp_info.tps[1].type != 2)//[不要向交通标志牌引导点行驶]
					{
						g_fast_turn_flag = 1;
						grid_operation(g_grid_map, GRID_WIDTH, GRID_HEIGHT, g_closed_grid_map);
						ret = roving(cur_goal_pt, 1);
						g_cross_und_state = CROSS_ROVING;

						if (ret == -1)
						{
							g_fidelity = 0;
							get_trace_lane_path(1);
							g_cross_und_state = CROSS_TRACE;
							g_reverse_start_timer++;
							if (g_reverse_start_timer > g_reverse_start_timer_value)
							{
								g_reverse_start_timer = 0;
								reversePlan.SetReverseFlag(true);
							}
						}
						else
						{
// 							double x_step, y_step;
// 							x_step = (cur_goal_pt.x + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
// 							y_step = (cur_goal_pt.y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
// 
// 							memset(pl_cs_data, 0, sizeof(PL_CS_DATA));
// 							for (int ii = 0; ii < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; ii++)
// 							{
// 								g_mid_line[ii].x = (int)(ii * x_step);
// 								g_mid_line[ii].y = (int)(ii * y_step);
// 							}

							g_fidelity = 1;
							g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
						}
					}
					else
					{
						g_fast_turn_flag = 0;
						g_navi_state = TRACE_LANE;
						ret = trace_lane(pl_input);
						//[根据不同状态生成规划线]
						if (g_navi_state == TRACE_LANE)
						{
							if (ret == 0)
							{
								g_fidelity = 1;
								g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
								get_trace_lane_path(0);
								g_cross_und_state = CROSS_TRACE;
								
							}
							else
							{
								grid_operation(g_grid_map, GRID_WIDTH, GRID_HEIGHT, g_closed_grid_map);
								ret = roving(cur_goal_pt, 1);
								g_cross_und_state = CROSS_ROVING;

								if (ret == -1)
								{
									g_fidelity = 0;
									get_trace_lane_path(1);
									g_cross_und_state = CROSS_TRACE;
									g_reverse_start_timer++;
									if (g_reverse_start_timer > g_reverse_start_timer_value)
									{
										g_reverse_start_timer = 0;
										reversePlan.SetReverseFlag(true);
									}
								}
								else
								{
									g_fidelity = 1;
									g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
								}

							}
						}
					}
			}
		}//[if (g_road_type == 0)]
		else
		{//[以下是乡村道路的路口理解]
			if (g_is_stopping == 1)
			{
				g_fidelity = 0;
#ifdef MBUG_OPEN_
				MBUG("S_CROSS_UND STOP LINE : %d\n", g_fidelity);
#endif
				get_trace_lane_path(1);
			}
			else
			{
				if (g_navi_state == TRACE_LANE)
				{
					keep_multi_lane(pl_input);
					ret = trace_lane(pl_input);
				}
				else if (g_navi_state == TRACE_IN_NATURAL_ROAD)
				{
					keep_multi_lane(pl_input);
					ret = trace_in_natural_road();
				}
				else if (g_navi_state == ROVING)
				{
					keep_multi_lane(pl_input);
					ret = trace_lane(pl_input);
				}
				//[********S_ROAD_NAV状态下，规划内部状态转换********]

				//[根据不同状态生成规划线]
				if (g_navi_state == TRACE_LANE)
				{
					if (ret == 0)
					{
						get_trace_lane_path(0);
					}
					else
					{
						g_fidelity = 0;
#ifdef MBUG_OPEN_
						MBUG("trace lane g_fidelity : %d\n", g_fidelity);
#endif
						get_trace_lane_path(1);
					}
				}
				else if (g_navi_state == TRACE_IN_NATURAL_ROAD)
				{
					get_trace_lane_in_natural_path();
					g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
					g_fidelity = 1;
				}
				else if (g_navi_state == ROVING)
				{
					//[先对栅格进行膨胀腐蚀处理]
					grid_operation(g_grid_map, GRID_WIDTH, GRID_HEIGHT, g_closed_grid_map);

					COOR2 pt1;
					COOR2 pt2;
					pt1.x = pl_input->gp_info.tps[1].x;
					pt1.y = pl_input->gp_info.tps[1].y;

					diff_coor2_e2v(&pl_input->state.pos, &pt1, &pt2);
#ifdef MBUG_OPEN_
					MBUG("current roving gps goal pt : (%d, %d)\n", pt1.x, pt1.y);
					MBUG("current roving vehicle goal pt : (%d, %d)\n", pt2.x, pt2.y);
#endif
	
					{
						COOR2 pt;
						pt.x = 0;
						pt.y = 0;
						g_cur_search_index = -1;
						g_cur_search_index = roving(pt, 0);

						if (g_cur_search_index == -1)
							g_fidelity = 0;
						else
							g_fidelity = 1;
					}
				}
			}
		}//[else  乡村道路S_CROSS_UND]
	}
	else if (g_stat == S_STRAIGHT ||
		g_stat == S_LEFT ||
		g_stat == S_RIGHT ||
		g_stat == S_UTURN)
	{
		keep_multi_lane(pl_input);//[道路维持，判断是否有稳定道边，进行状态切换]

		grid_operation(g_grid_map, GRID_WIDTH, GRID_HEIGHT, g_closed_grid_map);
		{
			ret = cross_intersection(pl_input, pl_cs_data, pl_local_data);
#ifdef MBUG_OPEN_
			for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
			{
				MBUG("(%d, %d) ", g_mid_line[i].x, g_mid_line[i].y);
			}
			MBUG("\n");
#endif
			if (ret >= 0)
				g_fidelity = 1;
			else
				g_fidelity = 0;
		}
	}
	else if (g_stat == S_TASK_OVER)
	{
		if (g_is_stopping == 1)
		{
			pl_cs_data->id = g_frame_id;
			pl_cs_data->isok = 1;
			pl_cs_data->number_of_effective_points = 0;
			memset(pl_cs_data->path, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
			pl_cs_data->speed = 0;
			memcpy(&pl_cs_data->state, &pl_input->state, sizeof(STATE));
			pl_cs_data->sys_command = SYS_COMMAND_LEFT | SYS_COMMAND_RIGHT;

			g_fidelity = 0;
#ifdef MBUG_OPEN_
			MBUG("S_TASK_OVER STOP LINE : %d\n", g_fidelity);
#endif
			get_trace_lane_path(1);
		}
		else
		{
		if (g_navi_state == TRACE_LANE)
		{
			keep_multi_lane(pl_input);
			ret = trace_lane(pl_input);
		}
		else if (g_navi_state == TRACE_IN_NATURAL_ROAD)
		{
			keep_multi_lane(pl_input);
			ret = trace_in_natural_road();
		}
		else if (g_navi_state == ROVING)
		{
			keep_multi_lane(pl_input);
			ret = trace_lane(pl_input);
		}
		//[********S_ROAD_NAV状态下，规划内部状态转换********]

		if (g_navi_state == TRACE_IN_NATURAL_ROAD || \
			g_navi_state == S_EMERGENCY_STOP)
		{
			g_navi_state = ROVING;
		}

		COOR2 cur_goal_pt;
		COOR2 temp_pt;
		temp_pt.x = pl_input->gp_info.tps[1].x;
		temp_pt.y = pl_input->gp_info.tps[1].y;
		coor2_e2v(&pl_input->state.pos, &temp_pt, &cur_goal_pt);

		double dist = dist_point(&cur_goal_pt, &g_vehicle_org_pt);
		if (dist < g_global_task_pt_valid_dist && \
			dist > 1500)
		{
			g_navi_state = ROVING;
		}


		//[根据不同状态生成规划线]
		if (g_navi_state == TRACE_LANE)
		{
			if (ret == 0)
			{
				get_trace_lane_path(0);
			}
			else
			{
				g_fidelity = 0;
#ifdef MBUG_OPEN_
				MBUG("trace lane g_fidelity : %d\n", g_fidelity);
#endif
				get_trace_lane_path(1);
			}
		}
		else if (g_navi_state == TRACE_IN_NATURAL_ROAD)
		{
			if (ret == 0)
			{
				get_trace_lane_path(0);
			}
			else
			{
				double x_step;
				double y_step;
				x_step = (g_last_frame_pt.x + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
				y_step = (g_last_frame_pt.y + 0.0) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);
				for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
				{
					g_mid_line[i].x = (INT32)(i * x_step);
					g_mid_line[i].y = (INT32)(i * y_step);
				}
				g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
				g_fidelity = 1;
#ifdef MBUG_OPEN_
				MBUG("trace lane g_fidelity : %d\n", g_fidelity);
#endif
			}
		}
		else if (g_navi_state == ROVING)
		{
			COOR2 cur_goal_pt;
			COOR2 temp_pt;
			temp_pt.x = pl_input->gp_info.tps[1].x;
			temp_pt.y = pl_input->gp_info.tps[1].y;
			coor2_e2v(&pl_input->state.pos, &temp_pt, &cur_goal_pt);

			grid_operation(g_grid_map, GRID_WIDTH, GRID_HEIGHT, g_closed_grid_map);
			ret = roving(cur_goal_pt, 1);
			g_cross_und_state = CROSS_ROVING;

			if (ret == -1)
			{
				g_fidelity = 0;
#ifdef MBUG_OPEN_
				MBUG("trace lane g_fidelity : %d\n", g_fidelity);
#endif
				get_trace_lane_path(1);
				g_cross_und_state = CROSS_TRACE;
			}
			else
			{
				g_fidelity = 1;
				g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
			}

		}
	}
	}
	if (g_is_s_obs == 1)
	{
		int mode = hd_s_obs_plan_mode();

// 		hd_grid2LMS();
// 		ret = hd_LMS_mid_plan();

		COOR2 point;
		point.x = 0;
		point.y = 0;
		if (mode == 1)
		{
			point = hd_s_obs_mid_plan();
		}
		else if (mode == 0)
		{
			hd_grid2LMS();
			ret = hd_mid_plan();

			if (ret == 0)
			{
				point.x = 1;
			}
			else
			{
				point.x = 0;
				point.y = 0;
			}
		}
		else
		{
			int a = 0;
			a = a;

			ret = hd_narrow_area_mid_plan();
			if (ret == 0)
			{
				point.x = 1;
			}
			else
			{
				point.x = 0;
				point.y = 0;
			}
		}



		if (point.x == 0 && point.y == 0)
		//if (ret == -1)
		{
			g_cur_search_index = -1;
			g_cur_search_index = roving_for_s_obs(pt, 0);

			if (g_cur_search_index == -1)
			{
				memcpy(g_mid_line, g_s_obs_plan_line, g_s_obs_plan_line_num * sizeof(COOR2));
				g_long_line_num = g_s_obs_plan_line_num;
				if (ret == 0)
				{
					g_fidelity = 1;
				}
				else
				{
					g_fidelity = 0;
				}

			}
			else
			{
				g_s_obs_crash_flag = 1;
				g_fidelity = 1;
			}
		}
		else
		{
// 			double x = (point.x - g_hd_grid_center.x) * GRID_LEN_PER_CELL_HD;
// 			double y = (point.y - g_hd_grid_center.y) * GRID_LEN_PER_CELL_HD;
// 			double x_step;
// 			double y_step;
// 			x_step = x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
// 			y_step = y / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
// 			g_mid_line[0].x = 0;
// 			g_mid_line[0].y = 0;
// 			for (i = 1; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
// 			{
// 				g_mid_line[i].x = (INT32)(i * x_step);
// 				g_mid_line[i].y = (INT32)(i * y_step);
// 			}
			g_fidelity = 1;
		}
		/*
		g_cur_search_index = -1;
		g_cur_search_index = roving_for_s_obs(pt, 0);

		if (g_cur_search_index == -1)
		{
			memcpy(g_mid_line, g_s_obs_plan_line, g_s_obs_plan_line_num * sizeof(COOR2));
			g_long_line_num = g_s_obs_plan_line_num;
			if (ret == 0)
			{
				g_fidelity = 1;
			}
			else
			{
				g_fidelity = 0;
			}

		}
		else
		{
			g_s_obs_crash_flag = 1;
			g_fidelity = 1;
		}
		*/
	}

	get_pl_speed(pl_input);

	if (g_cur_speed < 0)
		g_cur_speed = g_cur_speed;

	if (g_navi_state == ROVING)
		set_roving_effective_pt_num();
	else
	{
		for (int i = 1; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;i++)
		{
			if (g_mid_line[i].x == 0 && g_mid_line[i].y == 0)
			{
				g_long_line_num = i;
				break;
			}
		}
	}

	set_pl_result(pl_input, pl_cs_data);
	process_pl_cs_data(pl_cs_data);

	g_last_frame_pt.x = pl_cs_data->path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x;
	g_last_frame_pt.y = pl_cs_data->path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y;

#ifdef MBUG_OPEN_
	MBUG("\n");
	for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
	{
		MBUG("(%d, %d) ", pl_cs_data->path[i].x, pl_cs_data->path[i].y);
	}
	MBUG("\n");

	MBUG("pl_isok : %d \n", pl_cs_data->isok);
	MBUG("pl_speed : %d \n", pl_cs_data->speed);

	MBUG("the Process_Plan finish ---------------\n\n\n\n\n");
#endif
	g_last_navi_state = g_navi_state;

	set_zgcc_plan_status(0);
	return ret = 0;
}