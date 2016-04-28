#ifndef PL_H_
#define PL_H_

#include "../robix4/protocols/app_pl.h"
#include "../robix4/protocols/protocol_status.h"
#include <deque>
#include <vector>
using namespace std;

#define FU_ROAD_LENGTH 6000

//[规划自身维护的动态目标]
typedef struct
{
	COOR2 center;		//[目标中心坐标]
	COOR2 coor2[4];		//[目标四点坐标]
	SPD2D speed;		//[目标的速度，一般是用y方向]
	COOR2 nearest_pt;	//[目标最近的点]
}DYN_OBS;


//[巡航状态下，规划内部状态]
enum NAVI_STATE
{
	TRACE_LANE,					//[道路跟踪]
	CHANGE_TO_LEFT_LANE,		//[换至左道]
	CHANGE_TO_RIGHT_LANE,		//[换至右道]
	FOLLOW_THE_CAR,				//[跟随前方车辆]
	TAKEOVER_FROM_LEFT,			//[从左方超车]
	TAKEOVER_FROM_RIGHT,		//[从右方超车]
	TRACE_IN_NATURAL_ROAD,		//[乡村道路]
	TAKEOVER_IN_NATURAL_ROAD,	//[乡村道路内超车]
	ROVING,						//[漫游状态]
	D_EMERGENCY_STOP,			//[针对车辆的急停]
	S_EMERGENCY_STOP,			//[针对静态障碍物的急停]

	REVERSE_TRACE_GLOBAL_POINTS,//[沿着全局点倒车]
	TURN_ROUND,					//[掉头]
	REVERSE_DIRECT_BACK			//[直线倒车]
};

//[路口理解状态]
enum CROSS_UND_STATE
{
	CROSS_SLOW,				//[路口理解减速]
	CROSS_ADJUST,			//[路口理解调整路线]
	CROSS_ROVING,			//[路口理解漫游]
	CROSS_TRACE				//[路口理解车道跟踪]
};

//[超车过程的状态，目前没有展开考虑]
enum CHANGE_LANE_STATE
{
	NORMAL_CHANGE,				//[一般换道]
};


#define LANE_LINE_NUM 4

//[为了不改动原有函数，自定义了EDGE结构体，这里使用COOR2]
typedef struct {
	COOR2 line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	UINT8 valid_num_points; /*!< Valid numbers of points. */
	UINT8 line_type;        /*!< LINE_TYPE. */
	UINT8 line_color;       /*!< LINE_COLOR. */
	UINT8 dir_type_hor;     /*!< DIR_TYPE. */
	UINT8 fidelity; 	    /*!< Fidelity. */   
} PL_EDGE;

//[维护的多车道描述]
typedef struct
{
	//[0 左车道左边]
	//[1 当前车道左边]
	//[2 当前车道右边]
	//[3 右车道右边]
	PL_EDGE lane_line[LANE_LINE_NUM];		//[车道描述]
	int ok_times[LANE_LINE_NUM];			//[车道确认次数]
	POSE_INFO gps_info;						//[当前车道对应的车体GPS信息]
}MULTI_LANE;


typedef struct {
	COOR2 l_boundary[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE]; /*!< Boundary in left wing. */
	COOR2 r_boundary[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE]; /*!< Boundary in right wing. */
	int   l_nums;
	int   r_nums;
	UINT8 l_fidelity;     /*!< Fidelity for left boundary. */   
	UINT8 r_fidelity;     /*!< Fidelity for right boundary. */   
} NATURAL_BOUNDARY;



#define LMS_AREA_NUM 60
typedef struct
{
	int as;//[起始角度 从180 开始]
	int ae;
	int as_dist;
	int ae_dist;
	int max_dist;
	int am;
	int num;
	int concave;
}LMS_AREA;

typedef struct
{
	int num;
	int max_gap_idx;
	double max_gap;
}LMS_SEGMENT_AREA_DATE;

// #define  LMS_THROUGH_AREA_NUM 10
// typedef struct
// {
// 	int is;
// 	int ie;
// 	int max_gap;
// }LMS_THROUGH_AREA;


#define MAX_VALUE 100000//[用来初始化一些变量]


#define GRID_HEIGHT (LEN_VER_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER))
#define GRID_WIDTH (LEN_HOR_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR))
#define GRID_MID_LEFT GRID_WIDTH / 2 -1		//[栅格正中间左侧]
#define GRID_MID_RIGHT GRID_WIDTH / + 1		//[栅格正中间右侧]
#define GRID_LEN_PER_CELL 25				//[每个栅格25cm]

#define GRID_HEIGHT_HD (LEN_VER_HD * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER_HD))
#define GRID_WIDTH_HD	(LEN_HOR_HD * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR_HD))
#define GRID_LEN_PER_CELL_HD	12.5				//[每个栅格12.5cm]

static const float CM = 27.778f;		//[用来快速计算]
#define SYS_COMMAND_REV			0x00000001		//倒车
#define SYS_COMMAND_STOP		0x00000002		//急停
#define SYS_COMMAND_LEFT		0x00000004		//左灯
#define SYS_COMMAND_RIGHT		0x00000008		//右灯
#define SYS_COMMAND_BEEP		0x00000010		//喇叭
#define SYS_COMMAND_ALARM		0x00000020		//警报


//[用于回放平台显示倒车全局路径点]
extern deque<COOR2> g_reverse_pts_show;

//[用来记录路口通过时使用的子目标点，然后传递给显示窗口]
extern COOR2 subgoal_show[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
extern int subgoal_show_num;

extern int g_yaw;							//[航向角]
extern MULTI_LANE g_multi_lane;				//[多车道描述]
extern NATURAL_BOUNDARY g_natural_boundary;	//[自然道边描述]
extern int g_natural_road_search_dist;		//[自然道路下规划预瞄距离]

extern int g_road_type;						//[道路类型  0  结构化道路  1  非结构化道路]
extern int g_stat;							//[全局状态，**这么拼写是因为之前的杜师兄版本这么写了，没改]

extern COOR2 g_grid_center;						//[栅格图中所处中心位置]
extern int g_grid_dist;							//[栅格能够规划的距离]
extern int g_grid_map[GRID_HEIGHT][GRID_WIDTH];
extern int g_closed_grid_map[GRID_HEIGHT][GRID_WIDTH];
extern COOR2 g_hd_grid_center;
extern int g_hd_grid_map[GRID_HEIGHT_HD][GRID_WIDTH_HD];
extern int g_hd_closed_grid_map[GRID_HEIGHT_HD][GRID_WIDTH_HD];

extern int g_morphin_search_dist;
extern COOR2 g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
extern int g_long_line_num;

#define S_OBS_AVG_NUM 5
extern int g_s_obs_last_angle;
extern int g_s_obs_speed_down;
extern double g_s_obs_avg_angle[S_OBS_AVG_NUM];
extern int g_s_obs_avg_angle_num;
extern int g_s_obs_avg_index;

extern COOR2 g_cur_road_pt;
extern int g_roving_switch;
extern int g_roving_speed;
extern int g_nature_road_switch;				//[使用自然边界的开关]
extern int g_nature_road_to_obs_switch;			//[自然道边投影到栅格]
extern int g_real_speed;

#define  HISTOGRAM_DIST_SIZE 10
extern int g_histogram_dist[HISTOGRAM_DIST_SIZE];

enum ZG_PLAN_STATUS
{
	ZG_NORMAL,		//[其他不用特殊管理的状态]
	ZG_CHANGE_LANE,	//[换道]
	ZG_OVERTAKE,	//[超车]
	ZG_SOBS,		//[特殊S避障]
	ZG_REVERSE,		//[倒车，可以不用管]
	ZG_STOP			//[停等]
};


void set_db_flag(PL_CS_DATA *pl_cs_data);
void reset_for_wait();
void set_road_type(int val);
void plan_init();
int get_zgcc_plan_status();
int pl_road_trace_interface(int frame_id, PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data);


#endif /*PL_H_*/