#ifndef PL_H_
#define PL_H_

#include "../robix4/protocols/app_pl.h"
#include "../robix4/protocols/protocol_status.h"
#include <deque>
#include <vector>
using namespace std;

#define FU_ROAD_LENGTH 6000

//[�滮����ά���Ķ�̬Ŀ��]
typedef struct
{
	COOR2 center;		//[Ŀ����������]
	COOR2 coor2[4];		//[Ŀ���ĵ�����]
	SPD2D speed;		//[Ŀ����ٶȣ�һ������y����]
	COOR2 nearest_pt;	//[Ŀ������ĵ�]
}DYN_OBS;


//[Ѳ��״̬�£��滮�ڲ�״̬]
enum NAVI_STATE
{
	TRACE_LANE,					//[��·����]
	CHANGE_TO_LEFT_LANE,		//[�������]
	CHANGE_TO_RIGHT_LANE,		//[�����ҵ�]
	FOLLOW_THE_CAR,				//[����ǰ������]
	TAKEOVER_FROM_LEFT,			//[���󷽳���]
	TAKEOVER_FROM_RIGHT,		//[���ҷ�����]
	TRACE_IN_NATURAL_ROAD,		//[����·]
	TAKEOVER_IN_NATURAL_ROAD,	//[����·�ڳ���]
	ROVING,						//[����״̬]
	D_EMERGENCY_STOP,			//[��Գ����ļ�ͣ]
	S_EMERGENCY_STOP,			//[��Ծ�̬�ϰ���ļ�ͣ]

	REVERSE_TRACE_GLOBAL_POINTS,//[����ȫ�ֵ㵹��]
	TURN_ROUND,					//[��ͷ]
	REVERSE_DIRECT_BACK			//[ֱ�ߵ���]
};

//[·�����״̬]
enum CROSS_UND_STATE
{
	CROSS_SLOW,				//[·��������]
	CROSS_ADJUST,			//[·��������·��]
	CROSS_ROVING,			//[·���������]
	CROSS_TRACE				//[·����⳵������]
};

//[�������̵�״̬��Ŀǰû��չ������]
enum CHANGE_LANE_STATE
{
	NORMAL_CHANGE,				//[һ�㻻��]
};


#define LANE_LINE_NUM 4

//[Ϊ�˲��Ķ�ԭ�к������Զ�����EDGE�ṹ�壬����ʹ��COOR2]
typedef struct {
	COOR2 line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	UINT8 valid_num_points; /*!< Valid numbers of points. */
	UINT8 line_type;        /*!< LINE_TYPE. */
	UINT8 line_color;       /*!< LINE_COLOR. */
	UINT8 dir_type_hor;     /*!< DIR_TYPE. */
	UINT8 fidelity; 	    /*!< Fidelity. */   
} PL_EDGE;

//[ά���Ķ೵������]
typedef struct
{
	//[0 �󳵵����]
	//[1 ��ǰ�������]
	//[2 ��ǰ�����ұ�]
	//[3 �ҳ����ұ�]
	PL_EDGE lane_line[LANE_LINE_NUM];		//[��������]
	int ok_times[LANE_LINE_NUM];			//[����ȷ�ϴ���]
	POSE_INFO gps_info;						//[��ǰ������Ӧ�ĳ���GPS��Ϣ]
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
	int as;//[��ʼ�Ƕ� ��180 ��ʼ]
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


#define MAX_VALUE 100000//[������ʼ��һЩ����]


#define GRID_HEIGHT (LEN_VER_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER))
#define GRID_WIDTH (LEN_HOR_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR))
#define GRID_MID_LEFT GRID_WIDTH / 2 -1		//[դ�����м����]
#define GRID_MID_RIGHT GRID_WIDTH / + 1		//[դ�����м��Ҳ�]
#define GRID_LEN_PER_CELL 25				//[ÿ��դ��25cm]

#define GRID_HEIGHT_HD (LEN_VER_HD * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER_HD))
#define GRID_WIDTH_HD	(LEN_HOR_HD * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR_HD))
#define GRID_LEN_PER_CELL_HD	12.5				//[ÿ��դ��12.5cm]

static const float CM = 27.778f;		//[�������ټ���]
#define SYS_COMMAND_REV			0x00000001		//����
#define SYS_COMMAND_STOP		0x00000002		//��ͣ
#define SYS_COMMAND_LEFT		0x00000004		//���
#define SYS_COMMAND_RIGHT		0x00000008		//�ҵ�
#define SYS_COMMAND_BEEP		0x00000010		//����
#define SYS_COMMAND_ALARM		0x00000020		//����


//[���ڻط�ƽ̨��ʾ����ȫ��·����]
extern deque<COOR2> g_reverse_pts_show;

//[������¼·��ͨ��ʱʹ�õ���Ŀ��㣬Ȼ�󴫵ݸ���ʾ����]
extern COOR2 subgoal_show[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
extern int subgoal_show_num;

extern int g_yaw;							//[�����]
extern MULTI_LANE g_multi_lane;				//[�೵������]
extern NATURAL_BOUNDARY g_natural_boundary;	//[��Ȼ��������]
extern int g_natural_road_search_dist;		//[��Ȼ��·�¹滮Ԥ�����]

extern int g_road_type;						//[��·����  0  �ṹ����·  1  �ǽṹ����·]
extern int g_stat;							//[ȫ��״̬��**��ôƴд����Ϊ֮ǰ�Ķ�ʦ�ְ汾��ôд�ˣ�û��]

extern COOR2 g_grid_center;						//[դ��ͼ����������λ��]
extern int g_grid_dist;							//[դ���ܹ��滮�ľ���]
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
extern int g_nature_road_switch;				//[ʹ����Ȼ�߽�Ŀ���]
extern int g_nature_road_to_obs_switch;			//[��Ȼ����ͶӰ��դ��]
extern int g_real_speed;

#define  HISTOGRAM_DIST_SIZE 10
extern int g_histogram_dist[HISTOGRAM_DIST_SIZE];

enum ZG_PLAN_STATUS
{
	ZG_NORMAL,		//[����������������״̬]
	ZG_CHANGE_LANE,	//[����]
	ZG_OVERTAKE,	//[����]
	ZG_SOBS,		//[����S����]
	ZG_REVERSE,		//[���������Բ��ù�]
	ZG_STOP			//[ͣ��]
};


void set_db_flag(PL_CS_DATA *pl_cs_data);
void reset_for_wait();
void set_road_type(int val);
void plan_init();
int get_zgcc_plan_status();
int pl_road_trace_interface(int frame_id, PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data);


#endif /*PL_H_*/