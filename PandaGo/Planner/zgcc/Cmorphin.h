#ifndef MORPHIN_H_
#define MORPHIN_H_

#include "../robix4/protocols/app_pl.h"
#include <vector>

using namespace std;

#define MAX_VALUE 100000//[用来初始化一些变量]
#define PI 3.1415926
#define GRID_HEIGHT (LEN_VER_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER))
#define GRID_WIDTH	(LEN_HOR_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR))
#define GRID_MID_LEFT GRID_WIDTH / 2 -1		//[栅格正中间左侧]
#define GRID_MID_RIGHT GRID_WIDTH / + 1		//[栅格正中间右侧]
#define GRID_LEN_PER_CELL	25				//[每个栅格25cm]

#define GRID_HEIGHT_HD (LEN_VER_HD * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER_HD))
#define GRID_WIDTH_HD	(LEN_HOR_HD * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR_HD))
#define GRID_LEN_PER_CELL_HD	12.5				//[每个栅格12.5cm]

#define MORPHIN_LINE_NUM 65					//[MORPHIN线的个数]
#define MORPHIN_MID_INDEX 32				//[MORPHIN线中间值]

// #define ARC_LINE_NUM 81
// #define ARC_MID_INDEX 41

typedef struct{
	COOR2 g_morphin_lines[MORPHIN_LINE_NUM][NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int g_morphin_lines_num[MORPHIN_LINE_NUM];
	int g_best_index;
}MORPHIN;

typedef struct{
	COOR2 g_morphin_lines[MORPHIN_LINE_NUM][200];
	int g_morphin_lines_num[MORPHIN_LINE_NUM];
	int g_best_index;
}MORPHIN2;

typedef struct{
	COOR2 g_collision_lines[MORPHIN_LINE_NUM][200];
	int g_collision_lines_num[MORPHIN_LINE_NUM];
	int g_best_index;
}COLLISION_ARC;

// extern COOR2 g_morphin_lines[MORPHIN_LINE_NUM][NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
// extern int g_morphin_lines_num[MORPHIN_LINE_NUM];	//[每条弧线的描述点个数]
// extern double g_morphin_radius[MORPHIN_LINE_NUM];	//[每条弧线的半径]
extern double g_morphin_dist;						//[生成弧线的长度]

extern vector<MORPHIN2> g_morhpin2;
extern double g_morphin_radius2[MORPHIN_LINE_NUM];	//[每条弧线的半径]
extern double g_morphin_arclen2[MORPHIN_LINE_NUM];	//[每条弧线的长度]
extern double g_morphin_angle[MORPHIN_LINE_NUM];	//[每条弧线对应的角度]

extern COLLISION_ARC g_collision_arc;
extern double g_collision_arclen[MORPHIN_LINE_NUM];	//[每条弧线的长度]
extern double g_morphin_collision_radius[MORPHIN_LINE_NUM];		//[考虑碰撞的运动半径]
extern int g_morphin_include_radius[MORPHIN_LINE_NUM];		//[碰撞的包络圆半径]

extern double g_value[MORPHIN_LINE_NUM];

extern int g_cross_avg_index;						//[索引]
extern int g_cross_last_index;


#define CROSS_AVG_NUM 10							//[交叉口平均取值数目]
extern double g_cross_avg_angle[CROSS_AVG_NUM];
extern int g_cross_avg_angle_num;

extern double g_cross_travel_rate;

//[初始化]
int morphin3();
int collision_arc();

//[获取平滑后的搜索弧线]
void get_avg_mid_line(double angle);
//[获取最佳的探索曲线]
int get_best_morphin_line1(COOR2 sub_goal_pt, double &out_rate);
int get_best_morphin_line2(COOR2 sub_goal_pt, double &out_rate);
int get_best_morphin_line3(COOR2 sub_goal_pt, double &out_rate);
int get_best_morphin_line4(COOR2 sub_goal_pt, double &out_rate);
int get_best_morphin_line5(COOR2 sub_goal_pt, double &out_rate);
int get_best_morphin_line55(COOR2 sub_goal_pt, double &out_rate);
int get_best_morphin_line6(COOR2 sub_goal_pt, double &out_rate);
int get_best_morphin_line7(COOR2 sub_goal_pt, double &out_rate);

int get_best_arc_line1(COOR2 sub_goal_pt, double &out_rate);
int get_best_arc_line2(COOR2 sub_goal_pt, double &out_rate);
int get_best_arc_line3(COOR2 sub_goal_pt, double &out_rate);
int get_best_arc_line4(COOR2 sub_goal_pt, double &out_rate);
//[漫游]
int roving(COOR2 goal_pt, int mode);
//[S弯漫游，降低了危险距离]
int roving_for_s_obs(COOR2 goal_pt, int mode);
COOR2 s_obs_mid_plan();
COOR2 hd_s_obs_mid_plan();
void hd_get_avg_mid_line(double angle, double dist2obs);

void recover_one_frame();
void get_avg_mid_line(double angle, double dist2obs);
int cross_country(COOR2 sub_goal_pt, int &g_cross_speed);
void reset_cross_country_paragram();

void create_line_by_theta(double theta, COOR2 *pts, int &pts_num, int mode);
//[掉头]
void TurnRoundAction(double angle, COOR2 path[], int &path_pts_num);
void TurnRoundActionBack(double angle, COOR2 path[], int &path_pts_num);
#endif /*MORPHIN_H_*/