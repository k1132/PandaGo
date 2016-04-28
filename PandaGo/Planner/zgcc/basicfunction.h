#ifndef BASIC_FUCTION_H_
#define BASIC_FUCTION_H_

#define MBUG_OPEN_

#ifdef __GNUC__
#include "../robix4/rbx4api.h"

#else
#ifndef MBUG_
#define MBUG_
extern void MBUG(const char * msg, ...);
#endif
#endif

#include "../robix4/protocols/app_pl.h"

#define PI 3.1415926

#define YAW_TURN(x)	\
	if (x >= 360)	\
		x -= 360;	\
	else if (x < 0)	\
		x += 360;

//[将0-360变换到180-（-180）]
#define  CONVERT_YAW(x)  \
	if (x > 180)  \
	x -= 360;

//[计算正确的转向角]
#define  CHECK_YAW(x)  \
	if (x < -180)  \
	  x += 360;  \
	else if (x >= 180) \
	  x -= 360;

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include <stdarg.h>


/** transport the point from gps_pos2 coordinate to gps_pos1 coordinate */
COOR2 coordinate_transform(POSE_INFO gps_pos1, POSE_INFO gps_pos2, COOR2 point);
COOR2 diff_coordinate_transform(POSE_INFO gps_pos1, POSE_INFO gps_pos2, COOR2 point);
/* 大地坐标->车辆坐标  使用差分GPS */
void diff_coor2_e2v(POSE_INFO *gps, COOR2 *p1, COOR2 *p2);
/* 车辆坐标->大地坐标  使用差分GPS */
void diff_coor2_v2e(POSE_INFO *gps, COOR2 *p1, COOR2 *p2);
/* 大地坐标->车辆坐标 */
void coor2_e2v(POSE_INFO *gps, COOR2 *p1, COOR2 *p2);
/* 车辆坐标->大地坐标 */
void coor2_v2e(POSE_INFO *gps, COOR2 *p1, COOR2 *p2);
//[求两点距离]
double dist_point(COOR2 *p1, COOR2 *p2);

float decas(int degree, int coeff[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE], float t);
void bez_to_points(int npoints, int ppoints[100], int degree, int coeff[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE]);
void get_bezier_line(COOR2 *line, int line_num, COOR2 *bz_line, int bz_line_num);

//[交换一对点的值]
void swap_point(COOR2 *p1, COOR2 *p2);
//[求取一个栅格的四个点中，最小x、最大x以及中值x]
void get_mid_x(QUAD *box4, int *minx, int *maxx, int *midx);
//[求取一个栅格的四个点中，最小y、最大y以及中值y]
void get_mid_y(QUAD *box4, int *miny, int *maxy, int *midy);
//[给出一个点的y，求在一条边上对应的x坐标]
int get_x_coord(int y, COOR2 *line, int line_num, int *x);
//[对浮点数进行四舍五入]
int roundf2i(double num);
//[给一组点序列插值，最多插到200个点]
void line_fitting(COOR2 *edge, int edge_len, COOR2 *out_edge, int &out_edge_len, int step, int max_pts_num);
//[@brief 获取融合给出的一条边的有效点数]
int get_effective_points_num(COOR2 *edge);
//[给一个障碍物四点描述和一条边，求取障碍物到边的最短距离和最长距离]
void get_min_max(QUAD *box4, COOR2 *line, int num, int *mind, int *maxd, int car_len);


int PcaProcess(COOR2 *point, int point_num, double &k, double &b);
double get_road_direction(PL_FUNC_INPUT *pl_input, int mode, double del_theta);

//[A*算法寻找路径]
// #define GRID_HEIGHT_HD (LEN_VER_HD * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER_HD))
// #define GRID_WIDTH_HD	(LEN_HOR_HD * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR_HD))
#define GRID_HEIGHT (LEN_VER_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER))
#define GRID_WIDTH (LEN_HOR_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR))
int * AStarFindPath(int startingX, int startingY, int targetX, int targetY, int grid[][GRID_WIDTH], COOR2 grid_center, int &pathLength);
#endif /** BASIC_FUCTION_H_ */