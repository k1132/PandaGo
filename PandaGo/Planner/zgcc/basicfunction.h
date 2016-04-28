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

//[��0-360�任��180-��-180��]
#define  CONVERT_YAW(x)  \
	if (x > 180)  \
	x -= 360;

//[������ȷ��ת���]
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
/* �������->��������  ʹ�ò��GPS */
void diff_coor2_e2v(POSE_INFO *gps, COOR2 *p1, COOR2 *p2);
/* ��������->�������  ʹ�ò��GPS */
void diff_coor2_v2e(POSE_INFO *gps, COOR2 *p1, COOR2 *p2);
/* �������->�������� */
void coor2_e2v(POSE_INFO *gps, COOR2 *p1, COOR2 *p2);
/* ��������->������� */
void coor2_v2e(POSE_INFO *gps, COOR2 *p1, COOR2 *p2);
//[���������]
double dist_point(COOR2 *p1, COOR2 *p2);

float decas(int degree, int coeff[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE], float t);
void bez_to_points(int npoints, int ppoints[100], int degree, int coeff[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE]);
void get_bezier_line(COOR2 *line, int line_num, COOR2 *bz_line, int bz_line_num);

//[����һ�Ե��ֵ]
void swap_point(COOR2 *p1, COOR2 *p2);
//[��ȡһ��դ����ĸ����У���Сx�����x�Լ���ֵx]
void get_mid_x(QUAD *box4, int *minx, int *maxx, int *midx);
//[��ȡһ��դ����ĸ����У���Сy�����y�Լ���ֵy]
void get_mid_y(QUAD *box4, int *miny, int *maxy, int *midy);
//[����һ�����y������һ�����϶�Ӧ��x����]
int get_x_coord(int y, COOR2 *line, int line_num, int *x);
//[�Ը�����������������]
int roundf2i(double num);
//[��һ������в�ֵ�����嵽200����]
void line_fitting(COOR2 *edge, int edge_len, COOR2 *out_edge, int &out_edge_len, int step, int max_pts_num);
//[@brief ��ȡ�ںϸ�����һ���ߵ���Ч����]
int get_effective_points_num(COOR2 *edge);
//[��һ���ϰ����ĵ�������һ���ߣ���ȡ�ϰ��ﵽ�ߵ���̾���������]
void get_min_max(QUAD *box4, COOR2 *line, int num, int *mind, int *maxd, int car_len);


int PcaProcess(COOR2 *point, int point_num, double &k, double &b);
double get_road_direction(PL_FUNC_INPUT *pl_input, int mode, double del_theta);

//[A*�㷨Ѱ��·��]
// #define GRID_HEIGHT_HD (LEN_VER_HD * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER_HD))
// #define GRID_WIDTH_HD	(LEN_HOR_HD * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR_HD))
#define GRID_HEIGHT (LEN_VER_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER))
#define GRID_WIDTH (LEN_HOR_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR))
int * AStarFindPath(int startingX, int startingY, int targetX, int targetY, int grid[][GRID_WIDTH], COOR2 grid_center, int &pathLength);
#endif /** BASIC_FUCTION_H_ */