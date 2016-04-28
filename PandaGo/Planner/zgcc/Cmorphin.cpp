#include "./Cmorphin.h"
#include "./trace_road.h"
#include "./basicfunction.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>
using namespace std;
// vector<MORPHIN> g_morhpin;
// 
// COOR2 g_morphin_lines[MORPHIN_LINE_NUM][NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];//[ÿ�����ߵ�������]
// int g_morphin_lines_num[MORPHIN_LINE_NUM];	//[ÿ�����ߵ����������]
// double g_morphin_radius[MORPHIN_LINE_NUM];	//[ÿ�����ߵİ뾶]
double g_morphin_dist = 0;					//[���ɻ��ߵĳ���]

vector<MORPHIN2> g_morhpin2;
double g_morphin_radius2[MORPHIN_LINE_NUM];	//[ÿ�����ߵİ뾶]
double g_morphin_arclen2[MORPHIN_LINE_NUM];	//[ÿ�����ߵĳ���]

COLLISION_ARC g_collision_arc;
double g_collision_arclen[MORPHIN_LINE_NUM];	//[ÿ�����ߵĳ���]
double g_morphin_collision_radius[MORPHIN_LINE_NUM];		//[������ײ���˶��뾶]
int g_morphin_include_radius[MORPHIN_LINE_NUM];		//[��ײ�İ���Բ�뾶]

double g_value[MORPHIN_LINE_NUM];			//[ÿ�����߶�Ӧ������ֵ]

int g_cross_avg_index;						//[ƽ���������]
int g_cross_last_index;						//[��һ֡������]
double g_cross_travel_rate;					//[��ͨ���ʣ�Ŀǰǿ������Ϊ1]

double g_cross_avg_angle[CROSS_AVG_NUM];
int g_cross_avg_angle_num;

//static double g_morphin_angle[MORPHIN_LINE_NUM] = {119,116,113,110,107,105,103,101,99,97,95,94,93.5,93,92.5,92,91.5,91,90.5,90,89.5,89,88.5,88,87.5,87,86.5,86,85,83,81,79,77,75,73,70,67,64,61};
//static double g_morphin_angle[MORPHIN_LINE_NUM] = {119,116,113,110,107,105,103,101,99,97,95,94,93,92,91,90,89,88,87,86,85,83,81,79,77,75,73,70,67,64,61};//[31]

//[ÿ�����߶�Ӧ�ĽǶ�  MORPHIN_LINE_NUM 59]
double g_morphin_angle[MORPHIN_LINE_NUM] = {117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96,95,94.5,94,93.5,93,92.5,92,91.5,91,90.5,90,89.5,89,88.5,88,87,86.5,86,85.5,85,84.5,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,65,64,63};//[59]


//***********************************************************************************************
//                                zgccmax 2012.July.10
//int morphin3()
//return:   int  ��ʱû����
//discribe:����һ��̽������
//***********************************************************************************************
/*==================================================================
 * ������  ��	int morphin3()
 * ����    ��	Ԥ��������ʻ����
 * ���������	
 * ���������	
 * ����ֵ  ��	
 * ����    ��	
 * ����    ��	
 * �޸ļ�¼��	
 *==================================================================*/
int morphin3()
{
	int i;
	int ret = 0;
	COOR2 pts[200];
	int pts_num = 0;

	g_morhpin2.clear();
	MORPHIN2 temp_morphin;
	memset(&temp_morphin, 0, sizeof(MORPHIN2));

	double radian;
	double radius;

	double car_len = 400;
	double D = 0;

//	double a = 0;
	double sd = 0;

	double theta = 0;
	double theta_a = 0;
	double theta_b = 0;
	double step = 0;

	int last_x = 0;
	int last_y = 0;
	int temp_x = 0;
	int temp_y = 0;

	//[���㲢�洢ÿ��·���İ뾶]
	//[��ʽ�Ƶ��������ʼ�2012.04.10]
	for (i=0; i<MORPHIN_MID_INDEX; i++)
	{
		theta = g_morphin_angle[i] - 90.0;
		radius = car_len / sin(theta / 180 * PI);

		//[������������]
//		a = g_morphin_angle[i] - 90;
		sd = 3000;

		//radian = sd / radius;
		D = sqrt(radius * radius - car_len * car_len);
		radian = sd / D;

		step = radian / 199;

		theta_a = 0;
		theta_b = radian;
		pts_num = 0;
		last_x = 0;
		last_y = -1;
		for (theta = theta_a; theta <= theta_a + theta_b && theta < PI / 2; theta += step)
		{
			temp_x = 0;
			temp_y = 0;
			temp_x = (INT32)(D * cos(theta) - D);
			temp_y = (INT32)(D * sin(theta));

			temp_x = temp_x / GRID_LEN_PER_CELL + g_grid_center.x;
			temp_y = temp_y / GRID_LEN_PER_CELL + g_grid_center.y;

			if (temp_x != last_x || temp_y != last_y)
			{
				pts[pts_num].x = (INT32)(D * cos(theta) - D);
				pts[pts_num].y = (INT32)(D * sin(theta));
				pts_num++;
				last_x = temp_x;
				last_y = temp_y;
			}

			if (pts[pts_num - 1].x < -900)
			{
				break;
			}
		}
		memcpy(temp_morphin.g_morphin_lines[i], pts, pts_num * sizeof(COOR2));
		temp_morphin.g_morphin_lines_num[i] = pts_num;
		g_morphin_radius2[i] = -radius;
		g_morphin_arclen2[i] = (theta - step - theta_a) * radius;
	}

	{
		g_morphin_radius2[MORPHIN_MID_INDEX] = 1000000;

		step = 25;//2000 / (199);
		for (i=0; i<121; i++)
		{
			pts[i].x = 0;
			pts[i].y = (INT32)(i * step);
		}
		pts_num = 121;
		memcpy(temp_morphin.g_morphin_lines[MORPHIN_MID_INDEX], pts, sizeof(COOR2) * pts_num);
		temp_morphin.g_morphin_lines_num[MORPHIN_MID_INDEX] = pts_num;

		g_morphin_arclen2[MORPHIN_MID_INDEX] = 3000;
	}

	//[�ҷ������������߶Գ�]
	int j;
	for (i=0; i<MORPHIN_MID_INDEX; i++)
	{
		for (j=0; j<temp_morphin.g_morphin_lines_num[i]; j++)
		{
			temp_morphin.g_morphin_lines[MORPHIN_LINE_NUM - i - 1][j].x = -temp_morphin.g_morphin_lines[i][j].x;
			temp_morphin.g_morphin_lines[MORPHIN_LINE_NUM - i - 1][j].y = temp_morphin.g_morphin_lines[i][j].y;

		}
		temp_morphin.g_morphin_lines_num[MORPHIN_LINE_NUM - i - 1] = temp_morphin.g_morphin_lines_num[i];

		g_morphin_radius2[MORPHIN_LINE_NUM - i - 1] = -g_morphin_radius2[i];
		g_morphin_arclen2[MORPHIN_LINE_NUM - i - 1] = g_morphin_arclen2[i];
	}

	g_morhpin2.push_back(temp_morphin);

// 	int dist[MORPHIN_LINE_NUM];
// 	for (i=0;i<MORPHIN_LINE_NUM;i++)
// 	{
// 		dist[i] = g_morhpin2[0].g_morphin_lines[i][g_morhpin2[0].g_morphin_lines_num[i]-1].y;
// 	}

	return ret;
}


int collision_arc()
{
	int i;
	int j;
	int ret = 0;
	COOR2 pts[200];
	int pts_num = 0;

	COLLISION_ARC temp_collision_arc;
	memset(&temp_collision_arc, 0, sizeof(COLLISION_ARC));

	double radian;
	double radius;

	double car_len = 400;
	double car_wid = 230;
	double D = 0;

	double c = pow(0.6, 1.0/3);//[��Ӧ800/2000��xֵ]
	double a = 0;
	double b = 0;
	double sd = 0;

	double theta = 0;
// 	double theta_a = 0;
// 	double theta_b = 0;
	double step = 0;

	int last_x = 0;
	int last_y = 0;
	int temp_x = 0;
	int temp_y = 0;

	double r_left = 0;
	double r_right = 0;
	double r_collision = 0;
	double r_include = 0;

	double s_theta = 0;
	double e_theta = 0;

	double temp_len = 0;

	//[���㲢�洢ÿ��·���İ뾶]
	//[��ʽ�Ƶ��������ʼ�2012.04.10]
	//[��ջ���]
	for (i=0; i<MORPHIN_MID_INDEX; i++)
	{
		theta = g_morphin_angle[i] - 90.0;
		radius = car_len / sin(theta / 180 * PI);

		//[������������]
		a = g_morphin_angle[i] - 90;
		b = c * (a / 45);
		b = -pow(b, 3.0) + 1;
		sd = 2400 * b;

		radian = sd / radius;
		D = sqrt(radius * radius - car_len * car_len);

		r_left = D - car_wid / 2;
		r_right = sqrt(car_len * car_len + (D + car_wid / 2) * (D + car_wid / 2));
		r_collision = (r_left + r_right) / 2;
		r_include = (r_right - r_left) / 2;

		s_theta = 0;
		e_theta = radian;
		step = radian / 199;

		pts_num = 0;
		last_x = -1;
		last_y = -1;
		for (theta = s_theta; theta <= e_theta && theta < PI / 2; theta += step)
		{
			temp_x = 0;
			temp_y = 0;
			temp_x = (INT32)(r_collision * cos(theta) - D);
			temp_y = (INT32)(r_collision * sin(theta));

			temp_x = temp_x / GRID_LEN_PER_CELL + g_grid_center.x;
			temp_y = temp_y / GRID_LEN_PER_CELL + g_grid_center.y;

			//[��ֹ���ظ��ĵ�]
			if (temp_x != last_x || temp_y != last_y)
			{
				pts[pts_num].x = (INT32)(r_collision * cos(theta) - D);
				pts[pts_num].y = (INT32)(r_collision * sin(theta));
				pts_num++;
				last_x = temp_x;
				last_y = temp_y;
			}

			//[���ƻ����������]
			if (pts[pts_num - 1].x < -900)
			{
				break;
			}
		}

		memcpy(temp_collision_arc.g_collision_lines[i], pts, pts_num * sizeof(COOR2));
		temp_collision_arc.g_collision_lines_num[i] = pts_num;
		temp_len = (theta) * radius;
		g_collision_arclen[i] = temp_len > 2400 ? 2400 : temp_len;
		g_morphin_collision_radius[i] = r_collision;
		g_morphin_include_radius[i] = (int)((r_include + GRID_LEN_PER_CELL - 1) / GRID_LEN_PER_CELL);
	}

	{
		g_morphin_collision_radius[MORPHIN_MID_INDEX] = 1000000;
		//[2400cm]
		step = 25;//2000 / (199);
		for (i=0; i<96; i++)
		{
			pts[i].x = 0;
			pts[i].y = (INT32)(i * step);
		}
		pts_num = 96;
		memcpy(temp_collision_arc.g_collision_lines[MORPHIN_MID_INDEX], pts, sizeof(COOR2) * pts_num);
		temp_collision_arc.g_collision_lines_num[MORPHIN_MID_INDEX] = pts_num;
		g_collision_arclen[MORPHIN_MID_INDEX] = 2400;
		g_morphin_include_radius[MORPHIN_MID_INDEX] = 4;
	}

	//[ʹ�û������ҵȳ�]
	for (i=0; i<MORPHIN_MID_INDEX; i++)
	{
		for (j=0; j<temp_collision_arc.g_collision_lines_num[i]; j++)
		{
			temp_collision_arc.g_collision_lines[MORPHIN_LINE_NUM - i - 1][j].x = -temp_collision_arc.g_collision_lines[i][j].x;
			temp_collision_arc.g_collision_lines[MORPHIN_LINE_NUM - i - 1][j].y = temp_collision_arc.g_collision_lines[i][j].y;
		}
		temp_collision_arc.g_collision_lines_num[MORPHIN_LINE_NUM - i - 1] = temp_collision_arc.g_collision_lines_num[i];

		g_collision_arclen[MORPHIN_LINE_NUM - i - 1] = g_collision_arclen[i];
		g_morphin_collision_radius[MORPHIN_LINE_NUM - i - 1] = g_morphin_collision_radius[i];
		g_morphin_include_radius[MORPHIN_LINE_NUM - i - 1] = g_morphin_include_radius[i];
	}

	memcpy(&g_collision_arc, &temp_collision_arc, sizeof(COLLISION_ARC));
	return ret;
}

typedef struct
{
	int l_index;
	int r_index;
	int delta;
}AREA_VALUE;//[��ͨ������]
AREA_VALUE area_value[MORPHIN_LINE_NUM];
int area_value_num = 0;

//***********************************************************************************************
//                                zgccmax 2012.Mar.4
//int get_best_morphin_line(COOR2 sub_goal_pt)
//param:    COOR2 sub_goal_pt		��������ϵ����Ŀ���
//			double &out_rate			��ѡ��·����ͨ�б���
//return:   int						0-23 �ʺ�ͨ��·��������  -1 û�к��ʵ�
//discribe: ͨ�������ʱ��ʹ��Morphin�㷨��ѡȡ��ͨ��·��  **·��Ŀǰ�̶����ɵ���ͷǰ��16m������
//			��ⷶΧ��g_morphin_dist
//***********************************************************************************************
int get_best_morphin_line2(COOR2 sub_goal_pt, double &out_rate)
{
	int ret = -1;
	int i, j;

	//[0�ǹ켣�������ϰ���ľ���]
	//[1�ǹ켣�ߵ����ϰ��ﾭ����դ����/�켣����դ����]
	//[2�ǹ켣�ߵĿ�ͨ����  ֵԽСԽ��]
	//[3�ǹ켣�߲���ƽ�صľ���]
	double dist[MORPHIN_LINE_NUM][4];
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	int x;
	int y;
	double xx, yy;

	double dist_8m[MORPHIN_LINE_NUM];
	double dist_12m[MORPHIN_LINE_NUM];
	double dist_16m[MORPHIN_LINE_NUM];
	double dist_20m[MORPHIN_LINE_NUM];

	int flag;
	int flag_8m;
	int flag_12m;
	int flag_16m;
	int flag_20m;

	MORPHIN2 morphin = g_morhpin2[0];

	//[��ʼ��]
	double dist2[MORPHIN_LINE_NUM];
	memset(dist, 0, sizeof(double) * 4 * MORPHIN_LINE_NUM);
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		dist[i][0] = g_morphin_arclen2[i];
		dist[i][1] = 0;
		dist[i][2] = 0;
		dist[i][3] = g_morphin_arclen2[i];

		dist_8m[i] = g_morphin_arclen2[i];
		dist_12m[i] = g_morphin_arclen2[i];
		dist_16m[i] = g_morphin_arclen2[i];
		dist_20m[i] = g_morphin_arclen2[i];
	}


	memset(g_value, 0, MORPHIN_LINE_NUM * sizeof(double));

	int data_morphin[MORPHIN_LINE_NUM][200];
	memset(data_morphin, 0, MORPHIN_LINE_NUM * 200 * sizeof(int));

	//[�ϰ���ɨ��]
	for (j=0; j<MORPHIN_LINE_NUM; j++)
	{
		memset(temp_pts, 0, 200 * sizeof(COOR2));
		temp_pts_num = 0;

		memcpy(temp_pts, morphin.g_morphin_lines[j], morphin.g_morphin_lines_num[j] * sizeof(COOR2));
		temp_pts_num = morphin.g_morphin_lines_num[j];

		flag = 1;
		flag_8m = 1;
		flag_12m = 1;
		flag_16m = 1;
		flag_20m = 1;
		for (i=0; i<temp_pts_num; i++)//[ɨ����������]
		{
			x = temp_pts[i].x / GRID_LEN_PER_CELL + g_grid_center.x;
			y = temp_pts[i].y / GRID_LEN_PER_CELL + g_grid_center.y;

			data_morphin[j][i] = g_closed_grid_map[y][x];

			dist[j][1]++;
			dist[j][2] -= (8.0 - g_closed_grid_map[y][x]) / (i+1);

			if (g_closed_grid_map[y][x] > 4 && g_closed_grid_map[y][x] < 8 && flag == 1)
			{//[6��7����Զ���ϰ���8]
				int l_dist;
				xx = (double)temp_pts[i].x;
				yy = (double)temp_pts[i].y - 400;

				l_dist = (int)sqrt(xx * xx + yy * yy);
				dist[j][3] = l_dist;//[ֱ�Ӽ����ϰ���㵽��ͷ���룬�е㲻��ȷ����]

				flag = 0;
			}

			if (g_closed_grid_map[y][x] > 4 && g_closed_grid_map[y][x] <= 8 && flag_8m == 1)
			{//[6��7����Զ���ϰ���8]
				int l_dist;
				xx = (double)temp_pts[i].x;
				yy = (double)temp_pts[i].y - 400;

				l_dist = (int)sqrt(xx * xx + yy * yy);
				dist_8m[j] = l_dist >= 800 ? l_dist : 0;
				flag_8m = 0;
			}

			if (g_closed_grid_map[y][x] > 4 && g_closed_grid_map[y][x] <= 8 && flag_12m == 1)
			{//[6��7����Զ���ϰ���8]
				int l_dist;
				xx = (double)temp_pts[i].x;
				yy = (double)temp_pts[i].y - 400;

				l_dist = (int)sqrt(xx * xx + yy * yy);
				dist_12m[j] = l_dist >= 1200 ? l_dist : 0;
				flag_12m = 0;
			}

			if (g_closed_grid_map[y][x] > 4 && g_closed_grid_map[y][x] <= 8 && flag_16m == 1)
			{//[6��7����Զ���ϰ���8]
				int l_dist;
				xx = (double)temp_pts[i].x;
				yy = (double)temp_pts[i].y - 400;

				l_dist = (int)sqrt(xx * xx + yy * yy);
				dist_16m[j] = l_dist >= 1600 ? l_dist : 0;
				flag_16m = 0;
			}

			if (g_closed_grid_map[y][x] > 4 && g_closed_grid_map[y][x] <= 8 && flag_20m == 1)
			{//[6��7����Զ���ϰ���8]
				int l_dist;
				xx = (double)temp_pts[i].x;
				yy = (double)temp_pts[i].y - 400;

				l_dist = (int)sqrt(xx * xx + yy * yy);
				dist_20m[j] = l_dist >= 2000 ? l_dist : 0;
				flag_20m = 0;
			}

			if (g_closed_grid_map[y][x] == 8)
			{
				int l_dist;
				xx = (double)temp_pts[i].x;
				yy = (double)temp_pts[i].y - 400;

				l_dist = (int)sqrt(xx * xx + yy * yy);
				dist[j][0] = l_dist;//[ֱ�Ӽ����ϰ���㵽��ͷ���룬�е㲻��ȷ����]

				dist[j][1] = dist[j][1] / morphin.g_morphin_lines_num[j];
				dist[j][2] = (1 - dist[j][2] / dist2[j]);

				break;
			}
		}

		if (dist[j][0] == g_morphin_arclen2[j])//[δ�����ϰ�������ͨ����]
		{
			dist[j][1] = 1;
			dist[j][2] = (1 - dist[j][2] / dist2[j]);
		}

		dist[j][3] = dist[j][3];
	}

	int goal_index = -1;

	double radian;
	double radius;
	double min_radius = 100000000;

	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;

	double D = 0;
	double car_len = 400;
	//[ȷ��Ŀ������ڹ켣���±�]
	if (fabs(xx) <= 10)//|| xx < 10)//fabs(yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
	{
		//[Ŀ�����ǰ����ֱ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = PI / 2;

		radian = radian / PI * 180;
	}
	else
	{
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx < 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radius = -sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radius = sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 - (radian / PI * 180);
		}

		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (i==MORPHIN_MID_INDEX)
				continue;

			if (fabs(radius - g_morphin_radius2[i]) < min_radius)
			{
				min_radius = fabs(radius - g_morphin_radius2[i]);
				goal_index = i;
			}
		}
	}



	//[���й���]
	int sum_8m;
	int sum_12m;
	int sum_16m;
	int sum_20m;
	double dist_8m_temp[MORPHIN_LINE_NUM];
	double dist_12m_temp[MORPHIN_LINE_NUM];
	double dist_16m_temp[MORPHIN_LINE_NUM];
	double dist_20m_temp[MORPHIN_LINE_NUM];
	memset(dist_8m_temp, 0, MORPHIN_LINE_NUM * sizeof(double));
	memset(dist_12m_temp, 0, MORPHIN_LINE_NUM * sizeof(double));
	memset(dist_16m_temp, 0, MORPHIN_LINE_NUM * sizeof(double));
	memset(dist_20m_temp, 0, MORPHIN_LINE_NUM * sizeof(double));
	int win = 2;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		sum_8m = 0;
		sum_12m = 0;
		for (j=i-win; j<=i+win; j++)
		{
			if (j<0 || j>= MORPHIN_LINE_NUM)
				continue;

			if (dist_8m[j] < 800)
				sum_8m++;

			if (dist_12m[j] < 1200)
				sum_12m++;
		}

		if (sum_8m > 0)
			dist_8m_temp[i] = 0;
		else
			dist_8m_temp[i] = dist_8m[i];

		if (sum_12m > 0)
			dist_12m_temp[i] = 0;
		else
			dist_12m_temp[i] = dist_12m[i];
	}

	win = 1;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		sum_16m = 0;
		sum_20m = 0;
		for (j=i-win; j<=i+win; j++)
		{
			if (j<0 || j>= MORPHIN_LINE_NUM)
				continue;

			if (dist_16m[j] < 1600)
				sum_16m++;

			if (dist_20m[j] < 2000)
				sum_20m++;
		}

		if (sum_16m > 0)
			dist_16m_temp[i] = 0;
		else
			dist_16m_temp[i] = dist_16m[i];

		// 		if (sum_20m > 0)
		// 			dist_20m_temp[i] = 0;
		// 		else
		// 			dist_20m_temp[i] = dist_20m[i];
	}

	memcpy(dist_8m, dist_8m_temp, MORPHIN_LINE_NUM * sizeof(double));
	memcpy(dist_12m, dist_12m_temp, MORPHIN_LINE_NUM * sizeof(double));
	memcpy(dist_16m, dist_16m_temp, MORPHIN_LINE_NUM * sizeof(double));
	memcpy(dist_20m, dist_20m_temp, MORPHIN_LINE_NUM * sizeof(double));

	int nums = 0;

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_8m[i] == 0)
			nums++;
	}

	int flag_area = 0;
	int max_index = -1;
	int max_delta = -1;


	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	area_value_num = 0;

	flag_area = 0;
	max_index = -1;
	max_delta = -1;

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_8m[i] >= 800 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
		{
			area_value[area_value_num].delta++;
		}

		else if (dist_8m[i] >= 800 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
		{
			flag_area = 0;
			area_value[area_value_num].r_index = i - 1;
			area_value_num++;
		}

		else if (dist_8m[i] >= 800 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
		{
			flag_area = 1;
			area_value[area_value_num].l_index = i;
			area_value[area_value_num].delta++;
		}

		else if (dist_8m[i] >= 800 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
		{
			area_value[area_value_num].l_index = i;
			area_value[area_value_num].r_index = i;
			area_value[area_value_num].delta++;
			area_value_num++;
		}

		else if (dist_8m[i] < 800 && flag_area == 1)
		{
			flag_area = 0;
			area_value[area_value_num].r_index = i - 1;
			area_value_num++;
		}
	}

	for (i=0; i<area_value_num; i++)
	{
		if (area_value[i].delta > max_delta)
		{
			max_delta = area_value[i].delta;
			max_index = i;
		}
	}


	if (max_index != -1)
	{

		if (area_value[max_index].delta < (MORPHIN_LINE_NUM / 3))
		{
			goal_index = area_value[max_index].l_index + area_value[max_index].delta / 2;

			g_cross_last_index = goal_index;
			g_morhpin2[0].g_best_index = goal_index;
			ret = goal_index;
			out_rate = dist[goal_index][1];
			return ret;
		}

	}
	else
	{
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_morhpin2[0].g_best_index = -1;
		out_rate = 0;
		ret = -1;
		return ret;
	}



	// 
	// 	if (flag_obs == 1)
	// 	{
	// 		for (i=0; i<MORPHIN_MID_INDEX; i++)
	// 		{
	// 			dist_8m[i] = 0;
	// 			dist_12m[i] = 0;
	// 			dist_16m[i] = 0;
	// 			dist_20m[i] = 0;
	// 		}
	// 	}

	//[̽������]
	//[20m]
	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	area_value_num = 0;

	flag_area = 0;
	max_index = -1;
	max_delta = -1;

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_20m[i] >= 2000 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
		{
			area_value[area_value_num].delta++;
		}

		else if (dist_20m[i] >= 2000 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
		{
			flag_area = 0;
			area_value[area_value_num].r_index = i - 1;
			area_value_num++;
		}

		else if (dist_20m[i] >= 2000 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
		{
			flag_area = 1;
			area_value[area_value_num].l_index = i;
			area_value[area_value_num].delta++;
		}

		else if (dist_20m[i] >= 2000 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
		{
			area_value[area_value_num].l_index = i;
			area_value[area_value_num].r_index = i;
			area_value[area_value_num].delta++;
			area_value_num++;
		}

		else if (dist_20m[i] < 2000 && flag_area == 1)
		{
			flag_area = 0;
			area_value[area_value_num].r_index = i - 1;
			area_value_num++;
		}
	}

	for (i=0; i<area_value_num; i++)
	{
		if (area_value[i].delta > max_delta)
		{
			max_delta = area_value[i].delta;
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		if (goal_index < area_value[max_index].l_index || goal_index > area_value[max_index].r_index)
		{
			goal_index = area_value[max_index].l_index + area_value[max_index].delta / 2;
		}

		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		out_rate = dist[goal_index][1];
		return ret;
	}


	//[16m]
	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	area_value_num = 0;

	flag_area = 0;
	max_index = -1;
	max_delta = -1;

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_16m[i] >= 1600 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
		{
			area_value[area_value_num].delta++;
		}

		else if (dist_16m[i] >= 1600 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
		{
			flag_area = 0;
			area_value[area_value_num].r_index = i - 1;
			area_value_num++;
		}

		else if (dist_16m[i] >= 1600 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
		{
			flag_area = 1;
			area_value[area_value_num].l_index = i;
			area_value[area_value_num].delta++;
		}

		else if (dist_16m[i] >= 1600 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
		{
			area_value[area_value_num].l_index = i;
			area_value[area_value_num].r_index = i;
			area_value[area_value_num].delta++;
			area_value_num++;
		}

		else if (dist_16m[i] < 1600 && flag_area == 1)
		{
			flag_area = 0;
			area_value[area_value_num].r_index = i - 1;
			area_value_num++;
		}
	}

	for (i=0; i<area_value_num; i++)
	{
		if (area_value[i].delta > max_delta)
		{
			max_delta = area_value[i].delta;
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		if (goal_index < area_value[max_index].l_index || goal_index > area_value[max_index].r_index)
		{
			goal_index = area_value[max_index].l_index + area_value[max_index].delta / 2;
		}

		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		out_rate = dist[goal_index][1];
		return ret;
	}


	//[12m]
	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	area_value_num = 0;

	flag_area = 0;
	max_index = -1;
	max_delta = -1;

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_12m[i] >= 1200 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
		{
			area_value[area_value_num].delta++;
		}

		else if (dist_12m[i] >= 1200 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
		{
			flag_area = 0;
			area_value[area_value_num].r_index = i - 1;
			area_value_num++;
		}

		else if (dist_12m[i] >= 1200 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
		{
			flag_area = 1;
			area_value[area_value_num].l_index = i;
			area_value[area_value_num].delta++;
		}

		else if (dist_12m[i] >= 1200 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
		{
			area_value[area_value_num].l_index = i;
			area_value[area_value_num].r_index = i;
			area_value[area_value_num].delta++;
			area_value_num++;
		}

		else if (dist_12m[i] < 1200 && flag_area == 1)
		{
			flag_area = 0;
			area_value[area_value_num].r_index = i - 1;
			area_value_num++;
		}
	}

	for (i=0; i<area_value_num; i++)
	{
		if (area_value[i].delta > max_delta)
		{
			max_delta = area_value[i].delta;
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		if (goal_index < area_value[max_index].l_index || goal_index > area_value[max_index].r_index)
		{
			goal_index = area_value[max_index].l_index + area_value[max_index].delta / 2;
		}

		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		out_rate = dist[goal_index][1];
		return ret;
	}



	//[8m]
	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	area_value_num = 0;

	flag_area = 0;
	max_index = -1;
	max_delta = -1;

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_8m[i] >= 800 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
		{
			area_value[area_value_num].delta++;
		}

		else if (dist_8m[i] >= 800 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
		{
			flag_area = 0;
			area_value[area_value_num].r_index = i - 1;
			area_value_num++;
		}

		else if (dist_8m[i] >= 800 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
		{
			flag_area = 1;
			area_value[area_value_num].l_index = i;
			area_value[area_value_num].delta++;
		}

		else if (dist_8m[i] >= 800 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
		{
			area_value[area_value_num].l_index = i;
			area_value[area_value_num].r_index = i;
			area_value[area_value_num].delta++;
			area_value_num++;
		}

		else if (dist_8m[i] < 800 && flag_area == 1)
		{
			flag_area = 0;
			area_value[area_value_num].r_index = i - 1;
			area_value_num++;
		}
	}

	for (i=0; i<area_value_num; i++)
	{
		if (area_value[i].delta > max_delta)
		{
			max_delta = area_value[i].delta;
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		if (goal_index < area_value[max_index].l_index || goal_index > area_value[max_index].r_index)
		{
			goal_index = area_value[max_index].l_index + area_value[max_index].delta / 2;
		}

		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		out_rate = dist[goal_index][1];
		return ret;
	}
	else
	{
		g_cross_last_index = MORPHIN_MID_INDEX;
		g_morhpin2[0].g_best_index = -1;
		out_rate = 0;
		ret = -1;
		return ret;
	}
}

//[Ŀ�������]
int get_best_morphin_line(COOR2 sub_goal_pt, double &out_rate)
{
	int ret = -1;
	int i, j;

	//[0�ǹ켣�������ϰ���ľ���]
	//[1�ǹ켣�ߵ����ϰ��ﾭ����դ����/�켣����դ����]
	//[2�ǹ켣�ߵĿ�ͨ����  ֵԽСԽ��]
	//[3�ǹ켣�߲���ƽ�صľ���]
	double dist[MORPHIN_LINE_NUM][4];
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	int x;
	int y;
	double xx, yy;

	double dist_8m[MORPHIN_LINE_NUM];
	double dist_12m[MORPHIN_LINE_NUM];
	double dist_15m[MORPHIN_LINE_NUM];
	double dist_18m[MORPHIN_LINE_NUM];

	int flag;
//	int flag_8m;
//	int flag_12m;
//	int flag_15m;
//	int flag_18m;

	MORPHIN2 morphin = g_morhpin2[0];

	//[��ʼ��]
	double dist2[MORPHIN_LINE_NUM];
	memset(dist, 0, sizeof(double) * 4 * MORPHIN_LINE_NUM);
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		dist[i][0] = g_morphin_arclen2[i];
		dist[i][1] = 0;
		dist[i][2] = 0;
		dist[i][3] = g_morphin_arclen2[i];

		dist_8m[i] = 2000;
		dist_12m[i] = 2000;
		dist_15m[i] = 2000;
		dist_18m[i] = 2000;
	}


	memset(g_value, 0, MORPHIN_LINE_NUM * sizeof(double));

	int data_morphin[MORPHIN_LINE_NUM][200];
	memset(data_morphin, 0, MORPHIN_LINE_NUM * 200 * sizeof(int));

	//[�ϰ���ɨ��]
	for (j=0; j<MORPHIN_LINE_NUM; j++)
	{
		memset(temp_pts, 0, 200 * sizeof(COOR2));
		temp_pts_num = 0;

		memcpy(temp_pts, morphin.g_morphin_lines[j], morphin.g_morphin_lines_num[j] * sizeof(COOR2));
		temp_pts_num = morphin.g_morphin_lines_num[j];

		flag = 1;
// 		flag_8m = 1;
// 		flag_12m = 1;
// 		flag_15m = 1;
// 		flag_18m = 1;
		int flag2 = 1;
		for (i=0; i<temp_pts_num; i++)//[ɨ����������]
		{
			x = temp_pts[i].x / GRID_LEN_PER_CELL + g_grid_center.x;
			y = temp_pts[i].y / GRID_LEN_PER_CELL + g_grid_center.y;

			data_morphin[j][i] = g_closed_grid_map[y][x];

			dist[j][1]++;
			dist[j][2] -= (8.0 - g_closed_grid_map[y][x]) / (i+1);

			if (g_closed_grid_map[y][x] > 4 && g_closed_grid_map[y][x] < 8 && flag == 1)
			{//[6��7����Զ���ϰ���8]
				int l_dist;
				xx = (double)temp_pts[i].x;
				yy = (double)temp_pts[i].y - 400;

				l_dist = (int)sqrt(xx * xx + yy * yy);
				dist[j][3] = l_dist;//[ֱ�Ӽ����ϰ���㵽��ͷ���룬�е㲻��ȷ����]

				flag = 0;
			}



			if (g_closed_grid_map[y][x] > 4 && g_closed_grid_map[y][x] <= 8 && flag2 == 1)
			{//[6��7����Զ���ϰ���8]
				int l_dist;
				xx = (double)temp_pts[i].x;
				yy = (double)temp_pts[i].y - 400;

				l_dist = (int)sqrt(xx * xx + yy * yy);

				if (l_dist >=1800)
				{
					dist_8m[j] = l_dist;
					dist_12m[j] = l_dist;
					dist_15m[j] = l_dist;
					dist_18m[j] = l_dist;
				}
				else if (l_dist >= 1500)
				{
					dist_8m[j] = l_dist;
					dist_12m[j] = l_dist;
					dist_15m[j] = l_dist;
					dist_18m[j] = 0;
				}
				else if (l_dist >= 1200)
				{
					dist_8m[j] = l_dist;
					dist_12m[j] = l_dist;
					dist_15m[j] = 0;
					dist_18m[j] = 0;
				}
				else if (l_dist >= 800)
				{
					dist_8m[j] = l_dist;
					dist_12m[j] = 0;
					dist_15m[j] = 0;
					dist_18m[j] = 0;
				}
				else
				{
					dist_8m[j] = 0;
					dist_12m[j] = 0;
					dist_15m[j] = 0;
					dist_18m[j] = 0;
				}

				flag2 = 0;
			}

			// 
			// 			if (g_closed_grid_map[y][x] > 4 && g_closed_grid_map[y][x] <= 8 && flag_9m == 1)
			// 			{//[6��7����Զ���ϰ���8]
			// 				int l_dist;
			// 				xx = (double)temp_pts[i].x;
			// 				yy = (double)temp_pts[i].y - 400;
			// 
			// 				l_dist = (int)sqrt(xx * xx + yy * yy);
			// 				dist_9m[j] = l_dist >= 900 ? l_dist : 0;
			// 				flag_9m = 0;
			// 			}
			// 
			// 			if (g_closed_grid_map[y][x] > 4 && g_closed_grid_map[y][x] <= 8 && flag_12m == 1)
			// 			{//[6��7����Զ���ϰ���8]
			// 				int l_dist;
			// 				xx = (double)temp_pts[i].x;
			// 				yy = (double)temp_pts[i].y - 400;
			// 
			// 				l_dist = (int)sqrt(xx * xx + yy * yy);
			// 				dist_12m[j] = l_dist >= 1200 ? l_dist : 0;
			// 				flag_12m = 0;
			// 			}
			// 
			// 			if (g_closed_grid_map[y][x] > 4 && g_closed_grid_map[y][x] <= 8 && flag_15m == 1)
			// 			{//[6��7����Զ���ϰ���8]
			// 				int l_dist;
			// 				xx = (double)temp_pts[i].x;
			// 				yy = (double)temp_pts[i].y - 400;
			// 
			// 				l_dist = (int)sqrt(xx * xx + yy * yy);
			// 				dist_15m[j] = l_dist >= 1500 ? l_dist : 0;
			// 				flag_15m = 0;
			// 			}
			// 
			// 			if (g_closed_grid_map[y][x] > 4 && g_closed_grid_map[y][x] <= 8 && flag_18m == 1)
			// 			{//[6��7����Զ���ϰ���8]
			// 				int l_dist;
			// 				xx = (double)temp_pts[i].x;
			// 				yy = (double)temp_pts[i].y - 400;
			// 
			// 				l_dist = (int)sqrt(xx * xx + yy * yy);
			// 				dist_18m[j] = l_dist >= 1800 ? l_dist : 0;
			// 				flag_18m = 0;
			// 			}

			if (g_closed_grid_map[y][x] == 8)
			{
				int l_dist;
				xx = (double)temp_pts[i].x;
				yy = (double)temp_pts[i].y - 400;

				l_dist = (int)sqrt(xx * xx + yy * yy);
				dist[j][0] = l_dist;//[ֱ�Ӽ����ϰ���㵽��ͷ���룬�е㲻��ȷ����]

				dist[j][1] = dist[j][1] / morphin.g_morphin_lines_num[j];
				dist[j][2] = (1 - dist[j][2] / dist2[j]);

				break;
			}
		}

		if (flag2 == 1)
		{

			int l_dist;
			xx = (double)temp_pts[temp_pts_num - 1].x;
			yy = (double)temp_pts[temp_pts_num - 1].y - 400;

			l_dist = (int)sqrt(xx * xx + yy * yy);

			if (l_dist >=1800)
			{
				dist_8m[j] = l_dist;
				dist_12m[j] = l_dist;
				dist_15m[j] = l_dist;
				dist_18m[j] = l_dist;
			}
			else if (l_dist >= 1500)
			{
				dist_8m[j] = l_dist;
				dist_12m[j] = l_dist;
				dist_15m[j] = l_dist;
				dist_18m[j] = 0;
			}
			else if (l_dist >= 1200)
			{
				dist_8m[j] = l_dist;
				dist_12m[j] = l_dist;
				dist_15m[j] = 0;
				dist_18m[j] = 0;
			}
			else if (l_dist >= 800)
			{
				dist_8m[j] = l_dist;
				dist_12m[j] = 0;
				dist_15m[j] = 0;
				dist_18m[j] = 0;
			}
			else
			{
				dist_8m[j] = 0;
				dist_12m[j] = 0;
				dist_15m[j] = 0;
				dist_18m[j] = 0;
			}
		}

		if (dist[j][0] == g_morphin_arclen2[j])//[δ�����ϰ�������ͨ����]
		{
			dist[j][1] = 1;
			dist[j][2] = (1 - dist[j][2] / dist2[j]);
		}

		dist[j][3] = dist[j][3];
	}

	int goal_index = -1;

	double radian;
	double radius;
	double min_radius = 100000000;

	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;

	double D = 0;
	double car_len = 400;
	//[ȷ��Ŀ������ڹ켣���±�]
	if (fabs(xx) <= 10)//|| xx < 10)//fabs(yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
	{
		//[Ŀ�����ǰ����ֱ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = PI / 2;

		radian = radian / PI * 180;
	}
	else
	{
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx < 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radius = -sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radius = sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 - (radian / PI * 180);
		}

		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (i==MORPHIN_MID_INDEX)
				continue;

			if (fabs(radius - g_morphin_radius2[i]) < min_radius)
			{
				min_radius = fabs(radius - g_morphin_radius2[i]);
				goal_index = i;
			}
		}
	}



	//[���й���]
	int sum_8m;
	int sum_12m;
	int sum_15m;
	int sum_18m;
	double dist_8m_temp[MORPHIN_LINE_NUM];
	double dist_12m_temp[MORPHIN_LINE_NUM];
	double dist_15m_temp[MORPHIN_LINE_NUM];
	double dist_18m_temp[MORPHIN_LINE_NUM];
	memset(dist_8m_temp, 0, MORPHIN_LINE_NUM * sizeof(double));
	memset(dist_12m_temp, 0, MORPHIN_LINE_NUM * sizeof(double));
	memset(dist_15m_temp, 0, MORPHIN_LINE_NUM * sizeof(double));
	memset(dist_18m_temp, 0, MORPHIN_LINE_NUM * sizeof(double));
	int win = 2;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		sum_8m = 0;
		sum_12m = 0;
		sum_15m = 0;
		sum_18m = 0;
		for (j=i-win; j<=i+win; j++)
		{
			if (j<0 || j>= MORPHIN_LINE_NUM)
				continue;

			if (dist_8m[j] < 800)
				sum_8m++;

			if (dist_12m[j] < 1200)
				sum_12m++;

			if (dist_15m[j] < 1500)
				sum_15m++;

			if (dist_18m[j] < 1800)
				sum_18m++;
		}

		if (sum_8m > 0)
			dist_8m_temp[i] = 0;
		else
			dist_8m_temp[i] = dist_8m[i];

		if (sum_12m > 0)
			dist_12m_temp[i] = 0;
		else
			dist_12m_temp[i] = dist_12m[i];

		if (sum_15m > 0)
			dist_15m_temp[i] = 0;
		else
			dist_15m_temp[i] = dist_15m[i];

		if (sum_18m > 0)
			dist_18m_temp[i] = 0;
		else
			dist_18m_temp[i] = dist_18m[i];
	}

	memcpy(dist_8m, dist_8m_temp, MORPHIN_LINE_NUM * sizeof(double));
	memcpy(dist_12m, dist_12m_temp, MORPHIN_LINE_NUM * sizeof(double));
	memcpy(dist_15m, dist_15m_temp, MORPHIN_LINE_NUM * sizeof(double));
	memcpy(dist_18m, dist_18m_temp, MORPHIN_LINE_NUM * sizeof(double));


	int max_index;
	int min_delta = MAX_VALUE;

	//[̽������]
	//[18m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_8m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[15m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_12m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[12m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_15m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[9m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_18m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	else
	{
		g_cross_last_index = -1;
		g_morhpin2[0].g_best_index = -1;
		out_rate = 0;
		ret = -1;
		return ret;
	}
}

int get_best_morphin_line3(COOR2 sub_goal_pt, double &out_rate)
{
	int ret = -1;

	int goal_index = -1;

	int i, j, k;
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	int x;
	int y;
	double dist;
	double xx, yy;

	MORPHIN2 morphin = g_morhpin2[0];

	int value[MORPHIN_LINE_NUM];
	int num = 0;
	memset(value, 0, MORPHIN_LINE_NUM * sizeof(int));
	int l_dist = 0;
	int r_dist = 0;

	double dist_9m[MORPHIN_LINE_NUM];
	double dist_12m[MORPHIN_LINE_NUM];
	double dist_15m[MORPHIN_LINE_NUM];
	double dist_18m[MORPHIN_LINE_NUM];

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		dist_9m[i] = 900;
		dist_12m[i] = 1200;
		dist_15m[i] = 1500;
		dist_18m[i] = 1800;
	}

	//[�ϰ���ɨ��]
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		memset(temp_pts, 0, 200 * sizeof(COOR2));
		temp_pts_num = 0;

		memcpy(temp_pts, morphin.g_morphin_lines[i], morphin.g_morphin_lines_num[i] * sizeof(COOR2));
		temp_pts_num = morphin.g_morphin_lines_num[i];

		num = 0;
		for (j=0; j<temp_pts_num; j++)//[ɨ����������]
		{
			num++;
			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x-1);
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;

			l_dist = 0;
			for (k=x; k>=x-6; k--)
			{
				l_dist = x - k;

				if (g_grid_map[y][k] == 8)
					break;
			}

			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x+1);
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
			r_dist = 0;
			for (k=x; k<=x+6; k++)
			{
				r_dist = k - x;

				if (g_grid_map[y][k] == 8)
					break;
			}

			xx = (double)temp_pts[j].x;
			yy = (double)temp_pts[j].y - 400;
			dist = sqrt(xx * xx + yy * yy);

			if (dist < 800 && (l_dist < 6 || r_dist < 6))
			{
				dist_9m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;

				value[i] = 0;
				break;
			}
			else if (l_dist < 6 || r_dist < 6)
			{
				if (dist >=1800)
				{
					dist_9m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = dist;
				}
				else if (dist >= 1500)
				{
					dist_9m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = 0;
				}
				else if (dist >= 1200)
				{
					dist_9m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else if (dist >= 900)
				{
					dist_9m[i] = dist;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else
				{
					dist_9m[i] = 0;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}

				break;
			}
			else
			{
				value[i] += l_dist;
				value[i] += r_dist;
			}
		}

		if (num == temp_pts_num)
		{
			if (i==MORPHIN_MID_INDEX)
			{
				dist_9m[i] = 2000;
				dist_12m[i] = 2000;
				dist_15m[i] = 2000;
				dist_18m[i] = 2000;
			}
			else if (dist >=1800)
			{
				dist_9m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = dist;
			}
			else if (dist >= 1500)
			{
				dist_9m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = 0;
			}
			else if (dist >= 1200)
			{
				dist_9m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else if (dist >= 900)
			{
				dist_9m[i] = dist;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else
			{
				dist_9m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
		}

		if (num != 0)
			value[i] = value[i] / num;
		else
			value[i] = 0;
	}


	double radian;
	double radius;
	double min_radius = 100000000;
	double D = 0;
	double car_len = 400;
	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;
	//[ȷ��Ŀ������ڹ켣���±�]
	if (fabs(xx) <= 10)//|| xx < 10)//fabs(yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
	{
		//[Ŀ�����ǰ����ֱ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = PI / 2;

		radian = radian / PI * 180;
	}
	else
	{
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx < 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radius = -sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radius = sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 - (radian / PI * 180);
		}

		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (i==MORPHIN_MID_INDEX)
				continue;

			if (fabs(radius - g_morphin_radius2[i]) < min_radius)
			{
				min_radius = fabs(radius - g_morphin_radius2[i]);
				goal_index = i;
			}
		}
	}

	int max_index = -1;
	int max_dist = -1;

	//[̽������]
	//[20m]
	max_dist = 0;
	max_index = -1;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_18m[i] > max_dist)
		{
			max_dist = (int)dist_18m[i];
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	// 	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	// 	area_value_num = 0;
	// 
	// 	flag_area = 0;
	// 	max_index = -1;
	// 	max_delta = -1;
	// 
	// 	for (i=0; i<MORPHIN_LINE_NUM; i++)
	// 	{
	// 		if (dist_18m[i] >= 1800 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_18m[i] >= 1800 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_18m[i] >= 1800 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			flag_area = 1;
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_18m[i] >= 1800 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].r_index = i;
	// 			area_value[area_value_num].delta++;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_18m[i] < 1800 && flag_area == 1)
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 	}
	// 
	// 	for (i=0; i<area_value_num; i++)
	// 	{
	// 		if (area_value[i].delta > max_delta)
	// 		{
	// 			max_delta = area_value[i].delta;
	// 			max_index = i;
	// 		}
	// 	}

	// 	if (max_index != -1)
	// 	{
	// 		for (i=area_value[max_index].l_index; i<=area_value[max_index].r_index; i++)
	// 		{
	// 			if (dist_18m[i] > max_dist)
	// 			{
	// 				goal_index = i;
	// 				max_dist = dist_18m[i];
	// 			}
	// 		}
	// 
	// 		g_cross_last_index = goal_index;
	// 		g_morhpin2[0].g_best_index = goal_index;
	// 		ret = goal_index;
	// 		//out_rate = dist[goal_index][1];
	// 		return ret;
	// 	}


	//[15m]
	max_dist = 0;
	max_index = -1;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_15m[i] > max_dist)
		{
			max_dist = (int)dist_15m[i];
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	// 
	// 	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	// 	area_value_num = 0;
	// 
	// 	flag_area = 0;
	// 	max_index = -1;
	// 	max_delta = -1;
	// 
	// 	for (i=0; i<MORPHIN_LINE_NUM; i++)
	// 	{
	// 		if (dist_15m[i] >= 1500 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_15m[i] >= 1500 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_15m[i] >= 1500 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			flag_area = 1;
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_15m[i] >= 1500 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].r_index = i;
	// 			area_value[area_value_num].delta++;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_15m[i] < 1500 && flag_area == 1)
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 	}
	// 
	// 	for (i=0; i<area_value_num; i++)
	// 	{
	// 		if (area_value[i].delta > max_delta)
	// 		{
	// 			max_delta = area_value[i].delta;
	// 			max_index = i;
	// 		}
	// 	}
	// 
	// 	if (max_index != -1)
	// 	{
	// 		for (i=area_value[max_index].l_index; i<=area_value[max_index].r_index; i++)
	// 		{
	// 			if (dist_15m[i] > max_dist)
	// 			{
	// 				goal_index = i;
	// 				max_dist = dist_15m[i];
	// 			}
	// 		}
	// 
	// 		g_cross_last_index = goal_index;
	// 		g_morhpin2[0].g_best_index = goal_index;
	// 		ret = goal_index;
	// 		//out_rate = dist[goal_index][1];
	// 		return ret;
	// 	}


	//[12m]
	max_dist = 0;
	max_index = -1;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_12m[i] > max_dist)
		{
			max_dist = (int)dist_12m[i];
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	// 
	// 	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	// 	area_value_num = 0;
	// 
	// 	flag_area = 0;
	// 	max_index = -1;
	// 	max_delta = -1;
	// 
	// 	for (i=0; i<MORPHIN_LINE_NUM; i++)
	// 	{
	// 		if (dist_12m[i] >= 1200 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_12m[i] >= 1200 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_12m[i] >= 1200 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			flag_area = 1;
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_12m[i] >= 1200 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].r_index = i;
	// 			area_value[area_value_num].delta++;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_12m[i] < 1200 && flag_area == 1)
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 	}
	// 
	// 	for (i=0; i<area_value_num; i++)
	// 	{
	// 		if (area_value[i].delta > max_delta)
	// 		{
	// 			max_delta = area_value[i].delta;
	// 			max_index = i;
	// 		}
	// 	}
	// 
	// 	if (max_index != -1)
	// 	{
	// 		for (i=area_value[max_index].l_index; i<=area_value[max_index].r_index; i++)
	// 		{
	// 			if (dist_12m[i] > max_dist)
	// 			{
	// 				goal_index = i;
	// 				max_dist = dist_12m[i];
	// 			}
	// 		}
	// 
	// 		g_cross_last_index = goal_index;
	// 		g_morhpin2[0].g_best_index = goal_index;
	// 		ret = goal_index;
	// 		//out_rate = dist[goal_index][1];
	// 		return ret;
	// 	}



	//[9m]
	max_dist = 0;
	max_index = -1;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_9m[i] > max_dist)
		{
			max_dist = (int)dist_9m[i];
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	else
	{
		g_cross_last_index = -1;
		g_morhpin2[0].g_best_index = -1;
		out_rate = 0;
		ret = -1;
		return ret;
	}

	// 	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	// 	area_value_num = 0;
	// 
	// 	flag_area = 0;
	// 	max_index = -1;
	// 	max_delta = -1;
	// 
	// 	for (i=0; i<MORPHIN_LINE_NUM; i++)
	// 	{
	// 		if (dist_9m[i] >= 900 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_9m[i] >= 900 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_9m[i] >= 900 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			flag_area = 1;
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_9m[i] >= 900 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].r_index = i;
	// 			area_value[area_value_num].delta++;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_9m[i] < 900 && flag_area == 1)
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 	}
	// 
	// 	for (i=0; i<area_value_num; i++)
	// 	{
	// 		if (area_value[i].delta > max_delta)
	// 		{
	// 			max_delta = area_value[i].delta;
	// 			max_index = i;
	// 		}
	// 	}
	// 
	// 	if (max_index != -1)
	// 	{
	// 		for (i=area_value[max_index].l_index; i<=area_value[max_index].r_index; i++)
	// 		{
	// 			if (dist_9m[i] > max_dist)
	// 			{
	// 				goal_index = i;
	// 				max_dist = dist_9m[i];
	// 			}
	// 		}
	// 
	// 		g_cross_last_index = goal_index;
	// 		g_morhpin2[0].g_best_index = goal_index;
	// 		ret = goal_index;
	// 		//out_rate = dist[goal_index][1];
	// 		return ret;
	// 	}
	// 	else
	// 	{
	// 		g_cross_last_index = -1;
	// 		g_morhpin2[0].g_best_index = -1;
	// 		out_rate = 0;
	// 		ret = -1;
	// 		return ret;
	// 	}


	// 	int max_index = -1;
	// 	int max_value = 0;
	// 	for (i=0; i<MORPHIN_LINE_NUM; i++)
	// 	{
	// 		if (value[i] > max_value)
	// 		{
	// 			max_index = i;
	// 			max_value = value[i];
	// 		}
	// 	}
	// 
	// 
	// 	int flag_area = 0;
	// 	
	// 	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	// 	area_value_num = 0;
	// 
	// 	for (i=0; i<MORPHIN_LINE_NUM; i++)
	// 	{
	// 		if (value[i] == max_value && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (value[i] == max_value && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (value[i] == max_value && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			flag_area = 1;
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (value[i] == max_value && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].r_index = i;
	// 			area_value[area_value_num].delta++;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (value[i] != max_value && flag_area == 1)
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 	}
	// 
	// 	int max_delta = -1;
	// 	max_index = -1;
	// 	double max_rate = -1;
	// 	int max_dist = -1;
	// 	for (i=0; i<area_value_num; i++)
	// 	{
	// 		for (j=area_value[i].l_index; j<=area_value[i].r_index; j++)
	// 		{
	// 			if (rate[j] > max_rate)
	// 			{
	// 				max_rate = rate[j];
	// 				max_index = j;
	// 			}
	//		}

	// 		if (area_value[i].delta > max_delta)
	// 		{
	// 			max_delta = area_value[i].delta;
	// 			max_index = i;
	// 		}
	//	}


	// 	if (max_index == -1)
	// 	{
	// 		ret = -1;
	// 	}
	// 	else
	// 	{
	// 		ret = max_index;
	//ret = (area_value[max_index].l_index + area_value[max_index].r_index) / 2;
	//	}


	return ret;
}

//[Զ������]
/*==================================================================
 * ������  ��	int get_best_morphin_line4(COOR2 sub_goal_pt, double &out_rate)
 * ����    ��	����ײ���߽������ۣ���ʻ���볤������
 * ���������	COOR2 sub_goal_pt		�ֲ���Ŀ���
 * ���������	double &out_rate		ѡ������ߵĿ�ͨ���ʣ�Ŀǰû���õ���
 * ����ֵ  ��	
 * ����    ��	
 * ����    ��	
 * �޸ļ�¼��	
 *==================================================================*/
int get_best_morphin_line4(COOR2 sub_goal_pt, double &out_rate)
{
	int ret = -1;

	int goal_index = -1;
	out_rate = 0;

	int i, j, k;
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	int x;
	int y;
	double dist;
	double xx, yy;

	MORPHIN2 morphin = g_morhpin2[0];

	int value[MORPHIN_LINE_NUM];
	int num = 0;
	memset(value, 0, MORPHIN_LINE_NUM * sizeof(int));
	int l_dist = 0;
	int r_dist = 0;

	double dist_8m[MORPHIN_LINE_NUM];
	double dist_12m[MORPHIN_LINE_NUM];
	double dist_15m[MORPHIN_LINE_NUM];
	double dist_18m[MORPHIN_LINE_NUM];

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		dist_8m[i] = 800;
		dist_12m[i] = 1200;
		dist_15m[i] = 1500;
		dist_18m[i] = 1800;
	}

	//[�ϰ���ɨ��]
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		memset(temp_pts, 0, 200 * sizeof(COOR2));
		temp_pts_num = 0;

		memcpy(temp_pts, morphin.g_morphin_lines[i], morphin.g_morphin_lines_num[i] * sizeof(COOR2));
		temp_pts_num = morphin.g_morphin_lines_num[i];

		num = 0;
		for (j=0; j<temp_pts_num; j++)//[ɨ����������]
		{
			num++;

			if (temp_pts[j].y <= 400)
			{
				continue;
			}
			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x - 1);
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;

			l_dist = 0;
			for (k=x; k>=x-6; k--)
			{
				l_dist = x - k;

				if (g_closed_grid_map[y][k] == 8)
					break;
			}

			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x + 1);
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
			r_dist = 0;
			for (k=x; k<=x+6; k++)
			{
				r_dist = k - x;

				if (g_closed_grid_map[y][k] == 8)
					break;
			}

			xx = (double)temp_pts[j].x;
			yy = (double)temp_pts[j].y - 400;
			dist = sqrt(xx * xx + yy * yy);

			if (dist < 800 && (l_dist < 6 || r_dist < 6))
			{
				dist_8m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;

				value[i] = 0;
				break;
			}
			else if (l_dist < 6 || r_dist < 6)
			{
				if (dist >=1800)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = dist;
				}
				else if (dist >= 1500)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = 0;
				}
				else if (dist >= 1200)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else if (dist >= 800)
				{
					dist_8m[i] = dist;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else
				{
					dist_8m[i] = 0;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}

				break;
			}
			else
			{
				value[i] += l_dist;
				value[i] += r_dist;
			}
		}

		if (num == temp_pts_num)
		{
			if (i==MORPHIN_MID_INDEX)
			{
				dist_8m[i] = 2000;
				dist_12m[i] = 2000;
				dist_15m[i] = 2000;
				dist_18m[i] = 2000;
			}
			else if (dist >=1800)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = dist;
			}
			else if (dist >= 1500)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = 0;
			}
			else if (dist >= 1200)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else if (dist >= 800)
			{
				dist_8m[i] = dist;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else
			{
				dist_8m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
		}

		if (num != 0)
			value[i] = value[i] / num;
		else
			value[i] = 0;
	}


	double radian;
	double radius;
	double min_radius = 100000000;
	double D = 0;
	double car_len = 400;
	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;
	//[ȷ��Ŀ������ڹ켣���±�]
	if (fabs(xx) <= 10)//|| xx < 10)//fabs(yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
	{
		//[Ŀ�����ǰ����ֱ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = PI / 2;

		radian = radian / PI * 180;
	}
	else
	{
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx < 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radius = -sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radius = sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 - (radian / PI * 180);
		}

		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (i==MORPHIN_MID_INDEX)
				continue;

			if (fabs(radius - g_morphin_radius2[i]) < min_radius)
			{
				min_radius = fabs(radius - g_morphin_radius2[i]);
				goal_index = i;
			}
		}
	}

	int max_index = -1;
	int min_delta = MAX_VALUE;

	//[̽������]
	//[18m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_18m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}


	// 	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	// 	area_value_num = 0;
	// 
	// 	flag_area = 0;
	// 	max_index = -1;
	// 	max_delta = -1;
	// 
	// 	for (i=0; i<MORPHIN_LINE_NUM; i++)
	// 	{
	// 		if (dist_18m[i] >= 1800 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_18m[i] >= 1800 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_18m[i] >= 1800 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			flag_area = 1;
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_18m[i] >= 1800 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].r_index = i;
	// 			area_value[area_value_num].delta++;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_18m[i] < 1800 && flag_area == 1)
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 	}
	// 
	// 	for (i=0; i<area_value_num; i++)
	// 	{
	// 		if (area_value[i].delta > max_delta)
	// 		{
	// 			max_delta = area_value[i].delta;
	// 			max_index = i;
	// 		}
	// 	}

	// 	if (max_index != -1)
	// 	{
	// 		for (i=area_value[max_index].l_index; i<=area_value[max_index].r_index; i++)
	// 		{
	// 			if (dist_18m[i] > max_dist)
	// 			{
	// 				goal_index = i;
	// 				max_dist = dist_18m[i];
	// 			}
	// 		}
	// 
	// 		g_cross_last_index = goal_index;
	// 		g_morhpin2[0].g_best_index = goal_index;
	// 		ret = goal_index;
	// 		//out_rate = dist[goal_index][1];
	// 		return ret;
	// 	}


	//[15m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_15m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	// 
	// 	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	// 	area_value_num = 0;
	// 
	// 	flag_area = 0;
	// 	max_index = -1;
	// 	max_delta = -1;
	// 
	// 	for (i=0; i<MORPHIN_LINE_NUM; i++)
	// 	{
	// 		if (dist_15m[i] >= 1500 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_15m[i] >= 1500 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_15m[i] >= 1500 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			flag_area = 1;
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_15m[i] >= 1500 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].r_index = i;
	// 			area_value[area_value_num].delta++;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_15m[i] < 1500 && flag_area == 1)
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 	}
	// 
	// 	for (i=0; i<area_value_num; i++)
	// 	{
	// 		if (area_value[i].delta > max_delta)
	// 		{
	// 			max_delta = area_value[i].delta;
	// 			max_index = i;
	// 		}
	// 	}
	// 
	// 	if (max_index != -1)
	// 	{
	// 		for (i=area_value[max_index].l_index; i<=area_value[max_index].r_index; i++)
	// 		{
	// 			if (dist_15m[i] > max_dist)
	// 			{
	// 				goal_index = i;
	// 				max_dist = dist_15m[i];
	// 			}
	// 		}
	// 
	// 		g_cross_last_index = goal_index;
	// 		g_morhpin2[0].g_best_index = goal_index;
	// 		ret = goal_index;
	// 		//out_rate = dist[goal_index][1];
	// 		return ret;
	// 	}


	//[12m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_12m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	// 
	// 	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	// 	area_value_num = 0;
	// 
	// 	flag_area = 0;
	// 	max_index = -1;
	// 	max_delta = -1;
	// 
	// 	for (i=0; i<MORPHIN_LINE_NUM; i++)
	// 	{
	// 		if (dist_12m[i] >= 1200 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_12m[i] >= 1200 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_12m[i] >= 1200 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			flag_area = 1;
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_12m[i] >= 1200 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].r_index = i;
	// 			area_value[area_value_num].delta++;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_12m[i] < 1200 && flag_area == 1)
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 	}
	// 
	// 	for (i=0; i<area_value_num; i++)
	// 	{
	// 		if (area_value[i].delta > max_delta)
	// 		{
	// 			max_delta = area_value[i].delta;
	// 			max_index = i;
	// 		}
	// 	}
	// 
	// 	if (max_index != -1)
	// 	{
	// 		for (i=area_value[max_index].l_index; i<=area_value[max_index].r_index; i++)
	// 		{
	// 			if (dist_12m[i] > max_dist)
	// 			{
	// 				goal_index = i;
	// 				max_dist = dist_12m[i];
	// 			}
	// 		}
	// 
	// 		g_cross_last_index = goal_index;
	// 		g_morhpin2[0].g_best_index = goal_index;
	// 		ret = goal_index;
	// 		//out_rate = dist[goal_index][1];
	// 		return ret;
	// 	}



	//[9m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_8m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	else
	{
		g_cross_last_index = -1;
		g_morhpin2[0].g_best_index = -1;
		out_rate = 0;
		ret = -1;
		return ret;
	}

	// 	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	// 	area_value_num = 0;
	// 
	// 	flag_area = 0;
	// 	max_index = -1;
	// 	max_delta = -1;
	// 
	// 	for (i=0; i<MORPHIN_LINE_NUM; i++)
	// 	{
	// 		if (dist_9m[i] >= 900 && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_9m[i] >= 900 && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_9m[i] >= 900 && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			flag_area = 1;
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (dist_9m[i] >= 900 && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].r_index = i;
	// 			area_value[area_value_num].delta++;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (dist_9m[i] < 900 && flag_area == 1)
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 	}
	// 
	// 	for (i=0; i<area_value_num; i++)
	// 	{
	// 		if (area_value[i].delta > max_delta)
	// 		{
	// 			max_delta = area_value[i].delta;
	// 			max_index = i;
	// 		}
	// 	}
	// 
	// 	if (max_index != -1)
	// 	{
	// 		for (i=area_value[max_index].l_index; i<=area_value[max_index].r_index; i++)
	// 		{
	// 			if (dist_9m[i] > max_dist)
	// 			{
	// 				goal_index = i;
	// 				max_dist = dist_9m[i];
	// 			}
	// 		}
	// 
	// 		g_cross_last_index = goal_index;
	// 		g_morhpin2[0].g_best_index = goal_index;
	// 		ret = goal_index;
	// 		//out_rate = dist[goal_index][1];
	// 		return ret;
	// 	}
	// 	else
	// 	{
	// 		g_cross_last_index = -1;
	// 		g_morhpin2[0].g_best_index = -1;
	// 		out_rate = 0;
	// 		ret = -1;
	// 		return ret;
	// 	}


	// 	int max_index = -1;
	// 	int max_value = 0;
	// 	for (i=0; i<MORPHIN_LINE_NUM; i++)
	// 	{
	// 		if (value[i] > max_value)
	// 		{
	// 			max_index = i;
	// 			max_value = value[i];
	// 		}
	// 	}
	// 
	// 
	// 	int flag_area = 0;
	// 	
	// 	memset(area_value, 0, MORPHIN_LINE_NUM * sizeof(AREA_VALUE));
	// 	area_value_num = 0;
	// 
	// 	for (i=0; i<MORPHIN_LINE_NUM; i++)
	// 	{
	// 		if (value[i] == max_value && flag_area == 1 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (value[i] == max_value && flag_area == 1 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (value[i] == max_value && flag_area == 0 && i < MORPHIN_LINE_NUM - 1)
	// 		{
	// 			flag_area = 1;
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].delta++;
	// 		}
	// 
	// 		else if (value[i] == max_value && flag_area == 0 && (i == MORPHIN_LINE_NUM - 1))
	// 		{
	// 			area_value[area_value_num].l_index = i;
	// 			area_value[area_value_num].r_index = i;
	// 			area_value[area_value_num].delta++;
	// 			area_value_num++;
	// 		}
	// 
	// 		else if (value[i] != max_value && flag_area == 1)
	// 		{
	// 			flag_area = 0;
	// 			area_value[area_value_num].r_index = i - 1;
	// 			area_value_num++;
	// 		}
	// 	}
	// 
	// 	int max_delta = -1;
	// 	max_index = -1;
	// 	double max_rate = -1;
	// 	int max_dist = -1;
	// 	for (i=0; i<area_value_num; i++)
	// 	{
	// 		for (j=area_value[i].l_index; j<=area_value[i].r_index; j++)
	// 		{
	// 			if (rate[j] > max_rate)
	// 			{
	// 				max_rate = rate[j];
	// 				max_index = j;
	// 			}
	//		}

	// 		if (area_value[i].delta > max_delta)
	// 		{
	// 			max_delta = area_value[i].delta;
	// 			max_index = i;
	// 		}
	//	}


	// 	if (max_index == -1)
	// 	{
	// 		ret = -1;
	// 	}
	// 	else
	// 	{
	// 		ret = max_index;
	//ret = (area_value[max_index].l_index + area_value[max_index].r_index) / 2;
	//	}


	return ret;
}

//[��������]
int get_best_morphin_line5(COOR2 sub_goal_pt, double &out_rate)
{
	int ret = -1;

	int goal_index = -1;
	out_rate = 0;

	int i, j, k;
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	int x;
	int y;
	double dist;
	double xx, yy;

	MORPHIN2 morphin = g_morhpin2[0];

	int value[MORPHIN_LINE_NUM];
	int num = 0;
	memset(value, 0, MORPHIN_LINE_NUM * sizeof(int));
	int l_dist = 0;
	int r_dist = 0;

	double dist_8m[MORPHIN_LINE_NUM];
	double dist_12m[MORPHIN_LINE_NUM];
	double dist_15m[MORPHIN_LINE_NUM];
	double dist_18m[MORPHIN_LINE_NUM];

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		dist_8m[i] = 800;
		dist_12m[i] = 1200;
		dist_15m[i] = 1500;
		dist_18m[i] = 1800;
	}

	//[�ϰ���ɨ��]
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		memset(temp_pts, 0, 200 * sizeof(COOR2));
		temp_pts_num = 0;

		memcpy(temp_pts, morphin.g_morphin_lines[i], morphin.g_morphin_lines_num[i] * sizeof(COOR2));
		temp_pts_num = morphin.g_morphin_lines_num[i];

		num = 0;
		for (j=0; j<temp_pts_num; j++)//[ɨ����������]
		{
			num++;
			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x - 1);;
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;

			l_dist = 0;
			for (k=x; k>=x-5; k--)
			{
				l_dist = x - k;

				if (g_closed_grid_map[y][k] == 6)
					break;
			}

			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x + 1);;
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
			r_dist = 0;
			for (k=x; k<=x+5; k++)
			{
				r_dist = k - x;

				if (g_closed_grid_map[y][k] == 6)
					break;
			}

			xx = (double)temp_pts[j].x;
			yy = (double)temp_pts[j].y - 400;
			dist = sqrt(xx * xx + yy * yy);

			if (dist < 800 && (l_dist < 5 || r_dist < 5))
			{
				dist_8m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;

				value[i] = 0;
				break;
			}
			else if (l_dist < 5 || r_dist < 5)
			{
				if (dist >=1800)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = dist;
				}
				else if (dist >= 1500)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = 0;
				}
				else if (dist >= 1200)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else if (dist >= 800)
				{
					dist_8m[i] = dist;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else
				{
					dist_8m[i] = 0;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}

				break;
			}
			else
			{
				value[i] += l_dist;
				value[i] += r_dist;
			}
		}

		if (num == temp_pts_num)
		{
			if (i==MORPHIN_MID_INDEX)
			{
				dist_8m[i] = 2000;
				dist_12m[i] = 2000;
				dist_15m[i] = 2000;
				dist_18m[i] = 2000;
			}
			else if (dist >=1800)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = dist;
			}
			else if (dist >= 1500)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = 0;
			}
			else if (dist >= 1200)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else if (dist >= 800)
			{
				dist_8m[i] = dist;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else
			{
				dist_8m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
		}

		if (num != 0)
			value[i] = value[i] / num;
		else
			value[i] = 0;
	}


	double radian;
	double radius;
	double min_radius = 100000000;
	double D = 0;
	double car_len = 400;
	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;
	//[ȷ��Ŀ������ڹ켣���±�]
	if (fabs(xx) <= 10)//|| xx < 10)//fabs(yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
	{
		//[Ŀ�����ǰ����ֱ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = PI / 2;

		radian = radian / PI * 180;
	}
	else
	{
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx < 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radius = -sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radius = sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 - (radian / PI * 180);
		}

		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (i==MORPHIN_MID_INDEX)
				continue;

			if (fabs(radius - g_morphin_radius2[i]) < min_radius)
			{
				min_radius = fabs(radius - g_morphin_radius2[i]);
				goal_index = i;
			}
		}
	}

	int max_index = -1;
	int min_delta = MAX_VALUE;

	//[̽������]
	//[9m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_8m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[12m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_12m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[15m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_15m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[18m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_18m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	else
	{
		g_cross_last_index = -1;
		g_morhpin2[0].g_best_index = -1;
		out_rate = 0;
		ret = -1;
		return ret;
	}

	return ret;
}


//[�������ȵڶ����汾]
int get_best_morphin_line55(COOR2 sub_goal_pt, double &out_rate)
{
	int ret = -1;

	int goal_index = -1;
	out_rate = 0;

	int i, j, k;
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	int x;
	int y;
	double dist;
	double xx, yy;

	MORPHIN2 morphin = g_morhpin2[0];

	int value[MORPHIN_LINE_NUM];
	int num = 0;
	memset(value, 0, MORPHIN_LINE_NUM * sizeof(int));
	int l_dist = 0;
	int r_dist = 0;

	double dist_8m[MORPHIN_LINE_NUM];
	double dist_12m[MORPHIN_LINE_NUM];
	double dist_15m[MORPHIN_LINE_NUM];
	double dist_18m[MORPHIN_LINE_NUM];

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		dist_8m[i] = 600;
		dist_12m[i] = 1200;
		dist_15m[i] = 1500;
		dist_18m[i] = 1800;
	}

	//[�ϰ���ɨ��]
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		memset(temp_pts, 0, 200 * sizeof(COOR2));
		temp_pts_num = 0;

		memcpy(temp_pts, morphin.g_morphin_lines[i], morphin.g_morphin_lines_num[i] * sizeof(COOR2));
		temp_pts_num = morphin.g_morphin_lines_num[i];

		num = 0;
		for (j=0; j<temp_pts_num; j++)//[ɨ����������]
		{
			num++;

			if (temp_pts[j].y <= 400)
				continue;

			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x - 1);;
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;

			l_dist = 0;
			for (k=x; k>=x-5; k--)
			{
				l_dist = x - k;

				if (g_closed_grid_map[y][k] == 6)
					break;
			}

			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x + 1);;
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
			r_dist = 0;
			for (k=x; k<=x+5; k++)
			{
				r_dist = k - x;

				if (g_closed_grid_map[y][k] == 6)
					break;
			}

			xx = (double)temp_pts[j].x;
			yy = (double)temp_pts[j].y - 400;
			dist = sqrt(xx * xx + yy * yy);

			if (dist < 600 && (l_dist < 5 || r_dist < 5))
			{
				dist_8m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;

				value[i] = 0;
				break;
			}
			else if (l_dist < 5 || r_dist < 5)
			{
				if (dist >=1800)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = dist;
				}
				else if (dist >= 1500)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = 0;
				}
				else if (dist >= 1200)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else if (dist >= 600)
				{
					dist_8m[i] = dist;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else
				{
					dist_8m[i] = 0;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}

				break;
			}
			else
			{
				value[i] += l_dist;
				value[i] += r_dist;
			}
		}

		if (num == temp_pts_num)
		{
			if (i==MORPHIN_MID_INDEX)
			{
				dist_8m[i] = 2000;
				dist_12m[i] = 2000;
				dist_15m[i] = 2000;
				dist_18m[i] = 2000;
			}
			else if (dist >=1800)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = dist;
			}
			else if (dist >= 1500)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = 0;
			}
			else if (dist >= 1200)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else if (dist >= 600)
			{
				dist_8m[i] = dist;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else
			{
				dist_8m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
		}

		if (num != 0)
			value[i] = value[i] / num;
		else
			value[i] = 0;
	}


	double radian;
	double radius;
	double min_radius = 100000000;
	double D = 0;
	double car_len = 400;
	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;
	//[ȷ��Ŀ������ڹ켣���±�]
	if (fabs(xx) <= 10)//|| xx < 10)//fabs(yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
	{
		//[Ŀ�����ǰ����ֱ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = PI / 2;

		radian = radian / PI * 180;
	}
	else
	{
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx < 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radius = -sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radius = sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 - (radian / PI * 180);
		}

		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (i==MORPHIN_MID_INDEX)
				continue;

			if (fabs(radius - g_morphin_radius2[i]) < min_radius)
			{
				min_radius = fabs(radius - g_morphin_radius2[i]);
				goal_index = i;
			}
		}
	}

	int max_index = -1;
	int min_delta = MAX_VALUE;

	//[̽������]
	//[9m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_8m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)((int)(fabs((double)i - goal_index)));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[12m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_12m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[15m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_15m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[18m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_18m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	else
	{
		g_cross_last_index = -1;
		g_morhpin2[0].g_best_index = -1;
		out_rate = 0;
		ret = -1;
		return ret;
	}

	return ret;
}



//[������Ŀ���֮����ϰ������ϵ��ѡ�񿼲����]
int get_best_morphin_line6(COOR2 sub_goal_pt, double &out_rate)
{
	int ret = -1;

	int goal_index = -1;
	out_rate = 0;

	int i, j, k;
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	int x;
	int y;
	double dist;
	double xx, yy;

	MORPHIN2 morphin = g_morhpin2[0];

	int value[MORPHIN_LINE_NUM];
	int num = 0;
	memset(value, 0, MORPHIN_LINE_NUM * sizeof(int));
	int l_dist = 0;
	int r_dist = 0;

	double dist_8m[MORPHIN_LINE_NUM];
	double dist_12m[MORPHIN_LINE_NUM];
	double dist_15m[MORPHIN_LINE_NUM];
	double dist_18m[MORPHIN_LINE_NUM];

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		dist_8m[i] = 800;
		dist_12m[i] = 1200;
		dist_15m[i] = 1500;
		dist_18m[i] = 1800;
	}

	//[�ϰ���ɨ��]
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		memset(temp_pts, 0, 200 * sizeof(COOR2));
		temp_pts_num = 0;

		memcpy(temp_pts, morphin.g_morphin_lines[i], morphin.g_morphin_lines_num[i] * sizeof(COOR2));
		temp_pts_num = morphin.g_morphin_lines_num[i];

		num = 0;
		for (j=0; j<temp_pts_num; j++)//[ɨ����������]
		{
			num++;
			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x - 1);;
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;

			l_dist = 0;
			for (k=x; k>=x-5; k--)
			{
				l_dist = x - k;

				if (g_closed_grid_map[y][k] == 6)
					break;
			}

			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x + 1);;
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
			r_dist = 0;
			for (k=x; k<=x+5; k++)
			{
				r_dist = k - x;

				if (g_closed_grid_map[y][k] == 6)
					break;
			}

			xx = (double)temp_pts[j].x;
			yy = (double)temp_pts[j].y - 400;
			dist = sqrt(xx * xx + yy * yy);

			if (dist < 800 && (l_dist < 5 || r_dist < 5))
			{
				dist_8m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;

				value[i] = 0;
				break;
			}
			else if (l_dist < 5 || r_dist < 5)
			{
				if (dist >=1800)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = dist;
				}
				else if (dist >= 1500)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = 0;
				}
				else if (dist >= 1200)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else if (dist >= 800)
				{
					dist_8m[i] = dist;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else
				{
					dist_8m[i] = 0;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}

				break;
			}
			else
			{
				value[i] += l_dist;
				value[i] += r_dist;
			}
		}

		if (num == temp_pts_num)
		{
			if (i==MORPHIN_MID_INDEX)
			{
				dist_8m[i] = 2000;
				dist_12m[i] = 2000;
				dist_15m[i] = 2000;
				dist_18m[i] = 2000;
			}
			else if (dist >=1800)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = dist;
			}
			else if (dist >= 1500)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = 0;
			}
			else if (dist >= 1200)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else if (dist >= 800)
			{
				dist_8m[i] = dist;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else
			{
				dist_8m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
		}

		if (num != 0)
			value[i] = value[i] / num;
		else
			value[i] = 0;
	}


	double radian;
//	double radius;
//	double min_radius = 100000000;
	double D = 0;
	double car_len = 400;
	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;

	double dist_to_goal_pt_no_obs;
	dist_to_goal_pt_no_obs = sqrt(xx * xx + (yy - 400) * (yy - 400));
	double dist_to_goal_pt = 0;
	dist_to_goal_pt = dist_to_goal_pt_no_obs;
	int count = 0;
	double step_x = 0;
	double step_y = 0;

	int flag = 0;
	//[���´���ע�� �Գ�ͷǰ��Ϊ��y=0���ܶദ����ǽ������ת��]
	if (fabs(xx) >= fabs((yy - 400)))
	{
		count = (int)(fabs(xx) / GRID_LEN_PER_CELL);
#ifdef MBUG_OPEN_
		if (count == 0)
			MBUG("count : %d\n", count);
#endif
		step_y = (yy - 400) / count;
		if (xx < 0)
			step_x = -GRID_LEN_PER_CELL;
		else
			step_x = GRID_LEN_PER_CELL;


		for (i=0; i<count; i++)
		{
			x = (int)((i * step_x) / GRID_LEN_PER_CELL + (g_grid_center.x - 1));
			y = (int)((i * step_y) / GRID_LEN_PER_CELL + (g_grid_center.y + 8));

			for (k=x; k>=x-5; k--)
			{
				if (g_closed_grid_map[y][k] == 6)
				{
					xx = i * step_x;
					yy = i * step_y;
					dist_to_goal_pt = sqrt(xx * xx + yy * yy);
					flag = 1;
					break;
				}
			}

			x = (int)((i * step_x) / GRID_LEN_PER_CELL + (g_grid_center.x + 1));
			y = (int)((i * step_y) / GRID_LEN_PER_CELL + (g_grid_center.y + 8));
			for (k=x; k<=x+5; k++)
			{
				if (g_closed_grid_map[y][k] == 6)
				{
					xx = i * step_x;
					yy = i * step_y;
					dist_to_goal_pt = sqrt(xx * xx + yy * yy);
					flag = 1;
					break;
				}
			}

			if (flag == 1)
			{
				break;
			}
		}//[for (i=0; i<count; i++)]
	}
	else
	{
		count = (int)(fabs(yy - 400) / GRID_LEN_PER_CELL);
#ifdef MBUG_OPEN_
		if (count == 0)
			MBUG("count :  %d\n", count);
#endif
		step_x = (xx) / count;
		if (yy < 0)
			step_y = -GRID_LEN_PER_CELL;
		else
			step_y = GRID_LEN_PER_CELL;

		for (i=0; i<count; i++)
		{
			x = (int)((i * step_x) / GRID_LEN_PER_CELL + (g_grid_center.x - 1));
			y = (int)((i * step_y) / GRID_LEN_PER_CELL + (g_grid_center.y + 8));

			for (k=x; k>=x-5; k--)
			{
				if (g_closed_grid_map[y][k] == 6)
				{
					xx = i * step_x;
					yy = i * step_y;
					dist_to_goal_pt = sqrt(xx * xx + (yy - 400) * (yy - 400));
					flag = 1;
					break;
				}
			}

			x = (int)((i * step_x) / GRID_LEN_PER_CELL + (g_grid_center.x + 1));
			y = (int)((i * step_y) / GRID_LEN_PER_CELL + (g_grid_center.y + 8));
			for (k=x; k<=x+5; k++)
			{
				if (g_closed_grid_map[y][k] == 6)
				{
					xx = i * step_x;
					yy = i * step_y;
					dist_to_goal_pt = sqrt(xx * xx + (yy - 400) * (yy - 400));
					flag = 1;
					break;
				}
			}

			if (flag == 1)
			{
				break;
			}

		}//[for (i=0; i<count; i++)]
	}

	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;

	double theta;
	double min_angle = 1000000000;
	//[ȷ��Ŀ������ڹ켣���±�]
	if (fabs(xx) <= 10)//|| xx < 10)//fabs(yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
	{
		//[Ŀ�����ǰ����ֱ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = PI / 2;

		radian = radian / PI * 180;
	}
	else
	{
		/*
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx < 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radius = -sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radius = sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 - (radian / PI * 180);
		}

		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (i==MORPHIN_MID_INDEX)
				continue;

			if (fabs(radius - g_morphin_radius2[i]) < min_radius)
			{
				min_radius = fabs(radius - g_morphin_radius2[i]);
				goal_index = i;
			}
		}
		*/


		if (xx < 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radian = atan(car_len / D) * 1.5;
			theta = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radian = atan(car_len / D) * 1.5;
			theta = 90 - (radian / PI * 180);
		}

		//[������ӽ�Ŀ���Ļ��ߣ��൱��ȫ��Ŀ��Ƕ�����]
		for (i = 0; i < MORPHIN_LINE_NUM; i++)
		{
			if (i == MORPHIN_MID_INDEX)
				continue;

			if (fabs(theta - g_morphin_angle[i]) < min_angle)
			{
				min_angle = fabs(theta - g_morphin_angle[i]);
				goal_index = i;
			}
		}
	}

	int max_index = -1;
	int min_delta = MAX_VALUE;

	//[̽������]
	if ((dist_to_goal_pt_no_obs - dist_to_goal_pt) < 50)//[50cm���������]
	{//[֮�����ϰ�]
		//[8m]
		max_index = -1;
		min_delta = MAX_VALUE;
		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (dist_8m[i] == 0)
				continue;

			if ((int)(fabs((double)i - goal_index)) < min_delta)
			{
				min_delta = (int)(fabs((double)i - goal_index));
				max_index = i;
			}
		}

		if (max_index != -1)
		{
			goal_index = max_index;
			g_cross_last_index = goal_index;
			g_morhpin2[0].g_best_index = goal_index;
			ret = goal_index;
			return ret;
		}

		//[12m]
		max_index = -1;
		min_delta = MAX_VALUE;
		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (dist_12m[i] == 0)
				continue;

			if ((int)(fabs((double)i - goal_index)) < min_delta)
			{
				min_delta = (int)(fabs((double)i - goal_index));
				max_index = i;
			}
		}

		if (max_index != -1)
		{
			goal_index = max_index;
			g_cross_last_index = goal_index;
			g_morhpin2[0].g_best_index = goal_index;
			ret = goal_index;
			return ret;
		}

		//[15m]
		max_index = -1;
		min_delta = MAX_VALUE;
		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (dist_15m[i] == 0)
				continue;

			if ((int)(fabs((double)i - goal_index)) < min_delta)
			{
				min_delta = (int)(fabs((double)i - goal_index));
				max_index = i;
			}
		}

		if (max_index != -1)
		{
			goal_index = max_index;
			g_cross_last_index = goal_index;
			g_morhpin2[0].g_best_index = goal_index;
			ret = goal_index;
			return ret;
		}

		//[18m]
		max_index = -1;
		min_delta = MAX_VALUE;
		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (dist_18m[i] == 0)
				continue;

			if ((int)(fabs((double)i - goal_index)) < min_delta)
			{
				min_delta = (int)(fabs((double)i - goal_index));
				max_index = i;
			}
		}

		if (max_index != -1)
		{
			goal_index = max_index;
			g_cross_last_index = goal_index;
			g_morhpin2[0].g_best_index = goal_index;
			ret = goal_index;
			return ret;
		}
		else
		{
			g_cross_last_index = -1;
			g_morhpin2[0].g_best_index = -1;
			out_rate = 0;
			ret = -1;
			return ret;
		}
	}
	else
	{//[֮�����ϰ������Խ������ԽԶ]
		if (dist_to_goal_pt <= 1200)
		{
			//[18m]
			max_index = -1;
			min_delta = MAX_VALUE;
			for (i=0; i<MORPHIN_LINE_NUM; i++)
			{
				if (dist_18m[i] == 0)
					continue;

				if ((int)(fabs((double)i - goal_index)) < min_delta)
				{
					min_delta = (int)(fabs((double)i - goal_index));
					max_index = i;
				}
			}

			if (max_index != -1)
			{
				goal_index = max_index;
				g_cross_last_index = goal_index;
				g_morhpin2[0].g_best_index = goal_index;
				ret = goal_index;
				return ret;
			}
		}

		if (dist_to_goal_pt <= 1500 || max_index == -1)
		{//[���ϰ������15m���ϻ���18m̽���޽��]
			//[15m]
			max_index = -1;
			min_delta = MAX_VALUE;
			for (i=0; i<MORPHIN_LINE_NUM; i++)
			{
				if (dist_15m[i] == 0)
					continue;

				if ((int)(fabs((double)i - goal_index)) < min_delta)
				{
					min_delta = (int)(fabs((double)i - goal_index));
					max_index = i;
				}
			}

			if (max_index != -1)
			{
				goal_index = max_index;
				g_cross_last_index = goal_index;
				g_morhpin2[0].g_best_index = goal_index;
				ret = goal_index;
				return ret;
			}
		}

		if (dist_to_goal_pt <= 1800 || max_index == -1)
		{//[���ϰ������12m���ϻ���15m̽���޽��]
			//[12m]
			max_index = -1;
			min_delta = MAX_VALUE;
			for (i=0; i<MORPHIN_LINE_NUM; i++)
			{
				if (dist_12m[i] == 0)
					continue;

				if ((int)(fabs((double)i - goal_index)) < min_delta)
				{
					min_delta = (int)(fabs((double)i - goal_index));
					max_index = i;
				}
			}

			if (max_index != -1)
			{
				goal_index = max_index;
				g_cross_last_index = goal_index;
				g_morhpin2[0].g_best_index = goal_index;
				ret = goal_index;
				return ret;
			}
		}

		//[8m]
		max_index = -1;
		min_delta = MAX_VALUE;
		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (dist_8m[i] == 0)
				continue;

			if ((int)(fabs((double)i - goal_index)) < min_delta)
			{
				min_delta = (int)(fabs((double)i - goal_index));
				max_index = i;
			}
		}

		if (max_index != -1)
		{
			goal_index = max_index;
			g_cross_last_index = goal_index;
			g_morhpin2[0].g_best_index = goal_index;
			ret = goal_index;
			return ret;
		}
		else
		{
			g_cross_last_index = -1;
			g_morhpin2[0].g_best_index = -1;
			out_rate = 0;
			ret = -1;
			return ret;
		}

		// 		if (dist_to_goal_pt >= 1800)
		// 		{
		// 			//[18m]
		// 			max_index = -1;
		// 			min_delta = MAX_VALUE;
		// 			for (i=0; i<MORPHIN_LINE_NUM; i++)
		// 			{
		// 				if (dist_18m[i] == 0)
		// 					continue;
		// 
		// 				if ((int)(fabs((double)i - goal_index)) < min_delta)
		// 				{
		// 					min_delta = (int)(fabs((double)i - goal_index));
		// 					max_index = i;
		// 				}
		// 			}
		// 
		// 			if (max_index != -1)
		// 			{
		// 				goal_index = max_index;
		// 				g_cross_last_index = goal_index;
		// 				g_morhpin2[0].g_best_index = goal_index;
		// 				ret = goal_index;
		// 				return ret;
		// 			}
		// 		}
		// 
		// 		if (dist_to_goal_pt >= 1500 || max_index == -1)
		// 		{//[���ϰ������15m���ϻ���18m̽���޽��]
		// 			//[15m]
		// 			max_index = -1;
		// 			min_delta = MAX_VALUE;
		// 			for (i=0; i<MORPHIN_LINE_NUM; i++)
		// 			{
		// 				if (dist_15m[i] == 0)
		// 					continue;
		// 
		// 				if ((int)(fabs((double)i - goal_index)) < min_delta)
		// 				{
		// 					min_delta = (int)(fabs((double)i - goal_index));
		// 					max_index = i;
		// 				}
		// 			}
		// 
		// 			if (max_index != -1)
		// 			{
		// 				goal_index = max_index;
		// 				g_cross_last_index = goal_index;
		// 				g_morhpin2[0].g_best_index = goal_index;
		// 				ret = goal_index;
		// 				return ret;
		// 			}
		// 		}
		// 
		// 		if (dist_to_goal_pt >= 1200 || max_index == -1)
		// 		{//[���ϰ������12m���ϻ���15m̽���޽��]
		// 			//[12m]
		// 			max_index = -1;
		// 			min_delta = MAX_VALUE;
		// 			for (i=0; i<MORPHIN_LINE_NUM; i++)
		// 			{
		// 				if (dist_12m[i] == 0)
		// 					continue;
		// 
		// 				if ((int)(fabs((double)i - goal_index)) < min_delta)
		// 				{
		// 					min_delta = (int)(fabs((double)i - goal_index));
		// 					max_index = i;
		// 				}
		// 			}
		// 
		// 			if (max_index != -1)
		// 			{
		// 				goal_index = max_index;
		// 				g_cross_last_index = goal_index;
		// 				g_morhpin2[0].g_best_index = goal_index;
		// 				ret = goal_index;
		// 				return ret;
		// 			}
		// 		}
		// 
		// 		//[8m]
		// 		max_index = -1;
		// 		min_delta = MAX_VALUE;
		// 		for (i=0; i<MORPHIN_LINE_NUM; i++)
		// 		{
		// 			if (dist_8m[i] == 0)
		// 				continue;
		// 
		// 			if ((int)(fabs((double)i - goal_index)) < min_delta)
		// 			{
		// 				min_delta = (int)(fabs((double)i - goal_index));
		// 				max_index = i;
		// 			}
		// 		}
		// 
		// 		if (max_index != -1)
		// 		{
		// 			goal_index = max_index;
		// 			g_cross_last_index = goal_index;
		// 			g_morhpin2[0].g_best_index = goal_index;
		// 			ret = goal_index;
		// 			return ret;
		// 		}
		// 		else
		// 		{
		// 			g_cross_last_index = -1;
		// 			g_morhpin2[0].g_best_index = -1;
		// 			out_rate = 0;
		// 			ret = -1;
		// 			return ret;
		// 		}
	}

	return ret;
}



//[S��]
int get_best_morphin_line7(COOR2 sub_goal_pt, double &out_rate)
{
	int ret = -1;

	int goal_index = -1;
	out_rate = 0;

	int i, j, k;
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	int x;
	int y;
	double dist;
	double xx, yy;

	MORPHIN2 morphin = g_morhpin2[0];

	int value[MORPHIN_LINE_NUM];
	int num = 0;
	memset(value, 0, MORPHIN_LINE_NUM * sizeof(int));
	int l_dist = 0;
	int r_dist = 0;

	double dist_8m[MORPHIN_LINE_NUM];
	double dist_12m[MORPHIN_LINE_NUM];
	double dist_15m[MORPHIN_LINE_NUM];
	double dist_18m[MORPHIN_LINE_NUM];

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		dist_8m[i] = 150;
		dist_12m[i] = 350;
		dist_15m[i] = 550;
		dist_18m[i] = 750;
	}

	//[�ϰ���ɨ��]
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		memset(temp_pts, 0, 200 * sizeof(COOR2));
		temp_pts_num = 0;

		memcpy(temp_pts, morphin.g_morphin_lines[i], morphin.g_morphin_lines_num[i] * sizeof(COOR2));
		temp_pts_num = morphin.g_morphin_lines_num[i];

		if (i == 32)
		{
			i = i;
		}
		num = 0;
		for (j=0; j<temp_pts_num; j++)//[ɨ����������]
		{
			num++;
			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x - 1);;
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;

			l_dist = 0;
			for (k=x; k>=x-3; k--)
			{
				l_dist = x - k;

				if (g_closed_grid_map[y][k] == 8)
					break;
			}

			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x + 1);;
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
			r_dist = 0;
			for (k=x; k<=x+3; k++)
			{
				r_dist = k - x;

				if (g_closed_grid_map[y][k] == 8)
					break;
			}

			xx = (double)temp_pts[j].x;
			yy = (double)temp_pts[j].y - 400;
			dist = sqrt(xx * xx + yy * yy);

			if (dist < 130 && (l_dist < 3 || r_dist < 3))
			{
				dist_8m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;

				value[i] = 0;
				break;
			}
			/*
			else if (yy <= 0 && (l_dist < 3 || r_dist < 3))
			{
				dist_8m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;

				value[i] = 0;
				break;
			}
			*/
			else if (l_dist < 3 || r_dist < 3)
			{
				if (dist >= 750)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = dist;
				}
				else if (dist >= 550)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = 0;
				}
				else if (dist >= 350)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else if (dist >= 150)
				{
					dist_8m[i] = dist;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else
				{
					dist_8m[i] = 0;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}

				break;
			}
			else
			{
				value[i] += l_dist;
				value[i] += r_dist;
			}
		}

		if (num == temp_pts_num)
		{
			if (i==MORPHIN_MID_INDEX)
			{
				dist_8m[i] = 750;
				dist_12m[i] = 750;
				dist_15m[i] = 750;
				dist_18m[i] = 750;
			}
			else if (dist >= 750)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = dist;
			}
			else if (dist >= 550)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = 0;
			}
			else if (dist >= 350)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else if (dist >= 150)
			{
				dist_8m[i] = dist;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else
			{
				dist_8m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
		}

		if (num != 0)
			value[i] = value[i] / num;
		else
			value[i] = 0;
	}


	double radian;
	double radius;
	double min_radius = 100000000;
	double D = 0;
	double car_len = 400;
	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;
	//[ȷ��Ŀ������ڹ켣���±�]
	if (fabs(xx) <= 10)//|| xx < 10)//fabs(yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
	{
		//[Ŀ�����ǰ����ֱ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = PI / 2;

		radian = radian / PI * 180;
	}
	else
	{
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx < 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radius = -sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radius = sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 - (radian / PI * 180);
		}

		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (i==MORPHIN_MID_INDEX)
				continue;

			if (fabs(radius - g_morphin_radius2[i]) < min_radius)
			{
				min_radius = fabs(radius - g_morphin_radius2[i]);
				goal_index = i;
			}
		}
	}

	int max_index = -1;
	int min_delta = MAX_VALUE;

	//[̽������]
	//[18m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_18m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[15m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_15m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[12m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_12m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[9m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_8m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	else
	{
		g_cross_last_index = -1;
		g_morhpin2[0].g_best_index = -1;
		out_rate = 0;
		ret = -1;
		return ret;
	}

	return ret;
}

//[S��������ײ���]
int get_best_morphin_line8(COOR2 sub_goal_pt, double &out_rate)
{
	int ret = -1;

	int goal_index = -1;
	out_rate = 0;

	int i, j;
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	int x;
	int y;
	double dist;
	double xx, yy;

	MORPHIN2 morphin = g_morhpin2[0];

	int value[MORPHIN_LINE_NUM];
	int num = 0;
	memset(value, 0, MORPHIN_LINE_NUM * sizeof(int));
	int l_dist = 0;
	int r_dist = 0;

	double dist_8m[MORPHIN_LINE_NUM];
	double dist_12m[MORPHIN_LINE_NUM];
	double dist_15m[MORPHIN_LINE_NUM];
	double dist_18m[MORPHIN_LINE_NUM];

	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		dist_8m[i] = 150;
		dist_12m[i] = 350;
		dist_15m[i] = 550;
		dist_18m[i] = 750;
	}

	//[�ϰ���ɨ��]
	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		memset(temp_pts, 0, 200 * sizeof(COOR2));
		temp_pts_num = 0;

		memcpy(temp_pts, morphin.g_morphin_lines[i], morphin.g_morphin_lines_num[i] * sizeof(COOR2));
		temp_pts_num = morphin.g_morphin_lines_num[i];

		if (i == 32)
		{
			i = i;
		}
		num = 0;
		for (j = 0; j < temp_pts_num; j++)//[ɨ����������]
		{
			num++;

			if (temp_pts[j].y <= 400)
				continue;

			x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x - 1);;
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;

			xx = (double)temp_pts[j].x;
			yy = (double)temp_pts[j].y - 400;
			dist = sqrt(xx * xx + yy * yy);

			if (g_closed_grid_map[y][x] == 8)
			{
				if (dist >= 750)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = dist;
				}
				else if (dist >= 550)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = dist;
					dist_18m[i] = 0;
				}
				else if (dist >= 350)
				{
					dist_8m[i] = dist;
					dist_12m[i] = dist;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else if (dist >= 150)
				{
					dist_8m[i] = dist;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}
				else
				{
					dist_8m[i] = 0;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;
				}

				break;
			}
		}

		if (num == temp_pts_num)
		{
			if (i == MORPHIN_MID_INDEX)
			{
				dist_8m[i] = 750;
				dist_12m[i] = 750;
				dist_15m[i] = 750;
				dist_18m[i] = 750;
			}
			else if (dist >= 750)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = dist;
			}
			else if (dist >= 550)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = dist;
				dist_18m[i] = 0;
			}
			else if (dist >= 350)
			{
				dist_8m[i] = dist;
				dist_12m[i] = dist;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else if (dist >= 150)
			{
				dist_8m[i] = dist;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
			else
			{
				dist_8m[i] = 0;
				dist_12m[i] = 0;
				dist_15m[i] = 0;
				dist_18m[i] = 0;
			}
		}

		if (num != 0)
			value[i] = value[i] / num;
		else
			value[i] = 0;
	}


	double radian;
	double radius;
	double min_radius = 100000000;
	double D = 0;
	double car_len = 400;
	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;
	//[ȷ��Ŀ������ڹ켣���±�]
	if (fabs(xx) <= 10)//|| xx < 10)//fabs (yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
	{
		//[Ŀ�����ǰ����ֱ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = PI / 2;

		radian = radian / PI * 180;
	}
	else
	{
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx < 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radius = -sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radius = sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 - (radian / PI * 180);
		}

		for (i = 0; i < MORPHIN_LINE_NUM; i++)
		{
			if (i == MORPHIN_MID_INDEX)
				continue;

			if (fabs(radius - g_morphin_radius2[i]) < min_radius)
			{
				min_radius = fabs(radius - g_morphin_radius2[i]);
				goal_index = i;
			}
		}
	}

	int max_index = -1;
	int min_delta = MAX_VALUE;

	//[̽������]
	//[18m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		if (dist_18m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[15m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		if (dist_15m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[12m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		if (dist_12m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[9m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		if (dist_8m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	else
	{
		g_cross_last_index = -1;
		g_morhpin2[0].g_best_index = -1;
		out_rate = 0;
		ret = -1;
		return ret;
	}

	return ret;
}


typedef struct
{
	int is;//[start]
	int ie;//[end]
	int im;//[mid]
	int inum;
}S_OBS_AREA;
static int g_last_s_obs_index = MORPHIN_MID_INDEX;
int get_area_mid(S_OBS_AREA s_obs_area)
{
	int ret = -1;

	int i;
	int il, ir;
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	int x;
	int y;
	double dist;
	double xx, yy;
	COOR2 lpt, rpt;
	int num = 0;
	MORPHIN2 morphin = g_morhpin2[0];

	il = s_obs_area.is - 1;
	if (il < 0)
		il++;
	ir = s_obs_area.ie + 1;
	if (ir >= MORPHIN_LINE_NUM)
		ir--;

	//[��ɨ�������]
	memset(temp_pts, 0, 200 * sizeof(COOR2));
	temp_pts_num = 0;
	memcpy(temp_pts, morphin.g_morphin_lines[il], morphin.g_morphin_lines_num[il] * sizeof(COOR2));
	temp_pts_num = morphin.g_morphin_lines_num[il];

	num = 0;
	for (i = 0; i < temp_pts_num; i++)//[ɨ����������]
	{
		num++;

		if (temp_pts[i].y <= 400)
			continue;

		x = (int)(temp_pts[i].x * 1.0 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.x);
		y = (int)(temp_pts[i].y * 1.0 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.y);

		xx = (double)temp_pts[i].x;
		yy = (double)temp_pts[i].y - 400;
		dist = sqrt(xx * xx + yy * yy);

		//[��ײ]
		if (g_hd_closed_grid_map[y][x] >= 4)
		{
			lpt.x = temp_pts[i].x;
			lpt.y = temp_pts[i].y;
			break;
		}

		//[Morphinʹ�õ��ǵͷֱ��ʵģ����������ڲ�]
		x = (int)(temp_pts[i].x * 1.0 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.x);
		y = (int)(temp_pts[i].y * 1.0 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.y);

		xx = (double)temp_pts[i].x;
		yy = (double)temp_pts[i].y - 400;
		dist = sqrt(xx * xx + yy * yy);

		//[��ײ]
		if (g_hd_closed_grid_map[y][x] >= 4)
		{
			lpt.x = temp_pts[i].x;
			lpt.y = temp_pts[i].y;
			break;
		}
	}

	if (num == temp_pts_num)
	{
		lpt.x = temp_pts[num - 1].x;
		lpt.y = temp_pts[num - 1].y;
	}

	//[ɨ���Ҳ���]
	memset(temp_pts, 0, 200 * sizeof(COOR2));
	temp_pts_num = 0;
	memcpy(temp_pts, morphin.g_morphin_lines[ir], morphin.g_morphin_lines_num[ir] * sizeof(COOR2));
	temp_pts_num = morphin.g_morphin_lines_num[ir];

	num = 0;
	for (i = 0; i < temp_pts_num; i++)//[ɨ����������]
	{
		num++;

		if (temp_pts[i].y <= 400)
			continue;

		x = (int)(temp_pts[i].x * 1.0 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.x);
		y = (int)(temp_pts[i].y * 1.0 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.y);

		xx = (double)temp_pts[i].x;
		yy = (double)temp_pts[i].y - 400;
		dist = sqrt(xx * xx + yy * yy);

		//[��ײ]
		if (g_hd_closed_grid_map[y][x] >= 4)
		{
			rpt.x = temp_pts[i].x;
			rpt.y = temp_pts[i].y;
			break;
		}

		//[Morphinʹ�õ��ǵͷֱ��ʵģ����������ڲ�]
		x = (int)(temp_pts[i].x * 1.0 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.x);
		y = (int)(temp_pts[i].y * 1.0 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.y);

		xx = (double)temp_pts[i].x;
		yy = (double)temp_pts[i].y - 400;
		dist = sqrt(xx * xx + yy * yy);

		//[��ײ]
		if (g_hd_closed_grid_map[y][x] >= 4)
		{
			rpt.x = temp_pts[i].x;
			rpt.y = temp_pts[i].y;
			break;
		}
	}

	if (num == temp_pts_num)
	{
		rpt.x = temp_pts[num - 1].x;
		rpt.y = temp_pts[num - 1].y;
	}

	//[�ж�����]
	dist = dist_point(&lpt, &rpt);
	if (dist < CAR_WIDTH + 30)
	{
		return ret;
	}
	else
	{
		COOR2 mid_pt;
		mid_pt.x = (lpt.x + rpt.x) / 2;
		mid_pt.y = (lpt.y + rpt.y) / 2;

		double radian;
		double radius;
		double min_radius = 100000000;
		double D = 0;
		double car_len = 400;
		xx = mid_pt.x;
		yy = mid_pt.y;
		//[ȷ��Ŀ������ڹ켣���±�]
		if (fabs(xx) <= 10)//|| xx < 10)//fabs (yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
		{
			//[Ŀ�����ǰ����ֱ��]
			ret = MORPHIN_MID_INDEX;
		}
		else
		{
			//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
			if (xx < 0 && yy < -xx)
			{
				xx = -720;
				yy = 0;
			}
			else if (xx > 0 && yy < xx)
			{
				xx = 720;
				yy = 0;
			}

			if (xx < 0)
			{
				D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
				radius = -sqrt(D * D + car_len * car_len);
				radian = atan(car_len / D);

				radian = 90 + radian / PI * 180;
			}
			else if (xx > 0)
			{
				D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
				radius = sqrt(D * D + car_len * car_len);
				radian = atan(car_len / D);

				radian = 90 - (radian / PI * 180);
			}

			for (i = 0; i < MORPHIN_LINE_NUM; i++)
			{
				if (i == MORPHIN_MID_INDEX)
					continue;

				if (fabs(radius - g_morphin_radius2[i]) < min_radius)
				{
					min_radius = fabs(radius - g_morphin_radius2[i]);
					ret = i;
				}
			}
		}

		return ret;
	}

	return ret;
}

int get_best_morphin_line_hd(COOR2 sub_goal_pt, double &out_rate)
{
	int ret = -1;

	int goal_index = -1;
	out_rate = 0;

	int i, j;
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	int x;
	int y;
	double dist;
	double xx, yy;

	MORPHIN2 morphin = g_morhpin2[0];

	int value[MORPHIN_LINE_NUM];
	int num = 0;
	memset(value, 0, MORPHIN_LINE_NUM * sizeof(int));
	int l_dist = 0;
	int r_dist = 0;

	double dist_1[MORPHIN_LINE_NUM];
	double dist_2[MORPHIN_LINE_NUM];
	double dist_3[MORPHIN_LINE_NUM];
	double dist_4[MORPHIN_LINE_NUM];
	double dist_5[MORPHIN_LINE_NUM];
	double dist_6[MORPHIN_LINE_NUM];

	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		dist_1[i] = 150;
		dist_2[i] = 350;
		dist_3[i] = 550;
		dist_4[i] = 750;
		dist_5[i] = 950;
		dist_6[i] = 1150;
	}

	//[�ϰ���ɨ��]
	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		memset(temp_pts, 0, 200 * sizeof(COOR2));
		temp_pts_num = 0;

		memcpy(temp_pts, morphin.g_morphin_lines[i], morphin.g_morphin_lines_num[i] * sizeof(COOR2));
		temp_pts_num = morphin.g_morphin_lines_num[i];

		if (i == 32)
		{
			i = i;
		}
		num = 0;
		for (j = 0; j < temp_pts_num; j++)//[ɨ����������]
		{
			num++;

			if (temp_pts[j].y <= 400)
				continue;

			x = (int)(temp_pts[j].x * 1.0 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.x);
			y = (int)(temp_pts[j].y * 1.0 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.y);

			xx = (double)temp_pts[j].x;
			yy = (double)temp_pts[j].y - 400;
			dist = sqrt(xx * xx + yy * yy);

			//[��ײ]
			if (g_hd_closed_grid_map[y][x] >= 4)
			{
				if (dist >= 1150)
				{
					dist_1[i] = dist;
					dist_2[i] = dist;
					dist_3[i] = dist;
					dist_4[i] = dist;
					dist_5[i] = dist;
					dist_6[i] = dist;
				}
				else if (dist >= 950)
				{
					dist_1[i] = dist;
					dist_2[i] = dist;
					dist_3[i] = dist;
					dist_4[i] = dist;
					dist_5[i] = dist;
					dist_6[i] = 0;
				}
				else if (dist >= 750)
				{
					dist_1[i] = dist;
					dist_2[i] = dist;
					dist_3[i] = dist;
					dist_4[i] = dist;
					dist_5[i] = 0;
					dist_6[i] = 0;
				}
				else if (dist >= 550)
				{
					dist_1[i] = dist;
					dist_2[i] = dist;
					dist_3[i] = dist;
					dist_4[i] = 0;
					dist_5[i] = 0;
					dist_6[i] = 0;
				}
				else if (dist >= 350)
				{
					dist_1[i] = dist;
					dist_2[i] = dist;
					dist_3[i] = 0;
					dist_4[i] = 0;
					dist_5[i] = 0;
					dist_6[i] = 0;
				}
				else if (dist >= 150)
				{
					dist_1[i] = dist;
					dist_2[i] = 0;
					dist_3[i] = 0;
					dist_4[i] = 0;
					dist_5[i] = 0;
					dist_6[i] = 0;
				}
				else
				{
					dist_1[i] = 0;
					dist_2[i] = 0;
					dist_3[i] = 0;
					dist_4[i] = 0;
					dist_5[i] = 0;
					dist_6[i] = 0;
				}

				break;
			}

			//[Morphinʹ�õ��ǵͷֱ��ʵģ����������ڲ�]
			x = (int)(temp_pts[j].x * 1.0 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.x);
			y = (int)(temp_pts[j].y * 1.0 / GRID_LEN_PER_CELL_HD + g_hd_grid_center.y);

			xx = (double)temp_pts[j].x;
			yy = (double)temp_pts[j].y - 400;
			dist = sqrt(xx * xx + yy * yy);

			//[��ײ]
			if (g_hd_closed_grid_map[y][x] >= 4)
			{
				if (dist >= 1150)
				{
					dist_1[i] = dist;
					dist_2[i] = dist;
					dist_3[i] = dist;
					dist_4[i] = dist;
					dist_5[i] = dist;
					dist_6[i] = dist;
				}
				else if (dist >= 950)
				{
					dist_1[i] = dist;
					dist_2[i] = dist;
					dist_3[i] = dist;
					dist_4[i] = dist;
					dist_5[i] = dist;
					dist_6[i] = 0;
				}
				else if (dist >= 750)
				{
					dist_1[i] = dist;
					dist_2[i] = dist;
					dist_3[i] = dist;
					dist_4[i] = dist;
					dist_5[i] = 0;
					dist_6[i] = 0;
				}
				else if (dist >= 550)
				{
					dist_1[i] = dist;
					dist_2[i] = dist;
					dist_3[i] = dist;
					dist_4[i] = 0;
					dist_5[i] = 0;
					dist_6[i] = 0;
				}
				else if (dist >= 350)
				{
					dist_1[i] = dist;
					dist_2[i] = dist;
					dist_3[i] = 0;
					dist_4[i] = 0;
					dist_5[i] = 0;
					dist_6[i] = 0;
				}
				else if (dist >= 150)
				{
					dist_1[i] = dist;
					dist_2[i] = 0;
					dist_3[i] = 0;
					dist_4[i] = 0;
					dist_5[i] = 0;
					dist_6[i] = 0;
				}
				else
				{
					dist_1[i] = 0;
					dist_2[i] = 0;
					dist_3[i] = 0;
					dist_4[i] = 0;
					dist_5[i] = 0;
					dist_6[i] = 0;
				}

				break;
			}
		}

		if (num == temp_pts_num)
		{
			if (i == MORPHIN_MID_INDEX)
			{
				dist_1[i] = dist;
				dist_2[i] = dist;
				dist_3[i] = dist;
				dist_4[i] = dist;
				dist_5[i] = dist;
				dist_6[i] = dist;
			}
			else if (dist >= 1150)
			{
				dist_1[i] = dist;
				dist_2[i] = dist;
				dist_3[i] = dist;
				dist_4[i] = dist;
				dist_5[i] = dist;
				dist_6[i] = dist;
			}
			else if (dist >= 950)
			{
				dist_1[i] = dist;
				dist_2[i] = dist;
				dist_3[i] = dist;
				dist_4[i] = dist;
				dist_5[i] = dist;
				dist_6[i] = 0;
			}
			else if (dist >= 750)
			{
				dist_1[i] = dist;
				dist_2[i] = dist;
				dist_3[i] = dist;
				dist_4[i] = dist;
				dist_5[i] = 0;
				dist_6[i] = 0;
			}
			else if (dist >= 550)
			{
				dist_1[i] = dist;
				dist_2[i] = dist;
				dist_3[i] = dist;
				dist_4[i] = 0;
				dist_5[i] = 0;
				dist_6[i] = 0;
			}
			else if (dist >= 350)
			{
				dist_1[i] = dist;
				dist_2[i] = dist;
				dist_3[i] = 0;
				dist_4[i] = 0;
				dist_5[i] = 0;
				dist_6[i] = 0;
			}
			else if (dist >= 150)
			{
				dist_1[i] = dist;
				dist_2[i] = 0;
				dist_3[i] = 0;
				dist_4[i] = 0;
				dist_5[i] = 0;
				dist_6[i] = 0;
			}
			else
			{
				dist_1[i] = 0;
				dist_2[i] = 0;
				dist_3[i] = 0;
				dist_4[i] = 0;
				dist_5[i] = 0;
				dist_6[i] = 0;
			}
		}
	}

	S_OBS_AREA s_obs_area[MORPHIN_LINE_NUM];
	int s_obs_area_num = 0;
	memset(s_obs_area, 0, sizeof(S_OBS_AREA) * MORPHIN_LINE_NUM);
	int is;
	int ie;
	int im;
	int flag = 0;
	int max_num = -1;
	int max_num_index = -1;
	int cur_area_index = -1;
	int cur_s_obs_index;
	cur_s_obs_index = g_last_s_obs_index;
	//[��һ��]
	i = 0;
	while (dist_6[i] == 0 && i < MORPHIN_LINE_NUM)
	{
		i++;
	}
	if (i != MORPHIN_LINE_NUM)
	{
		is = ie = i;
		flag = 1;
		for (int ii = is + 1; ii < MORPHIN_LINE_NUM;ii++)
		{
			if (dist_6[ii] > 0)
			{
				if (flag == 0)
				{
					flag = 1;
					is = ie = ii;
				}
				else
				{
					ie = ii;

					if (ii == MORPHIN_LINE_NUM - 1)
					{
						im = (ie - is + 1) / 2 + is;
						s_obs_area[s_obs_area_num].is = is;
						s_obs_area[s_obs_area_num].ie = ie;
						s_obs_area[s_obs_area_num].im = im;
						s_obs_area[s_obs_area_num].inum = ie - is + 1;
						s_obs_area_num++;
					}
				}
			}
			else
			{
				if (flag == 0)
				{
					is = ie = ii;
				}
				else
				{
					im = (ie - is + 1) / 2 + is;
					s_obs_area[s_obs_area_num].is = is;
					s_obs_area[s_obs_area_num].ie = ie;
					s_obs_area[s_obs_area_num].im = im;
					s_obs_area[s_obs_area_num].inum = ie - is + 1;
					s_obs_area_num++;
					is = ie = ii;
					flag = 0;
				}
			}
		}

		//[�����������]
		for (i = 0; i < s_obs_area_num; i++)
		{
			if (s_obs_area[i].inum > max_num)
			{
				max_num = s_obs_area[i].inum;
				max_num_index = i;
			}
		}

		for (i = 0; i < s_obs_area_num; i++)
		{
			if (cur_s_obs_index >= s_obs_area[i].is && \
				cur_s_obs_index <= s_obs_area[i].ie)
			{
				cur_area_index = i;
			}
		}

		if (cur_area_index != -1)
		{
			ret = s_obs_area[cur_area_index].im;
			return ret;
		}
		else if (max_num_index != -1)
		{
			ret = s_obs_area[max_num_index].im;
			return ret;
		}
		/*
		if (cur_area_index != -1)
		{
			ret = get_area_mid(s_obs_area[cur_area_index]);

			if (ret == -1 && max_num_index != cur_area_index)
			{
				ret = get_area_mid(s_obs_area[max_num_index]);

				if (ret != -1)
				{
					g_last_s_obs_index = ret;
					return ret;
				}
			}
		}
		else if (max_num_index != -1)
		{
			ret = get_area_mid(s_obs_area[max_num_index]);

			if (ret != -1)
			{
				g_last_s_obs_index = ret;
				return ret;
			}
		}
		*/
	}
	
	//[�ڶ���]
	i = 0;
	s_obs_area_num = 0;
	memset(s_obs_area, 0, sizeof(S_OBS_AREA) * MORPHIN_LINE_NUM);
	while (dist_5[i] == 0 && i < MORPHIN_LINE_NUM)
	{
		i++;
	}
	if (i != MORPHIN_LINE_NUM)
	{
		is = ie = i;
		flag = 1;
		for (int ii = is + 1; ii < MORPHIN_LINE_NUM; ii++)
		{
			if (dist_5[ii] > 0)
			{
				if (flag == 0)
				{
					flag = 1;
					is = ie = ii;
				}
				else
				{
					ie = ii;

					if (ii == MORPHIN_LINE_NUM - 1)
					{
						im = (ie - is + 1) / 2 + is;
						s_obs_area[s_obs_area_num].is = is;
						s_obs_area[s_obs_area_num].ie = ie;
						s_obs_area[s_obs_area_num].im = im;
						s_obs_area[s_obs_area_num].inum = ie - is + 1;
						s_obs_area_num++;
					}
				}
			}
			else
			{
				if (flag == 0)
				{
					is = ie = ii;
				}
				else
				{
					im = (ie - is + 1) / 2 + is;
					s_obs_area[s_obs_area_num].is = is;
					s_obs_area[s_obs_area_num].ie = ie;
					s_obs_area[s_obs_area_num].im = im;
					s_obs_area[s_obs_area_num].inum = ie - is + 1;
					s_obs_area_num++;
					is = ie = ii;
					flag = 0;
				}
			}
		}

		max_num = -1;
		max_num_index = -1;
		cur_area_index = -1;
		//[�����������]
		for (i = 0; i < s_obs_area_num; i++)
		{
			if (s_obs_area[i].inum > max_num)
			{
				max_num = s_obs_area[i].inum;
				max_num_index = i;
			}
		}

		for (i = 0; i < s_obs_area_num; i++)
		{
			if (cur_s_obs_index >= s_obs_area[i].is && \
				cur_s_obs_index <= s_obs_area[i].ie)
			{
				cur_area_index = i;
			}
		}

		if (cur_area_index != -1)
		{
			ret = s_obs_area[cur_area_index].im;
			return ret;
		}
		else if (max_num_index != -1)
		{
			ret = s_obs_area[max_num_index].im;
			return ret;
		}
		/*
		if (cur_area_index != -1)
		{
		ret = get_area_mid(s_obs_area[cur_area_index]);

		if (ret == -1 && max_num_index != cur_area_index)
		{
		ret = get_area_mid(s_obs_area[max_num_index]);

		if (ret != -1)
		{
		g_last_s_obs_index = ret;
		return ret;
		}
		}
		}
		else if (max_num_index != -1)
		{
		ret = get_area_mid(s_obs_area[max_num_index]);

		if (ret != -1)
		{
		g_last_s_obs_index = ret;
		return ret;
		}
		}
		*/
	}

	//[������]
	i = 0;
	s_obs_area_num = 0;
	memset(s_obs_area, 0, sizeof(S_OBS_AREA) * MORPHIN_LINE_NUM);
	while (dist_4[i] == 0 && i < MORPHIN_LINE_NUM)
	{
		i++;
	}
	if (i != MORPHIN_LINE_NUM)
	{
		is = ie = i;
		flag = 1;
		for (int ii = is + 1; ii < MORPHIN_LINE_NUM; ii++)
		{
			if (dist_4[ii] > 0)
			{
				if (flag == 0)
				{
					flag = 1;
					is = ie = ii;
				}
				else
				{
					ie = ii;

					if (ii == MORPHIN_LINE_NUM - 1)
					{
						im = (ie - is + 1) / 2 + is;
						s_obs_area[s_obs_area_num].is = is;
						s_obs_area[s_obs_area_num].ie = ie;
						s_obs_area[s_obs_area_num].im = im;
						s_obs_area[s_obs_area_num].inum = ie - is + 1;
						s_obs_area_num++;
					}
				}
			}
			else
			{
				if (flag == 0)
				{
					is = ie = ii;
				}
				else
				{
					im = (ie - is + 1) / 2 + is;
					s_obs_area[s_obs_area_num].is = is;
					s_obs_area[s_obs_area_num].ie = ie;
					s_obs_area[s_obs_area_num].im = im;
					s_obs_area[s_obs_area_num].inum = ie - is + 1;
					s_obs_area_num++;
					is = ie = ii;
					flag = 0;
				}
			}
		}

		max_num = -1;
		max_num_index = -1;
		cur_area_index = -1;
		//[�����������]
		for (i = 0; i < s_obs_area_num; i++)
		{
			if (s_obs_area[i].inum > max_num)
			{
				max_num = s_obs_area[i].inum;
				max_num_index = i;
			}
		}

		for (i = 0; i < s_obs_area_num; i++)
		{
			if (cur_s_obs_index >= s_obs_area[i].is && \
				cur_s_obs_index <= s_obs_area[i].ie)
			{
				cur_area_index = i;
			}
		}

		if (cur_area_index != -1)
		{
			ret = s_obs_area[cur_area_index].im;
			return ret;
		}
		else if (max_num_index != -1)
		{
			ret = s_obs_area[max_num_index].im;
			return ret;
		}
		/*
		if (cur_area_index != -1)
		{
		ret = get_area_mid(s_obs_area[cur_area_index]);

		if (ret == -1 && max_num_index != cur_area_index)
		{
		ret = get_area_mid(s_obs_area[max_num_index]);

		if (ret != -1)
		{
		g_last_s_obs_index = ret;
		return ret;
		}
		}
		}
		else if (max_num_index != -1)
		{
		ret = get_area_mid(s_obs_area[max_num_index]);

		if (ret != -1)
		{
		g_last_s_obs_index = ret;
		return ret;
		}
		}
		*/
	}

	//[���Ĳ�]
	i = 0;
	s_obs_area_num = 0;
	memset(s_obs_area, 0, sizeof(S_OBS_AREA) * MORPHIN_LINE_NUM);
	while (dist_3[i] == 0 && i < MORPHIN_LINE_NUM)
	{
		i++;
	}
	if (i != MORPHIN_LINE_NUM)
	{
		is = ie = i;
		flag = 1;
		for (int ii = is + 1; ii < MORPHIN_LINE_NUM; ii++)
		{
			if (dist_4[ii] > 0)
			{
				if (flag == 0)
				{
					flag = 1;
					is = ie = ii;
				}
				else
				{
					ie = ii;

					if (ii == MORPHIN_LINE_NUM - 1)
					{
						im = (ie - is + 1) / 2 + is;
						s_obs_area[s_obs_area_num].is = is;
						s_obs_area[s_obs_area_num].ie = ie;
						s_obs_area[s_obs_area_num].im = im;
						s_obs_area[s_obs_area_num].inum = ie - is + 1;
						s_obs_area_num++;
					}
				}
			}
			else
			{
				if (flag == 0)
				{
					is = ie = ii;
				}
				else
				{
					im = (ie - is + 1) / 2 + is;
					s_obs_area[s_obs_area_num].is = is;
					s_obs_area[s_obs_area_num].ie = ie;
					s_obs_area[s_obs_area_num].im = im;
					s_obs_area[s_obs_area_num].inum = ie - is + 1;
					s_obs_area_num++;
					is = ie = ii;
					flag = 0;
				}
			}
		}

		max_num = -1;
		max_num_index = -1;
		cur_area_index = -1;
		//[�����������]
		for (i = 0; i < s_obs_area_num; i++)
		{
			if (s_obs_area[i].inum > max_num)
			{
				max_num = s_obs_area[i].inum;
				max_num_index = i;
			}
		}

		for (i = 0; i < s_obs_area_num; i++)
		{
			if (cur_s_obs_index >= s_obs_area[i].is && \
				cur_s_obs_index <= s_obs_area[i].ie)
			{
				cur_area_index = i;
			}
		}

		if (cur_area_index != -1)
		{
			ret = s_obs_area[cur_area_index].im;
			return ret;
		}
		else if (max_num_index != -1)
		{
			ret = s_obs_area[max_num_index].im;
			return ret;
		}
		/*
		if (cur_area_index != -1)
		{
		ret = get_area_mid(s_obs_area[cur_area_index]);

		if (ret == -1 && max_num_index != cur_area_index)
		{
		ret = get_area_mid(s_obs_area[max_num_index]);

		if (ret != -1)
		{
		g_last_s_obs_index = ret;
		return ret;
		}
		}
		}
		else if (max_num_index != -1)
		{
		ret = get_area_mid(s_obs_area[max_num_index]);

		if (ret != -1)
		{
		g_last_s_obs_index = ret;
		return ret;
		}
		}
		*/
	}

	//[�����]
	i = 0;
	s_obs_area_num = 0;
	memset(s_obs_area, 0, sizeof(S_OBS_AREA) * MORPHIN_LINE_NUM);
	while (dist_2[i] == 0 && i < MORPHIN_LINE_NUM)
	{
		i++;
	}
	if (i != MORPHIN_LINE_NUM)
	{
		is = ie = i;
		flag = 1;
		for (int ii = is + 1; ii < MORPHIN_LINE_NUM; ii++)
		{
			if (dist_2[ii] > 0)
			{
				if (flag == 0)
				{
					flag = 1;
					is = ie = ii;
				}
				else
				{
					ie = ii;

					if (ii == MORPHIN_LINE_NUM - 1)
					{
						im = (ie - is + 1) / 2 + is;
						s_obs_area[s_obs_area_num].is = is;
						s_obs_area[s_obs_area_num].ie = ie;
						s_obs_area[s_obs_area_num].im = im;
						s_obs_area[s_obs_area_num].inum = ie - is + 1;
						s_obs_area_num++;
					}
				}
			}
			else
			{
				if (flag == 0)
				{
					is = ie = ii;
				}
				else
				{
					im = (ie - is + 1) / 2 + is;
					s_obs_area[s_obs_area_num].is = is;
					s_obs_area[s_obs_area_num].ie = ie;
					s_obs_area[s_obs_area_num].im = im;
					s_obs_area[s_obs_area_num].inum = ie - is + 1;
					s_obs_area_num++;
					is = ie = ii;
					flag = 0;
				}
			}
		}

		max_num = -1;
		max_num_index = -1;
		cur_area_index = -1;
		//[�����������]
		for (i = 0; i < s_obs_area_num; i++)
		{
			if (s_obs_area[i].inum > max_num)
			{
				max_num = s_obs_area[i].inum;
				max_num_index = i;
			}
		}

		for (i = 0; i < s_obs_area_num; i++)
		{
			if (cur_s_obs_index >= s_obs_area[i].is && \
				cur_s_obs_index <= s_obs_area[i].ie)
			{
				cur_area_index = i;
			}
		}

		if (cur_area_index != -1)
		{
			ret = s_obs_area[cur_area_index].im;
			return ret;
		}
		else if (max_num_index != -1)
		{
			ret = s_obs_area[max_num_index].im;
			return ret;
		}
		/*
		if (cur_area_index != -1)
		{
		ret = get_area_mid(s_obs_area[cur_area_index]);

		if (ret == -1 && max_num_index != cur_area_index)
		{
		ret = get_area_mid(s_obs_area[max_num_index]);

		if (ret != -1)
		{
		g_last_s_obs_index = ret;
		return ret;
		}
		}
		}
		else if (max_num_index != -1)
		{
		ret = get_area_mid(s_obs_area[max_num_index]);

		if (ret != -1)
		{
		g_last_s_obs_index = ret;
		return ret;
		}
		}
		*/
	}

	//[������]
	i = 0;
	s_obs_area_num = 0;
	memset(s_obs_area, 0, sizeof(S_OBS_AREA) * MORPHIN_LINE_NUM);
	while (dist_1[i] == 0 && i < MORPHIN_LINE_NUM)
	{
		i++;
	}
	if (i != MORPHIN_LINE_NUM)
	{
		is = ie = i;
		flag = 1;
		for (int ii = is + 1; ii < MORPHIN_LINE_NUM; ii++)
		{
			if (dist_1[ii] > 0)
			{
				if (flag == 0)
				{
					flag = 1;
					is = ie = ii;
				}
				else
				{
					ie = ii;

					if (ii == MORPHIN_LINE_NUM - 1)
					{
						im = (ie - is + 1) / 2 + is;
						s_obs_area[s_obs_area_num].is = is;
						s_obs_area[s_obs_area_num].ie = ie;
						s_obs_area[s_obs_area_num].im = im;
						s_obs_area[s_obs_area_num].inum = ie - is + 1;
						s_obs_area_num++;
					}
				}
			}
			else
			{
				if (flag == 0)
				{
					is = ie = ii;
				}
				else
				{
					im = (ie - is + 1) / 2 + is;
					s_obs_area[s_obs_area_num].is = is;
					s_obs_area[s_obs_area_num].ie = ie;
					s_obs_area[s_obs_area_num].im = im;
					s_obs_area[s_obs_area_num].inum = ie - is + 1;
					s_obs_area_num++;
					is = ie = ii;
					flag = 0;
				}
			}
		}

		max_num = -1;
		max_num_index = -1;
		cur_area_index = -1;
		//[�����������]
		for (i = 0; i < s_obs_area_num; i++)
		{
			if (s_obs_area[i].inum > max_num)
			{
				max_num = s_obs_area[i].inum;
				max_num_index = i;
			}
		}

		for (i = 0; i < s_obs_area_num; i++)
		{
			if (cur_s_obs_index >= s_obs_area[i].is && \
				cur_s_obs_index <= s_obs_area[i].ie)
			{
				cur_area_index = i;
			}
		}

		if (cur_area_index != -1)
		{
			ret = s_obs_area[cur_area_index].im;
			return ret;
		}
		else if (max_num_index != -1)
		{
			ret = s_obs_area[max_num_index].im;
			return ret;
		}
		/*
		if (cur_area_index != -1)
		{
		ret = get_area_mid(s_obs_area[cur_area_index]);

		if (ret == -1 && max_num_index != cur_area_index)
		{
		ret = get_area_mid(s_obs_area[max_num_index]);

		if (ret != -1)
		{
		g_last_s_obs_index = ret;
		return ret;
		}
		}
		}
		else if (max_num_index != -1)
		{
		ret = get_area_mid(s_obs_area[max_num_index]);

		if (ret != -1)
		{
		g_last_s_obs_index = ret;
		return ret;
		}
		}
		*/
	}

// 	for (int i = 0; i < MORPHIN_LINE_NUM;i++)
// 	{
// 
// 	}

	/*
	double radian;
	double radius;
	double min_radius = 100000000;
	double D = 0;
	double car_len = 400;
	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;
	//[ȷ��Ŀ������ڹ켣���±�]
	if (fabs(xx) <= 10)//|| xx < 10)//fabs (yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
	{
		//[Ŀ�����ǰ����ֱ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = PI / 2;

		radian = radian / PI * 180;
	}
	else
	{
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx < 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radius = -sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radius = sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 - (radian / PI * 180);
		}

		for (i = 0; i < MORPHIN_LINE_NUM; i++)
		{
			if (i == MORPHIN_MID_INDEX)
				continue;

			if (fabs(radius - g_morphin_radius2[i]) < min_radius)
			{
				min_radius = fabs(radius - g_morphin_radius2[i]);
				goal_index = i;
			}
		}
	}

	int max_index = -1;
	int min_delta = MAX_VALUE;

	//[̽������]
	//[18m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		if (dist_18m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[15m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		if (dist_15m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[12m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		if (dist_12m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[9m]
	max_index = -1;
	min_delta = MAX_VALUE;
	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		if (dist_8m[i] == 0)
			continue;

		if ((int)(fabs((double)i - goal_index)) < min_delta)
		{
			min_delta = (int)(fabs((double)i - goal_index));
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	else
	{
		g_cross_last_index = -1;
		g_morhpin2[0].g_best_index = -1;
		out_rate = 0;
		ret = -1;
		return ret;
	}
	*/
	return ret;
}

//[�Ͻ�ɽ����]
//[��ʻ��������]
//[ˮƽɨ���ϰ�]
int get_best_arc_line1(COOR2 sub_goal_pt, double &out_rate)
{
	int ret = -1;

	int goal_index = -1;

	int i, j, k;
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	int x;
	int y;
	double dist;
	double xx, yy;

	COLLISION_ARC collision_arc = g_collision_arc;

	int value[MORPHIN_LINE_NUM];
	int num = 0;
	memset(value, 0, MORPHIN_LINE_NUM * sizeof(int));
	int l_dist = 0;
	int r_dist = 0;

	double dist_9m[MORPHIN_LINE_NUM];
	double dist_12m[MORPHIN_LINE_NUM];
	double dist_15m[MORPHIN_LINE_NUM];
	double dist_18m[MORPHIN_LINE_NUM];

	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		dist_9m[i] = 900;
		dist_12m[i] = 1200;
		dist_15m[i] = 1500;
		dist_18m[i] = 1800;
	}

	//[�ϰ���ɨ��]
	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		memset(temp_pts, 0, 200 * sizeof(COOR2));
		temp_pts_num = 0;

		memcpy(temp_pts, collision_arc.g_collision_lines[i], collision_arc.g_collision_lines_num[i] * sizeof(COOR2));
		temp_pts_num = collision_arc.g_collision_lines_num[i];

		int include_r = (g_morphin_include_radius[i] + GRID_LEN_PER_CELL - 1) / GRID_LEN_PER_CELL;
		num = 0;
		
		if (i<MORPHIN_MID_INDEX)
		{//[��գ������ҷ�������ײ]
			for (j=0; j<temp_pts_num; j++)
			{
				num++;

				x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x - 1);;
				y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
				l_dist = 0;
				for (k=x; k>=x-include_r; k--)
				{
					l_dist = x - k;

					if (g_grid_map[y][k] == 8)
						break;
				}

				x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x + 1);;
				y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
				r_dist = 0;
				if (temp_pts[j].y > 400)
				{
					for (k=x; k<=x+include_r; k++)
					{
						r_dist = k - x;

						if (g_grid_map[y][k] == 8)
							break;
					}
				}
				else
				{
					r_dist = include_r;
				}


				xx = (double)temp_pts[j].x;
				yy = (double)temp_pts[j].y - 400;
				dist = sqrt(xx * xx + yy * yy);

				if (dist < 800 && (l_dist < include_r || r_dist < include_r))
				{
					dist_9m[i] = 0;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;

					value[i] = 0;
					break;
				}
				else if (l_dist < include_r || r_dist < include_r)
				{
					if (dist >=1800)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = dist;
						dist_18m[i] = dist;
					}
					else if (dist >= 1500)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = dist;
						dist_18m[i] = 0;
					}
					else if (dist >= 1200)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
					else if (dist >= 900)
					{
						dist_9m[i] = dist;
						dist_12m[i] = 0;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
					else
					{
						dist_9m[i] = 0;
						dist_12m[i] = 0;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}

					break;
				}
				else
				{
					value[i] += l_dist;
					value[i] += r_dist;
				}

				if (num == temp_pts_num)
				{
					if (i==MORPHIN_MID_INDEX)
					{
						dist_9m[i] = 2400;
						dist_12m[i] = 2400;
						dist_15m[i] = 2400;
						dist_18m[i] = 2400;
					}
					else if (dist >=1800)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = dist;
						dist_18m[i] = dist;
					}
					else if (dist >= 1500)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = dist;
						dist_18m[i] = 0;
					}
					else if (dist >= 1200)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
					else if (dist >= 900)
					{
						dist_9m[i] = dist;
						dist_12m[i] = 0;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
					else
					{
						dist_9m[i] = 0;
						dist_12m[i] = 0;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
				}

				if (num != 0)
					value[i] = value[i] / num;
				else
					value[i] = 0;



			}//[for j]
		}//[if (i<MORPHIN_MID_INDEX)]
		else if (i==MORPHIN_MID_INDEX)
		{
			include_r = 4;
			for (j=0; j<temp_pts_num; j++)
			{
				num++;

				x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x - 1);;
				y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
				l_dist = 0;
				for (k=x; k>=x-4; k--)
				{
					l_dist = x - k;

					if (g_grid_map[y][k] == 8)
						break;
				}

				x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x + 1);;
				y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
				r_dist = 0;
				for (k=x; k<=x+4; k++)
				{
					r_dist = k - x;

					if (g_grid_map[y][k] == 8)
						break;
				}


				xx = (double)temp_pts[j].x;
				yy = (double)temp_pts[j].y - 400;
				dist = sqrt(xx * xx + yy * yy);

				if (dist < 800 && (l_dist < include_r || r_dist < include_r))
				{
					dist_9m[i] = 0;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;

					value[i] = 0;
					break;
				}
				else if (l_dist < include_r || r_dist < include_r)
				{
					if (dist >=1800)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = dist;
						dist_18m[i] = dist;
					}
					else if (dist >= 1500)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = dist;
						dist_18m[i] = 0;
					}
					else if (dist >= 1200)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
					else if (dist >= 900)
					{
						dist_9m[i] = dist;
						dist_12m[i] = 0;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
					else
					{
						dist_9m[i] = 0;
						dist_12m[i] = 0;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}

					break;
				}
				else
				{
					value[i] += l_dist;
					value[i] += r_dist;
				}

				if (num == temp_pts_num)
				{
					if (i==MORPHIN_MID_INDEX)
					{
						dist_9m[i] = 2400;
						dist_12m[i] = 2400;
						dist_15m[i] = 2400;
						dist_18m[i] = 2400;
					}
					else if (dist >=1800)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = dist;
						dist_18m[i] = dist;
					}
					else if (dist >= 1500)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = dist;
						dist_18m[i] = 0;
					}
					else if (dist >= 1200)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
					else if (dist >= 900)
					{
						dist_9m[i] = dist;
						dist_12m[i] = 0;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
					else
					{
						dist_9m[i] = 0;
						dist_12m[i] = 0;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
				}

				if (num != 0)
					value[i] = value[i] / num;
				else
					value[i] = 0;



			}//[for j]
		}//[else if (i==MORPHIN_MID_INDEX)]
		else
		{//[�ҹգ������󷽲�����ײ]
			for (j=0; j<temp_pts_num; j++)
			{
				num++;

				x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x - 1);;
				y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
				l_dist = 0;
				if (temp_pts[j].y >= 400)
				{
					for (k=x; k>=x-include_r; k--)
					{
						l_dist = x - k;

						if (g_grid_map[y][k] == 8)
							break;
					}
				}
				else
				{
					l_dist = include_r;
				}

				x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x + 1);;
				y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;
				r_dist = 0;
				for (k=x; k<=x+include_r; k++)
				{
					r_dist = k - x;

					if (g_grid_map[y][k] == 8)
						break;
				}

				xx = (double)temp_pts[j].x;
				yy = (double)temp_pts[j].y - 400;
				dist = sqrt(xx * xx + yy * yy);

				if (dist < 800 && (l_dist < include_r || r_dist < include_r))
				{
					dist_9m[i] = 0;
					dist_12m[i] = 0;
					dist_15m[i] = 0;
					dist_18m[i] = 0;

					value[i] = 0;
					break;
				}
				else if (l_dist < include_r || r_dist < include_r)
				{
					if (dist >=1800)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = dist;
						dist_18m[i] = dist;
					}
					else if (dist >= 1500)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = dist;
						dist_18m[i] = 0;
					}
					else if (dist >= 1200)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
					else if (dist >= 900)
					{
						dist_9m[i] = dist;
						dist_12m[i] = 0;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
					else
					{
						dist_9m[i] = 0;
						dist_12m[i] = 0;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}

					break;
				}
				else
				{
					value[i] += l_dist;
					value[i] += r_dist;
				}

				if (num == temp_pts_num)
				{
					if (i==MORPHIN_MID_INDEX)
					{
						dist_9m[i] = 2400;
						dist_12m[i] = 2400;
						dist_15m[i] = 2400;
						dist_18m[i] = 2400;
					}
					else if (dist >=1800)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = dist;
						dist_18m[i] = dist;
					}
					else if (dist >= 1500)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = dist;
						dist_18m[i] = 0;
					}
					else if (dist >= 1200)
					{
						dist_9m[i] = dist;
						dist_12m[i] = dist;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
					else if (dist >= 900)
					{
						dist_9m[i] = dist;
						dist_12m[i] = 0;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
					else
					{
						dist_9m[i] = 0;
						dist_12m[i] = 0;
						dist_15m[i] = 0;
						dist_18m[i] = 0;
					}
				}

				if (num != 0)
					value[i] = value[i] / num;
				else
					value[i] = 0;


			}//[for j]
		}
	}

	double radian;
	double radius;
	double min_radius = 100000000;
	double D = 0;
	double car_len = 400;
	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;
	//[ȷ��Ŀ������ڹ켣���±�]
	if (fabs(xx) <= 20)//|| xx < 10)//fabs(yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
	{
		//[Ŀ�����ǰ����ֱ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = PI / 2;

		radian = radian / PI * 180;
	}
	else
	{
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx <= 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		//[Ŀ��������˶�������]
		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radius = -sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radius = sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 - (radian / PI * 180);
		}

		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (i==MORPHIN_MID_INDEX)
				continue;

			if (fabs(radius - g_morphin_radius2[i]) < min_radius)
			{
				min_radius = fabs(radius - g_morphin_radius2[i]);
				goal_index = i;
			}
		}
	}

	int max_index = -1;
	int max_dist = -1;

	//[̽������]
	//[20m]
	max_dist = 0;
	max_index = -1;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_18m[i] > max_dist)
		{
			max_dist = (int)dist_18m[i];
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[15m]
	max_dist = 0;
	max_index = -1;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_15m[i] > max_dist)
		{
			max_dist = (int)dist_15m[i];
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[12m]
	max_dist = 0;
	max_index = -1;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_12m[i] > max_dist)
		{
			max_dist = (int)dist_12m[i];
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}

	//[9m]
	max_dist = 0;
	max_index = -1;
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		if (dist_9m[i] > max_dist)
		{
			max_dist = (int)dist_9m[i];
			max_index = i;
		}
	}

	if (max_index != -1)
	{
		goal_index = max_index;
		g_cross_last_index = goal_index;
		g_morhpin2[0].g_best_index = goal_index;
		ret = goal_index;
		return ret;
	}
	else
	{
		g_cross_last_index = -1;
		g_morhpin2[0].g_best_index = -1;
		out_rate = 0;
		ret = -1;
		return ret;
	}

	return ret;
}

/*==================================================================
 * ������  ��	void get_avg_mid_line(double angle)
 * ����    ��	 ����ƽ����Ĺ滮�Ƕ�����·��
 * ���������	double angle		��ǰ����õ��Ĺ滮�Ƕ�
 * ���������	
 * ����ֵ  ��	
 * ����    ��	
 * ����    ��	
 * �޸ļ�¼��	
 *==================================================================*/
void get_avg_mid_line(double angle)
{
	int i = 0;

	double radian = angle * PI / 180;
	double radius = 0;
	double step_radian = 0;
	double s_radian = 0;
	double e_radian = 0;
	COOR2 tmp_pt;

	double car_len = 400;
	double D = 0;
	double theta = 0;

	int step = g_morphin_search_dist / 20;
	if (fabs(radian - PI / 2) < 0.002)//[0.002���Ƚ���0.1��]
	{
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			g_mid_line[i].x = 0;
			g_mid_line[i].y = i * step;
		}
	}
	else if (radian > PI / 2)
	{//[���Morphin��]
		theta = radian - PI / 2;
		radius = car_len / sin(theta);

		radian = (g_morphin_search_dist - 400) / radius;

		if (radian + theta > PI / 2)
		{
			radian = PI / 2 - theta;
		}

		step_radian = radian / 18;
		D = sqrt(radius * radius - car_len * car_len);

		s_radian = theta;
		e_radian = radian + theta;

		g_mid_line[0].x = 0;
		g_mid_line[0].y = 0;
		i = 1;

		for (radian = s_radian; radian <= e_radian; radian += step_radian)
		{
			tmp_pt.x = (INT32)(cos(radian) * radius - D);
			tmp_pt.y = (INT32)(sin(radian) * radius);

			g_mid_line[i].x = tmp_pt.x;
			g_mid_line[i].y = tmp_pt.y - 200;
			i++;
		}
		g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x = (INT32)(cos(e_radian) * radius - D);
		g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y = (INT32)(sin(e_radian) * radius - 200);
	}
	else
	{//[�Ҳ�Morphin��]
		theta = PI / 2 - radian;
		radius = car_len / sin(theta);

		radian = (g_morphin_search_dist - 400) / radius;

		if (PI - radian - theta < PI / 2)
		{
			radian = PI / 2 - theta;
		}

		step_radian = radian / 18;
		D = sqrt(radius * radius - car_len * car_len);

		s_radian = PI - theta;
		e_radian = PI - radian - theta;

		g_mid_line[0].x = 0;
		g_mid_line[0].y = 0;
		i = 1;

		for (radian = s_radian; radian >= e_radian; radian -= step_radian)
		{
			tmp_pt.x = (INT32)(cos(radian) * radius + D);
			tmp_pt.y = (INT32)(sin(radian) * radius);

			g_mid_line[i].x = tmp_pt.x;
			g_mid_line[i].y = tmp_pt.y - 200;
			i++;
		}
		g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x = (INT32)(cos(e_radian) * radius + D);
		g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y = (INT32)(sin(e_radian) * radius - 200);
	}
}

/*==================================================================
 * ������  ��	int roving(COOR2 goal_pt, int mode)
 * ����    ��	����
 * ���������	COOR2 goal_pt			Ŀ��㣬�Գ�������ϵ��ʽ����
				int mode				0 ʹ��һ����ǰ��������Ŀ���  1 ʹ������Ŀ���  2  ʹ������� �Ҽ�ת��
 * ���������	
 * ����ֵ  ��	int						���ص���ѡ�������ߵ�index��-1��û�п���������
 * ����    ��	
 * ����    ��	
 * �޸ļ�¼��	
 *==================================================================*/
int roving(COOR2 goal_pt, int mode)
{
	int ret = 0;
	int i = 0;

	COOR2 virtual_pt;

	if (mode == 0)
	{
		virtual_pt.x = 11;
		virtual_pt.y = 800;
#ifdef MBUG_OPEN_
		MBUG("virtual_pt (%d, %d)\n", virtual_pt.x, virtual_pt.y);
#endif
	}
	else
	{
		virtual_pt = goal_pt;
	}

	double out_rate = 0;

// 	ret = get_best_arc_line2(virtual_pt, out_rate);//get_best_morphin_line4(virtual_pt, out_rate);
// 	if (g_road_type == 0)
// 	{
	if (g_stat == S_TASK_OVER)
	{
		ret = get_best_morphin_line4(virtual_pt, out_rate);
	}
	else if (g_stat == S_UTURN)
	{
		ret = get_best_morphin_line55(virtual_pt,out_rate);
	}
	else if (g_stat == S_PARKING)
	{
		ret = get_best_morphin_line5(virtual_pt, out_rate);
	}
	else if (g_stat == S_LEFT || \
		g_stat == S_RIGHT)
	{
		ret = get_best_morphin_line55(virtual_pt, out_rate);
	}
	else
		ret = get_best_morphin_line4(virtual_pt, out_rate);
//	}
	
	if (ret >= 0)
		g_cross_travel_rate = out_rate;
	else
		g_cross_travel_rate = 0;

	if (ret == -1)
	{
#ifdef MBUG_OPEN_
		MBUG("morphin no way to go\n");
#endif
		//g_fidelity = 0;
	}
	else
	{
		double avg = 0;
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

			for (i=0; i<g_cross_avg_angle_num; i++)
			{
				avg += g_cross_avg_angle[i];
			}
			avg /= g_cross_avg_angle_num;
			get_avg_mid_line(avg);
		}//[else]

// 		double angle = g_morphin_angle[ret];
// 		get_avg_mid_line(angle);

		ret = (int)avg;
		g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
		//g_fidelity = 1;
	}

	return ret;
}


int check_vehicle_lateral_obs()
{
	int ret = 0;



	return ret;
}

int get_s_obs_morphin_line()
{
	int ret = -1;

	//[1.��⳵�������ϰ���]

	return ret;
}

#define DIST1 600
#define DIST2 1200
#define DIST3 1800
#define DIST4 2400
#define FILTER_WIN 5
static double line_dist[4][MORPHIN_LINE_NUM];
static double line_rate[MORPHIN_LINE_NUM];	//[ͨ����]
static int global_index = -1;		//[ȫ������]
static int local_index = -1;		//[�ֲ�����]
static int last_index = MORPHIN_MID_INDEX;			//[��һ������]
static int steer_step_in_frame = 20;	//[�˲��еĲ���]
static int search_win_s = 5;
static int search_win_m = 8;
static int search_win_l = 30;
static int rate_search_win = 2;		//[��֤]
static double keep_mid_dist[MORPHIN_LINE_NUM];
static int keep_mid_win = 2;

//[ƽ������]
static int filter_num = 0;
static int filter_cur_idx = 0;
static double filter[FILTER_WIN];

//[ԽҰ�ٶȿ���]
static int cross_cur_speed;
static int speed_timer = 0;

static int recover_cross_cur_speed;
static int recover_speed_timer = 0;
static int recover_last_index;
static int recover_filter_num = 0;
static int recover_filter_cur_idx = 0;
static double recover_filter[FILTER_WIN];

/*==================================================================
 * ������  ��	int search_length_first(int goal_index, int mode)
 * ����    ��	����ģʽ����ѡ�߲���ѡ���ڽ������ϵ�����¿��ǿ�ͨ���ʴ��
 * ���������	int goal_index		Ŀ��������
 *				int mode			0  �������� ����С  1  ��������  ������  2  ��������  ���ڴ�
 * ���������	
 * ����ֵ  ��	int					ѡ�������
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.06.15
 * �޸ļ�¼��	
 *==================================================================*/
int search_length_first(int goal_index, int mode)
{
	int ret = -1;

	int i, j;
	int counter = 0;
	int decounter = 0;
	int decounter_safe = 0;
	int lsearch_index = -1;
	int ls_index = -1;
	int le_index = -1;
	int ldelta = -1;

	int rsearch_index = -1;
	int rs_index = -1;
	int re_index = -1;
	int rdelta = -1;

	int win = 0;
	switch (mode)
	{
	case 0:
		win = search_win_s;
		break;
	case 1:
		win = search_win_m;
		break;
	case 2:
	case 3:
	case 4:
		win = search_win_l;
		break;
	}

	if (mode == 0 || \
		mode == 1 || \
		mode == 2)
	{
		i = 3;//[��Զ����]
		if (mode == 2)
			i = 0;
		for (; i >= 0; i--)
		{
			counter = 0;
			lsearch_index = -1;
			ls_index = -1;
			le_index = -1;
			ldelta = -1;
			rsearch_index = -1;
			rs_index = -1;
			re_index = -1;
			rdelta = -1;
			//[��Ŀ����������������]
			for (j = goal_index; j >= 0; j--)
			{
				if (line_dist[i][j] == 0)
				{
					if (counter == 0)
						continue;//[���޿�ͨ������]
					else
						break;//[�Ѿ���������ͨ�����ߣ��˳�����������]
				}
				else if (line_dist[i][j] > 0)
				{
					if (ls_index == -1)
						ls_index = j;

					counter++;

					if (mode == 2)//[��������ģʽ�£�ʹ�ô��ڱ�Ե]
					{
						lsearch_index = ls_index - counter;
						le_index = lsearch_index;
					}
					else
						lsearch_index = ls_index - (counter / 2);

					if (lsearch_index < 0)
					{
						lsearch_index = 0;
						le_index = lsearch_index;
					}

					ldelta = abs(lsearch_index - goal_index);

					if (counter == win)
						break;
				}
			}

			//[��Ŀ����������������]
			counter = 0;
			for (j = goal_index; j < MORPHIN_LINE_NUM; j++)
			{
				if (line_dist[i][j] == 0)
				{
					if (counter == 0)
						continue;//[���޿�ͨ������]
					else
						break;//[�Ѿ���������ͨ�����ߣ��˳�����������]
				}
				else if (line_dist[i][j] > 0)
				{
					if (rs_index == -1)
						rs_index = j;

					counter++;
					if (mode == 2)
					{
						rsearch_index = rs_index + counter;
						re_index = rsearch_index;
					}
					else
						rsearch_index = rs_index + (counter / 2);

					if (rsearch_index >= MORPHIN_LINE_NUM)
					{
						rsearch_index = MORPHIN_LINE_NUM - 1;
						re_index = rsearch_index;
					}

					rdelta = abs(rsearch_index - goal_index);

					if (counter == win)
						break;
				}
			}

			if (line_dist[i][goal_index] == 0)
			{
				if (ldelta != -1 && rdelta != -1)
				{
					if (ldelta == rdelta)
						ret = goal_index;
					else
						rdelta < ldelta ? ret = rsearch_index : ret = lsearch_index;
					break;
				}
				else if (ldelta != -1)
				{
					ret = lsearch_index;
					break;
				}
				else if (rdelta != -1)
				{
					ret = rsearch_index;
					break;
				}
			}
			else
			{
				if (ldelta != -1 && rdelta != -1)
				{
					if (ldelta == rdelta)
						ret = goal_index;
					else
						rdelta < ldelta ? ret = lsearch_index : ret = rsearch_index;
					break;
				}
				else if (ldelta != -1)
				{
					ret = lsearch_index;
					break;
				}
				else if (rdelta != -1)
				{
					ret = rsearch_index;
					break;
				}
			}
		}

		int flag = 0;
		double max_rate = -1;
		int max_idx = -1;
		if (mode == 2 && i == 0)
		{//[�����رܣ�����ͨ����]
			if (ldelta != -1 && rdelta != -1)
			{
				if (rdelta > ldelta)//[�ұ߿տ�]
				{
					for (i = re_index; i >= rs_index; i--)
					{
						if (line_rate[i] == 1)
						{//[�ҵ���һ��ͨ���ʽϸߵ�]
							ret = i;
							flag = 1;
							break;
						}

						if (max_rate < line_rate[i])
						{
							max_rate = line_rate[i];
							max_idx = i;
						}
					}

					if (flag == 0)
						ret = max_idx;
				}
				else if (rdelta < ldelta)
				{
					for (i = le_index; i <= ls_index; i++)
					{
						if (line_rate[i] == 1)
						{//[�ҵ���һ��ͨ���ʽϸߵ�]
							ret = i;
							flag = 1;
							break;
						}

						if (max_rate < line_rate[i])
						{
							max_rate = line_rate[i];
							max_idx = i;
						}
					}

					if (flag == 0)
						ret = max_idx;
				}
				else
				{
					int l_max_rate = -1;
					int l_max_idx = -1;
					int r_max_rate = -1;
					int r_max_idx = -1;
					int l_flag = 0;
					int r_flag = 0;
					int l_ret = -1;
					int r_ret = -1;

					for (i = le_index; i <= ls_index; i++)
					{
						if (line_rate[i] == 1)
						{//[�ҵ���һ��ͨ���ʽϸߵ�]
							l_ret = i;
							l_flag = 1;
							break;
						}

						if (l_max_rate < line_rate[i])
						{
							l_max_rate = (int)line_rate[i];
							l_max_idx = i;
						}
					}

					for (i = re_index; i >= rs_index; i--)
					{
						if (line_rate[i] == 1)
						{//[�ҵ���һ��ͨ���ʽϸߵ�]
							r_ret = i;
							r_flag = 1;
							break;
						}

						if (r_max_rate < line_rate[i])
						{
							r_max_rate = (int)line_rate[i];
							r_max_idx = i;
						}
					}

					if (l_flag == 1 && r_flag == 1)
					{
						if (abs(l_ret - goal_index) <= abs(r_ret - goal_index))
							ret = l_ret;
						else
							ret = r_ret;
					}
					else if (l_flag == 1)
						ret = l_ret;
					else if (r_flag == 1)
						ret = r_ret;
					else
					{
						if (l_max_rate < r_max_idx)
							ret = r_max_idx;
						else if (l_max_rate > r_max_idx)
							ret = l_max_idx;
						else
						{
							if (abs(l_max_idx - goal_index) <= abs(r_max_idx - goal_index))
								ret = l_max_idx;
							else
								ret = r_max_idx;
						}
					}
				}
			}
			else if (ldelta != -1)
			{
				for (i = le_index; i <= ls_index; i++)
				{
					if (line_rate[i] == 1)
					{//[�ҵ���һ��ͨ���ʽϸߵ�]
						ret = i;
						flag = 1;
						break;
					}

					if (max_rate < line_rate[i])
					{
						max_rate = line_rate[i];
						max_idx = i;
					}
				}

				if (flag == 0)
					ret = max_idx;
			}
			else if (rdelta != -1)
			{
				for (i = re_index; i >= rs_index; i--)
				{
					if (line_rate[i] == 1)
					{//[�ҵ���һ��ͨ���ʽϸߵ�]
						ret = i;
						flag = 1;
						break;
					}

					if (max_rate < line_rate[i])
					{
						max_rate = line_rate[i];
						max_idx = i;
					}
				}

				if (flag == 0)
					ret = max_idx;
			}
		}
	}//[mode 1 2 3]
	else if (mode == 3)
	{//[��ת�䣬�ӽ�����]
		counter = 0;
		decounter = 0;
		lsearch_index = -1;
		ls_index = -1;
		le_index = -1;
		ldelta = -1;
		rsearch_index = -1;
		rs_index = -1;
		re_index = -1;
		rdelta = -1;
		//[��Ŀ����������������]
		for (i = goal_index; i < MORPHIN_LINE_NUM; i++)
		{
			if (line_dist[0][i] == 0)
			{
				counter = i + 1;
				decounter = 0;
				decounter_safe = 0;
// 				if (counter >= 10)
// 					counter = 10;
// 				if (counter == 0)
// 					continue;//[���޿�ͨ������]
// 				else
// 					break;//[�Ѿ���������ͨ�����ߣ��˳�����������]
			}
			else if (line_dist[0][i] > 1000 && line_rate[i] > 0.75)
			{
				decounter++;
				decounter_safe++;
				if (decounter_safe == 3)
				{
					ls_index = i;
					break;
				}
			}
			else if (line_dist[0][i] > 1000)
			{
				decounter++;
				if (decounter == (int)(counter * 2 / 3))
				{
					ls_index = i;
					break;
				}
			}
			else
			{
				counter = i + 1;
				decounter = 0;
				decounter_safe = 0;
			}
		}

		ret = ls_index;
	}
	else
	{
		//[��Ŀ����������������]
		counter = 0;
		for (i = goal_index; i >= 0; i--)
		{
			if (line_dist[0][i] == 0)
			{
				counter = MORPHIN_LINE_NUM - i;
				decounter = 0;
				decounter_safe = 0;
// 				if (counter == 0)
// 					continue;//[���޿�ͨ������]
// 				else
// 					break;//[�Ѿ���������ͨ�����ߣ��˳�����������]
			}
			else if (line_dist[0][i] > 1000 && line_rate[i] > 0.75)
			{
				decounter++;
				decounter_safe++;
				if (decounter_safe == 3)
				{
					rs_index = i;
					break;
				}
			}
			else if (line_dist[0][i] > 1000)
			{
				decounter++;
				if (decounter == (int)(counter * 2 / 3))
				{
					rs_index = i;
					break;
				}
// 				rs_index = i;
// 				counter++;
// 				break;
			}
			else
			{
				counter = MORPHIN_LINE_NUM - i;
				decounter = 0;
				decounter_safe = 0;
			}
		}

		ret = rs_index;
	}
	

	return ret;
}

/*==================================================================
 * ������  ��	int get_morphin_index(int goal_index, int mode)
 * ����    ��	����������ߵ�ȫ�ֽ������;ֲ�������
 * ���������	int goal_index		Ŀ����Ӧ����
 *				int mode			ѡ��ģʽ  0  ��������  1  ͨ��������  2  �����ر�  3  ����ת��  4  ����ת��
 * ���������	
 * ����ֵ  ��	0  �н�  -1  �޽�
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.06.07
 * �޸ļ�¼��	
 *==================================================================*/
int get_morphin_index(int goal_index, int mode)
{
	int ret = -1;
	double gmax = -1;
	double lmax = -1;

	switch (mode)
	{
	case 0://[��������]
		ret = search_length_first(goal_index, 0);
		if (ret != -1)
		{
			local_index = ret;
			ret = 0;
		}
		break;

 	case 1://[ͨ��������]
		ret = search_length_first(goal_index, 1);
		if (ret != -1)
		{
			local_index = ret;
			ret = 0;
		}
		break;

	case 2://[�����رܣ�ֻ��ȫ������]
		ret = search_length_first(goal_index, 2);
		if (ret != -1)
		{
			local_index = ret;
			ret = 0;
		}
		break;
	case 3:
		ret = search_length_first(goal_index, 3);
		if (ret != -1)
		{
			local_index = ret;
			ret = 0;
		}
		break;
	case 4:
		ret = search_length_first(goal_index, 4);
		if (ret != -1)
		{
			local_index = ret;
			ret = 0;
		}
		break;
	}

	if (gmax != -1 || lmax != -1)
		ret = 0;
	
	return ret;
}

/*==================================================================
 * ������  ��	int get_morphin_line(COOR2 sub_goal_pt, double dist2obs, int obs_clear_flag, int goal_index)
 * ����    ��	��õ�ǰ�������ʺϵ���ʻ����
 * ���������	COOR2 sub_goal_pt		Ŀ���
 *				double dist2obs			��ͷ���ϰ���ľ���
 *				int obs_clear_flag		��Ŀ���Ƿ����ϰ���ı�־
 *				int goal_index			����Ŀ���Ķ�Ӧ��������
 * ���������	
 * ����ֵ  ��	0  �н�  -1  �޽�
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.06.04
 * �޸ļ�¼��	
 *==================================================================*/
int get_morphin_line(COOR2 sub_goal_pt, double dist2obs, int obs_clear_flag, int goal_index, int fast_turn_flag)
{
	int ret = -1;
	//[1.���ݾ��뻮��]
//	double dist2goal;
//	COOR2 tmp_coor2 = {0, 400};
//	dist2goal = dist_point(&sub_goal_pt, &tmp_coor2);

	global_index = -1;
	local_index = -1;

	if (obs_clear_flag == 1)
	{
		local_index = goal_index;
		ret = goal_index;
		return ret;
	}

	if (fast_turn_flag > 0)
	{
		if (fast_turn_flag == 1)
		{//[����ת]
			ret = get_morphin_index(goal_index, 3);
		}
		else
		{//[����ת]
			ret = get_morphin_index(goal_index, 4);
		}
		
	}
	else
	{
		if (dist2obs >= 1600)
			ret = get_morphin_index(goal_index, 0);

		else if (dist2obs >= 500 && dist2obs < 1600)
			ret = get_morphin_index(goal_index, 1);

		else
			ret = get_morphin_index(goal_index, 2);
	}
	

	return ret;
}

/*==================================================================
 * ������  ��	void create_line_by_theta(double theta, COOR2 *pts, int &pts_num, int mode)
 * ����    ��	���������ǰ��ƫ������Morphin�켣��
 * ���������	double theta		ǰ��ƫ��
 * ���������	COOR2 *pts			���ɵĹ켣��
 *				int &pts_num		�켣�ߵ����(����200����)
 *				int mode			0  ����y�����������  1����y�����������
 * ����ֵ  ��	
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.06.04
 * �޸ļ�¼��	
 *==================================================================*/
void create_line_by_theta(double theta, COOR2 *pts, int &pts_num, int mode)
{
	int i;
	double radian;
	double radius;
	double D = 0;
	double car_len = 400;
	double step;
	double sd;
	double theta_a, theta_b;
	int last_x = 0;
	int last_y = 0;
	int temp_x = 0;
	int temp_y = 0;
	int flip_horizontal_flag = 0;


	memset(pts, 0, 200 * sizeof(COOR2));
	pts_num = 0;

	if (fabs(theta - 90) < 0.1)
	{
		for (i=0; i<121; i++)
		{
			pts[i].x = 0;
			pts[i].y = (INT32)(i * 25) + 400;
		}
		pts_num = 121;
	}
	else
	{
		if (theta > 90)
		{
			if (mode == 2)
			{
				if (theta > g_morphin_angle[0])
					theta = g_morphin_angle[0];
			}
			else
			{
				if (theta > g_morphin_angle[12])
				{
					theta = g_morphin_angle[12];
				}
			}
			theta = theta - 90;
		}
		else
		{
			if (theta < g_morphin_angle[MORPHIN_LINE_NUM - 1])
			{
				theta = g_morphin_angle[MORPHIN_LINE_NUM - 1];
			}
			theta = 90 - theta;
			flip_horizontal_flag = 1;
		}

		//[���ݽǶ���������]
		radius = car_len / sin(theta / 180 * PI);
		sd = 3000;
		radian = sd / radius;
		D = sqrt(radius * radius - car_len * car_len);
		step = radian / 199;

		theta_a = theta / 180 * PI;
		theta_b = radian;
		pts_num = 0;
		last_x = 0;
		last_y = 0;
		for (theta = theta_a; theta <= theta_a + theta_b && theta < PI / 2; theta += step)
		{
			temp_x = 0;
			temp_y = 0;
			temp_x = (INT32)(radius * cos(theta) - D);
			temp_y = (INT32)(radius * sin(theta));

			temp_x = temp_x / GRID_LEN_PER_CELL + g_grid_center.x;
			temp_y = temp_y / GRID_LEN_PER_CELL + g_grid_center.y;

			if (temp_x != last_x || temp_y != last_y)
			{
				pts[pts_num].x = (INT32)(radius * cos(theta) - D);
				pts[pts_num].y = (INT32)(radius * sin(theta));
				pts_num++;
				last_x = temp_x;
				last_y = temp_y;
			}

			//[�޶���900��]
			if (pts[pts_num - 1].x < -900)
			{
				break;
			}
		}

		if (flip_horizontal_flag == 1)
		{
			for (i=0; i<pts_num; i++)
				pts[i].x = -pts[i].x;
		}
	}

	if (mode == 1 || mode == 2)
	{
		for (i = 0; i<pts_num;i++)
		{
			pts[i].y = -(pts[i].y - 400);
		}
	}
}

/*==================================================================
 * ������  ��	int filter_indxe()
 * ����    ��	�����������˲�
 * ���������	
 * ���������	
 * ����ֵ  ��	int �˲��������ֵ
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.06.09
 * �޸ļ�¼��	
 *==================================================================*/
int filter_indxe()
{
	int ret = -1;

	if (local_index != -1)
	{
		if (abs(local_index - last_index) <= steer_step_in_frame)
		{
			ret = local_index;
		}
		else
		{
			ret = local_index;//local_index > last_index ? ret = last_index + steer_step_in_frame : ret = last_index - steer_step_in_frame;
			if (ret < 0)
			{
				ret = 0;
			}
			else if (ret >= MORPHIN_LINE_NUM)
			{
				ret = MORPHIN_LINE_NUM - 1;
			}
		}
	}

	return ret;
}

void recover_one_frame()
{
	filter_num = recover_filter_num;
	filter_cur_idx = recover_filter_cur_idx;
	memcpy(filter, recover_filter, sizeof(double) * FILTER_WIN);

	cross_cur_speed = recover_cross_cur_speed;
	speed_timer = recover_speed_timer;
	last_index = recover_last_index;
}

/*==================================================================
 * ������  ��	double smooth_angle(int index)
 * ����    ��	��һ����������ȥƽ������
 * ���������	int index		��ǰһ֡������
 * ���������	
 * ����ֵ  ��	double			ƽ��������ǰ��ת���
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.06.15
 * �޸ļ�¼��	
 *==================================================================*/
double smooth_angle(int index)
{
	double avg = 0;
	int i;

	recover_filter_num = filter_num;
	recover_filter_cur_idx = filter_cur_idx;
	memcpy(recover_filter, filter, sizeof(double) * FILTER_WIN);

	if (filter_num == FILTER_WIN)
	{
		filter[filter_cur_idx] = g_morphin_angle[index];
		filter_cur_idx++;
		if (filter_cur_idx >= FILTER_WIN)
			filter_cur_idx = 0;

		for (i=0; i<filter_num; i++)
			avg += filter[i];
	}
	else
	{
		filter[filter_cur_idx] = g_morphin_angle[index];
		filter_cur_idx++;
		filter_num++;

		for (i=1;i<=filter_num;i++)
		{
			int idx = (filter_cur_idx - i + FILTER_WIN) % FILTER_WIN;
			avg += filter[idx];
		}
	}
	avg /= filter_num;

	return avg;
}

/*==================================================================
 * ������  ��	int verify_clear_index(int goal_index)
 * ����    ��	������ϰ������߶��Ƿ����ٽ�״̬�����������ϰ������ϰ��Ľ��紦
 * ���������	
 * ���������	
 * ����ֵ  ��	int			-1  
 * ����    ��	
 * ����    ��	
 * �޸ļ�¼��	
 *==================================================================*/
int verify_clear_index(int goal_index)
{
	int ret = -1;
	int loffset = 0;
	int roffset = 0;
	int i;

	if (((int)line_rate[goal_index]) != 1)
	{
		loffset = 1;
		roffset = 1;
	}

	for (i=goal_index-1;i>goal_index-1-rate_search_win;i--)
	{
		if (i<0)
			break;

		if (line_rate[i] < 1)
			roffset++;
	}

	for (i=goal_index+1;i<goal_index+1+rate_search_win;i++)
	{
		if (i>MORPHIN_LINE_NUM-1)
			break;

		if (line_rate[i] < 1)
			loffset++;
	}

	if (loffset > roffset)
	{
		ret = goal_index - loffset;
		if (ret < 0)
			ret = 0;
	}
	else if (loffset < roffset)
	{
		ret = goal_index + roffset;
		if (ret > MORPHIN_LINE_NUM - 1)
			ret = MORPHIN_LINE_NUM - 1;
	}
	else if ((loffset == roffset) && (loffset == 0))
	{
		ret = goal_index;
	}

	return ret;
}

double check_dist2obs_by_angle(double angle)
{
	int i;
	COOR2 temp_pts[200];
	int temp_pts_num = 0;

	create_line_by_theta(angle, temp_pts, temp_pts_num, 0);

	int x, y;
	for (i=0; i<temp_pts_num; i++)//[ɨ����������]
	{
		x = temp_pts[i].x / GRID_LEN_PER_CELL + (g_grid_center.x - 1);;
		y = temp_pts[i].y / GRID_LEN_PER_CELL + g_grid_center.y;

		if (g_closed_grid_map[y][x] == 1)
			break;
	}

	double dist2obs = 0;
	double xx, yy;
	if (i == 0)
		dist2obs = 0;
	else
	{
		xx = (double)temp_pts[i - 1].x;
		yy = (double)temp_pts[i - 1].y - 400;
		dist2obs = sqrt(xx * xx + yy * yy);
	}

	return dist2obs;
}

void hd_get_avg_mid_line(double angle, double dist2obs)
{
	int i = 0;

	double radian = angle * PI / 180;
	double radius = 0;
	double step_radian = 0;
	double s_radian = 0;
	double e_radian = 0;
	COOR2 tmp_pt;

	double car_len = 400;
	double D = 0;
	double theta = 0;

	if (dist2obs == -1)
	{
		dist2obs = 2000;
	}
	

	int step = (int)(dist2obs / 20);
	if (fabs(radian - PI / 2) < 0.002)//[0.002���Ƚ���0.1��]
	{
/*		step = 158;*/
		for (i = 0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			g_mid_line[i].x = 0;
			g_mid_line[i].y = i * step;
		}
	}
	else if (radian > PI / 2)
	{//[���Morphin��]
		theta = radian - PI / 2;
		radius = car_len / sin(theta);

		D = sqrt(radius * radius - car_len * car_len);
		radian = (dist2obs) / D;

		if (radian > PI / 2)
			radian = PI / 2;

		step_radian = radian / 19;

		s_radian = 0;
		e_radian = radian;

		i = 0;

		for (radian = s_radian; radian <= e_radian; radian += step_radian)
		{
			tmp_pt.x = (INT32)(cos(radian) * D - D);
			tmp_pt.y = (INT32)(sin(radian) * D);

			g_mid_line[i].x = tmp_pt.x;
			g_mid_line[i].y = tmp_pt.y;
			i++;

			if (i == NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1)
				break;
		}
		g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x = (INT32)(cos(e_radian) * D - D);
		g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y = (INT32)(sin(e_radian) * D);
	}
	else
	{//[�Ҳ�Morphin��]
		theta = PI / 2 - radian;
		radius = car_len / sin(theta);

		D = sqrt(radius * radius - car_len * car_len);

		radian = (dist2obs) / D;
		if (PI - radian < PI / 2)
			radian = PI / 2;

		step_radian = radian / 19;

		s_radian = PI;
		e_radian = PI - radian;

		i = 0;

		for (radian = s_radian; radian >= e_radian; radian -= step_radian)
		{
			tmp_pt.x = (INT32)(cos(radian) * D + D);
			tmp_pt.y = (INT32)(sin(radian) * D);

			g_mid_line[i].x = tmp_pt.x;
			g_mid_line[i].y = tmp_pt.y;
			i++;

			if (i == NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1)
				break;
		}
		g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x = (INT32)(cos(e_radian) * D + D);
		g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y = (INT32)(sin(e_radian) * D);
	}
}

//***********************************************************************************************
//                                zgccmax 2014.June.28
//void get_avg_mid_line(double angle, double dist2obs)
//param:    double angle			ƽ������Ĺ滮�Ƕ�
//return:   void
//discribe: ����ƽ����Ĺ滮�Ƕ�����·��
//***********************************************************************************************
void get_avg_mid_line(double angle, double dist2obs)
{
	int i = 0;

	double radian = angle * PI / 180;
	double radius = 0;
	double step_radian = 0;
	double s_radian = 0;
	double e_radian = 0;
	COOR2 tmp_pt;

	double car_len = 400;
	double D = 0;
	double theta = 0;

	dist2obs = 2000;

	int step = (int)(dist2obs / 20);
	if (fabs(radian - PI / 2) < 0.002)//[0.002���Ƚ���0.1��]
	{
		step = 158;
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			g_mid_line[i].x = 0;
			g_mid_line[i].y = i * step;
		}
	}
	else if (radian > PI / 2)
	{//[���Morphin��]
		theta = radian - PI / 2;
		radius = car_len / sin(theta);

		D = sqrt(radius * radius - car_len * car_len);
		radian = (dist2obs) / D;

		if (radian > PI / 2)
			radian = PI / 2;

		step_radian = radian / 19;

		s_radian = 0;
		e_radian = radian;

		i = 0;

		for (radian = s_radian; radian <= e_radian; radian += step_radian)
		{
			tmp_pt.x = (INT32)(cos(radian) * D - D);
			tmp_pt.y = (INT32)(sin(radian) * D);

			g_mid_line[i].x = tmp_pt.x;
			g_mid_line[i].y = tmp_pt.y;
			i++;

			if (i==NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1)
				break;
		}
		g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x = (INT32)(cos(e_radian) * D - D);
		g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y = (INT32)(sin(e_radian) * D);
	}
	else
	{//[�Ҳ�Morphin��]
		theta = PI / 2 - radian;
		radius = car_len / sin(theta);

		D = sqrt(radius * radius - car_len * car_len);

		radian = (dist2obs) / D;
		if (PI - radian < PI / 2)
			radian = PI / 2;

		step_radian = radian / 19;
		
		s_radian = PI;
		e_radian = PI - radian;

		i = 0;

		for (radian = s_radian; radian >= e_radian; radian -= step_radian)
		{
			tmp_pt.x = (INT32)(cos(radian) * D + D);
			tmp_pt.y = (INT32)(sin(radian) * D);

			g_mid_line[i].x = tmp_pt.x;
			g_mid_line[i].y = tmp_pt.y;
			i++;

			if (i==NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1)
				break;
		}
		g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x = (INT32)(cos(e_radian) * D + D);
		g_mid_line[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y = (INT32)(sin(e_radian) * D);
	}
}

int speed_control(double angle, int best_index)
{
	int speed = 0;
	int max_speed = (int)(g_roving_speed * CM);

	//[1.��������ٶȿ���]
	double dist2obs =  check_dist2obs_by_angle(angle);
	if (dist2obs <= 250)
		dist2obs = check_dist2obs_by_angle(g_morphin_angle[best_index]);

	if (dist2obs <= 250)
		speed = 0;
	else if (dist2obs > 250 && dist2obs <= 750)
		speed = (int)(5 * CM);
	else if (dist2obs > 750 && dist2obs <= 1250)
		speed = (int)(10 * CM);
	else if (dist2obs > 1250 && dist2obs <= 1750)
		speed = (int)(15 * CM);
	else if (dist2obs > 1750 && dist2obs <= 2250)
		speed = (int)(20 * CM);
	else if (dist2obs > 2250 && dist2obs <= 2750)
		speed = (int)(25 * CM);
	else
		speed = (int)(30 * CM);
	speed = (speed > max_speed ? max_speed : speed);

	//[2.���ݷ����̽ǶȽ����ٶȿ���]
	double steer = fabs(angle - 90);
	if (steer < 2)
		speed = speed;
	else if (steer >= 2 && steer < 4)
	{
		speed = (int)(0.95 * speed);
		if (speed < (int)(5 * CM) && speed > 0)
			speed = (int)(5 * CM);
	}
	else if (steer >= 4 && steer < 7)
	{
		speed = (int)(0.9 * speed);
		if (speed < (int)(5 * CM) && speed > 0)
			speed = (int)(5 * CM);
	}
	else if (steer >= 7 && steer < 10)
	{
		speed = (int)(0.85 * speed);
		if (speed < (int)(5 * CM) && speed > 0)
			speed = (int)(5 * CM);
	}
	else if (steer >= 10 && steer < 15)
		speed = (speed > (int)(12 * CM) ? (int)(12 * CM) : speed);
	else if (steer >= 15 && steer < 20)
		speed = (speed > (int)(8 * CM) ? (int)(8 * CM) : speed);
	else if (steer >= 20)
		speed = (int)(5 * CM);

	//[3.�Ӽ��ٶȿ���]
	recover_cross_cur_speed = cross_cur_speed;
	recover_speed_timer = speed_timer;

	speed_timer = 0;
	if (speed >= cross_cur_speed)
	{
		if (speed_timer > 0)
		{
			//speed_timer--;
			speed_timer = 0;
		}

		else
		{
			speed_timer = 0;
			cross_cur_speed = speed;
		}
	}
	else
	{
		speed_timer = 0;
		cross_cur_speed = speed;
	}

	//[4.ʵ���ٶȹ滮]
	if (g_real_speed < (int)(5 * CM))
		cross_cur_speed = cross_cur_speed > (int)(7 * CM) ? (int)(7 * CM) : cross_cur_speed;
	else if (g_real_speed < (int)(10 * CM))
		cross_cur_speed = cross_cur_speed > (int)(12 * CM) ? (int)(12 * CM) : cross_cur_speed;
	else if (g_real_speed < (int)(15 * CM))
		cross_cur_speed = cross_cur_speed > (int)(17 * CM) ? (int)(17 * CM) : cross_cur_speed;
	else if (g_real_speed < (int)(20 * CM))
		cross_cur_speed = cross_cur_speed > (int)(22 * CM) ? (int)(22 * CM) : cross_cur_speed;
	else if (g_real_speed < (int)(25 * CM))
		cross_cur_speed = cross_cur_speed > (int)(27 * CM) ? (int)(27 * CM) : cross_cur_speed;
	else if (g_real_speed < (int)(30 * CM))
		cross_cur_speed = cross_cur_speed > (int)(32 * CM) ? (int)(32 * CM) : cross_cur_speed;

	return cross_cur_speed;
}

/*==================================================================
 * ������  ��	int cross_country(COOR2 sub_goal_pt)
 * ����    ��	����Ŀ������ԽҰ
 * ���������	Ŀ���
 * ����ֵ  ��	ѡ�����������  -1  �޷���ʻ
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.05.29
 * �޸ļ�¼��	
 *==================================================================*/
int cross_country(COOR2 sub_goal_pt, int &g_cross_speed)
{
	int ret = -1;
	int i, j, k;

// 	ofstream outfile;
// 	outfile.open("D:\\grid.txt", ios::app);
// 	for (i = GRID_HEIGHT - 1; i >= 0; i--)
// 	{
// 		for (j = 0; j < GRID_WIDTH - 1;j++)
// 		{
// 			outfile << g_closed_grid_map[i][j] << ' ';
// 		}
// 		outfile << "\r\n";
// 	}
// 	outfile.close();
	//[1.��ʼ��]
	memset(line_rate, 0, sizeof(double) * MORPHIN_LINE_NUM);
	memset(line_dist, 0, sizeof(double) * 4 * MORPHIN_LINE_NUM);
	for (i=0; i<MORPHIN_LINE_NUM; i++)
	{
		line_dist[0][i] = DIST1;
		line_dist[1][i] = DIST2;
		line_dist[2][i] = DIST3;
		line_dist[3][i] = DIST4;
	}

	//[2.��ײ���]
	MORPHIN2 morphin = g_morhpin2[0];
	COOR2 temp_pts[200];
	int temp_pts_num = 0;
	int x, y;
	double dist2obs = 0;
	double xx, yy;
	for (i = 0; i < MORPHIN_LINE_NUM; i++)
	{
		memset(temp_pts, 0, 200 * sizeof(COOR2));
		temp_pts_num = 0;
		memcpy(temp_pts, morphin.g_morphin_lines[i], morphin.g_morphin_lines_num[i] * sizeof(COOR2));
		temp_pts_num = morphin.g_morphin_lines_num[i];

		int keep_mid_flag = -1;
		for (j = 0; j < temp_pts_num; j++)//[ɨ����������]
		{
			if (i>MORPHIN_MID_INDEX)
			{
				x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x);
			}
			else
			{ 
				x = temp_pts[j].x / GRID_LEN_PER_CELL + (g_grid_center.x - 1);
			}
			
			y = temp_pts[j].y / GRID_LEN_PER_CELL + g_grid_center.y;

			if (keep_mid_flag == -1)
				for (k = x - keep_mid_win; k <= x + keep_mid_win; k++)
					if (g_closed_grid_map[y][k] == 1)
						keep_mid_flag = j;

			if (g_closed_grid_map[y][x] == 1)
				break;
		}

		if (j == 0)
		{
			dist2obs = 0;
			keep_mid_dist[i] = 0;
		}
		else
		{
			xx = (double)temp_pts[j - 1].x;
			yy = (double)temp_pts[j - 1].y - 400;
			dist2obs = sqrt(xx * xx + yy * yy);

			double dist2;
			if (keep_mid_flag == -1)
			{
				keep_mid_flag = j;
			}
			xx = (double)temp_pts[keep_mid_flag - 1].x;
			yy = (double)temp_pts[keep_mid_flag - 1].y - 400;
			if (yy <= 0)
			{
				dist2 = 0;
			}
			else
			{
				dist2 = sqrt(xx * xx + yy * yy);
			}
			
			keep_mid_dist[i] = dist2;
		}

		//[����ͨ����]
		line_rate[i] = (j + 0.f) / temp_pts_num;

		if (dist2obs >= DIST4)
		{
			line_dist[0][i] = dist2obs;
			line_dist[1][i] = dist2obs;
			line_dist[2][i] = dist2obs;
			line_dist[3][i] = dist2obs;
		}
		else if (dist2obs >= DIST3)
		{
			line_dist[0][i] = dist2obs;
			line_dist[1][i] = dist2obs;
			line_dist[2][i] = dist2obs;
			line_dist[3][i] = 0;
		}
		else if (dist2obs >= DIST2)
		{
			line_dist[0][i] = dist2obs;
			line_dist[1][i] = dist2obs;
			line_dist[2][i] = 0;
			line_dist[3][i] = 0;
		}
		else if (dist2obs >= DIST1)
		{
			line_dist[0][i] = dist2obs;
			line_dist[1][i] = 0;
			line_dist[2][i] = 0;
			line_dist[3][i] = 0;
		}
		else
		{
			line_dist[0][i] = 0;
			line_dist[1][i] = 0;
			line_dist[2][i] = 0;
			line_dist[3][i] = 0;
		}
	}

	//[3.Ŀ�������ĵ�]
	int goal_index = -1;
	double radian;
//	double radius;
	double theta;
	double min_angle = 1000000000;
	double D = 0;
	double car_len = 400;
	xx = sub_goal_pt.x;
	yy = sub_goal_pt.y;
	//[ȷ��Ŀ������ڹ켣���±꣬��������������Ŀ��Ĺ켣��]
	int fast_turn_flag = 0;
	if ((xx == 0 && yy > 0) || fabs(yy / xx) <= 0.0017453)
	{//[Ŀ���λ�ڽ���90�ȵ�λ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = PI * 0.5;
		theta = radian / PI * 180;
	}
	else
	{
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx < 0 && yy < -xx)
		{
			fast_turn_flag = 1;
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			fast_turn_flag = 2;
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radian = atan(car_len / D);
			theta = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radian = atan(car_len / D);
			theta = 90 - (radian / PI * 180);
		}

		//[������ӽ�Ŀ���Ļ��ߣ��൱��ȫ��Ŀ��Ƕ�����]
		for (i=0; i<MORPHIN_LINE_NUM; i++)
		{
			if (i==MORPHIN_MID_INDEX)
				continue;
				
			if (fabs(theta - g_morphin_angle[i]) < min_angle)
			{
				min_angle = fabs(theta - g_morphin_angle[i]);
				goal_index = i;
			}
		}
	}

	memcpy(temp_pts, morphin.g_morphin_lines[goal_index], morphin.g_morphin_lines_num[goal_index] * sizeof(COOR2));
	temp_pts_num = morphin.g_morphin_lines_num[goal_index];

	//[4.�����Ŀ���֮����ϰ���]
	int obs_clear_flag = 0;
	for (i=0; i<temp_pts_num; i++)//[ɨ����������]
	{
		x = temp_pts[i].x / GRID_LEN_PER_CELL + (g_grid_center.x - 1);;
		y = temp_pts[i].y / GRID_LEN_PER_CELL + g_grid_center.y;

		if (g_closed_grid_map[y][x] == 1)
			break;
	}
	if (i == temp_pts_num)
	{
		obs_clear_flag = 1;
		i--;
	}
	xx = (double)temp_pts[i].x;
	yy = (double)temp_pts[i].y - 400;
	dist2obs = sqrt(xx * xx + yy * yy);
	//[����Ƿ�����ٽ����]
	if (obs_clear_flag == 1)
	{
		ret = verify_clear_index(goal_index);
		if (ret == -1)
		{
			obs_clear_flag = 0;
		}
		else
		{
			goal_index = ret;
		}
	}

	//[5.����ѡ��]
	ret = get_morphin_line(sub_goal_pt, dist2obs, obs_clear_flag, goal_index, fast_turn_flag);
	if (ret == -2)
	{
		last_index = MORPHIN_MID_INDEX;
		filter_num = 0;
		filter_cur_idx = 0;
		memset(filter, 0, sizeof(double) * FILTER_WIN);
		return ret;
	}

	//[6.�����˲�]
	ret = filter_indxe();
	if (ret == -2)
	{
		last_index = MORPHIN_MID_INDEX;
		filter_num = 0;
		filter_cur_idx = 0;
		memset(filter, 0, sizeof(double) * FILTER_WIN);
		return ret;
	}

	//[7.ƽ���Ƕ�]
	double angle;
	if (ret != -1)
	{
		angle = smooth_angle(ret);
	}
	else
	{
		last_index = MORPHIN_MID_INDEX;
		filter_num = 0;
		filter_cur_idx = 0;
		memset(filter, 0, sizeof(double) * FILTER_WIN);
	}
	

	//[7.���ɹ滮��]
	recover_last_index = last_index;
	last_index = ret;
	get_avg_mid_line(angle, dist2obs);
	g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
#ifdef MBUG_OPEN_
	MBUG("output : ");
	for (i=0;i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;i++)
	{
		MBUG("(%d, %d), ", g_mid_line[i].x, g_mid_line[i].y);
	}
	MBUG("\n");
#endif
	//[8.�ٶȿ���]
	int best_index = ret;
	g_cross_speed = speed_control(angle, best_index);
	if (g_cross_speed == 0)
		ret = -1;

	return ret;
}

void reset_cross_country_paragram()
{
	last_index = MORPHIN_MID_INDEX;
	filter_num = 0;
	filter_cur_idx = 0;
	memset(filter, 0, sizeof(double) * FILTER_WIN);
}

//[S�����Σ�������Σ�վ���]
int roving_for_s_obs(COOR2 goal_pt, int mode)
{
	int ret = 0;
	int i = 0;

	COOR2 virtual_pt;

	if (mode == 0)
	{
		virtual_pt.x = 0;
		virtual_pt.y = 800;
#ifdef MBUG_OPEN_
		MBUG("virtual_pt (%d, %d)\n", virtual_pt.x, virtual_pt.y);
#endif
	}
	else
	{
		virtual_pt = goal_pt;
	}

	double out_rate = 0;

//	ret = get_best_morphin_line7(virtual_pt, out_rate);
//	ret = get_best_morphin_line8(virtual_pt, out_rate);
	ret = get_best_morphin_line_hd(virtual_pt, out_rate);
	if (ret >= 0)
		g_cross_travel_rate = out_rate;
	else
		g_cross_travel_rate = 0;

	if (g_s_obs_last_angle != -1)
	{
		if (ret != -1)
		{
			if (fabs((double)ret - g_s_obs_last_angle) > 10)
				g_s_obs_speed_down = 1;
			else if (fabs((double)ret - g_s_obs_last_angle) < 5)
				g_s_obs_speed_down = 0;
			else
				g_s_obs_speed_down = 2;
		}
	}
	else
	{
		g_s_obs_last_angle = ret;
	}

	double avg = 0;
	if (ret == -1)
	{
#ifdef MBUG_OPEN_
		MBUG("morphin no way to go\n");
#endif
	}
	else
	{
		if (g_s_obs_avg_angle_num != S_OBS_AVG_NUM)
		{
			double angle = g_morphin_angle[ret];
			g_s_obs_avg_angle[g_s_obs_avg_index] = angle;
			g_s_obs_avg_index++;
			if (g_s_obs_avg_index == S_OBS_AVG_NUM)
			{
				g_s_obs_avg_index = 0;
			}

			g_s_obs_avg_angle_num++;

			for (i = 0; i < g_s_obs_avg_angle_num; i++)
			{
				avg += g_s_obs_avg_angle[i];
			}
			avg /= g_s_obs_avg_angle_num;
		}//[if]
		else
		{
			double angle = g_morphin_angle[ret];
			g_s_obs_avg_angle[g_s_obs_avg_index] = angle;
			g_s_obs_avg_index++;

			if (g_s_obs_avg_index == S_OBS_AVG_NUM)
			{
				g_s_obs_avg_index = 0;
			}

			for (i = 0; i < g_s_obs_avg_angle_num; i++)
			{
				avg += g_s_obs_avg_angle[i];
			}
			avg /= g_s_obs_avg_angle_num;
		}//[else]

		/*
		double theta = (90 - avg) * PI / 180;
		double x_step;
		double y_step;
		double y = 1000;
		double x = y * atan(theta);
		x_step = x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
		y_step = 600 / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
		g_mid_line[0].x = 0;
		g_mid_line[0].y = 0;
		for (i = 1; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			g_mid_line[i].x = (INT32)(i * x_step);
			g_mid_line[i].y = (INT32)(i * y_step);
		}
		*/
		get_avg_mid_line(avg, 2000);
		/*
		for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;i++)
		{
			if (g_mid_line[i].y > 800)
				break;
		}
		double x_step;
		double y_step;
		x_step = g_mid_line[i].x / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
		y_step = g_mid_line[i].y / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 2);
		g_mid_line[0].x = 0;
		g_mid_line[0].y = 0;
		for (i = 1; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			g_mid_line[i].x = (INT32)(i * x_step);
			g_mid_line[i].y = (INT32)(i * y_step);
		}
		*/

		g_long_line_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
	}

	return ret;
}

typedef struct 
{
	int xs;
	int xe;
	int y;
	int mid;
}AREA_SEARCH;
#define  AREA_NUM 5
#define  HD_AREA_NUM 5
COOR2 s_obs_mid_plan()
{
	int xs = 0;
	int xe = 0;
	int sum = 0;
	int mid = 0;
	AREA_SEARCH area[26];
	int area_num = 0;
	memset(area, 0, sizeof(AREA_SEARCH) * 26);

	//[1.ͳ�ƿ�ͨ������]
	int last_x = g_grid_center.x;
	int cur_x;
	for (int i = g_grid_center.y + 20; i < g_grid_center.y + 30;i++)
	{
		cur_x = last_x;
		xs = cur_x;
		xe = cur_x;
		for (int j = cur_x; j > cur_x - 14;j--)
		{
			if (g_closed_grid_map[i][j] == 0)
			{
				xs = j;
			}
			else
			{
				break;
			}
		}

		for (int j = cur_x; j < cur_x + 14; j++)
		{
			if (g_closed_grid_map[i][j] == 0)
			{
				xe = j;
			}
			else
			{
				break;
			}
		}

		//[�ҵ���϶]
		if (xe - xs + 1 >= 3 && xe - xs + 1 <= 16)
		{
			last_x = xs + (xe - xs + 1) / 2;
			area[area_num].xs = xs;
			area[area_num].xe = xe;
			area[area_num].y = i;
			area[area_num].mid = xs + (xe - xs + 1) / 2;
			area_num++;
		}
		else
		{
			break;
		}
	}

	//[2.��ֵ�˲�]
	COOR2 result;
	result.x = 0;
	result.y = 0;
	AREA_SEARCH mid_win[AREA_NUM];
	memset(mid_win, 0, sizeof(AREA_SEARCH) * AREA_NUM);
	for (int i = area_num; i > area_num - AREA_NUM; i--)
	{
		mid_win[area_num - i].mid = area[i - 1].mid;
		mid_win[area_num - i].y = area[i - 1].y;
	}

	if (area_num < 6)
	{
		return result;
	}
	else
	{
		AREA_SEARCH tmp;
		for (int i = 0; i < AREA_NUM - 1; i++)
		{
			for (int j = i + 1; j<AREA_NUM; j++)
			{
				if (mid_win[i].mid > mid_win[j].mid)
				{
					tmp = mid_win[i];
					mid_win[i] = mid_win[j];
					mid_win[j] = tmp;
				}
			}
		}

		result.x = mid_win[2].mid;
		result.y = mid_win[2].y;
		return result;
	}
}

#define LOOK_NEAR 1
#define LOOK_FAR 2
#define HD_HISTOGRAM_WIDTH 22
static double g_hd_look_histogram[HD_HISTOGRAM_WIDTH];
int hd_get_look_mode()
{
	int mode = LOOK_NEAR;
	memset(g_hd_look_histogram, 0, sizeof(double) * HD_HISTOGRAM_WIDTH);
	for (int i = 0; i < HD_HISTOGRAM_WIDTH;i++)
	{
		int x = g_hd_grid_center.x - (HD_HISTOGRAM_WIDTH / 2 - i);
		int flag = 0;
		for (int y = 40; y < 120;y++)
		{
			if (g_hd_closed_grid_map[y][x] >= 4)
			{
				g_hd_look_histogram[i] = y * GRID_LEN_PER_CELL_HD - 400;
				flag = 1;
				break;
			}
		}
		if (flag == 0)
		{
			g_hd_look_histogram[i] = 1100;
		}
	}

	return mode;
}

COOR2 hd_s_obs_mid_plan()
{
	int xs = 0;
	int xe = 0;
	int sum = 0;
	int mid = 0;
	AREA_SEARCH area[26];
	int area_num = 0;
	memset(area, 0, sizeof(AREA_SEARCH) * 26);
//	double include_angle[26];

	//[1.ͳ�ƿ�ͨ������]
	int last_x = g_hd_grid_center.x;
	int cur_x;
	for (int i = g_hd_grid_center.y + 48; i < g_hd_grid_center.y + 64; i++)
	{
		cur_x = last_x;
		xs = cur_x;
		xe = cur_x;
		for (int j = cur_x; j > cur_x - 28; j--)
		{
			if (g_hd_closed_grid_map[i][j] < 4)
			{
				xs = j;
			}
			else
			{
				break;
			}
		}

		for (int j = cur_x; j < cur_x + 28; j++)
		{
			if (g_hd_closed_grid_map[i][j] < 4)
			{
				xe = j;
			}
			else
			{
				break;
			}
		}

		//[�ҵ���϶]
		if (xe - xs + 1 >= 8 && xe - xs + 1 <= 40)
		{
			last_x = xs + (xe - xs + 1) / 2;
			area[area_num].xs = xs;
			area[area_num].xe = xe;
			area[area_num].y = i;
			area[area_num].mid = xs + (xe - xs + 1) / 2;
			area_num++;
		}
	}

	/*
	memset(include_angle, 0, 26 * sizeof(double));
	for (int i = 0; i < area_num-1;i++)
	{
		double x1 = (area[i].mid - g_hd_grid_center.x) * GRID_LEN_PER_CELL_HD;
		double y1 = (area[i].y - g_hd_grid_center.y) * GRID_LEN_PER_CELL_HD;
		double x2 = (area[i + 1].mid - g_hd_grid_center.x) * GRID_LEN_PER_CELL_HD;
		double y2 = (area[i + 1].y - g_hd_grid_center.y) * GRID_LEN_PER_CELL_HD;

		include_angle[i] = atan2(y2 - y1, x2 - x1) * 180 / PI;
	}
	double mid_filter[5];
	double temp_angle[26];
	memset(temp_angle, 0, sizeof(double) * 26);
	double temp;
	temp_angle[0] = include_angle[0];
	temp_angle[1] = include_angle[1];
	temp_angle[25] = include_angle[25];
	temp_angle[24] = include_angle[24];

	if (area_num > 5)
	{
		for (int i = 2; i < 26 - 2;i++)
		{
			memcpy(mid_filter, include_angle + i - 2, sizeof(double) * 5);

			for (int j = 0; j < 4; j++)
			{
				for (int k = j + 1; k<5; k++)
				{
					if (mid_filter[j] > mid_filter[k])
					{
						temp = mid_filter[j];
						mid_filter[j] = mid_filter[k];
						mid_filter[k] = temp;
					}
				}
			}

			temp_angle[i] = mid_filter[2];
		}

		memcpy(include_angle, temp_angle, sizeof(double) * 26);
	}
	*/

//	hd_get_look_mode();
	//[2.��ֵ�˲�]
	COOR2 result;
	result.x = 0;
	result.y = 0;
	AREA_SEARCH mid_win[HD_AREA_NUM];
	memset(mid_win, 0, sizeof(AREA_SEARCH) * HD_AREA_NUM);
	for (int i = 0; i < area_num; i++)
	{
		if (i >= HD_AREA_NUM)
			break;
		mid_win[i].mid = area[i].mid;
		mid_win[i].y = area[i].y;
	}

	if (area_num < HD_AREA_NUM)
	{
		return result;
	}
	else
	{
		AREA_SEARCH tmp;
		for (int i = 0; i < HD_AREA_NUM - 1; i++)
		{
			for (int j = i + 1; j<HD_AREA_NUM; j++)
			{
				if (mid_win[i].mid > mid_win[j].mid)
				{
					tmp = mid_win[i];
					mid_win[i] = mid_win[j];
					mid_win[j] = tmp;
				}
			}
		}

		result.x = mid_win[2].mid;
		result.y = mid_win[2].y;
	}

	double radian;
	double radius;
	double min_radius = 100000000;
	double D = 0;
	double car_len = 400;
	double xx, yy;
	int goal_index;
	xx = (result.x - g_hd_grid_center.x) * GRID_LEN_PER_CELL_HD;
	yy = (result.y - g_hd_grid_center.y) * GRID_LEN_PER_CELL_HD;
	//[ȷ��Ŀ������ڹ켣���±�]
	if (fabs(xx) <= 10)//|| xx < 10)//fabs(yy / xx) > 60)//[��Ӧ89.5�����ϵ�tanֵ]
	{
		//[Ŀ�����ǰ����ֱ��]
		goal_index = MORPHIN_MID_INDEX;
		radian = 90;
	}
	else
	{
		//[Ŀ��㲻�ڳ���ǰ��-45�ȵ�45��֮��ʹ����Сת��뾶�������]
		if (xx < 0 && yy < -xx)
		{
			xx = -720;
			yy = 0;
		}
		else if (xx > 0 && yy < xx)
		{
			xx = 720;
			yy = 0;
		}

		if (xx < 0)
		{
			D = -(xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx));
			radius = -sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 + radian / PI * 180;
		}
		else if (xx > 0)
		{
			D = xx / 2.0 + (yy * yy - car_len * car_len) / (2 * xx);
			radius = sqrt(D * D + car_len * car_len);
			radian = atan(car_len / D);

			radian = 90 - (radian / PI * 180);
		}

		for (int i = 0; i < MORPHIN_LINE_NUM; i++)
		{
			if (i == MORPHIN_MID_INDEX)
				continue;

			if (fabs(radius - g_morphin_radius2[i]) < min_radius)
			{
				min_radius = fabs(radius - g_morphin_radius2[i]);
				goal_index = i;
			}
		}
	}

// 	double angle;
// 	angle = g_morphin_angle[goal_index];
	hd_get_avg_mid_line(radian, 2000);

	//[��ȡ�滮��]
	int cut_index = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1;
	int num = 0;
	for (int i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;i++)
	{
		int x = (int)((g_mid_line[i].x / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.x);
		int y = (int)((g_mid_line[i].y / GRID_LEN_PER_CELL_HD) + g_hd_grid_center.y);
		if (g_mid_line[i].y > yy || g_hd_closed_grid_map[y][x] >= 4)
		{
			cut_index = i;
			break;
		}
	}

	if (g_mid_line[cut_index].y < 500)
	{
		result.x = 0;
		result.y = 0;
		return result;
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
	return result;
}

/*==================================================================
 * ������  ��	void TurnRoundAction(double angle, COOR2 path[], int &path_pts_num)
 * ����    ��	�����Ƕ������˶�����
 * ���������	double angle		�����Ƕ�
 *				COOR2 path[]		����˶�����
 *				path_pts_num		���߸���
 * ���������	
 * ����ֵ  ��	
 * ����    ��	zgccmax@163.com
 * ����    ��	
 * �޸ļ�¼��	
 *==================================================================*/
void TurnRoundAction(double angle, COOR2 path[], int &path_pts_num)
{
	int i = 0;

	memset(path, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	path_pts_num = 0;

	//[��ֹ���]
	if (angle > g_morphin_angle[0])
	{
		angle = g_morphin_angle[0];
	}
	else if (angle < g_morphin_angle[MORPHIN_LINE_NUM - 1])
	{
		angle = g_morphin_angle[MORPHIN_LINE_NUM - 1];
	}

	double radian = angle * PI / 180;
	double radius = 0;
	double step_radian = 0;
	double s_radian = 0;
	double e_radian = 0;
	COOR2 tmp_pt;

	double car_len = 400;
	double D = 0;
	double theta = 0;

	int step = g_morphin_search_dist / 20;
	if (fabs(radian - (PI * 0.5)) < 0.002)//[0.002���Ƚ���0.1��]
	{
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			path[i].x = 0;
			path[i].y = i * step;
		}
	}
	else if (radian > (PI * 0.5))
	{//[���Morphin��]

		theta = radian - (PI * 0.5);
		radius = car_len / sin(theta);

		radian = (g_morphin_search_dist - 400) / radius;

		if (radian + theta > (PI * 0.5))
		{
			radian = (PI * 0.5) - theta;
		}

		step_radian = radian / 18;
		D = sqrt(radius * radius - car_len * car_len);

		s_radian = theta;
		e_radian = radian + theta;

		path[0].x = 0;
		path[0].y = 0;
		i = 1;

		for (radian = s_radian; radian <= e_radian; radian += step_radian)
		{
			tmp_pt.x = (INT32)(cos(radian) * radius - D);
			tmp_pt.y = (INT32)(sin(radian) * radius);

			path[i].x = tmp_pt.x;
			path[i].y = tmp_pt.y;
			i++;
		}
		path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x = (INT32)(cos(e_radian) * radius - D);
		path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y = (INT32)(sin(e_radian) * radius);
	}
	else
	{//[�Ҳ�Morphin��]
		theta = (PI * 0.5) - radian;
		radius = car_len / sin(theta);

		radian = (g_morphin_search_dist - 400) / radius;

		if (PI - radian - theta < (PI * 0.5))
		{
			radian = (PI * 0.5) - theta;
		}

		step_radian = radian / 18;
		D = sqrt(radius * radius - car_len * car_len);

		s_radian = PI - theta;
		e_radian = PI - radian - theta;

		path[0].x = 0;
		path[0].y = 0;
		i = 1;

		for (radian = s_radian; radian >= e_radian; radian -= step_radian)
		{
			tmp_pt.x = (INT32)(cos(radian) * radius + D);
			tmp_pt.y = (INT32)(sin(radian) * radius);

			path[i].x = tmp_pt.x;
			path[i].y = tmp_pt.y;
			i++;
		}
		path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x = (INT32)(cos(e_radian) * radius + D);
		path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y = (INT32)(sin(e_radian) * radius);
	}

	path_pts_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
}

/*==================================================================
 * ������  ��	void TurnRoundActionBack(double angle, COOR2 path[], int &path_pts_num)
 * ����    ��	�����Ƕ������˶�����
 * ���������	double angle		�����Ƕ�
 *				COOR2 path[]		����˶�����
 *				path_pts_num		���߸���
 * ���������	
 * ����ֵ  ��	
 * ����    ��	zgccmax@163.com
 * ����    ��	
 * �޸ļ�¼��	
 *==================================================================*/
void TurnRoundActionBack(double angle, COOR2 path[], int &path_pts_num)
{
	int i = 0;

	memset(path, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	path_pts_num = 0;

	//[��ֹ���]
	if (angle > g_morphin_angle[0])
	{
		angle = g_morphin_angle[0];
	}
	else if (angle < g_morphin_angle[MORPHIN_LINE_NUM - 1])
	{
		angle = g_morphin_angle[MORPHIN_LINE_NUM - 1];
	}

	double radian = angle * PI / 180;
	double radius = 0;
	double step_radian = 0;
	double s_radian = 0;
	double e_radian = 0;
	COOR2 tmp_pt;

	double car_len = 400;
	double D = 0;
	double theta = 0;

	int step = g_morphin_search_dist / 20;
	if (fabs(radian - (PI * 0.5)) < 0.002)//[0.002���Ƚ���0.1��]
	{
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			path[i].x = 0;
			path[i].y = i * step;
		}
	}
	else if (radian > (PI * 0.5))
	{//[���Morphin��]

		theta = radian - (PI * 0.5);
		radius = car_len / sin(theta);

		radian = (g_morphin_search_dist - 400) / radius;

		if (radian + theta > (PI * 0.5))
		{
			radian = (PI * 0.5) - theta;
		}

		step_radian = radian / 19;
		D = sqrt(radius * radius - car_len * car_len);

		s_radian = theta;
		e_radian = radian + theta;

		i = 0;
		for (radian = s_radian; radian <= e_radian; radian += step_radian)
		{
			tmp_pt.x = (INT32)(cos(radian) * radius - D);
			tmp_pt.y = (INT32)(sin(radian) * radius);

			path[i].x = tmp_pt.x;
			path[i].y = -(tmp_pt.y - 400);
			i++;
		}
		path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x = (INT32)(cos(e_radian) * radius - D);
		path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y = -((INT32)(sin(e_radian) * radius) - 400);
	}
	else
	{//[�Ҳ�Morphin��]
		theta = (PI * 0.5) - radian;
		radius = car_len / sin(theta);

		radian = (g_morphin_search_dist - 400) / radius;

		if (PI - radian - theta < (PI * 0.5))
		{
			radian = (PI * 0.5) - theta;
		}

		step_radian = radian / 19;
		D = sqrt(radius * radius - car_len * car_len);

		s_radian = PI - theta;
		e_radian = PI - radian - theta;

		i = 0;
		for (radian = s_radian; radian >= e_radian; radian -= step_radian)
		{
			tmp_pt.x = (INT32)(cos(radian) * radius + D);
			tmp_pt.y = (INT32)(sin(radian) * radius);

			path[i].x = tmp_pt.x;
			path[i].y = -(tmp_pt.y - 400);
			i++;
		}
		path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].x = (INT32)(cos(e_radian) * radius + D);
		path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1].y = -((INT32)(sin(e_radian) * radius) - 400);
	}

	path_pts_num = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
}