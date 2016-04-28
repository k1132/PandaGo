#include "./basicfunction.h"
#include "./trace_road.h"

#include <math.h>
#include <string.h>

#ifdef __GNUC__

#else
void MBUG(const char * msg, ...)
{
	va_list arg;
	char szMsg[256];
	va_start(arg, msg);
	vsnprintf_s(szMsg, 256, msg, arg);
	fprintf(stderr, szMsg);
	va_end(arg);
}
#endif

/** transport the point from gps_pos2 coordinate to gps_pos1 coordinate */
//***********************************************************************************************
//                                zgccmax 2012.Feb.17
//COOR2 coordinate_transform(POSE_INFO gps_pos1, POSE_INFO gps_pos2, COOR2 point)
//param:    POSE_INFO	gps_pos1		所求点的GPS数据
//			POSE_INFO	gps_pos2		已知点的GPS数据
//			COOR2		point			已知点（车体坐标系下）
//return:   COOR2 p					所求点的坐标（车体坐标系下）
//discribe: 利用惯导系统的GPS，将已知的车体坐标系下的点转到另一时刻的车体坐标点。
//***********************************************************************************************
COOR2 coordinate_transform(POSE_INFO gps_pos1, POSE_INFO gps_pos2, COOR2 point)
{
	if (gps_pos1.spd == 0 && gps_pos2.spd == 0)
	{
		return point;
	}

	double x0,y0,angle_temp,xx,yy;
	COOR2 p1;
	int x1,y1,x2,y2;

	x2 = point.x;
	y2 = point.y;

	angle_temp = -gps_pos2.yaw * GPS_ANGLE_PRECISION;
	x0 = x2 * cos(angle_temp) + y2 * sin(angle_temp) + ((double)gps_pos2.ins_coord.x);
	y0 = -x2 * sin(angle_temp) + y2 * cos(angle_temp) + ((double)gps_pos2.ins_coord.y);
	angle_temp = -gps_pos1.yaw * GPS_ANGLE_PRECISION;
	xx = x0 - (double)gps_pos1.ins_coord.x;
	yy = y0 - (double)gps_pos1.ins_coord.y;
	x1 = int(xx * cos(angle_temp) - yy * sin(angle_temp));
	y1 = int(xx * sin(angle_temp) + yy * cos(angle_temp));
	p1.x = x1;
	p1.y = y1;

	return p1;
}

//***********************************************************************************************
//                                zgccmax 2012.Feb.17
//COOR2 diff_coordinate_transform(POSE_INFO gps_pos1, POSE_INFO gps_pos2, COOR2 point)
//param:    POSE_INFO	gps_pos1		所求点的GPS数据
//			POSE_INFO	gps_pos2		已知点的GPS数据
//			COOR2		point			已知点（车体坐标系下）
//return:   COOR2 p					所求点的坐标（车体坐标系下）
//discribe: 利用惯导系统的GPS，将已知的车体坐标系下的点转到另一时刻的车体坐标点。
//***********************************************************************************************
COOR2 diff_coordinate_transform(POSE_INFO gps_pos1, POSE_INFO gps_pos2, COOR2 point)
{
	if (gps_pos1.spd == 0 && gps_pos2.spd == 0)
	{
		return point;
	}

	double x0,y0,angle_temp,xx,yy;
	COOR2 p1;
	int x1,y1,x2,y2;

	x2 = point.x;
	y2 = point.y;

	angle_temp = -gps_pos2.yaw * GPS_ANGLE_PRECISION;
	x0 = x2 * cos(angle_temp) + y2 * sin(angle_temp) + ((double)gps_pos2.com_coord.x);
	y0 = -x2 * sin(angle_temp) + y2 * cos(angle_temp) + ((double)gps_pos2.com_coord.y);
	angle_temp = -gps_pos1.yaw * GPS_ANGLE_PRECISION;
	xx = x0 - (double)gps_pos1.com_coord.x;
	yy = y0 - (double)gps_pos1.com_coord.y;
	x1 = int(xx * cos(angle_temp) - yy * sin(angle_temp));
	y1 = int(xx * sin(angle_temp) + yy * cos(angle_temp));
	p1.x = x1;
	p1.y = y1;

	return p1;
}

//***********************************************************************************************
//                                zgccmax 2012.Feb.17
//void diff_coor2_e2v(POSE_INFO *gps, COOR2 *p1, COOR2 *p2)
//param:    POSE_INFO	*gps		GPS数据
//			COOR2		*p1		大地差分坐标点
//			COOR2	    *p2		车体坐标点
//return:   void
//discribe: 使用车子的差分数据，将一个大地差分坐标点转换到车体坐标系下
//***********************************************************************************************
void diff_coor2_e2v(POSE_INFO *gps, COOR2 *p1, COOR2 *p2)
{
	double angel_temp;

	angel_temp = (double)gps->yaw / 100000000;

	if (angel_temp > PI)
		angel_temp = (double)(PI * 2 - angel_temp);
	else
		angel_temp = -angel_temp;

	p2->x = (int)((double)(p1->x - gps->com_coord.x) * cos(angel_temp) - (double)(p1->y - gps->com_coord.y) * sin(angel_temp));
	p2->y = (int)((double)(p1->x - gps->com_coord.x) * sin(angel_temp) + (double)(p1->y - gps->com_coord.y) * cos(angel_temp));
}

//***********************************************************************************************
//                                zgccmax 2012.Feb.17
//void diff_coor2_v2e(POSE_INFO *gps, COOR2 *p1, COOR2 *p2)
//param:    POSE_INFO *gps		GPS数据
//			COOR2	  *p1		车体坐标点
//			COOR2     *p2		大地差分坐标点
//return:   void
//discribe: 使用车子的差分数据，将一个车体坐标系点转换到大地坐标系下
//***********************************************************************************************
void diff_coor2_v2e(POSE_INFO *gps, COOR2 *p1, COOR2 *p2)
{
	float angel_temp;
	angel_temp = (float)gps->yaw / 100000000;//[逆时针为正值]
	p2->x = (int)((float)(p1->x) * cos(angel_temp) - (float)(p1->y) * sin(angel_temp)) + gps->com_coord.x;
	p2->y = (int)((float)(p1->x) * sin(angel_temp) + (float)(p1->y) * cos(angel_temp)) + gps->com_coord.y;
}

//***********************************************************************************************
//                                zgccmax 2012.Feb.17
//void coor2_e2v(POSE_INFO *gps, COOR2 *p1, COOR2 *p2)
//param:    POSE_INFO *gps		GPS数据
//			COOR2	  *p1		大地惯导系统坐标点
//			COOR2     *p2		车体坐标点
//return:   void
//discribe: 使用车子的惯导系统GPS，将一个大地坐标系点转换到车体坐标系下
//***********************************************************************************************
void coor2_e2v(POSE_INFO *gps, COOR2 *p1, COOR2 *p2)
{
	float angel_temp;

	angel_temp = (float)gps->yaw / 100000000;

	if (angel_temp > PI)
		angel_temp = (float)(PI * 2 - angel_temp);
	else
		angel_temp = -angel_temp;

	p2->x = (int)((float)(p1->x - gps->ins_coord.x) * cos(angel_temp) - (float)(p1->y - gps->ins_coord.y) * sin(angel_temp));
	p2->y = (int)((float)(p1->x - gps->ins_coord.x) * sin(angel_temp) + (float)(p1->y - gps->ins_coord.y) * cos(angel_temp));
}

//***********************************************************************************************
//                                zgccmax 2012.Feb.17
//void coor2_v2e(POSE_INFO *gps, COOR2 *p1, COOR2 *p2)
//param:    POSE_INFO *gps		GPS数据
//			COOR2	  *p1		车体坐标点
//			COOR2     *p2		大地惯导系统坐标点
//return:   void
//discribe: 使用车子的惯导系统GPS，将一个车体坐标系点转换到大地坐标系下
//***********************************************************************************************
void coor2_v2e(POSE_INFO *gps, COOR2 *p1, COOR2 *p2)
{
	float angel_temp;
	angel_temp = (float)gps->yaw / 100000000;
	p2->x = (int)((float)(p1->x) * cos(angel_temp) - (float)(p1->y) * sin(angel_temp)) + gps->ins_coord.x;
	p2->y = (int)((float)(p1->x) * sin(angel_temp) + (float)(p1->y) * cos(angel_temp)) + gps->ins_coord.y;
}

//***********************************************************************************************
//                                zgccmax 2012.Feb.17
//float dist_point(COOR2 *p1, COOR2 *p2)
//param:    COOR2 *p1		
//			COOR2 *p2
//return:   两点距离
//discribe: 求两点距离
//***********************************************************************************************
double dist_point(COOR2 *p1, COOR2 *p2)
{
	double x_diff = p1->x - p2->x;
	double y_diff = p1->y - p2->y;
	x_diff = x_diff * x_diff;
	y_diff = y_diff * y_diff;

	double dist2 = x_diff + y_diff;
	double dist = sqrt(dist2);

	return dist;
	//return sqrt((double)((p1->x - p2->x) * (p1->x - p2->x)) + (double)((p1->y - p2->y) * (p1->y - p2->y)));
}

float decas(int degree, int coeff[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE], float t)
{
	int r, i;
	float t1;
	float coeffa[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	t1 = 1.0f - t;

	for (i = 0; i < degree; i++)
		coeffa[i] = (float)coeff[i];

	for (r = 1; r < degree; r++)
	{
		for (i = 0; i < degree - r; i++)
		{
			coeffa[i] = t1 * coeffa[i] + t * coeffa[i+1];
		}
	}

	return coeffa[0];
}

//***********************************************************************************************
//                                zgccmax 2011.Nov.26
//void bez_to_points(int npoints, int ppoints[100], int degree, int coeff[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE])
//param:    int npoints										贝塞尔拟合点数
//			int ppoints[100]								贝塞尔拟合点缓存  (最多100个点)
//			int degree										输入拟合点的个数
//			int coeff[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE]	拟合点
//return:   void
//discribe: 对一组进行贝塞尔拟合
//***********************************************************************************************
void bez_to_points(int npoints, int ppoints[100], int degree, int coeff[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE])
{
	int i;
	float t, delt;

	delt = 1.0f / (float)(npoints - 1);//[一共是npoints个点，那么其间等分数为npoints - 1]

	for (i = 0, t = 0.0f; i < npoints; i ++, t += delt)
	{
		ppoints[i] = (int)decas(degree, coeff, t);
	}
}

/*==================================================================
 * 函数名  ：	void get_bezier_line(COOR2 *line, int line_num, COOR2 *bz_line, int bz_line_num)
 * 功能    ：	对一组点进行贝塞尔拟合
 * 输入参数：	 COOR2	*line			输入点
				 int		line_num		输入点个数  (最多20)
				 COOR2	*bz_line		输出的贝塞尔拟合点
				 int		bz_line_num		输入需要贝塞尔拟合点个数  (最多100)
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
void get_bezier_line(COOR2 *line, int line_num, COOR2 *bz_line, int bz_line_num)
{
	int i;
	int x[20], y[20];
	int bz_tmp_x[100], bz_tmp_y[100];

	memset(bz_line, 0, sizeof(COOR2) * bz_line_num);

	for (i = 0; i < line_num; i ++)
	{
		x[i] = line[i].x;
		y[i] = line[i].y;
	}

	bez_to_points(bz_line_num, bz_tmp_x, line_num, x);
	bez_to_points(bz_line_num, bz_tmp_y, line_num, y);

	for (i = 0; i < bz_line_num; i ++)
	{
		bz_line[i].x = bz_tmp_x[i];
		bz_line[i].y = bz_tmp_y[i];
	}
}

//***********************************************************************************************
//                                zgccmax 203.Feb.17
//void swap_point(COOR2 *p1, COOR2 *p2)
//param:    COOR2 *p1			点1
//			COOR2 *p2			点2
//return:   void
//discribe: 交换一对点的值
//***********************************************************************************************
void swap_point(COOR2 *p1, COOR2 *p2)
{
	COOR2 tmp_p;

	memcpy(&tmp_p, p1, sizeof(COOR2));
	memcpy(p1, p2, sizeof(COOR2));
	memcpy(p2, &tmp_p, sizeof(COOR2));
}

//***********************************************************************************************
//                                zgccmax 203.Feb.17
//void get_mid_x(QUAD *box4, int *minx, int *maxx, int *midx)
//param:    QUAD *box4			一个栅格的四个点
//			int *minx			最小的x
//			int *maxx			最大的x
//			int *midx			(minx + maxx) / 2
//return:   void
//discribe: 求取一个栅格的四个点中，最小x、最大x以及中值x
//***********************************************************************************************
void get_mid_x(QUAD *box4, int *minx, int *maxx, int *midx)
{
	int i;
	int x[4], xx, xx1;

	if (box4 == NULL)
		return;

	x[0] = box4->coor2[0].x;
	x[1] = box4->coor2[1].x;
	x[2] = box4->coor2[2].x;
	x[3] = box4->coor2[3].x;

	xx = x[0];
	xx1 = x[0];

	for (i = 1; i < 4; i ++)
	{
		if (xx > x[i])
		{
			xx = x[i];
		}

		if (xx1 < x[i])
		{
			xx1 = x[i];
		}
	}

	if (minx != NULL)
		*minx = xx;

	if (maxx != NULL)
		*maxx = xx1;

	if (midx != NULL)
		*midx = (xx + xx1) / 2;
}

//***********************************************************************************************
//                                zgccmax 203.Feb.17
//void get_mid_y(QUAD *box4, int *miny, int *maxy, int *midy)
//param:    QUAD *box4			一个栅格的四个点
//			int *miny			最小的y
//			int *maxy			最大的y
//			int *midy			(miny + maxy) / 2
//return:   void
//discribe: 求取一个栅格的四个点中，最小y、最大y以及中值y
//***********************************************************************************************
void get_mid_y(QUAD *box4, int *miny, int *maxy, int *midy)
{
	int i;
	int y[4], yy, yy1;

	if (box4 == NULL)
		return;

	y[0] = box4->coor2[0].y;
	y[1] = box4->coor2[1].y;
	y[2] = box4->coor2[2].y;
	y[3] = box4->coor2[3].y;

	yy = y[0];
	yy1 = y[0];

	for (i = 1; i < 4; i ++)
	{
		if (yy > y[i])
		{
			yy = y[i];
		}

		if (yy1 < y[i])
		{
			yy1 = y[i];
		}
	}

	if (miny != NULL)
		*miny = yy;

	if (maxy != NULL)
		*maxy = yy1;

	if (midy != NULL)
		*midy = (yy + yy1) / 2;
}

/*==================================================================
 * 函数名  ：	int get_x_coord(int y, COOR2 *line, int line_num, int *x)
 * 功能    ：	将一个点投影到道路边上，求得其投影点的x
 * 输入参数：	int y			已知的点的y
				COOR2 *line		道路边
				int line_num	道路边描述点个数
				int *x			投影点的x
 * 输出参数：	
 * 返回值  ：	int  成功返回点数		否则-1
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int get_x_coord(int y, COOR2 *line, int line_num, int *x)
{
	int i;

	if (line == NULL)
		return -1;

	if (line_num == 0)
	{
		return -1;
	}

	COOR2 temp[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	memset(temp, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	memcpy(temp, line, line_num * sizeof(COOR2));
	/* 如果所给y在边之下 */
	if (y <= line[0].y)
	{
		//*x = line[0].x;
		*x = (int)(line[0].x + (line[0].x - line[1].x) * (y - line[0].y) / (line[0].y - line[1].y + .0f));
	}

	else if (y >= line[line_num - 1].y)
		//*x = line[line_num - 1].x;
	{
		*x = (int)(line[line_num - 1].x + (line[line_num - 1].x - line[line_num - 2].x) * (y - line[line_num - 1].y) / (line[line_num - 1].y - line[line_num - 2].y + .0f));
	}

	else
	{
		for (i = 0; i < line_num - 1; i ++)
		{
			if (y >= line[i].y && y <= line[i + 1].y && line[i].y != line[i + 1].y)
				*x = (int)(line[i].x + (line[i].x - line[i + 1].x) * (y - line[i].y) / (line[i].y - line[i + 1].y + .0f));
		}
	}

	return 0;
}

//***********************************************************************************************
//                                zgccmax 2009.Dec.13
//int roundf2i(float num)
//param:    float num		需要四舍五入的浮点数
//return:   int				浮点数四舍五入后的结果
//discribe: 对浮点数进行四舍五入
//***********************************************************************************************
int roundf2i(double num)
{
	if (num >= 0)
		return (int)(num + 0.5);

	else
		return (int)(num - 0.5);
}

//***********************************************************************************************
//                                zgccmax 2011.Nov.26
//void line_fitting(COOR2 *edge, int edge_len, COOR2 *out_edge, int &out_edge_len, int step, int max_pts_num)
//param:    COOR2 *edge					输入边 (**大致方向为沿着车行驶方向)
//			int edge_len				输入边点数
//			COOR2 *out_edge				输出边 (需要分配足够空间 200*sizeof(COOR2) )
//			int &out_edge_len			输出边点数
//			int step					插值步长
//			int max_pts_num				输出点最大数目
//return:   void
//discribe: 给一组点序列插值，最多插到200个点
//***********************************************************************************************
void line_fitting(COOR2 *edge, int edge_len, COOR2 *out_edge, int &out_edge_len, int step, int max_pts_num)
{
	int i = 0;
	float tempx1 = 0;
	float tempy1 = 0;
	float tempx2 = 0;
	float tempy2 = 0;
	float dx = 0;//[x变化率]
	float dy = 0;//[y变化率]
	int max_len = max_pts_num;
//	float x = 0;
	float y = 0;

	out_edge_len = 0;
	memset(out_edge, 0, sizeof(COOR2) * 200);

//	x = (float)edge[0].x;
	y = (float)edge[0].y;

	//[每次取两点进行插值]
	for (i=0; i<edge_len - 1; i++)
	{
		tempx1 = (float)edge[i].x;
		tempy1 = (float)edge[i].y;
		tempx2 = (float)edge[i + 1].x;
		tempy2 = (float)edge[i + 1].y;

		dx = tempx1 - tempx2;
		dy = tempy1 - tempy2;

		//[两点处于水平，跳过]
		if (dy == 0)
			continue;

		while(y < tempy2)
		{
			if (out_edge_len >= max_len)
				return;

			out_edge[out_edge_len].y = (INT32)y;

			if (dx != 0)
				out_edge[out_edge_len].x = roundf2i((dx / dy * (y - tempy1)) + tempx1);

			else
				out_edge[out_edge_len].x = (INT32)tempx1;

			out_edge_len++;
			y += step;
		}

		if (y >= edge[edge_len - 1].y && out_edge_len < max_len)
		{
			out_edge[out_edge_len].x = edge[edge_len - 1].x;
			out_edge[out_edge_len].y = edge[edge_len - 1].y;
			out_edge_len++;
			break;
		}
	}
}

/**
* @brief 获取融合给出的一条边的有效点数
* @param edge {COOR2 *} [in] 点的指针,也即边
* @return {int}, 成功返回点数, 否则-1
* @note 默认全0为结束
*/
int get_effective_points_num(COOR2 *edge)
{
	int i;

	if (edge == NULL)
	{
#ifdef MBUG_OPEN_
		MBUG("get_effective_points_num edge = NULL\n");
#endif
		return -1;
	}

	for (i = 0; \
		i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE && !(edge[i].x == 0 && edge[i].y == 0); \
		i ++);

	return i;
}

//***********************************************************************************************
//                                zgccmax 2013.Feb.19
//void get_min_max(QUAD *box4, COOR2 *line, int num, int *mind, int *maxd)
//param:    QUAD *box4				障碍物栅格四个点
//			COOR2 *line				一条边
//			int num					描述边的点个数
//			int *mind				障碍物点到边最短的距离
//			int *maxd				障碍物点到边最长的距离
//			int car_len				车长
//return:   void
//discribe: 给一个障碍物四点描述和一条边，求取障碍物到边的最短距离和最长距离
//***********************************************************************************************
void get_min_max(QUAD *box4, COOR2 *line, int num, int *mind, int *maxd, int car_len)
{
	int i;
	int x0;
	int dist[4], d1, d2;
	QUAD l_box;
	int minx, maxx, miny, maxy;

	if (line == NULL || num <= 1)
		return;

	memcpy(&l_box, box4, sizeof(QUAD));

	get_mid_x(&l_box, &minx, &maxx, NULL);
	get_mid_y(&l_box, &miny, &maxy, NULL);
	miny -= car_len / 2;
	maxy += car_len / 2;

	l_box.coor2[0].x = minx;
	l_box.coor2[0].y = miny;
	l_box.coor2[1].x = maxx;
	l_box.coor2[1].y = miny;
	l_box.coor2[2].x = maxx;
	l_box.coor2[2].y = maxy;
	l_box.coor2[3].x = minx;
	l_box.coor2[3].y = maxy;

	for (i = 0; i < 4; i ++)
	{
		get_x_coord(l_box.coor2[i].y, line, num, &x0);
		dist[i] = l_box.coor2[i].x - x0;
	}

	d1 = dist[0];
	d2 = dist[0];

	for (i = 1; i < 4; i ++)
	{
		if (d1 > dist[i])
		{
			d1 = dist[i];
		}

		if (d2 < dist[i])
		{
			d2 = dist[i];
		}
	}

	if (mind != NULL)
		*mind = d1;

	if (maxd != NULL)
		*maxd = d2;
}

//**********************************************************************************************
//                                smiton 2010.JUN.15
//double Cal3OrderDeterminant(const double *d)
//param:    const double *d 3阶矩阵数据指针
//return:   3阶矩阵的值
//discribe: 计算3阶矩阵的值
//***********************************************************************************************
double Cal3OrderDeterminant(const double *d)
{
	double result = 0;
	result = d[0] * d[4] * d[8] + d[1] * d[5] * d[6] + d[2] * d[3] * d[7] \
		- d[0] * d[5] * d[7] - d[1] * d[3] * d[8] - d[2] * d[4] * d[6];
	return result;
}

//***********************************************************************************************
//                                smiton 2010.JUN.15
// void ExchangeValue(double &value1, double &value2)
//param:    double &value1 交换的数据1
//param:    double &value2 交换的数据2
//return:   NULL
//discribe: 将两个变量的值互换
//***********************************************************************************************
void ExchangeValue(double &value1, double &value2)
{
	double tmp;
	tmp = value1;
	value1 = value2;
	value2 = tmp;
}
//a*y^2+b*y+c=x;

void LeastSquares(COOR2 *point, int point_num, double &a, double &b, double &c)
{
	a = 0;
	b = 0;
	c = 0;
	if (point_num < 3)
		return;

	int i = 0;
	double x_coor, y_coor;		//以米算
	double tmp_res[3] = { 0 };
	double tmp_para[9] = { 0 };
	double square, cubic, biquadratic;
	double delta, delta1, delta2, delta3;

	//evaluate determinant
	for (i = 0; i < point_num; i++)
	{
		x_coor = point[i].y / 100.0;
		y_coor = point[i].x / 100.0;
		square = x_coor*x_coor;
		cubic = square*x_coor;
		biquadratic = cubic*x_coor;
		tmp_para[0] += biquadratic;
		tmp_para[1] += cubic;
		tmp_para[2] += square;
		tmp_para[4] += square;
		tmp_para[5] += x_coor;
		tmp_res[0] += y_coor*square;
		tmp_res[1] += y_coor*x_coor;
		tmp_res[2] += y_coor;
	}
	tmp_para[3] = tmp_para[1];
	tmp_para[6] = tmp_para[2];
	tmp_para[7] = tmp_para[5];
	tmp_para[8] = point_num;

	//evaluate delta
	delta = Cal3OrderDeterminant(tmp_para);
	if (delta == 0)
		return;

	//evaluate delta1
	for (i = 0; i < 3; i++)
		ExchangeValue(tmp_para[3 * i], tmp_res[i]);
	delta1 = Cal3OrderDeterminant(tmp_para);

	//evaluate delta2
	for (i = 0; i < 3; i++)
	{
		ExchangeValue(tmp_para[3 * i + 1], tmp_para[3 * i]);
		ExchangeValue(tmp_para[3 * i], tmp_res[i]);
	}
	delta2 = Cal3OrderDeterminant(tmp_para);

	//evaluate delta2
	for (i = 0; i < 3; i++)
	{
		ExchangeValue(tmp_para[3 * i + 2], tmp_para[3 * i + 1]);
		ExchangeValue(tmp_para[3 * i + 1], tmp_res[i]);
	}
	delta3 = Cal3OrderDeterminant(tmp_para);

	a = delta1 / delta;
	b = delta2 / delta;
	c = delta3 / delta;
}


//**********************************************************************
//                          //UINT8 LineFitting(COOR2 points[], int point_num, double &vk, double &vb)
//param:  IN  COOR2	   points		需要进行直线拟合的坐标点序列
//		  IN  int	   point_num	points中坐标的个数
//		  OUT double   vk
//		  OUT double   vb			直线拟合后的直线方程vk*x+vb*y+1=0
//return:   void
//discribe: 最小一乘算法拟合方法，根据所有数据点到直线的距离差的绝对
//值之和最小找出最优拟合直线方程
//**********************************************************************
/*
int LineFitting(COOR2 points[], int point_num, double &vk, double &vb)
{
	UINT8 flag = 0;
	int i = 0;
	int index = 0;
	double a, b, c;
	double radian;
	double min = -1;
	double avg_x = 0, avg_y = 0;
	double total_x = 0, total_y = 0;

	if (point_num == 0)
	{
		a = 0;
		b = 0;
		return flag;
	}

	for (i = 0; i < point_num; i++)
	{
		total_x += points[i].x;
		total_y += points[i].y;
	}

	avg_x = Roundf2i(total_x / point_num);
	avg_y = Roundf2i(total_y / point_num);

	double direction[180];
	memset(direction, 0, sizeof(double) * 180);

	//从0到180度遍历，看哪个方向上各点到该直线的距离和最小
	int theta = 0;
	for (; theta < 180; theta++)
	{
		radian = (double)(theta*PI) / 180;
		c = avg_y - tan(radian)*avg_x;
		a = tan(radian) / c;
		b = -1.0 / c;
		for (i = 0; i < point_num; i++)
		{
			direction[theta] += DistPointToLine(points[i], a, b);
		}
	}


	for (theta = 0; theta < 180; theta++)
	{
		if (min > direction[theta])
		{
			min = direction[theta];
			index = theta;
		}
	}


	if (radian == 90)
	{
		b = 0;
		if (avg_x == 0)
		{
			a = 0;
		}
		else
		{
			a = -1 / avg_x;
		}
	}
	else
	{
		radian = (double)(index*PI) / 180;
		c = avg_y - tan(radian)*avg_x;
		if (c == 0)
		{
			vk = 0;
			vb = 0;
		}
		else
		{
			vk = tan(radian) / c;
			vb = -1.0 / c;
		}
	}

	flag = 1;

	return flag;
}
*/

int PcaProcess(COOR2 *point, int point_num, double &k, double &b)
{
	int ret = 0;
	if (NULL == point || point_num < 1)
		return ret;

	int i, j, kk;
	double avg[2];
	double point_avg[2][NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	double sum_off[2];
	double dis[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	double xfc[2][2];
	double cs[2][2];
	double xsum = 0;
	double ysum = 0;
	double sum = 0;
	double lambda1 = 0;
	double lambda2 = 0;
	double lambda = 0;
	double x11 = 0, x12 = 0;
	double x21 = 0, x22 = 0;

	memset(avg, 0, sizeof(double) * 2);
	memset(sum_off, 0, sizeof(double) * 2);
	memset(point_avg, 0, sizeof(double) * 2 * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memset(dis, 0, sizeof(double) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	memset(xfc, 0, sizeof(double) * 2 * 2);
	memset(cs, 0, sizeof(double) * 2 * 2);

	for (i = 0; i < point_num; i++)
	{
		xsum += point[i].x;
		ysum += point[i].y;
	}

	avg[0] = xsum / point_num;
	avg[1] = ysum / point_num;

	for (i = 0; i < point_num; i++)
	{
		point_avg[0][i] = avg[0] - point[i].x;
		sum_off[0] += fabs(point_avg[0][i]);
		point_avg[1][i] = avg[1] - point[i].y;
		sum_off[1] += fabs(point_avg[1][i]);
	}

	//[协方差矩阵]
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 2; j++)
		{
			sum = 0;
			for (kk = 0; kk < point_num; kk++)
			{
				sum += point_avg[i][kk] * point_avg[j][kk];
			}
			xfc[i][j] = sum / point_num;
		}
	}
	double temp1 = 0;
	double temp2 = 0;
	//[求二阶矩阵特征值]
	temp1 = pow((xfc[0][0] + xfc[1][1]), 2);
	temp2 = sqrt(temp1 - 4 * (xfc[0][0] * xfc[1][1] - xfc[0][1] * xfc[1][0]));
	lambda1 = (xfc[0][0] + xfc[1][1] + temp2) / 2;
	lambda2 = (xfc[0][0] + xfc[1][1] - temp2) / 2;

	if (lambda1 >= lambda2)
		lambda = lambda2;
	else
		lambda = lambda1;

	x11 = xfc[0][0] - lambda;
	x12 = xfc[0][1];
	x21 = xfc[1][0];
	x22 = xfc[1][1] - lambda;

	if (x11 < 0.000001 || x21 <= 0.000001)
	{//[垂直x轴的直线]
		k = 0;
		b = avg[0];
		ret = -1;
		return ret;
	}
	else
	{
		k = x12 / x11;
		b = avg[1] - k * avg[0];
		return ret;
	}
}

//[获得道路趋向]
//[mode 1  左拐  2  右拐  3  Uturn]
double get_road_direction(PL_FUNC_INPUT *pl_input, int mode, double del_theta)
{
	int i, j;
	double x1, y1;
	double x2, y2;
	int num = 0;
	int points_num = 0;
	x1 = y1 = x2 = y2 = 0;
	double theta = 0;
	//	COOR2 grid_center;
	//	grid_center.x = pl_input->fu_pl_data.gridmap.center.x;
	//	grid_center.y = pl_input->fu_pl_data.gridmap.center.y;
	// 	COOR2 pt1;
	// 	COOR2 pt2;

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
	}
	else
	{//[没有任何道路信息，投影找寻]

		int grid_map[GRID_HEIGHT][GRID_WIDTH];
		memset(grid_map, 0, GRID_HEIGHT * GRID_WIDTH * sizeof(int));
		for (i = 0; i < GRID_HEIGHT; i++)
		{
			for (j = 0; j < GRID_WIDTH; j++)
			{
				if (g_closed_grid_map[i][j] > 0)
				{
					grid_map[i][j] = 1;
				}
			}
		}

		{
			theta = 0;
		}

	}
	return theta;
}

/*
;===================================================================
;A* Pathfinder (Version 1.71a) by Patrick Lester. Used without permission.
:关于二叉堆的说明http://www.policyalmanac.org/games/binaryHeaps.htm
;===================================================================
;Last updated 06/16/03 -- Visual C++ version
*/

//Declare constants
const int mapWidth = GRID_WIDTH;
const int mapHeight = GRID_HEIGHT;
int onClosedList = 10;
const int notfinished = 0;
const int notStarted = 0;// path-related constants
const int found = 1;
const int nonexistent = 2;
const int walkable = 0;
const int unwalkable = 1;// walkability array constants

//Create needed arrays
char walkability[mapHeight][mapWidth];
int openList[mapWidth*mapHeight + 2]; //1 dimensional array holding ID# of open list items
int whichList[mapHeight + 1][mapWidth + 1];  //2 dimensional array used to record 
// 		whether a cell is on the open list or on the closed list.
int openX[mapWidth*mapHeight + 2]; //1d array stores the x location of an item on the open list
int openY[mapWidth*mapHeight + 2]; //1d array stores the y location of an item on the open list
int parentX[mapHeight + 1][mapWidth + 1]; //2d array to store parent of each cell (x)
int parentY[mapHeight + 1][mapWidth + 1]; //2d array to store parent of each cell (y)
int Fcost[mapWidth*mapHeight + 2];	//1d array to store F cost of a cell on the open list
int Gcost[mapHeight + 1][mapWidth + 1]; 	//2d array to store G cost for each cell.
int Hcost[mapWidth*mapHeight + 2];	//1d array to store H cost of a cell on the open list

//-----------------------------------------------------------------------------
// Name: FindPath
// Desc: Finds a path using A*
//-----------------------------------------------------------------------------
int * AStarFindPath(int startingX, int startingY, int targetX, int targetY, int grid[][GRID_WIDTH], COOR2 grid_center, int &pathLength)
{
	int onOpenList = 0, parentXval = 0, parentYval = 0,
		a = 0, b = 0, m = 0, u = 0, v = 0, temp = 0, corner = 0, numberOfOpenListItems = 0,
		addedGCost = 0, tempGcost = 0, path = 0,
		tempx, pathX, pathY, cellPosition,
		newOpenListItemID = 0;

	int *pathBank = NULL;
	int i, j;
	for (i = 0; i < GRID_HEIGHT; i++)
	{
		for (j = 0; j < GRID_WIDTH; j++)
		{
			if (grid[i][j] > 0)
			{
				walkability[i][j] = 1;
			}
			else
			{
				walkability[i][j] = 0;
			}
		}
	}

	//[左右以及下方封闭起来，一定程度保证后轴中心轨迹角度不过大]
	for (i = grid_center.y; i <= grid_center.y; i++)
		walkability[i][grid_center.x - 4] = 1;
	for (i = grid_center.y; i <= grid_center.y; i++)
		walkability[i][grid_center.x + 4] = 1;
	for (i = grid_center.x - 4; i <= grid_center.x + 4; i++)
		walkability[grid_center.y][i] = 1;

	//1. Convert location data (in pixels) to coordinates in the walkability array.
	int startX = startingX;
	int startY = startingY;
	targetX = targetX;
	targetY = targetY;

	//2.Quick Path Checks: Under the some circumstances no path needs to
	//	be generated ...

	//	If starting location and target are in the same location...
	if (startX == targetX && startY == targetY)
		return pathBank;

	//	If target square is unwalkable, return that it's a nonexistent path.
	if (walkability[targetY][targetX] == unwalkable)
		goto noPath;

	//3.Reset some variables that need to be cleared
	if (onClosedList > 1000000) //reset whichList occasionally
	{
		for (int x = 0; x < mapWidth; x++) {
			for (int y = 0; y < mapHeight; y++)
				whichList[y][x] = 0;
		}
		onClosedList = 10;
	}
	onClosedList = onClosedList + 2; //changing the values of onOpenList and onClosed list is faster than redimming whichList() array
	onOpenList = onClosedList - 1;
	Gcost[startY][startX] = 0; //reset starting square's G value to 0

	//4.Add the starting location to the open list of squares to be checked.
	numberOfOpenListItems = 1;
	openList[1] = 1;//assign it as the top (and currently only) item in the open list, which is maintained as a binary heap (explained below)
	openX[1] = startX; openY[1] = startY;

	//5.Do the following until a path is found or deemed nonexistent.
	do
	{

		//6.If the open list is not empty, take the first cell off of the list.
		//	This is the lowest F cost cell on the open list.
		if (numberOfOpenListItems != 0)
		{

			//7. Pop the first item off the open list.
			parentXval = openX[openList[1]];
			parentYval = openY[openList[1]]; //record cell coordinates of the item


			whichList[parentYval][parentXval] = onClosedList;//add the item to the closed list

			//	Open List = Binary Heap: Delete this item from the open list, which
			//  is maintained as a binary heap. For more information on binary heaps, see:
			//	http://www.policyalmanac.org/games/binaryHeaps.htm
			numberOfOpenListItems = numberOfOpenListItems - 1;//reduce number of open list items by 1	

			//[从二叉堆里面删除Fcost最小的元素，该元素位于二叉堆的堆顶]
			//	Delete the top item in binary heap and reorder the heap, with the lowest F cost item rising to the top.
			openList[1] = openList[numberOfOpenListItems + 1];//move the last item in the heap up to slot #1
			v = 1;

			//	Repeat the following until the new item in slot #1 sinks to its proper spot in the heap.
			do
			{
				u = v;
				if (2 * u + 1 <= numberOfOpenListItems) //if both children exist
				{
					//Check if the F cost of the parent is greater than each child.
					//Select the lowest of the two children.
					if (Fcost[openList[u]] >= Fcost[openList[2 * u]])
						v = 2 * u;
					if (Fcost[openList[v]] >= Fcost[openList[2 * u + 1]])
						v = 2 * u + 1;
				}
				else
				{
					if (2 * u <= numberOfOpenListItems) //if only child #1 exists
					{
						//Check if the F cost of the parent is greater than child #1	
						if (Fcost[openList[u]] >= Fcost[openList[2 * u]])
							v = 2 * u;
					}
				}

				if (u != v) //if parent's F is > one of its children, swap them
				{
					temp = openList[u];
					openList[u] = openList[v];
					openList[v] = temp;
				}
				else
					break; //otherwise, exit loop

			} while (1);//reorder the binary heap


			//7.Check the adjacent squares. (Its "children" -- these path children
			//	are similar, conceptually, to the binary heap children mentioned
			//	above, but don't confuse them. They are different. Path children
			//	are portrayed in Demo 1 with grey pointers pointing toward
			//	their parents.) Add these adjacent child squares to the open list
			//	for later consideration if appropriate (see various if statements
			//	below).
			for (b = parentYval - 1; b <= parentYval + 1; b++){
				for (a = parentXval - 1; a <= parentXval + 1; a++){
					//[让搜索保持向y方向]
					if (b == parentYval - 1)
					{
						continue;
					}
					if ((b == parentYval && a == parentXval - 1) || (b == parentYval && a == parentXval + 1))
					{
						continue;
					}

					if (b < startingY)
						continue;

					//[防止从车体侧方搜索出去]
					//if (b < grid_center.y + 1)// && (a < (grid_center.x - 14) || a > (grid_center.x + 13)))
					//	continue;

					//	If not off the map (do this first to avoid array out-of-bounds errors)
					if (a != -1 && b != -1 && a != mapWidth && b != mapHeight){

						//	If not already on the closed list (items on the closed list have
						//	already been considered and can now be ignored).			
						if (whichList[b][a] != onClosedList) {

							//	If not a wall/obstacle square.
							if (walkability[b][a] != unwalkable) {

								//	Don't cut across corners
								corner = walkable;
								/*
								if (a == parentXval-1)
								{
								if (b == parentYval-1)
								{
								if (walkability[parentXval-1][parentYval] == unwalkable
								|| walkability[parentXval][parentYval-1] == unwalkable) \
								corner = unwalkable;
								}
								else if (b == parentYval+1)
								{
								if (walkability[parentXval][parentYval+1] == unwalkable
								|| walkability[parentXval-1][parentYval] == unwalkable)
								corner = unwalkable;
								}
								}
								else if (a == parentXval+1)
								{
								if (b == parentYval-1)
								{
								if (walkability[parentXval][parentYval-1] == unwalkable
								|| walkability[parentXval+1][parentYval] == unwalkable)
								corner = unwalkable;
								}
								else if (b == parentYval+1)
								{
								if (walkability[parentXval+1][parentYval] == unwalkable
								|| walkability[parentXval][parentYval+1] == unwalkable)
								corner = unwalkable;
								}
								}
								*/
								if (corner == walkable) {

									//	If not already on the open list, add it to the open list.			
									if (whichList[b][a] != onOpenList)
									{

										//[往二叉堆中增加新的数据]
										//Create a new open list item in the binary heap.
										newOpenListItemID = newOpenListItemID + 1; //each new item has a unique ID #
										m = numberOfOpenListItems + 1;
										openList[m] = newOpenListItemID;//place the new open list item (actually, its ID#) at the bottom of the heap
										openX[newOpenListItemID] = a;
										openY[newOpenListItemID] = b;//record the x and y coordinates of the new item

										//Figure out its G cost
										if (abs(a - parentXval) == 1 && abs(b - parentYval) == 1)
											addedGCost = 14;//cost of going to diagonal squares	
										else
											addedGCost = 10;//cost of going to non-diagonal squares				
										Gcost[b][a] = Gcost[parentYval][parentXval] + addedGCost;

										//Figure out its H and F costs and parent
										Hcost[openList[m]] = 10 * (abs(a - targetX) + abs(b - targetY));
										Fcost[openList[m]] = Gcost[b][a] + Hcost[openList[m]];
										parentX[b][a] = parentXval; parentY[b][a] = parentYval;

										//Move the new open list item to the proper place in the binary heap.
										//Starting at the bottom, successively compare to parent items,
										//swapping as needed until the item finds its place in the heap
										//or bubbles all the way to the top (if it has the lowest F cost).
										while (m != 1) //While item hasn't bubbled to the top (m=1)	
										{
											//Check if child's F cost is < parent's F cost. If so, swap them.	
											if (Fcost[openList[m]] <= Fcost[openList[m / 2]])
											{
												temp = openList[m / 2];
												openList[m / 2] = openList[m];
												openList[m] = temp;
												m = m / 2;
											}
											else
												break;
										}
										numberOfOpenListItems = numberOfOpenListItems + 1;//add one to the number of items in the heap

										//Change whichList to show that the new item is on the open list.
										whichList[b][a] = onOpenList;
									}

									//8.If adjacent cell is already on the open list, check to see if this 
									//	path to that cell from the starting location is a better one. 
									//	If so, change the parent of the cell and its G and F costs.	
									else //If whichList(a,b) = onOpenList
									{

										//Figure out the G cost of this possible new path
										if (abs(a - parentXval) == 1 && abs(b - parentYval) == 1)
											addedGCost = 14;//cost of going to diagonal tiles	
										else
											addedGCost = 10;//cost of going to non-diagonal tiles				
										tempGcost = Gcost[parentYval][parentXval] + addedGCost;

										//If this path is shorter (G cost is lower) then change
										//the parent cell, G cost and F cost. 		
										if (tempGcost < Gcost[b][a]) //if G cost is less,
										{
											parentX[b][a] = parentXval; //change the square's parent
											parentY[b][a] = parentYval;
											Gcost[b][a] = tempGcost;//change the G cost

											//Because changing the G cost also changes the F cost, if
											//the item is on the open list we need to change the item's
											//recorded F cost and its position on the open list to make
											//sure that we maintain a properly ordered open list.
											for (int x = 1; x <= numberOfOpenListItems; x++) //look for the item in the heap
											{
												if (openX[openList[x]] == a && openY[openList[x]] == b) //item found
												{
													Fcost[openList[x]] = Gcost[b][a] + Hcost[openList[x]];//change the F cost

													//See if changing the F score bubbles the item up from it's current location in the heap
													m = x;
													while (m != 1) //While item hasn't bubbled to the top (m=1)	
													{
														//Check if child is < parent. If so, swap them.	
														if (Fcost[openList[m]] < Fcost[openList[m / 2]])
														{
															temp = openList[m / 2];
															openList[m / 2] = openList[m];
															openList[m] = temp;
															m = m / 2;
														}
														else
															break;
													}
													break; //exit for x = loop
												} //If openX(openList(x)) = a
											} //For x = 1 To numberOfOpenListItems
										}//If tempGcost < Gcost(a,b)

									}//else If whichList(a,b) = onOpenList	
								}//If not cutting a corner
							}//If not a wall/obstacle square.
						}//If not already on the closed list 
					}//If not off the map
				}//for (a = parentXval-1; a <= parentXval+1; a++){
			}//for (b = parentYval-1; b <= parentYval+1; b++){

		}//if (numberOfOpenListItems != 0)

		//9.If open list is empty then there is no path.	
		else
		{
			path = nonexistent; break;
		}

		//[若出了搜索区域那么停止搜索]
		if ((parentYval > grid_center.y + 84) || (parentXval > grid_center.x + 24) || (parentXval < grid_center.x - 24))
		{
			path = found;
			targetX = parentXval;
			targetY = parentYval;
			break;
		}
		else if (parentYval < startingY)//[防止从车头后面寻路]
		{
			goto noPath;
		}

		//If target is added to open list then path has been found.
		if (whichList[targetY][targetX] == onOpenList)
		{
			path = found; break;
		}

	} while (1);//Do until path is found or deemed nonexistent



	if (path == found)
	{

		//a.Working backwards from the target to the starting location by checking
		//	each cell's parent, figure out the length of the path.
		pathLength = 0;
		pathX = targetX; pathY = targetY;
		do
		{
			//Look up the parent of the current cell.	
			tempx = parentX[pathY][pathX];
			pathY = parentY[pathY][pathX];
			pathX = tempx;

			//Figure out the path length
			pathLength++;
		} while (pathX != startX || pathY != startY);

		//b.Resize the data bank to the right size in bytes
		pathBank = (int*)realloc(pathBank, pathLength*sizeof(int) * 2);

		//c. Now copy the path information over to the databank. Since we are
		//	working backwards from the target to the start location, we copy
		//	the information to the data bank in reverse order. The result is
		//	a properly ordered set of path data, from the first step to the
		//	last.
		pathX = targetX; pathY = targetY;
		cellPosition = pathLength * 2;//start at the end	
		do
		{
			cellPosition = cellPosition - 2;//work backwards 2 integers
			pathBank[cellPosition] = pathY;
			pathBank[cellPosition + 1] = pathX;

			//d.Look up the parent of the current cell.	
			tempx = parentX[pathY][pathX];
			pathY = parentY[pathY][pathX];
			pathX = tempx;

			//e.If we have reached the starting square, exit the loop.	
		} while (pathX != startX || pathY != startY);

		//11.Read the first path step into xPath/yPath arrays
		//ReadPath(pathfinderID,startingX,startingY,1);

	}


	return pathBank;


	//13.If there is no path to the selected target, set the pathfinder's
	//	xPath and yPath equal to its current location and return that the
	//	path is nonexistent.
noPath:
	return pathBank;
}
