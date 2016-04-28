#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <time.h>

#include <vector>
#include <algorithm>
using namespace std;

#include "./take_over.h"

#define ___TAKEOVER_DEBUG 0

#if ___TAKEOVER_DEBUG

#define ___TAKEOVER_DEBUG_PRINTGM 0

//static int file_idx = 0;
//static char file_title_buff[260] = {0};
FILE* g_fp = NULL;

#endif


#define MAX2DPOINTS NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE
static const int GRIDMAP_CELL_SCALE = 25;
static const double HD_GRIDMAP_CELL_SCALE = 12.5;
static const int LINE_STEP = 200;
static const int EXTRA_PTS = 2;
//static const int GRID_HEIGHT = ( LEN_VER ) << ( POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER );
//static const int GRID_WIDTH = ( LEN_HOR ) << ( POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR );
static const int VEHICLE_GRIDMAP_X = 80;
static const int VEHICLE_GRIDMAP_Y = 80;
static const int HD_VEHICLE_GRIDMAP_X = 120;
static const int HD_VEHICLE_GRIDMAP_Y = 0;
static const int PLAN_LEN = LINE_STEP * ( MAX2DPOINTS - 1 + EXTRA_PTS) / GRIDMAP_CELL_SCALE;
static const int LINE_OFFSET = 50;
static const int OB_IGNOR = 150;//[可以忽略进入道路的障碍物的横向距离]
static const int FIX_LINE_IGNOR = 0;//[规划线压线时进行忽略]
static const int PATH_KERNEL = 4000;
static const int SOBS_KERNEL = 1000;
static int g_max_times = 150;




static int g_serial = 0;
static int g_gridmap[GRID_HEIGHT][GRID_WIDTH];
static int g_hd_gridmap[GRID_HEIGHT_HD][GRID_WIDTH_HD];
static DYN_OBS g_dyn_obs[MAX_DYN_AREA];
static int g_dyn_obs_num;
static MULTI_LANE g_multi_line;
static COOR2 g_plan_line[MAX2DPOINTS];

static MULTI_LANE g_cross_multi_line;

static int g_obs_cnt;
static COOR2 g_obs[PLAN_LEN * GRID_WIDTH];

static int g_is_sobs;
static int g_lasty, g_starty;
static int g_is_cross_und_adjust;


#define GET_X_FROM_2P(_Y, _P1, _P2) ((_P1).x + ((_P1).x - (_P2).x) * ((_Y) - (_P1).y) / ((_P1).y - (_P2).y))
#define X2GX(xscale) (( (xscale) + VEHICLE_GRIDMAP_X * GRIDMAP_CELL_SCALE ) / GRIDMAP_CELL_SCALE)
#define Y2GY(yscale) (( (yscale) + VEHICLE_GRIDMAP_Y * GRIDMAP_CELL_SCALE ) / GRIDMAP_CELL_SCALE)

#define GX2X(i) ( (i) * GRIDMAP_CELL_SCALE + GRIDMAP_CELL_SCALE / 2 - GRIDMAP_CELL_SCALE * VEHICLE_GRIDMAP_X )
#define GY2Y(j) ( (j) * GRIDMAP_CELL_SCALE + GRIDMAP_CELL_SCALE / 2 - GRIDMAP_CELL_SCALE * VEHICLE_GRIDMAP_Y )

#define HDGX2X(i) ( (i) * HD_GRIDMAP_CELL_SCALE + HD_GRIDMAP_CELL_SCALE / 2 - HD_GRIDMAP_CELL_SCALE * HD_VEHICLE_GRIDMAP_X )
#define HDGY2Y(j) ( (j) * HD_GRIDMAP_CELL_SCALE + HD_GRIDMAP_CELL_SCALE / 2 - HD_GRIDMAP_CELL_SCALE * HD_VEHICLE_GRIDMAP_Y )

#define SET_SOBS_SIGN(b_sign) (g_is_sobs = ((b_sign) == 0 ? 0 : 1))




static int counter = 0;

//[预瞄距离参数]
static int g_max_dist_l = 5500;//[左右转向能力不一样]
static int g_max_dist_r = 5500;
static int g_min_dist = 4000;
//[预瞄距离线性插值]
static int g_max_x = 560;//[离外侧边的最大值]
static int g_min_x = 230;//[离外侧边的最小值]
static int g_look_head_dist = 0;
static int g_cur_speed = 0;


// 求X坐标
int get_x_coord(int y, const COOR2* line, int line_num)
{
	int i, x;

	if (y <= line[0].y)
	{
		x = line[0].x;
	}
	else if (y >= line[line_num - 1].y)
	{
		x = line[line_num - 1].x;
	}
	else
	{
		for (i = 1; y > line[i].y; i++);

		x = GET_X_FROM_2P(y, line[i - 1], line[i]);
	}

	return x;
}


void cal_look_head_dist(int x, int mode)
{
	int dist = abs(x);
	if (dist >= 560)
	{
		if (mode == 0)
		{
			g_look_head_dist = g_max_dist_l;
		}
		else
		{
			g_look_head_dist = g_max_dist_r;
		}
	}
	else if (dist <= 230)
	{

		g_look_head_dist = g_min_dist;
	}
	else
	{
		if (mode == 0)
		{
			g_look_head_dist = (int)((dist - g_min_x + 0.0) / (g_max_x - g_min_x) * (g_max_dist_l - g_min_dist) + g_min_dist);
		}
		else
		{
			g_look_head_dist = (int)((dist - g_min_x + 0.0) / (g_max_x - g_min_x) * (g_max_dist_r - g_min_dist) + g_min_dist);
		}
	}
}

void init_plan_line(COOR2 * line, int line_num, int mode)
{
	int x1 = line[0].x;
	int y1 = line[0].y;
	int x2 = line[3].x;
	int y2 = line[3].y;
	int x3 = 0;
	int y3 = g_look_head_dist;

	//[1、生成远端预瞄点]
	if (mode == 0)
	{
		if (x2 == x1)
		{
			x3 = x1 + 187;
		}
		else
		{
			x3 = (int)((g_look_head_dist - y1 + 0.0) * (x2 - x1) / (y2 - y1) + x1 + 187);
		}
	}
	else
	{
		if (x2 == x1)
		{
			x3 = x1 - 187;
		}
		else
		{
			x3 = (int)((g_look_head_dist - y1 + 0.0) * (x2 - x1) / (y2 - y1) + x1 - 187);
		}
	}

	//[2、截取预瞄点]
	int cut_dist = line[line_num - 1].y;
	int x4 = 0;
	int y4 = 0;
	if (x3 == 0)
	{
		x4 = 0;
		y4 = cut_dist;
	}
	else
	{
		y4 = cut_dist;
		x4 = (int)(y4 * x3 * 1.0 / y3);
	}

	//[3、生成初始规划线]
	double step_x = 0;
	double step_y = 0;
	step_x = x4 / (MAX2DPOINTS - 1.0);
	step_y = y4 / (MAX2DPOINTS - 1.0);
	for (int i = 0;i <MAX2DPOINTS;i++)
	{
		g_plan_line[i].x = (INT32)(i * step_x);
		g_plan_line[i].y = (INT32)(i * step_y);
	}
}

void check_plan_line(COOR2* line_l, COOR2* line_r, int mode, int* ret )
{
	int i;
	int x0, x1;

	if ( mode == 0 )
	{
		for ( i = 1; i < MAX2DPOINTS; i ++ )
		{

			x0 = get_x_coord( g_plan_line[i].y, line_l, MAX2DPOINTS );
			x1 = get_x_coord( g_plan_line[i].y, line_r, MAX2DPOINTS);

			if ( x1 - x0 < 2 * OB_IGNOR && i < MAX2DPOINTS - 2 )
			{
				*ret = 1;
			}

			if (g_plan_line[i].x - x0 < OB_IGNOR)
			{
				g_plan_line[i].x = x0 + OB_IGNOR;
			}

			if (x1 - g_plan_line[i].x < OB_IGNOR)
			{
				g_plan_line[i].x = x1 - OB_IGNOR;
			}
		}
	}
	else if ( mode == 1 )
	{
		for ( i = 1; i < MAX2DPOINTS; i ++ )
		{
			x0 = get_x_coord(g_plan_line[i].y, line_r, MAX2DPOINTS);
			x1 = get_x_coord(g_plan_line[i].y, line_l, MAX2DPOINTS);

			if (x0 - x1 < 2 * OB_IGNOR && i < MAX2DPOINTS - 2)
			{
				*ret = 1;
			}

			if (g_plan_line[i].x - x1 < OB_IGNOR)
			{
				g_plan_line[i].x = x1 + OB_IGNOR;
			}

			if (x0 - g_plan_line[i].x < OB_IGNOR)
			{
				g_plan_line[i].x = x0 - OB_IGNOR;
			}
		}
	}
}

int check_the_dynobs(COOR2* line_l, COOR2* line_r)
{
	int ret = 0;
	int dist = line_l[MAX2DPOINTS - 1].y > line_r[MAX2DPOINTS - 1].y ? line_r[MAX2DPOINTS - 1].y : line_l[MAX2DPOINTS - 1].y;
	for (int i = 0; i < g_dyn_obs_num;i++)
	{
		if (g_dyn_obs[i].nearest_pt.y > dist || g_dyn_obs[i].nearest_pt.y < 450)
			continue;
		else
		{
			int xx;
			xx = get_x_coord(g_dyn_obs[i].nearest_pt.y, g_plan_line, MAX2DPOINTS);
			int x = abs(g_dyn_obs[i].nearest_pt.x - xx);

			if (x < 200)
			{
				double diff_spd = g_cur_speed - g_dyn_obs[i].speed.y;
				double time = (g_dyn_obs[i].nearest_pt.y - 400) / diff_spd;
				if ((time > 0 && time <= 3) || g_dyn_obs[i].nearest_pt.y <= 1500)
				{//[3s后碰撞]
					ret = -1;
					return ret;
				}
			}
			else
			{
				continue;
			}
		}
	}
	return ret;
}

#if ___TAKEOVER_DEBUG
void println( COOR2* l, const char* t )
{
	int i;
	fprintf( stderr, "\n%s", t );

	for ( i = 0; i < MAX2DPOINTS; i ++ )
	{
		fprintf( stderr, "(%d,%d)", l[i].x, l[i].y );
	}

	fprintf( stderr, "\n" );
}


void printgm( int g[][GRID_WIDTH] )
{
	int i, j;

	fprintf( stderr, "\nGRIDMAP_DATA\n" );

	for ( i = 0; i < GRID_HEIGHT; i ++ )
	{
		for ( j = 0; j < GRID_WIDTH; j ++ )
		{
			fprintf( stderr, "%d ", g[i][j] );
		}

		fprintf( stderr, "\n" );
	}

}
#endif




// 求X坐标
int get_x_coord1( int y, const vector< pair<int, int> >& line )
{
	int i, x;

	if ( y <= line[0].second )
	{
		x = line[0].first;
	}
	else if ( y >= line[line.size() - 1].second )
	{
		x = line[line.size() - 1].first;
	}
	else
	{
		for ( i = 1; y > line[i].second; i ++ );

		x = line[i - 1].first + ( line[i - 1].first - line[i].first ) * ( y - line[i - 1].second ) / ( line[i - 1].second - line[i].second );
	}

	return x;
}

int hd_get_x_coord1(int y, const vector< pair<int, int> >& line)
{
	int i, x;

	if (y <= line[0].second)
	{
		x = line[0].first;
	}
	else if (y >= line[line.size() - 1].second)
	{
		x = line[line.size() - 1].first;
	}
	else
	{
		for (i = 1; y > line[i].second; i++);

		x = line[i - 1].first + (line[i - 1].first - line[i].first) * (y - line[i - 1].second) / (line[i - 1].second - line[i].second);
	}

	return x;
}

// mode == 0  左  mode == 1 右   Y向规整
void fix_x_line1( COOR2* line, const vector< pair<int, int> >& edge )
{
	int i;
	int step;


	line[0].y = edge[0].second;
	line[0].x = get_x_coord1( line[0].y, edge );

	step = 2;

	for ( i = 1; i < MAX2DPOINTS; i ++ )
	{
		line[i].y = line[i - 1].y + step;
		line[i].x = get_x_coord1( line[i].y, edge );
	}
}

void hd_fix_x_line1(COOR2* line, const vector< pair<int, int> >& edge)
{
	int i;
	int step;


	line[0].y = edge[0].second;
	line[0].x = hd_get_x_coord1(line[0].y, edge);

	step = 2;

	for (i = 1; i < MAX2DPOINTS; i++)
	{
		line[i].y = line[i - 1].y + step;
		line[i].x = hd_get_x_coord1(line[i].y, edge);
	}
}

int get_vec_sum( const vector< pair<int, int> >& line )
{
	int i;
	int sum;

	if ( line.empty() )
	{
		return 0;
	}

	sum = 0;

	for ( i = 1; i < ( int )line.size(); i ++ )
	{
		if ( abs( line[i].first - line[i - 1].first ) + abs( line[i].second - line[i - 1].second ) > 4 )
		{
			sum += 12 - abs( VEHICLE_GRIDMAP_X - line[i].first ) / 2;
			sum += abs( line[i].second - line[i - 1].second );
		}
	}

	return sum;
}

int hd_get_vec_sum(const vector< pair<int, int> >& line)
{
	int i;
	int sum;

	if (line.empty())
	{
		return 0;
	}

	sum = 0;

	for (i = 1; i < (int)line.size(); i++)
	{
		if (abs(line[i].first - line[i - 1].first) + abs(line[i].second - line[i - 1].second) > 4)
		{
			sum += 12 - abs(HD_VEHICLE_GRIDMAP_X - line[i].first) / 2;
			sum += abs(line[i].second - line[i - 1].second);
		}
	}

	return sum;
}

int get_pairs( vector< pair<int, int> >& v1, vector< pair<int, int> >& v2 )
{
	int i, j;
	int sum;

	sum = 0;

	for ( i = 0; i < ( int )v1.size(); i ++ )
	{
		for ( j = 0; j < ( int )v2.size(); j ++ )
		{
			if ( v1[i].second >= v2[j].second - 3 && \
			        v1[i].second <= v2[j].second + 3 && \
			        v1[i].first <= v2[j].first - 10 && \
			        v1[i].first >= v2[j].first - 22 )
			{
				sum ++;
				break;
			}
		}
	}


	for ( i = 0; i < ( int )v2.size(); i ++ )
	{
		for ( j = 0; j < ( int )v1.size(); j ++ )
		{
			if ( v2[i].second >= v1[j].second - 3 && \
			        v2[i].second <= v1[j].second + 3 && \
			        v2[i].first - 22 <= v1[j].first && \
			        v2[i].first - 10 >= v1[j].first )
			{
				sum ++;
				break;
			}
		}
	}

	return sum;
}

int fix_s_plan( int gridmap[][GRID_WIDTH], const vector< pair<int, int> >& left_l, \
                const vector< pair<int, int> >& right_l, \
                const vector< pair<int, int> >& pair_lr, \
                COOR2* plan_line, int* line_num )
{
	int i, j, k;
	int cnt, sum, maxy;

	vector< pair<int, int> > l_tl, l_tr;

	// 边
	cnt = ( int )pair_lr.size();

	for ( i = 0; i < cnt; i ++ )
	{
		l_tl.push_back( left_l[pair_lr[i].first] );
		l_tr.push_back( right_l[pair_lr[i].second] );
		/*l_tl[i].first = GX2X(l_tl[i].first);
		l_tl[i].second = GY2Y(l_tl[i].second);
		l_tr[i].first = GX2X(l_tr[i].first);
		l_tr[i].second = GY2Y(l_tr[i].second);*/
	}

	maxy = l_tl[cnt - 1].second;

	if ( maxy < l_tr[cnt - 1].second )
	{
		maxy = l_tr[cnt - 1].second;
	}

	maxy = GY2Y( maxy );

	fix_x_line1( g_cross_multi_line.lane_line[0].line, l_tl );
	fix_x_line1( g_cross_multi_line.lane_line[3].line, l_tr );

	memset( g_plan_line, 0, sizeof( g_plan_line ) );

	for ( i = 1; i < MAX2DPOINTS; i ++ )
	{
		g_plan_line[i].x = ( g_cross_multi_line.lane_line[0].line[i - 1].x + g_cross_multi_line.lane_line[3].line[i - 1].x ) / 2;
		g_plan_line[i].y = ( g_cross_multi_line.lane_line[0].line[i - 1].y + g_cross_multi_line.lane_line[3].line[i - 1].y ) / 2;
	}


	for ( i = 1; i < MAX2DPOINTS; i ++ )
	{
		sum = 0;
		cnt = 0;

		for ( j = g_plan_line[i].y - 2; j < g_plan_line[i].y + 2; j ++ )
		{
			for ( k = g_plan_line[i].x - 5; k < g_plan_line[i].x + 6; k ++ )
			{
				if ( gridmap[j][k] != 0 )
				{
					if ( k >= g_plan_line[i].x )
					{
						sum -= 6 - ( k - g_plan_line[i].x );
						cnt ++;
					}
					else
					{
						sum += 6 - ( g_plan_line[i].x - k );
						cnt ++;
					}
				}
			}
		}

		if ( cnt != 0 )
		{
			sum /= cnt;
			g_plan_line[i].x += sum;
		}
	}


	for ( i = 1; i < MAX2DPOINTS; i ++ )
	{
		g_plan_line[i].x = GX2X( g_plan_line[i].x );
		g_plan_line[i].y = GY2Y( g_plan_line[i].y );

		if ( g_plan_line[i].y >= maxy )
		{
			g_plan_line[i].x = g_plan_line[i].y = 0;
			break;
		}
	}

	*line_num = i;

	memcpy( plan_line, g_plan_line, sizeof( g_plan_line ) );

	return 0;
}

int hd_fix_s_plan(int gridmap[][GRID_WIDTH_HD], const vector< pair<int, int> >& left_l, \
	const vector< pair<int, int> >& right_l, \
	const vector< pair<int, int> >& pair_lr, \
	COOR2* plan_line, int* line_num)
{
	int i, j, k;
	int cnt, sum, maxy;

	vector< pair<int, int> > l_tl, l_tr;

	// 边
	cnt = (int)pair_lr.size();

	for (i = 0; i < cnt; i++)
	{
		l_tl.push_back(left_l[pair_lr[i].first]);
		l_tr.push_back(right_l[pair_lr[i].second]);
		/*l_tl[i].first = GX2X(l_tl[i].first);
		l_tl[i].second = GY2Y(l_tl[i].second);
		l_tr[i].first = GX2X(l_tr[i].first);
		l_tr[i].second = GY2Y(l_tr[i].second);*/
	}

	maxy = l_tl[cnt - 1].second;

	if (maxy < l_tr[cnt - 1].second)
	{
		maxy = l_tr[cnt - 1].second;
	}

	maxy = (int)HDGY2Y(maxy);

	hd_fix_x_line1(g_cross_multi_line.lane_line[0].line, l_tl);
	hd_fix_x_line1(g_cross_multi_line.lane_line[3].line, l_tr);

	memset(g_plan_line, 0, sizeof(g_plan_line));

	for (i = 1; i < MAX2DPOINTS; i++)
	{
		g_plan_line[i].x = (g_cross_multi_line.lane_line[0].line[i - 1].x + g_cross_multi_line.lane_line[3].line[i - 1].x) / 2;
		g_plan_line[i].y = (g_cross_multi_line.lane_line[0].line[i - 1].y + g_cross_multi_line.lane_line[3].line[i - 1].y) / 2;
	}


	for (i = 1; i < MAX2DPOINTS; i++)
	{
		sum = 0;
		cnt = 0;

		for (j = g_plan_line[i].y - 4; j < g_plan_line[i].y + 4; j++)
		{
			for (k = g_plan_line[i].x - 11; k < g_plan_line[i].x + 12; k++)
			{
				if (gridmap[j][k] != 8)
				{
					if (k >= g_plan_line[i].x)
					{
						sum -= 12 - (k - g_plan_line[i].x);
						cnt++;
					}
					else
					{
						sum += 12 - (g_plan_line[i].x - k);
						cnt++;
					}
				}
			}
		}

		if (cnt != 0)
		{
			sum /= cnt;
			g_plan_line[i].x += sum;
		}
	}


	for (i = 1; i < MAX2DPOINTS; i++)
	{
		g_plan_line[i].x = (INT32)HDGX2X(g_plan_line[i].x);
		g_plan_line[i].y = (INT32)HDGY2Y(g_plan_line[i].y);

		if (g_plan_line[i].y >= maxy)
		{
			g_plan_line[i].x = g_plan_line[i].y = 0;
			break;
		}
	}

	*line_num = i;

	memcpy(plan_line, g_plan_line, sizeof(g_plan_line));

	return 0;
}
// mode == 0  左  mode == 1 右   Y向规整
void fix_x_line( COOR2* line, const COOR2* edge, int num )
{
	int i;

	line[0].y = EXTRA_PTS * LINE_STEP;
	line[0].x = get_x_coord(line[0].y, edge, num);

	for ( i = 1; i < MAX2DPOINTS; i ++ )
	{
		line[i].y = line[i - 1].y + LINE_STEP;
		line[i].x = get_x_coord( line[i].y, edge, num );
	}
}

// 初步清GRIDMAP 并调整边
void gridmap_pre_process( int gridmap[][GRID_WIDTH], COOR2* line_l, COOR2* line_r, int gridmap_out[][GRID_WIDTH], int mode )
{
	int i, j;
	int x, y, x0, x1, yy;
	int sign;

	// 清上下
	if ( g_is_sobs == 1 )
	{
		g_starty = VEHICLE_GRIDMAP_Y + 10;
		memcpy( gridmap_out + g_starty, gridmap + g_starty, 38 * GRID_WIDTH * sizeof( int ) );
		g_lasty = g_starty + 38;
	}
	else
	{
		g_starty = VEHICLE_GRIDMAP_Y + 20;
		memcpy( gridmap_out + g_starty, gridmap + g_starty, ( PLAN_LEN - 24 ) * GRID_WIDTH * sizeof( int ) );
		g_lasty = g_starty + PLAN_LEN - 24;
	}

	// 规整1,0
	for ( i = g_starty; i < g_lasty; i ++ )
	{
		for ( j = 0; j < GRID_WIDTH; j ++ )
		{
			if ( gridmap_out[i][j] != 0 && gridmap_out[i][j] != 9)
			{
				gridmap_out[i][j] = 1;
			}
		}
	}


#if ___TAKEOVER_DEBUG_PRINTGM
	printgm( gridmap_out );
#endif


	// 调整边
	if (mode == 0)
	{
		sign = 0;
		for (y = line_l[0].y, i = 0; i < 80 && sign == 0; y += GRIDMAP_CELL_SCALE, i ++)
		{
			x = get_x_coord(y, line_l, MAX2DPOINTS);
			x0 = X2GX(x - LINE_OFFSET);
			x1 = X2GX(x);
			yy = Y2GY(y);

			for (j = x0; j <= x1; j ++)
			{
				if (gridmap[yy][j] == 1)
				{
					sign ++;
				}
			}
		}

		if (!sign)
		{
			for (i = 0; i < MAX2DPOINTS; i ++)
			{
				line_l[i].x -= LINE_OFFSET;
			}
		}
	}
	else if ( mode == 1 )
	{
		sign = 0;

		for (y = line_r[0].y, i = 0; i < 80 && sign == 0; y += GRIDMAP_CELL_SCALE, i ++)
		{
			x = get_x_coord(y, line_r, MAX2DPOINTS);
			x0 = X2GX(x);
			x1 = X2GX(x + LINE_OFFSET);
			yy = Y2GY(y);

			for (j = x0; j <= x1; j ++)
			{
				if (gridmap[yy][j] == 1)
				{
					sign ++;
				}
			}
		}

		if (!sign)
		{
			for (i = 0; i < MAX2DPOINTS; i ++)
			{
				line_r[i].x += LINE_OFFSET;
			}
		}
	}


	// 清左右
	for ( i = g_starty, y = GY2Y( i ); i < g_lasty; y += GRIDMAP_CELL_SCALE, i ++ )
	{
		x = X2GX( get_x_coord( y, line_l, MAX2DPOINTS ) );

		if ( x > 0 )
		{
			memset( gridmap_out + i, 0, x * sizeof( int ) );
		}

		x = X2GX( get_x_coord( y, line_r, MAX2DPOINTS ) ) + 1;

		if ( x < GRID_WIDTH )
		{
			memset( gridmap_out[i] + x, 0, ( GRID_WIDTH - x ) * sizeof( int ) );
		}
	}
}

// 初步清DYN_OBS
void dynobs_pre_process( DYN_OBS *dyn, int dyn_obs_num, \
						COOR2* line_l, COOR2* line_r, int gridmap[][GRID_WIDTH] )
{
	int i, j;
	int x0, x1, y0, y1, xx, yy;

	for ( i = 0; i < dyn_obs_num; i ++ )
	{
		x0 = x1 = dyn[i].coor2[0].x;
		y0 = y1 = dyn[i].coor2[0].y;

		for ( j = 1; j < 4; j ++ )
		{
			if ( dyn[i].coor2[j].x < x0 )
			{
				x0 = dyn[i].coor2[j].x;
			}
			else if ( dyn[i].coor2[j].x > x1 )
			{
				x1 = dyn[i].coor2[j].x;
			}

			if ( dyn[i].coor2[j].y < y0 )
			{
				y0 = dyn[i].coor2[j].y;
			}
			else if ( dyn[i].coor2[j].y > y1 )
			{
				y1 = dyn[i].coor2[j].y;
			}
		}

		// 可大 不可小
		y0 -= dyn[i].speed.y / 10;
		y1 += dyn[i].speed.y / 10;

		x0 -= 25;
		x1 += 25;

		y0 = Y2GY(y0);
		y1 = Y2GY(y1);
		x0 = X2GX(x0);
		x1 = X2GX(x1);

		x0 = x0 < 0 ? 0 : x0;
		x1 = x1 >= GRID_WIDTH ? ( GRID_WIDTH - 1 ) : x1;
		y0 = y0 < 0 ? 0 : y0;
		y1 = y1 >= GRID_HEIGHT ? ( GRID_HEIGHT - 1 ) : y1;

		if (x0 >= GRID_WIDTH - 1 || x1 <= 0 || y0 >= GRID_HEIGHT - 1 || y1 <= 0)
		{
			continue;
		}

		gridmap[y0][x0] = 1;
		gridmap[y0][x1] = 1;
		gridmap[y1][x0] = 1;
		gridmap[y1][x1] = 1;

		xx = (x0 + x1) / 2;
		yy = (y0 + y1) / 2;

		if (x1 - x0 > 5 || y1 - y0 > 5)
		{
			gridmap[yy][xx] = 1;
		}
	}
}

// GRIDMAP归边
void gridmap_to_line( int gridmap[][GRID_WIDTH], COOR2* line_l, COOR2* line_r, int mode, int is_takeover, int *ret )
{
	int i, j, cnt;
	int x, y, x0, x1, xx;
	int dist, sign, sign1;

	// 收纳障碍
	g_obs_cnt = 0;
	memset(g_obs, 0, sizeof(g_obs));

	for ( i = 0; i < GRID_WIDTH; i ++ )
	{
		for ( j = g_starty; j < g_lasty; j ++ )
		{
			if ( gridmap[j][i] == 1 )
			{
				g_obs[g_obs_cnt].x = i;
				g_obs[g_obs_cnt].y = j;
				g_obs_cnt ++;
			}
		}
	}

	// 不可逾越或无用
	for ( i = g_starty; i < g_lasty; i ++ )
	{
		y = GY2Y(i);
		x0 = get_x_coord(y, line_l, MAX2DPOINTS);
		x1 = get_x_coord(y, line_r, MAX2DPOINTS);
		xx = (x0 + x1) / 2;

		for (j = 0; j < g_obs_cnt; j ++)
		{
			if (g_obs[j].y == i)
			{
				x = GX2X(g_obs[j].x);

				if (x <= x0 + GRIDMAP_CELL_SCALE / 2 || x >= x1 - GRIDMAP_CELL_SCALE / 2)
				{
					if (j < g_obs_cnt - 1)
					{
						memcpy(g_obs + j, g_obs + j + 1, sizeof(COOR2) * (g_obs_cnt - j - 1));
					}

					g_obs_cnt --;
					j --;
					continue;
				}
				else if ( y < 1500 && \
					((mode == 0 && x - x0 >= OB_IGNOR && x <= xx - OB_IGNOR) || \
					(mode == 1 && x1 - x >= OB_IGNOR && x >= xx + OB_IGNOR)) )
				{
					if ( is_takeover == 1 )
					{
#ifdef MBUG_OPEN_
						MBUG("goto err_exit 2\n");
#endif
						goto err_exit;
					}
				}
			}
		}
	}

	// 此处障碍皆可通过
	while (g_obs_cnt)
	{
		i = 0;
		dist = 0xFFFF;
		do {
			x = GX2X(g_obs[i].x);
			y = GY2Y(g_obs[i].y);
			x0 = get_x_coord(y, line_l, MAX2DPOINTS);
			x1 = get_x_coord(y, line_r, MAX2DPOINTS);


			if ( is_takeover == 1 )
			{
				if ( mode == 0 )
				{
					x1 -= 100;
				}
				else
				{
					x0 += 100;
				}
			}

			xx = x - x0;
			sign1 = 0;

			if (xx >= x1 - x)
			{
				sign1 = 1;
				xx = x1 - x;
			}


			if (xx < dist)
			{
				sign = sign1;
				dist = xx;
				j = i;
			}
		} 
		while ( ++ i < g_obs_cnt );


		x = GX2X(g_obs[j].x);
		y = GY2Y(g_obs[j].y);

		if (y <= EXTRA_PTS * LINE_STEP)
		{
			cnt = 0;
		}
		else
		{
			cnt = y / LINE_STEP - EXTRA_PTS;
		}

		if (sign == 0)
		{
			line_l[cnt].x = x;

			if ( line_l[cnt + 1].y - y < y - line_l[cnt].y )
			{
				line_l[cnt + 1].x = x;
			}
		}
		else
		{
			line_r[cnt].x = x;

			if ( line_r[cnt + 1].y - y < y - line_r[cnt].y )
			{
				line_r[cnt + 1].x = x;
			}
		}

		if (j < g_obs_cnt - 1)
		{
			memcpy(g_obs + j, g_obs + j + 1, sizeof(COOR2) * (g_obs_cnt - j - 1));
		}

		g_obs_cnt --;
	}


	for ( i = 0; i < MAX2DPOINTS - 2; i ++ )
	{
		if ( line_l[i + 1].x <= line_l[i].x && line_l[i + 1].x < line_l[i + 2].x )
		{
			line_l[i + 1].x = GET_X_FROM_2P( line_l[i + 1].y, line_l[i], line_l[i + 2] );
		}

		if ( line_r[i + 1].x >= line_r[i].x && line_r[i + 1].x > line_r[i + 2].x )
		{
			line_r[i + 1].x = GET_X_FROM_2P( line_r[i + 1].y, line_r[i], line_r[i + 2] );
		}
	}

	return;

err_exit:
	*ret = 1;
}



void get_plan_line( COOR2* line, COOR2* line_l, COOR2* line_r, int mode, int* ret )
{
	int i;
	int x0, x1;

	line[0].x = line[0].y = 0;

	if ( mode == 0 )
	{
		for ( i = 1; i < MAX2DPOINTS; i ++ )
		{
			line[i].x = ( line_l[i - 1].x * 3 + line_r[i - 1].x ) / 4;
			line[i].y = ( line_l[i - 1].y * 3 + line_r[i - 1].y ) / 4;

			x0 = get_x_coord( line[i].y, line_l, MAX2DPOINTS );
			x1 = get_x_coord( line[i].y, line_r, MAX2DPOINTS );

			if ( x1 - x0 < 2 * OB_IGNOR && i < MAX2DPOINTS - 2 )
			{
				*ret = 1;
			}

			if ( line[i].x - x0 < OB_IGNOR )
			{
				line[i].x = x0 + OB_IGNOR;
			}
		}
	}
	else if ( mode == 1 )
	{
		for ( i = 1; i < MAX2DPOINTS; i ++ )
		{
			line[i].x = ( line_l[i - 1].x + line_r[i - 1].x * 3 ) / 4;
			line[i].y = ( line_l[i - 1].y + line_r[i - 1].y * 3 ) / 4;

			x0 = get_x_coord( line[i].y, line_r, MAX2DPOINTS );
			x1 = get_x_coord( line[i].y, line_l, MAX2DPOINTS );

			if ( x0 - x1 < 2 * OB_IGNOR && i < MAX2DPOINTS - 2 )
			{
				*ret = 1;
			}

			if ( x0 - line[i].x < OB_IGNOR )
			{
				line[i].x = x0 - OB_IGNOR;
			}
		}
	}
	else
	{
		for ( i = 1; i < MAX2DPOINTS; i ++ )
		{
			line[i].x = ( line_l[i - 1].x + line_r[i - 1].x ) / 2;
			line[i].y = ( line_l[i - 1].y + line_r[i - 1].y ) / 2;

			x0 = get_x_coord( line[i].y, line_l, MAX2DPOINTS );
			x1 = get_x_coord( line[i].y, line_r, MAX2DPOINTS );

			if ( g_is_sobs == 1 )
			{
				if ( x1 - x0 < 2 * OB_IGNOR && line[i].y < SOBS_KERNEL && i < MAX2DPOINTS - 2 )
				{
					*ret = 1;
				}
			}
			else
			{
				if ( x1 - x0 < 2 * OB_IGNOR && i < MAX2DPOINTS - 2 )
				{
					*ret = 1;
				}
			}
		}
	}
}


void fix_plan_way( COOR2* plan_line, COOR2* line_l, COOR2* line_r )
{
	int i, j, k, m;
	int xx, yy;
	int x0, x1;
	int sign;
	int cnt;
	int path_kernel;
	COOR2 tmp_mid_way[MAX2DPOINTS];


	if ( g_is_sobs == 1 )
	{
		path_kernel = SOBS_KERNEL;
	}
	else if (g_is_cross_und_adjust == 1)
	{
		path_kernel = PATH_KERNEL - 2000;
	}
	else
	{
		path_kernel = PATH_KERNEL;
	}

	k = 0;

	for ( i = 0; i < MAX2DPOINTS; i ++ )
	{
		if ( plan_line[i].y >= path_kernel )
		{
			break;
		}
	}

	m = i + 1;

	if ( m < MAX2DPOINTS )
	{
		for ( ; k < m - 2; k ++ )
		{
			for ( i = m - 1; i > k + 1; i -- )
			{
				cnt = i - k;
				xx = ( int )( ( float )( plan_line[i].x - plan_line[k].x ) / cnt + 0.5f );
				yy = ( int )( ( float )( plan_line[i].y - plan_line[k].y ) / cnt + 0.5f );

				memset( tmp_mid_way, 0, sizeof( tmp_mid_way ) );

				for ( j = k + 1; j <= i; j ++ )
				{
					tmp_mid_way[j].x = xx * ( j - k ) + plan_line[k].x;
					tmp_mid_way[j].y = yy * ( j - k ) + plan_line[k].y;
				}

				sign = 1;

				for ( j = k + 1; j < i; j ++ )
				{
					x0 = get_x_coord( tmp_mid_way[j].y, line_l, m );

					x1 = get_x_coord( tmp_mid_way[j].y, line_r, m );

					if (tmp_mid_way[j].x - x0 < FIX_LINE_IGNOR || \
						x1 - tmp_mid_way[j].x < FIX_LINE_IGNOR)
					{
						sign = 0;
						break;
					}
				}

				if ( sign == 1 )
				{
					memcpy( plan_line + k + 1, tmp_mid_way + k + 1, cnt * sizeof( COOR2 ) );
					k = i - 1;
					break;
				}
			}
		}

		k = m;
	}

	for ( ; k < MAX2DPOINTS - 2; k ++ )
	{
		for ( i = MAX2DPOINTS - 1; i > k + 1; i -- )
		{
			cnt = i - k;
			xx = ( int )( ( float )( plan_line[i].x - plan_line[k].x ) / cnt + 0.5f );
			yy = ( int )( ( float )( plan_line[i].y - plan_line[k].y ) / cnt + 0.5f );

			memset( tmp_mid_way, 0, sizeof( tmp_mid_way ) );

			for ( j = k + 1; j <= i; j ++ )
			{
				tmp_mid_way[j].x = xx * ( j - k ) + plan_line[k].x;
				tmp_mid_way[j].y = yy * ( j - k ) + plan_line[k].y;
			}

			sign = 1;

			for ( j = k + 1; j < i; j ++ )
			{
				x0 = get_x_coord( tmp_mid_way[j].y, line_l, MAX2DPOINTS );
				x1 = get_x_coord( tmp_mid_way[j].y, line_r, MAX2DPOINTS );

				if (tmp_mid_way[j].x - x0 < FIX_LINE_IGNOR || \
					x1 - tmp_mid_way[j].x < FIX_LINE_IGNOR)
				{
					sign = 0;
					break;
				}
			}

			if ( sign == 1 )
			{
				memcpy( plan_line + k + 1, tmp_mid_way + k + 1, cnt * sizeof( COOR2 ) );
				k = i - 1;
				break;
			}
		}
	}
}


/*==================================================================
 * 函数名  ：	int take_over( int gridmap[][GRID_WIDTH], DYN_OBS* dyn_obs, int dyn_obs_num, MULTI_LANE* multi_lane, int mode, COOR2* plan_line, int* status )
 * 功能    ：	进行超车规划。
 * 输入参数：	int gridmap[][GRID_WIDTH]		栅格地图
				DYN_OBS* dyn_obs				动态障碍物
				int dyn_obs_num					动态障碍物数目
				MULTI_LANE* multi_lane			多车道描述
				int mode						0  左超车  1  右超车
				COOR2* plan_line				输出规划路径
				int* status						超车规划内部状态输出

 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	杜鹏桢
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int take_over(int cur_spd, int gridmap[][GRID_WIDTH], DYN_OBS* dyn_obs, int dyn_obs_num, MULTI_LANE* multi_lane, int mode, COOR2* plan_line, int* status)
{
	int i, j;
	int isok;
	int idxl, idxr;
	int ret = 0;

	assert( gridmap != NULL );
	assert( dyn_obs != NULL );
	assert( dyn_obs_num > -1 );
	assert( multi_lane != NULL );

	g_is_cross_und_adjust = 0;
	g_cur_speed = cur_spd;
#if ___TAKEOVER_DEBUG

	if ( g_serial == 0 )
	{
		//		memset( file_title_buff, 0, sizeof( file_title_buff ) );
		//		sprintf( file_title_buff, "take_over_debug_%04d.txt", file_idx ++ );
		//		g_fp = fopen( file_title_buff, "wt" );
	}

#endif


	g_serial ++;

	fprintf( stderr, "\n\n\nid == [%d]\n", g_serial );


	// 动态障碍已经消失


	isok = 0;

	// 边处理
	{
		memset( &g_multi_line, 0, sizeof( MULTI_LANE ) );

		idxl = idxr = -1;

		for ( i = 0; i < LANE_LINE_NUM; i ++ )
		{
			if ( multi_lane->ok_times[i] >= 1 )
			{
				idxl = i;
				break;
			}
			// 			if ( multi_lane->lane_line[i].valid_num_points >= 2 )
			// 			{
			// 				idxl = i;
			// 				break;
			// 			}
		}

		for ( i = LANE_LINE_NUM - 1; i > -1; i -- )
		{
			if ( multi_lane->ok_times[i] >= 1 )
			{
				idxr = i;
				break;
			}
			// 			if ( multi_lane->lane_line[i].valid_num_points >= 2 )
			// 			{
			// 				idxr = i;
			// 				break;
			// 			}
		}

		if ( idxl == -1 || idxr == -1 || idxl >= idxr )
		{
#ifdef MBUG_OPEN_
			MBUG("goto err_exit 1\n");
#endif
			goto err_exit;
		}

		// 左
		fix_x_line( g_multi_line.lane_line[idxl].line, multi_lane->lane_line[idxl].line, \
			multi_lane->lane_line[idxl].valid_num_points);
		g_multi_line.lane_line[idxl].valid_num_points = MAX2DPOINTS;

		// 右
		fix_x_line( g_multi_line.lane_line[idxr].line, multi_lane->lane_line[idxr].line, \
			multi_lane->lane_line[idxr].valid_num_points);
		g_multi_line.lane_line[idxr].valid_num_points = MAX2DPOINTS;
	}

	//[线性插值计算预瞄距离]
	if (mode == 0)
	{
		cal_look_head_dist(g_multi_line.lane_line[idxl].line[0].x, mode);
	}
	else
	{
		cal_look_head_dist(g_multi_line.lane_line[idxr].line[0].x, mode);
	}
	//g_look_head_dist = 8000;

	//[初始生成规划线]
	if (mode == 0)
	{
		init_plan_line(g_multi_line.lane_line[idxl].line, MAX2DPOINTS, mode);
	}
	else
	{
		init_plan_line(g_multi_line.lane_line[idxr].line, MAX2DPOINTS, mode);
	}

// 	g_serial++;
// 	if (g_serial == g_max_times)
// 	{
// 		g_serial = 0;
// 		*status = TRACE_LANE;
// 	}
// 	else
// 	{
// 		if (mode == 0)
// 		{
// 			*status = TAKEOVER_FROM_LEFT;
// 		}
// 		else
// 		{
// 			*status = TAKEOVER_FROM_RIGHT;
// 		}
// 	}
// 
// 	memcpy(plan_line, g_plan_line, sizeof(g_plan_line));
// 	return 0;
	






#if ___TAKEOVER_DEBUG
	println( g_multi_line.lane_line[idxl].line, "LINE_DATA0" );
	println( g_multi_line.lane_line[idxr].line, "LINE_DATA1" );
#endif



	if (dyn_obs_num < 1)
	{

		if (mode == 0)
		{
			if (abs(g_multi_line.lane_line[idxl].line[0].x) < 200)
			{
				counter++;

				if (counter == 10)
				{
					g_serial = g_max_times;
					counter = 0;
				}
			}


			//[确保道路安全]
			for (i=60; i<160; i++)
			{
				for (j=84; j< 100; j++)
				{
					if (gridmap[i][j] > 0)
					{
						if (g_serial == g_max_times)
						{
							g_serial--;
						}
						
						counter = 0;
					}
				}
			}
		}
		else
		{
			if (abs(g_multi_line.lane_line[idxr].line[0].x) < 200)
			{
				counter++;

				if (counter == 10)
				{
					g_serial = g_max_times;
					counter = 0;
				}
			}

			//[确保道路安全]
			for (i=60; i<160; i++)
			{
				for (j=60; j< 76; j++)
				{
					if (gridmap[i][j] > 0)
					{
						if (g_serial == g_max_times)
						{
							g_serial--;
						}

						counter = 0;
					}
				}
			}
		}



		//g_serial = g_max_times;
	}
	else
	{
		counter = 0;
	}

	// 处理GRIDMAP
	{
		memset( g_gridmap, 0, sizeof( g_gridmap ) );

		// 预处理并COPY,并处理边
		gridmap_pre_process( gridmap, \
			g_multi_line.lane_line[idxl].line, \
			g_multi_line.lane_line[idxr].line, \
			g_gridmap, \
			mode);


		// 处理DYN 划入GRIDMAP
		{
			
			memcpy( g_dyn_obs, dyn_obs, sizeof( DYN_OBS ) * dyn_obs_num );
			g_dyn_obs_num = dyn_obs_num;
			/*
			dynobs_pre_process( g_dyn_obs, \
				g_dyn_obs_num, \
				g_multi_line.lane_line[idxl].line, \
				g_multi_line.lane_line[idxr].line, \
				g_gridmap );

				*/
		}


		// 和边一起处理掉
		gridmap_to_line( g_gridmap, \
			g_multi_line.lane_line[idxl].line, \
			g_multi_line.lane_line[idxr].line, mode, 1, &isok );


#ifdef MBUG_OPEN_
		MBUG("overtake: ");
		for (int i = 0; i < MAX2DPOINTS;i++)
		{
			MBUG("(%d, %d) ", g_multi_line.lane_line[idxl].line[i].x, g_multi_line.lane_line[idxl].line[i].y);
		}
		MBUG("\n");

		MBUG("overtake: ");
		for (int i = 0; i < MAX2DPOINTS; i++)
		{
			MBUG("(%d, %d) ", g_multi_line.lane_line[idxr].line[i].x, g_multi_line.lane_line[idxr].line[i].y);
		}
		MBUG("\n");
#endif
		if ( isok )
		{
#ifdef MBUG_OPEN_
			MBUG("goto err_exit 3\n");
#endif
			goto err_exit;
		}

#if ___TAKEOVER_DEBUG
		println( g_multi_line.lane_line[idxl].line, "LINE_DATA0" );
		println( g_multi_line.lane_line[idxr].line, "LINE_DATA1" );
#endif

	}


	// 得出结果
	{
// 		memset( g_plan_line, 0, sizeof( g_plan_line ) );
// 
// 		get_plan_line( g_plan_line, \
// 			g_multi_line.lane_line[idxl].line, \
// 			g_multi_line.lane_line[idxr].line, \
// 			mode, &isok );
		check_plan_line(g_multi_line.lane_line[idxl].line, \
			g_multi_line.lane_line[idxr].line, \
			mode, &isok );

		if (isok)
		{//[有碰撞]
			goto err_exit;
		}
		else
		{
			ret = check_the_dynobs(g_multi_line.lane_line[idxl].line, \
				g_multi_line.lane_line[idxr].line);

			if (ret == -1)
				goto err_exit;
			else
				isok = 0;
		}

		if ( isok )
		{

			goto err_exit;
		}
		else
		{
// 			fix_plan_way( g_plan_line, \
// 				g_multi_line.lane_line[idxl].line, \
// 				g_multi_line.lane_line[idxr].line );


			if (mode == 0)
			{
				//[确保初始道路安全]
				for (i=60; i<180; i++)
				{
					for (j=84; j< 100; j++)
					{
						if (gridmap[i][j] > 0)
						{
							if (g_serial == g_max_times)
							{
								g_serial--;
							}

							counter = 0;
						}
					}
				}
			}
			else
			{
				//[确保初始道路安全]
				for (i=60; i<180; i++)
				{
					for (j=60; j< 76; j++)
					{
						if (gridmap[i][j] > 0)
						{
							if (g_serial == g_max_times)
							{
								g_serial--;
							}

							counter = 0;
						}
					}
				}
			}
			

			if ( g_serial == g_max_times )
			{
				g_serial = 0;
				*status = TRACE_LANE;
#if ___TAKEOVER_DEBUG
				//				fclose( g_fp );
#endif
			}
			else
			{
				if (mode == 0)
				{
					*status = TAKEOVER_FROM_LEFT;
				}
				else
				{
					*status = TAKEOVER_FROM_RIGHT;
				}
			}

			memcpy( plan_line, g_plan_line, sizeof( g_plan_line ) );
		}
	}

	return 0;

err_exit:
	g_serial = 0;
	*status = D_EMERGENCY_STOP;
#if ___TAKEOVER_DEBUG
	//	fclose( g_fp );
#endif
	return 0;
}


int get_x_coord2(int y, COOR2 *line, int line_num, int *x)
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
		*x = line[0].x + (line[0].x - line[1].x) * (y - line[0].y) / (line[0].y - line[1].y);
	}

	else if (y >= line[line_num - 1].y)
		//*x = line[line_num - 1].x;
	{
		*x = line[line_num - 1].x + (line[line_num - 1].x - line[line_num - 2].x) * (y - line[line_num - 1].y) / (line[line_num - 1].y - line[line_num - 2].y);
	}

	else
	{
		for (i = 0; i < line_num - 1; i ++)
		{
			if (y >= line[i].y && y <= line[i + 1].y && line[i].y != line[i + 1].y)
				*x = line[i].x + (line[i].x - line[i + 1].x) * (y - line[i].y) / (line[i].y - line[i + 1].y);
		}
	}

	return 0;
}

int check_s_obs(int gridmap[][GRID_WIDTH], COOR2* plan_line, int* line_num)
{
	int i, j, k, k1;
	int sign;
	int cnt, cnt1;
	int xs, xe, yy;
	int sum, sum1;
	vector< pair<int, int> > left_pt, right_pt, tmp_pt;
	vector< vector< pair<int, int> > > left_lines, right_lines;
	COOR2 ls, rs;

	/************************************************************************/
	/************************************************************************/

	// 处理GRIDMAP
	{

		memset(g_gridmap, 0, sizeof(g_gridmap));

		// 清上下
		g_starty = VEHICLE_GRIDMAP_Y + 8;
		memcpy(g_gridmap + g_starty, gridmap + g_starty, 72 * GRID_WIDTH * sizeof(int));
		g_lasty = g_starty + 72;

		// 清左右
		for (i = g_starty; i < g_lasty; i++)
		{
			xs = VEHICLE_GRIDMAP_X - 16;
			memset(g_gridmap + i, 0, xs * sizeof(int));
			xe = VEHICLE_GRIDMAP_X + 16;
			memset(g_gridmap[i] + xe, 0, (GRID_WIDTH - xe) * sizeof(int));
		}
	}


	/************************************************************************/
	/************************************************************************/

	tmp_pt.clear();

	// 左边
	{
		left_pt.clear();
		left_lines.clear();

		while (1)
		{
			// 求左起点
			sign = 0;

			for (i = VEHICLE_GRIDMAP_Y + 16; i < VEHICLE_GRIDMAP_Y + 40 && sign == 0; i++)
			{
				for (sign = 0, j = VEHICLE_GRIDMAP_X; j > xs - 1; j--)
				{
					if (g_gridmap[i][j] != 0 && find(tmp_pt.begin(), tmp_pt.end(), make_pair(j, i)) == tmp_pt.end())
					{
						sign = 1;
						ls.x = j;
						ls.y = i;
						break;
					}
				}

				for (j -= 1; j > xs - 1; j--)
				{
					g_gridmap[i][j] = 0;
				}
			}

			// 找到左起点, 接着找左边
			if (sign == 1)
			{
				left_pt.push_back(make_pair(ls.x, ls.y));

				while (1)
				{
					sign = 0;

					for (k = ls.y + 1; k < ls.y + 14 && sign == 0; k++)
					{
						yy = ls.x - 6;

						for (k1 = ls.x + 5; k1 > yy; k1--)
						{
							if (g_gridmap[k][k1] != 0)
							{
								left_pt.push_back(make_pair(k1, k));
								ls.x = k1;
								ls.y = k;
								sign = 1;
								break;
							}
						}

						for (k1 -= 1; k1 > yy; k1--)
						{
							g_gridmap[k][k1] = 0;
						}
					}

					if (sign == 0)
					{
						break;
					}
				}


				if (left_pt.size() >= 2)
				{
					left_lines.push_back(left_pt);
				}


				tmp_pt.insert(tmp_pt.end(), left_pt.begin(), left_pt.end());

				left_pt.clear();
			}
			else // 未找到左起点
			{
				break;
			}
		}

		tmp_pt.clear();
	}


	// 右边
	{
		right_pt.clear();
		right_lines.clear();

		while (1)
		{
			// 求右起点
			sign = 0;

			for (i = VEHICLE_GRIDMAP_Y + 16; i < VEHICLE_GRIDMAP_Y + 40 && sign == 0; i++)
			{
				for (sign = 0, j = VEHICLE_GRIDMAP_X + 1; j < xe; j++)
				{
					if (g_gridmap[i][j] != 0 && find(tmp_pt.begin(), tmp_pt.end(), make_pair(j, i)) == tmp_pt.end())
					{
						sign = 1;
						rs.x = j;
						rs.y = i;
						break;
					}
				}

				for (j += 1; j < xe; j++)
				{
					g_gridmap[i][j] = 0;
				}
			}

			// 找到右起点, 接着找右边
			if (sign == 1)
			{
				right_pt.push_back(make_pair(rs.x, rs.y));

				while (1)
				{
					sign = 0;

					for (k = rs.y + 1; k < rs.y + 14 && sign == 0; k++)
					{
						yy = rs.x + 6;

						for (k1 = rs.x - 5; k1 < yy; k1++)
						{
							if (g_gridmap[k][k1] != 0)
							{
								right_pt.push_back(make_pair(k1, k));
								rs.x = k1;
								rs.y = k;
								sign = 1;
								break;
							}
						}

						for (k1 += 1; k1 < yy; k1++)
						{
							g_gridmap[k][k1] = 0;
						}
					}

					if (sign == 0)
					{
						break;
					}
				}

				if (right_pt.size() >= 2)
				{
					right_lines.push_back(right_pt);
				}

				tmp_pt.insert(tmp_pt.end(), right_pt.begin(), right_pt.end());

				right_pt.clear();
			}
			else // 未找到右起点
			{
				break;
			}
		}

		tmp_pt.clear();
	}



#if ___TAKEOVER_DEBUG

	for (i = 0; i < (int)left_lines.size(); i++)
	{
		memset(d1, 0, sizeof(d1));

		for (j = 0; j < (int)left_lines[i].size() && j < MAX2DPOINTS; j++)
		{
			d1[j].x = GX2X(left_lines[i][j].first);
			d1[j].y = GY2Y(left_lines[i][j].second);
		}

		println(d1, "LINE_DATA0", g_fp);
	}

	for (i = 0; i < (int)right_lines.size(); i++)
	{
		memset(d2, 0, sizeof(d2));

		for (j = 0; j < (int)right_lines[i].size() && j < MAX2DPOINTS; j++)
		{
			d2[j].x = GX2X(right_lines[i][j].first);
			d2[j].y = GY2Y(right_lines[i][j].second);
		}

		println(d2, "LINE_DATA3", g_fp);
	}

#endif

	/************************************************************************/
	/************************************************************************/
	if (left_lines.empty() && right_lines.empty())
	{
		goto err_exit;
	}

	// 选择边

	int i1, i2;

	left_pt.clear();
	right_pt.clear();

	if (!left_lines.empty() && !right_lines.empty())
	{
		sum = -1;

		for (i = 0; i < (int)left_lines.size(); i++)
		{
			for (j = 0; j < (int)right_lines.size(); j++)
			{
				sum1 = get_pairs(left_lines[i], right_lines[j]);

				if (sum < sum1)
				{
					sum = sum1;
					i1 = i;
					i2 = j;
				}
			}
		}

		left_pt.assign(left_lines[i1].begin(), left_lines[i1].end());
		right_pt.assign(right_lines[i2].begin(), right_lines[i2].end());
	}
	else
	{
		goto err_exit;

		if (!left_lines.empty())
		{
			cnt1 = 0;
			sum = get_vec_sum(left_lines[0]);

			for (i = 1; i < (int)left_lines.size(); i++)
			{
				sum1 = get_vec_sum(left_lines[i]);

				if (sum1 > sum)
				{
					cnt1 = i;
					sum = sum1;
				}
			}

			left_pt.assign(left_lines[cnt1].begin(), left_lines[cnt1].end());
		}
		else
		{
			cnt1 = 0;
			sum = get_vec_sum(right_lines[0]);

			for (i = 1; i < (int)right_lines.size(); i++)
			{
				sum1 = get_vec_sum(right_lines[i]);

				if (sum1 > sum)
				{
					cnt1 = i;
					sum = sum1;
				}
			}

			right_pt.assign(right_lines[cnt1].begin(), right_lines[cnt1].end());
		}
	}

	left_lines.clear();
	right_lines.clear();


	// 过滤无关点
	/*
	COOR2 tmp_line[2];

	for (i = 0; i < (int)left_pt.size() - 2; i++)
	{
		tmp_line[0].x = left_pt[i + 1].first;
		tmp_line[0].y = left_pt[i + 1].second;
		tmp_line[1].x = left_pt[i + 2].first;
		tmp_line[1].y = left_pt[i + 2].second;

		sum = GET_X_FROM_2P(left_pt[i].second, tmp_line[0], tmp_line[1]);

		if (abs(left_pt[i].first - sum) > 3)
		{
			g_gridmap[left_pt[i].second][left_pt[i].first] = 0;
			left_pt.erase(left_pt.begin() + i);
			i--;
		}
	}

	for (i = 0; i < (int)right_pt.size() - 2; i++)
	{
		tmp_line[0].x = right_pt[i + 1].first;
		tmp_line[0].y = right_pt[i + 1].second;
		tmp_line[1].x = right_pt[i + 2].first;
		tmp_line[1].y = right_pt[i + 2].second;

		sum = GET_X_FROM_2P(right_pt[i].second, tmp_line[0], tmp_line[1]);

		if (abs(right_pt[i].first - sum) > 3)
		{
			g_gridmap[right_pt[i].second][right_pt[i].first] = 0;
			right_pt.erase(right_pt.begin() + i);
			i--;
		}
	}
	*/


	// 补边
	if (left_pt.empty() && !right_pt.empty())
	{
		for (i = 0; i < (int)right_pt.size(); i++)
		{
			left_pt.push_back(make_pair(right_pt[i].first - 15, right_pt[i].second));
		}
	}
	else if (right_pt.empty() && !left_pt.empty())
	{
		for (i = 0; i < (int)left_pt.size(); i++)
		{
			right_pt.push_back(make_pair(left_pt[i].first + 15, left_pt[i].second));
		}
	}

	// 起始位置调整
	{
		sign = 0;
		yy = left_pt[0].second;

		if (yy < right_pt[0].second)
		{
			sign = 1;
			yy = right_pt[0].second;
		}

		if (sign == 0)
		{
			while (!right_pt.empty())
			{
				if (right_pt[0].second < yy)
				{
					right_pt.erase(right_pt.begin());
				}
				else
				{
					break;
				}
			}
		}
		else
		{
			while (!left_pt.empty())
			{
				if (left_pt[0].second < yy)
				{
					left_pt.erase(left_pt.begin());
				}
				else
				{
					break;
				}
			}
		}
	}

	if (left_pt.size() < 8 || right_pt.size() < 8)
	{
		goto err_exit;
	}



#if ___TAKEOVER_DEBUG

	fprintf(g_fp, "BEST EDGE\n");

	if (!left_pt.empty())
	{
		memset(d1, 0, sizeof(d1));

		for (j = 0; j < (int)left_pt.size() && j < MAX2DPOINTS; j++)
		{
			d1[j].x = GX2X(left_pt[j].first);
			d1[j].y = GY2Y(left_pt[j].second);
		}

		println(d1, "LINE_DATA0", g_fp);
	}

	if (!right_pt.empty())
	{
		memset(d2, 0, sizeof(d2));

		for (j = 0; j < (int)right_pt.size() && j < MAX2DPOINTS; j++)
		{
			d2[j].x = GX2X(right_pt[j].first);
			d2[j].y = GY2Y(right_pt[j].second);
		}

		println(d2, "LINE_DATA3", g_fp);
	}

#endif
	/************************************************************************/
	/************************************************************************/
	cnt1 = cnt = (int)(left_pt.size());

	if (cnt > (int)right_pt.size())
	{
		sign = 0;
		cnt = (int)(right_pt.size());
	}
	else
	{
		sign = 1;
		cnt1 = (int)(right_pt.size());
	}

	// 长边少于6个点 或者 长边Y向不足两米
	if (cnt1 < 6 || \
		(sign == 1 && right_pt[cnt1 - 1].second - right_pt[0].second < 12) || \
		(sign == 0 && left_pt[cnt1 - 1].second - left_pt[0].second < 12))
	{
		goto err_exit;
	}

	if (cnt < 2 || \
		(sign == 1 && left_pt[cnt - 1].second - left_pt[0].second < 8) || \
		(sign == 0 && right_pt[cnt - 1].second - right_pt[0].second < 8))
	{
		goto err_exit;
	}

	// 对组
	k = 1;
	memset(plan_line, 0, sizeof(COOR2) * MAX2DPOINTS);
	tmp_pt.clear();

	if (sign == 0)
	{
		for (i = 0; i < cnt1; i++)
		{
			for (j = 0; j < cnt; j++)
			{
				if (left_pt[i].second >= right_pt[j].second - 3 && \
					left_pt[i].second <= right_pt[j].second + 3 && \
					left_pt[i].first <= right_pt[j].first - 11)
				{
					tmp_pt.push_back(make_pair(i, j));
					/*if ( k < MAX2DPOINTS )
					{
					plan_line[k].x = (GX2X(left_pt[i].first) + GX2X(right_pt[j].first)) / 2;
					plan_line[k ++].y = (GY2Y(left_pt[i].second) + GY2Y(right_pt[j].second)) / 2;
					}*/
					break;
				}
			}
		}
	}
	else
	{
		for (i = 0; i < cnt1; i++)
		{
			for (j = 0; j < cnt; j++)
			{
				if (left_pt[j].second - 3 <= right_pt[i].second && \
					left_pt[j].second + 3 >= right_pt[i].second && \
					left_pt[j].first + 11 <= right_pt[i].first)
				{
					tmp_pt.push_back(make_pair(j, i));

					/*if ( k < MAX2DPOINTS )
					{
					plan_line[k].x = (GX2X(left_pt[j].first) + GX2X(right_pt[i].first)) / 2;
					plan_line[k ++].y = (GY2Y(left_pt[j].second) + GY2Y(right_pt[i].second)) / 2;
					}*/
					break;
				}
			}
		}
	}

	if ((int)tmp_pt.size() < 5)
	{
		goto err_exit;
	}


	fix_s_plan(gridmap, left_pt, right_pt, tmp_pt, plan_line, line_num);


#if ___TAKEOVER_DEBUG
	println(plan_line, "LINE_DATA2");
	fprintf(g_fp, "pt count == %d  is s\n", *line_num);
#endif

	SET_SOBS_SIGN(1);
	return 0;


err_exit:
#if ___TAKEOVER_DEBUG
	fprintf(stderr, "is not s\n", cnt);
#endif
	SET_SOBS_SIGN(0);
	return -1;
}

int hd_check_s_obs(int gridmap[][GRID_WIDTH_HD], COOR2* plan_line, int* line_num)
{
	int i, j, k, k1;
	int sign;
	int cnt, cnt1;
	int xs, xe, yy;
	int sum, sum1;
	vector< pair<int, int> > left_pt, right_pt, tmp_pt;
	vector< vector< pair<int, int> > > left_lines, right_lines;
	COOR2 ls, rs;

	/************************************************************************/
	/************************************************************************/

	// 处理GRIDMAP
	{

		memset(g_hd_gridmap, 0, sizeof(g_hd_gridmap));

		// 清上下
		g_starty = HD_VEHICLE_GRIDMAP_Y + 16;
		memcpy(g_hd_gridmap + g_starty, gridmap + g_starty, 112 * GRID_WIDTH_HD * sizeof(int));
		g_lasty = g_starty + 112;

		// 清左右
		for (i = g_starty; i < g_lasty; i++)
		{
			xs = HD_VEHICLE_GRIDMAP_X - 32;
			memset(g_hd_gridmap + i, 0, xs * sizeof(int));
			xe = HD_VEHICLE_GRIDMAP_X + 32;
			memset(g_hd_gridmap[i] + xe, 0, (GRID_WIDTH_HD - xe) * sizeof(int));
		}
	}

	/************************************************************************/
	/************************************************************************/

	tmp_pt.clear();

	// 左边
	{
		left_pt.clear();
		left_lines.clear();

		while (1)
		{
			// 求左起点
			sign = 0;

			for (i = HD_VEHICLE_GRIDMAP_Y + 32; i < HD_VEHICLE_GRIDMAP_Y + 80 && sign == 0; i++)
			{
				for (sign = 0, j = HD_VEHICLE_GRIDMAP_X; j > xs - 1; j--)
				{
					if (g_hd_gridmap[i][j] == 8 && find(tmp_pt.begin(), tmp_pt.end(), make_pair(j, i)) == tmp_pt.end())
					{
						sign = 1;
						ls.x = j;
						ls.y = i;
						break;
					}
				}

				for (j -= 1; j > xs - 1; j--)
				{
					g_hd_gridmap[i][j] = 0;
				}
			}

			// 找到左起点, 接着找左边
			if (sign == 1)
			{
				left_pt.push_back(make_pair(ls.x, ls.y));

				while (1)
				{
					sign = 0;

					for (k = ls.y + 1; k < ls.y + 28 && sign == 0; k++)
					{
						yy = ls.x - 12;

						for (k1 = ls.x + 11; k1 > yy; k1--)
						{
							if (g_hd_gridmap[k][k1] == 8)
							{
								left_pt.push_back(make_pair(k1, k));
								ls.x = k1;
								ls.y = k;
								sign = 1;
								break;
							}
						}

						for (k1 -= 1; k1 > yy; k1--)
						{
							g_hd_gridmap[k][k1] = 0;
						}
					}

					if (sign == 0)
					{
						break;
					}
				}


				if (left_pt.size() >= 2)
				{
					left_lines.push_back(left_pt);
				}


				tmp_pt.insert(tmp_pt.end(), left_pt.begin(), left_pt.end());

				left_pt.clear();
			}
			else // 未找到左起点
			{
				break;
			}
		}

		tmp_pt.clear();
	}


	// 右边
	{
		right_pt.clear();
		right_lines.clear();

		while (1)
		{
			// 求右起点
			sign = 0;

			for (i = HD_VEHICLE_GRIDMAP_Y + 32; i < HD_VEHICLE_GRIDMAP_Y + 80 && sign == 0; i++)
			{
				for (sign = 0, j = HD_VEHICLE_GRIDMAP_X + 1; j < xe; j++)
				{
					if (g_hd_gridmap[i][j] == 8 && find(tmp_pt.begin(), tmp_pt.end(), make_pair(j, i)) == tmp_pt.end())
					{
						sign = 1;
						rs.x = j;
						rs.y = i;
						break;
					}
				}

				for (j += 1; j < xe; j++)
				{
					g_hd_gridmap[i][j] = 0;
				}
			}

			// 找到右起点, 接着找右边
			if (sign == 1)
			{
				right_pt.push_back(make_pair(rs.x, rs.y));

				while (1)
				{
					sign = 0;

					for (k = rs.y + 1; k < rs.y + 28 && sign == 0; k++)
					{
						yy = rs.x + 12;

						for (k1 = rs.x - 11; k1 < yy; k1++)
						{
							if (g_hd_gridmap[k][k1] == 8)
							{
								right_pt.push_back(make_pair(k1, k));
								rs.x = k1;
								rs.y = k;
								sign = 1;
								break;
							}
						}

						for (k1 += 1; k1 < yy; k1++)
						{
							g_hd_gridmap[k][k1] = 0;
						}
					}

					if (sign == 0)
					{
						break;
					}
				}

				if (right_pt.size() >= 2)
				{
					right_lines.push_back(right_pt);
				}

				tmp_pt.insert(tmp_pt.end(), right_pt.begin(), right_pt.end());

				right_pt.clear();
			}
			else // 未找到右起点
			{
				break;
			}
		}

		tmp_pt.clear();
	}



#ifdef MBUG_OPEN_

	for (i = 0; i < (int)left_lines.size(); i++)
	{
		MBUG("GRID LINE:\n");
		for (j = 0; j < (int)left_lines[i].size() && j < MAX2DPOINTS; j++)
		{
			MBUG("(%d, %d) ", HDGX2X(left_lines[i][j].first), HDGY2Y(left_lines[i][j].second));
		}
		MBUG("\n");
	}

	for (i = 0; i < (int)right_lines.size(); i++)
	{
		MBUG("GRID LINE:\n");
		for (j = 0; j < (int)right_lines[i].size() && j < MAX2DPOINTS; j++)
		{
			MBUG("(%d, %d) ", HDGX2X(right_lines[i][j].first), HDGY2Y(right_lines[i][j].second));
		}
		MBUG("\n");
	}

#endif

	/************************************************************************/
	/************************************************************************/
	if (left_lines.empty() && right_lines.empty())
	{
		goto err_exit;
	}

	// 选择边

	int i1, i2;

	left_pt.clear();
	right_pt.clear();

	if (!left_lines.empty() && !right_lines.empty())
	{
		sum = -1;

		for (i = 0; i < (int)left_lines.size(); i++)
		{
			for (j = 0; j < (int)right_lines.size(); j++)
			{
				sum1 = get_pairs(left_lines[i], right_lines[j]);

				if (sum < sum1)
				{
					sum = sum1;
					i1 = i;
					i2 = j;
				}
			}
		}

		left_pt.assign(left_lines[i1].begin(), left_lines[i1].end());
		right_pt.assign(right_lines[i2].begin(), right_lines[i2].end());
	}
	else
		goto err_exit;

	left_lines.clear();
	right_lines.clear();

	// 补边
	if (left_pt.empty() && !right_pt.empty())
	{
		for (i = 0; i < (int)right_pt.size(); i++)
		{
			left_pt.push_back(make_pair(right_pt[i].first - 30, right_pt[i].second));
		}
	}
	else if (right_pt.empty() && !left_pt.empty())
	{
		for (i = 0; i < (int)left_pt.size(); i++)
		{
			right_pt.push_back(make_pair(left_pt[i].first + 30, left_pt[i].second));
		}
	}

	// 起始位置调整
	{
		sign = 0;
		yy = left_pt[0].second;

		if (yy < right_pt[0].second)
		{
			sign = 1;
			yy = right_pt[0].second;
		}

		if (sign == 0)
		{
			while (!right_pt.empty())
			{
				if (right_pt[0].second < yy)
				{
					right_pt.erase(right_pt.begin());
				}
				else
				{
					break;
				}
			}
		}
		else
		{
			while (!left_pt.empty())
			{
				if (left_pt[0].second < yy)
				{
					left_pt.erase(left_pt.begin());
				}
				else
				{
					break;
				}
			}
		}
	}

	if (left_pt.size() < 8 || right_pt.size() < 8)
	{
		goto err_exit;
	}

#ifdef MBUG_OPEN_

	MBUG("BEST EDGE:\n");

	if (!left_pt.empty())
	{
		MBUG("GRID LINE:\n");
		for (i = 0; i < (int)left_pt.size() && i < MAX2DPOINTS; i++)
		{
			MBUG("(%d, %d) ", HDGX2X(left_pt[i].first), HDGY2Y(left_pt[i].second));
		}
		MBUG("\n");
	}

	if (!right_pt.empty())
	{
		MBUG("GRID LINE:\n");
		for (i = 0; i < (int)right_pt.size() && i < MAX2DPOINTS; i++)
		{
			MBUG("(%d, %d) ", HDGX2X(right_pt[i].first), HDGY2Y(right_pt[i].second));
		}
		MBUG("\n");
	}

#endif
	/************************************************************************/
	/************************************************************************/
	cnt1 = cnt = (int)(left_pt.size());

	if (cnt > (int)right_pt.size())
	{
		sign = 0;
		cnt = (int)(right_pt.size());
	}
	else
	{
		sign = 1;
		cnt1 = (int)(right_pt.size());
	}

	// 长边少于6个点 或者 长边Y向不足两米
	if (cnt1 < 6 || \
		(sign == 1 && right_pt[cnt1 - 1].second - right_pt[0].second < 24) || \
		(sign == 0 && left_pt[cnt1 - 1].second - left_pt[0].second < 24))
	{
		goto err_exit;
	}

	if (cnt < 2 || \
		(sign == 1 && left_pt[cnt - 1].second - left_pt[0].second < 16) || \
		(sign == 0 && right_pt[cnt - 1].second - right_pt[0].second < 16))
	{
		goto err_exit;
	}

	// 对组
	k = 1;
	memset(plan_line, 0, sizeof(COOR2) * MAX2DPOINTS);
	tmp_pt.clear();

	if (sign == 0)
	{
		for (i = 0; i < cnt1; i++)
		{
			for (j = 0; j < cnt; j++)
			{
				if (left_pt[i].second >= right_pt[j].second - 6 && \
					left_pt[i].second <= right_pt[j].second + 6 && \
					left_pt[i].first <= right_pt[j].first - 22)
				{
					tmp_pt.push_back(make_pair(i, j));
					break;
				}
			}
		}
	}
	else
	{
		for (i = 0; i < cnt1; i++)
		{
			for (j = 0; j < cnt; j++)
			{
				if (left_pt[j].second - 6 <= right_pt[i].second && \
					left_pt[j].second + 6 >= right_pt[i].second && \
					left_pt[j].first + 22 <= right_pt[i].first)
				{
					tmp_pt.push_back(make_pair(j, i));
					break;
				}
			}
		}
	}

	if ((int)tmp_pt.size() < 5)
	{
		goto err_exit;
	}


	hd_fix_s_plan(gridmap, left_pt, right_pt, tmp_pt, plan_line, line_num);


#if ___TAKEOVER_DEBUG
	println(plan_line, "LINE_DATA2");
	fprintf(g_fp, "pt count == %d  is s\n", *line_num);
#endif

	SET_SOBS_SIGN(1);
	return 0;


err_exit:
#if ___TAKEOVER_DEBUG
	fprintf(stderr, "is not s\n", cnt);
#endif
	SET_SOBS_SIGN(0);
	return -1;
}