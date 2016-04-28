#include "plsdk.h"

#include <string>
#include "./zgcc/trace_road.h"

typedef int KERNEL_RET;
#define RET_OK		0
#define RET_ERR		1

enum Action
{
	EXECU_STOP = 0,
	EXECU_REVERSE = 1,
	EXECU_LEFT_LANE = 2,
	EXECU_RIGHT_LANE = 3,

	TRACE_DUMMY_ROAD = 10,
	TRACE_NATURAL_ROAD = 11,
	TRACE_URBEN_ROAD = 12,
};

static Action REG_ACTION;
static float REG_SPEED;
static bool REG_IS_NATURAL_ROAD_BLOCKED;

extern KERNEL_RET Odometry2Speed(const ODO_INFO * odo, float * speed);//从ODO信息中计算当前车速
extern KERNEL_RET SetRegistry(const std::string & key, void * val);//改变内核注册项指定的内容
extern KERNEL_RET GetRegistry(const std::string & key, void * val);//获得内核注册项指定的内容
extern KERNEL_RET GenerateVirtualNaturalRoad();//从一条GPS序列生成一对自然道边(车体坐标)，车被包含在道边内部

extern KERNEL_RET ZGCC_TraceNaturalRoad(PL_FUNC_INPUT * pl_func_input);
extern KERNEL_RET ZGCC_CutNaturalEdge(NATURAL_BOUNDARY & boundary);//截取自然道边边线
extern KERNEL_RET ZGCC_FitNaturalEdge(NATURAL_BOUNDARY & boundary);//重新整理自然道边的边线
extern KERNEL_RET ZGCC_FixNaturalEdge(NATURAL_BOUNDARY & boundary);//将输入的一对自然道边(车体坐标)根据栅格图生成无碰撞的新自然道边

KERNEL_RET Odometry2Speed(const ODO_INFO * odo, float * speed)
{
	static ODO_INFO odo_pre = *odo;
	static unsigned short odo_win[5] = { 0, 0, 0, 0, 0 };
	static unsigned short odo_buf[5] = { 0, 0, 0, 0, 0 };

	unsigned short odo_l_dif = 0u;
	unsigned short odo_r_dif = 0u;
	//计算计数差值，当计数溢出时新值小于旧值，需要对计数做补偿
	(odo->left < odo_pre.left) ? (odo_l_dif = odo->left + (0xFFFF - odo_pre.left)) : (odo_l_dif = odo->left - odo_pre.left);
	(odo->right < odo_pre.right) ? (odo_r_dif = odo->right + (0xFFFF - odo_pre.right)) : (odo_l_dif = odo->right - odo_pre.right);

	//移动滑窗，将当前计数值插入到窗口内
	memmove(odo_win + 1, odo_win, sizeof(unsigned short) * 4);
	odo_win[0] = odo_l_dif + odo_r_dif;

	//更新缓存计数数据
	odo_pre = *odo;

	//采用中值滤波方法剔除过大或过小的野值
	memcpy(odo_buf, odo_win, sizeof(unsigned short) * 5);
	register int odo_mid = 0;
	for (unsigned int i = 0; i != 3; ++i)//事实上只需要计算出下标为2的值即可
	{
		odo_mid = odo_buf[i];
		for (unsigned int j = i + 1; j != 5; ++j)
		{
			if (odo_buf[i] >= odo_buf[j])
			{
				odo_mid = odo_buf[j];
				odo_buf[i] = odo_mid;
			}
		}
	}

	//通过标定参数，将左右里程计的脉冲计数换算成距离，猎豹参数 0.05028  巡洋舰参数 0.0513149  每单位脉冲的距离 单位m
	*speed = odo_mid * 0.5f * 51.3149f;
	if (1 == odo->revers) *speed = -*speed;

	return RET_OK;
}

KERNEL_RET SetRegistry(const std::string & key, void * val)
{
	if ("REG_ACTION" == key)
	{
		Action * p = static_cast<Action *>(val);
		REG_ACTION = *p;
		return RET_OK;
	}
	else if ("REG_SPEED" == key)
	{
		float * p = static_cast<float *>(val);
		REG_SPEED = *p;
		return RET_OK;
	}
	else
	{
		return RET_ERR;
	}
}

KERNEL_RET GetRegistry(const std::string & key, void * val)
{
	if ("REG_ACTION" == key)
	{
		Action * p = static_cast<Action *>(val);
		*p = REG_ACTION;
		return RET_OK;
	}
	else if ("REG_SPEED" == key)
	{
		float * p = static_cast<float *>(val);
		*p = REG_SPEED;
		return RET_OK;
	}
	else
	{
		val = nullptr;
		return RET_ERR;
	}
}

KERNEL_RET GenerateVirtualNaturalRoad(ROAD_NATURAL_BOUNDARY * natural, plsdk::Obs * obs)
{
	return RET_OK;
}

KERNEL_RET ZGCC_TraceNaturalRoad(PL_FUNC_INPUT * pl_input)
{
	static NATURAL_BOUNDARY g_natural_boundary;
	static int g_l_num, g_r_num;
	memset(&g_natural_boundary, 0, sizeof(NATURAL_BOUNDARY));

	//1 提取并修剪自然道边数据,这一部分的代码来源于zgcc/trace_road.cpp:keep_natural_road
	//1.1 从协议数据结构中提取出自然道边的数据
	ROAD_NATURAL_BOUNDARY & _natural = pl_input->fu_pl_data.boundary;
	for (unsigned int cnt = 0; cnt != NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; ++cnt)
	{
		if (_natural.l_boundary[cnt].x != 0 && _natural.l_boundary[cnt].y != 0)
		{
			g_natural_boundary.l_boundary[g_natural_boundary.l_nums++] = \
				COOR2{ (INT32)_natural.l_boundary[cnt].x, (INT32)_natural.l_boundary[cnt].y };
		}
		if (_natural.r_boundary[cnt].x != 0 && _natural.r_boundary[cnt].y != 0)
		{
			g_natural_boundary.r_boundary[g_natural_boundary.l_nums++] = \
				COOR2{ (INT32)_natural.r_boundary[cnt].x, (INT32)_natural.r_boundary[cnt].y };
		}
	}
	g_l_num = g_natural_boundary.l_nums;
	g_r_num = g_natural_boundary.r_nums;

	//1.2 修剪并重塑协议数据，使其y坐标满足从4m到30m的约束要求,将边线序列新插值至20个点
	//截取自然道边边线
	ZGCC_CutNaturalEdge(g_natural_boundary);
	//重新整理两条边线
	ZGCC_FitNaturalEdge(g_natural_boundary);
	/* Sealed!!!!! zgcc老的逻辑，现在使用最新的逻辑
	unsigned int ptr_l = 0, ptr_r = 0, flag_l = 0, flag_r = 0, num_l = 0, num_r = 0;
	for (unsigned int cnt = 0; cnt != g_natural_boundary.l_nums; ++cnt)
	{
		//在查找头模式下，寻找第一个满足大于4m条件的点
		if (0 == flag_l && g_natural_boundary.l_boundary[cnt].y >= 400){ ptr_l = cnt; flag_l = 1; num_l = 1; }
		if (0 == flag_r && g_natural_boundary.r_boundary[cnt].y >= 400){ ptr_r = cnt; flag_r = 1; num_r = 1; }
		//在查找尾模式下，为满足小于30m点计数
		if (1 == flag_l && g_natural_boundary.l_boundary[cnt].y < 3000){ ++num_l; }
		if (1 == flag_r && g_natural_boundary.r_boundary[cnt].y < 3000){ ++num_r; }
	}
	for (unsigned int cnt; cnt)
	if (num_l >= 2 && num_r >= 2)
	{
		memmove(g_natural_boundary.l_boundary, g_natural_boundary.l_boundary + ptr_l, sizeof(COOR2)*num_l);
		g_natural_boundary.l_nums = num_l;
		memmove(g_natural_boundary.r_boundary, g_natural_boundary.r_boundary + ptr_r, sizeof(COOR2)*num_r);
		g_natural_boundary.r_nums = num_r;
		
	}
	else
	{
		//没有有效的数据，清空自然道边的数据
		memset(g_natural_boundary.l_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		g_natural_boundary.l_nums = 0;
		memset(&g_natural_boundary.r_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		g_natural_boundary.r_nums = 0;
	}
	*/
	g_l_num = g_natural_boundary.l_nums;
	g_r_num = g_natural_boundary.r_nums;

	//2 修整自然道边，使得自然道边所夹的区域是一个无碰的区域，代码来源于zgcc/trace_road.cpp:check_the_lane
	static COOR2 l_edge_buf[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	static COOR2 r_edge_buf[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	//2.1 将输入数据拷贝到临时道路边界中
	memcpy(l_edge_buf, g_natural_boundary.l_boundary, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE*sizeof(COOR2));
	memcpy(r_edge_buf, g_natural_boundary.r_boundary, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE*sizeof(COOR2));
	//2.2 根据路边情况，构建障碍提取表

	return RET_OK;
}

KERNEL_RET ZGCC_CutNaturalEdge(NATURAL_BOUNDARY & boundary)
{
	//ASSERT!!!!!
	if (boundary.l_nums < 2 || boundary.r_nums < 2)
	{
		memset(boundary.l_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memset(boundary.r_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		boundary.l_nums = 0;
		boundary.r_nums = 0;
		return RET_ERR;
	}
	bool isValid_l = true, isValid_r = true;//边线合法标志,预置为合法

	//1 基线对齐
	//1.1 基线对齐，左边界线操作
	if (boundary.l_boundary[0].y > 400)//第一个点已经超出了400，向基线处延长并插入一个点
	{
		COOR2 * _p = boundary.l_boundary;
		COOR2 base_point;
		unsigned int move_num = 0;//计算移动量
		//判断当前边线点是否已经被占满
		(boundary.l_nums < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE) ?
			//边线未占满，统一向后移动一个位置，再插入一个新值，总点数加一
			(move_num = boundary.l_nums++) :
			//边线已占满，只移动前20-1个，再补一个新值，总点数不变
			(move_num = boundary.l_nums-1);
		memmove(boundary.l_boundary + 1, boundary.l_boundary, sizeof(COOR2)*move_num);//挪动点
		//插入基点
		base_point.x = INT32(_p->x + (_p->x - (_p + 1)->x) * float(400 - _p->y) / float(_p->y - (_p + 1)->y));
		base_point.y = 400;
		boundary.l_boundary[0] = base_point;
	}
	else if (boundary.l_boundary[0].y < 400)//第一个点没有超出400，依次向上寻找到跨越基线的一对点
	{
		//定位到基线前一个点
		int p_pre = 0;
		for (int cnt = 0; cnt != boundary.l_nums; ++cnt)
		{
			if (boundary.l_boundary[cnt].y < 400)
				p_pre = cnt;
			else
				break;
		}
		if (p_pre + 1 < boundary.l_nums)//定位点不是最后一个点，则修改该定位点，使其为定位点和下一个点在y=400的截取点
		{
			COOR2 * _p = boundary.l_boundary + p_pre;
			COOR2 base_point;
			base_point.x = INT32(_p->x + (_p->x - (_p + 1)->x) * float(400 - _p->y) / float(_p->y - (_p + 1)->y));
			base_point.y = 400;
			boundary.l_boundary[p_pre] = base_point;
			boundary.l_nums -= p_pre;//更新有效数据点个数
			memmove(boundary.l_boundary, boundary.l_boundary + p_pre, sizeof(COOR2)*boundary.l_nums);
		}
		else//定位点是最后一个点，该边线无法截取到有用的数据
		{
			isValid_l = false;
		}
	}
	//1.2 基线对齐，右边界线操作
	if (boundary.r_boundary[0].y > 400)//第一个点已经超出了400，向基线处延长并插入一个点
	{
		COOR2 * _p = boundary.r_boundary;
		COOR2 base_point;
		unsigned int move_num = 0;//计算移动量
		//判断当前边线点是否已经被占满
		(boundary.r_nums < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE) ?
			//边线未占满，统一向后移动一个位置，再插入一个新值，总点数加一
			(move_num = boundary.r_nums++) :
			//边线已占满，只移动前19个，再补一个新值，总点数不变
			(move_num = boundary.r_nums-1);
		memmove(boundary.r_boundary + 1, boundary.r_boundary, sizeof(COOR2)*move_num);//挪动点
		//插入新得到的基点
		base_point.x = INT32(_p->x + (_p->x - (_p + 1)->x) * float(400 - _p->y) / float(_p->y - (_p + 1)->y));
		base_point.y = 400;
		boundary.r_boundary[0] = base_point;
	}
	else if (boundary.r_boundary[0].y < 400)//第一个点没有超出400，依次向上寻找到跨越基线的一对点
	{
		//定位到基线前一个点
		int p_pre = 0;
		for (int cnt = 0; cnt != boundary.r_nums; ++cnt)
		{
			if (boundary.r_boundary[cnt].y < 400)
				p_pre = cnt;
			else
				break;
		}
		if (p_pre + 1 < boundary.r_nums)//定位点不是最后一个点，则修改该定位点的值，使其为定位点和下一个点在y=400的截取点
		{
			COOR2 * _p = boundary.r_boundary + p_pre;
			COOR2 base_point;
			base_point.x = INT32(_p->x + (_p->x - (_p + 1)->x) * float(400 - _p->y) / float(_p->y - (_p + 1)->y));
			base_point.y = 400;
			boundary.r_boundary[p_pre] = base_point;
			boundary.r_nums -= p_pre;//更新有效的数据点个数
			memmove(boundary.r_boundary, boundary.r_boundary + p_pre, sizeof(COOR2)*boundary.r_nums);
		}
		else//定位点是最后一个点，该边线无法截取到有用的数据
		{
			isValid_r = false;
		}
	}

	//发现无法截取到有效的数据，将目标边线清空，返回错误
	if (isValid_l && isValid_r)
	{
		//没有有效的数据，清空自然道边的数据
		memset(boundary.l_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		boundary.l_nums = 0;
		memset(boundary.r_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		boundary.r_nums = 0;
		return RET_ERR;
	}

	//2 顶线对齐
	//2.1 先找出两条线的最大y值, 对较长的线采取截取的方法，使其在y坐标的最大值上是一致的
	int l_max_y = boundary.l_boundary[boundary.l_nums - 1].y;
	int	r_max_y = boundary.r_boundary[boundary.r_nums - 1].y;
	if (l_max_y > r_max_y)//截取左边线，使其边线的最大y与右边线一致
	{
		//定位到顶线前一个点
		int p_pre = 0;
		for (int cnt = 0; cnt != boundary.l_nums; ++cnt)
		{
			if (boundary.l_boundary[cnt].y < r_max_y)
				p_pre = cnt;
			else
				break;
		}
		//截取指定y水平上的点
		COOR2 * _p = boundary.l_boundary + p_pre;
		COOR2 top_point;
		top_point.x = INT32(_p->x + (_p->x - (_p + 1)->x) * float(r_max_y - _p->y) / float(_p->y - (_p + 1)->y));
		top_point.y = r_max_y;
		memset(_p + 1, 0, sizeof(COOR2)*(NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - p_pre - 1));
		boundary.l_boundary[p_pre + 1] = top_point;
		boundary.l_nums = p_pre + 2;
	}
	else if (l_max_y < r_max_y)//截取右边线，使其边线的最大y与右边线一致
	{
		//定位到顶线前一个点
		int p_pre = 0;
		for (int cnt = 0; cnt != boundary.r_nums; ++cnt)
		{
			if (boundary.r_boundary[cnt].y < l_max_y)
				p_pre = cnt;
			else
				break;
		}
		//截取指定y水平上的点
		COOR2 * _p = boundary.r_boundary + p_pre;
		COOR2 top_point;
		top_point.x = INT32(_p->x + (_p->x - (_p + 1)->x) * float(l_max_y - _p->y) / float(_p->y - (_p + 1)->y));
		top_point.y = l_max_y;
		memset(_p + 1, 0, sizeof(COOR2)*(NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - p_pre - 1));
		boundary.r_boundary[p_pre + 1] = top_point;
		boundary.r_nums = p_pre + 2;
	}
	return RET_OK;
}

KERNEL_RET ZGCC_FitNaturalEdge(NATURAL_BOUNDARY & boundary)
{
	//ASSERT!!!!!
	if (boundary.l_nums < 2 || boundary.r_nums < 2) return RET_ERR;

	static COOR2 edge[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int edge_len = 0;
	float step_y = 0, yy = 0;//y增长的步长和实际值

	//1 整理左边线
	memcpy(edge, boundary.l_boundary, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);//左边线拷贝
	edge_len = boundary.l_nums;//记录边线点的个数
	step_y = (boundary.l_boundary[boundary.l_nums - 1].y - boundary.l_boundary[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);//计算y步长
	memset(boundary.l_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);//清空原有边线信息
	boundary.l_nums = 0;//清空原有边线的点个数
	//在y等距序列上插值
	yy = (float)edge[0].y;
	for (int cnt = 0; cnt < edge_len - 1; ++cnt)
	{
		COOR2 & td1 = edge[cnt];
		COOR2 & td2 = edge[cnt + 1];
		//两个点之间是水平关系，则跳过不处理
		if (td1.y == td2.y) continue;
		//在两点之间寻找一个y满足要求的点
		while (yy < td2.y && boundary.l_nums < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		{
			boundary.l_boundary[boundary.l_nums].y = (INT32)yy;
			boundary.l_boundary[boundary.l_nums].x = INT32((float(td1.x - td2.x) / float(td1.y - td2.y) * (yy - td1.y)) + td1.x + 0.5);
			boundary.l_nums++;
			yy += step_y;
		}
		//最后一个点即使不满足y的间距要求，在边线没填满的情况下，可以将其直接当作最后一个点
		if (yy >= edge[edge_len - 1].y && boundary.l_nums < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		{
			boundary.l_boundary[boundary.l_nums++] = edge[edge_len - 1];
			break;
		}
	}
	//整理右边线
	memcpy(edge, boundary.r_boundary, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);//左边线拷贝
	edge_len = boundary.r_nums;//记录边线点的个数
	step_y = (boundary.r_boundary[boundary.r_nums - 1].y - boundary.r_boundary[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);//计算y步长
	memset(boundary.r_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);//清空原有边线信息
	boundary.r_nums = 0;//清空原有边线的点个数
	//在y等距序列上插值
	yy = (float)edge[0].y;
	for (int cnt = 0; cnt < edge_len - 1; ++cnt)
	{
		COOR2 & td1 = edge[cnt];
		COOR2 & td2 = edge[cnt + 1];
		//两个点之间是水平关系，则跳过不处理
		if (td1.y == td2.y) continue;
		//在两点之间寻找一个y满足要求的点
		while (yy < td2.y && boundary.r_nums < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		{
			boundary.r_boundary[boundary.r_nums].y = (INT32)yy;
			boundary.r_boundary[boundary.r_nums].x = INT32((float(td1.x - td2.x) / float(td1.y - td2.y) * (yy - td1.y)) + td1.x + 0.5);
			boundary.r_nums++;
			yy += step_y;
		}
		//最后一个点即使不满足y的间距要求，在边线没填满的情况下，可以将其直接当作最后一个点
		if (yy >= edge[edge_len - 1].y && boundary.r_nums < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		{
			boundary.r_boundary[boundary.r_nums++] = edge[edge_len - 1];
			break;
		}
	}
	return RET_OK;
}

KERNEL_RET ZGCC_FixNaturalEdge(NATURAL_BOUNDARY & boundary, const float * lidar64_map)
{
	static int raster_l[MAP_64_CELL_VER];//大小指定为64线雷达栅格的高度，保证光栅充足够用
	static int raster_r[MAP_64_CELL_VER];//大小指定为64线雷达栅格的高度，保证光栅充足够用
	memset(raster_l, 0, sizeof(int)*MAP_ROI_CELL_VER);
	memset(raster_r, 0, sizeof(int)*MAP_ROI_CELL_VER);
	const int base_line_offset = (2000 + 400) / 25;//起始栅格raster_l[0]在大栅格地图中的坐标y偏移
	const int base_line_y = 400;//栅格起始为坐标位置，该位置是约定的车前4m
	const int raster_num = int((boundary.l_boundary[boundary.l_nums - 1].y - base_line_y) / MAP_ROI_CELL_SIZ);//有效光栅的数量

	//ASSERT!!!!! 
	if (raster_num < 1) return RET_ERR;

	//1  将两条边线光栅化，记录到相应的光栅位置中
	//1.1 处理左边的数据，结果存入raster_l结构
	int max_x_l= -2000;//默认不得少于-2000
	for (int cnt = 0; cnt != raster_num; ++cnt)
	{
		float y1 = base_line_y + cnt*25.0f;
		float y2 = y1 + 25.0f;;
		//确定在y1~y2间最大的x,存入对应的光栅结构中
		//求y1与线段的交点，定位目标点位置
		for (int idx = 0; idx != boundary.l_nums - 1; ++idx)
		{
			COOR2 & td1 = boundary.l_boundary[idx];
			COOR2 & td2 = boundary.l_boundary[idx + 1];
			//找到符合y1条件的点
			if (td1.y <= y1 && td2.y > y1)
				max_x_l = td1.x + (td1.x - td2.x) * (y1 - td1.y) / (td1.y - td2.y);
			else
				break;
		}
		//求y1与y2之间的点，并更新定位目标点位置
		for (int idx = 0; idx != boundary.l_nums; ++idx)
		{
			COOR2 & td = boundary.l_boundary[idx];
			if (td.y >= y1 && td.y < y2 && td.x > max_x_l)
				max_x_l = td.x;
		}
		//求y2与线段的交点，定位目标点位置
		for (int idx = 0; idx != boundary.l_nums - 1; ++idx)
		{
			COOR2 & td1 = boundary.l_boundary[idx];
			COOR2 & td2 = boundary.l_boundary[idx + 1];
			if (td1.y <= y2 && td2.y > y2)
			{
				float xx = td1.x + (td1.x - td2.x)*(y1 - td1.y) / (td1.y - td2.y);
				if (xx > max_x_l) max_x_l = xx;
			}
		}
		//将该栅格位置上的x锚点计算并以栅格坐标表示
		max_x_l += 2000.0f;//转换到栅格坐标
		raster_l[cnt] = int((max_x_l + 2000.0f) / MAP_ROI_CELL_SIZ);
	}
	//1.2 处理右边的数据，结果存入raster_r结构
	int max_x_r = 2000;//默认不得大于2000
	for (int cnt = 0; cnt != raster_num; ++cnt)
	{
		float y1 = base_line_y + cnt*25.0f;
		float y2 = y1 + 25.0f;;
		//确定在y1~y2间最大的x,存入对应的光栅结构中
		//求y1与线段的交点，定位目标点位置
		for (int idx = 0; idx != boundary.r_nums - 1; ++idx)
		{
			COOR2 & td1 = boundary.r_boundary[idx];
			COOR2 & td2 = boundary.r_boundary[idx + 1];
			//找到符合y1条件的点
			if (td1.y <= y1 && td2.y > y1)
				max_x_r = td1.x + (td1.x - td2.x) * (y1 - td1.y) / (td1.y - td2.y);
			else
				break;
		}
		//求y1与y2之间的点，并更新定位目标点位置
		for (int idx = 0; idx != boundary.r_nums; ++idx)
		{
			COOR2 & td = boundary.r_boundary[idx];
			if (td.y >= y1 && td.y < y2 && td.x < max_x_r)
				max_x_r = td.x;
		}
		//求y2与线段的交点，定位目标点位置
		for (int idx = 0; idx != boundary.r_nums - 1; ++idx)
		{
			COOR2 & td1 = boundary.r_boundary[idx];
			COOR2 & td2 = boundary.r_boundary[idx + 1];
			if (td1.y <= y2 && td2.y > y2)
			{
				float xx = td1.x + (td1.x - td2.x)*(y1 - td1.y) / (td1.y - td2.y);
				if (xx < max_x_r) max_x_r = xx;
			}
		}
		//将该栅格位置上的x锚点计算并以栅格坐标表示
		max_x_r += 2000.0f;//转换到栅格坐标
		raster_r[cnt] = int((max_x_r + 2000.0f) / MAP_ROI_CELL_SIZ);
	}

	//2 遍历光栅化边界，进行左右归边操作
	//2.1 左边线进行归边操作，边线据障碍物（膨胀后）的横向距离为0,1,2时，认为应该被归边
	for (int cnt = 0; cnt != raster_num; ++cnt)
	{
		int cur = 1;
		while (cur < 4)
		{
			if (raster_l[cnt] + cur >= MAP_64_CELL_HOR) break;//下一步测试越界，停止测试
			int lidar_pos = (base_line_offset + cnt)*MAP_64_CELL_HOR + raster_l[cnt];
			//继承上一次操作
			if (lidar64_map[lidar_pos + cur] > 0.0f)
			{
				raster_l[cnt] += cur;//测试命中障碍则更新，并重置测试游标
				cur = 1;
			}
			else
			{
				cur++;//测试没有命中，则自增测试游标
			}
		}
	}
	//2.2 右边线进行归边操作，边线障碍物（膨胀后）的横向距离为0,1,2时，认为应该被归边
	for (int cnt = 0; cnt != raster_num; ++cnt)
	{
		int cur = 1;
		while (cur < 4)
		{
			if (raster_r[cnt] - cur < 0) break;//下一步测试越界，停止测试
			int lidar_pos = (base_line_offset + cnt)*MAP_64_CELL_HOR + raster_r[cnt];
			//继承上一次操作
			if (lidar64_map[lidar_pos - cur] > 0.0f)
			{
				raster_l[cnt] -= cur;//测试命中障碍则更新，并重置测试游标
				cur = 1;
			}
			else
			{
				cur++;//测试没有命中，则自增测试游标
			}
		}
	}

	//3 对光栅化边线进行滤波优化，使其光滑
	static int cov_k[25] = {-7, -7, -6, -5, -4, -3, -2, -2, -1, -1, 0, 0, 0, -1, -1, -2, -2. -3, -4, -5, -6, -7, -7};
	
	return RET_OK;
}