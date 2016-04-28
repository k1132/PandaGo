#include "./TurnRound.h"
#include "./Cmorphin.h"
#include "string.h"
#include <cmath>
#include <cstdio>

/*==================================================================
 * 函数名  ：	CTurnRound::CTurnRound()
 * 功能    ：	构造函数，初始化所有变量
 * 输入参数：	
 * 返回值  ：	
 * 作者    ：	zgccmax@163.com
 * 日期    ：	2014.05.28
 * 修改记录：	
 *==================================================================*/
CTurnRound::CTurnRound()
{
	m_Action = MATCHING; //[模式匹配]
	m_ActionModel = MATCHING;

	m_bLog = false;
	m_bTurnRoundFinish = false;

	m_StartYaw = 0;
	m_CurYaw = 0;
	m_LastYaw = 0;
	m_AccumulationYaw = 0;

	m_MoveHeadSpeed = 0;
	m_MoveBackSpeed = 0;

	memset(m_grid_map, 0, sizeof(int) * GRID_HEIGHT * GRID_WIDTH);
	m_grid_center.x = 0;
	m_grid_center.y = 0;

	m_CarWidth = 0;
	m_SearchHeight = 0;;
	m_CarHeadSafeDist = 0;
	m_CarTailSafeDist = 0;
	m_TurnAngle = 0;
	m_ConfirmTimer = 0;

 	m_CarWidth_Cell = 0;

	m_SteerAdjustTimer = 0;

	memset(m_Path, 0, sizeof(COOR2) * NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	m_PathPtsNum = 0;
	m_Speed = 0;
	m_PredicamentCounter = 0;

	m_bInitialize = false;

	m_AccumulationDist = 0;
	memset(&m_LastPosition, 0, sizeof(COOR2));
	memset(&m_CurPosition, 0, sizeof(COOR2));
	m_AccumulationSwitch = 0;

	m_GoalYaw = 0;
	m_FinishYawThreshold = 0;
}

CTurnRound::~CTurnRound()
{

}

/*==================================================================
 * 函数名  ：	void CTurnRound::DefaultConfig()
 * 功能    ：	初始化默认配置，以防读取配置文件失败
 * 输入参数：	
 * 返回值  ：	
 * 作者    ：	zgccmax@163.com
 * 日期    ：	2014.05.28
 * 修改记录：	
 *==================================================================*/
void CTurnRound::DefaultConfig()
{
	m_CarWidth = 230;
	m_SearchHeight = 400;
	m_CarHeadSafeDist = 650;
	m_CarTailSafeDist = 350;
	m_CarTransverseDist = 400;
	m_TurnAngle = 165;
	m_ConfirmAngle = 140;
	m_MoveHeadSpeed = 5;
	m_MoveBackSpeed = 5;
}

/*==================================================================
 * 函数名  ：	void CTurnRound::TurnRoundInit(int syaw, bool blog)
 * 功能    ：	掉头模块初始化
 * 输入参数：	int syaw		启动掉头模块的起始航向角
 *				bool blog		日志功能开关
 * 返回值  ：	
 * 作者    ：	zgccmax@163.com
 * 日期    ：	2014.05.28
 * 修改记录：	
 *==================================================================*/
void CTurnRound::TurnRoundInit(int car_width, int syaw, bool blog)
{
	FILE *fp = NULL;
	char name[256] = "";
	int val = 0;

	m_bLog = blog;
	m_bTurnRoundFinish = false;

	m_CarWidth = car_width;
	m_StartYaw = syaw;
	m_LastYaw = syaw;
	if (m_bLog) MBUG("TurnRoundInit m_StartYaw : %d\n", m_StartYaw);

	DefaultConfig();

#ifdef __GNUC__
	if ((fp = fopen("/home/alv/ugv/bin/config/turnround.ini", "r")) == NULL)
	{
		MBUG("turnround.ini can't open\n");
		m_bInitialize = false;
		return;
	}
#else
	if ((fp = fopen("D:\\ALV\\PL\\Simulation\\turnround.ini", "r")) == NULL)
	{
		MBUG("turnround.ini can't open\n");
		m_bInitialize = false;
		return;
	}
#endif


	while ((fscanf(fp, "%s = %d", name, &val)) != EOF)
	{
		if (strcmp(name, "SearchHeight") == 0)
		{
			m_SearchHeight = val;
		}
		if (strcmp(name, "CarHeadSafeDist") == 0)
		{
			m_CarHeadSafeDist = val;
		}
		if (strcmp(name, "CarTailSafeDist") == 0)
		{
			m_CarTailSafeDist = val;
		}
		if (strcmp(name, "CarTransverseDist") == 0)
		{
			m_CarTransverseDist = val;
		}
		if (strcmp(name, "TurnAngle") == 0)
		{
			m_TurnAngle = val;
		}
		if (strcmp(name, "ConfirmAnlge") == 0)
		{
			m_ConfirmAngle = val;
		}
		if (strcmp(name, "MoveHeadSpeed") == 0)
		{
			m_MoveHeadSpeed = val;
		}
		if (strcmp(name, "MoveBackSpeed") == 0)
		{
			m_MoveBackSpeed = val;
		}
	}
	fclose(fp);

	//[物理坐标转栅格]
	m_CarWidth_Cell = m_CarWidth / GRID_LEN_PER_CELL;
	m_bInitialize = true;
}

/*==================================================================
* 函数名  ：	int CTurnRound::DoTurnRound(int frame, int yaw, int grid_map[GRID_HEIGHT][GRID_WIDTH], COOR2 grid_center)
* 功能    ：	
* 输入参数：	int frame		当前规划的帧号
*				int yaw			当前的航向角
*				int grid_map[GRID_HEIGHT][GRID_WIDTH]		当前的栅格地图
*				COOR2 grid_center							当前栅格地图的原点
* 返回值  ：	0 前进  1 倒车  2 停车  3 沿轨迹倒车
* 作者    ：	zgccmax@163.com
* 日期    ：	2014.05.28
* 修改记录：	
*==================================================================*/
int CTurnRound::DoTurnRound(int frame, int yaw, COOR2 pos, int grid_map[GRID_HEIGHT][GRID_WIDTH], COOR2 grid_center, NATURAL_BOUNDARY natural_boundary)
{
	int ret = MODE_FORWARD;
	if (m_bLog) MBUG("DoTurnRound Start **********************\n");

	//[更新数据]
	m_Frame = frame;
	m_CurYaw = yaw;
	m_CurPosition = pos;
	memcpy(m_grid_map, grid_map, sizeof(int) * GRID_WIDTH * GRID_HEIGHT);
	m_grid_center = grid_center;
	memcpy(&m_natural_boundary, &natural_boundary, sizeof(NATURAL_BOUNDARY));

	//[执行动作]
	switch (m_Action)
	{
	case MATCHING:
		ret = ModelMatch();
		if (ret == 3)//[沿轨迹倒车]
			return ret;
		break;

	case REVERSE:
		ret = Reverse();
		if (ret == 0)
			ret = 2;
		else
			ret = 3;
		break;

	case LACTION1:
		ret = DoLAction1();
		break;
	case LACTION2:
		ret = DoLAction2();
		break;
	case LACTION3:
		ret = DoLAction3();
		break;


	case RACTION1:
		ret = DoRAction1();
		break;
	case RACTION2:
		ret = DoRAction2();
		break;
	case RACTION3:
		ret = DoRAction3();
		break;
	default:
		;
	}

	if (m_PredicamentCounter >= 10)
	{//[陷入困境]
		if (m_bLog) MBUG("DoTurnRound Error : Be Traped in Predicament！！\n");
		ret = MODE_STOP;;
	}

	m_LastYaw = m_CurYaw;
	return ret;
}

/*==================================================================
* 函数名  ：	int CTurnRound::ModelMatch()
* 功能    ：	进行行为模式匹配
* 输入参数：	
* 返回值  ：	0  可以匹配到一个可执行模式  -1  无法匹配到可执行模式  3 沿轨迹倒车
* 作者    ：	zgccmax@163.com
* 日期    ：	2014.05.24
* 修改记录：	
*==================================================================*/
int CTurnRound::ModelMatch()
{
	int ret = 0;

	//[1.检测前方距离是否足够]
	ret = DetectStartDsit();
	if (ret == -2)
	{//[进行倒车规划，目前尚未实现]
		m_Action = REVERSE;
		ret = 3;
		return ret;
	}

	//[2.模式判断]
	if (ret == 0)
	{//[优先左拐掉头]
		m_Action = LACTION1;
		m_SteerAdjustTimer = 10;//[1s钟调整方向盘]
		m_PredicamentCounter = 0;//[困境计数器初始化]

		TurnRoundAction(135, m_Path, m_PathPtsNum);
	}
	else if (ret == 1)
	{//[右拐掉头]
		m_Action = RACTION1;
		m_SteerAdjustTimer = 10;//[1s钟调整方向盘]
		m_PredicamentCounter = 0;//[困境计数器初始化]

		TurnRoundAction(45, m_Path, m_PathPtsNum);
	}
	else
	{
		//[ret == -1  什么都不做]
	}

	return ret;
}

/*==================================================================
 * 函数名  ：	int CTurnRound::DetectHeadStartDsit()
 * 功能    ：	启动时前方安全距离检测
 * 输入参数：	
 * 返回值  ：	0  进行左拐掉头  1  进行右拐掉头  -1  没有足够横向距离  -2  不安全需要倒车
 * 作者    ：	zgccmax@163.com
 * 日期    ：	2014.05.28
 * 修改记录：	2014.06.14  修改碰撞检测策略
 *==================================================================*/
int CTurnRound::DetectStartDsit()
{
	int i, j;
	int ret = 0;

	int flagl = 0;//[左拐堵塞标志]
	int flagr = 0;//[右拐堵塞标志]
	double ldist = 0;
	double rdist = 0;
	int ldist_h = 0;//[横向距离]
	int rdist_h = 0;//[横向距离]

	COOR2 pts[200];
	int pts_num = 0;
	int x, y;
	double xx, yy;
	double dist2obs;
	int obs_flag = 0;

	//[1.5f 分两部分，整数部分是保证半车长多一个栅格，小数部分保证转int型时四舍五入]
	int cwidth = 4;//(int)(m_CarWidth_Cell * 0.5 + 0.5f);

	//[1.检测前方]
// 	obs_flag = 0;
// 	create_line_by_theta(90, pts, pts_num, 0);
// 	for (i=0; i<pts_num;i++)
// 	{
// 		x = pts[i].x / GRID_LEN_PER_CELL + (m_grid_center.x - 1);
// 		y = pts[i].y / GRID_LEN_PER_CELL + m_grid_center.y;
// 
// 		for (j=x-cwidth;j<=x+cwidth;j++)
// 		{
// 			if (m_grid_map[y][j] == OBS_FLAG)
// 			{
// 				obs_flag = 1;
// 				break;
// 			}
// 		}
// 
// 		if (obs_flag)
// 			break;
// 	}
// 
// 	if (obs_flag == 0)
// 		i--;
// 	xx = pts[i].x;
// 	yy = pts[i].y - 400;
// 	dist2obs = sqrt(xx*xx+yy*yy);
// 	if (dist2obs < m_SearchHeight)
// 	{
// 		if (m_bLog) MBUG("DetectStartDsit Unsafe H: %f\n", dist2obs);
// 		ret = -2;
// 		return ret;
// 	}

	//[2.检测左前方]
	obs_flag = 0;
	create_line_by_theta(135, pts, pts_num, 0);
	for (i=0; i<pts_num;i++)
	{
		x = pts[i].x / GRID_LEN_PER_CELL + (m_grid_center.x - 1);
		y = pts[i].y / GRID_LEN_PER_CELL + m_grid_center.y;

		for (j=x-cwidth;j<=x+cwidth;j++)
		{
			if (m_grid_map[y][j] == OBS_FLAG)
			{
				obs_flag = 1;
				break;
			}
		}

		if (obs_flag)
			break;
	}

	if (obs_flag == 0)
		i--;
	xx = pts[i].x;
	yy = pts[i].y - 400;
	//ldist = xx;
	dist2obs = sqrt(xx*xx+yy*yy);
	ldist_h = (int)abs(xx);
	ldist = dist2obs;
	if (dist2obs < m_CarHeadSafeDist)
	{
		if (m_bLog) MBUG("DetectStartDsit Unsafe LH: %f\n", dist2obs);
		flagl = 1;
	}


	//[3.检测右前方]
	obs_flag = 0;
	create_line_by_theta(45, pts, pts_num, 0);
	for (i=0; i<pts_num;i++)
	{
		x = pts[i].x / GRID_LEN_PER_CELL + (m_grid_center.x -1);
		y = pts[i].y / GRID_LEN_PER_CELL + m_grid_center.y;

		for (j=x-cwidth;j<=x+cwidth;j++)
		{
			if (m_grid_map[y][j] == OBS_FLAG)
			{
				obs_flag = 1;
				break;
			}
		}

		if (obs_flag)
			break;
	}

	if (obs_flag == 0)
		i--;
	xx = pts[i].x;
	yy = pts[i].y - 400;
	//rdist = xx;
	rdist_h = (int)abs(xx);
	dist2obs = sqrt(xx*xx+yy*yy);
	rdist = dist2obs;
	if (dist2obs < m_CarHeadSafeDist)
	{
		if (m_bLog) MBUG("DetectStartDsit Unsafe RH: %f\n", dist2obs);
		flagr = 1;
	}

	if (ldist_h < m_CarTransverseDist)
		flagl = 1;
	if (rdist_h < m_CarTransverseDist)
		flagr = 1;

	if (flagl == 0 && flagr == 0)
		//fabs(ldist) >= rdist ? ret = 0 : ret = 1;
		ldist >= rdist ? ret = 0 : ret = 1;
	else if (flagl == 0)
		ret = 0;
	else if (flagr == 0)
		ret = 1;
	else
	{
		if (m_bLog) MBUG("ModelMatch Error : No enough transverse distance r\n");
		ret = -1;//[左拐和右拐都不可以才返回-1]
	}
	return ret;
}

/*==================================================================
 * 函数名  ：	int CTurnRound::Reverse()
 * 功能    ：	
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	int			0  停止倒车  -1  继续倒车
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int CTurnRound::Reverse()
{
	int ret = 0;

	//[1.检测前方距离是否足够]
	ret = DetectStartDsit();
	if (ret == -2)
	{//[前方距离不够，继续倒车]
		m_Action = REVERSE;
		ret = -1;
		return ret;
	}
	else
	{
		m_Action = MATCHING;
		ret = 0;
	}

	return ret;
}

/*==================================================================
* 函数名  ：	int CTurnRound::DoLAction1()
* 功能    ：	左拐掉头第一个动作  前进
* 输入参数：	
* 返回值  ：	MODE_FORWARD  正常前进  MODE_STOP  停车
* 作者    ：	zgccmax@163.com
* 日期    ：	2014.05.28
* 修改记录：	
*==================================================================*/
int CTurnRound::DoLAction1()
{
	int ret = MODE_FORWARD;

	if (m_AccumulationSwitch == 0)
	{
		m_AccumulationSwitch = 1;
		m_LastPosition = m_CurPosition;
		m_AccumulationDist = 0;
	}
	else
	{
		double xx = m_CurPosition.x - m_LastPosition.x;
		double yy = m_CurPosition.y - m_LastPosition.y;
		m_LastPosition = m_CurPosition;
		m_AccumulationDist += sqrt(xx * xx + yy * yy);

		if (m_AccumulationDist >= 800)
		{
			m_AccumulationSwitch = 0;
			m_AccumulationDist = 0;

			m_PredicamentCounter++;
			m_SteerAdjustTimer = 10;
			m_Action = LACTION2;
			return MODE_STOP;
		}
	}


	//[1.检测车前是否安全]
	ret = DetectHead(135);
	if (ret == -1)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_PredicamentCounter++;
		m_SteerAdjustTimer = 10;
		m_Action = LACTION2;
		return MODE_STOP;
	}
	else
		m_PredicamentCounter = 0;

	//[2.执行掉头]
	TurnRoundAction(135, m_Path, m_PathPtsNum);

	//[3.方向盘计时器运行]
	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
		m_SteerAdjustTimer = 0;

	//[4.判断航向角，进行状态切换]
	CalculateYaw();
	if (m_AccumulationYaw >= m_TurnAngle)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_SteerAdjustTimer = 15;
		m_Action = LACTION3;
	}

	//[5.判断是否有道路，进行状态切换]
	if (ConfirmRoad() == 0)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_SteerAdjustTimer = 15;
		m_Action = LACTION3;
	}

	return ret;
}

/*==================================================================
* 函数名  ：	int CTurnRound::DoLAction2()
* 功能    ：	左拐掉头第二个动作  倒车
* 输入参数：	
* 返回值  ：	MODE_REVERSE 正常倒车  MODE_STOP  停车
* 作者    ：	zgccmax@163.com
* 日期    ：	2014.05.28
* 修改记录：	
*==================================================================*/
int CTurnRound::DoLAction2()
{
	int ret = MODE_REVERSE;

	if (m_AccumulationSwitch == 0)
	{
		m_AccumulationSwitch = 1;
		m_LastPosition = m_CurPosition;
		m_AccumulationDist = 0;
	}
	else
	{
		double xx = m_CurPosition.x - m_LastPosition.x;
		double yy = m_CurPosition.y - m_LastPosition.y;
		m_LastPosition = m_CurPosition;
		m_AccumulationDist += sqrt(xx * xx + yy * yy);

		if (m_AccumulationDist >= 800)
		{
			m_AccumulationSwitch = 0;
			m_AccumulationDist = 0;

			m_PredicamentCounter++;
			m_SteerAdjustTimer = 10;
			m_Action = LACTION1;
			return MODE_STOP;
		}
	}

	CalculateYaw();

	//[1.检测车尾是否安全]
	ret = DetectTail(45);
	if (ret == -1)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		if (m_AccumulationYaw >= m_TurnAngle)
		{//[角度满足直接进入第三个状态]
			m_PredicamentCounter = 0;
			m_SteerAdjustTimer = 15;
			m_Action = LACTION3;
		}
		else
		{
			m_PredicamentCounter++;
			m_SteerAdjustTimer = 10;
			m_Action = LACTION1;
		}
		return MODE_STOP;
	}
	else
		m_PredicamentCounter = 0;

	//[2.执行掉头]
	TurnRoundActionBack(45, m_Path, m_PathPtsNum);

	//[3.方向盘计时器运行]
	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
		m_SteerAdjustTimer = 0;

	//[4.判断航向角，进行状态切换]
	if (m_AccumulationYaw >= m_TurnAngle)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_SteerAdjustTimer = 15;
		m_Action = LACTION3;
	}

	//[5.判断是否有道路，进行状态切换]
	if (ConfirmRoad() == 0)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_SteerAdjustTimer = 15;
		m_Action = LACTION3;
	}

	return ret;
}

/*==================================================================
 * 函数名  ：	int CTurnRound::DoLAction3()
 * 功能    ：	左拐掉头结束时方向盘调整
 * 输入参数：	
 * 返回值  ：	MODE_STOP  停车
 * 作者    ：	zgccmax@163.com
 * 日期    ：	2014.05.28
 * 修改记录：	
 *==================================================================*/
int CTurnRound::DoLAction3()
{
	int i;

	if (m_SteerAdjustTimer > 0)
	{
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			m_Path[i].x = 0;
			m_Path[i].y = i * 100;
		}
		m_PathPtsNum = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
	}

	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
	{//[结束掉头]
		m_SteerAdjustTimer = 0;
		m_bTurnRoundFinish = true;
	}

	return MODE_STOP;
}

/*==================================================================
* 函数名  ：	int CTurnRound::DoRAction1()
* 功能    ：	右拐掉头第一个动作  前进
* 输入参数：	
* 返回值  ：	MODE_FORWARD  正常前进  MODE_STOP  停车
* 作者    ：	zgccmax@163.com
* 日期    ：	2014.05.28
* 修改记录：	
 *==================================================================*/
int CTurnRound::DoRAction1()
{
	int ret = MODE_FORWARD;

	if (m_AccumulationSwitch == 0)
	{
		m_AccumulationSwitch = 1;
		m_LastPosition = m_CurPosition;
		m_AccumulationDist = 0;
	}
	else
	{
		double xx = m_CurPosition.x - m_LastPosition.x;
		double yy = m_CurPosition.y - m_LastPosition.y;
		m_LastPosition = m_CurPosition;
		m_AccumulationDist += sqrt(xx * xx + yy * yy);

		if (m_AccumulationDist >= 800)
		{
			m_AccumulationSwitch = 0;
			m_AccumulationDist = 0;

			m_PredicamentCounter++;
			m_SteerAdjustTimer = 10;
			m_Action = RACTION2;
			return MODE_STOP;
		}
	}

	//[1.检测车前是否安全]
	ret = DetectHead(45);
	if (ret == -1)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_PredicamentCounter++;
		m_SteerAdjustTimer = 10;
		m_Action = RACTION2;
		return MODE_STOP;
	}
	else
		m_PredicamentCounter = 0;

	//[2.执行掉头]
	TurnRoundAction(45, m_Path, m_PathPtsNum);

	//[3.方向盘计时器运行]
	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
		m_SteerAdjustTimer = 0;

	//[4.判断航向角，进行状态切换]
	CalculateYaw();
	if (m_AccumulationYaw >= m_TurnAngle)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_SteerAdjustTimer = 15;
		m_Action = RACTION3;
	}

	//[5.判断是否有道路，进行状态切换]
	if (ConfirmRoad() == 0)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_SteerAdjustTimer = 15;
		m_Action = LACTION3;
	}

	return ret;
}

/*==================================================================
* 函数名  ：	int CTurnRound::DoRAction2()
* 功能    ：	右拐掉头第二个动作  倒车
* 输入参数：	
* 返回值  ：	MODE_REVERSE 正常倒车  MODE_STOP  停车
* 作者    ：	zgccmax@163.com
* 日期    ：	2014.05.28
* 修改记录：	
*==================================================================*/
int CTurnRound::DoRAction2()
{
	int ret = MODE_REVERSE;

	if (m_AccumulationSwitch == 0)
	{
		m_AccumulationSwitch = 1;
		m_LastPosition = m_CurPosition;
		m_AccumulationDist = 0;
	}
	else
	{
		double xx = m_CurPosition.x - m_LastPosition.x;
		double yy = m_CurPosition.y - m_LastPosition.y;
		m_LastPosition = m_CurPosition;
		m_AccumulationDist += sqrt(xx * xx + yy * yy);

		if (m_AccumulationDist >= 800)
		{
			m_AccumulationSwitch = 0;
			m_AccumulationDist = 0;

			m_PredicamentCounter++;
			m_SteerAdjustTimer = 10;
			m_Action = RACTION1;
			return MODE_STOP;
		}
	}

	CalculateYaw();

	//[1.检测车尾是否安全]
	ret = DetectTail(135);
	if (ret == -1)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		if (m_AccumulationYaw >= m_TurnAngle)
		{//[角度满足直接进入第三个状态]
			m_PredicamentCounter = 0;
			m_SteerAdjustTimer = 15;
			m_Action = RACTION3;
		}
		else
		{
			m_PredicamentCounter++;
			m_SteerAdjustTimer = 10;
			m_Action = RACTION1;
		}
		return MODE_STOP;
	}
	else
		m_PredicamentCounter = 0;

	//[2.执行掉头]
	TurnRoundActionBack(135, m_Path, m_PathPtsNum);

	//[3.方向盘计时器运行]
	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
		m_SteerAdjustTimer = 0;

	//[4.判断航向角，进行状态切换]
	if (m_AccumulationYaw >= m_TurnAngle)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_SteerAdjustTimer = 15;
		m_Action = RACTION3;
	}

	//[5.判断是否有道路，进行状态切换]
	if (ConfirmRoad() == 0)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_SteerAdjustTimer = 15;
		m_Action = LACTION3;
	}

	return ret;
}

/*==================================================================
 * 函数名  ：	int CTurnRound::DoRAction3()
 * 功能    ：	右拐掉头结束时方向盘调整
 * 输入参数：	
 * 返回值  ：	MODE_STOP  停车
 * 作者    ：	zgccmax@163.com
 * 日期    ：	2014.05.28
 * 修改记录：	
 *==================================================================*/
int CTurnRound::DoRAction3()
{
	int i;

	if (m_SteerAdjustTimer > 0)
	{
		for (i=0; i<NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			m_Path[i].x = 0;
			m_Path[i].y = i * 100;
		}
		m_PathPtsNum = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
	}

	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
	{
		m_SteerAdjustTimer = 0;
		m_bTurnRoundFinish = true;
	}

	return MODE_STOP;
}

/*==================================================================
* 函数名  ：	void CTurnRound::SetActionResult(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, int mode)
* 功能    ：	设置执行参数
* 输入参数：	PL_FUNC_INPUT *pl_input		规划的输入
*				PL_CS_DATA *pl_cs_data		发送给底层
*				int mode					MODE_FORWARD 正常前进  MODE_REVERSE 正常倒车  MODE_STOP 正常停车
* 返回值  ：	
* 作者    ：	zgccmax@163.com
* 日期    ：	2014.05.28
* 修改记录：	
*==================================================================*/
void CTurnRound::SetActionResult(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, int mode)
{
	memset(pl_cs_data, 0, sizeof(PL_CS_DATA));

	switch (mode)
	{
	case MODE_FORWARD:
		pl_cs_data->id = m_Frame;
		pl_cs_data->isok = 1;
		memcpy(pl_cs_data->path, m_Path, sizeof(COOR2) * m_PathPtsNum);
		pl_cs_data->number_of_effective_points = m_PathPtsNum;

		if (m_SteerAdjustTimer > 0)
			pl_cs_data->speed = 0;
		else
			pl_cs_data->speed = (int)(m_MoveHeadSpeed * CM);

		pl_cs_data->state = pl_input->state;
		if (m_Action == LACTION1)
			pl_cs_data->sys_command = SYS_COMMAND_LEFT;
		else if (m_Action == RACTION1)
			pl_cs_data->sys_command = SYS_COMMAND_RIGHT;
		else
			pl_cs_data->sys_command = 0;
			
		break;
	case MODE_REVERSE:
		pl_cs_data->id = m_Frame;
		pl_cs_data->isok = 1;
		memcpy(pl_cs_data->path, m_Path, sizeof(COOR2) * m_PathPtsNum);
		pl_cs_data->number_of_effective_points = m_PathPtsNum;

		if (m_SteerAdjustTimer > 0)
			pl_cs_data->speed = 0;
		else
			pl_cs_data->speed = (int)(m_MoveBackSpeed * CM);

		pl_cs_data->state = pl_input->state;

		if (m_Action == LACTION2)
			pl_cs_data->sys_command = SYS_COMMAND_REV | SYS_COMMAND_RIGHT;
		else if (m_Action == RACTION2)
			pl_cs_data->sys_command = SYS_COMMAND_REV | SYS_COMMAND_LEFT;
		else
			pl_cs_data->sys_command = SYS_COMMAND_REV | SYS_COMMAND_RIGHT | SYS_COMMAND_LEFT;
		break;
	case MODE_STOP:
		pl_cs_data->id = m_Frame;
		pl_cs_data->isok = 1;
		memcpy(pl_cs_data->path, m_Path, sizeof(COOR2) * m_PathPtsNum);
		pl_cs_data->number_of_effective_points = m_PathPtsNum;
		pl_cs_data->speed = 0;
		pl_cs_data->state = pl_input->state;
		pl_cs_data->sys_command = SYS_COMMAND_LEFT | SYS_COMMAND_RIGHT;
		break;
	default:
		;
	}

	if (m_bLog) MBUG("SetActionResult Finished **********************\n");
}

/*==================================================================
 * 函数名  ：	int CTurnRound::DetectHead(double angle)
 * 功能    ：	检测前进区域（左拐区域，右拐区域）是否安全
 * 输入参数：	double angle		前进时前轮转向角
 * 返回值  ：	MODE_FORWARD  安全  -1 不安全
 * 作者    ：	zgccmax@163.com
 * 日期    ：	2014.05.28
 * 修改记录：	
 *==================================================================*/
int CTurnRound::DetectHead(double angle)
{
	int i, j;
	int ret = 0;

	COOR2 pts[200];
	int pts_num = 0;
	int x, y;
	double xx, yy;
	double dist2obs;
	int obs_flag = 0;

	//[1.5f 分两部分，整数部分是保证半车长多一个栅格，小数部分保证转int型时四舍五入]
	int cwidth = 4;//(int)(m_CarWidth_Cell * 0.5 + 1.5f);

	//[检测左前]
	if (m_Action == LACTION1)
	{
		create_line_by_theta(angle, pts, pts_num, 0);
		for (i=0; i<pts_num;i++)
		{
			x = pts[i].x / GRID_LEN_PER_CELL + (m_grid_center.x - 1);
			y = pts[i].y / GRID_LEN_PER_CELL + m_grid_center.y;

			for (j=x-cwidth;j<=x+cwidth;j++)
			{
				if (m_grid_map[y][j] == OBS_FLAG)
				{
					obs_flag = 1;
					break;
				}
			}

			if (obs_flag)
				break;
		}

		if (obs_flag == 0)
			i--;
		xx = pts[i].x;
		yy = pts[i].y - 400;
		dist2obs = sqrt(xx*xx+yy*yy);
		if (dist2obs < m_CarHeadSafeDist)
		{
			if (m_bLog) MBUG("LACTION1 DetectHead Unsafe LH: %f\n", dist2obs);
			ret = -1;
			return ret;
		}
	}

	if (m_Action == RACTION1)
	{
		create_line_by_theta(angle, pts, pts_num, 0);
		for (i=0; i<pts_num;i++)
		{
			x = pts[i].x / GRID_LEN_PER_CELL + (m_grid_center.x - 1);
			y = pts[i].y / GRID_LEN_PER_CELL + m_grid_center.y;

			for (j=x-cwidth;j<=x+cwidth;j++)
			{
				if (m_grid_map[y][j] == OBS_FLAG)
				{
					obs_flag = 1;
					break;
				}
			}

			if (obs_flag)
				break;
		}

		if (obs_flag == 0)
			i--;
		xx = pts[i].x;
		yy = pts[i].y - 400;
		dist2obs = sqrt(xx*xx+yy*yy);
		if (dist2obs < m_CarHeadSafeDist)
		{
			if (m_bLog) MBUG("RACTION1 DetectHead Unsafe RH: %f\n", dist2obs);
			ret = -1;
			return ret;
		}
	}
	return ret;
}

/*==================================================================
 * 函数名  ：	int CTurnRound::DetectTail(double angle)
 * 功能    ：	检测倒车区域（左拐区域，右拐区域）是否安全
 * 输入参数：	double angle		倒车时前轮转向角
 * 返回值  ：	MODE_REVERSE  安全  -1 不安全
 * 作者    ：	zgccmax@163.com
 * 日期    ：	2014.05.28
 * 修改记录：	
 *==================================================================*/
int CTurnRound::DetectTail(double angle)
{
	int i, j;
	int ret = MODE_REVERSE;

	COOR2 pts[200];
	int pts_num = 0;
	int x, y;
	double xx, yy;
	double dist2obs;
	int obs_flag = 0;

	//[1.5f 分两部分，整数部分是保证半车长多一个栅格，小数部分保证转int型时四舍五入]
	int cwidth = 4;//(int)(m_CarWidth_Cell * 0.5 + 1.5f);

	//[检测左后]
	if (m_Action == RACTION2)
	{
		create_line_by_theta(angle, pts, pts_num, 2);
		for (i=0; i<pts_num;i++)
		{
			x = pts[i].x / GRID_LEN_PER_CELL + (m_grid_center.x - 1);
			y = pts[i].y / GRID_LEN_PER_CELL + m_grid_center.y;

			if (y <= 0)//[防止越界]
				break;

			for (j=x-cwidth;j<=x+cwidth;j++)
			{
				if (m_grid_map[y][j] == OBS_FLAG)
				{
					obs_flag = 1;
					break;
				}
			}

			if (obs_flag)
				break;
		}

		if (obs_flag == 0)
			i--;
		xx = pts[i].x;
		yy = pts[i].y;
		dist2obs = sqrt(xx*xx+yy*yy);
		if (dist2obs < m_CarTailSafeDist)
		{
			if (m_bLog) MBUG("DetectHead Unsafe LT: %f\n", dist2obs);
			ret = -1;
			return ret;
		}
	}

	if (m_Action == LACTION2)
	{
		create_line_by_theta(angle, pts, pts_num, 2);
		for (i=0; i<pts_num;i++)
		{
			x = pts[i].x / GRID_LEN_PER_CELL + (m_grid_center.x - 1);
			y = pts[i].y / GRID_LEN_PER_CELL + m_grid_center.y;

			if (y <= 0)//[防止越界]
				break;

			for (j=x-cwidth;j<=x+cwidth;j++)
			{
				if (m_grid_map[y][j] == OBS_FLAG)
				{
					obs_flag = 1;
					break;
				}
			}

			if (obs_flag)
				break;
		}

		if (obs_flag == 0)
			i--;
		xx = pts[i].x;
		yy = pts[i].y;
		dist2obs = sqrt(xx*xx+yy*yy);
		if (dist2obs < m_CarTailSafeDist)
		{
			if (m_bLog) MBUG("DetectHead Unsafe RT: %f\n", dist2obs);
			ret = -1;
			return ret;
		}
	}

	return ret;
}

/*==================================================================
 * 函数名  ：	void CTurnRound::CalculateYaw()
 * 功能    ：	进行航向角的累加
 * 输入参数：	
 * 返回值  ：	
 * 作者    ：	zgccmax@163.com
 * 日期    ：	2014.05.28
 * 修改记录：	
 *==================================================================*/
void CTurnRound::CalculateYaw()
{
	//[本函数里面对转过0度时，使用180这个值来进行判断，是考虑实际运动过程中，车子的航向角不可能发生180的突变]
	int deltayaw;
	deltayaw = m_CurYaw - m_LastYaw;
	switch (m_Action)
	{//[逆时针累加]
	case LACTION1:
	case LACTION2:
		if (deltayaw >= 0)
		{
			if (deltayaw > 180)//[顺时针转过0度]
				deltayaw -= 360;
			m_AccumulationYaw += deltayaw;
		}
		else
		{
			if (deltayaw < -180)//[逆时针转过0度]
				deltayaw += 360;
			m_AccumulationYaw += deltayaw;
		}
		break;

		//[顺时针累加]
	case RACTION1:
	case RACTION2:
		if (deltayaw >= 0)
		{
			if (deltayaw > 180)
				deltayaw = 360 - deltayaw;//[顺时针转过0度]
			else
				deltayaw = -deltayaw;
			m_AccumulationYaw += deltayaw;
		}
		else
		{
			if (deltayaw < -180)
				deltayaw = deltayaw - 360;//[逆时针转过0度]
			else
				deltayaw = -deltayaw;
			m_AccumulationYaw += deltayaw;
		}
		break;
	default:
		;
	}

	if (m_bLog) MBUG("m_AccumulationYaw : %d\n", m_AccumulationYaw);
}

/*==================================================================
 * 函数名  ：	bool CTurnRound::TurnRoundFinish()
 * 功能    ：	返回是否完成掉头动作
 * 输入参数：	
 * 返回值  ：	true  完成  false  没有完成
 * 作者    ：	zgccmax@163.com
 * 日期    ：	2014.05.28
 * 修改记录：	
 *==================================================================*/
bool CTurnRound::TurnRoundFinish()
{
	return m_bTurnRoundFinish;
}

int CTurnRound::GetAction()
{
	return m_Action;
}

bool CTurnRound::GetInitialStatus()
{
	return m_bInitialize;
}

int CTurnRound::ConfirmRoad()
{
	int ret = -1;
	if (m_AccumulationYaw >= m_ConfirmAngle)
	{
		if (m_natural_boundary.l_fidelity >= 60 && m_natural_boundary.r_fidelity >= 60)
			m_ConfirmTimer++;
		else
			m_ConfirmTimer = 0;

		if (m_ConfirmTimer >= 3)
			ret = 0;
	}

	return ret;
}

void CTurnRound::RightTurnRoundInit(int goal_yaw, int finish_yaw_threshold, int mode)
{
	DefaultConfig();
	m_GoalYaw = goal_yaw;
	m_FinishYawThreshold = finish_yaw_threshold;
	m_bTurnRoundFinish = false;
	m_bInitialize = true;
	switch (mode)
	{
	case FORWARD_FIRST:
		m_Action = RACTION1;
		break;
	case BACK_FIRST:
		m_Action = RACTION2;
		break;
	default:
		break;
	}
}

/*==================================================================
 * 函数名  ：	
 * 功能    ：	
 * 输入参数：	
 * 输出参数：	
 * 返回值  ：	
 * 作者    ：	
 * 日期    ：	
 * 修改记录：	
 *==================================================================*/
int CTurnRound::DoRightTurnRound(int frame, int yaw, int grid_map[GRID_HEIGHT][GRID_WIDTH], COOR2 grid_center)
{
	int ret = MODE_FORWARD;

	//[更新数据]
	m_Frame = frame;
	m_CurYaw = yaw;
	memcpy(m_grid_map, grid_map, sizeof(int) * GRID_WIDTH * GRID_HEIGHT);
	m_grid_center = grid_center;

	//[执行动作]
	switch (m_Action)
	{
	case RACTION1:
		ret = DoRTRA1();
		break;
	case RACTION2:
		ret = DoRTRA2();
		break;
	case RACTION3:
		ret = DoRTRA3();
		break;
	default:
		;
	}

	if (m_PredicamentCounter >= 10)
	{//[陷入困境]
		if (m_bLog) MBUG("DoTurnRound Error : Be Traped in Predicament！！\n");
		ret = MODE_STOP;
	}

	return ret;
}

int CTurnRound::DoRTRA1()
{
	int ret = MODE_FORWARD;
	//[1.检测车前是否安全]
	ret = DetectHead(45);
	if (ret == -1)
	{
		m_PredicamentCounter++;
		m_SteerAdjustTimer = 10;
		m_Action = RACTION2;
		return MODE_STOP;
	}
	else
		m_PredicamentCounter = 0;

	//[2.执行掉头]
	TurnRoundAction(45, m_Path, m_PathPtsNum);

	//[3.方向盘计时器运行]
	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
		m_SteerAdjustTimer = 0;

	//[4.判断航向角，进行状态切换]
	int delta_yaw = m_CurYaw - m_GoalYaw;
	CHECK_YAW(delta_yaw);
	if (abs(delta_yaw) <= m_FinishYawThreshold)
	{
		m_SteerAdjustTimer = 15;
		m_Action = RACTION3;
		ret = MODE_FORWARD;
		return ret;
	}

	return ret;
}

int CTurnRound::DoRTRA2()
{
	int ret = MODE_REVERSE;

	int delta_yaw = m_CurYaw - m_GoalYaw;
	CHECK_YAW(delta_yaw);
	if (abs(delta_yaw) <= m_FinishYawThreshold)
	{
		m_SteerAdjustTimer = 15;
		m_Action = RACTION3;
		ret = MODE_FORWARD;
		return ret;
	}

	//[1.检测车尾是否安全]
	ret = DetectTail(135);
	if (ret == -1)
	{
		m_PredicamentCounter++;
		m_SteerAdjustTimer = 10;
		m_Action = RACTION1;
		return MODE_STOP;
	}
	else
		m_PredicamentCounter = 0;

	//[2.执行掉头]
	TurnRoundActionBack(135, m_Path, m_PathPtsNum);

	//[3.方向盘计时器运行]
	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
		m_SteerAdjustTimer = 0;

	return ret;
}

int CTurnRound::DoRTRA3()
{
	int i;

	if (m_SteerAdjustTimer > 0)
	{
		for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
		{
			m_Path[i].x = 0;
			m_Path[i].y = i * 100;
		}
		m_PathPtsNum = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;
	}

	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
	{
		m_SteerAdjustTimer = 0;
		m_bTurnRoundFinish = true;
	}

	return MODE_STOP;
}