#include "./TurnRound.h"
#include "./Cmorphin.h"
#include "string.h"
#include <cmath>
#include <cstdio>

/*==================================================================
 * ������  ��	CTurnRound::CTurnRound()
 * ����    ��	���캯������ʼ�����б���
 * ���������	
 * ����ֵ  ��	
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.05.28
 * �޸ļ�¼��	
 *==================================================================*/
CTurnRound::CTurnRound()
{
	m_Action = MATCHING; //[ģʽƥ��]
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
 * ������  ��	void CTurnRound::DefaultConfig()
 * ����    ��	��ʼ��Ĭ�����ã��Է���ȡ�����ļ�ʧ��
 * ���������	
 * ����ֵ  ��	
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.05.28
 * �޸ļ�¼��	
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
 * ������  ��	void CTurnRound::TurnRoundInit(int syaw, bool blog)
 * ����    ��	��ͷģ���ʼ��
 * ���������	int syaw		������ͷģ�����ʼ�����
 *				bool blog		��־���ܿ���
 * ����ֵ  ��	
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.05.28
 * �޸ļ�¼��	
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

	//[��������תդ��]
	m_CarWidth_Cell = m_CarWidth / GRID_LEN_PER_CELL;
	m_bInitialize = true;
}

/*==================================================================
* ������  ��	int CTurnRound::DoTurnRound(int frame, int yaw, int grid_map[GRID_HEIGHT][GRID_WIDTH], COOR2 grid_center)
* ����    ��	
* ���������	int frame		��ǰ�滮��֡��
*				int yaw			��ǰ�ĺ����
*				int grid_map[GRID_HEIGHT][GRID_WIDTH]		��ǰ��դ���ͼ
*				COOR2 grid_center							��ǰդ���ͼ��ԭ��
* ����ֵ  ��	0 ǰ��  1 ����  2 ͣ��  3 �ع켣����
* ����    ��	zgccmax@163.com
* ����    ��	2014.05.28
* �޸ļ�¼��	
*==================================================================*/
int CTurnRound::DoTurnRound(int frame, int yaw, COOR2 pos, int grid_map[GRID_HEIGHT][GRID_WIDTH], COOR2 grid_center, NATURAL_BOUNDARY natural_boundary)
{
	int ret = MODE_FORWARD;
	if (m_bLog) MBUG("DoTurnRound Start **********************\n");

	//[��������]
	m_Frame = frame;
	m_CurYaw = yaw;
	m_CurPosition = pos;
	memcpy(m_grid_map, grid_map, sizeof(int) * GRID_WIDTH * GRID_HEIGHT);
	m_grid_center = grid_center;
	memcpy(&m_natural_boundary, &natural_boundary, sizeof(NATURAL_BOUNDARY));

	//[ִ�ж���]
	switch (m_Action)
	{
	case MATCHING:
		ret = ModelMatch();
		if (ret == 3)//[�ع켣����]
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
	{//[��������]
		if (m_bLog) MBUG("DoTurnRound Error : Be Traped in Predicament����\n");
		ret = MODE_STOP;;
	}

	m_LastYaw = m_CurYaw;
	return ret;
}

/*==================================================================
* ������  ��	int CTurnRound::ModelMatch()
* ����    ��	������Ϊģʽƥ��
* ���������	
* ����ֵ  ��	0  ����ƥ�䵽һ����ִ��ģʽ  -1  �޷�ƥ�䵽��ִ��ģʽ  3 �ع켣����
* ����    ��	zgccmax@163.com
* ����    ��	2014.05.24
* �޸ļ�¼��	
*==================================================================*/
int CTurnRound::ModelMatch()
{
	int ret = 0;

	//[1.���ǰ�������Ƿ��㹻]
	ret = DetectStartDsit();
	if (ret == -2)
	{//[���е����滮��Ŀǰ��δʵ��]
		m_Action = REVERSE;
		ret = 3;
		return ret;
	}

	//[2.ģʽ�ж�]
	if (ret == 0)
	{//[������յ�ͷ]
		m_Action = LACTION1;
		m_SteerAdjustTimer = 10;//[1s�ӵ���������]
		m_PredicamentCounter = 0;//[������������ʼ��]

		TurnRoundAction(135, m_Path, m_PathPtsNum);
	}
	else if (ret == 1)
	{//[�ҹյ�ͷ]
		m_Action = RACTION1;
		m_SteerAdjustTimer = 10;//[1s�ӵ���������]
		m_PredicamentCounter = 0;//[������������ʼ��]

		TurnRoundAction(45, m_Path, m_PathPtsNum);
	}
	else
	{
		//[ret == -1  ʲô������]
	}

	return ret;
}

/*==================================================================
 * ������  ��	int CTurnRound::DetectHeadStartDsit()
 * ����    ��	����ʱǰ����ȫ������
 * ���������	
 * ����ֵ  ��	0  ������յ�ͷ  1  �����ҹյ�ͷ  -1  û���㹻�������  -2  ����ȫ��Ҫ����
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.05.28
 * �޸ļ�¼��	2014.06.14  �޸���ײ������
 *==================================================================*/
int CTurnRound::DetectStartDsit()
{
	int i, j;
	int ret = 0;

	int flagl = 0;//[��ն�����־]
	int flagr = 0;//[�ҹն�����־]
	double ldist = 0;
	double rdist = 0;
	int ldist_h = 0;//[�������]
	int rdist_h = 0;//[�������]

	COOR2 pts[200];
	int pts_num = 0;
	int x, y;
	double xx, yy;
	double dist2obs;
	int obs_flag = 0;

	//[1.5f �������֣����������Ǳ�֤�복����һ��դ��С�����ֱ�֤תint��ʱ��������]
	int cwidth = 4;//(int)(m_CarWidth_Cell * 0.5 + 0.5f);

	//[1.���ǰ��]
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

	//[2.�����ǰ��]
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


	//[3.�����ǰ��]
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
		ret = -1;//[��պ��ҹն������Բŷ���-1]
	}
	return ret;
}

/*==================================================================
 * ������  ��	int CTurnRound::Reverse()
 * ����    ��	
 * ���������	
 * ���������	
 * ����ֵ  ��	int			0  ֹͣ����  -1  ��������
 * ����    ��	
 * ����    ��	
 * �޸ļ�¼��	
 *==================================================================*/
int CTurnRound::Reverse()
{
	int ret = 0;

	//[1.���ǰ�������Ƿ��㹻]
	ret = DetectStartDsit();
	if (ret == -2)
	{//[ǰ�����벻������������]
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
* ������  ��	int CTurnRound::DoLAction1()
* ����    ��	��յ�ͷ��һ������  ǰ��
* ���������	
* ����ֵ  ��	MODE_FORWARD  ����ǰ��  MODE_STOP  ͣ��
* ����    ��	zgccmax@163.com
* ����    ��	2014.05.28
* �޸ļ�¼��	
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


	//[1.��⳵ǰ�Ƿ�ȫ]
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

	//[2.ִ�е�ͷ]
	TurnRoundAction(135, m_Path, m_PathPtsNum);

	//[3.�����̼�ʱ������]
	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
		m_SteerAdjustTimer = 0;

	//[4.�жϺ���ǣ�����״̬�л�]
	CalculateYaw();
	if (m_AccumulationYaw >= m_TurnAngle)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_SteerAdjustTimer = 15;
		m_Action = LACTION3;
	}

	//[5.�ж��Ƿ��е�·������״̬�л�]
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
* ������  ��	int CTurnRound::DoLAction2()
* ����    ��	��յ�ͷ�ڶ�������  ����
* ���������	
* ����ֵ  ��	MODE_REVERSE ��������  MODE_STOP  ͣ��
* ����    ��	zgccmax@163.com
* ����    ��	2014.05.28
* �޸ļ�¼��	
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

	//[1.��⳵β�Ƿ�ȫ]
	ret = DetectTail(45);
	if (ret == -1)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		if (m_AccumulationYaw >= m_TurnAngle)
		{//[�Ƕ�����ֱ�ӽ��������״̬]
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

	//[2.ִ�е�ͷ]
	TurnRoundActionBack(45, m_Path, m_PathPtsNum);

	//[3.�����̼�ʱ������]
	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
		m_SteerAdjustTimer = 0;

	//[4.�жϺ���ǣ�����״̬�л�]
	if (m_AccumulationYaw >= m_TurnAngle)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_SteerAdjustTimer = 15;
		m_Action = LACTION3;
	}

	//[5.�ж��Ƿ��е�·������״̬�л�]
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
 * ������  ��	int CTurnRound::DoLAction3()
 * ����    ��	��յ�ͷ����ʱ�����̵���
 * ���������	
 * ����ֵ  ��	MODE_STOP  ͣ��
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.05.28
 * �޸ļ�¼��	
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
	{//[������ͷ]
		m_SteerAdjustTimer = 0;
		m_bTurnRoundFinish = true;
	}

	return MODE_STOP;
}

/*==================================================================
* ������  ��	int CTurnRound::DoRAction1()
* ����    ��	�ҹյ�ͷ��һ������  ǰ��
* ���������	
* ����ֵ  ��	MODE_FORWARD  ����ǰ��  MODE_STOP  ͣ��
* ����    ��	zgccmax@163.com
* ����    ��	2014.05.28
* �޸ļ�¼��	
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

	//[1.��⳵ǰ�Ƿ�ȫ]
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

	//[2.ִ�е�ͷ]
	TurnRoundAction(45, m_Path, m_PathPtsNum);

	//[3.�����̼�ʱ������]
	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
		m_SteerAdjustTimer = 0;

	//[4.�жϺ���ǣ�����״̬�л�]
	CalculateYaw();
	if (m_AccumulationYaw >= m_TurnAngle)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_SteerAdjustTimer = 15;
		m_Action = RACTION3;
	}

	//[5.�ж��Ƿ��е�·������״̬�л�]
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
* ������  ��	int CTurnRound::DoRAction2()
* ����    ��	�ҹյ�ͷ�ڶ�������  ����
* ���������	
* ����ֵ  ��	MODE_REVERSE ��������  MODE_STOP  ͣ��
* ����    ��	zgccmax@163.com
* ����    ��	2014.05.28
* �޸ļ�¼��	
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

	//[1.��⳵β�Ƿ�ȫ]
	ret = DetectTail(135);
	if (ret == -1)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		if (m_AccumulationYaw >= m_TurnAngle)
		{//[�Ƕ�����ֱ�ӽ��������״̬]
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

	//[2.ִ�е�ͷ]
	TurnRoundActionBack(135, m_Path, m_PathPtsNum);

	//[3.�����̼�ʱ������]
	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
		m_SteerAdjustTimer = 0;

	//[4.�жϺ���ǣ�����״̬�л�]
	if (m_AccumulationYaw >= m_TurnAngle)
	{
		m_AccumulationSwitch = 0;
		m_AccumulationDist = 0;

		m_SteerAdjustTimer = 15;
		m_Action = RACTION3;
	}

	//[5.�ж��Ƿ��е�·������״̬�л�]
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
 * ������  ��	int CTurnRound::DoRAction3()
 * ����    ��	�ҹյ�ͷ����ʱ�����̵���
 * ���������	
 * ����ֵ  ��	MODE_STOP  ͣ��
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.05.28
 * �޸ļ�¼��	
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
* ������  ��	void CTurnRound::SetActionResult(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, int mode)
* ����    ��	����ִ�в���
* ���������	PL_FUNC_INPUT *pl_input		�滮������
*				PL_CS_DATA *pl_cs_data		���͸��ײ�
*				int mode					MODE_FORWARD ����ǰ��  MODE_REVERSE ��������  MODE_STOP ����ͣ��
* ����ֵ  ��	
* ����    ��	zgccmax@163.com
* ����    ��	2014.05.28
* �޸ļ�¼��	
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
 * ������  ��	int CTurnRound::DetectHead(double angle)
 * ����    ��	���ǰ��������������ҹ������Ƿ�ȫ
 * ���������	double angle		ǰ��ʱǰ��ת���
 * ����ֵ  ��	MODE_FORWARD  ��ȫ  -1 ����ȫ
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.05.28
 * �޸ļ�¼��	
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

	//[1.5f �������֣����������Ǳ�֤�복����һ��դ��С�����ֱ�֤תint��ʱ��������]
	int cwidth = 4;//(int)(m_CarWidth_Cell * 0.5 + 1.5f);

	//[�����ǰ]
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
 * ������  ��	int CTurnRound::DetectTail(double angle)
 * ����    ��	��⵹��������������ҹ������Ƿ�ȫ
 * ���������	double angle		����ʱǰ��ת���
 * ����ֵ  ��	MODE_REVERSE  ��ȫ  -1 ����ȫ
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.05.28
 * �޸ļ�¼��	
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

	//[1.5f �������֣����������Ǳ�֤�복����һ��դ��С�����ֱ�֤תint��ʱ��������]
	int cwidth = 4;//(int)(m_CarWidth_Cell * 0.5 + 1.5f);

	//[������]
	if (m_Action == RACTION2)
	{
		create_line_by_theta(angle, pts, pts_num, 2);
		for (i=0; i<pts_num;i++)
		{
			x = pts[i].x / GRID_LEN_PER_CELL + (m_grid_center.x - 1);
			y = pts[i].y / GRID_LEN_PER_CELL + m_grid_center.y;

			if (y <= 0)//[��ֹԽ��]
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

			if (y <= 0)//[��ֹԽ��]
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
 * ������  ��	void CTurnRound::CalculateYaw()
 * ����    ��	���к���ǵ��ۼ�
 * ���������	
 * ����ֵ  ��	
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.05.28
 * �޸ļ�¼��	
 *==================================================================*/
void CTurnRound::CalculateYaw()
{
	//[�����������ת��0��ʱ��ʹ��180���ֵ�������жϣ��ǿ���ʵ���˶������У����ӵĺ���ǲ����ܷ���180��ͻ��]
	int deltayaw;
	deltayaw = m_CurYaw - m_LastYaw;
	switch (m_Action)
	{//[��ʱ���ۼ�]
	case LACTION1:
	case LACTION2:
		if (deltayaw >= 0)
		{
			if (deltayaw > 180)//[˳ʱ��ת��0��]
				deltayaw -= 360;
			m_AccumulationYaw += deltayaw;
		}
		else
		{
			if (deltayaw < -180)//[��ʱ��ת��0��]
				deltayaw += 360;
			m_AccumulationYaw += deltayaw;
		}
		break;

		//[˳ʱ���ۼ�]
	case RACTION1:
	case RACTION2:
		if (deltayaw >= 0)
		{
			if (deltayaw > 180)
				deltayaw = 360 - deltayaw;//[˳ʱ��ת��0��]
			else
				deltayaw = -deltayaw;
			m_AccumulationYaw += deltayaw;
		}
		else
		{
			if (deltayaw < -180)
				deltayaw = deltayaw - 360;//[��ʱ��ת��0��]
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
 * ������  ��	bool CTurnRound::TurnRoundFinish()
 * ����    ��	�����Ƿ���ɵ�ͷ����
 * ���������	
 * ����ֵ  ��	true  ���  false  û�����
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.05.28
 * �޸ļ�¼��	
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
 * ������  ��	
 * ����    ��	
 * ���������	
 * ���������	
 * ����ֵ  ��	
 * ����    ��	
 * ����    ��	
 * �޸ļ�¼��	
 *==================================================================*/
int CTurnRound::DoRightTurnRound(int frame, int yaw, int grid_map[GRID_HEIGHT][GRID_WIDTH], COOR2 grid_center)
{
	int ret = MODE_FORWARD;

	//[��������]
	m_Frame = frame;
	m_CurYaw = yaw;
	memcpy(m_grid_map, grid_map, sizeof(int) * GRID_WIDTH * GRID_HEIGHT);
	m_grid_center = grid_center;

	//[ִ�ж���]
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
	{//[��������]
		if (m_bLog) MBUG("DoTurnRound Error : Be Traped in Predicament����\n");
		ret = MODE_STOP;
	}

	return ret;
}

int CTurnRound::DoRTRA1()
{
	int ret = MODE_FORWARD;
	//[1.��⳵ǰ�Ƿ�ȫ]
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

	//[2.ִ�е�ͷ]
	TurnRoundAction(45, m_Path, m_PathPtsNum);

	//[3.�����̼�ʱ������]
	m_SteerAdjustTimer--;
	if (m_SteerAdjustTimer < 0)
		m_SteerAdjustTimer = 0;

	//[4.�жϺ���ǣ�����״̬�л�]
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

	//[1.��⳵β�Ƿ�ȫ]
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

	//[2.ִ�е�ͷ]
	TurnRoundActionBack(135, m_Path, m_PathPtsNum);

	//[3.�����̼�ʱ������]
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