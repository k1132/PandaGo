#ifndef TURN_ROUND_H_
#define TURN_ROUND_H_

#include "../robix4/protocols/app_pl.h"
#include "../robix4/protocols/protocol_pl.h"
#include "./trace_road.h"
#include "./basicfunction.h"

#define GRID_HEIGHT (LEN_VER_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER))
#define GRID_WIDTH (LEN_HOR_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR))
#define GRID_MID_LEFT GRID_WIDTH / 2 -1		//[դ�����м����]
#define GRID_MID_RIGHT GRID_WIDTH / + 1		//[դ�����м��Ҳ�]
#define GRID_LEN_PER_CELL	25				//[ÿ��դ��25cm]

//[ACTION1 ǰ��  ACTION2 ����  ACTION3  ��������]
enum Action {MATCHING = 0, REVERSE, LACTION1, LACTION2, LACTION3, RACTION1, RACTION2, RACTION3};

#define OBS_FLAG 1	//[�ϰ�����դ���е�ֵ]
#define CM 27.778f	//[�������ټ���]
#define SYS_COMMAND_REV			0x00000001		//����
#define SYS_COMMAND_STOP		0x00000002		//��ͣ
#define SYS_COMMAND_LEFT		0x00000004		//���
#define SYS_COMMAND_RIGHT		0x00000008		//�ҵ�
#define SYS_COMMAND_BEEP		0x00000010		//����
#define SYS_COMMAND_ALARM		0x00000020		//����

//[����ֵ]
#define MODE_FORWARD 0
#define MODE_REVERSE 1
#define MODE_STOP 2

#define FORWARD_FIRST 0
#define BACK_FIRST 1

//[������ȷ��ת���]
#define  CHECK_YAW(x)  \
	if (x < -180)  \
	  x += 360;  \
else if (x >= 180) \
	  x -= 360;

//[��ͷ�������������У�1�����������ǰ�����������Ҵ򵹳������������ǰ������ɵ�ͷ��2�����Ҵ������ԳƵĶ�������]
class CTurnRound
{
public:
	CTurnRound();
	~CTurnRound();

	//[��ʼ���������������������]
	void TurnRoundInit(int car_width, int syaw, bool blog);
	int DoTurnRound(int frame, int yaw, COOR2 pos, int grid_map[GRID_HEIGHT][GRID_WIDTH], COOR2 grid_center, NATURAL_BOUNDARY natural_boundary);		//[ִ�е�ͷ]
	void SetActionResult(PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, int mode);
	bool TurnRoundFinish();
	int GetAction();
	int ConfirmRoad();
	bool GetInitialStatus();


	void RightTurnRoundInit(int goal_yaw, int finish_yaw_threshold, int mode);
	int DoRightTurnRound(int frame, int yaw, int grid_map[GRID_HEIGHT][GRID_WIDTH], COOR2 grid_center);

private:
	bool m_bInitialize;
	bool m_bLog;
	bool m_bTurnRoundFinish;

	Action m_Action;
	Action m_ActionModel;

	int m_Frame;
	int m_StartYaw;
	int m_CurYaw;
	int m_LastYaw;
	int m_AccumulationYaw;

	int m_MoveHeadSpeed;
	int m_MoveBackSpeed;

	int m_CarWidth;				//[����  ��λcm]
	int m_SearchHeight;			//[�����߶�  ��λcm]
	int m_CarHeadSafeDist;		//[��ͷ��ȫ����  ��λcm]
	int m_CarTailSafeDist;		//[��β��ȫ����  ��λcm]
	int m_CarTransverseDist;	//[������ȫ����  ��λcm]
	int m_TurnAngle;			//[��ͷ���ʱת��Ƕ�  ��λ��]
	int m_ConfirmAngle;			//[��ʼȷ�ϵ�·�ĽǶ�  ��λ��]
	int m_ConfirmTimer;

	int m_CarWidth_Cell;			//[����  դ��]

	int m_SteerAdjustTimer;				//[���ڷ����̵ļ�ʱ��]

	int m_grid_map[GRID_HEIGHT][GRID_WIDTH];	//[դ���ͼ]
	COOR2 m_grid_center;
	NATURAL_BOUNDARY m_natural_boundary;

	COOR2 m_Path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int m_PathPtsNum;
	int m_Speed;

	int m_PredicamentCounter;	//[����������]


	double m_AccumulationDist;
	COOR2 m_LastPosition;
	COOR2 m_CurPosition;
	int m_AccumulationSwitch;


	int m_GoalYaw;
	int m_FinishYawThreshold;

	void CalculateYaw();

	void DefaultConfig();
	int ModelMatch();	//[ģ��ƥ��]
	int Reverse();		//[����]

	int DoLAction1();		//[��һ�ε���]
	int DoLAction2();		//[�ڶ��ε���]
	int DoLAction3();		//[�����ε���]

	int DoRAction1();		//[��һ�ε���]
	int DoRAction2();		//[�ڶ��ε���]
	int DoRAction3();		//[�����ε���]

	int DetectStartDsit();		//[��⳵ͷ�Ƿ����㹻��������]
	int DetectHead(double angle);		//[��⳵ͷ]
	int DetectTail(double angle);		//[��⳵β]


	int DoRTRA1();
	int DoRTRA2();
	int DoRTRA3();
};

#endif /*CROSSING_H_*/