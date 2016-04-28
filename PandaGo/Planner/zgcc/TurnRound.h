#ifndef TURN_ROUND_H_
#define TURN_ROUND_H_

#include "../robix4/protocols/app_pl.h"
#include "../robix4/protocols/protocol_pl.h"
#include "./trace_road.h"
#include "./basicfunction.h"

#define GRID_HEIGHT (LEN_VER_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER))
#define GRID_WIDTH (LEN_HOR_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR))
#define GRID_MID_LEFT GRID_WIDTH / 2 -1		//[栅格正中间左侧]
#define GRID_MID_RIGHT GRID_WIDTH / + 1		//[栅格正中间右侧]
#define GRID_LEN_PER_CELL	25				//[每个栅格25cm]

//[ACTION1 前进  ACTION2 倒车  ACTION3  调整方向]
enum Action {MATCHING = 0, REVERSE, LACTION1, LACTION2, LACTION3, RACTION1, RACTION2, RACTION3};

#define OBS_FLAG 1	//[障碍物在栅格中的值]
#define CM 27.778f	//[用来快速计算]
#define SYS_COMMAND_REV			0x00000001		//倒车
#define SYS_COMMAND_STOP		0x00000002		//急停
#define SYS_COMMAND_LEFT		0x00000004		//左灯
#define SYS_COMMAND_RIGHT		0x00000008		//右灯
#define SYS_COMMAND_BEEP		0x00000010		//喇叭
#define SYS_COMMAND_ALARM		0x00000020		//警报

//[返回值]
#define MODE_FORWARD 0
#define MODE_REVERSE 1
#define MODE_STOP 2

#define FORWARD_FIRST 0
#define BACK_FIRST 1

//[计算正确的转向角]
#define  CHECK_YAW(x)  \
	if (x < -180)  \
	  x += 360;  \
else if (x >= 180) \
	  x -= 360;

//[掉头动作，动作序列：1、方向盘左打前进，方向盘右打倒车，方向盘左打前进，完成掉头，2、先右打完成相对称的动作序列]
class CTurnRound
{
public:
	CTurnRound();
	~CTurnRound();

	//[初始化，参数具体意义见定义]
	void TurnRoundInit(int car_width, int syaw, bool blog);
	int DoTurnRound(int frame, int yaw, COOR2 pos, int grid_map[GRID_HEIGHT][GRID_WIDTH], COOR2 grid_center, NATURAL_BOUNDARY natural_boundary);		//[执行掉头]
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

	int m_CarWidth;				//[车宽  单位cm]
	int m_SearchHeight;			//[搜索高度  单位cm]
	int m_CarHeadSafeDist;		//[车头安全距离  单位cm]
	int m_CarTailSafeDist;		//[车尾安全距离  单位cm]
	int m_CarTransverseDist;	//[车横向安全距离  单位cm]
	int m_TurnAngle;			//[掉头完成时转向角度  单位度]
	int m_ConfirmAngle;			//[开始确认道路的角度  单位度]
	int m_ConfirmTimer;

	int m_CarWidth_Cell;			//[车宽  栅格]

	int m_SteerAdjustTimer;				//[调节方向盘的计时器]

	int m_grid_map[GRID_HEIGHT][GRID_WIDTH];	//[栅格地图]
	COOR2 m_grid_center;
	NATURAL_BOUNDARY m_natural_boundary;

	COOR2 m_Path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int m_PathPtsNum;
	int m_Speed;

	int m_PredicamentCounter;	//[困境计数器]


	double m_AccumulationDist;
	COOR2 m_LastPosition;
	COOR2 m_CurPosition;
	int m_AccumulationSwitch;


	int m_GoalYaw;
	int m_FinishYawThreshold;

	void CalculateYaw();

	void DefaultConfig();
	int ModelMatch();	//[模型匹配]
	int Reverse();		//[倒车]

	int DoLAction1();		//[第一次调整]
	int DoLAction2();		//[第二次调整]
	int DoLAction3();		//[第三次调整]

	int DoRAction1();		//[第一次调整]
	int DoRAction2();		//[第二次调整]
	int DoRAction3();		//[第三次调整]

	int DetectStartDsit();		//[检测车头是否有足够启动距离]
	int DetectHead(double angle);		//[检测车头]
	int DetectTail(double angle);		//[检测车尾]


	int DoRTRA1();
	int DoRTRA2();
	int DoRTRA3();
};

#endif /*CROSSING_H_*/