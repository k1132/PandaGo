#ifndef REVERSE_H_
#define REVERSE_H_

#include "../robix4/protocols/app_pl.h"
#include "../robix4/protocols/protocol_pl.h"

#define GRID_HEIGHT (LEN_VER_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER))
#define GRID_WIDTH	(LEN_HOR_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR))
#define GRID_MID_LEFT GRID_WIDTH / 2 -1		//[栅格正中间左侧]
#define GRID_MID_RIGHT GRID_WIDTH / + 1		//[栅格正中间右侧]
#define GRID_LEN_PER_CELL	25				//[每个栅格25cm]

#define OBS_FLAG 1	//[障碍物在栅格中的值]


#include <deque>
#include <algorithm>
#include <cstring>
using namespace std;

class CReverse
{
public:
	CReverse();
	~CReverse();

	void ReverseInit(int reverse_speed, 
					int reverse_plan_dist, 
					int reverse_plan_pts_step,
					int reverse_step, 
					int reverse_pt_num_limit, 
					int reverse_back_stop_dist,
					int reverse_test_start_dist, 
					int reverse_test_stop_dist);
	void MaintainPts(POSE_INFO pose_info);
	int ReversePlan(POSE_INFO pose_info, 
					STATE state,
					int grid_map[GRID_HEIGHT][GRID_WIDTH], 
					COOR2 grid_center, 
					int g_frame_id,
					PL_CS_DATA *pl_cs_data);
	int ReversePlan_DirectBack(POSE_INFO pose_info,
		STATE state,
		int grid_map[GRID_HEIGHT][GRID_WIDTH],
		COOR2 grid_center,
		int g_frame_id,
		PL_CS_DATA *pl_cs_data);
	bool GetReverseFlag();
	void SetReverseFlag(bool flag);
	void ReverseTest(POSE_INFO pose_info);

	deque<COOR2> m_reverse_pts;			//[倒车全局点]
private:

	int m_CarWidth;
	int m_grid_map[GRID_HEIGHT][GRID_WIDTH];	//[栅格地图]
	COOR2 m_grid_center;

	COOR2 m_Path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int m_PathPtsNum;

	
	int m_reverse_pt_num_limit;			//[倒车全局点个数限制]
	int m_reverse_speed;				//[倒车速度]
	bool m_breverse_flag;				//[倒车标志位]
	bool m_breverse_init;				//[初始化]
	COOR2 m_reverse_start_pt;			//[倒车起点]
	int m_reverse_plan_dist;			//[倒车规划路径长度]
	int m_reverse_plan_pts_step;		//[倒车规划线点距]
	int m_reverse_step;					//[倒车记录点间的步长]
	int m_reverse_forward_sum_dist;		//[前进累积位移]
	COOR2 m_reverse_last_pt;			//[上一帧全局点]
	int m_reverse_back_sum_dist;		//[倒车累积位移]
	int m_reverse_back_stop_dist;		//[倒车停止距离]
	COOR2 m_reverse_back_last_pt;		//[倒车上一帧全局点]
	int m_reverse_test_start_dist;		//[倒车测试启动距离]
	int m_reverse_test_stop_dist;		//[倒车测试停止距离]
	int m_reverse_test_inti_flag;
	int m_reverse_test_sum_dist;		//[倒车测试累积距离]
	COOR2 m_reverse_test_last_pt;		//[倒车测试上一帧全局点]

	int collision_check_and_replan(COOR2 grid_center);
	void Reset();
};
#endif /*REVERSE_H_*/