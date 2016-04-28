#ifndef REVERSE_H_
#define REVERSE_H_

#include "../robix4/protocols/app_pl.h"
#include "../robix4/protocols/protocol_pl.h"

#define GRID_HEIGHT (LEN_VER_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER))
#define GRID_WIDTH	(LEN_HOR_64 * (1 << POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR))
#define GRID_MID_LEFT GRID_WIDTH / 2 -1		//[դ�����м����]
#define GRID_MID_RIGHT GRID_WIDTH / + 1		//[դ�����м��Ҳ�]
#define GRID_LEN_PER_CELL	25				//[ÿ��դ��25cm]

#define OBS_FLAG 1	//[�ϰ�����դ���е�ֵ]


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

	deque<COOR2> m_reverse_pts;			//[����ȫ�ֵ�]
private:

	int m_CarWidth;
	int m_grid_map[GRID_HEIGHT][GRID_WIDTH];	//[դ���ͼ]
	COOR2 m_grid_center;

	COOR2 m_Path[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int m_PathPtsNum;

	
	int m_reverse_pt_num_limit;			//[����ȫ�ֵ��������]
	int m_reverse_speed;				//[�����ٶ�]
	bool m_breverse_flag;				//[������־λ]
	bool m_breverse_init;				//[��ʼ��]
	COOR2 m_reverse_start_pt;			//[�������]
	int m_reverse_plan_dist;			//[�����滮·������]
	int m_reverse_plan_pts_step;		//[�����滮�ߵ��]
	int m_reverse_step;					//[������¼���Ĳ���]
	int m_reverse_forward_sum_dist;		//[ǰ���ۻ�λ��]
	COOR2 m_reverse_last_pt;			//[��һ֡ȫ�ֵ�]
	int m_reverse_back_sum_dist;		//[�����ۻ�λ��]
	int m_reverse_back_stop_dist;		//[����ֹͣ����]
	COOR2 m_reverse_back_last_pt;		//[������һ֡ȫ�ֵ�]
	int m_reverse_test_start_dist;		//[����������������]
	int m_reverse_test_stop_dist;		//[��������ֹͣ����]
	int m_reverse_test_inti_flag;
	int m_reverse_test_sum_dist;		//[���������ۻ�����]
	COOR2 m_reverse_test_last_pt;		//[����������һ֡ȫ�ֵ�]

	int collision_check_and_replan(COOR2 grid_center);
	void Reset();
};
#endif /*REVERSE_H_*/