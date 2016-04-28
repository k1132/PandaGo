#include "./reverse.h"
#include "./trace_road.h"
#include "./basicfunction.h"
#include <cmath>

CReverse::CReverse()
{
	m_reverse_pts.clear();
	
	m_breverse_flag = false;
	
	m_CarWidth = 230;
	m_reverse_pt_num_limit = 300;
	m_reverse_speed = (int)(5 * CM);
	m_reverse_plan_dist = 700;
	m_reverse_plan_pts_step = 40;
	m_reverse_step = 50;

	m_breverse_init = false;
	memset(&m_reverse_start_pt, 0, sizeof(COOR2));
	m_reverse_forward_sum_dist = 0;
	memset(&m_reverse_last_pt, 0, sizeof(COOR2));
	m_reverse_back_sum_dist = 0;
	m_reverse_back_stop_dist = 0;
	memset(&m_reverse_back_last_pt, 0, sizeof(COOR2));

	m_reverse_test_start_dist = 0;
	m_reverse_test_stop_dist = 0;
	m_reverse_test_inti_flag = 0;
}

CReverse::~CReverse()
{

}

/*==================================================================
* ������  ��	void CReverse::ReverseInit(int reverse_speed, \
											int reverse_dist, \
											int reverse_step, \
											int reverse_pt_num_limit, \
											int reverse_test_dist)
 * ����    ��	���������滮��ʼ��
 * ���������	int reverse_speed			�����ٶ�
 *				int reverse_dist			�����滮�߳���
 *				int reverse_step			������¼���Ĳ���
 *				int reverse_pt_num_limit	����ȫ�ֵ��������
 *				reverse_test_dist			����������������
 * ���������	
 * ����ֵ  ��	
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.June.16
 * �޸ļ�¼��	
 *==================================================================*/
void CReverse::ReverseInit(int reverse_speed, \
						  int reverse_plan_dist, \
						  int reverse_plan_pts_step, \
						  int reverse_step, \
						  int reverse_pt_num_limit, \
						  int reverse_back_stop_dist, \
						  int reverse_test_start_dist, 
						  int reverse_test_stop_dist)
{
	m_reverse_pt_num_limit = reverse_pt_num_limit;
	m_reverse_back_stop_dist = reverse_back_stop_dist;
	m_reverse_speed = (int)(reverse_speed * CM);
	m_reverse_plan_dist = reverse_plan_dist;
	m_reverse_plan_pts_step = reverse_plan_pts_step;
	m_reverse_step = reverse_step;
	m_reverse_test_start_dist = reverse_test_start_dist;
	m_reverse_test_stop_dist = reverse_test_stop_dist;

	if (m_reverse_test_stop_dist > m_reverse_test_start_dist)
		m_reverse_test_stop_dist = m_reverse_test_start_dist;
}

/*==================================================================
 * ������  ��	void CReverse::MaintainPts(POSE_INFO pose_info)
 * ����    ��	ά��ȫ�ֵ�����
 * ���������	POSE_INFO pose_info		����λ��
 * ���������	
 * ����ֵ  ��	
 * ����    ��	zgccmax@163.com
 * ����    ��	2014.June.16
 * �޸ļ�¼��	
 *==================================================================*/
void CReverse::MaintainPts(POSE_INFO pose_info)
{
	COOR2 cur_pt;
	COOR2 old_pt;
	cur_pt.x = pose_info.ins_coord.x;
	cur_pt.y = pose_info.ins_coord.y;

	if (m_reverse_pts.size() == 0)
	{
		m_reverse_last_pt = cur_pt;
		m_reverse_pts.push_back(cur_pt);
		return;
	}
	else
	{
		if (m_breverse_flag == false)
		{//[������ʻά�������ȫ�ֵ�]
			double dist = dist_point(&cur_pt, &m_reverse_last_pt);
			m_reverse_forward_sum_dist += (int)dist;

			//[ÿ���ۻ���ʻ�������g_reverse_stepʱ����¼һ��]
			if (m_reverse_forward_sum_dist >= m_reverse_step && m_reverse_forward_sum_dist < 1500)
			{
				if ((int)m_reverse_pts.size() < m_reverse_pt_num_limit)
				{
					m_reverse_pts.push_back(cur_pt);
				}
				else
				{
#ifdef MBUG_OPEN_
					MBUG("m_reverse_pts num : %d\n", (int)m_reverse_pts.size());
#endif
					if (m_reverse_pts.size() != 0)
						m_reverse_pts.pop_front();
					m_reverse_pts.push_back(cur_pt);
				}
				m_reverse_forward_sum_dist -= m_reverse_step;
			}
			m_reverse_back_sum_dist = 0;
		}
		else
		{//[����ʱά��]
			m_reverse_forward_sum_dist = 0;
			coor2_e2v(&pose_info, &m_reverse_pts.back(), &old_pt);

			while (old_pt.y >= 0)
			{
				m_reverse_pts.pop_back();
				if (m_reverse_pts.size() == 0)
					break;
				coor2_e2v(&pose_info, &m_reverse_pts.back(), &old_pt);
			}

			m_reverse_pts.push_back(cur_pt);
		}
	}

	m_reverse_last_pt = cur_pt;
}

int CReverse::collision_check_and_replan(COOR2 grid_center)
{
	int ret = 0;

	int i, j;
	int x, y;
	double xx, yy;
	double dist2obs;
	int obs_flag = 0;

	//int car_width_cell = m_CarWidth / GRID_LEN_PER_CELL;
	//[0.5f ��֤תint��ʱ��������]
	int cwidth = 4;//(int)(car_width_cell * 0.5 + 0.5f);

	//[1.�Ե����滮�߽�����ײ���]
	for (i=0; i<m_PathPtsNum;i++)
	{
		x = m_Path[i].x / GRID_LEN_PER_CELL + (grid_center.x - 1);
		y = m_Path[i].y / GRID_LEN_PER_CELL + grid_center.y;

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

	if (i==0)
		i++;
	xx = m_Path[i-1].x;
	yy = m_Path[i-1].y;
	dist2obs = sqrt(xx*xx+yy*yy);
	if (dist2obs < 250)//[2.5m֮�ⲻ����ײ]
	{//[@@δ����ع滮����]
		ret = -1;
		return ret;
	}


	return ret;
}

/*==================================================================
* ������  ��	int CReverse::ReversePlan(POSE_INFO pose_info, 
											STATE state,
											int grid_map[GRID_HEIGHT][GRID_WIDTH], 
											COOR2 grid_center, 
											int g_frame_id,
											PL_CS_DATA *pl_cs_data)
 * ����    ��	�����滮
 * ���������	POSE_INFO pose_info
 *				PL_CS_DATA *pl_cs_data
 *				int g_frame_id
 * ���������	
 * ����ֵ  ��	
 * ����    ��	
 * ����    ��	
 * �޸ļ�¼��	
 *==================================================================*/
int CReverse::ReversePlan(POSE_INFO pose_info, 
						  STATE state,
						  int grid_map[GRID_HEIGHT][GRID_WIDTH], 
						  COOR2 grid_center, 
						  int g_frame_id,
						  PL_CS_DATA *pl_cs_data)
{
	int ret = 0;
	int i;
	POSE_INFO gps;
	double sum_dits;
	COOR2 pt;
	COOR2 pts[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int pts_num;
	deque<COOR2> reverse_path;
#ifdef MBUG_OPEN_
	MBUG("------------------ In reverse plan -------------------\n");
#endif
	//[��������]
	memcpy(m_grid_map, grid_map, sizeof(int) * GRID_WIDTH * GRID_HEIGHT);
	m_grid_center = grid_center;
	memset(pl_cs_data, 0, sizeof(PL_CS_DATA));
	gps = pose_info;

	//[1.��һ֡��ʼ��]
	if (m_breverse_init == false)
	{
		m_reverse_start_pt = gps.ins_coord;
		m_reverse_back_last_pt = gps.ins_coord;
		m_reverse_back_sum_dist = 0;
		m_reverse_forward_sum_dist = 0;
		m_breverse_init = true;
	}

	//[2.ͳ������·������ܳ���]
	sum_dits = 0;
	for (i=1; i<(int)m_reverse_pts.size(); i++)
		sum_dits += dist_point(&m_reverse_pts[i-1], &m_reverse_pts[i]);
	
	if (sum_dits < m_reverse_plan_dist)
	{//[ʣ��·�����Ȳ��㹻�滮]
		pl_cs_data->speed = 0;
		pl_cs_data->sys_command = 0;
		pl_cs_data->id = g_frame_id;
		pl_cs_data->isok = 1;
		pl_cs_data->state = state;
		memset(pl_cs_data->path, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		pl_cs_data->number_of_effective_points = 0;

		m_reverse_back_sum_dist = 0;
		m_reverse_forward_sum_dist = 0;
		m_breverse_flag = false;
		m_breverse_init = false;;
		ret = -1;
		return ret;
	}

	//[3.���ɵ����õľֲ�·����]
	COOR2 tmp1, tmp2;
	deque<COOR2> tmp_path;
	int num = 0;
	tmp1.x = 0;
	tmp1.y = 0;
	tmp_path.resize(m_reverse_pts.size());
	for (i=0; i<(int)m_reverse_pts.size(); i++)
	{
		pt = m_reverse_pts[i];
		coor2_e2v(&gps, &pt, &tmp2);
		if (dist_point(&tmp1, &tmp2) >= m_reverse_plan_pts_step)
		{
			tmp_path[num++] = tmp2;
			tmp1 = tmp2;
		}
	}
	reverse_path.resize(num);
	for (i=0;i<num;i++)
	{
		reverse_path[i] = tmp_path[i];
	}
	reverse(reverse_path.begin(), reverse_path.end());

	//[4.�����滮��]
	memset(m_Path, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	memset(pts, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	pts_num = 0;
	for (i=0; i<(int)reverse_path.size(); i++)
	{
		if (i == NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
			break;

		pts[i] = reverse_path[i];
	}
	pts_num = i;
	get_bezier_line(pts, pts_num, m_Path, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	m_PathPtsNum = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;

	//[5.��ײ���]
	ret = collision_check_and_replan(grid_center);
	if (ret == 0)
	{
		pl_cs_data->speed = m_reverse_speed;
		pl_cs_data->sys_command |= SYS_COMMAND_REV;
		pl_cs_data->id = g_frame_id;
		pl_cs_data->isok = 1;
		pl_cs_data->state = state;
		memcpy(pl_cs_data->path, m_Path, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		pl_cs_data->number_of_effective_points = m_PathPtsNum;
	}
	else
	{
		pl_cs_data->speed = (int)(0 * CM);
		pl_cs_data->sys_command = 0;
		pl_cs_data->id = g_frame_id;
		pl_cs_data->isok = 1;
		pl_cs_data->state = state;
		memset(pl_cs_data->path, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		pl_cs_data->number_of_effective_points = 0;

		m_reverse_back_sum_dist = 0;
		m_reverse_forward_sum_dist = 0;
		m_breverse_flag = false;
		m_breverse_init = false;;
		ret = -1;
		return ret;
	}

	//[�������뵽����ֵ�����������滮]
	pt = gps.ins_coord;
	m_reverse_back_sum_dist += (int)dist_point(&m_reverse_back_last_pt, &pt);

	if (m_reverse_back_sum_dist > m_reverse_back_stop_dist)
	{
		pl_cs_data->speed = (int)(0 * CM);
		pl_cs_data->sys_command = 0;
		pl_cs_data->id = g_frame_id;
		pl_cs_data->isok = 1;
		pl_cs_data->state = state;
		memset(pl_cs_data->path, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		pl_cs_data->number_of_effective_points = 0;

		m_reverse_back_sum_dist = 0;
		m_reverse_forward_sum_dist = 0;
		m_breverse_flag = false;
		m_breverse_init = false;;
		ret = -1;
		return ret;
	}

	m_reverse_back_last_pt = gps.ins_coord;
#ifdef MBUG_OPEN_
	MBUG("\n");
	MBUG("---------------------reverse plan END-----------------------\n");
#endif
	return 0;
}

bool CReverse::GetReverseFlag()
{
	return m_breverse_flag;
}

/*==================================================================
 * ������  ��	void CReverse::SetReverseFlag(bool flag)
 * ����    ��	���õ�����־
 * ���������	true  ��������  false  ��������  �����������¼�¼��ʻ�켣
 * ���������	
 * ����ֵ  ��	
 * ����    ��	
 * ����    ��	
 * �޸ļ�¼��	
 *==================================================================*/
void CReverse::SetReverseFlag(bool flag)
{
	m_breverse_flag = flag;
	if (m_breverse_flag == false)
		Reset();
}

void CReverse::Reset()
{
	m_breverse_flag = false;
	m_breverse_init = false;

	m_reverse_pts.clear();
	memset(&m_reverse_start_pt, 0, sizeof(COOR2));
	m_reverse_forward_sum_dist = 0;
	memset(&m_reverse_last_pt, 0, sizeof(COOR2));
	m_reverse_back_sum_dist = 0;
	memset(&m_reverse_back_last_pt, 0, sizeof(COOR2));

	m_reverse_test_start_dist = 0;
	m_reverse_test_stop_dist = 0;
	m_reverse_test_inti_flag = 0;
	m_reverse_test_sum_dist = 0;
	memset(&m_reverse_test_last_pt, 0, sizeof(COOR2));
}

void CReverse::ReverseTest(POSE_INFO pose_info)
{
	POSE_INFO gps = pose_info;
	if (m_reverse_test_start_dist > m_reverse_plan_dist)
	{
		if (m_breverse_flag == false)
		{//[����ǰ�������Ƿ�����������ֵ]
			if (m_reverse_test_inti_flag == 0)
			{
				m_reverse_test_sum_dist = 0;
				m_reverse_test_last_pt = gps.ins_coord;
				m_reverse_test_inti_flag = 1;
			}
			else
			{
				m_reverse_test_sum_dist += (int)dist_point(&m_reverse_test_last_pt, &gps.ins_coord);
				m_reverse_test_last_pt = gps.ins_coord;
			}

			if (m_reverse_test_sum_dist > m_reverse_test_start_dist)
				SetReverseFlag(true);
		}
		else
		{
			if (m_reverse_back_sum_dist > m_reverse_test_stop_dist)
			{
				m_reverse_test_start_dist = 0;
				m_reverse_test_stop_dist = 0;
				SetReverseFlag(false);
			}
		}
	}
	else
		m_reverse_test_sum_dist = 0;
}

/*==================================================================
 * ������  ��	int CReverse::ReversePlan_DirectBack(POSE_INFO pose_info,
													STATE state,
													int grid_map[GRID_HEIGHT][GRID_WIDTH],
													COOR2 grid_center,
													int g_frame_id,
													PL_CS_DATA *pl_cs_data)
 * ����    ��	ֱ�ߵ����滮
 * ���������	POSE_INFO pose_info		����λ������
				STATE state				�滮״̬��������д���������
				int grid_map[GRID_HEIGHT][GRID_WIDTH]		�����滮�õ���դ������
				COOR2 grid_center		դ������
				int g_frame_id			�滮֡��
				PL_CS_DATA *pl_cs_data	�滮�������
 * ���������	
 * ����ֵ  ��	int		0  �п�ִ��·��		-1  �޿�ִ��·��
 * ����    ��	
 * ����    ��	
 * �޸ļ�¼��	
 *==================================================================*/
int CReverse::ReversePlan_DirectBack(POSE_INFO pose_info,
	STATE state,
	int grid_map[GRID_HEIGHT][GRID_WIDTH],
	COOR2 grid_center,
	int g_frame_id,
	PL_CS_DATA *pl_cs_data)
{
	int ret = 0;
	int i;
	POSE_INFO gps;
	COOR2 pt;
	deque<COOR2> reverse_path;
#ifdef MBUG_OPEN_
	MBUG("------------------ In reverse plan -------------------\n");
#endif
	//[��������]
	memcpy(m_grid_map, grid_map, sizeof(int) * GRID_WIDTH * GRID_HEIGHT);
	m_grid_center = grid_center;
	memset(pl_cs_data, 0, sizeof(PL_CS_DATA));
	gps = pose_info;

	//[1.��һ֡��ʼ��]
	if (m_breverse_init == false)
	{
		m_reverse_start_pt = gps.ins_coord;
		m_reverse_back_last_pt = gps.ins_coord;
		m_reverse_back_sum_dist = 0;
		m_reverse_forward_sum_dist = 0;
		m_breverse_init = true;
	}

	//[2.ͳ������·������ܳ���]
// 	sum_dits = 0;
// 	for (i = 1; i < (int)m_reverse_pts.size(); i++)
// 		sum_dits += dist_point(&m_reverse_pts[i - 1], &m_reverse_pts[i]);
// 
// 	if (sum_dits < m_reverse_plan_dist)
// 	{//[ʣ��·�����Ȳ��㹻�滮]
// 		pl_cs_data->speed = 0;
// 		pl_cs_data->sys_command = 0;
// 		pl_cs_data->id = g_frame_id;
// 		pl_cs_data->isok = 1;
// 		pl_cs_data->state = state;
// 		memset(pl_cs_data->path, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
// 		pl_cs_data->number_of_effective_points = 0;
// 
// 		m_reverse_back_sum_dist = 0;
// 		m_reverse_forward_sum_dist = 0;
// 		m_breverse_flag = false;
// 		m_breverse_init = false;;
// 		ret = -1;
// 		return ret;
// 	}

	//[3.���ɵ����õľֲ�·����]
// 	COOR2 tmp1, tmp2;
// 	deque<COOR2> tmp_path;
// 	int num = 0;
// 	tmp1.x = 0;
// 	tmp1.y = 0;
// 	tmp_path.resize(m_reverse_pts.size());
// 	for (i = 0; i < (int)m_reverse_pts.size(); i++)
// 	{
// 		pt = m_reverse_pts[i];
// 		coor2_e2v(&gps, &pt, &tmp2);
// 		if (dist_point(&tmp1, &tmp2) >= m_reverse_plan_pts_step)
// 		{
// 			tmp_path[num++] = tmp2;
// 			tmp1 = tmp2;
// 		}
// 	}
// 	reverse_path.resize(num);
// 	for (i = 0; i < num; i++)
// 	{
// 		reverse_path[i] = tmp_path[i];
// 	}
// 	reverse(reverse_path.begin(), reverse_path.end());

	//[4.�����滮��]
	memset(m_Path, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	//memset(pts, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
	//pts_num = 0;
	for (i = 0; i < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; i++)
	{
		m_Path[i].x = 0;
		m_Path[i].y = i * (-50);
		//pts[i] = reverse_path[i];
	}
	//pts_num = i;
	//get_bezier_line(pts, pts_num, m_Path, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
	m_PathPtsNum = NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE;

	//[5.��ײ���]
	ret = collision_check_and_replan(grid_center);
	if (ret == 0)
	{
		pl_cs_data->speed = m_reverse_speed;
		pl_cs_data->sys_command |= SYS_COMMAND_REV;
		pl_cs_data->id = g_frame_id;
		pl_cs_data->isok = 1;
		pl_cs_data->state = state;
		memcpy(pl_cs_data->path, m_Path, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		pl_cs_data->number_of_effective_points = m_PathPtsNum;
	}
	else
	{
		pl_cs_data->speed = (int)(0 * CM);
		pl_cs_data->sys_command = 0;
		pl_cs_data->id = g_frame_id;
		pl_cs_data->isok = 1;
		pl_cs_data->state = state;
		memset(pl_cs_data->path, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		pl_cs_data->number_of_effective_points = 0;

		m_reverse_back_sum_dist = 0;
		m_reverse_forward_sum_dist = 0;
		m_breverse_flag = false;
		m_breverse_init = false;;
		ret = -1;
		return ret;
	}

	//[�������뵽����ֵ�����������滮]
	pt = gps.ins_coord;
	m_reverse_back_sum_dist += (int)dist_point(&m_reverse_back_last_pt, &pt);

	if (m_reverse_back_sum_dist > m_reverse_back_stop_dist)
	{
		pl_cs_data->speed = (int)(0 * CM);
		pl_cs_data->sys_command = 0;
		pl_cs_data->id = g_frame_id;
		pl_cs_data->isok = 1;
		pl_cs_data->state = state;
		memset(pl_cs_data->path, 0, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE * sizeof(COOR2));
		pl_cs_data->number_of_effective_points = 0;

		m_reverse_back_sum_dist = 0;
		m_reverse_forward_sum_dist = 0;
		m_breverse_flag = false;
		m_breverse_init = false;;
		ret = -1;
		return ret;
	}

	m_reverse_back_last_pt = gps.ins_coord;
#ifdef MBUG_OPEN_
	MBUG("\n");
	MBUG("---------------------reverse plan END-----------------------\n");
#endif
	return 0;
}