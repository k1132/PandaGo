#include "plsdk.h"

namespace plsdk
{
	using namespace std;

	Morphin & Morphin::GetInstance()
	{
		static Morphin inst;
		return inst;
	}

	Morphin::Morphin()
	{
		memset(morphin, 0, sizeof(Morph *)*MORPHIN_TEST_NUM);
		//生成转角列表
		for (int i = 0; i != MORPHIN_TEST_NUM; ++i)
			angle_list[i] = MORPHIN_TEST_L - MORPHIN_TEST_STEP*i;
		//计算每条线通过的栅格点，按顺排列存入
		for (int i = 0; i != MORPHIN_TEST_NUM; ++i)
		{
			morphin[i] = new Morph;
			memset(morphin[i], 0, sizeof(Morph));
			morphin[i]->idx = i;
			morphin[i]->angle = MORPHIN_TEST_L - MORPHIN_TEST_STEP*i;
			EnumMorphinPos(morphin[i], MORPHIN_TEST_MAX_LEN, 285.0f);
		}
		best_morph = MORPHIN_TEST_NUM / 2;
	}

	Morphin::~Morphin()
	{
		//TODO
		for (Morph * ptr : morphin)
		{
			delete ptr;
		}
	}

	int Morphin::selectMorphinLineV2(const float * map, const Target tar)
	{
		int ret = -1;
		/*
		float best_value = 1.0f;
		// 0 计算目标点的虚拟栅格位置，判断目标点是否处于不能执行的情况
		int goal_x = tar.x / MAP_DL_CELL_SIZ + MAP_DL_CELL_HOR / 2, goal_y = tar.y / MAP_DL_CELL_SIZ;
		// 1 求从车头位置出发，morphin测试的最大长度
		int test_dist = abs(tar.x / MAP_DL_CELL_SIZ) + abs(tar.y / MAP_DL_CELL_SIZ);
		if (test_dist > MORPHIN_TEST_MAX_LEN) test_dist = MORPHIN_TEST_NUM;//将测试的最大范围限制在mophin最大长度以内
		// 2 评价morphin的行驶风险, 将所有Morphin线中最大测试长度内有碰撞的morphin线的value全部置为1.0f, 其他记为均值
		for (const Morph * m_ptr : morphin)
		{
			float & value = value_list[m_ptr->idx];
			int n1 = 100;//默认需要100步
			bool hit = false;
			int steps = test_dist;
			if (steps > m_ptr->len) steps = m_ptr->len;
			for (int cnt = 0; cnt != steps; ++cnt)
			{
				int map_pos = m_ptr->pos[cnt].x + m_ptr->pos[cnt].y*MAP_DL_CELL_HOR;
				if (map[map_pos] < 255.0f)
				{
					value += map[map_pos];
				}
				else
				{
					value = 1.0f;
					hit = true;
					break;
				}

			}
			//如果未能穿过障碍物，则将值变为风险度量
			if (false == hit)
			{
				value /= m_ptr->len;
				int _n1 = abs(sqrt(pow(tar.x - m_ptr->o.x, 2) + pow(tar.y - m_ptr->o.y, 2)) - m_ptr->r) / MAP_DL_CELL_SIZ;
				if (_n1 < n1) n1 = _n1;
				
				//for (int cnt = 0; cnt != m_ptr->len; ++cnt)
				//{
				//int _n1 = abs(m_ptr->pos[cnt].x - goal_x) + abs(m_ptr->pos[cnt].y - goal_y);
				//if (_n1 < n1)	n1 = _n1;
				//}
				//对每一条morphin线计算一个最终的风险评价，风险评价由当前的碰撞评价和趋近目标点评价组成，其中趋近目标点占0.3
				value = value * 0.0f + (n1 / 100.0f)*1.0f;
				//如果计算出了更优的值，则采用更优的值
				if (value < best_value)
				{
					ret = m_ptr->idx;
					best_value = value;
				}
			}
		}
		//选择一条评价最好的线输出，其中不包括检测到碰撞的线
		best_morph = ret;
		*/
		return ret;
	}
}