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
		//����ת���б�
		for (int i = 0; i != MORPHIN_TEST_NUM; ++i)
			angle_list[i] = MORPHIN_TEST_L - MORPHIN_TEST_STEP*i;
		//����ÿ����ͨ����դ��㣬��˳���д���
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
		// 0 ����Ŀ��������դ��λ�ã��ж�Ŀ����Ƿ��ڲ���ִ�е����
		int goal_x = tar.x / MAP_DL_CELL_SIZ + MAP_DL_CELL_HOR / 2, goal_y = tar.y / MAP_DL_CELL_SIZ;
		// 1 ��ӳ�ͷλ�ó�����morphin���Ե���󳤶�
		int test_dist = abs(tar.x / MAP_DL_CELL_SIZ) + abs(tar.y / MAP_DL_CELL_SIZ);
		if (test_dist > MORPHIN_TEST_MAX_LEN) test_dist = MORPHIN_TEST_NUM;//�����Ե����Χ������mophin��󳤶�����
		// 2 ����morphin����ʻ����, ������Morphin���������Գ���������ײ��morphin�ߵ�valueȫ����Ϊ1.0f, ������Ϊ��ֵ
		for (const Morph * m_ptr : morphin)
		{
			float & value = value_list[m_ptr->idx];
			int n1 = 100;//Ĭ����Ҫ100��
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
			//���δ�ܴ����ϰ����ֵ��Ϊ���ն���
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
				//��ÿһ��morphin�߼���һ�����յķ������ۣ����������ɵ�ǰ����ײ���ۺ�����Ŀ���������ɣ���������Ŀ���ռ0.3
				value = value * 0.0f + (n1 / 100.0f)*1.0f;
				//���������˸��ŵ�ֵ������ø��ŵ�ֵ
				if (value < best_value)
				{
					ret = m_ptr->idx;
					best_value = value;
				}
			}
		}
		//ѡ��һ��������õ�����������в�������⵽��ײ����
		best_morph = ret;
		*/
		return ret;
	}
}