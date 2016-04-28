#include "plsdk.h"

namespace plsdk
{
	using namespace std;

	Fmt & Fmt::GetInstance()
	{
		static Fmt inst;
		return inst;
	}

	Fmt::Fmt()
	{
		prm_pl_data = nullptr;
		axisFmt = nullptr;
		axisPl = nullptr;
		fmtPath.clear();
		fmtLocal.clear();

		dist2aim = 0.0f;//Ŀ��ʵ�ʼ��Ϊ0.0
		dist_ctrl = 0.0f;//Ŀ����Ƽ����0.0
		speed_lim = 0.0;
		role = SELF;
		event = NONE;
		task = SERIAL;
		mode = START;
		signal = LOST;
	}

	Fmt::~Fmt()
	{

	}

	void Fmt::updateFmt(PRM_PL_DATA * data1, PL_PRM_DATA * data2, Axis * axisCord, Axis * axisPl)
	{
		this->prm_pl_data = data1;
		this->pl_prm_data = data2;
		this->axisFmt = axisCord;
		this->axisPl = axisPl;
		//׼������Эͬ������
		memcpy(pl_prm_data, prm_pl_data, sizeof(PRM_PL_DATA));
		doMaintainFmt();
		pl_prm_data->speed = (INT32)fmtCtrl.speed * 1000.0f / 36.0f;
		return;
	}

	void Fmt::doMaintainFmt()
	{
		//��prm����ñ�Ӳ�����Ϣ
		speed_lim = prm_pl_data->object[0].line_spd * 36.0f / 1000.0f;//�����ʻ�ٶȣ���λkm/h, ������10km/h 15km/h 20km/h 30km/h
		dist_ctrl = (float)prm_pl_data->dis;//������ࣺ��λcm, ������1500, 2000, 2500, 3000
		if (1500.0f == dist_ctrl) policy_num = 0;
		if (2000.0f == dist_ctrl) policy_num = 1;
		if (2500.0f == dist_ctrl) policy_num = 2;
		if (3000.0f == dist_ctrl) policy_num = 3;
		//��ó�����ɫ
		switch (prm_pl_data->robot_id)
		{
		case 0:
			role = SELF;
			break;
		case 1:
			role = MASTER;
			break;
		default:
			role = SLAVE;
			break;
		}
		//��ñ���������
		static bool task_entrance_flag = false;
		if (SERIAL == prm_pl_data->task_type && task != SERIAL)
		{
			task = SERIAL;
			task_entrance_flag = true;
		}
		else if (TRI == prm_pl_data->task_type && task != TRI)
		{
			task = TRI;
			task_entrance_flag = true;
		}
		else if (S2T == prm_pl_data->task_type && task != S2T)
		{
			task = S2T;
			task_entrance_flag = true;
		}
		else if (T2S == prm_pl_data->task_type && task != T2S)
		{
			task = T2S;
			task_entrance_flag = true;
		}
		else if (FREE == prm_pl_data->task_type && task != FREE)
		{
			task = FREE;
			task_entrance_flag = true;
		}
		else if (PATROL == prm_pl_data->task_type && task != PATROL)
		{
			task = PATROL;
			task_entrance_flag = true;
		}
		else if (HUNT == prm_pl_data->task_type && task != HUNT)
		{
			task = HUNT;
			task_entrance_flag = true;
		}
		//��ñ�ӿ����¼�
		event = prm_pl_data->event;
		//���ͨ��״̬
		signal = prm_pl_data->signal;
		//�����ʻģʽ
		mode = prm_pl_data->mode;
		//��ǰ��������������
		OBJECT_INFO & info = prm_pl_data->object[(unsigned int)prm_pl_data->robot_id];
		Target target;
		target.x = (float)info.x;
		target.y = (float)info.y;
		target.speed = info.line_spd * 36.0f / 1000.0f;
		target.yaw = (float)info.z;//!!!!!�����洢��ǰ���Ƿ��ǹ����
		//����������Ч�������㣬���Ի�����н��д���
		bool point_valid = (1 == info.valid) && (fmtPath.empty() || (target.x != fmtPath.back().x || target.y != fmtPath.back().y));
		if (true == point_valid)
		{
			if ((SERIAL == task) || (TRI == task))//���봮�б�ӻ������α��ʱ����������
			{
				fmtPath.push_back(target);
			}
			if ((S2T == task) || (T2S == task))//����S2T����л�
			{
				if (false == task_entrance_flag)
				{
					fmtPath.push_back(target);
				}
				if (true == task_entrance_flag && !fmtPath.empty())
				{
					Target start_point = fmtPath.front();
					fmtPath.clear();
					LinearInterpolation(start_point, target, 100.0f, fmtPath);//ֱ�������Բ��,��·���嵽Ŀ��λ��
					task_entrance_flag = false;
				}
			}
			if (FREE == task)
			{
				fmtPath.clear();
			}
			if (PATROL == task)
			{
				fmtPath.push_back(target);
			}
			if (HUNT == task)
			{
				if (false == task_entrance_flag)
				{
					fmtPath.push_back(target);
				}
				if (true == task_entrance_flag && !fmtPath.empty())
				{
					Target start_point = fmtPath.front();
					fmtPath.clear();
					LinearInterpolation(start_point, target, 100.0f, fmtPath);
					task_entrance_flag = false;
				}
			}
		}
		//��fmt_path���²�ֵ����ֹ����������������������ж�
		if (fmtPath.size() >= 2)
		{
			auto itr2 = fmtPath.begin();
			auto itr1 = itr2++;
			while (itr2 != fmtPath.end())
			{
				float d = sqrt(pow(itr1->x - itr2->x, 2) + pow(itr1->y - itr2->y, 2));
				if (d >= 100.0f * 1.9f)//������֮�������Բ���һ���㣬��ô�Ͳ���һ����
					itr2 = fmtPath.insert(itr2, Target{ (itr2->x + itr1->x) / 2, (itr2->y + itr1->y) / 2, itr1->yaw, itr1->speed });
				else
					itr1 = itr2++;
			}
		}
		//��ЭͬĿ���ת��������������,���Ҽ��㵽Ŀ���ľ���
		while (!fmtPath.empty())//�޳�300���ڵĵ�
		{
			Target t = fmtPath.front();
			Earth2Car(axisPl, &t.x, &t.y);
			if (t.y < 300.0f && abs(t.x) < 450.0f)
				fmtPath.pop_front();
			else
				break;
		}
		fmtLocal.clear();//��ճ��������µ�·��
		dist2aim = 0.0f;//��������Ϊ0.0��
		if (!fmtPath.empty())
		{
			Target now = fmtPath.front();//Ĭ�ϴӳ���λ�ÿ�ʼ����
			Earth2Car(axisPl, &now.x, &now.y);
			Target t;
			for (const Target & var : fmtPath)
			{
				t = var;
				Earth2Car(axisPl, &t.x, &t.y);
				dist2aim += sqrt(pow(t.x - now.x, 2) + pow(t.y - now.y, 2));
				fmtLocal.push_back(t);
				now = var;
			}
			/*
			auto itr = fmtPath.begin();
			while (itr != fmtPath.end())
			{
				//����Ҫת��һ���ٶȵ�λ��cm/sת����Ϊkm/h
				Target t{ itr->x, itr->y, itr->yaw, itr->speed };
				Earth2Car(axisPl, &t.x, &t.y);
				dist2aim += sqrt(pow(t.x - now.x, 2) + pow(t.y - now.y, 2));
				now = t;
				fmtLocal.push_back(now);
				++itr;
			}*/
		}
		
		//����ѡ��Ŀ����
		doExtractRefPointV4();
		//ά��isBlocked״̬�����ݶ�����������ٶ�
		Map & map = Map::GetInstance();
		float dist_collision = 0.0f;
		isBlocked = map.detectCollision(fmtLocal, dist_collision, EXPAND);
		if (true == isBlocked)
		{
			fmtCtrl.speed = SpeedLimitDecisionByDis(dist_collision, fmtCtrl.speed);//�����ϰ����Զ�������ٶȣ��ܹ����ϰ���ǰͣ��
		}

		return;
	}

	void Fmt::doExtractRefPointV4()
	{
		// 1 ��ʼ����صı����Ͳ���
		Parameter & param = Parameter::GetInstance();//��ò���������
		isFmtCtrlValid = false;//Ĭ������Ч��fmt��

		// 2 ����ͨ���쳣��ͨ���쳣��ͣ��
		if (LOST == signal) return;//ͨ���жϣ���������״̬

		// 3 ���ݽ�ɫ����ʻ����,������ʻĿ�����ʻ�ٶ�
		float look_ahead = 550.0f;//Ĭ�Ͽ���ͷǰ��550cmλ��
		float speed_ahead = param.speed_max;//Ĭ�ϰ�ϵͳ�������
		static float policy_table_fmt[4][7] = \
		{
			{ 900.00f, 1000.0f, 1300.0f, 1500.0f, 1700.0f, 2000.0f, 10000000.0f },  // 15km/h�ȼ��ļ�����
			{ 1100.0f, 1300.0f, 1600.0f, 1800.0f, 2000.0f, 2500.0f, 10000000.0f },  // 20km/h�ȼ��ļ�����
			{ 1300.0f, 1600.0f, 1900.0f, 2100.0f, 2300.0f, 3000.0f, 10000000.0f },  // 25km/h�ȼ��ļ�����
			{ 1500.0f, 1900.0f, 2200.0f, 2400.0f, 3000.0f, 3500.0f, 10000000.0f }	// 30km/h�ȼ��ļ�����
		};
		//>>>>>���кͲ��ж����µ�master
		if ((SERIAL == task || TRI == task) && MASTER == role)
		{
			Gis & gis = Gis::GetInstance();
			//��Ԥ��켣�г�ȡĿ������ʻ����
			fmtCtrl = gis.gisCtrl;
			isFmtCtrlValid = gis.isGisCtrlValid;
			//��������
			if (true == gis.isBend)
				pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
			//����ģʽ�£��ٶ�������7.0f
			if (START == mode) SpeedLimit(fmtCtrl.speed, 8.0f);
			//�г�δ��ɱ��,��������ٶ�����Ϊ0.75������ٶ�
			if (DROP == event) SpeedLimit(fmtCtrl.speed, 0.75f*speed_lim);
			//����Эͬ��Ӳ��ԶԳ�����������
			SpeedLimit(fmtCtrl.speed, speed_lim);
			//���ݳ��������ʻ�ٶ�
			SpeedLimit(fmtCtrl.speed, param.speed_max);
		}
		//>>>>>���кͲ��ж����µ�slave
		if ((SERIAL == task || TRI == task) && SLAVE == role)
		{
			//ǰ��û�е��ˣ�������ͣ��
			if (fmtLocal.empty())	return;
			if (START == mode)
			{
				if (dist2aim <= policy_table_fmt[policy_num][0])
					speed_ahead = 0.0f;
				else
					speed_ahead = 8.0f - (float)prm_pl_data->robot_id;//�𳵵ݼ��ٶȣ���������
				pl_prm_data->event = GOOD;
			}
			else
			{
				int area = 1;//��������ƥ�䣬ȷ����ǰ���Ĳ��Ա��
				while (area != 7)
				{
					float xx = policy_table_fmt[policy_num][area];
					if (xx <= dist2aim)
						area++;
					else
						break;
				}
				switch (area)
				{
				case 1:		//����ͣ����
					speed_ahead = 0.0f;
					pl_prm_data->event = GOOD;
					break;
				case 2:		//���ٵ�����
					speed_ahead = (prm_pl_data->speed * 36.0f / 1000.0f) * 0.8f; //����ǰ���ٶȵ�80%
					if (1.0f == fmtLocal.front().yaw) //��ǰ��������㣬Ҫ������,������������
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					pl_prm_data->event = GOOD;
					break;
				case 3:		//�ٶȱ�����1
					speed_ahead = prm_pl_data->speed * 36.0f / 1000.0f - 2.0f;
					if (1.0f == fmtLocal.front().yaw) //��ǰ��������㣬Ҫ������
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//���������Ҫ���ٶȱ���
					{
						//ǰ���Ѿ�ͣ������������Ҫ��֤����
						SpeedKeep(speed_ahead, 5.0f);
					}
					pl_prm_data->event = GOOD;
					break;
				case 4:		//�ٶȱ�����2
					speed_ahead = prm_pl_data->speed * 36.0f / 1000.0f + 2.0f;
					if (1.0f == fmtLocal.front().yaw)//��ǰ��������㣬Ҫ������
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//���������Ҫ���ٶȱ���
					{
						//ǰ���Ѿ�ͣ������������Ҫ��֤����
						SpeedKeep(speed_ahead, 7.0f);
					}
					pl_prm_data->event = GOOD;
					break;
				case 5:			//���ٵ�����
					speed_ahead = (prm_pl_data->speed * 36.0f / 1000.0f)*1.2f;
					if (1.0f == fmtLocal.front().yaw)//��ǰ��������㣬Ҫ������
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//���������Ҫ���ٶȱ���
					{
						//ǰ���Ѿ�ͣ������������Ҫ��֤����
						SpeedKeep(speed_ahead, 8.0f);
					}
					pl_prm_data->event = GOOD;
					break;
				case 6:
					speed_ahead = (prm_pl_data->speed * 36.0f / 1000.0f)*1.5f;
					if (1.0f == fmtLocal.front().yaw)//��ǰ��������㣬Ҫ������
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//b������������׷��
					{
						//ǰ���Ѿ�ͣ������������Ҫ��֤����
						SpeedKeep(speed_ahead, 12.0f);
					}
					pl_prm_data->event = DROP;
				default:
					break;
				}
			}
			SpeedLimit(speed_ahead, speed_lim);
			SpeedLimit(speed_ahead, param.speed_max);
			//����look_ahead����look_ahead��Ѱ��һ�����ʵ�Ŀ���
			//Note: 2015-08-22 ���̶����������Ըı��Ϊ550.0cm��10������Ѱ��һ���Ƕ���С�ĵ�(ʯ��ʦ)
			//		��������⣺�����㷨�����ϸ���ٵ㣬���ǿ���Ӧ��·�����ֺ���Ư�Ƶ㡣
			//		���ڵ����⣺�ڹ��䴦���ֵ�Ư�Ƶ��޷�����S����������޷��ϸ���ٹ켣
			float angle_less = 15.0f;//Ĭ�ϽǶ�(��abs(x/y)��ʾ)�ܴ�
			int counter = 0;
			for (const Target & tar : fmtLocal)
			{
				float point_dist = sqrt(pow(tar.y, 2) + pow(tar.x, 2));
				if (point_dist >= look_ahead)
				{
					if (angle_less >= abs(tar.x / tar.y))
					{
						fmtCtrl = tar;
						fmtCtrl.speed = speed_ahead;
						isFmtCtrlValid = true;
						angle_less = abs(tar.x / tar.y);
					}
					++counter;
					if (counter > 5) break;
				}
			}
		}
		//>>>>>���ж���任��Ʒ���� MASTER
		if (S2T == task && MASTER == role)
		{
			Gis & gis = Gis::GetInstance();
			fmtCtrl = gis.gisCtrl;
			isFmtCtrlValid = gis.isGisCtrlValid;
			SpeedLimit(fmtCtrl.speed, 8.0f);//�任���ο����ٶ�Ϊ8.0f
		}
		//>>>>>���ж���任��Ʒ���� SLAVE
		if (S2T == task && SLAVE == role)
		{
			//���ڱ��ʱ���ٶȿ���
			switch (prm_pl_data->robot_id)
			{
			case 1:
				speed_ahead = 8.0f;//һ�ų��ٶȵ�����8km/h
				break;
			case 2:
				speed_ahead = 9.0f;//���ų��ٶȵ�����10km/h
				break;
			case 3:
				speed_ahead = 13.0f;//���ų��ٶȵ�����13km/h
				break;
			default:
				speed_ahead = 0.0f;//�쳣���ͣ��
				break;
			}
			//�������ӽ�Ŀ��ʱ���ٶ�����Ϊ8.0f
			if (dist2aim < dist_ctrl) SpeedLimit(speed_ahead, 8.0f);
			//����look_ahead����look_ahead��Ѱ��һ�����ʵ�Ŀ���
			//Note: 2015-08-22 ���̶����������Ըı��Ϊ550.0cm��10������Ѱ��һ���Ƕ���С�ĵ�(ʯ��ʦ)
			//		��������⣺�����㷨�����ϸ���ٵ㣬���ǿ���Ӧ��·�����ֺ���Ư�Ƶ㡣
			//		���ڵ����⣺�ڹ��䴦���ֵ�Ư�Ƶ��޷�����S����������޷��ϸ���ٹ켣
			float angle_less = 15.0f;//Ĭ�ϽǶ�(��abs(x/y)��ʾ)�ܴ�
			int counter = 0;
			for (const Target & tar : fmtLocal)
			{
				float point_dist = sqrt(pow(tar.x, 2) + pow(tar.y, 2));
				if (point_dist >= look_ahead)
				{
					if (angle_less >= abs(tar.x / tar.y))
					{
						fmtCtrl = tar;
						fmtCtrl.speed = speed_ahead;
						isFmtCtrlValid = true;
						angle_less = abs(tar.x / tar.y);
					}
					++counter;
					if (counter > 10) break;
				}
			}

		}
		//Ʒ���ζ���䴮�У�MASTER
		if (T2S == task && MASTER == role)
		{

		}
		//Ʒ���ζ���䴮�У�SLAVE
		if (T2S == task && SLAVE == role)
		{

		}
		//���ɻ�����
		if (FREE == task)
		{
			return;
		}
		//Ѳ�߻�����
		if (PATROL == task)
		{
			Gis & gis = Gis::GetInstance();
			//��Ԥ��켣�г�ȡĿ������ʻ����
			fmtCtrl = gis.gisCtrl;
			isFmtCtrlValid = gis.isGisCtrlValid;
			//����ģʽ�£��ٶ�������8.0f
			if (START == mode) SpeedLimit(fmtCtrl.speed, 8.0f);
			//����Эͬ��Ӳ��ԶԳ�����������
			SpeedLimit(fmtCtrl.speed, speed_lim);
			//���ݳ��������ʻ�ٶ�
			SpeedLimit(fmtCtrl.speed, param.speed_max);
		}
		//Χ��ģʽ
		if (HUNT == task)
		{
			//ǰ��û�е��ˣ�������ͣ��
			if (fmtLocal.empty()) return;
			if (START == mode)
			{
				if (dist2aim <= policy_table_fmt[policy_num][0])
					speed_ahead = 0.0f;
				else
					speed_ahead = 8.0f - (float)prm_pl_data->robot_id;//�𳵵ݼ��ٶȣ���������
				pl_prm_data->event = GOOD;
			}
			else
			{
				int area = 1;//��������ƥ�䣬ȷ����ǰ���Ĳ��Ա��
				while (area != 7)
				{
					float xx = policy_table_fmt[policy_num][area];
					if (xx <= dist2aim)
						area++;
					else
						break;
				}
				switch (area)
				{
				case 1:		//����ͣ����
					speed_ahead = 0.0f;
					pl_prm_data->event = GOOD;
					break;
				case 2:		//���ٵ�����
					speed_ahead = (prm_pl_data->speed * 36.0f / 1000.0f) * 0.8f; //����ǰ���ٶȵ�80%
					if (1.0f == fmtLocal.front().yaw) //��ǰ��������㣬Ҫ������,������������
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					pl_prm_data->event = GOOD;
					break;
				case 3:		//�ٶȱ�����1
					speed_ahead = prm_pl_data->speed * 36.0f / 1000.0f - 2.0f;
					if (1.0f == fmtLocal.front().yaw) //��ǰ��������㣬Ҫ������
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//���������Ҫ���ٶȱ���
					{
						//ǰ���Ѿ�ͣ������������Ҫ��֤����
						SpeedKeep(speed_ahead, 7.0f);
					}
					pl_prm_data->event = GOOD;
					break;
				case 4:		//�ٶȱ�����2
					speed_ahead = prm_pl_data->speed * 36.0f / 1000.0f + 2.0f;
					if (1.0f == fmtLocal.front().yaw)//��ǰ��������㣬Ҫ������
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//���������Ҫ���ٶȱ���
					{
						//ǰ���Ѿ�ͣ������������Ҫ��֤����
						SpeedKeep(speed_ahead, 8.0f);
					}
					pl_prm_data->event = GOOD;
					break;
				case 5:			//���ٵ�����
					speed_ahead = (prm_pl_data->speed * 36.0f / 1000.0f)*1.2f;
					if (1.0f == fmtLocal.front().yaw)//��ǰ��������㣬Ҫ������
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//���������Ҫ���ٶȱ���
					{
						//ǰ���Ѿ�ͣ������������Ҫ��֤����
						SpeedKeep(speed_ahead, 10.0f);
					}
					pl_prm_data->event = GOOD;
					break;
				case 6:
					speed_ahead = (prm_pl_data->speed * 36.0f / 1000.0f)*1.5f;
					if (1.0f == fmtLocal.front().yaw)//��ǰ��������㣬Ҫ������
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//b������������׷��
					{
						//ǰ���Ѿ�ͣ������������Ҫ��֤����
						SpeedKeep(speed_ahead, 12.0f);
					}
					pl_prm_data->event = DROP;
				default:
					break;
				}
			}
			SpeedLimit(speed_ahead, speed_lim);
			SpeedLimit(speed_ahead, param.speed_max);
			//����look_ahead����look_ahead��Ѱ��һ�����ʵ�Ŀ���
			//Note: 2015-08-22 ���̶����������Ըı��Ϊ550.0cm��10������Ѱ��һ���Ƕ���С�ĵ�(ʯ��ʦ)
			//		��������⣺�����㷨�����ϸ���ٵ㣬���ǿ���Ӧ��·�����ֺ���Ư�Ƶ㡣
			//		���ڵ����⣺�ڹ��䴦���ֵ�Ư�Ƶ��޷�����S����������޷��ϸ���ٹ켣
			float angle_less = 15.0f;//Ĭ�ϽǶ�(��abs(x/y)��ʾ)�ܴ�
			int counter = 0;
			for (const Target & tar : fmtLocal)
			{
				float point_dist = sqrt(pow(tar.y, 2) + pow(tar.x, 2));
				if (point_dist >= look_ahead)
				{
					if (angle_less >= abs(tar.x / tar.y))
					{
						fmtCtrl = tar;
						fmtCtrl.speed = speed_ahead;
						isFmtCtrlValid = true;
						angle_less = abs(tar.x / tar.y);
					}
					++counter;
					if (counter > 5) break;
				}
			}
		}

		return;
	}

}