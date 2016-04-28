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

		dist2aim = 0.0f;//目标实际间距为0.0
		dist_ctrl = 0.0f;//目标控制间距是0.0
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
		//准备返回协同的数据
		memcpy(pl_prm_data, prm_pl_data, sizeof(PRM_PL_DATA));
		doMaintainFmt();
		pl_prm_data->speed = (INT32)fmtCtrl.speed * 1000.0f / 36.0f;
		return;
	}

	void Fmt::doMaintainFmt()
	{
		//从prm处获得编队策略信息
		speed_lim = prm_pl_data->object[0].line_spd * 36.0f / 1000.0f;//最高行驶速度：单位km/h, 可能是10km/h 15km/h 20km/h 30km/h
		dist_ctrl = (float)prm_pl_data->dis;//车辆间距：单位cm, 可能是1500, 2000, 2500, 3000
		if (1500.0f == dist_ctrl) policy_num = 0;
		if (2000.0f == dist_ctrl) policy_num = 1;
		if (2500.0f == dist_ctrl) policy_num = 2;
		if (3000.0f == dist_ctrl) policy_num = 3;
		//获得车辆角色
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
		//获得编队任务类别
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
		//获得编队控制事件
		event = prm_pl_data->event;
		//获得通信状态
		signal = prm_pl_data->signal;
		//获得行驶模式
		mode = prm_pl_data->mode;
		//对前方引导点做处理
		OBJECT_INFO & info = prm_pl_data->object[(unsigned int)prm_pl_data->robot_id];
		Target target;
		target.x = (float)info.x;
		target.y = (float)info.y;
		target.speed = info.line_spd * 36.0f / 1000.0f;
		target.yaw = (float)info.z;//!!!!!用来存储当前点是否是拐弯点
		//输入点如果有效，则插入点，并对缓存队列进行处理
		bool point_valid = (1 == info.valid) && (fmtPath.empty() || (target.x != fmtPath.back().x || target.y != fmtPath.back().y));
		if (true == point_valid)
		{
			if ((SERIAL == task) || (TRI == task))//进入串行编队或三角形编队时，正常处理
			{
				fmtPath.push_back(target);
			}
			if ((S2T == task) || (T2S == task))//进入S2T编队切换
			{
				if (false == task_entrance_flag)
				{
					fmtPath.push_back(target);
				}
				if (true == task_entrance_flag && !fmtPath.empty())
				{
					Target start_point = fmtPath.front();
					fmtPath.clear();
					LinearInterpolation(start_point, target, 100.0f, fmtPath);//直接做线性插点,将路径插到目标位置
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
		//对fmt_path重新插值，防止点间距产生过大噪声而干扰判断
		if (fmtPath.size() >= 2)
		{
			auto itr2 = fmtPath.begin();
			auto itr1 = itr2++;
			while (itr2 != fmtPath.end())
			{
				float d = sqrt(pow(itr1->x - itr2->x, 2) + pow(itr1->y - itr2->y, 2));
				if (d >= 100.0f * 1.9f)//当两者之间距离可以插入一个点，那么就插入一个点
					itr2 = fmtPath.insert(itr2, Target{ (itr2->x + itr1->x) / 2, (itr2->y + itr1->y) / 2, itr1->yaw, itr1->speed });
				else
					itr1 = itr2++;
			}
		}
		//将协同目标点转换到车体坐标下,并且计算到目标点的距离
		while (!fmtPath.empty())//剔除300以内的点
		{
			Target t = fmtPath.front();
			Earth2Car(axisPl, &t.x, &t.y);
			if (t.y < 300.0f && abs(t.x) < 450.0f)
				fmtPath.pop_front();
			else
				break;
		}
		fmtLocal.clear();//清空车体坐标下的路线
		dist2aim = 0.0f;//将车距置为0.0；
		if (!fmtPath.empty())
		{
			Target now = fmtPath.front();//默认从车体位置开始计算
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
				//这里要转换一下速度单位将cm/s转换成为km/h
				Target t{ itr->x, itr->y, itr->yaw, itr->speed };
				Earth2Car(axisPl, &t.x, &t.y);
				dist2aim += sqrt(pow(t.x - now.x, 2) + pow(t.y - now.y, 2));
				now = t;
				fmtLocal.push_back(now);
				++itr;
			}*/
		}
		
		//处理选择目标点的
		doExtractRefPointV4();
		//维护isBlocked状态，根据堵塞情况修正速度
		Map & map = Map::GetInstance();
		float dist_collision = 0.0f;
		isBlocked = map.detectCollision(fmtLocal, dist_collision, EXPAND);
		if (true == isBlocked)
		{
			fmtCtrl.speed = SpeedLimitDecisionByDis(dist_collision, fmtCtrl.speed);//根据障碍物的远近控制速度，能够在障碍物前停下
		}

		return;
	}

	void Fmt::doExtractRefPointV4()
	{
		// 1 初始化相关的变量和参数
		Parameter & param = Parameter::GetInstance();//获得参数配置器
		isFmtCtrlValid = false;//默认无有效的fmt点

		// 2 处理通行异常，通信异常则停车
		if (LOST == signal) return;//通信中断，则进入待命状态

		// 3 根据角色和行驶任务,决定行驶目标和行驶速度
		float look_ahead = 550.0f;//默认看车头前方550cm位置
		float speed_ahead = param.speed_max;//默认按系统最大车速走
		static float policy_table_fmt[4][7] = \
		{
			{ 900.00f, 1000.0f, 1300.0f, 1500.0f, 1700.0f, 2000.0f, 10000000.0f },  // 15km/h等级的间距策略
			{ 1100.0f, 1300.0f, 1600.0f, 1800.0f, 2000.0f, 2500.0f, 10000000.0f },  // 20km/h等级的间距策略
			{ 1300.0f, 1600.0f, 1900.0f, 2100.0f, 2300.0f, 3000.0f, 10000000.0f },  // 25km/h等级的间距策略
			{ 1500.0f, 1900.0f, 2200.0f, 2400.0f, 3000.0f, 3500.0f, 10000000.0f }	// 30km/h等级的间距策略
		};
		//>>>>>串行和并行队形下的master
		if ((SERIAL == task || TRI == task) && MASTER == role)
		{
			Gis & gis = Gis::GetInstance();
			//从预设轨迹中抽取目标点和行驶策略
			fmtCtrl = gis.gisCtrl;
			isFmtCtrlValid = gis.isGisCtrlValid;
			//留弯道标记
			if (true == gis.isBend)
				pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
			//在起步模式下，速度限制在7.0f
			if (START == mode) SpeedLimit(fmtCtrl.speed, 8.0f);
			//有车未完成编队,车辆最高速度设置为0.75倍最高速度
			if (DROP == event) SpeedLimit(fmtCtrl.speed, 0.75f*speed_lim);
			//根据协同编队策略对车辆进行限速
			SpeedLimit(fmtCtrl.speed, speed_lim);
			//根据车辆最高行驶速度
			SpeedLimit(fmtCtrl.speed, param.speed_max);
		}
		//>>>>>串行和并行队形下的slave
		if ((SERIAL == task || TRI == task) && SLAVE == role)
		{
			//前方没有点了，做紧急停车
			if (fmtLocal.empty())	return;
			if (START == mode)
			{
				if (dist2aim <= policy_table_fmt[policy_num][0])
					speed_ahead = 0.0f;
				else
					speed_ahead = 8.0f - (float)prm_pl_data->robot_id;//逐车递减速度，拉开距离
				pl_prm_data->event = GOOD;
			}
			else
			{
				int area = 1;//进行上限匹配，确定当前间距的策略编号
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
				case 1:		//紧急停车区
					speed_ahead = 0.0f;
					pl_prm_data->event = GOOD;
					break;
				case 2:		//减速调整区
					speed_ahead = (prm_pl_data->speed * 36.0f / 1000.0f) * 0.8f; //按照前车速度的80%
					if (1.0f == fmtLocal.front().yaw) //当前点是弯道点，要做限速,并留下弯道标记
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					pl_prm_data->event = GOOD;
					break;
				case 3:		//速度保持区1
					speed_ahead = prm_pl_data->speed * 36.0f / 1000.0f - 2.0f;
					if (1.0f == fmtLocal.front().yaw) //当前点是弯道点，要做限速
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//不是弯道点要做速度保持
					{
						//前车已经停车，本车还是要保证距离
						SpeedKeep(speed_ahead, 5.0f);
					}
					pl_prm_data->event = GOOD;
					break;
				case 4:		//速度保持区2
					speed_ahead = prm_pl_data->speed * 36.0f / 1000.0f + 2.0f;
					if (1.0f == fmtLocal.front().yaw)//当前点是弯道点，要做限速
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//不是弯道点要做速度保持
					{
						//前车已经停车，本车还是要保证距离
						SpeedKeep(speed_ahead, 7.0f);
					}
					pl_prm_data->event = GOOD;
					break;
				case 5:			//加速调整区
					speed_ahead = (prm_pl_data->speed * 36.0f / 1000.0f)*1.2f;
					if (1.0f == fmtLocal.front().yaw)//当前点是弯道点，要做限速
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//不是弯道点要做速度保持
					{
						//前车已经停车，本车还是要保证距离
						SpeedKeep(speed_ahead, 8.0f);
					}
					pl_prm_data->event = GOOD;
					break;
				case 6:
					speed_ahead = (prm_pl_data->speed * 36.0f / 1000.0f)*1.5f;
					if (1.0f == fmtLocal.front().yaw)//当前点是弯道点，要做限速
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//b不是弯道则快速追赶
					{
						//前车已经停车，本车还是要保证距离
						SpeedKeep(speed_ahead, 12.0f);
					}
					pl_prm_data->event = DROP;
				default:
					break;
				}
			}
			SpeedLimit(speed_ahead, speed_lim);
			SpeedLimit(speed_ahead, param.speed_max);
			//根据look_ahead，在look_ahead内寻找一个合适的目标点
			//Note: 2015-08-22 将固定距离瞄点策略改变成为550.0cm的10个点内寻找一个角度最小的点(石老师)
			//		解决的问题：这样算法不再严格跟踪点，但是可以应付路径出现横向漂移点。
			//		存在的问题：在拐弯处出现的漂移点无法处理，S弯跟点会出现无法严格跟踪轨迹
			float angle_less = 15.0f;//默认角度(用abs(x/y)表示)很大
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
		//>>>>>串行队伍变换成品字形 MASTER
		if (S2T == task && MASTER == role)
		{
			Gis & gis = Gis::GetInstance();
			fmtCtrl = gis.gisCtrl;
			isFmtCtrlValid = gis.isGisCtrlValid;
			SpeedLimit(fmtCtrl.speed, 8.0f);//变换队形控制速度为8.0f
		}
		//>>>>>串行队伍变换成品字形 SLAVE
		if (S2T == task && SLAVE == role)
		{
			//正在编队时的速度控制
			switch (prm_pl_data->robot_id)
			{
			case 1:
				speed_ahead = 8.0f;//一号车速度调整到8km/h
				break;
			case 2:
				speed_ahead = 9.0f;//二号车速度调整到10km/h
				break;
			case 3:
				speed_ahead = 13.0f;//三号车速度调整到13km/h
				break;
			default:
				speed_ahead = 0.0f;//异常情况停车
				break;
			}
			//当车辆接近目标时将速度限制为8.0f
			if (dist2aim < dist_ctrl) SpeedLimit(speed_ahead, 8.0f);
			//根据look_ahead，在look_ahead内寻找一个合适的目标点
			//Note: 2015-08-22 将固定距离瞄点策略改变成为550.0cm的10个点内寻找一个角度最小的点(石老师)
			//		解决的问题：这样算法不再严格跟踪点，但是可以应付路径出现横向漂移点。
			//		存在的问题：在拐弯处出现的漂移点无法处理，S弯跟点会出现无法严格跟踪轨迹
			float angle_less = 15.0f;//默认角度(用abs(x/y)表示)很大
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
		//品字形队伍变串行：MASTER
		if (T2S == task && MASTER == role)
		{

		}
		//品字形队伍变串行：SLAVE
		if (T2S == task && SLAVE == role)
		{

		}
		//自由机器人
		if (FREE == task)
		{
			return;
		}
		//巡逻机器人
		if (PATROL == task)
		{
			Gis & gis = Gis::GetInstance();
			//从预设轨迹中抽取目标点和行驶策略
			fmtCtrl = gis.gisCtrl;
			isFmtCtrlValid = gis.isGisCtrlValid;
			//在起步模式下，速度限制在8.0f
			if (START == mode) SpeedLimit(fmtCtrl.speed, 8.0f);
			//根据协同编队策略对车辆进行限速
			SpeedLimit(fmtCtrl.speed, speed_lim);
			//根据车辆最高行驶速度
			SpeedLimit(fmtCtrl.speed, param.speed_max);
		}
		//围捕模式
		if (HUNT == task)
		{
			//前方没有点了，做紧急停车
			if (fmtLocal.empty()) return;
			if (START == mode)
			{
				if (dist2aim <= policy_table_fmt[policy_num][0])
					speed_ahead = 0.0f;
				else
					speed_ahead = 8.0f - (float)prm_pl_data->robot_id;//逐车递减速度，拉开距离
				pl_prm_data->event = GOOD;
			}
			else
			{
				int area = 1;//进行上限匹配，确定当前间距的策略编号
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
				case 1:		//紧急停车区
					speed_ahead = 0.0f;
					pl_prm_data->event = GOOD;
					break;
				case 2:		//减速调整区
					speed_ahead = (prm_pl_data->speed * 36.0f / 1000.0f) * 0.8f; //按照前车速度的80%
					if (1.0f == fmtLocal.front().yaw) //当前点是弯道点，要做限速,并留下弯道标记
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					pl_prm_data->event = GOOD;
					break;
				case 3:		//速度保持区1
					speed_ahead = prm_pl_data->speed * 36.0f / 1000.0f - 2.0f;
					if (1.0f == fmtLocal.front().yaw) //当前点是弯道点，要做限速
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//不是弯道点要做速度保持
					{
						//前车已经停车，本车还是要保证距离
						SpeedKeep(speed_ahead, 7.0f);
					}
					pl_prm_data->event = GOOD;
					break;
				case 4:		//速度保持区2
					speed_ahead = prm_pl_data->speed * 36.0f / 1000.0f + 2.0f;
					if (1.0f == fmtLocal.front().yaw)//当前点是弯道点，要做限速
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//不是弯道点要做速度保持
					{
						//前车已经停车，本车还是要保证距离
						SpeedKeep(speed_ahead, 8.0f);
					}
					pl_prm_data->event = GOOD;
					break;
				case 5:			//加速调整区
					speed_ahead = (prm_pl_data->speed * 36.0f / 1000.0f)*1.2f;
					if (1.0f == fmtLocal.front().yaw)//当前点是弯道点，要做限速
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//不是弯道点要做速度保持
					{
						//前车已经停车，本车还是要保证距离
						SpeedKeep(speed_ahead, 10.0f);
					}
					pl_prm_data->event = GOOD;
					break;
				case 6:
					speed_ahead = (prm_pl_data->speed * 36.0f / 1000.0f)*1.5f;
					if (1.0f == fmtLocal.front().yaw)//当前点是弯道点，要做限速
					{
						SpeedLimit(speed_ahead, fmtLocal.front().speed);
						pl_prm_data->object[(unsigned int)prm_pl_data->robot_id].z = 1;
					}
					else//b不是弯道则快速追赶
					{
						//前车已经停车，本车还是要保证距离
						SpeedKeep(speed_ahead, 12.0f);
					}
					pl_prm_data->event = DROP;
				default:
					break;
				}
			}
			SpeedLimit(speed_ahead, speed_lim);
			SpeedLimit(speed_ahead, param.speed_max);
			//根据look_ahead，在look_ahead内寻找一个合适的目标点
			//Note: 2015-08-22 将固定距离瞄点策略改变成为550.0cm的10个点内寻找一个角度最小的点(石老师)
			//		解决的问题：这样算法不再严格跟踪点，但是可以应付路径出现横向漂移点。
			//		存在的问题：在拐弯处出现的漂移点无法处理，S弯跟点会出现无法严格跟踪轨迹
			float angle_less = 15.0f;//默认角度(用abs(x/y)表示)很大
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