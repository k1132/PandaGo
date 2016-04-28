#include "plsdk.h"

namespace plsdk
{
	Gis & Gis::GetInstance()
	{
		static Gis inst;
		return inst;
	}

	Gis::Gis()
	{
		//设置默认成员
		isBend = false;
		isOnTrack = false;
		isBlocked = false;
		isBak = false;
		gisLen = 0.0f;
		maxStr = 0.0f;
		gisRef.clear();
		gisCtrl = Target{ 0.0f, 0.0f, 0.0f, 0.0f };
		isGisCtrlValid = false;

		gisLocal.clear();
		gp = nullptr;
		axisGp = nullptr;
		axisPl = nullptr;
	}

	Gis::~Gis()
	{
	}

	void Gis::updateGis(GP_INFO *data, Axis *axisGeo, Axis *axisPl)
	{
		this->axisGp = axisGeo;
		this->axisPl = axisPl;
		this->gp = data;
		doMaintainGis();
		return;
	}

	void Gis::doMaintainGis()
	{
		//维护gisRef, 更新局部gis路径
		gisRef.clear();
		gisLocal.clear();
		gisRaster.clear();
		Target transf = Target{0.0f, 0.0f, 0.0f, 0.0f};
		for (int idx(0); idx < (int)gp->scene.gls.valid; ++idx)
		{
			transf.x = static_cast<float>(gp->scene.gls.gps[idx].x);
			transf.y = static_cast<float>(gp->scene.gls.gps[idx].y);
			Earth2Car(axisPl, &transf.x, &transf.y);
			if (transf.y >= 0)
			{
				gisLocal.push_back(transf);
				gisRef.push_back(transf);
			}
		}
		//维护gisLen, 计算给出gis线的总长度
		gisLen = 0.0f;
		if (gisRef.size() > 1)
		{
			auto j = gisRef.begin();
			auto i = j++;
			while (j != gisRef.end())
			{
				gisLen += sqrt(pow(j->x - i->x, 2) + pow(j->y - i->y, 2));
				i = j++;
			}
		}
		//维护isBak, gisCtrl, isGisCtrlValid, isBend ,maxStr为当前路径判断行驶速度
		if (0 == gp->scene.curve_level)
		{
			isBak = false;
			doAnylsisGisV4();
		}
		else if (1 == gp->scene.curve_level)
		{
			isBak = true;
			doHandleBackV1();
		}
		//维护isOnTrack, 判断当前车辆是否在gis给出的线路之上
		isOnTrack = false;
		for (const Target & t : gisRef)
		{
			float dis = sqrt(pow(t.x, 2) + pow(t.y, 2));
			if (dis < 180.0f)
			{
				isOnTrack = true;
				break;
			}
		}
		if (isOnTrack == false)
		{
			SpeedLimit(gisCtrl.speed, 12.0f);
		}
		//维护isBlocked状态，根据堵塞情况修正速度
		Map & map = Map::GetInstance();
		float dist_collision = 0.0f;
		isBlocked = map.detectCollision(gisRef, dist_collision, EXPAND);
		if (true == isBlocked)
		{
			gisCtrl.speed = SpeedLimitDecisionByDis(dist_collision, gisCtrl.speed);//根据障碍物的远近控制速度，能够在障碍物前停下
		}
		//TODO 如果发现是block状态，启动优先避障路径搜索
		//填充raster
		Rasterisation(gisRef, gisRaster);
		return;
	}

	void Gis::doAnylsisGisV4()
	{
		// 0 一些初始化工作
		Parameter & param = Parameter::GetInstance();
		gisCtrl = Target{ 0.0f, 0.0f, 0.0f, 0.0f };
		isGisCtrlValid = true;

		// 1 更新控制点，使得点处在车前轮前方，即y>300cm,但是当x偏移过大，则说明是其他情况，不予删除
		while (!gisLocal.empty())
		{
			if (gisLocal.front().y < 285.0f && abs(gisLocal.front().x) < 600.0f)
				gisLocal.pop_front();
			else
				break;
		}
		if (gisLocal.size() < 2)
		{
			isGisCtrlValid = false;
			return;
		}

		// 2.1 首先区别一下未来将要执行的点的数量，点的数量太少，将速度将为距离过近模式
		if (gisLocal.size() < 6)
		{
			gisCtrl = gisLocal.back();
			gisCtrl.speed = 5.0f;
			return;
		}

		// 2.2 若执行的点数量比较充足，那么为每一个点求一个曲率
		//首先为每一个点求取一个曲率值
		auto i_pt1 = gisLocal.begin();
		auto i_pt2 = i_pt1; ++i_pt2;
		auto i_pt3 = i_pt2; ++i_pt3;
		while (i_pt3 != gisLocal.end())
		{
			i_pt2->yaw = CurvityBy3Point(*i_pt1, *i_pt2, *i_pt3);
			++i_pt1;
			++i_pt2;
			++i_pt3;
		}

		//曲率平滑计算(均值滤波，一共做7次)
		for (int i = 0; i < 7; ++i)
		{
			i_pt1 = gisLocal.begin();
			i_pt2 = i_pt1; ++i_pt2;
			i_pt3 = i_pt2; ++i_pt3;
			while (i_pt3 != gisLocal.end())
			{
				i_pt2->yaw = (i_pt1->yaw + i_pt2->yaw + i_pt3->yaw) / 3.0f;
				++i_pt1;
				++i_pt2;
				++i_pt3;
			}
		}

		//第八次滤波，并保存值最大的曲率
		float max_k = 0.0f;
		i_pt1 = gisLocal.begin();
		i_pt2 = i_pt1; ++i_pt2;
		i_pt3 = i_pt2; ++i_pt3;
		while (i_pt3 != gisLocal.end())
		{
			i_pt2->yaw = (i_pt1->yaw + i_pt2->yaw + i_pt3->yaw) / 3.0f;
			if (abs(i_pt2->yaw) > abs(max_k)) max_k = i_pt2->yaw;
			++i_pt1;
			++i_pt2;
			++i_pt3;
		}
		
		// 2.3 根据目标曲线的曲率值，找到一个合适的预瞄点位置和速度
		maxStr = max_k * 1e5 / 6.0f;//计算大致的瞬时轮角极限,采用经验比例粗略估计
		float speed_ahead = SpeedLimitDecisionByStr(abs(maxStr));//根据前方曲线弯曲的情况设定速度上限
		(speed_ahead <= 20.0f) ? (isBend = true) : (isBend = false);
		//轨迹线的第3个点开始，在轨迹线上向后遍历最多8个点，找到偏角最小的点，并瞄向该点
		int counter = 0;
		float t = 1000.0f;
		for (const Target & p : gisLocal)
		{
			if (counter < 3)
			{
				counter++;
			}
			else if (counter >= 3 && counter < 12)
			{
				if (p.y != 0 && abs(p.x / p.y) < t)
				{
					t = abs(p.x / p.y);
					gisCtrl = p;
				}
				++counter;
			}
			else
			{
				break;
			}
		}
		//速度需要满足最高速度要求
		SpeedLimit(speed_ahead, param.speed_gis_lim);
		gisCtrl.speed = speed_ahead;
		return;
	}

	void Gis::doHandleBackV1()
	{
		isGisCtrlValid = false;//默认没能找到gis
		float speed_bak = 6.0f;//速度强制设定为6.0
		float look_bak = 500.0f;//向后看设定为至少500.0f
		//选择倒退的目标点
		float angle_less = 1000.0f;//默认角度(用abs(x/y)表示)很大
		int counter = 0;
		for (const Target & tar : gisLocal)
		{
			float point_dist = sqrt(pow(tar.y, 2) + pow(tar.x, 2));
			if (point_dist >= look_bak)
			{
				if (angle_less >= abs(tar.x / tar.y))
				{
					gisCtrl = tar;
					gisCtrl.speed = speed_bak;
					isGisCtrlValid = true;
					angle_less = abs(tar.x / tar.y);
				}
				++counter;
				if (counter > 5) break;
			}
		}
		return;
	}
}