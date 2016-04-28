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
		//����Ĭ�ϳ�Ա
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
		//ά��gisRef, ���¾ֲ�gis·��
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
		//ά��gisLen, �������gis�ߵ��ܳ���
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
		//ά��isBak, gisCtrl, isGisCtrlValid, isBend ,maxStrΪ��ǰ·���ж���ʻ�ٶ�
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
		//ά��isOnTrack, �жϵ�ǰ�����Ƿ���gis��������·֮��
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
		//ά��isBlocked״̬�����ݶ�����������ٶ�
		Map & map = Map::GetInstance();
		float dist_collision = 0.0f;
		isBlocked = map.detectCollision(gisRef, dist_collision, EXPAND);
		if (true == isBlocked)
		{
			gisCtrl.speed = SpeedLimitDecisionByDis(dist_collision, gisCtrl.speed);//�����ϰ����Զ�������ٶȣ��ܹ����ϰ���ǰͣ��
		}
		//TODO ���������block״̬���������ȱ���·������
		//���raster
		Rasterisation(gisRef, gisRaster);
		return;
	}

	void Gis::doAnylsisGisV4()
	{
		// 0 һЩ��ʼ������
		Parameter & param = Parameter::GetInstance();
		gisCtrl = Target{ 0.0f, 0.0f, 0.0f, 0.0f };
		isGisCtrlValid = true;

		// 1 ���¿��Ƶ㣬ʹ�õ㴦�ڳ�ǰ��ǰ������y>300cm,���ǵ�xƫ�ƹ�����˵�����������������ɾ��
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

		// 2.1 ��������һ��δ����Ҫִ�еĵ���������������̫�٣����ٶȽ�Ϊ�������ģʽ
		if (gisLocal.size() < 6)
		{
			gisCtrl = gisLocal.back();
			gisCtrl.speed = 5.0f;
			return;
		}

		// 2.2 ��ִ�еĵ������Ƚϳ��㣬��ôΪÿһ������һ������
		//����Ϊÿһ������ȡһ������ֵ
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

		//����ƽ������(��ֵ�˲���һ����7��)
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

		//�ڰ˴��˲���������ֵ��������
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
		
		// 2.3 ����Ŀ�����ߵ�����ֵ���ҵ�һ�����ʵ�Ԥ���λ�ú��ٶ�
		maxStr = max_k * 1e5 / 6.0f;//������µ�˲ʱ�ֽǼ���,���þ���������Թ���
		float speed_ahead = SpeedLimitDecisionByStr(abs(maxStr));//����ǰ����������������趨�ٶ�����
		(speed_ahead <= 20.0f) ? (isBend = true) : (isBend = false);
		//�켣�ߵĵ�3���㿪ʼ���ڹ켣�������������8���㣬�ҵ�ƫ����С�ĵ㣬������õ�
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
		//�ٶ���Ҫ��������ٶ�Ҫ��
		SpeedLimit(speed_ahead, param.speed_gis_lim);
		gisCtrl.speed = speed_ahead;
		return;
	}

	void Gis::doHandleBackV1()
	{
		isGisCtrlValid = false;//Ĭ��û���ҵ�gis
		float speed_bak = 6.0f;//�ٶ�ǿ���趨Ϊ6.0
		float look_bak = 500.0f;//����趨Ϊ����500.0f
		//ѡ���˵�Ŀ���
		float angle_less = 1000.0f;//Ĭ�ϽǶ�(��abs(x/y)��ʾ)�ܴ�
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