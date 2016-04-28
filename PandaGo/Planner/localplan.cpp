#include "plsdk.h"
#include <app_pl.h>
#include "zgcc.h"

#include <vector>
#include <list>
#include <fstream>
#include <time.h>

using std::vector;
using std::list;

namespace UGV_PL
{
	using namespace plsdk;
	using namespace std;

	static Axis axisFu = { 0.0, 0.0, 0.0, 0.0 };
	static Axis axisGp = { 0.0, 0.0, 0.0, 0.0 };
	static Axis axisPl = { 0.0, 0.0, 0.0, 0.0 };
	static FRAME_ID frameid = 0;
	static int gis_fail_counter = 0;
	static int gis_blck_counter = 0;
	static int gis_okay_counter = 0;
	static bool use_gis = true;

	//״̬��
	static STATUS status;

	//�ںϴ���ĸ���״̬��
	static bool flag_fu_stop = false;
	static float sign_spd_limit = 100.0f;//��ͨ��־�Ƶ����٣�Ĭ������100km/h
	static float stat_spd_limit = 100.0f;//����״̬�µĵ����٣�Ĭ������100km/h

	//gp������״̬
	static bool flag_bad_road = false;

	static float car_spd;
	static int rt = 0;

	//��ת����
	static PL_CS_DATA zgcc_pcd;
	static PL_CS_DATA lhf_pcd;
	static PL_CS_DATA gis_pcd;
	static bool is_gis_plan_ok = false;

	void traceGisRefV3(PL_CS_DATA * pcd)
	{
		//��ù滮������󣬳�ʼ�����
		Gis & gis = Gis::GetInstance();
		Map & map = Map::GetInstance();
		pcd->sys_command = 0x00;
		pcd->isok = 0;
		pcd->speed = (INT16)0;//��geoͣ���ȴ�
		pcd->number_of_effective_points = 0;
		if (true == gis.isGisCtrlValid)
		{
			list<Target> traj;
			//�����ݴ���
			UniformInterpolation(Target{ 0.0f, 0.0f, 0.0f, 0.0f }, gis.gisCtrl, 80.0f, traj);//�Ե�ǰ�㵽Ŀ���֮����о��Ȳ�ֵ
			if (gis.gisCtrl.speed < 1.0f) gis.gisCtrl.speed = 0.0f;//�Ե��������ٶ����ضϣ���ֹ���������ƶ�
			//��дpl���͸�cs������
			if (true == gis.isBak) pcd->sys_command = 0x01;//�õ���λ
			pcd->isok = 1;//����isOK

			//���ٺͼ�ͣ��ز���
			float sn = gis.gisCtrl.speed;
			SpeedLimit(sn, speedLimitDecisionByStu(status));//����ϵͳ״̬��������
			SpeedLimit(sn, SpeedLimitDecisionByStr(axisPl.str));//���ݵ�ǰ�ֽ����٣���ֹ����ת��
			SpeedLimit(sn, sign_spd_limit);//���ݽ�ͨ��־��������
			if (true == flag_fu_stop) sn = 0.0f;//�ںϴ���ļ�ͣ�������٣�ֱ��ͣ��
			if (0.0f == sn) pcd->sys_command = 0x02;//�ý���ɲ��

			//�жϹ滮���Ƿ��Ǳ�����,�����������is_gis_plan_ok��
			float col_y = 0.0f;
			is_gis_plan_ok = !map.detectCollision(traj, col_y, EXPAND);
			if (is_gis_plan_ok == false) sn = 0.0f;

			//��д�ٶȺ�·����
			pcd->speed = (INT16)(sn * 1000.0f / 36.0f);
			for (const Target & t : traj)
			{
				pcd->path[pcd->number_of_effective_points].x = (INT32)t.x;
				pcd->path[pcd->number_of_effective_points].y = (INT32)t.y;
				++pcd->number_of_effective_points;
				if (pcd->number_of_effective_points >= 20) break;
			}
		}
		return;
	}

	void traceGisLine(PL_CS_DATA * pcd)
	{
		//��ù滮������󣬳�ʼ�����
		Gis & gis = Gis::GetInstance();
		Map & map = Map::GetInstance();
		pcd->sys_command = 0x10;
		pcd->isok = 1;
		pcd->speed = (INT16)0;//��geoͣ���ȴ�
		pcd->number_of_effective_points = 0;

		float sn = gis.gisCtrl.speed;
		SpeedLimit(sn, speedLimitDecisionByStu(status));//����ϵͳ״̬��������
		SpeedLimit(sn, SpeedLimitDecisionByStr(axisPl.str));//���ݵ�ǰ�ֽ����٣���ֹ����ת��
		SpeedLimit(sn, sign_spd_limit);//���ݽ�ͨ��־��������
		float speed_bad_road = 8.0f;
		SpeedLimit(sn, speed_bad_road);
		pcd->speed = (INT16)(sn * 1000.0f / 36.0f);

		//�жϹ滮���Ƿ��Ǳ�����,�����������is_gis_plan_ok��
		float col_y = 0.0f;
		if (flag_bad_road == true)
		{
			list<Target> test = gis.gisRef;
			while (test.size() > 10)
				test.pop_back();
			is_gis_plan_ok = !map.detectCollision(test, col_y, NARROW);
			if (is_gis_plan_ok == false)
				pcd->speed = 0;
			else
				pcd->speed = 277.0f;
		}
		else
		{
			is_gis_plan_ok = is_gis_plan_ok && !map.detectCollision(gis.gisRef, col_y, NARROW);
		}
		
		if (is_gis_plan_ok == false) sn = 0.0f;

		//ֱ�����gis��·������ȡ����ǰ12m��
		for (const Target & t : gis.gisRef)
		{
			if (t.y > 1000) break;//ֵ�����Գ���10m
			pcd->path[pcd->number_of_effective_points].x = (INT32)t.x;
			pcd->path[pcd->number_of_effective_points].y = (INT32)t.y;
			++pcd->number_of_effective_points;
			if (pcd->number_of_effective_points > 10) break;//��಻�����������10����
		}
		return;
	}

	void traceFmtRef(PL_CS_DATA * pcd)
	{
		//��ù滮��������
		plsdk::Fmt & fmt(plsdk::Fmt::GetInstance());
		pcd->sys_command = 0x00;
		pcd->isok = 0;
		pcd->speed = (INT16)0;//��geoͣ���ȴ�
		pcd->number_of_effective_points = 0;

		//������õ���ֱ��Ѱ����㷨
		if (true == fmt.isFmtCtrlValid)
		{//��geo���龰, ����ֱ�߲�ֵ
			MBUG("\tIn traceFmtRef: get a valid ref point [%f, %f].\n", fmt.fmtCtrl.x, fmt.fmtCtrl.y);
			//����һ��ָ��Ŀ���Ĺ滮�ߣ��滮��
			list<Target> traj;
			UniformInterpolation(Target{ 0.0f, 0.0f, 0.0f, 0.0f }, fmt.fmtCtrl, 80.0f, traj);
			if (fmt.fmtCtrl.speed < 1.5f) fmt.fmtCtrl.speed = 0.0f;//�Ե��������ٶ����ضϣ���ֹ���������ƶ�
			//��дpl���͸�cs������
			pcd->isok = 1;
			pcd->speed = (INT16)(fmt.fmtCtrl.speed * 1000.0f / 36.0f);
			if (fmt.fmtCtrl.speed == 0.0f) pcd->sys_command = 0x02;//�ý���ɲ��
			for (const Target & t : traj)
			{
				pcd->path[pcd->number_of_effective_points].x = (INT32)t.x;
				pcd->path[pcd->number_of_effective_points].y = (INT32)t.y;
				++pcd->number_of_effective_points;
				if (pcd->number_of_effective_points >= 20) break;
			}
		}
		return;
	}

	int Process_Plan(PL_FUNC_INPUT * pfi, PL_CS_DATA * pcd, PL_PRM_DATA * ppd, PL_LOCAL_DATA * pld)
	{
		clock_t prc_tik = clock();
		++frameid;

		MBUG("------------------------------------FRAME %d------------------------------------\n", frameid);

		//1 ���л�����ʼ��
		Parameter & param = Parameter::GetInstance();
		Map & map = Map::GetInstance();
		Gis & gis = Gis::GetInstance();
		Fmt & fmt = Fmt::GetInstance();

		//1.1 ��������ϵ��Ϣ��������Ϣ
		axisFu.x = static_cast<float>(pfi->fu_pl_data.state.pos.ins_coord.x);
		axisFu.y = static_cast<float>(pfi->fu_pl_data.state.pos.ins_coord.y);
		axisFu.str = static_cast<float>(pfi->cs_pl_data.real_angle*1e-2);
		axisFu.yaw = static_cast<float>(pfi->fu_pl_data.state.pos.yaw*1e-8);

		axisGp.x = static_cast<float>(pfi->gp_info.state.pos.ins_coord.x);
		axisGp.y = static_cast<float>(pfi->gp_info.state.pos.ins_coord.y);
		axisGp.str = static_cast<float>(pfi->cs_pl_data.real_angle*1e-2);
		axisGp.yaw = static_cast<float>(pfi->gp_info.state.pos.yaw*1e-8);

		axisPl.x = static_cast<float>(pfi->state.pos.ins_coord.x);
		axisPl.y = static_cast<float>(pfi->state.pos.ins_coord.y);
		axisPl.str = static_cast<float>(pfi->cs_pl_data.real_angle*1e-2);
		axisPl.yaw = static_cast<float>(pfi->state.pos.yaw*1e-8);

		//car_spd = (float)pfi->state.pos.spd;
		car_spd = Odometry2Speed(&pfi->odo_info);
		status = (STATUS)pfi->state.status;
		status = getStatus(&status);

		//1.2 ����map����
		clock_t map_tik = clock();
		map.updateMap(&pfi->fu_pl_data, &axisFu, &axisPl);
		MBUG("Time Map__: %ums.\n", clock() - map_tik);//����������ܼ���
		/* ����geo����,ע�⣬geoҪ��fmt�ȸ��£������� */
		clock_t gis_tik = clock();
		gis.updateGis(&pfi->gp_info, &axisGp, &axisPl);
		MBUG("Time Gis__: %ums.\n", clock() - gis_tik);//����������ܼ���
		/* ����fmt���� */
		clock_t fmt_tik = clock();
		fmt.updateFmt(&pfi->prm_pl_data, ppd, &axisPl, &axisPl);
		MBUG("Time Fmt__: %ums.\n", clock() - fmt_tik);//����������ܼ���

		//Ϊ�����״̬��Ϣ
		pcd->id = frameid;
		memcpy(&pcd->state, &pfi->state, sizeof(STATE));

		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//                               ������й滮ģ��ĵ���
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		//1 ά��zgcc�滮�������ݺ�״̬
		if (1 == frameid) plan_init();

		int ret = 1;
		if (pfi->gp_info.scene.road_level == R_LEVEL1 && rt != 0)
		{
			//���У��߽ṹ����·,�������ר��
			rt = 0;
			set_road_type(rt);
		}
		if (pfi->gp_info.scene.road_level == R_LEVEL5 && rt != 0)
		{//���У��߽ṹ����·
			rt = 0;
			set_road_type(rt);
		}
		else if (pfi->gp_info.scene.road_level == R_LEVEL4 && rt != 1)
		{//����ԽҰ���߰�ṹ����·
			rt = 1;
			set_road_type(rt);
		}
		else if (pfi->gp_info.scene.road_level == R_LEVEL3 && rt != 1)
		{//�ݵأ��߰�ṹ����·
			rt = 1;
			set_road_type(rt);
		}

		//2 �����־�ж�,Ϊϵͳ�ø��ֱ�־
		//2.1 �ж���stop_lines��صı�־��Ϣ
		STOP_LINE & sl = pfi->fu_pl_data.stop_lines.stop_lines[0];
		// >>>��ͣ��־���ж�
		flag_fu_stop = (sl.start.x == 0) && (sl.start.y == 500) && (sl.end.x == 0) && (sl.end.y == 500) && (pfi->fu_pl_data.stop_lines.valid_line_num == 1);
		if(flag_fu_stop) MBUG("FU said: Emergency Stop!!! [0, 500, 0, 500]\n");
		// >>>�ں������Ƶ��ж�
		if (pfi->fu_pl_data.stop_lines.valid_line_num == 1)
		{
			int & a = sl.start.x, & b = sl.start.y, & c = sl.end.x, & d = sl.end.y;
			if ((status == S_CROSS_UND || status == S_ROAD_NAV) && (a > 0) && (a == b) && (b == c) && (c == d))
			{
				sign_spd_limit = a;//���������Ϣ
				MBUG("FU said: Speed Limit!!!\n");
			}
			if ((a < 0) && (a == b) && (b == c) && (c == d)) sign_spd_limit = 100.0f;//�������
		}
		//2.3 �жϽ���ϲ�·��
		if (pfi->gp_info.tps[0].direction == 9)
		{
			//��direction��9Ϊ���
			MBUG("GP said: Bad Road Now!!!\n");
			flag_bad_road = true;
		}
		else
		{
			flag_bad_road = false;
		}
		//2.2 ϵͳ״̬������ж�
		stat_spd_limit = speedLimitDecisionByStu(status);

		if (0 == param.planner) //mix zgcc & lhf
		{
			is_gis_plan_ok = true;
			memset(&zgcc_pcd, 0, sizeof(PL_CS_DATA));
			memset(&lhf_pcd, 0, sizeof(PL_CS_DATA));
			memset(&gis_pcd, 0, sizeof(PL_CS_DATA));
			memcpy(&zgcc_pcd.state, &pfi->state, sizeof(STATE));
			memcpy(&lhf_pcd.state, &pfi->state, sizeof(STATE));
			memcpy(&gis_pcd.state, &pfi->state, sizeof(STATE));
			pl_road_trace_interface(frameid, pfi, &zgcc_pcd, pld);
			traceGisRefV3(&lhf_pcd);//�˺�����Ϊis_gis_plan_ok��ֵ
			traceGisLine(&gis_pcd);//�˺�����Ϊis_gis_plan_ok��ֵ
			//Ϊ���߼�������������10֡�ǿ�����gis�ģ���ô�л���gis�滮
			if (true == use_gis)
			{
				//���滮����Ч������
				if (true == flag_bad_road)
				{
					//����bad_road��
					(false == is_gis_plan_ok) ? (gis_blck_counter++) : (gis_blck_counter = 0);
					(false == gis.isGisCtrlValid) ? (gis_fail_counter++) : (gis_fail_counter = 0);
					if (gis_blck_counter >= 1 || gis_fail_counter >= 1)
					{
						gis_blck_counter = 0;
						gis_fail_counter = 0;
						use_gis = false;
					}
					memcpy(pcd, &gis_pcd, sizeof(PL_CS_DATA));
				}
				else
				{
					(true == gis.isBlocked || false == is_gis_plan_ok) ? (gis_blck_counter++) : (gis_blck_counter = 0);
					(false == gis.isGisCtrlValid) ? (gis_fail_counter++) : (gis_fail_counter = 0);
					if (gis_blck_counter >= 1 || gis_fail_counter >= 1)
					{
						gis_blck_counter = 0;
						gis_fail_counter = 0;
						use_gis = false;
					}
					memcpy(pcd, &lhf_pcd, sizeof(PL_CS_DATA));
					if (false == gis.isGisCtrlValid) pcd->speed = zgcc_pcd.speed;
				}

			}
			if (false == use_gis)
			{
				if ((gis.isBlocked == false) && (gis.isGisCtrlValid == true))
					gis_okay_counter++;
				else
					gis_okay_counter = 0;
				if (gis_okay_counter >= 1)
				{
					gis_okay_counter = 0;
					use_gis = true;
				}
				memcpy(pcd, &zgcc_pcd, sizeof(PL_CS_DATA));
				set_db_flag(pcd);
			}
			if (get_zgcc_plan_status() == ZG_OVERTAKE)
			{
				memcpy(pcd, &zgcc_pcd, sizeof(PL_CS_DATA));
				set_db_flag(pcd);
			}
		}
		
		else if (1 == param.planner) //only zgcc planner
		{
			ret = pl_road_trace_interface(frameid, pfi, pcd, pld);
			set_db_flag(pcd);
		}
		else if (2 == param.planner) //only lhf planner
		{
			reset_for_wait();
			if (param.mode == 1)// point from gis
				traceGisRefV3(pcd);
			else if (param.mode == 2)//point from fmt
				traceFmtRef(pcd);
		}
		

		MBUG("Time Tol__: %.4fms.\n", ((double)(clock() - prc_tik)) / CLOCKS_PER_SEC);

		MBUG("-------------------------------------------------------------------------------\n\n");
		return ret;
	}
}