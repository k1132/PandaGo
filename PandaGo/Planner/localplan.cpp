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

	//状态机
	static STATUS status;

	//融合传输的各种状态量
	static bool flag_fu_stop = false;
	static float sign_spd_limit = 100.0f;//交通标志牌的限速，默认限速100km/h
	static float stat_spd_limit = 100.0f;//跟中状态下的的限速，默认限速100km/h

	//gp传出的状态
	static bool flag_bad_road = false;

	static float car_spd;
	static int rt = 0;

	//中转数据
	static PL_CS_DATA zgcc_pcd;
	static PL_CS_DATA lhf_pcd;
	static PL_CS_DATA gis_pcd;
	static bool is_gis_plan_ok = false;

	void traceGisRefV3(PL_CS_DATA * pcd)
	{
		//获得规划服务对象，初始化输出
		Gis & gis = Gis::GetInstance();
		Map & map = Map::GetInstance();
		pcd->sys_command = 0x00;
		pcd->isok = 0;
		pcd->speed = (INT16)0;//无geo停车等待
		pcd->number_of_effective_points = 0;
		if (true == gis.isGisCtrlValid)
		{
			list<Target> traj;
			//做数据处理
			UniformInterpolation(Target{ 0.0f, 0.0f, 0.0f, 0.0f }, gis.gisCtrl, 80.0f, traj);//对当前点到目标点之间进行均匀插值
			if (gis.gisCtrl.speed < 1.0f) gis.gisCtrl.speed = 0.0f;//对低速跳动速度做截断，防止车辆意外移动
			//填写pl发送给cs的数据
			if (true == gis.isBak) pcd->sys_command = 0x01;//置倒车位
			pcd->isok = 1;//设置isOK

			//限速和急停相关操作
			float sn = gis.gisCtrl.speed;
			SpeedLimit(sn, speedLimitDecisionByStu(status));//根据系统状态进行限速
			SpeedLimit(sn, SpeedLimitDecisionByStr(axisPl.str));//根据当前轮角限速，防止过速转向
			SpeedLimit(sn, sign_spd_limit);//根据交通标志进行限速
			if (true == flag_fu_stop) sn = 0.0f;//融合传输的急停进行限速，直接停车
			if (0.0f == sn) pcd->sys_command = 0x02;//置紧急刹车

			//判断规划线是否是避碰的,将结果保存在is_gis_plan_ok中
			float col_y = 0.0f;
			is_gis_plan_ok = !map.detectCollision(traj, col_y, EXPAND);
			if (is_gis_plan_ok == false) sn = 0.0f;

			//填写速度和路径点
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
		//获得规划服务对象，初始化输出
		Gis & gis = Gis::GetInstance();
		Map & map = Map::GetInstance();
		pcd->sys_command = 0x10;
		pcd->isok = 1;
		pcd->speed = (INT16)0;//无geo停车等待
		pcd->number_of_effective_points = 0;

		float sn = gis.gisCtrl.speed;
		SpeedLimit(sn, speedLimitDecisionByStu(status));//根据系统状态进行限速
		SpeedLimit(sn, SpeedLimitDecisionByStr(axisPl.str));//根据当前轮角限速，防止过速转向
		SpeedLimit(sn, sign_spd_limit);//根据交通标志进行限速
		float speed_bad_road = 8.0f;
		SpeedLimit(sn, speed_bad_road);
		pcd->speed = (INT16)(sn * 1000.0f / 36.0f);

		//判断规划线是否是避碰的,将结果保存在is_gis_plan_ok中
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

		//直接输出gis线路，并截取到车前12m处
		for (const Target & t : gis.gisRef)
		{
			if (t.y > 1000) break;//值不可以超过10m
			pcd->path[pcd->number_of_effective_points].x = (INT32)t.x;
			pcd->path[pcd->number_of_effective_points].y = (INT32)t.y;
			++pcd->number_of_effective_points;
			if (pcd->number_of_effective_points > 10) break;//最多不可以输出超过10个点
		}
		return;
	}

	void traceFmtRef(PL_CS_DATA * pcd)
	{
		//获得规划基础服务
		plsdk::Fmt & fmt(plsdk::Fmt::GetInstance());
		pcd->sys_command = 0x00;
		pcd->isok = 0;
		pcd->speed = (INT16)0;//无geo停车等待
		pcd->number_of_effective_points = 0;

		//下面采用的是直接寻点的算法
		if (true == fmt.isFmtCtrlValid)
		{//有geo的情景, 进行直线插值
			MBUG("\tIn traceFmtRef: get a valid ref point [%f, %f].\n", fmt.fmtCtrl.x, fmt.fmtCtrl.y);
			//生成一条指向目标点的规划线，规划线
			list<Target> traj;
			UniformInterpolation(Target{ 0.0f, 0.0f, 0.0f, 0.0f }, fmt.fmtCtrl, 80.0f, traj);
			if (fmt.fmtCtrl.speed < 1.5f) fmt.fmtCtrl.speed = 0.0f;//对低速跳动速度做截断，防止车辆意外移动
			//填写pl发送给cs的数据
			pcd->isok = 1;
			pcd->speed = (INT16)(fmt.fmtCtrl.speed * 1000.0f / 36.0f);
			if (fmt.fmtCtrl.speed == 0.0f) pcd->sys_command = 0x02;//置紧急刹车
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

		//1 运行环境初始化
		Parameter & param = Parameter::GetInstance();
		Map & map = Map::GetInstance();
		Gis & gis = Gis::GetInstance();
		Fmt & fmt = Fmt::GetInstance();

		//1.1 更新坐标系信息、车速信息
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

		//1.2 更新map数据
		clock_t map_tik = clock();
		map.updateMap(&pfi->fu_pl_data, &axisFu, &axisPl);
		MBUG("Time Map__: %ums.\n", clock() - map_tik);//处理代码性能计数
		/* 更新geo数据,注意，geo要比fmt先更新！！！！ */
		clock_t gis_tik = clock();
		gis.updateGis(&pfi->gp_info, &axisGp, &axisPl);
		MBUG("Time Gis__: %ums.\n", clock() - gis_tik);//处理代码性能计数
		/* 更新fmt数据 */
		clock_t fmt_tik = clock();
		fmt.updateFmt(&pfi->prm_pl_data, ppd, &axisPl, &axisPl);
		MBUG("Time Fmt__: %ums.\n", clock() - fmt_tik);//处理代码性能计数

		//为输出置状态信息
		pcd->id = frameid;
		memcpy(&pcd->state, &pfi->state, sizeof(STATE));

		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//                               下面进行规划模块的调用
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		//1 维护zgcc规划器的数据和状态
		if (1 == frameid) plan_init();

		int ret = 1;
		if (pfi->gp_info.scene.road_level == R_LEVEL1 && rt != 0)
		{
			//城市，走结构化道路,常熟比赛专用
			rt = 0;
			set_road_type(rt);
		}
		if (pfi->gp_info.scene.road_level == R_LEVEL5 && rt != 0)
		{//城市，走结构化道路
			rt = 0;
			set_road_type(rt);
		}
		else if (pfi->gp_info.scene.road_level == R_LEVEL4 && rt != 1)
		{//乡村和越野，走半结构化道路
			rt = 1;
			set_road_type(rt);
		}
		else if (pfi->gp_info.scene.road_level == R_LEVEL3 && rt != 1)
		{//草地，走半结构化道路
			rt = 1;
			set_road_type(rt);
		}

		//2 输入标志判断,为系统置各种标志
		//2.1 判断与stop_lines相关的标志信息
		STOP_LINE & sl = pfi->fu_pl_data.stop_lines.stop_lines[0];
		// >>>急停标志的判断
		flag_fu_stop = (sl.start.x == 0) && (sl.start.y == 500) && (sl.end.x == 0) && (sl.end.y == 500) && (pfi->fu_pl_data.stop_lines.valid_line_num == 1);
		if(flag_fu_stop) MBUG("FU said: Emergency Stop!!! [0, 500, 0, 500]\n");
		// >>>融合限速牌的判断
		if (pfi->fu_pl_data.stop_lines.valid_line_num == 1)
		{
			int & a = sl.start.x, & b = sl.start.y, & c = sl.end.x, & d = sl.end.y;
			if ((status == S_CROSS_UND || status == S_ROAD_NAV) && (a > 0) && (a == b) && (b == c) && (c == d))
			{
				sign_spd_limit = a;//添加限速信息
				MBUG("FU said: Speed Limit!!!\n");
			}
			if ((a < 0) && (a == b) && (b == c) && (c == d)) sign_spd_limit = 100.0f;//解除限速
		}
		//2.3 判断进入较差路段
		if (pfi->gp_info.tps[0].direction == 9)
		{
			//以direction是9为标记
			MBUG("GP said: Bad Road Now!!!\n");
			flag_bad_road = true;
		}
		else
		{
			flag_bad_road = false;
		}
		//2.2 系统状态方面的判断
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
			traceGisRefV3(&lhf_pcd);//此函数会为is_gis_plan_ok置值
			traceGisLine(&gis_pcd);//此函数会为is_gis_plan_ok置值
			//为回线计数，连续超过10帧是可以用gis的，那么切回用gis规划
			if (true == use_gis)
			{
				//做规划器有效性评定
				if (true == flag_bad_road)
				{
					//开在bad_road上
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