#ifndef PLSDK_H
#define PLSDK_H

#include <app_pl.h>
#include <protocol_status.h>
#include <vector>
#include <list>
#include <string.h>
#include <string>
#include <map>
#include <cmath>
#include "plsdk_def.h"
#include <opencv2/imgproc/imgproc.hpp>

namespace plsdk
{
	using namespace std;

	extern float Norm2(float * x_1, float * y_1, float * x_2, float * y_2);
	extern void Blh2Xyz(float lat, float lon, float alt, float * x, float * y, float * z);
	extern void Car2Earth(const Axis * car_axis, float * x, float * y);
	extern void Earth2Car(const Axis * car_axis, float * x, float * y);
	extern void Transform(const Axis * src_axis, const Axis * dest_axis, float * x, float * y);
	extern bool IsProjInBound(const Dot * A, const Dot * B, const Dot * T);
	extern float Dist2Line(const Dot * A, const Dot * B, const Dot * T, float * dist);
	extern float VecAgnleCos(const Dot * A, const Dot * B, const Dot *O = nullptr);
	extern float Dot2Yaw(const Dot * A, const Dot * O = nullptr);
	extern float DeltaYaw(const float yaw1, const float yaw2);
	extern void LinearInterpolation(const Target start, const Target end, const float dist, list<Target> & list);
	extern void UniformInterpolation(const Target start, const Target end, const float step, list<Target> & list);
	extern void Rasterisation(list<Target> targets, list<Pos> & gi);
	extern float CurvityBy3Point(const Target a, const Target b, const Target c);
	extern bool IsLocateInSector(const Dot * A, const float R_min, const float R_max, const float left_angle, const float right_angle);
	extern void EnumMorphinPos(Morph * morph, const int max_pos, const float car_len);
	extern void nextPt(Axis * pt, const float alpha, const float L = 285.0f, const float delta = 200.0f);
	
	extern float SpeedLimitDecisionByStr(float str);
	extern float SpeedLimitDecisionByTar(const Target tar, const float cur_str, float & speed);
	extern float SpeedLimitDecisionByDis(const float dist, float speed);
	extern float speedLimitDecisionByStu(const STATUS stat);
	extern float SpeedLimit(float & speed_in, const float speed_top);
	extern float SpeedKeep(float & speed_in, const float speed_floor);

	extern float CollisionAovidDist(float speed);
	extern float CalcuExpectStr(float tar_x, float tar_y);
	extern float Odometry2Speed(const ODO_INFO * odo, float * speed = nullptr);

	class Map
	{
	public:
		float * map_64_raw;
		float * map_64;
		float * map_hd_raw;
		float * map_hd;
		float * map_roi_raw;
		float * map_roi;

		Dot * n_l;//自然道边数据：左侧
		int n_l_num;
		Dot * n_r;//自然道边数据：右侧
		int n_r_num;
		ROAD_LINES * road_lines;

		void updateMap(FU_PL_DATA * data_fupl, Axis * axisFu, Axis * axisPl);
		bool detectCollision(list<Target> & seq, float & dist, MAP_SENSITIVITY sensitivity);
		bool refinePath(list<Target> & seq, int maxVirtulLane);

	private:
		Axis * axisFu;
		Axis * axisPl;
		cv::Mat raw;
		cv::Mat roi;
		cv::Mat kl9;
		cv::Mat kl5;

		//下面的代码用来实现单例，如无必要不要修改
	private:
		Map();
		Map(const Map &);
		Map & operator = (const Map &);
		~Map();
	public:
		static Map & GetInstance();
	};

	class Gis
	{/* Singleton compoment */
	public:
		bool isBend;//标志当前是否是弯道，只有限速到20一下的才会被判定为弯道
		bool isOnTrack;//标志当前是否在固定的航道上,横向偏差值为180cm以上的判定为不在航道上
		bool isBlocked;//标记当前默认gis引导是否被堵塞
		bool isBak;//标志当前是否是在分支路段,做倒车
		float maxStr;//模拟最大转弯半径对应的轮角值
		float gisLen;//前方gis线的最大长度

		list<Target> gisRef;//gis的参考路径
		list<Pos> gisRaster;
		Target gisCtrl;//gis参考路径的控制点
		bool isGisCtrlValid;//gis参考路径的控制点是否有效
		
		void updateGis(GP_INFO * data, Axis * axisGeo, Axis * axisPl);

	private:
		GP_INFO * gp;//输入数据指针
		Axis * axisGp;
		Axis * axisPl;

		list<Target> gisLocal;//从Geo发送过来的路线

		void doMaintainGis();
		void doAnylsisGisV4();//分析当前的gis情况
		void doHandleBackV1();//寻找倒车点算法
		
	//下面的代码用来实现单例，如无必要不要修改
	private:
		Gis();
		Gis(const Gis &);
		Gis & operator = (const Gis &);
		~Gis();
	public:
		static Gis& GetInstance();
	};

	class Fmt
	{/* Singleton compoment */
	public:
		bool isBlocked;

		Target fmtCtrl;
		bool isFmtCtrlValid;
		list<Target> fmtLocal;

		void updateFmt(PRM_PL_DATA * data1, PL_PRM_DATA * data2, Axis * axisCord, Axis * axisPl);

	private:
		PRM_PL_DATA * prm_pl_data;
		PL_PRM_DATA * pl_prm_data;
		Axis * axisFmt;
		Axis * axisPl;

		list<Target> fmtPath;
		float dist2aim;
		float dist_ctrl;
		float speed_lim;
		int policy_num;

		FMT_ROLE role;
		FMT_EVENT event;
		FMT_TASK task;
		FMT_MODE mode;
		FMT_SIGNAL signal;

		void doMaintainFmt();
		void doExtractRefPointV4();

		//下面的代码用来实现单例，如无必要不要修改
	private:
		Fmt();
		Fmt(const Fmt &);
		Fmt & operator = (const Fmt &);
		~Fmt();
	public:
		static Fmt & GetInstance();
	};

	class Morphin
	{/* Singleton compoment */
	public:
		int real_angle_idx;
		int plan_angle_idx;

		int best_morph;
		
		Morph * morphin[MORPHIN_TEST_NUM];
		float angle_list[MORPHIN_TEST_NUM];
		float len_list[MORPHIN_TEST_NUM];
		float value_list[MORPHIN_TEST_NUM];

		void updateMorphin(CS_PL_DATA * cpd);
		int selectMorphinLineV2(const float * map, const Target tar);

		//下面的代码用来实现单例，如无必要不要修改
	private:
		Morphin();
		Morphin(const Morphin &);
		Morphin & operator = (const Morphin &);
		~Morphin();
	public:
		static Morphin & GetInstance();
	};

	class Parameter
	{
	public:
		//dynamic parameters
		float speed_max;
		float speed_gis_lim;
		float speed_fmt_lim;
		int planner;
		int mode;

	private:
		Parameter();
		Parameter(const Parameter &);
		Parameter & operator = (const Parameter &);
		~Parameter();
	public:
		static Parameter & GetInstance();
	};
}

#endif