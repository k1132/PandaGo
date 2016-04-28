#include "plsdk.h"

namespace plsdk
{
	Map & Map::GetInstance()
	{
		static Map inst;
		return inst;
	}

	Map::Map()
	{
		//初始化 大栅格地图（原始）
		map_64_raw = new float[MAP_64_CELL_NUM];
		memset(map_64_raw, 0, sizeof(float)*MAP_64_CELL_NUM);

		//初始化 大栅格地图（标准）
		map_64 = new float[MAP_64_CELL_NUM];
		memset(map_64, 0, sizeof(float)*MAP_64_CELL_NUM);

		//初始化 高清栅格地图（原始）
		map_hd_raw = new float[MAP_HD_CELL_NUM];
		memset(map_hd_raw, 0, sizeof(float)*MAP_HD_CELL_NUM);

		//初始化 高清栅格地图（标准）
		map_hd = new float[MAP_HD_CELL_NUM];
		memset(map_hd, 0, sizeof(float)*MAP_HD_CELL_NUM);

		//初始化 兴趣栅格地图（原始）
		map_roi_raw = new float[MAP_ROI_CELL_NUM];
		memset(map_roi_raw, 0, sizeof(float)*MAP_ROI_CELL_NUM);

		//初始化 兴趣栅格地图（标准）
		map_roi = new float[MAP_ROI_CELL_NUM];
		memset(map_roi, 0, sizeof(float)*MAP_ROI_CELL_NUM);

		raw = cv::Mat(MAP_ROI_CELL_VER, MAP_ROI_CELL_HOR, CV_32F, map_roi_raw, sizeof(float)*MAP_ROI_CELL_VER);
		roi = cv::Mat(MAP_ROI_CELL_VER, MAP_ROI_CELL_HOR, CV_32F, map_roi, sizeof(float)*MAP_ROI_CELL_VER);
		kl9 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
		kl5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

		n_l = new Dot[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
		memset(n_l, 0, sizeof(Dot)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		n_l_num = 0;
		n_r = new Dot[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
		memset(n_r, 0, sizeof(Dot)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		n_r_num = 0;

		road_lines = new ROAD_LINES;
		memset(road_lines, 0, sizeof(ROAD_LINES));
	}

	Map::~Map()
	{
		delete[] map_64_raw;
		delete[] map_64;
		delete[] map_hd_raw;
		delete[] map_hd;
		delete[] map_roi_raw;
		delete[] map_roi;
		delete[] n_l;
		delete[] n_r;
		delete road_lines;
	}

	void Map::updateMap(FU_PL_DATA * data_fupl, Axis * axisFu, Axis * axisPl)
	{
		//1 处理栅格地图相关数据
		GRID_MAP_64 * data_lidar64 = &data_fupl->gridmap;
		GRID_MAP_HD * data_lidarHD = &data_fupl->hdmap;
		//将栅格解码并转换成为规划用的浮点栅格
		unsigned char dd1 = 0x00;
		unsigned char dd2 = 0x00;
		const unsigned char * d_64_ptr = &data_lidar64->grid[0][0];
		for (unsigned int cnt = 0; cnt != W_GRID_MAP_64*H_GRID_MAP_64; ++cnt)
		{
			dd1 = (((*(d_64_ptr + cnt)) & 0xf0) >> 4);
			dd2 = (((*(d_64_ptr + cnt)) & 0x0f) >> 0);
			(dd1 == 0x00) ? (map_64_raw[2 * cnt] = 0.0f) : (map_64_raw[2 * cnt] = 255.0f);
			(dd2 == 0x00) ? (map_64_raw[2 * cnt + 1] = 0.0f) : (map_64_raw[2 * cnt + 1] = 255.0f);
		}
		const unsigned char * d_hd_ptr = &data_lidarHD->grid[0][0];
		for (unsigned int cnt = 0; cnt != W_GRID_MAP_HD*H_GRID_MAP_HD; ++cnt)
		{
			dd1 = (((*(d_hd_ptr + cnt)) & 0xf0) >> 4);
			dd2 = (((*(d_hd_ptr + cnt)) & 0x0f) >> 0);
			(dd1 == 0x00) ? (map_hd[2 * cnt] = 0.0f) : (map_hd[2 * cnt] = 255.0f);
			(dd2 == 0x00) ? (map_hd[2 * cnt + 1] = 0.0f) : (map_hd[2 * cnt + 1] = 255.0f);
		}

		//将地图中的有效栅格做一次坐标转换，转换到目前的坐标系下
		register const float c2e_cos(cos(axisFu->yaw));
		register const float c2e_sin(sin(axisFu->yaw));
		register const float e2c_cos(cos(axisPl->yaw));
		register const float e2c_sin(sin(axisPl->yaw));
		memset(map_64, 0, sizeof(float)*MAP_64_CELL_NUM);
		for (int cnt(0); cnt != MAP_64_CELL_NUM; ++cnt)
		{
			if (map_64_raw[cnt] == 0.0f) continue;//不处理无意义的值
			//源地图：源地图坐标转源车体坐标
			const float src_x = (cnt % MAP_64_CELL_HOR - MAP_64_CELL_HOR / 2)*MAP_64_CELL_SIZ;
			const float src_y = (cnt / MAP_64_CELL_HOR - MAP_64_CELL_HOR / 2)*MAP_64_CELL_SIZ;
			//源地图：车体坐标转大地坐标
			const float ert_x(src_x*c2e_cos - src_y*c2e_sin + axisFu->x);
			const float ert_y(src_x*c2e_sin + src_y*c2e_cos + axisFu->y);
			//目标地图：大地坐标转目标车体坐标
			const float car_x((ert_x - axisPl->x)*e2c_cos + (ert_y - axisPl->y)*e2c_sin);
			const float car_y(-(ert_x - axisPl->x)*e2c_sin + (ert_y - axisPl->y)*e2c_cos);
			//目标地图：目标车体坐标转目标地图坐标（地图越界检测）
			const int tar_x = static_cast<int>(MAP_64_CELL_HOR / 2 + car_x / MAP_64_CELL_SIZ);
			const int tar_y = static_cast<int>(MAP_64_CELL_HOR / 2 + car_y / MAP_64_CELL_SIZ);
			if (tar_x < 0 || tar_x >= MAP_64_CELL_HOR || tar_y < 0 || tar_y >= MAP_64_CELL_VER) continue;
			//目标地图：更新目标地图栅格
			map_64[tar_x + tar_y*MAP_64_CELL_HOR] = 255.0f;
		}

		/* 不对高清栅格做坐标对齐处理，因为往往在低速情况下使用高清栅格
		memset(map_hd, 0, sizeof(float)*MAP_HD_CELL_NUM);
		for (unsigned int cnt = 0; cnt != MAP_HD_CELL_NUM; ++cnt)
		{
		if (map_hd_raw[cnt] == 0.0f) continue;//不处理无意义的值
		//源地图：源地图坐标转源车体坐标
		const float src_x = (cnt % MAP_HD_CELL_HOR - MAP_HD_CELL_HOR / 2)*MAP_HD_CELL_SIZ;
		const float src_y = (cnt / MAP_HD_CELL_HOR)*MAP_HD_CELL_SIZ;
		//源地图：车体坐标转大地坐标
		const float ert_x(src_x*c2e_cos - src_y*c2e_sin + axisFu->x);
		const float ert_y(src_x*c2e_sin + src_y*c2e_cos + axisFu->y);
		//目标地图：大地坐标转目标车体坐标
		const float car_x((ert_x - axisPl->x)*e2c_cos + (ert_y - axisPl->y)*e2c_sin);
		const float car_y(-(ert_x - axisPl->x)*e2c_sin + (ert_y - axisPl->y)*e2c_cos);
		//目标地图：目标车体坐标转目标地图坐标（地图越界检测）
		const int tar_x = static_cast<int>(MAP_HD_CELL_HOR / 2 + car_x / MAP_HD_CELL_SIZ);
		const int tar_y = static_cast<int>(car_y / MAP_HD_CELL_SIZ);
		if (tar_x < 0 || tar_x >= MAP_HD_CELL_HOR || tar_y < 0 || tar_y >= MAP_HD_CELL_VER) continue;
		//目标地图：更新目标地图栅格
		map_hd[tar_x + tar_y*MAP_HD_CELL_HOR] = 255.0f;
		}
		*/

		//提取兴趣区域，将兴趣区域做闭运算，同时再做保守膨胀
		for (unsigned int cnt = 0; cnt != MAP_ROI_CELL_VER; ++cnt)
		{
			float * s_ptr = map_64 + (MAP_64_CELL_HOR / 2 + cnt)*MAP_64_CELL_HOR + (MAP_64_CELL_HOR - MAP_ROI_CELL_HOR) / 2;
			float * d_ptr = map_roi_raw + MAP_ROI_CELL_HOR*cnt;
			memcpy(d_ptr, s_ptr, sizeof(float)*MAP_ROI_CELL_HOR);
		}
		cv::dilate(raw, roi, kl9);//闭运算第一步，raw保存原始栅格，roi保存膨胀栅格
		cv::erode(roi, raw, kl9);//闭运算第二步，raw保存开运算结果，roi保存膨胀栅格

		//兴趣区域膨胀方案分为2级
		//先膨胀：半径为2个栅格
		memset(map_roi, 0, sizeof(float)*MAP_ROI_CELL_NUM);
		cv::dilate(raw, roi, kl5);//膨胀栅格
		//将兴趣区域数据重组到栅格地图中
		for (unsigned int cnt = 0; cnt != MAP_ROI_CELL_NUM; ++cnt)
			if (255.0f == map_roi[cnt] && 0.0f == map_roi_raw[cnt])
				map_roi_raw[cnt] = 128.0f;
		//再膨胀：半径为5个栅格
		memset(map_roi, 0, sizeof(float)*MAP_ROI_CELL_NUM);
		cv::dilate(raw, roi, kl9);//膨胀栅格
		//将兴趣区域数据重组到栅格地图中
		for (unsigned int cnt = 0; cnt != MAP_ROI_CELL_NUM; ++cnt)
			if (255.0f == map_roi[cnt] && 0.0f == map_roi_raw[cnt])
				map_roi_raw[cnt] = 64.0f;

		//将处理后的兴趣区域重新映射到大栅格图上
		for (unsigned int cnt = 0; cnt != MAP_ROI_CELL_VER; ++cnt)
		{
			float * d_ptr = map_64 + (MAP_64_CELL_HOR / 2 + cnt)*MAP_64_CELL_HOR + (MAP_64_CELL_HOR - MAP_ROI_CELL_HOR) / 2;
			float * s_ptr = map_roi_raw + MAP_ROI_CELL_HOR*cnt;
			memcpy(d_ptr, s_ptr, sizeof(float)*MAP_ROI_CELL_HOR);
		}
		memcpy(map_roi, map_roi_raw, sizeof(float)*MAP_ROI_CELL_NUM);

		//2 处理自然道边相关数据
		ROAD_NATURAL_BOUNDARY * boundary_natural = &data_fupl->boundary;
		n_l_num = 0;
		n_r_num = 0;
		for (unsigned int cnt = 0; cnt != NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; ++cnt)
		{
			if (boundary_natural->l_boundary[cnt].x != 0 && boundary_natural->l_boundary[cnt].y != 0)
				n_l[n_l_num++] = Dot{ (float)boundary_natural->l_boundary[cnt].x, (float)boundary_natural->l_boundary[cnt].y };
			if (boundary_natural->r_boundary[cnt].x != 0 && boundary_natural->r_boundary[cnt].y != 0)
				n_r[n_r_num++] = Dot{ (float)boundary_natural->r_boundary[cnt].x, (float)boundary_natural->r_boundary[cnt].y };
		}

		//3 处理车道线相关数据
		memcpy(road_lines, &data_fupl->lines, sizeof(ROAD_LINES));
		
		return;
	}

	bool Map::detectCollision(list<Target> & seq, float & dist_y, MAP_SENSITIVITY sensitivity)
	{
		//栅格化序列对应了小地图的位置
		list<Pos> seq_p;
		Rasterisation(seq, seq_p);
		dist_y = 0.0f;
		float thre = 0.0f;
		if (ORIGIN == sensitivity)
			thre = 255.0f;
		else if (NARROW == sensitivity)
			thre = 128.0f;
		else if (EXPAND == sensitivity)
			thre = 64.0f;
		for (const Pos & p : seq_p)
		{
			if (map_roi[p.y*MAP_ROI_CELL_HOR + p.x] >= thre)
			{
				dist_y = p.y*25.0f;//估计碰转的y坐标
				return true;
			}
		}
		return false;
	}

	bool Map::refinePath(list<Target> & seq, int maxVirtualLane)
	{
		//先为前方路线构造一个虚拟的车道(3.75m)
		//先将曲线降采样1/2

		return true;
	}
}