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
		//��ʼ�� ��դ���ͼ��ԭʼ��
		map_64_raw = new float[MAP_64_CELL_NUM];
		memset(map_64_raw, 0, sizeof(float)*MAP_64_CELL_NUM);

		//��ʼ�� ��դ���ͼ����׼��
		map_64 = new float[MAP_64_CELL_NUM];
		memset(map_64, 0, sizeof(float)*MAP_64_CELL_NUM);

		//��ʼ�� ����դ���ͼ��ԭʼ��
		map_hd_raw = new float[MAP_HD_CELL_NUM];
		memset(map_hd_raw, 0, sizeof(float)*MAP_HD_CELL_NUM);

		//��ʼ�� ����դ���ͼ����׼��
		map_hd = new float[MAP_HD_CELL_NUM];
		memset(map_hd, 0, sizeof(float)*MAP_HD_CELL_NUM);

		//��ʼ�� ��Ȥդ���ͼ��ԭʼ��
		map_roi_raw = new float[MAP_ROI_CELL_NUM];
		memset(map_roi_raw, 0, sizeof(float)*MAP_ROI_CELL_NUM);

		//��ʼ�� ��Ȥդ���ͼ����׼��
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
		//1 ����դ���ͼ�������
		GRID_MAP_64 * data_lidar64 = &data_fupl->gridmap;
		GRID_MAP_HD * data_lidarHD = &data_fupl->hdmap;
		//��դ����벢ת����Ϊ�滮�õĸ���դ��
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

		//����ͼ�е���Чդ����һ������ת����ת����Ŀǰ������ϵ��
		register const float c2e_cos(cos(axisFu->yaw));
		register const float c2e_sin(sin(axisFu->yaw));
		register const float e2c_cos(cos(axisPl->yaw));
		register const float e2c_sin(sin(axisPl->yaw));
		memset(map_64, 0, sizeof(float)*MAP_64_CELL_NUM);
		for (int cnt(0); cnt != MAP_64_CELL_NUM; ++cnt)
		{
			if (map_64_raw[cnt] == 0.0f) continue;//�������������ֵ
			//Դ��ͼ��Դ��ͼ����תԴ��������
			const float src_x = (cnt % MAP_64_CELL_HOR - MAP_64_CELL_HOR / 2)*MAP_64_CELL_SIZ;
			const float src_y = (cnt / MAP_64_CELL_HOR - MAP_64_CELL_HOR / 2)*MAP_64_CELL_SIZ;
			//Դ��ͼ����������ת�������
			const float ert_x(src_x*c2e_cos - src_y*c2e_sin + axisFu->x);
			const float ert_y(src_x*c2e_sin + src_y*c2e_cos + axisFu->y);
			//Ŀ���ͼ���������תĿ�공������
			const float car_x((ert_x - axisPl->x)*e2c_cos + (ert_y - axisPl->y)*e2c_sin);
			const float car_y(-(ert_x - axisPl->x)*e2c_sin + (ert_y - axisPl->y)*e2c_cos);
			//Ŀ���ͼ��Ŀ�공������תĿ���ͼ���꣨��ͼԽ���⣩
			const int tar_x = static_cast<int>(MAP_64_CELL_HOR / 2 + car_x / MAP_64_CELL_SIZ);
			const int tar_y = static_cast<int>(MAP_64_CELL_HOR / 2 + car_y / MAP_64_CELL_SIZ);
			if (tar_x < 0 || tar_x >= MAP_64_CELL_HOR || tar_y < 0 || tar_y >= MAP_64_CELL_VER) continue;
			//Ŀ���ͼ������Ŀ���ͼդ��
			map_64[tar_x + tar_y*MAP_64_CELL_HOR] = 255.0f;
		}

		/* ���Ը���դ����������봦����Ϊ�����ڵ��������ʹ�ø���դ��
		memset(map_hd, 0, sizeof(float)*MAP_HD_CELL_NUM);
		for (unsigned int cnt = 0; cnt != MAP_HD_CELL_NUM; ++cnt)
		{
		if (map_hd_raw[cnt] == 0.0f) continue;//�������������ֵ
		//Դ��ͼ��Դ��ͼ����תԴ��������
		const float src_x = (cnt % MAP_HD_CELL_HOR - MAP_HD_CELL_HOR / 2)*MAP_HD_CELL_SIZ;
		const float src_y = (cnt / MAP_HD_CELL_HOR)*MAP_HD_CELL_SIZ;
		//Դ��ͼ����������ת�������
		const float ert_x(src_x*c2e_cos - src_y*c2e_sin + axisFu->x);
		const float ert_y(src_x*c2e_sin + src_y*c2e_cos + axisFu->y);
		//Ŀ���ͼ���������תĿ�공������
		const float car_x((ert_x - axisPl->x)*e2c_cos + (ert_y - axisPl->y)*e2c_sin);
		const float car_y(-(ert_x - axisPl->x)*e2c_sin + (ert_y - axisPl->y)*e2c_cos);
		//Ŀ���ͼ��Ŀ�공������תĿ���ͼ���꣨��ͼԽ���⣩
		const int tar_x = static_cast<int>(MAP_HD_CELL_HOR / 2 + car_x / MAP_HD_CELL_SIZ);
		const int tar_y = static_cast<int>(car_y / MAP_HD_CELL_SIZ);
		if (tar_x < 0 || tar_x >= MAP_HD_CELL_HOR || tar_y < 0 || tar_y >= MAP_HD_CELL_VER) continue;
		//Ŀ���ͼ������Ŀ���ͼդ��
		map_hd[tar_x + tar_y*MAP_HD_CELL_HOR] = 255.0f;
		}
		*/

		//��ȡ��Ȥ���򣬽���Ȥ�����������㣬ͬʱ������������
		for (unsigned int cnt = 0; cnt != MAP_ROI_CELL_VER; ++cnt)
		{
			float * s_ptr = map_64 + (MAP_64_CELL_HOR / 2 + cnt)*MAP_64_CELL_HOR + (MAP_64_CELL_HOR - MAP_ROI_CELL_HOR) / 2;
			float * d_ptr = map_roi_raw + MAP_ROI_CELL_HOR*cnt;
			memcpy(d_ptr, s_ptr, sizeof(float)*MAP_ROI_CELL_HOR);
		}
		cv::dilate(raw, roi, kl9);//�������һ����raw����ԭʼդ��roi��������դ��
		cv::erode(roi, raw, kl9);//������ڶ�����raw���濪��������roi��������դ��

		//��Ȥ�������ͷ�����Ϊ2��
		//�����ͣ��뾶Ϊ2��դ��
		memset(map_roi, 0, sizeof(float)*MAP_ROI_CELL_NUM);
		cv::dilate(raw, roi, kl5);//����դ��
		//����Ȥ�����������鵽դ���ͼ��
		for (unsigned int cnt = 0; cnt != MAP_ROI_CELL_NUM; ++cnt)
			if (255.0f == map_roi[cnt] && 0.0f == map_roi_raw[cnt])
				map_roi_raw[cnt] = 128.0f;
		//�����ͣ��뾶Ϊ5��դ��
		memset(map_roi, 0, sizeof(float)*MAP_ROI_CELL_NUM);
		cv::dilate(raw, roi, kl9);//����դ��
		//����Ȥ�����������鵽դ���ͼ��
		for (unsigned int cnt = 0; cnt != MAP_ROI_CELL_NUM; ++cnt)
			if (255.0f == map_roi[cnt] && 0.0f == map_roi_raw[cnt])
				map_roi_raw[cnt] = 64.0f;

		//����������Ȥ��������ӳ�䵽��դ��ͼ��
		for (unsigned int cnt = 0; cnt != MAP_ROI_CELL_VER; ++cnt)
		{
			float * d_ptr = map_64 + (MAP_64_CELL_HOR / 2 + cnt)*MAP_64_CELL_HOR + (MAP_64_CELL_HOR - MAP_ROI_CELL_HOR) / 2;
			float * s_ptr = map_roi_raw + MAP_ROI_CELL_HOR*cnt;
			memcpy(d_ptr, s_ptr, sizeof(float)*MAP_ROI_CELL_HOR);
		}
		memcpy(map_roi, map_roi_raw, sizeof(float)*MAP_ROI_CELL_NUM);

		//2 ������Ȼ�����������
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

		//3 ���������������
		memcpy(road_lines, &data_fupl->lines, sizeof(ROAD_LINES));
		
		return;
	}

	bool Map::detectCollision(list<Target> & seq, float & dist_y, MAP_SENSITIVITY sensitivity)
	{
		//դ�����ж�Ӧ��С��ͼ��λ��
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
				dist_y = p.y*25.0f;//������ת��y����
				return true;
			}
		}
		return false;
	}

	bool Map::refinePath(list<Target> & seq, int maxVirtualLane)
	{
		//��Ϊǰ��·�߹���һ������ĳ���(3.75m)
		//�Ƚ����߽�����1/2

		return true;
	}
}