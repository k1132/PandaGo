#include "plsdk.h"

#include <cmath>

namespace plsdk {

	using namespace std;

	float Norm2(float * x_1, float * y_1, float * x_2, float * y_2)
	{
		return sqrt(pow(*x_2 - *x_1, 2) + pow(*y_1 - *y_2, 2));
	}

	void Blh2Xyz(float lat, float lon, float alt, float * x, float * y, float * z)
	{
		int n, L0;
		double X, N54, W54, t, m, a54, e54, e_54;
		double iptr;
		double t_2 = 0, t_4 = 0, yita_2 = 0, yita_4 = 0;
		double lp = 0, lp_2 = 0;
		double SinL, CosL, CosL_2, SinL_2;
		double SinG, CosG;
		double daa, df, db2p, dl2p, dahm;
		double deltabo, deltalo;
		double w84, n84, m84, a84, e842, f84, f54, dx, dy, dz;
		double lati(lat), logi(lon), hegt(alt);
		double pi = 3.1415926535;

		lati = lati*pi / 180.0;
		logi = logi*pi / 180.0;

		SinL = sin(lati);
		CosL = cos(lati);
		SinG = sin(logi);
		CosG = cos(logi);
		CosL_2 = CosL*CosL;
		SinL_2 = SinL*SinL;
		a84 = 6378137.0;
		e842 = 0.00669437999014132;
		f84 = 1.0 / 298.257223563;
		a54 = 6378245.0;
		f54 = 1.0 / 298.3;
		dx = -16.0;
		dy = 147.0;
		dz = 77.0;
		w84 = sqrt(1 - e842*SinL_2);
		n84 = a84 / w84;
		m84 = a84*(1 - e842) / (w84*w84*w84);
		daa = a54 - a84;
		df = f54 - f84;
		db2p = (-dx*SinL*CosG - dy*SinL*SinG + dz*CosL + (a84*df + f84*daa)*sin(2 * lati)) / (m84*sin(1 / 3600.0*pi / 180.0));
		dl2p = (-dx*SinG + dy*CosG) / (n84*CosL*sin(1 / 3600.0*pi / 180.0));
		dahm = dx*CosL*CosG + dy*CosL*SinG + dz*SinL + (a84*df + f84*daa)*SinL_2 - daa;
		deltabo = (db2p / 3600.0)*pi / 180.0;
		deltalo = (dl2p / 3600.0)*pi / 180.0;
		logi = logi + deltalo;
		lati = lati + deltabo;
		hegt = hegt + dahm;
		SinL = sin(lati);
		CosL = cos(lati);
		CosL_2 = CosL*CosL;
		SinL_2 = SinL*SinL;
		a54 = 6378245.0;
		e54 = 0.0066934274898192;

		W54 = sqrt(1.0 - e54*SinL_2);
		N54 = a54 / W54;
		e_54 = 0.0067385254147;
		logi = logi * 180 / pi;
		modf(logi / 6.0, &iptr);
		n = (int)iptr + 1;
		L0 = n * 6 - 3;
		lp = (logi - L0)*pi / 180.0;
		lp_2 = lp*lp;
		m = CosL_2*lp_2;
		yita_2 = e_54*CosL_2;
		yita_4 = yita_2*yita_2;
		t = tan(lati);
		t_2 = t*t;
		t_4 = t_2*t_2;
		X = 111134.8611*lati*180.0 / pi
			- SinL*CosL*(32005.7799 + 133.9238*SinL_2 + 0.6973*SinL_2*SinL_2 + 0.0039*SinL_2*SinL_2*SinL_2);
		*y = static_cast<float>(X + N54*t*m*(0.5 + 1.0 / 24.0*(5.0 - t_2 + 9.0*yita_2 + 4.0*yita_4)*m
			+ 1.0 / 720.0*(61.0 - 58.0*t_2 + t_4)*m*m));
		*x = static_cast<float>(N54*CosL*lp*(1.0 + 1.0 / 6.0*(1 - t_2 + yita_2)*m
			+ 1.0 / 120.0*(5.0 - 18.0*t_2 + t_4 + 14.0*yita_2 - 58.0*yita_2*t_2)*m*m));
		*x = *x + 1000000 * n + 500000;
		*x = *x * 100 - X_OFFSET_DY;
		*y = *y * 100 - Y_OFFSET_DY;
		*z = alt * 100;
		return;
	}

	void Car2Earth(const Axis * car_axis, float * x, float * y)
	{
		float _x(*x), _y(*y);
		*x = _x*cos(car_axis->yaw) - _y*sin(car_axis->yaw) + car_axis->x;
		*y = _x*sin(car_axis->yaw) + _y*cos(car_axis->yaw) + car_axis->y;
		return;
	}

	void Earth2Car(const Axis * car_axis, float * x, float * y)
	{
		float _x(*x), _y(*y);
		*x = (_x - car_axis->x)*cos(car_axis->yaw) + (_y - car_axis->y)*sin(car_axis->yaw);
		*y = -(_x - car_axis->x)*sin(car_axis->yaw) + (_y - car_axis->y)*cos(car_axis->yaw);
		return;
	}

	void Transform(const Axis * src_axis, const Axis * dest_axis, float * x, float * y)
	{
		//@warning 使用此函数需要保证xy是车体坐标系下的坐标，否则转换是错误的
		float _x(0.0), _y(0.0);
		/* 先从车体坐标转换到大地坐标 */
		_x = *x*cos(src_axis->yaw) - *y*sin(src_axis->yaw) + src_axis->x;
		_y = *x*sin(src_axis->yaw) + *y*cos(src_axis->yaw) + src_axis->y;
		/* 再从大地坐标转回到车体坐标 */
		*x = (_x - dest_axis->x)*cos(dest_axis->yaw) + (_y - dest_axis->y)*sin(dest_axis->yaw);
		*y = -(_x - dest_axis->x)*sin(dest_axis->yaw) + (_y - dest_axis->y)*cos(dest_axis->yaw);
		return;
	}

	bool IsProjInBound(const Dot * A, const Dot * B, const Dot * T)
	{
		return (T->x - A->x)*(B->x - A->x) + (T->y - A->y)*(B->y - A->y) >= 0 && (T->x - B->x)*(A->x - B->x) + (T->y - B->y)*(A->y - B->y) >= 0;
	}

	float Dist2Line(const Dot * A, const Dot * B, const Dot * T)
	{
		return abs((A->x - T->x)*(B->y - T->y) - (B->x - T->x)*(A->y - T->y))*0.5f / sqrt(pow(A->x - B->x, 2) + pow(A->y - B->y, 2));
	}

	float VecAgnleCos(const Dot * A, const Dot * B, const Dot *O)
	{
		float oa_x(A->x), oa_y(A->y), ob_x(B->x), ob_y(B->y);
		if (O != nullptr)
		{
			oa_x -= O->x; oa_y -= O->y;
			ob_x -= O->x; ob_y -= O->y;
		}
		float cos = (oa_x*ob_x + oa_y*ob_y) / sqrt((oa_x*oa_x + oa_y*oa_y)*(ob_x*ob_x + ob_y*ob_y));
		if (cos > 1.0f) cos = 1.0f;//经过测试，可能会出现cos值稍微大于1.0的计算异常，这里做一些安全保护
		return cos;
	}

	float Dot2Yaw(const Dot * A, const Dot * O)
	{
		/*! 若调用时不给O，那么意味着A是车体坐标点*/
		float deltaX(A->x), deltaY(A->y);
		if (O != nullptr)
		{
			deltaX -= O->x;
			deltaY -= O->y;
		}
		//atan2: 求取从原点到目标点向量与x轴正方向的夹角，xy不可以同时等于0.
		float _atan = atan2(deltaY, deltaX) + 3 * M_PI / 2;;
		//转化成为正北为零，逆时针旋转的坐标系统
		if (_atan >= 2 * M_PI) _atan -= 2 * M_PI;
		return _atan;
	}

	float DeltaYaw(const float yaw1, const float yaw2)
	{
		float delta = abs(yaw1 - yaw2);
		if (delta > M_PI)
			return 2 * M_PI - delta;
		else
			return delta;
	}

	void LinearInterpolation(const Target start, const Target end, const float dist, list<Target> & list)
	{
		list.push_back(start);
		list.push_back(end);
		auto itr2 = list.begin();
		auto itr1 = itr2++;
		while (itr2 != list.end())
		{
			float d = sqrt(pow(itr1->x - itr2->x, 2) + pow(itr1->y - itr2->y, 2));
			if (d >= dist * 1.9f)//当两者之间距离可以插入一个点，那么就插入一个点
				itr2 = list.insert(itr2, Target{ (itr2->x + itr1->x) / 2, (itr2->y + itr1->y) / 2, itr1->yaw, itr1->speed });
			else
				itr1 = itr2++;
		}
		return;
	}

	void UniformInterpolation(const Target start, const Target end, const float step, list<Target> & list)
	{
		//首先使用向量表示
		list.clear();
		if ((start.x == end.x && start.y == end.y) || step <= 0.0f) return;
		float vec_a = end.x - start.x;//向量的x分量
		float vec_b = end.y - start.y;//向量的y分量
		float c = sqrt(step*step / (vec_a*vec_a + vec_b*vec_b));
		unsigned int count = 0;
		//通过比例缩放的方法均匀插值
		while (count*c <= 1.0f)
		{
			list.push_back(Target{ start.x + vec_a*count*c, start.y + vec_b*count*c, start.yaw, start.speed });
			count++;
		}
		return;
	}

	void Rasterisation(list<Target> targets, list<Pos> & gi)
	{
		float p_x, p_y;
		int X, Y;
		for (const Target & t : targets)
		{
			//这里处理越过小地图地图边界的情况
			if (abs(t.x) > 1500.0f || t.y >= 3000.0f || t.y < 0.0f) break;
			float h((t.x + 1500.0f) / 25.0f), v(t.y / 25.0f);
			modf(h, &p_x);
			modf(v, &p_y);
			X = static_cast<int>(p_x);
			Y = static_cast<int>(p_y);
			gi.push_back(Pos{ X, Y });
		}
		if (!gi.empty())
		{
			auto i2 = gi.begin();
			auto i1 = i2++;
			while (i2 != gi.end())
			{
				if (abs(i2->x - i1->x)>1 || abs(i2->y - i1->y)>1)
					i2 = gi.insert(i2, Pos{ (i2->x + i1->x) / 2, (i2->y + i1->y) / 2 });
				else
					i1 = i2++;
			}
		}
	}

	float CurvityBy3Point(const Target a, const Target b, const Target c)
	{
		float abc = sqrt((pow(a.x - b.x, 2) + pow(a.y - b.y, 2))*(pow(a.x - c.x, 2) + pow(a.y - b.y, 2))*(pow(b.x - c.x, 2) + pow(b.y - c.y, 2)));
		float abXac = ((b.x - a.x)*(c.y - a.y) - (c.x - a.x)*(b.y - a.y))*0.5f;//左正右负
		return 2.0f*abXac / abc;
	}

	bool IsLocateInSector(const Dot * A, const float R_min, const float R_max, const float angle_min, const float angle_max)
	{
		const float R_min_2 = R_min*R_min;
		const float R_max_2 = R_max*R_max;
		const float Range_2 = pow(A->x, 2) + pow(A->y, 2);
		float Yaw_A = atan2(A->x, A->y)*180.0f / M_PI;//!!!注意，这里求取的是与Y轴夹角，逆时针为负数
		bool check_yaw = Yaw_A >= angle_min && Yaw_A <= angle_max;
		return Range_2 >= R_min_2 && Range_2 <= R_max_2 && check_yaw;
	}

	void EnumMorphinPos(Morph * morph, const int max_pos, const float car_len)
	{
		morph->len = 0;
		if (morph->angle == 0.0f)//零转角特殊处理
		{
			while (morph->len < max_pos)
			{
				if (morph->len + int(car_len / MAP_ROI_CELL_SIZ) >= MAP_ROI_CELL_VER)	break;
				morph->pos[morph->len].x = MAP_ROI_CELL_HOR / 2 - 1;
				morph->pos[morph->len].y = morph->len + int(car_len / MAP_ROI_CELL_SIZ);
				++morph->len;
			}
		}
		else if (morph->angle < 0.0f)//向右打方向
		{
			const float car_len_2 = car_len * car_len;
			const float beta = 2 * car_len * tan(M_PI_2 - morph->angle * M_PI / 180.0f);
			const int bnd_y = static_cast<int>(car_len / MAP_ROI_CELL_SIZ);
			float y_1 = car_len, y_2 = car_len;
			for (int pos_x = 1, pos_y = static_cast<int>(car_len / MAP_ROI_CELL_SIZ);
				pos_x < MAP_ROI_CELL_HOR / 2 && morph->len < max_pos;
				++pos_x)
			{
				y_1 = y_2;
				y_2 = sqrt(car_len_2 - pow(pos_x*MAP_ROI_CELL_SIZ, 2) - pos_x*MAP_ROI_CELL_SIZ*beta);
				//NOTE: sealed @ 2015-08-16. 下面一段代码封存：原用于求无掉头morphin线的情况
				//if (y_2 < y_1) break;//圆弧仅计算上升的那一部分
				//const int min_y(y_1 / REMIX_RISK_MAP_SIZ);
				//const int max_y(y_2 / REMIX_RISK_MAP_SIZ);
				//for (pos_y = min_y;
				//	pos_y <= max_y && pos_y < REMIX_RISK_MAP_VER && num < max_pos;
				//	++pos_y, ++num)
				//{
				//	(pos + num)->x = pos_x + REMIX_RISK_MAP_HOR / 2 -1;
				//	(pos + num)->y = pos_y;
				//}
				if (y_2 >= y_1)
				{
					const int min_y = static_cast<int>(y_1 / MAP_ROI_CELL_SIZ);
					const int max_y = static_cast<int>(y_2 / MAP_ROI_CELL_SIZ);
					for (pos_y = min_y;
						pos_y <= max_y && pos_y < MAP_ROI_CELL_VER && pos_y >= bnd_y && morph->len < max_pos;
						++pos_y, ++morph->len)
					{
						morph->pos[morph->len].x = pos_x + MAP_ROI_CELL_HOR / 2 - 1;
						morph->pos[morph->len].y = pos_y;
					}
				}
				else
				{
					const int max_y = static_cast<int>(y_1 / MAP_ROI_CELL_SIZ);
					const int min_y = static_cast<int>(y_2 / MAP_ROI_CELL_SIZ);
					for (pos_y = max_y;
						pos_y >= min_y && pos_y < MAP_ROI_CELL_VER && pos_y >= bnd_y && morph->len < max_pos;
						--pos_y, ++morph->len)
					{
						morph->pos[morph->len].x = pos_x + MAP_ROI_CELL_HOR / 2 - 1;
						morph->pos[morph->len].y = pos_y;
					}
				}
			}
			//将轨迹圆参数存入到morph中
			morph->o.y = 0;
			morph->o.x = -tan(M_PI_2 - morph->angle * M_PI / 180.0f)*car_len;
			morph->r = sqrt(pow(morph->o.x, 2) + car_len_2);
		}
		else if (morph->angle > 0.0f)
		{
			const float car_len_2 = car_len * car_len;
			const float beta = 2 * car_len * tan(M_PI_2 - morph->angle * M_PI / 180.0f);
			const int bnd_y = static_cast<int>(car_len / MAP_ROI_CELL_SIZ);
			float y_1 = car_len, y_2 = car_len;
			for (int pos_x = -1, pos_y = static_cast<int>(car_len / MAP_ROI_CELL_SIZ);
				pos_x >= -MAP_ROI_CELL_HOR / 2 && morph->len < max_pos;
				--pos_x)
			{
				y_1 = y_2;
				y_2 = sqrt(car_len_2 - pow(pos_x*MAP_ROI_CELL_SIZ, 2) - pos_x*MAP_ROI_CELL_SIZ*beta);
				//NOTE: sealed @ 2015-08-16. 下面一段代码封存：原用于求无掉头morphin线的情况
				//if (y_2 < y_1) break;//圆弧仅计算上升的那一部分
				//const int min_y(y_1 / REMIX_RISK_MAP_SIZ);
				//const int max_y(y_2 / REMIX_RISK_MAP_SIZ);
				//for (pos_y = min_y;
				//	pos_y <= max_y && pos_y < REMIX_RISK_MAP_VER && num < max_pos;
				//	++pos_y, ++num)
				//{
				//	(pos + num)->x = pos_x + REMIX_RISK_MAP_HOR / 2;
				//	(pos + num)->y = pos_y;
				//}
				if (y_2 >= y_1)
				{
					const int min_y = static_cast<int>(y_1 / MAP_ROI_CELL_SIZ);
					const int max_y = static_cast<int>(y_2 / MAP_ROI_CELL_SIZ);
					for (pos_y = min_y;
						pos_y <= max_y && pos_y < MAP_ROI_CELL_VER && pos_y >= bnd_y && morph->len < max_pos;
						++pos_y, ++morph->len)
					{
						morph->pos[morph->len].x = pos_x + MAP_ROI_CELL_HOR / 2 - 1;
						morph->pos[morph->len].y = pos_y;
					}
				}
				else
				{
					const int max_y = static_cast<int>(y_1 / MAP_ROI_CELL_SIZ);
					const int min_y = static_cast<int>(y_2 / MAP_ROI_CELL_SIZ);
					for (pos_y = max_y;
						pos_y >= min_y && pos_y < MAP_ROI_CELL_VER && pos_y >= bnd_y && morph->len < max_pos;
						--pos_y, ++morph->len)
					{
						morph->pos[morph->len].x = pos_x + MAP_ROI_CELL_HOR / 2 - 1;
						morph->pos[morph->len].y = pos_y;
					}
				}
			}
			morph->o.y = 0;
			morph->o.x = -tan(M_PI_2 - morph->angle * M_PI / 180.0f)*car_len;
			morph->r = sqrt(pow(morph->o.x, 2) + car_len_2);
		}
		return;
	}

	void nextPt(Axis * pt, const float alpha, const float L, const float delta)
	{
		//alpha逆时针偏为正
		const float omiga = delta*tan(alpha) / L;
		//推算目标航向
		pt->yaw += omiga;
		if (pt->yaw >= 2 * M_PI)		pt->yaw -= 2 * M_PI;
		if (pt->yaw < 0.0f)			pt->yaw += 2 * M_PI;
		//推算x与y的坐标
		const float theta = pt->yaw + M_PI_2;
		pt->x += delta*cos(theta + omiga / 2);
		pt->y += delta*sin(theta + omiga / 2);
		return;
	}

	float SpeedLimitDecisionByStr(float str)
	{
		//预期轮角与路径曲率之间的关系
		//str=30	w=720 k=175*1e-5
		//str=25	w=600 k=148*1e-5
		//str=20	w=480 k=120*1e-5
		//str=15	w=360 k=91*1e-5
		//str=10	w=240 k=61*1e-5
		//str=7.5	w=180 k=46*1e-5
		//str=5		w=120 k=30*1e-5
		//str=2.5	w=60  k=15*1e-5
		//str=0		w=0   k=0
		//通过计算最大轮角来限制速度方向盘转角与轮角之间的关系24：1
		str = abs(str);
		float spd_limit;
		static float policy_table_str[3][12] = \
		{
			{22.00f, 13.00f, 9.500f, 6.500f, 3.500f, 1.800f, 1.000f, 0.700f, 0.500f, 0.350f, 0.200f, 0.000f},  //底层轮角
			{528.0f, 312.0f, 228.0f, 156.0f, 84.00f, 43.20f, 24.00f, 16.80f, 12.00f, 8.400f, 4.800f, 0.000f}, //方向盘角
			{5.000f, 6.000f, 7.500f, 12.00f, 15.00f, 20.00f, 25.00f, 30.00f, 35.00f, 40.00f, 45.00f, 50.00f}   //限速等级
		};
		for (int idx = 11; idx != -1; --idx)
		{
			if (str >= policy_table_str[0][idx])
				spd_limit = policy_table_str[2][idx];
			else
				break;
		}
		return spd_limit;
	}

	float SpeedLimitDecisionByTar(const Target tar, const float cur_str, float & speed)
	{
		//根据目标点的位置，计算一次规划到达目标点的轮角
		const float expected_str = CalcuExpectStr(tar.x, tar.y);//根据目标点计算出来的轮角
		float my_str;
		(abs(cur_str) > abs(expected_str)) ? (my_str = abs(cur_str)) : (my_str = abs(expected_str));
		float speed1 = SpeedLimitDecisionByStr(my_str);
		SpeedLimit(speed, speed1);
		return speed;
	}

	float SpeedLimitDecisionByDis(const float dist, float speed)
	{
		//刹车加速度定为160.0cm/s^2
		static float slow_pt[3][8] = \
		{
			{0.000f, 1060.0f, 1240.0f, 1540.0f, 1963.5f, 2337.0f, 3168.0f, 3946.0f},
			{0.000f, 0.0000f, 138.88f, 277.00f, 416.00f, 555.00f, 649.00f, 833.00f},
			{0.000f, 0.0000f, 5.0000f, 10.000f, 15.000f, 20.000f, 25.000f, 30.000f}
		};
		//根据匀加速直线模型限制当前规划速度
		for (int idx = 7; idx != -1; --idx)
		{
			if (dist <= slow_pt[0][idx])
				SpeedLimit(speed, slow_pt[2][idx]);
			else
				break;
		}
		return speed;
	}

	float speedLimitDecisionByStu(const STATUS stat)
	{
		float stat_spd_limit = 100.0f;
		switch (stat)
		{
		case S_INVALID:
			stat_spd_limit = 0.0f; break;
		case S_WAIT:
			stat_spd_limit = 0.0f; break;
		case S_WAIT_LIGHT:
			stat_spd_limit = 0.0f; break;
		case S_ROAD_NAV:
			stat_spd_limit = 50.0f; break;
		case S_CROSS_UND:
			stat_spd_limit = 20.0f; break;
		case S_STRAIGHT:
			stat_spd_limit = 20.0f; break;
		case S_LEFT:
			stat_spd_limit = 18.0f; break;
		case S_RIGHT:
			stat_spd_limit = 15.0f; break;
		case S_UTURN:
			stat_spd_limit = 8.0f; break;
		case S_PARKING:
			stat_spd_limit = 15.0f; break;
		case S_FREE_RUN:
			stat_spd_limit = 15.0f; break;
		case S_TASK_OVER:
			stat_spd_limit = 8.0f; break;
		}
		return stat_spd_limit;
	}

	float SpeedLimit(float & speed_in, const float speed_top)
	{
		if (speed_in > speed_top) speed_in = speed_top;
		if (speed_in < 0.0f) speed_in = 0.0f;
		return speed_in;
	}

	float SpeedKeep(float & speed_in, const float speed_floor)
	{
		if (speed_in < speed_floor) speed_in = speed_floor;
		return speed_in;
	}
	
	float CollisionAovidDist(float speed)
	{
		const float a = 300.0f; //设加速度为300cm/s_2
		const float car_len = 500.0f;//设车身长度为500cm
		const float safe_dist = 300.0f;//设安全避碰距离为3m
		const float response_dist = speed*0.2f;//设置动作反应距离：发送和接收
		const float break_dist = (speed*speed) / (2.0f*a);//匀加速直线运动求刹车距离
		return car_len + safe_dist + response_dist + break_dist;
	}
	
	float CalcuExpectStr(float tar_x, float tar_y)
	{
		//Earth2Car(pt, &tar_x, &tar_y);//目前使用局部坐标点
		if (abs(tar_x) < 1e-7) return 0.0f;//夹角太小则不计算圆方程，直接返回0轮角
		//轨迹圆过点(0, 285)和(tar_x, tar_y),圆心为(alpha, 0),先求得alpha
		//利用圆上每一点与圆心等距求得 alpha^2 + 285^2 = (tar_x - alpha)^2 + tar_y^2 解这个方程就可以得到圆心位置
		const float alpha = abs((pow(tar_x, 2) + pow(tar_y, 2) - 285.0f*285.0f) / (2.0f*tar_x));
		float str = atan2(285.0f, alpha) * 180.0f / M_PI;//根据圆心位置得到期望轮角
		if (str > 30.0f) str = 30.0f;//做轮角约束
		if (tar_x > 0.0f) str = -str;
		return str;
	}

	float Odometry2Speed(const ODO_INFO * odo, float * speed)
	{
		static ODO_INFO odo_pre = *odo;
		static unsigned short odo_win[5] = { 0, 0, 0, 0, 0 };
		static unsigned short odo_buf[5] = { 0, 0, 0, 0, 0 };

		unsigned short odo_l_dif = 0u;
		unsigned short odo_r_dif = 0u;
		//计算计数差值，当计数溢出时新值小于旧值，需要对计数做补偿
		(odo->left < odo_pre.left) ? (odo_l_dif = odo->left + (0xFFFF - odo_pre.left)) : (odo_l_dif = odo->left - odo_pre.left);
		(odo->right < odo_pre.right) ? (odo_r_dif = odo->right + (0xFFFF - odo_pre.right)) : (odo_l_dif = odo->right - odo_pre.right);

		//移动滑窗，将当前计数值插入到窗口内
		memmove(odo_win + 1, odo_win, sizeof(unsigned short) * 4);
		odo_win[0] = odo_l_dif + odo_r_dif;

		//更新缓存计数数据
		odo_pre = *odo;

		//采用中值滤波方法剔除过大或过小的野值
		memcpy(odo_buf, odo_win, sizeof(unsigned short) * 5);
		register int odo_mid = 0;
		for (unsigned int i = 0; i < 3; ++i)//事实上只需要计算出下标为2的值即可
		{
			odo_mid = odo_buf[i];
			for (unsigned int j = i + 1; j != 5; ++j)
			{
				if (odo_buf[i] >= odo_buf[j])
				{
					odo_mid = odo_buf[j];
					odo_buf[i] = odo_mid;
				}
			}
		}

		//通过标定参数，将左右里程计的脉冲计数换算成距离，猎豹参数 0.05028  巡洋舰参数 0.0513149  每单位脉冲的距离 单位m
		float ss = odo_mid * 0.5f * 51.3149f;
		//if (1 == odo->revers) *speed = -*speed;
		if (nullptr != speed) *speed = ss;

		return ss;
	}

}