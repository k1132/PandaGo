#ifndef PLSDK_DEF_H
#define PLSDK_DEF_H

//巡洋舰 长510cm 宽220cm 高240cm(215+25) 轴距285cm
//猎豹  长480cm 宽190cm 高240cm(215+25) 轴距270cm

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif
#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923f
#endif
#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616f
#endif
#ifndef M_E
#define M_E        2.71828182845904523536f
#endif

#ifdef USING_ROBIX4
#include "robix4/rbx4api.h"
#define CFG_FILE "/home/alv/ugv/bin/config/pl.cfg"
#else
#define MBUG(fmt, ...) (fprintf(stdout, fmt, ##__VA_ARGS__))
#define CFG_FILE "./Planner/pl.cfg"
#endif

#define X_OFFSET_DY 2056580900.0f
#define Y_OFFSET_DY 361405900.0f
#define PL_PATH_STEP 70.0f

#define MAP_64_CELL_NUM			51200
#define MAP_64_CELL_VER			320
#define MAP_64_CELL_HOR			160
#define MAP_64_CELL_SIZ			25.0f
#define MAP_HD_CELL_NUM			57600
#define MAP_HD_CELL_VER			240
#define MAP_HD_CELL_HOR			240
#define MAP_HD_CELL_SIZ			12.5f
#define MAP_ROI_CELL_NUM		14400
#define MAP_ROI_CELL_VER		120
#define MAP_ROI_CELL_HOR		120
#define MAP_ROI_CELL_SIZ		25.0f
enum MAP_SENSITIVITY { ORIGIN = 0, NARROW = 1, EXPAND = 2 };
enum MAP_LANE_MARK { NORMAL_LANE = 0, CURRENT_LANE = 1, IMAGINE_LANE = 2, WRONG_LANE = 3 };
enum MAP_LINE_TYPE { SOLID_LINE = 0, DASH_LINE = 1 };
enum MAP_LINE_COLOR{ WHITE_LINE = 0, YELLOW_LINE = 1 };

#define MORPHIN_TEST_STEP		0.25f
#define MORPHIN_TEST_L			30.0f
#define MORPHIN_TEST_R			-30.0f
#define MORPHIN_TEST_NUM		241
#define MORPHIN_TEST_MAX_LEN	180//最多存80个关联栅格步长的点（最远可以向前探测到达25米）

namespace plsdk
{
	struct Dot{ float x; float y; };
	struct Dot3{ float x; float y; float val; };
	struct Pos{ int x; int y; };
	struct Pair{ int key; float val; };
	struct Axis{ float x; float y; float yaw; float str; };
	struct Target{ float x; float y; float yaw; float speed; };
	struct Counter{ int cnt; int border; int step; int trigger; };

	struct Morph{ int idx; int len; float angle; float value; Dot o; float r; Pos pos[MORPHIN_TEST_MAX_LEN]; };
	struct Path{ Dot * dots; int dot_num; int dot_max; };
	struct Lane{ Dot edge_l[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE]; int num_l; Dot edge_r[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE]; int num_r; int type; int len; };
	struct Obs{ Dot * obs; int obs_num; int obs_max; };
};
#endif