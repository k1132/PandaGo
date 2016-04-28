#ifndef TAKE_OVER_H_
#define TAKE_OVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <time.h>
#include "../robix4/protocols/app_pl.h"
#include "./trace_road.h"
#include "./basicfunction.h"

int take_over( int gridmap[][GRID_WIDTH], DYN_OBS* dyn_obs, int dyn_obs_num, MULTI_LANE* multi_lane, int mode, COOR2 *plan_line, int *status );
int check_s_obs(int gridmap[][GRID_WIDTH], COOR2* plan_line, int* line_num);
int hd_check_s_obs(int gridmap[][GRID_WIDTH_HD], COOR2* plan_line, int* line_num);
#endif //[TAKE_OVER_H_]