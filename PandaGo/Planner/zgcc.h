#ifndef ZGCC_H
#define ZGCC_H

#include <app_pl.h>
#include "./zgcc/trace_road.h"

extern void plan_init();
extern void set_road_type(int val);
extern void reset_for_wait();
extern int pl_road_trace_interface(int frame_id, PL_FUNC_INPUT *pl_input, PL_CS_DATA *pl_cs_data, PL_LOCAL_DATA *pl_local_data);
extern void set_db_flag(PL_CS_DATA *pl_cs_data);
extern int get_zgcc_plan_status();

#endif