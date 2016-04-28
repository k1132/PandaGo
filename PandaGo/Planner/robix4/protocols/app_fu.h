/*  Copyright (C) 2011 - 2020
 *  ALV research group.
 *
 *  This file is part of the fu application.
 *
 *  ROBIX Library is "CLOSE SOURCE" software; Member of ALV research group
 *  of ZJU can redistribute it and/or modify it under the terms of ALV Lab 
 *  Software License 0.3.1415.
 */

/*! 
 *  \file   app_fu.h
 *  \brief  Interface to fu process function.
 *  \author Qian Hui
 *  \history:
 *    Qian Hui    2011/12/09   Created for 2012.
 *    zsb         2012/03/12   split fmv into fmv_n and fmv_f, so 
 *                              is with mv in FU_FUNC_INPUT.
 *                2012/07/20   Add task points in FU_FUNC_INPUT.
 *                2014/06/06   change task points headfile.
 */
#ifndef APP_FU_H
#define APP_FU_H

#include "protocol_fmv.h"
#include "protocol_flir.h"
#include "protocol_lidar.h"
#include "protocol_mwr.h"
#include "protocol_tsr.h"
#include "protocol_tlr.h"
#include "protocol_fu.h"
#include "protocol_event.h"
#include "protocol_gp.h"
#include "protocol_prm.h"

#pragma pack(push)
#pragma pack(1)

/*!< The Mask bit definition.  */
enum {
    FU_HAVE_FMV_N_FU_DATA = (1<<0),      /*!< Have Roadline data from fmv_n. */
    FU_HAVE_FMV_F_FU_DATA = (1<<1),      /*!< Have Roadline data from fmv_f. */
    FU_HAVE_FMV_N_FU_STOPLINE = (1<<2),  /*!< Have Stopline data from fmv_n. */
    FU_HAVE_FMV_F_FU_STOPLINE = (1<<3),  /*!< Have Stopline data from fmv_f. */
    FU_HAVE_FMV_N_FU_ZEBRALINE = (1<<4), /*!< Have Zebraline data from fmv_n. */
    FU_HAVE_FMV_F_FU_ZEBRALINE = (1<<5), /*!< Have Zebraline data from fmv_f. */
    FU_HAVE_MV_N_FU_DATA = (1<<6),       /*!< Have Roadline data from mv_n. */
    FU_HAVE_MV_F_FU_DATA = (1<<7),       /*!< Have Roadline data from mv_f. */
    FU_HAVE_MV_N_FU_STOPLINE = (1<<8),   /*!< Have Stopline data from mv_n. */
    FU_HAVE_MV_F_FU_STOPLINE = (1<<9),   /*!< Have Stopline data from mv_f. */
    FU_HAVE_MV_N_FU_ZEBRALINE = (1<<10),  /*!< Have Zebraline data from mv_n. */
    FU_HAVE_MV_F_FU_ZEBRALINE = (1<<11),  /*!< Have Zebraline data from mv_f. */
    FU_HAVE_LIDAR32_FU_DATA = (1<<12),    /*!< Have Gridmap data from lidar32. */
    FU_HAVE_LIDAR64_FU_DATA = (1<<13),    /*!< Have Gridmap data from lidar64. */
    FU_HAVE_LIDAR64_FU_ROAD = (1<<14),    /*!< Have Roadline data from lidar. */
    FU_HAVE_FLIR_FU_DATA = (1<<15),     /*!< Have Roadline data from flir. */
    FU_HAVE_TSR_FU_DATA = (1<<16),      /*!< Have T-signs data from tsr. */
    FU_HAVE_TLR_FU_DATA = (1<<17),     /*!< Have T-lights data from tlr. */
    FU_HAVE_MWR_FU_DATA = (1<<18),     /*!< Have Dyn-Objs data from mwr. */
    FU_HAVE_BED_FU_DATA = (1<<19),     /*!< Have BED data from bed. */
    FU_HAVE_LL_FU_STOP = (1<<20),      /*!< Have stopline data from LL. */
    FU_HAVE_LR_FU_STOP = (1<<21),      /*!< Have stopline data from LR. */
    FU_HAVE_UL_FU_STOP = (1<<22),      /*!< Have stopline data from UL. */
    FU_HAVE_UR_FU_STOP = (1<<23),      /*!< Have stopline data from UR. */
    FU_HAVE_UL_FU_ZEBRA = (1<<24),     /*!< Have zebraline data from UL. */
    FU_HAVE_UR_FU_ZEBRA = (1<<25),     /*!< Have zebraline data from UR. */
    FU_HAVE_UL_FU_DATA = (1<<26),      /*!< Have road data from UL. */
    FU_HAVE_UR_FU_DATA = (1<<27),      /*!< Have road data from UR. */
    FU_HAVE_GP_INFO = (1<<28),         /*!< Have global point from GP. */
    FU_HAVE_LMS_FU_DATA= (1<<29),      /*!< Have LMS data from LMS. */
    FU_HAVE_ODO_INFO = (1<<30),    	   /*!< Have odo info from broadcast. */
};

/*!< Input Data to Process_Fusion Function.  */
typedef struct {
    UINT32 valid_mask;                   /*!< Mask to decide data validation. */
    FMV_FU_DATA fmv_n_fu_data;           /*!< Roadline data from fmv_n. */
    FMV_FU_DATA fmv_f_fu_data;           /*!< Roadline data from fmv_f. */
    FMV_FU_STOPLINE fmv_n_fu_stopline;   /*!< Stopline data from fmv_n. */
    FMV_FU_STOPLINE fmv_f_fu_stopline;   /*!< Stopline data from fmv_f. */ 
	FMV_FU_ZEBRALINE fmv_n_fu_zebraline; /*!< Zebraline data from fmv_n. */ 
	FMV_FU_ZEBRALINE fmv_f_fu_zebraline; /*!< Zebraline data from fmv_f. */
    MV_FU_DATA mv_n_fu_data;           /*!< Roadline data from mv_n. */
    MV_FU_DATA mv_f_fu_data;           /*!< Roadline data from mv_f. */
    MV_FU_STOPLINE mv_n_fu_stopline;   /*!< Stopline data from mv_n. */
    MV_FU_STOPLINE mv_f_fu_stopline;   /*!< Stopline data from mv_f. */
    MV_FU_ZEBRALINE mv_n_fu_zebraline; /*!< Zebraline data from mv_n. */
    MV_FU_ZEBRALINE mv_f_fu_zebraline; /*!< Zebraline data from mv_f. */
    LIDAR32_FU_DATA lidar_fu_data32;   /*!< Gridmap data from lidar32. */
    LIDAR64_FU_DATA lidar_fu_data64;   /*!< Gridmap data from lidar32. */
    LIDAR_FU_ROAD lidar_fu_road;       /*!< Roadline data from lidar. */
    FLIR_FU_DATA flir_fu_data;         /*!< Roadline data from flir. */
    TSR_FU_DATA tsr_fu_data;           /*!< T-signs data from tsr. */
    TLR_FU_DATA tlr_fu_data;           /*!< T-lights data from tlr. */
    MWR_FU_DATA mwr_fu_data;           /*!< Dyn-Objs data from mwr. */
    BED_FU_DATA bed_fu_data;           /*!< BED data from bed. */
    LLLC_FU_STOPLINE ll_fu_stopline;   /*!< Stopline data from lllc. */
    LRLC_FU_STOPLINE lr_fu_stopline;   /*!< Stopline data from lrlc. */
    ULLC_FU_STOPLINE ul_fu_stopline;   /*!< Stopline data from ullc. */
    URLC_FU_STOPLINE ur_fu_stopline;   /*!< Stopline data from urlc. */
    ULLC_FU_ZEBRALINE ul_fu_zebraline; /*!< Zebraline data from ullc. */
    URLC_FU_ZEBRALINE ur_fu_zebraline; /*!< Zebraline data from urlc. */
    ULLC_FU_DATA ul_fu_data;           /*!< Road data from ullc. */
    URLC_FU_DATA ur_fu_data;           /*!< Road data from urlc. */
    GP_INFO gp_info;  	       		   /*!< gp info from gp. */
    LMS_FU_DATA lms_fu_data;           /*!< LMS data from lms. */
    ODO_INFO odo_info;   		       /*!< odo info from broadcast. */
    STATE state;                       /*!< Current state. */
}FU_FUNC_INPUT;

/*!< Local output Data from Process_Fusion Function.  */
typedef struct {
	FU_PRM_DATA fu_prm;
    EVENT event;
}FU_LOCAL_DATA;

namespace UGV_FU{

int Process_Fusion(FU_FUNC_INPUT *, FU_PL_DATA *, FU_LOCAL_DATA *);

}

#pragma pack(pop)

#endif /* APP_FU_H */
