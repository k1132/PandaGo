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
 *  \file   app_cac.h
 *  \brief  Interface to cac process function.
 *  \author Qian Hui
 *  \history:
 *    Qian Hui    2011/12/09   Created for 2012.
 *    Qian Hui    2014/06/06   delete rnpl data from arguments.
 */
#ifndef APP_CAC_H
#define APP_CAC_H

#include "protocol_fmv.h"
#include "protocol_flir.h"
#include "protocol_lidar.h"
#include "protocol_tsr.h"
#include "protocol_tlr.h"
#include "protocol_cac.h"
#include "protocol_event.h"

#pragma pack(push)
#pragma pack(1)

/*!< The Mask bit definition.  */
enum {
    CAC_HAVE_FMV_CAC_DATA = (1<<0),      /*!< Have Roadline data from fmv. */
    CAC_HAVE_FMV_CAC_STOPLINE = (1<<1),  /*!< Have Stopline data from fmv. */
    CAC_HAVE_FMV_CAC_ZEBRALINE = (1<<2), /*!< Have Zebraline data from fmv. */
    CAC_HAVE_MV_CAC_DATA = (1<<3),       /*!< Have Roadline data from mv. */
    CAC_HAVE_MV_CAC_STOPLINE = (1<<4),   /*!< Have Stopline data from mv. */
    CAC_HAVE_MV_CAC_ZEBRALINE = (1<<5),  /*!< Have Zebraline data from mv. */
    CAC_HAVE_LIDAR_CAC_DATA = (1<<6),    /*!< Have Gridmap data from lidar. */
    CAC_HAVE_FLIR_CAC_DATA = (1<<7),     /*!< Have Roadline data from flir. */
    CAC_HAVE_TSR_CAC_DATA = (1<<8),      /*!< Have T-signs data from tsr. */
    CAC_HAVE_TLR_CAC_DATA = (1<<9),      /*!< Have T-lights data from tlr. */
    CAC_HAVE_RNPL_CAC_DATA = (1<<10),    /*!< Have RNPL data from rnpl. */
    CAC_HAVE_LL_CAC_STOP = (1<<11),      /*!< Have stopline data from LL. */
    CAC_HAVE_LR_CAC_STOP = (1<<12),      /*!< Have stopline data from LR. */
    CAC_HAVE_UL_CAC_STOP = (1<<13),      /*!< Have stopline data from UL. */
    CAC_HAVE_UR_CAC_STOP = (1<<14),      /*!< Have stopline data from UR. */
    CAC_HAVE_UL_CAC_ZEBRA = (1<<15),     /*!< Have zebraline data from UL. */
    CAC_HAVE_UR_CAC_ZEBRA = (1<<16),     /*!< Have zebraline data from UR. */
    CAC_HAVE_UL_CAC_DATA = (1<<17),      /*!< Have road data from UL. */
    CAC_HAVE_UR_CAC_DATA = (1<<18),      /*!< Have road data from UR. */
};

/*!< Input Data to Process_Fusion Function.  */
typedef struct {
    UINT32 vali_mask;                    /*!< Mask to decide data validation. */
    FMV_CAC_DATA fmv_cac_data;           /*!< Roadline data from fmv. */
    FMV_CAC_STOPLINE fmv_cac_stopline;   /*!< Stopline data from fmv. */
    FMV_CAC_ZEBRALINE fmv_cac_zebraline; /*!< Zebraline data from fmv. */
    MV_CAC_DATA mv_cac_data;             /*!< Roadline data from mv. */
    MV_CAC_STOPLINE mv_cac_stopline;     /*!< Stopline data from mv. */
    MV_CAC_ZEBRALINE mv_cac_zebraline;   /*!< Zebraline data from mv. */
    LLLC_CAC_STOPLINE ll_cac_stopline;   /*!< Stopline data from lllc. */
    LRLC_CAC_STOPLINE lr_cac_stopline;   /*!< Stopline data from lrlc. */
    ULLC_CAC_STOPLINE ul_cac_stopline;   /*!< Stopline data from ullc. */
    URLC_CAC_STOPLINE ur_cac_stopline;   /*!< Stopline data from urlc. */
    ULLC_CAC_ZEBRALINE ul_cac_zebraline; /*!< Zebraline data from ullc. */
    URLC_CAC_ZEBRALINE ur_cac_zebraline; /*!< Zebraline data from urlc. */
    ULLC_CAC_DATA ul_cac_data;           /*!< Road data from ullc. */
    URLC_CAC_DATA ur_cac_data;           /*!< Road data from urlc. */
    LIDAR_CAC_ROAD lidar_cac_road;       /*!< Roadline data from lidar. */
    FLIR_CAC_DATA flir_cac_data;         /*!< Roadline data from flir. */
    TSR_CAC_DATA tsr_cac_data;           /*!< Traffic signs data from tsr. */
    TLR_CAC_DATA tlr_cac_data;           /*!< Traffic lights data from tlr. */
    STATE state;                         /*!< Current state. */
}CAC_FUNC_INPUT;

/*!< Local output Data from Process_Fusion Function.  */
typedef struct {
    EVENT event;
}CAC_LOCAL_DATA;

namespace UGV_CAC{

int Process_CrossAck(CAC_FUNC_INPUT *, CAC_PL_DATA *, CAC_LOCAL_DATA *);

}

#pragma pack(pop)

#endif /* APP_CAC_H */
