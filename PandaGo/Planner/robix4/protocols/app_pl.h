/*  Copyright (C) 2011 - 2020
 *  ALV research group.
 *
 *  This file is part of the pl application.
 *
 *  ROBIX Library is "CLOSE SOURCE" software; Member of ALV research group
 *  of ZJU can redistribute it and/or modify it under the terms of ALV Lab 
 *  Software License 0.3.1415.
 */

/*! 
 *  \file   app_pl.h
 *  \brief  Interface to PL process function.
 *  \author Qian Hui
 *  \history:
 *    Qian Hui    2011/12/09   Created for 2012.
 */
#ifndef APP_PL_H
#define APP_PL_H

#include "protocol_head.h"
#include "protocol_fu.h"
#include "protocol_cac.h"
#include "protocol_event.h"
#include "protocol_pl.h"
#include "protocol_gp.h"
#include "protocol_lidar.h"
#include "protocol_prm.h"

#pragma pack(push)
#pragma pack(1)

/*!< The Mask bit definition.  */
enum {
    PL_HAVE_FU_PL_DATA = (1<<0),  /*!< Have fu_pl_data from fu. */
    PL_HAVE_CAC_PL_DATA = (1<<1), /*!< Have cac_pl_data from cac. */
    PL_HAVE_CS_PL_DATA = (1<<2),  /*!< Have cs_pl_data from cs. */
    PL_HAVE_ODO_INFO = (1<<3),     /*!< Have cs_pl_data from cs. */
    PL_HAVE_GP_INFO = (1<<4),  /*!< Have gp_info from gp. */
    PL_HAVE_PRM_PL_DATA = (1<<5),  /*!< Have prm_pl_data from prm. */
};

/*!< Input Data to Process_Plan Function.  */
typedef struct {
	UINT32 valid_mask;       /*!< Mask to decide data validation. */
    FU_PL_DATA fu_pl_data;   /*!< Data from fu. */
    CAC_PL_DATA cac_pl_data; /*!< Data from cac. */
    CS_PL_DATA cs_pl_data;   /*!< Data from cs */
    ODO_INFO odo_info;       /*!< Data from odometer. */
    GP_INFO gp_info;   		 /*!< Current task and its neighbors  */
    PRM_PL_DATA prm_pl_data; /*!< Received data from other robot */
    STATE state;             /*!< Current state. */
}PL_FUNC_INPUT;

/*!< Local output Data from Process_Fusion Function.  */
typedef struct {
    EVENT event;
}PL_LOCAL_DATA;

namespace UGV_PL{

int Process_Plan(PL_FUNC_INPUT *, PL_CS_DATA *, PL_PRM_DATA *, PL_LOCAL_DATA *);

}
#pragma pack(pop)

#endif /* APP_PL_H */
