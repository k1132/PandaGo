/*  Copyright (C) 2011 - 2020
 *  ALV research group.
 *
 *  This file is part of the ROBIX Library.
 *
 *  ROBIX Library is "CLOSE SOURCE" software; Member of ALV research group
 *  of ZJU can redistribute it and/or modify it under the terms of ALV Lab 
 *  Software License 0.3.1415.
 */

/*! 
 *  \file   protocol_pl.h
 *  \brief  PL related def
 *  \author JZJ
 *  \history:
 *    jzj      2009/07/27  Creation.
 *    jzj      2009/09/26  add notificator and cur_wp
 *    cpp      2010/03/29  add CS_PL_DATA in PL_INSPECTOR_DATA
 *    cpp      2010/05/31  add PL_ROAD_DATA
 *    Qian Hui 2011/12/09  Modifed for 2012.
 *    Zhangshu 2012/07/12  Add task points in PL_INSPECTOR_DATA
 */

#ifndef PROTOCOL_PL_H
#define PROTOCOL_PL_H

#include "protocol_head.h"
#include "protocol_gp.h"
#include "protocol_gps.h"

#pragma pack(push)
#pragma pack(1)

#define PL_PATH_LEN 20
#define PT_REFE_NUM  2 

/*!< PL -> CS. */
typedef struct {
    FRAME_ID id;
    INT16 isok;                       /*!< If this frame is valid. */
    INT16 number_of_effective_points; /*!< Path valid points number. */
    INT32 sys_command;                /*!< System command:
                                         it is bit map, 
                                         0 represents OFF, 1 represents ON
                                         0x01: move backward, 
                                         0x02: urgency stop,
                                         0x04: left light,
                                         0x08: right light,
                                         0x10: double light,
                                         0x20: alarm light. 
                                       */
    COOR2 path[PL_PATH_LEN];          /*!< Path points , (0, 0) as invalid. */
    INT16 speed;                      /*!< Expected speed, in cm/sec. */
    STATE state;                      /*!< This car's state. */
    COOR2 ref_pts[PT_REFE_NUM];       /*!< Reference points. */
} PL_CS_DATA;

/*!< CS -> PL */
typedef struct {
    FRAME_ID id;             
    INT16 isok;                 /*!< If this frame is valid, 1 is valid. */
    TIME_STAMP begin;           /*!< Begin time. */
    TIME_STAMP end;             /*!< End time. */
    INT32 plan_angle;           /*!< The plan angle of cs, unit is degree. */
    INT32 real_angle;           /*!< The real angle of the car, unit is degree. */

    INT8 isauto;                /*!< if auto. */
    INT16 brake_value;          /*!< break value. */
    INT16 throttle_value;       /*!< throttle value. */
    COOR2 cs_path[PL_PATH_LEN];         /*!< Path points. */
    INT16 number_of_effective_points;   /*!< Effective num of points. */
    STATE state;
} CS_PL_DATA;

/*!< PL -> INSPECTOR  */
typedef struct {
    RBX_HEADER  header;

    FRAME_ID id_fu_data;           /*!< Frame id for fu road data. */
    FRAME_ID id_gp_data;           /*!< Frame id for gp data. */
    FRAME_ID id_prm_pl_data;       /*!< Frame id from prm to pl data. */
    FRAME_ID id_pl_prm_data;       /*!< Frame id from pl to prm data. */
    FRAME_ID id_prm_send_pack;     /*!< Frame id for prm send data. */
    FRAME_ID id_prm_recv_pack;     /*!< Frame id for prm recv data. */
    FRAME_ID id_cac_data;          /*!< Frame id for fu cross data. */
	FRAME_ID id_ctl_info;		   /*!< Frame id for control info. */

	PL_CS_DATA pl_cs_data;		   /*!< The data from pl to cs. */
    CS_PL_DATA cs_pl_data;         /*!< The date from cs to pl. */ 
	ODO_INFO odo_info;			   /*!< The data form odometer */
	GPS_RECV_DATA gps_recv;		   /*!< The data from gps. */
} PL_INSPECTOR_DATA;

#pragma pack(pop)

#endif  /* PROTOCOL_PL_H */
