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
 *  \file   protocol_lidar.h
 *  \brief  LIDAR related def
 *  \author JZJ
 *  \history:
 *    jzj         2009/07/13   Creation.
 *    Qian Hui    2011/12/09   Modifed for 2012.
 */

#ifndef PROTOCOL_LIDAR_H
#define PROTOCOL_LIDAR_H

#include "protocol_head.h"

#pragma pack(push)
#pragma pack(1)

/*!< LIDAR64 -> FU, basic data. */
typedef struct {
    RBX_HEADER header;    /*!< Robix header. */
    GRID_MAP_64 gridmap;  /*!< Grid map. */
    GRID_MAP_HD hdmap;    /*!< HD map. */
    STATE state;          /*!< State. */
} LIDAR64_FU_DATA;

/*!< LIDAR32 -> FU, basic data. */
typedef struct {
    RBX_HEADER header;    /*!< Robix header. */
    GRID_MAP_32 gridmap;  /*!< Grid map. */
    STATE state;          /*!< State. */
} LIDAR32_FU_DATA;

/*!< LIDAR -> FU and CAC, road data. */
typedef struct {
    RBX_HEADER header;                  /*!< Robix header. */
    ROAD_NATURAL_BOUNDARY boundary_cur; /*!< Boundary from curbs. */
    ROAD_NATURAL_BOUNDARY boundary_obs; /*!< Boundary from obstacles. */
    STATE state;                        /*!< State. */
} LIDAR_FU_ROAD, LIDAR_CAC_ROAD;

/*!< LIDAR64 -> FU, basic data. */
typedef struct {
    RBX_HEADER header; /*!< Robix header. */
    LMS lms;           /*!< Range data. */
    STATE state;       /*!< State. */
} LMS_FU_DATA,LMS_PL_DATA;
#pragma pack(pop)

#endif  /* PROTOCOL_LIDAR_H. */
