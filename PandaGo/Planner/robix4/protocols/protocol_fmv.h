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
 *  \file   protocol_fmv.h/protocol_mv.h
 *  \brief  front mv related def
 *  \author JZJ
 *  \history:
 *    jzj         2009/11/10/  Created
 *    Qian Hui    2011/12/09   Modifed for 2012.
 *    ZhangShubao 2012/7/1     Add member road_opening in FMV->FU data structure.
 */
#ifndef PROTOCOL_FMV_H
#define PROTOCOL_FMV_H

#include "protocol_head.h"

#pragma pack(push)
#pragma pack(1)

/*!< FMV -> FU, for basic data. */
typedef struct{
    RBX_HEADER header;                      /*!< Robix header. */
    ROAD_VER road_ver;                      /*!< Vertical road lines. */
    ROAD_HOR road_hor;                      /*!< Horizontal road lines. */
    ROAD_OPENING road_opening;              /*!< Road opening */
    ROAD_NATURAL_BOUNDARY_PX road_boundary; /*!< Road boundary. */
    QUESTIONABLE_AREAS_PX questional_areas; /*!< Questional areas. */
    STATE state;                            /*!< State. */
} FMV_FU_DATA, MV_FU_DATA, FMV_CAC_DATA, MV_CAC_DATA;

/*!< FMV -> FU, for stop line. */
typedef struct {
    RBX_HEADER header;           /*!< Robix header. */
    STOP_LINES_PX stop_lines_px; /*!< Stop line. */
    STATE state;                 /*!< State. */
} FMV_FU_STOPLINE, MV_FU_STOPLINE, FMV_CAC_STOPLINE, MV_CAC_STOPLINE;

/*!< FMV -> FU, for zebra line. */
typedef struct {
    RBX_HEADER header;             /*!< Robix header. */
    ZEBRA_LINES_PX zebra_lines_px; /*!< Zebra line. */
    STATE state;                   /*!< State. */
} FMV_FU_ZEBRALINE, MV_FU_ZEBRALINE, FMV_CAC_ZEBRALINE, MV_CAC_ZEBRALINE;

typedef FMV_FU_STOPLINE LRLC_FU_STOPLINE;
typedef FMV_FU_STOPLINE LLLC_FU_STOPLINE;
typedef FMV_FU_STOPLINE URLC_FU_STOPLINE;
typedef FMV_FU_STOPLINE ULLC_FU_STOPLINE;

typedef FMV_FU_ZEBRALINE LRLC_FU_ZEBRALINE;
typedef FMV_FU_ZEBRALINE LLLC_FU_ZEBRALINE;
typedef FMV_FU_ZEBRALINE URLC_FU_ZEBRALINE;
typedef FMV_FU_ZEBRALINE ULLC_FU_ZEBRALINE;

typedef FMV_CAC_STOPLINE LRLC_CAC_STOPLINE;
typedef FMV_CAC_STOPLINE LLLC_CAC_STOPLINE;
typedef FMV_CAC_STOPLINE URLC_CAC_STOPLINE;
typedef FMV_CAC_STOPLINE ULLC_CAC_STOPLINE;

typedef FMV_CAC_ZEBRALINE LRLC_CAC_ZEBRALINE;
typedef FMV_CAC_ZEBRALINE LLLC_CAC_ZEBRALINE;
typedef FMV_CAC_ZEBRALINE URLC_CAC_ZEBRALINE;
typedef FMV_CAC_ZEBRALINE ULLC_CAC_ZEBRALINE;

typedef FMV_FU_DATA URLC_FU_DATA;
typedef FMV_FU_DATA ULLC_FU_DATA;
typedef FMV_FU_DATA URLC_CAC_DATA;
typedef FMV_FU_DATA ULLC_CAC_DATA;

#pragma pack(pop)

#endif /* PROTOCOL_FMV_H */
