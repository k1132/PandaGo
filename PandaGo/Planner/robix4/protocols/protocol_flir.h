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
 *  \file   protocol_flir.h
 *  \brief  FLIR related def
 *  \author JZJ
 *  \history:
 *    JZJ         2009/09/22/  Add STATE in FLIR_FU_DATA
 *    JZJ         2009/10/18/  Add horizon_road_edge
 *                             Add HORIZON_TYPE
 *    Qian Hui    2011/12/09   Modified for 2012
 */
#ifndef PROTOCOL_FLIR_H
#define PROTOCOL_FLIR_H

#include "protocol_head.h"

#pragma pack(push)
#pragma pack(1)

/*!< struct FLIR -> FU. */
typedef struct{
    RBX_HEADER header;                      /*!< Robix header. */
    ROAD_NATURAL_BOUNDARY_PX road_boundary; /*!< Road boundary. */
    QUESTIONABLE_AREAS_PX questional_areas; /*!< Questional_area. */
    STATE state;                            /*!< State. */
} FLIR_FU_DATA, FLIR_CAC_DATA;

#pragma pack(pop)

#endif  /* PROTOCOL_FLIR_H */ 
