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
 *  \file   protocol_mwr.h
 *  \brief  millimeter radar
 *  \author JZJ
 *  \history:
 *    jzj         2010/09/22   Creation.
 *    Qian Hui    2011/12/09   Modifed for 2012.
 */

#ifndef PROTOCOL_MWR_H
#define PROTOCOL_MWR_H

#include "protocol_head.h"

#pragma pack(push)
#pragma pack(1)

/*!< MWR -> FU, and Ladar. */
typedef struct {
    RBX_HEADER header;
    DYNAMIC_POINT_OBJS objs;
    STATE state;
} MWR_FU_DATA, MWR_LIDAR_DATA;

#pragma pack(pop)

#endif
