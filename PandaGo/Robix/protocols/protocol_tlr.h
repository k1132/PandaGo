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
 *  \file   protocol_tlr.h
 *  \brief  traffic light recognition
 *  \author JZJ
 *  \history:
 *    jzj         2010/09/22   Creation.
 *    Qian Hui    2011/12/09   Modifed for 2012.
 */

#ifndef PROTOCOL_TLR_H
#define PROTOCOL_TLR_H

#pragma pack(push)
#pragma pack(1)

#include "protocol_head.h"

/*!< TLR -> FU and CAC, basic data. */
typedef struct {
    RBX_HEADER header;
    TRAFFIC_LIGHTS lights;   
    STATE state;
} TLR_FU_DATA, TLR_CAC_DATA;

#pragma pack(pop)

#endif /* PROTOCOL_TLR_H */
