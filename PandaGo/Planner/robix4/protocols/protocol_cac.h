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
 *  \file   protocol_cac.h
 *  \brief  Second FU for Cross Acknowledge.
 *  \author Qian Hui, qianhui@zju.edu.cn 
 *  \history:
 *    Qian Hui    2012/02/04  Creation.
 */

#ifndef PROTOCOL_CAC_H
#define PROTOCOL_CAC_H

#include "protocol_head.h"

#pragma pack(push)
#pragma pack(1)

typedef struct { 
    UINT8 fidelity; /*!< Fidelity. */
} CROSS_ACKNOWLEDGE;

/*!< CAC -> PL. */
typedef struct {
    RBX_HEADER header;                 /*!< Robix header */
    CROSS_ACKNOWLEDGE cross_ack;  /*!< Cross decision data */
    STATE state;                       /*!< State */
} CAC_PL_DATA;

#pragma pack(pop)

#endif /* PROTOCOL_CAC_H */ 
