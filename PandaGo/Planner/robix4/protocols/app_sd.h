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
 *  \file   app_sd.h
 *  \brief  Interface to Speed Detection.
 *  \author Qian Hui
 *  \history:
 *    Qian Hui    2014/06/06   Created for 2014.
 */
#ifndef APP_SD_H
#define APP_SD_H

#include "protocol_head.h"
#include "protocol_event.h"

#pragma pack(push)
#pragma pack(1)

/*!< Input Data to Process_Plan Function.  */
typedef struct {
    POSE_INFO pos_info_1;
    POSE_INFO pos_info_2;
    ODO_INFO  odo_info;
}SD_FUNC_INPUT;

/*!< Local output Data from Process_Fusion Function.  */
typedef struct {
    EVENT event;
}SD_LOCAL_DATA;

namespace UGV_SD{

int Speed_Detect(SD_FUNC_INPUT*, SD_LOCAL_DATA *);

}

#pragma pack(pop)

#endif /* APP_SD_H */
