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
 *  \file   protocol_event.h
 *  \brief  Event related.
 *  \author Qian Hui ( qianhui@zju.edu.cn )
 *  \history:
 *    Qian Hui    2011/12/09   Creation.
 *    zsb         2012/03/07   Modified for test.
 *    zsb         2012/03/07   Add E_WAIT_LIGHT and E_FAILURE.
 */

#ifndef PROTOCOL_EVENTS_H
#define PROTOCOL_EVENTS_H

typedef enum {
    E_INVALID = 0,

    E_ALL_READY,
    E_WAIT_LIGHT,
    E_RUN_ROAD,
    E_OBS,
    E_OBS_OVER,
    E_CROSS_ACK,
    E_STRAIGHT,
    E_LEFT,
    E_RIGHT,
    E_UTURN,
    E_PARK,
    E_UNKNOWN,
    E_CROSS_OVER,
    E_TASK_OVER,
    E_TIMEOUT,
    E_FAILURE,

	NUM_EVENTS,					/*!< number of events */
} EVENT;

#endif /* PROTOCOL_EVENTS_H */
