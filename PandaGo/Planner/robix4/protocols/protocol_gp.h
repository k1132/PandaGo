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
 *  \file   protocol_gp.h
 *  \brief  Global plan and global points related.
 *  \author Qian Hui ( qianhui@zju.edu.cn )
 *  \history:
 *    xsl    2015/10/28   Creation.
 */

#ifndef PROTOCOL_GP_H
#define PROTOCOL_GP_H

#include <stdio.h>
#include "protocol_head.h"

#pragma pack(push)
#pragma pack(1)

/*
 * type definition of task point
 */

#define RBX_TASKPOINT_NUM 4
#define RBX_TASKPOINT_MID 10
typedef struct {
	UINT32 id;
    INT32 x;
    INT32 y;			/*!< (0,0) means invalid points.*/
    INT32 type;			/*!< type of task point. */
    INT32 direction;	/*!< direction of task point. */
    INT32 rsvd;
}TASK_POINT_XY;

/*!< Road level. */
typedef enum {
    R_LEVEL0 = 0,	/*!< default road 0, no value. */
    R_LEVEL1,		/*!< default road 1, no value */
    R_LEVEL2,		/*!< default road 2, no value*/
    R_LEVEL3,		/*!< grass land*/
    R_LEVEL4,		/*!< sim-struct & field road. */
    R_LEVEL5,		/*!< struct road. */
} ROAD_LEVEL;

/*!< Curve level. */
typedef enum {
    C_LEVEL0 = 0,
    C_LEVEL1,
    C_LEVEL2,
    C_LEVEL3,
} CURVE_LEVEL;

typedef struct {
	UINT8 lane_num;				/*!< number of lane, 0 is unknow */
    ROAD_LEVEL road_level;		/*!< road type. */
    CURVE_LEVEL curve_level;	/*!< curve level. */
	GUIDE_LINES gls;			/*!< guide lines*/
	UINT32	landmark;			/*!< bitset for landmark. */
}SCENE;

/*
 * type definition of task points delivered by robix
 */
typedef struct {
    RBX_HEADER header;                    /*!< robix header. */
	/* 
     * tps[0] --> last point that the car passed through.
     * tps[1] --> current point that the car is heading for.
     * tps[2] --> the second point that the car will head for.
     * tps[3] --> the destination point.
     */
    TASK_POINT_XY tps[RBX_TASKPOINT_NUM];	/*!< task points.  */
	TASK_POINT_XY mid[RBX_TASKPOINT_MID];	/*!< mid point in the intersection in-and-out*/
	INT32 valid_mid; 						/*!< valid number of mid point */
	SCENE scene;							/*!< geo information */
    STATE state; 
}GP_INFO;

/*
 * type definition of recovery_points delivered by robix
 */
typedef struct{
	RBX_HEADER header;
	INT32 recovery;
}RECOVERY_POINT;

#pragma pack(pop)

#endif /* PROTOCOL_GP_H */
