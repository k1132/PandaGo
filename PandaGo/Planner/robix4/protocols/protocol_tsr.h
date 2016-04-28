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
 *  \brief  traffic sign recognition
 *  \author JZJ
 *  \history:
 *    jzj         2010/09/22   Creation.
 *    Qian Hui    2011/12/09   Modifed for 2012.
 *    ZhangShubao 2012/07/16   Add traffic sign type.
 */

#ifndef PROTOCOL_TSR_H
#define PROTOCOL_TSR_H

#pragma pack(push)
#pragma pack(1)

#include "protocol_head.h"

enum {
	TS_INVALID = 0X00,			/*!<0	invalid or fail detect*/
	TS_FORBID_LEFT,				/*!<1	forbid left turn*/
	TS_FORBID_RIGHT,			/*!<2	forbid right turn */
	TS_FORBID_STRAIGHT,			/*!<3	forbid straight */
	TS_FORBID_LEFT_RIGHT,		/*!<4	forbid left turn and right turn */
	TS_FORBID_STRAIGHT_LEFT,	/*!<5	forbid straight and left turn */
	TS_FORBID_STRAIGHT_RIGHT,	/*!<6	forbid straight and right turn */
	TS_FORBID_DRIVE_INTO,		/*!<7	forbid drive into */
	TS_STOP,					/*!<8	forbid stop */
	TS_SLOW_DOWN,				/*!<9	slow down */
	TS_PEDESTRIAN,				/*!<10	be careful of pedestrian */
	TS_UNEVEN_ROAD,				/*!<11	uneven road */
	TS_EMERGENCY_STOP,			/*!<12	emergency stop region */
	TS_STRAIGHT,				/*!<13	straight forward */
	TS_LEFT_TURN,				/*!<14	left turn forward */
	TS_RIGHT_TURN,				/*!<15	right turn forward */
	TS_STRAIGHT_LEFT,			/*!<16	straight or left turn forward */
	TS_STRAIGHT_RIGHT,			/*!<17	straight or right turn forward */
	TS_LEFT_RIGHT,				/*!<18	left turn or right turn forward */
	TS_ALONG_RIGHT_SIDE,		/*!<19	drive along with right side */
	TS_ALONG_LEFT_SIDE,			/*!<20	drive along with left side */
	TS_HONKING,					/*!<21	hoking */
	TS_DRIVE_AROUND_ISLAND,		/*!<22	drive along the island */
	TS_CROSSWALK,				/*!<23	crosswalk in front */
	TS_U_TURN,					/*!<24	allow u_turn */
	TS_PARKING,					/*!<25	parking in front */
	TS_TRAFFIC_MARK_CONE,		/*!<26	traffic mark cone */
	TS_LEFT_FORWORD,			/*!<27	drive left forward for construction*/
	TS_RIGHT_FORWORD,			/*!<28	drive right forward for construction*/
	TS_CHILD,					/*!<29  be careful of child */
	TS_CONSTRUCTION,			/*!<30  be careful of construction site */
	TS_SPEED_LIMIT,				/*!<31  Speed limit */

	NUM_TOTAL_TRAFFIC_SIGN		/*!<32	total number of events */

};

/*!< TSR -> FU and CAC, basic data. */
typedef struct {
    RBX_HEADER header;
    TRAFFIC_SIGNS signs;   
    ROAD_VER road_ver;                      /*!< yellow vertical road lines. */
    STATE state;
} TSR_FU_DATA, TSR_CAC_DATA;

#pragma pack(pop)

#endif /* PROTOCOL_TSR_H */
