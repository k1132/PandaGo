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
 *  \file   protocol_gps.h
 *  \brief  GPS and ODO related def
 *  \author CDP
 *  \history:
 *    	cdp      2015/08/4  Creation.
 */

#ifndef PROTOCOL_GPS_H
#define PROTOCOL_GPS_H

#include "protocol_head.h"


#pragma pack(push)
#pragma pack(1)

typedef struct
{
	UINT8		 	head;
	UINT8		 	flag;
	UINT8		 	mode; 				/*!< different mode for gps  */
	UINT8			sol_stat;			/*!< state of the inertial bavigation. */
	UINT8			pos_type;			/*!< type of gps. */
	UINT8			svs;				/*!< number of satellite. */
	UINT8			maxerror;			/*!< maximun deviation. */
	INT32		    position[3];		/*!< position of car. */
	INT32	 		velocity[3];		/*!< velctity of car. */
	INT16	 		attitude[3]; 		/*!< attitude of car. */
	INT16	 		gyro[3]; 			/*!< gyro of car. */
	INT16	 		acclerate[3];		/*!< acclerate of car. */
	UINT8			sum;				/*!< sum for check. */
}GPS_RECV_DATA;

typedef struct 
{
    INT8		head;    
    INT8		flag; 
    UINT8		revers;					/*!< 1: reverse; 0: forwarding. */
    UINT16		left;					/*!< left rounds of odo. */
    UINT16		right;					/*!< right rounds of odo. */
    UINT8		sum;					/*!< sum for check. */
}ODO_RECV_DATA;

#pragma pack(pop)
#endif
