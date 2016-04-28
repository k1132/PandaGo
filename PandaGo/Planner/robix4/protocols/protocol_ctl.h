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
 *  \file   protocol_ctl.h
 *  \brief  System control related def
 *  \author cdp
 *  \history:
 *    	cdp      2015/08/30  Creation.
 */

#ifndef PROTOCOL_CTL_H
#define PROTOCOL_CTL_H

#include "protocol_head.h"

#pragma pack(push)
#pragma pack(1)

/*!< All modules in system. */
typedef enum {
    M_NONE = 0,      		/*!< None for any module. */
	
	M_PL,					/*!< Target for PL module. */
	M_FU,					/*!< Target for FU module. */
	M_GP,					/*!< Target for GP module. */
	M_GEO,					/*!< Target for GEO module. */
	M_GPS,					/*!< Target for GPS module. */
	M_PRM,					/*!< Target for PRM module. */
	M_SD,					/*!< Target for SD module. */
	M_BED,					/*!< Target for BED module. */

	M_LIDAR,				/*!< Target for LIDAR module. */
	M_FLIR,					/*!< Target for FLIR module. */
	M_MV_N,					/*!< Target for MV_N module. */
	M_FMV_N,				/*!< Target for FMV_N module. */
	M_TLSR,					/*!< Target for TLSR module. */
	M_LIDAR32,				/*!< Target for LIDAR32 module. */
	M_LMS,					/*!< Target for LMS module. */
	M_MWR,					/*!< Target for MWR module. */

	M_SENSOR,				/*!< Target for all sensors,include GPS,LIDAR,FLIR,MV_N,FMV_N,LIDAR32,LMS,MWR.*/

    M_COUNT,   				/*!< Counts of all modules. */
} MODULE;

/*!< Operations for modules. */
typedef enum {
    CMD_INVALID = 0,		/*!< Operation is invalid. */

	CMD_RESTART,			/*!< Restart target module. */
	CMD_ENABLE,				/*!< Make state of target module enabled. */
							/*!< Majority is enabled by default, except prm.*/ 
	CMD_UNABLE,
	CMD_SEND,				/*!< Send data to target module. */ 

	CMD_ORIGIN_START,		/*!< Send start record when record original data,start replay under replay mode.*/
	CMD_ORIGIN_STOP,		/*!< Send stop record when record original data,stop replay under replay mode.*/

	CMD_ORIGIN_POS,			/*!< Send start position when replay with original data.*/

	CMD_COUNT,				/*!< Counts of all commands. */
} OPERATION;

/*!< Control data for all modules. */
typedef struct {
    RBX_HEADER  header;
	INT32	sendtime;		/*!< Time that ctl_info was send out.*/
	MODULE	module;			/*!< Target module for control. */
	OPERATION cmd;			/*!< Command for different modules. */
	UINT32 data;			/*!< Data for commands if needed. */
}CONTROL_INFO;

#pragma pack(pop)

#endif /* PROTOCOL_CTL_H */
