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
 *  \file   protocol_prm.h
 *  \brief  Partner relationship management related def
 *  \author XSL
 *  \history:
 *    	xsl      2015/05/17  Creation.
 */

#ifndef PROTOCOL_PRM_H
#define PROTOCOL_PRM_H

#include "protocol_head.h"

#pragma pack(push)
#pragma pack(1)

typedef enum { 
	SELF = 1, 
	MASTER = 2, 
	SLAVE = 3 
} FMT_ROLE;
typedef enum { 
	NONE = 1, 
	GOOD = 2, 
	DROP = 3 
} FMT_EVENT;
typedef enum { 
	SERIAL = 1, 
	TRI = 2, 
	S2T = 3, 
	T2S = 4, 
	FREE = 5,
	PATROL = 6,
	HUNT = 7
} FMT_TASK;
typedef enum { 
	OK = 1, 
	LOST = 2 
} FMT_SIGNAL;
typedef enum { 
	START = 1, 
	RUN = 2 
} FMT_MODE;

#define MAX_CAR_NUM 8

/*!< PL <---> PRM  */
typedef struct {
    RBX_HEADER  header;
	UINT8		valid;
    UINT8 		robot_id;           		/*!< Local robot id. */
    UINT8 		robot_num;           		/*!< Valid number of car. */
	FMT_TASK	task_type;					/*!< Formation type of cars. */
	FMT_EVENT	event;						/*!< Formation event: NONE GOOD DROP. */
	FMT_SIGNAL	signal;						/*!< communication signal status  */
	FMT_MODE	mode;						/*!< is in start state */
	INT32		dis;						/*!< Expected distance. */
	INT32		speed;						/*!< Expected speed. */
    UINT8 		robot_role[MAX_CAR_NUM];	/*!< ROBOT_ROLE Set target role for low level robot. */
	OBJECT_INFO	object[MAX_CAR_NUM];		/*!< object information */
}PRM_PL_DATA, PL_PRM_DATA;

typedef struct {
	RBX_HEADER	header;
	UINT8		valid;
	OBJECT_INFO	car_info[MAX_CAR_NUM];
}FU_PRM_DATA;

/******************
 * PRM -> PRM data *
 ******************/

//! struct PRM -> PRM, PRM to PRM data
typedef struct {
	UINT8		head;				 /*!< Packet head for check. 0x55 for default*/
    UINT8 		target_id;           /*!< Target robot id. */
    UINT8 		source_id;           /*!< Source robot id. */
	UINT8		flag;				 /*!< If the frame data is valid. */
    RBX_HEADER  header;
    FRAME_ID    ack;       			 /*!< Sended and received frame id. */	
	FMT_EVENT 	event; 			 /*!< If the cooperation state is enabled. */
    UINT8 		source_role;         /*!< ROBOT_ROLE Source robot role. */
    UINT8 		target_role;         /*!< ROBOT_ROLE Set target role for low level robot. */
    UINT8 		desired_role;        /*!< ROBOT_ROLE Desired role for low level robot. */
	FMT_TASK 	task_type;        	 /*!< FORMATION_TYPE Type of task. */
    UINT8 		desired_status;      /*!< Status desired from high level robot. */
    UINT8 		current_status;      /*!< Current status. */
	FMT_SIGNAL	signal;				 /*!< communication signal status */
	FMT_MODE	mode;				 /*!< is in start state */
	OBJECT_INFO	object;				 /*!< object information */
    STATE 		state;               /*!< This car's state. */
} PRM_SEND_DATA, PRM_RECV_DATA;

typedef struct {
	RBX_HEADER		header;
	UINT8 			valid_num;
	PRM_RECV_DATA	packet[MAX_CAR_NUM];
}PRM_RECV_PACKET, PRM_SEND_PACKET;


#pragma pack(pop)

#endif
