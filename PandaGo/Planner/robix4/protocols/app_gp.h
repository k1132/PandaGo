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
 *  \file   app_gp.h
 *  \brief  Interface to Global Planning function.
 *  \author Qian Hui
 *  \history:
 *    Qian Hui    2014/06/06   Created for 2014.
 */
#ifndef APP_GP_H
#define APP_GP_H

#include "protocol_gp.h"
#include "protocol_event.h"

#pragma pack(push)
#pragma pack(1)

/*!< Input Data to Process_Plan Function.  */
typedef struct {
    STATE state; 
	RECOVERY_POINT reco_point; 	/*!< recovery point read by rbxAssistant*/
}GP_FUNC_INPUT;

/*!< Local output Data from Process_Fusion Function.  */
typedef struct {
    EVENT event;
}GP_LOCAL_DATA;

namespace UGV_GP{

int Global_Plan(GP_FUNC_INPUT*, GP_INFO*, GP_LOCAL_DATA *);

}

#pragma pack(pop)

#endif /* APP_GP_H */
