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
 *  \file   protocol_status.h
 *  \brief  Status related def
 *  \author JZJ
 *  \history:
 *    jzj         2009/07/28/  Creation.
 *    Qian Hui    2011/12/09   Modified for 2012.
 *    Zhangshubao 2012/03/07   Modified for test.
 *    Zhangshubao 2012/03/07   Add S_FREE_RUN.
 */

#ifndef PROTOCOL_STATUS_H
#define PROTOCOL_STATUS_H
#define BITS_OF_STATUS 20            /*<status word's right BITS_OF_STATUS bits represent the STATUS*/
#define HAS_OBS(e) ((1U << 30) & e)  /*30th bit represent whether there is an obstacle*/
#define Get_STATUS(e) (((1U << BITS_OF_STATUS) -1) & e)

typedef enum {
    S_INVALID = 0,          /*!<0   system is starting up*/
	S_WAIT,					/*!<1	stop and wait */
	S_WAIT_LIGHT,		    /*!<2	wait the light becoming green*/
	S_ROAD_NAV,             /*!<3   navigation on road*/
	S_CROSS_UND,	        /*!<4	get info about cross*/

	S_STRAIGHT,		        /*!<5	drive straight when passing cross*/
	S_LEFT,			        /*!<6	turn left when passing cross*/
	S_RIGHT,			    /*!<7	turn right when passing cross*/
	S_UTURN,			    /*!<8	u-turn when passing cross*/
	S_PARKING,			    /*!<9	parking at target point*/
    S_FREE_RUN,             /*!<10   run freely */
    S_TASK_OVER,            /*!<11  task fished*/

    NUM_STATUS              /*!< number of status */
}STATUS;

static inline int hasObs(const STATUS *s)
{
    return ((1U << 30) & *s);
}

static inline void setObs(STATUS *s)
{
    *s = (STATUS)((1U << 30) | *s);
}

static inline STATUS getStatus(const STATUS *s)
{
    return (STATUS)(((1U << BITS_OF_STATUS) - 1) & *s);
}

static inline void clearObs(STATUS *s)
{
    *s = (STATUS)(~(1U << 30) & *s);
}

static inline int getEnv(const STATUS *s)
{
    if (((1U << 29) & *s) == 0)
        return 0;
    else
        return 1;
}

static inline void setEnv(STATUS *s)
{
    *s = (STATUS)((1U << 29) | *s);
}

static inline void clearEnv(STATUS *s)
{
    *s = (STATUS)(~(1U << 29) & *s);
}

#endif /* PROTOCOL_STATUS_H */
