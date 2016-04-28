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
 *  \file   protocol_fu.h
 *  \brief  FU related def
 *  \author JZJ
 *  \history:
 *    jzj         2009/07/28/  Creation.
 *    Qian Hui    2011/12/09   Modifed for 2012.
 */

#ifndef PROTOCOL_PLFU_H
#define PROTOCOL_PLFU_H

#include "protocol_head.h"
#include "protocol_bed.h"

#pragma pack(push)
#pragma pack(1)

/*!< FU -> PL, road data. */
typedef struct{
    RBX_HEADER header;              /*!< Robix header. */

    ROAD_LINES lines;               /*!< Road lines. */
    ROAD_NATURAL_BOUNDARY boundary; /*!< Boundary. */
    ROAD_NATURAL_WIDTH width;       /*!< Width. */
    CROSS cross;                    /*!< Cross. */

    ENV env;                        /*!< Environment detection data. */

    STOP_LINES stop_lines;          /*!< Stop lines. */
    ZEBRA_LINES zebra_lines;        /*!< Zebra lines. */

    TRAFFIC traffic;                /*!< Traffic lights and signs decode. */

    STATE state;                    /*!< State. */

    GRID_MAP_64 gridmap;		    /*!< Grid map. */
    GRID_MAP_HD hdmap;		    	/*!< HD map. */
    POZITION gridmap_pos;           /*!< Grid map position. */
    LMS  lms;                       /*!< lms. */
} FU_PL_DATA;

/*!< FU -> INSPECTOR, road data. */
typedef struct{
    RBX_HEADER  header;
    /*
     * For roads detecton 
     */
    FRAME_ID id_fmv_n;
    FRAME_ID id_fmv_f;
    FRAME_ID id_mv_n;
    FRAME_ID id_mv_f;
    FRAME_ID id_flir;  
    FRAME_ID id_lms;  
    FRAME_ID id_gp;  
    /*
     * For 3D roads and objs detecton 
     */
    FRAME_ID id_lidar64;
    FRAME_ID id_lidar32;
    FRAME_ID id_mwr;
    /*
     * For tlr and tsr detecton 
     */
    FRAME_ID id_tlr;
    FRAME_ID id_tsr;
    /*
     * For stop and zebra line detecton 
     */
    FRAME_ID id_mv_n_stopline;
    FRAME_ID id_mv_f_stopline;
    FRAME_ID id_mv_n_zebraline;
    FRAME_ID id_mv_f_zebraline;
    FRAME_ID id_fmv_n_stopline;
    FRAME_ID id_fmv_f_stopline;
    FRAME_ID id_fmv_n_zebraline;
    FRAME_ID id_fmv_f_zebraline;
    
    FRAME_ID id_urlc;
	FRAME_ID id_ullc;
	
    FRAME_ID id_lrlc_stopline;
    FRAME_ID id_lrlc_zebraline;

    FRAME_ID id_lllc_stopline;
    FRAME_ID id_lllc_zebraline;

    FRAME_ID id_urlc_stopline;
    FRAME_ID id_urlc_zebraline;

    FRAME_ID id_ullc_stopline;
    FRAME_ID id_ullc_zebraline;

	FRAME_ID id_fu_prm_data;

	FU_PL_DATA	fu_pl_data;
} FU_INSPECTOR_DATA;

#pragma pack(pop)

#endif /* PROTOCOL_FU_H */ 
