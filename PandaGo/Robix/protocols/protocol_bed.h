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
 *  \file   protocol_bed.h
 *  \brief  BED(basic environment detection) related def
 *  \author Qian Hui ( qianhui@zju.edu.cn )
 *  \history:
 *    Qian Hui    2011/12/09   Copy from Old Version written by JZJ.
 */

#ifndef PROTOCOL_BED_H
#define PROTOCOL_BED_H

#include "protocol_head.h"

#pragma pack(push)
#pragma pack(1)

/*!< Terraint type. */
typedef enum {
    BLACKTOP,          /*!< Blacktop. */
    CEMENT,            /*!< Cement. */
    SANDSTONE,         /*!< Sandstone. */
    WATER,             /*!< Water. */
    SOIL,              /*!< Soil. */
    NONTRAVERSABILITY  /*!< Nontraversability. */
} TERRAIN_TYPE;

/*!< Weather type. */
typedef enum {
    SUNNY,        /*!< Sunny. */
    CLOUDY,       /*!< Cloudy. */
    SOMBER,       /*!< Somber. */
    RAINY_SMALL,  /*!< Small rainy. */
    RAINY_MIDDLE, /*!< Middle rainy. */
    RAINY_HEAVY,  /*!< Heavy rainy. */
    SNOWY,        /*!< Snowy. */
    HAILY         /*!< Haily */
} WEATHER_TYPE;

/*!< Light degree. */
typedef enum {
    L_LEVEL_0 = 0,
    L_LEVEL_1 = 1,
    L_LEVEL_2 = 2,
    L_LEVEL_3 = 3,
    L_LEVEL_4 = 4
} LIGHT_DEGREE;

/*!< Terrain. */
typedef struct {
    UINT8 terrain_type; /*!< Terrain type. */
    UINT8 fidelity;     /*!< Fidelity. */
} TERRAIN;

/*!< Lightness. */
typedef struct 
{
    UINT8 light_degree; /*!< Light degree. */
    UINT8 fidelity;     /*!< Fidelity. */
} LIGHTNESS;

/*!< Weather. */
typedef struct {
    UINT8 weather_type; /*!< Whether type. */
    UINT8 fidelity;     /*!< Fidelity. */
} WEATHER;

/*!< Rough level. */
typedef enum {
    R_LEVEL_0, /*!< Level 0. */
    R_LEVEL_1, /*!< Level 1. */
    R_LEVEL_2, /*!< Level 2. */
    R_LEVEL_3, /*!< Level 3. */
    R_LEVEL_4  /*!< Level 4. */
} ROUGH_LEVEL;

/*!< Roughness. */
typedef struct {
    UINT8 level;    /*!< Roughness level. */
    UINT8 fidelity; /*!< Fidelity. */
} ROUGHNESS;

/*!< Environment data = terrain + weather + light + roughness. */
typedef struct {
    TERRAIN terrain;
    WEATHER weather;
    LIGHTNESS lightness; /*!< Lightness data. */
    ROUGHNESS roughness; /*!< Roughness data. */
} ENV;

/*!< BED -> FU. */
typedef struct {
    RBX_HEADER header; /*!< Robix header. */
    ENV env;
    STATE state;
} BED_FU_DATA;

#pragma pack(pop)

#endif /* PROTOCOL_BED_H */
