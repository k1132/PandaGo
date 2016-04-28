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
 *  \file   protocol_head.h
 *  \brief  common parts of protocols.
 *  \author Qian Hui ( qianhui@zju.edu.cn )
 *  \history:
 *    Qian Hui    2011/12/09   Creation.
 *    Qian Hui    2012/02/05   Modified for:
 *
 *       (1) Add BEVEL primitive datatype and STA_OBSTACLE_AREA;
 *       (2) Add road type, such as ROAD_NATURAL_BOUNDARY and ROAD_HOR;
 *       (3) Add TRAFFIC_SIGN to support location, sub-category, fidelity, etc;
 *       (4) Add TRAFFIC_light to support location, fidelity, etc;
 *
 *    Qian Hui    2012/02/12   Modified for:
 *
 *       (1) Add coordinates in image. ;
 *       (2) Add CROSS type. ;
 *
 *    Zsb         2012/03/15  Modified for:
 *       (1) Add CAR_WIDTH;
 *
 *    Qian Hui    2014/07/01   Modified for:
 *       (1) Add ODO_INFO for odometer;
 *
 *    xsl		  2014/10/06   Modified for:
 *       (1) Add GUIDE_LINES for geo;
 *       (2) Add STA_OBSTACLE_AREA in  GRID_MAP_64;
 *
 *    cdp		  2015/9/06    Modified for:
 *       (1) Add middle_point[10] and valid_num in CROSS;
 */

#ifndef PROTOCOL_HEAD_H
#define PROTOCOL_HEAD_H

#include <stdint.h>

#pragma pack(push)
#pragma pack(1)

/*!
 * (1) Type definition of primitive datatype.
 */

typedef uint32_t UINT32; /*!< 32 bits, unsigned integer. */
typedef int32_t  INT32;  /*!< 32 bits, signed integer. */
typedef uint16_t UINT16; /*!< 16 bits, unsigned short integer. */
typedef int16_t  INT16;  /*!< 16 bits, signed short integer. */
typedef uint8_t  UINT8;  /*!<  8 bits, unsigned char integer. */
typedef int8_t   INT8;   /*!<  8 bits, signed char integer. */


/*!
 * (2) Type definition of geometry description datatype. 
 */

/*!< Coordinate, speed, or acceleration of point in 1-D space. */
typedef INT32 COOR1, SPD1D, ACC1D;

/*!< Coordinate, speed, or acceleration of point in 2-D space. */
typedef struct {
    INT32  x; /*!< Component in X coordinate. */
    INT32  y; /*!< Component in Y coordinate. */
} COOR2, SPD2D, ACC2D;	

/*!< Coordinate, speed, or acceleration of point in 3-D space. */
typedef struct {
    INT32 x; /*!< Component in X coordinate. */
    INT32 y; /*!< Component in Y coordinate. */
    INT32 z; /*!< Component in Z coordinate. */
} COOR3, SPD3D, ACC3D;

/*!< Basic convex quadrilateral definition. */
typedef struct {
    COOR2 coor2[4]; /*!< Obstacle represented by 4 point */
} QUAD;

/*!< Basic rectangle definition. */
typedef struct {
    COOR2 left_top;     /*!< Left top coordinate of rectangle. */
    COOR2 right_bottom; /*!< Right bottom coordinate of rectangle. */
} REKT;

/*!< Bevel rectangle definition. */
typedef struct {
    COOR2 left_top;     /*!< Left top coordinate of rectangle. */
    COOR2 left_bottom;  /*!< Left bottom coordinate of rectangle. */
    COOR2 right_bottom; /*!< Right bottom coordinate of rectangle. */
} BEVEL;


/*!< Coordinate, speed, or acceleration of point in image. */
typedef INT16 COOR1_PX, COOR1_LC;

/*!< Coordinate, speed, or acceleration of point in image. */
typedef struct {
    INT16  x; /*!< Component in X coordinate. */
    INT16  y; /*!< Component in Y coordinate. */
} COOR2_PX, COOR2_LC;	

/*!< Basic convex quadrilateral definition in image. */
typedef struct {
    COOR2_PX coor_px[4]; /*!< Obstacle represented by 4 point */
} QUAD_PX, QUAD_LC;

/*!< Basic rectangle definition in image. */
typedef struct {
    COOR2_PX left_top;     /*!< Left top coordinate of rectangle. */
    COOR2_PX right_bottom; /*!< Right bottom coordinate of rectangle. */
} REKT_PX, REKT_LC;


/*!
 * (3) Type definition of basic datatype.  
 */

/*!< Type definition of frame id. */
typedef UINT32 FRAME_ID;  /*!< Frame id. */

/*!< Type definition of protocol id. */
typedef UINT32 PROTOCOL_ID;  /*!< Protocol id. */

/*!< Type definition of time stamp data. */
typedef struct {
    UINT32 tv_sec,  /*!< Second. */
           tv_usec; /*!< Microsecond. */
} TIME_STAMP;

/*!< The precision of angle (roll, pitch, yaw) in GPS data. */
#define	GPS_ANGLE_PRECISION (1e-8)

/*!< The precision of x, y, z in GPS data. */
#define GPS_POS_PRECISION (1e-2)

/*!< The length of car, in cm. */
#define CAR_LEN 360
#define CAR_WIDTH 200

/*!< Type definition of pose, configuration and speed data. */
typedef struct {
    FRAME_ID id; /*!< Frame id */
    TIME_STAMP time_stamp; /*!< Time stamp. */
    UINT16 err_no;    /*!< Error number. */
    COOR2  com_coord; /*!< Compositional coordinates, in cm. */
    COOR2  ins_coord; /*!< INS coordinates, in cm. */
    COOR1  z;         /*!< Relative height, in cm. */
    INT32  roll;      /*!< Precision as GPS_ANGLE_PRECISION, in 2^-8 radian. */
    INT32  pitch;     /*!< Precision as GPS_ANGLE_PRECISION, in 2^-8 radian. */
    INT32  yaw;       /*!< Precision as GPS_ANGLE_PRECISION, in 2^-8 radian. */
    SPD1D  spd;       /*!< Speed, in cm/sec. */
    ACC3D  acc;       /*!< Acceleration in x,y,z direction, in cm/(sec*sec).*/
	UINT32 checksum;  /*!< Checksum of above data. */
} POSE_INFO;

/*!< Type definition of speed info of both wheels. */
typedef struct {
    UINT16 	left;   /* left odo rounds. */
    UINT16	right;  /* right odo rounds. */
    UINT8 	revers; /* 1: reversing, 0: forwarding. */
} ODO_INFO;

/*!< Position in 2-D space. */
typedef struct {
    INT32 x;   /*!< Compostional coordinates, in cm. */
    INT32 y;   /*!< Compostional coordinates, in cm. */
    INT32 yaw; /*!< Precision as GPS_ANGLE_PRECISION, in 2^-8 radian. */
} POZITION;

/*!< Type definition of STATE. */
typedef struct {
    POSE_INFO pos; /*!< Postion data. */
    UINT32 status; /*!< Status. */
} STATE;

/*!< Data header. */ 
typedef struct {
    PROTOCOL_ID protocol; /*!< Protocol id. */
    FRAME_ID    id;       /*!< Frame id. */	
    TIME_STAMP  begin;    /*!< Begin time of unit processing. */
    TIME_STAMP  end;      /*!< End time of unit processing. */
} RBX_HEADER;


/*!
 *  (4) Type definition of grid map for 32/64Line Lidar. 
 */


/*!< One meter should be 
 *   2 ^ POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER cell's height.
 */
#define POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER	2

/*!< One meter should be 
 *   2 ^ POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR cell's width.
 */
#define POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR	2

/*!< One meter should be 
 *   2 ^ POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER_HD cell's height.
 */
#define POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER_HD	3

/*!< One meter should be for HD 
 *   2 ^ POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR_HD cell's width.
 */
#define POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR_HD	3

/*!< One cell's property should be 
 *   presented by  2^ POWER_BITS_FOR_NORMAL_CELL bits
 *   or 2^ POWER_BITS_FOR_SIMPLE_CELL bits.
 */
#define POWER_BITS_FOR_CELL	2

/*!< Property for cell. */
typedef enum {
    UNOCCUPIED = 0,      /*!< 00 00 Cell is transitable. */

    OCCUPIEDSTATIC = 1,  /*!< 00 01 Cell is occupied by static obstacle.  */
    OCCUPIEDDYNAMIC = 3, /*!< 00 11 Cell is occupied by dynamic obstacle.  */
    OCCUPIEDWATER = 5,   /*!< 01 01 Cell is occupied by water.  */
    OCCUPIEDDITCH = 7,   /*!< 01 11 Cell is occupied by ditch.  */

    UNKNOWN_OCCL = 13,   /*!< 11 01 Cell is unknown (occlusion). */
    UNKNOWN_NOTV = 15,   /*!< 11 11 Cell is unknown (no test value). */
} PROPERTY_OF_CELL;


/*!< 64Line Lidar: */
/*!< Range for 64Line. */
#define LEN_HOR_64	40 /*!< Horizontal length, in m. */ 
#define LEN_VER_64	80 /*!< Vertical length, in m. */
/*!< Grid map's height for 64Line. */
#define H_GRID_MAP_64 (LEN_VER_64 * \
                1 << ( POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER ))
/*!< Grid map's width for 64Line. */
#define W_GRID_MAP_64 (LEN_HOR_64 * (\
                1 << ( \
                        POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR + \
                        POWER_BITS_FOR_CELL ) \
                        ) / 8 )
/*!< Type definition of grid for 64Line. */
typedef UINT8 GRID_64[H_GRID_MAP_64][W_GRID_MAP_64];

/*!< HD Lidar: */
/*!< Range for HD Line. */
#define LEN_HOR_HD	30 /*!< Horizontal length, in m. */ 
#define LEN_VER_HD	30 /*!< Vertical length, in m. */
/*!< Grid map's height for 64Line. */
#define H_GRID_MAP_HD (LEN_VER_HD * \
                1 << ( POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER_HD ))
/*!< Grid map's width for 64Line. */
#define W_GRID_MAP_HD (LEN_HOR_HD * (\
                1 << ( \
                        POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR_HD + \
                        POWER_BITS_FOR_CELL ) \
                        ) / 8 )
/*!< Type definition of grid for HD Line. */
typedef UINT8 GRID_HD[H_GRID_MAP_HD][W_GRID_MAP_HD];


/*!< 32Line Lidar: */
/*!< Range for 32Line. */
#define LEN_HOR_32	20 /*!< Horizontal length, in m. */ 
#define LEN_VER_32	50 /*!< Vertical length, in m. */
/*!< Grid map's height for 32Line. */
#define H_GRID_MAP_32 (LEN_VER_32 * \
                1 << ( POWER_NUMBER_OF_CELL_FOR_ONE_METER_VER ))
/*!< Grid map's width for 32Line. */
#define W_GRID_MAP_32 (LEN_HOR_32 * (\
                1 << ( \
                        POWER_NUMBER_OF_CELL_FOR_ONE_METER_HOR + \
                        POWER_BITS_FOR_CELL ) \
                        ) / 8 )
/*!< Type definition of grid for 32Line. */
typedef UINT8 GRID_32[H_GRID_MAP_32][W_GRID_MAP_32];



/*!< Type definition of dynamic area in gridmap. */
typedef struct {
    QUAD mask; /*!< Obstacle in representation of rectangle mask,in cell. */
               /*!< coor2[0] <--> left-botton, anticlockwise */
    SPD2D speed;  /*!< Two-dimensional speed. */
    INT16 height; /*!< Height of obstacles, in cm. */
} DYN_OBSTACLE_AREA;

/*!< Type definition of static area in gridmap. */
typedef struct {
	UINT8 id;
    QUAD mask; /*!< Obstacle in representation of rectangle mask,in cell. */
} STA_OBSTACLE_AREA;

#define MAX_DYN_AREA 10
/*!< Type definition of dynamic areas in gridmap. */
typedef struct {
    DYN_OBSTACLE_AREA areas[MAX_DYN_AREA]; 
    UINT8 valid_area_num; 
} DYN_OBSTACLE_AREAS; 

#define MAX_STA_AREA 60
/*!< Type definition of static areas in gridmap. */
typedef struct {
    STA_OBSTACLE_AREA areas[MAX_STA_AREA]; 
    UINT8 valid_area_num; 
} STA_OBSTACLE_AREAS; 

/*!< Type definition of grid map for 64Line. */
typedef struct {
    GRID_64 grid;             /*!< Grid map cells. */
    DYN_OBSTACLE_AREAS d_areas; /*!< Dynamic areas. */
    STA_OBSTACLE_AREAS s_areas; /*!< Static areas. */
    COOR2_PX center;          /*!< Car center in gridmap. */
    COOR2 dir;                /*!< Car direction. */
} GRID_MAP_64; 

/*!< Type definition of grid map for HD line. */
typedef struct {
    GRID_HD grid;             /*!< Grid map cells. */
    COOR2_PX center;          /*!< Car center in gridmap. */
    COOR2 dir;                /*!< Car direction. */
} GRID_MAP_HD; 

/*!< Type definition of grid map for 32Line. */
typedef struct {
    GRID_32 grid;             /*!< Grid map cells. */
    DYN_OBSTACLE_AREAS areas; /*!< Dynamic areas. */
    COOR2_PX center;          /*!< Car center in gridmap. */
    COOR2 dir;                /*!< Car direction. */
} GRID_MAP_32; 


/*!
 * (5) Type definition of dynamic point object. 
 */

/*!< Type definition of single dynamic object. */
typedef struct {
    COOR2 position;	/*!< Signal center of dynamic object. */
    SPD2D speed;    /*!< Speed of signal center. */
} DYNAMIC_POINT_OBJ;

#define	MAX_OBJ_NUM 20

/*!< Type definition of dynamic object array. */
typedef DYNAMIC_POINT_OBJ DYNAMIC_POINT_OBJS[MAX_OBJ_NUM];


/*!
 * (6) Type definition of edge, road etc. 
 */

/*!< Number of 2D point in a edge. */
#define NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE 20 

/*!< Edge type. */
typedef enum {
    LT_UNCLASSIFIED  = 0, /*!< 000 00000: Unclassified line. */
    DASHED = 1,           /*!< 000 00001: Dashed line. */
    SOLID = 7,            /*!< 000 00111: Solid line. */
    DOUBLEDASHED = 9,     /*!< 000 01001: Double-dashed line. */
    DASHEDSOLID = 11,     /*!< 000 01011: Dashed-solid line. */
    SOLIDDASHED = 13,     /*!< 000 01101: Solid-dashed line. */
    DOUBLESOLID = 15,     /*!< 000 01111: Double-solid line. */
    NATURAL = 17,         /*!< 000 10001: Natural line. */
    RIVERBANK = 49,       /*!< 001 10001: line divided by river. */
    MUNTAINBANK = 81,     /*!< 010 10001: line divided by mountain. */
    DRYDITCHBANK = 113,   /*!< 011 10001: line divided by dry ditch. */
} LINE_TYPE;

/*!< Edge color. */
typedef enum {
    LC_UNKNOWN = 0,        /*!< UNKNOWN. */
    YELLOW = 1, WHITE = 2, /*!< Yellow and white line. */
} LINE_COLOR;

/*!< Type of horizontal road. */
typedef enum {
    DTH_UNKNOWN_DIR = 0,   /*!< 0000 : Unknown type. */
    HORIZONTAL_R_WING = 3, /*!< 0011 : Horizontal edge in left wing. */
    HORIZONTAL_L_WING = 5, /*!< 0101 : Horizontal edge in right wing. */
    HORIZONTAL_A_WING = 7  /*!< 0111 : Horizontal edge in both wings. */
} DIR_TYPE_HOR;

/*!< Type definition of edge line in 2D space. */
typedef COOR2_LC EDGE_LINE[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];

/*!< Type definition of edge line in image. */
typedef COOR2_PX EDGE_LINE_PX[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
/*!< Type definition of edge in 2D space. */
typedef struct {
    EDGE_LINE line;         /*!< Points in edge. */
    UINT8 valid_num_points; /*!< Valid numbers of points. */
    UINT8 line_type;        /*!< LINE_TYPE. */
    UINT8 line_color;       /*!< LINE_COLOR. */
    UINT8 dir_type_hor;     /*!< DIR_TYPE. */
    UINT8 fidelity; 	    /*!< Fidelity. */   
} EDGE;
/*!< Type definition of edge in image. */
typedef struct {
    EDGE_LINE_PX line;      /*!< Points in edge. */
    UINT8 valid_num_points; /*!< Valid numbers of points. */
    UINT8 line_type;        /*!< LINE_TYPE. */
    UINT8 line_color;       /*!< LINE_COLOR. */
    UINT8 dir_type_hor;     /*!< DIR_TYPE. */
    UINT8 fidelity; 	    /*!< Fidelity. */   
} EDGE_PX;

/*!< Number of edge in lane. */
#define NUM_OF_EDGE_IN_L_WING 3 
#define NUM_OF_EDGE_IN_R_WING 3

/*!< Type definition of road in 2D space. */
typedef struct {
    EDGE l_edges[NUM_OF_EDGE_IN_L_WING]; /*!< Edges in left wing. */
    EDGE r_edges[NUM_OF_EDGE_IN_R_WING]; /*!< Edges in right wing. */
    UINT8 valid_num_in_l_wing;           /*!< Valid number of edge. */ 
    UINT8 valid_num_in_r_wing;           /*!< Valid number of edge. */ 
} ROAD_LINES;

/*!< Type definition of natural boundary in 2D space. */
typedef struct {
    EDGE_LINE l_boundary; /*!< Boundary in left wing. */
    EDGE_LINE r_boundary; /*!< Boundary in right wing. */
    UINT8 l_fidelity;     /*!< Fidelity for left boundary. */   
    UINT8 r_fidelity;     /*!< Fidelity for right boundary. */   
} ROAD_NATURAL_BOUNDARY;

/*!< Type definition of natural boundary in image. */
typedef struct {
    EDGE_LINE_PX l_boundary; /*!< Boundary in left wing. */
    EDGE_LINE_PX r_boundary; /*!< Boundary in right wing. */
    UINT8 l_fidelity;        /*!< Fidelity for left boundary. */   
    UINT8 r_fidelity;        /*!< Fidelity for right boundary. */   
} ROAD_NATURAL_BOUNDARY_PX;

#define NUM_OF_EDGE_IN_HOR 6
#define NUM_OF_EDGE_IN_VER 6

/*!< Type definition of vertical road in image. */
typedef struct {
    EDGE_PX edges[NUM_OF_EDGE_IN_VER]; /*!< Edges in vertical road. */
    UINT8 valid_num_in_ver;            /*!< Valid number of edge. */ 
} ROAD_VER;

/*!< Type definition of horizontal road in image. */
typedef struct {
    EDGE_PX edges[NUM_OF_EDGE_IN_HOR]; /*!< Edges in horizontal road. */
    UINT8 valid_num_in_hor;            /*!< Valid number of edge. */ 
} ROAD_HOR;

/*!< Type definition of road opening in image. */
typedef struct {
    int valid;    /*!< Valid flag of road opening */
    QUAD opening; /*!< Opening in road */        
} ROAD_OPENING;

/*!< Number of questionable area in road. */
#define NUM_OF_QUESTIONABLE_AREA_IN_ROAD 16

/*!< Type definition of questionable area in image. */
typedef QUAD_PX QUESTIONABLE_AREAS_PX [NUM_OF_QUESTIONABLE_AREA_IN_ROAD];

/*!< Type definition of road natural width in 2D space. */
typedef struct {
    UINT16 width;   /*!< Width of natural road in 2D space. */
    UINT8 fidelity; /*!< Fidelity. */
} ROAD_NATURAL_WIDTH;

/*!
 * (7) Type definition of stopline and zebraline. 
 */

#define MAX_STOP_LINES_NUM 2

/*!< Type definition of stopline in 2D space. */
typedef struct {
    COOR2 start; /*!< Start point. */
    COOR2 end;   /*!< End point. */
} STOP_LINE;

/*!< Type definition of stoplines in 2D space. */
typedef struct 
{
    STOP_LINE stop_lines[MAX_STOP_LINES_NUM];
	UINT8 valid_line_num;
} STOP_LINES;

/*!< Type definition of stopline in image. */
typedef struct {
    COOR2_PX start; /*!< Start point. */
    COOR2_PX end;   /*!< End point. */ 
} STOP_LINE_PX;

/*!< Type definition of stoplines in image. */
typedef struct 
{
    STOP_LINE_PX stop_lines_px[MAX_STOP_LINES_NUM];
	UINT8 valid_line_num;
} STOP_LINES_PX;

#define MAX_ZEBRA_LINES_NUM 4

/*!< Type definition of zebraline in 2D space. */
typedef QUAD ZEBRA_LINE;

/*!< Type definition of zebralines in 2D space. */
typedef struct 
{
    ZEBRA_LINE zebra_lines[MAX_ZEBRA_LINES_NUM];
	UINT8 valid_line_num;
} ZEBRA_LINES;

/*!< Type definition of zebraline in image. */
typedef QUAD_PX ZEBRA_LINE_PX;

/*!< Type definition of zebralines in image. */
typedef struct 
{
    ZEBRA_LINE_PX zebra_lines_px[MAX_ZEBRA_LINES_NUM];
	UINT8 valid_line_num;
} ZEBRA_LINES_PX;

/*!< Type definition of cross in 2D space. */
typedef struct {
	
	UINT8 valid_num;	      /*!< For middle point. */ 
    QUAD cross_quad;          /*!< Quad of cross. */
    COOR2 entrance_point;     /*!< Entrance point. */
	COOR2 middle_point[10];   /*!< Middle point. */
    COOR2 exit_point;         /*!< Exit point. */
    UINT8 fidelity_quad;      /*!< Fidelity of quad. */
    UINT8 fidelity_entrance;  /*!< Fidelity of entrance. */
    UINT8 fidelity_exit;      /*!< Fidelity of exit. */
} CROSS;


/*!
 * (8) Type definition of traffic signs. 
 */

/*!< Max number of traffic signs. */
#define MAX_TRAFFIC_SIGNS 12

/*!< Type definition of traffic sign. */
typedef struct {
    UINT8 main;     /*!< Category. */
    UINT8 subc;     /*!< Sub-category. */
    UINT8 fidelity; /*!< Fidelity. */ 	   
    REKT_PX rekt;   /*!< Location in image. */
} TRAFFIC_SIGN;

/*!< Type definition of traffic signs. */
typedef struct {
    TRAFFIC_SIGN signs[MAX_TRAFFIC_SIGNS]; /*!< Signs. */
    UINT8 valid_num;                       /*!< Number of traffic signs.*/
} TRAFFIC_SIGNS;


/*!
 * (9) Type definition of traffic light. 
 */

/*!< Max number of traffic lights. */
#define MAX_TRAFFIC_LIGHTS 12

/*!< Type definition of passability. */
typedef enum {
    UNKNOWN_PASSABILITY = 0, /*!< 0000 : Unknown. */
    PASSABLE = 1,            /*!< 0001 : Passable. */
    PASSABLECOUNTDOWN = 3,   /*!< 0011 : Impassable. */
    IMPASSABLE = 5,          /*!< 0101 : Impassable. */
    IMPASSABLECOUNTDOWN = 7, /*!< 0111 : Impassable. */
} PASSABILITY;

/*!< Type definition of direktional passability. */
typedef struct {
    UINT8 passability; /*!< Passability of light. */
    UINT8 countdown;   /*!< Countdown value of light. */
} DIRECTION_PASSABILITY;

/*!< Type definition of four direktional passabilities. */
typedef struct { 
    DIRECTION_PASSABILITY forward; 
    DIRECTION_PASSABILITY backward; 
    DIRECTION_PASSABILITY left; 
    DIRECTION_PASSABILITY right; 
} DIRECTION_PASSABILITIES;

/*!< Type definition of traffic light. */
typedef struct {
    REKT_PX rekt;   /*!< Location in image. */
    UINT8 fidelity; /*!< Fidelity. */ 	   
} TRAFFIC_LIGHT;

/*!< Type definition of traffic lights. */
typedef struct {
    DIRECTION_PASSABILITIES passability;      /*!< Light for turn right. */
    TRAFFIC_LIGHT lights[MAX_TRAFFIC_LIGHTS]; /*!< Traffic lights .*/
    UINT8 valid_num;                          /*!< Number of traffic rights.*/
} TRAFFIC_LIGHTS;

/*!
 * (10) Type definition of traffic. 
 */

/*!< Type definition of direction. */
typedef enum {
    FORWARD,
    LEFT,
    RIGHT,
    TURNBACK,
} DIRECTION;

/*!< Type definition of traffic. */
typedef struct {
    UINT8 direction;
    DIRECTION_PASSABILITY passability;
    UINT16 speed_limit;
} TRAFFIC;


/*!
 * (11) Type definition of LMS.
 */
#define POINT_NUM_OF_CURB 20

/*!< Type definition of curblines in 2D space. */
typedef struct {
    COOR2 l_edges[POINT_NUM_OF_CURB]; /*!< Curb in left wing. */
    COOR2 r_edges[POINT_NUM_OF_CURB]; /*!< Curb in right wing. */
    UINT8 valid_l;           			/*!< Valid number of curb. */ 
    UINT8 valid_r;           			/*!< Valid number of curb. */ 
} CURB_LINES;

#define LMS_BACK_RANGE_LENGTH    361     /*!< define max length of back lms lidar. */
#define LMS_FORWARD_RANGE_LENGTH 541     /*!< define max length of forward LMS lidar */

typedef struct {
	COOR2	LmsLocationShift[3];	/*!< back-left-right lms distance shift relative to origin */
	INT16	LmsMountAngleShift[3];	/*!< back-left-right lms angle shift relative to Y axial */
	UINT16	Flag[3];				/*!< if back-left-right lms is valid. 
											0x01:OK, 0x00:lost and filled by last data */
	UINT16	B_length;				/*!< back lms length */
	UINT16	range_back[LMS_BACK_RANGE_LENGTH];		/*!< back lms lidar data */
	UINT16	LR_length;				/*!< back lms length */
	UINT16	range_left[LMS_FORWARD_RANGE_LENGTH];	/*!< left lms lidar data */
	UINT16	range_right[LMS_FORWARD_RANGE_LENGTH];	/*!< right lms lidar data */
	CURB_LINES	curb;				/*!< Curb lines for lms data. */
}LMS;


/*!
 * (12) Type definition for GEO.
 */
#define MAX_GUIDE_POINT_NUM 50
typedef struct {
	UINT8	valid;
	INT32	delta_yaw;					  /*!< delta yaw of the guide lines. */
	INT32	delta_dis;					  /*!< delta distance between the points. */
    COOR2	gps[MAX_GUIDE_POINT_NUM]; 	  /*!< guide points. */
}GUIDE_LINES;

/*!
 * (13) Type definition for PL OBJECT.
 */
typedef struct {
	UINT8	valid;
	INT32	x;					  /*!< Longitude of car. */
	INT32	y;					  /*!< Latitude of car. */
	INT32	z;					  	  /*!< Height of car. */
	INT32	yaw;				  /*!< Orientation of car, angle from 0 to 360, 
												the north is 0, and clockwize.*/	
    SPD1D  	line_spd;       			  /*!< Line speed, in cm/sec. */
    SPD1D  	angular_spd;       			  /*!< Angular speed, TODO. */
}OBJECT_INFO;

#pragma pack(pop)
#endif
