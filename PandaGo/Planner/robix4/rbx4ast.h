#ifndef __MODULE_KAST__
#define __MODULE_KAST__

#include "rbx4err.h"
#include "rbx4head.h"
#include "protocols/protocol_head.h"
#include "protocols/protocol_gps.h"

#ifdef __cplusplus
extern "C" {
#endif


extern int (*mv_n_start_func) (void *);
extern void (*mv_n_exit_func) (void *);

extern int (*mv_f_start_func) (void *);
extern void (*mv_f_exit_func) (void *);

extern int (*lidar_start_func) (void *);
extern void (*lidar_exit_func) (void *);

extern int (*lidar32_start_func) (void *);
extern void (*lidar32_exit_func) (void *);

extern int (*flir_start_func) (void *);
extern void (*flir_exit_func) (void *);

extern int (*fmv_n_start_func) (void *);
extern void (*fmv_n_exit_func) (void *);

extern int (*fu_start_func) (void *);
extern void (*fu_exit_func) (void *);

extern int (*pl_start_func) (void *);
extern void (*pl_exit_func) (void *);

extern int (*rsn_start_func) (void *);
extern void (*rsn_exit_func) (void *);

extern int (*estop_start_func) (void *);
extern void (*estop_exit_func) (void *);

extern int (*mwr_start_func) (void *);
extern void (*mwr_exit_func) (void *);

extern int (*ullc_start_func) (void *);
extern void (*ullc_exit_func) (void *);

extern int (*urlc_start_func) (void *);
extern void (*urlc_exit_func) (void *);

extern int (*fmv_f_start_func) (void *);
extern void (*fmv_f_exit_func) (void *);

extern int (*lllc_start_func) (void *);
extern void (*lllc_exit_func) (void *);

extern int (*lrlc_start_func) (void *);
extern void (*lrlc_exit_func) (void *);

extern int (*bed_start_func) (void *);
extern void (*bed_exit_func) (void *);

extern int (*tlsr_start_func) (void *);
extern void (*tlsr_exit_func) (void *);

extern int (*inspector_start_func) (void *);
extern void (*inspector_exit_func) (void *);

extern int (*gps_start_func) (void *);
extern void (*gps_exit_func) (void *);

extern int (*gp_start_func) (void *);
extern void (*gp_exit_func) (void *);

extern int (*geo_start_func) (void *);
extern void (*geo_exit_func) (void *);

extern int (*sd_start_func) (void *);
extern void (*sd_exit_func) (void *);

extern int (*prm_start_func) (void *);
extern void (*prm_exit_func) (void *);

#define S1E_8   1e-8
#define S1E_7   1e-7
#define S1E_3   1e-3
#define S1E_2   0.01
#define SECONDS_PER_WEEK 604800
#define GPS_WEEK_OFFSET 345600


#define CAC_MODULE              cac
#define RNPL_MODULE             rnpl
#define FLIR_MODULE			    flir
#define FU_MODULE				fu
#define MV_N_MODULE			    mv_n
#define MV_F_MODULE			    mv_f
#define PL_MODULE				pl
#define LIDAR_MODULE			lidar
#define LIDAR32_MODULE			lidar32
#define RSN_MODULE			    rsn
#define	BED_MODULE			    bed
#define INSPECTOR_MODULE		inspector
#define FMV_N_MODULE            fmv_n
#define FMV_F_MODULE            fmv_f
#define TLSR_MODULE			    tlsr
#define MWR_MODULE			    mwr
#define ULLC_MODULE			    ullc
#define URLC_MODULE			    urlc
#define LLLC_MODULE			    lllc
#define LRLC_MODULE			    lrlc
#define ESTOP_MODULE			estop
#define GPS_MODULE			    gps
#define GP_MODULE			    gp
#define GEO_MODULE			    geo
#define SD_MODULE			    sd
#define PRM_MODULE			    prm

/* Agent names ...*/
#define CAC_NAME            "cac"
#define RNPL_NAME           "rnpl"
#define FLIR_NAME			"flir"
#define FU_NAME				"fu"
#define MV_N_NAME			"mv_n"
#define MV_F_NAME			"mv_f"
#define PL_NAME				"pl"
#define LIDAR_NAME			"lidar"
#define LIDAR32_NAME		"lidar32"
#define RSN_NAME			"rsn"
#define	BED_NAME			"bed"
#define INSPECTOR_NAME		"inspector"
#define FMV_N_NAME          "fmv_n"
#define FMV_F_NAME          "fmv_f"
#define TLSR_NAME			"tlsr"
#define MWR_NAME			"mwr"
#define ULLC_NAME			"ullc"
#define URLC_NAME			"urlc"
#define LLLC_NAME			"lllc"
#define LRLC_NAME			"lrlc"
#define ESTOP_NAME			"estop"
#define GPS_NAME			"gps"
#define GP_NAME			    "gp"
#define GEO_NAME			"geo"
#define SD_NAME			    "sd"
#define LMS_NAME			"lms"
#define PRM_NAME			"prm"

/* Data names ...*/
#define FLIR_FU_DATA_NAME          			  "flir_fu"
#define FLIR_CAC_DATA_NAME          		  "flir_cac"
#define LIDAR64_FU_DATA_NAME          		  "lidar64_fu"
#define LIDAR32_FU_DATA_NAME          		  "lidar32_fu"
#define	LIDAR_FU_ROAD_DATA_NAME          	  "lidar_fu_road"
#define LIDAR_CAC_ROAD_DATA_NAME              "lidar_cac_road"
#define MV_N_FU_DATA_NAME          			  "mv_n_fu"
#define MV_N_CAC_DATA_NAME          		  "mv_n_cac"
#define MV_N_FU_STOPLINE_DATA_NAME            "mv_n_fu_stopline"
#define MV_N_CAC_STOPLINE_DATA_NAME           "mv_n_cac_stopline"
#define MV_N_FU_ZEBRALINE_DATA_NAME           "mv_n_fu_zebraline"
#define MV_N_CAC_ZEBRALINE_DATA_NAME          "mv_n_cac_zebraline"
#define MV_F_FU_DATA_NAME          			  "mv_f_fu"
#define MV_F_CAC_DATA_NAME          		  "mv_f_cac"
#define MV_F_FU_STOPLINE_DATA_NAME            "mv_f_fu_stopline"
#define MV_F_CAC_STOPLINE_DATA_NAME           "mv_f_cac_stopline"
#define MV_F_FU_ZEBRALINE_DATA_NAME           "mv_f_fu_zebraline"
#define MV_F_CAC_ZEBRALINE_DATA_NAME          "mv_f_cac_zebraline"
#define FMV_N_FU_DATA_NAME          		  "fmv_n_fu"
#define FMV_N_CAC_DATA_NAME          		  "fmv_n_cac"
#define FMV_N_FU_STOPLINE_DATA_NAME           "fmv_n_fu_stopline"
#define FMV_N_CAC_STOPLINE_DATA_NAME          "fmv_n_cac_stopline"
#define FMV_N_FU_ZEBRALINE_DATA_NAME          "fmv_n_fu_zebraline"
#define FMV_N_CAC_ZEBRALINE_DATA_NAME         "fmv_n_cac_zebraline"
#define FMV_F_FU_DATA_NAME          		  "fmv_f_fu"
#define FMV_F_CAC_DATA_NAME          		  "fmv_f_cac"
#define FMV_F_FU_STOPLINE_DATA_NAME           "fmv_f_fu_stopline"
#define FMV_F_CAC_STOPLINE_DATA_NAME          "fmv_f_cac_stopline"
#define FMV_F_FU_ZEBRALINE_DATA_NAME          "fmv_f_fu_zebraline"
#define FMV_F_CAC_ZEBRALINE_DATA_NAME         "fmv_f_cac_zebraline"
#define FU_PL_DATA_NAME          			  "fu_pl"
#define FU_INSPECTOR_DATA_NAME          	  "fu_inspector"
#define BED_FU_DATA_NAME          	          "bed_fu"
#define PL_INSPECTOR_DATA_NAME                "pl_inspector"
#define TSR_FU_DATA_NAME          	          "tsr_fu"
#define TSR_CAC_DATA_NAME          	          "tsr_cac"
#define TLR_FU_DATA_NAME          	          "tlr_fu"
#define TLR_CAC_DATA_NAME          	          "tlr_cac"
#define MWR_FU_DATA_NAME          	          "mwr_fu"
#define MWR_LIDAR_DATA_NAME          	      "mwr_lidar"
#define URLC_FU_DATA_NAME          	          "urlc_fu"
#define ULLC_FU_DATA_NAME          	          "ullc_fu"
#define URLC_CAC_DATA_NAME          	      "urlc_cac"
#define ULLC_CAC_DATA_NAME          	      "ullc_cac"
#define URLC_FU_STOPLINE_DATA_NAME            "urlc_fu_stopline"
#define ULLC_FU_STOPLINE_DATA_NAME            "ullc_fu_stopline"
#define LRLC_FU_STOPLINE_DATA_NAME            "lrlc_fu_stopline"
#define LLLC_FU_STOPLINE_DATA_NAME            "lllc_fu_stopline"
#define URLC_FU_ZEBRALINE_DATA_NAME           "urlc_fu_zebraline"
#define ULLC_FU_ZEBRALINE_DATA_NAME           "ullc_fu_zebraline"
#define LRLC_FU_ZEBRALINE_DATA_NAME           "lrlc_fu_zebraline"
#define LLLC_FU_ZEBRALINE_DATA_NAME           "lllc_fu_zebraline"
#define URLC_CAC_STOPLINE_DATA_NAME           "urlc_cac_stopline"
#define ULLC_CAC_STOPLINE_DATA_NAME           "ullc_cac_stopline"
#define LRLC_CAC_STOPLINE_DATA_NAME           "lrlc_cac_stopline"
#define LLLC_CAC_STOPLINE_DATA_NAME           "lllc_cac_stopline"
#define URLC_CAC_ZEBRALINE_DATA_NAME          "urlc_cac_zebraline"
#define ULLC_CAC_ZEBRALINE_DATA_NAME          "ullc_cac_zebraline"
#define LRLC_CAC_ZEBRALINE_DATA_NAME          "lrlc_cac_zebraline"
#define LLLC_CAC_ZEBRALINE_DATA_NAME          "lllc_cac_zebraline"
#define EVENT_DATA_NAME                       "event"
#define STATUS_DATA_NAME                      "status"
#define POSE_INFO1_DATA_NAME				  "pose_info1"
#define POSE_INFO2_DATA_NAME                  "pose_info2"
#define RNPL_PL_DATA_NAME                     "rnpl_pl"
#define RNPL_CAC_DATA_NAME                    "rnpl_cac"
#define CAC_PL_DATA_NAME                      "cac_pl"
#define GP_FU_DATA_NAME                       "gp_fu"
#define GP_PL_DATA_NAME                       "gp_pl"
#define ODO_INFO_DATA_NAME                    "odo_info"
#define LMS_FU_DATA_NAME                      "lms_fu"
#define LMS_PL_DATA_NAME                      "lms_pl"
#define RECOVERY_POINT_NAME                   "recovery_point"
#define GEO_FU_DATA_NAME                      "geo_fu"
#define GEO_PL_DATA_NAME                      "geo_pl"
#define PRM_PL_DATA_NAME                      "prm_pl"
#define PL_PRM_DATA_NAME                      "pl_prm"
#define PRM_SEND_PACKET_NAME                  "prm_send"
#define PRM_RECV_PACKET_NAME                  "prm_recv"

#define __REGISTER_MODULE_FUNC(module, start_func, exit_func)\
        int (* module ## _start_func)(void *) = start_func;\
        void (* module ## _exit_func)(void *) = exit_func;\

#define REGISTER_MODULE_FUNC(module, start_func, exit_func)\
        __REGISTER_MODULE_FUNC(module, start_func, exit_func)

#pragma pack(push)
#pragma pack(1)

typedef struct {
    char module_name[MODULE_NAME_SIZE];
    int (**start_func) (void *);
    void *start_data;
    void (**exit_func) (void *);
    void *exit_data;
} module_info_t;

typedef struct {
    int hz;
} option_hz_t;

/* URBX Subscribe information ...*/
typedef struct {
    char *s_data;	/* Data Name ... */
    int s_num;	    /* Number of subscribes ... */
} option_su_t;

typedef struct {
    int32_t  opt_a;
    int32_t  opt_s;
    int32_t  opt_h;
    option_hz_t hz;
    option_su_t su;
} option_t;

/*
typedef struct
{
	unsigned char 	head;
	unsigned char 	flag;
	unsigned char 	mode;
	unsigned int	timer;
	int		    position[3];
	int 		velocity[3];
	short 		attitude[3];
	short 		gyro[3];
	short 		acclerate[3];
	unsigned char	sum;
}GPS_RECV_DATA;
*/

typedef int (*write_func_t) (int data, void *buf, int size);

extern option_t g_option;
extern write_func_t write_block;

extern module_info_t *get_module_info(char *name);
extern int get_struct_size(char *data_name);
extern int resolve_option(int argc, char *argv[]);
extern char *get_rbx_status_name(int status);
extern char *get_car_status_name(int status);
extern char *get_rbx_event_name(int event);
extern char *get_car_event_name(int event);
extern void convert_gps(const GPS_RECV_DATA *, POSE_INFO *);

static inline int 
OPTION_HZ(option_hz_t *opt_hz) 
{
    if (opt_hz != NULL) {
        *opt_hz = g_option.hz;
    }
    return g_option.opt_h;
}

static inline int 
OPTION_SUBSCRIBE(option_su_t *opt_su) 
{
    if (opt_su != NULL) {
        *opt_su = g_option.su;
    }
    return g_option.opt_s;
}

static inline int 
OPTION_AGENT(void) 
{
    return g_option.opt_a;
}

static inline char *
GET_S_DATA_NAME(int index) 
{
    return g_option.su.s_data + index * DATA_NAME_SIZE;
}

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif
