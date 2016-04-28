#ifndef __MODULE_RBX4CFG_H__
#define __MODULE_RBX4CFG_H__

#include "rbx4err.h"
#include "rbx4ast.h"
#include "rbx4head.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push)
#pragma pack(1)

#define BRIDGE_BEGIN    "<bridge>"
#define BRIDGE_END      "</bridge>"
#define HOST_BEGIN		"<host>"
#define HOST_END		"</host>"
#define DATA_BEGIN		"<data>"
#define DATA_END		"</data>"
#define APP_BEGIN       "<application>"
#define APP_END         "</application>"

#define FRAME_NUM       (4)

#define  for_each_data(index)\
    for (index = 0; index < get_data_num( ); index++)

#define  for_each_agent(index)\
    for (index = 0; index < get_agent_num( ); index++)

#define  CFG_DATA(index)    (g_config.data[index])
#define  CFG_APP(index)     (g_config.app[index])

typedef struct {
    uint16_t data_no;
    uint16_t frame_num;
    uint32_t data_size;
    char	 data_name[DATA_NAME_SIZE];
    char	 app_name[AGENT_NAME_SIZE];
} cfg_data_t;

typedef struct {
    char	 app_name[AGENT_NAME_SIZE];
} cfg_app_t;

typedef struct {
    int8_t		role;
    uint16_t	host_no;
    uint16_t	app_num;
    uint16_t	data_num;
    uint32_t	local_ip;
    uint32_t	remote_ip;
    cfg_app_t  *app;
    cfg_data_t *data;
} config_t;

extern config_t g_config;

static inline uint32_t 
get_remote_ip(void) 
{
    return g_config.remote_ip;
} 

static inline uint32_t 
get_local_ip(void) 
{
    return g_config.local_ip;
}

static inline int 
get_host_no(void) 
{
    return g_config.host_no;
}

static inline int 
get_data_num(void) 
{
    return g_config.data_num;
}

static inline int 
get_agent_num(void) 
{
    if (OPTION_AGENT() == TRUE) {
        return 0;
	}
    return g_config.app_num;
}

static inline int 
IS_AGENT(int agent) 
{
    return (agent < get_agent_num() && agent >= 0);
}

static inline char *
get_agent_name(int agent_no) 
{
    if (!IS_AGENT(agent_no)) {
        return NULL;
	}
    return CFG_APP(agent_no).app_name;
}

static inline int 
get_agent_no(const char *name) 
{
    int index;

    for_each_agent(index) {
        if (!strcmp(CFG_APP(index).app_name, name))
            return index;
    }
    return RET_ERROR;
}

static inline int 
IS_DATA(int data_no) 
{
    return (data_no >= 0 && data_no < get_data_num());
}

static inline int 
get_data_size(int data_no) 
{
    if(IS_DATA(data_no) == FALSE) {
        return RET_ERROR;
	}
    return CFG_DATA(data_no).data_size;
}

static inline int 
get_data_no(const char *name) 
{
    int index;
    for_each_data(index) 
	{
        if (!strcmp(CFG_DATA(index).data_name, name))
            return index;
    }
    return RET_ERROR;
}

static inline int 
get_frame_size(int data_no) 
{
    return get_data_size(data_no);
}

static inline int 
get_max_data_size(void) 
{
    int index;
    uint32_t max = 0;

    for_each_data(index) 
	{
        if (CFG_DATA(index).data_size > max)
            max = CFG_DATA(index).data_size;
    }
    return max;
}

static inline int 
get_frame_num(int data_no) 
{
    return CFG_DATA(data_no).frame_num;
}

static inline int
IS_GPS (int data_no)
{
	int gps1 = get_data_no(POSE_INFO1_DATA_NAME);
	int gps2 = get_data_no(POSE_INFO2_DATA_NAME);
	return data_no == gps1 || data_no == gps2;
}

static inline int 
GENERATED_BY_LOCAL(int data) 
{
    char *agent_name = CFG_DATA(data).app_name;
    return get_agent_no(agent_name) != RET_ERROR;
}

static inline int
IS_UROBIX(void)
{
	return g_config.role == ROLE_UROBIX;
}

static inline int 
IS_LOOPBACK(void) 
{
    return g_config.role == ROLE_LOOPBACK;
}

static inline int
IS_SAMPLER(void)
{
	return g_config.role == ROLE_SAMPLER;
}

static inline int
IS_CONTROLLER(void)
{
	return g_config.role == ROLE_CONTROLLER;
}

static inline int
IS_INSPECTOR(void)
{
	return g_config.role == ROLE_INSPECTOR;
}

static inline enum PORT_SET 
get_port_set(void)
{
	enum PORT_SET set = DEFAULT_SET;
	
	switch(g_config.role) {
	case ROLE_UROBIX:
		set = ROBIX_SET;
		break;
	case ROLE_LOOPBACK:
		set = LOOPBACK_SET;
		break;
	case ROLE_SAMPLER:
		set = SAMPLER_SET;
		break;
	case ROLE_CONTROLLER:
		set = CONTROLLER_SET;
		break;
	case ROLE_INSPECTOR:
		set = INSPECTOR_SET;
		break;
	default:
		set = DEFAULT_SET;
		break;
	}
	return set;
}

static inline int
get_remote_port(enum PORT_TYPE type)
{
	enum PORT_SET set = get_port_set(); 
	return __get_remote_port(set, type);
}

static inline int
get_local_port(enum PORT_TYPE type)
{
	enum PORT_SET set = get_port_set(); 
	return __get_local_port(set, type);
}

extern int init_sys_cfg(char *file, char *type, int role);
extern void exit_sys_cfg();

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif
