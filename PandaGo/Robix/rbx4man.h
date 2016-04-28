#ifndef __MODULE_MANAGE_H__
#define __MODULE_MANAGE_H__

#include "rbx4lock.h"
#include "rbx4cfg.h"
#include "rbx4eve.h"
#include <sys/uio.h>

#define ANY_AGENT               (-1)
#define GET_NOTIFY_OPS          (system_info.notify_ops)
#define SET_NOTIFY_OPS(ops)     (GET_NOTIFY_OPS = ops)
#define SYSTEM_READY_FUNC       (GET_NOTIFY_OPS.system_ready)
#define SYSTEM_EXIT_FUNC        (GET_NOTIFY_OPS.system_exiting)
#define SYSTEM_PAUSE_FUNC       (GET_NOTIFY_OPS.system_pause)
#define SYSTEM_CONTINUE_FUNC    (GET_NOTIFY_OPS.system_continue)
#define NOTIFY_CAR_EVENT_FUNC   (GET_NOTIFY_OPS.notify_car_event)
#define NOTIFY_PTHREAD_FUNC		(GET_NOTIFY_OPS.notify_pthread)
#define SET_SYSTEM_READY(func)  (SYSTEM_READY_FUNC = func)
#define SET_SYSTEM_EXIT(func)   (SYSTEM_EXIT_FUNC = func)

#define GET_URBX_STATUS         (system_info.urbx_stat)
#define SET_URBX_STATUS(stat)   (GET_URBX_STATUS = stat)
#define GET_CAR_STATUS          (system_info.car_stat)
#define SET_CAR_STATUS(stat)    (GET_CAR_STATUS = stat)
#define GET_NET_STATUS          (system_info.conn_stat)
#define SET_NET_STATUS(stat)    (GET_NET_STATUS = stat)

typedef uint32_t agent_status_t;

typedef struct {
	int (*system_ready) (void);
	int (*system_pause) (void);
	int (*system_continue) (void);
	int (*system_exiting) (void);
	int (*notify_car_event) (int);
	int (*notify_pthread) (int, void *);
} notify_ops_t;

typedef struct {
	pthread_mutex_t mutex_lock;
	agent_status_t  conn_stat;   /* Connection state of network .*/
	agent_status_t *agent_stat;	 /* Record which agent's ready .*/
	agent_status_t  urbx_stat;	 /* Current status of urobix .*/
	agent_status_t  car_stat;	 /* Current status of smart car .*/
	notify_ops_t	notify_ops;
	wait_conds_t   *wait_conds;
} system_info_t;

extern system_info_t system_info;

static inline int
SYS_IN_PAUSING_STATUS(void)
{
	return (GET_URBX_STATUS == _S_PAUSE);
}

extern int init_sys_manage(void);
extern void exit_sys_manage(void);
extern int report_event(int agent, int event);
extern int agent_report_ready(int agent);
extern int handle_status(int car_status, int rbx_status);

#endif
