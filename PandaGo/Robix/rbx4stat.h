#ifndef __MODULE_STATISTIC_H__
#define __MODULE_STATISTIC_H__

#include <time.h>
#include <sys/time.h>

#include "rbx4cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

struct recv_info {
    int				pack_recv;
    int				pack_lost;
    int				pack_error;
    int				pack_failure;
    int				last_packid;
    double			rtime_avg;	/* Average value of recv time */
    double			rtime_var;	/* Variance value of recv time */
    struct timespec last_rtime;
};

static inline double 
timespec_to_msec(struct timespec prec, struct timespec cur) 
{
    double sec, msec, nsec;

    sec = cur.tv_sec - prec.tv_sec;
    nsec = cur.tv_nsec - prec.tv_nsec;
    msec = (double) (sec * BILLION + nsec) / MILLION;
    return msec;
} 

extern int init_sys_stat(void);
extern void exit_sys_stat(void);
extern int update_recv_info(int agent);

#ifdef __cplusplus
}
#endif

#endif
