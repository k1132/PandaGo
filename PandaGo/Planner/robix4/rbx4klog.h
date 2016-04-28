#ifndef __ROBIX4_KLOG_H__
#define __ROBIX4_KLOG_H__

#include "rbx4cfg.h"

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KLOG_PATH_FIX   "/var/log/alv/"
#define KLOG_TYPE       "w"
#define KLOG_BUF_SIZE   (128 * __KB)
#define KLOG_PN_SIZE    40

typedef struct {
    FILE *fp;
    void *buf_addr;	/* Log buffer ... */
    int   buf_size;
} klog_buf_t;

typedef struct {
    void       *log_addr;
    int         log_size;
    klog_buf_t *log_bufs;
} klog_t;

extern int init_sys_klog(void);
extern void exit_sys_klog(void);
extern int PRINT(int agent_no, const char *fmt, ...);

#ifdef __cplusplus
}
#endif
#endif
