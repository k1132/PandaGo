#ifndef __ROBIX4_INIT_H__
#define __ROBIX4_INIT_H__

#include "rbx4api.h"
#include "rbx4cfg.h"
#include "rbx4log.h"
#include "rbx4thd.h"
#include "rbx4thd_common.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void wait_for_exiting(void);
extern int init_sys_robix(write_func_t, int);
extern void exit_sys_robix(void);

#ifdef __cplusplus
}
#endif
#endif
