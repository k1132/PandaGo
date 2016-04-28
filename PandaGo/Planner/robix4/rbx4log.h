#ifndef __ROBIX4_LOG_H__
#define __ROBIX4_LOG_H__

#include <stdio.h>
#include <fcntl.h>

#include "rbx4err.h"
#include "rbx4head.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_PATH    "/var/log/alv/"
#define LOG_MODE    0644
#define LOG_FLAGS   (O_RDWR | O_CREAT | O_TRUNC)

extern int init_robix_log(char *name);

extern void write_robix_log(const char *fmt, ...);

#ifdef __cplusplus
}
#endif
#endif
