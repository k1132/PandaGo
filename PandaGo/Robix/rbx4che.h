#ifndef __MODULE_CACHE__
#define __MODULE_CACHE__

#include "rbx4cfg.h"
#include "rbx4head.h"
#include "rbx4lock.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    RW_READ = 0,
    RW_WRITE = 1,
    POLICY_FPS_LAST = 0,	/* Policy : Send LAST frame ... */
    POLICY_FPS_NEW = 1,		/* Policy : Send NEW frame ... */
    POLICY_FPS_FIFO,		/* Policy : First In First Out ... */
    POLICY_FPS_LIFO,		/* Policy : Last In First Out ... */
    POLICY_NUM,
};

typedef struct {
    uint8_t  last_frame_pos;	/* Postion of last frame ... */
    uint8_t  next_write_pos;	/* Next postion to write ... */
    uaddr_t *data_frame_pos;	/* Array of data frames ... */
    rwlock_t data_rwlock;	    /* Read-Write lock ... */
} cache_data_t;

typedef struct {
    cache_data_t *data;
    rwlock_ops_t *syn_ops;
} cache_cfg_t;

static inline int 
IS_POLICY(int policy) 
{
    return (policy < POLICY_NUM && policy >= 0);
} 

static inline int 
IS_READ_OR_WRITE(int read_write) 
{
    return (read_write == RW_WRITE || read_write == RW_READ);
}

extern int init_sys_cache(char *path, char *mode, int role);
extern void exit_sys_cache(void);
extern int cache_read(int data, void *buf, int size, int policy);
extern int cache_write(int data, void *buf, int size);

#ifdef __cplusplus
}
#endif

#endif
