#ifndef __ROBIX4_REQUEST_H__
#define __ROBIX4_REQUEST_H__

#include <sys/uio.h>

#include "rbx4cfg.h"
#include "rbx4lock.h"
#include "rbx4list.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ADD_REQ,
    DEL_REQ,
    MOD_REQ,
    NONE_REQ,
} request_type_t;

typedef struct {
    int hz;
    int policy;
    int data_no;
    int agent_no;
    int state;
    struct list_head data_list;
    struct list_head agent_list;
} request_t;

typedef struct {
    int agent_num;
    pthread_mutex_t data_lock;
    struct list_head data_head;
} data_request_t;

typedef struct {
    int request_num;
    int reach_num;
    struct list_head agent_head;
} agent_request_t;

static inline int 
IS_HZ(char hz) 
{
    return (hz >= MIN_HZ_VALUE && hz <= MAX_HZ_VALUE);
} 

extern int init_sys_sub(void);
extern void exit_sys_sub(void);

extern void update_request_state(int data_no);

extern int subscribe_by_local(int data_no);

extern struct iovec add_subscribe(int agent, int data, int p, int hz);
extern struct iovec cancel_subscribe(int agent, int data);
extern struct iovec modify_subscribe(int agent, int data, int p, int hz);

extern int read_data(int agent, int data, void *buf, int bufsize);

#ifdef __cplusplus
}
#endif

#endif
