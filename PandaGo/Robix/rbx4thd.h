#ifndef __ROBIX4_THREAD_H__
#define __ROBIX4_THREAD_H__

#include "rbx4rlv.h"
#include "rbx4log.h"
#include "rbx4cfg.h"
#include "rbx4list.h"
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*start_func_t) (void *);
typedef void (*exit_func_t) (void *);

typedef enum {
    THREAD_STOPPED,
    THREAD_RUNNING,
    THREAD_EXITING,
    THREAD_EXITED,
    STATE_NUM,
} thread_state_t;

struct sub_pthread {
    pthread_t tid;
    struct list_head list;
};

struct pthread_task {
    pthread_t tid;
    char name[AGENT_NAME_SIZE];	/* Name of agent ... */
    int thread_state;			/* Running state of pthread ... */
    void *exit_data;
    void *start_data;
    struct list_head head;		/* Link all subpthreads ... */
    exit_func_t exit_func;
    start_func_t start_func;
    struct pthread_task *next;
};

extern struct pthread_task *register_pthread(char * agent_name, 
        start_func_t, void *, 
        exit_func_t, void *);
extern int register_subpthread(const char *agent_name);

extern int run_all_pthreads(void);
extern void kill_all_pthreads(void);

extern int will_pthread_exit(void);
extern int pthread_exit_by_itself(void);
extern int notify_pthread(int agent, void *);
extern int agent_no_of_pthread(void);

extern int init_sys_pthread(char *path, char *mode, int role);
extern void exit_sys_pthread(void);

#ifdef __cplusplus
}
#endif
#endif
