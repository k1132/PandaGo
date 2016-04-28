#ifndef __MODULE_Kwreply_H__
#define __MODULE_Kwreply_H__

#include "rbx4list.h"
#include "rbx4lock.h"
#include "rbx4pack.h"

#include <time.h>
#include <sys/time.h>
#include <sys/uio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define  PACK_SLOT_NUM      (64)
#define  MAX_TRY_TIMES      (5)
#define  FIRST_TIME         (1)
#define  FIRST_WAIT_TIME    (MILLION / 10)	/* 100 ms */

/* Struct used to Wait for REPLY ...*/
typedef struct {
    int pack_id;
    int pack_len;
    void *pack;
    int try_times;
    struct timeval end_time;
    struct list_head time_list;
    struct list_head pack_list;
	void (*call_back) (void *);
	int  (*send_func) (struct iovec *);
} reply_t;

typedef struct {
    struct list_head pack_head;
} pack_slot_t;

typedef struct {
    pack_slot_t slot[PACK_SLOT_NUM];
} pack_table_t;

/* Translate packet id to index ...*/
static int inline 
PID_TO_INDEX(int pack_id) 
{
    return MOD(pack_id, PACK_SLOT_NUM);
} 

extern int init_sys_reply(void);
extern void exit_sys_reply(void);

extern int register_sreq_func(int (*func) (struct iovec *));
extern int register_sdata_func(int (*func) (struct iovec *));
extern int handle_reply(void *pack, void (*func) (void *), int);
extern int recv_reply(int pack_id);

#ifdef __cplusplus
}
#endif

#endif
