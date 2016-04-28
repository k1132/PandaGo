#ifndef __MODULE_KFRAGMENT_H__
#define __MODULE_KFRAGMENT_H__

#define  HASH_SIZE          (64)
#define  TIME_OUT_INTERVAL  (30 * 1000)	/* 30s ... */

#include "rbx4err.h"
#include "rbx4rlv.h"

#include "rbx4head.h"
#include "rbx4list.h"
#include "rbx4lock.h"

#include <time.h>
#include <sys/time.h>
#include <sys/uio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    struct list_head list;
    void *pack;
    int frag_id;
    int frag_len;	/* Fragment size, include packet header ... */
} fragment_t;

typedef struct {
    struct list_head collide_list;
    pthread_mutex_t lock;
    int pack_len;
    int recv_len;
    int pack_id;
    struct timespec time_stamp;
    struct list_head frag_list;
} frag_manage_t; 

typedef struct {
    pthread_mutex_t chain_lock;
    struct list_head chain_list;
} host_frag_t;

typedef struct {
    int pack_num;
    host_frag_t host;
} hash_slot_t;

typedef struct {
    struct timespec last_collect;
    hash_slot_t slots[HASH_SIZE];
} hash_frag_t;

static inline int 
PACKID_TO_INDEX(int pack_id) 
{
    return MOD(pack_id, HASH_SIZE);
} 

extern int init_sys_frag(void);

extern void exit_sys_frag(void);

extern void *handle_fragment(pack_info_t *info);

extern void free_all_fragments(struct iovec *vec, int num);

extern int frag_pack(void *pack, pack_info_t *info,
        struct iovec **ret);

#ifdef __cplusplus
}
#endif

#endif
