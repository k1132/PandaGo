#ifndef __ROBIX4_LOCK_H__
#define __ROBIX4_LOCK_H__

#include <errno.h>
#include <stdlib.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEFAULT_TIMEOUT		(50)

#define  MUTEX_INIT(entry)      (mutex_ops.init(entry))
#define  MUTEX_FREE(entry)      (mutex_ops.free(entry))
#define  MUTEX_LOCK(entry)      (mutex_ops.lock(entry))
#define  MUTEX_UNLOCK(entry)    (mutex_ops.unlock(entry))

#define  CONDS_INIT(entry)		(conds_ops.init(entry))
#define  CONDS_FREE(entry)		(conds_ops.free(entry))
#define  CONDS_WAIT(entry)		(conds_ops.wait(entry, 0))
#define  CONDS_WAITS(entry, s)  (conds_ops.wait(entry, s))
#define	 CONDS_SIGNAL(entry)	(conds_ops.signal(entry))

#define  RWLOCK_INIT(lock)		(rwlock_ops.init(lock))
#define  RWLOCK_FREE(lock)		(rwlock_ops.free(lock))
#define  RWLOCK_RLOCK(lock)		(rwlock_ops.reader_lock(lock))
#define  RWLOCK_WLOCK(lock)		(rwlock_ops.writer_lock(lock))
#define  RWLOCK_RUNLOCK(lock)	(rwlock_ops.reader_unlock(lock))
#define  RWLOCK_WUNLOCK(lock)	(rwlock_ops.writer_unlock(lock))

typedef pthread_rwlock_t rwlock_t;
typedef int (*rwlock_func_t) (rwlock_t *);
typedef int (*mutex_func_t) (pthread_mutex_t *);

typedef struct {
    mutex_func_t init;
    mutex_func_t free;
    mutex_func_t lock;
    mutex_func_t trylock;
    mutex_func_t unlock;
} mutex_ops_t;

typedef struct {
    rwlock_func_t init;
    rwlock_func_t free;
    rwlock_func_t reader_lock;
    rwlock_func_t reader_trylock;
    rwlock_func_t reader_unlock;
    rwlock_func_t writer_lock;
    rwlock_func_t writer_trylock;
    rwlock_func_t writer_unlock;
} rwlock_ops_t;


typedef struct {
    int32_t ready;
    pthread_cond_t cond;
    pthread_mutex_t mutex;
} wait_conds_t;

typedef int (*conds_func_t) (wait_conds_t *);
typedef int (*conds_wait_t) (wait_conds_t *, int);

typedef struct {
    conds_func_t init;
    conds_func_t free;
    conds_wait_t wait;
    conds_func_t signal;
} conds_ops_t;

extern mutex_ops_t  mutex_ops;
extern rwlock_ops_t rwlock_ops;
extern conds_ops_t  conds_ops;

#ifdef __cplusplus
}
#endif
#endif
