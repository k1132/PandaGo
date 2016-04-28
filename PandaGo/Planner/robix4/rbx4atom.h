#ifndef __MODULE_ATOMIC_H__
#define __MODULE_ATOMIC_H__

#ifdef __cplusplus
extern "C" {
#endif

#define atomic_read(v)		(atomic_add_return(0, v))

static inline int 
atomic_add_return(int i, int *v) 
{
    int __i;
    __i = i;
    __asm__ __volatile__("lock; xaddl %0, %1;":"=r"(i)
            :"m"(*v), "0"(i));
    return i + __i;
} 

static inline int 
atomic_inc_return(int *v) 
{
    return atomic_add_return(1, v);
}

static inline void 
atomic_inc(int *v) 
{
    __asm__ __volatile__("lock; incl %0":"=m"(*v)
            :"m"(*v));
}

static inline void 
atomic_dec(int *v) 
{
    __asm__ __volatile__("lock; decl %0":"=m"(*v)
            :"m"(*v));
}

static inline int 
atomic_sub_return(int i, int *v) 
{
    return atomic_add_return(-i, v);
}

static inline int 
atomic_dec_return(int *v) 
{
    return atomic_sub_return(1, v);
}

#ifdef __cplusplus
}
#endif

#endif
