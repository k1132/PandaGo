#ifndef __MODULE_ALLOC_H__
#define __MODULE_ALLOC_H__

#include "rbx4err.h"
#include "rbx4head.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALLOC_STR_ADDR(info)  ((info)->str_addr)
#define ALLOC_ELE_NUM(info)   ((info)->ele_num)
#define ALLOC_ELE_SIZE(info)  ((info)->ele_size)
#define ALLOC_BUF_CAP(info)   ((info)->ele_cap)

typedef enum {
	LINEAR_MODE = 0,
	EXPONENT_MODE,
	MODE_NUM,
} alloc_mode_t;

typedef struct {
	void *str_addr;	/* Start addr of buffer ...*/
	int ele_size;	/* Size of per-element ...*/
	int ele_cap;	/* Num of elements that buf can contain ...*/
	int ele_index;	/* Index of next element added to buf ...*/
	int ele_num;	/* Num of elements added to buf ...*/
	int inc_mode;	/* Mode : LINEAR_MODE/EXPONENT_MODE ...*/
	int inc_value;  /* increasing Delta ...*/
} alloc_info_t;

static inline int 
IS_ALLOC_MODE(int mode) 
{
    return mode >= LINEAR_MODE && mode < MODE_NUM;
}

extern alloc_info_t element_alloc(int size, int num);
extern void element_free(alloc_info_t *);
extern int element_set(alloc_info_t *, int mode, int val);
extern int element_realloc(alloc_info_t *, int mode, int val);
extern int element_add(alloc_info_t *, void *ele);
extern int element_put(alloc_info_t *, void *ele, int pos);

#ifdef __cplusplus
}
#endif

#endif
