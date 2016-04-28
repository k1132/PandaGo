#ifndef __ROBIX4_ERROR_H__
#define __ROBIX4_ERROR_H__

#include <stdio.h>
#include <string.h>

//make it protable
#ifdef __cplusplus
extern "C" {
#endif

#define RET_SUCCESS		0
#define RET_ERROR	   -1

#ifndef TRUE
#define TRUE		1
#endif

#ifndef	FALSE
#define FALSE		0
#endif

//debug info,when encounter error
#define KBUG(fmt, args...)\
    fprintf(stderr, "Debug : %s, %s, %d,\n\t "fmt, __FILE__,\
            __FUNCTION__, __LINE__, ##args)
//print useful infomation,make sure
//that the program is running correctly
#define KPRINT(fmt, args...)\
    fprintf(stderr, fmt, ##args)

#ifdef __cplusplus
}
#endif
#endif
