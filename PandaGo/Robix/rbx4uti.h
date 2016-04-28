#ifndef __MODULE_ROBIX4UTI_H__
#define __MODULE_ROBIX4UTI_H__

#include "rbx4eve.h"
#include "rbx4pack.h"
#include "rbx4reply.h"
#include "rbx4file.h"
#include <pthread.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>

#include "protocols/protocol_head.h"
#include "protocols/protocol_event.h"
#include "protocols/protocol_status.h"
#define MAX_HOST_NUM	(32)

#ifdef __cplusplus
extern "C" {
#endif

static inline double rbxAtof(char *value)
{
	return atof(value);
}

static inline long rbxAtol(char *value)
{
	return atol(value);
}

static inline int rbxAtoi(char *value)
{
	return atoi(value);
}

inline static struct timeval rbxTimeToSys(TIME_STAMP t)
{
    struct timeval sys = {t.tv_sec, t.tv_usec};
    return sys;
}

inline static TIME_STAMP rbxTimeToUGV(struct timeval ts)
{
    TIME_STAMP ugv = {ts.tv_sec, ts.tv_usec};
    return ugv;
}

inline static void rbxSetBeginTime(void *time)
{
	struct timeval ts;
	gettimeofday(&ts, NULL);
	((RBX_HEADER *)time)->begin = rbxTimeToUGV(ts);
	return ;
}

inline static void rbxSetEndTime(void *time)
{
	struct timeval ts;
	gettimeofday(&ts, NULL);
	((RBX_HEADER *)time)->end = rbxTimeToUGV(ts);
	return ;
}

inline static void rbxSetTimestamp(TIME_STAMP *time)
{
	struct timeval ts;
	gettimeofday(&ts, NULL);
	*time = rbxTimeToUGV(ts);
	return ;
}

static inline void delay(int sec, int usec)
{
	struct timeval delay;
	
	if (sec < 0) {
		sec = 0; usec = 1;
	} else if (sec > 0) {
		if (usec < 0) {
			sec -= 1; usec += MILLION;
		}
	} else {
		usec = (usec > 0) ? usec : 1;
	}

	delay.tv_sec 	= sec;
	delay.tv_usec	= usec;
	select(0, NULL, NULL, NULL, &delay);
	return ;
}

extern int  init_sys_uti(int role);
extern void exit_sys_uti(void);

/* Transmit RBX Event && Car Event to KRBX ...*/
extern int rbxTransmitEvent(int rbx_event, int car_event);

/* rbxInquireStatus : Get Car Status && KRBX Status ...*/
extern int rbxInquireStatus(status_info_t * si);

/* Following 2 functions : Get Each Host status ...*/
extern int rbxInquireSystemState(rs_info_t * ri);
extern host_status_t rbxGetHostStatus(int index);

/* Cfg file operations ...*/
extern int rbxReadCfgFile(char *filename);
extern char *rbxGetCfg(int fd, char *name);
extern int rbxCloseCfgFile(int fd);

/* Translate the Status/Event Index to Responding Name ...*/
extern char *rbxGetRbxStatusName(int status);
extern char *rbxGetCarStatusName(int status);
extern char *rbxGetRbxEventName(int event);
extern char *rbxGetCarEventName(int event);

/* Get GPS1/GPS2 from KRBX ...*/
extern int rbxGetGPS1(POSE_INFO *pose1);
extern int rbxGetGPS2(POSE_INFO *pose2);

/* Notify PL ...*/
extern int rbxPLReadTask(void);
extern int rbxPLRecoveryTask(uint32_t);
extern int rbxPLInformContinue(void);
extern int rbxPLInformPause(void);

/* TCP operations : Used by PL connect to CS ...*/
extern ssize_t readn(int fd, void *vptr, size_t n);
extern ssize_t writen(int fd, void *vptr, size_t n);
extern int getSockfdToServer(char *ip, uint16_t port, int block);
extern int listenPort(int port, int listenq);
extern int getSockfdFromClient(int listenfd);

#ifdef __cplusplus
}
#endif

#endif
