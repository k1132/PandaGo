#ifndef __ROBIX4_THREAD_H
#define __ROBIX4_THREAD_H

#include "rbx4man.h"
#include "rbx4rlv.h"
#include "rbx4che.h"
#include "rbx4stat.h"

#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int publish_hz;

extern int thread_recv_data(void *param);
extern void recv_data_exit(void *packet);

extern int thread_recv_req(void *param);
extern void recv_req_exit(void *packet);

extern int init_sys_sreq(void);
extern void exit_sys_sreq(void);
extern int send_request(struct iovec *vec);
extern int inquire_connect_state(void);
extern int report_ready(void);

extern int init_sys_sdata(void);
extern void exit_sys_sdata(void);
extern int send_data(struct iovec *vec);
extern int transmit_data(int data_no, void *pack, int size);

extern int init_all_threads(char *path, char *mode, int role);
extern void exit_all_threads(void);

#ifdef __cplusplus
}
#endif
#endif
