#ifndef __ROBIX4_RESOLVE__
#define __ROBIX4_RESOLVE__

#include "rbx4cfg.h"
#include "rbx4lock.h"
#include "rbx4pack.h"
#include <string.h>
#include <sys/uio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void *buf_pos;
    int buf_size;
    int pack_len;
} relv_result_t;

extern int init_sys_resolve(char *path, char *mode, int role);
extern void exit_sys_resolve(void);
extern int cap_packet(void *buf, 
					  uint16_t data_no, 
					  uint32_t data_size,
					  int8_t policy, 
					  int8_t read_cache,
					  struct iovec **vec);
int free_packets(struct iovec *vec, int num);
extern int relv_packet(void *pack, int pack_len, void *result);

#ifdef __cplusplus
}
#endif
#endif
