#ifndef __ROBIX4_NET_H__
#define __ROBIX4_NET_H__

#include "rbx4err.h"
#include "rbx4head.h"

#include <sys/uio.h>

#ifdef __cplusplus
extern "C" {
#endif

struct sockaddr;

extern int udp_create(uint32_t ip, uint16_t port);
extern int udp_close(int fd);
extern int udp_recv4(int fd, 
					 void *buf, 
					 int size,
					 struct sockaddr *addr);
extern int udp_recv5(int fd, 
					 void *buf, 
					 int size,
					 uint32_t *ip, 
					 uint16_t *port);
extern int udp_send4(int fd, 
					 void *buf, 
					 int size,
					 struct sockaddr *addr);
extern int udp_send5(int fd, 
					 void *buf, 
					 int size,
					 uint32_t ip, 
					 uint16_t port);
extern int udp_sendv(int fd, 
					 struct iovec *vecs, 
					 int num,
					 uint32_t ip, 
					 uint16_t port);
extern int get_net_ip(char *eth, uint32_t * ip);
extern int ip_to_str(uint32_t ip, char *buf);
extern int str_to_ip(char *buf, uint32_t * ip);

#ifdef __cplusplus
}
#endif
#endif
