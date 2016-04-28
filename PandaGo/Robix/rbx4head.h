#ifndef __MODULE_KHEAD_H__
#define __MODULE_KHEAD_H__


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Define KB, MB, GB ..*/
#define __KB                (1 << 10)
#define __MB                (1 << 20)
#define __GB                (1 << 30)

//used for UDP packet
#define MAX_FRAG_SIZE       (64 * __KB)
#define MAX_RCVBUF_SIZE     (1024 * __KB)
#define MAX_SNDBUF_SIZE     (1024 * __KB)

#define BILLION             (1000000000)
#define MILLION             (1000000)
#define HUNDRED             (100)
#define THOUSAND            (1000)
#define MIN_CHAR            (-127)
#define MAX_INT             (0x7fffffff)

/* Ports ...*/
#define GPS1_PORT   4001
#define GPS2_PORT   4003

/* Cfg file ...*/
#define  CONFIG_PATH	"/etc/robix/urobix.cfg"
#define  CONFIG_MODE	"r"
#define  ROBIX4_NAME    "URobix"

/*
 * Note : When FU/PL URobix Run in the same host with KRobix4,
 * We need to use another set of ports to communications ...
 */
#define RECV_DATA_PORT	10000
#define SEND_DATA_PORT  10001
#define RECV_REQ_PORT   10002
#define SEND_REQ_PORT   10003
#define RECV_CTL_PORT	10004
#define SEND_CTL_PORT	10005

/*
 * FU/PL URobix use set of ports :
 * RECV_DATA_PORT	10000 + PORT_DELTA
 * SEND_DATA_PORT	10001 + PORT_DELTA
 * RECV_REQ_PORT	10002 + PORT_DELTA
 * SEND_REQ_PORT	10003 + PORT_DELTA
 */
#define PORT_DELTA      1000

/* Size ...*/
#define DATA_NAME_SIZE      40
#define MODULE_NAME_SIZE    20
#define AGENT_NAME_SIZE     20
#define IP_ADDR_SIZE        20
#define BRIDGE_NAME_SIZE    20
#define NIC_NAME_SIZE       20

#define PAGE_SHIFT      12
#define PAGE_SIZE       (1 << PAGE_SHIFT)
#define PAGE_MASK       ((1 << PAGE_SHIFT) - 1)

/* Must be same as KRobix ...*/
#define DEFAULT_HZ      10
#define MAX_HZ_VALUE    60
#define MIN_HZ_VALUE    (-9)

/* Base Operations ...*/
#define MAX(a, b)           ((a) > (b) ? (a) : (b))
#define MOD(a, b)           ((a) % (b))
#define ALIGN(x, a)         (((x) + (a) - 1) & ~((a) - 1))

//portable with 32bit/64bit computer
#if defined __x86_64__
	typedef uint64_t uaddr_t;
#else
	typedef uint32_t uaddr_t;
#endif

enum ROLE_TYPE {
	ROLE_UROBIX = 0,
	ROLE_LOOPBACK,//KROBIX
	ROLE_SAMPLER,
	ROLE_CONTROLLER,
	ROLE_INSPECTOR,
};

enum PORT_SET {
	DEFAULT_SET = 0,
	ROBIX_SET = 0,
	LOOPBACK_SET,
	SAMPLER_SET,
	CONTROLLER_SET,
	INSPECTOR_SET,
};

enum PORT_TYPE {
	RECV_DATA = 0,
	SEND_DATA,
	RECV_REQ,
	SEND_REQ,
	RECV_CTL,
	SEND_CTL,
};

static inline int
__type_to_port(enum PORT_TYPE type)
{
	return RECV_DATA_PORT + type;
}

static inline int
__get_remote_port(enum PORT_SET set, enum PORT_TYPE type)
{
	return __type_to_port(type) + (set << 1) * PORT_DELTA;
}

static inline int
__get_local_port(enum PORT_SET set, enum PORT_TYPE type)
{
	if (set == ROBIX_SET)
		return __get_remote_port(set, type);
	return __get_remote_port(set, type) + PORT_DELTA;
}

#ifdef __cplusplus
}
#endif

#endif
