#ifndef __MODULE_PACKET_H__
#define __MODULE_PACKET_H__

#include "rbx4cfg.h"
#include "rbx4atom.h"

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push)
#pragma pack(1)

/* magic of packet header ...*/
#define MAGIC               "RBX4"
#define MAGIC_LENGTH        4
#define FIRST_FRAGMENT_ID   1
#define MAX_LOAD_SIZE       (63 * __KB)

/* Macro for packet operatio ...*/
#define PACKET(pack)		((packet_header_t *)pack)
#define GET_PACK_ID(pack)	(PACKET(pack)->pack_id)
#define GET_FRAG_ID(pack)	(PACKET(pack)->frag_id)
#define GET_PACK_LEN(pack)	(PACKET(pack)->pack_len)
#define GET_PACK_TYPE(pack) (PACKET(pack)->pack_type)
#define GET_OP_TYPE(pack)   (PACKET(pack)->op_type)
#define SET_FRAG_ID(p, id)	(PACKET(p)->frag_id = id)

#define DATA_INFO_LEN		(sizeof(data_info_t))
#define PACKET_HEAD_LEN		(sizeof(packet_header_t))
#define REQUEST_INFO_LEN	(sizeof(req_info_t))
#define STATUS_INFO_LEN	    (sizeof(status_info_t))
#define REPLY_INFO_LEN		(sizeof(reply_info_t))
#define EVENT_INFO_LEN      (sizeof(event_info_t))
#define RS_INFO_LEN			(sizeof(rs_info_t))
#define PLCTL_INFO_LEN		(sizeof(plctl_info_t))

/* Packet table operations ...*/
#define PACK_TABLE(index)			(packet_table[index])
#define PH_OPT_LEN(type)			(PACK_TABLE(type).phopt_len)
#define PACK_OP_NUM(i)				(PACK_TABLE(i).op_num)
#define PACK_HANDLE(i)				(PACK_TABLE(i).handle_packet)
#define DETECT_HANDLE(i)			(PACK_TABLE(i).detect_option)
#define SET_PACK_HANDLE(i, handle)  (PACK_HANDLE(i) = handle)
#define SET_DETECT_HANDLE(i, func)  (DETECT_HANDLE(i) = func)

#define VOIDS_ADD_INT(a, b) ((unsigned long)(a) + (b))

#define GET_OPTION(pack, info)\
    ({\
        memcpy(info, PACK_OPTION_POS(pack), sizeof(*info));\
        sizeof(*info);\
    })

typedef enum {
    TRANSMIT_PACK,
    STATUS_PACK,
    REPLY_PACK,
    NOTIFY_PACK,
    SUBREQ_PACK,
    INQUIRE_PACK,
	RS_PACK,
	PLCTL_PACK,
	PACK_TYPE_NUM,
} packet_type_t; 

typedef enum {
	/* TRANSMIT_PACK ...*/
	DATA_TRANSMIT = 0,
	DATA_REQUEST = 1,
	DATA_RESPOND = 2,
	TRANSMIT_OP_NUM = 3,
	/* STATUS_PACK ...*/
	READ_STATUS = 0,
	WRITE_STATUS = 1,
	STATUS_OP_NUM = 2,
	/* REPLY_PACK ...*/
	OP_FAILURE = 0,
	OP_SUCCESS = 1,
	OP_WARNING = 2,
	REPLY_OP_NUM = 3,
	/* NOTIFY_PACK ...*/
	NOTIFY_RBXEVENT = 0,
	NOTIFY_CAREVENT = 1,
	NOTIFY_OP_NUM = 2,
	/* SUBREQ_PACK ...*/
	ADD_REQUEST = 0,
	CAN_REQUEST = 1,
	MOD_REQUEST = 2,
	SUBREQ_OP_NUM = 3,
	/* INQUIRE_PACK ...*/
	NET_INQUIRE = 0,
	RS_INQUIRE  = 1,  /* Running Status of Whole System ...*/
	INQUIRE_OP_NUM = 2,
	/* RS_PACK ...*/
	TRANSMIT_RS = 0,
	RS_OP_NUM = 1,
	/* PLCTL_PACK ...*/
	READ_TASK = 0,
	RECOVERY_TASK = 1,
	INFORM_CONTINUE = 2,
	INFORM_PAUSE = 3,
	PLCTL_OP_NUM = 4,
} operation_type_t;

/* Packet header struct ...*/
typedef struct {
    int8_t magic[MAGIC_LENGTH];	/* magic = "RBX4" ... */
	uint16_t pack_type;	/* packet type ... */
    uint16_t op_type;	/* operation type ... */
    uint32_t pack_len;	/* length of packet ... */
    uint32_t pack_id;	/* id of packet ... */
    uint16_t frag_id;	/* id of fragment ... */
} packet_header_t;

typedef struct {
	uint32_t phopt_len;	 /* Length of Packet header + option ...*/
	uint32_t op_num;	 /* Number of operatons ...*/
	int (*detect_option)(void *);
	int (*handle_packet)(void *, int, void *);
} packet_operation_t;

/* data infomation ...*/
typedef struct {
    uint16_t data_no;
    uint16_t load_size;
} data_info_t;

/* subscribe request info ...*/
typedef struct {
    int8_t policy;
    int8_t hz;
    uint16_t data_no;
    uint32_t data_size;
} req_info_t;

/* Reply packet info ...*/
typedef struct {
	uint16_t pack_type;
    uint16_t op_type;
    uint32_t sub_type;
    uint32_t pack_id;
} reply_info_t;

/* Status packet info ...*/
typedef struct {
	uint32_t pack_id;	  /* Reply to Which READ_STATUS pack ...*/
    uint32_t car_status;
    uint32_t rbx_status;
} status_info_t;

/* Running Status Info ...*/
typedef struct {
	uint32_t pack_id;	  /* Reply to Which RS_INQUIRE pack ...*/
	uint32_t element_num;
	uint32_t rs_info_size;
} rs_info_t;

/* Host Running status ...*/
typedef struct {
	uint32_t host_ip;
	uint32_t host_status;
} host_status_t;

/* Event packet info ...*/
typedef struct {
    uint32_t event;
} event_info_t;

/* PL Ctl packet ...*/
typedef struct {
	uint32_t priv_data;
	uint32_t task_point;
} plctl_info_t;

/* Information of packet, used to fragment ...*/
typedef struct {
    void *pack;
    int pack_id;
    int frag_id;
    int frag_len;	/* Fragment size, include packet header ... */
    int pack_len;	/* Total length of the whole packet ... */
} pack_info_t;

extern packet_operation_t packet_table[PACK_TYPE_NUM];
extern void add_ph_option(void *packet, 
						  uint16_t pack_type, uint16_t op_type,
						  uint32_t pack_len, void *option);
extern int get_pack_info(void *pack, pack_info_t *pinfo);
extern int security_detect(void *packet, int recv_len);
extern int detect_transmit_pack(void *packet);
extern int detect_notify_pack(void *packet);
extern int detect_status_pack(void *packet);
extern int detect_reply_pack(void *packet);

static inline void *
PACK_OPTION_POS(void *buf) 
{
    return (void *) VOIDS_ADD_INT(buf, PACKET_HEAD_LEN);
}

static inline void 
SET_LOAD_SIZE(void *pack, int load_size) 
{
    void *data_pos = PACK_OPTION_POS(pack);
    ((data_info_t *) data_pos)->load_size = load_size;
    return;
}

static inline int 
PACK_LENGTH(int pack_type, int data_size) 
{
    return PH_OPT_LEN(pack_type) + data_size;
}

static inline int 
MAX_PACKET_SIZE(void) 
{
    int tmp = get_max_data_size();
	tmp += PH_OPT_LEN(TRANSMIT_PACK);
    return ALIGN(tmp, PAGE_SIZE);
}

static inline int 
NEED_FRAGMENT(int load_size) 
{
    return (load_size > MAX_LOAD_SIZE);
}

static inline void *
PACK_DATA_POS(void *buf) 
{
	int pack_type = GET_PACK_TYPE(buf);
    return (void *) VOIDS_ADD_INT(buf, PH_OPT_LEN(pack_type));
}

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif
