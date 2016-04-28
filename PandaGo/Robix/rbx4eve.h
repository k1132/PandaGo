#ifndef __MODULE_EVENT_H__
#define __MODULE_EVENT_H__

#include "rbx4err.h"
#include "rbx4head.h"
#include "protocols/protocol_event.h"
#include "protocols/protocol_status.h"

/* BITS[24 - 31] --> RBX_EVENT, BITS[0 - 23] --> CAR_EVENT ...*/
#define E_RBX_BITS          (8)
#define E_CAR_BITS          (24)
#define R_SHIFT(a, bit)     ((a) >> (bit))
#define L_SHIFT(a, bit)     ((a) << (bit))

typedef enum {
	_E_READY = 0,		/* Sended by URBX, when AGENTS are ready ... */
	_E_ALL_READY,		/* Sended by KRBX, When URBXS  are ready ... */
	_E_PAUSE,
	_E_CONTINUE,
	_E_EXITING,
	_E_EVENTNUM,
} RBX_EVENT;

typedef enum {
	_S_INVALID = 0,
	_S_READY,
	_S_RUNNING,
	_S_PAUSE,
	_S_EXITING,
	_S_STATNUM,
} RBX_STATUS;

typedef enum {
	_NET_DISCONNECT,
	_NET_CONNECTION,
	_NET_STATUSNUM,
} NET_STATUS;

static inline int
IS_RBX_STATUS(uint32_t status)
{
	return (status >= 0 && status < _S_STATNUM);
}

static inline int
IS_CAR_STATUS(uint32_t status)
{
	return (status >= 0 && status < NUM_STATUS);
}

static inline int
IS_RBX_EVENT(uint32_t event)
{
	return (event >= 0 && event < _E_EVENTNUM);
}

static inline int
IS_CAR_EVENT(uint32_t event)
{
	return (event >= 0 && event < NUM_EVENTS);
}	

static inline uint32_t
SET_EVENT(uint32_t rbx_event, uint32_t car_event)
{
	return (L_SHIFT(rbx_event, E_CAR_BITS) | car_event);
}

static inline uint32_t
GET_RBX_EVENT(uint32_t event)
{
	return R_SHIFT(event, E_CAR_BITS);
}

static inline uint32_t
GET_CAR_EVENT(uint32_t event)
{
	return R_SHIFT(L_SHIFT(event, E_RBX_BITS), E_RBX_BITS);
}

extern uint32_t transform_status(uint32_t stat, uint32_t event);

#endif
