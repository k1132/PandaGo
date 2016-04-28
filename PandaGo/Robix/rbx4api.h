#ifndef __ROBIX4_API_H__
#define __ROBIX4_API_H__

#include "rbx4man.h"
#include "rbx4thd.h"
#include "rbx4sub.h"
#include "rbx4klog.h"
#include "rbx4reply.h"
#include "rbx4thd_common.h"
#include "protocols/protocol_head.h"
#include "protocols/protocol_event.h"
#include "protocols/protocol_status.h"

#ifdef __cplusplus
extern "C" {
#endif

#define  MBUG(fmt, args...)\
    PRINT(agent_no_of_pthread( ), fmt, ##args)

/**
 * @brief   Check ROBIX Connect to KRBX Success .
 *
 * @return  Return TRUE, if ROBIX Conn State is OK .
 *          Return FALSE, otherwise .
 */
extern int rbxConnectSuccess(void);

/**
 * @brief   Check Whether ROBIX Can Connect to KRBX  .
 * This Operation Will Block Until One of following
 * conditions becomes true :
 *		1). ROBIX has receiced respond from KRBX .
 *		2). Call has waited for 1 seconds .
 *
 * @return  Return _NET_CONNECTION, if ROBIX SUCCEED .
 *          Return _NET_DISCONNECT, otherwise .
 */
extern int rbxInquireConnState(void);

/**
 * @brief   Check Whether ROBIX will exit .
 *
 * Each Agent should detect this condition in each loop . 
 * If ROBIX will exit soon, each Agent should exit soon
 * by itself, or it will be killed by ROBIX .
 *
 * @return  Return TRUE, if ROBIX WILL exit .
 *          Return FALSE, otherwise .
 */
extern int rbxWillExit(void);

/** 
 * @brief   Agent talks to ROBIX that it will exit by itself .
 * 
 * If Agent Call this function, Means that agent will exit by
 * itself Rather than be killed by ROBIX .
 *
 * @return  Return  0, if Agent exists .
 *          Return -1, otherwise . 
 */
extern int rbxPthreadExiting(void);

/**
 * @brief   Get the index of data .
 *
 * @param   [in] data_name  The name of the data, Which is
 *                          Defined in the configure file .
 *
 * @return  On success, return the data index .
 *          On failure, return -1 .
 */
extern int rbxGetDataIndex(const char *name);

/**
 * @brief   Read Data from ROBIX cache .
 * 
 * If Agent want to read data from ROBIX cache, Agent should
 * Call `rbxAddSubscribe` to subscribe the data first.
 *
 * @param   [in]  data_no  Which data you want to read .
 *          [out] buf      The Buffer used to save the data .
 *          [in]  bufsize  The size of the Buffer .
 * 
 * @return  On success, return  0 . 
 *          On failure, return -1 .
 */
extern int rbxReadData(int data_no, void *buf, int bufsize);

extern int rbxReadGPS(POSE_INFO *gps1);
extern int rbxReadGPS2(POSE_INFO *gps2);

/**
 * @brief   Write Data to ROBIX cache and ROBIX domain .
 * 
 * Agent write the data produced by itself, So that the Agent 
 * which need to subscribe can read it from ROBIX domain .
 *
 * @param   [in]  data_no  Which data you want to write .
 *          [in]  buf      The Buffer used to save the data .
 *          [in]  bufsize  The size of the Buffer .
 * 
 * @return  On success, return  0 . 
 *          On failure, return -1 .
 */
extern int rbxWriteData(int data_no, void *buf, int bufsize);

/**
 * @brief   Subscribe operations .
 *
 * If the Agent want to subscribe from ROBIX domain, it should
 * Call rbxAddSubscribe to subscribe the data it need at first . 
 * If the Agent want to modify subscribe, such as modify policy
 * of subscribe OR frequence of subscribe, it should call func
 * rbxModSubscribe . If the Agent want to cancel the subscribe
 * due to it Doesn't the data after that,  it should call func
 * rbxDelSubscribe .
 * 
 * @param   [in]  data_no   Which data you want to subscribe .
 *          [in]  policy    Which policy, you want to subscirbe
 *                          the data . Used 0 by default .
 *          [in]  hz        The frequence of data produced by 
 *                          ROBIX domain .
 *
 * @return  On success, return  0 .
 *          On failure, return -1 . 
 */
extern int rbxAddSubscribe(int data_no, int policy, int hz);
extern int rbxDelSubscribe(int data_no);
extern int rbxModSubscribe(int data_no, int policy, int hz);

extern int rbxReportReady(const char *module_name);

/**
 * @brief   Send EVENT to ROBIX domain .
 *
 * When Agent Want to Send event to ROBIX domain, Due to one
 * EVENT has occured, please Call this function .
 * 
 * @param   [in]  event   The EVENT sended to ROBIX domain .
 *
 * @return  On success, return  0 .
 *          On failure, return -1 .
 */
extern int rbxWriteEvent(EVENT event);

/**
 * @brief   Read Status from ROBIX domain .
 *
 * Read current status of Smart Car from ROBIX domain .
 *
 * @param   [out] status   The STATUS read from ROBIX domain .
 *
 * @return  On success, return  0 .
 *          On failure, return -1 .
 */
extern int rbxReadStatus(STATUS *status);

/**
 * @brief   Register subpthread of Agent .
 *
 * When One Agent create subpthread in the Agent, Agent should
 * Call this function to Register subpthread in the subpthread.
 * If Agent doesn't do that, The subpthread of Agent can't use
 * APIS that call `agent_no_of_pthread` function to fulfill .
 *
 *
 * @param   [in]  agent_name   The Name of Agent,  Please  use
 *                             the macro of Agent name .  Such
 *                             as `PL_NAME`, `LIDAR_NAME` .
 *
 * @return  On success, return  0 .
 *          On failure, return -1 . 
 */
extern int rbxRegisterSubpthread(const char *agent_name);

/**
 * @brief   These functions are used by ROBIX only .
 * 
 * The two functions following, used to init and cleanup
 * the subsystem of ROBIX api .
 *
 * @return  On success, return  0 .
 *          On failure, return -1 .
 */
extern int init_sys_api(void);
extern void exit_sys_api(void);

#ifdef __cplusplus
}
#endif
#endif
