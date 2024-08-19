/*-----------------------------------------------------------------------------
 * EcDemoApp.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Holger Oelhaf
 * Description              Application specific settings for EC-Master demo
 *---------------------------------------------------------------------------*/

#ifndef INC_ECDEMOAPP_H
#define INC_ECDEMOAPP_H 1

/*-LOGGING-------------------------------------------------------------------*/
#ifndef pEcLogParms
#define pEcLogParms (&(pAppContext->LogParms))
#endif

#define INCLUDE_EC_MASTER

/*-INCLUDES------------------------------------------------------------------*/
#include "AtEthercat.h"
#include "EcDemoPlatform.h"
#include "EcDemoParms.h"
#include "EcLogging.h"
#include "EcNotification.h"
#include "EcSdoServices.h"
#include "EcSelectLinkLayer.h"
#include "EcSlaveInfo.h"
#include <iostream>
#include <math.h>
#include <mutex>

/*-DEFINES-------------------------------------------------------------------*/
#define EC_DEMO_APP_NAME (EC_T_CHAR *)"EcMasterDemoDc"

/* the RAS server is necessary to support the EC-Engineer or other remote applications */
#if (!defined INCLUDE_RAS_SERVER) && (defined EC_SOCKET_SUPPORTED)
#define INCLUDE_RAS_SERVER
#endif

#if (defined INCLUDE_RAS_SERVER)
#include "AtEmRasSrv.h"
#define ATEMRAS_MAX_WATCHDOG_TIMEOUT 10000
#define ATEMRAS_CYCLE_TIME 2
#endif

#define MOTOR_STATUS_NO_ERROR 0
#define MOTOR_STATUS_ERROR 1
#define MOTOR_STATUS_REINIT 2

/***********************************************/
/* static EtherCAT master configuration values */
/***********************************************/
#define ETHERCAT_DC_TIMEOUT 12000          /* DC initialization timeout in ms */
#define ETHERCAT_DC_ARMW_BURSTCYCLES 10000 /* DC burst cycles (static drift compensation) */
#define ETHERCAT_DC_ARMW_BURSTSPP 12       /* DC burst bulk (static drift compensation) */
#define ETHERCAT_DC_DEV_LIMIT 13           /* DC deviation limit (highest bit tolerate by the broadcast read) */
#define ETHERCAT_DC_SETTLE_TIME 1500       /* DC settle time in ms */

/*--------------------------------------------------------------------------*/
/* Performance measurements of jobs                                         */
/* This is only available on CPUs with TSC support                          */
/*--------------------------------------------------------------------------*/
#define PERF_MEASURE_JOBS_INIT(numJobs)                             \
  ecatPerfMeasInit(&pAppContext->TscMeasDesc, 0, numJobs, EC_NULL); \
  ecatPerfMeasEnable(&pAppContext->TscMeasDesc)
#define PERF_MEASURE_JOBS_DEINIT() ecatPerfMeasDeinit(&pAppContext->TscMeasDesc)
#define PERF_MEASURE_JOBS_RESET() ecatPerfMeasReset(&pAppContext->TscMeasDesc, 0xFFFFFFFF);
#define PERF_MEASURE_JOBS_SHOW() ecatPerfMeasShow(&pAppContext->TscMeasDesc, 0xFFFFFFFF, S_aszMeasInfo)
#define PERF_JOB_START(nJobIndex) ecatPerfMeasStart(&pAppContext->TscMeasDesc, (EC_T_DWORD)(nJobIndex))
#define PERF_JOB_END(nJobIndex) ecatPerfMeasEnd(&pAppContext->TscMeasDesc, (EC_T_DWORD)(nJobIndex))

#pragma pack(1)
typedef struct
{
  int32_t position_actual_value;
  int16_t torque_actual_value;
  uint16_t status_word;
  int16_t mode_of_opration_display;
  int32_t position_demand_raw;
  int32_t velocity_demand_raw;
  int32_t velocity_actual_value;
  int16_t torque_demand_raw;
  uint32_t dc_link_circuit_voltage;
  // int16_t current_actual_value;
  uint16_t error_code;
} SlaveRead_t;


typedef struct
{
  int32_t target_position;
  int32_t target_velocity;
  int16_t target_torque;
  uint16_t max_torque;
  uint16_t control_word;
  int16_t mode_of_opration;
  int32_t position_offset;
  int32_t velocit_offset;
  int16_t torque_offset;
} SlaveWrite_t;
#pragma pack()

typedef struct
{
  double position;
  double velocity;
  double torque;
  double maxTorque;
  double positionOffset;
  double velocityOffset;
  double torqueOffset;
  double acceleration;
  uint8_t status;
  double position_demand_trans;
  double velocity_demand_trans;
  double torque_demand_trans;
  double driver_dc_voltage;
  uint16_t status_word;
  uint16_t error_code;
}MotorParam_t;

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
EC_T_VOID ShowSyntaxAppUsage(T_EC_DEMO_APP_CONTEXT *pAppContext);
EC_T_VOID ShowSyntaxApp(T_EC_DEMO_APP_CONTEXT *pAppContext);
EC_T_VOID ShowSyntaxLinkLayer(EC_T_VOID);
EC_T_DWORD EcDemoApp(T_EC_DEMO_APP_CONTEXT *pAppContext);

extern bool isEcMasterExit();
void setEcMasterExit();

bool motorIsEnable(const uint16_t id);

uint8_t motorStatus(const uint16_t id);

extern void motorGetData(const uint16_t *ids, uint32_t num, MotorParam_t *data);

extern void motorSetPosition(const uint16_t *ids, uint32_t num, MotorParam_t *params);
extern void motorSetVelocity(const uint16_t *ids, uint32_t num, MotorParam_t *params);
extern void motorSetTorque(const uint16_t *ids, uint32_t num, MotorParam_t *params);

extern void motorToPosition(T_EC_DEMO_APP_CONTEXT *pAppContext, const uint16_t *ids, uint32_t num_id, const double *q_d, double speed, double dt);
extern void motorToPosition(const uint16_t *ids, uint32_t num_id, const double *q_d, double speed, double dt);

extern bool isMotorEnable(void);
extern uint32_t getNumSlave(void);
void setEcEncoderRange(uint32_t *encoder_range_set, uint16_t num);

// void motorGetCurrent(double *current_actual);

/* function in EcNotification */
void fixEmergencyRequest(const int slaveAddr,const int errorCode);
void getMotorPositionOffset(double *offset, uint16_t len);
void setMotorPositionOffset(double *offset, uint16_t len);

#define PRINT_PERF_MEAS() ((EC_NULL != pEcLogContext) ? ((CAtEmLogging *)pEcLogContext)->PrintPerfMeas(pAppContext->dwInstanceId, 0, pEcLogContext) : 0)
#define PRINT_HISTOGRAM() ((EC_NULL != pEcLogContext) ? ((CAtEmLogging *)pEcLogContext)->PrintHistogramAsCsv(pAppContext->dwInstanceId) : 0)

#endif /* INC_ECDEMOAPP_H */

/*-END OF SOURCE FILE--------------------------------------------------------*/
