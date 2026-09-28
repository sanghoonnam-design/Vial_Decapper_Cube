/*******************************************************************************
 * XSystemInfo.h
 *
 *  Created on: 2025.6.20
 *      Author: RND. Kang PilSoon.
 ******************************************************************************/
#ifndef __XSYSTEMINFO_H__
#define __XSYSTEMINFO_H__

#include "XGlobal.h"
#include "_01_HW_and_Device.h"
#include "XSystemInfo_ModuleDef.h"
#include "_01_XSystemManagement.h"
#include "XEEPROMParam.h"
#include "XErrorCode.h"

//-------------------------------------------------
//   용도	             권장 임계값
//-------------------------------------------------
//   경고 (Warning)      80~85 °C
//   제한 (Limit)	     90 °C
//   에러 (Fault)	     95~100 °C
//   절대 한계	          125 °C
//-------------------------------------------------
#define PL_DEFAULT_DG_CPU_TEMPERATURE_OVERHEAT_CRITERIA /* */ (80)	   // 80도 이상일 때 CPU 온도 과열로 판단하는 기준값, 단위: 섭씨도
#define PL_DEFAULT_DG_CPU_TEMP_ALARM_INTERVAL_10msec /*    */ (__5min) // CPU 온도 과열 알람 주기, 단위: 호출 주기 기준

//==================================================================================
typedef enum
{
	FW_MODE_DEFAULT /*    */ = 0,  // FW 기본동작 모드
	FW_MODE_IDLE /*       */ = 1,  // FW idling mode : 메인 태스크가 헛도는 모드, test mode
	FW_MODE_TIMER_STOP /* */ = 2,  // Timer 정지, 메인 태스크 정지 모드
	FW_MODE_APC_STOP /*   */ = 3,  // APC 제어 태스크 정지 모드[메인 로직은 수행되지 않음]
	FW_MODE_FACTORY_TEST /**/ = 4, // 제조/양산 공정용 모드 (출하검사, 기능검증)
	FW_MODE_COUNT
} teXFW_Mode;

#define MODEL_NAME_STR ("HEM_CT_Handler") // 시스템 모델명 문자열

#define FW_MODE_TEST (FW_MODE_IDLE)
#define FW_MODE_MAIN_TASK_STOP (FW_MODE_TIMER_STOP)

typedef struct SystemInfoGroup
{
	U08 cd_resv;				// 예약
	U08 pl_resv;				// 예약
								//
	U08 sl_isStartMainLoop;		//!> 메인루프가 시작되었는지 알리는 플래그
	U08 sl_isDateChanged;	 	//!> 날짜가 변경되었는지 알리는 플래그

	U32 cd_ErrorCount;			//!> system error counting, //TODO 업데이트 로직 추가
	F32 cd_CpuUsage;			//!> CPU 사용률
	F32 cd_CpuTemperature;		//!> CPU 내부 온도
								//
	U32 pl_FW_Version;			//!> [컴파일시 정해짐] from EEPROM or SD
	char cd_FWVersion_str[20];	//!> [컴파일시 정해짐] version 문자열로 재생성한 버전
	U08 pl_systemType;			//!> [컴파일시 정해짐] Robo-I, Robo-IS, Robo-S
	U08 pl_modelType;			//!> [컴파일시 정해짐] 시스템별 모델 타입
								//
	Network_t network;			//!> ip 정보
								//
								//   [디버깅용 파라미터]
	U08 pl_FW_Mode;				//!> FW 동작 모드, teXFW_Mode 참조
	S32 pl_IsExecutedDiagnosis; //!> 1(default): Diagnosis, 0: No Diagnosis, 디버깅용

	VOI (*SL_Set_isStartMainLoop)(U08);
	U08 (*SL_Get_isStartMainLoop)(void);

	VOI (*CD_Set_CpuUsage)(F32);
	F32 (*CD_Get_CpuUsage)(void);
	VOI (*CD_Set_CpuTemperature)(F32);
	F32 (*CD_Get_CpuTemperature)(void);

	VOI (*PL_Set_FW_Version)(U32);
	U32 (*PL_Get_FW_Version)(void);
	VOI (*PL_Set_FW_Mode)(U08);
	U08 (*PL_Get_FW_Mode)(void);
	VOI (*PL_Set_IsExecutedDiagnosis)(U32);
	U32 (*PL_Get_IsExecutedDiagnosis)(void);

} tsXSystemInfo;

extern tsXSystemInfo xSystemInfo;

/************************************************************************** */
void System_Initialize(void);

// teSystem_Type GetSystemType(void);
U08 GetSystemType(void);
U08 GetModelType(void);
const char *GetSystemTypeString(void);
const char *GetModelTypeString(void);

SET_GET_FUNC_0(SystemInfo, U08, sl_isStartMainLoop)
SET_GET_FUNC_0(SystemInfo, F32, cd_ErrorCount)
SET_GET_FUNC_0(SystemInfo, F32, cd_CpuUsage)
SET_GET_FUNC_0(SystemInfo, F32, cd_CpuTemperature)
// SET_GET_FUNC_0(SystemInfo, U32, pl_FW_Version)
SET_GET_FUNC_0(SystemInfo, U08, pl_FW_Mode)
SET_GET_FUNC_0(SystemInfo, U32, pl_IsExecutedDiagnosis)
void SystemInfo_Set_pl_FW_Version(U32 version);
U32 SystemInfo_Get_pl_FW_Version(void);

void SystemInfo_Network_SetIP(Network_t *net, const U8 ip[4]);

void makeVersionString(void);

#endif /* __XSYSTEMINFO_H__ */
