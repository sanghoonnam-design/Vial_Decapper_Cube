/*******************************************************************************
 * XSystem_DB.h
 *
 *  Created on: 2025.09.04
 *      Author: RND. Kang PilSoon.
 *
 ******************************************************************************/
#ifndef __XSYSTEM_DB_H__
#define __XSYSTEM_DB_H__

#include "XGlobal.h"
#include "_01_HW_and_Device.h"

#include "CT_Handler.h"

// 시스템 DB의 헤더 부분, 향후 확장 가능하도록 미리 정의
// 4 * 16 = 64 Bytes 로 설계함. => 고정
typedef struct
{
    /* [PL Structure & FW Default Information] */ //
    int ID;                                       // [1] Parameter List ID, 식별자
    int Size_PL;                                  // [2] Parameter List 크기, 고정값
                                                  //
    int Size_Header;                              // [3] Header 크기, 고정값
    U32 FW_Version;                               // [4] FW version
    U32 UpdateDate;                               // [5] [YYMMDD 형식] parameter를 EEPROM에 저장한 날짜
                                                  //
    /* [System Default Information] */            /** @note USER CODE - START */
    F32 DG_CPU_Temperature_Overheat_Criteria;     // [6] CPU 온도 과열 판단 기준값, 단위: 섭씨도
    U32 DG_CPU_Temp_Alarm_Interval_10msec;        // [7] CPU 온도 과열 알람 주기, 단위: 호출 주기 기준
    int DG_IsDiagnosisEnabled;                    // [8] 고장진단 활성화 여부
    int DG_Client2Host_LogMode;                   // [9] Client -> Host 명령어 로그 활성화 모드, teLogMode_ClientToHostCmd 참조
    int rsv10;                                    // [10] future use
    int rsv11;                                    // [11] future use
    int rsv12;                                    // [12] future use
                                                  //
    /* [Application Default Information] */       //
    int rsv13;                                    // [13] future use
    int rsv14;                                    // [14] future use
    int rsv15;                                    // [15] future use
    F32 Time;                                     // [16] Host -> Client 업데이트 시간
                                                  // System Time == gTriggerTime, 시스템 가동 후 시간, 단위: 초
                                                  /** @note USER CODE - END */
} tsXPL_Header;                                   /** @brief : 64 bytes 로 고정 */
// =====================================================================================
// =====================================================================================

#pragma pack(push, 1)
typedef struct
{
    F32 Time; // [sec] 시스템 가동 후 시간
    /**──────────────────────────────── 여기까지 기본 포멧 [수정하지 말것!~] */

    int isBusy;
    int isError;
    int isEnable;
    int isHomed;
    int errorCode;

    tsXSL_Decapper Decapper; 				//is homed 과 같은 형재 상태 확인
    xSLecapping_MotorRun xZ_Motor_Run;		//Z축 모터의 움직임 확인
    xSLecapping_MotorRun xR_Motor_Run;		//R축 모터의 움직임 확인

} tsXStateList;

typedef struct
{
    F32 ID;                // [1] [sec] 시스템 가동 후 시간
    F32 rsv2;              // [2] future use
    F32 rsv3;              // [3] future use
    F32 rsv4;              // [4] future use
    F32 rsv5;              // [5] future use
    F32 FilteredData;      // [6] 필터링된 데이터, 단위는 상황에 따라 다름 (예: °C, % 등)
    F32 CPU_Temperature;   // [7] [°C] CPU 온도
    F32 BOARD_Temperature; // [8] [°C] 보드 온도
    F32 SL_Size;           // [9] SL 사이즈, [note] 유지보수 때문에  float 으로 정의
    tsXStateList SL;       // [10] SL, 사이즈 가변
    F32 Time;              // [11] [sec] 시스템 가동 후 시간 = gTriggerTime
    /**──────────────────────────────── 여기까지 기본 포멧 [수정하지 말것!~] */
    tsXCD_Decapper 		Decapper;			//Command 시 특정 값에 따른 동작 수행
    xCDecapping_Grip	CT;					//CT CAP/Body Grip ON/OFF
    tsxCDcap xCDecap;

} tsXControlData;

typedef struct
{
    tsXPL_Header Header;
    /**──────────────────────────────── 여기까지 기본 포멧 [수정하지 말것!~] */
    tsXPL_Decapper Decapper;				//Capping, Decapping에 관한 파라미터
} tsXParameterList;

#pragma pack(pop)

extern tsXStateList xSL;
extern tsXControlData xCD;
extern tsXParameterList xPL;
extern teXActionType xAT;
extern SystemInfo_t SystemInfo;

void SystemDB_Initialize(void);

// []. 부팅시 초기화 부분
void SystemDB_SL_Init(void); // 부팅시 초기화 해야 하는 부분 처리
void SystemDB_CD_Init(void); // 부팅시 초기화 해야 하는 부분 처리
void SystemDB_PL_Init(void); // 부팅할때 마다 EEPROM 에서 읽어와 초기화해야 하는 부분 처리

// []. factory 셋팅에 사용된는 부분
void SystemDB_PL_Init_Factory(tsXParameterList *pl); // EEPROM 에 저장한 factory setting 값 셋팅

// []. 모니터링 프로그램과의 통신 부분
void SystemDB_SL_Push(void);
void SystemDB_CD_Push(void);
void SystemDB_PL_Push(void);

#endif /* __XSYSTEM_DB_H__ */
