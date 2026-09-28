/*
 * CDCap.h
 *
 *  Created on: 2026. 6. 18.
 *      Author: RND
 */

#ifndef   __CDECAP_H__
#define   __CDECAP_H__

#include <stdbool.h>
#include <stdint.h>
#include "XGlobal.h"   // U08, U32, S32, F32 등 기본 타입
#include "TMC2660.h"   // STEP_CH_MAX, sMotionStatus_t, TMC2660_GetMotorRun
#include "_01_XSystemManagement.h"

#ifdef __CDCAP_C__
	#define CDCEAP_EXT
#else
	#define CDECAP_EXT extern
#endif

// 모터 파라미터 관련 (pk223)
#define T_MOTOR_ACC				300000
#define T_MOTOR_VEL				20000
/*
#define MOTOR_ACC (800000)
#define MOTOR_SPD_SAFE (70000) // 모터 속도 안전 마진
#define MOTOR_SPD_XRECOV_SAFE (40000)
 * */
#define MOTOR_ACC				300000
#define MOTOR_SPD_SAFE			20000
#define MOTOR_SPD_XRECOV_SAFE 	40000
#define R_MOTOR_POS				20000

#define ZDECAP_ACC				300000
#define ZDECAP_VEL				20000

//TEST 하여 수정
#define ZCAP_CAP_UP_POS				MM2PULSE_R * 2
#define ZCAP_CAP_SIDE_POS			MM2PULSE_R * 2
#define ZCAP_ORIGIN_POSITION	0

#define RDECAP_ACC				300000
#define RDECAP_VEL				20000
#define RDECAP_POS				MM2PULSE_R * 2

#define MOTOR_SPD_HOME_FAST (20000) // 모터 속도 빠르게
#define MOTOR_SPD_HOME_SLOW (3000)  // 모터 속도 느리게

/* 실제 기구물의 가동 범위에 따른 mm 수정*/
#define Z_TOTAL_mm 50
#define Z_Limmit_mm (MM2PULSE_Z * Z_TOTAL_mm) //

#define R_TOTAL_mm 50
#define R_Limmit_mm (MM2PULSE_R * R_TOTAL_mm) //

// 1.8도 기준 1CYCLE 3200 --> PKp223 0.05도 *36
#define PKP223	36
#define PK266	1
#define PK235	1

#define MM2PULSE_Z (3200 * PK235) //1mm 기준
#define MM2PULSE_R (3200 * PKP223) //1mm 기준

//티칭 포인트==============================================================================
#define CDECAPPOS_Z_CAP_TOP 5 //ex 32
#define CDECAPPOS_Z_MOVE_TOP	MM2PULSE * CDECAPPOS_Z_CAP_TOP

#define CDECAPPOS_Z_CAP_SIDE 5 //ex 32
#define CDECAPPOS_Z_MOVE_SIDE 	MM2PULSE * CDECAPPOS_Z_CAP_SIDE

//==============================================================================

// 축 설정
#define TOP (-1)
#define BOTTOM (1)
#define FRONT (-1)
#define BACK (1)

#define ZH (2)  // Z+: 모터 반대편
#define ZL (3)  // Z-: 모터 있는 쪽

//Master
#define aZ (0) // Z축 모터 채널
#define aR (1) // Rotate

#define DECAPPING (1)
#define CAPPING (2)

#define DEBUG_SL_SIZE (10)
#define DEBUG_CD_SIZE (10)
#define DEBUG_PL_SIZE (10)

#define SEC_TO_TICKS(sec) ((U32)((sec) * 1000.0f / (float)FW_TICK_MS))
#define _TO_TICKS(ms) ((U32)((ms) * 1000.0f / (float)FW_TICK_MS))
#define FW_TICK_MS (10)

#define HOMING_TIMEOUT_MS		(5000U)
#define CAPPING_TIMEOUT_MS		(5000U)
#define DECAPPING_TIMEOUT_MS	(5000U)
#define READY_TIMEOUT_MS   		(5000U)


#define IS_STARTED(idx) (sStartedFlags & (1UL << (idx)))
#define SET_STARTED(idx) (sStartedFlags |= (1UL << (idx)))
#define CLR_STARTED(idx) (sStartedFlags &= ~(1UL << (idx)))

#define IS_DONE(idx) (sDoneFlags & (1UL << (idx)))
#define SET_DONE(idx) (sDoneFlags |= (1UL << (idx)))
#define CLR_DONE(idx) (sDoneFlags &= ~(1UL << (idx)))

/* Physical input numbers from IO List HEM_CT_Handler.xlsx (1-based sheet number - 1). */
#define CT_CAP_GRIP_OPEN_SENSOR_PIN          (0U)
#define CT_CAP_GRIP_CLOSE_SENSOR_PIN         (1U)
#define CT_BODY_MIDDLE_GRIP_OPEN_SENSOR_PIN  (2U)
#define CT_BODY_MIDDLE_GRIP_CLOSE_SENSOR_PIN (3U)
#define CT_BODY_TOP_GRIP_OPEN_SENSOR_PIN     (4U)
#define CT_BODY_TOP_GRIP_CLOSE_SENSOR_PIN    (5U)
#define Z2_H_LIMIT_SENSOR_PIN                (6U)
#define Z2_L_LIMIT_SENSOR_PIN                (7U)
#define Z_L_LIMIT_SENSOR_PIN                 (8U)
#define Y_H_LIMIT_SENSOR_PIN                 (9U)
#define Y_L_LIMIT_SENSOR_PIN                 (10U)

/*Y축 공압기*/
#define Y_PIN_H_CONTROLLER		(3)
#define Y_PIN_L_CONTROLLER		(4)

/*CT*/
#define AIR_CT_BODY_GRIP_PIN	(0)
#define AIR_CT_CAP_GRIP_PIN		(1)

typedef void (*FSM_Handler)(void);

/*ACTION */
typedef enum{
    ACTION_NONE = 0,
	/*User Command*/
    ACTION_STOP,
	ACTION_HOME,

	ACTION_DECAP,
	ACTION_CAP,

	ACTION_PAUSE,
	ACTION_RESUME,

	/*Debug Command*/
	ACTION_ORIGIN,

	ACTION_RMOVEZ,
	ACTION_AMOVEZ,
	ACTION_RROTATE,
	ACTION_AROTATE,

	ACTION_Y_H_MOVE,
	ACTION_Y_L_MOVE,

	ACTION_UDECAP,
	ACTION_UCAP,
	ACTION_LONGRUN,

	ACTION_BODY_GRIP,
	ACTION_CAP_GRIP,

    ACTION_MAX 		// 항상 마지막에 추가
} teXActionType;

typedef enum{
    DFSM_IDLE = 0,

	/*User Command*/
	DFSM_STOP,
	DFSM_HOME,

	DFSM_DECAP,
    DFSM_CAP,

	DFSM_PAUSE,
	DFSM_RESUME,

	/*Debug Command*/
	DFSM_ORIGIN,

	DFSM_RROTATE,
	DFSM_AROTATE,

    DFSM_RMOVEZ,
    DFSM_AMOVEZ,

	DFSM_Y_H_MOVE,
	DFSM_Y_L_MOVE,

	DFSM_UDECAP,
	DFSM_UCAP,
	DFSM_LONGRUN,

	DFSM_CT_BODY_GRIP,
	DFSM_CT_CAP_GRIP,

	DFSM_ABNORMAL,
    DFSM_END_OK,
    DFSM_PHASE_MAX
} teFSM_Decapper;

typedef enum{
    CDECAP_HOMING_NONE = 0,
	CDECAP_HOMING_START,
	CDECAP_HOMING_H_SEARCH,
	CDECAP_HOMING_H_REACH,
	CDECAP_HOMING_H_WAIT,
	CDECAP_HOMING_L_SEARCH,
	CDECAP_HOMING_L_REACH_OFF,
	CDECAP_HOMING_L_WAIT,
	CDECAP_HOMING_S_SEARCH,
	CDECAP_HOMING_S_REACH,
	CDECAP_HOMING_S_WAIT,
	CDECAP_HOMING_Z_DONE,
	CDECAP_HOMING_Y_HOME,
	CDECAP_HOMING_Y_WAIT,
	CDECAP_HOMING_COMPLETE,
}CDecap_HomingStep_t;

typedef struct{
	CDecap_HomingStep_t Step;
    int Dir;

    U32 H_Acc;
    S32 H_Vel;
    U32 L_Acc;
    S32 L_Vel;

} sHomingSequence_t;

typedef struct{
    struct{
        int HomingRun;
        int HomingComplete;
        int Run;
        int Dir;
        int PositiveLimit;
        int NegativeLimit;
        int HomeSensor;
    } Bit;
} sDriveStatus_t;


typedef struct{
    int isSWLimit;
} SystemInfo_t;

typedef enum{
    LONGRUN_DECAP = 0,
    LONGRUN_CAP
} teLongRunPhase;

typedef struct{
    S32 Motor_CurPos;
    S32 Motor_TargetPos[2];

    teLongRunPhase phaseDecapCap;
    U32 LongRunCount;
    U08 SpeedPercent;

    SystemInfo_t SystemInfo;

    int debug[DEBUG_CD_SIZE];
    S32 chMotor;

} tsXCD_Decapper;

typedef struct{
    F32 RunCur[2];
    int SelMaxCur[2];
    int StopCurRate[2];
    int StepResolution;
    //int PowerEnable[2];

    //Z,Y,R Limit POS 지정
    S32 Limit_PosZ;
    S32 Limit_PosR;

    //하드웨어적 최대 이동 위치
    S32 SwNegLimit[STEP_CH_MAX];
    S32 SwPosLimit[STEP_CH_MAX];
    U8 SoftLimitEnable;

    //Cap_Decap시 필요한 PL --> DB에 상세히 기록
    U32 ZDecapAcc;
    S32 ZDecapVel;
    S32 ZCap_UpPos;
    S32 ZCap_SidePos;
    S32 ZCap_Origin_Position;

    U32 RDecapAcc;
    S32 RDecapVel;
    S32 RDecapPos;

    int ForDebug[DEBUG_PL_SIZE];

} tsXPL_Decapper;

typedef struct{
	/* Physical sensor inputs */
	unsigned char CT_Cap_Grip_Open;
	unsigned char CT_Cap_Grip_Close;
	unsigned char CT_Body_Middle_Grip_Open;
	unsigned char CT_Body_Middle_Grip_Close;
	unsigned char CT_Body_Top_Grip_Open;
	unsigned char CT_Body_Top_Grip_Close;
	unsigned char Z2_H_Limit;
	unsigned char Z2_L_Limit;
	unsigned char Z_L_Limit;
	unsigned char Y_H_Limit;
	unsigned char Y_L_Limit;

	bool Cap_is;
    bool Body_is;

    unsigned char Z2_HL_isError;
    unsigned char Y_HL_isError;
    unsigned char CT_Cap_Grip_isError;
    unsigned char CT_Body_Middle_Grip_isError;
    unsigned char CT_Body_Top_Grip_isError;

    //int ForDebug[DEBUG_SL_SIZE];

    //sMotionStatus_t MotionStatus[STEP_CH_MAX];

} tsXSL_Decapper;

typedef struct{

	void (*Senser_Update)(void);
    void (*Status_Update)(void);
    void (*CT_Cap_Body_Update)(void);

    void (*CheckSensorValidity)(void);

    void (*Action_Fnc)(teXActionType actionType);       //
} tsxCDcap;

typedef enum{
    SUB_IDLE = 0,
    SUB_zRMOVE,
	SUB_rRMOVE,
	SUB_zAMOVE,
	SUB_rAMOVE,
	SUB_yHMOVE,
	SUB_yLMOVE,
	SUB_HOMEZ,
	SUB_HOMEY,
	SUB_sMOVE,
	SUB_BODY_GRIP_WAIT,
	SUB_CAPPING,
	SUB_DECAPPING,
    SUB_ACTION_COUNT,
	SUB_MAX
} teDecapperSubIndex;

typedef enum{
    MOVE_IDLE = 0,
    MOVING,
    MOVE_DONE
} move_Step_t;

typedef enum{
	Recovery__IDLE = 0,
	Recovery_Z,
	Recovery_Y,
	Recovery_COMPLETE,
	Recovery__DONE
} Recovery_Step_t;

typedef enum{
	CDecapping_Idle = 0,
	CDecapping_Body_Grip,
	CDecapping_Move_Pos,

	CDecapping_Capping_Ready,
	CDecapping_Decapping_Ready,

	CDecapping_Cap_Capgrip,

	CDecapping_sData_Capping,
	CDecapping_sData_Decapping,

	CDecapping_Move_PrevR,
	CDecapping_Move_PrevZ,
	CDecapping_Move_PrevY,
	CDecapping_Complete_Wait,
	CDecapping_Done,

} CDecapping_Statmachin_state;

typedef enum{
	Ready_Idle= 0,
	Ready_Y_L,
	Ready_Done,
}Ready_Statemachin_state;

typedef struct{
	unsigned char Motor_Run;
} xSLecapping_MotorRun;

typedef struct{
	unsigned char Cap;
	unsigned char Body;
} xCDecapping_Grip;

typedef enum{
    PAUSE_Y_NONE = 0,
    PAUSE_Y_HIGH,
    PAUSE_Y_LOW
} Pause_Y_Direction_t;

typedef struct{
    bool isPaused;
    bool resumeZ;
    bool resumeR;
    bool resumeY;

    S32 zTargetPos;
    S32 rTargetPos;

    U32 zAcc;
    S32 zVel;

    U32 rAcc;
    S32 rVel;

    Pause_Y_Direction_t yDirection;
    teFSM_Decapper savedPhase;
    U32 pauseStartTick;

} Pause_Context_t;

extern tsxCDcap xCDecap;

//Initialize
void CDecap_Error_Clear(void);
void CDecap_Init(void);
//Update SigDate============================================
void CT_Handler_Sensor_Update(void);
void CDecap_M_S_Detect(void);
void YZ_Motor_Status_Update(void);
void Cap_CAP_Body_Detect_Sensor(void);
//Diagnose==================================================
void CDecapping_CheckSensorValidity(void);
//Application===============================================
void CDecap_Action_Statemachine(teXActionType actionType);
void CDecap_SetBodyGripCommand(U08 onoff);
void CDecap_SetCapGripCommand(U08 onoff);
S32 CDecap_GetZPosition(void);
void CDecap_SetSpeedPercent(U08 percent);
U08 CDecap_GetSpeedPercent(void);
#endif /* APP_HW_DEVICE_CDCAP_H_ */
