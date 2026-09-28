/*
 * CDecap.c
 *
 *  Created on: 2026. 6. 18.
 *      Author: RND
 */
#include <CT_Handler.h>
#include "Drive.h"
#include "TMC2660.h"
#include "XDebug.h"
#include "XSystemInfo.h"
#include "XSystem_DB.h"
#include "_01_XSystemManagement.h"
#include "_04_XDiagnose.h"
#include "TMC429.h"
#include "stm32f7xx_hal.h"
#include "_02_XUpdateSIGData.h"

teDecapperSubIndex subIdx;
tsxCDcap xCDecap;
sHomingSequence_t HomingSeq;
CDecapping_Statmachin_state CD_State_Flag;
Ready_Statemachin_state Read_Step;

unsigned char CDecapping_Step = 0;

static teFSM_Decapper sPhase = DFSM_IDLE; 		/* 현재 FSM Phase */
static teFSM_Decapper sResumePhase = DFSM_IDLE; /* END_OK 후 복귀할 Phase */

static U32 sStartedFlags = 0; 					/* 비트마스크: 시작 플래그 */
static U32 sDoneFlags = 0; 						/* 비트마스크: 완료 플래그 */

static const U32 Wait_1s = SEC_TO_TICKS(1);
static U08 sSubPhase[SUB_MAX] = { 0 }; 			/* 서브페이즈 (step 값) */
static U32 previousTime[SUB_MAX] = { 0 }; 		/* 서브페이즈별 타이머 */

static Recovery_Step_t Origin_Step = Recovery__IDLE;
static U32 Step_StartTick = 0;

static Pause_Context_t PauseContext;

bool sHome_end = false;
bool BGrip_Flag,CGrip_Flag=false;

static void CDecap_StartTimeout(void);
static int CDecap_CheckTimeout(U32 timeout_ms);

static void CDecap_Idle(void);

static void Air_Y_High(void);
static void Air_Y_Low(void);

static void CDecap_Relmove(unsigned char Idx, unsigned char ch, unsigned int  Acc, unsigned int Vel, S32 Pos, S32 limit);
static void CDecap_Absmove(unsigned char Idx, unsigned char ch, unsigned int  Acc, unsigned int Vel, S32 Pos, S32 limit);
static void CD_ABSMove(unsigned char ch, unsigned int Acc, unsigned int Vel, S32 Pos, S32 limit);
static S32 CDecap_ScaleVelocity(S32 velocity);

/* 자동 동작 액션 */
static void CDecap_Homing(void);
static void Decapper_StopAll(void);

static void CDecap_Capping(void);
static void CDecap_Decapping(void);

static void CDecap_Pause(void);
static void CDecap_Resume(void);

/* 수동 제어 액션 */
static void CDecap_Stop(unsigned char ch);

static void CDecap_Origin(void);

static void CDecap_Absmove_Z(void);
static void CDecap_Relmove_Z(void);
static void CDecap_RRotate(void);
static void CDecap_ARotate(void);

static void Air_Y_High_Move(void);
static void Air_Y_Low_Move(void);
static void Air_Y_StopOutput(void);
static void Air_Y_Stop(void);

static void UCDecap_Capping(void);
static void UCDecap_Decapping(void);
static void UCDecap_CDecap_Longrun(void);

static void Air_CTBody_Gripper(unsigned char onoff);
static void Air_CTCap_Gripper(unsigned char onoff);

/* FSM 종료 및 비정상 처리 보조 함수 */
static void FSM_End_ok(void);
static void FSM_Abnormal(void);
static void ResetAllSubPhases(void);
static void ResetPauseContext(void);

/* 초기화 */

void CDecap_Init(void){
	sMotionLimit_t MotionLimit;
	sMotionSwLimitPos_t MotionSwLimitPos;

	memset((char*) &xCDecap, 0x00, sizeof(xCDecap));
	memset((char*) &HomingSeq, 0x00, sizeof(HomingSeq));
	xCD.Decapper.SpeedPercent = 100U;

	/* 상태 갱신 콜백 등록 */
	xCDecap.Senser_Update /*            */= CT_Handler_Sensor_Update;
	xCDecap.Status_Update /*            */= YZ_Motor_Status_Update;
	xCDecap.CT_Cap_Body_Update /*       */= Cap_CAP_Body_Detect_Sensor;
	/* 센서 진단 콜백 등록 */
	xCDecap.CheckSensorValidity /*      */=	CDecapping_CheckSensorValidity;
	/* 디캐퍼 상태 머신 콜백 등록 */
	xCDecap.Action_Fnc /*             	*/= CDecap_Action_Statemachine;
	/* 상태 머신 컨텍스트 초기화 */
	sPhase = DFSM_IDLE;
	sResumePhase = DFSM_IDLE;
	subIdx = SUB_IDLE;
	CD_State_Flag = CDecapping_Idle;

	/* Z/R 모터의 리미트, 전류, 분해능 설정 */
	MotionLimit.EnableNegLimit = false;
	MotionLimit.PolarityNegLimit = false;
	MotionLimit.EnablePosLimit = false;
	MotionLimit.PolarityPosLimit = false;
	MotionLimit.EnableSoftLimit = (bool) xPL.Decapper.SoftLimitEnable;

	for (U08 ch = 0; ch < STEP_CH_MAX; ch++){
		MotionSwLimitPos.SwNegLimit = xPL.Decapper.SwNegLimit[ch];
		MotionSwLimitPos.SwPosLimit = xPL.Decapper.SwPosLimit[ch];

		Drive_SelMaxCurrent(ch, xPL.Decapper.SelMaxCur[ch]);
		Drive_SetCurrent(ch, xPL.Decapper.RunCur[ch], xPL.Decapper.StopCurRate[ch]);

		Drive_SetResoultion(ch, xPL.Decapper.StepResolution);

		Drive_SetHwLimit(ch, MotionLimit);
		Drive_SetSwLimitPos(ch, MotionSwLimitPos);
		Drive_PowerEnable(ch, ENABLE);

		TMC429_SetPosition(ch, 0); /* HOMING에서 기준 위치를 다시 설정한다. */
	}
	/* 공압 그리퍼 초기 상태 */
	Air_CTBody_Gripper(OFF);
	Air_CTCap_Gripper(OFF);

    ResetAllSubPhases();
    sPhase = DFSM_IDLE;
    sResumePhase = DFSM_IDLE;
    xSL.isHomed = false;
    ResetPauseContext();
}
/* IO List의 물리 입력을 가공하지 않고 현재 상태에 기록한다. */
void CT_Handler_Sensor_Update(void) {
	xSL.Decapper.CT_Cap_Grip_Open = (IOEXP_ReadIObit(READ_IN, CT_CAP_GRIP_OPEN_SENSOR_PIN) == GPIO_PIN_SET) ? 1U : 0U;
	xSL.Decapper.CT_Cap_Grip_Close = (IOEXP_ReadIObit(READ_IN, CT_CAP_GRIP_CLOSE_SENSOR_PIN) == GPIO_PIN_SET) ? 1U : 0U;
	xSL.Decapper.CT_Body_Middle_Grip_Open = (IOEXP_ReadIObit(READ_IN, CT_BODY_MIDDLE_GRIP_OPEN_SENSOR_PIN) == GPIO_PIN_SET) ? 1U : 0U;
	xSL.Decapper.CT_Body_Middle_Grip_Close = (IOEXP_ReadIObit(READ_IN, CT_BODY_MIDDLE_GRIP_CLOSE_SENSOR_PIN) == GPIO_PIN_SET) ? 1U : 0U;
	xSL.Decapper.CT_Body_Top_Grip_Open = (IOEXP_ReadIObit(READ_IN, CT_BODY_TOP_GRIP_OPEN_SENSOR_PIN) == GPIO_PIN_SET) ? 1U : 0U;
	xSL.Decapper.CT_Body_Top_Grip_Close = (IOEXP_ReadIObit(READ_IN, CT_BODY_TOP_GRIP_CLOSE_SENSOR_PIN) == GPIO_PIN_SET) ? 1U : 0U;
	xSL.Decapper.Z2_H_Limit = (IOEXP_ReadIObit(READ_IN, Z2_H_LIMIT_SENSOR_PIN) == GPIO_PIN_SET) ? 1U : 0U;
	xSL.Decapper.Z2_L_Limit = (IOEXP_ReadIObit(READ_IN, Z2_L_LIMIT_SENSOR_PIN) == GPIO_PIN_SET) ? 1U : 0U;
	xSL.Decapper.Z_L_Limit = (IOEXP_ReadIObit(READ_IN, Z_L_LIMIT_SENSOR_PIN) == GPIO_PIN_SET) ? 1U : 0U;
	xSL.Decapper.Y_H_Limit = (IOEXP_ReadIObit(READ_IN, Y_H_LIMIT_SENSOR_PIN) == GPIO_PIN_SET) ? 1U : 0U;
	xSL.Decapper.Y_L_Limit = (IOEXP_ReadIObit(READ_IN, Y_L_LIMIT_SENSOR_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}
/* Z축과 회전축의 구동 상태를 갱신한다. */
void YZ_Motor_Status_Update(void){
	/* Z축 및 회전축 모터의 동작 여부 */
	xSL.xZ_Motor_Run.Motor_Run = (TMC2660_GetMotorRun(aZ) == 1) ? 1 : 0;
	xSL.xR_Motor_Run.Motor_Run = (TMC2660_GetMotorRun(aR) == 1) ? 1 : 0;
}
/* 그리퍼 센서로부터 Body/Cap 보유 상태를 갱신한다. */
void Cap_CAP_Body_Detect_Sensor(void){
	//CT를 잡고 있는지
	if(xSL.Decapper.CT_Body_Middle_Grip_Open == false
			&& xSL.Decapper.CT_Body_Middle_Grip_Close == false){
		xSL.Decapper.Body_is = true;
	}
	else {
		xSL.Decapper.Body_is = false;
	}
	//현재 Cap을 가지고 있는지 확인
	if(xSL.Decapper.CT_Cap_Grip_Open == false
			&& xSL.Decapper.CT_Cap_Grip_Close == false){
		xSL.Decapper.Cap_is = true;
	}
	else{
		xSL.Decapper.Cap_is = false;
	}
}
/* 센서 진단 */
void CDecapping_CheckSensorValidity(void){
	/* 서로 배타적인 리미트/그리퍼 위치가 동시에 감지되면 오류로 래치한다. */
	if (xSL.Decapper.Z2_H_Limit && xSL.Decapper.Z2_L_Limit){
		xSL.Decapper.Z2_HL_isError = true;
	}
	if (xSL.Decapper.Y_H_Limit && xSL.Decapper.Y_L_Limit){
		xSL.Decapper.Y_HL_isError = true;
	}
	if (xSL.Decapper.CT_Cap_Grip_Open && xSL.Decapper.CT_Cap_Grip_Close){
		xSL.Decapper.CT_Cap_Grip_isError = true;
	}
	if (xSL.Decapper.CT_Body_Middle_Grip_Open && xSL.Decapper.CT_Body_Middle_Grip_Close){
		xSL.Decapper.CT_Body_Middle_Grip_isError = true;
	}
	if (xSL.Decapper.CT_Body_Top_Grip_Open && xSL.Decapper.CT_Body_Top_Grip_Close){
		xSL.Decapper.CT_Body_Top_Grip_isError = true;
	}
}
/* 하드웨어 출력 보조 함수 */
/* CT Body 그리퍼 출력 */
static void Air_CTBody_Gripper(unsigned char onoff){
	IOEXP_WriteIObit(AIR_CT_BODY_GRIP_PIN, onoff);
}
/* CT Cap 그리퍼 출력 */
static void Air_CTCap_Gripper(unsigned char onoff){
	IOEXP_WriteIObit(AIR_CT_CAP_GRIP_PIN, onoff);
}

void CDecap_SetBodyGripCommand(U08 onoff){
	xCD.CT.Body = onoff;
	BGrip_Flag = true;
}

void CDecap_SetCapGripCommand(U08 onoff){
	xCD.CT.Cap = onoff;
	CGrip_Flag = true;
}

/* 전체 속도 비율(1~100%)을 저장한다. */
void CDecap_SetSpeedPercent(U08 percent){
	if ((percent >= 1U) && (percent <= 100U)){
		xCD.Decapper.SpeedPercent = percent;
	}
}

U08 CDecap_GetSpeedPercent(void){
	return xCD.Decapper.SpeedPercent;
}

/* 방향을 유지한 채 설정된 전체 속도 비율을 적용한다. */
static S32 CDecap_ScaleVelocity(S32 velocity){
	S32 scaled = (velocity * (S32)xCD.Decapper.SpeedPercent) / 100;

	if ((velocity > 0) && (scaled == 0)) return 1;
	if ((velocity < 0) && (scaled == 0)) return -1;

	return scaled;
}
/* 공압 Y축을 High 방향으로 구동한다. */
static void Air_Y_High(void){
	PauseContext.yDirection = PAUSE_Y_HIGH;
	IOEXP_WriteIObit(Y_PIN_H_CONTROLLER, ON);
	IOEXP_WriteIObit(Y_PIN_L_CONTROLLER, OFF);
}
/* 공압 Y축을 Low 방향으로 구동한다. */
static void Air_Y_Low(void){
	PauseContext.yDirection = PAUSE_Y_LOW;
	IOEXP_WriteIObit(Y_PIN_H_CONTROLLER, OFF);
	IOEXP_WriteIObit(Y_PIN_L_CONTROLLER, ON);
}
/* Y축 출력만 정지한다. PAUSE 중에는 저장된 방향을 유지한다. */
static void Air_Y_StopOutput(void){
	IOEXP_WriteIObit(Y_PIN_H_CONTROLLER, ON);
	IOEXP_WriteIObit(Y_PIN_L_CONTROLLER, ON);
}
/* 공압 Y축을 정지하고 저장된 방향을 초기화한다. */
static void Air_Y_Stop(void){
	Air_Y_StopOutput();
	PauseContext.yDirection = PAUSE_Y_NONE;
}
/* 공압 Y축을 High 리미트까지 이동한다. */
static void Air_Y_High_Move(void){
	if (!IS_STARTED(SUB_yHMOVE)){
		SET_STARTED(SUB_yHMOVE);

		previousTime[SUB_yHMOVE] = gTriggerCount;
		sSubPhase[SUB_yHMOVE] = MOVING;
	}

	switch (sSubPhase[SUB_yHMOVE]){
		case MOVING:
			Air_Y_High();
			sSubPhase[SUB_yHMOVE] = MOVE_DONE;
		break;
		case MOVE_DONE:
			if (!xSL.Decapper.Y_H_Limit){
				Air_Y_Stop();
				sSubPhase[SUB_yHMOVE] = MOVE_IDLE;
				SET_DONE(SUB_yHMOVE);
			}
		break;
	default:
		break;
	}

	if (IS_DONE(SUB_yHMOVE)){
		sPhase = DFSM_END_OK;
	}
}
/* 공압 Y축을 Low 리미트까지 이동한다. */
static void Air_Y_Low_Move(void){
	if (!IS_STARTED(SUB_yLMOVE)){
		SET_STARTED(SUB_yLMOVE);

		previousTime[SUB_yLMOVE] = gTriggerCount;
		sSubPhase[SUB_yLMOVE] = MOVING;
	}

	switch (sSubPhase[SUB_yLMOVE]){
		case MOVING:
			Air_Y_Low();
			sSubPhase[SUB_yLMOVE] = MOVE_DONE;
		break;
		case MOVE_DONE:
			if (!xSL.Decapper.Y_L_Limit){
				Air_Y_Stop();
				sSubPhase[SUB_yLMOVE] = MOVE_IDLE;
				SET_DONE(SUB_yLMOVE);
			}
		break;
	default:
		break;
	}

	if (IS_DONE(SUB_yLMOVE)){
		sPhase = DFSM_END_OK;
	}
}

/* 현재 단계의 타임아웃 측정을 시작한다. */
static void CDecap_StartTimeout(void){
	Step_StartTick = gTriggerCount;
}
/* 타임아웃 시 모든 동작을 멈추고 비정상 상태로 전환한다. */
static int CDecap_CheckTimeout(U32 timeout_ms){
	if ((gTriggerCount - Step_StartTick) >= (timeout_ms / 10)) {

		LOG_MSG_SEND("Decapper timeout");
		Decapper_StopAll();
		xSL.isError = YES;
		xAT = ACTION_NONE;
		sPhase = DFSM_ABNORMAL;

		return 1;
    }
    return 0;
}

/* 래치된 디캐퍼 오류 상태를 초기화한다. */
void CDecap_Error_Clear(void){
	xSL.isError = false;
	xSL.Decapper.CT_Body_Middle_Grip_isError = false;
	xSL.Decapper.CT_Body_Top_Grip_isError = false;
	xSL.Decapper.CT_Cap_Grip_isError = false;
	xSL.Decapper.Y_HL_isError = false;
	xSL.Decapper.Z2_HL_isError = false;
}

/* Z축 상대 이동 */
static void CDecap_Relmove_Z(void){
	CDecap_Relmove(SUB_zRMOVE,aZ, T_MOTOR_ACC, T_MOTOR_VEL, xCD.Decapper.Motor_TargetPos[aZ], xPL.Decapper.Limit_PosZ);
}
/* Z축 또는 회전축의 공통 상대 이동 */
static void CDecap_Relmove(unsigned char Idx,unsigned char ch, unsigned int Acc, unsigned int Vel, S32 Pos, S32 limit){
	if (!IS_STARTED(Idx)){
		SET_STARTED(Idx);

		previousTime[Idx] = gTriggerCount;
		sSubPhase[Idx] = MOVING;
		sMotionCommand_t MotionCmd;
		S32 Clamp = 0;

		if(Pos > limit){
			Clamp = limit;
		}

		else if(Pos < -limit){
			Clamp = -limit;
		}
		else Clamp = Pos;

		MotionCmd.Axis = ch;
		MotionCmd.Mode = MODE_REL;
		MotionCmd.Acc = Acc;
		MotionCmd.Vel = CDecap_ScaleVelocity((S32)Vel);
		MotionCmd.Pos = Clamp;
		Drive_RelMove(&MotionCmd);
	}

	switch (sSubPhase[Idx]){
		case MOVING:
			if((((xSL.xZ_Motor_Run.Motor_Run)||(xSL.xR_Motor_Run.Motor_Run)) != TRUE) && ((gTriggerCount - previousTime[Idx]) > Wait_1s)){
				sSubPhase[Idx] = MOVE_DONE;
			}
		break;
	case MOVE_DONE:
		CDecap_Stop(ch);
		sSubPhase[Idx] = MOVE_IDLE;
		SET_DONE(Idx);
		break;
	default:
		break;
	}

	if (IS_DONE(Idx)){
		sPhase = DFSM_END_OK;
	}
}
/* Z축 절대 이동 */
static void CDecap_Absmove_Z(void){
	CDecap_Absmove(SUB_zAMOVE,aZ, T_MOTOR_ACC, T_MOTOR_VEL, xCD.Decapper.Motor_TargetPos[aZ], Z_Limmit_mm);
}
/* Z축 또는 회전축의 공통 절대 이동 */
static void CDecap_Absmove(unsigned char Idx, unsigned char ch, unsigned int Acc, unsigned int Vel, S32 Pos, S32 limit) {
	if (!IS_STARTED(Idx)) {
		SET_STARTED(Idx);

		previousTime[Idx] = gTriggerCount;
		sSubPhase[Idx] = MOVING;

		sMotionCommand_t MotionCmd;
		S32 clamped;

		if(Pos > limit)			clamped = limit;
		else if(Pos < -limit)	clamped = -limit;
		else					clamped = Pos;

		MotionCmd.Axis = ch;
		MotionCmd.Mode = MODE_ABS;
		MotionCmd.Acc = Acc;
		MotionCmd.Vel = CDecap_ScaleVelocity((S32)Vel);
		MotionCmd.Pos = clamped;
		Drive_AbsMove(&MotionCmd);
	}

	switch (sSubPhase[Idx]){
		case MOVING:
			if((((xSL.xZ_Motor_Run.Motor_Run)||(xSL.xR_Motor_Run.Motor_Run)) != TRUE) && ((gTriggerCount - previousTime[Idx]) > Wait_1s)){
							sSubPhase[Idx] = MOVE_DONE;
			}
			break;
		case MOVE_DONE:
			CDecap_Stop(ch);
			sSubPhase[Idx] = MOVE_IDLE;
			SET_DONE(Idx);
			break;
		default:
			break;
		}

		if (IS_DONE(Idx)) {
			sPhase = DFSM_END_OK;
		}
}
/* Capping/Decapping용 회전축 이동 */
static void CDecap_RRotate(void){
	CDecap_Relmove(SUB_rRMOVE,aR, T_MOTOR_ACC, T_MOTOR_VEL, xCD.Decapper.Motor_TargetPos[aR], xPL.Decapper.Limit_PosR);
}

static void CDecap_ARotate(void){
	CDecap_Absmove(SUB_rAMOVE,aR, T_MOTOR_ACC, T_MOTOR_VEL, xCD.Decapper.Motor_TargetPos[aR], xPL.Decapper.Limit_PosR);
}
/* 동작하지 않는 대기 상태 */
static void CDecap_Idle(void){}
/* 지정 축 스테퍼 모터 정지 */
static void CDecap_Stop(unsigned char ch){
	sMotionCommand_t MotionCmd;
	MotionCmd.Axis = ch;
	MotionCmd.Mode = MODE_STOP;
	MotionCmd.Acc = MOTOR_ACC;
	MotionCmd.Vel = MOTOR_SPD_SAFE;
	MotionCmd.Pos = 0;
	Drive_Stop(&MotionCmd);
	TMC2660_SetHoldCurrent(ch);
}
/* 모든 모션을 정지하고 FSM 컨텍스트를 초기화한다. */
static void Decapper_StopAll(void){
	for (U08 ch = 0; ch < STEP_CH_MAX; ch++){
		CDecap_Stop(ch);
	}
	Air_Y_Stop();
	sPhase = DFSM_IDLE;
	sResumePhase = DFSM_IDLE;
	xSL.isBusy = NO;
	CDecapping_Step = CDecapping_Idle;
	xCD.Decapper.phaseDecapCap = LONGRUN_DECAP;
	xAT = ACTION_NONE;
	ResetAllSubPhases();
	ResetPauseContext();
}
/* 수동 Cap 그리퍼 제어 */
static void CT_Cap_Grip(void){
	Air_CTCap_Gripper(xCD.CT.Cap);

	if(CGrip_Flag == true){
		CGrip_Flag = false;
		sPhase = DFSM_END_OK;
	}
}
/* 수동 Body 그리퍼 제어 */
static void CT_Body_Grip(void){
	Air_CTBody_Gripper(xCD.CT.Body);

	if(BGrip_Flag == true){
		BGrip_Flag = false;
		sPhase = DFSM_END_OK;
	}
}
/* Z축을 홈 기준으로 설정한 뒤 공압 Y축을 High 리미트로 복귀한다. */
static void CDecap_Homing(void){
	sHomingSequence_t*   Homing;
	Homing = &HomingSeq;

	if (!IS_STARTED(SUB_HOMEZ)) {
		SET_STARTED(SUB_HOMEZ);
		if((CDECAP_HOMING_NONE < HomingSeq.Step) && (HomingSeq.Step < CDECAP_HOMING_COMPLETE)) { return; }
		/* 모터 속도는 기구 테스트 후 조정한다. */
		HomingSeq.H_Acc = MOTOR_ACC;
		/* TMC429_VelMove()는 실제 pulse/s가 아닌 내부 속도 단위를 받음 */
		HomingSeq.H_Vel = CDecap_ScaleVelocity(MOTOR_SPD_HOME_FAST / STEP_PULSE_RATE_R);
		HomingSeq.L_Acc = MOTOR_ACC;
		HomingSeq.L_Vel = -CDecap_ScaleVelocity(MOTOR_SPD_HOME_SLOW / STEP_PULSE_RATE_R);
		HomingSeq.Step = CDECAP_HOMING_START;
	}

	if(xSL.isHomed == true){
		sPhase = DFSM_END_OK;
		return;
	}

	switch(Homing->Step){
	    case CDECAP_HOMING_START:
	    	Homing->Step = CDECAP_HOMING_H_SEARCH;
	        break;
	    case CDECAP_HOMING_H_SEARCH:
	        TMC429_VelMove(aZ, Homing->H_Acc, Homing->H_Vel);
	        CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_H_REACH;
	        break;
	    case CDECAP_HOMING_H_REACH:
	        if (CDecap_CheckTimeout(HOMING_TIMEOUT_MS)){return;}
	    if(xSL.Decapper.Z2_H_Limit){ return; }
	    	TMC429_MotorStop(aZ, Homing->H_Acc);
	    	CDecap_StartTimeout();
	    	Homing->Step = CDECAP_HOMING_H_WAIT;
	        break;
	    case CDECAP_HOMING_H_WAIT:
	    	if (CDecap_CheckTimeout(HOMING_TIMEOUT_MS)){return;}
	        if(xSL.xZ_Motor_Run.Motor_Run) { return; }
	        Homing->Step = CDECAP_HOMING_L_SEARCH;
	        break;
	    case CDECAP_HOMING_L_SEARCH:
	    	/* Home sensor를 해제하기 위해 반대 방향으로 저속 후퇴 */
	    	TMC429_VelMove(aZ, Homing->L_Acc, Homing->L_Vel);
	    	CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_L_REACH_OFF;
	        break;
	    case CDECAP_HOMING_L_REACH_OFF:
	        /* 현재 프로젝트의 기준: 0 = Home 감지, 1 = 센서 해제 */
	        if (CDecap_CheckTimeout(HOMING_TIMEOUT_MS)){return;}
	        if(!xSL.Decapper.Z2_H_Limit) { return; }
	        TMC429_MotorStop(aZ, Homing->L_Acc);
	        CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_L_WAIT;
	        break;
	    case CDECAP_HOMING_L_WAIT:
	    	if (CDecap_CheckTimeout(HOMING_TIMEOUT_MS)){return;}
	        if(xSL.xZ_Motor_Run.Motor_Run) { return; }
	        Homing->Step = CDECAP_HOMING_S_SEARCH;
	        break;
	    case CDECAP_HOMING_S_SEARCH:
	        /* 해제 위치에서 원래 방향으로 저속 재접근 */
	        TMC429_VelMove(aZ, Homing->L_Acc, -Homing->L_Vel);
	        CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_S_REACH;
	        break;
	    case CDECAP_HOMING_S_REACH:
	        /* Home 센서가 다시 감지될 때까지 이동 */
	    	if (CDecap_CheckTimeout(HOMING_TIMEOUT_MS)){return;}
	        if(xSL.Decapper.Z2_H_Limit) { return; }
	        TMC429_MotorStop(aZ, Homing->L_Acc);
	        CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_S_WAIT;
	        break;
	    case CDECAP_HOMING_S_WAIT:
	    	if (CDecap_CheckTimeout(HOMING_TIMEOUT_MS)){return;}
	        if(xSL.xZ_Motor_Run.Motor_Run) { return; }
	        Homing->Step = CDECAP_HOMING_Z_DONE;
	        break;
	    case CDECAP_HOMING_Z_DONE:
	        TMC429_SetPosition(aZ, 0);
	        Homing->Step = CDECAP_HOMING_Y_HOME;
	        break;
	    case CDECAP_HOMING_Y_HOME:
	    	Air_Y_High();
	    	CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_Y_WAIT;
	        break;
	    case CDECAP_HOMING_Y_WAIT:
	    	if (CDecap_CheckTimeout(HOMING_TIMEOUT_MS)){return;}
	    if(!xSL.Decapper.Y_H_Limit){
	    		Air_Y_Stop();
	    		Homing->Step = CDECAP_HOMING_COMPLETE;
	    	}
	        break;
	    case CDECAP_HOMING_COMPLETE:
	    	Homing->Step = CDECAP_HOMING_NONE;
	    	xSL.isHomed = true;
	    	xprintf("Homing Complete.\n");
	    	SET_DONE(SUB_HOMEZ);
	    	SET_DONE(SUB_HOMEY);
	    	sPhase = DFSM_END_OK; /* 중앙 종료 처리로 전환 */
	    	Homing->Step = CDECAP_HOMING_NONE;
		    break;
	}
}

/* Pause/Resume에서 재사용할 Z/R 절대 이동 명령을 저장하고 실행한다. */
static void CD_ABSMove(unsigned char ch, unsigned int Acc, unsigned int Vel, S32 Pos, S32 limit){
	sMotionCommand_t MotionCmd;
	S32 clamped;

	if(Pos > limit)			clamped = limit;
	else if(Pos < -limit)	clamped = -limit;
	else					clamped = Pos;

    if(ch == aZ){
        PauseContext.zTargetPos = clamped;
        PauseContext.zAcc = Acc;
        PauseContext.zVel = Vel;
    }
    else if(ch == aR){
        PauseContext.rTargetPos = clamped;
        PauseContext.rAcc = Acc;
        PauseContext.rVel = Vel;
    }

	MotionCmd.Axis = ch;
	MotionCmd.Mode = MODE_ABS;
	MotionCmd.Acc = Acc;
	MotionCmd.Vel = CDecap_ScaleVelocity((S32)Vel);
	MotionCmd.Pos = clamped;
	Drive_AbsMove(&MotionCmd);
}
/* 진행 중인 Z/R/Y 동작 정보를 보존한 채 모션 출력을 정지한다. */
static void CDecap_Pause(void){
	if (PauseContext.isPaused){
		return;
	}

	/* 정지 전에 실제 이동 중이던 축과 Y 방향을 저장한다. */
	PauseContext.resumeZ = (xSL.xZ_Motor_Run.Motor_Run != 0U);
	PauseContext.resumeR = (xSL.xR_Motor_Run.Motor_Run != 0U);
	PauseContext.resumeY = (PauseContext.yDirection != PAUSE_Y_NONE);
	PauseContext.pauseStartTick = gTriggerCount;

	for (U08 ch = 0; ch < STEP_CH_MAX; ch++){
		CDecap_Stop(ch);
	}
	Air_Y_StopOutput();
	PauseContext.isPaused = true;
}

/* 저장된 Z/R 목표 위치와 Y 방향을 복원하여 동작을 재개한다. */
static void CDecap_Resume(void){
	teFSM_Decapper savedPhase = PauseContext.savedPhase;
	Pause_Y_Direction_t savedYDirection = PauseContext.yDirection;
	bool resumeZ = PauseContext.resumeZ;
	bool resumeR = PauseContext.resumeR;
	bool resumeY = PauseContext.resumeY;
	U32 pausedTicks = gTriggerCount - PauseContext.pauseStartTick;

	/* PAUSE 시간은 기존 timeout/dwell 경과시간에서 제외한다. */
	Step_StartTick += pausedTicks;
	for (U08 idx = 0; idx < SUB_MAX; idx++){
		previousTime[idx] += pausedTicks;
	}

	PauseContext.isPaused = false;
	PauseContext.resumeZ = false;
	PauseContext.resumeR = false;
	PauseContext.resumeY = false;
	PauseContext.savedPhase = DFSM_IDLE;
	PauseContext.pauseStartTick = 0U;

	if(resumeZ){
		CD_ABSMove(aZ,PauseContext.zAcc,PauseContext.zVel,PauseContext.zTargetPos, xPL.Decapper.Limit_PosZ);
	}

	if(resumeR){
		CD_ABSMove(aR,PauseContext.rAcc,PauseContext.rVel,PauseContext.rTargetPos, xPL.Decapper.Limit_PosR);
	}

	if(resumeY && savedYDirection == PAUSE_Y_HIGH){
		Air_Y_High();
	}

	if(resumeY && savedYDirection == PAUSE_Y_LOW){
		Air_Y_Low();
	}

	sPhase = savedPhase;
}

/* HOMING 완료 후 Z를 0 pulse, Y를 High 리미트로 복귀한다. */
static void CDecap_Origin(void){
    switch(Origin_Step){
        case Recovery__IDLE:
            if(xSL.isHomed == false){
                Origin_Step = Recovery__IDLE;
                sPhase = DFSM_END_OK;
                break;
            }
            /* Z축을 pulse 0 위치로 절대 이동 */
            CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc, xPL.Decapper.ZDecapVel, 0, xPL.Decapper.Limit_PosZ);
            CDecap_StartTimeout();
            Origin_Step = Recovery_Z;
            break;

        case Recovery_Z:
            if(CDecap_CheckTimeout(HOMING_TIMEOUT_MS)){ return; }

            /* Z축 이동 완료 확인 */
            if(!xSL.xZ_Motor_Run.Motor_Run){
                /* Y축을 H 방향으로 이동 */
                Air_Y_High();
                CDecap_StartTimeout();
                Origin_Step = Recovery_Y;
            }
            break;

        case Recovery_Y:
            if(CDecap_CheckTimeout(HOMING_TIMEOUT_MS)){ return; }

            /* Y_H 센서는 Active Low: 0이면 H 위치 도착 */
            if(!xSL.Decapper.Y_H_Limit){
                Air_Y_Stop();
                Origin_Step = Recovery_COMPLETE;
            }
            break;

        case Recovery_COMPLETE:
            Origin_Step = Recovery__IDLE;
            sPhase = DFSM_END_OK;
            break;

        default:
            ERR_MSG_SEND("Invalid Origin Step: %d", Origin_Step);
            Air_Y_Stop();
            Origin_Step = Recovery__IDLE;
            sPhase = DFSM_END_OK;
            break;
    }
}

/* 자동 Capping 시퀀스 */
static void CDecap_Capping(void){
	//CT가 있는지 없는지 확인
	switch(CDecapping_Step){
		case CDecapping_Idle:
			/* IO List에 CT presence 입력이 정의되기 전까지 자동 동작을 시작하지 않는다. */
			if(false){
				xprintf("There is CT \n");
				CDecapping_Step = CDecapping_Body_Grip;

			}
			else{
				xprintf("There is no CT.\n");
				sPhase = DFSM_END_OK;
			}
			break;
		case CDecapping_Body_Grip:	//CT이동을 위한 CT Body 잡기
			Air_CTBody_Gripper(ON);
			previousTime[SUB_BODY_GRIP_WAIT] = gTriggerCount;
			CDecapping_Step = CDecapping_Move_Pos;
			break;
		case CDecapping_Move_Pos: //Y축을 앞으로(Pos)
			if ((gTriggerCount - previousTime[SUB_BODY_GRIP_WAIT]) < Wait_1s){
				break;
			}
			Air_Y_Low();
			CDecap_StartTimeout();
			CDecapping_Step = CDecapping_Capping_Ready;
			break;
		case CDecapping_Capping_Ready: //y축이 특정위치로 도착했다는 명령어를 받으면 z축은 뚜껑위까지 이동
			if (CDecap_CheckTimeout(CAPPING_TIMEOUT_MS)){return;}
			if (!xSL.Decapper.Y_L_Limit){
				Air_Y_Stop();
				CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc,xPL.Decapper.ZDecapVel,xPL.Decapper.ZCap_UpPos, xPL.Decapper.Limit_PosZ);
				CDecap_StartTimeout();
				CDecapping_Step = CDecapping_sData_Capping;
			}
			break;
		case CDecapping_sData_Capping://Cap은 천천히 돌면서 z축은 아래로 이동
			if (CDecap_CheckTimeout(CAPPING_TIMEOUT_MS)){return;}
			if((!xSL.xZ_Motor_Run.Motor_Run)){
				CD_ABSMove(aR,xPL.Decapper.RDecapAcc,xPL.Decapper.RDecapVel,xPL.Decapper.RDecapPos, xPL.Decapper.Limit_PosR);
				CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc,xPL.Decapper.ZDecapVel,xPL.Decapper.ZCap_SidePos,xPL.Decapper.Limit_PosZ);
				CDecap_StartTimeout();
				CDecapping_Step = CDecapping_Move_PrevR;
			}
			break;
		case CDecapping_Move_PrevR://Capping 행동이 끝나면 Cap Grip 풀기
			if (CDecap_CheckTimeout(CAPPING_TIMEOUT_MS)){return;}
			if((!xSL.xZ_Motor_Run.Motor_Run) && (!xSL.xR_Motor_Run.Motor_Run)){
				Air_CTCap_Gripper(OFF);
				CDecapping_Step = CDecapping_Move_PrevZ;
			}
			break;
		case CDecapping_Move_PrevZ://z축은 다시 원위치로 이동
			CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc,xPL.Decapper.ZDecapVel,xPL.Decapper.ZCap_Origin_Position, xPL.Decapper.Limit_PosZ);
			CDecap_StartTimeout();
			CDecapping_Step = CDecapping_Move_PrevY;
			break;
		case CDecapping_Move_PrevY://Z축 이동이 완료된 이후 Y축은 다시 앞으로 이동 --> Slave에게 명령
			if (CDecap_CheckTimeout(CAPPING_TIMEOUT_MS)){return;}
			if(!xSL.xZ_Motor_Run.Motor_Run){
				Air_Y_High();
				CDecap_StartTimeout();
				CDecapping_Step = CDecapping_Complete_Wait;
			}
			break;
		case CDecapping_Complete_Wait://Y축이 다 이동되었다고 명령어 받기 --> 이후 CT Body Grip 풀기
			if (CDecap_CheckTimeout(CAPPING_TIMEOUT_MS)){return;}
			if (!xSL.Decapper.Y_H_Limit){
				Air_Y_Stop();
				Air_CTBody_Gripper(OFF);
				CDecapping_Step = CDecapping_Done;
			}
			break;
		case CDecapping_Done:
			CDecapping_Step = CDecapping_Idle;
	    	sPhase = DFSM_END_OK; /* 중앙 종료 처리로 전환 */
			break;
	}
}
/* 자동 Decapping 시퀀스 */
static void CDecap_Decapping(void){
	switch(CDecapping_Step){
		case CDecapping_Idle:
			/* IO List에 CT presence 입력이 정의되기 전까지 자동 동작을 시작하지 않는다. */
			if(false){
				xprintf("There is CT \n");
				CDecapping_Step = CDecapping_Body_Grip;

			}
			else{
				xprintf("There is no CT.\n");
				sPhase = DFSM_END_OK;
			}
			break;
		case CDecapping_Body_Grip: //CT이동을 위한 CT Body 잡기
			Air_CTBody_Gripper(ON);
			previousTime[SUB_BODY_GRIP_WAIT] = gTriggerCount;
			CDecapping_Step = CDecapping_Move_Pos;
			break;
		case CDecapping_Move_Pos: //Y축을 앞으로 오게 Slave에게 명령
			if ((gTriggerCount - previousTime[SUB_BODY_GRIP_WAIT]) < Wait_1s){
				break;
			}
			Air_Y_Low();
			CDecap_StartTimeout();
			CDecapping_Step = CDecapping_Decapping_Ready;
			break;
		case CDecapping_Decapping_Ready: //Cap을 잡기 위해 CAP Side 위치 까지 이동
			if (CDecap_CheckTimeout(DECAPPING_TIMEOUT_MS)){return;}
			if (!xSL.Decapper.Y_L_Limit){
				Air_Y_Stop();
				CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc,xPL.Decapper.ZDecapVel,xPL.Decapper.ZCap_SidePos, xPL.Decapper.Limit_PosZ);
				CDecap_StartTimeout();
				CDecapping_Step = CDecapping_Cap_Capgrip;
			}
			break;
		case CDecapping_Cap_Capgrip: //Cap Grip
			if (CDecap_CheckTimeout(DECAPPING_TIMEOUT_MS)){return;}
			if(!xSL.xZ_Motor_Run.Motor_Run){
				Air_CTCap_Gripper(ON);
				previousTime[SUB_CAPPING] = gTriggerCount;
				CDecapping_Step = CDecapping_sData_Decapping;
			}
			break;
		case CDecapping_sData_Decapping: // Decapping을 위해 Rotate와 z축 위로 이동
			if((gTriggerCount - previousTime[SUB_CAPPING]) > Wait_1s){
				CD_ABSMove(aR,xPL.Decapper.RDecapAcc,xPL.Decapper.RDecapVel,-(xPL.Decapper.RDecapPos), xPL.Decapper.Limit_PosR);
				CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc,xPL.Decapper.ZDecapVel,xPL.Decapper.ZCap_UpPos, xPL.Decapper.Limit_PosZ);
				CDecap_StartTimeout();
				CDecapping_Step = CDecapping_Move_PrevR;
			}
			break;
		case CDecapping_Move_PrevR: // Rotate, Z축 모터의 행동이 끝났는지 확인
			if (CDecap_CheckTimeout(DECAPPING_TIMEOUT_MS)){return;}
			if((!xSL.xZ_Motor_Run.Motor_Run) && (!xSL.xR_Motor_Run.Motor_Run)){
				CDecapping_Step = CDecapping_Move_PrevZ;
			}
			break;
		case CDecapping_Move_PrevZ: //Z축 원위치
			CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc,xPL.Decapper.ZDecapVel,xPL.Decapper.ZCap_Origin_Position, xPL.Decapper.Limit_PosZ);
			CDecap_StartTimeout();
			CDecapping_Step = CDecapping_Move_PrevY;
			break;
		case CDecapping_Move_PrevY: //축이 원위치 되고 모터가 멈췄다면 Y축 원위치 --> Slave 명령
			if (CDecap_CheckTimeout(DECAPPING_TIMEOUT_MS)){return;}
			if(!xSL.xZ_Motor_Run.Motor_Run){
				Air_Y_High();
				CDecap_StartTimeout();
				CDecapping_Step = CDecapping_Complete_Wait;
			}
			break;
		case CDecapping_Complete_Wait: //y축 원위치 되었는지 명령 대기 --> 원위치 되었다면 CT Body Grip 풀기
			if (CDecap_CheckTimeout(DECAPPING_TIMEOUT_MS)){return;}
			if (!xSL.Decapper.Y_H_Limit){
				Air_Y_Stop();
				Air_CTBody_Gripper(OFF);
				CDecapping_Step = CDecapping_Done;
			}
			break;
		case CDecapping_Done:
			CDecapping_Step = CDecapping_Idle;
			sPhase = DFSM_END_OK; /* 중앙 종료 처리로 전환 */
			break;
	}
}

/* 유닛 Capping 시험 시퀀스 */
static void UCDecap_Capping(void){
	switch(CDecapping_Step){
			case CDecapping_Idle:
				CDecapping_Step = CDecapping_Capping_Ready;
				break;
			case CDecapping_Capping_Ready: //z축은 뚜껑위까지 이동
				CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc,xPL.Decapper.ZDecapVel,xPL.Decapper.ZCap_UpPos, xPL.Decapper.Limit_PosZ);
				CDecap_StartTimeout();
				CDecapping_Step = CDecapping_sData_Capping;
				break;
			case CDecapping_sData_Capping://Cap은 천천히 돌면서 z축은 아래로 이동
				if (CDecap_CheckTimeout(CAPPING_TIMEOUT_MS)){return;}
				if((!xSL.xZ_Motor_Run.Motor_Run)){
					CD_ABSMove(aR,xPL.Decapper.RDecapAcc,xPL.Decapper.RDecapVel,xPL.Decapper.RDecapPos, xPL.Decapper.Limit_PosR);
					CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc,xPL.Decapper.ZDecapVel,xPL.Decapper.ZCap_SidePos,xPL.Decapper.Limit_PosZ);
					CDecap_StartTimeout();
					CDecapping_Step = CDecapping_Move_PrevR;
				}
				break;
			case CDecapping_Move_PrevR://Capping 행동이 끝나면 Cap Grip 풀기
				if (CDecap_CheckTimeout(CAPPING_TIMEOUT_MS)){return;}
				if((!xSL.xZ_Motor_Run.Motor_Run) && (!xSL.xR_Motor_Run.Motor_Run)){
					Air_CTCap_Gripper(OFF);
					CDecapping_Step = CDecapping_Move_PrevZ;
				}
				break;
			case CDecapping_Move_PrevZ://z축은 다시 원위치로 이동
				CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc,xPL.Decapper.ZDecapVel,xPL.Decapper.ZCap_Origin_Position, xPL.Decapper.Limit_PosZ);
				CDecap_StartTimeout();
				CDecapping_Step = CDecapping_Move_PrevY;
				break;
			case CDecapping_Move_PrevY://Z축 이동이 완료된 이후 Y축은 다시 앞으로 이동 --> Slave에게 명령
				if (CDecap_CheckTimeout(CAPPING_TIMEOUT_MS)){return;}
				if(!xSL.xZ_Motor_Run.Motor_Run){
					CDecapping_Step = CDecapping_Done;
				}
				break;
			case CDecapping_Done:
				CDecapping_Step = CDecapping_Idle;
		    	sPhase = DFSM_END_OK; /* 중앙 종료 처리로 전환 */
				break;
		}
}

/* 유닛 Decapping 시험 시퀀스 */
static void UCDecap_Decapping(void){
	switch(CDecapping_Step){
			case CDecapping_Idle:
				CDecapping_Step = CDecapping_Decapping_Ready;
				break;

			case CDecapping_Decapping_Ready: //Cap을 잡기 위해 CAP Side 위치 까지 이동
				CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc,xPL.Decapper.ZDecapVel,xPL.Decapper.ZCap_SidePos, xPL.Decapper.Limit_PosZ);
				CDecap_StartTimeout();
				CDecapping_Step = CDecapping_Cap_Capgrip;
				break;

			case CDecapping_Cap_Capgrip: //Cap Grip
				if (CDecap_CheckTimeout(DECAPPING_TIMEOUT_MS)){return;}
				if(!xSL.xZ_Motor_Run.Motor_Run){
					Air_CTCap_Gripper(ON);
					previousTime[SUB_CAPPING] = gTriggerCount;
					CDecapping_Step = CDecapping_sData_Decapping;
				}
				break;
			case CDecapping_sData_Decapping: // Decapping을 위해 Rotate와 z축 위로 이동
				if((gTriggerCount - previousTime[SUB_CAPPING]) > Wait_1s){
					CD_ABSMove(aR,xPL.Decapper.RDecapAcc,xPL.Decapper.RDecapVel,-(xPL.Decapper.RDecapPos), xPL.Decapper.Limit_PosR);
					CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc,xPL.Decapper.ZDecapVel,xPL.Decapper.ZCap_UpPos, xPL.Decapper.Limit_PosZ);
					CDecap_StartTimeout();
					CDecapping_Step = CDecapping_Move_PrevR;
				}
				break;
			case CDecapping_Move_PrevR: // Rotate, Z축 모터의 행동이 끝났는지 확인
				if (CDecap_CheckTimeout(DECAPPING_TIMEOUT_MS)){return;}
				if((!xSL.xZ_Motor_Run.Motor_Run) && (!xSL.xR_Motor_Run.Motor_Run)){
					CDecapping_Step = CDecapping_Move_PrevZ;
				}
				break;
			case CDecapping_Move_PrevZ: //Z축 원위치
				CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc,xPL.Decapper.ZDecapVel,xPL.Decapper.ZCap_Origin_Position, xPL.Decapper.Limit_PosZ);
				CDecap_StartTimeout();
				CDecapping_Step = CDecapping_Done;
				break;
			case CDecapping_Done:
				if (CDecap_CheckTimeout(DECAPPING_TIMEOUT_MS)){return;}
				if(!xSL.xZ_Motor_Run.Motor_Run){
					CDecapping_Step = CDecapping_Idle;
					sPhase = DFSM_END_OK; /* 중앙 종료 처리로 전환 */
				}
				break;
		}
}

/* STOP 전까지 Decapping과 Capping을 반복하고, Capping 완료마다 횟수를 증가시킨다. */
static void UCDecap_CDecap_Longrun(void){
	switch (xCD.Decapper.phaseDecapCap){
		case LONGRUN_DECAP:
			UCDecap_Decapping();
			if (sPhase == DFSM_END_OK){
				sPhase = DFSM_LONGRUN;
				xCD.Decapper.phaseDecapCap = LONGRUN_CAP;
			}
			break;

		case LONGRUN_CAP:
			UCDecap_Capping();
			if (sPhase == DFSM_END_OK){
				xCD.Decapper.LongRunCount++;
				sPhase = DFSM_LONGRUN;
				xCD.Decapper.phaseDecapCap = LONGRUN_DECAP;
			}
			break;

		default:
			CDecapping_Step = CDecapping_Idle;
			xCD.Decapper.phaseDecapCap = LONGRUN_DECAP;
			sPhase = DFSM_LONGRUN;
			break;
	}
}

/* 모션 컨트롤러에서 현재 Z 위치를 직접 읽는다. */
S32 CDecap_GetZPosition(void){
	xCD.Decapper.Motor_CurPos = TMC429_GetPosition(aZ);
	return xCD.Decapper.Motor_CurPos;
}

/* ACTION → Phase 매핑 테이블 */
static const teFSM_Decapper ActionToPhase[ACTION_MAX] = {
		[ACTION_NONE] /*     	*/ = DFSM_IDLE,
		/* 자동 동작 */
		[ACTION_STOP] /*     	*/ = DFSM_STOP,
		[ACTION_HOME] /*     	*/ = DFSM_HOME,
    	[ACTION_DECAP] /*    	*/ = DFSM_DECAP,
		[ACTION_CAP] /*      	*/ = DFSM_CAP,

		[ACTION_PAUSE] /*     	*/ = DFSM_PAUSE,
		[ACTION_RESUME] /*     	*/ = DFSM_RESUME,

		/* 수동 동작 */
		[ACTION_ORIGIN] /*   	*/ = DFSM_ORIGIN,

		[ACTION_RMOVEZ] /*   	*/ = DFSM_RMOVEZ,
		[ACTION_AMOVEZ] /*   	*/ = DFSM_AMOVEZ,
		[ACTION_RROTATE] /*   	*/ = DFSM_RROTATE,
		[ACTION_AROTATE] /*   	*/ = DFSM_AROTATE,

		[ACTION_Y_H_MOVE] /*   	*/ = DFSM_Y_H_MOVE,
		[ACTION_Y_L_MOVE] /*   	*/ = DFSM_Y_L_MOVE,


		[ACTION_UDECAP] /*   	*/ = DFSM_UDECAP,
		[ACTION_UCAP] /*   		*/ = DFSM_UCAP,
		[ACTION_LONGRUN] /*   	*/ = DFSM_LONGRUN,

		[ACTION_BODY_GRIP] /*  	*/ = DFSM_CT_BODY_GRIP,
		[ACTION_CAP_GRIP] /*   	*/ = DFSM_CT_CAP_GRIP,
};

/* FSM 핸들러 테이블 */
static FSM_Handler FSM_Table[DFSM_PHASE_MAX] = {
		[DFSM_IDLE] /*      	*/ = CDecap_Idle,
		/* 자동 동작 */
		[DFSM_STOP] /*      	*/ = Decapper_StopAll,
		[DFSM_HOME] /*      	*/ = CDecap_Homing,

		[DFSM_DECAP] /*    		*/ = CDecap_Decapping,
    	[DFSM_CAP] /*       	*/ = CDecap_Capping,

		[DFSM_PAUSE] /*    		*/ = CDecap_Pause,
    	[DFSM_RESUME] /*       	*/ = CDecap_Resume,

		/* 수동 동작 */
		[DFSM_ORIGIN] /*    	*/ = CDecap_Origin,

    	[DFSM_RROTATE] /*    	*/ = CDecap_RRotate,
		[DFSM_AROTATE] /*    	*/ = CDecap_ARotate,

		[DFSM_RMOVEZ] /*    	*/ = CDecap_Relmove_Z,
		[DFSM_AMOVEZ] /*    	*/ = CDecap_Absmove_Z,

		[DFSM_Y_H_MOVE] /*    	*/ = Air_Y_High_Move,
		[DFSM_Y_L_MOVE] /*    	*/ = Air_Y_Low_Move,

		[DFSM_UDECAP] /*    	*/ = UCDecap_Decapping,
		[DFSM_UCAP] /*    		*/ = UCDecap_Capping,
		[DFSM_LONGRUN] /*    	*/ = UCDecap_CDecap_Longrun,

		[DFSM_CT_CAP_GRIP] /*   */ = CT_Cap_Grip,
		[DFSM_CT_BODY_GRIP] /*  */ = CT_Body_Grip,

    	[DFSM_ABNORMAL] /*  	*/ = FSM_Abnormal,
		[DFSM_END_OK] /*    	*/ = FSM_End_ok,
};

/* 요청된 액션에 따라 디캐퍼 FSM을 한 주기 실행한다. */
void CDecap_Action_Statemachine(teXActionType actionType) {
	xCD.Decapper.Motor_CurPos = TMC429_GetPosition(aZ);

	/* STOP은 현재 상태와 관계없이 즉시 처리한다. */
	if (actionType == ACTION_STOP) {
		sPhase = DFSM_STOP;
	}
	else if (actionType == ACTION_PAUSE) {
		if (sPhase == DFSM_CAP || sPhase == DFSM_DECAP || sPhase == DFSM_LONGRUN) {
			PauseContext.savedPhase = sPhase;
			sPhase = DFSM_PAUSE;
		}
		xAT = ACTION_NONE;
	}
	else if (actionType == ACTION_RESUME) {
		if (sPhase == DFSM_PAUSE && PauseContext.isPaused) {
			sPhase = DFSM_RESUME;
		}
		xAT = ACTION_NONE;
	}

	/* IDLE 상태에서만 일반 액션을 FSM Phase로 전환한다. */
	else if (sPhase == DFSM_IDLE && actionType != ACTION_NONE) {
		if ((int) actionType > 0 && (int) actionType < ACTION_MAX) {
			if (actionType == ACTION_LONGRUN) {
				xCD.Decapper.phaseDecapCap = LONGRUN_DECAP;
				xCD.Decapper.LongRunCount = 0U;
				CDecapping_Step = CDecapping_Idle;
			}
			sPhase = ActionToPhase[actionType];
			xSL.isBusy = YES;
		} else {
			ERR_MSG_SEND("Invalid actionType: %d", actionType);
			xAT = ACTION_NONE;
			return;
		}
	}

	/* [1] Phase 유효성 검증 */
	if (sPhase < 0 || sPhase >= DFSM_PHASE_MAX){
		ERR_MSG_SEND("Invalid Phase=%d (0x%08X). Forcing ABNORMAL.", sPhase, (unsigned)sPhase);
		sPhase = DFSM_ABNORMAL;
	}

	/* 래치된 센서 오류가 있으면 비정상 처리 상태로 전환한다. */
	if (xSL.Decapper.Z2_HL_isError || xSL.Decapper.Y_HL_isError
		|| xSL.Decapper.CT_Cap_Grip_isError
		|| xSL.Decapper.CT_Body_Middle_Grip_isError
		|| xSL.Decapper.CT_Body_Top_Grip_isError){
		xSL.isBusy = NO;
		sPhase = DFSM_ABNORMAL;
	}

	/* 현재 Phase의 핸들러를 실행한다. */
	if (FSM_Table[sPhase]) {
		FSM_Table[sPhase]();
	}
}
/* 액션 완료 후 상태를 정리하고 필요하면 보류된 Phase로 복귀한다. */
static void FSM_End_ok(void) {
	/* Resume 래치: 시퀀스 중간에 HOME 등이 끝난 후 원래 Phase로 복귀 */
	if (sResumePhase != DFSM_IDLE) {
		sPhase = sResumePhase;
		sResumePhase = DFSM_IDLE;
		ResetAllSubPhases();
		return;
	}
	/* 정상 종료: IDLE 복귀 */
	sPhase = DFSM_IDLE;
	CDecapping_Step = CDecapping_Idle;
	xSL.isBusy = NO;
	xAT = ACTION_NONE;
	ResetAllSubPhases();
	ResetPauseContext();
}
/* 오류 상태를 보고하고 모든 디캐퍼 모션을 정지한다. */
static void FSM_Abnormal(void){
    ERR_MSG_SEND("Z2 H/L Limit Sensor	%d",xSL.Decapper.Z2_HL_isError);
    ERR_MSG_SEND("CT Body Middle Sensor	%d",xSL.Decapper.CT_Body_Middle_Grip_isError);
    ERR_MSG_SEND("CT Body Top Sensor	%d",xSL.Decapper.CT_Body_Top_Grip_isError);
    ERR_MSG_SEND("CT_Cap Sensor 	%d",xSL.Decapper.CT_Cap_Grip_isError);
    ERR_MSG_SEND("Y Limit Sensor 	%d",xSL.Decapper.Y_HL_isError);

    Decapper_StopAll();
    xSL.isError = YES;
}
/* 모든 하위 단계와 타이머를 초기화한다. */
static void ResetAllSubPhases(void) {
	memset(sSubPhase, 0, sizeof(sSubPhase));
	memset(previousTime, 0, sizeof(previousTime));
	sStartedFlags = 0;
	sDoneFlags = 0;
	HomingSeq.Step = CDECAP_HOMING_NONE;
	Origin_Step = Recovery__IDLE;
}

/* Pause/Resume에 사용한 저장 컨텍스트를 초기화한다. */
static void ResetPauseContext(void) {
	memset(&PauseContext, 0, sizeof(PauseContext));
	PauseContext.savedPhase = DFSM_IDLE;
	PauseContext.yDirection = PAUSE_Y_NONE;
}

