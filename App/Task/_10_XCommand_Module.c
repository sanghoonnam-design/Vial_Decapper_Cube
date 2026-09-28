/** ****************************************************************************
 * XCommand_Module.c
 *
 * Created on: 2026.02.14
 * Author    : RND. Kang PilSoon.
 * @brief
 *             1. 상위 client 명령 실행함수 처리 모듈
 * @note
 *             1. 최대 USB 명령, RS232/485, TCP/UDP 통신 모듈이 공통으로 사용하는 구조로
 *                설계됨.
 ******************************************************************************/
#include "XSystemInfo.h"
#include "_10_XCommand_Module.h"
#include "XSystem_DB.h"
#include "_04_XDiagnose.h"

// #include "_04_XDiagnose_Def.h"

bool gZCapUpPosSavePending = false;
S32 gZCapUpPosPendingValue = 0;

const tsXCommandMapping gModuleCommandTable[] =
    {
        /** @note USER CODE BEGIN */

        /*User Command*/
        {0, "GSTA", /*      */ CMD_Handle_GSTA, /*            */ "Get the system State.", /*            */ "GSTA"},     // 시스템 상태 반환
        {0, "ST", /*        */ CMD_Handle_GSTA, /*            */ "Get the system State.", /*            */ "ST"},       // 시스템 상태 반환

        /** @note 각 장비에 해당하는 명령어 */
		{0, "SPD", /*       */ CMD_Handle_SPEED, /*           */ "Get/set motor speed percent.", /*		*/ "SPD [1~100]"},
		{0, "SPEED", /*     */ CMD_Handle_SPEED, /*           */ "Get/set motor speed percent.", /*    	*/ "SPEED [1~100]"},

		{0, "HOME", /*      */ CMD_Handle_HOME, /*            */ "Run homing sequence.", /*         	*/ "HOME"}, // 원심분리기 축 및 도어 초기 위치 복귀 (Homing)

		{0, "STOP", /*      */ CMD_Handle_STOP, /*            */ "Stop all Decapper motion.", /*    	*/ "STOP"}, // 원심분리기 축 및 도어 초기 위치 복귀 (Homing)
		{0, "PAUSE", /*     */ CMD_Handle_PAUSE, /*           */ "Pause current operation.", /*     	*/ "PAUSE"}, // 원심분리기 축 및 도어 초기 위치 복귀 (Homing)
		{0, "RESUME", /*    */ CMD_Handle_RESUME, /*          */ "Resume paused operation.", /*     	*/ "RESUME"}, // 원심분리기 축 및 도어 초기 위치 복귀 (Homing)

		{0, "DECAP", /*     */ CMD_Handle_DECAP, /*           */ "Run automatic decapping.", /*     	*/ "DECAP"}, // 원심분리기 축 및 도어 초기 위치 복귀 (Homing)
		{0, "CAP", /*       */ CMD_Handle_CAP, /*             */ "Run automatic capping.", /*       	*/ "CAP"}, // 원심분리기 축 및 도어 초기 위치 복귀 (Homing)

		/*Debug Command*/
		{1, "SL", /*        */ CMD_Handle_Print_SL, /*        */ "Print SL.", /*                        */ "SL"},       // SL 출력
		{1, "CD", /*        */ CMD_Handle_Print_CD, /*        */ "Print CD.", /*                        */ "CD"},       // CD 출력
		{1, "PL", /*        */ CMD_Handle_Print_PL, /*        */ "Print PL.", /*                        */ "PL <...>"}, // PL 출력

		{1, "ORG", /*       */ CMD_Handle_ORIGIN, /*          */ "Move to origin position.", /*     	*/ "ORG"},  // 원심분리기 축 및 도어 초기 위치 복귀 (Homing)
		{1, "MOVE", /*      */ CMD_Handle_MOVE, /*            */ "Move Z/R axis.", /*               	*/ "MOVE <R|A> <Z|R> <pulse>"}, // Z/R 축 상대/절대 위치 이동
		{1, "READY", /*     */ CMD_Handle_READY, /*            */ "Move Y axis to limit.", /*          	*/ "READY <0|1>"}, // 0: Y High, 1: Y Low

		{1, "UDECAP", /*    */ CMD_Handle_UDECAP, /*          */ "Run unit decapping.", /*          	*/ "UDECAP"}, // 서보/스텝모터 절대 위치 이동 명령 (deg 또는 step 기준)
		{1, "UCAP", /*      */ CMD_Handle_UCAP, /*            */ "Run unit capping.", /*            	*/ "UCAP"}, // 서보/스텝모터 절대 위치 이동 명령 (deg 또는 step 기준)

		{1, "CAPDECAPLR", /**/ CMD_Handle_LongRun, /*         */ "Run long-run test.", /*           	*/ "CAPDECAPLR"}, // 롱런 테스트
		{1, "LR", /*        */ CMD_Handle_LongRun, /*         */ "Run long-run test.", /*           	*/ "LR"}, // 롱런 테스트

		{1, "BGRIP", /*     */ CMD_Handle_BGRIP, /*           */ "Set body gripper output.", /*     	*/ "BGRIP <0|1>"}, // 서보/스텝모터 절대 위치 이동 명령 (deg 또는 step 기준)
		{1, "CGRIP", /*     */ CMD_Handle_CGRIP, /*           */ "Set cap gripper output.", /*      	*/ "CGRIP <0|1>"}, // 서보/스텝모터 절대 위치 이동 명령 (deg 또는 step 기준)
		{1, "RPOS", /*		*/ CMD_Handle_RPOS, /*            */ "Read current Z position.", /*      	*/ "RPOS"}, // 서보/스텝모터 절대 위치 이동 명령 (deg 또는 step 기준)

        /** @note USER CODE END */
};

const int gModuleCommandCount = sizeof(gModuleCommandTable) / sizeof(tsXCommandMapping);

static int GetParamInt(const tsXParsedData *parsedData, int index, int *out);
static int GetParamFloat(const tsXParsedData *parsedData, int index, float *out);
static bool CMD_RejectIfDecapperBusy(void);

/*==============================================================================
 * Local helpers
 *============================================================================*/
static bool CMD_RejectIfDecapperBusy(void)
{
    if ((xSL.isBusy == YES) || (xAT != ACTION_NONE))
    {
        XBuffer_AddString(xSendMsg, "BUSY", NO_COMMA);
        return true;
    }

    return false;
}

__attribute__((unused)) static int GetParamInt(const tsXParsedData *parsedData, int index, int *out)
{
    if (parsedData == NULL || out == NULL)
        return NO;

    if (index < 0 || index >= parsedData->ParamCount)
        return NO;

    if (parsedData->Params[index].type == PARAM_TYPE_INT)
    {
        *out = parsedData->Params[index].value._int;
        return YES;
    }

    if (parsedData->Params[index].type == PARAM_TYPE_FLOAT)
    {
        *out = (int)parsedData->Params[index].value._float;
        return YES;
    }

    return NO;
}

__attribute__((unused)) static int GetParamFloat(const tsXParsedData *parsedData, int index, float *out)
{
    if (parsedData == NULL || out == NULL)
        return NO;

    if (index < 0 || index >= parsedData->ParamCount)
        return NO;

    if (parsedData->Params[index].type == PARAM_TYPE_FLOAT)
    {
        *out = parsedData->Params[index].value._float;
        return YES;
    }

    if (parsedData->Params[index].type == PARAM_TYPE_INT)
    {
        *out = (float)parsedData->Params[index].value._int;
        return YES;
    }

    return NO;
}

//======================================================================================
// @USER CODE START
//======================================================================================

/*User Command*/
void CMD_Handle_GSTA(const tsXParsedData *parsedData, U08 useTCP){
    if (parsedData->ParamCount == 0)
    {
        //  ==============================================================================
        // default info.
        /* [01] */ XBuffer_AddInt(xSendMsg, xSL.isBusy, COMMA); //= (!xServoA6.IsStop() && xDoor.IsStop())
        /* [02] */ XBuffer_AddInt(xSendMsg, xSL.isEnable, COMMA);
        /* [03] */ XBuffer_AddInt(xSendMsg, xSL.isHomed, COMMA);
        /* [04] */ XBuffer_AddInt(xSendMsg, xSL.isError, COMMA);
        /* [05] */ XBuffer_AddString(xSendMsg, GetErrorCode_char(), COMMA);
        //  ==============================================================================
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);

        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);

        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_SPEED(const tsXParsedData *parsedData, U08 useTCP){
    if (parsedData->ParamCount == 0)
    {
		XBuffer_AddCommandString(xSendMsg, "SPEED", NO_COMMA);
		XBuffer_AddInt(xSendMsg, (int)CDecap_GetSpeedPercent(), NO_COMMA);
    }
	else if ((parsedData->ParamCount == 1) &&
			 (parsedData->Params[0].type == PARAM_TYPE_INT) &&
			 (parsedData->Params[0].value._int >= 1) &&
			 (parsedData->Params[0].value._int <= 100))
	{
		CDecap_SetSpeedPercent((U08)parsedData->Params[0].value._int);
		XBuffer_AddCommandString(xSendMsg, "SPEED", NO_COMMA);
		XBuffer_AddInt(xSendMsg, (int)CDecap_GetSpeedPercent(), NO_COMMA);
	}
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
		if ((parsedData->ParamCount > 0) &&
			(parsedData->Params[0].value._int != '?'))
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_HOME(const tsXParsedData *parsedData, U08 useTCP)
{
    if (parsedData->ParamCount == 0)
    {
        if (CMD_RejectIfDecapperBusy()) return;

        xAT = ACTION_HOME;
        XBuffer_AddString(xSendMsg, "home", NO_COMMA);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_STOP(const tsXParsedData *parsedData, U08 useTCP)
{
    if (parsedData->ParamCount == 0)
    {
        xAT = ACTION_STOP;
        XBuffer_AddString(xSendMsg, "stop", NO_COMMA);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_PAUSE(const tsXParsedData *parsedData, U08 useTCP)
{
    if (parsedData->ParamCount == 0)
    {
        if (xSL.isBusy == YES)
        {
            xAT = ACTION_PAUSE;
            XBuffer_AddString(xSendMsg, "pause", NO_COMMA);
        }
        else
        {
            XBuffer_AddString(xSendMsg, "isbusy = no -> not pause", NO_COMMA);
        }
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_RESUME(const tsXParsedData *parsedData, U08 useTCP)
{
    if (parsedData->ParamCount == 0)
    {
        if (xSL.isBusy == YES)
        {
            xAT = ACTION_RESUME;
            XBuffer_AddString(xSendMsg, "resume", NO_COMMA);
        }
        else
        {
            XBuffer_AddString(xSendMsg, "isbusy = no -> not pause", NO_COMMA);
        }
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_DECAP(const tsXParsedData *parsedData, U08 useTCP)
{
    if (parsedData->ParamCount == 0)
    {
        if (CMD_RejectIfDecapperBusy())
            return;

        xAT = ACTION_DECAP;
        XBuffer_AddString(xSendMsg, "Decap", NO_COMMA);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_CAP(const tsXParsedData *parsedData, U08 useTCP)
{
    if (parsedData->ParamCount == 0)
    {
        if (CMD_RejectIfDecapperBusy())
            return;

        xAT = ACTION_CAP;
        XBuffer_AddString(xSendMsg, "Cap", NO_COMMA);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}


/*Debug Command*/
void CMD_Handle_Print_SL(const tsXParsedData *parsedData, U08 useTCP)
{
    if ((useTCP == COMM_USB) &&
        (parsedData->ParamCount == 1) &&
        (parsedData->Params[0].value._int == '?'))
    {
        return;
    }

    if (parsedData->ParamCount == 0)
    {
        xcprintf(ANSI_TX_LightYellow);
        xprintf("\t======================================================================");
        xprintf("\t                         [ State List : xSL ]                         ");
        xprintf("\t======================================================================");
        xcprintf(ANSI_TX_ORG);

        xcprintf(ANSI_TX_LightCyan);
        xprintf("\t[ Common ]");
        xcprintf(ANSI_TX_ORG);
        xprintf("\t  %-42s : %10.3f sec", "xSL.Time", (double)xSL.Time);
        xprintf("\t  %-42s : %10d", "xSL.isBusy", xSL.isBusy);
        xprintf("\t  %-42s : %10d", "xSL.isError", xSL.isError);
        xprintf("\t  %-42s : %10d", "xSL.isEnable", xSL.isEnable);
        xprintf("\t  %-42s : %10d", "xSL.isHomed", xSL.isHomed);
        xprintf("\t  %-42s : %10d", "xSL.errorCode", xSL.errorCode);
        xprintf("\t  %-42s : %10lu Bytes", "sizeof(tsXStateList)", (unsigned long)sizeof(tsXStateList));

        xcprintf(ANSI_TX_LightGreen);
        xprintf("\t----------------------------------------------------------------------");

        xprintf("\t[ xSL.Decapper State ]");
        xprintf("\t  %-42s : %10d", "xSL.Decapper.Cap_is", xSL.Decapper.Cap_is);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.Body_is", xSL.Decapper.Body_is);
        xprintf("\t----------------------------------------------------------------------");
        xprintf("\t[ xSL.Decapper Sensor ]");
        xprintf("\t  %-42s : %10d", "xSL.Decapper.CT_Cap_Grip_Open", xSL.Decapper.CT_Cap_Grip_Open);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.CT_Cap_Grip_Close", xSL.Decapper.CT_Cap_Grip_Close);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.CT_Body_Middle_Grip_Open", xSL.Decapper.CT_Body_Middle_Grip_Open);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.CT_Body_Middle_Grip_Close", xSL.Decapper.CT_Body_Middle_Grip_Close);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.CT_Body_Top_Grip_Open", xSL.Decapper.CT_Body_Top_Grip_Open);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.CT_Body_Top_Grip_Close", xSL.Decapper.CT_Body_Top_Grip_Close);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.Z2_H_Limit", xSL.Decapper.Z2_H_Limit);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.Z2_L_Limit", xSL.Decapper.Z2_L_Limit);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.Z_L_Limit", xSL.Decapper.Z_L_Limit);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.Y_H_Limit", xSL.Decapper.Y_H_Limit);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.Y_L_Limit", xSL.Decapper.Y_L_Limit);

        xprintf("\t----------------------------------------------------------------------");
        xprintf("\t[ xSL.Decapper Diagnosis ]");
        xprintf("\t  %-42s : %10d", "xSL.Decapper.Z2_HL_isError", xSL.Decapper.Z2_HL_isError);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.Y_HL_isError", xSL.Decapper.Y_HL_isError);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.CT_Cap_Grip_isError", xSL.Decapper.CT_Cap_Grip_isError);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.CT_Body_Middle_Grip_isError", xSL.Decapper.CT_Body_Middle_Grip_isError);
        xprintf("\t  %-42s : %10d", "xSL.Decapper.CT_Body_Top_Grip_isError", xSL.Decapper.CT_Body_Top_Grip_isError);

        xprintf("\t----------------------------------------------------------------------");
        xprintf("\t[ Motor Run ]");
        xprintf("\t  %-42s : %10d", "xSL.xZ_Motor_Run.Motor_Run", xSL.xZ_Motor_Run.Motor_Run);
        xprintf("\t  %-42s : %10d", "xSL.xR_Motor_Run.Motor_Run", xSL.xR_Motor_Run.Motor_Run);

        xcprintf(ANSI_TX_LightYellow);
        xprintf("\t======================================================================");
        xcprintf(ANSI_TX_ORG);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);

        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_Print_CD(const tsXParsedData *parsedData, U08 useTCP)
{
    if ((useTCP == COMM_USB) &&
        (parsedData->ParamCount == 1) &&
        (parsedData->Params[0].value._int == '?'))
    {
        return;
    }

    if (parsedData->ParamCount == 0)
    {
        xcprintf(ANSI_TX_LightYellow);
        xprintf("\t======================================================================");
        xprintf("\t                       [ Control Data : xCD ]                         ");
        xprintf("\t======================================================================");
        xcprintf(ANSI_TX_ORG);

        xcprintf(ANSI_TX_LightCyan);
        xprintf("\t[ Common / Header ]");
        xcprintf(ANSI_TX_ORG);
        xprintf("\t  %-42s : %10.3f", "xCD.ID", (double)xCD.ID);
        xprintf("\t  %-42s : %10.3f", "xCD.FilteredData", (double)xCD.FilteredData);
        xprintf("\t  %-42s : %10.3f degC", "xCD.CPU_Temperature", (double)xCD.CPU_Temperature);
        xprintf("\t  %-42s : %10.3f degC", "xCD.BOARD_Temperature", (double)xCD.BOARD_Temperature);
        xprintf("\t  %-42s : %10.0f Bytes", "xCD.SL_Size", (double)xCD.SL_Size);
        xprintf("\t  %-42s : %10.3f sec", "xCD.Time", (double)xCD.Time);
        xprintf("\t  %-42s : %10lu Bytes", "sizeof(tsXControlData)", (unsigned long)sizeof(tsXControlData));

        xcprintf(ANSI_TX_LightMagenta);
        xprintf("\t----------------------------------------------------------------------");
        xprintf("\t[ xCD.Decapper ]");
        xcprintf(ANSI_TX_ORG);
        xprintf("\t  %-42s : %10ld pulse", "xCD.Decapper.Motor_CurPos",
                (long)xCD.Decapper.Motor_CurPos);
        xprintf("\t  %-42s : %10ld pulse", "xCD.Decapper.Motor_TargetPos[Z]",
                (long)xCD.Decapper.Motor_TargetPos[aZ]);
        xprintf("\t  %-42s : %10ld pulse", "xCD.Decapper.Motor_TargetPos[R]",
                (long)xCD.Decapper.Motor_TargetPos[aR]);
        xprintf("\t  %-42s : %10d (%s)", "xCD.Decapper.phaseDecapCap",
                (int)xCD.Decapper.phaseDecapCap,
                (xCD.Decapper.phaseDecapCap == LONGRUN_CAP) ? "CAP" : "DECAP");
        xprintf("\t  %-42s : %10lu", "xCD.Decapper.LongRunCount",
                (unsigned long)xCD.Decapper.LongRunCount);
        xprintf("\t  %-42s : %10u %%", "xCD.Decapper.SpeedPercent",
                (unsigned int)xCD.Decapper.SpeedPercent);
        xprintf("\t  %-42s : %10d", "xCD.Decapper.SystemInfo.isSWLimit",
                xCD.Decapper.SystemInfo.isSWLimit);
        xprintf("\t  %-42s : %10ld", "xCD.Decapper.chMotor", (long)xCD.Decapper.chMotor);

        xprintf("\t  [ Debug Data ]");
        for (int i = 0; i < DEBUG_CD_SIZE; i++)
        {
            xprintf("\t  %-36s[%2d] : %10d", "xCD.Decapper.debug", i, xCD.Decapper.debug[i]);
        }

        xcprintf(ANSI_TX_LightGreen);
        xprintf("\t----------------------------------------------------------------------");
        xprintf("\t[ xCD.CT : Cap / Body Grip Command ]");
        xcprintf(ANSI_TX_ORG);
        xprintf("\t  %-42s : %10u", "xCD.CT.Body", (unsigned int)xCD.CT.Body);
        xprintf("\t  %-42s : %10u", "xCD.CT.Cap", (unsigned int)xCD.CT.Cap);

        xcprintf(ANSI_TX_LightYellow);
        xprintf("\t======================================================================");
        xcprintf(ANSI_TX_ORG);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);

        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

static int RPL_SetParameter(int index, const tsXParsedData *parsedData)
{
    int value_i;
    float value_f;

    switch (index)
    {
    /* --- Diagnose (PL Header) --- */
    case RPL_SET_DG_CPU_TEMP_OVERHEAT:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        xPL.Header.DG_CPU_Temperature_Overheat_Criteria = value_f;
        LOG_MSG_SEND("[PL] Set DG_CPU_Temperature_Overheat_Criteria = %.3f", (double)value_f);
        return YES;

    case RPL_SET_DG_CPU_TEMP_ALARM_INTERVAL:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;

        xPL.Header.DG_CPU_Temp_Alarm_Interval_10msec = (U32)value_i;
        LOG_MSG_SEND("[PL] Set DG_CPU_Temp_Alarm_Interval_10msec = %lu",
                     (unsigned long)xPL.Header.DG_CPU_Temp_Alarm_Interval_10msec);
        return YES;

    case RPL_SET_DG_IS_DIAGNOSIS_ENABLED:
        if (GetParamInt(parsedData, 1, &value_i) == NO)
            return NO;

        xPL.Header.DG_IsDiagnosisEnabled = (value_i != 0) ? YES : NO;
        LOG_MSG_SEND("[PL] Set DG_IsDiagnosisEnabled = %d", xPL.Header.DG_IsDiagnosisEnabled);
        return YES;

    case RPL_SET_DG_CLIENT2HOST_LOGMODE:
        if (GetParamInt(parsedData, 1, &value_i) == NO)
            return NO;

        xPL.Header.DG_Client2Host_LogMode = value_i;
        LOG_MSG_SEND("[PL] Set DG_Client2Host_LogMode = %d", xPL.Header.DG_Client2Host_LogMode);
        return YES;

    /* --- Robot Door --- */
    case RPL_SET_DOOR_DIRECTION:
        if (GetParamInt(parsedData, 1, &value_i) == NO)
            return NO;

//        xPL.Door.Direction = value_i;
//        LOG_MSG_SEND("[PL] Set Door.Direction = %d", xPL.Door.Direction);
        return YES;

    case RPL_SET_DOOR_CONTROL_TIMEOUT:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;

        return YES;

    case RPL_SET_DOOR_CLOSE_OVERTIME:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;

        return YES;

    case RPL_SET_DOOR_OPEN_OVERTIME:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;

        return YES;

    case RPL_SET_DOOR_RELATIVE_DISTANCE:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;

        return YES;

    case RPL_SET_DOOR_MOTOR_SPEED:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;

        return YES;

    case RPL_SET_DOOR_MOTOR_ACCEL:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;

        return YES;

    case RPL_SET_DOOR_MOTOR_NORMAL_CURRENT:
        if (GetParamFloat(parsedData, 1, &value_f) == NO || value_f < 0.0f)
            return NO;

        return YES;

    case RPL_SET_DOOR_MOTOR_HOLDING_CURRENT:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;

        return YES;

    case RPL_SET_DOOR_MOTOR_RESOLUTION:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;

        return YES;

    /* --- Servo A6 --- */
    case RPL_SET_SERVO_DIRECTION:
        if (GetParamInt(parsedData, 1, &value_i) == NO)
            return NO;

//        xPL.ServoA6.Direction = value_i;
//        LOG_MSG_SEND("[PL] Set ServoA6.Direction = %d", xPL.ServoA6.Direction);
        return YES;

    case RPL_SET_SERVO_PULSE_PER_REV:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i <= 0)
            return NO;

        return YES;

    case RPL_SET_SERVO_HOME_FWD_SPEED:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    case RPL_SET_SERVO_HOME_BWD_SPEED:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    case RPL_SET_SERVO_HOME_ACCEL:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    case RPL_SET_SERVO_HOME_OFFSET:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    case RPL_SET_SERVO_SLOT_SPEED:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    case RPL_SET_SERVO_SLOT_ACCEL:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    case RPL_SET_SERVO_SLOT_DECEL:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    case RPL_SET_SERVO_SLOT_POSITION_OFFSET ...(RPL_SET_SERVO_SLOT_POSITION_OFFSET + SLOT_COUNT - 1):
    {
        /* slot 위치 오프셋: slot 1~6 을 index 연속으로 사용, PL <idx>,<value> */
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;

        return YES;
    }

    case RPL_SET_SERVO_JOG_SPEED:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    case RPL_SET_SERVO_JOG_ACCEL:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    case RPL_SET_SERVO_JOG_DECEL:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    case RPL_SET_SERVO_BASE_SPEED:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    case RPL_SET_SERVO_BASE_ACCEL:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    case RPL_SET_SERVO_BASE_DECEL:
        if (GetParamFloat(parsedData, 1, &value_f) == NO)
            return NO;

        return YES;

    /* --- Vial Decapper --- */
    case RPL_SET_DECAP_RUN_CUR_Z:
        if (GetParamFloat(parsedData, 1, &value_f) == NO || value_f < 0.0f)
            return NO;
        xPL.Decapper.RunCur[aZ] = value_f;
        return YES;

    case RPL_SET_DECAP_RUN_CUR_R:
        if (GetParamFloat(parsedData, 1, &value_f) == NO || value_f < 0.0f)
            return NO;
        xPL.Decapper.RunCur[aR] = value_f;
        return YES;

    case RPL_SET_DECAP_SEL_MAX_CUR_Z:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;
        xPL.Decapper.SelMaxCur[aZ] = value_i;
        return YES;

    case RPL_SET_DECAP_SEL_MAX_CUR_R:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;
        xPL.Decapper.SelMaxCur[aR] = value_i;
        return YES;

    case RPL_SET_DECAP_STOP_CUR_RATE_Z:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0 || value_i > 100)
            return NO;
        xPL.Decapper.StopCurRate[aZ] = value_i;
        return YES;

    case RPL_SET_DECAP_STOP_CUR_RATE_R:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0 || value_i > 100)
            return NO;
        xPL.Decapper.StopCurRate[aR] = value_i;
        return YES;

    case RPL_SET_DECAP_STEP_RESOLUTION:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i <= 0)
            return NO;
        xPL.Decapper.StepResolution = value_i;
        return YES;

    case RPL_SET_DECAP_LIMIT_POS_Z:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;
        xPL.Decapper.Limit_PosZ = (S32)value_i;
        return YES;

    case RPL_SET_DECAP_LIMIT_POS_R:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;
        xPL.Decapper.Limit_PosR = (S32)value_i;
        return YES;

    case RPL_SET_DECAP_SW_NEG_LIMIT_Z:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;
        xPL.Decapper.SwNegLimit[aZ] = (S32)value_i;
        return YES;

    case RPL_SET_DECAP_SW_POS_LIMIT_Z:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;
        xPL.Decapper.SwPosLimit[aZ] = (S32)value_i;
        return YES;

    case RPL_SET_DECAP_SW_NEG_LIMIT_R:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;
        xPL.Decapper.SwNegLimit[aR] = (S32)value_i;
        return YES;

    case RPL_SET_DECAP_SW_POS_LIMIT_R:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i < 0)
            return NO;
        xPL.Decapper.SwPosLimit[aR] = (S32)value_i;
        return YES;

    case RPL_SET_DECAP_SOFT_LIMIT_ENABLE:
        if (GetParamInt(parsedData, 1, &value_i) == NO || (value_i != 0 && value_i != 1))
            return NO;
        xPL.Decapper.SoftLimitEnable = (U8)value_i;
        return YES;

    case RPL_SET_DECAP_Z_ACC:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i <= 0)
            return NO;
        xPL.Decapper.ZDecapAcc = (U32)value_i;
        return YES;

    case RPL_SET_DECAP_Z_VEL:
        if (GetParamInt(parsedData, 1, &value_i) == NO)
            return NO;
        xPL.Decapper.ZDecapVel = (S32)value_i;
        return YES;

    case RPL_SET_DECAP_Z_CAP_UP_POS:
        if (GetParamInt(parsedData, 1, &value_i) == NO)
            return NO;
        xPL.Decapper.ZCap_UpPos = (S32)value_i;
        return YES;

    case RPL_SET_DECAP_Z_CAP_SIDE_POS:
        if (GetParamInt(parsedData, 1, &value_i) == NO)
            return NO;
        xPL.Decapper.ZCap_SidePos = (S32)value_i;
        return YES;

    case RPL_SET_DECAP_Z_ORIGIN_POS:
        if (GetParamInt(parsedData, 1, &value_i) == NO)
            return NO;
        xPL.Decapper.ZCap_Origin_Position = (S32)value_i;
        return YES;

    case RPL_SET_DECAP_R_ACC:
        if (GetParamInt(parsedData, 1, &value_i) == NO || value_i <= 0)
            return NO;
        xPL.Decapper.RDecapAcc = (U32)value_i;
        return YES;

    case RPL_SET_DECAP_R_VEL:
        if (GetParamInt(parsedData, 1, &value_i) == NO)
            return NO;
        xPL.Decapper.RDecapVel = (S32)value_i;
        return YES;

    case RPL_SET_DECAP_R_POS:
        if (GetParamInt(parsedData, 1, &value_i) == NO)
            return NO;
        xPL.Decapper.RDecapPos = (S32)value_i;
        return YES;

    default:
        return NO;
    }
}

static void RPL_GetParameter(void)
{
    xcprintf(ANSI_TX_LightYellow);
    xprintf("\t======================================================================");
    xprintf("\t                     [ Parameter List : xPL ]                         ");
    xprintf("\t======================================================================");
    xcprintf(ANSI_TX_ORG);

    xcprintf(ANSI_TX_LightCyan);
    xprintf("\t[ Header ]");
    xcprintf(ANSI_TX_ORG);
    xprintf("\t  %-42s : %10d", "xPL.Header.ID", xPL.Header.ID);
    xprintf("\t  %-42s : %10d Bytes", "xPL.Header.Size_PL", xPL.Header.Size_PL);
    xprintf("\t  %-42s : %10d Bytes", "xPL.Header.Size_Header", xPL.Header.Size_Header);
    xprintf("\t  %-42s : %10s ver.", "xPL.Header.FW_Version", xSystemInfo.cd_FWVersion_str);
    xprintf("\t  %-42s : %10lu YYMMDD", "xPL.Header.UpdateDate", (unsigned long)xPL.Header.UpdateDate);
    xprintf("\t  %-42s : %10.3f degC", "xPL.Header.DG_CPU_Temp_Overheat_Criteria",
            (double)xPL.Header.DG_CPU_Temperature_Overheat_Criteria);
    xprintf("\t  %-42s : %10lu 10ms", "xPL.Header.DG_CPU_Temp_Alarm_Interval_10ms",
            (unsigned long)xPL.Header.DG_CPU_Temp_Alarm_Interval_10msec);
    xprintf("\t  %-42s : %10d", "xPL.Header.DG_IsDiagnosisEnabled", xPL.Header.DG_IsDiagnosisEnabled);
    xprintf("\t  %-42s : %10d", "xPL.Header.DG_Client2Host_LogMode", xPL.Header.DG_Client2Host_LogMode);
    xprintf("\t  %-42s : %10.3f sec", "xPL.Header.Time", (double)xPL.Header.Time);

    xcprintf(ANSI_TX_LightGreen);
    xprintf("\t----------------------------------------------------------------------");
    xprintf("\t[ Decapper Parameter ]");

    xprintf("\t  [ Motor Current ]");
    xprintf("\t  %-42s : %10.3f A", "xPL.Decapper.RunCur[Z]", (double)xPL.Decapper.RunCur[aZ]);
    xprintf("\t  %-42s : %10.3f A", "xPL.Decapper.RunCur[R]", (double)xPL.Decapper.RunCur[aR]);
    xprintf("\t  %-42s : %10d", "xPL.Decapper.SelMaxCur[Z]", xPL.Decapper.SelMaxCur[aZ]);
    xprintf("\t  %-42s : %10d", "xPL.Decapper.SelMaxCur[R]", xPL.Decapper.SelMaxCur[aR]);
    xprintf("\t  %-42s : %10d %%", "xPL.Decapper.StopCurRate[Z]", xPL.Decapper.StopCurRate[aZ]);
    xprintf("\t  %-42s : %10d %%", "xPL.Decapper.StopCurRate[R]", xPL.Decapper.StopCurRate[aR]);

    xprintf("\t  [ Motor Configuration ]");
    xprintf("\t  %-42s : %10d", "xPL.Decapper.StepResolution", xPL.Decapper.StepResolution);
    xprintf("\t  %-42s : %10s", "xPL.Decapper.SoftLimitEnable",
            xPL.Decapper.SoftLimitEnable ? "ON" : "OFF");

    xprintf("\t  [ Motion Limit ]");
    xprintf("\t  %-42s : %10ld pulse", "xPL.Decapper.Limit_PosZ", (long)xPL.Decapper.Limit_PosZ);
    xprintf("\t  %-42s : %10ld pulse", "xPL.Decapper.Limit_PosR", (long)xPL.Decapper.Limit_PosR);
    xprintf("\t  %-42s : %10ld pulse", "xPL.Decapper.SwNegLimit[Z]", (long)xPL.Decapper.SwNegLimit[aZ]);
    xprintf("\t  %-42s : %10ld pulse", "xPL.Decapper.SwPosLimit[Z]", (long)xPL.Decapper.SwPosLimit[aZ]);
    xprintf("\t  %-42s : %10ld pulse", "xPL.Decapper.SwNegLimit[R]", (long)xPL.Decapper.SwNegLimit[aR]);
    xprintf("\t  %-42s : %10ld pulse", "xPL.Decapper.SwPosLimit[R]", (long)xPL.Decapper.SwPosLimit[aR]);

    xprintf("\t  [ Z Capping / Decapping ]");
    xprintf("\t  %-42s : %10lu pulse/s^2", "xPL.Decapper.ZDecapAcc", (unsigned long)xPL.Decapper.ZDecapAcc);
    xprintf("\t  %-42s : %10ld pulse/s", "xPL.Decapper.ZDecapVel", (long)xPL.Decapper.ZDecapVel);
    xprintf("\t  %-42s : %10ld pulse", "xPL.Decapper.ZCap_UpPos", (long)xPL.Decapper.ZCap_UpPos);
    xprintf("\t  %-42s : %10ld pulse", "xPL.Decapper.ZCap_SidePos", (long)xPL.Decapper.ZCap_SidePos);
    xprintf("\t  %-42s : %10ld pulse", "xPL.Decapper.ZCap_Origin_Position",
            (long)xPL.Decapper.ZCap_Origin_Position);

    xprintf("\t  [ R Capping / Decapping ]");
    xprintf("\t  %-42s : %10lu pulse/s^2", "xPL.Decapper.RDecapAcc", (unsigned long)xPL.Decapper.RDecapAcc);
    xprintf("\t  %-42s : %10ld pulse/s", "xPL.Decapper.RDecapVel", (long)xPL.Decapper.RDecapVel);
    xprintf("\t  %-42s : %10ld pulse", "xPL.Decapper.RDecapPos", (long)xPL.Decapper.RDecapPos);
    xprintf("\t======================================================================");
    xcprintf(ANSI_TX_ORG);
}

static void RPL_PrintHelp(void){
    xcprintf(ANSI_TX_LightYellow);
    xprintf("\t======================================================================");
    xprintf("\t                            [ PL Help ]                               ");
    xprintf("\t======================================================================");
    xcprintf(ANSI_TX_ORG);

    xprintf("\tPL                 : Print Parameter List.");
    xprintf("\tPL ?               : Print this help.");
    xprintf("\tPL <idx>,<value>   : Set parameter value.");
    xprintf("\t----------------------------------------------------------------------");

    xcprintf(ANSI_TX_LightCyan);
    xprintf("\t[ Settable Parameter Index ]");
    xcprintf(ANSI_TX_ORG);

    xprintf("\t  [Idx] %-50s : %8s : %s", "Parameter", "Current", "Type");
    xprintf("\t----------------------------------------------------------------------");

    /* --- Diagnose (PL Header) --- */
    xprintf("\t  [%2d] %-51s : %8.3f : F32, degC",
            RPL_SET_DG_CPU_TEMP_OVERHEAT,
            "xPL.Header.DG_CPU_Temperature_Overheat_Criteria",
            (double)xPL.Header.DG_CPU_Temperature_Overheat_Criteria);

    xprintf("\t  [%2d] %-51s : %8lu : U32, 10ms",
            RPL_SET_DG_CPU_TEMP_ALARM_INTERVAL,
            "xPL.Header.DG_CPU_Temp_Alarm_Interval_10msec",
            (unsigned long)xPL.Header.DG_CPU_Temp_Alarm_Interval_10msec);

    xprintf("\t  [%2d] %-51s : %8d : int, 0/1",
            RPL_SET_DG_IS_DIAGNOSIS_ENABLED,
            "xPL.Header.DG_IsDiagnosisEnabled",
            xPL.Header.DG_IsDiagnosisEnabled);

    xprintf("\t  [%2d] %-51s : %8d : int",
            RPL_SET_DG_CLIENT2HOST_LOGMODE,
            "xPL.Header.DG_Client2Host_LogMode",
            xPL.Header.DG_Client2Host_LogMode);

    xprintf("\t----------------------------------------------------------------------");
    xprintf("\t[ Decapper Settable Parameter ]");
    xprintf("\t  [%2d] %-51s : %8.3f : F32, A",
            RPL_SET_DECAP_RUN_CUR_Z, "xPL.Decapper.RunCur[Z]", (double)xPL.Decapper.RunCur[aZ]);
    xprintf("\t  [%2d] %-51s : %8.3f : F32, A",
            RPL_SET_DECAP_RUN_CUR_R, "xPL.Decapper.RunCur[R]", (double)xPL.Decapper.RunCur[aR]);
    xprintf("\t  [%2d] %-51s : %8d : int",
            RPL_SET_DECAP_SEL_MAX_CUR_Z, "xPL.Decapper.SelMaxCur[Z]", xPL.Decapper.SelMaxCur[aZ]);
    xprintf("\t  [%2d] %-51s : %8d : int",
            RPL_SET_DECAP_SEL_MAX_CUR_R, "xPL.Decapper.SelMaxCur[R]", xPL.Decapper.SelMaxCur[aR]);
    xprintf("\t  [%2d] %-51s : %8d : int, 0~100 %%",
            RPL_SET_DECAP_STOP_CUR_RATE_Z, "xPL.Decapper.StopCurRate[Z]", xPL.Decapper.StopCurRate[aZ]);
    xprintf("\t  [%2d] %-51s : %8d : int, 0~100 %%",
            RPL_SET_DECAP_STOP_CUR_RATE_R, "xPL.Decapper.StopCurRate[R]", xPL.Decapper.StopCurRate[aR]);
    xprintf("\t  [%2d] %-51s : %8d : int, > 0",
            RPL_SET_DECAP_STEP_RESOLUTION, "xPL.Decapper.StepResolution", xPL.Decapper.StepResolution);
    xprintf("\t  [%2d] %-51s : %8ld : S32, pulse",
            RPL_SET_DECAP_LIMIT_POS_Z, "xPL.Decapper.Limit_PosZ", (long)xPL.Decapper.Limit_PosZ);
    xprintf("\t  [%2d] %-51s : %8ld : S32, pulse",
            RPL_SET_DECAP_LIMIT_POS_R, "xPL.Decapper.Limit_PosR", (long)xPL.Decapper.Limit_PosR);
    xprintf("\t  [%2d] %-51s : %8ld : S32, pulse",
            RPL_SET_DECAP_SW_NEG_LIMIT_Z, "xPL.Decapper.SwNegLimit[Z]", (long)xPL.Decapper.SwNegLimit[aZ]);
    xprintf("\t  [%2d] %-51s : %8ld : S32, pulse",
            RPL_SET_DECAP_SW_POS_LIMIT_Z, "xPL.Decapper.SwPosLimit[Z]", (long)xPL.Decapper.SwPosLimit[aZ]);
    xprintf("\t  [%2d] %-51s : %8ld : S32, pulse",
            RPL_SET_DECAP_SW_NEG_LIMIT_R, "xPL.Decapper.SwNegLimit[R]", (long)xPL.Decapper.SwNegLimit[aR]);
    xprintf("\t  [%2d] %-51s : %8ld : S32, pulse",
            RPL_SET_DECAP_SW_POS_LIMIT_R, "xPL.Decapper.SwPosLimit[R]", (long)xPL.Decapper.SwPosLimit[aR]);
    xprintf("\t  [%2d] %-51s : %8u : U8, 0/1",
            RPL_SET_DECAP_SOFT_LIMIT_ENABLE, "xPL.Decapper.SoftLimitEnable",
            (unsigned int)xPL.Decapper.SoftLimitEnable);
    xprintf("\t  [%2d] %-51s : %8lu : U32, pulse/s^2",
            RPL_SET_DECAP_Z_ACC, "xPL.Decapper.ZDecapAcc", (unsigned long)xPL.Decapper.ZDecapAcc);
    xprintf("\t  [%2d] %-51s : %8ld : S32, pulse/s",
            RPL_SET_DECAP_Z_VEL, "xPL.Decapper.ZDecapVel", (long)xPL.Decapper.ZDecapVel);
    xprintf("\t  [%2d] %-51s : %8ld : S32, pulse",
            RPL_SET_DECAP_Z_CAP_UP_POS, "xPL.Decapper.ZCap_UpPos", (long)xPL.Decapper.ZCap_UpPos);
    xprintf("\t  [%2d] %-51s : %8ld : S32, pulse",
            RPL_SET_DECAP_Z_CAP_SIDE_POS, "xPL.Decapper.ZCap_SidePos", (long)xPL.Decapper.ZCap_SidePos);
    xprintf("\t  [%2d] %-51s : %8ld : S32, pulse",
            RPL_SET_DECAP_Z_ORIGIN_POS, "xPL.Decapper.ZCap_Origin_Position",
            (long)xPL.Decapper.ZCap_Origin_Position);
    xprintf("\t  [%2d] %-51s : %8lu : U32, pulse/s^2",
            RPL_SET_DECAP_R_ACC, "xPL.Decapper.RDecapAcc", (unsigned long)xPL.Decapper.RDecapAcc);
    xprintf("\t  [%2d] %-51s : %8ld : S32, pulse/s",
            RPL_SET_DECAP_R_VEL, "xPL.Decapper.RDecapVel", (long)xPL.Decapper.RDecapVel);
    xprintf("\t  [%2d] %-51s : %8ld : S32, pulse",
            RPL_SET_DECAP_R_POS, "xPL.Decapper.RDecapPos", (long)xPL.Decapper.RDecapPos);

    xprintf("\t----------------------------------------------------------------------");
    xprintf("\tExample:");
    xprintf("\t  PL %d,70.0      -> CPU overheat criteria = 70.0 degC", RPL_SET_DG_CPU_TEMP_OVERHEAT);
    xprintf("\t  PL %d,20000     -> Z decapper velocity = 20000 pulse/s", RPL_SET_DECAP_Z_VEL);
    xprintf("\t  PL %d,115200    -> Z cap upper position = 115200 pulse", RPL_SET_DECAP_Z_CAP_UP_POS);
    xprintf("\t  PL %d,1         -> Soft limit enable", RPL_SET_DECAP_SOFT_LIMIT_ENABLE);
    xcprintf(ANSI_TX_LightYellow);
    xprintf("\t======================================================================");
    xcprintf(ANSI_TX_ORG);
}

void CMD_Handle_Print_PL(const tsXParsedData *parsedData, U08 useTCP)
{
    int index;

    /* Help */
    if ((useTCP == COMM_USB) &&
        (parsedData->ParamCount == 1) &&
        (parsedData->Params[0].value._int == '?'))
    {
        RPL_PrintHelp();
        return;
    }

    /* Set : PL <idx>,<value> */
    if (parsedData->ParamCount == 2)
    {
        if (GetParamInt(parsedData, 0, &index) == NO)
        {
            SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
            XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
            return;
        }

        if (RPL_SetParameter(index, parsedData) == NO)
        {
            SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
            XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
            return;
        }

        XBuffer_AddString(xSendMsg, "OK", NO_COMMA);
        return;
    }

    /* Get */
    if (parsedData->ParamCount == 0)
    {
        RPL_GetParameter();

        return;
    }

    /* Invalid */
    SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
    XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);

    if (parsedData->ParamCount > 0 &&
        parsedData->Params[0].value._int != '?')
    {
        xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_ORIGIN(const tsXParsedData *parsedData, U08 useTCP)
{
    if (parsedData->ParamCount == 0){
        if (CMD_RejectIfDecapperBusy())
            return;

        xAT = ACTION_ORIGIN;
        XBuffer_AddString(xSendMsg, "origin", NO_COMMA);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_MOVE(const tsXParsedData *parsedData, U08 useTCP)
{
    int mode;
    int axis;
    int pulse;

    if ((parsedData->ParamCount == 3) &&
        (GetParamInt(parsedData, 0, &mode) == YES) &&
        (GetParamInt(parsedData, 1, &axis) == YES) &&
        (GetParamInt(parsedData, 2, &pulse) == YES) &&
        ((mode == 'R') || (mode == 'A')) &&
        ((axis == 'Z') || (axis == 'R')))
    {
        if (CMD_RejectIfDecapperBusy())
            return;

        if (axis == 'Z')
        {
            xCD.Decapper.Motor_TargetPos[aZ] = (S32)pulse;
            xAT = (mode == 'R') ? ACTION_RMOVEZ : ACTION_AMOVEZ;
        }
        else
        {
            xCD.Decapper.Motor_TargetPos[aR] = (S32)pulse;
            xAT = (mode == 'R') ? ACTION_RROTATE : ACTION_AROTATE;
        }

        XBuffer_AddString(xSendMsg, "OK", NO_COMMA);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if ((parsedData->ParamCount > 0) &&
            (parsedData->Params[0].value._int != '?'))
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_READY(const tsXParsedData *parsedData, U08 useTCP)
{
    int value;

    if ((parsedData->ParamCount == 1) &&
        (GetParamInt(parsedData, 0, &value) == YES) &&
        ((value == 0) || (value == 1)))
    {
        if (CMD_RejectIfDecapperBusy())
            return;

        xAT = (value == 1) ? ACTION_Y_L_MOVE : ACTION_Y_H_MOVE;
        XBuffer_AddString(xSendMsg, "OK", NO_COMMA);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if ((parsedData->ParamCount > 0) &&
            (parsedData->Params[0].value._int != '?'))
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_UDECAP(const tsXParsedData *parsedData, U08 useTCP)
{
    if (parsedData->ParamCount == 0)
    {
        if (CMD_RejectIfDecapperBusy())
            return;

        xAT = ACTION_UDECAP;
        XBuffer_AddString(xSendMsg, "udecap", NO_COMMA);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_UCAP(const tsXParsedData *parsedData, U08 useTCP)
{
    if (parsedData->ParamCount == 0)
    {
        if (CMD_RejectIfDecapperBusy())
            return;

        xAT = ACTION_UCAP;
        XBuffer_AddString(xSendMsg, "ucap", NO_COMMA);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_LongRun(const tsXParsedData *parsedData, U08 useTCP)
{
    /* LR : long-run 시작, 정지는 STOP 명령으로 처리 */
    if (parsedData->ParamCount == 0)
    {
        if (CMD_RejectIfDecapperBusy())
            return;

        xAT = ACTION_LONGRUN;
        XBuffer_AddString(xSendMsg, "LR", NO_COMMA);
        return;
    }

    SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
    XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);

    if (parsedData->Params[0].value._int != '?')
        xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
}

void CMD_Handle_BGRIP(const tsXParsedData *parsedData, U08 useTCP)
{
    int value;

    if ((parsedData->ParamCount == 1) &&
    		(GetParamInt(parsedData, 0, &value) == YES) &&
        ((value == 0) || (value == 1)))
    {
        if (CMD_RejectIfDecapperBusy())
            return;

        CDecap_SetBodyGripCommand((U08)value);
        xAT = ACTION_BODY_GRIP;
        XBuffer_AddString(xSendMsg, "OK", NO_COMMA);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if ((parsedData->ParamCount > 0) &&
            (parsedData->Params[0].value._int != '?'))
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_CGRIP(const tsXParsedData *parsedData, U08 useTCP)
{
    int value;

    if ((parsedData->ParamCount == 1) &&
        (GetParamInt(parsedData, 0, &value) == YES) &&
        ((value == 0) || (value == 1)))
    {
        if (CMD_RejectIfDecapperBusy())
            return;

        CDecap_SetCapGripCommand((U08)value);
        xAT = ACTION_CAP_GRIP;
        XBuffer_AddString(xSendMsg, "OK", NO_COMMA);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if ((parsedData->ParamCount > 0) &&
            (parsedData->Params[0].value._int != '?'))
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

void CMD_Handle_RPOS(const tsXParsedData *parsedData, U08 useTCP)
{
    if (parsedData->ParamCount == 0){
        gZCapUpPosPendingValue = CDecap_GetZPosition();
        XBuffer_AddInt(xSendMsg, (int)gZCapUpPosPendingValue, NO_COMMA);
        gZCapUpPosSavePending = true;
        xprintf("Currunt Z_Position = %ld",xCD.Decapper.Motor_CurPos);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}

/*Unused Command*/
// debugging code
void CMD_Handle_PSTA(const tsXParsedData *parsedData, U08 useTCP)
{
    U08 i = 1;

    if (parsedData->ParamCount == 0)
    {
        __newLine();
        xcprintf(ANSI_TX_Yellow);
        /*[ 1]*/ xprintf("\t[%2d] %6d : xSL.isBusy", i++, xSL.isBusy);
        //*[ 2]*/ xprintf("\t[%2d] %6d : xServoA6.IsServoOn()", i++, xServoA6.IsServoOn());
        //*[ 3]*/ xprintf("\t[%2d] %6d : xServoA6.IsHomed()", i++, xServoA6.IsHomed());
        /*[ 4]*/ xprintf("\t[%2d] %6d : IsError()", i++, IsError());
        /*[ 5]*/ xprintf("\t[%2d] %6s : error code.", i++, GetErrorCode_char());
        xcprintf(ANSI_TX_Cyan);
        //*[ 7]*/ xprintf("\t[%2d] %6d : xServoA6.GetPosition_SlotNum() + 1", i++, xServoA6.GetPosition_SlotNum() + 1);
        xcprintf(ANSI_TX_ORG);
        //*[ 9]*/ xprintf("\t[%2d] %6d : xServoA6.IsDriverError()", i++, xServoA6.IsDriverError());
        xcprintf(ANSI_TX_Red);
        /*[13]*/ xprintf("\t[%2d] %6d : rsv.", i++, 0);
        /*[14]*/ xprintf("\t[%2d] %6d : rsv.", i++, 0);
        /*[15]*/ xprintf("\t[%2d] %6d : rsv.", i++, 0);
        /*[16]*/ xprintf("\t[%2d] %6d : rsv.", i++, 0);
        /*[17]*/ xprintf("\t[%2d] %6d : rsv.", i++, 0);
        /*[18]*/ xprintf("\t[%2d] %6d : rsv.", i++, 0);
        /*[19]*/ xprintf("\t[%2d] %6d : rsv.", i++, 0);
        /*[20]*/ xprintf("\t[%2d] %6d : rsv.", i++, 0);
        xcprintf(ANSI_TX_ORG);
    }
    else
    {
        SetErrorCode(ERROR_CODE_INVALID_ARGUMENT, __func__, __LINE__);
        XBuffer_AddString(xSendMsg, GetErrorCode_char(), NO_COMMA);
        if (parsedData->Params[0].value._int != '?')
            xParser_HandleError(__func__, PARSER_ERR_INVALID_CMD, 0);
    }
}
