/*******************************************************************************
 * XSystemInfo.c
 *
 *  Created on: 2024.10.15
 *      Author: RND. Kang PilSoon.
 ******************************************************************************/
#include "XSystemInfo.h"
#include "Version.h"

#include "FreeRTOS.h"
#include "task.h"
#include "XSystem_DB.h"

#include "XModbus.h"
#include "bootloader.h"

tsXSystemInfo xSystemInfo;

void System_Initialize(void)
{
    /* system 구조체 초기화 */
    memset((char *)&xSystemInfo, 0, sizeof(tsXSystemInfo));
    //==========================================================================
    xSystemInfo.SL_Set_isStartMainLoop /*     */ = SystemInfo_Set_sl_isStartMainLoop;
    xSystemInfo.SL_Get_isStartMainLoop /*     */ = SystemInfo_Get_sl_isStartMainLoop;
    xSystemInfo.CD_Set_CpuUsage /*            */ = SystemInfo_Set_cd_CpuUsage;
    xSystemInfo.CD_Get_CpuUsage /*            */ = SystemInfo_Get_cd_CpuUsage;
    xSystemInfo.CD_Set_CpuTemperature /*      */ = SystemInfo_Set_cd_CpuTemperature;
    xSystemInfo.CD_Get_CpuTemperature /*      */ = SystemInfo_Get_cd_CpuTemperature;
    xSystemInfo.PL_Set_FW_Version /*          */ = SystemInfo_Set_pl_FW_Version;
    xSystemInfo.PL_Get_FW_Version /*          */ = SystemInfo_Get_pl_FW_Version;
    xSystemInfo.PL_Set_FW_Mode /*             */ = SystemInfo_Set_pl_FW_Mode;
    xSystemInfo.PL_Get_FW_Mode /*             */ = SystemInfo_Get_pl_FW_Mode;
    xSystemInfo.PL_Set_IsExecutedDiagnosis /* */ = SystemInfo_Set_pl_IsExecutedDiagnosis;
    xSystemInfo.PL_Get_IsExecutedDiagnosis /* */ = SystemInfo_Get_pl_IsExecutedDiagnosis;
    //==========================================================================

    /* System 변수 default 값 초기화 */
    xSystemInfo.SL_Set_isStartMainLoop(NO);
    xSystemInfo.CD_Set_CpuUsage(0.0f);
    xSystemInfo.CD_Set_CpuTemperature(0.0f);

    xSystemInfo.pl_systemType = (U08)GetSystemType();
    xSystemInfo.pl_modelType = GetModelType();

    xSystemInfo.PL_Set_FW_Mode(FW_MODE_DEFAULT);
    xSystemInfo.PL_Set_IsExecutedDiagnosis(YES);

    SystemDB_Initialize();

    HWDev_Initialize(); // HW  & Device 초기화
    InitErrorCode();    // error  초기화

    EEPROMPL_Initialize();                  //
    SYSPL_UpdateFromEeprom_Flash(&gEEPROM); // EEPROM 으로 부터 system 파라미터 초기화
    
    U16 crc = CRC_Calculate((U8 *)&gEEPROM, sizeof(tsEEPROM_Config) - sizeof(U16));
    
    if (gEEPROM.header.FactorySetConfirm_Key == EEPROM_FACTORY_SETTING_KEY)
    {
        if (crc != gEEPROM.crc)
        {
            gEEPROM.header.isCrcValid = NO;
        }
        else
        {
            gEEPROM.header.isCrcValid = YES;
        }
    }
    else
    {
        SYSPL_FactorySetting();
    }

    //==========================================================================
    if (xSystemInfo.pl_FW_Version != FW_VERSION || // 시스템 정보 최신화 체크
        GetSystemType() != gEEPROM.header.SystemType ||
        GetModelType() != gEEPROM.header.ModelType)
    {
        xSystemInfo.pl_FW_Version = FW_VERSION;
        xSystemInfo.pl_systemType = GetSystemType();
        xSystemInfo.pl_modelType = GetModelType();

        // [YYMMDD] parameter를 EEPROM에 저장한 날짜 저장
        xPL.Header.UpdateDate = SWRTC_GetTime_YYMMDDHH();
        
        EEPROMPL_SaveToEEPROMandFlash();
    }
    makeVersionString();

    // ==========================================================================
    BootParamInfo_SetNetworkIP(xSystemInfo.network);
    Network_Init(); // EEPROM 로딩이 끝나면 network 정보 반영 및 초기화

#if configWatchDog_ENABLE
    IWDG_Init();
#endif
}

SET_GET_FUNC_0_IMPL(SystemInfo, U08, sl_isStartMainLoop)
SET_GET_FUNC_0_IMPL(SystemInfo, F32, cd_CpuUsage)
SET_GET_FUNC_0_IMPL(SystemInfo, F32, cd_CpuTemperature)
//SET_GET_FUNC_0_IMPL(SystemInfo, U32, pl_FW_Version)
SET_GET_FUNC_0_IMPL(SystemInfo, U08, pl_FW_Mode)
SET_GET_FUNC_0_IMPL(SystemInfo, U32, pl_IsExecutedDiagnosis)

void SystemInfo_Set_pl_FW_Version(U32 version)
{
    xSystemInfo.pl_FW_Version = version;
    makeVersionString();
}

U32 SystemInfo_Get_pl_FW_Version(void)
{
    return xSystemInfo.pl_FW_Version;
}

void SystemInfo_Network_SetIP(Network_t *net, const U8 ip[4])
{
    if (net == NULL)
        return;
    memcpy(net->ip, ip, sizeof(net->ip));
}

void makeVersionString(void)
{
    U08 modelType = xSystemInfo.pl_modelType;
    int major = (xSystemInfo.pl_FW_Version / 10000);     // Major 버전
    int minor = (xSystemInfo.pl_FW_Version / 100) % 100; // Minor 버전
    int patch = xSystemInfo.pl_FW_Version % 100;         // Patch 버전
    snprintf(xSystemInfo.cd_FWVersion_str, sizeof(xSystemInfo.cd_FWVersion_str),
             "%d.%d.%dA%02u", major, minor, patch, (unsigned int)modelType);
}

U08 GetSystemType(void)
{

    return SYSTEM_TYPE;
}

U08 GetModelType(void)
{

    return MODEL_TYPE;
}

const char *GetSystemTypeString(void)
{
    return SYSTEM_TYPE_STR;
}

const char *GetModelTypeString(void){
    switch (GetModelType()){
    case MODEL_TUBE:
        return "TUBE";
    case MODEL_50ML:
        return "50mL";
    default:
        return "UNKNOWN";
    }
}
