/*******************************************************************************
 * XDiagnose.c
 *
 *  Created on: 2025.10.30
 *      Author: RND. Kang PilSoon.
 *
 ******************************************************************************/

#include "_04_XDiagnose.h"
#include "XSystemInfo.h"
#include "_01_XSystemManagement.h"
#include "XDebug.h"
#include "CT_Handler.h"

Semaphore_Handle semHD_SDG;
//==============================================================================
#if 1 // debugging flag.
BOOL DB_isDiagnosisDisabled = NO;
#else
BOOL DB_isDiagnosisEnabled = YES;
#endif
//==============================================================================

typedef struct
{
    U08 isPossible;
    teErrorCode errorCode;
} tsControlAvailability;

VOID TASK_Diagnose(void *pvParameters)
{
    /** @note: USER CODE, Init. Task */
    uint32_t startTick;

    FOREVER
    {
        if (xSemaphoreTake(semHD_SDG, RTOS_WAIT_FOREVER) == pdTRUE)
        {
            __TASK_TRIGGER_START_Using(TEST_PORT_1, TP_IDX_taskSDG);
            //==================================================================
            startTick = ITIMER_StartMeasure_us();
            __xTaskStatus[TP_IDX_taskSDG] = true;

            if (xSystemInfo.PL_Get_FW_Mode() == FW_MODE_IDLE || DB_isDiagnosisDisabled == YES)
            {
                gTick_SDG = ITIMER_StopMeasure_us(startTick);
                __TASK_TRIGGER_END_Using(TEST_PORT_1, TP_IDX_taskSDG);
                continue;
            }
            XTP_CheckTaskUsingLED(XHW_STATUS_LED_3);
            //==================================================================
            xCDecap.CheckSensorValidity();
            //==================================================================
            ErrorMonitor_LED();
            // SystemInfo_CheckCpuTemperature();
            gTick_SDG = ITIMER_StopMeasure_us(startTick);
            __TASK_TRIGGER_END_Using(TEST_PORT_1, TP_IDX_taskSDG);
        }
    } /*@end FOREVER{}*/
}

int Init_Diagnose(int Index)
{
    int result = EXIT_SUCCESS;

    semHD_SDG = xSemaphoreCreateBinary();

    return result;
}

