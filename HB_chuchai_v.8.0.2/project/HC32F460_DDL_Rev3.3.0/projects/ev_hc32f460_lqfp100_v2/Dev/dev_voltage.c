#include "dev_voltage.h"
#include "TickTimer.h"
#include "Gpio_io.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>

// ========== Ä£ï¿½ï¿½Ä£Ê½È«ï¿½Ö±ï¿½ï¿½ï¿½ ==========
#ifdef VOLTAGE_SIMULATION_MODE
    // Ä£ï¿½ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Keil Watchï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½Þ¸Ä£ï¿½
    // volatileï¿½ï¿½Ö¹ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Å»ï¿½ï¿½ï¿½È·ï¿½ï¿½Watchï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÊµÊ±ï¿½ï¿½È¡ï¿½ï¿½ï¿½ä»¯
    static volatile uint16_t s_u16SimAdcVoltageMv = 2000;  // Ä¬ï¿½ï¿½2000mVï¿½ï¿½24.00VÄ¸ï¿½ßµï¿½Ñ¹ï¿½ï¿½
    // È«ï¿½ï¿½Ö¸ï¿½ë£¬ï¿½ï¿½ï¿½ï¿½Keil Watchï¿½ï¿½ï¿½ï¿½Ö±ï¿½Ó¹Û²ï¿½ï¿½ï¿½Þ¸ï¿½Ä£ï¿½ï¿½ï¿½Ñ¹Öµ
    // ï¿½ï¿½Watchï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½: *g_pu16DbgSimVoltageMv
    volatile uint16_t* const g_pu16DbgSimVoltageMv = &s_u16SimAdcVoltageMv;
    #define SIM_ADC_RAW_VALUE(voltage)  ((uint16_t)((uint32_t)(voltage) * 4095 / 3300))
#endif

// ========== ï¿½Ú²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ==========

// ï¿½ï¿½ADCï¿½è±¸ï¿½ï¿½È¡Ô­Ê¼ï¿½ï¿½ï¿½ï¿½
static DeviceResult_t Voltage_ReadFromAdc(Voltage_Device_t* pstcDev) {
    if (!pstcDev) return RESULT_PARAM_ERR;

#ifdef VOLTAGE_SIMULATION_MODE
    // ========== Ä£ï¿½ï¿½Ä£Ê½ï¿½ï¿½Ê¹ï¿½ï¿½ï¿½Ö¶ï¿½ï¿½ï¿½ï¿½Ãµï¿½Öµ ==========
    pstcDev->u16AdcVoltageMv = s_u16SimAdcVoltageMv;
    pstcDev->u16AdcRawValue = SIM_ADC_RAW_VALUE(s_u16SimAdcVoltageMv);

    VOLTAGE_DEBUG("SIM Mode: Voltage=%d mV, Raw=%d\r\n",
                 pstcDev->u16AdcVoltageMv, pstcDev->u16AdcRawValue);
    return RESULT_OK;
#else
    // ========== ï¿½ï¿½ÊµÄ£Ê½ï¿½ï¿½ï¿½ï¿½ADCï¿½è±¸ï¿½ï¿½È¡ ==========
    // Í¨ï¿½ï¿½Device Managerï¿½ï¿½È¡ADCï¿½è±¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    ADC_ReadResponse_t stcAdcResp;
    DeviceResult_t res = Device_Read(pstcDev->stcConfig.u8AdcDevId, &stcAdcResp, sizeof(ADC_ReadResponse_t));

    if (res == RESULT_OK) {
        pstcDev->u16AdcRawValue = stcAdcResp.u16RawValue;
        pstcDev->u16AdcVoltageMv = stcAdcResp.u16VoltageMv;
    }

    return res;
#endif
}

// ï¿½ï¿½ï¿½ï¿½Êµï¿½ï¿½Ä¸ï¿½ßµï¿½Ñ¹
// ï¿½ï¿½Ñ¹ï¿½ï¿½ = (110k + 10k) / 10k = 12
// Êµï¿½Êµï¿½Ñ¹ = ADCï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ñ¹ ï¿½ï¿½ 12
static void Voltage_CalcBusVoltage(Voltage_Device_t* pstcDev) {
    if (!pstcDev) return;

    // Ä¸ï¿½ßµï¿½Ñ¹(mV) = ADCï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ñ¹(mV) ï¿½ï¿½ ï¿½ï¿½Ñ¹ï¿½ï¿½
    pstcDev->u32BusVoltageMv = (uint32_t)pstcDev->u16AdcVoltageMv * VOLTAGE_DIVIDER_RATIO + VOLTAGE_COMPENSATION_MV;

    // Ä¸ï¿½ßµï¿½Ñ¹(V)ï¿½ï¿½100 = Ä¸ï¿½ßµï¿½Ñ¹(mV) / 10
    // ï¿½ï¿½ï¿½ç£º24000mV ï¿½ï¿½ 2400 (ï¿½ï¿½24.00V)
    pstcDev->u16BusVoltageVx100 = (uint16_t)(pstcDev->u32BusVoltageMv / 10UL);
}

// ========== ï¿½ï¿½Ñ¹/Ç·Ñ¹ï¿½ï¿½ï¿½ ==========
// ï¿½ï¿½Öµï¿½ï¿½ï¿½Í»Ø¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½ stcConfig ï¿½á¹¹ï¿½ï¿½ï¿½Ð¶ï¿½È¡
// ï¿½ï¿½Ð©Öµï¿½ï¿½ Voltage_Device_Create() Ê±ï¿½Éµï¿½ï¿½ï¿½ï¿½ß´ï¿½ï¿½ï¿½

// ï¿½ï¿½Ñ¹/Ç·Ñ¹ï¿½ï¿½âº¯ï¿½ï¿½
static void Voltage_QueueEvent(Voltage_AlarmState_t* pstcAlarm, uint8_t u8AlarmType,
                               uint32_t u32BusVoltageMv, uint32_t u32ThresholdMv,
                               uint8_t u8IsActive) {
    if (!pstcAlarm) return;

    if (u8AlarmType == VOLTAGE_ALARM_OVERVOLTAGE) {
        pstcAlarm->stcOvervoltagePendingEvent.u8AlarmType = u8AlarmType;
        pstcAlarm->stcOvervoltagePendingEvent.u32BusVoltageMv = u32BusVoltageMv;
        pstcAlarm->stcOvervoltagePendingEvent.u32ThresholdMv = u32ThresholdMv;
        pstcAlarm->stcOvervoltagePendingEvent.u8IsActive = u8IsActive;
        pstcAlarm->u8OvervoltageEventPending = 1;
    } else {
        pstcAlarm->stcUndervoltagePendingEvent.u8AlarmType = u8AlarmType;
        pstcAlarm->stcUndervoltagePendingEvent.u32BusVoltageMv = u32BusVoltageMv;
        pstcAlarm->stcUndervoltagePendingEvent.u32ThresholdMv = u32ThresholdMv;
        pstcAlarm->stcUndervoltagePendingEvent.u8IsActive = u8IsActive;
        pstcAlarm->u8UndervoltageEventPending = 1;
    }
}

// ¹ýÑ¹/Ç·Ñ¹¼ì²â£ºÁ¬ÐøÔ½ÏÞ 30ms ²ÅÈ·ÈÏ£»»Ö¸´Õý³£ºó°´³ÙÖÍÁ¢¼´ÊÍ·Å
static void Voltage_CheckAlarm(Voltage_Device_t* pstcDev) {
    if (!pstcDev) return;

    uint32_t u32BusMv = pstcDev->u32BusVoltageMv;
    Voltage_AlarmState_t* pstcAlarm = &pstcDev->stcAlarmState;
    Voltage_Config_t* pstcCfg = &pstcDev->stcConfig;

    uint32_t u32OvConfirmMs = pstcCfg->u32OvervoltageConfirmMs;
    uint32_t u32UvConfirmMs = pstcCfg->u32UndervoltageConfirmMs;
    if (u32OvConfirmMs == 0) u32OvConfirmMs = VOLTAGE_CONFIRM_TIME_MS;
    if (u32UvConfirmMs == 0) u32UvConfirmMs = VOLTAGE_CONFIRM_TIME_MS;

    // ---- ¹ýÑ¹ ----
    if (!pstcAlarm->u8OvervoltageAlarm) {
        if (u32BusMv >= pstcCfg->u32OvervoltageThresholdMv) {
            if (!pstcAlarm->u8OvervoltageTimerRunning) {
                nbDelay_Init(&pstcAlarm->stcOvervoltageTimer, u32OvConfirmMs);
                nbDelay_Start(&pstcAlarm->stcOvervoltageTimer);
                pstcAlarm->u8OvervoltageTimerRunning = 1;
            } else if (nbDelay_IsComplete(&pstcAlarm->stcOvervoltageTimer)) {
                pstcAlarm->u8OvervoltageTimerRunning = 0;
                pstcAlarm->u8OvervoltageAlarm = 1;
                pstcAlarm->u32OvervoltageReleaseMv =
                    (pstcCfg->u32OvervoltageThresholdMv > pstcCfg->u32OvervoltageHysteresisMv)
                        ? (pstcCfg->u32OvervoltageThresholdMv - pstcCfg->u32OvervoltageHysteresisMv)
                        : 0;

                // ¹ýÑ¹È·ÈÏºóÔÙÇåÇ·Ñ¹£¬±ÜÃâË²¼äÔ½¹ýÁíÒ»²àãÐÖµÎóÇå
                if (pstcAlarm->u8UndervoltageAlarm) {
                    pstcAlarm->u8UndervoltageAlarm = 0;
                    pstcAlarm->u8UndervoltageTimerRunning = 0;
                    nbDelay_Stop(&pstcAlarm->stcUndervoltageTimer);
                    Voltage_QueueEvent(pstcAlarm, VOLTAGE_ALARM_UNDERVOLTAGE,
                                       u32BusMv, pstcCfg->u32UndervoltageThresholdMv, 0);
                }

                Voltage_QueueEvent(pstcAlarm, VOLTAGE_ALARM_OVERVOLTAGE,
                                   u32BusMv, pstcCfg->u32OvervoltageThresholdMv, 1);
            }
        } else {
            if (pstcAlarm->u8OvervoltageTimerRunning) {
                nbDelay_Stop(&pstcAlarm->stcOvervoltageTimer);
                pstcAlarm->u8OvervoltageTimerRunning = 0;
            }
        }
    } else {
        if (u32BusMv <= pstcAlarm->u32OvervoltageReleaseMv) {
            pstcAlarm->u8OvervoltageAlarm = 0;
            pstcAlarm->u8OvervoltageTimerRunning = 0;
            nbDelay_Stop(&pstcAlarm->stcOvervoltageTimer);
#if VOLTAGE_CLEAR_MODE == VOLTAGE_CLEAR_AUTO
            Voltage_QueueEvent(pstcAlarm, VOLTAGE_ALARM_OVERVOLTAGE,
                               u32BusMv, pstcCfg->u32OvervoltageThresholdMv, 0);
#else
            /* ¿çµ½Ç·Ñ¹²àÊ±ÑØÓÃÔ­Âß¼­£¬Á¢¼´Í¨Öª¹ýÑ¹¹ÊÕÏ½â³ý */
            if (u32BusMv <= pstcCfg->u32UndervoltageThresholdMv) {
                Voltage_QueueEvent(pstcAlarm, VOLTAGE_ALARM_OVERVOLTAGE,
                                   u32BusMv, pstcCfg->u32OvervoltageThresholdMv, 0);
            }
#endif
        }
    }

    // ---- Ç·Ñ¹ ----
    if (!pstcAlarm->u8UndervoltageAlarm) {
        if (u32BusMv <= pstcCfg->u32UndervoltageThresholdMv) {
            if (!pstcAlarm->u8UndervoltageTimerRunning) {
                nbDelay_Init(&pstcAlarm->stcUndervoltageTimer, u32UvConfirmMs);
                nbDelay_Start(&pstcAlarm->stcUndervoltageTimer);
                pstcAlarm->u8UndervoltageTimerRunning = 1;
            } else if (nbDelay_IsComplete(&pstcAlarm->stcUndervoltageTimer)) {
                pstcAlarm->u8UndervoltageTimerRunning = 0;
                pstcAlarm->u8UndervoltageAlarm = 1;
                pstcAlarm->u32UndervoltageReleaseMv =
                    pstcCfg->u32UndervoltageThresholdMv + pstcCfg->u32UndervoltageHysteresisMv;

                // Ç·Ñ¹È·ÈÏºóÔÙÇå¹ýÑ¹
                if (pstcAlarm->u8OvervoltageAlarm) {
                    pstcAlarm->u8OvervoltageAlarm = 0;
                    pstcAlarm->u8OvervoltageTimerRunning = 0;
                    nbDelay_Stop(&pstcAlarm->stcOvervoltageTimer);
                    Voltage_QueueEvent(pstcAlarm, VOLTAGE_ALARM_OVERVOLTAGE,
                                       u32BusMv, pstcCfg->u32OvervoltageThresholdMv, 0);
                }

                Voltage_QueueEvent(pstcAlarm, VOLTAGE_ALARM_UNDERVOLTAGE,
                                   u32BusMv, pstcCfg->u32UndervoltageThresholdMv, 1);
            }
        } else {
            if (pstcAlarm->u8UndervoltageTimerRunning) {
                nbDelay_Stop(&pstcAlarm->stcUndervoltageTimer);
                pstcAlarm->u8UndervoltageTimerRunning = 0;
            }
        }
    } else {
        if (u32BusMv >= pstcAlarm->u32UndervoltageReleaseMv) {
            pstcAlarm->u8UndervoltageAlarm = 0;
            pstcAlarm->u8UndervoltageTimerRunning = 0;
            nbDelay_Stop(&pstcAlarm->stcUndervoltageTimer);
#if VOLTAGE_CLEAR_MODE == VOLTAGE_CLEAR_AUTO
            Voltage_QueueEvent(pstcAlarm, VOLTAGE_ALARM_UNDERVOLTAGE,
                               u32BusMv, pstcCfg->u32UndervoltageThresholdMv, 0);
#else
            /* ¿çµ½¹ýÑ¹²àÊ±ÑØÓÃÔ­Âß¼­£¬Á¢¼´Í¨ÖªÇ·Ñ¹¹ÊÕÏ½â³ý */
            if (u32BusMv >= pstcCfg->u32OvervoltageThresholdMv) {
                Voltage_QueueEvent(pstcAlarm, VOLTAGE_ALARM_UNDERVOLTAGE,
                                   u32BusMv, pstcCfg->u32UndervoltageThresholdMv, 0);
            }
#endif
        }
    }
}

void Voltage_Device_ResetDetection(Voltage_Device_t* pstcDev) {
    if (!pstcDev) return;
    Voltage_AlarmState_t* pstcAlarm = &pstcDev->stcAlarmState;
    pstcAlarm->u8OvervoltageTimerRunning = 0;
    pstcAlarm->u8UndervoltageTimerRunning = 0;
    nbDelay_Stop(&pstcAlarm->stcOvervoltageTimer);
    nbDelay_Stop(&pstcAlarm->stcUndervoltageTimer);
}

// 1ms ÖÐ¶ÏÄÚ´Ó ADC »º´æ¶ÁÈ¡µçÑ¹£¬²»ÄÜµ÷ÓÃ´øËøµÄ Device_Read
static void Voltage_ReadFromAdcIsr(Voltage_Device_t* pstcDev) {
    if (!pstcDev) return;

#ifdef VOLTAGE_SIMULATION_MODE
    pstcDev->u16AdcVoltageMv = s_u16SimAdcVoltageMv;
    pstcDev->u16AdcRawValue = SIM_ADC_RAW_VALUE(s_u16SimAdcVoltageMv);
#else
    DeviceNode_t* pstcNode = DeviceManager_Get(pstcDev->stcConfig.u8AdcDevId);
    if (pstcNode == NULL || pstcNode->private_data == NULL) return;

    ADC_Device_t* pstcAdc = (ADC_Device_t*)pstcNode->private_data;
    pstcDev->u16AdcRawValue = pstcAdc->u16RawValue;
    pstcDev->u16AdcVoltageMv = pstcAdc->u16VoltageMv;
#endif
}

void Voltage_Device_UpdateIsr(Voltage_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return;

    Voltage_ReadFromAdcIsr(pstcDev);
    Voltage_CalcBusVoltage(pstcDev);

    // ÉÏµç 120ms ÎÈ¶¨ÆÚÄÚÖ»¸üÐÂµçÑ¹£¬²»ÅÐ¶¨¸æ¾¯
    if (!pstcDev->stcStableState.u8StableDone) return;

    Voltage_CheckAlarm(pstcDev);
}

void Voltage_ProcessPendingEvents(Voltage_Device_t* pstcDev) {
    if (!pstcDev) return;

    Voltage_AlarmState_t* pstcAlarm = &pstcDev->stcAlarmState;

    if (pstcAlarm->u8OvervoltageEventPending) {
        pstcAlarm->u8OvervoltageEventPending = 0;
        Voltage_AlarmEvent_t stcEvent = pstcAlarm->stcOvervoltagePendingEvent;
        EventBus_Publish(TOPIC_VOLTAGE_ALARM, &stcEvent);
    }

    if (pstcAlarm->u8UndervoltageEventPending) {
        pstcAlarm->u8UndervoltageEventPending = 0;
        Voltage_AlarmEvent_t stcEvent = pstcAlarm->stcUndervoltagePendingEvent;
        EventBus_Publish(TOPIC_VOLTAGE_ALARM, &stcEvent);
    }
}

// ========== ï¿½ï¿½×¼ï¿½è±¸ï¿½ï¿½ï¿½ï¿½Êµï¿½ï¿½ ==========

DeviceResult_t Voltage_Device_Init(void* handle) {
    Voltage_Device_t* pstcDev = (Voltage_Device_t*)handle;
    if (!pstcDev) return RESULT_PARAM_ERR;

    if (pstcDev->stcConfig.u32OvervoltageConfirmMs == 0) {
        pstcDev->stcConfig.u32OvervoltageConfirmMs = VOLTAGE_CONFIRM_TIME_MS;
    }
    if (pstcDev->stcConfig.u32UndervoltageConfirmMs == 0) {
        pstcDev->stcConfig.u32UndervoltageConfirmMs = VOLTAGE_CONFIRM_TIME_MS;
    }

    VOLTAGE_DEBUG("Init: ADC Dev ID=%d, Divider=%lu+%lu, Ratio=%lu\r\n",
                  pstcDev->stcConfig.u8AdcDevId,
                  VOLTAGE_DIVIDER_TOP_OHM,
                  VOLTAGE_DIVIDER_BOTTOM_OHM,
                  VOLTAGE_DIVIDER_RATIO);

    VOLTAGE_DEBUG("Init: OverV Threshold=%lu mV, Hysteresis=%lu mV, Confirm=%lu ms\r\n",
                  pstcDev->stcConfig.u32OvervoltageThresholdMv,
                  pstcDev->stcConfig.u32OvervoltageHysteresisMv,
                  pstcDev->stcConfig.u32OvervoltageConfirmMs);

    VOLTAGE_DEBUG("Init: UnderV Threshold=%lu mV, Hysteresis=%lu mV, Confirm=%lu ms\r\n",
                  pstcDev->stcConfig.u32UndervoltageThresholdMv,
                  pstcDev->stcConfig.u32UndervoltageHysteresisMv,
                  pstcDev->stcConfig.u32UndervoltageConfirmMs);

#ifdef VOLTAGE_SIMULATION_MODE
    VOLTAGE_DEBUG("SIMULATION MODE ENABLED - Use sim voltage to set test value\r\n");
    VOLTAGE_DEBUG("  Voltage 2000mV = 24.00V, 2500mV = 30.00V, 3000mV = 36.00V, 3300mV = 39.60V\r\n");
#else
    VOLTAGE_DEBUG("REAL MODE - Reading from ADC device ID=%d\r\n", pstcDev->stcConfig.u8AdcDevId);
#endif

    pstcDev->u16AdcRawValue = 0;
    pstcDev->u16AdcVoltageMv = 0;
    pstcDev->u32BusVoltageMv = 0;
    pstcDev->u16BusVoltageVx100 = 0;

    pstcDev->stcStableState.u8StableDone = 0;
    pstcDev->stcStableState.u32StartTime = 0;

    memset(&pstcDev->stcAlarmState, 0, sizeof(Voltage_AlarmState_t));
    nbDelay_Init(&pstcDev->stcAlarmState.stcOvervoltageTimer, 0);
    nbDelay_Init(&pstcDev->stcAlarmState.stcUndervoltageTimer, 0);

    Voltage_ReadFromAdc(pstcDev);
    Voltage_CalcBusVoltage(pstcDev);

    pstcDev->u8Initialized = 1;
    pstcDev->u32LastUpdateTime = tickTimer_GetCount();

    VOLTAGE_DEBUG("Init success: ADC Raw=%d, ADC mV=%d, Bus mV=%lu\r\n",
                  pstcDev->u16AdcRawValue,
                  pstcDev->u16AdcVoltageMv,
                  pstcDev->u32BusVoltageMv);

    return RESULT_OK;
}

DeviceResult_t Voltage_Device_Deinit(void* handle) {
    Voltage_Device_t* pstcDev = (Voltage_Device_t*)handle;
    if (!pstcDev) return RESULT_PARAM_ERR;

    VOLTAGE_DEBUG("Deinit\r\n");
    pstcDev->u8Initialized = 0;
    return RESULT_OK;
}

DeviceResult_t Voltage_Device_Read(void* handle, void* data, uint32_t size) {
    Voltage_Device_t* pstcDev = (Voltage_Device_t*)handle;
    if (!pstcDev || !data) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;

    if (size == sizeof(Voltage_ReadResponse_t)) {
        Voltage_ReadResponse_t* pstcResp = (Voltage_ReadResponse_t*)data;
        pstcResp->u32BusVoltageMv = pstcDev->u32BusVoltageMv;
        pstcResp->u16BusVoltageVx100 = pstcDev->u16BusVoltageVx100;
        pstcResp->u16AdcRawValue = pstcDev->u16AdcRawValue;
        pstcResp->u16AdcVoltageMv = pstcDev->u16AdcVoltageMv;
        return RESULT_OK;
    }

    return RESULT_PARAM_ERR;
}

DeviceResult_t Voltage_Device_Write(void* handle, const void* data, uint32_t size) {
    (void)handle;
    (void)data;
    (void)size;
    return RESULT_ERROR;
}

DeviceResult_t Voltage_Device_Control(void* handle, DeviceCommandData_t* pstcCmd) {
    Voltage_Device_t* pstcDev = (Voltage_Device_t*)handle;
    if (!pstcDev || !pstcCmd) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;

    switch (pstcCmd->cmd) {
        case CMD_VOLTAGE_GET_BUS_MV:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(uint32_t)) {
                *(uint32_t*)pstcCmd->response = pstcDev->u32BusVoltageMv;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;

        case CMD_VOLTAGE_GET_BUS_V:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(uint16_t)) {
                *(uint16_t*)pstcCmd->response = pstcDev->u16BusVoltageVx100;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;

#ifdef VOLTAGE_SIMULATION_MODE
        case CMD_VOLTAGE_SET_SIM_VALUE:
            if (pstcCmd->params && pstcCmd->param_size >= sizeof(uint16_t)) {
                s_u16SimAdcVoltageMv = *(uint16_t*)pstcCmd->params;
                VOLTAGE_DEBUG("SIM: Voltage set to %d mV (Bus = %lu mV = %lu.%02luV)\r\n",
                              s_u16SimAdcVoltageMv,
                              (uint32_t)s_u16SimAdcVoltageMv * VOLTAGE_DIVIDER_RATIO,
                              ((uint32_t)s_u16SimAdcVoltageMv * VOLTAGE_DIVIDER_RATIO) / 1000UL,
                              ((uint32_t)s_u16SimAdcVoltageMv * VOLTAGE_DIVIDER_RATIO / 10UL) % 100UL);
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
#endif

        default:
            return RESULT_ERROR;
    }
}

DeviceResult_t Voltage_Device_Update(void* handle) {
    Voltage_Device_t* pstcDev = (Voltage_Device_t*)handle;
    if (!pstcDev || !pstcDev->u8Initialized) return RESULT_ERROR;

    // ÉÏµçÎÈ¶¨ÑÓÊ±ÈÔÔÚÖ÷Ñ­»·ÀïÍê³É£»µçÑ¹²É¼¯ºÍÅÐ¶¨ÒÑÒÆµ½ 1ms ÖÐ¶Ï
    if (!pstcDev->stcStableState.u8StableDone) {
        if (pstcDev->stcStableState.u32StartTime == 0) {
            pstcDev->stcStableState.u32StartTime = tickTimer_GetCount();
            VOLTAGE_DEBUG("Voltage stabilization started...\r\n");
            return RESULT_OK;
        }

        uint32_t u32Elapsed = tickTimer_GetCount() - pstcDev->stcStableState.u32StartTime;
        if (u32Elapsed < VOLTAGE_STABLE_DELAY_MS) {
            pstcDev->u32LastUpdateTime = tickTimer_GetCount();
            return RESULT_OK;
        }

        pstcDev->stcStableState.u8StableDone = 1;
        VOLTAGE_DEBUG("Voltage stabilized after %lu ms, enabling alarm detection\r\n",
                      (unsigned long)u32Elapsed);
    }

    // LED µøÂäµ÷ÊÔ£º±£³ÖÔ­À´µÄÊµÊ±¸ú×ÙÐÐÎª
    {
        static uint8_t s_u8LedDipLastState = 0xFFu;
        uint8_t u8LedNowState = (pstcDev->u32BusVoltageMv < VOLTAGE_LED_DIP_TRIG_MV) ? 1u : 0u;
        if (u8LedNowState != s_u8LedDipLastState) {
            s_u8LedDipLastState = u8LedNowState;
            LL_PERIPH_WE(LL_PERIPH_GPIO);
            if (u8LedNowState != 0u) {
                GPIO_RESET(GPIO_LED_PORT, GPIO_LED_PIN);
            } else {
                GPIO_SET(GPIO_LED_PORT, GPIO_LED_PIN);
            }
            LL_PERIPH_WP(LL_PERIPH_GPIO);
            VOLTAGE_DEBUG("LED DIP: bus=%lu mV, LED %s\r\n",
                          (unsigned long)pstcDev->u32BusVoltageMv,
                          (u8LedNowState != 0u) ? "ON" : "OFF");
        }
    }

    VOLTAGE_DEBUG("[VOLTAGE] Bus=%lu.%02luV (ADC Raw=%d, ADC mV=%d)%s%s\r\n",
           pstcDev->u16BusVoltageVx100 / 100UL,
           (unsigned long)pstcDev->u16BusVoltageVx100 % 100UL,
           pstcDev->u16AdcRawValue,
           pstcDev->u16AdcVoltageMv,
           pstcDev->stcAlarmState.u8OvervoltageAlarm ? " [OVERVOLTAGE]" : "",
           pstcDev->stcAlarmState.u8UndervoltageAlarm ? " [UNDERVOLTAGE]" : "");

    pstcDev->u32LastUpdateTime = tickTimer_GetCount();
    return RESULT_OK;
}

// ========== ï¿½ï¿½Ñ¹Ä¸ï¿½ï¿½ï¿½Ø¶ï¿½ï¿½Ó¿ï¿½ ==========

uint32_t Voltage_Device_GetBusVoltageMV(Voltage_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->u32BusVoltageMv;
}

uint16_t Voltage_Device_GetBusVoltageVx100(Voltage_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->u16BusVoltageVx100;
}

Voltage_Device_t* Voltage_Device_Create(const Voltage_Config_t* pstcConfig) {
    if (!pstcConfig) return NULL;

    Voltage_Device_t* pstcDev = (Voltage_Device_t*)malloc(sizeof(Voltage_Device_t));
    if (!pstcDev) return NULL;

    memset(pstcDev, 0, sizeof(Voltage_Device_t));
    pstcDev->stcConfig = *pstcConfig;

    if (pstcDev->stcConfig.u32OvervoltageConfirmMs == 0) {
        pstcDev->stcConfig.u32OvervoltageConfirmMs = VOLTAGE_CONFIRM_TIME_MS;
    }
    if (pstcDev->stcConfig.u32UndervoltageConfirmMs == 0) {
        pstcDev->stcConfig.u32UndervoltageConfirmMs = VOLTAGE_CONFIRM_TIME_MS;
    }

    pstcDev->u8Initialized = 0;

    VOLTAGE_DEBUG("Create: ADC Dev ID=%d\r\n", pstcConfig->u8AdcDevId);
    VOLTAGE_DEBUG("Create: OverV Threshold=%lu mV, Hysteresis=%lu mV, Confirm=%lu ms\r\n",
                  pstcDev->stcConfig.u32OvervoltageThresholdMv,
                  pstcDev->stcConfig.u32OvervoltageHysteresisMv,
                  pstcDev->stcConfig.u32OvervoltageConfirmMs);
    VOLTAGE_DEBUG("Create: UnderV Threshold=%lu mV, Hysteresis=%lu mV, Confirm=%lu ms\r\n",
                  pstcDev->stcConfig.u32UndervoltageThresholdMv,
                  pstcDev->stcConfig.u32UndervoltageHysteresisMv,
                  pstcDev->stcConfig.u32UndervoltageConfirmMs);

    return pstcDev;
}

// ========== Ä£ï¿½ï¿½Ä£Ê½ï¿½Ó¿ï¿½ ==========
#ifdef VOLTAGE_SIMULATION_MODE
void Voltage_SetSimulationValue(uint16_t u16VoltageMv) {
    s_u16SimAdcVoltageMv = u16VoltageMv;
    VOLTAGE_DEBUG("SIM: Voltage set to %d mV via API (Bus = %lu mV)\r\n",
                  u16VoltageMv, (uint32_t)u16VoltageMv * VOLTAGE_DIVIDER_RATIO);
}
#endif

// ========== ï¿½ï¿½Ñ¹ï¿½æ¾¯ï¿½ï¿½ï¿½ï¿½Ó¿Ú£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¶ï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ï¿½ ==========
DeviceResult_t Voltage_Device_ClearAlarm(Voltage_Device_t* pstcDev, uint8_t u8AlarmType) {
    if (!pstcDev || !pstcDev->u8Initialized) return RESULT_PARAM_ERR;

    Voltage_AlarmState_t* pstcAlarm = &pstcDev->stcAlarmState;

    switch (u8AlarmType) {
    case VOLTAGE_ALARM_OVERVOLTAGE:
        pstcAlarm->u8OvervoltageTimerRunning = 0;
        nbDelay_Stop(&pstcAlarm->stcOvervoltageTimer);
        pstcAlarm->u8OvervoltageEventPending = 0;

        if (pstcAlarm->u8OvervoltageAlarm) {
            pstcAlarm->u8OvervoltageAlarm = 0;

            VOLTAGE_DEBUG("ClearAlarm: Overvoltage cleared by manual command\r\n");

            Voltage_AlarmEvent_t stcEvent;
            stcEvent.u8AlarmType = VOLTAGE_ALARM_OVERVOLTAGE;
            stcEvent.u32BusVoltageMv = pstcDev->u32BusVoltageMv;
            stcEvent.u32ThresholdMv = pstcDev->stcConfig.u32OvervoltageThresholdMv;
            stcEvent.u8IsActive = 0;
            EventBus_Publish(TOPIC_VOLTAGE_ALARM, &stcEvent);
        }
        return RESULT_OK;

    case VOLTAGE_ALARM_UNDERVOLTAGE:
        pstcAlarm->u8UndervoltageTimerRunning = 0;
        nbDelay_Stop(&pstcAlarm->stcUndervoltageTimer);
        pstcAlarm->u8UndervoltageEventPending = 0;

        if (pstcAlarm->u8UndervoltageAlarm) {
            pstcAlarm->u8UndervoltageAlarm = 0;

            VOLTAGE_DEBUG("ClearAlarm: Undervoltage cleared by manual command\r\n");

            Voltage_AlarmEvent_t stcEvent;
            stcEvent.u8AlarmType = VOLTAGE_ALARM_UNDERVOLTAGE;
            stcEvent.u32BusVoltageMv = pstcDev->u32BusVoltageMv;
            stcEvent.u32ThresholdMv = pstcDev->stcConfig.u32UndervoltageThresholdMv;
            stcEvent.u8IsActive = 0;
            EventBus_Publish(TOPIC_VOLTAGE_ALARM, &stcEvent);
        }
        return RESULT_OK;

    default:
        return RESULT_PARAM_ERR;
    }
}

// ========== È«ï¿½Ö²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ==========
const DeviceOps_t g_voltage_ops = {
    .init = Voltage_Device_Init,
    .deinit = Voltage_Device_Deinit,
    .read = Voltage_Device_Read,
    .write = Voltage_Device_Write,
    .control = Voltage_Device_Control,
    .update = Voltage_Device_Update
};
