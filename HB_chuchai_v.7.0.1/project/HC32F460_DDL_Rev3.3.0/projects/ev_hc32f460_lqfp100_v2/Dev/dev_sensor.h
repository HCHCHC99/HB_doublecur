#ifndef DEV_SENSOR_H_
#define DEV_SENSOR_H_

#include "device_manager.h"
#include "dev_adc.h"
#include "TickTimer.h"
#include <stdint.h>
#include <stdbool.h>
#include "Adapter.h"
#include "rtt_manager.h"

// ========== 电压分压模式选择 ==========
// 0: 无分压 - 传感器输出直接接入 MCU 0~3.3V 对应0~2A 量程
// 1: 有分压 - 外部分压电路接入
#define SENSOR_VOLTAGE_DIVIDER_ENABLE    0   // 1=有分压, 0=无分压


// ========== 校准配置 ==========
#define SENSOR_CALIB_DELAY_MS       (120U)   // 上电后等待时间，避开尖峰电流 (ms)
#define SENSOR_CALIB_STABLE_MS      (50U)    // ADC 稳定等待时间 (ms)


// ========== 电压分压参数（有分压模式时生效） ==========
#if SENSOR_VOLTAGE_DIVIDER_ENABLE
    #define SENSOR_DIVIDER_R1            (10000)   // 上分压电阻 R1 (欧姆)
    #define SENSOR_DIVIDER_R2            (20000)   // 下分压电阻 R2 (欧姆)


#endif

#ifdef DEV_SENSOR_EMA_DEBUG
    #define SENSOR_EMA_DBG(fmt, ...)    MAIN_D("[SENSOR_EMA] " fmt, ##__VA_ARGS__)
#else
    #define SENSOR_EMA_DBG(fmt, ...)    ((void)0)
#endif


// ========== 硬件板本：传感器类型选择 ==========
// 在 main.h 中通过 BOARD_VERSION 统一管理
#include "main.h"
#if BOARD_VERSION == 0
    // 原HB_chuchai板：霍尔电流传感器 (零点1650mV, 灵敏度66mV/A, 量程±25A)
    #define SENSOR_TYPE_DIFF_AMP_ENABLE    0
#else
    // 整合板：差分运放 (Vout = I * 0.1, 0mV为0A, 100mV/A)
    #define SENSOR_TYPE_DIFF_AMP_ENABLE    1
#endif


#ifdef DEV_SENSOR_TIMER_DEBUG
    #define SENSOR_TIMER_DBG(fmt, ...)    MAIN_D("[SENSOR_TIMER] " fmt, ##__VA_ARGS__)
#else
    #define SENSOR_TIMER_DBG(fmt, ...)    ((void)0)
#endif


// ========== 调试宏定义 ==========
#ifdef DEV_SENSOR
    #define SENSOR_DEBUG(fmt, ...)    MAIN_D("[SENSOR] " fmt, ##__VA_ARGS__)
#else
    #define SENSOR_DEBUG(fmt, ...)    ((void)0)
#endif

#ifdef DEV_SENSOR_REAL
    #define SENSOR_REAL_DEBUG(fmt, ...)    MAIN_D("[SENSOR_REAL] " fmt, ##__VA_ARGS__)
#else
    #define SENSOR_REAL_DEBUG(fmt, ...)    ((void)0)
#endif

#ifdef DEV_SENSOR_SLOW
    #define SENSOR_DEBUG_SLOW(fmt, ...)    MAIN_D("[SENSOR] " fmt, ##__VA_ARGS__)
#else
    #define SENSOR_DEBUG_SLOW(fmt, ...)    ((void)0)
#endif

// ========== 模拟模式宏定义 ==========
// #define SENSOR_SIMULATION_MODE

// ========== 过流检测模式 ==========
#define OVERCURRENT_MODE_SAMPLE_COUNT    0
#define OVERCURRENT_MODE_TIME_WINDOW     1

// ========== 窗口占比判据 ==========
// 窗口内 >= 阈值的样本占比 >= OVERCURRENT_WINDOW_PERCENT(%) 即该点过流。
// 开窗与关窗使用同一判据。
#define OVERCURRENT_WINDOW_PERCENT        (20U)

// ========== 过流告警清除模式选择 ==========
#define OVERCURRENT_CLEAR_AUTO      0
#define OVERCURRENT_CLEAR_MANUAL    1

#ifndef OVERCURRENT_CLEAR_MODE
#define OVERCURRENT_CLEAR_MODE      OVERCURRENT_CLEAR_MANUAL   
#endif

// ========== 判定方向（zone） ==========
#define SENSOR_ZONE_STOP             0U
#define SENSOR_ZONE_OPEN             1U   /* 开窗（正转） */
#define SENSOR_ZONE_CLOSE            2U   /* 关窗（反转） */

// ========== 报警状态位（CMD_SENSOR_GET_ALARM_STATUS 返回值） ==========
#define SENSOR_ALARM_BIT_CLOSE       0x01U
#define SENSOR_ALARM_BIT_OPEN        0x02U

// ========== DEBUG 窗口缓冲区 ==========
#define DEBUG_SENSOR_WINDOW_BUFFER
#define SENSOR_WINDOW_BUFFER_SIZE     (200)

// ========== 传感器硬件参数（根据板型自动选择） ==========
#if SENSOR_TYPE_DIFF_AMP_ENABLE
    // ===== 差分运放模式参数 (3.3V供电, ±2.5A量程) =====
    // 零点=1650mV(Vcc/2), 灵敏度=264mV/A
    #define SENSOR_RAW_ZERO_MV             (0)
    #define SENSOR_RAW_SENSITIVITY_MV_PER_A (100)
#else
    // ===== 霍尔电流传感器模式参数 (原HB_chuchai) =====
    #define SENSOR_RAW_ZERO_MV             (1650)
    #define SENSOR_RAW_SENSITIVITY_MV_PER_A (264)
#endif

#if SENSOR_VOLTAGE_DIVIDER_ENABLE
    #define SENSOR_VOUT_ZERO_MA_INT     (SENSOR_RAW_ZERO_MV * SENSOR_DIVIDER_R2 / (SENSOR_DIVIDER_R1 + SENSOR_DIVIDER_R2))
    #define SENSOR_SENSITIVITY_INT      (SENSOR_RAW_SENSITIVITY_MV_PER_A * SENSOR_DIVIDER_R2 / (SENSOR_DIVIDER_R1 + SENSOR_DIVIDER_R2))
    #define SENSOR_VOUT_ZERO_MV         ((float)SENSOR_VOUT_ZERO_MA_INT)
    #define SENSOR_SENSITIVITY_MV_PER_A ((float)SENSOR_SENSITIVITY_INT)
#else
    #define SENSOR_VOUT_ZERO_MA_INT     (SENSOR_RAW_ZERO_MV)
    #define SENSOR_SENSITIVITY_INT      (SENSOR_RAW_SENSITIVITY_MV_PER_A)
    #define SENSOR_VOUT_ZERO_MV         ((float)SENSOR_VOUT_ZERO_MA_INT)
    #define SENSOR_SENSITIVITY_MV_PER_A ((float)SENSOR_SENSITIVITY_INT)
#endif

// ========== 100ms 平均电流（Modbus 0x2733 实时显示用） ==========
// 与过流检测解耦：过流检测仍使用窗口占比判据，不受该平均值影响
#define SENSOR_AVG_WINDOW_MS         (100U)   // 平均窗口时长 (ms)

// ========== 传感器设备命令定义 ==========
#define CMD_SENSOR_GET_CURRENT_MA      (CMD_BASE_ADC + 0x20)
#define CMD_SENSOR_GET_CURRENT_AX100   (CMD_BASE_ADC + 0x21)
#define CMD_SENSOR_SET_SIM_VALUE       (CMD_BASE_ADC + 0x22)
#define CMD_SENSOR_GET_ALARM_STATUS    (CMD_BASE_ADC + 0x23)
#define CMD_SENSOR_GET_CALIBRATION     (CMD_BASE_ADC + 0x24)
#define CMD_SENSOR_GET_CURRENT_MA_AVG  (CMD_BASE_ADC + 0x25)   /* 100ms 窗口平均电流 (mA) */

// ========== 校准参数结构体 ==========
typedef struct {
    int32_t s32ZeroOffsetMv;
    int16_t s16SensitivityScale;
    int32_t s32CalibrationValid;
} Sensor_Calibration_t;

// ========== 单套过流判定配置 ==========
typedef struct {
    int32_t     s32OvercurrentThresholdMa;
    int32_t     s32OvercurrentHysteresisMa;
    uint8_t     u8OvercurrentMode;
    uint16_t    u16TriggerWindowSize;
    uint16_t    u16ReleaseWindowSize;
    uint32_t    u32TriggerWindowMs;
    uint32_t    u32ReleaseWindowMs;
} Overcurrent_Config_t;

// ========== 传感器设备配置 ==========
typedef struct {
    uint8_t     u8AdcDevId;
    Overcurrent_Config_t stcCloseOc;   /* 关窗专用过流判定 */
    Overcurrent_Config_t stcOpenOc;    /* 开窗专用过流判定 */
} Sensor_Config_t;

// ========== 单套告警状态 ==========
typedef struct {
    uint8_t  u8OvercurrentAlarm;
    uint16_t u16ConsecutiveCount;
    NonBlockingDelay_t stcTriggerTimer;
    NonBlockingDelay_t stcReleaseTimer;
    uint8_t  u8TimerRunning;
    volatile uint8_t u8OcTriggerPending;   /* ISR置位: 过流触发待主循环发布 */
    volatile uint8_t u8OcReleasePending;   /* ISR置位: 过流恢复待主循环发布 */
    int32_t  s32PendingCurrentMa;          /* 触发时刻电流(mA) */
    int32_t  s32PendingThresholdMa;        /* 触发时刻阈值(mA) */
} Sensor_AlarmState_t;

// ========== 过流告警事件结构体 ==========
typedef struct {
    int32_t  s32CurrentMa;
    int32_t  s32ThresholdMa;
    uint8_t  u8IsActive;
    uint8_t  u8Zone;                       /* SENSOR_ZONE_xxx */
} Current_AlarmEvent_t;

// ========== 传感器设备结构体 ==========
typedef struct {
    Sensor_Config_t     stcConfig;
    uint8_t             u8Initialized;
    Sensor_Calibration_t stcCalibration;
    uint16_t            u16AdcRawValue;
    uint16_t            u16AdcVoltageMv;
    int32_t             s32CurrentMa;
    int16_t             s16CurrentAx100;
    Sensor_AlarmState_t stcCloseAlarmState; /* 关窗判定状态 */
    Sensor_AlarmState_t stcOpenAlarmState;  /* 开窗判定状态 */
    uint32_t            u32LastUpdateTime;
    uint8_t             u8Calibrated;
    uint32_t            u32InitTime;

    /* 100ms 平均电流状态（主循环维护） */
    int32_t             s32AvgCurrentMa;    /* 上一个完整窗口的平均电流 (mA) */
    int32_t             s32AvgAccumMa;      /* 窗口内样本累加 (mA) */
    uint32_t            u32AvgSampleCnt;    /* 窗口内样本数 */
    uint32_t            u32AvgWindowStart;  /* 窗口起始 tick (ms) */

    struct {
        uint8_t  u8CalibDelayDone;   // 120ms 延时是否已完成
        uint32_t u32CalibStartTime;  // 50ms 校准开始时间
    } stcCalibState;
        
} Sensor_Device_t;

// ========== 读响应结构体 ==========
typedef struct {
    int32_t  s32CurrentMa;
    int16_t  s16CurrentAx100;
    uint16_t u16AdcRawValue;
    uint16_t u16AdcVoltageMv;
} Sensor_ReadResponse_t;

// ========== 标准设备接口 ==========
DeviceResult_t Sensor_Device_Init(void* handle);
DeviceResult_t Sensor_Device_Deinit(void* handle);
DeviceResult_t Sensor_Device_Read(void* handle, void* data, uint32_t size);
DeviceResult_t Sensor_Device_Write(void* handle, const void* data, uint32_t size);
DeviceResult_t Sensor_Device_Control(void* handle, DeviceCommandData_t* cmd);
DeviceResult_t Sensor_Device_Update(void* handle);
void Sensor_Device_UpdateIsr(Sensor_Device_t* pstcDev, uint8_t u8Zone);   /* 1ms ISR快速路径 */

// ========== 传感器取值接口 ==========
int32_t Sensor_Device_GetCurrentMA(Sensor_Device_t* pstcDev);
int16_t Sensor_Device_GetCurrentAx100(Sensor_Device_t* pstcDev);
Sensor_Device_t* Sensor_Device_Create(const Sensor_Config_t* pstcConfig);

// ========== 校准接口 ==========
void Sensor_Device_CalibrateZero(Sensor_Device_t* pstcDev);
void Sensor_Device_SetSensitivityScale(Sensor_Device_t* pstcDev, int16_t s16ScalePercent);
void Sensor_Device_GetCalibration(Sensor_Device_t* pstcDev, Sensor_Calibration_t* pstcCal);

// ========== 模拟模式接口 ==========
#ifdef SENSOR_SIMULATION_MODE
void Sensor_SetSimulationValue(uint16_t u16VoltageMv);      // 设置模拟的原始电压(mV)
void Sensor_SetSimulationCurrent(int32_t s32CurrentMa);     // 直接设置模拟电流值(mA)
uint16_t Sensor_GetSimulationSensorRawMv(void);             // 读取当前模拟传感器的原始值
#endif

// ========== 过流告警手动清除接口 ==========
void Sensor_Device_ClearAlarm(Sensor_Device_t* pstcDev);

// ========== 全局操作函数表 ==========
extern const DeviceOps_t g_sensor_ops;

#endif /* DEV_SENSOR_H_ */
