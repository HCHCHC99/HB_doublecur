#ifndef DEV_VOLTAGE_H_
#define DEV_VOLTAGE_H_

#include "device_manager.h"
#include "dev_adc.h"
#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"

// ========== 调试宏定义 ==========
// 所有调试输出统一使用 rtt_manager.h 中的宏，通过 DEV_VOLTAGE 控制开关

#ifdef DEV_VOLTAGE
    #define VOLTAGE_DEBUG(fmt, ...)    MAIN_D("[VOLTAGE_DEBUG] " fmt, ##__VA_ARGS__)
#else
    #define VOLTAGE_DEBUG(fmt, ...)    ((void)0)
#endif

// ========== 上电稳定配置 ==========
#define VOLTAGE_STABLE_DELAY_MS     (120U)   // 上电后等待 ADC 稳定的时间 (ms)

// ========== 模拟模式宏定义 ==========
// 启用模拟模式时，不需要实际ADC设备，可直接设置模拟值
// #define VOLTAGE_SIMULATION_MODE      // 启用电压模拟模式

// ========== 过压/欠压告警清除模式 ==========
// VOLTAGE_CLEAR_AUTO:   电压恢复正常后自动清除告警状态并移除block
// VOLTAGE_CLEAR_MANUAL: 电压恢复正常后仍保持告警状态，需通过 Modbus 写 REG_FAULT_STATUS 清除
#define VOLTAGE_CLEAR_AUTO      0
#define VOLTAGE_CLEAR_MANUAL    1

#ifndef VOLTAGE_CLEAR_MODE
#define VOLTAGE_CLEAR_MODE      VOLTAGE_CLEAR_MANUAL
#endif

// ========== 电压分压电阻配置 ==========
// 分压比 = (上拉 + 下拉) / 下拉
// 上拉电阻：110kΩ + 10kΩ
// 分压比 = (110 + 10) / 10 = 12
#include "main.h"
#if BOARD_VERSION == 0
    // 原HB_chuchai板：110kΩ 上分压电阻
    #define VOLTAGE_DIVIDER_TOP_OHM        (100000UL)
#else
    // 整合板：150kΩ 上分压电阻
    #define VOLTAGE_DIVIDER_TOP_OHM        (150000UL)
#endif
#define VOLTAGE_DIVIDER_BOTTOM_OHM     (3300UL)    // 下分压电阻 3.3kΩ
#define VOLTAGE_DIVIDER_RATIO          ((VOLTAGE_DIVIDER_TOP_OHM + VOLTAGE_DIVIDER_BOTTOM_OHM) / VOLTAGE_DIVIDER_BOTTOM_OHM)

/* 电压补偿值 (mV)，用于抵消二极管压降/线路损耗等 */
#define VOLTAGE_COMPENSATION_MV         (400UL)

// ========== 电压设备命令定义 ==========
#define CMD_VOLTAGE_GET_BUS_MV         (CMD_BASE_ADC + 0x10)   // 获取母线电压(mV)
#define CMD_VOLTAGE_GET_BUS_V          (CMD_BASE_ADC + 0x11)   // 获取母线电压(V，放大100倍)
#ifdef VOLTAGE_SIMULATION_MODE
#define CMD_VOLTAGE_SET_SIM_VALUE      (CMD_BASE_ADC + 0x12)   // 设置模拟电压值(mV)
#endif

// ========== 电压设备配置结构体 ==========
typedef struct {
    uint8_t     u8AdcDevId;             // 对应的ADC设备ID（如 ID_ADC_VOLTAGE）

    // 过压配置
    uint32_t    u32OvervoltageThresholdMv;   // 过压阈值(mV)
    uint32_t    u32OvervoltageHysteresisMv;  // 过压迟滞值(mV)
    uint8_t     u8OvervoltageTriggerCount;   // 连续过压检测触发次数

    // 欠压配置
    uint32_t    u32UndervoltageThresholdMv;   // 欠压阈值(mV)
    uint32_t    u32UndervoltageHysteresisMv;  // 欠压迟滞值(mV)
    uint8_t     u8UndervoltageTriggerCount;   // 连续低于欠压阈值触发次数
} Voltage_Config_t;

// ========== 过压/欠压告警状态 ==========
typedef struct {
    // 过压告警
    uint8_t  u8OvervoltageAlarm;         // 过压告警状态：0=正常,1=告警触发
    uint8_t  u8OvervoltageCount;         // 连续过压检测次数
    uint32_t u32OvervoltageReleaseMv;    // 过压释放阈值(mV) = 阈值 - 迟滞

    // 欠压告警
    uint8_t  u8UndervoltageAlarm;        // 欠压告警状态：0=正常,1=告警触发
    uint8_t  u8UndervoltageCount;        // 连续低于欠压阈值次数
    uint32_t u32UndervoltageReleaseMv;   // 欠压释放阈值(mV) = 阈值 + 迟滞
} Voltage_AlarmState_t;

// ========== 电压告警事件结构体（用于EventBus发布） ==========
typedef struct {
    uint8_t  u8AlarmType;               // 告警类型：0=过压, 1=欠压
    uint32_t u32BusVoltageMv;           // 当前母线电压值(mV)
    uint32_t u32ThresholdMv;            // 阈值(mV)
    uint8_t  u8IsActive;                // 1=告警触发, 0=告警解除
} Voltage_AlarmEvent_t;

// 告警类型定义
#define VOLTAGE_ALARM_OVERVOLTAGE   0
#define VOLTAGE_ALARM_UNDERVOLTAGE  1

// ========== 电压设备结构体 ==========
typedef struct {
    Voltage_Config_t    stcConfig;      // 设备配置（阈值、迟滞等）
    uint8_t             u8Initialized;  // 初始化标志

    // 实时数据
    uint16_t            u16AdcRawValue;     // 从ADC读取的原始值
    uint16_t            u16AdcVoltageMv;    // ADC采样电压值(mV) = 下拉电阻电压
    uint32_t            u32BusVoltageMv;    // 母线电压值(mV) = ADC电压 * 分压比
    uint16_t            u16BusVoltageVx100; // 母线电压(V)*100，用于精确调试显示 2400 = 24.00V

    // 过压/欠压告警状态
    Voltage_AlarmState_t stcAlarmState;

    // 更新计时
    uint32_t            u32LastUpdateTime;

    struct {
        uint8_t  u8StableDone;       // 上电稳定延时是否已完成
        uint32_t u32StartTime;       // 延时开始时间
    } stcStableState;

} Voltage_Device_t;

// ========== 读响应结构体 ==========
typedef struct {
    uint32_t u32BusVoltageMv;       // 母线电压(mV)
    uint16_t u16BusVoltageVx100;    // 母线电压(V)*100
    uint16_t u16AdcRawValue;        // ADC原始值
    uint16_t u16AdcVoltageMv;       // ADC采样电压值(mV)
} Voltage_ReadResponse_t;

// ========== 标准设备操作（符合Device Manager接口） ==========
DeviceResult_t Voltage_Device_Init(void* handle);
DeviceResult_t Voltage_Device_Deinit(void* handle);
DeviceResult_t Voltage_Device_Read(void* handle, void* data, uint32_t size);
DeviceResult_t Voltage_Device_Write(void* handle, const void* data, uint32_t size);
DeviceResult_t Voltage_Device_Control(void* handle, DeviceCommandData_t* cmd);
DeviceResult_t Voltage_Device_Update(void* handle);

// ========== 电压值获取接口 ==========
uint32_t Voltage_Device_GetBusVoltageMV(Voltage_Device_t* pstcDev);
uint16_t Voltage_Device_GetBusVoltageVx100(Voltage_Device_t* pstcDev);
Voltage_Device_t* Voltage_Device_Create(const Voltage_Config_t* pstcConfig);

// ========== 模拟模式接口 ==========
#ifdef VOLTAGE_SIMULATION_MODE
/**
 * @brief 设置模拟电压值，用于离线调试
 * @param u16VoltageMv 模拟的ADC采样电压值(mV)，对应下拉电阻端电压
 * @note 母线电压 = 模拟电压 * 分压比
 *       例如设置2000mV 则 母线电压 = 2000*12 = 24000mV = 24.00V
 */
void Voltage_SetSimulationValue(uint16_t u16VoltageMv);
#endif

// ========== 电压告警清除接口（用于手动清除故障） ==========
/**
 * @brief 清除电压告警状态（过压或欠压）
 * @param pstcDev 电压设备指针
 * @param u8AlarmType 告警类型：VOLTAGE_ALARM_OVERVOLTAGE 或 VOLTAGE_ALARM_UNDERVOLTAGE
 * @return DeviceResult_t
 * @note 仅在 VOLTAGE_CLEAR_MODE == VOLTAGE_CLEAR_MANUAL 时使用
 *       清除后需要在下一次 Update 时重新检测告警状态
 */
DeviceResult_t Voltage_Device_ClearAlarm(Voltage_Device_t* pstcDev, uint8_t u8AlarmType);

// ========== 全局操作函数表 ==========
extern const DeviceOps_t g_voltage_ops;

#endif /* DEV_VOLTAGE_H_ */
