#ifndef DEV_MOTOR_H_
#define DEV_MOTOR_H_

#include "device_manager.h"
#include "EventBus.h"
#include "dev_power.h"
#include "dev_hall.h"
#include "dev_voltage.h"
#include "dev_adc.h"
#include "dev_sensor.h"
#include "dev_rturn.h"
#include "dev_motor_hall.h"
#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"

// ========== ï¿½ï¿½ï¿½Ôºê¶¨ï¿½ï¿½ ==========
// ï¿½ï¿½ï¿½Ðµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Í³Ò»Ê¹ï¿½ï¿? rtt_manager.h ï¿½ÐµÄºê£¬Í¨ï¿½ï¿½ DEV_MOTOR ï¿½ï¿½ï¿½Æ¿ï¿½ï¿½ï¿½

#ifdef DEV_MOTOR
    #define MOTOR_DEBUG(fmt, ...)    MAIN_D("[MOTOR_DEBUG] " fmt, ##__VA_ARGS__)
    #define MOTOR_OUT(fmt, ...)      MAIN_D("[MOTOR_OUT] " fmt, ##__VA_ARGS__)
#else
    #define MOTOR_DEBUG(fmt, ...)    ((void)0)
    #define MOTOR_OUT(fmt, ...)      ((void)0)
#endif

// ========== Ó²ï¿½ï¿½ï¿½æ±¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê? ==========
// ï¿½ï¿½ main.h ï¿½ï¿½Í¨ï¿½ï¿½ BOARD_VERSION Í³Ò»ï¿½ï¿½ï¿½ï¿½
#include "main.h"
#if BOARD_VERSION == 0
    // Ô­HB_chuchaiï¿½å£ºGPIO PB8/PB9 Ö±ï¿½Ó¿ï¿½ï¿½ï¿½ï¿½ï¿½×ª/ï¿½ï¿½×ª/Í£Ö¹
    #define MOTOR_CONTROL_MODE  0
#else
    // ï¿½ï¿½ï¿½Ï°å£º4Í¨ï¿½ï¿½ PWM Õ¼ï¿½Õ±È¿ï¿½ï¿½Æ£ï¿½ï¿½ï¿½ï¿½ï¿½/ï¿½ï¿½Í£
    #define MOTOR_CONTROL_MODE  1
#endif

// ========== ï¿½ï¿½ï¿½ï¿½è±¸ï¿½ï¿½ï¿½î¶¨ï¿½ï¿? ==========
// ×¢ï¿½â£ºCMD_BASE_MOTOR ï¿½ï¿½ device_manager.h ï¿½ï¿½Ã»ï¿½ï¿½Ô¤ï¿½ï¿½ï¿½å£¬ï¿½Ú´Ë¶ï¿½ï¿½ï¿½
#define CMD_BASE_MOTOR              0x9000
#define CMD_MOTOR_STOP              (CMD_BASE_MOTOR + 0x01)
#define CMD_MOTOR_RUN_FWD           (CMD_BASE_MOTOR + 0x02)
#define CMD_MOTOR_RUN_REV           (CMD_BASE_MOTOR + 0x03)
#define CMD_MOTOR_SET_SPEED         (CMD_BASE_MOTOR + 0x04)
#define CMD_MOTOR_EMERGENCY_STOP    (CMD_BASE_MOTOR + 0x05)
// ï¿½ï¿½ï¿½Óµï¿½ dev_motor.h ï¿½Ðµï¿½ï¿½ï¿½ï¿½î¶¨ï¿½ï¿½ï¿½ï¿½Õ¹
#define CMD_MOTOR_GET_DESIRED_DIR   (CMD_BASE_MOTOR + 0x06)   // ï¿½ï¿½È¡ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù²Ã½ï¿½ï¿½ï¿½ï¿?

// ========== ï¿½ï¿½ï¿½ï¿½è±¸ï¿½ï¿½ï¿½Ãºï¿? ==========
// Ä£Ê½ï¿½Ð»ï¿½ï¿½ê£º0=ï¿½ï¿½ï¿½ï¿½ï¿½Ô£ï¿½ï¿½ï¿½ï¿½ï¿½Ô´È«ï¿½Å£ï¿½ï¿½ï¿½1=Ë«ï¿½ï¿½ï¿½Ô£ï¿½Ë«ï¿½ï¿½Ô´È«ï¿½Å£ï¿½
#ifndef MOTOR_MODE_BIPOLAR
#define MOTOR_MODE_BIPOLAR      0
#endif

// ï¿½ï¿½ï¿½È¼ï¿½Ä£Ê½ï¿½ï¿½ï¿½Ã£ï¿½1=IOï¿½ï¿½ï¿½È¼ï¿½ï¿½ï¿½, 0=CANï¿½ï¿½ï¿½È¼ï¿½ï¿½ï¿½
#ifndef MOTOR_PRIORITY_IO_HIGH
#define MOTOR_PRIORITY_IO_HIGH  1
#endif

// ï¿½è±¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ã£ï¿½Î»ï¿½ï¿½ï¿½ï¿½
#ifndef MOTOR_MANUAL_CAPABILITY
#define MOTOR_MANUAL_CAPABILITY  (CAP_ALLOW | CAP_BLOCK)  // IOï¿½è±¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ë¶ï¿½Ò²ï¿½ï¿½ï¿½Ô½ï¿½Ö¹ï¿½Ë¶ï¿½
#endif

#ifndef MOTOR_CAN_CAPABILITY
#define MOTOR_CAN_CAPABILITY     (CAP_ALLOW)  // CANï¿½è±¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ë¶ï¿½
#endif

// ï¿½è±¸ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½ï¿½ï¿½
#define CAP_BLOCK      (1 << 0)
#define CAP_ALLOW      (1 << 1)

// ========== ï¿½ï¿½ï¿½ï¿½Ã¶ï¿½Ù¶ï¿½ï¿½ï¿½ ==========
// ï¿½ï¿½ï¿½×´Ì¬ï¿½á¹¹ï¿½å£¨ï¿½ï¿½ï¿½ï¿? Device_Read Ò»ï¿½ï¿½ï¿½Ô¶ï¿½È¡ï¿½ï¿½
typedef enum {
    DIR_NONE = 0,
    DIR_FWD = 1,
    DIR_REV = 2
} MotorDir_t;

typedef enum {
    MODE_NONE = 0,
    MODE_AUTO = 1,
    MODE_REMOTE = 2,
    MODE_MANUAL = 3
} MotorMode_t;

typedef enum {
    MS_IDLE = 0,
    MS_RAMPING = 1,
    MS_RUNNING = 2,
    MS_LOCKED = 3
} MotorState_t;

typedef enum {
    CMD_TYPE_NONE_USE = 255,
    CMD_TYPE_STOP = 1,
    CMD_TYPE_RUN_FWD = 2,
    CMD_TYPE_RUN_REV = 3,
    CMD_TYPE_RAMP_FWD = 4,
    CMD_TYPE_RAMP_REV = 5,
    CMD_TYPE_BLOCK_FWD = 6,
    CMD_TYPE_BLOCK_REV = 7,
    CMD_TYPE_BLOCK_BOTH = 8,
} CmdType_t;

// ========== ï¿½è±¸IDÃ¶ï¿½ï¿½ ==========
typedef enum {
    DEV_ID_NONE             = 255,
    DEV_ID_POWER_POS        = 1,
    DEV_ID_POWER_NEG        = 2,
    DEV_ID_LIMIT_FWD        = 3,
    DEV_ID_LIMIT_REV        = 4,
    DEV_ID_CAN              = 5,
    DEV_ID_IO_FWD           = 6,    // ï¿½ï¿½×ªIOï¿½è±¸
    DEV_ID_IO_REV           = 7,    // ï¿½ï¿½×ªIOï¿½è±¸
    DEV_ID_EMERGENCY        = 8,
    DEV_ID_RTURN_FWD        = 9,    // ï¿½ï¿½×ªï¿½ï¿½Î»-ï¿½ï¿½×ªï¿½ï¿½Î»
    DEV_ID_RTURN_REV        = 10,   // ï¿½ï¿½×ªï¿½ï¿½Î»-ï¿½ï¿½×ªï¿½ï¿½Î»
    DEV_ID_OVERVOLTAGE_FWD  = 11,   // ï¿½ï¿½Ñ¹-ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×ª
    DEV_ID_OVERVOLTAGE_REV  = 12,   // ï¿½ï¿½Ñ¹-ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×ª
    DEV_ID_UNDERVOLTAGE_FWD = 13,   // Ç·Ñ¹-ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×ª
    DEV_ID_UNDERVOLTAGE_REV = 14,   // Ç·Ñ¹-ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×ª
    DEV_ID_OVERCUR_FWD      = 15,   // ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿½(ï¿½ï¿½ï¿½ï¿½) - block_fwd
    DEV_ID_OVERCUR_REV      = 16,   // ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿½(ï¿½Ø´ï¿½) - block_rev
    DEV_ID_MAX
} MotorDeviceId_t;

// ï¿½ï¿½ï¿½È¼ï¿½Ã¶ï¿½ï¿½
typedef enum {
    PRIO_NONE = 255,
    PRIO_EMERGENCY = 0,
    PRIO_LIMIT = 2,
    PRIO_MANUAL = 3,
    PRIO_CAN = 4,
    PRIO_POWER = 5
} MotorPriority_t;

// ========== ï¿½ï¿½ï¿½ï¿½á¹¹ï¿½ï¿? ==========
typedef struct {
    MotorDeviceId_t device_id;
    MotorPriority_t priority;
    CmdType_t type;
    float param;
    uint32_t timestamp;
} MotorControlCommand_t;

typedef struct {
    MotorDir_t desired_dir;     // ï¿½ï¿½Ç°ï¿½Ù²ï¿½ï¿½ï¿½ï¿½Ã³ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    MotorState_t state;         // ï¿½ï¿½ï¿½×´Ì¬ï¿½ï¿½IDLE/RAMPING/RUNNINGï¿½ï¿½
    MotorDir_t active_dir;      // ï¿½ï¿½Ç°ï¿½î¶¯ï¿½ï¿½ï¿½ï¿½
    float current_duty;         // ï¿½ï¿½Ç°Õ¼ï¿½Õ±ï¿½
    uint8_t enable;             // ï¿½ï¿½ï¿½Ê¹ï¿½ï¿½×´Ì?
} Motor_StateInfo_t;

#define MAX_COMMANDS_PER_DIRECTION 20

typedef struct {
    MotorControlCommand_t commands[MAX_COMMANDS_PER_DIRECTION];
    uint8_t count;
} MotorCommandList_t;

// ========== ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ï¢ï¿½á¹¹ï¿½ï¿½ ==========
typedef struct {
    struct {
        MotorDeviceId_t device_ids[MAX_COMMANDS_PER_DIRECTION];
        uint8_t count;
    } block_fwd;

    struct {
        MotorDeviceId_t device_ids[MAX_COMMANDS_PER_DIRECTION];
        uint8_t count;
    } block_rev;

    struct {
        MotorDeviceId_t device_ids[MAX_COMMANDS_PER_DIRECTION];
        uint8_t priorities[MAX_COMMANDS_PER_DIRECTION];
        uint8_t count;
    } allow_fwd;

    struct {
        MotorDeviceId_t device_ids[MAX_COMMANDS_PER_DIRECTION];
        uint8_t priorities[MAX_COMMANDS_PER_DIRECTION];
        uint8_t count;
    } allow_rev;

    MotorDeviceId_t active_device_id;
    MotorState_t state;
    MotorDir_t active_dir;
    float current_duty;
    bool conflict_fault;
} MotorDebugInfo_t;

// ========== ï¿½ï¿½ï¿½ï¿½è±¸ï¿½á¹¹ï¿½ï¿? ==========
typedef struct {
    // ï¿½Ù²Ã¶ï¿½ï¿½ï¿½
    MotorCommandList_t block_fwd;
    MotorCommandList_t block_rev;
    MotorCommandList_t allow_fwd;
    MotorCommandList_t allow_rev;
    MotorState_t state;
    MotorDir_t active_dir;
    MotorDeviceId_t active_device_id;
    float current_duty;

    // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ï¢
    MotorDebugInfo_t debug_info;

    // ï¿½Ú²ï¿½×´Ì¬
    float last_sent_duty;
    uint32_t last_arbitration_time;

    // ï¿½è±¸ï¿½ï¿½ï¿½ï¿½
    uint8_t motor_id;           // ï¿½ï¿½ï¿½IDï¿½ï¿½ï¿½ï¿½ï¿½Ú¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    uint8_t enable;             // ï¿½ï¿½ï¿½Ê¹ï¿½ï¿?
} MotorDevice_t;

// ========== ï¿½Â¼ï¿½ï¿½ï¿½ï¿½Ý½á¹¹ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½EventBusï¿½ï¿½ ==========
typedef struct {
    MotorDir_t dir;
    bool is_active;
} MotorLimitEvent_t;

typedef struct {
    MotorDeviceId_t power_id;
    bool is_on;
} MotorPowerEvent_t;

typedef struct {
    MotorDir_t dir;
    CmdType_t type;
    float speed;
} MotorManualIOEvent_t;

typedef struct {
    MotorDir_t dir;
    CmdType_t type;
    float speed;
} MotorCANEvent_t;

typedef struct {
    uint8_t adc_id;             // ADCï¿½è±¸IDï¿½ï¿½ï¿½Ä¸ï¿½ADCï¿½ï¿½âµ½ï¿½Ä£ï¿?
    uint16_t current_ma;        // ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½(mA)
    uint16_t threshold_ma;      // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµ(mA)
    uint32_t duration_ms;       // ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½(ms)
} MotorOvercurrentEvent_t;

// ========== ï¿½Ù²Ã»Øµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½å£¬ï¿½Ã»ï¿½ï¿½ï¿½ï¿½ï¿½Ð´ï¿½ï¿½ ==========
/**
 * @brief ï¿½Ù²Ã¾ï¿½ï¿½ï¿½ÎªÍ£Ö¹Ê±ï¿½ï¿½ï¿½ÃµÄ»Øµï¿½
 * @param motor ï¿½ï¿½ï¿½ï¿½è±¸Ö¸ï¿½ï¿?
 */
void Motor_OnArbitrationStop(MotorDevice_t* motor);

/**
 * @brief ï¿½Ù²Ã¾ï¿½ï¿½ï¿½Îªï¿½ï¿½×ªÊ±ï¿½ï¿½ï¿½ÃµÄ»Øµï¿½
 * @param motor ï¿½ï¿½ï¿½ï¿½è±¸Ö¸ï¿½ï¿?
 * @param duty ï¿½ï¿½Ç°Õ¼ï¿½Õ±ï¿½
 */
void Motor_OnArbitrationFwd(MotorDevice_t* motor, float duty);

/**
 * @brief ï¿½Ù²Ã¾ï¿½ï¿½ï¿½Îªï¿½ï¿½×ªÊ±ï¿½ï¿½ï¿½ÃµÄ»Øµï¿½
 * @param motor ï¿½ï¿½ï¿½ï¿½è±¸Ö¸ï¿½ï¿?
 * @param duty ï¿½ï¿½Ç°Õ¼ï¿½Õ±ï¿½
 */
void Motor_OnArbitrationRev(MotorDevice_t* motor, float duty);

// ========== ï¿½ï¿½ï¿½ï¿½è±¸ï¿½Ó¿Ú£ï¿½ï¿½ï¿½ï¿½ï¿½DeviceManagerï¿½æ·¶ï¿½ï¿½ ==========
// ï¿½ï¿½×¼ï¿½è±¸ï¿½ï¿½ï¿½ï¿½
DeviceResult_t Motor_Init(void* handle);
DeviceResult_t Motor_Deinit(void* handle);
DeviceResult_t Motor_Read(void* handle, void* data, uint32_t size);
DeviceResult_t Motor_Write(void* handle, const void* data, uint32_t size);
DeviceResult_t Motor_Control(void* handle, DeviceCommandData_t* cmd);
DeviceResult_t Motor_Update(void* handle);  // ï¿½ï¿½Ê±ï¿½ï¿½Ñ¯

// ï¿½Ø¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¿ï¿½
void Motor_SetSpeed(MotorDevice_t* motor, float duty);
void Motor_Start(MotorDevice_t* motor, MotorDir_t dir);
void Motor_Stop(MotorDevice_t* motor);
void Motor_EmergencyStop(MotorDevice_t* motor);

// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ð½Ó¿Ú£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ allow Ö¸ï¿½î£¬ï¿½ï¿½Ó°ï¿½ï¿½ blockï¿½ï¿½
void Motor_ClearAllowFwd(MotorDevice_t* motor);
void Motor_ClearAllowRev(MotorDevice_t* motor);

// ï¿½ï¿½ï¿½Ô½Ó¿ï¿½
const MotorDebugInfo_t* Motor_GetDebugInfo(MotorDevice_t* motor);

// ========== EventBusï¿½Øµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ==========
void Motor_OnPowerEvent(void* payload);
void Motor_OnHardLimit(void* payload);
void Motor_OnManualIO(void* payload);
void Motor_OnCANEvent(void* payload);
void Motor_OnSpeedFeedback(void* payload);  // ï¿½Ù¶È·ï¿½ï¿½ï¿½ï¿½ï¿½Ô¤ï¿½ï¿½ï¿½ï¿½
void Motor_OnOvercurrent(void* payload);
void Motor_OnVoltageAlarm(void* payload);
void Motor_OnCurrentAlarm(void* payload);  // ï¿½Ñ½ï¿½ï¿½ï¿½
void Motor_OnOvercurrentFwd(void* payload);  // ï¿½ï¿½×ª(ï¿½ï¿½ï¿½ï¿½)ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ block_fwd
void Motor_OnOvercurrentRev(void* payload);  // ï¿½ï¿½×ª(ï¿½Ø´ï¿½)ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ block_rev
void Motor_OnRTurnLimit(void* payload);  // ï¿½ï¿½×ªï¿½ï¿½Î»
// ï¿½ï¿½È¡ï¿½ï¿½Ç°ï¿½Ù²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ò£¨¼ï¿½ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½Ö´ï¿½ÐµÄ·ï¿½ï¿½ï¿½
MotorDir_t Motor_GetDesiredDirection(MotorDevice_t* motor);

// ========== ï¿½ï¿½Ñ¹ï¿½ï¿½ï¿½ï¿½ï¿½Ö¶ï¿½ï¿½ï¿½ï¿½ï¿½Ó¿ï¿? ==========
/**
 * @brief ï¿½ï¿½ï¿½ï¿½ï¿½Ñ¹ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ãµï¿? block Ö¸ï¿½ï¿½
 * @param motor ï¿½ï¿½ï¿½ï¿½è±¸Ö¸ï¿½ï¿?
 * @param u8AlarmType ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Í£ï¿½VOLTAGE_ALARM_OVERVOLTAGE ï¿½ï¿½ VOLTAGE_ALARM_UNDERVOLTAGE
 * @note ï¿½ï¿½ï¿½ï¿½ VOLTAGE_CLEAR_MODE == VOLTAGE_CLEAR_MANUAL Ê±Ê¹ï¿½ï¿½
 *       ï¿½ï¿½ App_FaultHandler ï¿½Õµï¿½ TOPIC_FAULT_CLEAR ï¿½Â¼ï¿½Ê±ï¿½ï¿½ï¿½ï¿½
 */
void Motor_ClearVoltageBlock(MotorDevice_t* motor, uint8_t u8AlarmType);

// ========== ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¶ï¿½ï¿½ï¿½ï¿½ï¿½Ó¿ï¿? ==========
/**
 * @brief ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ãµï¿? block Ö¸ï¿½ï¿½
 * @param motor ï¿½ï¿½ï¿½ï¿½è±¸Ö¸ï¿½ï¿?
 * @note ï¿½ï¿½ App_FaultHandler ï¿½Õµï¿½ TOPIC_FAULT_CLEAR ï¿½Â¼ï¿½Ê±ï¿½ï¿½ï¿½ï¿½
 *       ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ïºï¿½ï¿½ï¿½Òªï¿½ï¿½ï¿½ï¿½Ù²ï¿½ï¿½ï¿½ï¿½Ðµï¿½Ë«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
 */
void Motor_ClearOvercurrentBlock(MotorDevice_t* motor);

// ========== ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ==========
#define MOTOR_FORWARD_BLANK_MS     (50U)     // æ­£è½¬æ¶ˆéš�æ—¶é•¿ï¼ˆmsï¼?
#define MOTOR_REVERSE_BLANK_MS     (50U)   // ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿½Ê±ï¿½ä£¨msï¿½ï¿½

// ========== ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½â²¿ï¿½ï¿½ï¿½ï¿½ ==========
// dev_sensor Ê¹ï¿½Ã´Ë±ï¿½Ö¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ú¹ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
extern volatile uint8_t g_u8MotorForwardBlankActive;
extern volatile uint8_t g_u8MotorReverseBlankActive;

// ========== Keil Watch ï¿½ï¿½ï¿½ï¿½È«ï¿½Ö±ï¿½ï¿½ï¿½ ==========
// ï¿½ï¿½ Watch ï¿½ï¿½ï¿½ï¿½ï¿½Ð²é¿´ g_pMotorDevWatch ï¿½É¹Û²ï¿½ï¿½Ù²ï¿½ï¿½ï¿½ï¿½Ú²ï¿½×´Ì¬
extern MotorDevice_t* volatile g_pMotorDevWatch;

#endif /* DEV_MOTOR_H_ */
