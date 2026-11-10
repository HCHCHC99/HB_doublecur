#ifndef __APP_MODBUS_H__
#define __APP_MODBUS_H__

#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"
#include "App_Params.h"

/*=============================================================================
 * 自动发送配置（从机模式，默认关闭）
 * 主机主动轮询，从机不应主动发数据
 * 如需调试，取消注释 #define MODBUS_AUTO_SEND_ENABLE
 *============================================================================*/
// #define MODBUS_AUTO_SEND_ENABLE

/*=============================================================================
 * 调试宏定义（旧名称兼容）
 * 开关在 rtt_manager.h 中统一管理：APP_MODBUS_xxx
 *============================================================================*/
#ifdef APP_MODBUS_AUTO_DBG
    #define MODBUS_AUTO_DBG(fmt, ...)   MAIN_D("[MODBUS_AUTO] " fmt, ##__VA_ARGS__)
#else
    #define MODBUS_AUTO_DBG(fmt, ...)   ((void)0)
#endif

#ifdef APP_MODBUS_CRC_DBG
    #define MODBUS_CRC_DBG(fmt, ...)    MAIN_D("[MODBUS_CRC] " fmt, ##__VA_ARGS__)
#else
    #define MODBUS_CRC_DBG(fmt, ...)    ((void)0)
#endif

#ifdef APP_MODBUS_INIT_DBG
    #define MODBUS_INIT_DBG(fmt, ...)   MAIN_D("[MODBUS_INIT] " fmt, ##__VA_ARGS__)
#else
    #define MODBUS_INIT_DBG(fmt, ...)   ((void)0)
#endif

#ifdef APP_MODBUS_PARSE_DBG
    #define MODBUS_PARSE_DBG(fmt, ...)  MAIN_D("[MODBUS_PARSE] " fmt, ##__VA_ARGS__)
#else
    #define MODBUS_PARSE_DBG(fmt, ...)  ((void)0)
#endif

#ifdef APP_MODBUS_POLL_DBG
    #define MODBUS_POLL_DBG(fmt, ...)   MAIN_D("[MODBUS_POLL] " fmt, ##__VA_ARGS__)
#else
    #define MODBUS_POLL_DBG(fmt, ...)   ((void)0)
#endif

#ifdef APP_MODBUS_RX_DBG
    #define MODBUS_RX_DBG(fmt, ...)     MAIN_D("[MODBUS_RX] " fmt, ##__VA_ARGS__)
#else
    #define MODBUS_RX_DBG(fmt, ...)     ((void)0)
#endif

/*=============================================================================
 * 参数打印调试宏（每5秒打印配置参数）
 * 开关：定义 APP_MODBUS_PARAM_PRINT 启用
 *============================================================================*/
#ifdef APP_MODBUS_PARAM_PRINT
    #define MODBUS_PARAM_PRINT(fmt, ...)   MAIN_D("[PARAM] " fmt, ##__VA_ARGS__)
#else
    #define MODBUS_PARAM_PRINT(fmt, ...)   ((void)0)
#endif

/*=============================================================================
 * Modbus 配置
 *============================================================================*/
#define MODBUS_FRAME_FIX_LEN        (8U)    /* 最小帧长度: addr(1) + func(1) + data(4) + crc(2) */
#define MODBUS_RESP_BUF_SIZE        (256U)  /* 响应缓冲区大小 */

/*=============================================================================
 * Modbus 功能码
 *============================================================================*/
#define MODBUS_FUNC_READ_HOLDING    (0x03U) /* 读保持寄存器 */
#define MODBUS_FUNC_WRITE_SINGLE    (0x06U) /* 写单个寄存器 */

#define MODBUS_ENABLE_WRITE_MULTI

#ifdef MODBUS_ENABLE_WRITE_MULTI
#define MODBUS_FUNC_WRITE_MULTI     (0x10U) /* 写多个寄存器 */
#endif
/*=============================================================================
 * Modbus 异常码
 *============================================================================*/
#define MODBUS_EXCEPTION_ILLEGAL_FUNC       (0x01U) /* 非法功能码 */
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR  (0x02U) /* 非法数据地址 */
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VAL   (0x03U) /* 非法数据值 */
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAIL  (0x04U) /* 从设备故障 */
#define MODBUS_EXCEPTION_ACKNOWLEDGE        (0x05U) /* 确认 */
#define MODBUS_EXCEPTION_SLAVE_BUSY         (0x06U) /* 从设备忙 */

/*=============================================================================
 * 函数声明
 *============================================================================*/

/**
 * @brief  Modbus 初始化
 */
void Modbus_Init(void);

/**
 * @brief  Modbus 主循环轮询
 */
void Modbus_Poll(void);

/**
 * @brief  设置参数默认值（供 param_manager 回调使用）
 */
void Modbus_SetParamDefaults(void);

#endif /* __APP_MODBUS_H__ */
