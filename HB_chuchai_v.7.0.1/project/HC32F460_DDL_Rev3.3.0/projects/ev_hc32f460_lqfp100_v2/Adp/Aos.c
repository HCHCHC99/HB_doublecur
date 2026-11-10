/**
 *******************************************************************************
 * @file  Aos.c
 * @brief AOS (阿波罗事件路由系统) 适配层
 *******************************************************************************
 */

#include "Aos.h"
#include "rtt_log.h"
/*==============================================================================
 * 静态函数
 *============================================================================*/

/*==============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief  AOS 初始化
 */
void AOS_Init(void)
{
    /* 使能 AOS 时钟 */
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    /* 第一级触发：TMR0的CMP_B事件触发ADC1序列A */
    AOS_Connect(AOS_ADC1_0, EVT_SRC_TMR0_1_CMP_B);

    /* 第二级触发：ADC1转换完成事件触发DMA */
    /* 注意：使用正确的DMA目标宏 */
    AOS_Connect(AOS_DMA1_1, EVT_SRC_ADC1_EOCA);  // 对应DMA通道1 -> ADC CH4
    AOS_Connect(AOS_DMA1_2, EVT_SRC_ADC1_EOCA);  // 对应DMA通道2 -> ADC CH5

    /* 打印配置信息 */
    // MAIN_D("AOS configured: TMR0->ADC1, ADC1->DMA1_CH1/CH2\r\n");
}

/**
 * @brief  AOS 反初始化
 */
void AOS_DeInit(void)
{
    /* 禁用 AOS 时钟 */
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, DISABLE);
}

/**
 * @brief  AOS 使能（AOS 模块本身没有全局使能位，此函数保留为接口）
 */
void AOS_Enable(void)
{
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
}

/**
 * @brief  AOS 禁用
 */
void AOS_Disable(void)
{
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, DISABLE);
}

/**
 * @brief  连接事件源到目标
 * @param  u32Target   目标 @ref AOS_Target_Select
 * @param  enSource    事件源 @ref en_event_src_t
 */
void AOS_Connect(uint32_t u32Target, en_event_src_t enSource)
{
    /* 先确保 AOS 时钟已使能 */
    AOS_Enable();

    /* 设置触发事件源 */
    AOS_SetTriggerEventSrc(u32Target, enSource);
}

/**
 * @brief  断开事件源与目标的连接
 * @param  u32Target   目标 @ref AOS_Target_Select
 */
void AOS_Disconnect(uint32_t u32Target)
{
    /* 将事件源设置为 0（无触发） */
    AOS_SetTriggerEventSrc(u32Target, (en_event_src_t)0);
}

/**
 * @brief  使能通用触发
 * @param  u32Target          目标 @ref AOS_Target_Select
 * @param  u32CommonTrigger   通用触发ID @ref AOS_COMM_TRIG1/2
 */
void AOS_EnableCommonTrigger(uint32_t u32Target, uint32_t u32CommonTrigger)
{
    AOS_CommonTriggerCmd(u32Target, u32CommonTrigger, ENABLE);
}

/**
 * @brief  禁用通用触发
 * @param  u32Target          目标 @ref AOS_Target_Select
 * @param  u32CommonTrigger   通用触发ID @ref AOS_COMM_TRIG1/2
 */
void AOS_DisableCommonTrigger(uint32_t u32Target, uint32_t u32CommonTrigger)
{
    AOS_CommonTriggerCmd(u32Target, u32CommonTrigger, DISABLE);
}

/**
 * @brief  软件触发
 */
void AOS_SoftwareTrigger(void)
{
    AOS_SW_Trigger();
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
