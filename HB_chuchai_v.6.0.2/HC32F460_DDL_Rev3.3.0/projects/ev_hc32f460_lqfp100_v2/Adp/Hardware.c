#include "Hardware.h"
#include "Adc.h"
#include "Aos.h"
#include "Dma.h"
#include "rtt_log.h"

/**
 * ============================================================================
 * @brief  演示如何使用 PA6 (ADC1_CH6) 进行 ADC 采样
 *
 *   本演示展示了如何使用 Adp 层的三个模块配合进行 ADC 采样：
 *   1. Adc.c/.h   - ADC 管理层：管理 ADC 实例，实现采样/停止
 *   2. Aos.c/.h   - AOS 事件路由层：配置 TMR0 触发 ADC1 和 DMA 传输事件
 *   3. Dma.c/.h   - DMA 管理层：配置 DMA 通道并处理传输完成中断
 *
 *   数据流路径：
 *   TMR0 (定时器周期) -> AOS 触发 ADC1 (开始转换) -> AOS 触发 DMA1 (传输数据到内存)
 *
 *   采样模式可通过 ADC_USE_DMA_MODE 宏选择：
 *   - 中断模式：ADC 转换完成后触发中断，在中断中读取数据
 *   - DMA模式：ADC 转换完成后由 DMA 自动将数据搬运到内存缓冲区
 * ============================================================================
 */

/* 选择 PA6 的采样模式：1=DMA模式, 0=中断模式 */
#define ADC_USE_DMA_MODE   1

/* PA6 对应的 ADC 通道：ADC1_CH6 */
#define PA6_ADC_CHANNEL    ADC_CH6
#define PA6_ADC_PORT       GPIO_PORT_A
#define PA6_ADC_PIN        GPIO_PIN_06

/* PA6 的 ADC 实例 ID（由 Adc_Create 返回） */
static uint8_t s_u8Pa6AdcId = 0xFF;

/* PA6 的 DMA 通道号（DMA模式使用） */
#define PA6_DMA_CHANNEL    1   /* 使用 DMA1 通道 1 */
#define PA6_DMA_BUFFER_SIZE 8  /* DMA 缓冲区大小 */

/* ============================================================================
 *      1. PA6 中断模式回调函数
 * ============================================================================
 *   中断模式使用定时器，每次 ADC 转换完成时调用此回调函数。
 * 注意：回调函数在中断上下文中执行，应尽量减少耗时操作。
 *    在 Keil Debug 时可在本函数设置断点，Watch 窗口中可观察 u16Voltage 的值
 * ============================================================================ */
static void Pa6_AdcCallback_Interrupt(uint16_t u16AdcValue)
{
    /* 将 ADC 原始值 (12bit, 0~4095) 转换为电压值 (mV) */
    uint16_t u16Voltage = (uint16_t)(((uint32_t)u16AdcValue * 3300UL) / 4095UL);

    /* 可在 Keil Watch 窗口中观察：u16AdcValue, u16Voltage */
    __nop(); // Debug: 在此处断点查看 PA6 中断模式 ADC 值

    /* 注意：MAIN_D 在中断中频繁调用会影响实时性，调试时请取消注释 */
    // MAIN_D("PA6[INT] Raw=%u, %umV\r\n", u16AdcValue, u16Voltage);
}

/* ============================================================================
 *      2. DMA 传输完成回调函数
 * ============================================================================
 *   DMA 完成一轮数据搬运后调用。
 * 注意：回调函数在 DMA 中断上下文中执行，应尽量减少耗时操作。
 *    在 Keil Debug 时可在本函数设置断点，观察 DMA 缓冲区的数据。
 * ============================================================================ */
static void Pa6_DmaCallback(void)
{
    /* 此回调由 Dma 层在 DMA 传输完成中断中调用 */
    /* 实际数据读取在 main loop 中通过 Adc_GetLatestValue() 进行 */
    __nop(); // Debug: 在此处断点，确认 DMA 已触发
}

/* ============================================================================
 *      3. 初始化 PA6 ADC 采样（在 Hardware_Init 中调用）
 * ============================================================================
 * 本函数演示了完整的初始化流程：
 *   1. 创建 ADC 实例 (Adc_Create)
 *   2. 初始化 ADC 和 DMA (Adc_Init 内部自动完成)
 *   3. 启动 ADC 转换 (Adc_Start)
 *
 * 注意：AOS 初始化 (AOS_Init) 需要在本函数调用之前完成。
 *   因为 AOS_Init 配置了 TMR0、ADC1、DMA 的事件路由。
 * ============================================================================ */
static void Pa6_Adc_Init(void)
{
    stc_adc_config_t stcAdcConfig;

    /* ---- 配置 ADC 参数 ---- */
    stcAdcConfig.u8Channel = PA6_ADC_CHANNEL;          /* ADC1_CH6 */
    stcAdcConfig.stcPin.u8Port = PA6_ADC_PORT;         /* GPIO_PORT_A */
    stcAdcConfig.stcPin.u8Pin = PA6_ADC_PIN;           /* GPIO_PIN_06 */

#if (ADC_USE_DMA_MODE == 1)
    /* ===== DMA 模式 ===== */
    stcAdcConfig.enMode = ADC_MODE_DMA;
    stcAdcConfig.pfnCallback = NULL;                    /* DMA 模式不需要 ADC 回调 */
    stcAdcConfig.stcDmaConfig.u16BufferSize = PA6_DMA_BUFFER_SIZE;
    stcAdcConfig.stcDmaConfig.u8DmaChannel = PA6_DMA_CHANNEL;

    MAIN_D("[PA6_ADC] Initializing PA6 (CH6) in DMA mode, DMA1 CH%d, buffer=%d\r\n",
           PA6_DMA_CHANNEL, PA6_DMA_BUFFER_SIZE);
#else
    /* ===== 中断模式 ===== */
    stcAdcConfig.enMode = ADC_MODE_INTERRUPT;
    stcAdcConfig.pfnCallback = Pa6_AdcCallback_Interrupt;

    MAIN_D("[PA6_ADC] Initializing PA6 (CH6) in Interrupt mode\r\n");
#endif

    /* ---- 创建 ADC 实例 ---- */
    s_u8Pa6AdcId = Adc_Create(&stcAdcConfig);
    if (s_u8Pa6AdcId == 0xFF) {
        MAIN_D("[PA6_ADC] ERROR: Failed to create ADC instance!\r\n");
        return;
    }

    /* ---- 初始化 ADC（内部会同时初始化 DMA） ---- */
    Adc_Init();

    /* ---- 启动 ADC 转换（由 TMR0 定时器触发） ---- */
    Adc_Start();

    MAIN_D("[PA6_ADC] PA6 ADC initialized successfully! ADC_ID=%d\r\n", s_u8Pa6AdcId);
}

/* ============================================================================
 *      4. main loop 轮询获取 PA6 ADC 数据
 * ============================================================================
 * 本函数应在 main loop 中周期性调用，用于获取最新的 ADC 值。
 *    在 Keil Debug 时可在本函数设置断点，Watch 窗口中可观察各变量值
 * ============================================================================ */
void Pa6_Adc_Process(void)
{
    static uint32_t s_u32LastPrintMs = 0;
    uint32_t u32Now = tickTimer_GetCount();

    /* 每 500ms 打印一次 PA6 ADC 值 */
    if ((u32Now - s_u32LastPrintMs) >= 500) {
        s_u32LastPrintMs = u32Now;

        if (s_u8Pa6AdcId != 0xFF) {
            /* 获取最新的 ADC 原始值 */
            uint16_t u16Raw = Adc_GetLatestValue(s_u8Pa6AdcId);

            /* 转换为电压 (mV) */
            uint16_t u16Voltage = (uint16_t)(((uint32_t)u16Raw * 3300UL) / 4095UL);

#if (ADC_USE_DMA_MODE == 1)
            /* DMA 模式还可以获取平均值和采样计数 */
            uint16_t u16Avg = Adc_GetAverageValue(s_u8Pa6AdcId);
            uint16_t u16AvgMv = (uint16_t)(((uint32_t)u16Avg * 3300UL) / 4095UL);
            uint32_t u32Samples = Adc_GetSampleCount(s_u8Pa6AdcId);

            MAIN_D("PA6 DMA: Raw=%4u, %3umV, Avg=%4u, %3umV, Cnt=%lu\r\n",
                   u16Raw, u16Voltage, u16Avg, u16AvgMv, u32Samples);
#else
            /* 中断模式 */
            MAIN_D("PA6 INT: Raw=%4u, %3umV\r\n", u16Raw, u16Voltage);
#endif

            /* 可在 Keil Watch 窗口中观察：u16Raw, u16Voltage */
            __nop(); // Debug: 在此处断点查看 PA6 ADC 值
        }
    }
}

/* ============================================================================
 *      5. AOS 事件路由说明
 * ============================================================================
 * AOS_Init() 在 Hardware_Init() 中被调用，配置了以下事件路由：
 *
 *     第一个：TMR0_1 CMP_B 触发 ADC1 开始转换
 *     - AOS_Connect(AOS_ADC1_0, EVT_SRC_TMR0_1_CMP_B);
 *     - 配置后，TMR0 定时器每隔一个周期触发 ADC1 开始一次 A 组转换
 *     - 定时频率由 Adc.c 中的 Timer0_Config(1000) 设置，默认 1ms
 *
 *     第二个：ADC1 EOCA 触发 DMA1 通道 1/2
 *     - AOS_Connect(AOS_DMA1_1, EVT_SRC_ADC1_EOCA);  // 对应 DMA1 CH1
 *     - AOS_Connect(AOS_DMA1_2, EVT_SRC_ADC1_EOCA);  // 对应 DMA1 CH2
 *     - 配置后，ADC1 转换完成后自动触发 DMA 将数据搬运到内存缓冲区
 *
 * 注意：如果使用中断模式 (ADC_USE_DMA_MODE=0)，第二个 AOS 路由可以不需要配置，
 *   因为 ADC 中断直接读取数据，而不是通过 AOS 触发 DMA 路由。
 *   但 AOS_Init() 仍然会配置 DMA 路由，不影响中断模式的使用。
 * ============================================================================ */

/* ============================================================================
 *      6. DMA 配置使用说明
 * ============================================================================
 * 使用 DMA 模式时，Adc_Init() 内部会自动完成以下配置：
 *
 *   1. 为每个 DMA 模式的 ADC 实例调用 Dma_Create() 创建 DMA 实例
 *      - 源地址为 ADC 数据寄存器 (ADC1->DR6)
 *      - 目标地址为 DMA 内部自动分配的环形缓冲区
 *      - 传输方向为外设到内存
 *      - 传输模式为循环传输（环形缓冲模式）
 *      - 数据宽度为 16bit（ADC 为 12bit）
 *      - 每轮传输大小为 ADC_DMA_BUFFER_SIZE（默认 8）
 *
 *   2. 调用 Dma_Init() 初始化所有 DMA 通道
 *      - 分配 DMA 传输描述符（malloc）
 *      - 配置 DMA 硬件寄存器
 *      - 注册 DMA 传输完成中断
 *
 *   3. 调用 Dma_StartAll() 启动所有 DMA 通道
 *      - 使能 DMA 通道，等待 ADC 转换完成后触发
 *
 *    完整数据流：
 *   TMR0 (1ms 定时)
 *        -> AOS 路由触发 ADC1 开始 A 组转换
 *        -> ADC1 采样 PA6 (CH6)
 *        -> ADC1 转换完成 (EOCA)
 *        -> AOS 路由触发 DMA1 CH1 开始传输
 *        -> DMA1 将 ADC1->DR6 数据搬运到内存环形缓冲区
 *        -> DMA 传输完成中断通知数据就绪
 *        -> main loop 中 Pa6_Adc_Process() 获取最新值并打印
 * ============================================================================ */

/* ============================================================================
 * 原有的回调函数示例（保留参考）
 * ============================================================================ */
void MyAdcCallback_PA1(uint16_t u16AdcValue)
{
    uint16_t u16Voltage = (u16AdcValue * 3300) / 4096;
    __nop(); // Debug: PA1 ADC 回调
}

void MyAdcCallback_PA2(uint16_t u16AdcValue)
{
    uint16_t u16Voltage = (u16AdcValue * 3300) / 4096;
    __nop(); // Debug: PA2 ADC 回调
}

void MyAdcCallback_PA6(uint16_t u16AdcValue)
{
    uint16_t u16Voltage = (u16AdcValue * 3300) / 4096;
    __nop(); // Debug: PA6 ADC 回调（已被 Pa6_AdcCallback_Interrupt 替代）
}

/* ============================================================================
 *  硬件初始化函数
 * ============================================================================ */
void Hardware_Init(void)
{
    /* 1. 系统定时器初始化（在硬件抽象层执行） */
    if (Systick_config() != 0) {
        /* 初始化失败时，停止执行（常见原因：PLL/时钟源初始化失败） */
        while (1);
    }
    TMR0_Unit2_Init(TMR0_CHANNEL_A_2, 1000UL, TICK_RESET_2);
    TMR0_Unit2_Init(TMR0_CHANNEL_B_2, 5000UL, TICK_RESET_2);
    // TMR0_Unit1 CH_B 已被 Adc.c 占用为 ADC AOS 硬件触发 (200us)
    // TMR0_Unit1_Init(TMR0_CHANNEL_B_1, 500UL, TICK_RESET_1);
    TMR0_Unit1_Init(TMR0_CHANNEL_A_1, 1000UL, TICK_RESET_1);

    tickTimer_DelayMs(150);
    // Output_GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_INIT_HIGH);

    /* 2. AOS 事件路由层初始化
     *     配置 TMR0、ADC1、DMA 的事件路由
     *     必须在 ADC 初始化之前完成 */
    AOS_Init();

    /* 3. GPIO 初始化 */
    Output_GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_INIT_LOW);

    Output_GPIO_Init(PHU_PORT, PHU_PIN, GPIO_INIT_LOW);
    Output_GPIO_Init(PHV_PORT, PHV_PIN, GPIO_INIT_LOW);

    // TMR0_Unit2_Reconfig(TMR0_CHANNEL_A_2);
    // TMR0_Unit2_Reconfig(TMR0_CHANNEL_A_1);
    // TMR0_Unit2_Reconfig(TMR0_CHANNEL_B_2);
    // TMR0_Unit2_Reconfig(TMR0_CHANNEL_B_1);
    // TMR0_Unit2_Reconfig(TMR0_CHANNEL_B_2);
    // /* 5. PA6 ADC 采样初始化
    //  *    演示 Adc + Aos + Dma 三个模块配合使用的方法
    //  *
    //  *    数据流路径：
    //  *      TMR0 (1ms 定时)
    //  *           -> AOS 路由触发 ADC1 开始 A 组转换
    //  *           -> ADC1 采样 PA6 (CH6)
    //  *           -> ADC1 转换完成 (EOCA)
    //  *           -> AOS 路由触发 DMA1 CH1 开始传输
    //  *           -> DMA1 将数据搬运到内存环形缓冲区
    //  *           -> DMA 传输完成中断通知数据就绪
    //  *           -> main loop 中 Pa6_Adc_Process() 获取数据并打印
    //  *
    //  *    切换模式只需修改 ADC_USE_DMA_MODE 宏
    //  *      = 1: DMA 模式，自动搬运，CPU 无需中断等待
    //  *      = 0: 中断模式，每次转换触发中断 */
    // Pa6_Adc_Init();

    // MAIN_D("[Hardware] Hardware init complete! PA6 ADC running in %s mode\r\n",
    //        (ADC_USE_DMA_MODE ? "DMA" : "Interrupt"));

}
