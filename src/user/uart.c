#include <stdio.h>
#include "headfile.h"
#include "uart.h"


/* =================================================================================================
 *  UART 发送完成标志（用于 printf 等阻塞发送）
 * ================================================================================================= */
static volatile bool uart_send_complete_flag = false;

/* =================================================================================================
 *  IMU 接收 (SCI5/UART5) — 中断模式
 *  - 每收到一个字节触发 imu_callback → RX_CHAR
 *  - 收满 IMU_RX_BUF_SIZE 字节后置 imu_rx_complete = true
 * ================================================================================================= */
volatile uint8_t imu_rx_buf[IMU_RX_BUF_SIZE];
volatile bool imu_rx_complete = false;

/** IMU 回调 - UART5（SCI5），DMAC 模式下 RX_CHAR 由硬件处理 */
void imu_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
            uart_send_complete_flag = true;
            break;
        default:
            break;
    }
}

void UART5_IMU_Init(void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart5_ctrl, &g_uart5_cfg);
    assert(FSP_SUCCESS == err);
}

/* =================================================================================================
 *  LoRa 接收 (SCI2/UART2) — 中断模式
 * ================================================================================================= */
volatile uint8_t lora_rx_buf[LORA_RX_BUF_SIZE];
volatile bool lora_rx_complete = false;

/** LoRa 回调 - UART2（SCI2），DMAC 模式下 RX_CHAR 由硬件处理 */
void lora_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
            uart_send_complete_flag = true;
            break;
        default:
            break;
    }
}

void UART2_LoRa_Init(void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);
    assert(FSP_SUCCESS == err);
}

/* =================================================================================================
 *  GPS 接收 (SCI9/UART9) — 中断模式
 *  - NMEA 协议，每收到 '\n' 表示一帧结束
 * ================================================================================================= */
volatile uint8_t gps_rx_buf[GPS_RX_BUF_SIZE];
volatile bool gps_rx_complete = false;
/** GPS 回调 - UART9（SCI9），DMAC 模式下 RX_CHAR 由硬件处理 */
void gps_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
            uart_send_complete_flag = true;
            break;
        default:
            break;
    }
}

void UART9_GPS_Init(void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
    assert(FSP_SUCCESS == err);
}

/* =================================================================================================
 *  DMAC 辅助函数
 * ================================================================================================= */
void set_transfer_length(transfer_cfg_t const * const p_config, volatile uint16_t _length)
{
    p_config->p_info->length = _length;
}

void set_transfer_dst_src_address(transfer_cfg_t const * const p_config,
                                   const volatile uint8_t * _p_src,
                                   const volatile uint8_t * _p_dest)
{
    p_config->p_info->p_src  = (void const * volatile)_p_src;
    p_config->p_info->p_dest = (void * volatile)_p_dest;
}

/* =================================================================================================
 *  DMAC 初始化
 *  配置 DMAC0（IMU/SCI5）、DMAC2（LoRa/SCI2）、DMAC4（GPS/SCI9）
 * ================================================================================================= */
void DMAC_Init(void)
{
    fsp_err_t err;

    /* ---------- IMU RX (DMAC0, SCI5 RXI) ---------- */
    set_transfer_length(&g_transfer0_cfg, IMU_RX_BUF_SIZE);
    set_transfer_dst_src_address(&g_transfer0_cfg,
                                  (const volatile uint8_t *)&R_SCI5->RDR,
                                  (const volatile uint8_t *)imu_rx_buf);
    err = g_transfer_on_dmac.open(&g_transfer0_ctrl, &g_transfer0_cfg);
    assert(FSP_SUCCESS == err);
    err = g_transfer_on_dmac.enable(&g_transfer0_ctrl);
    assert(FSP_SUCCESS == err);

    /* ---------- LoRa RX (DMAC2, SCI2 RXI) ---------- */
    set_transfer_length(&g_transfer2_cfg, LORA_RX_BUF_SIZE);
    set_transfer_dst_src_address(&g_transfer2_cfg,
                                  (const volatile uint8_t *)&R_SCI2->RDR,
                                  (const volatile uint8_t *)lora_rx_buf);
    err = g_transfer_on_dmac.open(&g_transfer2_ctrl, &g_transfer2_cfg);
    assert(FSP_SUCCESS == err);
    err = g_transfer_on_dmac.enable(&g_transfer2_ctrl);
    assert(FSP_SUCCESS == err);

    /* ---------- GPS RX (DMAC4, SCI9 RXI) ---------- */
    set_transfer_length(&g_transfer4_cfg, GPS_RX_BUF_SIZE);
    set_transfer_dst_src_address(&g_transfer4_cfg,
                                  (const volatile uint8_t *)&R_SCI9->RDR,
                                  (const volatile uint8_t *)gps_rx_buf);
    err = g_transfer_on_dmac.open(&g_transfer4_ctrl, &g_transfer4_cfg);
    assert(FSP_SUCCESS == err);
    err = g_transfer_on_dmac.enable(&g_transfer4_ctrl);
    assert(FSP_SUCCESS == err);
}

/* =================================================================================================
 *  DMAC 回调（传输完成后由 DMAC 中断调用）
 * ================================================================================================= */

/** IMU DMAC 回调 - 在中断中复位 DMAC */
void transfer_imu_rx_callback(transfer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    imu_rx_complete = true;

    set_transfer_length(&g_transfer0_cfg, IMU_RX_BUF_SIZE);
    set_transfer_dst_src_address(&g_transfer0_cfg,
                                  (const volatile uint8_t *)&R_SCI5->RDR,
                                  (const volatile uint8_t *)imu_rx_buf);
    (void)g_transfer_on_dmac.reconfigure(&g_transfer0_ctrl,
                                         g_transfer0_cfg.p_info);
}

/** LoRa DMAC 回调 */
void transfer_lora_rx_callback(transfer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    lora_rx_complete = true;
}

/** GPS DMAC 回调 */
void transfer_gps_rx_callback(transfer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    gps_rx_complete = true;
}

/* =================================================================================================
 *  DMAC 重置
 * ================================================================================================= */
void IMU_DMAC_Reset(void)
{
    fsp_err_t err;
    imu_rx_complete = false;
    set_transfer_length(&g_transfer0_cfg, IMU_RX_BUF_SIZE);
    set_transfer_dst_src_address(&g_transfer0_cfg,
                                  (const volatile uint8_t *)&R_SCI5->RDR,
                                  (const volatile uint8_t *)imu_rx_buf);
    err = g_transfer_on_dmac.reconfigure(&g_transfer0_ctrl, g_transfer0_cfg.p_info);
    assert(FSP_SUCCESS == err);
}

void LORA_DMAC_Reset(void)
{
    fsp_err_t err;
    lora_rx_complete = false;
    set_transfer_length(&g_transfer2_cfg, LORA_RX_BUF_SIZE);
    set_transfer_dst_src_address(&g_transfer2_cfg,
                                  (const volatile uint8_t *)&R_SCI2->RDR,
                                  (const volatile uint8_t *)lora_rx_buf);
    err = g_transfer_on_dmac.reconfigure(&g_transfer2_ctrl, g_transfer2_cfg.p_info);
    assert(FSP_SUCCESS == err);
}

void GPS_DMAC_Reset(void)
{
    fsp_err_t err;
    gps_rx_complete = false;
    set_transfer_length(&g_transfer4_cfg, GPS_RX_BUF_SIZE);
    set_transfer_dst_src_address(&g_transfer4_cfg,
                                  (const volatile uint8_t *)&R_SCI9->RDR,
                                  (const volatile uint8_t *)gps_rx_buf);
    err = g_transfer_on_dmac.reconfigure(&g_transfer4_ctrl, g_transfer4_cfg.p_info);
    assert(FSP_SUCCESS == err);
}

/* =================================================================================================
 *  printf 重定向（通过 UART5 输出）
 * ================================================================================================= */
int fputc(int ch, FILE *f)
{
    fsp_err_t err;
    (void)f;

    err = R_SCI_UART_Write(&g_uart5_ctrl, (uint8_t *)&ch, 1);
    if (FSP_SUCCESS != err) __BKPT();
    while (!uart_send_complete_flag) {}
    uart_send_complete_flag = false;
    return ch;
}


