#include <stdio.h>
#include "uart.h"
#include "headfile.h"

/* =================================================================================================
 *  UART 发送完成标志（用于 printf 等阻塞发送）
 * ================================================================================================= */
static volatile bool uart_send_complete_flag = false;

/* =================================================================================================
 *  IMU 接收 (SCI2/UART2 + DMAC0)
 *  - DMAC0 通过 ELC 由 SCI2 RXI 事件触发
 *  - 每次收到 IMU_RX_BUF_SIZE 字节后触发 DMAC 回调
 * ================================================================================================= */
BSP_ALIGN_VARIABLE(4) volatile uint8_t imu_rx_buf[IMU_RX_BUF_SIZE];
volatile bool imu_rx_complete = false;

/** UART2 回调（IMU）—— DMA 模式下 RX_CHAR 不会频繁触发 */
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

/** DMAC0 传输完成回调（IMU 数据接收完毕） */
void transfer_imu_rx_callback(transfer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    imu_rx_complete = true;
}

/** 初始化 UART2（IMU） */
void UART2_IMU_Init(void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);
    assert(FSP_SUCCESS == err);
}

/* =================================================================================================
 *  LoRa 接收 (SCI5/UART5 + DMAC2)
 *  - DMAC2 通过 ELC 由 SCI5 RXI 事件触发
 *  - 每次收到 LORA_RX_BUF_SIZE 字节后触发 DMAC 回调
 * ================================================================================================= */
BSP_ALIGN_VARIABLE(4) volatile uint8_t lora_rx_buf[LORA_RX_BUF_SIZE];
volatile bool lora_rx_complete = false;

/** UART5 回调（LoRa）—— DMA 模式下 RX_CHAR 不会频繁触发 */
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

/** DMAC2 传输完成回调（LoRa 数据接收完毕） */
void transfer_lora_rx_callback(transfer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    lora_rx_complete = true;
}

/** 初始化 UART5（LoRa） */
void UART5_LoRa_Init(void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart5_ctrl, &g_uart5_cfg);
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

/* =================================================================================================
 *  DMAC 初始化
 *  配置 DMAC0（IMU）和 DMAC2（LoRa）：
 *    - 源地址：SCI 接收数据寄存器 RDR（固定地址）
 *    - 目标地址：接收缓冲区（递增地址）
 *    - 传输长度：缓冲区大小
 *    - 触发源：SCI RXI 事件（通过 ELC）
 * ================================================================================================= */
void DMAC_Init(void)
{
    fsp_err_t err;

    /* ---------- IMU RX (DMAC0, SCI2 RXI) ---------- */
    /* 先配置传输信息再 open */
    g_transfer_imu_rx_cfg.p_info->length = IMU_RX_BUF_SIZE;
    g_transfer_imu_rx_cfg.p_info->p_src  = (void const *)&R_SCI2->RDR;
    g_transfer_imu_rx_cfg.p_info->p_dest = (void *)imu_rx_buf;

    err = g_transfer_on_dmac.open(&g_transfer_imu_rx_ctrl, &g_transfer_imu_rx_cfg);
    assert(FSP_SUCCESS == err);

    err = g_transfer_on_dmac.enable(&g_transfer_imu_rx_ctrl);
    assert(FSP_SUCCESS == err);

    /* ---------- LoRa RX (DMAC2, SCI5 RXI) ---------- */
    g_transfer_lora_rx_cfg.p_info->length = LORA_RX_BUF_SIZE;
    g_transfer_lora_rx_cfg.p_info->p_src  = (void const *)&R_SCI5->RDR;
    g_transfer_lora_rx_cfg.p_info->p_dest = (void *)lora_rx_buf;

    err = g_transfer_on_dmac.open(&g_transfer_lora_rx_ctrl, &g_transfer_lora_rx_cfg);
    assert(FSP_SUCCESS == err);

    err = g_transfer_on_dmac.enable(&g_transfer_lora_rx_ctrl);
    assert(FSP_SUCCESS == err);
}

/* =================================================================================================
 *  DMAC 重置（下一轮接收准备）
 *  使用 reconfigure() 重新设置源/目标地址和传输长度
 * ================================================================================================= */

/** IMU DMAC 重置：准备下一帧接收 */
void IMU_DMAC_Reset(void)
{
    fsp_err_t err;

    imu_rx_complete = false;

    /* 更新 info 结构体中的地址和长度 */
    g_transfer_imu_rx_cfg.p_info->length = IMU_RX_BUF_SIZE;
    g_transfer_imu_rx_cfg.p_info->p_src  = (void const *)&R_SCI2->RDR;
    g_transfer_imu_rx_cfg.p_info->p_dest = (void *)imu_rx_buf;

    /* 重新配置 DMAC */
    err = g_transfer_on_dmac.reconfigure(&g_transfer_imu_rx_ctrl, g_transfer_imu_rx_cfg.p_info);
    assert(FSP_SUCCESS == err);
}

/** LoRa DMAC 重置：准备下一帧接收 */
void LORA_DMAC_Reset(void)
{
    fsp_err_t err;

    lora_rx_complete = false;

    g_transfer_lora_rx_cfg.p_info->length = LORA_RX_BUF_SIZE;
    g_transfer_lora_rx_cfg.p_info->p_src  = (void const *)&R_SCI5->RDR;
    g_transfer_lora_rx_cfg.p_info->p_dest = (void *)lora_rx_buf;

    err = g_transfer_on_dmac.reconfigure(&g_transfer_lora_rx_ctrl, g_transfer_lora_rx_cfg.p_info);
    assert(FSP_SUCCESS == err);
}



