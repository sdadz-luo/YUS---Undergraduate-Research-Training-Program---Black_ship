#include <stdio.h>
#include "headfile.h"
#include "uart.h"

gyroTypeDef GYRO;
angleTypeDef ANGEL;

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
static uint16_t imu_rx_idx = 0;

/** IMU 回调 - UART5（SCI5） */
void imu_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
            imu_rx_buf[imu_rx_idx++] = (uint8_t)p_args->data;
            if (imu_rx_idx >= IMU_RX_BUF_SIZE)
            {
                imu_rx_idx = 0;
                imu_rx_complete = true;
            }
            break;
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
static uint16_t lora_rx_idx = 0;

/** LoRa 回调 - UART2（SCI2） */
void lora_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
            lora_rx_buf[lora_rx_idx++] = (uint8_t)p_args->data;
            if (lora_rx_idx >= LORA_RX_BUF_SIZE)
            {
                lora_rx_idx = 0;
                lora_rx_complete = true;
            }
            break;
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
static uint16_t gps_rx_idx = 0;

void gps_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            uint8_t ch = (uint8_t)p_args->data;
            if (gps_rx_idx < GPS_RX_BUF_SIZE)
            {
                gps_rx_buf[gps_rx_idx++] = ch;
            }
            /* NMEA 语句以 \n 结尾，检测到换行表示一帧结束 */
            if (ch == '\n')
            {
                gps_rx_idx = 0;
                gps_rx_complete = true;
            }
            break;
        }
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


