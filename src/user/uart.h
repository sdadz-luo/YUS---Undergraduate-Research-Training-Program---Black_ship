#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>

/* ========== IMU 接收 (SCI2/UART2 + DMAC0) ========== */
#define IMU_RX_BUF_SIZE     28      /* IMU 数据包长度（根据实际IMU模块调整）*/
extern volatile uint8_t  imu_rx_buf[IMU_RX_BUF_SIZE];
extern volatile bool     imu_rx_complete;

/* ========== LoRa 接收 (SCI5/UART5 + DMAC2) ========== */
#define LORA_RX_BUF_SIZE    256     /* LoRa 数据包最大长度 */
extern volatile uint8_t  lora_rx_buf[LORA_RX_BUF_SIZE];
extern volatile bool     lora_rx_complete;

/* ========== 函数声明 ========== */
void UART2_IMU_Init(void);          /* IMU串口初始化 */
void UART5_LoRa_Init(void);         /* LoRa串口初始化 */
void DMAC_Init(void);               /* DMAC初始化（IMU + LoRa）*/
void IMU_DMAC_Reset(void);          /* IMU DMAC重置，准备下一轮接收 */
void LORA_DMAC_Reset(void);         /* LoRa DMAC重置，准备下一轮接收 */

#endif