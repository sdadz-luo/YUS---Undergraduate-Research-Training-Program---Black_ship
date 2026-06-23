#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>


typedef struct{
    uint8_t WxL;
    uint8_t WxH;
    uint8_t WyL;
    uint8_t WyH;
    uint8_t WzL;
    uint8_t WzH;
    int gyroX;
    int gyroY;
    float gyroZ;
}gyroTypeDef;

typedef struct{  
    uint8_t RollL;
    uint8_t RollH;
	uint8_t PitchL;
    uint8_t PitchH;
	uint8_t YawL;
	uint8_t YawH;
	double angleX;
	double angleY;
	double angleZ;
}angleTypeDef;

extern gyroTypeDef GYRO;
extern angleTypeDef ANGEL;

/* ========== IMU 接收 (SCI5/UART5) ========== */
#define IMU_RX_BUF_SIZE     22      /* IMU 数据包长度（根据实际IMU模块调整）*/
extern volatile uint8_t  imu_rx_buf[IMU_RX_BUF_SIZE];
extern volatile bool     imu_rx_complete;

/* ========== LoRa 接收 (SCI2/UART2) ========== */
#define LORA_RX_BUF_SIZE    256     /* LoRa 数据包最大长度 */
extern volatile uint8_t  lora_rx_buf[LORA_RX_BUF_SIZE];
extern volatile bool     lora_rx_complete;

/* ========== GPS 接收 (SCI9/UART9) ========== */
#define GPS_RX_BUF_SIZE     128     /* GPS NMEA 数据包最大长度 */
extern volatile uint8_t  gps_rx_buf[GPS_RX_BUF_SIZE];
extern volatile bool     gps_rx_complete;

/* ========== 函数声明 ========== */
void UART5_IMU_Init(void);          /* IMU串口初始化 (SCI5) */
void UART2_LoRa_Init(void);         /* LoRa串口初始化 (SCI2) */
void UART9_GPS_Init(void);          /* GPS串口初始化 (SCI9) */

#endif