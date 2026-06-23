#include "gpt.h"
#include "headfile.h"  

void gpt0_init(void){

	R_GPT_Open(&g_timer0_ctrl,&g_timer0_cfg);
	R_GPT_Start(&g_timer0_ctrl);
	
}

void gpt0_callback(timer_callback_args_t *p_args){
    (void)p_args;

    if (imu_rx_complete){
        // === imu_rx_buf[] 中已有一帧完整数据 ===
		gyro=(float)((imu_rx_buf[7]<<8)|imu_rx_buf[6])/32768*2000;
		yaw=(float)((imu_rx_buf[18]<<8)|imu_rx_buf[17])  /32768*180;
        imu_rx_complete = false;  // 标记数据已消费
    }

}