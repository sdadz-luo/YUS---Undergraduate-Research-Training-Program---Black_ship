#include "gpt.h"
#include "headfile.h"

/* 控制变量 */
float gyro, yaw;                /* IMU 解析值 */
float pwm_l, pwm_r, pwm_turn;   /* PWM 计算结果 */
extern pid pid_gyro, pid_yaw;
extern uint8_t v, move;

/* 5ms 定时器初始化 */
void gpt0_init(void)
{
    R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);
    R_GPT_Start(&g_timer0_ctrl);
}

/* 5ms 定时回调：IMU → PID → PWM */
void gpt0_callback(timer_callback_args_t *p_args)
{
    (void)p_args;

    /* ---- 读取最新 IMU 数据 ---- */
    if (imu_rx_complete)
    {
        int16_t gyro_raw = (int16_t)((imu_rx_buf[7] << 8) | imu_rx_buf[6]);
        int16_t yaw_raw  = (int16_t)((imu_rx_buf[18] << 8) | imu_rx_buf[17]);
        gyro = (float)gyro_raw / 32768.0f * 2000.0f;
        yaw  = (float)yaw_raw  / 32768.0f * 180.0f;
        imu_rx_complete = false;
    }

    /* ---- PID 控制 + PWM 输出 ---- */
    pwm_turn = pid_location(&pid_gyro, gyro);
    pwm_l = 1000 * v - pwm_turn;
    pwm_r = 1000 * v + pwm_turn;
    pwm_setduty(pwm_l, pwm_r);
}