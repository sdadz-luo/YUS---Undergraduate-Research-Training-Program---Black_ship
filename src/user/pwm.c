#include "headfile.h"
#include "pwm.h"

#define PWM_MAX     8000

void pwm_init(void){

	R_GPT_Open(&g_timer6_ctrl,&g_timer6_cfg);
	R_GPT_Start(&g_timer6_ctrl);
	
}

void pwm_setduty(float B, float A){

    if (A < 0) {
        A = -A;
        R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_05_PIN_11, BSP_IO_LEVEL_LOW);
        R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_06, BSP_IO_LEVEL_HIGH);
    }else{
        R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_05_PIN_11, BSP_IO_LEVEL_HIGH);
        R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_06, BSP_IO_LEVEL_LOW);
    }
    if (B < 0) {
        B = -B;
        R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_08_PIN_03, BSP_IO_LEVEL_LOW);
        R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_08_PIN_04, BSP_IO_LEVEL_HIGH);
    }else{
        R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_08_PIN_03, BSP_IO_LEVEL_HIGH);
        R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_08_PIN_04, BSP_IO_LEVEL_LOW);
    }

    if (A >= PWM_MAX) A = PWM_MAX;
    if (B >= PWM_MAX) B = PWM_MAX;
    // Ë«Â·PWMÊä³ö
    R_GPT_DutyCycleSet(&g_timer6_ctrl, (uint32_t)A, GPT_IO_PIN_GTIOCA);
    R_GPT_DutyCycleSet(&g_timer6_ctrl, (uint32_t)B, GPT_IO_PIN_GTIOCB);
}
