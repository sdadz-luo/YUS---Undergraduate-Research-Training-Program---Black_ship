#include "headfile.h"
#include "pwm.h"

#define PWM_MAX     8000

void pwm_init(void){

	R_GPT_Open(&g_timer6_ctrl,&g_timer6_cfg);
	R_GPT_Start(&g_timer6_ctrl);
	
}

void pwm_setduty(uint32_t A, uint32_t B){
    
    A = (uint32_t)A; B = (uint32_t)B;

    if (A < 0) {
        A = -A;
    }else{

    }
    if (B < 0) {
        B = -B;
    }else{

    }

    if (A >= PWM_MAX) A = PWM_MAX;
    if (B >= PWM_MAX) B = PWM_MAX;
    // Ë«Â·PWMÊä³ö
    R_GPT_DutyCycleSet(&g_timer6_ctrl, A, GPT_IO_PIN_GTIOCA);
    R_GPT_DutyCycleSet(&g_timer6_ctrl, B, GPT_IO_PIN_GTIOCB);
}
