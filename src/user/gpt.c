#include "gpt.h"
#include "headfile.h"

void gpt0_init(void){

	R_GPT_Open(&g_timer0_ctrl,&g_timer0_cfg);
	R_GPT_Start(&g_timer0_ctrl);
	
}

void gpt0_callback(timer_callback_args_t *p_args){
    (void)p_args;
}