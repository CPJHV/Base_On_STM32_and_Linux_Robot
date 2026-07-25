#include "user_controller.h"
extern Pwm_ pwm;
extern Encoder_ encoder;
extern  Monitor_Gpio gpio;
extern Adc_ adc;
extern Pid_ pidx;
extern Monitor_Controller monitor_contr;
void 	User_::Init_all_drivers(){

//	pidx.pid_init();
	
//初始化所有外设
	/*pwm.pwm1_tim_cplm_pwm_init();
	pwm.pwm2_tim_cplm_pwm_init();
	pwm.pwm3_tim_cplm_pwm_init();
	pwm.pwm4_tim_cplm_pwm_init();
	
	encoder.pwm1_tim_encoder_init();
	encoder.pwm2_tim_encoder_init();
	encoder.pwm3_tim_encoder_init();
	encoder.pwm4_tim_encoder_init();
	
	encoder.pwm1_btim_int_init();
	encoder.pwm2_btim_int_init();
encoder.pwm3_btim_int_init();
encoder.pwm4_btim_int_init();
	
	gpio.Init();
	
	adc.Init();
	
	monitor_contr.Monitor_Gpio_Init();
	
	S_Curve_Init();
	*/
}
