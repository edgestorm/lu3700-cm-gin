#include <linux/module.h>
#include <linux/kernel.h> 
#include <linux/interrupt.h>
#include <mach/msm_rpcrouter.h>
#include "rpc_server_testmode_motor.h"
#include "rpc_server_testmode_sound.h"
#include <linux/syscalls.h>
#include <linux/fcntl.h> 
#include "testmode_input.h"
#include <linux/delay.h>
#include <linux/slab.h>

#define TESTMODE_SUCCESS		0
#define TESTMODE_FAIL	1
#define TESTMODE_NOT_SUPPORTED 2

static int need_to_wait_for_motor = 0;

/* Motor Test (250-3-x) Test mode 7.8 */
void *testmode_motor_test(uint32_t sub1_cmd, uint32_t sub2_cmd)
{
	struct testmode_relay_result *relay_result;
	//int cnt =0;
	relay_result = (struct testmode_relay_result *)kzalloc(sizeof(struct testmode_relay_result), GFP_KERNEL);

	relay_result->ret_value = TESTMODE_SUCCESS;

	if(check_lcd_status() <= 0)
		need_to_wait_for_motor = true;

	testmode_input_report_evt(TESTMODE_INPUT_KEY_INIT);
	if(need_to_wait_for_motor==true)
		msleep(10);
	testmode_input_report_evt(TESTMODE_INPUT_KEY_UNLOCK);

	if(need_to_wait_for_motor==true)
		msleep(20);
	
	//for(cnt=0;cnt<6;cnt++)
		//testmode_input_report_evt(TESTMODE_INPUT_KEY_BACK);

	switch (sub2_cmd) 
	{
		case MOTOR_TURN_ON:
			testmode_input_report_evt(TESTMODE_Motor_ON);
			break;
			
		case MOTOR_TURN_OFF:
		 	testmode_input_report_evt(TESTMODE_Moter_OFF);
			break;

		default:
			return testmode_reponse_not_supported();
	}

	need_to_wait_for_motor=false;
	return relay_result;
}

