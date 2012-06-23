#include <linux/module.h>
#include <linux/kernel.h> 
#include <mach/msm_rpcrouter.h>
#include <linux/slab.h>
#include "rpc_server_testmode.h"
#include "testmode_input.h"



#define TESTMODE_SUCCESS		0
#define TESTMODE_FAIL	1
#define TESTMODE_NOT_SUPPORTED 2


//extern void akm_test_mode_get_value( int is_pitch, short *value);
extern int ami304_test_mode_get_value(int sub_cmd, int *value);
extern void proximity_test_mode_get_value( int *value );
extern void *testmode_dispath_command(uint32_t sub1_cmd, uint32_t sub2_cmd);


void* testmode_proximity(uint32_t sub1_cmd, uint32_t sub2_cmd)
{
	int value;

	struct testmode_relay_result *relay_result;

	relay_result = (struct testmode_relay_result *)kzalloc(sizeof(struct testmode_relay_result), GFP_KERNEL);

	switch( sub2_cmd )
	{
		case 0:
			proximity_test_mode_get_value( &value );
			relay_result->ret_value = TESTMODE_SUCCESS;

			relay_result->ret_string[0] = value;
			printk("****testmode_proximity ON****: value = %d \n", relay_result->ret_string[0] );
			break;

		case 1:
			
			relay_result->ret_value = TESTMODE_SUCCESS;
			printk("****testmode_proximity OFF**** \n" );
			break;

		default:

			break;
	}

	return relay_result;


}

void* testmode_accelator(uint32_t sub1_cmd, uint32_t sub2_cmd)
{
	int value;
	char pitch, roll;

	struct testmode_relay_result *relay_result;

	relay_result = (struct testmode_relay_result *)kzalloc(sizeof(struct testmode_relay_result), GFP_KERNEL);
	relay_result->ret_value = TESTMODE_FAIL;

	testmode_input_report_evt(TESTMODE_INPUT_KEY_INIT);
	testmode_input_report_evt(TESTMODE_INPUT_KEY_HOME);

	switch( sub2_cmd )
	{
		case 0: // pitch
			if(ami304_test_mode_get_value( 0, &value ))
				relay_result->ret_value = TESTMODE_SUCCESS;
			else
				break;

			value = (int)(value/10);
			if( value < 0 )
			{
				value = (-1)*value;	
				relay_result->ret_string[0] = 0x01;
			}
			else
			{
				relay_result->ret_string[0] = 0x02;	
			}
			if(value > 180 ) value =180;
			pitch = (char)(value > 90 ? 180-value : value);
			relay_result->ret_string[1] = pitch;
			printk("****testmode_accelator****: sign = %d, pitch = %d \n", relay_result->ret_string[0], relay_result->ret_string[1] );
			break;

		case 1: // roll
			if(	ami304_test_mode_get_value( 1, &value ))
				relay_result->ret_value = TESTMODE_SUCCESS;
			else
				break;

			value = (int)(value/10);
			if( value < 0 )
			{
				value = (-1)*value;					
				relay_result->ret_string[0] = 0x01;
			}
			else
			{
				relay_result->ret_string[0] = 0x02;
			}
			if(value > 180 ) value =180;
			roll = (char)(value > 90 ? (180-value) : value);
			relay_result->ret_string[1] = roll;
			printk("****testmode_accelator****: sign = %d, roll = %d \n", relay_result->ret_string[0], relay_result->ret_string[1] );
			break;

		default:
			relay_result->ret_value = TESTMODE_FAIL;
			break;
	}

	//relay_result = testmode_dispath_command(sub1_cmd, sub2_cmd);

	return relay_result;
		
}

