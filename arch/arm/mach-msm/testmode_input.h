#define TESTMODE_INPUT_KEY_HOME				1
#define TESTMODE_INPUT_KEY_BACK				2

#define TESTMODE_INPUT_KEY_INIT					3
#define TESTMODE_INPUT_KEY_UNLOCK			4
#define TESTMODE_INPUT_KEY_TDMB				5
// 250-27 MP3 Test
#define TESTMODE_1kHz_0dB_LR_128k					6
#define TESTMODE_1kHz_0dB_L_128k					7
#define TESTMODE_1kHz_0dB_R_128k					8 
#define TESTMODE_MultiSine_20_20kHz_0dBp		9
#define TESTMODE_MP3_Play_Mode_OFF				10
#define TESTMODE_MP3_Sample_File_Compare	11
#define TESTMODE_NoSignal_LR_128k					12
// 250-43 Speaker Phone Test
#define TESTMODE_Speaker_Phone_ON					13
#define TESTMODE_Speaker_Phone_OFF				14
#define TESTMODE_Normal_Mic1								15
// 250-51 Volume Level Test
#define TESTMODE_Volume_Level_0						16
#define TESTMODE_Minimum_Volume_Level 			17
#define TESTMODE_Medium_Volume_Level			18
#define TESTMODE_Maximum_Volume_Level			19
// 250-4 Acoustic Test
#define TESTMODE_Acoustic_ON								20
#define TESTMODE_Headset_Path_Open				21
#define TESTMODE_Handset_Path_Open				22
#define TESTMODE_ACOUSTIC_Loopback_ON								23
#define TESTMODE_Acoustic_OFF								24
#define TESTMODE_Acoustic_Loopback_OFF			25
// 250-3 Motor Test
#define TESTMODE_Motor_ON									26
#define TESTMODE_Moter_OFF									27
// 250-7 Camera Test
#define TESTMODE_CAMERA_MODE_ON					28
#define TESTMODE_CAMERA_SHOT							29
#define TESTMODE_CAMERA_SAVE_IMAGE			30
#define TESTMODE_CAMERA_CALL_IMAGE			31
#define TESTMODE_CAMERA_ERASE_IMAGE			32
#define TESTMODE_CAMERA_MODE_OFF				33
#define TESTMODE_CAMCORDER_MODE_ON			34
#define TESTMODE_CAMCORDER_SHOT_RECORD_START			35
#define TESTMODE_CAMCORDER_RECORD_STOP_AND_SAVE	36
#define TESTMODE_CAMCORDER_PLAY_MOVING_FILE				37
#define TESTMODE_CAMCORDER_ERASE_MOVING_FILE			38	
#define TESTMODE_CAMCORDER_CAMCORDER_MODE_OFF		39	
// 250-1 LCD Test
#define TESTMODE_LCD_INITIAL								40
#define TESTMODE_LCD_TILT									41
#define TESTMODE_LCD_COLOR_DISPLAY				42

// 250-1 TDMB Test
#define TESTMODE_INPUT_KEY_TDMB_ON				43
#define TESTMODE_INPUT_KEY_TDMB_OFF				44
#define TESTMODE_INPUT_KEY_TDMB_RM_CH			45
#define TESTMODE_INPUT_KEY_TDMB_CHK				46

// 250-50 Factory Reset Test
#define TESTMODE_INPUT_FACTORY_RESET			47

//250-4 Acoustic Test 7.9  update
#define TESTMODE_PATH_CHANGE_TO_MAIN_EARJACK	48
#define TESTMODE_PATH_CHANGE_TO_SUB_EARJACK		49
#define TESTMODE_PATH_CHANGE_TO_DEFAULT			50
#define TESTMODE_PATH_CHANGE_TO_BT_SCO_CALL_PATH 51
#define TESTMODE_PATH_CHANGE_TO_BT_SCO_CALL_PATH_OFF 52

#define TESTMODE_POUND 	53
#define TESTMODE_STAR	54
#define TESTMODE_NUM_0 	55
#define TESTMODE_NUM_1 	56
#define TESTMODE_NUM_2 	57
#define TESTMODE_NUM_3 	58
#define TESTMODE_NUM_4 	59
#define TESTMODE_NUM_5 	60
#define TESTMODE_NUM_6 	61 
#define TESTMODE_NUM_7 	62 
#define TESTMODE_NUM_8 	63 
#define TESTMODE_NUM_9 	64
#define TESTMODE_CALL 		65
#define TESTMODE_ENDCALL 	66 
#define TESTMODE_CLEAR 		67
#define TESTMODE_DPAD_CENTER 68 
#define TESTMODE_DPAD_UP	69 
#define TESTMODE_DPAD_DOWN 	70
#define TESTMODE_DPAD_LEFT 		71 
#define TESTMODE_DPAD_RIGHT 	72
#define TESTMODE_LCDOFF_AND_LOCK 	73
#define TESTMODE_ENABLE_ADB 	74


#if 1
#define MAX_KEYS	74
#else
#define MAX_KEYS	73
#endif

/* exported function to deliver an event to the android event hub */
void testmode_input_report_evt(int evtcode);
