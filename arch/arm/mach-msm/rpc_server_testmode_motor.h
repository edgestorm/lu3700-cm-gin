typedef enum 
{
  MOTOR_TURN_OFF,
  MOTOR_TURN_ON  
} test_mode_req_motor_type;

extern void *testmode_motor_test(uint32_t sub1_cmd, uint32_t sub2_cmd);
extern void *testmode_reponse_not_supported(void);



