
#include <stddef.h>

#ifndef __ARCH_MSM_LGE_DIAG_TEST_H
#define __ARCH_MSM_LGE_DIAG_TEST_H

#define eta_execute(str) printk("Warning.  Using unsafe function eta_execute. %s:%d.", __FILE__, __LINE__); eta_execute_n(str, strlen(str))

extern uint8_t if_condition_is_on_key_buffering;
extern uint8_t lgf_factor_key_test_rsp(char);

extern int eta_execute_n(char *, size_t);
extern int base64_encode(char *, int, char *);

#endif

