
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <mach/board_lge.h>
#include "../proc_comm.h"

#if defined(CONFIG_LGE_DETECT_PIF_PATCH)
unsigned lge_get_pif_info(void)
{
	int err;
	unsigned pif_value = -1;
	unsigned cmd_pif = 8;

	err = msm_proc_comm(PCOM_CUSTOMER_CMD2, &pif_value, &cmd_pif);
	if (err < 0) {
		pr_err("%s: msm_proc_comm(PCOM_CUSTOMER_CMD2) failed\n",
		       __func__);
		return err;
	}
	
	return pif_value;
}
EXPORT_SYMBOL(lge_get_pif_info);

unsigned lge_get_lpm_info(void)
{
	int err;
	unsigned low_power_mode = 0;
	unsigned cmd_lpm = 7;
	
	err = msm_proc_comm(PCOM_CUSTOMER_CMD2, &low_power_mode, &cmd_lpm);
	if (err < 0) {
		pr_err("%s: msm_proc_comm(PCOM_CUSTOMER_CMD2) failed\n",
		       __func__);
		return err;
	}

	return low_power_mode;
}
EXPORT_SYMBOL(lge_get_lpm_info);
#endif
