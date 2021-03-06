
#ifndef __U_LGEUSB_H__
#define __U_LGEUSB_H__


#ifdef CONFIG_USB_SUPPORT_LGE_GADGET_CDMA

#define LGE_FACTORY_CABLE_TYPE 3
#define LGE_FACTORY_CABLE_130K_TYPE 10
#define LT_ADB_CABLE 0xff

#endif /* CDMA */

#ifdef CONFIG_USB_SUPPORT_LGE_GADGET_GSM

#define LGE_FACTORY_CABLE_TYPE 1
#define MAX_IMEI_LEN 19
#define LGE_PIF_CABLE 2

#endif /* GSM/WCDMA */

#define LGE_FACTORY_PID 0x6000
#define LGE_DEFAULT_PID 0x618E
/* #define LGE_PLATFORM_PID 0x618E */
#define LGE_UMSONLY_PID 0x61C5 /* LGP500 */

#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
#define LGE_CDONLY_PID 0x91C8 /* TEST */
#define LGE_CHARGEONLY_PID 0xFFFF
#endif

enum lgeusb_mode {
	LGEUSB_FACTORY_MODE = 0,
	LGEUSB_ANDROID_MODE,
	LGEUSB_DEFAULT_MODE,
};

struct lgeusb_info {
	int current_pid;
	enum lgeusb_mode current_mode;
	char *serialno;
	const char *defaultno;
	void (*switch_func)(int pid, int need_reset);
	int (*get_pid)(void);
};

int lgeusb_detect_factory_cable(void);
int lgeusb_set_current_mode(int need_reset);
int lgeusb_get_current_mode(void);

#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
int lgeusb_get_usb_usermode(void);
#endif

void lgeusb_switch_factory_mode(int need_reset);
void lgeusb_switch_android_mode(int need_reset);

void lgeusb_register_usbinfo(struct lgeusb_info *info);

#if defined(LGEUSB_DYNAMIC_DEBUG)
static int lgeusb_debug_mask;

module_param_named(debug, lgeusb_debug_mask, int,
		S_IRUGO | S_IWUSR | S_IWGRP);

#define lgeusb_debug(fmt, args...) \
	do { \
		if (lgeusb_debug_mask) \
		printk(KERN_INFO "lgeusb[%-18s:%5d] - " \
				fmt, __func__, __LINE__, ## args); \
	} while (0)
#elif defined(LGEUSB_DEBUG)
#define lgeusb_debug(fmt, args...) \
		printk(KERN_INFO "lgeusb[%-18s:%5d] - " \
				fmt, __func__, __LINE__, ## args); \

#else
#define lgeusb_debug(fmt, args...) do {} while (0)
#endif /* LGEUSB_DYNAMIC_DEBUG */

#define lgeusb_info(fmt, args...) \
	printk(KERN_INFO "lgeusb[%-18s:%5d] - " \
				fmt, __func__, __LINE__, ## args); \

#endif /* __U_LGEUSB_H__ */
