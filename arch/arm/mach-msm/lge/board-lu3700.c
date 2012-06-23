
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/power_supply.h>


#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/pmic.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_serial_hs.h>
#include <mach/memory.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#include <mach/camera.h>
#include <mach/socinfo.h>
#include "devices.h"
#include "clock.h"
#include "msm-keypad-devices.h"
#include "pm.h"
#ifdef CONFIG_ARCH_MSM7X27
#include <linux/msm_kgsl.h>
#endif
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include <mach/board_lge.h>
#include "board-lu3700.h"

extern struct msm_pm_platform_data msm7x25_pm_data[MSM_PM_SLEEP_MODE_NR];
extern struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR];
#if 0
struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 20000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 20000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 0,
};
#endif

#ifdef CONFIG_USB_ANDROID

char *usb_functions_lge_all[] = {
#ifdef CONFIG_USB_ANDROID_MTP
	"mtp",
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
#ifdef CONFIG_USB_ANDROID_CDC_ECM
	"ecm",
	"acm2",
#endif
#ifdef CONFIG_USB_F_SERIAL
	"nmea",
#endif
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
	"usb_cdrom_storage",
	"charge_only",
#endif
	"usb_mass_storage",
	"adb",
};


char *usb_functions_lge_android_plat[] = {
	"acm", "diag", "nmea", "usb_mass_storage",
};

char *usb_functions_lge_android_plat_adb[] = {
	"acm", "diag", "nmea", "usb_mass_storage", "adb",
};

#ifdef CONFIG_USB_ANDROID_CDC_ECM
char *usb_functions_lge_android_net[] = {
	"diag", "ecm", "acm2", "nmea", "usb_mass_storage",
};

char *usb_functions_lge_android_net_adb[] = {
	"diag", "ecm", "acm2", "nmea", "usb_mass_storage", "adb",
};
#endif

#ifdef CONFIG_USB_ANDROID_RNDIS
char *usb_functions_lge_android_rndis[] = {
	"rndis",
};

char *usb_functions_lge_android_rndis_adb[] = {
	"rndis", "adb",
};
#endif

#ifdef CONFIG_USB_ANDROID_MTP
char *usb_functions_lge_android_mtp[] = {
	"mtp",
};

char *usb_functions_lge_android_mtp_adb[] = {
	"mtp", "adb",
};
#endif

char *usb_functions_lge_manufacturing[] = {
	"acm", "diag",
};

char *usb_functions_lge_mass_storage_only[] = {
	"usb_mass_storage",
};

#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
char *usb_functions_lge_cdrom_storage_only[] = {
	"usb_cdrom_storage",
};

char *usb_functions_lge_cdrom_storage_adb[] = {
	"usb_cdrom_storage", "adb",
};

char *usb_functions_lge_charge_only[] = {
	"charge_only",
};
#endif

struct android_usb_product usb_products[] = {
	{
		.product_id = 0x618E,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_plat),
		.functions = usb_functions_lge_android_plat,
	},
	{
		.product_id = 0x618E,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_plat_adb),
		.functions = usb_functions_lge_android_plat_adb,
	},
#ifdef CONFIG_USB_ANDROID_CDC_ECM
	{
		.product_id = 0x61A2,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_net),
		.functions = usb_functions_lge_android_net,
	},
	{
		.product_id = 0x61A1,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_net_adb),
		.functions = usb_functions_lge_android_net_adb,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		.product_id = 0x61DA,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_rndis),
		.functions = usb_functions_lge_android_rndis,
	},
	{
		.product_id = 0x61D9,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_rndis_adb),
		.functions = usb_functions_lge_android_rndis_adb,
	},
#endif
#ifdef CONFIG_USB_ANDROID_MTP
	{
		.product_id = 0x61C7,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_mtp),
		.functions = usb_functions_lge_android_mtp,
	},
	{
		.product_id = 0x61F9,
		.num_functions = ARRAY_SIZE(usb_functions_lge_android_mtp_adb),
		.functions = usb_functions_lge_android_mtp_adb,
	},
#endif
	{
		.product_id = 0x6000,
		.num_functions = ARRAY_SIZE(usb_functions_lge_manufacturing),
		.functions = usb_functions_lge_manufacturing,
	},
	{
		.product_id = 0x61C5,
		.num_functions = ARRAY_SIZE(usb_functions_lge_mass_storage_only),
		.functions = usb_functions_lge_mass_storage_only,
	},
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
	{
		/* FIXME: This pid is just for test */
		.product_id = 0x91C8,
		.num_functions = ARRAY_SIZE(usb_functions_lge_cdrom_storage_only),
		.functions = usb_functions_lge_cdrom_storage_only,
	},
	{
		.product_id = 0x61A6,
		.num_functions = ARRAY_SIZE(usb_functions_lge_cdrom_storage_adb),
		.functions = usb_functions_lge_cdrom_storage_adb,
	},
	{
		/* Charge only doesn't have no specific pid */
		.product_id = 0xFFFF,
		.num_functions = ARRAY_SIZE(usb_functions_lge_charge_only),
		.functions = usb_functions_lge_charge_only,
	},
#endif
};

struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns      = 1,
	.vendor     = "LGE",
	.product    = "Android Platform",
	.release    = 0x0100,
	.can_stall	= 1, 
};

struct platform_device usb_mass_storage_device = {
	.name   = "usb_mass_storage",
	.id 	= -1,
	.dev    = {
		.platform_data = &mass_storage_pdata,
	},
};

struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID   	= 0x1004,
	.vendorDescr    = "LG Electronics Inc.",
};

struct platform_device rndis_device = {
	.name   = "rndis",
	.id 	= -1,
	.dev    = {
		.platform_data = &rndis_pdata,
	},
};


#ifdef CONFIG_USB_ANDROID_CDC_ECM
struct usb_ether_platform_data ecm_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID   	= 0x1004,
	.vendorDescr    = "LG Electronics Inc.",
};

struct platform_device ecm_device = {
	.name   = "ecm",
	.id 	= -1,
	.dev    = {
		.platform_data = &ecm_pdata,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID_ACM
struct acm_platform_data acm_pdata = {
	.num_inst	    = 1,
};

struct platform_device acm_device = {
	.name   = "acm",
	.id 	= -1,
	.dev    = {
		.platform_data = &acm_pdata,
	},
};
#endif

#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
struct usb_cdrom_storage_platform_data cdrom_storage_pdata = {
	.nluns      = 1,
	.vendor     = "LGE",
	.product    = "Android Platform",
	.release    = 0x0100,
};

struct platform_device usb_cdrom_storage_device = {
	.name   = "usb_cdrom_storage",
	.id = -1,
	.dev    = {
		.platform_data = &cdrom_storage_pdata,
	},
};
#endif

struct android_usb_platform_data android_usb_pdata = {
	.vendor_id  = 0x1004,
	.product_id = 0x618E,
	.version    = 0x0100,
	.product_name       = "LGE USB Device",
	.manufacturer_name  = "LG Electronics Inc.",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_lge_all),
	.functions = usb_functions_lge_all,
#ifdef CONFIG_MACH_MSM7X27_LU3700
  .serial_number = "LG_ANDROID_LU3700**",
#elif defined(CONFIG_MACH_MSM7X27_KU3700)
  .serial_number = "LG_ANDROID_KU3700**",
#else
	.serial_number = "LG_ANDROID_P500_GB_",
#endif  
};

#endif /* CONFIG_USB_ANDROID */

#define DMB_POWER_GPIO    124
#define DMB_RESET_GPIO    100
#define DMB_INT_GPIO       99

#define DMB_POWER_GPIO_CONFIG     GPIO_CFG(DMB_POWER_GPIO, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define DMB_RESET_GPIO_CONFIG     GPIO_CFG(DMB_RESET_GPIO, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define DMB_INT_GPIO_CONFIG       GPIO_CFG(DMB_INT_GPIO, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio dmb_ebi2_gpios[] = {
	{ .gpio_cfg = DMB_POWER_GPIO_CONFIG,  .label =  "dmb_power_gpio", },
	{ .gpio_cfg = DMB_RESET_GPIO_CONFIG,   .label =  "dmb_reset_gpio", },
	{ .gpio_cfg = DMB_INT_GPIO_CONFIG, .label =  "dmb_int_gpio", },

};

//static struct msm_tsif_platform_data dmb_ebi2_platform_data = {
//	.num_gpios = ARRAY_SIZE(dmb_ebi2_gpios),
//	.gpios = dmb_ebi2_gpios,
//};

#define EBI2_CS2_BASE_PHYS   0x98000000
#define EBI2_CR_REG_BASE_PHYS   0xA0D00000
#define EBI2_XM_REG_BASE_PHYS   0xA0D10000


static struct resource dmb_ebi2_resources[] = {
	{
		.name	= "dmb_ebi2_cr_base",
		.start	= EBI2_CR_REG_BASE_PHYS,
		.end	       = EBI2_CR_REG_BASE_PHYS + 0x100 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "dmb_ebi2_xm_base",
		.start	= EBI2_XM_REG_BASE_PHYS,
		.end	       = EBI2_XM_REG_BASE_PHYS + 0x100 - 1,
		.flags	= IORESOURCE_MEM,
	},	
	{
		.name	= "dmb_ebi2_phys_memory",
		.start	= EBI2_CS2_BASE_PHYS,
		.end	       = EBI2_CS2_BASE_PHYS + 0xFFFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "dmb_ebi2_int",
		.start	= MSM_GPIO_TO_INT(DMB_INT_GPIO),
		.end	= MSM_GPIO_TO_INT(DMB_INT_GPIO),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_dmb_ebi2_device = {
	.name = "tdmb_lg2102",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(dmb_ebi2_resources),
	.resource	= dmb_ebi2_resources,
};

int usb_power_on( int on_off )
{
	int rc = 0;

    unsigned usb_en_mpp = 6; //mpp7

	if( on_off ) 
		rc = mpp_config_digital_out(usb_en_mpp,
							MPP_CFG(MPP_DLOGIC_LVL_MSMP,
							MPP_DLOGIC_OUT_CTRL_HIGH));
	else
		rc = mpp_config_digital_out(usb_en_mpp,
							MPP_CFG(MPP_DLOGIC_LVL_MSMP,
							MPP_DLOGIC_OUT_CTRL_LOW));
	return rc ;
}

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_dmov,
	&msm_device_nand,
	&msm_device_i2c,
	&msm_device_uart_dm1,
	&msm_device_snd,
	&msm_device_adspdec,
	&msm_dmb_ebi2_device,
};

extern struct sys_timer msm_timer;

static void __init msm7x2x_init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data msm7x2x_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 400000, 
	.vdd_switch_time_us = 62,
	.max_axi_khz = 160000,
};

void msm_serial_debug_init(unsigned int base, int irq,
			   struct device *clk_device, int signal_irq);

static void msm7x27_wlan_init(void)
{
	int rc = 0;
	if (machine_is_msm7x27_ffa()) {
		rc = mpp_config_digital_out(3, MPP_CFG(MPP_DLOGIC_LVL_MSMP,
				MPP_DLOGIC_OUT_CTRL_LOW));
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
				__func__, rc);
	}
}

unsigned pmem_fb_size = 	0x96000;
unsigned pmem_adsp_size =	0xAE4000;

static void __init msm7x2x_init(void)
{
	msm_clock_init(msm_clocks_7x27, msm_num_clocks_7x27);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART1_PHYS, INT_UART1,
			&msm_device_uart1.dev, 1);
#endif

	if (cpu_is_msm7x27())
		msm7x2x_clock_data.max_axi_khz = 200000;

	msm_acpu_clock_init(&msm7x2x_clock_data);

	msm_add_pmem_devices();
	msm_add_fb_device();
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (lge_get_uart_mode())
		platform_device_register(&msm_device_uart3);
#endif
	platform_add_devices(devices, ARRAY_SIZE(devices));
#ifdef CONFIG_ARCH_MSM7X27
	msm_add_kgsl_device();
#endif
	msm_add_usb_devices();

#ifdef CONFIG_MSM_CAMERA
	config_camera_off_gpios(); /* might not be necessary */
#endif
	msm_device_i2c_init();
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	if (cpu_is_msm7x27())
		msm_pm_set_platform_data(msm7x27_pm_data,
					ARRAY_SIZE(msm7x27_pm_data));
	else
		msm_pm_set_platform_data(msm7x25_pm_data,
					ARRAY_SIZE(msm7x25_pm_data));
	msm7x27_wlan_init();

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	lge_add_ramconsole_devices();
	lge_add_ers_devices();
	lge_add_panic_handler_devices();
#endif
	lge_add_camera_devices();
	lge_add_lcd_devices();
	lge_add_btpower_devices();
	lge_add_mmc_devices();
	lge_add_input_devices();
	lge_add_misc_devices();
	lge_add_pm_devices();

	lge_add_gpio_i2c_devices();
	
	create_testmode_status();  
	create_sw_version_status();  
	create_smpl_reset_status();    
	create_qpst_enable_status(); 
}

static void __init msm7x2x_map_io(void)
{
	msm_map_common_io();

	msm_msm7x2x_allocate_memory_regions();

	if (socinfo_init() < 0)
		BUG();

#ifdef CONFIG_CACHE_L2X0
	if ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) > 1)
		|| ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 1)
		&& (SOCINFO_VERSION_MINOR(socinfo_get_version()) >= 3)))
		l2x0_init(MSM_L2CC_BASE, 0x0006801B, 0xfe000000);
	else
		l2x0_init(MSM_L2CC_BASE, 0x00068012, 0xfe000000);
#endif
}

MACHINE_START(MSM7X27_LU3700, "LU3700 Global board (LGE LU3700)")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io			= msm7x2x_map_io,
	.init_irq		= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer			= &msm_timer,
MACHINE_END
