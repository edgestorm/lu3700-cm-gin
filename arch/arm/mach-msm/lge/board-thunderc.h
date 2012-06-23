#ifndef __ARCH_MSM_BOARD_THUNDERC_H
#define __ARCH_MSM_BOARD_THUNDERC_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <asm/setup.h>
#include "pm.h"

/* sdcard related macros */
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
#define GPIO_SD_DETECT_N    49
#define VREG_SD_LEVEL       3000

#define GPIO_SD_DATA_3      51
#define GPIO_SD_DATA_2      52
#define GPIO_SD_DATA_1      53
#define GPIO_SD_DATA_0      54
#define GPIO_SD_CMD         55
#define GPIO_SD_CLK         56
#endif

/* touch-screen macros */
#define TS_X_MIN			0
#define TS_X_MAX			320
#define TS_Y_MIN			0
#define TS_Y_MAX			480
#define TS_GPIO_I2C_SDA		91
#define TS_GPIO_I2C_SCL		90
#define TS_GPIO_IRQ			92
#define TS_I2C_SLAVE_ADDR	0x20

/* camera */
#define CAM_I2C_SLAVE_ADDR			0x1a
#define GPIO_CAM_RESET		 		0		/* GPIO_0 */
#define GPIO_CAM_PWDN		 		1		/* GPIO_1 */
#define GPIO_CAM_MCLK				15		/* GPIO_15 */

#define CAM_POWER_OFF				0
#define CAM_POWER_ON				1

//int aat2870_camera_power_ctrl(int on_off);
#define LDO_CAM_AVDD_NO		2	/* 2.7V */
#define LDO_CAM_DVDD_NO		3	/* 1.2V */
#define LDO_CAM_IOVDD_NO	4	/* 2.6V */

/* proximity sensor */
#define PROXI_GPIO_I2C_SCL	107
#define PROXI_GPIO_I2C_SDA 	108
#define PROXI_GPIO_DOUT		109
#define PROXI_I2C_ADDRESS	0x44 /*slave address 7bit*/
#define PROXI_LDO_NO_VCC	1

/* accelerometer */
#define ACCEL_GPIO_INT	 		39
#define ACCEL_GPIO_I2C_SCL  	2
#define ACCEL_GPIO_I2C_SDA  	3
#define ACCEL_I2C_ADDRESS		0x09 /*kr3dm slave address 7bit*/
#define ACCEL_I2C_ADDRESS_H		0x19 /*kr3dh slave address 7bit*/

/*Ecompass*/
#define ECOM_GPIO_I2C_SCL		107
#define ECOM_GPIO_I2C_SDA		108
#define ECOM_GPIO_RST
#define ECOM_GPIO_INT		31
#define ECOM_I2C_ADDRESS		0x0F /* slave address 7bit */

/* ear sense driver macros */
#define GPIO_EAR_SENSE		29
#define GPIO_HS_MIC_BIAS_EN	26

/* lcd & backlight */
#define GPIO_LCD_BL_EN		82
#define GPIO_BL_I2C_SCL		88
#define GPIO_BL_I2C_SDA		89
#define GPIO_LCD_VSYNC_O	97
#define GPIO_LCD_MAKER_LOW	101
#define GPIO_LCD_RESET_N	102

#define BL_POWER_SUSPEND	0
#define BL_POWER_RESUME		1

/* bluetooth gpio pin */
enum {
	BT_WAKE         = 42,
	BT_RFR          = 43,
	BT_CTS          = 44,
	BT_RX           = 45,
	BT_TX           = 46,
	BT_PCM_DOUT     = 68,
	BT_PCM_DIN      = 69,
	BT_PCM_SYNC     = 70,
	BT_PCM_CLK      = 71,
	BT_HOST_WAKE    = 83,
	BT_RESET_N			= 123,
};

/* interface variable */
extern struct platform_device msm_device_snd;
extern struct platform_device msm_device_adspdec;
extern struct i2c_board_info i2c_devices[1];

extern int camera_power_state;
extern int lcd_bl_power_state;

/* interface functions */
void config_camera_on_gpios(void);
void config_camera_off_gpios(void);
void camera_power_mutex_lock(void);
void camera_power_mutex_unlock(void);

struct device* thunderc_backlight_dev(void);
void thunderc_pwrsink_resume(void);
#endif
