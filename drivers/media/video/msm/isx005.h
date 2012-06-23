/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef ISX005_H
#define ISX005_H

#include <linux/types.h>
#include <mach/camera.h>

extern struct isx005_reg isx005_regs;

enum isx005_width {
	BYTE_LEN,
	WORD_LEN,
};

struct isx005_register_address_value_pair {
	uint16_t register_address;
	uint16_t register_value;
	enum isx005_width register_length;
};

struct isx005_reg {
	const struct isx005_register_address_value_pair *init_reg_settings_6ma;
	uint16_t init_reg_settings_6ma_size;
	const struct isx005_register_address_value_pair *init_reg_settings_4ma;
	uint16_t init_reg_settings_4ma_size;
#if !defined (CONFIG_MACH_MSM7X27_THUNDERG) && !defined (CONFIG_MACH_MSM7X27_SU370) && !defined (CONFIG_MACH_MSM7X27_KU3700) && \
!defined (CONFIG_MACH_MSM7X27_LU3700) && !defined (CONFIG_MACH_MSM7X27_SU310) && !defined (CONFIG_MACH_MSM7X27_LU3100)
	const struct isx005_register_address_value_pair *init_reg32_settings;
	uint16_t init_reg32_settings_size;
#endif
	const struct isx005_register_address_value_pair *tuning_reg_settings;
	uint16_t tuning_reg_settings_size;
	
	const struct isx005_register_address_value_pair *prev_reg_settings;
	uint16_t prev_reg_settings_size;
	const struct isx005_register_address_value_pair *video_reg_settings;
	uint16_t video_reg_settings_size;
	const struct isx005_register_address_value_pair *video_15fps_reg_settings;
	uint16_t video_15fps_reg_settings_size;
	const struct isx005_register_address_value_pair *snap_reg_settings;
	uint16_t snap_reg_settings_size;
	
	const struct isx005_register_address_value_pair *af_normal_reg_settings;
	uint16_t af_normal_reg_settings_size;
	const struct isx005_register_address_value_pair *af_macro_reg_settings;
	uint16_t af_macro_reg_settings_size;
	const struct isx005_register_address_value_pair *af_manual_reg_settings;
	uint16_t af_manual_reg_settings_size;
	
	const struct isx005_register_address_value_pair *af_start_low_light_reg_settings;
	uint16_t af_start_low_light_reg_settings_size;

  	const struct isx005_register_address_value_pair *af_start_high_light_reg_settings;
	uint16_t af_start_high_light_reg_settings_size;

	const struct isx005_register_address_value_pair 
		*scene_auto_reg_settings;
	uint16_t scene_auto_reg_settings_size;	
	const struct isx005_register_address_value_pair 
		*scene_portrait_reg_settings;
	uint16_t scene_portrait_reg_settings_size;
	const struct isx005_register_address_value_pair 
		*scene_landscape_reg_settings;
	uint16_t scene_landscape_reg_settings_size;
	const struct isx005_register_address_value_pair 
		*scene_sports_reg_settings;
	uint16_t scene_sports_reg_settings_size;
	const struct isx005_register_address_value_pair 
		*scene_sunset_reg_settings;
	uint16_t scene_sunset_reg_settings_size;
	const struct isx005_register_address_value_pair 
		*scene_night_reg_settings;
	uint16_t scene_night_reg_settings_size;
	
};

/* this value is defined in Android native camera */
enum isx005_focus_mode {
	FOCUS_NORMAL,
	FOCUS_MACRO,
	FOCUS_AUTO,
	FOCUS_MANUAL,
};

/* this value is defined in Android native camera */
enum isx005_wb_type {
	CAMERA_WB_MIN_MINUS_1,
	CAMERA_WB_AUTO = 1,  /* This list must match aeecamera.h */
	CAMERA_WB_CUSTOM,
	CAMERA_WB_INCANDESCENT,
	CAMERA_WB_FLUORESCENT,
	CAMERA_WB_DAYLIGHT,
	CAMERA_WB_CLOUDY_DAYLIGHT,
	CAMERA_WB_TWILIGHT,
	CAMERA_WB_SHADE,
	CAMERA_WB_MAX_PLUS_1
};

enum isx005_antibanding_type {
	CAMERA_ANTIBANDING_OFF,
	CAMERA_ANTIBANDING_60HZ,
	CAMERA_ANTIBANDING_50HZ,
	CAMERA_ANTIBANDING_AUTO,
	CAMERA_MAX_ANTIBANDING,
};

/* Enum Type for different ISO Mode supported */
enum isx005_iso_value {
	CAMERA_ISO_AUTO = 0,
	CAMERA_ISO_DEBLUR,
	CAMERA_ISO_100,
	CAMERA_ISO_200,
	CAMERA_ISO_400,
	CAMERA_ISO_800,
	CAMERA_ISO_MAX
};

/* Enum type for scene mode */
enum {
	CAMERA_SCENE_AUTO = 1,
	CAMERA_SCENE_PORTRAIT,
	CAMERA_SCENE_LANDSCAPE,
	CAMERA_SCENE_SPORTS,
	CAMERA_SCENE_NIGHT,
	CAMERA_SCENE_SUNSET,
};

enum isx005_preview_mode_value{
	CAMERA_PREVIEW_MODE_FPS = 0,
	CAMERA_CAMCORD_MODE_FPS,
	CAMERA_CAMCORD_MODE_FPS_FOR_DELIVERING,
	CAMERA_MODE_DEFAULT,	
	CAMERA_PREVIEW_MODE_MAX
};

enum isx005_pad_strengh_value{
	CAMERA_PCLK_PAD_STRENGTH_6MA = 0,
	CAMERA_PCLK_PAD_STRENGTH_4MA,
	CAMERA_PCLK_PAD_STRENGTH_MAX
};

#endif /* ISX005_H */
