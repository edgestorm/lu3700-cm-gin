
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <linux/kthread.h>

#include "isx006.h"
#include "isx006_reg.h"
#include "register_common_init.h"


typedef enum {
	CAMERA_PREVIEW_MODE_FPS = 0,
	CAMERA_CAMCORD_MODE_FPS,
	CAMERA_CAMCORD_MODE_FPS_FOR_DELIVERING,
	CAMERA_MODE_DEFAULT,
	CAMERA_PREVIEW_MODE_MAX
}camera_preview_mode_t;

#define ISX006_INTERVAL_T2		8	/* 8ms */
#define ISX006_INTERVAL_T3		1	/* 0.5ms */
#define ISX006_INTERVAL_T4		2	/* 15ms */
#define ISX006_INTERVAL_T5		25	/* 200ms */

/*
* AF Total steps parameters
*/
#define ISX006_TOTAL_STEPS_NEAR_TO_FAR	30

/*  ISX006 Registers  */
#define REG_ISX006_INTSTS_ID			0x00F8	/* Interrupt status */
#define REG_ISX006_INTCLR_ID			0x00FC	/* Interrupt clear */

#define ISX006_OM_CHANGED				0x0001	/* Operating mode */
#define ISX006_CM_CHANGED				0x0002	/* Camera mode */

#define ISX006_MAX_ZOOM_STEP    16
/* It is distinguish normal from macro focus */
static int prev_af_mode;
/* It is distinguish scene mode */
static int prev_scene_mode;
static int32_t prevew_mode_status;
static int32_t prev_zoom_step;

struct isx006_work {
	struct work_struct work;
};
static struct isx006_work *isx006_sensorw;

static struct i2c_client *isx006_client;

struct isx006_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};

static struct isx006_ctrl *isx006_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(isx006_wait_queue);

DEFINE_MUTEX(isx006_mutex);

struct platform_device *isx006_pdev;

static int always_on = 0;

static int is_from_capture;

static int zoom_ratio = 0x105;

static int zoom_offset_x = 0;

static int zoom_offset_y = 0;

static int zoom_table[ISX006_MAX_ZOOM_STEP] = {0x0100, 0x0104, 0x0108, 0x010C, 0x0111, 0x0115, 0x0119, 0x011D, 0x0122, 0x0126, \
                                               0x012A, 0x012E, 0x0133, 0x0137, 0x013B, 0x0140};

static int32_t isx006_i2c_txdata(unsigned short saddr,
	unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	if (i2c_transfer(isx006_client->adapter, msg, 1) < 0) {
		CDBG("isx006_i2c_txdata failed\n");
		return -EIO;
	}

	return 0;
}

static int32_t isx006_i2c_write(unsigned short saddr, unsigned short waddr, unsigned int wdata, enum isx006_width width)
{
	int32_t rc = -EIO;
  int int_buf[2];
	unsigned char *pbuf = ((unsigned char*)int_buf) + 2;  // to align multipul 4

  pbuf[0] = (waddr & 0xFF00) >> 8;
  pbuf[1] = (waddr & 0x00FF);
  *((unsigned int*)&pbuf[2]) = wdata;
  
	switch (width) {
		case BYTE_LEN:	
		case WORD_LEN:
    case QUAD_LEN:
			rc = isx006_i2c_txdata(saddr, pbuf, width + 2);
			break;

		default:
			break;
	}

	if (rc < 0)
		printk(KERN_ERR "i2c_write failed, addr = 0x%x, val = 0x%x!\n", waddr, wdata);

	return rc;
}

static int32_t isx006_i2c_write_table(
	struct isx006_register_address_value_pair const *reg_conf_tbl,
	int num_of_items_in_table)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num_of_items_in_table; ++i) {
		rc = isx006_i2c_write(isx006_client->addr,
			reg_conf_tbl->register_address, reg_conf_tbl->register_value,
			reg_conf_tbl->register_length);
		if (rc < 0)
			break;

		reg_conf_tbl++;
	}

	return rc;
}

static int isx006_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr   = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr   = saddr,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = rxdata,
		},
	};

	if (i2c_transfer(isx006_client->adapter, msgs, 2) < 0) {
		printk(KERN_ERR "isx006_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t isx006_i2c_read(unsigned short   saddr,
	unsigned short raddr, unsigned int *rdata, enum isx006_width width)
{
	int32_t rc = 0;
  int int_buf;
	unsigned char *pbuf = (unsigned char*)&int_buf;

	if (!rdata)
		return -EIO;

	//memset(buf, 0, sizeof(buf));

  pbuf[0] = (raddr & 0xFF00) >> 8;
  pbuf[1] = (raddr & 0x00FF);

  rc = isx006_i2c_rxdata(saddr, pbuf, (int)width);
  if (rc < 0)
    return rc;

	switch (width) {
		case BYTE_LEN:
			*rdata = pbuf[0];
			break;
			
		case WORD_LEN:
			*rdata = *((unsigned short *)pbuf);
			break;

		case QUAD_LEN:
			*rdata = *((unsigned int *)pbuf);
			break;

		default:
			break;
	}

	if (rc < 0)
		printk(KERN_ERR "isx006_i2c_read failed!\n");

	return rc;
}

static int isx006_reg_init(void)
{
	int rc = 0;
	int i;

	/* Configure sensor for Initial setting (PLL, Clock, etc) */
	for (i = 0; i < isx006_regs.init_reg_settings_size; ++i) {
		rc = isx006_i2c_write(isx006_client->addr,
			isx006_regs.init_reg_settings[i].register_address,
			isx006_regs.init_reg_settings[i].register_value,
			isx006_regs.init_reg_settings[i].register_length);

		if (rc < 0)
			return rc;
	}
	
	return rc;
}

static int isx006_reg_preview_addition(void)
{
	int rc = 0;
	int i, CHIPID_L, reg_len;
  const struct isx006_register_address_value_pair *preg_settings;

  rc = isx006_i2c_read(isx006_client->addr,
			0x00F8, &CHIPID_L, BYTE_LEN);

  if ((CHIPID_L & 0xF) == 0x1)
  {
    preg_settings = isx006_regs.preview_addition_AP003_reg_settings;
    reg_len = isx006_regs.preview_addition_AP003_reg_settings_size;
    printk("OTP_CHIPID_L == 0x01 !\n");
  }
  else
  {
    preg_settings = isx006_regs.preview_addition_AP001_reg_settings;
    reg_len = isx006_regs.preview_addition_AP001_reg_settings_size;
    printk("OTP_CHIPID_L != 0x01 !\n");
  }
  
	for (i = 0; i < reg_len; ++i) {
		rc = isx006_i2c_write(isx006_client->addr,
			preg_settings[i].register_address,
			preg_settings[i].register_value,
			preg_settings[i].register_length);

		if (rc < 0)
			return rc;
	}

	return rc;
}


static int isx006_reg_tuning(void)
{
	int rc = 0;
	int i;
	
	/* Configure sensor for various tuning */
	for (i = 0; i < isx006_regs.tuning_reg_settings_size; ++i) {
		rc = isx006_i2c_write(isx006_client->addr,
			isx006_regs.tuning_reg_settings[i].register_address,
			isx006_regs.tuning_reg_settings[i].register_value,
			isx006_regs.tuning_reg_settings[i].register_length);

		if (rc < 0)
			return rc;
	}

	return rc;
}

static int isx006_reg_preview(void)
{
	int rc = 0;
	int i, CM_CHANGED_STS;

    
#if 0
  // Zoom 1.00
  rc = isx006_i2c_write(isx006_client->addr, 0x0032, 0x0100, WORD_LEN);  // EZOOM_MAG
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x0034, 0x0000, WORD_LEN);  // OFFSET_X
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x0036, 0x0000, WORD_LEN);  // OFFSET_Y
  if (rc < 0)
    return rc;

#endif


	/* Configure sensor for Preview mode */
	for (i = 0; i < isx006_regs.prev_reg_settings_size; ++i) {
		rc = isx006_i2c_write(isx006_client->addr,
		  isx006_regs.prev_reg_settings[i].register_address,
		  isx006_regs.prev_reg_settings[i].register_value,
		  isx006_regs.prev_reg_settings[i].register_length);

		if (rc < 0)
			return rc;
	}

  if (is_from_capture)
  {
    do
    {
      msleep(10);
      isx006_i2c_read(isx006_client->addr, 0x00F8, &CM_CHANGED_STS, 0x01);
    } while( !(CM_CHANGED_STS & 0x02) );
    is_from_capture = 0;
  }

	return rc;
}

static int isx006_reg_snapshot(void)
{
	int rc = 0;
	int i;

	/* Configure sensor for Snapshot mode */
	for (i = 0; i < isx006_regs.snap_reg_settings_size; ++i) {
		rc = isx006_i2c_write(isx006_client->addr,
			isx006_regs.snap_reg_settings[i].register_address,
			isx006_regs.snap_reg_settings[i].register_value,
			isx006_regs.snap_reg_settings[i].register_length);

		if (rc < 0)
			return rc;
	}

	return rc;
}

static int isx006_reg_raw_snapshot(void)
{
	int rc = 0;
	int i;

	/* Configure sensor for Raw-Snapshot mode */
	for (i = 0; i < isx006_regs.snap_reg_settings_size; ++i) {
		rc = isx006_i2c_write(isx006_client->addr,
			isx006_regs.snap_reg_settings[i].register_address,
			isx006_regs.snap_reg_settings[i].register_value,
			isx006_regs.snap_reg_settings[i].register_length);

		if (rc < 0)
			return rc;
	}

	return rc;
}

static int isx006_snapshot_config(int width, int height)
{
  int rc, CM_CHANGED_STS;
  
  rc = isx006_i2c_write(isx006_client->addr, 0x0024, width, WORD_LEN);
  if (rc < 0)
    return rc;
  
  rc = isx006_i2c_write(isx006_client->addr, 0x002A, height, WORD_LEN);
  if (rc < 0)
    return rc;

  rc = isx006_i2c_write(isx006_client->addr, 0x0224, 0x00, BYTE_LEN);  //SIZE_HOLD_EN
  if (rc < 0)
    return rc;

  rc = isx006_i2c_write(isx006_client->addr, 0x0014, 0x00, BYTE_LEN);  //CAPNUM
  if (rc < 0)
    return rc;

  rc = isx006_i2c_write(isx006_client->addr, 0x001D, 0x00, BYTE_LEN);  //OUTFMT_CAP     YUV MODE
  if (rc < 0)
    return rc;
  
#if 1
  // Zoom 1.02
  rc = isx006_i2c_write(isx006_client->addr, 0x0032, zoom_table[prev_zoom_step] + 0x0005, WORD_LEN);  // EZOOM_MAG
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x0034, zoom_offset_x, WORD_LEN);  // OFFSET_X
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x0036, zoom_offset_y, WORD_LEN);  // OFFSET_Y
  if (rc < 0)
    return rc;
#endif
  
  rc = isx006_i2c_write(isx006_client->addr, 0x0011, 0x02, BYTE_LEN); // CAPTURE COMMAND
  if (rc < 0)
    return rc;  

	rc = isx006_i2c_write(isx006_client->addr, 0x00FC, 0x1F, BYTE_LEN); // CM_CHANGED, Interrupt Clear
	if (rc < 0)
		return rc;	

  do
  {
    msleep(10);
    isx006_i2c_read(isx006_client->addr, 0x00F8, &CM_CHANGED_STS, 0x01);
  } while( !(CM_CHANGED_STS & 0x02) );
  

  is_from_capture = 1;

  return rc;
}

static int isx006_raw_snapshot_config(int width, int height)
{
  int rc, CM_CHANGED_STS, JPEG_UPDATE_STS, JPG_STS;
  
  rc = isx006_i2c_write(isx006_client->addr, 0x0024, width, WORD_LEN);
  if (rc < 0)
    return rc;
  
  rc = isx006_i2c_write(isx006_client->addr, 0x002A, height, WORD_LEN);
  if (rc < 0)
    return rc;

#if 0 // need ?
	rc = isx006_i2c_write(isx006_client->addr, 0x02D3, 0x01, BYTE_LEN);  //VIFADRDUMP_MODE(YUV address Table Output On)
	if (rc < 0)
		return rc;
#endif

	rc = isx006_i2c_write(isx006_client->addr, 0x0224, 0x01, BYTE_LEN);  //SIZE_HOLD_EN
	if (rc < 0)
		return rc;

	rc = isx006_i2c_write(isx006_client->addr, 0x0014, 0x00, BYTE_LEN);  //CAPNUM
	if (rc < 0)
		return rc;

	rc = isx006_i2c_write(isx006_client->addr, 0x001D, 0x1B, BYTE_LEN);  //OUTFMT_CAP     INTERLEAVE MODE SPOOF FREE-RUNNING MODE
	if (rc < 0)
		return rc;
  
#if 1
  // Zoom 1.02
  rc = isx006_i2c_write(isx006_client->addr, 0x0032, zoom_table[prev_zoom_step] + 0x0005, WORD_LEN);  // EZOOM_MAG
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x0034, zoom_offset_x, WORD_LEN);  // OFFSET_X
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x0036, zoom_offset_y, WORD_LEN);  // OFFSET_Y
  if (rc < 0)
    return rc;
#endif

	rc = isx006_i2c_write(isx006_client->addr, 0x0011, 0x02, BYTE_LEN); // CAPTURE COMMAND
	if (rc < 0)
		return rc;	

	rc = isx006_i2c_write(isx006_client->addr, 0x00FC, 0x1F, BYTE_LEN); // CM_CHANGED, Interrupt Clear
	if (rc < 0)
		return rc;	

  do
  {
    msleep(10);
    isx006_i2c_read(isx006_client->addr, 0x00F8, &CM_CHANGED_STS, 0x01);
  } while( !(CM_CHANGED_STS & 0x02) );

  rc = isx006_i2c_write(isx006_client->addr, 0x00FC, 0x04, BYTE_LEN); // CM_CHANGED, Interrupt Clear
  if (rc < 0)
    return rc;  

  do
  {
    do
    {
      msleep(10);
      isx006_i2c_read(isx006_client->addr, 0x00F8, &JPEG_UPDATE_STS, 0x01);
    } while(!(JPEG_UPDATE_STS & 0x0004));

    isx006_i2c_write(isx006_client->addr, 0x00FC, 0x04, BYTE_LEN); // CM_CHANGED, Interrupt Clear
    isx006_i2c_read(isx006_client->addr, 0x0200, &JPG_STS, 0x01);
  } while(JPG_STS != 0x00);

  is_from_capture = 1;

  printk("zoom ration = %x\n", zoom_ratio);

  return rc;
}


static int isx006_set_sensor_mode(int mode, int width, int height)
{
	int rc = 0;

  switch (prev_scene_mode) {
    case CAMERA_SCENE_AUTO:
      if (mode == SENSOR_SNAPSHOT_MODE || mode == SENSOR_RAW_SNAPSHOT_MODE)
      {
        rc = isx006_i2c_write(isx006_client->addr, 0x401c, 0x0364, WORD_LEN);
        if (rc < 0)
          return rc;
        rc = isx006_i2c_write(isx006_client->addr, 0x4020, 0x03ca, WORD_LEN);
        if (rc < 0)
          return rc;
      }
      else if (mode == SENSOR_PREVIEW_MODE)
      {
        rc = isx006_i2c_write(isx006_client->addr, 0x401c, 0x0331, WORD_LEN);
        if (rc < 0)
          return rc;
        rc = isx006_i2c_write(isx006_client->addr, 0x4020, 0x0000, WORD_LEN);
        if (rc < 0)
          return rc;
      }
      break;

    case CAMERA_SCENE_SPORTS:
      if (mode == SENSOR_SNAPSHOT_MODE || mode == SENSOR_RAW_SNAPSHOT_MODE)
      {
        rc = isx006_i2c_write(isx006_client->addr, 0x401c, 0x032e, WORD_LEN);
        if (rc < 0)
          return rc;
        rc = isx006_i2c_write(isx006_client->addr, 0x4020, 0x01cc, WORD_LEN);
        if (rc < 0)
          return rc;
      }
      else if (mode == SENSOR_PREVIEW_MODE)
      {
        rc = isx006_i2c_write(isx006_client->addr, 0x401c, 0x00fc, WORD_LEN);
        if (rc < 0)
          return rc;
        rc = isx006_i2c_write(isx006_client->addr, 0x4020, 0x0000, WORD_LEN);
        if (rc < 0)
          return rc;
      }
      break;
  }

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = isx006_reg_preview();
		if (rc < 0)
			printk(KERN_ERR "[ERROR]%s:Sensor Preview Mode Fail\n", __func__);
		break;

	case SENSOR_SNAPSHOT_MODE:
		rc = isx006_snapshot_config(width, height);
		if (rc < 0)
			printk(KERN_ERR "[ERROR]%s:Sensor Snapshot Mode Fail\n", __func__);
		break;
    
  case SENSOR_RAW_SNAPSHOT_MODE:
		rc = isx006_raw_snapshot_config(width, height);
		if (rc < 0)
			printk(KERN_ERR "[ERROR]%s:Sensor Raw Snapshot Mode Fail\n", __func__);
		break;

	default:
		return -EINVAL;
	}
  
	printk("%s : %d, rc = %d\n", __func__, mode, rc);

	//msleep(20);

	return rc;
}

static int isx006_cancel_focus(int mode)
{
	int rc;
	int lense_po_back = 0;

  printk("%s\n",__func__);
    
	switch(mode){
	case 0:
    	rc = isx006_i2c_read(isx006_client->addr, 0x4800, &lense_po_back, WORD_LEN);
      if (rc < 0)
        return rc;

		//lense_po_back = 0x0032;
		break;
	
	case 1:
    rc = isx006_i2c_read(isx006_client->addr, 0x4802, &lense_po_back, WORD_LEN);
    if (rc < 0)
      return rc;

		//lense_po_back = 0x0304;
		break;
	}

	rc = isx006_i2c_write(isx006_client->addr,
			0x002E, 0x02, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx006_i2c_write(isx006_client->addr,
			0x0012, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;

	rc = isx006_i2c_write(isx006_client->addr,
			0x4852, lense_po_back, WORD_LEN);
	if (rc < 0)
		return rc;
  
	rc = isx006_i2c_write(isx006_client->addr,
			0x4850, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;

	rc = isx006_i2c_write(isx006_client->addr,
			0x00FC, 0x1F, BYTE_LEN);
	if (rc < 0)
		return rc;

	return rc;
}

static int isx006_check_af_lock(void)
{
	int rc;
	int i;
	unsigned int af_lock = 0;
	
  printk("%s\n",__func__);
  
	for (i = 0; i < 10; ++i) {
		/*INT state read -*/
		rc = isx006_i2c_read(isx006_client->addr,
			0x00F8, &af_lock, BYTE_LEN);
		
		if (rc < 0) {
			CDBG("isx006: reading af_lock fail\n");
			return rc;
		}

		/* af interruption lock state read compelete */
		if((af_lock & 0x10) == 0x10)
			break;

		msleep(10);
	}

	for (i = 0; i < 10; ++i) {
    /* INT clear */
    rc = isx006_i2c_write(isx006_client->addr,
      0x00FC, 0x10, BYTE_LEN);
    if (rc < 0)
      return rc;
    
		msleep(10);

		/*INT state read to confirm INT release state*/
		rc = isx006_i2c_read(isx006_client->addr,
				0x00F8, &af_lock, BYTE_LEN);
		
		if (rc < 0) {
			CDBG("isx006: reading af_lock fail\n");
			return rc;
		}

		if ((af_lock & 0x10) == 0x00) {
			CDBG("af_lock is released\n");
			break;
		}
	}

	return rc;
}

static int isx006_check_focus(int *lock)
{
	int rc;
	unsigned int af_status;
	unsigned int af_result;

	printk("isx006_check_focus\n");

	/*af status check  0:load, 1: init,  8: af_lock */
	rc = isx006_i2c_read(isx006_client->addr,
		0x6D76, &af_status, BYTE_LEN);

	if (af_status != 0x8)
		return -ETIME;

	isx006_check_af_lock();
	
	/* af result read  success/ fail*/
	rc = isx006_i2c_read(isx006_client->addr, 0x6D77, &af_result, BYTE_LEN);
	if (rc < 0) {
		printk("[isx006.c]%s: fai; in reading af_result\n",__func__);
		return rc;
	}

	/* single autofocus off */
	rc = isx006_i2c_write(isx006_client->addr, 0x002E, 0x03, BYTE_LEN);
	if (rc < 0)
		return rc;
		
	/* single autofocus refresh*/
	rc = isx006_i2c_write(isx006_client->addr, 0x0012, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;

  /* change normal fps */
  rc = isx006_i2c_write(isx006_client->addr, 0x4014, 0x0F07, WORD_LEN); // AGC_SCL_L : Analog gain
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x0103, 0x01, BYTE_LEN);  // AE_SN1 : Normal AE ==> 고정 fps로 된다.
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x0108, 0x01, BYTE_LEN);  // AE_SN1 : Normal AE ==> 고정 fps로 된다.
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x010D, 0x01, BYTE_LEN);  // AE_SN1 : Normal AE ==> 고정 fps로 된다.
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x0112, 0x01, BYTE_LEN);  // AE_SN1 : Normal AE ==> 고정 fps로 된다.
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x0117, 0x01, BYTE_LEN);  // AE_SN1 : Normal AE ==> 고정 fps로 된다.
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x011C, 0x01, BYTE_LEN);  // AE_SN1 : Normal AE ==> 고정 fps로 된다.
  if (rc < 0)
    return rc;
  

	if (af_result == 1) {
		*lock = CFG_AF_LOCKED;  // success
		return rc;
	} else {
		*lock = CFG_AF_UNLOCKED; //0: focus fail or 2: during focus
		return rc;
	}

	return -ETIME;
}

static int isx006_set_af_start(int mode)
{
	int rc = 0, af_lock = 0, i;

  printk("%s\n",__func__);

	for (i = 0; i < 10; ++i) {
    /* INT clear */
    rc = isx006_i2c_write(isx006_client->addr,
      0x00FC, 0x10, BYTE_LEN);
    if (rc < 0)
      return rc;
    
		msleep(10);

		/*INT state read to confirm INT release state*/
		rc = isx006_i2c_read(isx006_client->addr,
				0x00F8, &af_lock, BYTE_LEN);
		
		if (rc < 0) {
			CDBG("isx006: reading af_lock fail\n");
			return rc;
		}

		if ((af_lock & 0x10) == 0x00) {
			CDBG("af_lock is released\n");
			break;
		}
	}

  /* change max fps */
  if (mode != FOCUS_MANUAL)
  {
    //rc = isx006_i2c_write(isx006_client->addr, 0x4014, 0x15BA, WORD_LEN); // AGC_SCL_L : Analog gain
    rc = isx006_i2c_write(isx006_client->addr, 0x4014, 0x0F07, WORD_LEN); // AGC_SCL_L : Analog gain
    if (rc < 0)
      return rc;
    rc = isx006_i2c_write(isx006_client->addr, 0x0103, 0x00, BYTE_LEN);  // AE_SN1 : Normal AE ==> 고정 fps로 된다.
    if (rc < 0)
      return rc;
    rc = isx006_i2c_write(isx006_client->addr, 0x0108, 0x00, BYTE_LEN);  // AE_SN1 : Normal AE ==> 고정 fps로 된다.
    if (rc < 0)
      return rc;
    rc = isx006_i2c_write(isx006_client->addr, 0x010D, 0x00, BYTE_LEN);  // AE_SN1 : Normal AE ==> 고정 fps로 된다.
    if (rc < 0)
      return rc;
    rc = isx006_i2c_write(isx006_client->addr, 0x0112, 0x00, BYTE_LEN);  // AE_SN1 : Normal AE ==> 고정 fps로 된다.
    if (rc < 0)
      return rc;
    rc = isx006_i2c_write(isx006_client->addr, 0x0117, 0x00, BYTE_LEN);  // AE_SN1 : Normal AE ==> 고정 fps로 된다.
    if (rc < 0)
      return rc;
    rc = isx006_i2c_write(isx006_client->addr, 0x011C, 0x00, BYTE_LEN);  // AE_SN1 : Normal AE ==> 고정 fps로 된다.
    if (rc < 0)
      return rc;
  }
  
	if(prev_af_mode == mode) {
		rc = isx006_i2c_write_table(isx006_regs.af_start_reg_settings,
			isx006_regs.af_start_reg_settings_size);
	} else {
		switch (mode) {
			case FOCUS_NORMAL:
				rc = isx006_i2c_write_table(isx006_regs.af_normal_reg_settings,
					isx006_regs.af_normal_reg_settings_size);
				break;

			case FOCUS_MACRO:
				rc = isx006_i2c_write_table(isx006_regs.af_macro_reg_settings,
					isx006_regs.af_macro_reg_settings_size);
				break;

			case FOCUS_AUTO:	
				rc = isx006_i2c_write_table(isx006_regs.af_normal_reg_settings,
					isx006_regs.af_normal_reg_settings_size);
				break;

			case FOCUS_MANUAL:	
				rc = isx006_i2c_write_table(isx006_regs.af_manual_reg_settings,
					isx006_regs.af_manual_reg_settings_size);
				break;

			default:
				printk(KERN_ERR "[ERROR]%s: invalid af mode\n", __func__);
				break;
		}
		/*af start*/
		rc = isx006_i2c_write_table(isx006_regs.af_start_reg_settings,
			isx006_regs.af_start_reg_settings_size);
	}	

	prev_af_mode = mode;
		
	return rc;
}

static int isx006_move_focus(int32_t steps)
{
	int32_t rc;
	unsigned int cm_changed_sts, cm_changed_clr, af_pos, manual_pos;
	int i;
  static uint32_t manual_focus_value[] = {514, 468, 408, 375, 342, 309, 276, 243, 210, 150, 130};
  
	rc = isx006_i2c_write_table(isx006_regs.af_manual_reg_settings,
			isx006_regs.af_manual_reg_settings_size);

  prev_af_mode = FOCUS_MANUAL;

  printk(KERN_ERR "isx006_move_focus : step=%d\n", steps);
    
	if (rc < 0) {
		printk(KERN_ERR "[ERROR]%s:fail in writing for move focus\n",
			__func__);
		return rc;
	}

	/* check cm_changed_sts */
	for(i = 0; i < 24; ++i) {
		rc = isx006_i2c_read(isx006_client->addr,
				0x00F8, &cm_changed_sts, BYTE_LEN);
		if (rc < 0){
			printk(KERN_ERR "[ERROR]%s; fail in reading cm_changed_sts\n",
				__func__);
			return rc;
		}

		if((cm_changed_sts & 0x02) == 0x02)
			break;

		msleep(10);
	}

	/* check cm_changed_clr */
	for(i = 0; i < 24; ++i) {
    /* clear the interrupt register */
    rc = isx006_i2c_write(isx006_client->addr, 0x00FC, 0x02, BYTE_LEN);
    if (rc < 0)
      return rc;

		rc = isx006_i2c_read(isx006_client->addr,
			0x00F8, &cm_changed_clr, BYTE_LEN);
		if (rc < 0) {
			printk(KERN_ERR "[ERROR]%s:fail in reading cm_changed_clr\n",
				__func__);
			return rc;
		}

		if((cm_changed_clr & 0x00) == 0x00)
			break;

		msleep(10);
	}

#if 0
	if (steps <= 10)
		manual_pos = 50 + (50 * steps);// cpu_to_be16(50 + (50 * steps));
	else
		manual_pos = 50;
#endif

	if (steps <= 10)
	    manual_pos = manual_focus_value[steps];
	else
		manual_pos = 130;
  
	rc = isx006_i2c_write(isx006_client->addr, 0x4852, manual_pos, WORD_LEN);
	if (rc < 0)
		return rc;

	rc = isx006_i2c_write(isx006_client->addr, 0x4850, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx006_i2c_write(isx006_client->addr, 0x0015, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;

	isx006_check_af_lock();
	
	/* check lens position */
	for(i = 0; i < 24; ++i) {
		rc = isx006_i2c_read(isx006_client->addr, 0x6D7A, &af_pos, WORD_LEN);
		if (rc < 0)
			printk(KERN_ERR "[ERROR]%s:fail in reading af_lenspos\n",
				__func__);
	
		if(af_pos == manual_pos)
			break;
		
		msleep(10);
	}

	return rc;
}

static int isx006_set_default_focus()
{
	int rc;

  printk(KERN_ERR "isx006_set_default_focus ..... \n");

	rc = isx006_cancel_focus(prev_af_mode);
	if (rc < 0) {
		printk(KERN_ERR "[ERROR]%s:fail in cancel_focus\n", __func__);
		return rc;
	}

	rc = isx006_i2c_write_table(isx006_regs.af_normal_reg_settings,
		isx006_regs.af_normal_reg_settings_size);

	prev_af_mode = FOCUS_AUTO;

	if (rc < 0) {
		printk(KERN_ERR "[ERROR]%s:fail in writing for focus\n", __func__);
		return rc;
	}

	msleep(60);
	
	isx006_check_focus(&rc);

	return rc;
}


static int isx006_set_effect(int effect)
{
	int rc = 0;

  printk("%s\n",__func__);
  
	switch (effect) {
	case CAMERA_EFFECT_OFF:
		rc = isx006_i2c_write(isx006_client->addr, 0x005F, 0x00, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx006_i2c_write(isx006_client->addr, 0x038A, 0x1169, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	case CAMERA_EFFECT_MONO:
		rc = isx006_i2c_write(isx006_client->addr, 0x005F, 0x04, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx006_i2c_write(isx006_client->addr, 0x038A, 0x1169, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	case CAMERA_EFFECT_NEGATIVE:
		rc = isx006_i2c_write(isx006_client->addr, 0x005F, 0x02, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx006_i2c_write(isx006_client->addr, 0x038A, 0x1169, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	case CAMERA_EFFECT_SOLARIZE:
		rc = isx006_i2c_write(isx006_client->addr, 0x005F, 0x01, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx006_i2c_write(isx006_client->addr, 0x038A, 0x1169, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	case CAMERA_EFFECT_SEPIA:
		rc = isx006_i2c_write(isx006_client->addr, 0x005F, 0x03, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx006_i2c_write(isx006_client->addr, 0x038A, 0x1169, WORD_LEN);
		if (rc < 0)
			return rc;
		
		break;

	/* This effect is not supported in ISX006 */
	case CAMERA_EFFECT_POSTERIZE:
		rc = isx006_i2c_write(isx006_client->addr, 0x005F, 0x00, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx006_i2c_write(isx006_client->addr, 0x038A, 0x1169, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	/* This effect is not supported in ISX006 */
	case CAMERA_EFFECT_AQUA:
		rc = isx006_i2c_write(isx006_client->addr, 0x005F, 0x00, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx006_i2c_write(isx006_client->addr, 0x038A, 0x1169, WORD_LEN);
		if (rc < 0)
			return rc;

		break;

	case CAMERA_EFFECT_NEGATIVE_SEPIA:
		rc = isx006_i2c_write(isx006_client->addr, 0x005F, 0x02, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx006_i2c_write(isx006_client->addr, 0x038A, 0x1169, WORD_LEN);
		if (rc < 0)
			return rc;
		
		break;

	case CAMERA_EFFECT_BLUE:
		rc = isx006_i2c_write(isx006_client->addr, 0x005F, 0x03, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx006_i2c_write(isx006_client->addr, 0x038A, 0x1169, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

	case CAMERA_EFFECT_PASTEL:
		rc = isx006_i2c_write(isx006_client->addr, 0x005F, 0x05, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx006_i2c_write(isx006_client->addr, 0x038A, 0x1169, WORD_LEN);
		if (rc < 0)
			return rc;
		break;		

	default:
		return -EINVAL;
	}

	CDBG("Effect : %d, rc = %d\n", effect, rc);

	return rc;
}

static int isx006_set_wb(int mode)
{
	int rc;

  printk(KERN_INFO"isx006_set_wb : %d \n", mode);

	switch (mode) {
		case CAMERA_WB_AUTO:
			rc = isx006_i2c_write(isx006_client->addr, 0x4453, 0x7B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx006_i2c_write(isx006_client->addr, 0x0102, 0x20, BYTE_LEN);
			if (rc < 0)
				return rc;
			
			break;

		case CAMERA_WB_INCANDESCENT:
			rc = isx006_i2c_write(isx006_client->addr, 0x4453, 0x7B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx006_i2c_write(isx006_client->addr, 0x0102, 0x28, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_WB_FLUORESCENT:
			rc = isx006_i2c_write(isx006_client->addr, 0x4453, 0x7B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx006_i2c_write(isx006_client->addr, 0x0102, 0x27, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;
			
		case CAMERA_WB_DAYLIGHT:
			rc = isx006_i2c_write(isx006_client->addr, 0x4453, 0x7B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx006_i2c_write(isx006_client->addr, 0x0102, 0x24, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_WB_CLOUDY_DAYLIGHT:
			rc = isx006_i2c_write(isx006_client->addr, 0x4453, 0x3B, BYTE_LEN);
			if (rc < 0)
				return rc;

			rc = isx006_i2c_write(isx006_client->addr, 0x0102, 0x26, BYTE_LEN);
			if (rc < 0)
				return rc;
			
			break;

		case CAMERA_WB_TWILIGHT:	/* Do not support */
    case CAMERA_WB_CUSTOM:  /* Do not support */
		case CAMERA_WB_SHADE:		/* Do not support */
		default:
			return -EINVAL;
	}
	return rc;
}

static int isx006_set_antibanding(int mode)
{
	int rc;

  printk(KERN_ERR "isx006_set_antibanding : %d\n", mode);

	switch (mode) {
		case CAMERA_ANTIBANDING_OFF:
			rc = isx006_i2c_write(isx006_client->addr, 0x4001, 0x00, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_ANTIBANDING_60HZ:
			rc = isx006_i2c_write(isx006_client->addr, 0x4001, 0x04, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_ANTIBANDING_50HZ:
			rc = isx006_i2c_write(isx006_client->addr, 0x4001, 0x03, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_ANTIBANDING_AUTO:
			rc = isx006_i2c_write(isx006_client->addr, 0x4001, 0x00, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		case CAMERA_MAX_ANTIBANDING:
			rc = isx006_i2c_write(isx006_client->addr, 0x4001, 0x04, BYTE_LEN);
			if (rc < 0)
				return rc;

			break;

		default:
			return -EINVAL;
	}

	return rc;	
}

static int isx006_set_iso(int iso)
{
	int32_t rc;

  printk("%s\n",__func__);

	switch (iso) {
		case CAMERA_ISO_AUTO:
			rc = isx006_i2c_write(isx006_client->addr,
					0x01E5, 0x00, BYTE_LEN);
			break;

		case CAMERA_ISO_DEBLUR:	/* Do not support */
		case CAMERA_ISO_100:
			rc = isx006_i2c_write(isx006_client->addr,
					0x01E5, 0x07, BYTE_LEN);
			break;

		case CAMERA_ISO_200:
			rc = isx006_i2c_write(isx006_client->addr,
					0x01E5, 0x0A, BYTE_LEN);
			break;

		case CAMERA_ISO_400:
			rc = isx006_i2c_write(isx006_client->addr,
					0x01E5, 0x0D, BYTE_LEN);
			break;
			
		case CAMERA_ISO_800:
			rc = isx006_i2c_write(isx006_client->addr,
					0x01E5, 0x10, BYTE_LEN);
			break;

		default:
			rc = -EINVAL;
	}
	
	return rc;
}

static int32_t isx006_set_scene_mode(int8_t mode)
{
	int32_t rc = 0;

  printk("%s\n",__func__);

	if (prev_scene_mode == mode)
		return rc;

	switch (mode) {
		case CAMERA_SCENE_AUTO:
			rc = isx006_i2c_write_table(isx006_regs.scene_auto_reg_settings,
				isx006_regs.scene_auto_reg_settings_size);
			break;

		case CAMERA_SCENE_PORTRAIT:
			rc = isx006_i2c_write_table(isx006_regs.scene_portrait_reg_settings,
				isx006_regs.scene_portrait_reg_settings_size);
			break;

		case CAMERA_SCENE_LANDSCAPE:
			rc = isx006_i2c_write_table(isx006_regs.scene_landscape_reg_settings,
				isx006_regs.scene_landscape_reg_settings_size);
			break;

		case CAMERA_SCENE_SPORTS:
			rc = isx006_i2c_write_table(isx006_regs.scene_sports_reg_settings,
				isx006_regs.scene_sports_reg_settings_size);
			break;

		case CAMERA_SCENE_SUNSET:
			rc = isx006_i2c_write_table(isx006_regs.scene_sunset_reg_settings,
				isx006_regs.scene_sunset_reg_settings_size);
			break;

		case CAMERA_SCENE_NIGHT:
			rc = isx006_i2c_write_table(isx006_regs.scene_night_reg_settings,
				isx006_regs.scene_night_reg_settings_size);
			break;

		default:
			printk(KERN_ERR "[ERROR]%s:Incorrect scene mode value\n", __func__);
	}

	prev_scene_mode = mode;

	return rc;
}

static int32_t isx006_set_brightness(int8_t brightness)
{
	int32_t rc=0;

  printk("%s\n",__func__);

	switch (brightness) {
		case 0:
			rc = isx006_i2c_write(isx006_client->addr,
					0x0060, 0x80, BYTE_LEN);
			if(rc<0)
				return rc;
			
			rc = isx006_i2c_write(isx006_client->addr,
					0x0061, 0x50, BYTE_LEN);
			if(rc<0)
				return rc;
			
			break;

		case 1:
			rc = isx006_i2c_write(isx006_client->addr,
					0x0060, 0x80, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx006_i2c_write(isx006_client->addr,
					0x0061, 0x60, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 2:
			rc = isx006_i2c_write(isx006_client->addr,
					0x0060, 0x80, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx006_i2c_write(isx006_client->addr,
					0x0061, 0x70, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 3:
			rc = isx006_i2c_write(isx006_client->addr,
					0x0060, 0xCD, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx006_i2c_write(isx006_client->addr,
					0x0061, 0x80, BYTE_LEN);
			if(rc<0)
			return rc;	

			break;

		case 4:
			rc = isx006_i2c_write(isx006_client->addr,
					0x0060, 0xEF, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx006_i2c_write(isx006_client->addr,
					0x0061, 0x80, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 5:
			rc = isx006_i2c_write(isx006_client->addr,
					0x0060, 0x00, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx006_i2c_write(isx006_client->addr,
					0x0061, 0x80, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 6:
			rc = isx006_i2c_write(isx006_client->addr,
					0x0060, 0x18, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx006_i2c_write(isx006_client->addr,
					0x0061, 0x80, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 7:
			rc = isx006_i2c_write(isx006_client->addr,
					0x0060, 0x7F, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx006_i2c_write(isx006_client->addr,
					0x0061, 0x8A, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 8:
			rc = isx006_i2c_write(isx006_client->addr,
					0x0060, 0x7F, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx006_i2c_write(isx006_client->addr,
					0x0061, 0x9C, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 9:
			rc = isx006_i2c_write(isx006_client->addr,
					0x0060, 0x7F, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx006_i2c_write(isx006_client->addr,
					0x0061, 0xAA, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		case 10:
			rc = isx006_i2c_write(isx006_client->addr,
					0x0060, 0x7F, BYTE_LEN);
			if(rc<0)
				return rc;	

			rc = isx006_i2c_write(isx006_client->addr,
					0x0061, 0xC8, BYTE_LEN);
			if(rc<0)
				return rc;	

			break;

		default:
			printk(KERN_ERR "[ERROR]%s:incoreect brightness value\n",
				__func__);
	}
	
	return rc;
}

static int isx006_set_jpeg_quality(int8_t quality)
{
	int32_t rc = 0;
  uint8_t jpeg_qulity;

  if (quality >= 85)
    jpeg_qulity = 0x02;
  else if (quality >= 75)
    jpeg_qulity = 0x01;
  else
    jpeg_qulity = 0x00;
    
  rc = isx006_i2c_write(isx006_client->addr, 0x0204, jpeg_qulity, BYTE_LEN);
  
  printk(KERN_INFO "ISX006: " "jpeg quality %d\n", quality);

  return rc;
}

static int isx006_set_jpeg_rotation(int32_t rotation)
{
	int32_t rc = 0;
  uint16_t jpeg_dri;
  
  if (rotation)
    jpeg_dri = 0x0001;
  else
    jpeg_dri = 0x0000;

  rc = isx006_i2c_write(isx006_client->addr, 0x0202, jpeg_dri, WORD_LEN);  //JPEG DRI
  
  printk(KERN_INFO "ISX006: " "jpeg rotation %d\n", rotation);
   
  return rc;
}

static int32_t isx006_set_preview_mode(int32_t mode)
{
	int32_t rc = 0;
	//int retry = 0;  

  printk(KERN_ERR "prevew_mode_status->%d , mode -> %d\n", prevew_mode_status, mode);

  if(prevew_mode_status == mode)
  { 
      return rc; 
  }
    
	switch (mode) {
		case CAMERA_PREVIEW_MODE_FPS:
      isx006_i2c_write(isx006_client->addr,0x4016,0x243D,2);// 35.00   ms   
      isx006_i2c_write(isx006_client->addr,0x4018,0x03CA,2);// 5.70  dB   
      isx006_i2c_write(isx006_client->addr,0x401A,0x029B,2);// 55.00   ms   
      isx006_i2c_write(isx006_client->addr,0x401C,0x0364,2);// 10.80   dB   
      isx006_i2c_write(isx006_client->addr,0x401E,0x03BB,2);// 105.00  ms   
      isx006_i2c_write(isx006_client->addr,0x4020,0x03CA,2);// 16.50   dB   
      isx006_i2c_write(isx006_client->addr,0x02A4,0x0000,2); //VADJ_SENS_1
      isx006_i2c_write(isx006_client->addr,0x0383,0x02,1); //FPSTYPE_MONI   
      isx006_i2c_write(isx006_client->addr,0x0012,0x01,1); // MONI_REFRESH_F
			break;

		case CAMERA_CAMCORD_MODE_FPS:
      isx006_i2c_write(isx006_client->addr,0x4016,0x1D03,2);	//10.00 	ms
      isx006_i2c_write(isx006_client->addr,0x4018,0x03FD,2);	//6.00 	dB
      isx006_i2c_write(isx006_client->addr,0x401A,0x0549,2);	//25.00 	ms
      isx006_i2c_write(isx006_client->addr,0x401C,0x0331,2);	//10.80 	dB
      isx006_i2c_write(isx006_client->addr,0x401E,0x02FE,2);	//42.00 	ms
      isx006_i2c_write(isx006_client->addr,0x4020,0x03CA,2);	//16.50 	dB
      isx006_i2c_write(isx006_client->addr,0x02A4,0x00C0,2);	//VADJ_SENS_1_2
      isx006_i2c_write(isx006_client->addr,0x0383,0x02,1);	//FPSTYPE_MONI
      isx006_i2c_write(isx006_client->addr,0x0012,0x01,1);	// MONI_REFRESH_F
			break;

		case CAMERA_CAMCORD_MODE_FPS_FOR_DELIVERING:
      isx006_i2c_write(isx006_client->addr,0x4016,0x1D03,2); //10.00   ms
      isx006_i2c_write(isx006_client->addr,0x4018,0x03FD,2); //6.00  dB
      isx006_i2c_write(isx006_client->addr,0x401A,0x0657,2); //30.00   ms
      isx006_i2c_write(isx006_client->addr,0x401C,0x0331,2); //10.80   dB
      isx006_i2c_write(isx006_client->addr,0x401E,0x048C,2); //66.00   ms
      isx006_i2c_write(isx006_client->addr,0x4020,0x03CA,2); //16.50   dB
      isx006_i2c_write(isx006_client->addr,0x02A4,0x0000,2); //VADJ_SENS_1_2
      isx006_i2c_write(isx006_client->addr,0x0383,0x03,1); //FPSTYPE_MONI
      isx006_i2c_write(isx006_client->addr,0x0012,0x01,1); // MONI_REFRESH_F
			break;
      
		default:
			printk(KERN_ERR "Incorrect preview mode value\n");
	}

  prevew_mode_status = mode;
  
  return rc;
}

static int32_t isx006_set_zoom(int32_t mode)
{
  int rc;
  
  if ((mode / 3) >= ISX006_MAX_ZOOM_STEP)
  {
    printk(KERN_ERR "isx006_set_zoom : invalid >%d %d \n", mode, mode / 3);
    return 0;
  }
  prev_zoom_step = mode / 3;
  
  rc = isx006_i2c_write(isx006_client->addr, 0x0032, zoom_table[prev_zoom_step], WORD_LEN);  // EZOOM_MAG
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x0034, zoom_offset_x, WORD_LEN);  // OFFSET_X
  if (rc < 0)
    return rc;
  rc = isx006_i2c_write(isx006_client->addr, 0x0036, zoom_offset_y, WORD_LEN);  // OFFSET_Y
  if (rc < 0)
    return rc;


  printk(KERN_ERR "isx006_set_zoom : >%d %d \n", mode, mode / 3);

  return rc;
}

static int isx006_reset(const struct msm_camera_sensor_info *dev, int value)
{
	int rc = 0;
	
	rc = gpio_request(dev->sensor_reset, "isx006");
	if (!rc) 
		rc = gpio_direction_output(dev->sensor_reset, value);
	else{
		printk("isx006_reset: reset gpio_direction_output fail\n");
		return rc;
	}

	gpio_free(dev->sensor_reset);
	return rc;
}

static int isx006_pwdn(const struct msm_camera_sensor_info *dev, int value)
{
	int rc = 0;
	
	rc = gpio_request(dev->sensor_pwd, "isx006");
	if (!rc) 
		rc = gpio_direction_output(dev->sensor_pwd, value);
	else{
		printk("isx006_pwdn: pwdn gpio_direction_output fail\n");
		return rc;
	}

	gpio_free(dev->sensor_pwd);
	return rc;
}

static int isx006_init_sensor(const struct msm_camera_sensor_info *data)
{
	int rc;
	int nNum = 0;
#ifndef IMAGE_TUNNING
  common_reg_list_type* pstRegisterList = NULL, *ptr_list;
  int loop;
#endif

	rc = isx006_reset(data, 0);
	if (rc < 0) {
		printk("reset failed!\n");
		goto init_fail;
	}
    rc = isx006_pwdn(data, 0);
	if (rc < 0) {
		printk("pwdn failed!\n");
		goto init_fail;
    }
    
	rc = data->pdata->camera_power_on();
	if (rc < 0) {
		printk(KERN_ERR "[ERROR]%s:failed to power on!\n", __func__);
		return rc;
	}

	printk("isx006_sensor_power_enable\n");	
	mdelay(1);
	
	/* Input MCLK = 27MHz */
	msm_camio_clk_rate_set(27000000);
	msm_camio_camif_pad_reg_reset();
	mdelay(1);
	printk("msm_camio_camif_pad_reg_reset\n");	

	rc = isx006_reset(data,1);
	if (rc < 0) {
		printk("reset failed!\n");
		goto init_fail;
	}
    mdelay(1);
    rc = isx006_pwdn(data,1);
	if (rc < 0) {
		printk("pwdn failed!\n");
		goto init_fail;
    }

    mdelay(8);  // T2
    

	/*pll register write*/
	rc = isx006_reg_init();
	if (rc < 0) {
		for(nNum = 0; nNum<5; nNum++)
		{
		 msleep(2);
			printk(KERN_ERR "[ERROR]%s:Set initial register error! retry~! \n", __func__);
			rc = isx006_reg_init();
			if(rc < 0)
			{
				nNum++;
				printk(KERN_ERR "[ERROR]%s:Set initial register error!- loop no:%d \n", __func__, nNum);
			}
			else
			{
				printk(KERN_DEBUG"[%s]:Set initial register Success!\n", __func__);
				break;
			}
		}
	}

  /* preview addition code */
  isx006_reg_preview_addition();

  /* move to here */
  rc = isx006_i2c_write(isx006_client->addr, 0x0009, 0x1F, BYTE_LEN); // INCK_SET          : INCK Free  (26.66667MHz)   
  if (rc < 0)
    return rc;

	mdelay(16);  // T3+T4

#ifndef IMAGE_TUNNING
  common_register_init(COMMON_REG_MEM_VAR4, &pstRegisterList);
  if (!pstRegisterList)
  {
    rc = isx006_reg_tuning();
  }
  else
  {
    ptr_list = pstRegisterList;
  
    for (loop = 0; loop < ptr_list->num_regs; loop++)
    {
      if (ptr_list->list_regs[loop].mem_var4.addr == 0xFFFF)
      {
        mdelay(ptr_list->list_regs[loop].mem_var4.vals.val32);
        continue;
      }
      isx006_i2c_write(isx006_client->addr, ptr_list->list_regs[loop].mem_var4.addr, ptr_list->list_regs[loop].mem_var4.vals.val32, ptr_list->list_regs[loop].mem_var4.len);
    }
  }
  
  if (pstRegisterList)
    kfree(pstRegisterList);
#else
	/*tuning register write*/
	rc = isx006_reg_tuning();
	if (rc < 0) {
		for(nNum = 0; nNum<5 ;nNum++)
		{
		  msleep(2);
			printk(KERN_ERR "[ERROR]%s:Set initial register error! retry~! \n", __func__);
			rc = isx006_reg_init();
			if(rc < 0)
			{
				nNum++;
				printk(KERN_ERR "[ERROR]%s:Set tuning register error! loop no:%d\n", __func__, nNum);
			}
			else
			{
				printk(KERN_DEBUG"[%s]:Set initial tuning Success!\n", __func__);
				break;
			}
		}
	
	}
#endif

  is_from_capture = 1;
  prev_scene_mode = CAMERA_SCENE_AUTO;
  prevew_mode_status = CAMERA_MODE_DEFAULT;
  prev_zoom_step = 0;
  
	return rc;

init_fail:
	printk("isx006: isx006_sensor_init failed\n");
	kfree(isx006_ctrl);
	return rc;  
}

static int isx006_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int rc;

	CDBG("init entry \n");

	rc = isx006_init_sensor(data);
	if (rc < 0) {
		printk(KERN_ERR "[ERROR]%s:failed to initialize sensor!\n", __func__);
		goto init_probe_fail;
	}

	prev_af_mode = -1;
	prev_scene_mode = -1;

	return rc;

init_probe_fail:
	return rc;
}

int isx006_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	isx006_ctrl = kzalloc(sizeof(struct isx006_ctrl), GFP_KERNEL);
	if (!isx006_ctrl) {
		printk(KERN_ERR "[ERROR]%s:isx006_init failed!\n", __func__);
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		isx006_ctrl->sensordata = data;

	rc = isx006_sensor_init_probe(data);
	if (rc < 0) {
		printk(KERN_ERR "[ERROR]%s:isx006_sensor_init failed!\n", __func__);
		goto init_fail;
	}

init_done:
	return rc;

init_fail:
	kfree(isx006_ctrl);
	return rc;
}

int isx006_sensor_release(void)
{
	int rc = 0;

	if(always_on) {
		printk("always power-on camera.\n");
		return rc;
	}

	mutex_lock(&isx006_mutex);

	rc = isx006_ctrl->sensordata->pdata->camera_power_off();

	kfree(isx006_ctrl);

	mutex_unlock(&isx006_mutex);

	return rc;
}

int isx006_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	int rc;

	rc = copy_from_user(&cfg_data, (void *)argp,
		sizeof(struct sensor_cfg_data));

	if (rc < 0)
		return -EFAULT;

	CDBG("isx006_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

	mutex_lock(&isx006_mutex);

	switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = isx006_set_sensor_mode(cfg_data.mode, cfg_data.width, cfg_data.height);
			break;

		case CFG_SET_EFFECT:
			rc = isx006_set_effect(cfg_data.mode);
			break;

		case CFG_MOVE_FOCUS:
			rc = isx006_move_focus(cfg_data.cfg.focus.steps);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc = isx006_set_default_focus();
			break;

		case CFG_GET_AF_MAX_STEPS:
			cfg_data.max_steps = ISX006_TOTAL_STEPS_NEAR_TO_FAR;
			if (copy_to_user((void *)argp,
					&cfg_data,
					sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_START_AF_FOCUS:
			rc = isx006_set_af_start(cfg_data.mode);
			break;

		case CFG_CHECK_AF_DONE:
			rc = isx006_check_focus(&cfg_data.mode);
			if (copy_to_user((void *)argp,
					&cfg_data,
					sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_CHECK_AF_CANCEL:
			rc = isx006_cancel_focus(cfg_data.mode);
			break;

		case CFG_SET_WB:
			rc = isx006_set_wb(cfg_data.mode);
			break;

		case CFG_SET_ANTIBANDING:
			rc= isx006_set_antibanding(cfg_data.mode);
			break;

		case CFG_SET_ISO:
			rc = isx006_set_iso(cfg_data.mode);
			break;

		case CFG_SET_SCENE:
			rc = isx006_set_scene_mode(cfg_data.mode);
			break;

		case CFG_SET_BRIGHTNESS:
			rc = isx006_set_brightness(cfg_data.mode);
			break;

		case CFG_SET_JPEG_QUALITY:
			rc = isx006_set_jpeg_quality(cfg_data.cfg.jpeg_quality);
			break;
      
    case CFG_SET_JPEG_ROTATION:
      rc = isx006_set_jpeg_rotation(cfg_data.cfg.jpeg_rotation);
      break;

		case CFG_SET_PREVIEW_MODE:
			rc = isx006_set_preview_mode(cfg_data.mode);
			break;

		case CFG_SET_ZOOM:
			rc = isx006_set_zoom(cfg_data.mode);  // use mode field
			break;

		default:
			rc = -EINVAL;
			break;
	}

	mutex_unlock(&isx006_mutex);

	return rc;
}

static const struct i2c_device_id isx006_i2c_id[] = {
	{ "isx006", 0},
	{ },
};

static int isx006_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&isx006_wait_queue);
	return 0;
}

static int isx006_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	isx006_sensorw = kzalloc(sizeof(struct isx006_work), GFP_KERNEL);
	if (!isx006_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, isx006_sensorw);
	isx006_init_client(client);
	isx006_client = client;

	CDBG("isx006_probe succeeded!\n");

	return rc;

probe_failure:
	printk(KERN_ERR "[ERROR]%s:isx006_probe failed!\n", __func__);
	return rc;
}

static struct i2c_driver isx006_i2c_driver = {
	.id_table = isx006_i2c_id,
	.probe  = isx006_i2c_probe,
	.remove = __exit_p(isx006_i2c_remove),
	.driver = {
		.name = "isx006",
	},
};

static ssize_t mclk_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t mclk_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	return size;
}

static DEVICE_ATTR(mclk, S_IRWXUGO, mclk_show, mclk_store);

static ssize_t always_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("always_on = %d\n", always_on);
	return 0;
}

static ssize_t always_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	always_on = value;

	printk("always_on = %d\n", always_on);
	return size;
}

static DEVICE_ATTR(always_on, S_IRWXUGO, always_on_show, always_on_store);

static ssize_t isx006_wb_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int val;
	long rc;

	if (isx006_ctrl == NULL)
		return 0;

	sscanf(buf,"%x",&val);

	if(val < CAMERA_WB_AUTO || val > CAMERA_WB_SHADE) {
		printk("isx006: invalid white balance input\n");
		return 0;
	}

	rc = isx006_set_wb(val);
	if (rc < 0)
		printk("isx006: failed to set white balance\n");

	return n;
}
static DEVICE_ATTR(wb, S_IRUGO|S_IWUGO, NULL, isx006_wb_store);

static ssize_t isx006_effect_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int val;
	long rc;

	if (isx006_ctrl == NULL)
		return 0;

	sscanf(buf,"%x",&val);

	if(val < CAMERA_EFFECT_OFF || val > CAMERA_EFFECT_MAX) {
		printk("isx006: invalid effect input\n");
		return 0;
	}

	rc = isx006_set_effect(val);
	if (rc < 0)
		printk("isx006: failed to set effect\n");

	return n;
}
static DEVICE_ATTR(effect, S_IRUGO|S_IWUGO, NULL, isx006_effect_store);


static ssize_t isx006_zoom_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("zoom ration = %x\n", zoom_ratio);
  
	return sprintf(buf, "0x%x\n", zoom_ratio);;
}

static ssize_t isx006_zoom_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int value;

	sscanf(buf, "%x", &value);
	zoom_ratio = value;

	printk("ration = %x\n", zoom_ratio);

  return n;
}

static DEVICE_ATTR(zoom, S_IRUGO|S_IWUGO, isx006_zoom_show, isx006_zoom_store);

static ssize_t isx006_zoom_offset_x_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("zoom offset x = %x\n", zoom_offset_x);
  
	return sprintf(buf, "0x%x\n", zoom_offset_x);;
}

static ssize_t isx006_zoom_offset_x_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int value;

	sscanf(buf, "%x", &value);
	zoom_offset_x = value;

	printk("zoom offset x = %x\n", zoom_offset_x);

  return n;
}

static DEVICE_ATTR(zoom_x, S_IRUGO|S_IWUGO, isx006_zoom_offset_x_show, isx006_zoom_offset_x_store);

static ssize_t isx006_zoom_offset_y_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("zoom offset y = %x\n", zoom_offset_y);
  
	return sprintf(buf, "0x%x\n", zoom_offset_y);;
}

static ssize_t isx006_zoom_offset_y_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int value;

	sscanf(buf, "%x", &value);
	zoom_offset_y = value;

	printk("zoom offset y = %x\n", zoom_offset_y);

  return n;
}

static DEVICE_ATTR(zoom_y, S_IRUGO|S_IWUGO, isx006_zoom_offset_y_show, isx006_zoom_offset_y_store);


static int isx006_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc = i2c_add_driver(&isx006_i2c_driver);
	if (rc < 0 || isx006_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

	s->s_init = isx006_sensor_init;
	s->s_release = isx006_sensor_release;
	s->s_config  = isx006_sensor_config;

	rc = device_create_file(&isx006_pdev->dev, &dev_attr_mclk);
	if (rc < 0) {
		printk("device_create_file error!\n");
		return rc;
	}

	rc = device_create_file(&isx006_pdev->dev, &dev_attr_always_on);
	if (rc < 0) {
		printk("device_create_file error!\n");
		return rc;
	}

  rc = device_create_file(&isx006_pdev->dev, &dev_attr_wb);
  if (rc < 0) {
    printk("device_create_file error!\n");
    return rc;
  }

  rc = device_create_file(&isx006_pdev->dev, &dev_attr_effect);
  if (rc < 0) {
    printk("device_create_file error!\n");
    return rc;
  }
  
  rc = device_create_file(&isx006_pdev->dev, &dev_attr_zoom);
  if (rc < 0) {
    printk("device_create_file error!\n");
    return rc;
  }

  rc = device_create_file(&isx006_pdev->dev, &dev_attr_zoom_x);
  if (rc < 0) {
    printk("device_create_file error!\n");
    return rc;
  }
  rc = device_create_file(&isx006_pdev->dev, &dev_attr_zoom_y);
  if (rc < 0) {
    printk("device_create_file error!\n");
    return rc;
  }


probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __isx006_probe(struct platform_device *pdev)
{
	isx006_pdev = pdev;
	return msm_camera_drv_start(pdev, isx006_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __isx006_probe,
	.driver = {
		.name = "msm_camera_isx006",
		.owner = THIS_MODULE,
	},
};

static int __init isx006_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

late_initcall(isx006_init);
