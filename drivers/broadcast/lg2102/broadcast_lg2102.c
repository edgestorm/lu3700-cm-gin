

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/i2c.h>
//#include <linux/spi/spi.h>
//#include <linux/spi/spidev.h>
#include <linux/interrupt.h>

#include <linux/gpio.h>
#include <linux/delay.h>
//#include <linux/workqueue.h>
//#include <linux/wakelock.h> 		/* wake_lock, unlock */
#include <linux/slab.h>

#include "../broadcast_tdmb_drv_ifdef.h"
#include "broadcast_lg2102.h"
#include "LGD_INCLUDES.h"
#include "tdmb_tunerbbdrv_lg2102def.h"

#define	MS_DIVIDE_UNIT		((int)100)

/* external function */
//extern void broadcast_tdmb_read_data(void);

#define DELAY_USING_WAIT_EVENT_TIMEOUT  /* wait_event_timeout instead of msleep */
#ifdef DELAY_USING_WAIT_EVENT_TIMEOUT
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#endif

#if 0
/* proto type declare */
static int broadcast_tdmb_lg2102_probe(struct spi_device *spi);
static int broadcast_tdmb_lg2102_remove(struct spi_device *spi);
static int broadcast_tdmb_lg2102_suspend(struct spi_device *spi, pm_message_t mesg);
static int broadcast_tdmb_lg2102_resume(struct spi_device *spi);
#endif

//#define DMB_EN 			124 //GPIO28
//#define DMB_INT_N 		99 //GPIO29
//#define DMB_RESET_N 	100 //GPIO62

/************************************************************************/
/* LINUX Driver Setting                                                 */
/************************************************************************/
#define DMB_POWER_GPIO    124
#define DMB_RESET_GPIO    100
#define DMB_INT_GPIO       99
#ifdef DELAY_USING_WAIT_EVENT_TIMEOUT
#define SLEEP_WAIT_UNTIL_MS 0
#define SLEEP_EXIT_USER_STOP 1
static DECLARE_WAIT_QUEUE_HEAD(msleep_wait_queue);  /*wait_event_timeout queue */
static uint32 msleep_exit_condition = SLEEP_WAIT_UNTIL_MS;		/* sleep exit condition not timeout(sleep) */
static uint32 msleep_wait_queue_init = 0;
#else
static uint32 user_stop_flg = 0;
static uint32 mdelay_in_flg = 0;
#endif  /* DELAY_USING_WAIT_EVENT_TIMEOUT */
struct TDMB_LG2102_CTRL
{
	boolean 					TdmbPowerOnState;
	struct i2c_client*			pClient;
//	struct spi_device* 		pSpiDevice;
//	struct work_struct 		spi_work;
//	struct workqueue_struct* 	spi_wq;
//	struct mutex				mutex;
//	struct wake_lock 			wake_lock;	/* wake_lock,wake_unlock */
	//test
//	boolean 					spi_irq_status;
};

//static broadcast_pwr_func pwr_func;

static struct TDMB_LG2102_CTRL TdmbCtrlInfo;

struct i2c_client* INC_GET_I2C_DRIVER(void)
{
	return TdmbCtrlInfo.pClient;
}

void LGD_RW_TEST(void);

#ifdef DELAY_USING_WAIT_EVENT_TIMEOUT
void tdmb_lg2102_set_userstop(void)
{
	if(msleep_exit_condition == SLEEP_WAIT_UNTIL_MS)
	{
		msleep_exit_condition = SLEEP_EXIT_USER_STOP;
		wake_up(&msleep_wait_queue);
	}
}

int tdmb_lg2102_mdelay(int32 ms)
{
	int rc = 1;
	int wait_rc = OK;

	if(msleep_wait_queue_init == 0)
	{
		init_waitqueue_head(&msleep_wait_queue);
		msleep_wait_queue_init = 1;
	}

	msleep_exit_condition = SLEEP_WAIT_UNTIL_MS;
	/* sleep during msec set or msleep_exit_condition meet */
	wait_rc = wait_event_timeout(msleep_wait_queue, 
		(msleep_exit_condition == SLEEP_EXIT_USER_STOP), msecs_to_jiffies(ms));

	/* wait exit becaus of user stop not timeout */
	if(msleep_exit_condition == SLEEP_EXIT_USER_STOP)
	{
		rc = 0;
	}
	
	msleep_exit_condition = SLEEP_WAIT_UNTIL_MS;
	return rc;
}
#else
void tdmb_lg2102_set_userstop(void)
{
	user_stop_flg = ((mdelay_in_flg == 1)? 1: 0 );
}


int tdmb_lg2102_mdelay(int32 ms)
{
	long 		wait_loop =0;
	long 		wait_ms = ms;
	int 		rc = 1;

	/* Input ms must be the multiple of MS_DIVIDE_UNIT. Otherwise this must be modifyed */
	if(ms > MS_DIVIDE_UNIT)
	{
		wait_loop = (ms /MS_DIVIDE_UNIT);
		wait_ms = MS_DIVIDE_UNIT;
	}

	mdelay_in_flg = 1;
	do
	{
		msleep(wait_ms);
		if(user_stop_flg == 1)
		{
			printk("~~~~~~~~ Ustop flag is set so return false ~~~~~~~~\n");
			rc = 0;
			break;
		}
	}while((--wait_loop) > 0);
	mdelay_in_flg = 0;
	user_stop_flg = 0;

	return rc;
}
#endif  /* DELAY_USING_WAIT_EVENT_TIMEOUT */

int tdmb_lg2102_power_on(void)
{
	//int rc = 0;
	// DMB_INT = GPIO29
	// DMB_EN = GPIO28(1.2V) , 1.8V_VIO(alyways on)
	// DMB_RESET = GPIO62
	if ( TdmbCtrlInfo.TdmbPowerOnState == FALSE )
	{
		//wake_lock(&TdmbCtrlInfo.wake_lock);
	gpio_set_value(DMB_RESET_GPIO, 0);
	gpio_set_value(DMB_POWER_GPIO, 1);
	//udelay(2000);
	mdelay(2); /* 2msec delay */
	gpio_set_value(DMB_RESET_GPIO, 1);
	//udelay(10);
	LGD_CMD_WRITE(TDMB_RFBB_DEV_ADDR, APB_SPI_BASE+ 0x00, 0x0011);  /* SPI Disable */

	udelay(100);
	gpio_set_value(DMB_RESET_GPIO, 0);
	//udelay(1000);
	mdelay(1);
	gpio_set_value(DMB_RESET_GPIO, 1);
	//udelay(10);
	LGD_CMD_WRITE(TDMB_RFBB_DEV_ADDR, APB_SPI_BASE+ 0x00, 0x0011);	/* SPI Disable */
	udelay(10);
		//tdmb_lg2102_interrupt_free();
		TdmbCtrlInfo.TdmbPowerOnState = TRUE;
	}
	else
	{
		printk("tdmb_lg2102_power_on the power already turn on \n");
	}

	printk("tdmb_lg2102_power_on completed \n");

	return TRUE;
}

int tdmb_lg2102_power_off(void)
{
	//int rc = FALSE;

	if ( TdmbCtrlInfo.TdmbPowerOnState == TRUE )
	{
		//tdmb_lg2102_interrupt_lock();
		TdmbCtrlInfo.TdmbPowerOnState = FALSE;
		gpio_set_value(DMB_POWER_GPIO, 0);
		gpio_set_value(DMB_RESET_GPIO, 0);
		udelay(10);
	}
	else
	{
		printk("tdmb_lg2102_power_on the power already turn off \n");
	}	

	printk("tdmb_lg2102_power_off completed \n");
	
	return TRUE;
}


 static int tdmb_lg2102_i2c_write(uint8* txdata, int length)
{
	struct i2c_msg msg = 
	{	
		TdmbCtrlInfo.pClient->addr,
		0,
		length,
		txdata 
	};


	if (i2c_transfer( TdmbCtrlInfo.pClient->adapter, &msg, 1) < 0) 
	{
		printk("tdmb lg2102 i2c write failed\n");
		return FALSE;
	}

	//printk("tdmb lg2102 i2c write addr = %x data = %x!! \n",addr, data);
	
	return TRUE;
}
 
int tdmb_lg2102_i2c_write_burst(uint16 waddr, uint8* wdata, int length)
{
 	uint8 *buf;
	int	wlen;

	int rc;

	wlen = length + 2;

	buf = (uint8*)kmalloc( wlen, GFP_KERNEL);

	if((buf == NULL) || ( length <= 0 ))
	{
		printk("tdmb_lg2102_i2c_write_burst buf alloc failed\n");
		return FALSE;
	}

	buf[0] = (waddr>>8)&0xFF;
	buf[1] = (waddr&0xFF);

	memcpy(&buf[2], wdata, length);
 
	rc = tdmb_lg2102_i2c_write(buf, wlen);

	kfree(buf);
		
	return rc;
}

static int tdmb_lg2102_i2c_read( uint16 raddr,	uint8 *rxdata, int length)
{
	uint8	r_addr[2] = {raddr>>8, raddr&0xff};
	//uint8	acBuff[384];
	
	struct i2c_msg msgs[] = 
	{
		{
			.addr   = TdmbCtrlInfo.pClient->addr,
			.flags = 0,
			.len   = 2,
			.buf   = &r_addr[0],
		},
		{
			.addr   = TdmbCtrlInfo.pClient->addr,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = rxdata,
		},
	};
	
	//memset(acBuff, 0, sizeof(acBuff));

	if (i2c_transfer(TdmbCtrlInfo.pClient->adapter, msgs, 2) < 0) 
	{
		printk("tdmb_lg2102_i2c_read failed! %x \n",TdmbCtrlInfo.pClient->addr);
		return FALSE;
	}
	
	//memcpy(rxdata,acBuff, length);

	
	//printk("tdmb lg2102 i2c read addr = %x data = %x!! \n",addr, data);
	
	return TRUE;
}

int tdmb_lg2102_i2c_read_burst(uint16 raddr, uint8* rdata, int length)
{
	int rc;
 
	rc = tdmb_lg2102_i2c_read(raddr, rdata, length);

	return rc;

}


int tdmb_lg2102_i2c_write16(unsigned short reg, unsigned short val)
{
	unsigned int err;
	unsigned char buf[4] = { reg>>8, reg&0xff, val>>8, val&0xff };
	struct i2c_msg	msg = 
	{	
		TdmbCtrlInfo.pClient->addr,
		0,
		4,
		buf 
	};
	
	if ((err = i2c_transfer( TdmbCtrlInfo.pClient->adapter, &msg, 1)) < 0) 
	{
		dev_err(&TdmbCtrlInfo.pClient->dev, "i2c write error\n");
		err = FALSE;
	}
	else
	{
		//printk(KERN_INFO "tdmb : i2c write ok:addr = %x data = %x\n", reg, val);
		err = TRUE;
	}

	return err;
}


int tdmb_lg2102_i2c_read16(uint16 reg, uint16 *ret)
{

	uint32 err;
	uint8 w_buf[2] = {reg>>8, reg&0xff};	
	uint8 r_buf[2] = {0,0};

	struct i2c_msg msgs[2] = 
	{
		{ TdmbCtrlInfo.pClient->addr, 0, 2, &w_buf[0] },
		{ TdmbCtrlInfo.pClient->addr, I2C_M_RD, 2, &r_buf[0]}
	};

	if ((err = i2c_transfer(TdmbCtrlInfo.pClient->adapter, msgs, 2)) < 0) 
	{
		dev_err(&TdmbCtrlInfo.pClient->dev, "i2c read error\n");
		err = FALSE;
	}
	else
	{
		//printk( "tdmb addr = %x : i2c read ok: data[0] = %x data[1] = %x \n", TdmbCtrlInfo.pClient->addr, r_buf[0], r_buf[1]);
		*ret = r_buf[0]<<8 | r_buf[1];
		//printk( "tdmb : i2c read ok: data = %x\n", *ret);
		err = TRUE;
	}

	return err;
}


void LGD_RW_TEST(void)
{
	unsigned short i = 0;
	unsigned short w_val = 0;
	unsigned short r_val = 0;
	unsigned short err_cnt = 0;

	err_cnt = 0;
	for(i=1;i<11;i++)
	{
		w_val = (i%0xFF);
		tdmb_lg2102_i2c_write16( 0x0a00+ 0x05, w_val);
		tdmb_lg2102_i2c_read16(0x0a00+ 0x05, &r_val );
		if(r_val != w_val)
		{
			err_cnt++;
			printk("w_val:%x, r_val:%x\n", w_val,r_val);
		}
	}
}
int tdmb_lg2102_select_antenna(unsigned int sel)
{
	if(LGE_BROADCAST_TDMB_ANT_TYPE_INTENNA == sel)
	{
	}
	else if(LGE_BROADCAST_TDMB_ANT_TYPE_INTENNA == sel)
	{
	}
	else
	{
		return FALSE;
	}
	
	return TRUE;
}
