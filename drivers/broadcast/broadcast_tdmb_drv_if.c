#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/uaccess.h>        /* copy_to_user */

/* BUS clock speed fixed */
//#include <linux/pm_qos_params.h>
#include <linux/clk.h> //ojh
#include <mach/camera.h>
#include "broadcast_tdmb_typedef.h"
#include "broadcast_tdmb_drv_ifdef.h"

#define BROADCAST_TDMB_NUM_DEVS 	1 /**< support this many devices */

/* BUS clock speed fixed */
#define BROADCAST_AXI_QOS_RATE 200000//160000
#define BROADCAST_AXI_DEFAULT_RATE 0
#define MSM_DMB_AXI_QOS_NAME  "broadcast_dmb"

/* Thunder Alessi LCD and Display Quality Tunning */
#if defined(CONFIG_MACH_MSM7X27_THUNDERG) || defined(CONFIG_MACH_MSM7X27_THUNDERC) || defined(CONFIG_MACH_MSM7X27_SU370) || defined(CONFIG_MACH_MSM7X27_KU3700) || defined(CONFIG_MACH_MSM7X27_LU3700) || defined(CONFIG_MACH_MSM7X27_SU310) || defined(CONFIG_MACH_MSM7X27_LU3100)
#define  CONFIG_BROADCAST_USE_LUT
#endif


static struct class *broadcast_tdmb_class;
static dev_t broadcast_tdmb_dev;
struct broadcast_tdmb_chdevice 
{
	struct cdev cdev;
	struct device *dev;
	resource_size_t ebi2_phys_base;
	void __iomem *ebi2_virt_base;
	resource_size_t ebi2_cr_phys_base;	
	void __iomem *ebi2_cr_virt_base;
	resource_size_t ebi2_xm_phys_base;	
	void __iomem *ebi2_xm_virt_base;
	uint8    *gp_buffer;
	unsigned int irq;
	unsigned int power_enable;
	unsigned int reset;
	unsigned int isr_gpio;
	struct wake_lock wake_lock;	/* wake_lock,wake_unlock */
	wait_queue_head_t wq_read;
	void *cookie;
};
/*
* EBI2 Interface : buffer copy length control parameter according to DMB op_mode.
* These parameters are used in broadcast_tdmb_tune_set_ch( ) and broadcast_tdmb_get_data().
* These default value are related to TDMB_TSIF_BUFF_XXX defined in mbthal_tsif.h.
*/

static uint32	      isr_setup_flag = FALSE;

/* user stop count */
//static int user_stop_mode_cnt = 0;
static struct broadcast_tdmb_chdevice tdmb_dev;
struct broadcast_tdmb_chdevice *pbroadcast = NULL;
/* Thunder, Aless Picture Tunning */
#ifdef CONFIG_BROADCAST_USE_LUT
static int dmb_lut_table = -1;
#endif
static struct clk *ebi1_pm_qos_clk; //ojh

#ifdef CONFIG_BROADCAST_USE_LUT
extern void mdp_load_thunder_lut(int lut_type);
#endif

static int broadcast_tdmb_add_axi_qos(void)
{
	int rc = 0;
	printk("broadcast_tdmb_add_axi_qos\n");
	ebi1_pm_qos_clk = clk_get(NULL, "ebi1_tdmb_clk");
	if (IS_ERR(ebi1_pm_qos_clk))
	{
		ebi1_pm_qos_clk = NULL;
		printk("request ebi1_pm_qos_clk fails. rc = %d\n", rc);
	}
	else
		clk_enable(ebi1_pm_qos_clk);
	

	return rc;
}

static void broadcast_tdmb_release_axi_qos(void)
{
	printk("broadcast_tdmb_release_axi_qos\n");
	if (!ebi1_pm_qos_clk)
		return;

	clk_disable(ebi1_pm_qos_clk);
	clk_put(ebi1_pm_qos_clk);
}


static int broadcast_tdmb_axi_qos_on(int  rate)
{
	int rc = 0;

		
	if (!ebi1_pm_qos_clk)
		return 0;
	printk("broadcast_tdmb_axi_qos_on rate = %d\n", rate);
	rc=clk_set_rate(ebi1_pm_qos_clk, rate * 1000); //200MHZ 
	if(rc<0)
		printk("broadcast_tdmb_axi_qos_on clk_set_rate fail = %d\n", rc);
	return rc;
}
static int broadcast_tdmb_power_on(void)
{
	int rc = ERROR;
		
	rc = broadcast_drv_if_power_on();

	broadcast_drv_if_user_stop();

	return rc;
}

static int broadcast_tdmb_power_off(void)
{
	int rc = ERROR;
	
	rc = broadcast_drv_if_power_off();

	broadcast_drv_if_user_stop();

	return rc;
}

static int broadcast_tdmb_open(void)
{
	int rc = ERROR;
		
	rc = broadcast_drv_if_open();

#ifdef CONFIG_BROADCAST_USE_LUT
	dmb_lut_table = -1;
#endif
	return rc;
}

static int broadcast_tdmb_close(void)
{
	int rc = ERROR;

	rc = broadcast_drv_if_close();
	
#ifdef CONFIG_BROADCAST_USE_LUT
	if(dmb_lut_table == 0)
	{
		mdp_load_thunder_lut(1);
		dmb_lut_table = -1;
	}
#endif
	return rc;
}

static int broadcast_tdmb_set_ebi2_address(void *addr)
{
	int rc = ERROR;
		
	rc = broadcast_drv_if_set_ebi2_address(addr);

	return rc;
}



static irqreturn_t broadcast_tdmb_isr(int irq, void *handle)
{
	int rc = ERROR;
	rc = broadcast_drv_if_isr();
	
	return IRQ_HANDLED;

}
static int8 broadcast_tdmb_setup_isr(void)
{
	int ret;
	/* ISR setup is done so just return */
	if(isr_setup_flag == 1)
	{
		printk("[broadcast_irq]broadcast_tdmb_setup_isr immediately done\n");
		return OK;
	}
	
	ret= gpio_tlmm_config(GPIO_CFG(DMB_INT_GPIO, 0, GPIO_CFG_INPUT,GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (ret < 0) {
		printk(KERN_ERR
			"broadcast_tdmb_power_on for  %s"
			"(rc = %d)\n", "gpio_tlmm_config", ret);
		ret = -EIO;
	}
	
	
	ret = gpio_request(DMB_INT_GPIO, "dmb_ebi2_int");
	if (ret < 0) {
		printk(KERN_ERR
			"broadcast_tdmb_power_on for  %s "
			"(rc = %d)\n", "gpio_request", ret);
		ret = -EIO;
	}
	
	pbroadcast->irq = MSM_GPIO_TO_INT(DMB_INT_GPIO);
	ret = request_irq(pbroadcast->irq , &broadcast_tdmb_isr ,
				(IRQF_DISABLED | IRQF_TRIGGER_FALLING) , "tdmb_lg2102", pbroadcast);
	if (ret < 0) {
		printk(KERN_ERR
			"Could not request_irq	for  %s interrupt "
			"(rc = %d)\n", "tdmb_lg2102", ret);
		ret = -EIO;
	}
	
	printk("[broadcast_irq]broadcast_tdmb_setup_isr = (0x%X)\n", pbroadcast->irq);
	isr_setup_flag = 1;
	return OK;
}

static int8 broadcast_tdmb_release_isr(void)
{
	if(isr_setup_flag == 0)
	{
		printk("[broadcast_irq]broadcast_tdmb_release_isr immediately done\n");
		return OK;
	}
	free_irq(pbroadcast->irq, pbroadcast);
	isr_setup_flag = 0;
	printk("[broadcast_irq]broadcast_tdmb_release_isr irq =(0x%X)\n", pbroadcast->irq);
	return OK;
}
static int broadcast_tdmb_set_channel(void __user *arg)
{
	int8 rc = ERROR;

	struct broadcast_tdmb_set_ch_info udata;

	if(copy_from_user(&udata, arg, sizeof(struct broadcast_tdmb_set_ch_info)))
	{	
		//ERR_COPY_FROM_USER();
		printk("broadcast_tdmb_set_ch fail!!! \n");
		ret = ERROR;
	}
	else
	{
		printk("broadcast_tdmb_set_ch ch_num = %d, mode = %d, sub_ch_id = %d \n", udata.ch_num, udata.mode, udata.sub_ch_id);
			/* Release IRQ Handler If there is */
		broadcast_tdmb_release_isr( );
		
		rc = broadcast_drv_if_set_channel(udata.ch_num, udata.sub_ch_id, udata.mode);


		/* Setup IRQ Handler */
		broadcast_tdmb_setup_isr( );

#ifdef CONFIG_BROADCAST_USE_LUT
		if((dmb_lut_table == -1) 
			&& ((udata.mode == 2)||(udata.mode == 3)))
		{
			mdp_load_thunder_lut(3);
			dmb_lut_table = 0;
		}
#endif
	}

	return rc;

}



static int broadcast_tdmb_resync(void __user *arg)
{
	return 0;
}

static int broadcast_tdmb_detect_sync(void __user *arg)
{
	int rc = ERROR;
	int udata;
	int __user* puser = (int __user*)arg;
	udata = *puser;

	rc = broadcast_drv_if_detect_sync(udata);

	return rc;
}

static int broadcast_tdmb_get_sig_info(void __user *arg)
{
	int rc = ERROR;
	struct broadcast_tdmb_sig_info udata;
	
	if(copy_from_user(&udata, arg, sizeof(struct broadcast_tdmb_sig_info)))
	{
		printk("broadcast_tdmb_get_sig_info copy_from_user error!!! \n");
		rc = ERROR;
	}
	else
	{		
		rc = broadcast_drv_if_get_sig_info(&udata);
	
		if(copy_to_user((void *)arg, &udata, sizeof(struct broadcast_tdmb_sig_info)))
		{
			printk("broadcast_tdmb_get_sig_info copy_to_user error!!! \n");
			rc = ERROR;
		}
		else
		{
			rc = OK;
		}
	}

	return rc;

}

static int broadcast_tdmb_get_ch_info(void __user *arg)
{
	int rc = ERROR;
	char fic_kernel_buffer[400];
	unsigned int fic_len = 0;

	struct broadcast_tdmb_ch_info __user* puserdata = (struct broadcast_tdmb_ch_info __user*)arg;

	if((puserdata == NULL)||( puserdata->ch_buf == NULL))
	{
		printk("broadcast_tdmb_get_ch_info argument error\n");
		return rc;
	}

	memset(fic_kernel_buffer, 0x00, sizeof(fic_kernel_buffer));
	rc = broadcast_drv_if_get_ch_info(fic_kernel_buffer, &fic_len);
	
	if(rc == OK)
	{
		if(copy_to_user((void __user*)puserdata->ch_buf, (void*)fic_kernel_buffer, fic_len))
		{
			fic_len = 0;
			rc = ERROR;
		}		
	}
	else
	{
		fic_len = 0;
		rc = ERROR;
	}
	
	puserdata->buf_len = fic_len;
	
	return rc;
}

static int broadcast_tdmb_get_dmb_data(void __user *arg)
{
	int				rc					= ERROR;
	char*			read_buffer_ptr		= NULL;
	unsigned int 	read_buffer_size 	= 0;
	unsigned int 	read_packet_cnt 	= 0;
	unsigned int 	copied_buffer_size 	= 0;
	
	struct broadcast_tdmb_data_info __user* puserdata = (struct broadcast_tdmb_data_info  __user*)arg;	
	
	while(1)
	{
		rc = broadcast_drv_if_get_dmb_data(&read_buffer_ptr, &read_buffer_size, puserdata->data_buf_size - copied_buffer_size);
		if(rc != OK)
		{			
			break;
		}
		
		if ( puserdata->data_buf_size < copied_buffer_size + read_buffer_size )
		{
			printk("broadcast_tdmb_get_dmb_data, output buffer is small\n");
			break;
		}

		if(copy_to_user((void __user*)(puserdata->data_buf + copied_buffer_size), (void*)read_buffer_ptr, read_buffer_size))
		{
			puserdata->copied_size= 0;
			puserdata->packet_cnt = 0;
			rc = ERROR;
			break;
		}
		else
		{
			copied_buffer_size += read_buffer_size;
			puserdata->copied_size = copied_buffer_size;

			read_packet_cnt = copied_buffer_size/188;
			puserdata->packet_cnt = read_packet_cnt;
			rc = OK;
		}
	}

	if(puserdata->copied_size)
	{
		rc = OK;
	}

	return rc;
}

static int8 broadcast_tdmb_reset_ch(void)
{
	int rc = ERROR;

	rc = broadcast_drv_if_reset_ch();
	
	return rc;
}

static int8 broadcast_tdmb_user_stop(void __user *arg)
{
	int udata;
	int __user* puser = (int __user*)arg;

	udata = *puser;

	printk("broadcast_tdmb_user_stop data =(%d) (IN)\n", udata);
	broadcast_drv_if_user_stop();

	return OK;
}

static int8 broadcast_tdmb_select_antenna(void __user *arg)
{
	int rc = ERROR;
	int udata;
	int __user* puser = (int __user*)arg;

	udata = *puser;

	rc = broadcast_drv_if_select_antenna(udata);
	
	return rc;
}

static ssize_t broadcast_tdmb_open_control(struct inode *inode, struct file *file)
{
	struct broadcast_tdmb_chdevice *the_dev =
	       container_of(inode->i_cdev, struct broadcast_tdmb_chdevice, cdev);

	printk("broadcast_tdmb_open_control start \n");
	 
	file->private_data = the_dev;
	
	printk("broadcast_tdmb_open_control OK \n");
	
	return nonseekable_open(inode, file);
}

static long broadcast_tdmb_ioctl_control(struct file *filep, unsigned int cmd,	unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	
	switch (cmd) 
	{
	case LGE_BROADCAST_TDMB_IOCTL_ON:
		rc = broadcast_tdmb_power_on();
		printk("LGE_BROADCAST_TDMB_IOCTL_ON OK %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_OFF:		
		rc = broadcast_tdmb_power_off();
		printk("LGE_BROADCAST_TDMB_IOCTL_OFF OK %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_OPEN:
		rc = broadcast_tdmb_open();
		printk("LGE_BROADCAST_TDMB_IOCTL_OPEN OK %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_CLOSE:
		broadcast_tdmb_close();
		printk("LGE_BROADCAST_TDMB_IOCTL_CLOSE OK \n");
		rc = 0;
		break;
	case LGE_BROADCAST_TDMB_IOCTL_TUNE:
		//rc = broadcast_tdmb_set_channel(argp);
		rc =0;
		printk("LGE_BROADCAST_TDMB_IOCTL_TUNE result = %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_SET_CH:
		rc = broadcast_tdmb_set_channel(argp);
		printk("LGE_BROADCAST_TDMB_IOCTL_SET_CH result = %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_RESYNC:
		rc = broadcast_tdmb_resync(argp);
		printk("LGE_BROADCAST_TDMB_IOCTL_RESYNC result = %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_DETECT_SYNC:
		rc = broadcast_tdmb_detect_sync(argp);
		printk("LGE_BROADCAST_TDMB_IOCTL_DETECT_SYNC result = %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_GET_SIG_INFO:
		rc = broadcast_tdmb_get_sig_info(argp);
		//printk("LGE_BROADCAST_TDMB_IOCTL_GET_SIG_INFO result = %d \n", rc);
		break;
	case LGE_BROADCAST_TDMB_IOCTL_GET_CH_INFO:
		rc = broadcast_tdmb_get_ch_info(argp);
		//printk("LGE_BROADCAST_TDMB_IOCTL_GET_CH_INFO result = %d \n", rc);
		break;

	case LGE_BROADCAST_TDMB_IOCTL_RESET_CH:
		rc = broadcast_tdmb_reset_ch();
		printk("LGE_BROADCAST_TDMB_IOCTL_RESET_CH result = %d \n", rc);
		break;
		
	case LGE_BROADCAST_TDMB_IOCTL_USER_STOP:
		rc = broadcast_tdmb_user_stop(argp);
		printk("LGE_BROADCAST_TDMB_IOCTL_USER_STOP !!! \n");
		break;

	case LGE_BROADCAST_TDMB_IOCTL_GET_DMB_DATA:
		rc = broadcast_tdmb_get_dmb_data(argp);
		//printk("LGE_BROADCAST_TDMB_IOCTL_GET_DMB_DATA TBD... !!! \n");
		break;

	case LGE_BROADCAST_TDMB_IOCTL_SELECT_ANTENNA:
		rc = broadcast_tdmb_select_antenna(argp);
		break;
	
	default:
		printk("broadcast_tdmb_ioctl_control OK \n");
		rc = -EINVAL;
		break;
	}

	return rc;
}

static ssize_t broadcast_tdmb_release_control(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations broadcast_tdmb_fops_control = 
{
	.owner = THIS_MODULE,
	.open = broadcast_tdmb_open_control,
	.unlocked_ioctl = broadcast_tdmb_ioctl_control,
	.release = broadcast_tdmb_release_control,
};

static int broadcast_tdmb_device_init(struct broadcast_tdmb_chdevice *pbroadcast, int index)
{
	int rc;

	cdev_init(&pbroadcast->cdev, &broadcast_tdmb_fops_control);

	pbroadcast->cdev.owner = THIS_MODULE;

	rc = cdev_add(&pbroadcast->cdev, broadcast_tdmb_dev, 1);

	pbroadcast->dev = device_create(broadcast_tdmb_class, NULL, MKDEV(MAJOR(broadcast_tdmb_dev), 0),
					 NULL, "broadcast%d", index);

	printk("broadcast_tdmb_device_add add add%d broadcast_tdmb_dev = %d \n", rc, MKDEV(MAJOR(broadcast_tdmb_dev), 0));
	
	if (IS_ERR(pbroadcast->dev)) {
		rc = PTR_ERR(pbroadcast->dev);
		pr_err("device_create failed: %d\n", rc);
		rc = -1;
	}
	
	printk("broadcast_tdmb_device_init start %d\n", rc);

	return rc;
}


int8 broadcast_tdmb_blt_power_on(void)
{
	int rc = ERROR;
		
	rc = broadcast_drv_if_power_on();

	broadcast_drv_if_user_stop();

	return rc;

}
EXPORT_SYMBOL(broadcast_tdmb_blt_power_on);

int8 broadcast_tdmb_blt_power_off(void)
{
	int rc = ERROR;
	
	rc = broadcast_drv_if_power_off();

	broadcast_drv_if_user_stop();

	return rc;

}
EXPORT_SYMBOL(broadcast_tdmb_blt_power_off);

int8 broadcast_tdmb_blt_open(void)
{
	int rc = ERROR;
		
	rc = broadcast_drv_if_open();

	return rc;
}
EXPORT_SYMBOL(broadcast_tdmb_blt_open);

int8 broadcast_tdmb_blt_close(void)
{
	int rc = ERROR;

	rc = broadcast_drv_if_close();
	
	return rc;
}
EXPORT_SYMBOL(broadcast_tdmb_blt_close);

int8 broadcast_tdmb_blt_tune_set_ch(int32 freq_num)
{
	int8 rc = ERROR;
	int32 freq_number = freq_num;
	uint8 sub_ch_id = 0;
	uint8 op_mode = 2;

	rc = broadcast_drv_if_set_channel(freq_number, sub_ch_id, op_mode);

	return rc;
}
EXPORT_SYMBOL(broadcast_tdmb_blt_tune_set_ch);

int8 broadcast_tdmb_blt_get_sig_info(void* sig_info)
{
	int rc = ERROR;
	struct broadcast_tdmb_sig_info udata;

	if(sig_info == NULL)
	{
		return rc;
	}
	memset((void*)&udata, 0x00, sizeof(struct broadcast_tdmb_sig_info));

	rc = broadcast_drv_if_get_sig_info(&udata);

	memcpy(sig_info, (void*)&udata, sizeof(struct broadcast_tdmb_sig_info));

	return rc;
}


int broadcast_tdmb_drv_start(void)
{
//	struct broadcast_tdmb_chdevice *pbroadcast = NULL;
	int rc = -ENODEV;
	
	if (!broadcast_tdmb_class) {

		broadcast_tdmb_class = class_create(THIS_MODULE, "broadcast_tdmb");
		if (IS_ERR(broadcast_tdmb_class)) {
			rc = PTR_ERR(broadcast_tdmb_class);
			pr_err("broadcast_tdmb_class: create device class failed: %d\n",
				rc);
			return rc;
		}

		rc = alloc_chrdev_region(&broadcast_tdmb_dev, 0, BROADCAST_TDMB_NUM_DEVS, "broadcast_tdmb");
		printk("broadcast_tdmb_drv_start add add%d broadcast_tdmb_dev = %d \n", rc, broadcast_tdmb_dev);
		if (rc < 0) {
			pr_err("broadcast_class: failed to allocate chrdev: %d\n",
				rc);
			return rc;
		}
	}

	pbroadcast = &tdmb_dev;
	
	rc = broadcast_tdmb_device_init(pbroadcast, 0);
	if (rc < 0) {
		return rc;
	}
	
	printk("broadcast_tdmb_drv_start start %d\n", rc);

	return rc;
}

EXPORT_SYMBOL(broadcast_tdmb_drv_start);

int broadcast_tdmb_get_stop_mode(void)
{
	return 0;
}

EXPORT_SYMBOL(broadcast_tdmb_get_stop_mode);

