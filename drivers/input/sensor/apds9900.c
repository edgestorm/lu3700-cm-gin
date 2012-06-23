/*
 *  apds9900.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifdef CONFIG_MACH_LGE
#define LGE_APDS9900
#endif

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#ifdef LGE_APDS9900
#include <mach/board_lge.h>
#endif

#define APDS9900_DRV_NAME	"apds9900"
#define DRIVER_VERSION		"1.0.0"

/*
 * Defines
 */

#define APDS9900_ENABLE_REG	0x00
#define APDS9900_ATIME_REG	0x01
#define APDS9900_PTIME_REG	0x02
#define APDS9900_WTIME_REG	0x03
#define APDS9900_AILTL_REG	0x04
#define APDS9900_AILTH_REG	0x05
#define APDS9900_AIHTL_REG	0x06
#define APDS9900_AIHTH_REG	0x07
#define APDS9900_PILTL_REG	0x08
#define APDS9900_PILTH_REG	0x09
#define APDS9900_PIHTL_REG	0x0A
#define APDS9900_PIHTH_REG	0x0B
#define APDS9900_PERS_REG	0x0C
#define APDS9900_CONFIG_REG	0x0D
#define APDS9900_PPCOUNT_REG	0x0E
#define APDS9900_CONTROL_REG	0x0F
#define APDS9900_REV_REG	0x11
#define APDS9900_ID_REG		0x12
#define APDS9900_STATUS_REG	0x13
#define APDS9900_CDATAL_REG	0x14
#define APDS9900_CDATAH_REG	0x15
#define APDS9900_IRDATAL_REG	0x16
#define APDS9900_IRDATAH_REG	0x17
#define APDS9900_PDATAL_REG	0x18
#define APDS9900_PDATAH_REG	0x19

#define CMD_BYTE	0x80
#define CMD_WORD	0xA0
#define CMD_SPECIAL	0xE0

#define CMD_CLR_PS_INT	0xE5
#define CMD_CLR_ALS_INT	0xE6
#define CMD_CLR_PS_ALS_INT	0xE7

/*
 * Structs
 */

struct apds9900_data {
	struct i2c_client *client;
	struct mutex update_lock;

	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;
};

/*
 * Global data
 */

/*
 * Management functions
 */

static int apds9900_set_command(struct i2c_client *client, int command)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	int clearInt;

	if (command == 0)
		clearInt = CMD_CLR_PS_INT;
	else if (command == 1)
		clearInt = CMD_CLR_ALS_INT;
	else
		clearInt = CMD_CLR_PS_ALS_INT;
		
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte(client, clearInt);
	mutex_unlock(&data->update_lock);

	return ret;
}

static int apds9900_set_enable(struct i2c_client *client, int enable)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9900_ENABLE_REG, enable);
	mutex_unlock(&data->update_lock);

	data->enable = enable;

	return ret;
}

static int apds9900_set_atime(struct i2c_client *client, int atime)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9900_ATIME_REG, atime);
	mutex_unlock(&data->update_lock);

	data->atime = atime;

	return ret;
}

static int apds9900_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9900_PTIME_REG, ptime);
	mutex_unlock(&data->update_lock);

	data->ptime = ptime;

	return ret;
}

static int apds9900_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9900_WTIME_REG, wtime);
	mutex_unlock(&data->update_lock);

	data->wtime = wtime;

	return ret;
}

static int apds9900_set_ailt(struct i2c_client *client, int threshold)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9900_AILTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->ailt = threshold;

	return ret;
}

static int apds9900_set_aiht(struct i2c_client *client, int threshold)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9900_AIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->aiht = threshold;

	return ret;
}

static int apds9900_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9900_PILTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->ailt = threshold;

	return ret;
}

static int apds9900_set_piht(struct i2c_client *client, int threshold)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9900_PIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->aiht = threshold;

	return ret;
}

static int apds9900_set_pers(struct i2c_client *client, int pers)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9900_PERS_REG, pers);
	mutex_unlock(&data->update_lock);

	data->pers = pers;

	return ret;
}

static int apds9900_set_config(struct i2c_client *client, int config)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9900_CONFIG_REG, config);
	mutex_unlock(&data->update_lock);

	data->config = config;

	return ret;
}

static int apds9900_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9900_PPCOUNT_REG, ppcount);
	mutex_unlock(&data->update_lock);

	data->ppcount = ppcount;

	return ret;
}

static int apds9900_set_control(struct i2c_client *client, int control)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9900_CONTROL_REG, control);
	mutex_unlock(&data->update_lock);

	data->control = control;

	return ret;
}

/*
 * SysFS support
 */

static ssize_t apds9900_store_command(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	if (val < 0 || val > 2)
		return -EINVAL;

	ret = apds9900_set_command(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(command, S_IWUSR,
		   NULL, apds9900_store_command);

static ssize_t apds9900_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable);
}

static ssize_t apds9900_store_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9900_set_enable(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		   apds9900_show_enable, apds9900_store_enable);

static ssize_t apds9900_show_atime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->atime);
}

static ssize_t apds9900_store_atime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9900_set_atime(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(atime, S_IWUSR | S_IRUGO,
		   apds9900_show_atime, apds9900_store_atime);

static ssize_t apds9900_show_ptime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ptime);
}

static ssize_t apds9900_store_ptime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9900_set_ptime(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(ptime, S_IWUSR | S_IRUGO,
		   apds9900_show_ptime, apds9900_store_ptime);

static ssize_t apds9900_show_wtime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->wtime);
}

static ssize_t apds9900_store_wtime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9900_set_wtime(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(wtime, S_IWUSR | S_IRUGO,
		   apds9900_show_wtime, apds9900_store_wtime);

static ssize_t apds9900_show_ailt(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ailt);
}

static ssize_t apds9900_store_ailt(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9900_set_ailt(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(ailt, S_IWUSR | S_IRUGO,
		   apds9900_show_ailt, apds9900_store_ailt);

static ssize_t apds9900_show_aiht(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->aiht);
}

static ssize_t apds9900_store_aiht(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9900_set_aiht(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(aiht, S_IWUSR | S_IRUGO,
		   apds9900_show_aiht, apds9900_store_aiht);

static ssize_t apds9900_show_pilt(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->pilt);
}

static ssize_t apds9900_store_pilt(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9900_set_pilt(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(pilt, S_IWUSR | S_IRUGO,
		   apds9900_show_pilt, apds9900_store_pilt);

static ssize_t apds9900_show_piht(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->piht);
}

static ssize_t apds9900_store_piht(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9900_set_piht(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(piht, S_IWUSR | S_IRUGO,
		   apds9900_show_piht, apds9900_store_piht);

static ssize_t apds9900_show_pers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->pers);
}

static ssize_t apds9900_store_pers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9900_set_pers(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(pers, S_IWUSR | S_IRUGO,
		   apds9900_show_pers, apds9900_store_pers);

static ssize_t apds9900_show_config(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->config);
}

static ssize_t apds9900_store_config(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9900_set_config(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(config, S_IWUSR | S_IRUGO,
		   apds9900_show_config, apds9900_store_config);

static ssize_t apds9900_show_ppcount(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ppcount);
}

static ssize_t apds9900_store_ppcount(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9900_set_ppcount(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(ppcount, S_IWUSR | S_IRUGO,
		   apds9900_show_ppcount, apds9900_store_ppcount);

static ssize_t apds9900_show_control(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->control);
}

static ssize_t apds9900_store_control(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9900_set_control(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(control, S_IWUSR | S_IRUGO,
		   apds9900_show_control, apds9900_store_control);

static ssize_t apds9900_show_rev(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	int rev;

	mutex_lock(&data->update_lock);
	rev = i2c_smbus_read_byte_data(client, APDS9900_REV_REG);
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", rev);
}

static DEVICE_ATTR(rev, S_IRUGO,
		   apds9900_show_rev, NULL);

static ssize_t apds9900_show_id(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	int id;

	mutex_lock(&data->update_lock);
	id = i2c_smbus_read_byte_data(client, APDS9900_ID_REG);
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", id);
}

static DEVICE_ATTR(id, S_IRUGO,
		   apds9900_show_id, NULL);

static ssize_t apds9900_show_status(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	int status;

	mutex_lock(&data->update_lock);
	status = i2c_smbus_read_byte_data(client, APDS9900_STATUS_REG);
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", status);
}

static DEVICE_ATTR(status, S_IRUGO,
		   apds9900_show_status, NULL);

static ssize_t apds9900_show_cdata(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	int cdata;

	mutex_lock(&data->update_lock);
	cdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9900_CDATAL_REG);
	mutex_unlock(&data->update_lock);
	
	return sprintf(buf, "%d\n", cdata);
}

static DEVICE_ATTR(cdata, S_IRUGO,
		   apds9900_show_cdata, NULL);

static ssize_t apds9900_show_irdata(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	int irdata;

	mutex_lock(&data->update_lock);
	irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9900_IRDATAL_REG);
	mutex_unlock(&data->update_lock);
	
	return sprintf(buf, "%d\n", irdata);
}

static DEVICE_ATTR(irdata, S_IRUGO,
		   apds9900_show_irdata, NULL);

static ssize_t apds9900_show_pdata(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	int pdata;

	mutex_lock(&data->update_lock);
	pdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9900_PDATAL_REG);
	mutex_unlock(&data->update_lock);
	
	return sprintf(buf, "%d\n", pdata);
}

static DEVICE_ATTR(pdata, S_IRUGO,
		   apds9900_show_pdata, NULL);

static struct attribute *apds9900_attributes[] = {
	&dev_attr_command.attr,
	&dev_attr_enable.attr,
	&dev_attr_atime.attr,
	&dev_attr_ptime.attr,
	&dev_attr_wtime.attr,
	&dev_attr_ailt.attr,
	&dev_attr_aiht.attr,
	&dev_attr_pilt.attr,
	&dev_attr_piht.attr,
	&dev_attr_pers.attr,
	&dev_attr_config.attr,
	&dev_attr_ppcount.attr,
	&dev_attr_control.attr,
	&dev_attr_rev.attr,
	&dev_attr_id.attr,
	&dev_attr_status.attr,
	&dev_attr_cdata.attr,
	&dev_attr_irdata.attr,
	&dev_attr_pdata.attr,
	NULL
};

static const struct attribute_group apds9900_attr_group = {
	.attrs = apds9900_attributes,
};

/*
 * Initialization function
 */

static int apds9900_init_client(struct i2c_client *client)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int err;

	err = apds9900_set_enable(client, 0);

	if (err < 0)
		return err;

	mdelay(1);

	mutex_lock(&data->update_lock);
	err = i2c_smbus_read_byte_data(client, APDS9900_ENABLE_REG);
	mutex_unlock(&data->update_lock);

	if (err != 0)
		return -ENODEV;

	data->enable = 0;

	return 0;
}

/*
 * I2C init/probing/exit functions
 */

static struct i2c_driver apds9900_driver;
static int __devinit apds9900_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds9900_data *data;
	int err = 0;
#ifdef LGE_APDS9900
	struct proximity_platform_data	*pdata;
#endif

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct apds9900_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	data->client = client;
	i2c_set_clientdata(client, data);

#ifdef LGE_APDS9900
	pdata = data->client->dev.platform_data;
	pdata->power(1);
#endif

	data->enable = 0;	/* default mode is standard */
	dev_info(&client->dev, "enable = %s\n",
			data->enable ? "1" : "0");

	mutex_init(&data->update_lock);

	/* Initialize the APDS9900 chip */
	err = apds9900_init_client(client);
	if (err)
		goto exit_kfree;

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds9900_attr_group);
	if (err)
		goto exit_kfree;

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

exit_kfree:
	kfree(data);
exit:
	return err;
}

static int __devexit apds9900_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &apds9900_attr_group);

	/* Power down the device */
	apds9900_set_enable(client, 0);

	kfree(i2c_get_clientdata(client));

	return 0;
}

#ifdef CONFIG_PM

static int apds9900_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return apds9900_set_enable(client, 0);
}

static int apds9900_resume(struct i2c_client *client)
{
	return apds9900_set_enable(client, 0);
}

#else

#define apds9900_suspend	NULL
#define apds9900_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id apds9900_id[] = {
	{ "apds9900", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds9900_id);

static struct i2c_driver apds9900_driver = {
	.driver = {
		.name	= APDS9900_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = apds9900_suspend,
	.resume	= apds9900_resume,
	.probe	= apds9900_probe,
	.remove	= __devexit_p(apds9900_remove),
	.id_table = apds9900_id,
};

static int __init apds9900_init(void)
{
	return i2c_add_driver(&apds9900_driver);
}

static void __exit apds9900_exit(void)
{
	i2c_del_driver(&apds9900_driver);
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS9900 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds9900_init);
module_exit(apds9900_exit);
