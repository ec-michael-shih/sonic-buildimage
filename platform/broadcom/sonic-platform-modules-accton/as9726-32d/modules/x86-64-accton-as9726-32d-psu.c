/*
 * An hwmon driver for accton as9726_32d Power Module
 *
 * Copyright (C) 2014 Accton Technology Corporation.
 * Brandon Chuang <brandon_chuang@accton.com.tw>
 *
 * Based on ad7414.c
 * Copyright 2006 Stefan Roese <sr at denx.de>, DENX Software Engineering
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/dmi.h>
#include "accton_psu_api.h"

#define MAX_MODEL_NAME          12
#define MAX_SERIAL_NUMBER       19

static ssize_t show_status(struct device *dev, struct device_attribute *da, char *buf);
static ssize_t show_string(struct device *dev, struct device_attribute *da, char *buf);
static int as9726_32d_psu_read_block(struct i2c_client *client, u8 command, u8 *data,int data_len);
extern int as9726_32d_cpld_read(unsigned short cpld_addr, u8 reg);

/* Addresses scanned
 */
static const unsigned short normal_i2c[] = { 0x50, 0x51, I2C_CLIENT_END };

/* Each client has this additional data
 */
struct as9726_32d_psu_data {
    struct device      *hwmon_dev;
    struct mutex        update_lock;
    char                valid;           /* !=0 if registers are valid */
    unsigned long       last_updated;    /* In jiffies */
    u8  index;           /* PSU index */
    u8  status;          /* Status(present/power_good) register read from CPLD */
    char model_name[MAX_MODEL_NAME]; /* Model name, read from eeprom */
    char serial_number[MAX_SERIAL_NUMBER];
};

static struct as9726_32d_psu_data *as9726_32d_psu_update_device(struct device *dev);

enum as9726_32d_psu_sysfs_attributes {
    PSU_PRESENT,
    PSU_MODEL_NAME,
    PSU_POWER_GOOD,
    PSU_SERIAL_NUMBER
};

/* sysfs attributes for hwmon
 */
static SENSOR_DEVICE_ATTR(psu_present,    S_IRUGO, show_status,    NULL, PSU_PRESENT);
static SENSOR_DEVICE_ATTR(psu_model_name, S_IRUGO, show_string,    NULL, PSU_MODEL_NAME);
static SENSOR_DEVICE_ATTR(psu_power_good, S_IRUGO, show_status,    NULL, PSU_POWER_GOOD);
static SENSOR_DEVICE_ATTR(psu_serial_number, S_IRUGO, show_string, NULL, PSU_SERIAL_NUMBER);


static struct attribute *as9726_32d_psu_attributes[] = {
    &sensor_dev_attr_psu_present.dev_attr.attr,
    &sensor_dev_attr_psu_model_name.dev_attr.attr,
    &sensor_dev_attr_psu_power_good.dev_attr.attr,
    &sensor_dev_attr_psu_serial_number.dev_attr.attr,
    NULL
};

static ssize_t show_status(struct device *dev, struct device_attribute *da,
                           char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct as9726_32d_psu_data *data = as9726_32d_psu_update_device(dev);
    u8 status = 0;

    //printk("data->status=0x%x, attr->index=%d,data->index=%d \n", data->status, attr->index, data->index);
    if (attr->index == PSU_PRESENT) {
        if(data->index==0)
            status = !( (data->status >> 1) & 0x1);
        else
            status = !( (data->status) & 0x1);
    }
    else { /* PSU_POWER_GOOD */
        if(data->index==0)
           status = ( (data->status >> 3) & 0x1);
        else
           status = ( (data->status >> 2) & 0x1);
    }

    return sprintf(buf, "%d\n", status);
}

static ssize_t show_string(struct device *dev, struct device_attribute *da,
                               char *buf)
{
   struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct as9726_32d_psu_data *data = as9726_32d_psu_update_device(dev);
    char *ptr = NULL;

    if (!data->valid) {
        return -EIO;
    }

	switch (attr->index) {
	case PSU_MODEL_NAME:
		ptr = data->model_name;
		break;
	case PSU_SERIAL_NUMBER:
		ptr = data->serial_number;
		break;
	default:
		return -EINVAL;
	}

    return sprintf(buf, "%s\n", ptr);
}

static const struct attribute_group as9726_32d_psu_group = {
    .attrs = as9726_32d_psu_attributes,
};

static int as9726_32d_psu_probe(struct i2c_client *client,
                                const struct i2c_device_id *dev_id)
{
    struct as9726_32d_psu_data *data;
    int status;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
        status = -EIO;
        goto exit;
    }

    data = kzalloc(sizeof(struct as9726_32d_psu_data), GFP_KERNEL);
    if (!data) {
        status = -ENOMEM;
        goto exit;
    }

    i2c_set_clientdata(client, data);
    data->valid = 0;
    data->index = dev_id->driver_data;
    mutex_init(&data->update_lock);

    dev_info(&client->dev, "chip found\n");

    /* Register sysfs hooks */
    status = sysfs_create_group(&client->dev.kobj, &as9726_32d_psu_group);
    if (status) {
        goto exit_free;
    }

    data->hwmon_dev = hwmon_device_register(&client->dev);
    if (IS_ERR(data->hwmon_dev)) {
        status = PTR_ERR(data->hwmon_dev);
        goto exit_remove;
    }

    dev_info(&client->dev, "%s: psu '%s'\n",
             dev_name(data->hwmon_dev), client->name);

    return 0;

exit_remove:
    sysfs_remove_group(&client->dev.kobj, &as9726_32d_psu_group);
exit_free:
    kfree(data);
exit:

    return status;
}

static void as9726_32d_psu_remove(struct i2c_client *client)
{
    struct as9726_32d_psu_data *data = i2c_get_clientdata(client);

    hwmon_device_unregister(data->hwmon_dev);
    sysfs_remove_group(&client->dev.kobj, &as9726_32d_psu_group);
    kfree(data);

}

enum psu_index
{
    as9726_32d_psu1,
    as9726_32d_psu2
};

static const struct i2c_device_id as9726_32d_psu_id[] = {
    { "as9726_32d_psu1", as9726_32d_psu1 },
    { "as9726_32d_psu2", as9726_32d_psu2 },
    {}
};
MODULE_DEVICE_TABLE(i2c, as9726_32d_psu_id);

static struct i2c_driver as9726_32d_psu_driver = {
    .class        = I2C_CLASS_HWMON,
    .driver = {
        .name     = "as9726_32d_psu",
    },
    .probe        = as9726_32d_psu_probe,
    .remove       = as9726_32d_psu_remove,
    .id_table     = as9726_32d_psu_id,
    .address_list = normal_i2c,
};

static int as9726_32d_psu_read_block(struct i2c_client *client, u8 command, u8 *data,
                                     int data_len)
{
    int result = 0;
    int retry_count = 5;

    while (retry_count) {
        retry_count--;

        result = i2c_smbus_read_i2c_block_data(client, command, data_len, data);

        if (unlikely(result < 0)) {
            msleep(10);
            continue;
        }

        if (unlikely(result != data_len)) {
            result = -EIO;
            msleep(10);
            continue;
        }

        result = 0;
        break;
    }

    return result;
}

static struct as9726_32d_psu_data *as9726_32d_psu_update_device(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct as9726_32d_psu_data *data = i2c_get_clientdata(client);

    mutex_lock(&data->update_lock);

    if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
            || !data->valid) {
        int status;
        int psu_present = 0;

        dev_dbg(&client->dev, "Starting as9726_32d update\n");

        /* Read psu status */
        status = as9726_32d_cpld_read(0x60, 0x03);
        //printk("status=0x%x in %s\n", status, __FUNCTION__);

        if (status < 0) {
            dev_dbg(&client->dev, "cpld reg 0x60 err %d\n", status);
        }
        else {
            data->status = status;
        }

        /* Read model name */
        memset(data->model_name, 0, sizeof(data->model_name));
        memset(data->serial_number, 0, sizeof(data->serial_number));
        psu_present = (data->status >> (1 - data->index) & 0x1); //0:present, 1:not present
       
        if (!psu_present) {
            status = as9726_32d_psu_read_block(client, 0x21, data->model_name,
                                               ARRAY_SIZE(data->model_name)-1);
            if (status < 0) {
                data->model_name[0] = '\0';
                dev_dbg(&client->dev, "unable to read model name from (0x%x)\n", client->addr);
                printk("unable to read model name from (0x%x)\n", client->addr);
            }
            else {
                data->model_name[ARRAY_SIZE(data->model_name)-1] = '\0';
                
            }
             /* Read from offset 0x35 ~ 0x46 (18 bytes) */
            status = as9726_32d_psu_read_block(client, 0x3A,data->serial_number,
                                               ARRAY_SIZE(data->serial_number)-1);
            if (status < 0)
            {
                data->serial_number[0] = '\0';
                dev_dbg(&client->dev, "unable to read model name from (0x%x) offset(0x2e)\n", client->addr);
                printk("unable to read model name from (0x%x) offset(0x2e)\n", client->addr);
            }
            data->serial_number[ARRAY_SIZE(data->serial_number)-1]='\0';
        }

        data->last_updated = jiffies;
        data->valid = 1;
    }

    mutex_unlock(&data->update_lock);

    return data;
}

int as9726_32d_psu_get_presence(void *client_ptr)
{
    int status = 0;
    int psu_index = 0;
    int psu_present = 0;
    struct i2c_client *client = NULL;

    if (!client_ptr)
        return -EINVAL;

    client = client_ptr;
    status = as9726_32d_cpld_read(0x60, 0x03);
    if (status < 0) {
        dev_dbg(&client->dev, "cpld reg 0x60 err %d\n", status);
        return 0;
    }

    psu_index = (client->addr == 0x58) ? as9726_32d_psu1 : as9726_32d_psu2;
    if (psu_index == as9726_32d_psu1) {
        psu_present = !((status >> 1) & 0x1);
    } else {
        psu_present = !((status) & 0x1);
    }

    return psu_present;
}

int as9726_32d_psu_get_powergood(void *client_ptr)
{
    int status = 0;
    int psu_index = 0;
    int psu_powergood = 0;
    struct i2c_client *client = NULL;

    if (!client_ptr)
        return -EINVAL;

    client = client_ptr;
    status = as9726_32d_cpld_read(0x60, 0x03);
    if (status < 0) {
        dev_dbg(&client->dev, "cpld reg 0x60 err %d\n", status);
        return 0;
    }

    psu_index = (client->addr == 0x58) ? as9726_32d_psu1 : as9726_32d_psu2;
    if (psu_index == as9726_32d_psu1) {
        psu_powergood = ((status >> 3) & 0x1);
    } else {
        psu_powergood = ((status >> 2) & 0x1);
    }

    return psu_powergood;
}

static int __init as9726_32d_psu_init(void)
{
    PSU_STATUS_ENTRY access_psu_status = {
        as9726_32d_psu_get_presence,
        as9726_32d_psu_get_powergood
    };
    register_psu_status_entry(&access_psu_status);

    return i2c_add_driver(&as9726_32d_psu_driver);
}

static void __exit as9726_32d_psu_exit(void)
{
    register_psu_status_entry(NULL);
    i2c_del_driver(&as9726_32d_psu_driver);
}


MODULE_AUTHOR("Michael Shih <michael_shih@edge-core.com>");
MODULE_DESCRIPTION("as9726_32d_psu driver");
MODULE_LICENSE("GPL");

module_init(as9726_32d_psu_init);
module_exit(as9726_32d_psu_exit);
