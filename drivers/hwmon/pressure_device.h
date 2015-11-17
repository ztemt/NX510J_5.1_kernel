/*
 *
 * FocalTech ft5x06 TouchScreen driver header file.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __LINUX_PRESSURE_H__
#define __LINUX_PRESSURE_H__

#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/workqueue.h>


#define FW_NAME_MAX_LEN 30

enum HOST_STATUS{
		NORMAL = 0,
		STANDBY,
		POWEROFF,
		STARTUP,
		numStates,
};

struct pressure_fw_info {
	u16 manufactor_id;
	u16 module_id;
	u16 fw_version;
};

struct pressure_sensor_data {
	struct i2c_client *client;
	struct device *pressure_dev;
	struct mutex lock;

	char fw_name[FW_NAME_MAX_LEN];
	int reset_pin;
	int fw_size;
	int page_size;
	struct pressure_fw_info fw_info;

	struct delayed_work debug_work;
	bool loading_fw;
	u8 sensitivity;
	bool suspended;
};

static int pressure_sensor_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int pressure_sensor_remove(struct i2c_client *client);
static int pressure_sensor_resume(struct device *dev);
static int pressure_sensor_suspend(struct device *dev);

static int pressure_fw_upgrade(struct device *dev, bool force);
static int pressure_fw_write(struct pressure_sensor_data *pdata, const char *buf,int count);
static int pressure_fw_check(struct pressure_sensor_data *pdata, const char *fw_buf,int length, bool flag);
static int pressure_eeprom_read(struct pressure_sensor_data *pdata, u8 *buf, unsigned offset, int count);
static int pressure_eeprom_write(struct pressure_sensor_data *pdata, const char *buf, unsigned offset, int count, u8 *write_buf);
static int pressure_read_reg(struct pressure_sensor_data *pdata, u16 reg ,void *val, u16 length);
static int pressure_write_reg(struct pressure_sensor_data *pdata ,u16 addr, void *buf, u16 length);
static bool perssure_need_upgrade_flag(const struct firmware *fw,struct pressure_sensor_data *pdata);
static int _pressure_read_reg(struct i2c_client *client, u16 reg, u8 *val, u16 len);
static int _pressure_write_reg(struct i2c_client *client, u16 addr, u8 *value, u16 length);
static int pressure_device_reset_init(struct pressure_sensor_data *pdata, bool on);
static int pressure_device_reset_on(struct pressure_sensor_data *pdata, bool on);
static int prepare_fw_upgrade_work(struct pressure_sensor_data *pdata, bool on);
static int pressure_host_status_report(struct pressure_sensor_data *pdata, enum HOST_STATUS status);
static int pressure_slave_ic_reset(struct pressure_sensor_data *pdata);
static int pressure_file_write(char *file_path, const char *write_buf ,int count);
static int pressure_ic_info_read(struct pressure_sensor_data *pdata);

static ssize_t pressure_update_fw_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t pressure_update_fw_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size);
static ssize_t pressure_force_update_fw_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size);
static ssize_t pressure_fw_name_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t pressure_fw_name_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size);
static ssize_t pressure_fw_info_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t pressure_key_value_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t pressure_sensitivity_attr_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size);
static ssize_t pressure_sensitivity_attr_show(struct device *dev,struct device_attribute *attr, char *buf);

#endif
