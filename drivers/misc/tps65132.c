/*
 * fsa8069.c -- FSA8069 Jack detection driver
 * 
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>


/******************************************************************************
* Register addresses	
******************************************************************************/
#define REG_VPOS				0x00
#define REG_VNEG				0x01
#define REG_DISP				0x03
#define REG_CONTROL				0xFF

/**********************************************/
#define TRUE 1
#define FALSE 0



struct tps_info {
	struct i2c_client		*client;
	struct mutex		mutex;
	struct class *tps_class;
    int gpio_switch ;
    int gpio_enable;
};

static struct i2c_client *this_client;

char tps_regs[] = {
    REG_VPOS,
    REG_VNEG,
    REG_DISP,
    REG_CONTROL,
};
ssize_t tps_show_regs(struct device *dev,struct device_attribute *attr,char *buf)
{
    int ret;
    struct tps_info *info = dev_get_drvdata(dev);
    struct i2c_client *client = info->client;
    int i =0;
    for(;i<sizeof(tps_regs);i++) {
        ret = i2c_smbus_read_byte_data(client, tps_regs[i]);
        if(ret < 0 ) 
        {
            printk("read data error %d\n",ret);
            break;
        }
        printk("0x%0x,-> 0x%x\n",tps_regs[i],ret);
    }
    return sprintf(buf, "read ok\n");
}


ssize_t tps_store_regs(struct device *dev,struct device_attribute *attr,
			 const char *buf, size_t count)
{
    bool value;
    struct tps_info *info = dev_get_drvdata(dev);
    if(strtobool(buf,&value))
        return -EINVAL;
    gpio_direction_output(info->gpio_enable,((value > 0) ? 1: 0));
    gpio_direction_output(info->gpio_switch,((value > 0) ? 1: 0));
    printk("tps_store_regs:  info->gpio_switch  value %d, info->gpio_enable value %d\n",
            gpio_get_value(info->gpio_switch),gpio_get_value(info->gpio_enable));
    return count;
}


DEVICE_ATTR(tps_regs,0664,tps_show_regs,tps_store_regs);

static void tps_initialization(struct tps_info *info)
{
//	struct i2c_client *client = info->client;
    int ret = 0;
//    ret = i2c_smbus_write_byte_data(client,REG_JDET_TIME,0xF9);
    printk("i2c_smbus_write_byte_data return value is %d\n",ret);
}

static int tps_probe(struct i2c_client *client,const struct i2c_device_id *id)
{

	int ret = 0;
	struct tps_info *info;

    info = kzalloc(sizeof(struct tps_info), GFP_KERNEL);
	info->client = client;
	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);

    printk("tps_probe ==========\n");

	this_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk("%s: i2c check functionality error\n", __func__);
		ret = -ENODEV;
		goto check_funcionality_failed;
	}
	device_create_file(&client->dev, &dev_attr_tps_regs);

	/* GPIO setting */

	info->gpio_switch = of_get_named_gpio(client->dev.of_node, "tps-gpio-switch", 0);	
	info->gpio_enable = of_get_named_gpio(client->dev.of_node, "tps-gpio-enable", 0);	
    gpio_request(info->gpio_switch,"tps-gpio-switch");
    gpio_request(info->gpio_enable,"tps-gpio-enable");
    gpio_direction_output(info->gpio_enable,0);
    gpio_direction_output(info->gpio_switch,0);
    printk("info->gpio_switch  value %d, info->gpio_enable value %d\n",
            gpio_get_value(info->gpio_switch),gpio_get_value(info->gpio_enable));
	tps_initialization(info);

	return 0;

check_funcionality_failed:

	return ret;	

}
static int tps_remove(struct i2c_client *client)
{
    struct tps_info *info = i2c_get_clientdata(client);
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
	
	kfree(info);
	
	return 0;
}

static int  tps_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int  tps_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tps_i2c_id[] = {
	{ "tps-i2c", 0 },
	{ }
};

static const struct of_device_id of_tps_device_idtable[] = {
    { .compatible = "tps-i2c",0},
	{ },
};


 
static struct i2c_driver tps_i2c_driver = {
	.driver = {
		.name = "tps",
		.owner = THIS_MODULE,			
        .of_match_table = of_tps_device_idtable,
	},
	.probe    = tps_probe,
	.remove   = tps_remove,
	.suspend  = tps_suspend,
	.resume	  = tps_resume,
	.id_table = tps_i2c_id,
};

static __init int tps_i2c_init(void)
{

    printk("tps_i2c_init ========= \n");
	return i2c_add_driver(&tps_i2c_driver);
}

static __exit void tps_i2c_exit(void)
{
	i2c_del_driver(&tps_i2c_driver);
}

module_init(tps_i2c_init);
module_exit(tps_i2c_exit);

MODULE_AUTHOR(" ");
MODULE_DESCRIPTION("I2C bus driver for TPS65132 ");
MODULE_LICENSE("GPL v2");

