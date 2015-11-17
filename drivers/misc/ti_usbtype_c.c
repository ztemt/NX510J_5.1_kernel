/*
*  ti_usbtype_c.c is linux kernel modules for ti usb type c chip
*   
 *  Copyright (C) 2015 nubia software department
 *  Xiaojun Xue <xue.xiaojun@zte.com.cn>
 *  Copyright (c) 2015-2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>


struct ti_usbtypec_chip {
    struct i2c_client *client;
    bool power_enable;
    struct pinctrl *usbtypec_pinctrl;
	struct pinctrl_state *gpio_power_active;
	struct pinctrl_state *gpio_power_supend;
};


static int ti_usb_read_reg(struct i2c_client *client, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
/*

static int ti_usb_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
*/

#ifdef CONFIG_OF
static int ti_usbtypec_parse_dt(struct device *dev,
			struct ti_usbtypec_chip *chip_data)
{

	chip_data->usbtypec_pinctrl = pinctrl_get(dev);
	if (IS_ERR(chip_data->usbtypec_pinctrl)) {
		dev_err(dev,"%s: Unable to get pinctrl handle\n", __func__);
		return -EINVAL;
	}
    
	/* get all the states handles from Device Tree */
	chip_data->gpio_power_active = pinctrl_lookup_state(chip_data->usbtypec_pinctrl, "active");
	if (IS_ERR(chip_data->gpio_power_active)) {
		dev_err(dev,"%s: Unable to get pinctrl enable state handle, err: %ld\n",
				__func__, PTR_ERR(chip_data->gpio_power_active));
		return -EINVAL;
	}
    
	chip_data->gpio_power_supend = pinctrl_lookup_state(chip_data->usbtypec_pinctrl, "suspend");
	if (IS_ERR(chip_data->gpio_power_supend)) {
		dev_err(dev,"%s: Unable to get pinctrl disable state handle, err: %ld\n",
				__func__, PTR_ERR(chip_data->gpio_power_supend));
		return -EINVAL;
	}
	return 0;
}
#else
static int ti_usbtypec_parse_dt(struct device *dev,
			struct ti_usbtypec_chip *pdata)
{
	return -ENODEV;
}
#endif

static int  ti_usbtypec_chip_poweron(struct ti_usbtypec_chip *ti_usbtypec_chip)
{
    int ret = 0;
    
    if(ti_usbtypec_chip->power_enable == true){
        
        /* Set powron pin on for active state */
        ret = pinctrl_select_state(ti_usbtypec_chip->usbtypec_pinctrl,
            ti_usbtypec_chip->gpio_power_active);
        if (ret != 0) {
            pr_err("%s: Failed to enable gpio pins\n",__func__);
            return -EIO;
        }
    }else{
      
        /* Set powron pin on for sleep state */
        ret = pinctrl_select_state(ti_usbtypec_chip->usbtypec_pinctrl,
            ti_usbtypec_chip->gpio_power_supend);
        if (ret != 0) {
            pr_err("%s: Failed to enable gpio pins\n",__func__);
            return -EIO;
        }
    } 
    return ret;
}

static int ti_usbtypec_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct ti_usbtypec_chip * usbtypec_data = NULL;
    int ret = 0;
     

    if(!i2c_check_functionality(client->adapter,
        I2C_FUNC_SMBUS_BYTE_DATA)){
        dev_err(&client->dev,"no support for i2c rw\n");
        return -EIO;
    }

    usbtypec_data = kzalloc(sizeof(struct ti_usbtypec_chip),GFP_KERNEL);

    if(!usbtypec_data){
		ret = -ENOMEM;
		goto mem_alloc_fail;
    }
    usbtypec_data->client = client;
    
    if (client->dev.of_node) {
        dev_info(&client->dev,"function:%s,client->dev.of_node\n",__func__);
		
		ret = ti_usbtypec_parse_dt(&client->dev, usbtypec_data);
		if (ret) {
			dev_err(&client->dev, "DT parsing failed\n");
			goto dt_parse_fail;
        }
    }
    
	i2c_set_clientdata(client, usbtypec_data);
    
    usbtypec_data->power_enable = true;
    ti_usbtypec_chip_poweron(usbtypec_data);

    
    printk(KERN_ERR"=chip id = 0x%x",ti_usb_read_reg(client,0x30));

    dev_info(&client->dev,"%s,probe successful!!!",__func__);

    return ret;

 dt_parse_fail:
    kfree(usbtypec_data);
 mem_alloc_fail:
    return ret;
}

static int ti_usbtypec_remove(struct i2c_client *client)
{
	struct ti_usbtypec_chip *typec_data = i2c_get_clientdata(client);

    typec_data->power_enable=false;
    ti_usbtypec_chip_poweron(typec_data);

    kfree(typec_data);
	return 0;

}
#ifdef CONFIG_PM
static int ti_usbtypec_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ti_usbtypec_chip *typec_data = i2c_get_clientdata(client);
	int ret = 0;
    
    /* Set powron pin on for sleep state */
    ret = pinctrl_select_state(typec_data->usbtypec_pinctrl,
        typec_data->gpio_power_supend);
    if (ret != 0) {
        pr_err("%s: Failed to enable gpio pins\n",__func__);
        return -EIO;
    }
    
    return ret;
}
static int ti_usbtypec_resume(struct i2c_client *client)
{
	struct ti_usbtypec_chip *typec_data = i2c_get_clientdata(client);
	int ret = 0;

    /* Set powron pin on for active state */
    ret = pinctrl_select_state(typec_data->usbtypec_pinctrl,
        typec_data->gpio_power_active);
    if (ret != 0) {
        pr_err("%s: Failed to enable gpio pins\n",__func__);
        return -EIO;
    }
    
    return ret;
}
#else
#define  ti_usbtypec_suspend NULL
#define  ti_usbtypec_resume  NULL
#endif


static const struct i2c_device_id ti_usbtypec_id[] = {
	{ "ti_typec", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ti_usbtypec_id);


#ifdef CONFIG_OF
static struct of_device_id ti_usbtypec_match_table[] = {
        { .compatible = "ti,usbtypec",},
        {},
};
#else
#define  ti_usbtypec_match_table NULL
#endif

static struct i2c_driver usb_type_c_driver = {
    .driver = {
        .name ="ti_usbtypec",
        .of_match_table = ti_usbtypec_match_table,
    },
    .probe = ti_usbtypec_probe,
    .remove = ti_usbtypec_remove,
#ifdef CONFIG_PM
    .suspend = ti_usbtypec_suspend,
    .resume = ti_usbtypec_resume,
#endif
    .id_table = ti_usbtypec_id,
};

static int  __init usb_type_c_init(void)
{
    return i2c_add_driver(&usb_type_c_driver);    
}
static void  __exit usb_type_c_exit(void)
{
    i2c_del_driver(&usb_type_c_driver);
}
module_init(usb_type_c_init);
module_exit(usb_type_c_exit);
MODULE_AUTHOR("xiaojun xue <xue.xiaojun@zte.com.cn>");
MODULE_DESCRIPTION("ti usb type c chip drivers");
MODULE_LICENSE("GPL");
