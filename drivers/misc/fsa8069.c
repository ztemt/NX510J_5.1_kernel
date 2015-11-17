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
#include <linux/fsa8069.h>
#include <linux/of_gpio.h>


/******************************************************************************
* Register addresses	
******************************************************************************/
#define REG_DEVICE_ID				0x01
#define REG_CONTROL					0x02
#define REG_STATUS					0x03
#define REG_INT_1					0x04
#define REG_INT_2					0x05
#define REG_INT_MSK_1				0x07
#define REG_INT_MSK_2				0x08
#define REG_JDET_TIME				0x0A
#define REG_MUSIC_TIME				0x0B
#define REG_MIC_DEB_TIME			0x0C
#define REG_DET_THRS				0x0F
#define REG_RST						0x10

/******************************************************************************
* Register bits	
******************************************************************************/

/* REG_DEVICE_ID (0x01) */
#define DEVICE_VER_ID_SHIFT				4
#define DEVICE_VER_ID					(0x0f << DEVICE_VER_ID_SHIFT)

/* REG_CONTROL (0x02) */
#define CONTROL_MUSIC_EN				0x01
#define CONTROL_KEY_DET_EN_SHIFT		2
#define CONTROL_KEY_DET_EN				(0x01 << CONTROL_KEY_DET_EN_SHIFT)
#define CONTROL_LDO_EN_SHIFT			3
#define CONTROL_LDO_EN					(0x01 << CONTROL_LDO_EN_SHIFT)

/* REG_STATUS (0x03) */
#define STATUS_IMPEDANCE				0x07
#define STATUS_IMPEDANCE_ACC_ATCH_SHIFT	3
#define STATUS_IMPEDANCE_ACC_ATCH		(0x01 << STATUS_IMPEDANCE_ACC_ATCH_SHIFT)

/* REG_INT_1 (0x04) */
#define INT1_PLUG_INSRT					0x01
#define INT1_PLUG_REMOVE_SHIFT			1
#define INT1_PLUG_REMOVE				(0x01 << INT1_PLUG_REMOVE_SHIFT)
#define INT1_MOIST_CHG_SHIFT			2
#define INT1_MOIST_CHG					(0x01 << INT1_MOIST_CHG_SHIFT)

/* REG_INT_2 (0x05) */
#define INT2_KEY_PRESS					0x01
#define INT2_KEY_REL_SHIFT				3
#define INT2_KEY_REL					(0x01 << INT2_KEY_REL_SHIFT)

/* REG_INT_MSK_1 (0x07) */
#define INT1_M_PLUG_INSRT					0x01
#define INT1_M_PLUG_REMOVE_SHIFT			1
#define INT1_M_PLUG_REMOVE					(0x01 << INT1_M_PLUG_REMOVE_SHIFT)
#define INT1_M_MOIST_CHG_SHIFT				2
#define INT1_M_MOIST_CHG					(0x01 << INT1_M_MOIST_CHG_SHIFT)

/* REG_INT_MSK_2 (0x08) */
#define INT2_M_KEY_PRESS					0x01
#define INT2_M_KEY_REL_SHIFT				3
#define INT2_M_KEY_REL						(0x01 << INT2_M_KEY_REL_SHIFT)

/* REG_JDET_TIME (0x0A) */
#define JDET_TIME_REMOVE					0x0F  //tDET_REM
#define JDET_TIME_INSERT_SHIFT				4
#define JDET_TIME_INSERT					(0x0F << JDET_TIME_INSERT_SHIFT)  //tDET_IN

/* REG_MUSIC_TIME (0x0B) */
#define KEY_TIME_WAIT						0x0F  //tWAIT
#define KEY_TIME_POLL_SHIFT					4
#define KEY_TIME_POLL 						(0x0F << KEY_TIME_POLL_SHIFT)  //tPOLL

/* REG_MIC_DEB_TIME (0x0C) */
#define MIC_DEB_TIME_KEY_SHIFT				0x0F  //tKBK

/* REG_DET_THRS (0x0F) */
#define DET_THRS_KEY_SHIFT					4
#define DET_THRS_KEY						(0x0F << DET_THRS_KEY_SHIFT)

/* REG_RST (0x10) */
#define RST_ALL								0x01

/******************************************************************************
* bit definitions
******************************************************************************/

/**********************************************/
#define TRUE 1
#define FALSE 0

enum {
    IMPEDANCE_TYPE1 = 0,  //16(ohm)
	IMPEDANCE_TYPE2,  //32(ohm)  
	IMPEDANCE_TYPE3,  //64(ohm)
	IMPEDANCE_TYPE4,  //150(ohm)
	IMPEDANCE_TYPE5,  //300(ohm)
	IMPEDANCE_TYPE6,  //600(ohm)
	IMPEDANCE_TYPE7,  //2K(ohm)
	MOISTURE_DETECTED 

};

struct fsa8069_info {
	struct i2c_client		*client;
	struct fsa8069_platform_data	*pdata;
	struct mutex		mutex;
	struct workqueue_struct	*int_wqueue;
	struct work_struct  int_work;
	int cur_impedance;
	struct completion bl_ready;
	bool update;
	struct class *fsa_class;
};

static struct i2c_client *this_client;

void fsa8069_reset_enable(void)
{
    int ret = 0;
    ret =  i2c_smbus_write_byte_data(this_client,REG_RST,0x01);
    ret += i2c_smbus_write_byte_data(this_client,REG_JDET_TIME,0xF9);
    ret += i2c_smbus_write_byte_data(this_client,REG_INT_MSK_1,0x04);
    ret += i2c_smbus_write_byte_data(this_client,REG_INT_MSK_2,0x09);
    pr_debug("%s: return %d\n",__func__,ret);
}
EXPORT_SYMBOL_GPL(fsa8069_reset_enable);
ssize_t fsa8069_show_impedance(struct device *dev, 	struct device_attribute *attr,char *buf)
{
    int ret;
    struct fsa8069_info *info = dev_get_drvdata(dev);
    struct i2c_client *client = info->client;
    int count = 3;
    bool update;

    while(count-- != 0)
    {
        mutex_lock(&info->mutex);
        update = info->update;
        mutex_unlock(&info->mutex);
        if(!update)
        {
            pr_debug("%s: the impedance is not ok,and sleep \n",__func__);
            msleep(50);
        } else {
            pr_debug("%s: the impedance is ok === ok\n",__func__);
            break;
        }
    }
    ret = i2c_smbus_read_byte_data(client, REG_STATUS);


    if(!(ret & STATUS_IMPEDANCE_ACC_ATCH))  // check impedance accessory attached bit if it is valid
    {
        return sprintf(buf, "-2\n");
    }

    ret &= STATUS_IMPEDANCE;

    switch(ret){
        case IMPEDANCE_TYPE1:
            return sprintf(buf, "16\n");
        case IMPEDANCE_TYPE2:
            return sprintf(buf, "32\n");
        case IMPEDANCE_TYPE3:
            return sprintf(buf, "64\n");
        case IMPEDANCE_TYPE4:
            return sprintf(buf, "150\n");
        case IMPEDANCE_TYPE5:
            return sprintf(buf, "300\n");
        case IMPEDANCE_TYPE6:
            return sprintf(buf, "600\n");
        case IMPEDANCE_TYPE7:
            return sprintf(buf, "2000\n");
        case MOISTURE_DETECTED:
            return sprintf(buf, "-1\n");

        default:
            return sprintf(buf, "ERROR\n");

    }

}
	
static DEVICE_ATTR(impedance, S_IRUGO, fsa8069_show_impedance,NULL);


char fsa8069_regs[] = {
    REG_DEVICE_ID,
    REG_CONTROL,
    REG_STATUS,
    REG_INT_1,
    REG_INT_2,
    REG_INT_MSK_1,
    REG_INT_MSK_2,
    REG_JDET_TIME,
    REG_MUSIC_TIME,
    REG_MIC_DEB_TIME,
    REG_DET_THRS,
    REG_RST,
};
ssize_t fsa8069_show_regs(struct device *dev, 	struct device_attribute *attr,char *buf)
{
    int ret;
    struct fsa8069_info *info = dev_get_drvdata(dev);
    struct i2c_client *client = info->client;
    int i =0;
    for(;i<sizeof(fsa8069_regs);i++) {
        ret = i2c_smbus_read_byte_data(client, fsa8069_regs[i]);
        if(ret < 0 ) 
        {
            printk("read data error %d\n",ret);
            break;
        }
        printk("0x%0x,-> 0x%x\n",fsa8069_regs[i],ret);
    }
    return sprintf(buf, "read ok\n");
}
DEVICE_ATTR(regs,S_IRUGO,fsa8069_show_regs,NULL);

static irqreturn_t fsa8069_irq_thread(int irq, void *handle)
{
	struct fsa8069_info *info = (struct fsa8069_info *)handle;

	queue_work(info->int_wqueue, &info->int_work);

	return IRQ_HANDLED;

}

/*********************************************************************
* Function: fsa8069_check_impedance(struct fsa8069_info *info)
*
* Parameters: None
*             
* Return:  IMPEDANCE_TYPE1 ~ MOISTURE_DETECTED
*              OR Impedance attached bit is invalid ( -1 )
*             
*
* Description:    
*
*******************************************************************/

static int fsa8069_check_impedance(struct fsa8069_info *info)
{
    int ret, count = 3;
	struct i2c_client *client = info->client;

	while(count-- != 0)  
	{
	    ret = i2c_smbus_read_byte_data(client, REG_STATUS);
		if(!(ret & STATUS_IMPEDANCE_ACC_ATCH))
			break;
		msleep(10);
	}

	if(count != 0)
	{
	    ret &= STATUS_IMPEDANCE;
		pr_info(" %s Impedance is %d.\n\n", __func__, ret);
		return ret;
	}

	else
	{
	    pr_info(" %s Impedance attached bit is INVALID.\n", __func__);
		return -1;
	}
		
	
}

static void fsa8069_jack_int_work_func(struct work_struct *work)
{
	struct fsa8069_info *info =
		container_of(work, struct fsa8069_info, int_work);

	struct i2c_client *client = info->client;
    int ret;
	unsigned char val1, val2;

    mutex_lock(&info->mutex);
	ret = i2c_smbus_read_word_data(client, REG_INT_1);
	val1 = ret & 0xff;
	val2 = ret >> 8;	
	info->update = true;

	if(val1)
	{
	    if(val1 & INT1_PLUG_INSRT)
	    {
	        pr_debug(" %s INT1_PLUG_INSRT interrupt!!.\n", __func__);
	        info->cur_impedance = fsa8069_check_impedance(info);
            pr_debug("info->cur_impedance is %d \n",info->cur_impedance);
	    }
		else if(val1 & INT1_PLUG_REMOVE)
		{
		    pr_debug(" %s INT1_PLUG_REMOVE interrupt!!.\n", __func__);
			info->cur_impedance = IMPEDANCE_TYPE1; // default
			info->update = false;
		}
		else if(val1 & INT1_MOIST_CHG)
		{
		    pr_debug(" %s: MOISTURE detected.\n", __func__);
		}

	} else if(val2) {
	    pr_debug("%s: key press\n",__func__);
    } else
		pr_debug("%s :INT1-> 0x%x, INT2-> 0x%x Weird interrupt happend.\n", __func__,val1,val2);
	
	mutex_unlock(&info->mutex);

}

static void fsa8069_init(struct fsa8069_info *info)
{
	struct i2c_client *client = info->client;
    int ret = 0;
    ret =  i2c_smbus_write_byte_data(this_client,REG_RST,0x01);
    ret += i2c_smbus_write_byte_data(client,REG_JDET_TIME,0xF9);
    ret += i2c_smbus_write_byte_data(client,REG_INT_MSK_1,0x04);
	ret += i2c_smbus_write_byte_data(client,REG_INT_MSK_2,0x09);
    printk("%s:return value is %d\n",__func__,ret);
}


struct pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *extncodec_sus;
	struct pinctrl_state *extncodec_act;
};

static struct pinctrl_info pinctrl_info;

int intr_gpio_fsa8069 = 0;
static int fsa8069_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	int ret = 0;
	struct device *dev_t;
	struct fsa8069_info *info;
	struct pinctrl *pinctrl;


    info = kzalloc(sizeof(struct fsa8069_info), GFP_KERNEL);
	info->client = client;
	info->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);
	init_completion(&info->bl_ready);

	this_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk("%s: i2c check functionality error\n", __func__);
		ret = -ENODEV;
		goto check_funcionality_failed;
	}
	info->fsa_class = class_create(THIS_MODULE, "headset_impedance");
	dev_t = device_create(info->fsa_class, NULL, 0, NULL, "fsa8069"); 
	device_create_file(dev_t, &dev_attr_impedance);
	device_create_file(dev_t, &dev_attr_regs);
	dev_set_drvdata(dev_t, info);

	/* GPIO setting */
	fsa8069_init(info);

	intr_gpio_fsa8069 = of_get_named_gpio(client->dev.of_node, "fsa8069-intr-gpio", 0);	
	client->irq = gpio_to_irq(intr_gpio_fsa8069);
	ret = gpio_request_one(intr_gpio_fsa8069, GPIOF_IN, "fsa8069-intr-gpio");
	if(ret < 0) printk("gpio_request_one === error %d\n",ret);
	pinctrl = pinctrl_get(&client->dev);
	pinctrl_info.pinctrl = pinctrl;
	pinctrl_info.extncodec_act = pinctrl_lookup_state(pinctrl, "fsa8069_irq_active");
	pinctrl_info.extncodec_sus = pinctrl_lookup_state(pinctrl, "fsa8069_irq_suspend");
	ret = pinctrl_select_state(pinctrl,pinctrl_info.extncodec_act);


    info->int_wqueue = create_singlethread_workqueue("fsa8069_wqueue");
	INIT_WORK(&info->int_work, fsa8069_jack_int_work_func);

    ret = request_irq(client->irq, fsa8069_irq_thread,IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	    "fsa8069 jack interrupt", info);
    if (ret) {
	    dev_err(&client->dev, "failed to reqeust IRQ\n");
	    return ret;
    }
	pr_info("fsa8069 ret %d ,and intr gpio is %d\n",ret,gpio_get_value(intr_gpio_fsa8069));
	return 0;

check_funcionality_failed:

	return ret;	

}
static int fsa8069_remove(struct i2c_client *client)
{
    struct fsa8069_info *info = i2c_get_clientdata(client);

	if (client->irq) {
		free_irq(client->irq, info);
	}

	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
	
	kfree(info);
	
	return 0;
}

static int  fsa8069_suspend(struct i2c_client *client, pm_message_t message)
{
    int ret = 0;
	ret = pinctrl_select_state(pinctrl_info.pinctrl,pinctrl_info.extncodec_act);
	pr_debug("fsa8069_suspend and gpio is %d\n",gpio_get_value(intr_gpio_fsa8069));

	return 0;
}

static int  fsa8069_resume(struct i2c_client *client)
{
	pr_debug("fsa8069_resume and gpio is %d\n",gpio_get_value(intr_gpio_fsa8069));
	return 0;
}

static const struct i2c_device_id fsa8069_i2c_id[] = {
	{ "fsa8069", 0 },
	{ }
};
 
static struct i2c_driver fsa8069_i2c_driver = {
	.driver = {
		.name = "fsa8069",
		.owner = THIS_MODULE,			
	},
	.probe    = fsa8069_probe,
	.remove   = fsa8069_remove,
	.suspend  = fsa8069_suspend,
	.resume	  = fsa8069_resume,
	.id_table = fsa8069_i2c_id,
};

static __init int fsa8069_i2c_init(void)
{

	return i2c_add_driver(&fsa8069_i2c_driver);
}

static __exit void fsa8069_i2c_exit(void)
{
	i2c_del_driver(&fsa8069_i2c_driver);
}

module_init(fsa8069_i2c_init);
module_exit(fsa8069_i2c_exit);

MODULE_AUTHOR(" ");
MODULE_DESCRIPTION("I2C bus driver for FSA8069 Jack detection IC");
MODULE_LICENSE("GPL v2");

