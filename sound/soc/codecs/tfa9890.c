#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

int reset_gpio = -1;
int intr_gpio = -1;
#if 1
static ssize_t tfa9890_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = -1;
    if(reset_gpio > 0)
        ret = gpio_get_value(reset_gpio);
    pr_info("tfa reset_gpio status ...  %d\n",ret);
    return snprintf(buf, PAGE_SIZE, "%d\n",ret) ;
}

static ssize_t tfa9890_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    bool value;
    if (strtobool(buf, &value))
        return -EINVAL;
    if(reset_gpio >0)
        gpio_set_value(reset_gpio,((value > 0) ? 1: 0));
    pr_debug("Reset the tfa9890.... %d\n",value);
    return size;

}

static DEVICE_ATTR(reset_gpio, 0666, tfa9890_enable_show, tfa9890_enable_store);
#endif


#define MAX_BUFFER_SIZE 48
struct tfa9890_dev {
    struct miscdevice tfa_misc_dev;
	struct mutex read_mutex;
	struct i2c_client *tfa_i2c_client;
};

//static u8 *Tfa9890I2CDMABuf_va = NULL;
//static u32 Tfa9890I2CDMABuf_pa = NULL;


struct tfa9890_dev tfa9890_device;

#define TFA9890_MAX_XFER_LEN		254

#define TFA9890_MAX_BUF			(TFA9890_MAX_XFER_LEN + 1)

static int tfa9890_send_cmd_only(u8 reg)
{
	struct i2c_msg msg[1];
	u8 buf;
	int ret;

	buf = reg;

	msg[0].addr = tfa9890_device.tfa_i2c_client->addr;
	msg[0].flags = 0;
	msg[0].buf = &buf;
	msg[0].len = 1;

	ret = i2c_transfer(tfa9890_device.tfa_i2c_client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(&tfa9890_device.tfa_i2c_client->dev, "write err: %d", ret);
		return ret;
	}

	if (ret != ARRAY_SIZE(msg))
		return -EIO;

	return 0;
}

static int tfa9890_write_chunk(u8 reg, u8 *val, int len)
{
	unsigned int offset = 0;
	unsigned int size;
	struct i2c_msg msg[1];
	u8 buf[TFA9890_MAX_BUF];
	int ret;

	/* Simple write command has zero length */
	if (len == 0)
		return tfa9890_send_cmd_only(reg);

	while (len > 0) {
		if (len > TFA9890_MAX_XFER_LEN)
			size = TFA9890_MAX_XFER_LEN;
		else
			size = len;

		memset(buf, 0x0, sizeof(buf));
		buf[0] = reg;
		memcpy(&buf[1], val + offset, size);

		msg[0].addr = tfa9890_device.tfa_i2c_client->addr;
		msg[0].flags = 0;
		msg[0].buf = buf;
		msg[0].len = size + 1;

		ret = i2c_transfer(tfa9890_device.tfa_i2c_client->adapter, msg, ARRAY_SIZE(msg));
		if (ret < 0) {
			dev_err(&tfa9890_device.tfa_i2c_client->dev, "write err: %d", ret);
			return ret;
		}

		if (ret != ARRAY_SIZE(msg))
			return -EIO;

		len -= size;
		offset += size;
	}

	return 0;
}


static ssize_t tfa_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	int ret;
	char *tmp;
	char reg_addr;
	char *data;
	//if (count > 8192)
	//	count = 8192;

	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;
	if (copy_from_user(tmp,buf,count)) {
		kfree(tmp);
		return -EFAULT;
	}
	printk("i2c-dev: i2c-%d writing %zu bytes.\n", iminor(file_inode(filp)), count);
	reg_addr = tmp[0];
	data = tmp+1;
	printk("tfa_write:reg is 0x%x,data length %zu	\n",reg_addr,count-1);
	ret = tfa9890_write_chunk(reg_addr,data,count-1);
	return 0;
	

}

static long tfa_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int  ret = 0;
    pr_debug("tfa_ioctl cmd = 0x%x arg = %lx\n", cmd, arg);
    switch (cmd)
    {
        default:
        {
            pr_debug("tfa_ioctl command: %x \n", cmd);
            ret = 0;
            break;
        }
    }
    return ret;

}


static int tfa_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t tfa_read(struct file *filp,  char __user *buf,
			size_t count, loff_t *offset)
{
   	char *tmp;
	int ret;

	struct i2c_client *client = tfa9890_device.tfa_i2c_client;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	printk("tfa_read: reading %zu bytes.\n",count);

	ret = i2c_master_recv(client, tmp, count);
	if (ret >= 0)
		ret = copy_to_user(buf, tmp, count) ? -EFAULT : ret;
	kfree(tmp);
	return ret;
}

static long tfa_compat_ioctl(struct file *pfile, unsigned int cmd,
			unsigned long arg)
{
    int  ret = 0;

   printk("tfa_compat_ioctl cmd = 0x%x arg = %lx\n", cmd, arg);

    switch (cmd)
    {
        default:
        {
            printk("tfa_compat_ioctl command: %x \n", cmd);
            ret = 0;
            break;
        }
    }
    return ret;

}

static const struct file_operations tfa_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	//.poll  = tfa_poll,
	.read  = tfa_read,
	.write = tfa_write,
	.open = tfa_open,
	.unlocked_ioctl = tfa_ioctl,
	.compat_ioctl = tfa_compat_ioctl

};

static int  tfa9890_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
    int ret = 0;
    struct device_node *np = i2c->dev.of_node;
    reset_gpio =  of_get_named_gpio(np, "tfa9890-reset-gpio", 0);
    intr_gpio =  of_get_named_gpio(np, "tfa9890-intr-gpio", 0);
    if(reset_gpio > 0)
    {
        gpio_set_value_cansleep(reset_gpio,0);
        printk("reset_gpio ==== is %d\n",reset_gpio);
        printk("%s... getvalue %d\n",__func__,gpio_get_value(reset_gpio));
    }
	device_create_file(&i2c->dev, &dev_attr_reset_gpio);
	tfa9890_device.tfa_i2c_client = i2c;
	tfa9890_device.tfa_misc_dev.minor = MISC_DYNAMIC_MINOR;
	tfa9890_device.tfa_misc_dev.name = "sma";
	tfa9890_device.tfa_misc_dev.fops = &tfa_dev_fops;	
	mutex_init(&tfa9890_device.read_mutex);
	ret = misc_register(&tfa9890_device.tfa_misc_dev);
	if(ret) {
		dev_err(&i2c->dev,"misc_register err %d\n",ret);
	}
	i2c_set_clientdata(i2c, &tfa9890_device);
	//Tfa9890I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &Tfa9890I2CDMABuf_pa, GFP_KERNEL);
	return 0;
}

static int  tfa9890_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tfa9890_i2c_id[] = {
	{ "nxp,tfa9890", 0},
	{ },
};

static const struct of_device_id of_tfa_device_idtable[] = {
    { .compatible = "nxp,tfa9890",0},
	{ },
};

/* corgi i2c codec control layer */
static struct i2c_driver tfa9890_i2c_driver = {
	.driver = {
		.name = "tfa9890",
        .of_match_table = of_tfa_device_idtable,
		.owner = THIS_MODULE,
	},
	.probe = tfa9890_i2c_probe,
	.remove = tfa9890_i2c_remove,
	.id_table = tfa9890_i2c_id,
};


static int __init tfa9890_modinit(void)
{
	int ret = 0;

    pr_info("%s enter \n",__func__);
	ret = i2c_add_driver(&tfa9890_i2c_driver);
    return ret;
    
}
module_init(tfa9890_modinit);

static void __exit tfa9890_exit(void)
{
	i2c_del_driver(&tfa9890_i2c_driver);
}
module_exit(tfa9890_exit);

MODULE_DESCRIPTION("NXP tfa9890 I2C driver");
MODULE_AUTHOR("wuzehui");
MODULE_LICENSE("GPL");
