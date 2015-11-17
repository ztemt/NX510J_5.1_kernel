/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <asm/uaccess.h>
#include <linux/ktime.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>  //irq no.
#include <linux/completion.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
//#include <mach/gpiomux.h>
#ifdef CONFIG_BBK_DRIVER_INFO
#include <linux/bbk_drivers_info.h>
#endif
#include <linux/timer.h>

#define GF66XX_FASYNC 		1//If support fasync mechanism.

#include "gf66xx.h"

#include <linux/ctype.h>
#include <linux/ctype.h>
#include <linux/kernel.h>


#define GF816_PID          "GF816"
/*spi device name*/
#define SPI_DEV_NAME   "gf66xx-spi"
/*device name after register in charater*/
#define DEV_NAME "gf66xx-spi"

//#define FP_HAL_64

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define	CHRD_DRIVER_NAME		"gf66xx"
#define	CLASS_NAME			"gf66xx-spi"
#define SPIDEV_MAJOR			225	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */
#define GF66XX_CFG_LEN			249	/*config data length*/
#define GF66XX_CFG_ADDR			0x8047
#define FW_LENGTH 			(42*1024)
#define ESD_PROTECT	        0
#define SEND_CONFIG         0
#define FW_UPDATE	        0

#if FW_UPDATE
static unsigned char GF66XX_FW[]=
{
	#include "gf66xx_fw.i"
};
#endif

static DECLARE_BITMAP(minors, N_SPI_MINORS);

//#undef GF66XX_FASYNC

/**************************debug*******************************/
#define GF66XX_DEBUG    1
#undef GF66XX_DEBUG
#define GF66XX_SPI_TEST	1

#define TEST_BUF_LEN 2048 
#define TEST_CNT 10000

#ifdef GF66XX_DEBUG
#define   gf66xx_dbg(fmt, args...) do{ \
					pr_warn("gf66xx:" fmt, ##args);\
				}while(0)
#define FUNC_ENTRY()  pr_warn("gf66xx:%s, entry\n", __func__)
#define FUNC_EXIT()  pr_warn("gf66xx:%s, exit\n", __func__)
#else
#define gf66xx_dbg(fmt, args...)
#define FUNC_ENTRY()
#define FUNC_EXIT()
#endif
/***********************************************************
gf66xx config for GF66XX_2000_6D89.BIN 
************************************************************/
u8 gf66xx_config[] = {
    0x00,0xf0,0xf0,0x0c,0xe4,0x90,0x4d,0x03,0x00,0x50,0x32,0xf0,0xf0,0xe4,0x0c,0xb0,
    0xcd,0x03,0x00,0x03,0x11,0xa0,0x0d,0x00,0x14,0x19,0x0f,0x0f,0x0f,0xb2,0x3f,0xb3,
    0x33,0x03,0x90,0x01,0x20,0x08,0x14,0x1e,0x64,0x0f,0x14,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x11,0x40,0x25,0x04,0x01,0x07,0x00,0xa4,0x26,
    0x66,0x00,0x50,0x00,0x01,0x00,0x01,0x0f,0x96,0x00,0x01,0x02,0x84,0x00,0x03,0x00,
    0x00,0x00,0x22,0x00,0x0c,0x00,0x1c,0x00,0x1c,0x03,0x99,0x00,0x31,0x00,0x03,0x00,
    0x00,0x00,0x03,0x00,0x00,0x80,0x89,0x00,0x20,0x14,0x00,0x00,0x2f,0x00,0xd0,0x01,
    0x68,0x80,0x00,0x00,0x02,0x0f,0xa0,0xc8,0x30,0x13,0xf4,0x00,0x08,0x00,0x00,0x00,
    0x28,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9c,0x01,
};

//static struct regulator *vcc_gx_l10;
static struct regulator *gf66xx_power_vdd;
struct gf66xx_dev   *gf66xx_dev_p;

/*************************************************************/

extern int gf66xx_fw_update(struct gf66xx_dev* gf66xx_dev, unsigned char *buf, unsigned short len);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/*************************data stream***********************
*	FRAME NO  | RING_H  | RING_L  |  DATA STREAM | CHECKSUM |
*     1B      |   1B    |  1B     |    2048B     |  2B      |
************************************************************/
static unsigned int bufsiz = 14336+5;

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static struct gf66xx_pinctrl_info gf66xx_pctrl;

static int gf66xx_pinctrl_init(struct device *dev) 
{
    gf66xx_dbg("enter\n");
    
	gf66xx_pctrl.pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(gf66xx_pctrl.pinctrl)) 
    { 
        gf66xx_dbg("Getting pinctrl handle failed\n"); 
        return -EINVAL; 
    }
	gf66xx_pctrl.gpio_state_active = pinctrl_lookup_state( gf66xx_pctrl.pinctrl, "gf66xx_active");
	
	if (IS_ERR_OR_NULL(gf66xx_pctrl.gpio_state_active)) 
	{ 
		gf66xx_dbg("Failed to get the active state pinctrl handle\n"); 
		return -EINVAL; 
	}
	
	gf66xx_pctrl.gpio_state_suspend = pinctrl_lookup_state( gf66xx_pctrl.pinctrl, "gf66xx_suspend");
	if (IS_ERR_OR_NULL(gf66xx_pctrl.gpio_state_suspend)) 
	{
		gf66xx_dbg("Failed to get the suspend state pinctrl handle\n"); 
		return -EINVAL; 
	}

    gf66xx_dbg("success\n");
		 
	return 0; 
}


/*Confure the IRQ pin for GF66XX irq if necessary*/
static void gf66xx_irq_cfg(struct gf66xx_dev *dev)
{
	gpio_request(dev->irq_gpio,  "gf66xx_irq");	
	
	gpio_direction_output(dev->irq_gpio, 1);
	gpio_direction_input(dev->irq_gpio);	
    gpio_set_value(dev->irq_gpio, 1);
	//gpio_free(dev->irq_gpio);

}

/********************************************************************
*CPU output low level in RST pin to reset GF66XX. This is the MUST action for GF66XX.
*Take care of this function. IO Pin driver strength / glitch and so on.
********************************************************************/
static void gf66xx_hw_reset(struct gf66xx_dev *dev)
{	
	gpio_request(dev->reset_gpio, "gf66xx_rst");
	gpio_direction_output(dev->reset_gpio, 1);

	gpio_set_value(dev->reset_gpio, 1);	
	mdelay(5);
	
	gpio_set_value(dev->reset_gpio, 0);
	mdelay(8);
	
	gpio_set_value(dev->reset_gpio, 1);
    	mdelay(3);	
	
	//gpio_free(dev->reset_gpio);		
	gf66xx_irq_cfg(dev);
	mdelay(100);
}

static void gf66xx_spi_complete(void *arg)
{
	complete(arg);
}
/**********************************************************
*Message format:
*	write cmd   |  ADDR_H |ADDR_L  |  data stream  |
*    1B         |   1B    |  1B    |  length       |
*
* read buffer length should be 1 + 1 + 1 + data_length
***********************************************************/
 int gf66xx_spi_write_bytes(struct gf66xx_dev *gf66xx_dev,
				u16 addr, u32 data_len, u8 *tx_buf)
{
	DECLARE_COMPLETION_ONSTACK(read_done);
	struct spi_message msg;
	struct spi_transfer *xfer;
	int ret = 0;

	xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	if( xfer == NULL){
		gf66xx_dbg("No memory for command.\n");
		return -ENOMEM;
	}

    /*send gf66xx command to device.*/
	spi_message_init(&msg);
	tx_buf[0] = GF66XX_W;
	tx_buf[1] = (u8)((addr >> 8)&0xFF);
	tx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = tx_buf;
	xfer[0].len = data_len + 3;
	spi_message_add_tail(xfer, &msg);
       ////pr_info("CMD address,tx_buf[1]=0x%2x,tx_buf[2]=0x%2x ,xfer[0].len=%d\n",tx_buf[1],tx_buf[2],(int)xfer[0].len);
	msg.complete = gf66xx_spi_complete;
	msg.context = &read_done;

	spin_lock_irq(&gf66xx_dev->spi_lock);
	ret = spi_async(gf66xx_dev->spi, &msg);
	spin_unlock_irq(&gf66xx_dev->spi_lock);
	if(ret == 0) {
		wait_for_completion(&read_done);
		if(msg.status == 0) {
			gf66xx_dbg("gf66xx_spi_write_bytes actual_length = %d\n", (int)msg.actual_length);
			ret = data_len;
		}
	}

	kfree(xfer);
	if(xfer != NULL)
		xfer = NULL;
	
	return ret;
}

/*************************************************************
*First message:
*	write cmd   |  ADDR_H |ADDR_L  |
*    1B         |   1B    |  1B    |
*Second message:
*	read cmd   |  data stream  |
*    1B        |   length    |
*
* read buffer length should be 1 + 1 + 1 + 1 + data_length
**************************************************************/
int gf66xx_spi_read_bytes(struct gf66xx_dev *gf66xx_dev,
				u16 addr, u32 data_len, u8 *rx_buf)
{
	//gf66xx_dbg("gf66xx_spi_read_bytes entry.\n");
	DECLARE_COMPLETION_ONSTACK(write_done);
	struct spi_message msg;
	struct spi_transfer *xfer;
	int ret = 0;
	
	xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	if( xfer == NULL){
		gf66xx_dbg("No memory for command.\n");
		return -ENOMEM;
	}

    /*send gf66xx command to device.*/
	spi_message_init(&msg);
	rx_buf[0] = GF66XX_W;
	rx_buf[1] = (u8)((addr >> 8)&0xFF);
	rx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = rx_buf;
	xfer[0].len = 3;
	//pr_info("gf66xx_spi_read_bytes CMD address,rx_buf[1]=0x%2x,rx_buf[2]=0x%2x ,xfer[0].len=%d\n",rx_buf[1],rx_buf[2],(int)xfer[0].len);
	spi_message_add_tail(&xfer[0], &msg);
	
	spi_sync(gf66xx_dev->spi, &msg);
	spi_message_init(&msg);
	/*if wanted to read data from gf66xx. 
	 *Should write Read command to device
	 *before read any data from device.
	 */
	rx_buf[3] = GF66XX_R;
	xfer[1].tx_buf = &rx_buf[3];
	
	xfer[1].rx_buf = &rx_buf[3];
	xfer[1].len = data_len + 1;
//pr_info("gf66xx_spi_read_bytes CMD address,rx_buf[3]=0x%2x,xfer[1].tx_buf=0x%2x ,xfer[1].len=%d\n",rx_buf[3],xfer[1].tx_buf,(int)xfer[1].len);
	spi_message_add_tail(&xfer[1], &msg);

	msg.complete = gf66xx_spi_complete;
	msg.context = &write_done;

	spin_lock_irq(&gf66xx_dev->spi_lock);
	ret = spi_async(gf66xx_dev->spi, &msg);
	spin_unlock_irq(&gf66xx_dev->spi_lock);
	if(ret == 0) {
		wait_for_completion(&write_done);
		if(msg.status == 0) {
			gf66xx_dbg("gf66xx_spi_read_bytes actual_length = %d\n", (int)msg.actual_length);
			ret = data_len;
		}
	}

	kfree(xfer);
	if(xfer != NULL)
		xfer = NULL;

	//gf66xx_dbg("gf66xx_spi_read_bytes exit.\n");
	return ret;
}

static int gf66xx_spi_read_byte(struct gf66xx_dev *gf66xx_dev, u16 addr, u8 *value)
{
	int status = 0;
	//pr_info("gf66xx_spi_read_byte addr display =0x%2x\n",addr);
	mutex_lock(&gf66xx_dev->buf_lock);
	status = gf66xx_spi_read_bytes(gf66xx_dev, addr, 1, gf66xx_dev->buffer);
	*value = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	//pr_info("Rs = %d, va = 0x%x, buffer[3] = 0x%x, buffer[4] = 0x%x, buffer[5] = 0x%x\n", (int)status, *value, 
	//	gf66xx_dev->buffer[3], gf66xx_dev->buffer[4], gf66xx_dev->buffer[5]);
	mutex_unlock(&gf66xx_dev->buf_lock);
	return status;
}
static int gf66xx_spi_write_byte(struct gf66xx_dev *gf66xx_dev, u16 addr, u8 value)
{
	int status = 0;
	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = value;
	status = gf66xx_spi_write_bytes(gf66xx_dev, addr, 1, gf66xx_dev->buffer);
	gf66xx_dbg("Ws = %d, va = 0x%x, buffer[3] = 0x%x, buffer[4] = 0x%x, buffer[5] = 0x%x\n", (int)status, value , 
		gf66xx_dev->buffer[3], gf66xx_dev->buffer[4], gf66xx_dev->buffer[5]);
	mutex_unlock(&gf66xx_dev->buf_lock);
	return status;
}

/* Read-only message with current device setup */
static ssize_t
gf66xx_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	ssize_t			status = 0;
	FUNC_ENTRY();
	if ((count > bufsiz)||(count == 0)) {
		pr_warn("Max size for write buffer is %d. wanted length is %d\n", (int)bufsiz, (int)count);
		FUNC_EXIT();
		return -EMSGSIZE;
	}

	gf66xx_dev = filp->private_data;
	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_dev->spi->max_speed_hz=4*1000*1000;
	spi_setup(gf66xx_dev->spi);
	status = gf66xx_spi_read_bytes(gf66xx_dev, GF66XX_BUFFER_DATA, count, gf66xx_dev->buffer);
	if(status > 0) {
		unsigned long missing = 0;
		missing = copy_to_user(buf, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, status);
		if(missing == status)
			status = -EFAULT;
	} else {
		pr_err("Failed to read data from SPI device.\n");
		status = -EFAULT;
	}

	mutex_unlock(&gf66xx_dev->buf_lock);

	return status;
}

/* Write-only message with current device setup */
static ssize_t
gf66xx_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	ssize_t			status = 0;
	FUNC_ENTRY();
	if(count > bufsiz) {
		pr_warn("Max size for write buffer is %d\n", (int)bufsiz);
		return -EMSGSIZE;
	} 

	mutex_lock(&gf66xx_dev->buf_lock);
	status = copy_from_user(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET, buf, count);
	if(status == 0) {
		gf66xx_dev->spi->max_speed_hz=4*1000*1000;
		spi_setup(gf66xx_dev->spi);
		status = gf66xx_spi_write_bytes(gf66xx_dev, GF66XX_BUFFER_DATA, count, gf66xx_dev->buffer);
	} else {
		pr_err("Failed to xfer data through SPI bus.\n");
		status = -EFAULT;
	}
	mutex_unlock(&gf66xx_dev->buf_lock);
	FUNC_EXIT();
	return status;
}

static long
gf66xx_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	
	struct gf66xx_dev *gf66xx_dev = NULL;
	struct gf66xx_ioc_transfer *ioc = NULL;
	int			err = 0;
	u32			tmp = 0;
	int 		retval = 0;
	unsigned long buf_addr;
	int i = 0;
	
	//pr_info("gf66xx_ioctl entry cmd=0x%16x\n",(unsigned int)cmd);
	FUNC_ENTRY();
	if (_IOC_TYPE(cmd) != GF66XX_IOC_MAGIC)
	{
		pr_info("gf66xx: _IOC_TYPE(cmd) != GF66XX_IOC_MAGIC\n");
		return -ENOTTY;
	}	
	//pr_info("_IOC_TYPE(cmd)=0x%8x\n",(unsigned int)_IOC_TYPE(cmd));
	//pr_info("_IOC_DIR(cmd) 0x%8x\n",(unsigned int)_IOC_DIR(cmd));
	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
						(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
						(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;
	
	gf66xx_dev = (struct gf66xx_dev *)filp->private_data;
	//pr_info("gf66xx raw cmd =0x%16x",cmd);
	cmd&=0xffffffff;
	//pr_info("gf66xx convert cmd =0x%16x",cmd);
	
	//pr_info("GF66xx _IOC_SIZE(cmd)=0x%16x\n",(unsigned int)_IOC_SIZE(cmd));
	//pr_info("GF66xx _GF66XX_IOC_CMD=0x%16x\n",(unsigned int)GF66XX_IOC_CMD);
	//pr_info("GF66xx _GF66XX_IOC_REINIT=0x%16x\n",(unsigned int)GF66XX_IOC_REINIT);
	//pr_info("GF66xx _GF66XX_IOC_SETSPEED=0x%16x\n",(unsigned int)GF66XX_IOC_SETSPEED);

	switch(cmd&0x0000ffff) {
	case (GF66XX_IOC_CMD&0x0000ffff):
		ioc = kzalloc(sizeof(*ioc), GFP_KERNEL);
		/*copy command data from user to kernel.*/

		#ifdef FP_HAL_64
		if(copy_from_user(ioc, (struct gf66xx_ioc_transfer*)arg, sizeof(*ioc))){
			pr_err("gf66xx :Failed to copy command from user to kernel.\n");
			retval = -EFAULT;
			break;
		}
		
        #else
        
        if(copy_from_user(ioc, (struct gf66xx_ioc_transfer*)arg, sizeof(*ioc))){
			pr_err("gf66xx :Failed to copy command from user to kernel.\n");
			retval = -EFAULT;
			break;
		}

        buf_addr = (unsigned long)ioc->buf;
        buf_addr = buf_addr & 0xffffffff;
		#endif
		
		if((ioc->len > bufsiz)||(ioc->len == 0)) {
			pr_warn("gf66xx :The request length[%d] is longer than supported maximum buffer length[%d].\n", 
					(int)ioc->len, (int)bufsiz);
			retval = -EMSGSIZE;
			break;
		}
		
		mutex_lock(&gf66xx_dev->buf_lock);
		gf66xx_dev->spi->max_speed_hz=1*1000*1000;
		spi_setup(gf66xx_dev->spi);

        #ifdef FP_HAL_64
        
		if(ioc->cmd == GF66XX_R) {
			/*if want to read data from hardware.*/
			gf66xx_dbg("gf66xx :Read data from 0x%x, len = 0x%x buf = 0x%p\n", ioc->addr, ioc->len, ioc->buf);
			gf66xx_spi_read_bytes(gf66xx_dev, ioc->addr, ioc->len, gf66xx_dev->buffer);
			if(copy_to_user(ioc->buf, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, ioc->len)) {
				pr_err("gf66xx:Failed to copy data from kernel to user.\n");
				retval = -EFAULT;
				mutex_unlock(&gf66xx_dev->buf_lock);
				break;
			}
		} else if (ioc->cmd == GF66XX_W) {
			/*if want to read data from hardware.*/
			gf66xx_dbg("gf66xx:Write data from 0x%x, len = 0x%x\n", ioc->addr, ioc->len);
			if(copy_from_user(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET,ioc->buf, ioc->len)){
				pr_err("gf66xx:Failed to copy data from user to kernel.\n");
				retval = -EFAULT;
				mutex_unlock(&gf66xx_dev->buf_lock);
				break;
			}

			gf66xx_spi_write_bytes(gf66xx_dev, ioc->addr, ioc->len, gf66xx_dev->buffer);
		} else {
			pr_warn("gf66xx:Error command for gf66xx.\n");
		}
		if(ioc != NULL) {
			kfree(ioc);
			ioc = NULL;
		}
		mutex_unlock(&gf66xx_dev->buf_lock);
		break;
		
		#else
		if(ioc->cmd == GF66XX_R) {
			/*if want to read data from hardware.*/
			gf66xx_dbg("gf66xx :Read data from 0x%x, len = 0x%x buf_addr = %ld\n", ioc->addr, ioc->len, buf_addr);
			gf66xx_spi_read_bytes(gf66xx_dev, ioc->addr, ioc->len, gf66xx_dev->buffer);	
			
            for (i=0; i<=4; i++)
            {
                gf66xx_dbg("gf66xx ++ : buf[%d] = 0x%0x\n", i, gf66xx_dev->buffer[i]);
            }

			if(copy_to_user((unsigned char*)buf_addr, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, ioc->len)) {
				pr_err("gf66xx:Failed to copy data from kernel to user.\n");
				retval = -EFAULT;
				mutex_unlock(&gf66xx_dev->buf_lock);
				break;
			}
		} else if (ioc->cmd == GF66XX_W) {
			/*if want to read data from hardware.*/
			gf66xx_dbg("gf66xx:Write data from 0x%x, len = 0x%x\n", ioc->addr, ioc->len);
			if(copy_from_user(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET,(unsigned char*)buf_addr, ioc->len)){
				pr_err("gf66xx:Failed to copy data from user to kernel.\n");
				retval = -EFAULT;
				mutex_unlock(&gf66xx_dev->buf_lock);
				break;
			}

			gf66xx_spi_write_bytes(gf66xx_dev, ioc->addr, ioc->len, gf66xx_dev->buffer);
		} else {
			pr_warn("gf66xx:Error command for gf66xx.\n");
		}
		if(ioc != NULL) {
			kfree(ioc);
			ioc = NULL;
		}
		mutex_unlock(&gf66xx_dev->buf_lock);
		break;

		
		#endif
	case (GF66XX_IOC_REINIT&0x0000ffff):
		disable_irq(gf66xx_dev->spi->irq);
		//disable_irq_wake(gf66xx_dev->spi->irq);
		gf66xx_hw_reset(gf66xx_dev);
		enable_irq(gf66xx_dev->spi->irq);
		//enable_irq_wake(gf66xx_dev->spi->irq);
		pr_warn("wake-up gf66xx\n");
		break;
	case (GF66XX_IOC_SETSPEED&0x0000ffff):
		retval = __get_user(tmp, (u32 __user*)arg);
		if(tmp > 8*1000*1000) {
			pr_warn("The maximum SPI speed is 8MHz.\n");
			retval = -EMSGSIZE;
			break;
		}
		if(retval == 0) {
			gf66xx_dev->spi->max_speed_hz=tmp;
			spi_setup(gf66xx_dev->spi);
			gf66xx_dbg("spi speed changed to %d\n", (int)tmp);
		}	
		break;
	default:
		pr_warn("gf66xx doesn't support this command(0x%16x)\n", cmd);
		break;
	}
	FUNC_EXIT();
	return retval;
}


static long gf66xx_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	//compat_ptr()convert the 64 bit unsigned long type to 32 bit address.
	return gf66xx_ioctl(filp,cmd,(unsigned long)compat_ptr(arg));
}

static unsigned int gf66xx_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	gf66xx_spi_read_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, &gf66xx_dev->buf_status);
	if((gf66xx_dev->buf_status & GF66XX_BUF_STA_MASK) == GF66XX_BUF_STA_READY) {
		return (POLLIN|POLLRDNORM);
	} else {
		gf66xx_dbg("Poll no data.\n");
	}
	return 0;
}
/*
static  void gf66xx_spi_work(struct work_struct *work)
{
	struct gf66xx_dev *gf66xx_dev = container_of(work, struct gf66xx_dev, spi_work);

	gf66xx_spi_read_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, &gf66xx_dev->buf_status);
	pr_warn("In work. status = 0x%x\n", gf66xx_dev->buf_status);
	if((gf66xx_dev->buf_status & GF66XX_BUF_STA_MASK) == GF66XX_BUF_STA_READY) {
#ifdef GF66XX_FASYNC
		if(gf66xx_dev->async) {
			kill_fasync(&gf66xx_dev->async, SIGIO, POLL_IN);
		}

#endif
	}
}
*/
static irqreturn_t gf66xx_irq(int irq, void* handle)
{
	struct gf66xx_dev *gf66xx_dev = (struct gf66xx_dev *)handle;
#if 0
	if(gf66xx_dev->spi_wq != NULL)
		queue_work(gf66xx_dev->spi_wq, &gf66xx_dev->spi_work);
#endif
	u8 mode = 0x80;
	u8	status;

    //gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, 1, buf_temp);
    //pr_info("0x%2x,",buf_temp[4]);

	
    gf66xx_spi_read_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, &status);
    pr_info("gf66xx GF66XX_BUFFER_STATUS=0x%2x\n",status);
    if(!(status & GF66XX_BUF_STA_MASK)) {
        gf66xx_dbg("Invalid IRQ = 0x%x\n", status);        
        //gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, (status & 0x7F));//just for test
        return IRQ_HANDLED;
    }

	gf66xx_spi_read_byte(gf66xx_dev, GF66XX_MODE_STATUS, &mode);
	//pr_info("status = 0x%x, mode = %d\n", status, (int)mode);
	switch(mode){
		case GF66XX_FF_MODE:
			//flash finger. Wake up.
			gf66xx_dbg("Flash finger mode. Wake!\n");
	        gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, GF66XX_IMAGE_MODE);
            //input_report_key(gf66xx_dev->input, KEY_HOME, 1);
            input_report_key(gf66xx_dev->input, KEY_POWER, 1);
			input_sync(gf66xx_dev->input);
			//input_report_key(gf66xx_dev->input, KEY_HOME, 0);
			input_report_key(gf66xx_dev->input, KEY_POWER, 0);
            input_sync(gf66xx_dev->input);    
	    case GF66XX_IMAGE_MODE:
			gf66xx_dbg("image mode:\n");
		#ifdef GF66XX_FASYNC
			if(gf66xx_dev->async) {
				kill_fasync(&gf66xx_dev->async, SIGIO, POLL_IN);
			}
		#endif
			break;
		case GF66XX_KEY_MODE:
			gf66xx_dbg("key mode:\n");
			if  ((status & GF66XX_KEY_MASK) && (status & GF66XX_BUF_STA_MASK)) {
				input_report_key(gf66xx_dev->input, KEY_HOME, (status & GF66XX_KEY_STA)>>4);
				input_sync(gf66xx_dev->input);
			}
			gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, (status & 0x7F));
			break;
		case GF66XX_SLEEP_MODE:
			pr_warn("Should not happen in sleep mode.\n");
			break;
		default:
			pr_warn("Unknown mode. mode = 0x%x\n", mode);
			break;
	}

    //gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, (status & 0x7F));//just for test

	
	return IRQ_HANDLED;
}

static int gf66xx_open(struct inode *inode, struct file *filp)
{
	struct gf66xx_dev *gf66xx_dev;
	int			status = -ENXIO;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);

	list_for_each_entry(gf66xx_dev, &device_list, device_entry) {
		if(gf66xx_dev->devt == inode->i_rdev) {
			gf66xx_dbg("Found\n");
			status = 0;
			break;
		}
	}

	if(status == 0){
		mutex_lock(&gf66xx_dev->buf_lock);
		if( gf66xx_dev->buffer == NULL) {
			gf66xx_dev->buffer = kzalloc(bufsiz + GF66XX_RDATA_OFFSET, GFP_KERNEL);
			if(gf66xx_dev->buffer == NULL) {
				dev_dbg(&gf66xx_dev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		mutex_unlock(&gf66xx_dev->buf_lock);

		if(status == 0) {
			gf66xx_dev->users++;
			filp->private_data = gf66xx_dev;
			nonseekable_open(inode, filp);
			gf66xx_dbg("Succeed to open device. irq = %d\n", (int)gf66xx_dev->spi->irq);
			if(gf66xx_dev->users == 1)
				enable_irq(gf66xx_dev->spi->irq);
			  //enable_irq_wake(gf66xx_dev->spi->irq);
		}
	} else {
		gf66xx_dbg("No device for minor %d\n", (int)iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

#ifdef GF66XX_FASYNC
static int gf66xx_fasync(int fd, struct file *filp, int mode)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	int ret;

	FUNC_ENTRY();
	ret = fasync_helper(fd, filp, mode, &gf66xx_dev->async);
	FUNC_EXIT();
	gf66xx_dbg("ret = %d\n", ret);
	return ret;
}
#endif

static int gf66xx_release(struct inode *inode, struct file *filp)
{
	struct gf66xx_dev *gf66xx_dev;
	int			status = 0;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);
	gf66xx_dev = filp->private_data;
	filp->private_data = NULL;

	/*last close??*/
	gf66xx_dev->users --;
	if(!gf66xx_dev->users) {
		
		gf66xx_dbg("disble_irq. irq = %d\n", (int)gf66xx_dev->spi->irq);
		disable_irq(gf66xx_dev->spi->irq);
		//disable_irq_wake(gf66xx_dev->spi->irq);
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

static const struct file_operations gf66xx_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	gf66xx_write,
	.read =		gf66xx_read,
	.unlocked_ioctl = gf66xx_ioctl,
	.compat_ioctl=  gf66xx_compat_ioctl,
	.open =		gf66xx_open,
	.release =	gf66xx_release,
	.poll   = gf66xx_poll,
#ifdef GF66XX_FASYNC
	.fasync = gf66xx_fasync,
#endif
};

#if FW_UPDATE
static int isUpdate(struct gf66xx_dev *gf66xx_dev)
{
    unsigned char version[16];
    unsigned short ver_fw = 0;
    unsigned char* fw = GF66XX_FW;
    unsigned char fw_running = 0;
    unsigned short ver_file = 0;
    return 0;
    
    gf66xx_spi_read_bytes(gf66xx_dev, 0x41e4, 1, gf66xx_dev->buffer);
    fw_running = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
    if(fw_running == 0xbe) {
        /*firmware running*/
        ver_file = (*(fw+12))<<8 | (*(fw+13)); //get the fw version in the i file;
        /*In case we want to upgrade to a special firmware. Such as debug firmware.*/
        if(ver_file != 0x5a5a) {
            gf66xx_spi_read_bytes(gf66xx_dev,0x8000,16,gf66xx_dev->buffer);
            memcpy(version, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, 16);
            if(memcmp(version, GF816_PID, 5)) {
                //pr_info("version: 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n", version[0], version[1],version[2], version[3], version[4], version[5]);
                return 1;
            }
            if((version[7]>9) || (((version[8]&0xF0)>>4)>9) || ((version[8]&0x0F)>9)) {
                //pr_info("version: 7-0x%x; 8-0x%x\n", version[7], version[8]);
                return 1;
            }
            ver_fw = ((version[7]<<4)<<8) | (version[8]<<4); //get the current fw version
            if(ver_fw >= ver_file){
                /*If the running firmware is or ahead of the file's firmware. No need to do upgrade.*/
                return 0;
            }
        }
       //pr_info("Current Ver: 0x%x, Upgrade to Ver: 0x%x\n", ver_fw, ver_file);
    }else {
        /*no firmware.*/
       //pr_info("No running firmware. Value = 0x%x\n", fw_running);
    }
    return 1;
}

static int gf66xx_fw_update_init(struct gf66xx_dev *gf66xx_dev)
{
	u8 retry_cnt = 5;
	u8 value[2];
	/*1.reset output low level and delay 2ms*/
	gpio_set_value(gf66xx_dev->reset_gpio, 0);
	mdelay(5);

	/*2.reset output high level to reset chip*/
	gpio_set_value(gf66xx_dev->reset_gpio, 1);

	/*3.delay 100ms*/
	mdelay(60);
#if 1
	while(retry_cnt--)
	{
		/*4.Hold SS51 and DSP(0x4180 == 0x0C)*/
		gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0x0c;
		gf66xx_spi_write_bytes(gf66xx_dev, 0x4180, 1, gf66xx_dev->buffer);
		gf66xx_spi_read_bytes(gf66xx_dev, 0x4180,1, gf66xx_dev->buffer);
		value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
		gf66xx_spi_read_bytes(gf66xx_dev, 0x4030,1, gf66xx_dev->buffer);
		value[1] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
		pr_info("[info] %s hold SS51 and DSP,0x4180=0x%x,0x4030=0x%x,retry_cnt=%d !\n",__func__,value[0] ,value[1],(int)retry_cnt);
		if (value[0] == 0x0C)/* && value[1] == 0*/
		{
			pr_info("[info] %s hold SS51 and DSP successfully!\n",__func__);
			break;
		}
	}
	pr_info("Hold retry_cnt=%d\n",(int)retry_cnt);
	/*5.enable DSP and MCU power(0x4010 == 0x00)*/
	gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0x0;
	gf66xx_spi_write_bytes(gf66xx_dev, 0x4010, 1, gf66xx_dev->buffer);
#endif
	return 1;
}
#endif

#if ESD_PROTECT
static void gf66xx_timer_work(struct work_struct *work)
{
#if 1
	unsigned char value[3];
	int ret = 0;
	struct gf66xx_dev *gf66xx_dev;
#if FW_UPDATE
	unsigned char* p_fw = GF66XX_FW;
#endif
	if(work == NULL)
	{
		pr_info("[info] %s wrong work\n",__func__);
		return;
	}
	gf66xx_dev = container_of(work, struct gf66xx_dev, spi_work);
	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_dev->spi->max_speed_hz= 1000*1000;
	spi_setup(gf66xx_dev->spi);
	gf66xx_spi_read_bytes(gf66xx_dev, 0x8040,1, gf66xx_dev->buffer);
	value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, 1, gf66xx_dev->buffer);
	value[1] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	gf66xx_spi_read_bytes(gf66xx_dev, 0x8046, 1, gf66xx_dev->buffer); //&& value[1] == 0x47 && value[2] == 0x56
	value[2] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	if(value[0] == 0xC6&& value[1] == 0x47){
		//pr_info("######Jason no need to kick dog 111!\n");
		gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0xAA;
		gf66xx_spi_write_bytes(gf66xx_dev, 0x8040, 1, gf66xx_dev->buffer);
	}else{
		gf66xx_spi_read_bytes(gf66xx_dev, 0x8040,1, gf66xx_dev->buffer);
		value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
		gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, 1, gf66xx_dev->buffer);
		value[1] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
		gf66xx_spi_read_bytes(gf66xx_dev, 0x8046, 1, gf66xx_dev->buffer); //&& value[1] == 0x47 && value[2] == 0x56
		value[2] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
		if(value[0] == 0xC6&& value[1] == 0x47){
			//pr_info("######Jason no need to kick dog 222!\n");
			gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0xAA;
			gf66xx_spi_write_bytes(gf66xx_dev, 0x8040, 1, gf66xx_dev->buffer);
		} else {
            unsigned char version[16] = {0};
			pr_info("######Jason hardware works abnormal, do reset! 0x8040 = 0x%x  0x8000 = 0x%x 0x8046 = 0x%x \n",value[0],value[1],value[2]);
            mutex_unlock(&gf66xx_dev->buf_lock);
			disable_irq(gf66xx_dev->spi->irq);
            mutex_lock(&gf66xx_dev->buf_lock);
			gf66xx_hw_reset(gf66xx_dev);
            msleep(300);
			gf66xx_spi_read_bytes(gf66xx_dev, 0x41e4, 1, gf66xx_dev->buffer);
			value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
            gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, 16, gf66xx_dev->buffer);
            memcpy(version, gf66xx_dev->buffer + GF66XX_RDATA_OFFSET, 16);
           //pr_info("[info] %s read 0x41e4 finish value = 0x%x, version[0]=%x\n", __func__,value[0], version[0]);
 
#if 1
            if((value[0] != 0xbe) || memcmp(version, GF816_PID, 5)) {
				regulator_disable(gf66xx_power_vdd);
				mdelay(10);
				ret = regulator_enable(gf66xx_power_vdd);
				if(ret)
					pr_info("[info] %s power on fail\n", __func__);
			}
#endif
#if FW_UPDATE
            if((value[0] != 0xbe) || memcmp(version, GF816_PID, 5)) {
                gf66xx_spi_read_bytes(gf66xx_dev, 0x41e4, 1, gf66xx_dev->buffer);
                value[0] = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
 
				if(value[0] != 0xbe) {
					/***********************************firmware update*********************************/
					pr_info("[info] %s firmware update start\n", __func__);
					del_timer_sync(&gf66xx_dev->gf66xx_timer);
					gf66xx_fw_update_init(gf66xx_dev);
					ret = gf66xx_fw_update(gf66xx_dev, p_fw, FW_LENGTH);
					gf66xx_hw_reset(gf66xx_dev);
					gf66xx_dev->gf66xx_timer.expires = jiffies + 2 * HZ;
					add_timer(&gf66xx_dev->gf66xx_timer);
				}
			}

			/***************************************update config********************************/
			pr_info("[info] %s write config \n", __func__);
			gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = 0xAA;
			ret = gf66xx_spi_write_bytes(gf66xx_dev, 0x8040, 1, gf66xx_dev->buffer);
			if(!ret)
				pr_info("[info] %s write 0x8040 fail\n", __func__);
			
            memcpy(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET, gf66xx_config, GF66XX_CFG_LEN);
	        ret = gf66xx_spi_write_bytes(gf66xx_dev, GF66XX_CFG_ADDR, GF66XX_CFG_LEN, gf66xx_dev->buffer);
			if(ret <= 0)
				pr_info("[info] %s write config fail\n", __func__);
#endif
			enable_irq(gf66xx_dev->spi->irq);
		}
	}
		mutex_unlock(&gf66xx_dev->buf_lock);
#endif
}


static void gf66xx_timer_func(unsigned long arg)
{
	struct gf66xx_dev *gf66xx_dev = (struct gf66xx_dev*)arg;
	if(gf66xx_dev == NULL)
	{
		pr_info("[info] %s can't get the gf66xx_dev\n",__func__);
		return;
	}
	schedule_work(&gf66xx_dev->spi_work);
	mod_timer(&gf66xx_dev->gf66xx_timer, jiffies + 2*HZ);
}
#endif

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *gf66xx_spi_class;

/*-------------------------------------------------------------------------*/

#if 0

static ssize_t gf66xx_irq_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t size)
{
	int val;

	sscanf(buf, "%d", &val);

	if (val == 1)
    {
        enable_irq(gf66xx_dev_p->spi->irq);
    }
	else
	{
	    disable_irq(gf66xx_dev_p->spi->irq);
	}

	return size;
}

static ssize_t gf66xx_irq_show(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t size)
{

	return size;
}

static struct kobj_attribute gf66xx_irq_attribute   = __ATTR(gf66xx_irq,	0664,	   gf66xx_irq_show, gf66xx_irq_store);

static struct attribute *attrs[] = {
	&gf66xx_irq_attribute.attr,
	NULL, /* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *id_kobj;

static int __init id_init(void)
{
	int retval;

	id_kobj = kobject_create_and_add("fp_gf66xx", kernel_kobj);
	if (!id_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(id_kobj, &attr_group);
	if (retval)
		kobject_put(id_kobj);

	return retval;
}
#endif

//static int __devinit gf66xx_probe(struct spi_device *spi)
static int  gf66xx_probe(struct spi_device *spi)

{
	struct gf66xx_dev	*gf66xx_dev;
	int			status;
	unsigned long		minor;
	int err = 0;
	//add daiss1
	//int ret;
	/*qcom volatage start*/
	int rc;
	unsigned char version;
    	int i;
 	struct device_node *np = spi->dev.of_node;
	u32 temp_val;
	FUNC_ENTRY();
        
/* Allocate driver data */
	gf66xx_dev = kzalloc(sizeof(*gf66xx_dev), GFP_KERNEL);
	if (!gf66xx_dev){
		gf66xx_dbg("Failed to alloc memory for gf66xx device.\n");
		FUNC_EXIT();
		return -ENOMEM;
	}
	else
	{
	    gf66xx_dev_p = gf66xx_dev;
	}
	
	
    if (0)
    {
    
    	gf66xx_dev->avdd_gpio = of_get_named_gpio_flags(np, "goodix,gpio_3p3v",0, &temp_val);
    	if(gf66xx_dev->avdd_gpio == 0) 
    	{
    		gf66xx_dbg("Failed to get avdd gpio.\n");
    		return 0;
    	}
            gpio_direction_output(gf66xx_dev->avdd_gpio, 1);

    	gpio_set_value(gf66xx_dev->avdd_gpio, 1);	
    	mdelay(5);

    }


	gf66xx_power_vdd= regulator_get(&spi->dev, "vdd");   //set vreg_l17 3.3V
	if (IS_ERR(gf66xx_power_vdd)) {
		rc = PTR_ERR(gf66xx_power_vdd);
		dev_err(&spi->dev,
			"---- Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(gf66xx_power_vdd) > 0) {
		rc = regulator_set_voltage(gf66xx_power_vdd, 2850000,	3300000);
		if (rc) {
			dev_err(&spi->dev,
				"----regulator set_vtg failed rc=%d\n", rc);
			regulator_put(gf66xx_power_vdd);
			return rc ;
		}
	}
	rc = regulator_set_optimum_mode(gf66xx_power_vdd, 60000);
	if (rc < 0) {
		pr_err("set_optimum_mode l21 failed, rc=%d\n", rc);
		return rc ;
	}
    
	rc = regulator_enable(gf66xx_power_vdd);
	if (rc) {
		dev_err(&spi->dev,
			"----Regulator gf66xx_power_vdd enable failed rc=%d\n", rc);
		return rc ;
	}
	msleep(10);
	/*qcom volatage end*/


	pr_info( KERN_ERR "Power ok.\n");

         
	

	/* Initialize the driver data */
	gf66xx_dev->spi = spi;
	spin_lock_init(&gf66xx_dev->spi_lock);
	mutex_init(&gf66xx_dev->buf_lock);

	INIT_LIST_HEAD(&gf66xx_dev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		gf66xx_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf66xx_spi_class, &spi->dev, gf66xx_dev->devt,
				    gf66xx_dev, DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf66xx_dev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);
	if (status == 0){
		pr_info( KERN_ERR "Malloc.\n");
		gf66xx_dev->buffer = kzalloc(bufsiz + GF66XX_RDATA_OFFSET, GFP_KERNEL);
		if(gf66xx_dev->buffer == NULL) {
			kfree(gf66xx_dev);
			status = -ENOMEM;
			goto err;
		}

		if(spi->dev.of_node){				
			

			/* reset, irq gpio info */
			gf66xx_dev->reset_gpio = of_get_named_gpio_flags(np, "goodix-gpio-rst",
						0, &temp_val);
			//gf66xx_dev->reset_gpio = 121;
			if(gf66xx_dev->reset_gpio == 0) {
				gf66xx_dbg("Failed to get reset gpio.\n");
				return 0;
			}
			gf66xx_dev->irq_gpio = of_get_named_gpio_flags(np, "goodix-gpio-drdy",
					0, &temp_val);
			//gf66xx_dev->irq_gpio = 120;
			if(gf66xx_dev->irq_gpio == 0) {
				gf66xx_dbg("Failed to get irq gpio.\n");
				return 0;
			}
			gf66xx_dbg("gf66xx_dev->irq_gpio NO. = %d\n", (int)gf66xx_dev->irq_gpio);
			gf66xx_dbg("gf66xx_dev->reset_gpio NO. = %d\n", (int)gf66xx_dev->reset_gpio);



            gf66xx_pinctrl_init(&spi->dev);
            
            rc = pinctrl_select_state(gf66xx_pctrl.pinctrl, gf66xx_pctrl.gpio_state_active); 
            if (rc)
            {
			    gf66xx_dbg("Failed to set pin to gpio_state_active state.\n");
            }
            else
            {                    
                 gf66xx_dbg("Success to set pin to gpio_state_active state.\n");
            }












			//gf66xx_dev->miso_gpio = of_get_named_gpio_flags(np, "goodix,miso-gpio",
			//		0, &temp_val);	
			//gf66xx_dev->miso_gpio = 1;
			//if(gf66xx_dev->miso_gpio == 0) {
			//	gf66xx_dbg("Failed to get irq gpio.\n");
			//	return 0;
			//}
		}else{
			pr_info("gf66xx init GPIO failed.\n");
		}
		spi_set_drvdata(spi, gf66xx_dev);
		gf66xx_dev->spi = spi;
/*
		gf66xx_dev->spi_wq = create_singlethread_workqueue("gf66xx_spi_msg");
		if(gf66xx_dev->spi_wq == NULL) {
			pr_err("Failed to create thread.\n");
			kfree(gf66xx_dev->buffer);
			kfree(gf66xx_dev);
			status = -EFAULT;
			goto err;
		}
		INIT_WORK(&gf66xx_dev->spi_work, gf66xx_spi_work);
*/
		/*register device within input system.*/
		gf66xx_dev->input = input_allocate_device();
		if(gf66xx_dev->input == NULL) {
			gf66xx_dbg("Failed to allocate input device.\n");
			status = -ENOMEM;
			free_pages((unsigned long)gf66xx_dev->buffer, get_order(bufsiz + GF66XX_RDATA_OFFSET));
			kfree(gf66xx_dev);
			goto err;
		}

		__set_bit(EV_KEY, gf66xx_dev->input->evbit);
		__set_bit(KEY_HOME, gf66xx_dev->input->keybit);
		__set_bit(KEY_POWER, gf66xx_dev->input->keybit);

		//gf66xx_dev->input->name = "tiny4412-key";
		gf66xx_dev->input->name = "qwerty";
		if(input_register_device(gf66xx_dev->input)) {
			gf66xx_dbg("Failed to register input device.\n");
		}
		/*setup gf66xx configurations.*/
		gf66xx_dbg("Setting gf66xx device configuration.\n");
		gf66xx_irq_cfg(gf66xx_dev);
		
		/*SPI parameters.*/
		gf66xx_dev->spi->mode =SPI_MODE_0; //CPOL=CPHA=0
		gf66xx_dev->spi->max_speed_hz = 8*1000*1000; 
		gf66xx_dev->spi->irq =	gpio_to_irq(gf66xx_dev->irq_gpio); 
		gf66xx_dev->spi->bits_per_word = 8; 

		spi_setup(gf66xx_dev->spi);
		gf66xx_hw_reset(gf66xx_dev);

	    msleep(300);	
#if GF66XX_SPI_TEST  /*read version*/
		pr_info("read version to test spi communication.\n");
		gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, 16, gf66xx_dev->buffer);
		for(i=0; i<16; i++) {
			pr_info("0x%2x,",gf66xx_dev->buffer[i]);
		}
#endif /*read version*/
        
		enable_irq_wake(gf66xx_dev->spi->irq);		
#if FW_UPDATE
		if(isUpdate(gf66xx_dev)) {
			unsigned char* fw = GF66XX_FW;
			/*Do upgrade action.*/
			gf66xx_fw_update_init(gf66xx_dev);
			gf66xx_fw_update(gf66xx_dev, fw, FW_LENGTH);
			gf66xx_hw_reset(gf66xx_dev);
		}
#endif

#if SEND_CONFIG
		/*write config*/
		memcpy(gf66xx_dev->buffer + GF66XX_WDATA_OFFSET, gf66xx_config, GF66XX_CFG_LEN);
		rc = gf66xx_spi_write_bytes(gf66xx_dev, GF66XX_CFG_ADDR, GF66XX_CFG_LEN, gf66xx_dev->buffer);
		if(rc <= 0)
			pr_info("[info] %s write config fail\n", __func__);
#endif		

		gf66xx_spi_read_byte(gf66xx_dev,0x8000,&version);
		pr_info("gf66xx version test :%0x\n",version);
		if(version != 0x47)
		{
			pr_err("[ERR] %s device detect error!!\n", __func__);
			status = -ENODEV;
			//goto err;
		}
		gf66xx_dbg("gf66xx interrupt NO. = %d\n", (int)gf66xx_dev->spi->irq);

#if 1
		err = request_threaded_irq(spi->irq, NULL, gf66xx_irq, 
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"gf66xx-spi", gf66xx_dev);
#else
		err = request_irq(gf66xx_dev->spi->irq, gf66xx_irq, 
				IRQ_TYPE_EDGE_RISING,//IRQ_TYPE_LEVEL_HIGH,
				dev_name(&gf66xx_dev->spi->dev), gf66xx_dev);
#endif
		if(!err) {
		    gf66xx_dbg("disable irq\n");
			disable_irq(gf66xx_dev->spi->irq);
			//disable_irq_wake(gf66xx_dev->spi->irq);
		}
		else
		{
            gf66xx_dbg("enable irq\n");
		    enable_irq(gf66xx_dev->spi->irq);		    
		}
		
		mdelay(500);

#if ESD_PROTECT
		INIT_WORK(&gf66xx_dev->spi_work, gf66xx_timer_work);
		init_timer(&gf66xx_dev->gf66xx_timer);
		gf66xx_dev->gf66xx_timer.function = gf66xx_timer_func;
		gf66xx_dev->gf66xx_timer.expires = jiffies + 20*HZ;
		gf66xx_dev->gf66xx_timer.data = (unsigned long)gf66xx_dev;
		add_timer(&gf66xx_dev->gf66xx_timer);
#endif	
	}
	else
		kfree(gf66xx_dev);

err:
	FUNC_EXIT();
	return status;
}

//static int __devexit gf66xx_remove(struct spi_device *spi)
static int  gf66xx_remove(struct spi_device *spi)
{
	struct gf66xx_dev	*gf66xx_dev = spi_get_drvdata(spi);
	FUNC_ENTRY();

	/* make sure ops on existing fds can abort cleanly */
	if(gf66xx_dev->spi->irq) {
		free_irq(gf66xx_dev->spi->irq, gf66xx_dev);
	}
	spin_lock_irq(&gf66xx_dev->spi_lock);
	gf66xx_dev->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&gf66xx_dev->spi_lock);
/*
	if(gf66xx_dev->spi_wq != NULL) {
		flush_workqueue(gf66xx_dev->spi_wq);
		destroy_workqueue(gf66xx_dev->spi_wq);
	}
	*/
	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf66xx_dev->device_entry);
	device_destroy(gf66xx_spi_class, gf66xx_dev->devt);
	clear_bit(MINOR(gf66xx_dev->devt), minors);
	if (gf66xx_dev->users == 0) {
		if(gf66xx_dev->input != NULL)
			input_unregister_device(gf66xx_dev->input);

		if(gf66xx_dev->buffer != NULL)
			kfree(gf66xx_dev->buffer);
		kfree(gf66xx_dev);
	}
	mutex_unlock(&device_list_lock);

	FUNC_EXIT();
	return 0;
}

static int gf66xx_suspend_test(struct device *dev)
{
    //pr_info(KERN_ERR"gf66xx_suspend_test.\n");
     return 0;
}

static int gf66xx_resume_test(struct device *dev)
{
    //pr_info(KERN_ERR"gf66xx_resume_test.\n");
     return 0;
}
static const struct dev_pm_ops gx_pm = {
	.suspend = gf66xx_suspend_test,
	.resume = gf66xx_resume_test
};
//#ifdef CONFIG_OF
static struct of_device_id gx_match_table[] = {
        { .compatible = "goodix,gf66xx",},
        { },
};
//#else
//#define gx_match_table NULL
//#endif

static struct spi_driver gf66xx_spi_driver = {
	.driver = {
		.name	= SPI_DEV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.pm		= &gx_pm,
		.of_match_table = gx_match_table,
	},
	.probe	= gf66xx_probe,
	//.remove	= __devexit_p(gf66xx_remove)
	.remove	= gf66xx_remove,
};


/*-------------------------------------------------------------------------*/

static int __init gf66xx_init(void)
{
	int status;
	FUNC_ENTRY();

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
*/
         
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf66xx_fops);
	if (status < 0){
		pr_info("Failed to register char device!\n");
		FUNC_EXIT();
		return status;
	}
	gf66xx_spi_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gf66xx_spi_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf66xx_spi_driver.driver.name);
		pr_info("Failed to create class.\n");
		FUNC_EXIT();
		return PTR_ERR(gf66xx_spi_class);
	}
	        
	status = spi_register_driver(&gf66xx_spi_driver);
	if (status < 0) {
		class_destroy(gf66xx_spi_class);
		unregister_chrdev(SPIDEV_MAJOR, gf66xx_spi_driver.driver.name);
		pr_info("Failed to register SPI driver.\n");
	}


	FUNC_EXIT();
	return status;
}

module_init(gf66xx_init);

static void __exit gf66xx_exit(void)
{
    FUNC_ENTRY();
	spi_unregister_driver(&gf66xx_spi_driver);
	class_destroy(gf66xx_spi_class);
	unregister_chrdev(SPIDEV_MAJOR, gf66xx_spi_driver.driver.name);
	FUNC_EXIT();
}
module_exit(gf66xx_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf66xx-spi");

