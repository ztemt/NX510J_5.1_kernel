#ifndef __GF66XX_SPI_H
#define __GF66XX_SPI_H

#include <linux/types.h>

/********************GF66XX Mapping**********************/
#define GF66XX_BASE        (0x8000)
#define GF66XX_OFFSET(x)   (GF66XX_BASE + x)

#define GF66XX_VERSION			GF66XX_OFFSET(0)
#define GF66XX_CONFIG_DATA  	GF66XX_OFFSET(0x40)
#define GF66XX_MIXER_DATA		GF66XX_OFFSET(0x140)
#define GF66XX_BUFFER_STATUS	GF66XX_OFFSET(0x140)
#define GF66XX_BUFFER_DATA		GF66XX_OFFSET(0x141)
#define	GF66XX_MODE_STATUS		GF66XX_OFFSET(0x043)

#define GF66XX_BUF_STA_MASK		(0x1<<7)
#define	GF66XX_BUF_STA_READY		(0x1<<7)
#define	GF66XX_BUF_STA_BUSY		(0x0<<7)

#define	GF66XX_IMAGE_MASK			(0x1<<6)
#define	GF66XX_IMAGE_ENABLE		(0x1)
#define	GF66XX_IMAGE_DISABLE		(0x0)

#define	GF66XX_KEY_MASK			(0x1<<5)
#define	GF66XX_KEY_ENABLE			(0x1)
#define	GF66XX_KEY_DISABLE		(0x0)

#define	GF66XX_KEY_STA			(0x1<<4)

#define	GF66XX_IMAGE_MODE			(0x00)
#define	GF66XX_KEY_MODE			(0x01)
#define GF66XX_SLEEP_MODE			(0x02)
#define GF66XX_FF_MODE			(0x03)
#define GF66XX_DEBUG_MODE			(0x56)
/**********************GF66XX ops****************************/
#define GF66XX_W          0xF0
#define GF66XX_R          0xF1
#define GF66XX_WDATA_OFFSET	(0x3)
#define GF66XX_RDATA_OFFSET	(0x4)
/**********************************************************/

/**********************IO Magic**********************/
#define  GF66XX_IOC_MAGIC    'g'  //define magic number

struct gf66xx_ioc_transfer {
	u8	cmd;
	u8 reserve;
	u16	addr;
	u32 len;
	u8 *buf;
};
//define commands
/*read/write GF66XX registers*/
#define  GF66XX_IOC_CMD	_IOWR(GF66XX_IOC_MAGIC, 1, struct gf66xx_ioc_transfer)
#define  GF66XX_IOC_REINIT	_IO(GF66XX_IOC_MAGIC, 0)
#define  GF66XX_IOC_SETSPEED	_IOW(GF66XX_IOC_MAGIC, 2, u32)

#define  GF66XX_IOC_MAXNR    3

struct gf66xx_pinctrl_info { 
	struct pinctrl *pinctrl; 
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};


struct gf66xx_dev {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct input_dev        *input;

	struct workqueue_struct *spi_wq;
	struct work_struct     spi_work;
	struct timer_list 	gf66xx_timer;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned		users;
	u8			*buffer;
	u8			buf_status;
	int			miso_gpio;
        int			avdd_gpio;
#ifdef GF66XX_FASYNC
	struct  fasync_struct *async;
#endif
};
/*******************Refering to hardware platform****************************
#define 	GF66XX_RST_PIN   	EXYNOS4_GPX1(5)
#define 	GF66XX_IRQ_PIN   	EXYNOS4_GPX1(4)
#define 	GF66XX_IRQ_NUM   	gpio_to_irq(GF66XX_IRQ_PIN)
#define		GF66XX_MISO_PIN	EXYNOS4_GPB(2)
*/
#endif
