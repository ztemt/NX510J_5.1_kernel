/*
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LM5802_H__
#define __LM5802_H__

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

/* Registers */
#define LM5802_REG_APP_STATUS			0x00
#define LM5802_IN_TRAINING			BIT(3)
#define LM5802_APPCODE_MASK			0xFF

#define LM5802_REG_BOOT_STATUS			0x01
#define LM5802_BOOT_READY			BIT(0)
#define LM5802_BOOT_INVALID_FW			BIT(3)
#define LM5802_BOOT_VALID_FW			BIT(4)
#define LM5802_BOOTCODE_MASK			0xFF

#define LM5802_REG_RESET			0x02

#define LM5802_REG_APPVER			0x04

#define LM5802_REG_ENABLE_DETECT		0x11
#define LM5802_USER1_ENABLED			BIT(1)
#define LM5802_USER2_ENABLED			BIT(2)
#define LM5802_USER3_ENABLED			BIT(3)
#define LM5802_USER4_ENABLED			BIT(4)
#define LM5802_USER_INDEP_ENABLED		BIT(5)

#define LM5802_REG_POWER_MODE			0x12

#define LM5802_REG_ADVANCED_SETTINGS		0x13
#define LM5802_TRIGGER_MODE_INDEX		2
#define LM5802_BLACKOUT_TIME_INDEX		14
#define LM5802_CHECKSUM_INDEX			15
#define LM5802_SINGLE_TRIGGER			0
#define LM5802_DUAL_TRIGGER			1
#define LM5802_MAX_BLACKOUT			25500
#define LM5802_ADV_SETTINGS_LENGTH		16

#define LM5802_REG_MICBIAS_CTRL			0x15
#define LM5802_MAX_MICBIAS			2800
#define LM5802_MIN_MICBIAS			1300
#define LM5802_MICBIAS_STEP			100

#define LM5802_REG_ABORT_TRAINING		0x18

#define LM5802_REG_TRAINING_INFO		0x19
#define LM5802_TRAINING_COUNT_MASK		(BIT(0) | BIT(1))
#define LM5802_TRAINING_SCORE_MASK		0xFF00
#define LM5802_TRAINING_SCORE_SHIFT		8
#define LM5802_TRAINING_IS_GOOD			0
#define LM5802_REASON_HIGH_VOLUME		BIT(0)
#define LM5802_REASON_LOW_VOLUME		BIT(1)
#define LM5802_REASON_LONG_CMD			BIT(2)
#define LM5802_REASON_SHORT_CMD			BIT(3)
#define LM5802_REASON_NOISY_ENV			BIT(4)
#define LM5802_REASON_SIMPLE_CMD		BIT(5)
#define LM5802_REASON_INCONSISTENT		BIT(7)
#define LM5802_LAST_PHRASE			3

#define LM5802_REG_XFER_CTRL			0x1A
#define LM5802_APP_XFER_RESUME			BIT(0)

#define LM5802_REG_FLUSH_ENROLLMENT		0x1B

#define LM5802_REG_VALIDATE_ENROLLMENT		0x1C

#define LM5802_REG_ENROLLMENT_SIZE		0x20

#define LM5802_REG_ENROLLMENT_BASE		0x21

#define LM5802_REG_INTERRUPT_STATUS		0x31
#define LM5802_IRQ_TRAINING_DONE		BIT(0)
#define LM5802_IRQ_APPCODE_READY		BIT(4)
#define LM5802_IRQ_USER1_CMD			BIT(9)
#define LM5802_IRQ_USER2_CMD			BIT(10)
#define LM5802_IRQ_USER3_CMD			BIT(11)
#define LM5802_IRQ_USER4_CMD			BIT(12)
#define LM5802_IRQ_USER_INDEP_CMD		BIT(13)

#define LM5802_REG_START_TRAINING		0x37

#define LM5802_REG_EVENT_CTRL			0x60
#define LM5802_NOTIFY_CHECKSUM_COMPLETE		BIT(0)

#define LM5802_REG_FW_INFO			0x61
#define LM5802_BOOT_XFER_RESUME			BIT(2)

#define LM5802_REG_FW				0x62

#define LM5802_REG_FW_DECRYPT			0x63

#define LM5802_REG_FW_START			0x64

enum lm5802_enrollment_id {
	LM5802_INVALID_ID,
	LM5802_ID_USER1,
	LM5802_ID_USER2,
	LM5802_ID_USER3,
	LM5802_ID_USER4,
	LM5802_ID_USER_INDEP,
};

enum lm5802_training_mode {
	LM5802_TRAINING_ABORT,
	LM5802_TRAINING_USER1,
	LM5802_TRAINING_USER2,
	LM5802_TRAINING_USER3,
	LM5802_TRAINING_USER4,
};

/**
 * struct lm5802_platform_data
 * @fw_name: Firmware file name
 * @en_gpio: GPIO number for HW enable pin (optional)
 * @irq1_gpio: GPIO number for interrupt pin 1.
 * @irq2_gpio: GPIO number for Interrupt pin 2.
 * @micbias: Micbias voltage (Range: 1300mV to 2800mV, step: 100mV)
 * @blackout_time: Black out timer (Range: 0 to 25500ms, step: 100ms)
 * @is_chunk_xfer: If the host has size limitation of I2C data transfer,
 *                 then set true.
 */
struct lm5802_platform_data {
	const char *fw_name;
	int en_gpio;
	int irq1_gpio;
	int irq2_gpio;
	int micbias;
	int blackout_time;
	bool is_chunk_xfer;
};

/**
 * struct lm5802
 * @dev: Parent device pointer
 * @cl: I2C client structure
 * @pdata: LM5802 platform data
 * @irq_status: Interrupt status
 * @irq_work: Workqueue for handling interrupts
 * @which_irq: Which interrupt is generated
 * @irq1: Interrupt 1
 * @irq2: Interrupt 2
 * @size_userdata: Size of user data
 * @training_mode: Training mode selection
 * @enrollment_id: User enrollment ID.
 *                 Used for enabling detection, user data download and
 *                 voice detected event.
 * @training_result: Training result code.
 *                   0 is good. Otherwise, it is a reason code.
 * @dbgfile: Dentry structure for debug file system
 */
struct lm5802 {
	struct device *dev;
	struct i2c_client *cl;
	struct lm5802_platform_data *pdata;

	unsigned int irq_status;
	struct delayed_work irq_work;
	int which_irq;
	int irq1;
	int irq2;

	unsigned int size_userdata;
	enum lm5802_training_mode training_mode;
	enum lm5802_enrollment_id enrollment_id;
	int training_result;

	struct dentry *dbgfile;
};

/* Register access functions */
int lm5802_reg_write(struct lm5802 *lm5802, u8 reg, u8 *val, int len);
int lm5802_reg_read(struct lm5802 *lm5802, u8 reg, u8 *val, int len);
int lm5802_reg_update_bit(struct lm5802 *lm5802, u8 reg, u8 mask, u8 val);

/* Device attributes */
int lm5802_add_sysfs(struct lm5802 *lm5802);
void lm5802_remove_sysfs(struct lm5802 *lm5802);

/* Load request from user space */
int lm5802_load_by_user(struct lm5802 *lm5802);

/* Compute checksum prior to sending data */
u8 lm5802_compute_checksum(const u8 *val, int len);

#endif
