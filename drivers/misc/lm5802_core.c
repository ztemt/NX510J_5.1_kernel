/*
 * LM5802 Core Driver
 *
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/platform_data/lm5802.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define LM5802_DELAY_JIFFIES		msecs_to_jiffies(50)

#define LM5802_FW_FILE_ID0		0xAA
#define LM5802_FW_FILE_ID1		0x16
#define LM5802_FW_HEADER_OFFSET		4
#define LM5802_FW_DATA_OFFSET		21
#define LM5802_FW_SIZE_INDEX		3

#define LM5802_MAX_RETRY_COUNT		3
#define LM5802_MAX_XFER_LEN		254
#define LM5802_MAX_BUF			(LM5802_MAX_XFER_LEN + 1)

#define LM5802_UEVENT_ARG1_SIZE		32
#define LM5802_UEVENT_ARG2_SIZE		16
#define LM5802_UEVENT_ARGS_NUM		3

/*
 * This header information is inside the firmware.(offset is 4)
 * After the firmware is downloaded, header information should be updated
 * via the I2C.
 */
struct lm5802_firmware_header {
	unsigned short arm_size;
	unsigned short enroll_start;
	unsigned short enroll_size;
	unsigned short data_start;
	unsigned short data_size;
	unsigned short checksum_size;
	unsigned short ext_checksum_start;
	unsigned short ext_checksum_size;
	u8 ctrl;
};

static int lm5802_send_cmd_only(struct lm5802 *lm5802, u8 reg)
{
	struct i2c_msg msg[1];
	u8 buf;
	int ret;

	buf = reg;

	msg[0].addr = lm5802->cl->addr;
	msg[0].flags = 0;
	msg[0].buf = &buf;
	msg[0].len = 1;

	ret = i2c_transfer(lm5802->cl->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(lm5802->dev, "write err: %d", ret);
		return ret;
	}

	if (ret != ARRAY_SIZE(msg))
		return -EIO;

	return 0;
}

/*
 * lm5802_write/read_chunk()
 *
 * Some hosts have I2C transfer size limitation.
 * In this case, data should be split.
 * At this moment, maximum size length is 254. This limit depends on the host.
 * So modification maybe required.
 */
static int lm5802_write_chunk(struct lm5802 *lm5802, u8 reg, u8 *val, int len)
{
	unsigned int offset = 0;
	unsigned int size;
	struct i2c_msg msg[1];
	u8 buf[LM5802_MAX_BUF];
	int ret;

	/* Simple write command has zero length */
	if (len == 0)
		return lm5802_send_cmd_only(lm5802, reg);

	while (len > 0) {
		if (len > LM5802_MAX_XFER_LEN)
			size = LM5802_MAX_XFER_LEN;
		else
			size = len;

		memset(buf, 0x0, sizeof(buf));
		buf[0] = reg;
		memcpy(&buf[1], val + offset, size);

		msg[0].addr = lm5802->cl->addr;
		msg[0].flags = 0;
		msg[0].buf = buf;
		msg[0].len = size + 1;

		ret = i2c_transfer(lm5802->cl->adapter, msg, ARRAY_SIZE(msg));
		if (ret < 0) {
			dev_err(lm5802->dev, "write err: %d", ret);
			return ret;
		}

		if (ret != ARRAY_SIZE(msg))
			return -EIO;

		len -= size;
		offset += size;
	}

	return 0;
}

static int lm5802_read_chunk(struct lm5802 *lm5802, u8 reg, u8 *val, int len)
{
	unsigned int offset = 0;
	unsigned int size;
	struct i2c_msg msg[2];
	u8 buf[LM5802_MAX_BUF];
	int ret;

	while (len > 0) {
		if (len > LM5802_MAX_XFER_LEN)
			size = LM5802_MAX_XFER_LEN;
		else
			size = len;

		memset(buf, 0x0, sizeof(buf));

		msg[0].addr = lm5802->cl->addr;
		msg[0].flags = 0;
		msg[0].buf = &reg;
		msg[0].len = sizeof(reg);

		msg[1].addr = lm5802->cl->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].buf = buf;
		msg[1].len = size;

		ret = i2c_transfer(lm5802->cl->adapter, msg, ARRAY_SIZE(msg));
		if (ret < 0) {
			dev_err(lm5802->dev, "read err: %d", ret);
			return ret;
		}

		if (ret != ARRAY_SIZE(msg))
			return -EIO;

		memcpy(val + offset, buf, size);

		len -= size;
		offset += size;
	}

	return 0;
}

/*
 * lm5802_write/read()_oneshot
 *
 * LM5802 I2C supports various length of R/W data.
 * So, memory allocation and free are required in functions.
 */
static int lm5802_write_oneshot(struct lm5802 *lm5802, u8 reg, u8 *val, int len)
{
	struct i2c_msg msg[1];
	int ret;
	u8 *buf;

	buf = kzalloc(len + 1, GFP_KERNEL);
	if (!buf) {
		dev_err(lm5802->dev, "write buffer allocation err");
		return -ENOMEM;
	}

	buf[0] = reg;
	memcpy(&buf[1], val, len);

	msg[0].addr = lm5802->cl->addr;
	msg[0].flags = 0;
	msg[0].buf = buf;
	msg[0].len = len + 1;

	ret = i2c_transfer(lm5802->cl->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(lm5802->dev, "write err: %d", ret);
		goto out;
	}

	if (ret != ARRAY_SIZE(msg))
		ret = -EIO;
	else
		ret = 0;
out:
	kfree(buf);
	return ret;
}

static int lm5802_read_oneshot(struct lm5802 *lm5802, u8 reg, u8 *val, int len)
{
	struct i2c_msg msg[2];
	int ret;
	u8 *buf;

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf) {
		dev_err(lm5802->dev, "read buffer allocation err");
		return -ENOMEM;
	}

	msg[0].addr = lm5802->cl->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);

	msg[1].addr = lm5802->cl->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = len;

	ret = i2c_transfer(lm5802->cl->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(lm5802->dev, "read err: %d", ret);
		goto out;
	}

	memcpy(val, buf, len);

	if (ret != ARRAY_SIZE(msg))
		ret = -EIO;
	else
		ret = 0;
out:
	kfree(buf);
	return ret;
}

int lm5802_reg_write(struct lm5802 *lm5802, u8 reg, u8 *val, int len)
{
	/*
	 * If the host has limited size of I2C transfer, send chunk data.
	 * Otherwise, send data at one time.
	 */

	if (lm5802->pdata->is_chunk_xfer)
		return lm5802_write_chunk(lm5802, reg, val, len);
	else
		return lm5802_write_oneshot(lm5802, reg, val, len);
}
EXPORT_SYMBOL_GPL(lm5802_reg_write);

int lm5802_reg_read(struct lm5802 *lm5802, u8 reg, u8 *val, int len)
{
	/*
	 * Like register write command,
	 * both transfer options are supported.
	 */

	if (lm5802->pdata->is_chunk_xfer)
		return lm5802_read_chunk(lm5802, reg, val, len);
	else
		return lm5802_read_oneshot(lm5802, reg, val, len);
}
EXPORT_SYMBOL_GPL(lm5802_reg_read);

int lm5802_reg_update_bit(struct lm5802 *lm5802, u8 reg, u8 mask, u8 val)
{
	int ret;
	u8 tmp = 0;
	u8 orig = 0;

	ret = lm5802_reg_read(lm5802, reg, &orig, 1);
	if (ret)
		return ret;

	tmp = orig & ~mask;
	tmp |= val & mask;

	return lm5802_reg_write(lm5802, reg, &tmp, 1);
}
EXPORT_SYMBOL_GPL(lm5802_reg_update_bit);

static bool lm5802_is_valid_firmware(const struct firmware *fw)
{
	if (fw->data[0] == LM5802_FW_FILE_ID0 &&
	    fw->data[1] == LM5802_FW_FILE_ID1)
		return true;
	else
		return false;
}

static int lm5802_update_firmware_header(struct lm5802 *lm5802,
					 const struct firmware *fw)
{
	struct lm5802_firmware_header header;
	const u8 size = fw->data[LM5802_FW_SIZE_INDEX];

	memcpy((u8 *)&header, fw->data + LM5802_FW_HEADER_OFFSET, size);

	/*
	 * LM5802_BOOT_XFER_RESUME is required when the platform has
	 * size limitation of I2C transfer data.
	 */

	if (lm5802->pdata->is_chunk_xfer)
		header.ctrl |= LM5802_BOOT_XFER_RESUME;

	return lm5802_reg_write(lm5802, LM5802_REG_FW_INFO, (u8 *)&header,
				size);
}

static int lm5802_update_firmware_data(struct lm5802 *lm5802,
				       const struct firmware *fw)
{
	unsigned int size = fw->size - LM5802_FW_DATA_OFFSET;

	/* Update firmware appcode data  */
	return lm5802_reg_write(lm5802, LM5802_REG_FW,
				(u8 *)(fw->data + LM5802_FW_DATA_OFFSET), size);
}

static int lm5802_enable_interrupt_for_checksum(struct lm5802 *lm5802)
{
	/* Enable interrupt after appcode checksum is done */
	return lm5802_reg_update_bit(lm5802, LM5802_REG_EVENT_CTRL,
				     LM5802_NOTIFY_CHECKSUM_COMPLETE,
				     LM5802_NOTIFY_CHECKSUM_COMPLETE);
}

static int lm5802_decrypt_firmware(struct lm5802 *lm5802)
{
	/* Firmware is encrypted, so descryption is required before running */
	return lm5802_reg_write(lm5802, LM5802_REG_FW_DECRYPT, NULL, 0);
}

static void lm5802_firmware_loaded(const struct firmware *fw, void *context)
{
	struct lm5802 *lm5802 = context;
	int ret;

	if (!fw) {
		dev_err(lm5802->dev, "Firwmware file is missing\n");
		return;
	}

	dev_info(lm5802->dev, "Firmware is downloaded. Size = %zu\n", fw->size);

	/*
	 * LM5802 firmware includes
	 *     - file ID (0xAA, 0x16)
	 *     - header
	 *     - data (actual app code)
	 *
	 * All section should be checked/updated before running it
	 */

	if (!lm5802_is_valid_firmware(fw)) {
		dev_err(lm5802->dev, "Firwmware file is invalid\n");
		goto out;
	}

	ret = lm5802_update_firmware_header(lm5802, fw);
	if (ret) {
		dev_err(lm5802->dev, "Update firmware info err: %d\n", ret);
		goto out;
	}

	ret = lm5802_update_firmware_data(lm5802, fw);
	if (ret) {
		dev_err(lm5802->dev, "Firmware load err: %d\n", ret);
		goto out;
	}

	ret = lm5802_enable_interrupt_for_checksum(lm5802);
	if (ret) {
		dev_err(lm5802->dev,
			"Can not enable checksum interrupt: %d\n", ret);
		goto out;
	}

	ret = lm5802_decrypt_firmware(lm5802);
	if (ret)
		dev_err(lm5802->dev, "Firmware decrypt err: %d\n", ret);

out:
	release_firmware(fw);
}

static int lm5802_request_firmware(struct lm5802 *lm5802)
{
	return request_firmware_nowait(THIS_MODULE, true,
				       lm5802->pdata->fw_name, lm5802->dev,
				       GFP_KERNEL, lm5802,
				       lm5802_firmware_loaded);
}

static int lm5802_start_firmware(struct lm5802 *lm5802)
{
	/* Set the appcode mode */
	return lm5802_reg_write(lm5802, LM5802_REG_FW_START, NULL, 0);
}

static bool lm5802_is_ready_to_download_firmware(u16 status)
{
	return status & LM5802_BOOT_READY;
}

static bool lm5802_is_ready_to_start_firmware(u16 status)
{
	return status & LM5802_BOOT_VALID_FW;
}

static bool lm5802_is_invalid_firmware(u16 status)
{
	return status & LM5802_BOOT_INVALID_FW;
}

static void lm5802_disable_hw(struct lm5802 *lm5802)
{
	if (lm5802->pdata->en_gpio > 0) {
		gpio_set_value(lm5802->pdata->en_gpio, 0);
		gpio_free(lm5802->pdata->en_gpio);
	}
}

static void lm5802_retry_to_request_firmware(struct lm5802 *lm5802)
{
	static unsigned int retry_count;

	if (retry_count++ < LM5802_MAX_RETRY_COUNT) {
		dev_warn(lm5802->dev, "Retry to request firmware: %d\n",
			 retry_count);
		lm5802_request_firmware(lm5802);
	} else {
		dev_err(lm5802->dev, "Firmware load failure. Disable HW..\n");
		retry_count = 0;
		lm5802_disable_hw(lm5802);
	}
}

static int lm5802_boot_handler(struct lm5802 *lm5802, u16 status)
{
	int ret;

	if (lm5802_is_ready_to_start_firmware(status)) {
		dev_info(lm5802->dev, "Start the firmware\n");
		ret = lm5802_start_firmware(lm5802);
		if (ret) {
			dev_err(lm5802->dev, "Start firmware err: %d\n", ret);
			return ret;
		}
	} else if (lm5802_is_invalid_firmware(status)) {
		lm5802_retry_to_request_firmware(lm5802);
	}

	return 0;
}

static int lm5802_get_training_info(struct lm5802 *lm5802)
{
	int ret;
	u16 val;

	ret = lm5802_reg_read(lm5802, LM5802_REG_TRAINING_INFO, (u8 *)&val, 2);
	if (ret)
		return -EINVAL;

	dev_info(lm5802->dev, "training status = 0x%.x\n", val);

	return val;
}

static int lm5802_set_micbias(struct lm5802 *lm5802)
{
	u8 index;
	unsigned int micbias = lm5802->pdata->micbias;

	/* If micbias is out of range, use the default voltage */
	if (micbias > LM5802_MAX_MICBIAS || micbias < LM5802_MIN_MICBIAS) {
		dev_info(lm5802->dev, "Default MICBIAS is used\n");
		return 0;
	}

	dev_info(lm5802->dev, "MICBIAS is set to %dmV\n", micbias);

	index = (LM5802_MAX_MICBIAS - micbias) / LM5802_MICBIAS_STEP;
	return lm5802_reg_write(lm5802, LM5802_REG_MICBIAS_CTRL, &index, 1);
}

static u8 lm5802_get_blackout_code(struct lm5802 *lm5802)
{
	int time = min_t(int, lm5802->pdata->blackout_time, LM5802_MAX_BLACKOUT);

	dev_info(lm5802->dev, "Blackout time is set to %d sec\n", time / 1000);

	return (u8)(time / 100);
}

static int lm5802_set_blackout_time(struct lm5802 *lm5802)
{
	int ret;
	u8 val[LM5802_ADV_SETTINGS_LENGTH] = { 0, };

	/* Read advanced settings register */
	ret = lm5802_reg_read(lm5802, LM5802_REG_ADVANCED_SETTINGS, (u8 *)&val,
			      sizeof(val));
	if (ret)
		return ret;

	/* Set blackout time */
	val[LM5802_BLACKOUT_TIME_INDEX] = lm5802_get_blackout_code(lm5802);

	/* Compute the checksum */
	val[LM5802_CHECKSUM_INDEX] =
		lm5802_compute_checksum(val, LM5802_ADV_SETTINGS_LENGTH - 1);

	/* Update advanced settings register */
	ret = lm5802_reg_write(lm5802, LM5802_REG_ADVANCED_SETTINGS,
			       (u8 *)&val, sizeof(val));
	if (ret)
		dev_err(lm5802->dev, "Can not update settings: %d", ret);

	return ret;
}

static int lm5802_disable_detection(struct lm5802 *lm5802)
{
    u8 val = 0;
    return lm5802_reg_write(lm5802, LM5802_REG_ENABLE_DETECT, &val, 1);
}


/* Report an event to the user space */
static void lm5802_report_event(struct lm5802 *lm5802)
{
	char arg1[LM5802_UEVENT_ARG1_SIZE];
	char arg2[LM5802_UEVENT_ARG2_SIZE];
	char *args[LM5802_UEVENT_ARGS_NUM];
	char *class;
	int code = 0;
	int irq_status = lm5802->irq_status;
	int training_info;

	switch (irq_status) {
	case LM5802_IRQ_APPCODE_READY:
		class = "APPCODE_READY";
		(void)lm5802_set_micbias(lm5802);
		(void)lm5802_set_blackout_time(lm5802);
        lm5802_disable_detection(lm5802);
		break;
	case LM5802_IRQ_USER_INDEP_CMD:
		class = "VOICE_TRIGGER";
		code  = LM5802_ID_USER_INDEP;
		break;
	case LM5802_IRQ_USER1_CMD:
		class = "VOICE_TRIGGER";
		code  = LM5802_ID_USER1;
		break;
	case LM5802_IRQ_USER2_CMD:
		class = "VOICE_TRIGGER";
		code  = LM5802_ID_USER2;
		break;
	case LM5802_IRQ_USER3_CMD:
		class = "VOICE_TRIGGER";
		code  = LM5802_ID_USER3;
		break;
	case LM5802_IRQ_USER4_CMD:
		class = "VOICE_TRIGGER";
		code  = LM5802_ID_USER4;
		break;
	case LM5802_IRQ_TRAINING_DONE:
		training_info = lm5802_get_training_info(lm5802);

		dev_info(lm5802->dev, "Training info = 0x%.x\n", training_info);

		if (training_info < 0) {
			dev_err(lm5802->dev, "Training info err\n");
			return;
		}

		class = "TRAINING_INFO";
		code = training_info;
		break;
	default:
		dev_err(lm5802->dev, "Do not send uevent for interrupt: 0x%x\n",
			irq_status);
		return;
	}

	snprintf(arg1, sizeof(arg1), "EVENT_CLASS=%s", class);
	snprintf(arg2, sizeof(arg2), "EVENT_CODE=%d", code);
	args[0] = arg1;
	args[1] = arg2;
	args[2] = NULL;

	dev_info(lm5802->dev, "%s %s\n", args[0], args[1]);

	kobject_uevent_env(&lm5802->dev->kobj, KOBJ_CHANGE, args);
}

static bool lm5802_is_boot_status(struct lm5802 *lm5802, u16 *status)
{
	int ret;

	ret = lm5802_reg_read(lm5802, LM5802_REG_BOOT_STATUS, (u8 *)status, 2);
	if (ret)
		return false;

	dev_info(lm5802->dev, "%s: boot status = 0x%x\n", __func__, *status);

	return *status & LM5802_BOOTCODE_MASK;
}

static bool lm5802_is_user_indep_voice_detected(struct lm5802 *lm5802)
{
	int ret;
	u16 val = 0;

	ret = lm5802_reg_read(lm5802, LM5802_REG_INTERRUPT_STATUS,
			      (u8 *)&val, 2);
	if (ret)
		return false;

	if (val & LM5802_IRQ_USER_INDEP_CMD) {
		lm5802->irq_status = val;
		return true;
	}

	return false;
}

static void lm5802_interrupt1_handler(struct lm5802 *lm5802)
{
	u16 status = 0;

	dev_info(lm5802->dev, "%s\n", __func__);

	if (lm5802_is_boot_status(lm5802, &status))
		lm5802_boot_handler(lm5802, status);
	else if (lm5802_is_user_indep_voice_detected(lm5802))
		lm5802_report_event(lm5802);
}

static bool lm5802_is_appcode_mode(struct lm5802 *lm5802)
{
	int ret;
	u16 val = 0;

	ret = lm5802_reg_read(lm5802, LM5802_REG_APP_STATUS, (u8 *)&val, 2);
	if (ret)
		return false;

	return val & LM5802_APPCODE_MASK;
}

static void lm5802_get_userdata_size(struct lm5802 *lm5802)
{
	lm5802_reg_read(lm5802, LM5802_REG_ENROLLMENT_SIZE,
			(u8 *)&lm5802->size_userdata, 2);
}

static void lm5802_interrupt2_handler(struct lm5802 *lm5802)
{
	int ret;
	u16 val = 0;

	dev_info(lm5802->dev, "%s\n", __func__);

	if (!lm5802_is_appcode_mode(lm5802)) {
		dev_err(lm5802->dev, "appcode is not ready\n");
		return;
	}

	ret = lm5802_reg_read(lm5802, LM5802_REG_INTERRUPT_STATUS,
			      (u8 *)&val, 2);
	if (ret)
		return;

	dev_info(lm5802->dev, "interrupt status = 0x%x\n", val);

	lm5802->irq_status = val;

	/*
	 * Get enrollmet size when appcode is ready.
	 * This size data is used for reading or writing userdata.
	 */
	if (lm5802->irq_status == LM5802_IRQ_APPCODE_READY)
		lm5802_get_userdata_size(lm5802);

	lm5802_report_event(lm5802);
}

static void lm5802_interrupt_work(struct work_struct *work)
{
	struct lm5802 *lm5802 = container_of(to_delayed_work(work),
					     struct lm5802, irq_work);

	if (lm5802->which_irq == lm5802->irq1)
		lm5802_interrupt1_handler(lm5802);
	else if (lm5802->which_irq == lm5802->irq2)
		lm5802_interrupt2_handler(lm5802);
}

static irqreturn_t lm5802_interrupt_handler(int irq, void *data)
{
	struct lm5802 *lm5802 = data;

	/* Delay work guarantees the I2C read operation after CPU is up */
	lm5802->which_irq = irq;
	schedule_delayed_work(&lm5802->irq_work, LM5802_DELAY_JIFFIES);

	return IRQ_HANDLED;
}

static int lm5802_interrupt_pin_config(struct lm5802 *lm5802)
{
	struct lm5802_platform_data *pdata = lm5802->pdata;
	int ret;

	ret = gpio_request_one(pdata->irq1_gpio, GPIOF_IN, "lm5802_irq1");
	if (ret)
		return ret;

	return gpio_request_one(pdata->irq2_gpio, GPIOF_IN, "lm5802_irq2");
}

static int lm5802_interrupt_config(struct lm5802 *lm5802)
{
	struct lm5802_platform_data *pdata = lm5802->pdata;
	unsigned long flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	int ret;

	/* System interrupt */
	lm5802->irq1 = gpio_to_irq(pdata->irq1_gpio);
	ret = request_threaded_irq(lm5802->irq1, NULL, lm5802_interrupt_handler,
				   flags, "lm5802-irq1", lm5802);
	if (ret) {
		dev_err(lm5802->dev, "IRQ1 config err: %d\n", ret);
		return ret;
	}

	/* User interrupt */
	lm5802->irq2 = gpio_to_irq(pdata->irq2_gpio);
	ret = request_threaded_irq(lm5802->irq2, NULL, lm5802_interrupt_handler,
				   flags, "lm5802-irq2", lm5802);
	if (ret) {
		dev_err(lm5802->dev, "IRQ2 config err: %d\n", ret);
		return ret;
	}

	/* Both interrupt pins are used for system wakeup */
	ret = enable_irq_wake(lm5802->irq1);
	if (ret)
		dev_warn(lm5802->dev, "Can not use IRQ1 as wake up source\n");

	ret = enable_irq_wake(lm5802->irq2);
	if (ret)
		dev_warn(lm5802->dev, "Can not use IRQ2 as wake up source\n");

	return 0;
}

int lm5802_load_by_user(struct lm5802 *lm5802)
{
	int ret;
	u16 status = 0;

	/*
	 * Read boot status and request to download a firmware.
	 * Loading process is the interrupt-driven, so interrupt pins should be
	 * configured.
	 */

	ret = lm5802_reg_read(lm5802, LM5802_REG_BOOT_STATUS, (u8 *)&status, 2);
	if (ret)
		return ret;

	dev_info(lm5802->dev, "Boot status: 0x%x\n", status);

	if (lm5802_is_ready_to_download_firmware(status)) {
		ret = lm5802_request_firmware(lm5802);
		if (ret) {
			dev_err(lm5802->dev, "Request firmware err: %d\n", ret);
			return ret;
		}
	} else {
		dev_info(lm5802->dev, "not ready to request firmware\n");
	}

	return lm5802_interrupt_config(lm5802);
}
EXPORT_SYMBOL_GPL(lm5802_load_by_user);

u8 lm5802_compute_checksum(const u8 *val, int len)
{
	int i;
	long checksum = 0;

	for (i = 0; i < len; i++)
		checksum += val[i];

	return checksum &= 0xFF;
}
EXPORT_SYMBOL_GPL(lm5802_compute_checksum);

static int lm5802_parse_dt(struct device *dev, struct lm5802 *lm5802)
{
	struct device_node *node = dev->of_node;
	struct lm5802_platform_data *pdata;
	int ret;

	if (!node) {
		dev_err(dev, "Invalid device node\n");
		return -EINVAL;
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = of_property_read_string(node, "firmware-name", &pdata->fw_name);
	if (ret) {
		dev_err(dev, "Invalid firmware name: %s\n", pdata->fw_name);
		return ret;
	}

	pdata->en_gpio = of_get_named_gpio(node, "enable-gpio", 0);
	pdata->irq1_gpio = of_get_named_gpio(node, "irq1-gpio", 0);
	pdata->irq2_gpio = of_get_named_gpio(node, "irq2-gpio", 0);

	of_property_read_u32(node, "micbias-mv", &pdata->micbias);
	of_property_read_u32(node, "blackout-ms", &pdata->blackout_time);

	if (of_property_read_bool(node, "transfer-chunk-enabled"))
		pdata->is_chunk_xfer = true;

	if (pdata->irq1_gpio < 0 || pdata->irq2_gpio < 0) {
		dev_err(dev, "Invalid IRQ gpio\n");
		return -EINVAL;
	}

	lm5802->pdata = pdata;
	return 0;
}

static int lm5802_enable_hw(struct lm5802 *lm5802, int gpio)
{
	int ret = 0;

	if (gpio > 0)
		ret = gpio_request_one(gpio, GPIOF_INIT_HIGH, "lm5802_enable");
    usleep(2000);

	/* Reset to enter the boot mode */
	lm5802_reg_write(lm5802, LM5802_REG_RESET, NULL, 0);

	return ret;
}

/*
 * In probe, enable hardware and add device attributes.
 * Other initialization is done later by accessing 'load' attributes from
 * the user-space.
 */
static int lm5802_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct device *dev = &cl->dev;
	struct lm5802_platform_data *pdata = dev_get_platdata(dev);
	struct lm5802 *lm5802;
	int ret;

	lm5802 = devm_kzalloc(dev, sizeof(*lm5802), GFP_KERNEL);
	if (!lm5802)
		return -ENOMEM;

	lm5802->pdata = pdata;
	if (!lm5802->pdata) {
		if (IS_ENABLED(CONFIG_OF))
			ret = lm5802_parse_dt(dev, lm5802);
		else
			ret = -ENODEV;

		if (ret) {
			dev_err(dev, "Invalid platform data: %d", ret);
			return ret;
		}
	}

	if (!lm5802->pdata->fw_name) {
		dev_err(dev, "No firmware file exists\n");
		return -EINVAL;
	}

	lm5802->dev = dev;
	lm5802->cl = cl;

	ret = lm5802_enable_hw(lm5802, lm5802->pdata->en_gpio);
	if (ret) {
		dev_err(dev, "HW enable err: %d\n", ret);
		return ret;
	}

	ret = lm5802_add_sysfs(lm5802);
	if (ret) {
		dev_err(dev, "Can not add sysfs: %d\n", ret);
		goto err;
	}

	i2c_set_clientdata(cl, lm5802);
	dev_set_drvdata(dev, lm5802);

	ret = lm5802_interrupt_pin_config(lm5802);
	if (ret) {
		dev_err(lm5802->dev, "Interrupt pin config err: %d\n", ret);
		goto err;
	}

	INIT_DELAYED_WORK(&lm5802->irq_work, lm5802_interrupt_work);

	return device_init_wakeup(dev, 1);

err:
	lm5802_disable_hw(lm5802);
	return ret;
}

static int lm5802_remove(struct i2c_client *cl)
{
	struct lm5802 *lm5802 = i2c_get_clientdata(cl);

	device_wakeup_disable(lm5802->dev);
	lm5802_remove_sysfs(lm5802);

	free_irq(lm5802->irq1, lm5802);
	free_irq(lm5802->irq2, lm5802);

	cancel_delayed_work_sync(&lm5802->irq_work);

	lm5802_disable_hw(lm5802);

	return 0;
}

static const struct i2c_device_id lm5802_ids[] = {
	{"lm5802", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm5802_ids);

#ifdef CONFIG_OF
static const struct of_device_id lm5802_of_match[] = {
	{ .compatible = "ti,lm5802", },
	{ }
};
MODULE_DEVICE_TABLE(of, lm5802_of_match);
#endif

static struct i2c_driver lm5802_driver = {
	.probe    = lm5802_probe,
	.remove   = lm5802_remove,
	.driver   = {
		.name = "lm5802",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(lm5802_of_match),
	},
	.id_table = lm5802_ids,
};

static int __init lm5802_init(void)
{
	return i2c_add_driver(&lm5802_driver);
}
module_init(lm5802_init);

static void __exit lm5802_exit(void)
{
	i2c_del_driver(&lm5802_driver);
}
module_exit(lm5802_exit);

MODULE_DESCRIPTION("LM5802 Core Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL v2");
