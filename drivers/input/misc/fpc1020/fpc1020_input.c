/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

//#define DEBUG

#include <linux/input.h>
#include <linux/delay.h>
#include <linux/time.h>

#ifndef CONFIG_OF
#include <linux/spi/fpc1020_common.h>
#include <linux/spi/fpc1020_input.h>
#else
#include "fpc1020_common.h"
#include "fpc1020_input.h"
#endif

/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */
static int fpc1020_write_lpm_setup(fpc1020_data_t *fpc1020);

static int fpc1020_wait_finger_present_lpm(fpc1020_data_t *fpc1020);


/* -------------------------------------------------------------------- */
/* driver constants							*/
/* -------------------------------------------------------------------- */
#define FPC1020_INPUT_POLL_TIME_MS	1000u

/* -------------------------------------------------------------------- */
/* function definitions							*/
/* -------------------------------------------------------------------- */
int fpc1020_input_init(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->input_dev = input_allocate_device();

	if (!fpc1020->input_dev) {
		dev_err(&fpc1020->spi->dev, "Input_allocate_device failed.\n");
		error  = -ENOMEM;
	}

	if (!error) {
		fpc1020->input_dev->name = FPC1020_DEV_NAME;

		set_bit(EV_KEY,		fpc1020->input_dev->evbit);

		input_set_capability(fpc1020->input_dev, EV_KEY, FPC1020_KEY_FINGER_PRESENT);
		input_set_capability(fpc1020->input_dev, EV_KEY, KEY_VOLUMEUP);
		input_set_capability(fpc1020->input_dev, EV_KEY, KEY_VOLUMEDOWN);
		if (fpc1020->single_click_key_code)
			input_set_capability(fpc1020->input_dev, EV_KEY, fpc1020->single_click_key_code);
		if (fpc1020->double_click_key_code)
			input_set_capability(fpc1020->input_dev, EV_KEY, fpc1020->double_click_key_code);
		if (fpc1020->long_key_code)
			input_set_capability(fpc1020->input_dev, EV_KEY, fpc1020->long_key_code);

		error = input_register_device(fpc1020->input_dev);
	}

	if (error) {
		dev_err(&fpc1020->spi->dev, "Input_register_device failed.\n");
		input_free_device(fpc1020->input_dev);
		fpc1020->input_dev = NULL;
	}

	return error;
}


/* -------------------------------------------------------------------- */
void fpc1020_input_destroy(fpc1020_data_t *fpc1020)
{
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if (fpc1020->input_dev != NULL)
		input_free_device(fpc1020->input_dev);
}


/* -------------------------------------------------------------------- */
int fpc1020_input_enable(fpc1020_data_t *fpc1020, bool enabled)
{
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->input.enabled = enabled;

	return 0;
}


/* -------------------------------------------------------------------- */
int fpc1020_input_task(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	while (!error) {
		error = fpc1020_wait_finger_present_lpm(fpc1020);
	}
	return error;
}


/* -------------------------------------------------------------------- */
static int fpc1020_write_lpm_setup(fpc1020_data_t *fpc1020)
{
	const int mux = FPC1020_MAX_ADC_SETTINGS - 1;
	int error = 0;
	u16 temp_u16;
	fpc1020_reg_access_t reg;

	dev_dbg(&fpc1020->spi->dev, "%s %d\n", __func__, mux);

	error = fpc1020_write_sensor_setup(fpc1020);
	if(error)
		goto out;

	temp_u16 = fpc1020->setup.adc_shift[mux];
	temp_u16 <<= 8;
	temp_u16 |= fpc1020->setup.adc_gain[mux];

	FPC1020_MK_REG_WRITE(reg, FPC102X_REG_ADC_SHIFT_GAIN, &temp_u16);
	error = fpc1020_reg_access(fpc1020, &reg);
	if (error)
		goto out;

	temp_u16 = fpc1020->setup.pxl_ctrl[mux];
	FPC1020_MK_REG_WRITE(reg, FPC102X_REG_PXL_CTRL, &temp_u16);
	error = fpc1020_reg_access(fpc1020, &reg);
	if (error)
		goto out;

out:
	return error;
}

int fpc1020_setup_key_event(fpc1020_data_t *fpc1020)
{
	fpc1020->key_event = kzalloc(sizeof(fpc1020_key_event_t), GFP_KERNEL);
	if (!fpc1020->key_event)
		return -ENOMEM;

	fpc1020->key_event->key = 0;
	fpc1020->key_event->dir = 0;

	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1020_check_finger_present(int zones)
{
	u16 mask = FPC1020_FINGER_DETECT_ZONE_MASK;
	u8 count = 0;

	if (zones < 0)
		return zones;
	else {
		zones &= mask;
		while (zones && mask) {
			count += (zones & 1) ? 1 : 0;
			zones >>= 1;
			mask >>= 1;
		}
		return (int)count;
	}
}

static int maybe_is_long_key_down(int zone_count)
{
	return (zone_count == 12);
}

static int finger_is_up(int zone_count)
{
	return (zone_count < 2);
}

void fpc1020_report_key(fpc1020_data_t *fpc1020)
{
	fpc1020_key_event_t *key_event = fpc1020->key_event;
	input_report_key(fpc1020->input_dev, key_event->key, key_event->dir);
	input_sync(fpc1020->input_dev);
	pr_info("%s keyvalue %d %s\n", __func__, key_event->key,
			key_event->dir ? "down" : "up");
}

static int fpc1020_wake_system_up(fpc1020_data_t *fpc1020, int zone_raw)
{
	const int zmask_5 = 1 << 5;
	const int zmask_6 = 1 << 6;
	const int zmask_ext = FPC1020_FINGER_DETECT_ZONE_MASK;
	bool wakeup_center = false;
	bool wakeup_ext    = false;
	fpc1020_key_event_t *key_event = fpc1020->key_event;
	int zone_count = fpc1020_check_finger_present(zone_raw);

	wakeup_center = (zone_raw & zmask_5) ||
	                (zone_raw & zmask_6);

	/* Todo: refined extended processing ? */
	wakeup_ext = ((zone_raw & zmask_ext) == zmask_ext);

	if (zone_count > 5 && (wakeup_center && wakeup_ext)) {
		key_event->dir = 1;
		fpc1020_report_key(fpc1020);
		key_event->dir = 0;
		fpc1020_report_key(fpc1020);
		dev_info(&fpc1020->spi->dev, "%s Wake up !\n", __func__);
		return 1;
	}

	return 0;
}

static int input_finger_get_press_lines(int zone_raw)
{
	int press_lines = 0;
	int zone = zone_raw & FPC1020_FINGER_DETECT_ZONE_MASK;

	if (zone & INPUT_FINGER_MASK_LINE0)
		press_lines |= INPUT_FINGER_PRESS_LINE0;

	if (zone & INPUT_FINGER_MASK_LINE1)
		press_lines |= INPUT_FINGER_PRESS_LINE1;

	if (zone & INPUT_FINGER_MASK_LINE2)
		press_lines |= INPUT_FINGER_PRESS_LINE2;

	if (zone & INPUT_FINGER_MASK_LINE3)
		press_lines |= INPUT_FINGER_PRESS_LINE3;

	return press_lines;
}

static int lines_to_center[16] = {
	INPUT_FINGER_CENTER_ERROR, INPUT_FINGER_CENTER0,
	INPUT_FINGER_CENTER2, INPUT_FINGER_CENTER1,
	INPUT_FINGER_CENTER4, INPUT_FINGER_CENTER_ERROR,
	INPUT_FINGER_CENTER3, INPUT_FINGER_CENTER2,
	INPUT_FINGER_CENTER6, INPUT_FINGER_CENTER_ERROR,
	INPUT_FINGER_CENTER_ERROR, INPUT_FINGER_CENTER_ERROR,
	INPUT_FINGER_CENTER5, INPUT_FINGER_CENTER_ERROR,
	INPUT_FINGER_CENTER4, INPUT_FINGER_CENTER3,
};

static int input_finger_get_center_coordinate(int zone_raw)
{
	return lines_to_center[input_finger_get_press_lines(zone_raw)];
}

static int input_finger_get_gesture(int zone_last, int zone_new, int *accuracy)
{
	int finger_gesture;
	int finger_center_last;
	int finger_center_new;
	int zone_count_last;
	int zone_count_new;
	*accuracy = INPUT_FINGER_GESTURE_ACC_NONE;

	if (zone_last < 0 || zone_new < 0)
		return INPUT_FINGER_GESTURE_NONE;

	finger_center_last = input_finger_get_center_coordinate(zone_last);
	finger_center_new = input_finger_get_center_coordinate(zone_new);

	if (finger_center_last < 0 ||
			finger_center_new < 0)
		return INPUT_FINGER_GESTURE_NONE;

	*accuracy = INPUT_FINGER_GESTURE_ACC_HIGH;
	if (finger_center_last < finger_center_new)
		finger_gesture = INPUT_FINGER_GESTURE_MOVING_DOWN;
	else if (finger_center_last > finger_center_new)
		finger_gesture = INPUT_FINGER_GESTURE_MOVING_UP;
	else {
		finger_gesture = INPUT_FINGER_GESTURE_NO_MOVING;
		zone_count_last = fpc1020_check_finger_present(zone_last);
		zone_count_new = fpc1020_check_finger_present(zone_new);
	}

	if (finger_gesture == INPUT_FINGER_GESTURE_NO_MOVING &&
			zone_count_new != zone_count_last) {
		int line_diff_center;
		line_diff_center = input_finger_get_center_coordinate(zone_new ^ zone_last);
		if (line_diff_center < 0) {
			*accuracy = INPUT_FINGER_GESTURE_ACC_NONE;
			return finger_gesture;
		}

		*accuracy = INPUT_FINGER_GESTURE_ACC_LOW;
		if (zone_count_last > zone_count_new) {
			if (line_diff_center > finger_center_new)
				finger_gesture = INPUT_FINGER_GESTURE_MOVING_UP;
			else
				finger_gesture = INPUT_FINGER_GESTURE_MOVING_DOWN;
		} else {
			if (line_diff_center > finger_center_new)
				finger_gesture = INPUT_FINGER_GESTURE_MOVING_DOWN;
			else
				finger_gesture = INPUT_FINGER_GESTURE_MOVING_UP;
		}
	}

	return finger_gesture;
}

enum hrtimer_restart fpc1020_fingerup_hrtimer_func(struct hrtimer *timer)
{
	fpc1020_data_t *fpc1020 = container_of(timer,
			fpc1020_data_t, finger_up_timer);

	fpc1020_dynamic_debug("%s %d finger is up\n", __func__, __LINE__);
	fpc1020->finger_is_up = 1;
	if (fpc1020->key_event->key && fpc1020->key_event->dir) {
		getnstimeofday(&fpc1020->ts_finger_up);
		fpc1020->key_event->dir = 0;
		fpc1020_report_key(fpc1020);
	}

	return HRTIMER_NORESTART;
}

/* -------------------------------------------------------------------- */
static int fpc1020_wait_finger_present_lpm(fpc1020_data_t *fpc1020)
{
	int i;
	int error = 0;
	int zone_count = 0;
	int zone_raw_last = -1;
	int zone_raw_new = -1;
	int finger_gesture_last = -1;
	int finger_gesture_new = -1;
	int long_key_count = 0;
	int accuracy = 0;
	int gesture_count[INPUT_FINGER_GESTURE_COUNT] = {0};
	struct timespec ts_finger_down, ts_now, ts_diff;
	struct timespec ts_first_long_key, ts_long_key;
	int first_time_flag = 0;
	int time_ms;
	int acc_count;
	int long_key_code = fpc1020->long_key_code;
	int long_key_time = fpc1020->long_key_detect_time;
	fpc1020_key_event_t *key_event = fpc1020->key_event;

	key_event->key = 0;
	key_event->dir = 0;

	error = fpc1020_wake_up(fpc1020);

	if (!error)
		error = fpc1020_calc_finger_detect_threshold_min(fpc1020);

	if (error >= 0)
		error = fpc1020_set_finger_detect_threshold(fpc1020, error);

	if (error >= 0)
		error = fpc1020_write_lpm_setup(fpc1020);

#if 0
	if (!error) {
		error = fpc1020_sleep(fpc1020, false);

		if (error == -EAGAIN) {
			error = fpc1020_sleep(fpc1020, false);

			if (error == -EAGAIN)
				error = 0;
		}
	}
#endif

	while (!error) {
		fpc1020_dynamic_debug("\n%s %d\n", __func__, __LINE__);
		if (!error)
			error = fpc1020_wait_finger_present(fpc1020);

		if (!error)
			error = fpc1020_check_finger_present_raw(fpc1020);
		else
			goto out;

		if (error < 0)
			goto out;

		if (fpc1020->finger_is_up) {
			fpc1020_dynamic_debug("%s %d\n", __func__, __LINE__);
			fpc1020->finger_is_up = 0;
			first_time_flag = 0;
			zone_raw_new = -1;
			finger_gesture_new = -1;
			for (i = 0; i < INPUT_FINGER_GESTURE_COUNT; i++)
				gesture_count[i] = 0;
			long_key_count = 0;
		}

		hrtimer_start(&fpc1020->finger_up_timer, ktime_set(0, 50000000), HRTIMER_MODE_REL);

		zone_raw_new = (error >= 0) ? error : 0;
		fpc1020_dynamic_debug("%s %d zone_raw = 0x%x\n", __func__, __LINE__, zone_raw_new);
		error = (error >= 0) ? 0 : error;
		zone_count = fpc1020_check_finger_present(zone_raw_new);

		if (fpc1020->resume_flag) {
			fpc1020_dynamic_debug("%s %d\n", __func__, __LINE__);
			getnstimeofday(&ts_now);
			ts_diff = timespec_sub(ts_now, fpc1020->ts_resume);
			time_ms = ts_diff.tv_sec * MSEC_PER_SEC +
				(ts_diff.tv_nsec / NSEC_PER_MSEC);
			if (time_ms < 500) {
				key_event->key = FPC1020_KEY_FINGER_PRESENT;
				if (fpc1020_wake_system_up(fpc1020, zone_raw_new)) {
					error = 1;
					goto out;
				} else {
					goto next;
				}
			}
		}

		/* finger up debounce */
		if (key_event->key != 0 && key_event->dir == 0) {
			fpc1020_dynamic_debug("%s %d\n", __func__, __LINE__);
			getnstimeofday(&ts_now);
			ts_diff = timespec_sub(ts_now, fpc1020->ts_finger_up);

			time_ms = ts_diff.tv_sec * MSEC_PER_SEC +
				(ts_diff.tv_nsec / NSEC_PER_MSEC);
			if (time_ms < FINGER_UP_DEBOUNCE_TIME_MS)
				continue;
		}

		/* finger down debounce */
		if (first_time_flag == 0) {
			fpc1020_dynamic_debug("%s %d\n", __func__, __LINE__);
			/* A finger was detected,
			 * Update the timestamp for the last detected finger presence
			 */
			first_time_flag = 1;
			getnstimeofday(&ts_finger_down);
			continue;
		}
		fpc1020_dynamic_debug("%s %d\n", __func__, __LINE__);
		getnstimeofday(&ts_now);
		ts_diff = timespec_sub(ts_now, ts_finger_down);
		time_ms = ts_diff.tv_sec * MSEC_PER_SEC +
			(ts_diff.tv_nsec / NSEC_PER_MSEC);
		if (time_ms < FINGER_DOWN_DEBOUNCE_TIME_MS)
			continue;

		fpc1020_dynamic_debug("%s %d\n", __func__, __LINE__);
		if (fpc1020->input_status == INPUT_STATUS_WAIT_FOR_WAKEUP) {
			long_key_code = FPC1020_KEY_FINGER_PRESENT;
			long_key_time = FINGER_WAKEUP_KEY_DOWN_TIME_MS;
		}
		else if (fpc1020->input_status == INPUT_STATUS_WAIT_FOR_LONGKEY) {
			long_key_code = fpc1020->long_key_code;
			long_key_time = fpc1020->long_key_detect_time;
		}

		/* if key was down, then ... */
		if (zone_raw_last > 0 && zone_raw_new > 0 &&
				zone_raw_last != zone_raw_new &&
				key_event->dir == 1 &&
				abs(fpc1020_check_finger_present(zone_raw_new) -
					fpc1020_check_finger_present(zone_raw_last)) < 2) {
			fpc1020_dynamic_debug("%s %d\n", __func__, __LINE__);
			continue;
		}

		fpc1020_dynamic_debug("%s %d\n", __func__, __LINE__);
		if (maybe_is_long_key_down(zone_count) &&
				gesture_count[INPUT_FINGER_GESTURE_MOVING_UP] == 0 &&
				gesture_count[INPUT_FINGER_GESTURE_MOVING_DOWN] == 0 &&
				gesture_count[INPUT_FINGER_GESTURE_NO_MOVING] == 0) {
			fpc1020_dynamic_debug("%s %d\n", __func__, __LINE__);
			long_key_count++;
			if (long_key_count == 1) {
				getnstimeofday(&ts_first_long_key);
				continue;
			}
			getnstimeofday(&ts_long_key);
			ts_diff = timespec_sub(ts_long_key, ts_first_long_key);
			time_ms = ts_diff.tv_sec * MSEC_PER_SEC +
				(ts_diff.tv_nsec / NSEC_PER_MSEC);
			if (time_ms > long_key_time) {
				long_key_count = 0;
				/* do not report if long key was down
				 * or volume key isn't up
				 */
				if (key_event->dir == 0) {
					key_event->key = long_key_code;
					key_event->dir = 1;
					fpc1020_report_key(fpc1020);
				}
			}
		} else {
			fpc1020_dynamic_debug("%s %d\n", __func__, __LINE__);
			if (finger_is_up(zone_count)) {
				fpc1020_dynamic_debug("%s %d finger is up\n", __func__, __LINE__);
				if (key_event->dir == 1) {
					getnstimeofday(&fpc1020->ts_finger_up);
					key_event->dir = 0;
					fpc1020_report_key(fpc1020);
				}
				first_time_flag = 0;
				zone_raw_new = -1;
				finger_gesture_new = -1;
				for (i = 0; i < INPUT_FINGER_GESTURE_COUNT; i++)
					gesture_count[i] = 0;
				long_key_count = 0;
				goto next;
			}

			long_key_count = 0;

			/* if long key was down
			 * then wait log key up
			 */
			if (key_event->key == long_key_code &&
					key_event->dir == 1)
				goto next;

			finger_gesture_new = input_finger_get_gesture(
					zone_raw_last, zone_raw_new, &accuracy);
			if (fpc1020->change_up_down_layout) {
				if (finger_gesture_new == INPUT_FINGER_GESTURE_MOVING_UP)
					finger_gesture_new = INPUT_FINGER_GESTURE_MOVING_DOWN;
				else if (finger_gesture_new == INPUT_FINGER_GESTURE_MOVING_DOWN)
					finger_gesture_new = INPUT_FINGER_GESTURE_MOVING_UP;
			}

			gesture_count[finger_gesture_new]++;
			for (i = 0; i < INPUT_FINGER_GESTURE_COUNT; i++)
				fpc1020_dynamic_debug("gesture_count[%d] = %d\n", i, gesture_count[i]);
			switch (finger_gesture_new) {
			case INPUT_FINGER_GESTURE_MOVING_UP:
				if (accuracy == INPUT_FINGER_GESTURE_ACC_HIGH)
					acc_count = 3;
				else if (accuracy == INPUT_FINGER_GESTURE_ACC_LOW)
					acc_count = 4;

				if (fpc1020->input_status == INPUT_STATUS_WAIT_FOR_WAKEUP)
					acc_count = 5;

				if (gesture_count[finger_gesture_new] < acc_count ||
						(key_event->key == KEY_VOLUMEUP &&
						gesture_count[INPUT_FINGER_GESTURE_NO_MOVING] > 35))
					goto next;

				if (key_event->key == KEY_VOLUMEDOWN &&
						key_event->dir == 1) {
					if (gesture_count[INPUT_FINGER_GESTURE_MOVING_UP] == 1) {
						gesture_count[INPUT_FINGER_GESTURE_NO_MOVING] = 0;
						continue;
					}

					if (gesture_count[INPUT_FINGER_GESTURE_NO_MOVING] < 50 &&
							gesture_count[INPUT_FINGER_GESTURE_MOVING_UP] < 6) {
						continue;
					} else {
						gesture_count[INPUT_FINGER_GESTURE_MOVING_DOWN] = 0;
						gesture_count[INPUT_FINGER_GESTURE_NO_MOVING] = 0;
						key_event->key = KEY_VOLUMEDOWN;
						key_event->dir = 0;
						fpc1020_report_key(fpc1020);

						key_event->key = KEY_VOLUMEUP;
						key_event->dir = 1;
						fpc1020_report_key(fpc1020);
					}
				}

				if (key_event->dir == 1)
					goto next;

				key_event->key = KEY_VOLUMEUP;
				key_event->dir = 1;
				fpc1020_report_key(fpc1020);
				gesture_count[INPUT_FINGER_GESTURE_NO_MOVING] = 0;
				break;

			case INPUT_FINGER_GESTURE_MOVING_DOWN:
				if (accuracy == INPUT_FINGER_GESTURE_ACC_HIGH)
					acc_count = 3;
				else if (accuracy == INPUT_FINGER_GESTURE_ACC_LOW)
					acc_count = 4;

				if (fpc1020->input_status == INPUT_STATUS_WAIT_FOR_WAKEUP)
					acc_count = 5;

				if (gesture_count[finger_gesture_new] < acc_count ||
						(key_event->key == KEY_VOLUMEDOWN &&
						gesture_count[INPUT_FINGER_GESTURE_NO_MOVING] > 35))
					goto next;

				if (key_event->key == KEY_VOLUMEUP &&
						key_event->dir == 1) {
					if (gesture_count[INPUT_FINGER_GESTURE_MOVING_DOWN] == 1) {
						gesture_count[INPUT_FINGER_GESTURE_NO_MOVING] = 0;
						continue;
					}

					if (gesture_count[INPUT_FINGER_GESTURE_NO_MOVING] < 50 &&
							gesture_count[INPUT_FINGER_GESTURE_MOVING_DOWN] < 6) {
						continue;
					}
					else {
						gesture_count[INPUT_FINGER_GESTURE_MOVING_UP] = 0;
						gesture_count[INPUT_FINGER_GESTURE_NO_MOVING] = 0;
						key_event->key = KEY_VOLUMEUP;
						key_event->dir = 0;
						fpc1020_report_key(fpc1020);

						key_event->key = KEY_VOLUMEDOWN;
						key_event->dir = 1;
						fpc1020_report_key(fpc1020);
					}
				}

				if (key_event->dir == 1)
					goto next;

				key_event->key = KEY_VOLUMEDOWN;
				key_event->dir = 1;
				fpc1020_report_key(fpc1020);
				gesture_count[INPUT_FINGER_GESTURE_NO_MOVING] = 0;
				break;

			case INPUT_FINGER_GESTURE_NO_MOVING:
				zone_raw_last = zone_raw_new;
				continue;

			case INPUT_FINGER_GESTURE_NONE:
				break;

			default:
				break;
			}

		}
next:
		finger_gesture_last = finger_gesture_new;
		zone_raw_last = zone_raw_new;
	}

out:
	if (error < 0)
		dev_dbg(&fpc1020->spi->dev,
			"%s %s %d!\n", __func__,
			(error == -EINTR) ? "TERMINATED" : "FAILED", error);
	hrtimer_cancel(&fpc1020->finger_up_timer);

	return error;
}


/* -------------------------------------------------------------------- */

