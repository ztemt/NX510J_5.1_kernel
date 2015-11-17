/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#ifndef LINUX_SPI_FPC1020_INPUT_H
#define LINUX_SPI_FPC1020_INPUT_H

#define FPC1020_KEY_FINGER_PRESENT		KEY_WAKEUP      /* 143*/
#define FINGER_UP_DEBOUNCE_TIME_MS		50
#define FINGER_DOWN_DEBOUNCE_TIME_MS		80
#define FINGER_WAKEUP_KEY_DOWN_TIME_MS		50
#define PFC1020_MAX_ZONE_COUNT			12
#define PFC1020_MAX_LINE_COUNT			4
#define PFC1020_MAX_LINE_COUNT_MASK		0x0fU
#define INPUT_FINGER_MASK_LINE0			0x0111U
#define INPUT_FINGER_MASK_LINE1			0x0222U
#define INPUT_FINGER_MASK_LINE2			0x0444U
#define INPUT_FINGER_MASK_LINE3			0x0888U

typedef enum {
	INPUT_FINGER_PRESS_LINE0 = 1 << 0,
	INPUT_FINGER_PRESS_LINE1 = 1 << 1,
	INPUT_FINGER_PRESS_LINE2 = 1 << 2,
	INPUT_FINGER_PRESS_LINE3 = 1 << 3,
} input_finger_press_line_t;

typedef enum {
	INPUT_FINGER_CENTER_ERROR = -1,
	INPUT_FINGER_CENTER0,
	INPUT_FINGER_CENTER1,
	INPUT_FINGER_CENTER2,
	INPUT_FINGER_CENTER3,
	INPUT_FINGER_CENTER4,
	INPUT_FINGER_CENTER5,
	INPUT_FINGER_CENTER6,
} input_finger_center_t;

typedef enum {
	INPUT_FINGER_GESTURE_NONE = 0,
	INPUT_FINGER_GESTURE_MOVING_UP,
	INPUT_FINGER_GESTURE_MOVING_DOWN,
	INPUT_FINGER_GESTURE_NO_MOVING,
	INPUT_FINGER_GESTURE_COUNT,
} input_finger_gesture_t;

typedef enum {
	INPUT_FINGER_GESTURE_ACC_NONE = 0,
	INPUT_FINGER_GESTURE_ACC_LOW,
	INPUT_FINGER_GESTURE_ACC_HIGH,
} input_finger_gesture_acc_t;

extern int fpc1020_input_init(fpc1020_data_t *fpc1020);

extern void fpc1020_input_destroy(fpc1020_data_t *fpc1020);

extern int fpc1020_input_enable(fpc1020_data_t *fpc1020, bool enabled);

extern int fpc1020_input_task(fpc1020_data_t *fpc1020);

extern void fpc1020_report_key(fpc1020_data_t *fpc1020);

extern int fpc1020_setup_key_event(fpc1020_data_t *fpc1020);

enum hrtimer_restart fpc1020_fingerup_hrtimer_func(struct hrtimer *timer);

#ifdef FPC1020_NAV_HEAP_BUF
int fpc1020_alloc_img_buf(fpc1020_data_t *fpc1020);
#endif

#endif /* LINUX_SPI_FPC1020_NAV_H */

