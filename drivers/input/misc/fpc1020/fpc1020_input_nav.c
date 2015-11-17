/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013, 2014, 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

//#define DEBUG
//#define	FPC1020_NAV_DBG
//#define	FPC1020_NAV_TOUCH_DBG
//#define FPC1020_NAV_DPAD_DBG
//#define DEBUG_TIME
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include "fpc1020_common.h"
#include "fpc1020_input.h"
#include "fpc1020_capture.h"
#include "fpc1020_input.h"
#include "fpc1020_navlib.h"
#include "fpc1020_common.h"

/* -------------------------------------------------------------------- */
/* data types								*/
/* -------------------------------------------------------------------- */

typedef enum {
	FPC_1020_NAVIGATION_UNQUALIFIED,
	FPC_1020_NAVIGATION_LONGPRESS,
	FPC_1020_NAVIGATION_MOVEMENT
} fpc1020_navigation_state_t;

/* Pointers into huge-buffer for image data */
#ifndef FPC1020_NAV_HEAP_BUF
static u8* p_prev_img = 0;
static u8* p_curr_img = 0;
#endif

/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */
static int fpc1020_write_nav_setup(fpc1020_data_t* fpc1020);

static int capture_nav_image(fpc1020_data_t* fpc1020);

/* -------------------------------------------------------------------- */
/* driver constants							*/
/* -------------------------------------------------------------------- */
#define FPC1020_KEY_ENTER           KEY_ENTER
#define FPC1020_KEY_BACK            KEY_BACK
#define FPC1020_KEY_FINGER_PRESENT	KEY_WAKEUP	/* 143*/

#define FPC1020_INPUT_POLL_INTERVAL	(20000u)
#define FPC1020_CLICK_INTERVAL		(11000u)
#define THRESHOLD_DURATION_TAP		1500
#define THRESHOLD_DURATION_DTAP		500

#define FLOAT_MAX 100

#define DEVICE_WIDTH 1080
#define DEVICE_HEIGHT 1920

enum
{
    FNGR_ST_NONE = 0,
    FNGR_ST_DETECTED,
    FNGR_ST_LOST,
    FNGR_ST_TAP,
    FNGR_ST_HOLD,
    FNGR_ST_MOVING,
    FNGR_ST_L_HOLD,
    FNGR_ST_DOUBLE_TAP,
};

enum
{
    FPC1020_INPUTMODE_TRACKPAD	= 0,	//trackpad(navi) event report
    FPC1020_INPUTMODE_MOUSE		= 1,	//mouse event report
    FPC1020_INPUTMODE_TOUCH		= 2,	//touch event report
    FPC1020_INPUTMODE_DPAD		= 3,	//dpad event report
};

/* -------------------------------------------------------------------- */
/* function definitions							*/
/* -------------------------------------------------------------------- */

int fpc1020_capture_nav_wait_finger_down(fpc1020_data_t* fpc1020)
{
    int error = 0;

    error = fpc1020_wait_finger_present(fpc1020);


    fpc1020_read_irq(fpc1020, true);

    return (error >= 0) ? 0 : error;
}
void init_enhanced_navi_setting(fpc1020_data_t* fpc1020)
{
    dev_info(&fpc1020->spi->dev, "%s\n", __func__);
    switch (fpc1020->nav.input_mode)
    {
        case FPC1020_INPUTMODE_TRACKPAD:
            fpc1020->nav.p_sensitivity_key = 160; // 25;
            fpc1020->nav.p_sensitivity_ptr = 180;
            fpc1020->nav.p_multiplier_x = 120; // 75;
            fpc1020->nav.p_multiplier_y = 120; // 95
            fpc1020->nav.multiplier_key_accel = 1;
            fpc1020->nav.multiplier_ptr_accel = 2;
            fpc1020->nav.threshold_key_accel = 70;
            fpc1020->nav.threshold_ptr_accel = 10;
            fpc1020->nav.threshold_ptr_start = 5;
            fpc1020->nav.duration_ptr_clear = 100;
            fpc1020->nav.nav_finger_up_threshold = 3;
            break;

        case FPC1020_INPUTMODE_DPAD:
            fpc1020->nav.p_sensitivity_key = 200;
            fpc1020->nav.p_sensitivity_ptr = 180;
            fpc1020->nav.p_multiplier_x = 400;
            fpc1020->nav.p_multiplier_y = 600;
            fpc1020->nav.multiplier_key_accel = 1;
            fpc1020->nav.multiplier_ptr_accel = 2;
            fpc1020->nav.threshold_key_accel = 40;
            fpc1020->nav.threshold_ptr_accel = 20;
            fpc1020->nav.threshold_ptr_start = 5;
            fpc1020->nav.duration_ptr_clear = 100;
            fpc1020->nav.nav_finger_up_threshold = 1;
            fpc1020->nav.threshold_key_start = 8;
            fpc1020->nav.threshold_key_offset = 8;
            fpc1020->nav.duration_key_clear = 2000;
            fpc1020->nav.duration_ptr_clear = 100;
            fpc1020->nav.threshold_key_dispatch = 4;
            break;
        case FPC1020_INPUTMODE_TOUCH:
            fpc1020->nav.p_sensitivity_key = 160;//180;
            fpc1020->nav.p_sensitivity_ptr = 180;
            fpc1020->nav.p_multiplier_x = 1000;//110;
            fpc1020->nav.p_multiplier_y = 2000 ; // 110;
            fpc1020->nav.multiplier_key_accel = 1;// 1;
            fpc1020->nav.multiplier_ptr_accel = 2;
            fpc1020->nav.threshold_key_accel = 40;
            fpc1020->nav.threshold_ptr_accel = 20;
            fpc1020->nav.threshold_ptr_start = 5;
            fpc1020->nav.duration_ptr_clear = 100;
            fpc1020->nav.nav_finger_up_threshold = 1;
            break;
        default:
            break;
    }
}
/**********************************************************/
int fpc1020_setup_key_event(fpc1020_data_t *fpc1020)
{
	fpc1020->key_event = kzalloc(sizeof(fpc1020_key_event_t), GFP_KERNEL);
	if (!fpc1020->key_event)
		return -ENOMEM;

	fpc1020->key_event->key = 0;
	fpc1020->key_event->dir = 0;

	return 0;
}

void fpc1020_report_key(fpc1020_data_t *fpc1020)
{
	fpc1020_key_event_t *key_event = fpc1020->key_event;
	input_report_key(fpc1020->touch_pad_dev, key_event->key, key_event->dir);
	input_sync(fpc1020->touch_pad_dev);
	pr_info("%s keyvalue %d %s\n", __func__, key_event->key,
			key_event->dir ? "down" : "up");
}

/*--------------------------------------------------------*/
static void dispatch_dpad_event(fpc1020_data_t *fpc1020, int x, int y, int finger_status)
{
    int abs_x, abs_y;
    int sign_x, sign_y;
    int key_code;
    int reset = 0;
    static int active = 0;
    static int issued_key = 0;
    static unsigned long prev_time = 0;
    unsigned long current_time = jiffies * 1000 / HZ;
	fpc1020_key_event_t *key_event = fpc1020->key_event;

    switch (finger_status)
    {
        case FNGR_ST_DETECTED:
            fpc1020_dynamic_debug("[FPC/DPAD] report finger present\n");
	    break;
        case FNGR_ST_LOST:
	    if (fpc1020->nav.key_code) {
		    key_event->key = fpc1020->nav.key_code;
		    if (key_event->dir == 1) {
			    key_event->dir = 0;
			    fpc1020_report_key(fpc1020);
		    }
		    fpc1020->nav.key_code = 0;
	    }
            fpc1020->nav.sum_x = 0;
            fpc1020->nav.sum_y = 0;
            fpc1020->nav.move_up_cnt = 0;
            fpc1020->nav.move_down_cnt = 0;
            fpc1020->nav.no_move_cnt = 0;
            break;

        case FNGR_ST_TAP:
	    if (fpc1020->input_status != INPUT_STATUS_WAIT_FOR_WAKEUP &&
			    fpc1020->single_click_key_code)  {
		    key_event->key = fpc1020->single_click_key_code;
		    key_event->dir = 1;
		    fpc1020_report_key(fpc1020);
		    key_event->dir = 0;
		    fpc1020_report_key(fpc1020);
		    fpc1020->nav.key_code = key_event->key;
		    fpc1020_dynamic_debug("[FPC/DPAD] report single click\n");
	    }
            break;

        case FNGR_ST_DOUBLE_TAP:
	    if (fpc1020->double_click_key_code) {
		    key_event->key = fpc1020->double_click_key_code;
		    key_event->dir = 1;
		    fpc1020_report_key(fpc1020);
		    key_event->dir = 0;
		    fpc1020_report_key(fpc1020);
		    fpc1020->nav.key_code = key_event->key;
		    fpc1020_dynamic_debug("[FPC/DPAD] report double click\n");
	    }
            break;

        case FNGR_ST_HOLD:
            fpc1020->nav.throw_event = 1;
            fpc1020->nav.tap_status = -1;
	    if (fpc1020->long_key_code) {
		    key_event->key = fpc1020->long_key_code;
		    key_event->dir = 1;
		    fpc1020_report_key(fpc1020);
		    fpc1020->nav.key_code = key_event->key;
		    fpc1020_dynamic_debug("[FPC/DPAD] report long press\n");
	    }
            break;

        case FNGR_ST_MOVING:
            fpc1020_dynamic_debug("[FPC/DPAD] %s: raw axis x=%d, y=%d\n", __func__, x, y);
            // Handle Acceleration
	    if (x!=0 && y !=0) y = 0; /*yongqiang*/
            sign_x = x > 0 ? 1 : -1;
            sign_y = y > 0 ? 1 : -1;
            abs_x = x * sign_x;
            abs_y = y * sign_y;
            if (prev_time != 0 && (current_time - prev_time > fpc1020->nav.duration_key_clear))
            {
                reset = 1;
            }
            prev_time = current_time;
            //spin_lock_irqsave(&easy_wake_guesure_lock, irq_flags);
            abs_x = x > 0 ? x : -x;
            abs_y = y > 0 ? y : -y;
            if (abs_x > fpc1020->nav.threshold_key_accel)
                x = (fpc1020->nav.threshold_key_accel
                     + (abs_x - fpc1020->nav.threshold_key_accel) * fpc1020->nav.multiplier_key_accel) * sign_x;
            if (abs_y > fpc1020->nav.threshold_key_accel)
                y = (fpc1020->nav.threshold_key_accel
                     + (abs_y - fpc1020->nav.threshold_key_accel) * fpc1020->nav.multiplier_key_accel) * sign_y;
            // Correct axis factor
            x = x * fpc1020->nav.p_multiplier_x / FLOAT_MAX;
            y = y * fpc1020->nav.p_multiplier_y / FLOAT_MAX;
            // Adjust Sensitivity
            fpc1020_dynamic_debug("[FPC/DPAD] %s:multiplied axis x=%d, y=%d\n", __func__, x, y);
            fpc1020->nav.sum_x += x;
            fpc1020->nav.sum_y += y;
#if 0
            if (abs_x > abs_y)
            {
                //fpc1020->nav.sum_x += x;
                if (issued_key)
                {
                    if (x < 0 && issued_key != KEY_LEFT)
                    {
                        reset = 1;
                    }
                    if ( x > 0 && issued_key != KEY_RIGHT )
                    {
                        reset = 1;
                    }
                }
            }
            else
            {
                if (issued_key)
                {
                    if (y < 0 && issued_key != KEY_VOLUMEUP)
                    {
                        reset = 1;
                    }
                    if (y > 0 && issued_key != KEY_VOLUMEDOWN)
                    {
                        reset = 1;
                    }
                }
            }
            if (reset)
            {
                active = 0;
                issued_key = 0;
            }
#endif

            fpc1020_dynamic_debug("[FPC/DPAD] Navigation:active=%d, fpc1020->nav.sum_x:%d, fpc1020->nav.sum_y:%d,\n nav.threshold_key_start:%d, nav.threshold_key_offset=%d\n",
                    fpc1020->nav.throw_event, fpc1020->nav.sum_x, fpc1020->nav.sum_y, fpc1020->nav.threshold_key_start, fpc1020->nav.threshold_key_offset);
            //if(fpc1020->nav.throw_event == 0)
            {
                if (abs_x > abs_y && fpc1020->nav.sum_x > fpc1020->nav.threshold_key_start
                    && (fpc1020->nav.sum_y < fpc1020->nav.threshold_key_offset
                        && fpc1020->nav.sum_y > -fpc1020->nav.threshold_key_offset))
                {
                    //key_code = KEY_RIGHT;
                    key_code = 0;
                    fpc1020->nav.sum_y = 0;
                }
                else if (abs_x > abs_y && fpc1020->nav.sum_x < -fpc1020->nav.threshold_key_start
                         && (fpc1020->nav.sum_y < fpc1020->nav.threshold_key_offset
                             && fpc1020->nav.sum_y > -fpc1020->nav.threshold_key_offset))
                {
                    //key_code = KEY_LEFT;
                    key_code = 0;
                    fpc1020->nav.sum_y = 0;
                }
                else if (abs_x < abs_y && fpc1020->nav.sum_y < -fpc1020->nav.threshold_key_start)
                {
                    fpc1020->nav.move_up_cnt++;
                    key_code = KEY_VOLUMEUP;
                    fpc1020->nav.sum_x = 0;
                }
                else if (abs_x < abs_y && fpc1020->nav.sum_y > fpc1020->nav.threshold_key_start)
                {
                    fpc1020->nav.move_down_cnt++;
                    key_code = KEY_VOLUMEDOWN;
                    fpc1020->nav.sum_x = 0;
                }
                else
                {
                    key_code = 0;
		    if (fpc1020->input_status == INPUT_STATUS_WAIT_FOR_WAKEUP) {
                    	fpc1020->nav.no_move_cnt++;
			if (fpc1020->nav.no_move_cnt > 2) {
				fpc1020->nav.no_move_cnt = 0;
				key_event->key = FPC1020_KEY_FINGER_PRESENT;
				fpc1020->nav.key_code = FPC1020_KEY_FINGER_PRESENT;
				if (key_event->dir == 0) {
					key_event->dir = 1;
					fpc1020_report_key(fpc1020);
				}
			}
		    }
                }
                if (key_code)
                {
                    active = 1;
		    //if (fpc1020->nav.throw_event == 0)
                    { 
			if (key_code != fpc1020->nav.key_code)
				fpc1020->nav.throw_event = 0; /*reset the flag*/
			/*volume up*/
			if (fpc1020->nav.move_up_cnt > 2 && key_code == KEY_VOLUMEUP) {
				if (fpc1020->nav.move_down_cnt > 0) {
					key_event->key = KEY_VOLUMEDOWN;
					key_event->dir = 0;
					fpc1020_report_key(fpc1020);
					fpc1020->nav.move_down_cnt = 0;
				}
				if (fpc1020->nav.throw_event == 0) 
				{
					key_event->key = key_code;
					key_event->dir = 1;
                        		fpc1020_report_key(fpc1020);
					//fpc1020->nav.move_up_cnt = 0;
				}
				fpc1020->nav.throw_event = 1;
		        }
			/*volume down*/
			if (fpc1020->nav.move_down_cnt > 2 && key_code == KEY_VOLUMEDOWN) {
				if (fpc1020->nav.move_up_cnt > 0) {
					key_event->key = KEY_VOLUMEUP;
					key_event->dir = 0;
		                	fpc1020_report_key(fpc1020);
					fpc1020->nav.move_up_cnt = 0;
				}
				if (fpc1020->nav.throw_event == 0) {
					key_event->key = key_code;
					key_event->dir = 1;
	                		fpc1020_report_key(fpc1020);
				}
	   			fpc1020->nav.throw_event = 1;
	                 }

			 fpc1020->nav.key_code = key_code;

#ifdef FPC1020_NAV_DPAD_DBG_LVL
                        switch (key_code)
                        {
                            case KEY_VOLUMEUP:
                                pr_info("[FPC/DPAD] report movement  key up\n");
                                break;
                            case KEY_VOLUMEDOWN:
                                pr_info("[FPC/DPAD] report movement key down\n");
                                break;
                            case KEY_LEFT:
                                pr_info("[FPC/DPAD] report movement key left\n");
                                break;
                            case KEY_RIGHT:
                                pr_info("[FPC/DPAD] report movement key right\n");
                                break;
                            default:
                                break;
                        }
#endif
                    }

                    issued_key = key_code;
                }
                else
                {
                    return;
                }
            }
    }/*switch (finger_status)*/
}


static void dispatch_trackpad_event(fpc1020_data_t* fpc1020, int x, int y, int finger_status)
{
    int abs_x, abs_y;
    int sign_x, sign_y;

    if (finger_status == FNGR_ST_TAP)
    {
        input_report_key(fpc1020->input_dev, KEY_ENTER, 1);
        input_sync(fpc1020->input_dev);
        input_report_key(fpc1020->input_dev, KEY_ENTER, 0);
        input_sync(fpc1020->input_dev);
        return;
    }

    sign_x = x > 0 ? 1 : -1;
    sign_y = y > 0 ? 1 : -1;
    abs_x = x * sign_x;
    abs_y = y * sign_y;

    abs_x = x > 0 ? x : -x;
    abs_y = y > 0 ? y : -y;

    if (abs_x > fpc1020->nav.threshold_key_accel)
        x = (fpc1020->nav.threshold_key_accel
             + (abs_x - fpc1020->nav.threshold_key_accel) * fpc1020->nav.multiplier_key_accel) * sign_x;
    if (abs_y > fpc1020->nav.threshold_key_accel)
        y = (fpc1020->nav.threshold_key_accel
             + (abs_y - fpc1020->nav.threshold_key_accel) * fpc1020->nav.multiplier_key_accel) * sign_y;

    // Correct axis factor
    x = x * fpc1020->nav.p_multiplier_x / FLOAT_MAX;
    y = y * fpc1020->nav.p_multiplier_y / FLOAT_MAX;

    // Adjust Sensitivity
    x = x * fpc1020->nav.p_sensitivity_key / FLOAT_MAX;
    y = y * fpc1020->nav.p_sensitivity_key / FLOAT_MAX;

    input_report_rel(fpc1020->input_dev, REL_X, x);
    input_report_rel(fpc1020->input_dev, REL_Y, y);

    input_sync(fpc1020->input_dev);
}


/* -------------------------------------------------------------------- */
/* -------------------------------------------------------------------- */
static void dispatch_touch_event(fpc1020_data_t *fpc1020, int x, int y, int finger_status)
{
	int sign_x, sign_y;
	int abs_x, abs_y;

	switch(finger_status) {
		case FNGR_ST_DETECTED:
			fpc1020->nav.nav_sum_x = DEVICE_WIDTH/2;
			fpc1020->nav.nav_sum_y = DEVICE_HEIGHT/2;
			input_report_abs(fpc1020->touch_pad_dev, ABS_X, fpc1020->nav.nav_sum_x);
			input_report_abs(fpc1020->touch_pad_dev, ABS_Y, fpc1020->nav.nav_sum_y);
			input_report_abs(fpc1020->touch_pad_dev, ABS_Z, 0);
			input_report_key(fpc1020->touch_pad_dev, BTN_TOUCH, 1);
			input_sync(fpc1020->touch_pad_dev);
			break;

		case FNGR_ST_LOST:
			input_report_abs(fpc1020->touch_pad_dev, ABS_X, fpc1020->nav.nav_sum_x);
			input_report_abs(fpc1020->touch_pad_dev, ABS_Y, fpc1020->nav.nav_sum_y);
			input_report_abs(fpc1020->touch_pad_dev, ABS_Z, 0);
			input_report_key(fpc1020->touch_pad_dev, BTN_TOUCH, 0);
			input_sync(fpc1020->touch_pad_dev);
                        fpc1020->nav.nav_sum_x = 0;
                        fpc1020->nav.nav_sum_y = 0;
			break;
                case FNGR_ST_DOUBLE_TAP:
                        input_report_key(fpc1020->touch_pad_dev, KEY_POWER, 1);
                        input_sync(fpc1020->touch_pad_dev);
                        input_report_key(fpc1020->touch_pad_dev, KEY_POWER, 0);
                        input_sync(fpc1020->touch_pad_dev);
                        fpc1020->nav.throw_event = 1;
                        mdelay(50);
                        fpc1020_dynamic_debug("[FPC/TOUCH] report double click\n");
                        break;

                case FNGR_ST_HOLD:
                        input_report_key(fpc1020->touch_pad_dev, KEY_ENTER, 1);
                        input_sync(fpc1020->touch_pad_dev);
                        input_report_key(fpc1020->touch_pad_dev, KEY_ENTER, 0);
                        input_sync(fpc1020->touch_pad_dev);
                        fpc1020->nav.throw_event = 1;
                        mdelay(50);
                        fpc1020_dynamic_debug("[FPC/TOUCH] report long press\n");
                        break;


		case FNGR_ST_MOVING:
			sign_x = x > 0 ? 1 : -1;
			sign_y = y > 0 ? 1 : -1; //reverse direction
			abs_x = x > 0 ? x : -x;
			abs_y = y > 0 ? y : -y;
			if (abs_x > fpc1020->nav.threshold_key_accel)
				x = ( fpc1020->nav.threshold_key_accel
					+ ( abs_x - fpc1020->nav.threshold_key_accel ) * fpc1020->nav.multiplier_key_accel ) * sign_x;
			if (abs_y > fpc1020->nav.threshold_key_accel)
				y = ( fpc1020->nav.threshold_key_accel
					+ ( abs_y - fpc1020->nav.threshold_key_accel ) * fpc1020->nav.multiplier_key_accel ) * sign_y;

			x = x * fpc1020->nav.p_multiplier_x / FLOAT_MAX;
			y = y * fpc1020->nav.p_multiplier_y / FLOAT_MAX;

			if(x != 0 || y != 0) {
				if(abs(x) > abs(y)) {
					int newX = fpc1020->nav.nav_sum_x;
					newX += x;
					if(newX < 0) {
						newX = 0;
					} else if(newX >= DEVICE_WIDTH) {
						newX = DEVICE_WIDTH -1;
					}

					if(newX != fpc1020->nav.nav_sum_x) {
						fpc1020->nav.nav_sum_x = newX;
						input_report_abs(fpc1020->touch_pad_dev, ABS_X, fpc1020->nav.nav_sum_x);
						input_report_abs(fpc1020->touch_pad_dev, ABS_Y, fpc1020->nav.nav_sum_y);
						input_report_abs(fpc1020->touch_pad_dev, ABS_Z, 0);
						input_report_key(fpc1020->touch_pad_dev, BTN_TOUCH, 1);
						input_sync(fpc1020->touch_pad_dev);
					}
				} else {
					int newY = fpc1020->nav.nav_sum_y;
					newY += y;
					if(newY < 0) {
						newY = 0;
					} else if(newY >= DEVICE_HEIGHT) {
						newY = DEVICE_HEIGHT -1;
					}

					if(newY != fpc1020->nav.nav_sum_y) {
						fpc1020->nav.nav_sum_y = newY;

						input_report_abs(fpc1020->touch_pad_dev, ABS_X, fpc1020->nav.nav_sum_x);
						input_report_abs(fpc1020->touch_pad_dev, ABS_Y, fpc1020->nav.nav_sum_y);
						input_report_abs(fpc1020->touch_pad_dev, ABS_Z, 0);
						input_report_key(fpc1020->touch_pad_dev, BTN_TOUCH, 1);
						input_sync(fpc1020->touch_pad_dev);
					}
				}
			}
			break;

		default:
			break;
	}
}
/* -------------------------------------------------------------------- */
static void process_navi_event(fpc1020_data_t* fpc1020, int dx, int dy, int finger_status)
{
    const int THRESHOLD_RANGE_TAP = 500000;
    const int THRESHOLD_RANGE_MIN_TAP = 4;
    int filtered_finger_status = finger_status;
    static int deviation_x = 0;
    static int deviation_y = 0;
    int deviation;
    static unsigned long tick_down = 0;
    unsigned long tick_curr = jiffies * 1000 / HZ;
    unsigned long duration = 0;

    if (finger_status == FNGR_ST_DETECTED)
    {
        tick_down = tick_curr;
    }

    if (tick_down > 0)
    {
        duration = tick_curr - tick_down;
        deviation_x += dx;
        deviation_y += dy;
        deviation =
            deviation_x * deviation_x + deviation_y * deviation_y;
        fpc1020_dynamic_debug("[FPC] %s:deviation=%d, duration=%ld, finger_status=%d\n",
                __func__, deviation, duration, finger_status);
        if (deviation > THRESHOLD_RANGE_TAP)
        {
            deviation_x = 0;
            deviation_y = 0;
            tick_down = 0;
            fpc1020->nav.tap_status = -1;
#ifdef FPC1020_NAV_DBG
            printk("[FPC] %s:throw the events\n", __func__);
#endif
            if (duration > THRESHOLD_DURATION_TAP)
            {
#ifdef FPC1020_NAV_DBG
                printk("[FPC] %s: prepare long press because of outside\n", __func__);
#endif
                filtered_finger_status = FNGR_ST_HOLD;// FNGR_ST_L_HOLD;
            }
        }
        else
        {
            if (duration < THRESHOLD_DURATION_TAP)
            {
                if (finger_status == FNGR_ST_LOST && fpc1020->nav.detect_zones > 0)
                {
                    if (fpc1020->nav.tap_status == FNGR_ST_TAP
				&& tick_curr - fpc1020->nav.tap_start <= THRESHOLD_DURATION_DTAP
				&& (deviation == 0))
                    {
                        fpc1020->nav.tap_status = FNGR_ST_DOUBLE_TAP;
                        filtered_finger_status = FNGR_ST_DOUBLE_TAP;
#ifdef FPC1020_NAV_DBG
                        pr_info("[FPC] %s:prepare report double click\n", __func__);
#endif
                    }
                    else if (deviation <= THRESHOLD_RANGE_MIN_TAP)
                    {
                        filtered_finger_status = FNGR_ST_TAP;
                        fpc1020->nav.tap_status = FNGR_ST_TAP;
                        fpc1020->nav.tap_start = tick_curr;
#ifdef FPC1020_NAV_DBG
                        printk("[FPC] %s:prepare report single click\n", __func__);
#endif
                    }
		   else if ((deviation <= THRESHOLD_RANGE_MIN_TAP)
			&& tick_curr - fpc1020->nav.tap_start > fpc1020->long_key_detect_time)
		   {
#ifdef FPC1020_NAV_DBG
			printk("[FPC] %s: prepare1 report long press \n", __func__);
#endif
		        //if (deviation < THRESHOLD_RANGE_MIN_TAP)
			filtered_finger_status = FNGR_ST_HOLD;// FNGR_ST_L_HOLD;
		        fpc1020->nav.tap_status = -1;
			tick_down = 0;
			deviation_x = 0;
			deviation_y = 0;
	           }

                    else
                    {
#ifdef FPC1020_NAV_DBG
                        printk("[FPC] %s: still report finger lost\n", __func__);
#endif
                        filtered_finger_status = FNGR_ST_LOST;
                        fpc1020->nav.throw_event = 0;
                    }
                    deviation_x = 0;
                    deviation_y = 0;
                }
            }
            else if ((deviation <= THRESHOLD_RANGE_MIN_TAP ||
                   ((dy == fpc1020->nav.move_pre_y)&& (dx==fpc1020->nav.move_pre_x)))
		&& tick_curr - fpc1020->nav.tap_start > fpc1020->long_key_detect_time)
            {
#ifdef FPC1020_NAV_DBG
                printk("[FPC] %s: prepare report long press\n", __func__);
#endif
                //if (deviation < THRESHOLD_RANGE_MIN_TAP)
                filtered_finger_status = FNGR_ST_HOLD;// FNGR_ST_L_HOLD;
                fpc1020->nav.tap_status = -1;
                tick_down = 0;
                deviation_x = 0;
                deviation_y = 0;
            }
            fpc1020->nav.move_pre_x = dx;
            fpc1020->nav.move_pre_y = dy;
        }

    }

    //dev_info(&fpc1020->spi->dev, "[INFO] mode[%d] dx : %d / dy : %d\n", fpc1020->nav_settings.btp_input_mode, dx, dy);

    switch (fpc1020->nav.input_mode)
    {
        case FPC1020_INPUTMODE_TRACKPAD :
            dispatch_trackpad_event(fpc1020,
                                    dx, dy, filtered_finger_status);
            break;
        case FPC1020_INPUTMODE_DPAD:
            dispatch_dpad_event(fpc1020, dx, dy, filtered_finger_status);
            break;
        case FPC1020_INPUTMODE_TOUCH:
            dispatch_touch_event(fpc1020, dx, dy, filtered_finger_status);
            break;
        default:
            pr_info("[FPC]: undefined input mode\n");
            break;
    }
}


/* -------------------------------------------------------------------- */
int  fpc1020_input_init(fpc1020_data_t* fpc1020)
{
	int error = 0;
    int nav_image_width = NAV_IMG_W;
    int nav_image_height = NAV_IMG_H;
	//fpc1020->input_dev = input_allocate_device();
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

		if ((fpc1020->chip.type != FPC1020_CHIP_1020A)
        && (fpc1020->chip.type != FPC1020_CHIP_1021A)
        && (fpc1020->chip.type != FPC1020_CHIP_1021B)
        && (fpc1020->chip.type != FPC1020_CHIP_1150A)&&
		(fpc1020->chip.type != FPC1020_CHIP_1025X) &&
				fpc1020->chip.type != FPC1020_CHIP_1145X &&
				fpc1020->chip.type != FPC1020_CHIP_1140B) {
			dev_err(&fpc1020->spi->dev, "%s, chip not supported (%s)\n",
					__func__,
					fpc1020_hw_id_text(fpc1020));
			return -EINVAL;
		}

    dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

    fpc1020->input_dev = input_allocate_device();

    if (!fpc1020->input_dev)
    {
        dev_err(&fpc1020->spi->dev, "Input_allocate_device failed.\n");
        error  = -ENOMEM;
    }

    if (!error)
    {
        fpc1020->input_dev->name = FPC1020_INPUT_NAME;

        	/* Set event bits according to what events we are generating */
        	set_bit(EV_KEY, fpc1020->input_dev->evbit);
        	set_bit(EV_REL, fpc1020->input_dev->evbit);
        	input_set_capability(fpc1020->input_dev, EV_REL, REL_X);
        	input_set_capability(fpc1020->input_dev, EV_REL, REL_Y);
        	input_set_capability(fpc1020->input_dev, EV_KEY, BTN_MOUSE);
        	input_set_capability(fpc1020->input_dev, EV_KEY, KEY_ENTER);
#if defined (SUPPORT_DOUBLE_TAP)
        	input_set_capability(fpc1020->input_dev, EV_KEY, KEY_VOLUMEUP);
        	input_set_capability(fpc1020->input_dev, EV_KEY, KEY_VOLUMEDOWN);
#endif
       		 input_set_capability(fpc1020->input_dev, EV_KEY, KEY_POWER);
        	input_set_capability(fpc1020->input_dev, EV_KEY, KEY_SLEEP);

        	//set_bit(FPC1020_KEY_FINGER_PRESENT, fpc1020->input_dev->keybit);

        	/* Register the input device */
       		 error = input_register_device(fpc1020->input_dev);


        if (error)
        {
            dev_err(&fpc1020->spi->dev, "Input_register_device failed.\n");
            input_free_device(fpc1020->input_dev);
            fpc1020->input_dev = NULL;
        }
    }

	fpc1020->touch_pad_dev = input_allocate_device();
	if (!fpc1020->touch_pad_dev) {
        	dev_err(&fpc1020->spi->dev, "Input_allocate_device failed.\n");
        	error  = -ENOMEM;
	}
	if (!error) {
        	fpc1020->touch_pad_dev->name = FPC1020_TOUCH_PAD_DEV_NAME;

        	/* Set event bits according to what events we are generating */
        	set_bit(EV_KEY, fpc1020->touch_pad_dev->evbit);
        	/*dpad*/
		input_set_capability(fpc1020->touch_pad_dev, EV_KEY, FPC1020_KEY_FINGER_PRESENT);
		input_set_capability(fpc1020->touch_pad_dev, EV_KEY, KEY_VOLUMEUP);
		input_set_capability(fpc1020->touch_pad_dev, EV_KEY, KEY_VOLUMEDOWN);
		if (fpc1020->single_click_key_code)
			input_set_capability(fpc1020->touch_pad_dev, EV_KEY, fpc1020->single_click_key_code);
		if (fpc1020->double_click_key_code)
			input_set_capability(fpc1020->touch_pad_dev, EV_KEY, fpc1020->double_click_key_code);
		if (fpc1020->long_key_code)
			input_set_capability(fpc1020->touch_pad_dev, EV_KEY, fpc1020->long_key_code);

        	/* Register the input device */
        	error = input_register_device(fpc1020->touch_pad_dev);
		if (error) {
			dev_err(&fpc1020->spi->dev, "Input_register_device failed.\n");
			input_free_device(fpc1020->touch_pad_dev);
			fpc1020->touch_pad_dev = NULL;
		}
	}
#ifndef FPC1020_NAV_HEAP_BUF
    if (!error)
    {
        if (NAV_IMG_SIZE * 2 > fpc1020->huge_buffer_size)
        {
            dev_err(&fpc1020->spi->dev, "Huge buffer not large enough for navigation.\n");
            error = -ENOMEM;
        }
        else
        {
            p_prev_img = fpc1020->huge_buffer;
            p_curr_img = fpc1020->huge_buffer + NAV_IMG_SIZE;
        }
    }
#endif
    if (!error)
    {
        if (fpc1020->chip.type == FPC1020_CHIP_1150A)
        {
            nav_image_width = NAV_IMG_W;
            nav_image_height = NAV_IMG_H;
        }
        /* sub area setup */
        fpc1020->nav.image_nav_row_start = ((fpc1020->chip.pixel_rows - nav_image_height) / 2);
        fpc1020->nav.image_nav_row_count = nav_image_height;
        fpc1020->nav.image_nav_col_start = ((fpc1020->chip.pixel_columns - nav_image_width) / 2) / fpc1020->chip.adc_group_size;
        fpc1020->nav.image_nav_col_groups = (nav_image_width + fpc1020->chip.adc_group_size - 1) / fpc1020->chip.adc_group_size;

        fpc1020->nav.input_mode = FPC1020_INPUTMODE_DPAD;
        //fpc1020->nav.input_mode = FPC1020_INPUTMODE_TOUCH;
        init_enhanced_navi_setting(fpc1020);
    }

    return error;
}


/* -------------------------------------------------------------------- */
void  fpc1020_input_destroy(fpc1020_data_t* fpc1020)
{
    dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

    if (fpc1020->input_dev != NULL)
    { input_free_device(fpc1020->input_dev); }
    if (fpc1020->touch_pad_dev != NULL)
    { input_free_device(fpc1020->touch_pad_dev); }
}

/* -------------------------------------------------------------------- */
int fpc1020_input_enable(fpc1020_data_t* fpc1020, bool enabled)
{
    dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->nav.enabled = enabled;

	return 0;
}

/* -------------------------------------------------------------------- */
#if 1
int fpc1020_subzrea_zones_sum(int zones)
{
    u8 count = 0;
    u16 mask = FPC1020_FINGER_DETECT_ZONE_MASK;
    if (zones < 0)
        return zones;
    else {
        zones &= mask;
        while (zones && mask) {
            count += (zones & 1) ? 1 : 0;
            zones >>= 1;
            mask >>= 1;
        }
    }
        // dev_dbg(&fpc1020->spi->dev, "%s %d zones\n", __func__, count);
    return (int)count;
}

#ifdef FPC1020_NAV_HEAP_BUF
int fpc1020_alloc_img_buf(fpc1020_data_t *fpc1020)
{
	fpc1020->prev_img_buf = kzalloc(NAV_FRAME_SIZE * sizeof(u8), GFP_KERNEL);
	if (!fpc1020->prev_img_buf)
		return -1;

	fpc1020->cur_img_buf = kzalloc(NAV_FRAME_SIZE * sizeof(u8), GFP_KERNEL);
	if (!fpc1020->cur_img_buf) {
		kfree(fpc1020->prev_img_buf);
		return -1;
	}

	return 0;
}
#endif

enum hrtimer_restart fpc1020_fingerup_hrtimer_func(struct hrtimer *timer)
{
	fpc1020_data_t *fpc1020 = container_of(timer,
			fpc1020_data_t, finger_up_timer);

	fpc1020_dynamic_debug("%s %d finger is up\n", __func__, __LINE__);
	if (fpc1020->key_event->key && fpc1020->key_event->dir) {
		fpc1020->key_event->dir = 0;
		fpc1020_report_key(fpc1020);
		fpc1020->nav.sum_x = 0;
		fpc1020->nav.sum_y = 0;
		fpc1020->nav.move_up_cnt = 0;
		fpc1020->nav.move_down_cnt = 0;
		fpc1020->nav.no_move_cnt = 0;
	}

	return HRTIMER_NORESTART;
}

/* -------------------------------------------------------------------- */
int fpc1020_input_task(fpc1020_data_t* fpc1020)
{
    u8 zones;
#ifdef DEBUG_TIME
    ktime_t capture_start_time;
#endif
    bool isReverse = false;
    int dx = 0;
    int dy = 0;
    int dy_bak = 0;
    int sumX = 0;
    int sumY = 0;
    int error = 0;
    unsigned char *prevBuffer = NULL;
    unsigned char *curBuffer = NULL;
    unsigned long diffTime = 0;

    fpc1020_dynamic_debug("%s %d\n", __func__, __LINE__);
    //set_cpus_allowed_ptr(current, cpumask_of(3));
    error = fpc1020_write_nav_setup(fpc1020);
    while (!fpc1020->worker.stop_request &&
           fpc1020->nav.enabled && (error >= 0))
    {
    	fpc1020_dynamic_debug("\n%s %d\n", __func__, __LINE__);
        error = fpc1020_capture_nav_wait_finger_down(fpc1020);
        if (error < 0)
        { break; }
        error = fpc1020_write_nav_setup(fpc1020);
        if (error < 0)
        { break; }
        error = fpc1020_check_finger_present_raw(fpc1020);
        process_navi_event(fpc1020, 0, 0, FNGR_ST_DETECTED);
	fpc1020->nav.click_start = jiffies; 
        zones = fpc1020_subzrea_zones_sum(error);
        fpc1020->nav.detect_zones = zones;
        fpc1020_dynamic_debug("[FPC] report touch zones=%d\n", fpc1020->nav.detect_zones);

#ifdef DEBUG_TIME
        capture_start_time = ktime_get();
#endif // DEBUG_TIME

#ifdef FPC1020_NAV_HEAP_BUF
        error = capture_nav_image(fpc1020);
#else
        error = fpc1020_capture_buffer(fpc1020, p_prev_img, 0, NAV_IMG_SIZE);
#endif /*FPC1020_NAV_HEAP_BUF*/

#ifdef DEBUG_TIME
        dev_dbg(&fpc1020->spi->dev,
                "Navigation, init capture time: %lld ns\n",
                (ktime_get().tv64 - capture_start_time.tv64));
#endif // DEBUG_TIME
        if (error < 0)
        { break; }
#ifdef FPC1020_NAV_HEAP_BUF
        memcpy(fpc1020->prev_img_buf,
               fpc1020->huge_buffer, NAV_IMG_SIZE);
#endif

        while (!fpc1020->worker.stop_request && (error >= 0))
        {
    	    fpc1020_dynamic_debug("\n%s %d\n", __func__, __LINE__);
	    hrtimer_start(&fpc1020->finger_up_timer, ktime_set(0, 50000000), HRTIMER_MODE_REL);

            error = fpc1020_check_finger_present_sum(fpc1020);
            if (error < fpc1020->setup.capture_finger_up_threshold + 1)
            {
#ifdef  FPC1020_NAV_DBG
                pr_info("[FPC] prepare report finger up\n");
#endif

                process_navi_event(fpc1020, 0, 0, FNGR_ST_LOST);
                fpc1020->nav.throw_event = 0;
                fpc1020->nav.report_keys = 0;
                sumX = 0;
                sumY = 0;
                fpc1020->nav.time = 0;
                isReverse = false;
                break;
            }

#ifdef FPC1020_NAV_HEAP_BUF
            if (isReverse)
            {
                prevBuffer = fpc1020->cur_img_buf;
                curBuffer = fpc1020->prev_img_buf;
            }
            else
            {
                prevBuffer = fpc1020->prev_img_buf;
                curBuffer = fpc1020->cur_img_buf;
            }
#else
            u8* p_tmp_curr_img = p_curr_img;
            p_curr_img = p_prev_img;
            p_prev_img = p_tmp_curr_img;
#endif
#ifdef DEBUG_TIME
            capture_start_time = ktime_get();
#endif // DEBUG_TIME
#ifdef FPC1020_NAV_HEAP_BUF
            error = capture_nav_image(fpc1020);
#else
            error = fpc1020_capture_buffer(fpc1020, p_curr_img, 0, NAV_IMG_SIZE);
#endif
            //fpc1020_debug_store_image(p_curr_img, NAV_IMG_W, NAV_IMG_H);
#ifdef DEBUG_TIME
            dev_dbg(&fpc1020->spi->dev,
                    "Navigation, capture time: %lld ns\n",
                    (ktime_get().tv64 - capture_start_time.tv64));
#endif
            if (error < 0)
            { break; }
#ifdef FPC1020_NAV_HEAP_BUF
            memcpy(curBuffer,
                   fpc1020->huge_buffer,
                   NAV_IMG_SIZE);
#endif
            //fpc1020_debug_store_image(fpc1020->cur_img_buf, NAV_IMG_W, NAV_IMG_H);
            error = fpc1020_check_finger_present_sum(fpc1020);
    	    fpc1020_dynamic_debug("%s %d\n", __func__, __LINE__);
            if (error < fpc1020->setup.capture_finger_up_threshold + 1)
            {
#ifdef	FPC1020_NAV_DBG
		pr_info("[FPC] prepare report finger up\n");
#endif
                process_navi_event(fpc1020, 0, 0, FNGR_ST_LOST);
                fpc1020->nav.throw_event = 0;
                fpc1020->nav.report_keys = 0;
                sumX = 0;
                sumY = 0;
                fpc1020->nav.time = 0;
                isReverse = false;
                break;
            }
#ifdef FPC1020_NAV_HEAP_BUF
            isReverse = !isReverse;
#endif
#ifdef DEBUG_TIME
            capture_start_time = ktime_get();
#endif // DEBUG_TIME
#ifdef FPC1020_NAV_HEAP_BUF
            //preempt_disable();
            get_movement(prevBuffer, curBuffer, &dx, &dy);
            //preempt_enable();
#else
            //preempt_disable();
            get_movement(p_prev_img, p_curr_img, &dx, &dy);
            //preempt_enable();
#endif
#ifdef DEBUG_TIME
            dev_dbg(&fpc1020->spi->dev,
                    "Navigation, caculate time: %lld ns\n",
                    (ktime_get().tv64 - capture_start_time.tv64));
#endif
	    if (dx!=0 && dy!=0) {
		fpc1020->nav.poll_filter++;
	    }
	    fpc1020->nav.poll_counter++;
            sumX += dx;
#if 1
	    if ((dy_bak > 0 && dy < 0) ||
			    (dy_bak < 0 && dy > 0))
		    sumY = 0;
#endif
            sumY += dy;
	    if(dx > 48 || dx < -48) dx = 0;
	    if(dy > 48 || dy < -48) dy = 0;
	    dy_bak = dy;
            fpc1020_dynamic_debug("[FPC] get_movement dx=%d, dy=%d, sumx=%d, sumy=%d\n", dx, dy, sumX, sumY);
            diffTime = abs(jiffies - fpc1020->nav.time);
            if (diffTime > 0)
            {
                diffTime = diffTime * 1000000 / HZ;

                if (diffTime >= FPC1020_INPUT_POLL_INTERVAL)
                {
#ifdef FPC1020_NAV_DBG
                    pr_info("[FPC] move poll \n");
#endif
		    if (fpc1020->nav.poll_counter > 1 && fpc1020->nav.poll_filter == 0)
                    	process_navi_event(fpc1020, sumX, sumY, FNGR_ST_MOVING);
		    else
			process_navi_event(fpc1020, 0, 0, FNGR_ST_MOVING);
	            fpc1020->nav.poll_counter = 0;
		    fpc1020->nav.poll_filter = 0;
                    sumX = 0;
                    sumY = 0;
		    
                    fpc1020->nav.time = jiffies;
                }
            }

        }
    }

    if (error < 0)
    {
        dev_err(&fpc1020->spi->dev,
                "%s %s (%d)\n\n",
                __func__,
                (error == -EINTR) ? "TERMINATED" : "FAILED", error);
    }
    fpc1020->nav.enabled = false;
    hrtimer_cancel(&fpc1020->finger_up_timer);
    fpc1020_dynamic_debug("\n%s %d\n", __func__, __LINE__);
    return error;
}
#else
int fpc1020_input_task(fpc1020_data_t *fpc1020) {
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	while (!fpc1020->worker.stop_request && fpc1020->input.enabled
			&& (error >= 0)) {		
		error = fpc1020_write_nav_setup(fpc1020);

		if (!error) {
			error = fpc1020_wait_finger_present(fpc1020);
		}

		if (!error) {
			// Send key to wake device
			input_report_key(fpc1020->input_dev, FPC1020_KEY_FINGER_PRESENT, 1);
			input_report_key(fpc1020->input_dev, FPC1020_KEY_FINGER_PRESENT, 0);
			input_sync(fpc1020->input_dev);
			error = fpc1020_input_loop(fpc1020);
		}
	}

	if (error < 0) {
		dev_err(&fpc1020->spi->dev, "%s %s (%d)\n", __func__,
				(error == -EINTR) ? "TERMINATED" : "FAILED", error);
	}

	fpc1020->input.enabled = false;

	return error;
}
#endif
/* -------------------------------------------------------------------- */
static int fpc1020_write_nav_setup(fpc1020_data_t* fpc1020)
{
    const int mux = 2;
    int error = 0;
    u16 temp_u16;
    u32 temp_u32;
    u8 temp_u8;
    fpc1020_reg_access_t reg;

    dev_dbg(&fpc1020->spi->dev, "%s %d\n", __func__, mux);
#if 1
    error = fpc1020_wake_up(fpc1020);
    if (error)
    { goto out; }

    error = fpc1020_write_sensor_setup(fpc1020);
    if (error)
    { goto out; }
#endif

#if 0
    temp_u16 = fpc1020->setup.adc_shift[mux];
    temp_u16 <<= 8;
    temp_u16 |= fpc1020->setup.adc_gain[mux];
#else
    temp_u16 = 8;
    temp_u16 <<= 8;
    temp_u16 |= 5;
#endif
    FPC1020_MK_REG_WRITE(reg, FPC102X_REG_ADC_SHIFT_GAIN, &temp_u16);
    error = fpc1020_reg_access(fpc1020, &reg);
    if (error)
    { goto out; }

    //temp_u16 = fpc1020->setup.pxl_ctrl[mux];
    temp_u16 = 0x0f1f;
    FPC1020_MK_REG_WRITE(reg, FPC102X_REG_PXL_CTRL, &temp_u16);
    error = fpc1020_reg_access(fpc1020, &reg);
    if (error)
    { goto out; }

    error = fpc1020_capture_set_crop(fpc1020,
                                     NAV_IMG_CROP_X / fpc1020->chip.adc_group_size,
                                     NAV_IMG_CROP_W / fpc1020->chip.adc_group_size, NAV_IMG_CROP_Y,
                                     NAV_IMG_CROP_H);
    if (error)
    { goto out; }

    // Setup skipping of rows
    switch (NAV_IMG_ROW_SKIP)
    {
        case 1:
            temp_u32 = 0;
            break;
        case 2:
            temp_u32 = 1;
            break;
        case 4:
            temp_u32 = 2;
            break;
        case 8:
            temp_u32 = 3;
            break;
        default:
            error = -EINVAL;
            break;
    }
    if (error)
    { goto out; }

    temp_u32 <<= 8;

    // Setup masking of columns
    switch (NAV_IMG_COL_MASK)
    {
        case 1:
            temp_u32 |= 0xff;
            break;
        case 2:
            temp_u32 |= 0xcc;
            break;
        case 4:
            temp_u32 |= 0x88;
            break;
        case 8:
            temp_u32 |= 0x80;
            break;
        default:
            error = -EINVAL;
            break;
    }
    if (error)
    { goto out; }

    temp_u32 <<= 8;
    temp_u32 |= 0; // No multisampling
    FPC1020_MK_REG_WRITE(reg, FPC102X_REG_IMG_SMPL_SETUP, &temp_u32);
    error = fpc1020_reg_access(fpc1020, &reg);
    if (error)
    { goto out; }

    temp_u8 = 0x08;
    FPC1020_MK_REG_WRITE(reg, FPC1020_REG_FNGR_DET_THRES, &temp_u8);
    error = fpc1020_reg_access(fpc1020, &reg);

    dev_dbg(&fpc1020->spi->dev, "%s, (%d, %d, %d, %d)\n", __func__,
            fpc1020->nav.image_nav_col_start, fpc1020->nav.image_nav_col_groups,
            fpc1020->nav.image_nav_row_start, fpc1020->nav.image_nav_row_count);

out:
    return error;
}


/* -------------------------------------------------------------------- */

/* -------------------------------------------------------------------- */
void fpc1020_rotate_image(fpc1020_data_t* fpc1020,
                          int sensor_width,
                          int sensor_height)
{
    u8* rotateBuffer;
    u8* curr_buffer;
    int dx = 0;
    int dy = 0;
    size_t image_size_bytes = sensor_width * sensor_height;

    curr_buffer = fpc1020->huge_buffer;
    rotateBuffer = kzalloc(image_size_bytes * sizeof(u8), GFP_KERNEL);
    for (dx = 0; dx < sensor_width; dx++)
    {
        for (dy = 0; dy < sensor_height; dy++)
        {
            rotateBuffer[dx + (dy * sensor_width)] = curr_buffer[(sensor_width - dx - 1) * sensor_height + dy];
        }
    }
    memcpy(curr_buffer, &rotateBuffer[0], image_size_bytes);
    kfree(rotateBuffer);
}

#if 0
static int fpc1020_check_image(fpc1020_data_t* fpc1020,
                          int sensor_width,
                          int sensor_height)
{
    u8* curr_buffer;
    u32 count;
    int dx;
    int dy;

    curr_buffer = fpc1020->huge_buffer;
    count = 0;
    for (dx = 0; dx < sensor_width; dx++)
    {
        for (dy = 0; dy < sensor_height; dy++)
        {
            if (curr_buffer[dx + (dy * sensor_width)] == 255)
		count ++;
        }
    }
    return count;
}
#endif

/* -------------------------------------------------------------------- */
static int capture_nav_image(fpc1020_data_t* fpc1020)
{
    int error = 0;


    error = fpc1020_capture_buffer(fpc1020,
                                   fpc1020->huge_buffer,
                                   0,
                                   NAV_IMG_SIZE);
    //printk("[FPC] capture_nav_image report grey count =%d\n", fpc1020_check_image(fpc1020,NAV_IMG_W,NAV_IMG_H));

    return error;
}

