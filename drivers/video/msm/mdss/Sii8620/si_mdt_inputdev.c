








/*
 Silicon Image Driver Extension

 Copyright (C) 2012 Silicon Image Inc.

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License as
 published by the Free Software Foundation version 2.

 This program is distributed .as is. WITHOUT ANY WARRANTY of any
 kind, whether express or implied; without even the implied warranty
 of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 PURPOSE.  See the
 GNU General Public License for more details.
*/
//!file     si_mdt_inputdev.c
//!brief    Silicon Image implementation of MDT function.
//
//***************************************************************************/

#include "si_mdt_inputdev.h"
#include "sii8338_drv_mdt_tx.h"
#ifdef MDT_SUPPORT

#ifdef __KERNEL__				//these includes must precede others
#include <linux/input.h>
#include <linux/kernel.h>
#ifdef  KERNEL_2_6_38_AND_LATER
#include <linux/input/mt.h>
#endif
#else
#include "si_c99support.h"
#endif

#ifndef __KERNEL__				//2012-05-11 - remove headers to support driver compilation with Linux kernel
#include "si_mhl_tx_api.h"			//	Are these necessary ?
#include "si_mhl_defs.h"			// 	This include must preceede si_mhl_tx.h
#include "si_mhl_tx.h"				//
#include "string.h"				//	String.h location varies depending on enviornment.
#else
#include <linux/string.h>
#endif
#ifdef MDT_SUPPORT_DEBUG
static ssize_t set_touch_parameter(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
#endif
 								//This is a workaround for Android ICS 4.0.4. 
#ifdef	MDT_SUPPORT_WORKAROUND			//     The workaround may be necessary for earlier ICS and JB.
struct	input_dev * get_native_touchscreen_dev(void);// In ICS 4.0.4, a MT input device created in this driver
void 	release_native_touchscreen_dev (void);	//     will incorrectly generate uevents.
#endif						//Workaround with input_dev from native touch screen driver.

#ifdef __KERNEL__
uint8_t 		g_dpad_keys[4] = {
#else
code const uint8_t 	g_dpad_keys[4] = {	//8051 memory / code space allows
#endif
	KEY_UP, KEY_RIGHT, KEY_DOWN, KEY_LEFT
};

#ifdef __KERNEL__
uint8_t usb_kbd_keycode[256] = {
#else
code const uint8_t 	usb_kbd_keycode[256] = {//8051 memory / code space allows
#endif						//   an array to live in code and 8051 doens't call Linux input
		// keycode map from usbmouse.c
	  0,  0,  0,  0, 30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38,
	 50, 49, 24, 25, 16, 19, 31, 20, 22, 47, 17, 45, 21, 44,  2,  3,
	  4,  5,  6,  7,  8,  9, 10, 11, 28,  1, 14, 15, 57, 12, 13, 26,
	 27, 43, 43, 39, 40, 41, 51, 52, 53, 58, 59, 60, 61, 62, 63, 64,
	 65, 66, 67, 68, 87, 88, 99, 70,119,110,102,104,111,107,109,106,
	105,108,103, 69, 98, 55, 74, 78, 96, 79, 80, 81, 75, 76, 77, 71,
	 72, 73, 82, 83, 86,127,116,117,183,184,185,186,187,188,189,190,
	191,192,193,194,134,138,130,132,128,129,131,137,133,135,136,113,
	115,114,  0,  0,  0,121,  0, 89, 93,124, 92, 94, 95,  0,  0,  0,
	122,123, 90, 91, 85,  0,  0,  0,  0,  0,150,155,  0,  0,  0,  0,			//support Spider laptop events
	  171,  169,  58,  250,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	200,201,207,208,213,215,216,217,226,139,172,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 29, 42, 56,125, 97, 54,100,126,164,166,165,163,161,115,114,113,
	150,158,159,128,136,177,178,176,142,152,173,140
};

struct si_mdt_inputdevs_t	mdt; 								// 20 bytes in 8051 firmware

static unsigned char  init_keyboard(void);							
static unsigned char  init_mouse (void);
static unsigned char  init_touchscreen(void);
static uint8_t 	is_mdt_dev_active(unsigned char dev_type);
static uint8_t 	is_mdt_dev_waiting(unsigned char dev_type);
//static uint8_t	is_mdt_dev_disabled(unsigned char dev_type);
static uint8_t	do_touchscreen_work(struct mdt_cursor_other_t *touchPacket);
static void keyboard_deregister(void);
static void mouse_deregister(void);
static void touch_deregister(void);
#if (SINGLE_TOUCH == 1)
static void 	submit_touchscreen_events_with_as_single_touch(void);
#elif   defined(GINGERBREAD)
static void 	submit_touchscreen_events_with_protocol_A(uint8_t contactID);
#elif defined(KERNEL_2_6_38_AND_LATER)
static void 	submit_touchscreen_events_with_protocol_B(uint8_t contactID);
#endif	
#ifndef __KERNEL__
static uint8_t *memscan(uint8_t *str_a, uint8_t key, uint8_t length)					// 11 bytes in 8051 firmware
{
	
	uint8_t *tail = str_a + length;
	uint8_t	*pStr = str_a;
	
	while(pStr != tail) {
	 	if (*pStr==key) return pStr;
		pStr++;
	}
	
	return 0;
}
#endif		   

static void input_event_mouse(uint8_t buttons, int x, int y, int z)
{
    si_input_dev 	*dev_mouse 		= mdt.dev_mouse;

    if (dev_mouse == 0) { SI_MDT_DEBUG_PRINT("MDT_ERR_NOMOUSE\n"); return; }	// return if mouse not ready

    MHL_log_event(MDT_EVENT_PARSED, 0x10, buttons);
    MHL_log_event(MDT_EVENT_PARSED, 0x11, x);
    MHL_log_event(MDT_EVENT_PARSED, 0x12, y);
    MHL_log_event(MDT_EVENT_PARSED, 0x13, z);

    if (buttons != mdt.prior_mouse_buttons) {	       
	    si_input_report_key(dev_mouse, BTN_LEFT,   	buttons & MDT_BUTTON_LEFT);	// decode data read in to determine
	    si_input_report_key(dev_mouse, BTN_RIGHT,  	buttons & MDT_BUTTON_RIGHT);	//     which buttons where asserted
	    si_input_report_key(dev_mouse, BTN_MIDDLE, 	buttons & MDT_BUTTON_MIDDLE);
	    mdt.prior_mouse_buttons = buttons;
    }

    if (x) si_input_report_rel(dev_mouse, REL_X,	x); 			// convereted to a signed type for further
    if (y) si_input_report_rel(dev_mouse, REL_Y,	y);
    if (z) si_input_report_rel(dev_mouse, REL_WHEEL, 	z);	
    si_input_sync(dev_mouse);							// generate event

}

// This function generates keyboard events
static void repeat_for_gamepad_func(struct work_struct *p) {

#define MDT_DPAD_CENTER			0x80
#define MDT_DPAD_ERROR_ALLOWANCE	5
	if ((mdt.prior_game_event.abs_x < (MDT_DPAD_CENTER - MDT_DPAD_ERROR_ALLOWANCE)) ||
		(mdt.prior_game_event.abs_x > (MDT_DPAD_CENTER + MDT_DPAD_ERROR_ALLOWANCE)) ||
	  (mdt.prior_game_event.abs_y < (MDT_DPAD_CENTER - MDT_DPAD_ERROR_ALLOWANCE)) || 
		(mdt.prior_game_event.abs_y > (MDT_DPAD_CENTER + MDT_DPAD_ERROR_ALLOWANCE))) {
		input_event_mouse (mdt.prior_mouse_buttons, mdt.prior_game_event.x_delta, mdt.prior_game_event.y_delta, 0);
		queue_delayed_work(mdt.mdt_joystick_wq, &(mdt.repeat_for_gamepad), msecs_to_jiffies(10));
	} else {
		mdt.prior_game_event.x_delta = 0;
		mdt.prior_game_event.y_delta = 0;
	}
}

// This function can be called from 3rd party drivers to 
//     generate affect a keyboard event.
static void toggle_keyboard_keycode(unsigned char keycode) {
    if (mdt.dev_keyboard == 0) {SI_MDT_DEBUG_PRINT("MDT_ERR_NOKEY\n");	return;}

    input_report_key(mdt.dev_keyboard, keycode, KEY_PRESSED);
    input_sync(mdt.dev_keyboard);

    input_report_key(mdt.dev_keyboard, keycode, KEY_RELEASED);
    input_sync(mdt.dev_keyboard);
}


// This function generates game controller events using keyboard and mouse devices
void mdt_generate_event_gamepad (struct mdt_cursor_other_t *gamePacket) 
{
    struct mdt_game_XYZRz_t	*gamePayload	= &(gamePacket->body.gameXYZRz);
    int 		deltaX 		= 0;
    int 		deltaY 		= 0;
    //int 		deltaZ 		= 0; 
    //int 		deltaRz 	= 0;
    uint8_t		mouse_buttons	= gamePacket->header.game.button;
    uint8_t 		dpad_key_bits	= 0;
    uint8_t		i;
    uint8_t		other_buttons	= gamePayload->buttons_ex;
    uint8_t		dpad_event	= gamePacket->body.suffix.other_data_bits;
    uint8_t		X 		= gamePayload->xyzRz_byteLen[MDT_GAMEXYRZ_X];	//Retrieve absolute X
    uint8_t		Y 		= gamePayload->xyzRz_byteLen[MDT_GAMEXYRZ_Y];	//Retrieve absolute Y

    mdt_register_device(DEV_TYPE_GAME);    

     //Device may not be ready immediately after registration.
    if ((mdt.dev_mouse == 0)  || (mdt.dev_keyboard == 0)) {SI_MDT_DEBUG_PRINT("MDT_ERR_NOMOUSE\n"); return;}


    // First handle the case in which the directional pad is not asserted.
    if ((mdt.prior_game_event.dpad_event == MDT_HID_DPAD_IDLE) && (dpad_event == MDT_HID_DPAD_IDLE))  {

	deltaX 		= ((X - MDT_DPAD_CENTER) / MDT_DPAD_NORMALIZE_RANGE_TO_5);
	deltaY 		= ((Y - MDT_DPAD_CENTER) / MDT_DPAD_NORMALIZE_RANGE_TO_5);

        if ((mdt.prior_game_event.abs_x != X) || (mdt.prior_game_event.abs_y != Y)) {
    	    if (!cancel_delayed_work( &(mdt.repeat_for_gamepad) ))
				flush_workqueue(mdt.mdt_joystick_wq);

	    if (mdt.prior_game_event.x_delta != 0) deltaX -= mdt.prior_game_event.x_delta;
	    if (mdt.prior_game_event.y_delta != 0) deltaY -= mdt.prior_game_event.y_delta;

	    mdt.prior_game_event.abs_x = X;
	    mdt.prior_game_event.abs_y = Y;	
	    mdt.prior_game_event.x_delta = deltaX;
	    mdt.prior_game_event.y_delta = deltaY;			    

	    queue_delayed_work( mdt.mdt_joystick_wq, &(mdt.repeat_for_gamepad), msecs_to_jiffies(10));
   	}

	input_event_mouse(mouse_buttons, deltaX, deltaY, 0);

	//Place holder.
	//Retrieve relative Z  offset for scroll wheel
	//Retrieve relative Rz offset
	
    } else {

	switch (dpad_event) {
		case MDT_HID_DPAD_000_DEGREES:		// 0 degrees
			dpad_key_bits = MDT_DPAD_UP;
			break;
		case MDT_HID_DPAD_045_DEGREES:		// 45 degrees
			dpad_key_bits = (MDT_DPAD_UP | MDT_DPAD_RIGHT); 
			break;
		case MDT_HID_DPAD_090_DEGREES:		// 90 degrees
			dpad_key_bits = MDT_DPAD_RIGHT;
			break;
		case MDT_HID_DPAD_135_DEGREES:		// 135 degress
			dpad_key_bits = (MDT_DPAD_RIGHT | MDT_DPAD_DOWN);
			break;
		case MDT_HID_DPAD_180_DEGREES:		// 180 degress
			dpad_key_bits = MDT_DPAD_DOWN;
			break;
		case MDT_HID_DPAD_225_DEGREES:		// 225 degrees
			dpad_key_bits = MDT_DPAD_DOWN | MDT_DPAD_LEFT;
			break;
		case MDT_HID_DPAD_270_DEGREES:		// 270 degress			
			dpad_key_bits = MDT_DPAD_LEFT;
			break;
		case MDT_HID_DPAD_315_DEGREES:		// 315 degress
			dpad_key_bits = MDT_DPAD_LEFT | MDT_DPAD_UP;
			break;
		case 8:
			dpad_key_bits = 0;
	}
        for (i = 0; i < 4; i++)				// generate up, down, left, right arrow key events
		input_report_key(mdt.dev_keyboard, g_dpad_keys[i], (dpad_key_bits & (1 << i))); // HID byte

	input_sync(mdt.dev_keyboard);
    }

    if (other_buttons != mdt.prior_game_event.other_buttons) {
	if (other_buttons & MDT_OTHER_BUTTONS_4) toggle_keyboard_keycode(KEY_ENTER);
	if (other_buttons & MDT_OTHER_BUTTONS_5) toggle_keyboard_keycode(KEY_ESC);
    } 

    input_event_mouse(mouse_buttons, deltaX, deltaY, 0);

    mdt.prior_game_event.dpad_event 	= dpad_event;
    mdt.prior_game_event.other_buttons	= other_buttons;
}

unsigned char	mdt_generate_event_touchscreen(struct mdt_cursor_other_t *touchPacket, uint8_t submitEvent)
{
    uint8_t contactID;

    mdt_register_device(DEV_TYPE_TOUCH);

    if (mdt.dev_touchscreen == 0 ) { SI_MDT_DEBUG_PRINT("MDT_ERR_NOTOUCHSCREEN\n"); return 0;} // return if touch is not yet ready

    if (!cancel_delayed_work( &(mdt.repeat_for_gamepad) ))//Abort any pending attempts by a game controller to move the cursor.
		flush_workqueue(mdt.mdt_joystick_wq);
    contactID = do_touchscreen_work(touchPacket);

    MHL_log_event(MDT_EVENT_PARSED, 0x80, contactID);


    if (contactID == 0xFF) return 0xFF;

    if (submitEvent) {
#if 	(SINGLE_TOUCH == 1)
	submit_touchscreen_events_with_as_single_touch();
#elif   defined(GINGERBREAD)
	submit_touchscreen_events_with_protocol_A(contactID);
#elif 	defined(KERNEL_2_6_38_AND_LATER)
	submit_touchscreen_events_with_protocol_B(contactID);
#else
	return 0;
#endif
	input_sync(mdt.dev_touchscreen);		//Generate touchscreen assertion
    }
    return 0;
}

#ifdef PHOENIX_BLADE
static unsigned char doubletouch_triggered_mouse_button_simulation(unsigned char isTouched)
{

    si_input_dev 	*dev_mouse 		= mdt.dev_mouse;
    
    //Double touch support
    //Update double touch state machine
    switch (mdt.double_touch.state)
    {
	case MDT_PB_WAIT_FOR_TOUCH_RELEASE: 			
	    if (isTouched == 1) break;		//wait for touch relesae
	    mdt.double_touch.last_release = jiffies_to_msecs(jiffies);
	    mdt.double_touch.state	  = MDT_PB_WAIT_FOR_1ST_TOUCH;
	    break;
	case MDT_PB_WAIT_FOR_1ST_TOUCH:
	    if ((isTouched == 0) ||		//wait for 1st touch in a double touch
		(mdt.touch_debounce_counter != 1)) break;	//    debounce; only transition once
	    mdt.double_touch.last_touch  = jiffies_to_msecs(jiffies);
	    mdt.double_touch.state	 = MDT_PB_WAIT_FOR_1ST_TOUCH_RELEASE;
	    break;
	case MDT_PB_WAIT_FOR_1ST_TOUCH_RELEASE:
	    if (isTouched == 1) break;		//wait for 1st touch release
	    mdt.double_touch.duration_touch = (jiffies_to_msecs(jiffies) - mdt.double_touch.last_release);
	    mdt.double_touch.last_release = jiffies_to_msecs(jiffies);
	    if  (mdt.double_touch.duration_touch < ONE_SECOND) 	//only recognize short contacts
		mdt.double_touch.state	  = MDT_PB_WAIT_FOR_2ND_TOUCH;
	    else 						//touch was too long; start over
		mdt.double_touch.state	  = MDT_PB_WAIT_FOR_1ST_TOUCH;
	    break;
	case MDT_PB_WAIT_FOR_2ND_TOUCH: 			
	    if ((isTouched == 0) ||		//wait for 2nd touch of a double touch
		(mdt.touch_debounce_counter != 1)) break;	//    debounce; only transition once
	    mdt.double_touch.duration_release = (jiffies_to_msecs(jiffies) - mdt.double_touch.last_touch) ;
	    mdt.double_touch.last_touch = jiffies_to_msecs(jiffies);
	    if  ( mdt.double_touch.duration_release< ONE_SECOND)	//only recognize short intervals between contacts
		mdt.double_touch.state	  = MDT_PB_WAIT_FOR_2ND_TOUCH_RELEASE;
	    else 	//release was too long; start over
		mdt.double_touch.state	  = MDT_PB_WAIT_FOR_TOUCH_RELEASE;
 	    break;
	case MDT_PB_WAIT_FOR_2ND_TOUCH_RELEASE:			//wait for final release in a double touch
	    if (isTouched == 1) break;				//wait for 2nd touch release
	    mdt.double_touch.duration_touch	= (jiffies_to_msecs(jiffies) - mdt.double_touch.last_release);
	    mdt.double_touch.last_release	= jiffies_to_msecs(jiffies);
	    if  (mdt.double_touch.duration_touch < ONE_SECOND) {	//only recognize short contacts
		si_input_report_key(dev_mouse, BTN_LEFT, 1);	//generate contact	
		si_input_report_key(dev_mouse, BTN_LEFT, 0);	
  	    	si_input_sync(dev_mouse);
		break;
  	    } 
	    // start over. wait for next touch
	    mdt.double_touch.state	  = MDT_PB_WAIT_FOR_1ST_TOUCH;	
     }

     return 1;
}

unsigned char mdt_generate_event_mouse_from_nativetouch(struct mdt_touch_history_t *touch_event)
{
    int 		deltaX 		= 0;
    int 		deltaY		= 0;
    unsigned char	mouse_buttons   = 0;

    //mouse pointer simulation is disabled
    if (mdt.phoenix_blade_state == MDT_PB_NATIVE_TOUCH_SCREEN) return 0xFF;	

    //if contact made per isTouched, convert touch to mouse
    if (touch_event->isTouched == 0)			
	 mdt.touch_debounce_counter = 0;			//reset counter when released
    else {
	    if (mdt.touch_debounce_counter < 5) {		//debounce; ignore the first few touches
		mdt.touch_debounce_counter++;
	    } else {						//only move if this is a subsequent touch	


		deltaX = (int)(touch_event->abs_x - mdt.prior_native_touch.abs_x);
		deltaY = (int)(touch_event->abs_y - mdt.prior_native_touch.abs_y);
								
								//reflect cached button state for pre-ICS phones		
		if (mdt.simulated.left   != 0) mouse_buttons |= MDT_BUTTON_LEFT;
		if (mdt.simulated.middle != 0) mouse_buttons |= MDT_BUTTON_MIDDLE;
		if (mdt.simulated.right  != 0) mouse_buttons |= MDT_BUTTON_RIGHT;

	        #ifdef PHOENIX_BLADE_V1
		input_event_mouse(mouse_buttons, deltaX, deltaY, 0);							
	        #else						//support every screen orientation
		switch (mdt.phoenix_blade_state) {	    
			case MDT_PB_SIMULATED_MOUSE_0: 
				deltaX *=-1;
				deltaY *=-1;
			case MDT_PB_SIMULATED_MOUSE_90:
				input_event_mouse(mouse_buttons, deltaX, deltaY, 0);
				break;
			case MDT_PB_SIMULATED_MOUSE_180:
				deltaX *=-1;
				input_event_mouse(mouse_buttons, deltaX, deltaY, 0);
				break;
			case MDT_PB_SIMULATED_MOUSE_270:
				deltaY *=-1;
				input_event_mouse(mouse_buttons, deltaX, deltaY, 0);
				break;
			case MDT_PB_NATIVE_TOUCH_SCREEN:			// do nothing for native touch screen mode
				break;
		}
	        #endif
	    }


	    mdt.prior_native_touch.abs_x = touch_event->abs_x;
	    mdt.prior_native_touch.abs_y = touch_event->abs_y;
     }

    return doubletouch_triggered_mouse_button_simulation(touch_event->isTouched);
}
#endif

static uint8_t do_touchscreen_work(struct mdt_cursor_other_t *touchPacket)
{
    int				abs_x  = 0, abs_y = 0;
    struct mdt_touch_history_t	*prior_event;
    uint8_t			contactID		= touchPacket->header.touch.contactID;

    #if defined(MDT_SUPPORT_WORKAROUND)
    if (mdt.dev_keyboard == 0) 	  {SI_MDT_DEBUG_PRINT("MDT_ERR_NOKEY\n");	return 0xFF;}
    #else
    if (mdt.dev_touchscreen == 0) {SI_MDT_DEBUG_PRINT("MDT_ERR_NOKEY\n");	return 0xFF;}
    #endif

    MHL_log_event(MDT_EVENT_PARSED, 0x20, touchPacket->header.raw);
    MHL_log_event(MDT_EVENT_PARSED, 0x21, touchPacket->body.raw[0]);
    MHL_log_event(MDT_EVENT_PARSED, 0x22, touchPacket->body.raw[1]);
    MHL_log_event(MDT_EVENT_PARSED, 0x23, touchPacket->body.raw[2]);
    MHL_log_event(MDT_EVENT_PARSED, 0x24, touchPacket->body.raw[3]);
    MHL_log_event(MDT_EVENT_PARSED, 0x25, touchPacket->body.raw[4]);
    MHL_log_event(MDT_EVENT_PARSED, 0x26, touchPacket->body.raw[5]);
    MHL_log_event(MDT_EVENT_PARSED, 0x27, contactID);

#if (SINGLE_TOUCH == 1)
    contactID = 0;
#endif
    if (contactID > 3)
	return 0xFF;
    //if (contactID == 0) {	// VNC2 firmware update to support 0 based contact ID
    //	return 0xFF;		//    for touch screens with 0 and 1 based IDs.
    //}
    //contactID--;
#endif
    prior_event	= (struct mdt_touch_history_t *)&(mdt.prior_touch_events[contactID]);    

    abs_x  = touchPacket->body.touchXYZ.xy_wordLen[MDT_TOUCH_X][MDT_TOUCH_X_LOW] |
		(touchPacket->body.touchXYZ.xy_wordLen[MDT_TOUCH_X][MDT_TOUCH_X_HIGH] << 8);
    abs_y  = touchPacket->body.touchXYZ.xy_wordLen[MDT_TOUCH_Y][MDT_TOUCH_Y_LOW] |
		(touchPacket->body.touchXYZ.xy_wordLen[MDT_TOUCH_Y][MDT_TOUCH_Y_HIGH] << 8);

#if (CORNER_BUTTON == 1)
    if (( abs_x > X_CORNER_RIGHT_LOWER ) && ( abs_y > Y_CORNER_RIGHT_LOWER )) {//Handle LOWER RIGHT corner like a EXIT button (ESC key)
	if (touchPacket->header.touch.isTouched != mdt.prior_touch_button)
	{
	    mdt.prior_touch_button	= touchPacket->header.touch.isTouched;	
	    if (touchPacket->header.touch.isTouched) {		
		#if defined(MDT_SUPPORT_WORKAROUND)

   		input_report_key(mdt.dev_keyboard, KEY_ESC, 1);
   		input_report_key(mdt.dev_keyboard, KEY_ESC, 0);
	        input_sync(	 mdt.dev_keyboard);

		#else
   		input_report_key(mdt.dev_touchscreen, KEY_ESC, 1);
   		input_report_key(mdt.dev_touchscreen, KEY_ESC, 0);
	        input_sync(	 mdt.dev_touchscreen);
		#endif
	    }
	}
	return 0xFF;
    }
#elif (ICS_BAR == 1)	
//2012-12-12 - found that JB421 doesn't allow this driver to trigger buttons on the bar.
//    	       implement custom buttons to workaround the problem.
   if ((touchPacket->header.touch.isTouched != mdt.prior_touch_button) && ( abs_x >= X_BUTTON_BAR_START)) {
	if (( abs_y > Y_BUTTON_RECENTAPPS_TOP) && ( abs_y < Y_BUTTON_RECENTAPPS_BOTTOM)) {
   		input_report_key(mdt.dev_touchscreen, KEY_MENU, 1);
   		input_report_key(mdt.dev_touchscreen, KEY_MENU, 0);
	        input_sync(	 mdt.dev_touchscreen);
	}
	else if (( abs_y > Y_BUTTON_HOME_TOP) && ( abs_y < Y_BUTTON_HOME_BOTTOM)) {
   		input_report_key(mdt.dev_touchscreen, KEY_HOMEPAGE, 1);
   		input_report_key(mdt.dev_touchscreen, KEY_HOMEPAGE, 0);
	        input_sync(	 mdt.dev_touchscreen);
	}
	else if (( abs_y > Y_BUTTON_BACK_TOP) && ( abs_y < Y_BUTTON_BACK_BOTTOM)) {
   		input_report_key(mdt.dev_touchscreen, KEY_BACK, 1);
   		input_report_key(mdt.dev_touchscreen, KEY_BACK, 0);
	        input_sync(	 mdt.dev_touchscreen);
	} 
	return 0xFF;
   }
#endif

//2012-12-12 - support dynamic configuration through ATTRIBUTES
if (mdt.swap_xy != 0) {
    prior_event->abs_x	= abs_y;
    prior_event->abs_y  = abs_x;
} else {
    prior_event->abs_x	= abs_x;
    prior_event->abs_y  = abs_y;
}

if ((mdt.x_raw != 0) && (mdt.x_screen != 0) && (prior_event->abs_x != 0)) {
      prior_event->abs_x *= mdt.x_screen;
      prior_event->abs_x /= mdt.x_raw;
}

if ((mdt.y_raw != 0) && (mdt.y_screen != 0) && (prior_event->abs_y != 0)) {
      prior_event->abs_y *= mdt.y_screen;
      prior_event->abs_y /= mdt.y_raw;
}

if ((mdt.swap_leftright) && (mdt.x_max >= prior_event->abs_x))
    prior_event->abs_x = (mdt.x_max - prior_event->abs_x);

if ((mdt.swap_updown) && (mdt.y_max >= prior_event->abs_y))
    prior_event->abs_y = (mdt.y_max - prior_event->abs_y);

prior_event->abs_x += mdt.x_shift;
prior_event->abs_y += mdt.y_shift;

    if (touchPacket->header.touch.isTouched == 0) {
	    if (prior_event->isTouched == 0) 
	    prior_event->state 	= MDT_TOUCH_INACTIVE;		//Multiple release events; declare contact inactive & ignore
    } else {
	    prior_event->state 	= MDT_TOUCH_ACTIVE;
    }
    prior_event->isTouched	= touchPacket->header.touch.isTouched;

    MHL_log_event(MDT_EVENT_PARSED, 0x28, mdt.x_screen);
    MHL_log_event(MDT_EVENT_PARSED, 0x29, mdt.x_raw);
    MHL_log_event(MDT_EVENT_PARSED, 0x2A, mdt.y_screen);
    MHL_log_event(MDT_EVENT_PARSED, 0x2B, mdt.y_raw);
    MHL_log_event(MDT_EVENT_PARSED, 0x2C, prior_event->abs_x);
    MHL_log_event(MDT_EVENT_PARSED, 0x2D, prior_event->abs_y);
    MHL_log_event(MDT_EVENT_PARSED, 0x2E, touchPacket->header.touch.isTouched);

    return contactID;
}

#if (SINGLE_TOUCH == 1)
static void 	submit_touchscreen_events_with_as_single_touch(void)
{
    struct mdt_touch_history_t 	*prior_event;

    prior_event = &(mdt.prior_touch_events[0]);
    input_report_key(mdt.dev_touchscreen, BTN_TOUCH, prior_event->isTouched);
    input_report_abs(mdt.dev_touchscreen, ABS_X, prior_event->abs_x);
    input_report_abs(mdt.dev_touchscreen, ABS_Y, prior_event->abs_y);
}
#elif   defined(GINGERBREAD)
static void 	submit_touchscreen_events_with_protocol_A(uint8_t contactID)
{
    struct mdt_touch_history_t 	*prior_event;
    uint8_t 			i;
    uint8_t 			count = 0;

    for (i=0; i< MDT_MAX_TOUCH_CONTACTS; i++) {		
	prior_event = &(mdt.prior_touch_events[i]);
	
	if (prior_event->state == MDT_TOUCH_INACTIVE) continue;

	count++;

	if (prior_event->isTouched == 0)				//Event handled; don't handle it again.			
		prior_event->state = MDT_TOUCH_INACTIVE;

        input_report_key(mdt.dev_touchscreen, BTN_TOUCH, prior_event->isTouched);
        input_report_abs(mdt.dev_touchscreen, ABS_MT_TOUCH_MAJOR, prior_event->isTouched);
	input_report_abs(mdt.dev_touchscreen, ABS_MT_TRACKING_ID, i);

	#ifndef  FROYO_SGHi997
	input_report_abs(mdt.dev_touchscreen, 		ABS_MT_WIDTH_MAJOR,  1);
	#else
        input_report_abs(mdt.dev_touchscreen, 		ABS_MT_WIDTH_MAJOR,  (i << 8) | 1);
	#endif
	input_report_abs(mdt.dev_touchscreen, 		ABS_MT_POSITION_X,	prior_event->abs_x);
	input_report_abs(mdt.dev_touchscreen,		ABS_MT_POSITION_Y,	prior_event->abs_y);
	input_mt_sync(mdt.dev_touchscreen);
    }

    if (count == 0)
	input_mt_sync(mdt.dev_touchscreen);
}
#elif defined(KERNEL_2_6_38_AND_LATER)
static void 	submit_touchscreen_events_with_protocol_B(uint8_t contactID)
{
    struct mdt_touch_history_t 	*prior_event;
    uint8_t 			i;
    uint8_t			counter = 0;

    for (i=0; i< MDT_MAX_TOUCH_CONTACTS; i++) {
		
	prior_event = &(mdt.prior_touch_events[i]);
	
	if (prior_event->state == MDT_TOUCH_INACTIVE) continue;

	MHL_log_event(MDT_EVENT_PARSED, 0x70, i);
	MHL_log_event(MDT_EVENT_PARSED, 0x71, prior_event->state);
	MHL_log_event(MDT_EVENT_PARSED, 0x72, prior_event->isTouched);
	MHL_log_event(MDT_EVENT_PARSED, 0x73, prior_event->abs_x);
	MHL_log_event(MDT_EVENT_PARSED, 0x74, prior_event->abs_y);
	MHL_log_event(MDT_EVENT_PARSED, 0x75, counter);

	input_mt_slot(mdt.dev_touchscreen, i);
	input_mt_report_slot_state(mdt.dev_touchscreen, MT_TOOL_FINGER, prior_event->isTouched);

	if (prior_event->isTouched == 0) {			//Event handled; don't handle it again.		
		prior_event->state = MDT_TOUCH_INACTIVE;
	} else {
		counter++;
		input_report_abs(mdt.dev_touchscreen, 		ABS_MT_TOUCH_MAJOR, 	15);
//#if defined(MDT_SUPPORT_WORKAROUND)
		#if !defined(FROYO_SGHi997)
		input_report_abs(mdt.dev_touchscreen, 		ABS_MT_PRESSURE, 	50);
		#endif
//#endif

		input_report_abs(mdt.dev_touchscreen, 		ABS_MT_POSITION_X,	prior_event->abs_x);
		input_report_abs(mdt.dev_touchscreen,		ABS_MT_POSITION_Y,	prior_event->abs_y);
	}

	// 2012-12-12 - found that BTN_TOUCH breaks support under JB42 on Nexus.
	#if !defined(MDT_SUPPORT_WORKAROUND) && !defined(JB_421)
 	if (counter == 1)
		input_report_key(mdt.dev_touchscreen, BTN_TOUCH, 1);
	else
		input_report_key(mdt.dev_touchscreen, BTN_TOUCH, 0);
	#endif
    }
}
#endif
					
// This function generates mouse events
void 	mdt_generate_event_keyboard(struct mdt_keyboard_event_t *keyboardPacket)
{
    	int 			i;
    si_input_dev		*dev_keyboard	= mdt.dev_keyboard;
    uint8_t			*keycodes_new	= mdt.keycodes_new;
    uint8_t			*keycodes_old	= mdt.keycodes_old;

    mdt_register_device(DEV_TYPE_KEYBOARD);

    if (dev_keyboard == 0) 	{SI_MDT_DEBUG_PRINT("MDT_ERR_NOKEY\n");	return;}

    keycodes_new[0] = keyboardPacket->header.modifier_keys;
    memcpy((keycodes_new + 1), keyboardPacket->body.keycodes_all, 6);	

    MHL_log_event(MDT_EVENT_PARSED, 0x30, keycodes_new[0]);
    MHL_log_event(MDT_EVENT_PARSED, 0x31, keycodes_new[1]);
    MHL_log_event(MDT_EVENT_PARSED, 0x32, keycodes_new[2]);

    // following code was copied from usbkeyboard.c
    for (i = 0; i < 3; i++)	{			// generate events for CRL, SHIFT, in the first
	si_input_report_key(dev_keyboard, usb_kbd_keycode[i + 224], (keycodes_new[0] >> i) & 1); // HID byte
	//printk(KERN_INFO KERN_INFO "usb_kbd_keycode[keycodes_old[i]]:0x%02x.\n", usb_kbd_keycode[keycodes_old[i]]); 
    	}				

    for (i = 1; i < 7; i++) {			// generate events for the subsequent bytes
  
		// if keycode in pervious HID payload doens't appear in NEW HID payload, generate deassertion event
		if ((keycodes_old[i] > 3) && ((uint8_t *)memscan(keycodes_new + 1, keycodes_old[i], 6) == ((uint8_t *)(keycodes_new) + 7))) {
			if (usb_kbd_keycode[keycodes_old[i]] != 0) {
				switch(usb_kbd_keycode[keycodes_old[i]]) {
					#ifdef PHOENIX_BLADE
					case KEY_F1:  
					case KEY_F2:  
					case KEY_F3:  
					case KEY_F4:  
					case KEY_F12: break;
					case KEY_LEFTCTRL:  
					case KEY_RIGHTCTRL: mdt.simulated.left = 0; break;
					#endif
					default:
					printk(KERN_INFO "usb_kbd_keycode[keycodes_old[i]]:0x%02x.\n", usb_kbd_keycode[keycodes_old[i]]); 
						//si_input_report_key(dev_keyboard, usb_kbd_keycode[keycodes_old[i]], 1);
						si_input_report_key(dev_keyboard, usb_kbd_keycode[keycodes_old[i]], 0);
						//si_input_sync(dev_keyboard);
				}
			} else {
				printk(KERN_INFO "Unknown key (scancode %#x) released.\n", keycodes_old[i]);
			}
		}
	
	 // if keycode in NEW HID paylaod doesn't appear in previous HID payload, generate assertion event
		if (keycodes_new[i] > 3 && memscan(keycodes_old + 1, keycodes_new[i], 6) == keycodes_old + 7) {
			if (usb_kbd_keycode[keycodes_new[i]] != 0) {
				switch(usb_kbd_keycode[keycodes_new[i]]) {
					#ifdef PHOENIX_BlADE
					case KEY_F1:  mdt.phoenix_blade_state = MDT_PB_SIMULATED_MOUSE_0; 	mdt_register_device(DEV_TYPE_MOUSE);     break;
					case KEY_F2:  mdt.phoenix_blade_state = MDT_PB_SIMULATED_MOUSE_90; 	mdt_register_device(DEV_TYPE_MOUSE);     break;
					case KEY_F3:  mdt.phoenix_blade_state = MDT_PB_SIMULATED_MOUSE_180;	mdt_register_device(DEV_TYPE_MOUSE);     break;
					case KEY_F4:  mdt.phoenix_blade_state = MDT_PB_SIMULATED_MOUSE_270; 	mdt_register_device(DEV_TYPE_MOUSE);     break;
					case KEY_F12: mdt.phoenix_blade_state = MDT_PB_NATIVE_TOUCH_SCREEN;	mdt_deregister_device(DEV_TYPE_MOUSE,1); break;
					case KEY_LEFTCTRL:  
					case KEY_RIGHTCTRL: mdt.simulated.left = 1; break;
					#endif
					default:
					printk(KERN_INFO"usb_kbd_keycode[keycodes_new[i]]:0x%02x.\n", usb_kbd_keycode[keycodes_new[i]]); 
					
						si_input_report_key(dev_keyboard, usb_kbd_keycode[keycodes_new[i]], 1);
						//si_input_report_key(dev_keyboard, usb_kbd_keycode[keycodes_new[i]], 0);
						//si_input_sync(dev_keyboard);
				}
			} else {
				printk(KERN_INFO "Unknown key (scancode %#x) pressed.\n", keycodes_new[i]);
			}
		
		}
     }

     si_input_sync(dev_keyboard);	  			// generate event
     memcpy(keycodes_old, keycodes_new, 7);			// NEW HID payload is now OLD
}
	  
// This function is a wrapper for input_event_mouse used to generate mouse events
void mdt_generate_event_mouse(struct mdt_cursor_mouse_t *mousePacket)
{
	mdt_register_device(DEV_TYPE_MOUSE);

	input_event_mouse(mousePacket->header.button,
			mousePacket->body.XYZ.x_byteLen,
			mousePacket->body.XYZ.y_byteLen,
			mousePacket->body.XYZ.z_byteLen);
}


// Local, keyboard specific helper function for initialize keybaord input_dev
static unsigned char init_keyboard(void) 
{	
#ifdef __KERNEL__
	int i;
	int error;
#endif
	si_input_dev *dev_keyboard;

    dev_keyboard = si_input_allocate_device();
    if (!dev_keyboard)
	{
		SI_MDT_DEBUG_PRINT("MDTkeyboard: Not enough memory\n");
		return -ENOMEM;
    }
#ifdef __KERNEL__
	set_bit(EV_KEY, dev_keyboard->evbit);
	set_bit(EV_REP, dev_keyboard->evbit);			//driver doesn't use this but, can in the future

	dev_keyboard->phys 		 = "atakbd/input0";
	dev_keyboard->name		 = "MDTkeyboard";
	dev_keyboard->id.bustype = BUS_HOST;
	dev_keyboard->keycode 	 = usb_kbd_keycode;
	dev_keyboard->keycodesize= sizeof(unsigned char);
	dev_keyboard->keycodemax = ARRAY_SIZE(usb_kbd_keycode);

	for (i = 1; i < 256; i++)
		set_bit(usb_kbd_keycode[i], dev_keyboard->keybit);

	dev_keyboard->id.bustype = BUS_USB;
	dev_keyboard->id.vendor  = 0x1095;
	dev_keyboard->id.product = 0x8240;
	dev_keyboard->id.version = 0xA;					//use version to distinguish mouse from keyboard

	error = input_register_device(dev_keyboard);
	if (error) {
		SI_MDT_DEBUG_PRINT("MDTkeyboard: Failed to register device\n");
		return error;
	}
	mdt.dev_keyboard = dev_keyboard;
#endif
	SI_MDT_DEBUG_PRINT("MDTkeyboard: driver loaded\n");
	return REGISTRATION_SUCCESS;
}

// Local, mouse specific helper function for initialize mouse input_dev
static unsigned char init_mouse(void)
{
	si_input_dev *dev_mouse;
#ifdef __KERNEL__
	int error;
#endif

    dev_mouse = si_input_allocate_device();
    if (!dev_mouse) {
		SI_MDT_DEBUG_PRINT("MDTmouse: Not enough memory\n");
		return -ENOMEM;
	}

#ifdef __KERNEL__
	set_bit(EV_REL, 	dev_mouse->evbit);
	set_bit(EV_KEY,		dev_mouse->evbit);
	set_bit(BTN_LEFT,	dev_mouse->keybit);
	set_bit(BTN_RIGHT,	dev_mouse->keybit);
	set_bit(BTN_MIDDLE,	dev_mouse->keybit);
	set_bit(BTN_SIDE,	dev_mouse->keybit);
	set_bit(BTN_EXTRA,	dev_mouse->keybit);
	set_bit(REL_X,		dev_mouse->relbit);
	set_bit(REL_Y,		dev_mouse->relbit);
	set_bit(REL_WHEEL,	dev_mouse->relbit);

	dev_mouse->name	      = "MDTmouse";
	dev_mouse->id.bustype = BUS_USB;
	dev_mouse->id.vendor  = 0x1095;
	dev_mouse->id.product = 0x8240;
	dev_mouse->id.version = 0xB;				//use version to distinguish mouse from keyboard

	error = input_register_device(dev_mouse);
        if (error) {
		SI_MDT_DEBUG_PRINT("MDTmouse: Failed to register device\n");
		return error;
	}
	mdt.dev_mouse = dev_mouse;

	mdt.prior_game_event.x_delta		= 0;
	mdt.prior_game_event.y_delta		= 0;
	mdt.prior_game_event.abs_x		= 0;
	mdt.prior_game_event.abs_y		= 0;
	mdt.prior_game_event.dpad_event		= 0;
	mdt.prior_game_event.other_buttons	= 0;
	mdt.prior_mouse_buttons 		= 0;

#endif

	SI_MDT_DEBUG_PRINT("MDTmouse: driver loaded\n");
	return REGISTRATION_SUCCESS;
}

// Local, touchscreen specific helper function for initialize touchscreen input_dev
static unsigned char init_touchscreen(void)
{
#ifdef MDT_SUPPORT_WORKAROUND
	mdt.dev_touchscreen = get_native_touchscreen_dev();

	if (mdt.dev_touchscreen != 0) 
		return REGISTRATION_SUCCESS;
	else
		return REGISTRATION_ERROR;
#else
	si_input_dev *dev_touchscreen;
#ifdef __KERNEL__
        int error;
#endif
        dev_touchscreen = si_input_allocate_device();
        if (!dev_touchscreen) {
		SI_MDT_DEBUG_PRINT("MDTtouchscreen: Not enough memory\n");
		return -ENOMEM;
        }

#ifdef __KERNEL__
#if (SINGLE_TOUCH == 0)
	#ifdef  KERNEL_2_6_38_AND_LATER
	input_mt_init_slots (dev_touchscreen, MDT_MAX_TOUCH_CONTACTS);
	#endif
#endif
	dev_touchscreen->name	    = "MDTtouchscreen";
	dev_touchscreen->id.bustype = BUS_USB;
	dev_touchscreen->id.vendor  = 0x1095;
	dev_touchscreen->id.product = 0x8240;
	dev_touchscreen->id.version = 0xC;				//use version to distinguish touchscreen from keyboard and mouse

#if (SINGLE_TOUCH == 1)
	__set_bit(EV_ABS, 	   dev_touchscreen->evbit);
	__set_bit(ABS_X,	   dev_touchscreen->absbit);
	__set_bit(ABS_Y,	   dev_touchscreen->absbit);
	__set_bit(EV_KEY, 	   dev_touchscreen->evbit);
	#if     (CORNER_BUTTON == 1)
	__set_bit(KEY_ESC,	   dev_touchscreen->keybit);
	#endif
	__set_bit(BTN_TOUCH,	   dev_touchscreen->keybit);
	#ifdef  KERNEL_2_6_38_AND_LATER
	__set_bit(INPUT_PROP_DIRECT, dev_touchscreen->propbit);
	#endif
	input_set_abs_params(	   dev_touchscreen, ABS_X,		0, mdt.x_max, 0, 0);	
	input_set_abs_params(	   dev_touchscreen, ABS_Y, 		0, mdt.y_max, 0, 0);
#else
	__set_bit(EV_ABS,          dev_touchscreen->evbit);
	__set_bit(EV_KEY, 	   dev_touchscreen->evbit);
	#ifdef  KERNEL_2_6_38_AND_LATER
	__set_bit(EV_SYN, 	    dev_touchscreen->evbit);
	__set_bit(MT_TOOL_FINGER,   dev_touchscreen->keybit);
	__set_bit(INPUT_PROP_DIRECT,dev_touchscreen->propbit);
	input_mt_init_slots (dev_touchscreen, MDT_MAX_TOUCH_CONTACTS);
	#else
	__set_bit(BTN_TOUCH,	   dev_touchscreen->keybit);
	input_set_abs_params(	   dev_touchscreen, ABS_MT_WIDTH_MAJOR, 0, 3,	  0, 0);
	input_set_abs_params(	   dev_touchscreen, ABS_MT_TRACKING_ID, 0, 3,	  0, 0);
	#endif
	#if     (CORNER_BUTTON == 1)
	__set_bit(KEY_ESC,	   dev_touchscreen->keybit);
	#endif
	input_set_abs_params(	   dev_touchscreen, ABS_MT_TOUCH_MAJOR, 0, 30,	  0, 0);
	//#if defined(MDT_SUPPORT_WORKAROUND)
	#if !defined(FROYO_SGHi997)
	input_set_abs_params(	   dev_touchscreen, ABS_MT_PRESSURE, 	0, 255,	  0, 0);
	#endif
	//#endif
	input_set_abs_params(      dev_touchscreen, ABS_MT_POSITION_X,  0, mdt.x_max, 0, 0);
	input_set_abs_params(	   dev_touchscreen, ABS_MT_POSITION_Y,  0, mdt.y_max, 0, 0);
#endif

#ifdef JB_421
	#ifdef ICS_BAR
	    #ifdef Y_BUTTON_RECENTAPPS_TOP 
	    __set_bit(KEY_MENU, dev_touchscreen->keybit);
	    #endif
	    #ifdef Y_BUTTON_HOME_TOP 
	    __set_bit(KEY_HOMEPAGE, dev_touchscreen->keybit);
	    #endif
	    #ifdef Y_BUTTON_BACK_TOP
	    __set_bit(KEY_BACK, dev_touchscreen->keybit);
	    #endif
	#endif
#endif
        error = input_register_device(dev_touchscreen);
        if (error) {
		SI_MDT_DEBUG_PRINT("MDTtouch: Failed to register device\n");
		return error;
        }
	mdt.dev_touchscreen = dev_touchscreen;
#endif
#endif
	// need to initialize history; in parcitular initialize state elements with MDT_TOUCH_INACTIVE
	memset(mdt.prior_touch_events, 0, MDT_MAX_TOUCH_CONTACTS * sizeof(mdt.prior_touch_events[0]));

#ifdef PHOENIX_BLADE
	mdt.phoenix_blade_state = MDT_PB_NATIVE_TOUCH_SCREEN;
#endif

	SI_MDT_DEBUG_PRINT("MDTtouchscreen: driver loaded\n");

	return REGISTRATION_SUCCESS; 
}
 

// Local, helper functions to initialize input_dev
static uint8_t is_mdt_dev_active(unsigned char dev_type) {
	if (mdt.is_dev_registered[dev_type]==INPUT_ACTIVE) 
		return 1;
	else 
		return 0;
}
#if 0
static uint8_t is_mdt_dev_disabled(unsigned char dev_type) {
	if (mdt.is_dev_registered[dev_type]==INPUT_DISABLED) 
		return 1;
	else 
		return 0;
}
#endif
static uint8_t is_mdt_dev_waiting(unsigned char dev_type) {
	if (mdt.is_dev_registered[dev_type]==INPUT_WAITING_FOR_REGISTRATION) 
		return 1;
	else 
		return 0;
}

uint8_t mdt_input_init(void) {

	memset(mdt.is_dev_registered,	INPUT_WAITING_FOR_REGISTRATION,	DEV_TYPE_COUNT);
	memset(mdt.keycodes_old,	0,				HID_INPUT_REPORT);
	memset(mdt.keycodes_new,	0,				HID_INPUT_REPORT);
	// need to initialize history; in parcitular initialize state elements with MDT_TOUCH_INACTIVE
	memset(mdt.prior_touch_events,	0, MDT_MAX_TOUCH_CONTACTS *	sizeof(mdt.prior_touch_events[0]));
	memset(&mdt.prior_game_event, 	0,				sizeof(mdt.prior_game_event));

	mdt.dev_mouse 			= 0;
	mdt.dev_keyboard 		= 0;
	mdt.dev_touchscreen 		= 0;
	mdt.mdt_joystick_wq		= 0;

	mdt.prior_game_event.x_delta	= 0;
	mdt.prior_game_event.y_delta	= 0;
	mdt.prior_game_event.abs_x	= 0;
	mdt.prior_game_event.abs_y	= 0;
	mdt.prior_game_event.dpad_event	= 0;
	mdt.prior_game_event.other_buttons = 0;
	mdt.prior_mouse_buttons 	= 0;

#ifdef PHOENIX_BLADE
	mdt.phoenix_blade_state		= MDT_PB_NATIVE_TOUCH_SCREEN;
	mdt.touch_debounce_counter	= 0;
	mdt.prior_native_touch.abs_x	= 0;
	mdt.prior_native_touch.abs_y	= 0;
	//mdt.prior_native_touch.isTouched= 0;				// These 2 fields are not used for pior_native_touch
	//mdt.prior_native_touch.state	= 0;

	mdt.simulated.left		= 0;
	mdt.simulated.middle		= 0;
	mdt.simulated.right		= 0;

	mdt.double_touch.duration_release=0;
	mdt.double_touch.duration_touch  =0;
	mdt.double_touch.last_release	 =0;
	mdt.double_touch.last_touch	 =0;
	mdt.double_touch.state		 =MDT_PB_WAIT_FOR_TOUCH_RELEASE;
#endif
	printk(KERN_INFO "mdt_input_init....\n");

   	mdt.mdt_joystick_wq = create_singlethread_workqueue("mdt_joystick_wq");
	INIT_DELAYED_WORK(&(mdt.repeat_for_gamepad), repeat_for_gamepad_func);

#ifdef  MDT_SUPPORT_WORKAROUND
	release_native_touchscreen_dev();
#endif

	mdt.x_max = X_MAX;
	mdt.x_screen = SCALE_X_SCREEN;
	mdt.x_raw = SCALE_X_RAW;
	mdt.x_shift = X_SHIFT;
	mdt.y_max = Y_MAX; 	
	mdt.y_screen = SCALE_Y_SCREEN; 
	mdt.y_raw = SCALE_Y_RAW; 
	mdt.y_shift = Y_SHIFT;
	mdt.swap_xy = SWAP_XY;
	mdt.swap_updown = SWAP_UPDOWN;
	mdt.swap_leftright = SWAP_LEFTRIGHT;

	return 0;
}

static void mouse_deregister(void) {
	if (mdt.dev_mouse == 0) return;

	input_unregister_device(mdt.dev_mouse);
	mdt.prior_mouse_buttons 		= 0;
	mdt.dev_mouse 				= 0;
}

static void touch_deregister(void) {
	if (mdt.dev_touchscreen == 0) return;

#ifdef PHOENIX_BLADE
	mdt.phoenix_blade_state = MDT_PB_NATIVE_TOUCH_SCREEN;
#endif

#ifdef MDT_SUPPORT_WORKAROUND
	release_native_touchscreen_dev();
#else
	input_unregister_device(mdt.dev_touchscreen);
#endif
	memset(mdt.prior_touch_events,	0, MDT_MAX_TOUCH_CONTACTS *	sizeof(mdt.prior_touch_events[0]));
	mdt.dev_touchscreen	= 0;
}


// 2012-11-19
static void keyboard_deregister(void) {
	if (mdt.dev_keyboard == 0) return;

	input_unregister_device(mdt.dev_keyboard);
	mdt.dev_keyboard = 0;
}

// 2012-11-19 - internal helper for new hot-plug API
static uint8_t registration_helper(unsigned char mdt_device_type)
{								// this is the recursive part of the register function
    MHL_log_event(MDT_EVENT_PARSED, 0xAB, mdt_device_type);

    if (mdt_device_type == DEV_TYPE_GAME) {

	if (registration_helper(DEV_TYPE_KEYBOARD) == REGISTRATION_ERROR) return REGISTRATION_ERROR;
	if (registration_helper(DEV_TYPE_MOUSE) == REGISTRATION_ERROR)	  return REGISTRATION_ERROR;

    } else if (mdt_device_type == DEV_TYPE_KEYBOARD) {

	if (mdt.dev_keyboard != 0) return REGISTRATION_SUCCESS;	//device is already registered
	return init_keyboard();

    } else if (mdt_device_type == DEV_TYPE_MOUSE) {

	if (mdt.dev_mouse != 0 ) 			return REGISTRATION_SUCCESS;	//device is already registered
	if (init_mouse() != REGISTRATION_SUCCESS) 	return REGISTRATION_ERROR;
	mdt_deregister_device(DEV_TYPE_TOUCH,1);		// for now, can't have both a pointer and touch
								//      this logic will create a problem in certain srequences
    } else if (mdt_device_type == DEV_TYPE_TOUCH) {

	if (mdt.dev_touchscreen != 0) 			return REGISTRATION_SUCCESS;
	if (init_touchscreen() != REGISTRATION_SUCCESS) return REGISTRATION_ERROR;
	mdt_deregister_device(DEV_TYPE_MOUSE,1);		// for now, can't have both a pointer and touch
								//      this logic will create a problem in certain srequences
    }

    return REGISTRATION_SUCCESS;
}

// 2012-11-19 - API for hot unplug
uint8_t mdt_register_device(unsigned char mdt_device_type)
{
    unsigned char error = 0;

    MHL_log_event(MDT_EVENT_PARSED, 0xAA, mdt_device_type);

    if ((mdt_device_type >= DEV_TYPE_COUNT) ||
	(is_mdt_dev_waiting(mdt_device_type) == 0)) return REGISTRATION_ERROR;	//MDT was not previously initialized    

    error = registration_helper(mdt_device_type);		//Go to recursive part of the function; don't update is_dev state there

    if (error != REGISTRATION_SUCCESS) {
	mdt.is_dev_registered[mdt_device_type] = INPUT_DISABLED;
    } else
	mdt.is_dev_registered[mdt_device_type] = INPUT_ACTIVE;

    MHL_log_event(MDT_EVENT_PARSED, 0xAC, error);

    return error;
}

// 2012-11-19 - API for hot plug
uint8_t mdt_deregister_device(unsigned char mdt_device_type, unsigned char isReset)
{
     if ((mdt_device_type >= DEV_TYPE_COUNT) ||
	(is_mdt_dev_active(mdt_device_type) == 0)) return REGISTRATION_ERROR; 	// error condition; nothing to deregister

     MHL_log_event(MDT_EVENT_PARSED, 0xDD, mdt_device_type);
     MHL_log_event(MDT_EVENT_PARSED, 0xDE, mdt.is_dev_registered[mdt_device_type]);

     switch (mdt_device_type) {
	case DEV_TYPE_GAME:							
		if (!cancel_delayed_work( &(mdt.repeat_for_gamepad) ))		// flush but, leave the workqueue in place
			flush_workqueue(mdt.mdt_joystick_wq);
		if (is_mdt_dev_active(DEV_TYPE_MOUSE) == 0)			// deregister only if not shared
			mouse_deregister();
		if (is_mdt_dev_active(DEV_TYPE_KEYBOARD) == 0)
			keyboard_deregister();
		break;
	case DEV_TYPE_TOUCH:
		touch_deregister();
		break;
	case DEV_TYPE_MOUSE:
		mouse_deregister();		
		break;
	case DEV_TYPE_KEYBOARD:
		keyboard_deregister();
		break;

     }

     //if (isReset == 0) 
     //	mdt.is_dev_registered[mdt_device_type]	= INPUT_DISABLED;
     //else
     mdt.is_dev_registered[mdt_device_type]	= INPUT_WAITING_FOR_REGISTRATION;	

     MHL_log_event(MDT_EVENT_PARSED, 0xDF, mdt.is_dev_registered[mdt_device_type]);

     return REGISTRATION_SUCCESS;
}


void mdt_input_deregister(void) {

// 2012-11-19 support new hot-plug and unplug API
	mdt_deregister_device(DEV_TYPE_KEYBOARD,1);
	mdt_deregister_device(DEV_TYPE_MOUSE,   1);
	mdt_deregister_device(DEV_TYPE_TOUCH,   1);
	mdt_deregister_device(DEV_TYPE_GAME,	1);

	if (mdt.mdt_joystick_wq) {
	    destroy_workqueue(mdt.mdt_joystick_wq);
	    mdt.mdt_joystick_wq = 0;
	}
}	

 
#ifdef PHOENIX_BLADE
unsigned char get_mdtdemo_simulated_btn(unsigned char button) {
    switch(button) {
	case (unsigned char)BTN_LEFT: 	return mdt.simulated.left;	break;
	case (unsigned char)BTN_MIDDLE:	return mdt.simulated.middle;	break;
	case (unsigned char)BTN_RIGHT: 	return mdt.simulated.right;	break;
    };
    return 0xFF;
}

void set_mdtdemo_simulated_btn(unsigned char button, unsigned char value, unsigned char do_sync) {

    si_input_dev 	*dev_mouse 		= mdt.dev_mouse;

    if (dev_mouse == 0)   { SI_MDT_DEBUG_PRINT("MDT_ERR_NOMOUSE\n"); return; }

    switch(button) {
	case (unsigned char)BTN_LEFT: 	mdt.simulated.left = value;	break;
	case (unsigned char)BTN_MIDDLE:	mdt.simulated.middle = value;	break;
	case (unsigned char)BTN_RIGHT:	mdt.simulated.right = value;	break;
    };

    if (do_sync) {							// return if mouse not ready	

   	if ((value & mdt.prior_mouse_buttons) == 0) {	    

	    si_input_report_key(dev_mouse, button, value);	
	    si_input_sync(dev_mouse);
	    if(value)
		    mdt.prior_mouse_buttons |=  button;
	    else
		    mdt.prior_mouse_buttons &= ~button;
    	}

    }
	
}

unsigned char get_phoenix_blade_state(void) {
    return (unsigned char)mdt.phoenix_blade_state;
}

void set_phoenix_blade_state(unsigned char value) {
    mdt.phoenix_blade_state = value;

    if (value == MDT_PB_NATIVE_TOUCH_SCREEN)	
	mdt_deregister_device(DEV_TYPE_MOUSE,1);
    else
	mdt_register_device(DEV_TYPE_MOUSE);
}
#endif

#ifdef MDT_SUPPORT_DEBUG
// 2012-12-12 - The following code is for debugging purposes. Since HID decoding for touch
//     still seems like black magic, the following allows configuration selection through
//     characterization / experimentation.

static DEVICE_ATTR(touch_x_max, 	0222, NULL, set_touch_parameter);
static DEVICE_ATTR(touch_x_raw, 	0222, NULL, set_touch_parameter);
static DEVICE_ATTR(touch_x_screen, 	0222, NULL, set_touch_parameter);
static DEVICE_ATTR(touch_y_max, 	0222, NULL, set_touch_parameter);
static DEVICE_ATTR(touch_y_raw, 	0222, NULL, set_touch_parameter);
static DEVICE_ATTR(touch_y_screen, 	0222, NULL, set_touch_parameter);
static DEVICE_ATTR(touch_swap_xy, 	0222, NULL, set_touch_parameter);
static DEVICE_ATTR(touch_swap_leftright,0222, NULL, set_touch_parameter);
static DEVICE_ATTR(touch_swap_updown, 	0222, NULL, set_touch_parameter);


static ssize_t set_touch_parameter(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{	
	unsigned short i;

	#ifdef KERNEL_2_6_38_AND_LATER
	if (simple_strtos16(buf, 10, &i)) return -EINVAL;
	#else
	i = (unsigned short)simple_strtoul(buf, NULL, 10);
	#endif

	mdt_deregister_device(DEV_TYPE_TOUCH,1);

	if 	(attr == (struct device_attribute *)&(dev_attr_touch_x_max.attr)) {	mdt.x_max = i;
			    MHL_log_event(MDT_EVENT_PARSED, 0xF0, mdt.x_max);
		}
	else if (attr == (struct device_attribute *)&(dev_attr_touch_x_raw.attr)) {	mdt.x_raw = i;
			    MHL_log_event(MDT_EVENT_PARSED, 0xF1, mdt.x_raw);
		}
	else if (attr == (struct device_attribute *)&(dev_attr_touch_x_screen.attr)){ 	mdt.x_screen = i;
			    MHL_log_event(MDT_EVENT_PARSED, 0xF2, mdt.x_screen);
		}
	else if (attr == (struct device_attribute *)&(dev_attr_touch_y_max.attr)) {	mdt.y_max = i;
			    MHL_log_event(MDT_EVENT_PARSED, 0xF3, mdt.y_max);
		}
	else if (attr == (struct device_attribute *)&(dev_attr_touch_y_raw.attr)) {	mdt.y_raw = i;
			    MHL_log_event(MDT_EVENT_PARSED, 0xF4, mdt.y_raw);
		}
	else if (attr == (struct device_attribute *)&(dev_attr_touch_y_screen.attr)){ 	mdt.y_screen = i;
			    MHL_log_event(MDT_EVENT_PARSED, 0xF5, mdt.y_screen);
		}
	else if (attr == (struct device_attribute *)&(dev_attr_touch_swap_xy.attr)){	mdt.swap_xy = i;
			    MHL_log_event(MDT_EVENT_PARSED, 0xF6, mdt.swap_xy);
		}
	else if (attr == (struct device_attribute *)&(dev_attr_touch_swap_updown.attr)){mdt.swap_updown = i;
			    MHL_log_event(MDT_EVENT_PARSED, 0xF7, mdt.swap_updown);
		}
	else if (attr == (struct device_attribute *)&(dev_attr_touch_swap_leftright.attr)){
		mdt.swap_leftright = i;
			    MHL_log_event(MDT_EVENT_PARSED, 0xF8, mdt.swap_leftright);
		}
	
	mdt_register_device(DEV_TYPE_TOUCH); 
	return count;
}

static struct attribute *mdt_attributes[] = {
	&dev_attr_touch_x_max.attr,
	&dev_attr_touch_x_raw.attr,
	&dev_attr_touch_x_screen.attr,
	&dev_attr_touch_y_max.attr,
	&dev_attr_touch_y_raw.attr,
	&dev_attr_touch_y_screen.attr,
	&dev_attr_touch_swap_xy.attr,
	&dev_attr_touch_swap_leftright.attr,
	&dev_attr_touch_swap_updown.attr,
	NULL,
};

struct attribute_group mdt_attr_group = {
	.attrs = mdt_attributes,
};
#endif
