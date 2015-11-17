/*
 * fsa8069.h -- FSA8069 Jack detection driver
 *
 * Copyright (C) 2012 Fairchild semiconductor Co.Ltd
 * Author: Chris Jeong <Chris.jeong@fairchildsemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __FSA8069_JACK_DETECT_H__
#define __FSA8069_JACK_DETECT_H__

typedef unsigned char BYTE;

enum {
	FSA_JACK_NO_DEVICE,				
	FSA_HEADSET_4POLE,				
	FSA_HEADSET_3POLE,
	FSA_MOISTURE_DET,
};

struct fsa_jack_buttons_zone {
	unsigned int code;
	unsigned int adc_low;
	unsigned int adc_high;
};

struct fsa8069_platform_data{
	void (*set_micbias_state)(bool);
	int (*get_adc_value)(void);

};

extern void fsa8069_MUSIC_mode(int onoff);
extern void fsa8069_LDO_output(int onoff);
#endif
