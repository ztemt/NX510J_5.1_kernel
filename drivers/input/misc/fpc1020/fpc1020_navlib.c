/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2014, 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include "fpc1020_navlib.h"

#define I32_MAX      (0x7fffffff)                // Max positive value representable as 32-bit signed integer 
#define ABS(X)       (((X) < 0) ? -(X) : (X))    // Fast absolute value for 32-bit integers



#if 1
#define C_H (48 / NAV_IMG_ROW_SKIP)
#define C_HP (48 / NAV_IMG_ROW_SKIP)
#define C_W 12
#define MIN_DIFF_THRESHOLD 40000


static int calculate_diff(const u8* p_curr, const u8* p_prev, int cmp_width, int cmp_height) {
	int x, y;
	int diff = 0;
	
	for (y = 0; y < cmp_height; y++) {
		for (x = 0; x < cmp_width; x++) {
			int i = x + y * NAV_IMG_W;
			diff += ABS(p_curr[i] - p_prev[i]);
		}
	}

	return diff;
}


// TODO There is a problem with this algorithm where it returns movement when the finger enters/leaves the sensor 

void get_movement(const u8* p_curr, const u8* p_prev, int* p_dx, int* p_dy) {

	int x, y;
	int compare_width  = NAV_IMG_W - C_W;
	int compare_height = NAV_IMG_H - 2 * C_HP;
	int min_diff = I32_MAX;

	// Move pointers to after padding
	p_curr += C_HP * NAV_IMG_W;
        p_prev += C_HP * NAV_IMG_W;

   	// Default vector

    	*p_dx = 0;
    	*p_dy = 0;
	//Calculate translation vector
	for (y = 0; y <= C_H; ++y) {
		for (x = 0; x <= C_W; ++x) {
			int diff;

			diff = calculate_diff(p_prev, p_curr + x + y*NAV_IMG_W, compare_width, compare_height);
			if (diff < min_diff) {
				min_diff = diff;
				*p_dx = x;
				*p_dy = y; 
			}

			diff = calculate_diff(p_curr, p_prev + x + y*NAV_IMG_W, compare_width, compare_height);
			if (diff < min_diff) {
				min_diff = diff;
				*p_dx = -x;
				*p_dy = -y; 

			}
			
			diff = calculate_diff(p_prev, p_curr + x - y*NAV_IMG_W, compare_width, compare_height);
			if (diff < min_diff) {
				min_diff = diff;
				*p_dx = x;
				*p_dy = -y; 
			}

			diff = calculate_diff(p_curr, p_prev + x - y*NAV_IMG_W, compare_width, compare_height);
			if (diff < min_diff) {
				min_diff = diff;
				*p_dx = -x;
				*p_dy = y; 
			}

		}

	}

	if (min_diff > MIN_DIFF_THRESHOLD) {
		*p_dx = 0;
		*p_dy = 0;
	}

	// Account for skipped rows
	*p_dy *= NAV_IMG_ROW_SKIP;
        *p_dy *= NAV_IMG_COL_MASK;
}

#else
#define MAXDY (64 / NAV_IMG_ROW_SKIP)
#define MAXDX (32/ NAV_IMG_COL_MASK)

// Threshold of how different the best match may be, if the best match is more different than this value then we consider the movement to be nonexistant
#define NORM_MIN_DIFF_THRESHOLD 20

static int calculate_diff(const u8* p_curr, const u8* p_prev, int cmp_width, int cmp_height, int diff_limit) {
	int x, y;
	int diff = 0;
	
	for (y = 0; y < cmp_height; y++) {
		for (x = 0; x < cmp_width; x++) {
			int i = x + y * NAV_IMG_W;
			diff += ABS(p_curr[i] - p_prev[i]);
		}

		// Not good enough, abandon early
		if (diff > diff_limit) {
			return I32_MAX;
		}
	}
	
	return diff;
}

// TODO There is a problem with this algorithm where it returns movement when the finger enters/leaves the sensor 
void get_movement(const u8* p_curr, const u8* p_prev, int* p_dx, int* p_dy) 
{
	int x, y;
	int comp_w = NAV_IMG_W - MAXDX;
	int comp_h = NAV_IMG_H - MAXDY;
	int min_diff = I32_MAX;

	// Default vector
	*p_dx = 0;
	*p_dy = 0;

	// Calculate translation vector
	for (y = -MAXDY; y <= MAXDY; ++y) {
		for (x = -MAXDX; x <= MAXDX; ++x) {
			int diff;

			diff = calculate_diff(
				p_curr + ((x + MAXDX) / 2) + ((y + MAXDY) / 2) * NAV_IMG_W,
				p_prev + ((MAXDX - x) / 2) + ((MAXDY - y) / 2) * NAV_IMG_H,
				comp_w,
				comp_h,
				min_diff);
			if (diff < min_diff) {
				min_diff = diff;
				*p_dx = x;
				*p_dy = y; 
			}
		}
	}

	if ((min_diff / (comp_w * comp_h)) > NORM_MIN_DIFF_THRESHOLD) {
		*p_dx = 0;
		*p_dy = 0;
	}
	
	// Account for masked columns and skipped rows
	*p_dx *= NAV_IMG_COL_MASK;
	*p_dy *= NAV_IMG_ROW_SKIP;
}
#endif
