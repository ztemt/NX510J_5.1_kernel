






/*
 SiI8334 Linux Driver

 Copyright (C) 2011 Silicon Image Inc.

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License as
 published by the Free Software Foundation version 2.

 This program is distributed .as is. WITHOUT ANY WARRANTY of any
 kind, whether express or implied; without even the implied warranty
 of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 PURPOSE.  See the
 GNU General Public License for more details.
*/
/*
   @file sii8338_drv_mdt_tx.h
 */

#include <linux/i2c.h>

#define MDT_SUPPORT
//#define MDT_SUPPORT_DEBUG
#define I2C_ANALYZER
#define CONFIG_SPIDER_MHL		// Spider laptop support

#ifdef MDT_SUPPORT_DEBUG
#include <linux/input.h>
#endif

#ifdef CONFIG_SPIDER_MHL
#include "sii8338_driver_spider.h"
#define MDT_DRIVER_OFF				0x00
#define MDT_DRIVER_INITIALIZED			0x01
#define MDT_DRIVER_ACTIVE			0x10
#define MDT_DRIVER_ACTIVE_MDT			0x11
#define MDT_DRIVER_ACTIVE_SPIDER		0x12

#define SAMSUNG_ADOPTER_ID_H			0x01	// ADOPTER_ID for this phone, a Samsung phone
#define SAMSUNG_ADOPTER_ID_L			0x41
#endif

#if defined(ICS_GTi9250)

#define MHL_INT_ASSERTED_VALUE		0x01
#define MHL_INT_DEASSERTED_VALUE	0x00
#define INTERRUPT_CLEARED		MHL_INT_DEASSERTED_VALUE
#define GPIO_MHL_INT_			175

#else//	defined(ICS_BeagleboardxM) || defined(FROYO_SGHi997) || defined(GB_GTi9100) || defined(ICS_GTi9100)

#define MHL_INT_ASSERTED_VALUE		0x00
#define MHL_INT_DEASSERTED_VALUE	!(MHL_INT_ASSERTED_VALUE)
#define INTERRUPT_CLEARED		MHL_INT_ASSERTED_VALUE
//#define GPIO_MHL_INT_			135//GPIO_MHL_INT // based on board-tuna-connector.c

#endif

#define MDT_TX_SCRATCHPAD_0		0xC0
#define MDT_TX_SCRATCHPAD_2		(MDT_TX_SCRATCHPAD_0 + 2)
#define MDT_TX_SCRATCHPAD_9		(MDT_TX_SCRATCHPAD_0 + 9)

#define MDT_MIN_PACKET_LENGTH		4
#define MDT_KEYBOARD_PACKET_TAIL_LENGTH	3
#define MDT_KEYBOARD_PACKET_LENGTH	(MDT_MIN_PACKET_LENGTH + MDT_KEYBOARD_PACKET_TAIL_LENGTH)
#define MDT_MAX_PACKET_LENGTH		MDT_KEYBOARD_PACKET_LENGTH
#define MDT_MIN_PACKET_LENGTH		4
#define MDT_SCRATCHPAD_LENGTH		0x0F

#define MDT_EVENT_HANDLED		1

#define MDT_CBUS_INTR_STATUS		0x08
#define MDT_CBUS_INTR_STATUS_MASK	0x09
#define MDT_CBUS_PRI_START			0x12
#define MDT_CBUS_PRI_ADDR_CMD		0x13
#define MDT_CBUS_PRI_WR_DATA_1ST	0x14
#define MDT_CBUS_MSC_INT2_STATUS	0x1E
#define MDT_CBUS_MSC_INT2_STATUS_MASK	0x1F
#define MDT_OFFSET_SET_INT		0x20
#define MDT_OFFSET_SCRATCHPAD		0x40
#define MDT_CBUS_INTR_REG_0		0xA0
#define MDT_CBUS_INTR_REG_0_MASK	0xF0

#define MDT_BIT_RCVD_WRITE_BURST 	(1 << 0)
#define MDT_BIT_RCVD_SET_INT		(1 << 2)
#define MDT_MSC_START_BIT_WRITE_REG	(1 << 3)
#define MDT_BIT_MSC_XFR_DONE		(1 << 4)

#define MDT_INT_DSCR_CHG		(1 << 1)
#define MDT_INT_REQ_WRT			(1 << 2)
#define MDT_INT_GRT_WRT			(1 << 3)

#define MDT_MSC_REQ_DONE_MASK		(1 << 4)
#define MDT_MSC_MSG_RECD_MASK		(1 << 3)

#define MDT_CBUS_I2C_ADDRESS		0xC8

#define MSC_PREP_NO_SEND		0
#define MSC_PREP_AND_SEND		1

#define PARSE_FAILURE				4
#define PARSE_HP_ERROR				2
#define PARSE_HID_ERROR				1
#define PARSE_HID_OK				0
#define PARSE_OK				0


enum mdt_state_t {
	  WAIT_FOR_REQ_WRT = 0
	, WAIT_FOR_GRT_WRT_COMPLETE
	, WAIT_FOR_WRITE_BURST_COMPLETE
	, WAIT_FOR_REQ_WRT_COMPLETE
	, WAIT_FOR_GRT_WRT
	, WAIT_FOR_WRITE_BURST_SENT
	, WAIT_FOR_WRITE_BURST_COMPLETE_FROM_SPIDER
	, MDT_IDLE
};

union msc_request {
	struct {
		unsigned char offset;
		unsigned char first_data;
	} fields;
	unsigned char bytes[2];
};

 

enum mdt_debug {
	   IRQ_HEARTBEAT
	 , IRQ_WAKE
	 , IRQ_RECEIVED			= 2
	 , ISR_WRITEBURST_CAUGHT
	 , ISR_WRITEBURST_MISSED
	 , ISR_DEFFER_SCHEDULED
	 , ISR_DEFFER_NO
	 , ISR_DEFFER_BEGIN
	 , ISR_DEFFER_END
	 , I2C_BLOCK_R_UNKNOWN
	 , I2C_BLOCK_W_UNKNOWN		= 10
	 , I2C_BLOCK_0x72
	 , I2C_BLOCK_0x7A
	 , I2C_BLOCK_0x92
	 , I2C_BLOCK_0xC8
	 , I2C_BLOCK_0x73
	 , I2C_BLOCK_0x7B		= 0x10
	 , I2C_BLOCK_0x93
	 , I2C_BLOCK_0xC9		=0x12
	 , I2C_BYTE_0x72
	 , I2C_BYTE_0x7A		= 20
	 , I2C_BYTE_0x92
	 , I2C_BYTE_0xC8
	 , I2C_BYTE_0x73
	 , I2C_BYTE_0x7B
	 , I2C_BYTE_0x93
	 , I2C_BYTE_0xC9
	 , ISR_THREADED_BEGIN
	 , ISR_THREADED_END
	 , ISR_MDT_BEGIN		=0x1D
	 , ISR_MDT_END			=0x1E
	 , ISR_FULL_BEGIN
	 , ISR_FULL_END			= 0x20
	 , TOUCHPAD_BEGIN
	 , TOUCHPAD_END
	 , MHL_ESTABLISHED
	 , MSC_READY
	 , MDT_EVENT_PARSED
	 , MDT_UNLOCK
	 , MDT_LOCK
	 , MDT_DISCOVER_REQ
};

struct g_mdt_t {
	struct i2c_client   *i2c_cbus_client;
	unsigned char 	    is_ready;
	struct device	    *dev;
	struct class 	    *class;
	union msc_request   prior_msc_request;
					
#ifdef MDT_SUPPORT_DEBUG
	struct input_event  events[15000];
	struct input_event  *e;
#endif
#ifdef CONFIG_SPIDER_MHL
	struct sii8338_spider_data spider_data;
#endif

};


#ifdef MDT_SUPPORT
//unsigned char sii8338_irq_for_mdt(unsigned char mdt_state);
uint8_t sii8338_irq_for_mdt(uint8_t *mdt_buf); 

void MHL_log_event(int type, int code, int value);

void mdt_init(void);
void mdt_deregister(void);
void mdt_devices_deregister(void);
//void mdt_reset(void);
void mdt_update_msc_send_buffer(unsigned char cbus_reg_13, unsigned char cbus_reg_14);
#endif 
