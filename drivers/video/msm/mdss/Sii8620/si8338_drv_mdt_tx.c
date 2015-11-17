








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
//!file     sii8338_drv_mdt_tx.c
//!brief    Silicon Image implementation of MDT function.
//
//***************************************************************************/

// Include headers from parent driver
//     These definitions may be referenced else where: source or headers

// include dedicated headers
#include "si_mdt_inputdev.h"
#include "sii8338_drv_mdt_tx.h"
//#include "si_mhl_tx_api.h"
#ifdef CONFIG_SPIDER_MHL
#include "spidermhl.h"
#endif


#ifdef MDT_SUPPORT
#ifdef MDT_SUPPORT_DEBUG
static void init_log_file(void);
static void deregster_log_file(void);
#endif

static struct g_mdt_t g_mdt = {
	.i2c_cbus_client	= 0,
	.is_ready 		= 0,
	.dev			= 0,
	.class			= 0,
	.prior_msc_request.bytes= {0,0},	
};

#endif
//2012-12-12 - allow run time touch calibration
extern struct attribute_group mdt_attr_group;

//in case block read/write is not provided, implement it here

#if 0
static void mdt_cbus_read_block(unsigned char reg_offset, unsigned char byte_count, unsigned char *data)
{
	unsigned char ret;
#if 0
	struct i2c_msg i2cMsg[2];

	i2cMsg[0].addr = (MDT_CBUS_I2C_ADDRESS >> 1);
	i2cMsg[0].flags = 0;
	i2cMsg[0].len = 1;
 	i2cMsg[0].buf = &reg_offset;

	i2cMsg[1].addr = (MDT_CBUS_I2C_ADDRESS >> 1);
	i2cMsg[1].flags = 1;
	i2cMsg[1].len = byte_count;
 	i2cMsg[1].buf = data;

	if(g_mdt.i2c_cbus_client == 0)
	{
		return;
	}
	
	ret = i2c_transfer(g_mdt.i2c_cbus_client->adapter, i2cMsg, 2);

	if (ret < 0)
	{
		return;
	}

#else

	    int32_t	status;
        u32 client_main_addr = g_mdt.i2c_cbus_client->addr;
        g_mdt.i2c_cbus_client->addr = MDT_CBUS_I2C_ADDRESS >> 1;

        //set spedd, panjie
        g_mdt.i2c_cbus_client->ext_flag |= I2C_HS_FLAG;
        //

        g_mdt.i2c_cbus_client->ext_flag |= I2C_DIRECTION_FLAG;
        status = i2c_master_send(g_mdt.i2c_cbus_client, (const char*)&reg_offset, 1);
        if (status < 0)
        {
            printk(" mdt_cbus_read_block,%d send error\n", __LINE__);
        }
        //set spedd, panjie
        g_mdt.i2c_cbus_client->ext_flag |= I2C_HS_FLAG;
        //


        status = i2c_master_recv(g_mdt.i2c_cbus_client, data, byte_count);

        g_mdt.i2c_cbus_client->addr = client_main_addr;
#endif




#ifdef I2C_ANALYZER
	//printk(KERN_INFO "R: 0xC8 %02X %02X %02X %02X ...\n", reg_offset, byte_count, data[0], data[1]);
	MHL_log_event(I2C_BLOCK_0xC9, reg_offset, data[0]);
#endif
}

static void mdt_cbus_write_block(unsigned char reg_offset, unsigned char byte_count, unsigned char *data)
{
#if 0
	unsigned char ret;
	unsigned char buffer[MDT_SCRATCHPAD_LENGTH+1];

	if (byte_count > MDT_SCRATCHPAD_LENGTH) 
		return;

	buffer[0] = reg_offset;

	memcpy( &buffer[1], data, byte_count);
	ret = i2c_master_send(g_mdt.i2c_cbus_client, buffer, (byte_count+1));
	if (ret < 0)
	{
		return;
	}

#ifdef I2C_ANALYZER
	MHL_log_event(I2C_BLOCK_0xC8, reg_offset, data[0]);
#endif
#else


	unsigned char ret;
	unsigned char buffer[MDT_SCRATCHPAD_LENGTH+1];

	if (byte_count > MDT_SCRATCHPAD_LENGTH) 
		return;

	buffer[0] = reg_offset;

	memcpy( &buffer[1], data, byte_count);

	//set spedd, panjie
	g_mdt.i2c_cbus_client->ext_flag |= I2C_HS_FLAG;
	g_mdt.i2c_cbus_client->ext_flag |= I2C_DIRECTION_FLAG;

	
	ret = i2c_master_send(g_mdt.i2c_cbus_client, buffer, (byte_count+1));
	if (ret < 0)
	{
		return;
	}

#ifdef I2C_ANALYZER
	MHL_log_event(I2C_BLOCK_0xC8, reg_offset, data[0]);
#endif





#endif

}

static void mdt_cbus_write_byte(unsigned char reg_offset, unsigned char value)
{
#if 0
	unsigned char ret;
	struct i2c_msg i2cMsg;

	unsigned char buffer[2] = {reg_offset, value};

	i2cMsg.addr = (MDT_CBUS_I2C_ADDRESS >> 1);
	i2cMsg.flags = 0;
	i2cMsg.len = 2;
 	i2cMsg.buf = buffer;



	if(g_mdt.i2c_cbus_client == 0)
	{
		return;
	}
	
	ret = i2c_transfer(g_mdt.i2c_cbus_client->adapter, &i2cMsg, 1);

	if (ret < 0)
	{
		return;
	}

#ifdef I2C_ANALYZER
	MHL_log_event(I2C_BLOCK_0xC8, reg_offset, value);
#endif

#else

	int32_t status;
	u32 client_main_addr = g_mdt.i2c_cbus_client->addr;
	g_mdt.i2c_cbus_client->addr = MDT_CBUS_I2C_ADDRESS >> 1;

	//set spedd, panjie
	g_mdt.i2c_cbus_client->ext_flag |= I2C_HS_FLAG;
	//

	g_mdt.i2c_cbus_client->ext_flag |= I2C_DIRECTION_FLAG;

	unsigned char buffer[2] = {reg_offset, value};
	
	status = i2c_master_send(g_mdt.i2c_cbus_client, buffer, 2);
	if (status < 0)
	{
		printk(" mdt_cbus_read_block,%d send error\n", __LINE__);
	}

	g_mdt.i2c_cbus_client->addr = client_main_addr;

#endif


}

static unsigned char mdt_cbus_read_byte(unsigned char reg_offset)
{
	unsigned char ret;
	mdt_cbus_read_block(reg_offset, 1, &ret);
	return ret;
	
}
#endif
// -------------------------------------------
// -------------------------------------------
// Changes beyond this point affect all MDT implementations.
// -------------------------------------------
// -------------------------------------------
unsigned char sii8338_parse_adopter_id (unsigned char *adopter_id)
{
	unsigned char adopter_id_h = *adopter_id;
	unsigned char adopter_id_l = *(adopter_id + 1);

	MHL_log_event(MDT_EVENT_PARSED, 0xD0, adopter_id_l);
	MHL_log_event(MDT_EVENT_PARSED, 0xD1, adopter_id_h);

	if ((adopter_id_h == 0) && (adopter_id_l == 0)) 
		return MDT_DRIVER_ACTIVE_MDT;
	else if ((adopter_id_h == SAMSUNG_ADOPTER_ID_H) && (adopter_id_l == SAMSUNG_ADOPTER_ID_L)) 
		return MDT_DRIVER_OFF;
	else 	return MDT_DRIVER_ACTIVE_SPIDER;
}

#if 0
static unsigned char parse_received_mdt_hp(union mdt_event_t *mdt_packet)
{

	// HP packets are 7 bytes
	mdt_cbus_read_block((MDT_TX_SCRATCHPAD_2 + MDT_MIN_PACKET_LENGTH), 
				MDT_KEYBOARD_PACKET_TAIL_LENGTH, (mdt_packet->bytes + MDT_MIN_PACKET_LENGTH));

	MHL_log_event(MDT_EVENT_PARSED, 0xA0, mdt_packet->bytes[0]);
	MHL_log_event(MDT_EVENT_PARSED, 0xA1, mdt_packet->bytes[1]);
	MHL_log_event(MDT_EVENT_PARSED, 0xA2, mdt_packet->bytes[2]);
	MHL_log_event(MDT_EVENT_PARSED, 0xA3, mdt_packet->bytes[3]);
	MHL_log_event(MDT_EVENT_PARSED, 0xA4, mdt_packet->bytes[4]);
	MHL_log_event(MDT_EVENT_PARSED, 0xA5, mdt_packet->bytes[5]);

	if (((mdt_packet->bytes[0]			!= M_CHAR) &&		// Confirm prefix
    		(mdt_packet->bytes[1] 			!= D_CHAR) &&
    		(mdt_packet->bytes[2] 			!= T_CHAR)) &&	  
		(mdt_packet->bytes[5]		 	!= MDT_VERSION))	// Confirm revision 1
		return PARSE_HP_ERROR;

	if (mdt_packet->bytes[3] == NOTICE_DEV_PLUG) {
		mdt_register_device(mdt_packet->bytes[4]);	// in the future, support response with ACK or NACK
	} else if (mdt_packet->bytes[3] == NOTICE_DEV_UNPLUG) {
		mdt_deregister_device(mdt_packet->bytes[4], 1);
	} else	return PARSE_HP_ERROR;

	return PARSE_OK;
}


static uint8_t sii8338_parse_received_burst_for_mdt(union mdt_event_t *mdt_packet, unsigned char scratchpad_offset)
{
	memset(mdt_packet->bytes, 0x0, MDT_SCRATCHPAD_LENGTH);

	mdt_cbus_read_block(scratchpad_offset, (unsigned char)MDT_MIN_PACKET_LENGTH, mdt_packet->bytes);

	if (mdt_packet->header.isHID == 0) 
		return PARSE_HID_ERROR;								// return error

	MHL_log_event(MDT_EVENT_PARSED, 0xB8, mdt_packet->bytes[0]);
	MHL_log_event(MDT_EVENT_PARSED, 0xB9, mdt_packet->bytes[1]);
	MHL_log_event(MDT_EVENT_PARSED, 0xBA, mdt_packet->bytes[2]);
	MHL_log_event(MDT_EVENT_PARSED, 0xBB, mdt_packet->bytes[3]);

	if ((mdt_packet->header.isKeyboard) || (mdt_packet->header.isNotLast)			//Full length packet; read the remaining 3 bytes
		|| (mdt_packet->event_mouse.header.isNotMouse)) {
		mdt_cbus_read_block(scratchpad_offset + MDT_MIN_PACKET_LENGTH, 
			MDT_KEYBOARD_PACKET_TAIL_LENGTH, (mdt_packet->bytes + MDT_MIN_PACKET_LENGTH));
	
		MHL_log_event(MDT_EVENT_PARSED, 0xBC, mdt_packet->bytes[4]);
		MHL_log_event(MDT_EVENT_PARSED, 0xBD, mdt_packet->bytes[5]);
		MHL_log_event(MDT_EVENT_PARSED, 0xBE, mdt_packet->bytes[6]);
	
	}

	MHL_log_event(MDT_EVENT_PARSED, 0xC8, mdt_packet->header.isKeyboard);
	MHL_log_event(MDT_EVENT_PARSED, 0xC9, mdt_packet->event_cursor.header.touch.isNotMouse);
	MHL_log_event(MDT_EVENT_PARSED, 0xCA, mdt_packet->event_cursor.body.suffix.isGame);

	if (mdt_packet->header.isKeyboard)  {
		mdt_generate_event_keyboard(&(mdt_packet->event_keyboard));	//Handle keyboard input       
	} else if (mdt_packet->event_cursor.header.touch.isNotMouse == 0) {	//Handle mouse input since isKeyboard == 0
		mdt_generate_event_mouse(	&(mdt_packet->event_mouse));	//	this is a relative coordinate event
	} else if (mdt_packet->event_cursor.body.suffix.isGame == 0) {		//Handle touchpad events since isNotMouse == 1
		mdt_generate_event_touchscreen( &(mdt_packet->event_cursor),1);//this is an absolute coordinate event
	} else {
		mdt_generate_event_gamepad(	&(mdt_packet->event_cursor));	//Gamepad events carry DPAD, buttons, & coordinates
	}
	return PARSE_OK;
}
#endif

/*
static void sii8338_msc_req_for_mdt(unsigned char req_type,
		unsigned char offset, unsigned char first_data)
{
	if ((offset != g_mdt.prior_msc_request.fields.offset) &&
		(first_data != g_mdt.prior_msc_request.fields.first_data)) 
	{
		g_mdt.prior_msc_request.fields.offset = offset;
		g_mdt.prior_msc_request.fields.first_data = first_data;
		mdt_cbus_write_block(MDT_CBUS_PRI_ADDR_CMD, 2, g_mdt.prior_msc_request.bytes);
	}
	else if (offset != g_mdt.prior_msc_request.fields.offset)
	{
		g_mdt.prior_msc_request.fields.offset = offset;
		mdt_cbus_write_byte(MDT_CBUS_PRI_ADDR_CMD, g_mdt.prior_msc_request.fields.offset);
	}
	else if (first_data != g_mdt.prior_msc_request.fields.first_data)
	{
		g_mdt.prior_msc_request.fields.first_data = first_data;
		mdt_cbus_write_byte(MDT_CBUS_PRI_WR_DATA_1ST, g_mdt.prior_msc_request.fields.first_data);
	}

		mdt_cbus_write_byte(MDT_CBUS_PRI_START, req_type);
}
*/
#if 0
uint8_t sii8338_irq_for_mdt(unsigned char *mdt_state) 
{
	uint8_t			ret	 = 0;
	uint8_t			intr	 = 0;
	uint8_t			scratchpad[16];				// update to accomodate Spider
	union mdt_event_t	*mdt_packet 	= (union mdt_event_t*)&(scratchpad[2]);	// 7 byte packet structure
	uint8_t			*adopter_id	= &(scratchpad[0]);	// 2 byte ID
	int i;


	MHL_log_event(ISR_MDT_BEGIN, 0, *mdt_state);

	if (g_mdt.is_ready == 0) 			//In the unlikely event someone forgot to call init.
	{
		printk(KERN_INFO "MDT ERROR:: ISR called before mdt_init completed.\n");
		return 0;
	}

	switch (*mdt_state) {
		case  WAIT_FOR_REQ_WRT:
			intr = mdt_cbus_read_byte(MDT_CBUS_INTR_REG_0);				// now, check for interrupt status
			MHL_log_event(ISR_WRITEBURST_CAUGHT, MDT_CBUS_INTR_REG_0, intr);
			if (intr & MDT_INT_REQ_WRT)  
			{									// ignore all but, REQ_WRT
				sii8338_msc_req_for_mdt(MDT_MSC_START_BIT_WRITE_REG,		// send GRT_WRT; make this call in case buff changed
					MDT_OFFSET_SET_INT, MDT_INT_GRT_WRT); 			// force retry through add'l handling
				mdt_cbus_write_byte(MDT_CBUS_INTR_REG_0,			// clear interrupt while GRT_WRT is pending
					(intr & (MDT_INT_REQ_WRT | MDT_INT_DSCR_CHG))); 	

				ret = MDT_EVENT_HANDLED;					// update state information
				*mdt_state = WAIT_FOR_GRT_WRT_COMPLETE;							
			}
			break;
		case WAIT_FOR_GRT_WRT_COMPLETE:
			intr = mdt_cbus_read_byte(MDT_CBUS_INTR_STATUS);
			MHL_log_event(ISR_WRITEBURST_CAUGHT, MDT_CBUS_INTR_STATUS, intr);

			if(intr & MDT_BIT_MSC_XFR_DONE)						 // check if GRT_WRT is complete
			{
				mdt_cbus_write_byte(MDT_CBUS_INTR_STATUS, MDT_BIT_MSC_XFR_DONE);

				ret = MDT_EVENT_HANDLED;
				*mdt_state = WAIT_FOR_WRITE_BURST_COMPLETE;
			}
			break;
		case WAIT_FOR_WRITE_BURST_COMPLETE:
			intr = mdt_cbus_read_byte(MDT_CBUS_MSC_INT2_STATUS);
			MHL_log_event(ISR_WRITEBURST_CAUGHT, MDT_CBUS_MSC_INT2_STATUS, intr);
			if (intr & MDT_BIT_RCVD_WRITE_BURST)
			{		
				#ifdef CONFIG_SPIDER_MHL
				if 	((g_mdt.is_ready & MDT_DRIVER_ACTIVE) == 0) {		// Confirm burst is intended for this recipient.
					memset((char *)adopter_id, 0x0, 2);			// Although it is appropriate to check every time
					mdt_cbus_read_block(MDT_TX_SCRATCHPAD_0, 2, adopter_id);//    per MHL specification, do this one time
					g_mdt.is_ready = sii8338_parse_adopter_id(adopter_id);	//    since MHL is point-to-point and subsequent
					if 	(g_mdt.is_ready == MDT_DRIVER_ACTIVE_SPIDER) {

						scratchpad[0] = SM_DEV_TYPE_MHL;		// Simulate a Spider connect event inside the phone.
						scratchpad[1] = 0;				// Mouse and keyboard hot-plug is simulated as 
						scratchpad[2] = 0;				//    needed when events arrive.
						scratchpad[3] = SM_DEV_STATE_CONNECTED;
						spider_handle_msg(&g_mdt.spider_data, (void *)&scratchpad, SPIDER_WRITE_BURST_MSG);

						*mdt_state = WAIT_FOR_WRITE_BURST_COMPLETE_FROM_SPIDER;
						return sii8338_irq_for_mdt(mdt_state);
					}
				}
				else if (g_mdt.is_ready != MDT_DRIVER_ACTIVE_MDT) return MDT_EVENT_HANDLED;
				#endif

		
				if (PARSE_HID_ERROR != sii8338_parse_received_burst_for_mdt(mdt_packet,MDT_TX_SCRATCHPAD_2)) {
					if (mdt_packet->header.isNotLast)			//support packet aggregation: 2 packets per burst
						sii8338_parse_received_burst_for_mdt(mdt_packet,
							MDT_TX_SCRATCHPAD_9);
				} else {
					parse_received_mdt_hp(mdt_packet);
				}
				mdt_cbus_write_byte(MDT_CBUS_MSC_INT2_STATUS, 			// clear the interrupt
							MDT_BIT_RCVD_WRITE_BURST);

				ret = MDT_EVENT_HANDLED;
				*mdt_state = WAIT_FOR_REQ_WRT;
			} else if (intr & MDT_BIT_RCVD_SET_INT)	{				//since reg is already read check if SET_INT asserted
				ret = MDT_EVENT_HANDLED;					//   if new request arrived before WRITE_BURST, 
				*mdt_state = WAIT_FOR_REQ_WRT;					//   assume that the prior burst was aborted and
			}									//   setup to handle WAIT_FOR_REQ_WRT in a new call
			break;
		case WAIT_FOR_WRITE_BURST_COMPLETE_FROM_SPIDER:
			intr = mdt_cbus_read_byte(MDT_CBUS_MSC_INT2_STATUS);
			MHL_log_event(ISR_WRITEBURST_CAUGHT, MDT_CBUS_MSC_INT2_STATUS, intr);
			if (intr & MDT_BIT_RCVD_WRITE_BURST)
			{		
				memset(scratchpad, 0x0, 16);
				mdt_cbus_read_block(MDT_TX_SCRATCHPAD_0, 8, scratchpad);
				mdt_cbus_read_block(MDT_TX_SCRATCHPAD_0+8, 8, scratchpad+8);
				spider_handle_msg(&g_mdt.spider_data, scratchpad, SPIDER_WRITE_BURST_MSG);
		
				mdt_cbus_write_byte(MDT_CBUS_MSC_INT2_STATUS, 			// clear the interrupt
							MDT_BIT_RCVD_WRITE_BURST);

				ret = MDT_EVENT_HANDLED;
				*mdt_state = WAIT_FOR_WRITE_BURST_COMPLETE_FROM_SPIDER;		//spider is not compliant and doesn't support SET_INT
			}
			break;

		case WAIT_FOR_REQ_WRT_COMPLETE:							//undefined for now			
		case WAIT_FOR_GRT_WRT:


		case WAIT_FOR_WRITE_BURST_SENT:
		case MDT_IDLE:									//state machine is currently disabled
		default:									//... for performance, better to put
			return 0;								//    these conditions at the end of switch

	}

	MHL_log_event(ISR_MDT_END, 0xFF, 0xFF);

	return ret;
}
#endif
uint8_t mdt_state=WAIT_FOR_WRITE_BURST_COMPLETE;

uint8_t sii8338_irq_for_mdt(uint8_t *mdt_buf) 
{
	uint8_t			ret	 = 0;
	uint8_t			scratchpad[16];				// update to accomodate Spider
	uint8_t			*adopter_id	= &(scratchpad[0]);	// 2 byte ID
	//int i;


	//MHL_log_event(ISR_MDT_BEGIN, 0, mdt_state);

	if (g_mdt.is_ready == 0) 			//In the unlikely event someone forgot to call init.
	{
		printk(KERN_INFO "MDT ERROR:: ISR called before mdt_init completed.\n");
		return 0;
	}

	switch (mdt_state) {
		
		case WAIT_FOR_WRITE_BURST_COMPLETE:
			//printk(KERN_ERR "WAIT_FOR_WRITE_BURST_COMPLETE。。,g_mdt.is_ready=0x%x...\n",g_mdt.is_ready);
			
			#ifdef CONFIG_SPIDER_MHL
			//if 	((g_mdt.is_ready & MDT_DRIVER_ACTIVE) == 0) 
				{		// Confirm burst is intended for this recipient.
				memset((char *)adopter_id, 0x0, 2);			// Although it is appropriate to check every time
				memset(scratchpad, 0x0, 16);
				memcpy(adopter_id, mdt_buf, 2);
				
					//printk(KERN_ERR "adopter_id:\n");
				//for(i=0;i<2;i++)
				//	printk(KERN_ERR "0x%x ",*(mdt_buf+i));
				g_mdt.is_ready = sii8338_parse_adopter_id(adopter_id);	//    since MHL is point-to-point and subsequent
				if 	(g_mdt.is_ready == MDT_DRIVER_ACTIVE_SPIDER) {
					printk(KERN_ERR "MDT_DRIVER_ACTIVE_SPIDER:\n");
					scratchpad[0] = SM_DEV_TYPE_MHL;		// Simulate a Spider connect event inside the phone.
					scratchpad[1] = 0;				// Mouse and keyboard hot-plug is simulated as 
					scratchpad[2] = 0;				//    needed when events arrive.
					scratchpad[3] = SM_DEV_STATE_CONNECTED;
					spider_handle_msg(&g_mdt.spider_data, (void *)&scratchpad, SPIDER_WRITE_BURST_MSG);

					mdt_state = WAIT_FOR_WRITE_BURST_COMPLETE_FROM_SPIDER;
				}
			}
			#endif
			
			break;
		case WAIT_FOR_WRITE_BURST_COMPLETE_FROM_SPIDER:
			//printk(KERN_ERR "WAIT_FOR_WRITE_BURST_COMPLETE_FROM_SPIDER。。。\n");
				
			//memset(scratchpad, 0x0, 16);
			memcpy(scratchpad,mdt_buf, 16);
			//printk(KERN_ERR "scratchpad:\n");
			//	for(i=0;i<16;i++)
			//		printk(KERN_ERR "0x%x ",*(scratchpad+i));
			spider_handle_msg(&g_mdt.spider_data, scratchpad, SPIDER_WRITE_BURST_MSG);

			ret = MDT_EVENT_HANDLED;
			mdt_state = WAIT_FOR_WRITE_BURST_COMPLETE_FROM_SPIDER;		//spider is not compliant and doesn't support SET_INT
		
			break;								
		default:	
			printk(KERN_ERR "default。。\n");//... for performance, better to put
			return 0;								//    these conditions at the end of switch

	}

	//MHL_log_event(ISR_MDT_END, 0xFF, 0xFF);

	return ret;
}


void mdt_init(void) {

	if (g_mdt.is_ready != 0 ) return;

	printk(KERN_INFO "MDT INIT started\n");
	mdt_state=WAIT_FOR_WRITE_BURST_COMPLETE;

	/*if (client == 0) {
		printk(KERN_INFO "MDT ERROR :: INIT called with NULL i2c client\n");
		return;
	}*/

#ifdef MDT_SUPPORT_DEBUG
	init_log_file();
#endif
	//printk(KERN_INFO "MDT INIT started111111111111\n");

	//g_mdt.i2c_cbus_client = client;

	mdt_input_init();
	//printk(KERN_INFO "MDT INIT started222222222\n");

	g_mdt.is_ready = 1;

	//mdt_reset();

// [START] HELIXTECH: KT_SPIDER_FEATURE ====================================
#ifdef CONFIG_SPIDER_MHL
	if (NULL == &(g_mdt.spider_data)) {
		pr_err("spider: %s: spider_init failed@@@@\n", __func__);
		return;
	}
	if (0 > spider_init(&(g_mdt.spider_data)))
		printk(KERN_INFO "SPIDER FAILED INIT\n");	
	//printk(KERN_INFO "MDT INIT started33333333333\n");

#endif	/* CONFIG_SPIDER_MHL */
// [END] HELIXTECH: KT_SPIDER_FEATURE ======================================

	printk(KERN_INFO "MDT INIT done\n");
}

void mdt_deregister(void) {
	if (g_mdt.is_ready == 0 ) return;

	printk(KERN_INFO "MDT DEREGISTER started\n");

	//g_mdt.i2c_cbus_client = 0;

	// add code to deregister log file
#ifdef MDT_SUPPORT_DEBUG
	deregster_log_file();
#endif
	mdt_input_deregister();

	#ifdef CONFIG_SPIDER_MHL
	spidermouse_disable(SM_DEV_TYPE_MOUSE);
	#endif

	g_mdt.is_ready = 0;
	printk(KERN_INFO "MDT DEREGISTER done\n");
}
#if 0
void mdt_reset(void) {
	unsigned char buffer;	

       if ( g_mdt.is_ready == 0 ) {
	   printk(KERN_INFO "MDT ERROR: RESET called before init.\n");
	   return;
	}

	//Interrupts are activated on behalf of the driver.
	buffer = mdt_cbus_read_byte(MDT_CBUS_MSC_INT2_STATUS_MASK);
	if (!(buffer & MDT_BIT_RCVD_WRITE_BURST)) {
		printk(KERN_INFO "MDT WARNING :: CBUS_INT_STATUS_2 mask was not set for MDT. Corrected. Chance that MDT ISR may not get called.\n");
		buffer |= MDT_BIT_RCVD_WRITE_BURST;
		mdt_cbus_write_byte(MDT_CBUS_MSC_INT2_STATUS_MASK, buffer);
	}

	buffer = mdt_cbus_read_byte(MDT_CBUS_INTR_REG_0_MASK);
	if (!(buffer & (MDT_INT_REQ_WRT))) {
		printk(KERN_INFO "MDT WARNING :: MHL_INTR_EN mask was not set for MDT. Corrected. Chance that MDT ISR may not get called.\n");
		buffer |= MDT_INT_REQ_WRT;
		mdt_cbus_write_byte(MDT_CBUS_INTR_REG_0_MASK, buffer);
	}

	buffer = mdt_cbus_read_byte(MDT_CBUS_INTR_STATUS_MASK);
	if ((buffer & (MDT_MSC_REQ_DONE_MASK | MDT_MSC_MSG_RECD_MASK))	!= (MDT_MSC_REQ_DONE_MASK | MDT_MSC_MSG_RECD_MASK)) {
		printk(KERN_INFO "MDT WARNING :: CBUS_INT_1 mask was not set for MDT. Corrected. Chance that MDT ISR may not get called.\n");
		buffer |= (MDT_MSC_REQ_DONE_MASK | MDT_MSC_MSG_RECD_MASK);
		mdt_cbus_write_byte(MDT_CBUS_INTR_STATUS_MASK, buffer);
	}

	memset(g_mdt.prior_msc_request.bytes, 0, 2);
	printk(KERN_INFO "MDT RESET\n");
}
#endif
void mdt_update_msc_send_buffer(unsigned char cbus_reg_13, unsigned char cbus_reg_14)
{
	g_mdt.prior_msc_request.fields.offset = cbus_reg_13;
	g_mdt.prior_msc_request.fields.first_data = cbus_reg_14;

}

#ifdef MDT_SUPPORT_DEBUG

static void init_events(void)
{
	int wevent;
	g_mdt.e = g_mdt.events;
	for (wevent = (int)0; wevent < (int)ARRAY_SIZE(g_mdt.events); wevent++)
		g_mdt.events[wevent].type = (int)-1;
}
#if 0
void MHL_log_event(int type, int code, int value)
{
	struct input_event *e 	 = g_mdt.e;

	if (g_mdt.is_ready == 0)
		return;

	do_gettimeofday(&(e->time));
	e->type = type;
	e->code = code;
	e->value = value;

	if ((++e - g_mdt.events) >= ARRAY_SIZE(g_mdt.events))
		e = g_mdt.events;
	e->type = -1;
	g_mdt.e = e;
}
#else
void MHL_log_event(int type, int code, int value)
{
printk(KERN_INFO KERN_INFO "MDT:type: %x ,code:%x, value:%x.\n", type, code, value);
}
#endif

struct input_event *for_each_valid_event(struct input_event *e)
{
	struct input_event *estart = e++;

	if ((e - g_mdt.events) >= ARRAY_SIZE(g_mdt.events))
		e = g_mdt.events;

	while (e->type == (__u16)-1 && e != estart) {
		e++;
		if ((e - g_mdt.events) >= ARRAY_SIZE(g_mdt.events))
			e = g_mdt.events;
	}

	if (e == estart || e->type == (__u16)-1)
		return NULL;
	return e;
}
#endif

//This part always compiles to support conditional debug control 
#ifdef MDT_SUPPORT_DEBUG
static ssize_t show_events(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct input_event *e = g_mdt.e;
	int ret = 0;

	if (g_mdt.is_ready == 0)
		return 0;

	while ((e = for_each_valid_event(e)) != NULL) {
		ret += sprintf(buf + ret, "%8lu.%06lu,",
			       e->time.tv_sec, e->time.tv_usec);
		switch (e->type) {
			 case IRQ_HEARTBEAT:
				ret += sprintf(buf + ret, "%s", "IRQ_HEARTBEAT,");
				break;
 			 case IRQ_WAKE:
				ret += sprintf(buf + ret, "%s", "IRQ_WAKE,");
				break;
 			 case IRQ_RECEIVED:
				ret += sprintf(buf + ret, "%s", "IRQ_RECEIVED,");
				break;
			 case ISR_WRITEBURST_CAUGHT:
				ret += sprintf(buf + ret, "%s", "ISR_WRITEBURST_CAUGHT,");
				break;
			 case ISR_WRITEBURST_MISSED:
				ret += sprintf(buf + ret, "%s", "ISR_WRITEBURST_MISSED,");
				break;

			 case ISR_DEFFER_SCHEDULED:
				ret += sprintf(buf + ret, "%s", "ISR_DEFFER_SCHEDULED,");
				break;
			 case ISR_DEFFER_NO:
				ret += sprintf(buf + ret, "%s", "ISR_DEFFER_NO,");
				break;
			 case ISR_DEFFER_BEGIN:
				ret += sprintf(buf + ret, "%s", "ISR_DEFFER_BEGIN,");
				break;
			 case ISR_DEFFER_END:
				ret += sprintf(buf + ret, "%s", "ISR_DEFFER_END,");
				break;
			 case I2C_BLOCK_R_UNKNOWN:
				ret += sprintf(buf + ret, "%s", "I2C_BLOCK_UNKNOWN_R,");
				break;
			 case I2C_BLOCK_W_UNKNOWN:
				ret += sprintf(buf + ret, "%s", "I2C_BLOCK_UNKNOWN_W,");
				break;
			 case I2C_BLOCK_0x72:
				ret += sprintf(buf + ret, "%s", "I2C_BLOCK_0x72,");
				break;
			 case I2C_BLOCK_0x7A:
				ret += sprintf(buf + ret, "%s", "I2C_BLOCK_0x7A,");
				break;
			 case I2C_BLOCK_0x92:
				ret += sprintf(buf + ret, "%s", "I2C_BLOCK_0x93,");
				break;
			 case I2C_BLOCK_0xC8:
				ret += sprintf(buf + ret, "%s",  "I2C_BLOCK_0xC8,");
				break;
			 case I2C_BLOCK_0x73:
				ret += sprintf(buf + ret, "%s", "I2C_BLOCK_0x73,");
				break;
			 case I2C_BLOCK_0x7B:
				ret += sprintf(buf + ret, "%s",  "I2C_BLOCK_0x7B,");
				break;
			 case I2C_BLOCK_0x93:
				ret += sprintf(buf + ret, "%s",  "I2C_BLOCK_0x93,");
				break;
			 case I2C_BLOCK_0xC9:
				ret += sprintf(buf + ret, "%s", "I2C_BLOCK_0xC9,");
				break;
			 case I2C_BYTE_0x72:
				ret += sprintf(buf + ret, "%s", "I2C_BYTE_0x72,");
				break;
			 case I2C_BYTE_0x7A:
				ret += sprintf(buf + ret, "%s", "I2C_BYTE_0x7A,");
				break;
			 case I2C_BYTE_0x92:
				ret += sprintf(buf + ret, "%s", "I2C_BYTE_0x93,");
				break;
			 case I2C_BYTE_0xC8:
				ret += sprintf(buf + ret, "%s", "I2C_BYTE_0xC8,");
				break;
			 case I2C_BYTE_0x73:
				ret += sprintf(buf + ret, "%s", "I2C_BYTE_0x73,");
				break;
			 case I2C_BYTE_0x7B:
				ret += sprintf(buf + ret, "%s", "I2C_BYTE_0x7B,");
				break;
			 case I2C_BYTE_0x93:
				ret += sprintf(buf + ret, "%s",  "I2C_BYTE_0x93,");
				break;
			 case I2C_BYTE_0xC9:
				ret += sprintf(buf + ret, "%s", "I2C_BYTE_0xC9,");
				break;
	 		 case ISR_THREADED_BEGIN:
				ret += sprintf(buf + ret, "%s", "ISR_THREADED_BEGIN,");
				break;
			 case ISR_THREADED_END:
				ret += sprintf(buf + ret, "%s", "ISR_THREADED_END,");
				break;
			 case ISR_MDT_BEGIN:
				ret += sprintf(buf + ret, "%s", "ISR_MDT_BEGIN,");
				break;
			 case ISR_MDT_END:
				ret += sprintf(buf + ret, "%s", "ISR_MDT_END,");
				break;
			 case ISR_FULL_BEGIN:
				ret += sprintf(buf + ret, "%s", "ISR_FULL_BEGIN,");
				break;
			 case ISR_FULL_END:
				ret += sprintf(buf + ret, "%s", "ISR_FULL_END,");
				break;
			 case TOUCHPAD_BEGIN:
				ret += sprintf(buf + ret, "%s", "TOUCHPAD_BEGIN,");
				break;
			 case TOUCHPAD_END:
				ret += sprintf(buf + ret, "%s", "TOUCHPAD_END,");
				break;
			 case MHL_ESTABLISHED:
				ret += sprintf(buf + ret, "%s", "MHL_ESTABLISHED,");
				break;
			 case MSC_READY:
				ret += sprintf(buf + ret, "%s", "MSC_READY,");
				break;
			 case MDT_EVENT_PARSED:
				ret += sprintf(buf + ret, "%s", "MDT_EVENT_PARSED,");
				break;
			 case MDT_UNLOCK:
				ret += sprintf(buf + ret, "%s", "MDT_UNLOCK,");
				break;
			 case MDT_LOCK:
				ret += sprintf(buf + ret, "%s", "MDT_LOCK,");
				break;
 			 case MDT_DISCOVER_REQ:
				ret += sprintf(buf + ret, "%s", "MDT_DISCOVERY_REQ,");
				break;
		}

		ret += sprintf(buf + ret, "%02x,%02x,%02x\n",
			       e->type, e->code, e->value);
		e->type = -1;

		if (ret > (PAGE_SIZE - 512))
			return ret;
	}

	return ret;
}
static DEVICE_ATTR(MDT_file, S_IRUGO, show_events, NULL);

static void deregster_log_file(void)
{
	if (g_mdt.is_ready 	== 0) return;
	if (g_mdt.class 	== 0) return;
	if (g_mdt.dev		== 0) return;

	device_destroy(g_mdt.class,0);
	class_destroy(g_mdt.class);

	g_mdt.class = 0;
	g_mdt.dev   = 0;
}

static void init_log_file(void) 
{
	if (g_mdt.is_ready 	!= 0) return;
	if (g_mdt.class 	!= 0) return;
	if (g_mdt.dev		!= 0) return;

	init_events();

	g_mdt.is_ready = 1;

	g_mdt.class = class_create(THIS_MODULE, "mdt");
	if (IS_ERR(g_mdt.class)) {
		printk(KERN_INFO "MDT ERR: Failed to create debug helper class.\n");
		return;
	}

	g_mdt.dev = device_create(g_mdt.class, NULL, 0, NULL, "mdt_debug_dev");

	if (IS_ERR(g_mdt.dev)) {
		printk(KERN_INFO "MDT ERR: Failed to create debug helper device.\n");	
		return;
	}	

	if (device_create_file(g_mdt.dev, &dev_attr_MDT_file) <0) {
		printk(KERN_INFO "MDT ERR: Failed to create debug helper device file.\n");
		return;
	}
	if (sysfs_create_group(&(g_mdt.dev->kobj), &mdt_attr_group) != 0)
	{
		printk(KERN_INFO "MDT ERR: Failed to create attributes group.\n");
		return;
	}

}


#else
void MHL_log_event(int type, int code, int value)
{
printk(KERN_ERR "MDT:type: %x ,code:%x , value:%x.\n", type, code, value);
//TX_DEBUG_PRINT(("Drv: MDT:type: %x ,code:%x , value:%x\n", type, code, value));
}
//static void init_log_file(void) {}
//static void deregster_log_file(void){}

#endif



