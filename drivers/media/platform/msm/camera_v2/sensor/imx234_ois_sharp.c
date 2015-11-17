/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 #if 1
//#include <mach/gpiomux.h>
#include "msm_sensor.h"
#include "msm_sd.h"
//#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
//#include "msm_camera_i2c_mux.h"
//#include <mach/rpm-regulator.h>
//#include <mach/rpm-regulator-smd.h>
#include <linux/regulator/consumer.h>
#include "../../../../../../video/msm/mdss/mdss_fb.h"
//#include "ois_lc898122_sharp/msm_ois_lc898122_sharp.h"

#endif

extern struct msm_fb_data_type *zte_camera_mfd;

#define IMX234_OIS_SHARP_SENSOR_NAME "imx234_ois_sharp"
DEFINE_MSM_MUTEX(imx234_ois_sharp_mut);
#define CONFIG_OIS_DEBUG
#ifdef CONFIG_OIS_DEBUG
#define CDBG_OIS(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG_OIS(fmt, args...) do { } while (0)
#endif
//static void zte_camera_backlight(struct msm_fb_data_type *mfd, u32 bkl_lvl);

extern void read_ois_byte_data_lc898122_sharp(unsigned short reg_addr, unsigned char *read_data_8);
extern void read_ois_word_data_lc898122_sharp(unsigned short reg_addr, uint16_t *read_data_16);
#if 1
extern void RegReadA(unsigned short reg_addr, unsigned char *read_data_8);
extern void RegWriteA(unsigned short reg_addr, unsigned char write_data_8);
extern void RamReadA(unsigned short ram_addr, void *read_data_16);
extern void RamWriteA(unsigned short ram_addr, unsigned short write_data_16);
extern void RamRead32A(unsigned short ram_addr, void *read_data_32);
extern void RamWrite32A(unsigned short ram_addr, uint32_t write_data_32);
#endif
extern unsigned char RtnCen(unsigned char	UcCmdPar);
extern void SetPanTiltMode(unsigned char UcPnTmod);

extern void OisEna(void);
extern void GyrCon( unsigned char UcGyrCon );  //ZTEMT-wangqiaoming::modify OIS drift problem
extern void	IniSet( void );
extern void	SrvCon( unsigned char	UcDirSel, unsigned char	UcSwcCon );
extern void	S2cPro( unsigned char uc_mode );
extern void msm_ois_init_cci_lc898122_sharp(void);
extern void msm_ois_release_cci_lc898122_sharp(void);

extern void RamAccFixMod( unsigned char UcAccMod );
extern void IniSetAf( void );
extern void	SetH1cMod( unsigned char	UcSetNum );
//extern void    SetTregAf_lc898122_sharp( unsigned short UsTregAf );
                   
unsigned char read_otp_ready_flag_lc898122_sharp=0;

/* 16bits RAM */
unsigned short  hall_offset_x_lc898122_sharp=0; 			   
unsigned short  hall_offset_y_lc898122_sharp=0; 
unsigned short  hall_bias_x_lc898122_sharp=0; 
unsigned short  hall_bias_y_lc898122_sharp=0; 
unsigned short  hall_ad_offset_x_lc898122_sharp=0; 
unsigned short  hall_ad_offset_y_lc898122_sharp=0; 
unsigned short  loop_gain_x_lc898122_sharp=0; 
unsigned short  loop_gain_y_lc898122_sharp=0; 

unsigned char  hall_offset_x_lc898122_lsb_sharp=0; 			   
unsigned char  hall_offset_y_lc898122_lsb_sharp=0; 
unsigned char  hall_bias_x_lc898122_lsb_sharp=0; 
unsigned char  hall_bias_y_lc898122_lsb_sharp=0; 
unsigned char  hall_ad_offset_x_lc898122_lsb_sharp=0; 
unsigned char  hall_ad_offset_y_lc898122_lsb_sharp=0; 
unsigned char  loop_gain_x_lc898122_lsb_sharp=0; 
unsigned char  loop_gain_y_lc898122_lsb_sharp=0; 

unsigned char  hall_offset_x_lc898122_msb_sharp=0; 			   
unsigned char  hall_offset_y_lc898122_msb_sharp=0; 
unsigned char  hall_bias_x_lc898122_msb_sharp=0; 
unsigned char  hall_bias_y_lc898122_msb_sharp=0; 
unsigned char  hall_ad_offset_x_lc898122_msb_sharp=0; 
unsigned char  hall_ad_offset_y_lc898122_msb_sharp=0; 
unsigned char  loop_gain_x_lc898122_msb_sharp=0; 
unsigned char  loop_gain_y_lc898122_msb_sharp=0; 

/* 8bits Register */
unsigned char gyro_offset_x_msb_lc898122_sharp=0;
unsigned char gyro_offset_x_lsb_lc898122_sharp=0;			   
unsigned char gyro_offset_y_msb_lc898122_sharp=0;
unsigned char gyro_offset_y_lsb_lc898122_sharp=0;

/* 32bits RAM */ 
uint32_t  gyro_gain_x_lc898122_sharp=0;
unsigned char  gyro_gain_x_31_24_lc898122_sharp=0;
unsigned char  gyro_gain_x_23_16_lc898122_sharp=0;
unsigned char  gyro_gain_x_15_8_lc898122_sharp=0;
unsigned char  gyro_gain_x_7_0_lc898122_sharp=0;
uint32_t  gyro_gain_y_lc898122_sharp=0;
unsigned char  gyro_gain_y_31_24_lc898122_sharp=0;
unsigned char  gyro_gain_y_23_16_lc898122_sharp=0;
unsigned char  gyro_gain_y_15_8_lc898122_sharp=0;
unsigned char  gyro_gain_y_7_0_lc898122_sharp=0;

/* 8bits Register */
unsigned char osc_value_lc898122_sharp=1;
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/

unsigned short af_start_value_lc898122_lsb_sharp = 0;
unsigned short af_infinity_value_lc898122_lsb_sharp = 0;
unsigned short af_macro_value_lc898122_lsb_sharp = 0;

unsigned short af_start_value_lc898122_msb_sharp = 0;
unsigned short af_infinity_value_lc898122_msb_sharp = 0;
unsigned short af_macro_value_lc898122_msb_sharp = 0;

/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/

#define OTP_PAGE_ADDR_LC898122_SHARP			0x3B02
#define	OTP_READ_MODE_ADDR_LC898122_SHARP		0x3B00
#define	OTP_READ_READY_ADDR_LC898122_SHARP		0x3B01
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/
#define AF_START_CURRENT_LC898122_LSB_SHARP        0x18
#define AF_START_CURRENT_LC898122_MSB_SHARP        0x19

#define AF_START_INFINITY_LC898122_LSB_SHARP       0x14
#define AF_START_INFINITY_LC898122_MSB_SHARP       0x15

#define AF_START_MACRO_LC898122_LSB_SHARP          0x12
#define AF_START_MACRO_LC898122_MSB_SHARP          0x13
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/


#define HALL_OFFSET_X_ADDR_LC898122_LSB_SHARP		0x26
#define HALL_OFFSET_X_ADDR_LC898122_MSB_SHARP		0x27

#define HALL_OFFSET_Y_ADDR_LC898122_LSB_SHARP		0x28
#define HALL_OFFSET_Y_ADDR_LC898122_MSB_SHARP		0x29

#define HALL_BIAS_X_ADDR_LC898122_LSB_SHARP		0x2A
#define HALL_BIAS_X_ADDR_LC898122_MSB_SHARP		0x2B

#define HALL_BIAS_Y_ADDR_LC898122_LSB_SHARP		0x2C
#define HALL_BIAS_Y_ADDR_LC898122_MSB_SHARP		0x2D

#define HALL_AD_OFFSET_X_ADDR_LC898122_LSB_SHARP	0x2E
#define HALL_AD_OFFSET_X_ADDR_LC898122_MSB_SHARP	0x2F

#define HALL_AD_OFFSET_Y_ADDR_LC898122_LSB_SHARP	0x30
#define HALL_AD_OFFSET_Y_ADDR_LC898122_MSB_SHARP	0x31

#define LOOP_GAIN_X_ADDR_LC898122_LSB_SHARP		0x32
#define LOOP_GAIN_X_ADDR_LC898122_MSB_SHARP		0x33

#define LOOP_GAIN_Y_ADDR_LC898122_LSB_SHARP		0x34
#define LOOP_GAIN_Y_ADDR_LC898122_MSB_SHARP		0x35

#define	GYRO_OFFSET_X_MSB_ADDR_LC898122_SHARP	0x3B
#define	GYRO_OFFSET_X_LSB_ADDR_LC898122_SHARP	0x3A

#define	GYRO_OFFSET_Y_MSB_ADDR_LC898122_SHARP	0x3D
#define	GYRO_OFFSET_Y_LSB_ADDR_LC898122_SHARP	0x3C

#define GYRO_GAIN_X_31_24_ADDR_LC898122_SHARP	0x42
#define GYRO_GAIN_X_23_16_ADDR_LC898122_SHARP	0x41
#define GYRO_GAIN_X_15_8_ADDR_LC898122_SHARP	0x40
#define GYRO_GAIN_X_7_0_ADDR_LC898122_SHARP	0x3F

#define GYRO_GAIN_Y_31_24_ADDR_LC898122_SHARP	0x46
#define GYRO_GAIN_Y_23_16_ADDR_LC898122_SHARP	0x45
#define GYRO_GAIN_Y_15_8_ADDR_LC898122_SHARP	0x44
#define GYRO_GAIN_Y_7_0_ADDR_LC898122_SHARP	0x43

#define OSC_VALUE_ADDR_LC898122_SHARP			0x3E
extern unsigned short af_start_value_lc898122;
extern unsigned short af_infinity_value_lc898122;
extern unsigned short af_macro_value_lc898122 ;
extern unsigned short af_start_value_lc898122_sharp;
extern unsigned short af_infinity_value_lc898122_sharp;
extern unsigned short af_macro_value_lc898122_sharp ;
 //void imx234_ois_otp_ois_sharp(struct work_struct *work)

 void imx234_ois_otp_ois_sharp(void)
{
	//struct msm_sensor_ctrl_t *s_ctrl = container_of(to_delayed_work(work),
	//				struct msm_sensor_ctrl_t, zte_otp_worker);
	static int ois_otp_lc898122_flag_sharp = 0;
	//mutex_lock(&s_ctrl->zte_otp_mutex);
	unsigned char read_data;
	//printk(" enter imx234_ois_otp_ois_sharp \n");
	msm_ois_init_cci_lc898122_sharp();
	
	IniSetAf();
       
	IniSet();
       

	if (ois_otp_lc898122_flag_sharp == 0)
	{
		read_ois_byte_data_lc898122_sharp(HALL_OFFSET_X_ADDR_LC898122_LSB_SHARP,(unsigned char *)&hall_offset_x_lc898122_lsb_sharp);
	       CDBG_OIS("hall_offset_x_lc898122_lsb = 0x%x\n",hall_offset_x_lc898122_lsb_sharp);
		read_ois_byte_data_lc898122_sharp(HALL_OFFSET_X_ADDR_LC898122_MSB_SHARP,(unsigned char *)&hall_offset_x_lc898122_msb_sharp);
	       CDBG_OIS("hall_offset_x_lc898122_msb = 0x%x\n",hall_offset_x_lc898122_msb_sharp);
       
		read_ois_byte_data_lc898122_sharp(HALL_OFFSET_Y_ADDR_LC898122_LSB_SHARP,(unsigned char *)&hall_offset_y_lc898122_lsb_sharp);
	       CDBG_OIS("hall_offset_y_lc898122_lsb = 0x%x\n",hall_offset_y_lc898122_lsb_sharp);
		read_ois_byte_data_lc898122_sharp(HALL_OFFSET_Y_ADDR_LC898122_MSB_SHARP,(unsigned char *)&hall_offset_y_lc898122_msb_sharp);
	       CDBG_OIS("hall_offset_y_lc898122_msb = 0x%x\n",hall_offset_y_lc898122_msb_sharp);
		   
		read_ois_byte_data_lc898122_sharp(HALL_BIAS_X_ADDR_LC898122_LSB_SHARP,(unsigned char *)&hall_bias_x_lc898122_lsb_sharp);
	       CDBG_OIS("hall_bias_x_lc898122_lsb = 0x%x\n",hall_bias_x_lc898122_lsb_sharp);
		read_ois_byte_data_lc898122_sharp(HALL_BIAS_X_ADDR_LC898122_MSB_SHARP,(unsigned char *)&hall_bias_x_lc898122_msb_sharp);
	       CDBG_OIS("hall_bias_x_lc898122_msb = 0x%x\n",hall_bias_x_lc898122_msb_sharp);
		   
		read_ois_byte_data_lc898122_sharp(HALL_BIAS_Y_ADDR_LC898122_LSB_SHARP,(unsigned char *)&hall_bias_y_lc898122_lsb_sharp);
	       CDBG_OIS("hall_bias_y_lc898122_lsb = 0x%x\n",hall_bias_y_lc898122_lsb_sharp);
		read_ois_byte_data_lc898122_sharp(HALL_BIAS_Y_ADDR_LC898122_MSB_SHARP,(unsigned char *)&hall_bias_y_lc898122_msb_sharp);
	       CDBG_OIS("hall_bias_y_lc898122_msb = 0x%x\n",hall_bias_y_lc898122_msb_sharp);
		   
		read_ois_byte_data_lc898122_sharp(HALL_AD_OFFSET_X_ADDR_LC898122_LSB_SHARP,(unsigned char *)&hall_ad_offset_x_lc898122_lsb_sharp);
	       CDBG_OIS("hall_ad_offset_x_lc898122_lsb = 0x%x\n",hall_ad_offset_x_lc898122_lsb_sharp);
		read_ois_byte_data_lc898122_sharp(HALL_AD_OFFSET_X_ADDR_LC898122_MSB_SHARP,(unsigned char *)&hall_ad_offset_x_lc898122_msb_sharp);
	       CDBG_OIS("hall_ad_offset_x_lc898122_msb = 0x%x\n",hall_ad_offset_x_lc898122_msb_sharp);
		   
		read_ois_byte_data_lc898122_sharp(HALL_AD_OFFSET_Y_ADDR_LC898122_LSB_SHARP,(unsigned char *)&hall_ad_offset_y_lc898122_lsb_sharp);
	       CDBG_OIS("hall_ad_offset_y_lc898122_lsb = 0x%x\n",hall_ad_offset_y_lc898122_lsb_sharp);
		read_ois_byte_data_lc898122_sharp(HALL_AD_OFFSET_Y_ADDR_LC898122_MSB_SHARP,(unsigned char *)&hall_ad_offset_y_lc898122_msb_sharp);
	       CDBG_OIS("hall_ad_offset_y_lc898122_msb = 0x%x\n",hall_ad_offset_y_lc898122_msb_sharp);
		   
		read_ois_byte_data_lc898122_sharp(LOOP_GAIN_X_ADDR_LC898122_LSB_SHARP,(unsigned char *)&loop_gain_x_lc898122_lsb_sharp);
	       CDBG_OIS("loop_gain_x_lc898122_lsb= 0x%x\n",loop_gain_x_lc898122_lsb_sharp);
		read_ois_byte_data_lc898122_sharp(LOOP_GAIN_X_ADDR_LC898122_MSB_SHARP,(unsigned char *)&loop_gain_x_lc898122_msb_sharp);
	       CDBG_OIS("loop_gain_x_lc898122_msb = 0x%x\n",loop_gain_x_lc898122_msb_sharp);
		   
		read_ois_byte_data_lc898122_sharp(LOOP_GAIN_Y_ADDR_LC898122_LSB_SHARP,(unsigned char *)&loop_gain_y_lc898122_lsb_sharp);
		CDBG_OIS("loop_gain_y_lc898122_lsb = 0x%x\n",loop_gain_y_lc898122_lsb_sharp);
              read_ois_byte_data_lc898122_sharp(LOOP_GAIN_Y_ADDR_LC898122_MSB_SHARP,(unsigned char *)&loop_gain_y_lc898122_msb_sharp);
		CDBG_OIS("loop_gain_y_lc898122_msb = 0x%x\n",loop_gain_y_lc898122_msb_sharp);
		
              hall_offset_x_lc898122_sharp = hall_offset_x_lc898122_msb_sharp <<8 | hall_offset_x_lc898122_lsb_sharp;
	       hall_offset_y_lc898122_sharp =hall_offset_y_lc898122_msb_sharp <<8 | hall_offset_y_lc898122_lsb_sharp;
		hall_bias_x_lc898122_sharp = hall_bias_x_lc898122_msb_sharp <<8 | hall_bias_x_lc898122_lsb_sharp;
		hall_bias_y_lc898122_sharp = hall_bias_y_lc898122_msb_sharp << 8 | hall_bias_y_lc898122_lsb_sharp;
		hall_ad_offset_x_lc898122_sharp = hall_ad_offset_x_lc898122_msb_sharp << 8 | hall_ad_offset_x_lc898122_lsb_sharp;
		hall_ad_offset_y_lc898122_sharp = hall_ad_offset_y_lc898122_msb_sharp << 8 | hall_ad_offset_y_lc898122_lsb_sharp;
		loop_gain_x_lc898122_sharp = loop_gain_x_lc898122_msb_sharp << 8 | loop_gain_x_lc898122_lsb_sharp;
		loop_gain_y_lc898122_sharp = loop_gain_y_lc898122_msb_sharp << 8| loop_gain_y_lc898122_lsb_sharp;
		
	       CDBG_OIS(" hall_offset_x = 0x%x\n",hall_offset_x_lc898122_sharp);
	       CDBG_OIS(" hall_offset_y = 0x%x\n",hall_offset_y_lc898122_sharp);
	       CDBG_OIS(" hall_bias_x = 0x%x\n",hall_bias_x_lc898122_sharp);
	       CDBG_OIS(" hall_bias_y = 0x%x\n",hall_bias_y_lc898122_sharp);
	       CDBG_OIS(" hall_ad_offset_x = 0x%x\n",hall_ad_offset_x_lc898122_sharp);
	       CDBG_OIS(" hall_ad_offset_y = 0x%x\n",hall_ad_offset_y_lc898122_sharp);
	       CDBG_OIS(" loop_gain_x = 0x%x\n",loop_gain_x_lc898122_sharp);
		CDBG_OIS(" loop_gain_y = 0x%x\n",loop_gain_y_lc898122_sharp);

		
	       read_ois_byte_data_lc898122_sharp(GYRO_OFFSET_X_MSB_ADDR_LC898122_SHARP,(unsigned char *)&gyro_offset_x_msb_lc898122_sharp);
	       CDBG_OIS(" gyro_offset_x_msb = 0x%x\n",gyro_offset_x_msb_lc898122_sharp);
		read_ois_byte_data_lc898122_sharp(GYRO_OFFSET_X_LSB_ADDR_LC898122_SHARP,(unsigned char *)&gyro_offset_x_lsb_lc898122_sharp);
	       CDBG_OIS(" gyro_offset_x_lsb = 0x%x\n",gyro_offset_x_lsb_lc898122_sharp);
		read_ois_byte_data_lc898122_sharp(GYRO_OFFSET_Y_MSB_ADDR_LC898122_SHARP,(unsigned char *)&gyro_offset_y_msb_lc898122_sharp);
	       CDBG_OIS(" gyro_offset_y_msb = 0x%x\n",gyro_offset_y_msb_lc898122_sharp);
		read_ois_byte_data_lc898122_sharp(GYRO_OFFSET_Y_LSB_ADDR_LC898122_SHARP,(unsigned char *)&gyro_offset_y_lsb_lc898122_sharp);
	       CDBG_OIS(" gyro_offset_y_lsb = 0x%x\n",gyro_offset_y_lsb_lc898122_sharp);
		read_ois_byte_data_lc898122_sharp(OSC_VALUE_ADDR_LC898122_SHARP,(unsigned char *)&osc_value_lc898122_sharp);
	       CDBG_OIS(" osc_value = 0x%x\n",osc_value_lc898122_sharp);
		read_ois_byte_data_lc898122_sharp(GYRO_GAIN_X_31_24_ADDR_LC898122_SHARP,(unsigned char *)&gyro_gain_x_31_24_lc898122_sharp);
	       CDBG_OIS(" gyro_gain_x_31_24 = 0x%x\n",gyro_gain_x_31_24_lc898122_sharp);
		read_ois_byte_data_lc898122_sharp(GYRO_GAIN_X_23_16_ADDR_LC898122_SHARP,(unsigned char *)&gyro_gain_x_23_16_lc898122_sharp);
	       CDBG_OIS(" gyro_gain_x_23_16 = 0x%x\n",gyro_gain_x_23_16_lc898122_sharp);
		read_ois_byte_data_lc898122_sharp(GYRO_GAIN_X_15_8_ADDR_LC898122_SHARP,(unsigned char *)&gyro_gain_x_15_8_lc898122_sharp);
	       CDBG_OIS(" gyro_gain_x_15_8 = 0x%x\n",gyro_gain_x_15_8_lc898122_sharp);
		read_ois_byte_data_lc898122_sharp(GYRO_GAIN_X_7_0_ADDR_LC898122_SHARP,(unsigned char *)&gyro_gain_x_7_0_lc898122_sharp);
	       CDBG_OIS(" gyro_gain_x_7_0 = 0x%x\n",gyro_gain_x_7_0_lc898122_sharp);

		gyro_gain_x_lc898122_sharp = (gyro_gain_x_31_24_lc898122_sharp <<24) | (gyro_gain_x_23_16_lc898122_sharp <<16)|(gyro_gain_x_15_8_lc898122_sharp<<8)|gyro_gain_x_7_0_lc898122_sharp;

		
		read_ois_byte_data_lc898122_sharp(GYRO_GAIN_Y_31_24_ADDR_LC898122_SHARP,(unsigned char *)&gyro_gain_y_31_24_lc898122_sharp);
	       CDBG_OIS(" gyro_gain_y_31_24 = 0x%x\n",gyro_gain_y_31_24_lc898122_sharp);
		read_ois_byte_data_lc898122_sharp(GYRO_GAIN_Y_23_16_ADDR_LC898122_SHARP,(unsigned char *)&gyro_gain_y_23_16_lc898122_sharp);
	       CDBG_OIS(" gyro_gain_y_23_16 = 0x%x\n",gyro_gain_y_23_16_lc898122_sharp);
		read_ois_byte_data_lc898122_sharp(GYRO_GAIN_Y_15_8_ADDR_LC898122_SHARP,(unsigned char *)&gyro_gain_y_15_8_lc898122_sharp);
	       CDBG_OIS(" gyro_gain_y_15_8 = 0x%x\n",gyro_gain_y_15_8_lc898122_sharp);
		read_ois_byte_data_lc898122_sharp(GYRO_GAIN_Y_7_0_ADDR_LC898122_SHARP,(unsigned char *)&gyro_gain_y_7_0_lc898122_sharp);
		CDBG_OIS(" gyro_gain_y_7_0 = 0x%x\n",gyro_gain_y_7_0_lc898122_sharp);
			
		gyro_gain_y_lc898122_sharp = (gyro_gain_y_31_24_lc898122_sharp <<24) | (gyro_gain_y_23_16_lc898122_sharp <<16)|(gyro_gain_y_15_8_lc898122_sharp<<8)|gyro_gain_y_7_0_lc898122_sharp;
              
		ois_otp_lc898122_flag_sharp = 1;

	}

       RegReadA(0x27F,&read_data);
       //printk(" read_data 0x27F 0xac= 0x%x\n",read_data);
	
	RamAccFixMod(0x01);
	
	RamWriteA(0x1479 ,hall_offset_x_lc898122_sharp );
	RamWriteA(0x14F9 ,hall_offset_y_lc898122_sharp);
	RamWriteA(0x147A ,hall_bias_x_lc898122_sharp );
	RamWriteA(0x14FA ,hall_bias_y_lc898122_sharp );
	RamWriteA(0x1450 ,hall_ad_offset_x_lc898122_sharp );
	RamWriteA(0x14D0 ,hall_ad_offset_y_lc898122_sharp );
	RamWriteA(0x10D3 ,loop_gain_x_lc898122_sharp );
	RamWriteA(0x11D3 ,loop_gain_y_lc898122_sharp );

	RamAccFixMod(0x00);
	
	RegWriteA(0x02A0,gyro_offset_x_msb_lc898122_sharp);
	RegWriteA(0x02A1,gyro_offset_x_lsb_lc898122_sharp);
	RegWriteA(0x02A2,gyro_offset_y_msb_lc898122_sharp);
	RegWriteA(0x02A3,gyro_offset_y_lsb_lc898122_sharp);
	
	RamWrite32A(0x1020,gyro_gain_x_lc898122_sharp);
	RamWrite32A(0x1120,gyro_gain_y_lc898122_sharp);
	
	RegWriteA(0x0257,osc_value_lc898122_sharp);

	//SetTregAf_lc898122_sharp( 0x400 );
	
	RegWriteA(0x0302,0x00);
	RegWriteA(0x0303,0xDF);
       
	RtnCen(0x00);
	

	SetPanTiltMode(1);
	//OisEna();
	GyrCon(1) ; //ZTEMT-wangqiaoming::modify OIS drift problem
	msm_ois_release_cci_lc898122_sharp();
	//printk(" after imx234_ois_otp_ois_sharp \n");

       
	//zte_camera_backlight(zte_camera_mfd,1);
       
	//mutex_unlock(&s_ctrl->zte_otp_mutex);
}
EXPORT_SYMBOL(imx234_ois_otp_ois_sharp);
//static struct msm_sensor_ctrl_t imx234_ois_sharp_s_ctrl;
#if 1

//static int zte_adaptive_imx234_ois_sharp(struct msm_sensor_ctrl_t* s_ctrl)
int zte_adaptive_imx234_ois_sharp(struct msm_sensor_ctrl_t* s_ctrl)
{
	int rc;
	int sharp_temp_i2c_data_type = 0 ;
 //   unsigned short sharp_module_id = 0;
    unsigned short sharp_module_id_lsb = 0;
//    unsigned short sharp_module_id_msb = 0;
	s_ctrl->sensor_i2c_client->cci_client->sid = 0xA0 >> 1;
	sharp_temp_i2c_data_type = s_ctrl->sensor_i2c_client->addr_type;
	s_ctrl->sensor_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	//check if it is sharp module
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		                     s_ctrl->sensor_i2c_client,0x00,
					(uint16_t *)&sharp_module_id_lsb, MSM_CAMERA_I2C_BYTE_DATA);
#if 0
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		                     s_ctrl->sensor_i2c_client,0x01,
					(uint16_t *)&sharp_module_id_msb, MSM_CAMERA_I2C_BYTE_DATA);
#endif
	

//	sharp_module_id = sharp_module_id_msb << 8 | sharp_module_id_lsb;

	//printk(" sharp_module_id = 0x%x\n",sharp_module_id);
	printk(" sharp_module_id = 0x%x\n",sharp_module_id_lsb);

	if (rc < 0) {
			pr_err("%s failed\n", __func__);

	}
	if (sharp_module_id_lsb != 0x21) {
			pr_err("%s failed\n", __func__);
			rc = -1;
	}
	s_ctrl->sensor_i2c_client->cci_client->sid = s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
	s_ctrl->sensor_i2c_client->addr_type = sharp_temp_i2c_data_type;
	return rc;
}
#endif
 void zte_read_otp_imx234_ois_sharp(struct msm_sensor_ctrl_t* s_ctrl)
{
	static int ois_init_flag_up_lc898122_sharp=0;
	enum msm_camera_i2c_reg_addr_type temp_i2c_data_type; 
	int rc;
	if (ois_init_flag_up_lc898122_sharp==0) {
		ois_init_flag_up_lc898122_sharp=1;
			/*ZTEMT: Add for Read AF OTP  ---Start*/
    	s_ctrl->sensor_i2c_client->cci_client->sid = 0xA0 >> 1;
		temp_i2c_data_type = s_ctrl->sensor_i2c_client->addr_type;
		s_ctrl->sensor_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		                     s_ctrl->sensor_i2c_client,AF_START_CURRENT_LC898122_LSB_SHARP,
					(uint16_t *)&af_start_value_lc898122_lsb_sharp, MSM_CAMERA_I2C_BYTE_DATA);

		//printk("af_start_value_lc898122_lsb_sharp = 0x%x\n",af_start_value_lc898122_lsb_sharp);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		                     s_ctrl->sensor_i2c_client,AF_START_CURRENT_LC898122_MSB_SHARP,
					(uint16_t *)&af_start_value_lc898122_msb_sharp, MSM_CAMERA_I2C_BYTE_DATA);

		//printk(" af_start_value_lc898122_msb_sharp = 0x%x\n",af_start_value_lc898122_msb_sharp);
		
	       rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		                     s_ctrl->sensor_i2c_client,AF_START_INFINITY_LC898122_LSB_SHARP,
					(uint16_t *)&af_infinity_value_lc898122_lsb_sharp, MSM_CAMERA_I2C_BYTE_DATA);

		//printk(" af_infinity_value_lc898122_lsb_sharp = 0x%x\n",af_infinity_value_lc898122_lsb_sharp);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		                     s_ctrl->sensor_i2c_client,AF_START_INFINITY_LC898122_MSB_SHARP,
					(uint16_t *)&af_infinity_value_lc898122_msb_sharp, MSM_CAMERA_I2C_BYTE_DATA);

		//printk(" af_infinity_value_lc898122_msb_sharp = 0x%x\n",af_infinity_value_lc898122_msb_sharp);
			
	       rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		                     s_ctrl->sensor_i2c_client,AF_START_MACRO_LC898122_LSB_SHARP,
					(uint16_t *)&af_macro_value_lc898122_lsb_sharp, MSM_CAMERA_I2C_BYTE_DATA);
		   
		//printk(" af_macro_value_lc898122_lsb_sharp = 0x%x\n",af_macro_value_lc898122_lsb_sharp);
		
	       rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		                     s_ctrl->sensor_i2c_client,AF_START_MACRO_LC898122_MSB_SHARP,
					(uint16_t *)&af_macro_value_lc898122_msb_sharp, MSM_CAMERA_I2C_BYTE_DATA);

		//printk(" af_macro_value_lc898122_msb_sharp = 0x%x\n",af_macro_value_lc898122_msb_sharp);

		af_start_value_lc898122_sharp = af_start_value_lc898122_msb_sharp <<8 |af_start_value_lc898122_lsb_sharp;

		af_infinity_value_lc898122_sharp = af_infinity_value_lc898122_msb_sharp <<8 |af_infinity_value_lc898122_lsb_sharp;

		af_macro_value_lc898122_sharp = af_macro_value_lc898122_msb_sharp <<8 | af_macro_value_lc898122_lsb_sharp;

		//printk(" af_start_value_lc898122_sharp = 0x%x,af_infinity_value_lc898122_sharp = 0x%x,af_macro_value_lc898122_sharp = 0x%x\n",af_start_value_lc898122_sharp,af_infinity_value_lc898122_sharp,af_macro_value_lc898122_sharp);
		
		if (rc < 0) {
				pr_err("%s cci_init failed\n", __func__);
				//goto power_up_failed;
		}
		
		s_ctrl->sensor_i2c_client->cci_client->sid = s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;

		s_ctrl->sensor_i2c_client->addr_type = temp_i2c_data_type;
		/*ZTEMT: Add for Read AF OTP  ---End*/
	}
}
#if 0
static uint32_t otp_duration = HZ/1000;
static void zte_workquene_schedule_imx234_ois_sharp(struct msm_sensor_ctrl_t* s_ctrl)
{
	static int once = 1;
	if (once == 1) {
		once = 0;
		return;
	}
	schedule_delayed_work(&s_ctrl->zte_otp_worker,
			otp_duration);
}
static void zte_power_down(struct msm_sensor_ctrl_t* s_ctrl)
{
	static int ois_init_flag_down=0;

	if (ois_init_flag_down==0)
		ois_init_flag_down=1;
	else {
		mutex_lock(&s_ctrl->zte_otp_mutex);	
		RtnCen(0x00);
		SrvCon(0x00,0);  
		SrvCon(0x01,0);  
		//SetTregAf_lc898122_sharp(0x400);	
		msm_ois_release_cci_lc898122_sharp();
		//zte_camera_backlight(zte_camera_mfd,0);
		mutex_unlock(&s_ctrl->zte_otp_mutex);	
	}
}
static void zte_workquene_cancel_imx234_ois_sharp(struct msm_sensor_ctrl_t* s_ctrl)
{
	static int once = 1;
	if (once == 1) {
		once = 0;
		return;
	}
	cancel_delayed_work_sync(&s_ctrl->zte_otp_worker);
}

static void zte_workquene_init_imx234_ois_sharp(struct msm_sensor_ctrl_t* s_ctrl)
{	     
	INIT_DELAYED_WORK(&s_ctrl->zte_otp_worker, imx234_ois_otp_ois_sharp);
	mutex_init(&s_ctrl->zte_otp_mutex);

}
static void zte_control_imx234_ois_sharp(struct msm_sensor_ctrl_t* s_ctrl, int enable)
{
	if (enable) {
		mutex_lock(&s_ctrl->zte_otp_mutex);
		OisEna();
		mutex_unlock(&s_ctrl->zte_otp_mutex);
	} else {
		mutex_lock(&s_ctrl->zte_otp_mutex);
		RtnCen(0x00);
		mutex_unlock(&s_ctrl->zte_otp_mutex);
	}
}

#endif

