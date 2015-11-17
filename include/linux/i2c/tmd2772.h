/*******************************************************************************
*                                                                              *
*       File Name:      tmd2772.h                                              *
*                                                                              *
*******************************************************************************/
#ifndef _TMD2772_H_
#define _TMD2772_H_

#include <linux/wakelock.h>

#define TAOS_DEVICE_NAME                "taos"
#define TAOS_DEVICE_ID                  "tritonFN"
#define TAOS_ID_NAME_SIZE               10
#define TAOS_TRITON_CHIPIDVAL           0x00
#define TAOS_TRITON_MAXREGS             32
#define TAOS_DEVICE_ADDR1               0x29
#define TAOS_DEVICE_ADDR2               0x39
#define TAOS_DEVICE_ADDR3               0x49
#define TAOS_MAX_NUM_DEVICES            3
#define TAOS_MAX_DEVICE_REGS            32
#define I2C_MAX_ADAPTERS                12

// TRITON register offsets
#define TAOS_TRITON_CNTRL               0x00
#define TAOS_TRITON_ALS_TIME            0X01
#define TAOS_TRITON_PRX_TIME            0x02
#define TAOS_TRITON_WAIT_TIME           0x03
#define TAOS_TRITON_ALS_MINTHRESHLO     0X04
#define TAOS_TRITON_ALS_MINTHRESHHI     0X05
#define TAOS_TRITON_ALS_MAXTHRESHLO     0X06
#define TAOS_TRITON_ALS_MAXTHRESHHI     0X07
#define TAOS_TRITON_PRX_MINTHRESHLO     0X08
#define TAOS_TRITON_PRX_MINTHRESHHI     0X09
#define TAOS_TRITON_PRX_MAXTHRESHLO     0X0A
#define TAOS_TRITON_PRX_MAXTHRESHHI     0X0B
#define TAOS_TRITON_INTERRUPT           0x0C
#define TAOS_TRITON_PRX_CFG             0x0D
#define TAOS_TRITON_PRX_COUNT           0x0E
#define TAOS_TRITON_GAIN                0x0F
#define TAOS_TRITON_REVID               0x11
#define TAOS_TRITON_CHIPID              0x12
#define TAOS_TRITON_STATUS              0x13
#define TAOS_TRITON_ALS_CHAN0LO         0x14
#define TAOS_TRITON_ALS_CHAN0HI         0x15
#define TAOS_TRITON_ALS_CHAN1LO         0x16
#define TAOS_TRITON_ALS_CHAN1HI         0x17
#define TAOS_TRITON_PRX_LO              0x18
#define TAOS_TRITON_PRX_HI              0x19
#define TAOS_TRITON_TEST_STATUS         0x1F
#define TAOS_TRITON_REG_ALL             0xFF

// Triton cmd reg masks
#define TAOS_TRITON_CMD_REG             0X80
#define TAOS_TRITON_CMD_AUTO            0x20
#define TAOS_TRITON_CMD_BYTE_RW         0x00
#define TAOS_TRITON_CMD_WORD_BLK_RW     0x20
#define TAOS_TRITON_CMD_SPL_FN          0x60
#define TAOS_TRITON_CMD_PROX_INTCLR     0X05
#define TAOS_TRITON_CMD_ALS_INTCLR      0X06
#define TAOS_TRITON_CMD_PROXALS_INTCLR  0X07
#define TAOS_TRITON_CMD_TST_REG         0X08
#define TAOS_TRITON_CMD_USER_REG        0X09

// Triton cntrl reg masks
#define TAOS_TRITON_CNTL_PROX_INT_ENBL  0X20
#define TAOS_TRITON_CNTL_ALS_INT_ENBL   0X10
#define TAOS_TRITON_CNTL_WAIT_TMR_ENBL  0X08
#define TAOS_TRITON_CNTL_PROX_DET_ENBL  0X04
#define TAOS_TRITON_CNTL_ADC_ENBL       0x02
#define TAOS_TRITON_CNTL_PWRON          0x01

// Triton status reg masks
#define TAOS_TRITON_STATUS_ADCVALID     0x01
#define TAOS_TRITON_STATUS_PRXVALID     0x02
#define TAOS_TRITON_STATUS_ADCINTR      0x10
#define TAOS_TRITON_STATUS_PRXINTR      0x20

// lux constants
#define TAOS_MAX_LUX                    10000
#define TAOS_SCALE_MILLILUX             3
#define TAOS_FILTER_DEPTH               3
#define CHIP_ID                         0x3d

#define TAOS_INPUT_NAME                 "lightsensor"
#define	POLL_DELAY	                    msecs_to_jiffies(5)
#define	TAOS_ALS_ADC_TIME_WHEN_PROX_ON	0xF0//0XF5//0XEB
#define TAOS_ALS_GAIN_DIVIDE            1000
#define TAOS_ALS_GAIN_1X                0
#define TAOS_ALS_GAIN_8X                1
#define TAOS_ALS_GAIN_16X               2
#define TAOS_ALS_GAIN_120X              3

#define CAL_THRESHOLD                      "/persist/proxdata/threshold"
#define PATH_PROX_OFFSET                   "/persist/sensors/proximity/offset/proximity_offset"
#define PATH_PROX_UNCOVER_DATA             "/persist/sensors/proximity/uncover_data"

#define PROX_LED_PULSE_CNT                  12
#define PROX_THRESHOLD_DISTANCE             100
#define PROX_DATA_TARGET                    150
#define PROX_DATA_MAX                       1023
#define PROX_OFFSET_CAL_BUFFER_SIZE         10
#define PROX_OFFSET_CAL_THRESHOLD           800
#define PROX_OFFSET_CAL_ABILITY_MAX         72 // -9*72
#define PROX_DATA_SAFE_RANGE_MIN            (PROX_DATA_TARGET - 50)
#define PROX_DATA_SAFE_RANGE_MAX            (PROX_DATA_TARGET + 250)
#define PROX_OFFSET_CAL_GETDATA_DELAY       10
#define PROX_DEFAULT_THRESHOLD_HIGH         800
#define PROX_DEFAULT_THRESHOLD_LOW          700
#define PROX_THRESHOLD_HIGH_MAX             800
#define PROX_THRESHOLD_HIGH_MIN             500
#define PROX_DEFAULT_OFFSET_CNT              0
#define PROX_THRESHOLD_SAFE_DISTANCE        300
#define PROX_NEED_OFFSET_CAL_THRESHOLD      100
#define PROX_LIGHT_CHANEL_UNCOVERY_THRES    50
#define PROX_LIGHT_LUX_UNCOVERY_THRES       50

/* POWER SUPPLY VOLTAGE RANGE */
#define TMD2772_VDD_MIN_UV	2000000
#define TMD2772_VDD_MAX_UV	3300000
#define TMD2772_VIO_MIN_UV	1750000
#define TMD2772_VIO_MAX_UV	1950000

// proximity data
struct taos_prox_info
{
        u16 prox_clear;
        u16 prox_data;
        int prox_event;
};

struct taos_wake_lock{
    struct wake_lock lock;
    bool   locked;
    char   *name;
};

enum
{
	DEBUG_DEFAULT  = 1U << 0,
	DEBUG_CRITICAL = 1U << 2,
	DEBUG_ABNORMAL = 1U << 3, //don't modify it
};

enum
{
    PROX_THRESHOLD_USE_DEFAULT    =  1,
    PROX_THRESHOLD_USE_CALIBRATE  =  2,
};

struct taos_cfg
{
    u16 als_time;
    u16 scale_factor_als;
	u16 scale_factor_prox;
    u16 gain_trim;
    u8  filter_history;
    u8  filter_count;
    u8  gain;
	u16	prox_threshold_hi;
	u16 prox_threshold_lo;
	u16	als_threshold_hi;
	u16 als_threshold_lo;
	u8	prox_int_time;
	u8	prox_adc_time;
	u8	prox_wait_time;
	u8	prox_intr_filter;
	u8	prox_config;
	u8	prox_pulse_cnt;
	u8	prox_gain;
	u8	prox_config_offset;
};

struct taos_data {
	char taos_id;
	char taos_name[TAOS_ID_NAME_SIZE];
	char *prox_name;
	char *als_name;
	char *chip_name;
	char valid;

	bool prox_calibrate_result;
	bool prox_offset_cal_result;
	bool phone_is_sleep;
	bool prox_offset_cal_verify;
	bool prox_calibrate_verify;
    bool prox_on;
	bool als_on;
	bool irq_enabled;
	bool irq_work_status;
	bool init;
	bool prox_uncover;
	bool prox_offset_need_cal;
	bool prox_offset_spy_ok;
    bool prox_spy_enable;
    bool prox_offset_cal_start;
    bool pro_ft;
    bool prox_debug;
    bool als_debug;
    bool prox_auto_calibrate;
    bool prox_thres_have_cal;

	int  prox_calibrate_times;
	int  prox_thres_hi_max;
	int  prox_thres_hi_min;
	int  prox_thres_lo_max;
	int  prox_thres_lo_min;
	int  prox_data_max;
	int  irq_pin_num;
    int  prox_led_plus_cnt;
    int  prox_offset_cal_ability;
    int  prox_offset_cal_per_bit;
    int  prox_uncover_data;
    int  light_percent;
	int  als_poll_time_mul;
	int  prox_thres_type;

	unsigned int addr;

	struct i2c_client *client;
	struct cdev cdev;
	struct delayed_work work;
	struct work_struct irq_work;
	struct workqueue_struct *irq_work_queue;
	struct taos_wake_lock proximity_wakelock;
	struct mutex lock;
	struct mutex prox_work_lock;
	struct device *class_dev;
	struct delayed_work als_poll_work;
	struct delayed_work prox_calibrate_work;
	struct delayed_work prox_offset_cal_work;
	struct delayed_work prox_flush_work;

	struct hrtimer  prox_unwakelock_timer;
	struct input_dev *p_idev;
	struct input_dev *a_idev;

	struct device *proximity_dev;
	struct device *light_dev;
	struct device *gesture_dev;
	struct regulator	*vdd;
	struct regulator	*vio;
	struct semaphore update_lock;
};

#define TAOS_IOCTL_MAGIC        	0XCF
#define TAOS_IOCTL_ALS_ON       	_IO(TAOS_IOCTL_MAGIC, 1)
#define TAOS_IOCTL_ALS_OFF      	_IO(TAOS_IOCTL_MAGIC, 2)
#define TAOS_IOCTL_ALS_DATA     	_IOR(TAOS_IOCTL_MAGIC, 3, short)
#define TAOS_IOCTL_ALS_CALIBRATE	_IO(TAOS_IOCTL_MAGIC, 4)
#define TAOS_IOCTL_CONFIG_GET   	_IOR(TAOS_IOCTL_MAGIC, 5, struct taos_cfg *)
#define TAOS_IOCTL_CONFIG_SET		_IOW(TAOS_IOCTL_MAGIC, 6, struct taos_cfg)
#define TAOS_IOCTL_PROX_ON		    _IO(TAOS_IOCTL_MAGIC, 7)
#define TAOS_IOCTL_PROX_OFF		    _IO(TAOS_IOCTL_MAGIC, 8)
#define TAOS_IOCTL_PROX_DATA		_IOR(TAOS_IOCTL_MAGIC, 9, struct taos_prox_info)
#define TAOS_IOCTL_PROX_EVENT       _IO(TAOS_IOCTL_MAGIC, 10)
#define TAOS_IOCTL_PROX_CALIBRATE	_IO(TAOS_IOCTL_MAGIC, 11)
#define TAOS_IOCTL_SENSOR_ON	    (TAOS_IOCTL_MAGIC, 12)
#define TAOS_IOCTL_SENSOR_OFF	    _IO(TAOS_IOCTL_MAGIC, 13)
#define TAOS_IOCTL_SENSOR_CONFIG	_IOW(TAOS_IOCTL_MAGIC, 14, struct taos_cfg)
#define TAOS_IOCTL_SENSOR_CHECK	    _IO(TAOS_IOCTL_MAGIC, 15)
#define TAOS_IOCTL_SENSOR_test	    _IO(TAOS_IOCTL_MAGIC, 16)
#define TAOS_IOCTL_ALS_SET_DELAY	_IO(TAOS_IOCTL_MAGIC, 17)

static int  tmd2772_probe(struct i2c_client *clientp, const struct i2c_device_id *idp);
static int  tmd2772_remove(struct i2c_client *client);
static int  taos_get_lux(void);
static int  taos_lux_filter(int raw_lux);
static int  taos_device_name(unsigned char *bufp, char **device_name);
static int  taos_prox_poll(struct taos_prox_info *prxp);
static void taos_prox_poll_timer_func(unsigned long param);
static void taos_prox_poll_timer_start(void);
static int  taos_prox_threshold_set(void);
static int  taos_als_get_data(void);
static int  taos_interrupts_clear(void);
static int  taos_resume(struct i2c_client *client);
static int  taos_suspend(struct i2c_client *client,pm_message_t mesg);
static int  taos_sensors_als_poll_on(void);
static int  taos_sensors_als_poll_off(void);
static void taos_als_poll_work_func(struct work_struct *work);
static int  taos_als_gain_set(unsigned als_gain);
static void taos_update_sat_als(void);
static int  taos_prox_on(void);
static int  taos_prox_off(void);
static int  taos_prox_calibrate(void);
static void taos_prox_calibrate_work_func(struct work_struct *work);
static void taos_prox_offset_cal_work_func(struct work_struct *work);
static void taos_wakelock_ops(struct taos_wake_lock *wakelock, bool enable);
static int  taos_write_cal_file(char *file_path,unsigned int value);
static int  taos_read_cal_value(char *file_path);
static enum hrtimer_restart  taos_prox_unwakelock_work_func(struct hrtimer *timer);
static void taos_prox_threshold_auto_cal(void);
static void taos_prox_spy(struct taos_data *p_taos_data);
static void taos_uncover_check(struct taos_data *p_taos_data);
static void taos_prox_threshold_use_default(struct taos_cfg *p_taos_cfgp);
static int  taos_prox_threshold_use_calibrate(struct taos_data *p_taos_data);
static int  taos_prox_offset_cal(void);
static void taos_prox_threshold_safe_check(void);
#endif
