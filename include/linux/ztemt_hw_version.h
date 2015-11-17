#ifndef __ZTEMT_HW_VERSION_H__
#define __ZTEMT_HW_VERSION_H__

typedef enum
{
	HW_A,
	HW_B,
	HW_C,
	HW_D,
	HW_E,
	HW_F,
	HW_UN// unknow, fail read
}hw_version_type;

enum ztemt_gpio_status {
 ZTE_GPIO_PULL_DOWN = 0,//gpio pull down
 ZTE_GPIO_FLOAT,//gpio float
 ZTE_GPIO_PULL_UP,//gpio pull up
 ZTE_GPIO_UNKNOWN,
};



struct hardware_id_map_st {
	int low_mv;
	int high_mv;
	hw_version_type hw_type;
	int gpio_A;
	int gpio_B;
	int gpio_C;
	char hw_ver[20];
	char hw_wifi[20];
#ifdef CONFIG_ZTEMT_HW_CONFIG_ITEM
	char hw_config[20];
#endif
};
#endif
