/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/ztemt_hw_version.h>


#ifdef CONFIG_ZTEMT_HW_VERSION_DEBUG
static int debug_value=1;
#else
static int debug_value=0;
#endif
#define ztemt_hw_version_debug(fmt, args...) do {if(debug_value==1)printk(KERN_DEBUG "[ztemt_hw_version]"fmt, ##args);} while(0)

#ifdef  CONFIG_ZTEMT_HW_VERSION_NX517J
//ADC for hw_version , GPIO for wifi
static const struct hardware_id_map_st hardware_id_map[] = {
	{0,    300,  HW_A, ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,"MB_A","wifi_epcos",  "SC"},  //id_mv=0
	{0,    300,  HW_A, ZTE_GPIO_FLOAT,    ZTE_GPIO_PULL_UP,  ZTE_GPIO_FLOAT,"MB_A","wifi_samsung","SC"},  //id_mv=0
};
#elif defined CONFIG_ZTEMT_HW_VER_BY_ADC
//ADC for hw_version , GPIO for wifi
static const struct hardware_id_map_st hardware_id_map[] = {
	{0,    300,  HW_A, ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,"MB_A","wifi_samsung","SC"},  //id_mv=0
	{300,  600,  HW_B, ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,"MB_B","wifi_samsung","SC"},  //id_mv=544
	{300,  600,  HW_B, ZTE_GPIO_PULL_UP,  ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,"MB_B","wifi_samsung","HC"},  //id_mv=544  (high config 64G  4G)
	{300,  600,  HW_B, ZTE_GPIO_FLOAT,    ZTE_GPIO_PULL_UP,  ZTE_GPIO_FLOAT,"MB_B","wifi_epcos",  "SC"},  //id_mv=544
	{300,  600,  HW_B, ZTE_GPIO_PULL_UP,  ZTE_GPIO_PULL_UP,  ZTE_GPIO_FLOAT,"MB_B","wifi_epcos",  "HC"},  //id_mv=544 (high config 64G  4G)
	{600,  900,  HW_C, ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,"MB_C","111",         "SC"},  //id_mv=720
	{900,  1200, HW_D, ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,"MB_D","111",         "SC"},  //id_mv=1120
	{1200, 1500, HW_E, ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,"MB_E","111",         "SC"},  //id_mv=1475
	{1500, 1800, HW_F, ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,"MB_F","111",         "SC"},  //id_mv=1800
};
#else
//GPIO for hw_version , ADC for wifi
static const struct hardware_id_map_st hardware_id_map[] = {
	{0,    80,  HW_A, ZTE_GPIO_PULL_UP,   ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,  "MB_A","wifi_samsung"},
	{0,    80,  HW_B, ZTE_GPIO_PULL_DOWN, ZTE_GPIO_FLOAT,    ZTE_GPIO_FLOAT,  "MB_B","wifi_samsung"},
	{0,    80,  HW_C, ZTE_GPIO_PULL_DOWN, ZTE_GPIO_PULL_DOWN,ZTE_GPIO_FLOAT,  "MB_C","wifi_samsung"},
	{80,   220, HW_C, ZTE_GPIO_PULL_DOWN, ZTE_GPIO_PULL_DOWN,ZTE_GPIO_FLOAT,  "MB_C","wifi_epcos"},
};
#endif

static int ztemt_hw_id = -1;
static int ztemt_hw_mv = 900;
static int ztemt_hw_mv_2 = 900;
char board_type_gpio_str[100]="0,0,0";

static int  ztemt_board_type_setup(char *param)
{
	int magic_num = 0;
    get_option(&param, &magic_num);
	ztemt_hw_mv = magic_num;
    return 0;
}
early_param("board_type", ztemt_board_type_setup);

static int  ztemt_board_type_setup_2(char *param)
{
	int magic_num = 0;
    get_option(&param, &magic_num);
	ztemt_hw_mv_2 = magic_num;
    return 0;
}
early_param("board_type_2", ztemt_board_type_setup_2);


static int  ztemt_board_type_setup_gpio(char *param)
{
	//printk("%s=%s",__func__, param);
    memcpy(board_type_gpio_str, param, strlen(param));
    return 0;
}
early_param("board_type_gpio", ztemt_board_type_setup_gpio);


#ifdef CONFIG_ZTEMT_HW_VER_BY_ADC
//使用adc来判断硬件版本号，使用GPIO来判断wifi

static int32_t ztemt_get_hardware_type(const struct hardware_id_map_st *pts,
		uint32_t tablesize, int input)
{
	uint32_t i = 0;

	if ( pts == NULL )
		return -EINVAL;

	while (i < tablesize) {
		if ( (pts[i].low_mv <= input) && (input <= pts[i].high_mv) ) 
			break;
		else
			i++;
	}

	if ( i < tablesize )
		return pts[i].hw_type;
	else
		return HW_UN;
}

int ztemt_get_hw_id(void)
{
	if(ztemt_hw_id >= 0)
	    return ztemt_hw_id;

    ztemt_hw_id = ztemt_get_hardware_type(
		                hardware_id_map,
						ARRAY_SIZE(hardware_id_map),
						ztemt_hw_mv);
	
    ztemt_hw_version_debug("hw_id_mv=%d mv hw_id=%d hw_ver=%s\n",
		    ztemt_hw_mv,ztemt_hw_id,hardware_id_map[ztemt_hw_id].hw_ver);

	return ztemt_hw_id;
}


const char* ztemt_get_hw_wifi(void)
{
	uint32_t tablesize;
	int hw_type;
	uint32_t i = 0;
	char * gpio_str_temp=board_type_gpio_str;
	int gpio_value_A,gpio_value_B,gpio_value_C;
	const struct hardware_id_map_st *pts= hardware_id_map;

	if ( pts == NULL )
		return NULL;

	tablesize=sizeof(hardware_id_map)/sizeof(hardware_id_map[0]);
	ztemt_hw_version_debug("tablesize=%d\n",tablesize);

    hw_type=ztemt_get_hw_id();

	ztemt_hw_version_debug("%s=%s\n",__func__, gpio_str_temp);
    sscanf(gpio_str_temp ,"%d,%d,%d", &gpio_value_A, &gpio_value_B, &gpio_value_C);

	while (i < tablesize) {
		if ((pts[i].hw_type == hw_type) &&
			(pts[i].gpio_A == gpio_value_A) && (pts[i].gpio_B == gpio_value_B) && (pts[i].gpio_C == gpio_value_C))
			break;
		else
			i++;
	}

	//ztemt_hw_version_debug("i=%d, hw_wifi=%s\n", i,pts[i].hw_wifi);
	if ( i < tablesize ){
		ztemt_hw_version_debug("hw_wifi=%s\n", pts[i].hw_wifi);
		return pts[i].hw_wifi;
	}
	else
		return "unknow";
}

#else
//使用gpio口来识别硬件版本号，使用adc来识别wifi
static int32_t ztemt_get_hardware_type_gpio(const struct hardware_id_map_st *pts,
		uint32_t tablesize, char * gpio_str)
{
	uint32_t i = 0;
    char * gpio_str_temp=gpio_str;
	int gpio_value_A,gpio_value_B,gpio_value_C;

	//printk("%s=%s\n",__func__, gpio_str_temp);
    sscanf(gpio_str_temp ,"%d,%d,%d", &gpio_value_A, &gpio_value_B, &gpio_value_C);

	if ( pts == NULL )
		return -EINVAL;

	while (i < tablesize) {
		if ( (pts[i].gpio_A == gpio_value_A) && (pts[i].gpio_B == gpio_value_B) && (pts[i].gpio_C == gpio_value_C) ) 
			break;
		else 
			i++;
	}

	if ( i < tablesize ) 
		return pts[i].hw_type;
    else 
		return HW_UN;
}

int ztemt_get_hw_id(void)
{
	if(ztemt_hw_id >= 0)
	    return ztemt_hw_id;

    ztemt_hw_id = ztemt_get_hardware_type_gpio(
                        hardware_id_map,
                        ARRAY_SIZE(hardware_id_map),
                        board_type_gpio_str);
    ztemt_hw_version_debug("hw_id=%d",ztemt_hw_id);

	return ztemt_hw_id;
}

const char* ztemt_get_hw_wifi(void)
{
	uint32_t tablesize;
	int hw_type;
	uint32_t i = 0;
	const struct hardware_id_map_st *pts= hardware_id_map;

	if ( pts == NULL )
	    return NULL;

	tablesize=sizeof(hardware_id_map)/sizeof(hardware_id_map[0]);
	ztemt_hw_version_debug("tablesize=%d\n",tablesize);

    hw_type=ztemt_get_hw_id();

	ztemt_hw_version_debug("ztemt_hw_mv=%d\n",ztemt_hw_mv);
	while (i < tablesize) {
		if ((pts[i].hw_type == hw_type) &&
		     (pts[i].low_mv <= ztemt_hw_mv) && (ztemt_hw_mv <= pts[i].high_mv))
			break;
		else
			i++;
	}

	if ( i < tablesize ){
		ztemt_hw_version_debug("hw_wifi=%s\n", pts[i].hw_wifi);
		return pts[i].hw_wifi;
	}
	else
		return "unknow";
}

#endif

EXPORT_SYMBOL_GPL(ztemt_get_hw_wifi);

EXPORT_SYMBOL_GPL(ztemt_get_hw_id);


void ztemt_get_hw_version(char* result)
{
    int hw_id;
    if(!result)
		return;

    hw_id = ztemt_get_hw_id();
	
    if(hw_id != HW_UN){
        strcpy(result,hardware_id_map[hw_id].hw_ver); 
     }else
	    sprintf(result, "%s","unknow");
}
EXPORT_SYMBOL_GPL(ztemt_get_hw_version);

static ssize_t ztemt_hw_version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
    ztemt_get_hw_version(buf);
    //printk("%s : %d : version=%s\n",__func__,__LINE__,buf);
    ztemt_hw_version_debug("version=%s\n",buf);
    return sprintf(buf,"%s",buf);
}
static struct kobj_attribute version_attr=
    __ATTR(version, 0664, ztemt_hw_version_show, NULL);


#ifdef CONFIG_ZTEMT_HW_CONFIG_ITEM
const struct hardware_id_map_st *ztemt_get_table_item(void)
{

	uint32_t tablesize;
	int hw_type;
	uint32_t i = 0;
	char * gpio_str_temp=board_type_gpio_str;
	int gpio_value_A,gpio_value_B,gpio_value_C;
	const struct hardware_id_map_st *pts= hardware_id_map;

	if ( pts == NULL )
		return NULL;

	tablesize=sizeof(hardware_id_map)/sizeof(hardware_id_map[0]);
	ztemt_hw_version_debug("tablesize=%d\n",tablesize);

	hw_type=ztemt_get_hw_id();

	ztemt_hw_version_debug("%s=%s\n",__func__, gpio_str_temp);
	sscanf(gpio_str_temp ,"%d,%d,%d", &gpio_value_A, &gpio_value_B, &gpio_value_C);

	while (i < tablesize) {
		if ((pts[i].hw_type == hw_type) &&
			(pts[i].gpio_A == gpio_value_A) && (pts[i].gpio_B == gpio_value_B) && (pts[i].gpio_C == gpio_value_C))
			break;
		else
			i++;
	}

	if ( i < tablesize ){
		return &pts[i];
	}
	else
		return NULL;

}

static ssize_t ztemt_hw_config_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
    const struct hardware_id_map_st *pts_tmp;

    pts_tmp=ztemt_get_table_item();
	if(pts_tmp==NULL)
		return sprintf(buf,"%s","unknown");

	ztemt_hw_version_debug("hw_config=%s\n", pts_tmp->hw_config);

	return sprintf(buf,"%s",pts_tmp->hw_config);

}

static struct kobj_attribute config_attr=
    __ATTR(config, 0664, ztemt_hw_config_show, NULL);
#endif

static ssize_t debug_value_store(struct kobject *kobj,
	    struct kobj_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%d", &debug_value);
    return count;
}

static ssize_t debug_value_show(struct kobject *kobj,
	   struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", debug_value);	
}

static struct kobj_attribute debug_value_attr=
    __ATTR(debug_value, 0664, debug_value_show, debug_value_store);


static struct attribute *ztemt_hw_version_attrs[] = {
    &debug_value_attr.attr,
    &version_attr.attr,
#ifdef CONFIG_ZTEMT_HW_CONFIG_ITEM
    &config_attr.attr,
#endif
    NULL,
};

static struct attribute_group ztemt_hw_version_attr_group = {
    .attrs = ztemt_hw_version_attrs,
};


struct kobject *hw_version_kobj;

int __init
ztemt_hw_version_init(void)
{
    int rc = 0;

    ztemt_hw_version_debug("ztemt_hw_version creat attributes start \n");
  
    //hw_version__kobj = kobject_create_and_add("ztemt_hw_version", kernel_kobj);
    hw_version_kobj = kobject_create_and_add("ztemt_hw_version", NULL);
    if (!hw_version_kobj){
	printk(KERN_ERR "%s: ztemt_hw_version kobj create error\n", __func__);
	return -ENOMEM;
    }

    rc=sysfs_create_group(hw_version_kobj,&ztemt_hw_version_attr_group);
    if(rc)
      printk(KERN_ERR "%s: failed to create ztemt_hw_version group attributes\n", __func__);

    ztemt_hw_version_debug("ztemt_hw_version creat attributes end \n");
    return rc;
}


static void __exit
ztemt_hw_version_exit(void)
{
    sysfs_remove_group(hw_version_kobj,&ztemt_hw_version_attr_group);
    kobject_put(hw_version_kobj);	
}

module_init(ztemt_hw_version_init);
module_exit(ztemt_hw_version_exit);

MODULE_DESCRIPTION("ztemt_hw_version driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ztemt_hw_version" );
