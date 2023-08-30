/* drivers/input/touchscreen/gt1x.c
 *
 * 2010 - 2014 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * Version: 1.4   
 * Release Date:  2015/07/10
 */
 
#include <linux/irq.h>
#include "gt1x.h"
#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif

static struct work_struct gt1x_work;
static struct input_dev *input_dev;
static struct workqueue_struct *gt1x_wq;
static const char *gt1x_ts_name = "goodix-ts";
static const char *input_dev_phys = "input/ts";
#ifdef GTP_CONFIG_OF
int gt1x_rst_gpio;
int gt1x_int_gpio;
bool resume_in_workqueue; //add by wkh for resume slowly
#endif

#if 1
#include "../tpd_sys.h"//zz
#ifdef CONFIG_CREATE_TPD_SYS_INTERFACE
#include <linux/ctype.h>
#include "test_function.h"
static unsigned char g_str_save_file_path[256];
static unsigned char g_str_ini_file_path[256];
static unsigned char g_str_ini_filename[128];
static int g_node_data_type = -1;
//struct mutex tpd_test_mutex;
struct gtx_test_buffer g_gtx_test_buffer;
extern int test_error_code;
extern s32 gtp_test_sysfs_init(void); 
extern void gtp_test_sysfs_deinit(void);
#endif
#endif

static int gt1x_register_powermanger(void);
static int gt1x_unregister_powermanger(void);



/**
 * gt1x_i2c_write - i2c write.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_write(u16 addr, u8 * buffer, s32 len)
{
	struct i2c_msg msg = {
		.flags = 0,
		.addr = gt1x_i2c_client->addr,
	};
	return _do_i2c_write(&msg, addr, buffer, len);
}

/**
 * gt1x_i2c_read - i2c read.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_read(u16 addr, u8 * buffer, s32 len)
{
	u8 addr_buf[GTP_ADDR_LENGTH] = { (addr >> 8) & 0xFF, addr & 0xFF };
	struct i2c_msg msgs[2] = {
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = 0,
		 .buf = addr_buf,
		 .len = GTP_ADDR_LENGTH},
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = I2C_M_RD}
	};
	return _do_i2c_read(msgs, addr, buffer, len);
}

static spinlock_t irq_lock;
static s32 irq_is_disable = 0;

/**
 * gt1x_irq_enable - enable irq function.
 *
 */
void gt1x_irq_enable(void)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&irq_lock, irqflags);
	if (irq_is_disable) {
		enable_irq(gt1x_i2c_client->irq);
		irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

/**
 * gt1x_irq_enable - disable irq function.
 *
 */
void gt1x_irq_disable(void)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_is_disable) {
		irq_is_disable = 1;
		disable_irq_nosync(gt1x_i2c_client->irq);
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

#ifndef GTP_CONFIG_OF
int gt1x_power_switch(s32 state)
{
    return 0;
}
#endif

int gt1x_debug_proc(u8 * buf, int count)
{
	return -1;
}

#if GTP_CHARGER_SWITCH
u32 gt1x_get_charger_status(void)
{
#error Need to get charger status of your platform.
}
#endif

/**
 * gt1x_ts_irq_handler - External interrupt service routine for interrupt mode.
 * @irq:  interrupt number.
 * @dev_id: private data pointer.
 * Return: Handle Result.
 *  		IRQ_HANDLED: interrupt handled successfully
 */
static irqreturn_t gt1x_ts_irq_handler(int irq, void *dev_id)
{
	GTP_DEBUG_FUNC();
    gt1x_irq_disable();
	queue_work(gt1x_wq, &gt1x_work);
	return IRQ_HANDLED;
}

/**
 * gt1x_touch_down - Report touch point event .
 * @id: trackId
 * @x:  input x coordinate
 * @y:  input y coordinate
 * @w:  input pressure
 * Return: none.
 */
void gt1x_touch_down(s32 x, s32 y, s32 size, s32 id)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

#if GTP_ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_report_abs(input_dev, ABS_MT_PRESSURE, size);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
#else
	input_report_key(input_dev, BTN_TOUCH, 1);
	if ((!size) && (!id)) {
		/* for virtual button */
		input_report_abs(input_dev, ABS_MT_PRESSURE, 100);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 100);
	} else {
		input_report_abs(input_dev, ABS_MT_PRESSURE, size);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);
		input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	}
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_touch_up -  Report touch release event.
 * @id: trackId
 * Return: none.
 */
void gt1x_touch_up(s32 id)
{
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
#else
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_ts_work_func - Goodix touchscreen work function.
 * @iwork: work struct of gt1x_workqueue.
 * Return: none.
 */
static void gt1x_ts_work_func(struct work_struct *work)
{
	u8 end_cmd = 0;
	u8 finger = 0;
	s32 ret = 0;
	u8 point_data[11] = { 0 };

    if (update_info.status) {
        GTP_DEBUG("Ignore interrupts during fw update.");
        return;
    }
    
#if GTP_GESTURE_WAKEUP
    if(gt1x_resume_flag)
		goto exit_work_func;
	else
	{
	ret = gesture_event_handler(input_dev);
	if (ret >= 0) {
		goto exit_work_func;
	}
	}
#endif

	if (gt1x_halt) {
		GTP_DEBUG("Ignore interrupts after suspend...");
        return;
	}

	ret = gt1x_i2c_read(GTP_READ_COOR_ADDR, point_data, sizeof(point_data));
	if (ret < 0) {
		GTP_ERROR("I2C transfer error!");
#if !GTP_ESD_PROTECT
		gt1x_power_reset();
#endif
		goto exit_work_func;
	}

	finger = point_data[0];
	if (finger == 0x00) {
		gt1x_request_event_handler();
	}

	if ((finger & 0x80) == 0) {
#if HOTKNOT_BLOCK_RW
		if (!hotknot_paired_flag)
#endif
		{
			//GTP_ERROR("buffer not ready:0x%02x", finger);
			goto exit_eint;
		}
	}
#if HOTKNOT_BLOCK_RW
	ret = hotknot_event_handler(point_data);
	if (!ret) {
		goto exit_work_func;
	}
#endif

#if GTP_PROXIMITY
	ret = gt1x_prox_event_handler(point_data);
	if (ret > 0) {
		goto exit_work_func;
	}
#endif

#if GTP_WITH_STYLUS
	ret = gt1x_touch_event_handler(point_data, input_dev, pen_dev);
#else
	ret = gt1x_touch_event_handler(point_data, input_dev, NULL);
#endif

exit_work_func:
	if (!gt1x_rawdiff_mode && (ret >= 0 || ret == ERROR_VALUE)) {
		ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
		if (ret < 0) {
			GTP_ERROR("I2C write end_cmd  error!");
		}
	}
exit_eint:
    gt1x_irq_enable();
    
}

/* 
 * Devices Tree support, 
*/
#ifdef GTP_CONFIG_OF

static struct regulator *vdd_ana;
static struct regulator *vcc_i2c;

/**
 * gt1x_parse_dt - parse platform infomation form devices tree.
 */
static int gt1x_parse_dt(struct device *dev)
{
	struct device_node *np;
    int ret = 0;

    if (!dev)
        return -ENODEV;
    
    np = dev->of_node;
	gt1x_int_gpio = of_get_named_gpio(np, "goodix,irq-gpio", 0);
	gt1x_rst_gpio = of_get_named_gpio(np, "goodix,rst-gpio", 0);

    if (!gpio_is_valid(gt1x_int_gpio) || !gpio_is_valid(gt1x_rst_gpio)) {
        GTP_ERROR("Invalid GPIO, irq-gpio:%d, rst-gpio:%d",
            gt1x_int_gpio, gt1x_rst_gpio);
        return -EINVAL;
    }

    vdd_ana = regulator_get(dev, "vdd_ana");
    if (IS_ERR(vdd_ana)) {
	    GTP_ERROR("regulator get of vdd_ana failed");
	    ret = PTR_ERR(vdd_ana);
	    vdd_ana = NULL;
	    return ret;
    }

	vcc_i2c = regulator_get(dev, "vcc_i2c");
	if (IS_ERR(vcc_i2c)) {
		GTP_ERROR("regulator get of vcc_i2c failed");
		ret = PTR_ERR(vcc_i2c);
		vcc_i2c = NULL;
		goto ERR_GET_VCC;
	}
//add by wkh for resume slowly begin 
	resume_in_workqueue = of_property_read_bool(np, "goodix,resume-in-workqueue");
	if(resume_in_workqueue)
	    printk("gtp: it's true\n");
//end
    return 0;
ERR_GET_VCC:
    regulator_put(vdd_ana);
    vdd_ana = NULL;
    return ret;

}

/**
 * gt1x_power_switch - power switch .
 * @on: 1-switch on, 0-switch off.
 * return: 0-succeed, -1-faileds
 */
int gt1x_power_switch(int on)
{

	int ret;
	struct i2c_client *client = gt1x_i2c_client;

    if (!client || !vdd_ana || !vcc_i2c)
        return -1;
	
	if (on) {
		GTP_DEBUG("GTP power on.");
		ret = regulator_enable(vdd_ana);
		udelay(2);
		ret = regulator_enable(vcc_i2c);
	} else {
		GTP_DEBUG("GTP power off.");
		ret = regulator_disable(vcc_i2c);
		udelay(2);
		ret = regulator_disable(vdd_ana);
	}
	return ret;
	
}
#endif

static void gt1x_remove_gpio_and_power(void)
{
    if (gpio_is_valid(gt1x_int_gpio))
        gpio_free(gt1x_int_gpio);

    if (gpio_is_valid(gt1x_rst_gpio))
        gpio_free(gt1x_rst_gpio);
    
#ifdef GTP_CONFIG_OF      
    if (vcc_i2c)
        regulator_put(vcc_i2c);

    if (vdd_ana)
        regulator_put(vdd_ana);
#endif

    if (gt1x_i2c_client && gt1x_i2c_client->irq)
        free_irq(gt1x_i2c_client->irq, gt1x_i2c_client);
    
}


/**
 * gt1x_request_io_port - Request gpio(INT & RST) ports.
 */
static s32 gt1x_request_io_port(void)
{
	s32 ret = 0;

	GTP_DEBUG_FUNC();
	ret = gpio_request(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_INT_PORT, ret);
		ret = -ENODEV;
	} else {
		GTP_GPIO_AS_INT(GTP_INT_PORT);
		gt1x_i2c_client->irq = GTP_INT_IRQ;
	}

	ret = gpio_request(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_RST_PORT, ret);
		ret = -ENODEV;
	}

	GTP_GPIO_AS_INPUT(GTP_RST_PORT);
	if (ret < 0) {
		gpio_free(GTP_RST_PORT);
		gpio_free(GTP_INT_PORT);
	}

	return ret;
}

/**
 * gt1x_request_irq - Request interrupt.
 * Return
 *      0: succeed, -1: failed.
 */
static s32 gt1x_request_irq(void)
{
	s32 ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG_FUNC();
	GTP_DEBUG("INT trigger type:%x", gt1x_int_type);

	ret = request_irq(gt1x_i2c_client->irq, gt1x_ts_irq_handler, irq_table[gt1x_int_type], gt1x_i2c_client->name, gt1x_i2c_client);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		GTP_GPIO_AS_INPUT(GTP_INT_PORT);
		gpio_free(GTP_INT_PORT);

		return -1;
	} else {
		gt1x_irq_disable();
		return 0;
	}
}

/**
 * gt1x_request_input_dev -  Request input device Function.
 * Return
 *      0: succeed, -1: failed.
 */
static s8 gt1x_request_input_dev(void)
{
	s8 ret = -1;
#if GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif

	GTP_DEBUG_FUNC();

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#if GTP_ICS_SLOT_REPORT
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0))
    input_mt_init_slots(input_dev, 16, INPUT_MT_DIRECT);
#else
    input_mt_init_slots(input_dev, 16); 
#endif
#else
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++) {
		input_set_capability(input_dev, EV_KEY, gt1x_touch_key_array[index]);
	}
#endif

#if GTP_GESTURE_WAKEUP
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOUBLEC/*KEY_GES_REGULAR*/);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT/*KEY_GES_CUSTOM*/);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT/*KEY_GES_CUSTOM*/);
#endif

#if GTP_CHANGE_X2Y
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_x_max, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_y_max, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	input_dev->name = gt1x_ts_name;
	input_dev->phys = input_dev_phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xDEAD;
	input_dev->id.product = 0xBEEF;
	input_dev->id.version = 10427;

	ret = input_register_device(input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed", input_dev->name);
		return -ENODEV;
	}

	return 0;
}
#if 1
#ifdef CONFIG_CREATE_TPD_SYS_INTERFACE
static char *string_trim_tail(unsigned char * str_buf)
{
	int length = 0;
	int i = 0;

	length = strlen(str_buf);

	for(i = length -1; i >=0; i--) {
		if((!isprint(str_buf[i])) || ( ' ' == str_buf[i]) ) {
			str_buf[i] = '\0';
		} else {
			break;
		}
	}

	return str_buf;
}

static char *dir_path_add_slash(unsigned char * str_buf)
{
	int length = 0;

	string_trim_tail(str_buf);

	length = strlen(str_buf);
	
	if( '/' != str_buf[length - 1]) {
		strcat(str_buf, "/");
	}

	return str_buf;
}

static int tpd_test_save_file_path_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	//mutex_lock(&tpd_test_mutex);

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_save_file_path);

	//mutex_unlock(&tpd_test_mutex);

	return num_read_chars;
}

static int tpd_test_save_file_path_store(struct tpd_classdev_t *cdev, const char *buf)
{
	memset(g_str_save_file_path, 0, sizeof(g_str_save_file_path));
	snprintf(g_str_save_file_path, 256, "%s", buf);

	dir_path_add_slash(g_str_save_file_path);

	GTP_INFO("save file path:%s.", g_str_save_file_path);

	return 0;
}

static int tpd_test_ini_file_path_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	//mutex_lock(&tpd_test_mutex);

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_ini_file_path);

	//mutex_unlock(&tpd_test_mutex);

	return num_read_chars;
}

static int tpd_test_ini_file_path_store(struct tpd_classdev_t *cdev, const char *buf)
{
	memset(g_str_ini_file_path, 0, sizeof(g_str_ini_file_path));
	snprintf(g_str_ini_file_path, 256, "%s", buf);

	dir_path_add_slash(g_str_ini_file_path);

	GTP_INFO("ini file path:%s.", g_str_ini_file_path);

	return 0;
}

static int tpd_test_filename_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	//mutex_lock(&tpd_test_mutex);

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_ini_filename);

	//mutex_unlock(&tpd_test_mutex);

	return num_read_chars;
}

static int tpd_test_filename_store(struct tpd_classdev_t *cdev, const char *buf)
{
	memset(g_str_ini_filename, 0, sizeof(g_str_ini_filename));
	snprintf(g_str_ini_filename, 128, "%s", buf);

	string_trim_tail(g_str_ini_filename);
	
	GTP_INFO("fwname:%s.", g_str_ini_filename);

	return 0;
}

static int tpd_test_cmd_store(struct tpd_classdev_t *cdev, const char *buf)
{
	struct i2c_client *client = gt1x_i2c_client;
	unsigned long command = 0;
	int retval = -1;
	static int node_opened = 0;
	struct gtx_test_buffer * stp_test = &g_gtx_test_buffer;
	
	retval = kstrtoul(buf, 10, &command);
	if (retval) {
		GTP_DEBUG("invalid param:%s", buf);
		return 0;
	}
	
	printk("tpd %s [func] command:%ld, ini filename:%s.\n", __func__, command, g_str_ini_filename);

	//mutex_lock(&tpd_test_mutex);

	//command 1:open node; 2:start test; 3:close node.
	if(1 == command) {    //open test node, alloc space etc...
		if(NULL == stp_test->result_buffer ) {
			int buffer[5] = {0, 0, 0, 0, 0};;
			//tpd_test_get_channel_setting(&buffer[0]);
			stp_test->i_txNum = buffer[0] ;
			stp_test->i_rxNum = buffer[1] ;
			
			stp_test->result_buffer = (char*)malloc(TEST_RESULT_LENGTH);
			stp_test->result_length = 0;
		}
		if(NULL == stp_test->node_failed_buffer ) {
			stp_test->node_failed_buffer = (char*)malloc(TEST_RESULT_LENGTH);
			stp_test->node_failed_buffer_length = 0;
			stp_test->node_failed_count = 0;
		}
		if(NULL == stp_test->temp_buffer ) {
			stp_test->temp_buffer = (char*)malloc(TEST_TEMP_LENGTH);
		}
		if(NULL == stp_test->procedure_buffer ) {
			stp_test->procedure_buffer = (char*)malloc(TEST_RESULT_LENGTH);
			stp_test->procedure_length = 0;
		}
		node_opened = 1;
	} else if(2 == command) {
		if(1 == node_opened) {   //start test
		
			stp_test->result_length = 0;
			stp_test->node_failed_buffer_length = 0;
			stp_test->node_failed_count = 0;
			stp_test->procedure_length = 0;

			stp_test->test_result = test_error_code = 0;
			
			disable_irq(client->irq);
			open_short_test(NULL);
			enable_irq(client->irq);
		} else {
			GTP_DEBUG("command:0x%ld,  open node before start test.", command);
		}
	} else if(3 == command) {    //close test node, free space etc...
		node_opened = 0;
	} else {
		GTP_DEBUG("invalid command %ld", command);
	}
	//mutex_unlock(&tpd_test_mutex);

	return 0;
}

static int tpd_test_cmd_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;
	//struct i2c_client *client = fts_i2c_client;
	struct gtx_test_buffer * stp_test = &g_gtx_test_buffer;
	int buffer_length = 0;
	int i_len = 0;

//	mutex_lock(&tpd_test_mutex);
	//printk("tpd %d [func] in %s.\n", stp_test->test_result,stp_test->result_buffer);
	
	i_len = sprintf(buf, "%d,%d,%d", stp_test->i_txNum, stp_test->i_rxNum, stp_test->node_failed_count);
	printk("tpd %s [func] test resutl:0x%x && rawdata node failed count:0x%x.\n", __func__, stp_test->test_result, stp_test->node_failed_count);
	
	buffer_length = (stp_test->node_failed_buffer_length + 1) > (PAGE_SIZE - i_len) ? (PAGE_SIZE - i_len - 1) : stp_test->node_failed_buffer_length;
	printk("tpd %s [func] failed node string lenght:0x%x, buffer_length:0x%x.\n", __func__, stp_test->node_failed_buffer_length, buffer_length);

	if(stp_test->node_failed_buffer != NULL && buffer_length > 0) {
		memcpy(buf + i_len, stp_test->node_failed_buffer, buffer_length);
		buf[buffer_length + i_len] = '\0';
	}
	/**/
	printk("tpd %s test:%s.", __func__, buf);
	
	num_read_chars = buffer_length + i_len;
//	mutex_unlock(&tpd_test_mutex);

	return num_read_chars;
}

static int tpd_test_result_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "0x%x", test_error_code);

	return num_read_chars;
}

static int tpd_test_node_data_store(struct tpd_classdev_t *cdev, const char *buf)
{
	int data_type = 0;

	sscanf(buf, "%d", &data_type);

	
//	mutex_lock(&tpd_test_mutex);

	g_node_data_type = data_type;

//	mutex_unlock(&tpd_test_mutex);

	return 0;
}

static int tpd_test_node_data_show(struct tpd_classdev_t *cdev, char *buf)
{
	/*ssize_t num_read_chars = 0;
	int iLen = 0;
	
	mutex_lock(&fts_input_dev->mutex);
	init_i2c_write_func(fts_test_i2c_write);
	init_i2c_read_func(fts_test_i2c_read);
	fts_test_funcs();

	iLen = tpd_test_get_tp_node_data(g_node_data_type, buf, 4096);

	num_read_chars = iLen;
	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;*/
	return 0;
}

static int tpd_test_channel_show(struct tpd_classdev_t *cdev, char *buf)
{
	/*ssize_t num_read_chars = 0;
	int buffer[5] = {0, 0, 0, 0, 0};
	
	init_i2c_write_func(fts_test_i2c_write);
	init_i2c_read_func(fts_test_i2c_read);
	fts_test_funcs();

	tpd_test_get_channel_setting(&buffer[0]);
	g_fts_test_buffer.i_txNum = buffer[0];
	g_fts_test_buffer.i_rxNum = buffer[1];

	num_read_chars = snprintf(buf, PAGE_SIZE, "%d %d %d %d %d", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);

	return num_read_chars;*/
	return 0;
}

static int tpd_write_block(struct tpd_classdev_t *cdev, u16 addr, u8 *buf, int len)
{
	int ret = 0;
	unsigned char buffer[256];

	buffer[0] = (u8) addr;
	memcpy(&buffer[1], buf, len);
	ret = gt1x_i2c_write(addr, buffer, len + 1);
	
	return ret;
}
	

static int tpd_read_block(struct tpd_classdev_t *cdev, u16 addr, u8 *buf, int len)
{
	int ret = 0;
	int i = 0;
	
	ret = gt1x_i2c_read(addr, buf, len);
	GTP_INFO("Read from addr:0x%x val=", addr);
	for(i = 0; i < (len < 8? len : 8); i++)
	{
		GTP_INFO("0x%x ", buf[i]);
	}
	GTP_INFO("\n");
	
	return ret;
}
	
static int tpd_init_tpinfo(struct tpd_classdev_t *cdev)
{
      GTP_INFO("GTP tpd_init_tpinfo: %s", GTP_DRIVER_VERSION);
	strcpy(cdev->ic_tpinfo.tp_name, "Goodix");
	cdev->ic_tpinfo.chip_model_id = 5;
	
	cdev->ic_tpinfo.chip_part_id= gt1x_chip_type;
	cdev->ic_tpinfo.module_id= gt1x_version.sensor_id;
	cdev->ic_tpinfo.chip_ver = 0;
	cdev->ic_tpinfo.firmware_ver= gt1x_version.patch_id;
	cdev->ic_tpinfo.i2c_type = 0;
	cdev->ic_tpinfo.i2c_addr = gt1x_i2c_client->addr;
	return 0;
}

static int tpd_register_fw_class(void)
{
      GTP_INFO("GTP tpd_register_fw_class: %s", GTP_DRIVER_VERSION);
	tpd_fw_cdev.private = NULL;
	tpd_fw_cdev.flash_fw = NULL;
	tpd_fw_cdev.read_block = tpd_read_block;
	tpd_fw_cdev.write_block = tpd_write_block;
	tpd_fw_cdev.compare_tp_version = NULL;
	tpd_fw_cdev.get_tpinfo = tpd_init_tpinfo;
	
	(void)tpd_init_tpinfo(&tpd_fw_cdev);

	tpd_fw_cdev.tpd_test_set_save_filepath = tpd_test_save_file_path_store;
	tpd_fw_cdev.tpd_test_get_save_filepath = tpd_test_save_file_path_show;
	tpd_fw_cdev.tpd_test_set_ini_filepath = tpd_test_ini_file_path_store;
	tpd_fw_cdev.tpd_test_get_ini_filepath = tpd_test_ini_file_path_show;
	tpd_fw_cdev.tpd_test_set_filename = tpd_test_filename_store;
	tpd_fw_cdev.tpd_test_get_filename = tpd_test_filename_show;
	tpd_fw_cdev.tpd_test_get_result = tpd_test_result_show;
	tpd_fw_cdev.tpd_test_set_cmd = tpd_test_cmd_store;
	tpd_fw_cdev.tpd_test_get_cmd = tpd_test_cmd_show;
	tpd_fw_cdev.tpd_test_set_node_data_type = tpd_test_node_data_store;
	tpd_fw_cdev.tpd_test_get_node_data = tpd_test_node_data_show;
	tpd_fw_cdev.tpd_test_get_channel_info = tpd_test_channel_show;
/*	*/

	return 0;
}
#endif
#endif 
/**
 * gt1x_ts_probe -   I2c probe.
 * @client: i2c device struct.
 * @id: device id.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 ret = -1;
#if GTP_AUTO_UPDATE
	struct task_struct *thread = NULL;
#endif
	//do NOT remove these logs
	GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

	gt1x_i2c_client = client;
	spin_lock_init(&irq_lock);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}

#ifdef GTP_CONFIG_OF	/* device tree support */
	if (client->dev.of_node) {
		gt1x_parse_dt(&client->dev);
	}
#endif

	ret = gt1x_request_io_port();
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		return ret;
	}

	gt1x_init();

	INIT_WORK(&gt1x_work, gt1x_ts_work_func);

	ret = gt1x_request_input_dev();
	if (ret < 0) {
		GTP_ERROR("GTP request input dev failed");
	}

	ret = gt1x_request_irq();
	if (ret < 0) {
		GTP_INFO("GTP works in polling mode.");
	} else {
		GTP_INFO("GTP works in interrupt mode.");
	}

#if GTP_GESTURE_WAKEUP
	enable_irq_wake(client->irq);
#endif

	gt1x_irq_enable();

#if GTP_ESD_PROTECT
	// must before auto update
	gt1x_init_esd_protect();
	gt1x_esd_switch(SWITCH_ON);
#endif

#if GTP_AUTO_UPDATE
	thread = kthread_run(gt1x_auto_update_proc, (void *)NULL, "gt1x_auto_update");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		GTP_ERROR("Failed to create auto-update thread: %d.", ret);
	}
#endif
	gt1x_register_powermanger();

	gtp_test_sysfs_init();//zz
	#ifdef CONFIG_CREATE_TPD_SYS_INTERFACE
	(void)tpd_register_fw_class();
      #endif
	
	return 0;
}

/**
 * gt1x_ts_remove -  Goodix touchscreen driver release function.
 * @client: i2c device struct.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_remove(struct i2c_client *client)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver removing...");
	gt1x_unregister_powermanger();

#if GTP_GESTURE_WAKEUP
	disable_irq_wake(client->irq);
#endif
    gt1x_deinit();
	input_unregister_device(input_dev);
    gt1x_remove_gpio_and_power();

    gtp_test_sysfs_deinit();//zz
    
    return 0;
}

#if   defined(CONFIG_FB)	
/* frame buffer notifier block control the suspend/resume procedure */
static struct notifier_block gt1x_fb_notifier;
//add by wkh for resume slowly begin
struct work_struct fb_notify_work;

static void fb_notify_resume_work(struct work_struct *work)
{
	gt1x_resume();
}
//end
static int gtp_fb_notifier_callback(struct notifier_block *noti, unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	int *blank;

#if GTP_INCELL_PANEL
    #ifndef FB_EARLY_EVENT_BLANK
        #error Need add FB_EARLY_EVENT_BLANK to fbmem.c
    #endif
    
	if (ev_data && ev_data->data && event == FB_EARLY_EVENT_BLANK) {
		blank = ev_data->data;
        if (*blank == FB_BLANK_UNBLANK) {
			GTP_DEBUG("Resume by fb notifier.");
			gt1x_resume();
        }
    }
	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
        if (*blank == FB_BLANK_POWERDOWN) {
			GTP_DEBUG("Suspend by fb notifier.");
			gt1x_suspend();
        }
    }
#else
	if (ev_data && ev_data->data /*&& event == FB_EVENT_BLANK*/) {
		blank = ev_data->data;
		if(resume_in_workqueue)
		{
		   if(event == FB_EARLY_EVENT_BLANK &&				       
		   	*blank == FB_BLANK_UNBLANK) 
		   	schedule_work(&fb_notify_work);
		   else if(event == FB_EVENT_BLANK &&				      
		   	*blank == FB_BLANK_POWERDOWN)
		   	{
		   	    flush_work(&fb_notify_work);            	
				gt1x_suspend();				
				//rmi4_data->fb_ready = false;
		   	}
		}
		else
		{
		    if (event == FB_EVENT_BLANK)
	    	{
	    	    if (*blank == FB_BLANK_POWERDOWN) 
				{				
			gt1x_suspend();
					//rmi4_data->fb_ready = false;			
				} 
				else if (*blank == FB_BLANK_UNBLANK) 
				{	
			GTP_DEBUG("Resume by fb notifier.");
				    gt1x_resume();				
					//rmi4_data->fb_ready = true;			
				}
		}
	}

    }
#endif
	return 0;
}
#elif defined(CONFIG_PM)
/**
 * gt1x_ts_suspend - i2c suspend callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_pm_suspend(struct device *dev)
{
    return gt1x_suspend();
}

/**
 * gt1x_ts_resume - i2c resume callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_pm_resume(struct device *dev)
{
	return gt1x_resume();
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops gt1x_ts_pm_ops = {
	.suspend = gt1x_pm_suspend,
	.resume = gt1x_pm_resume,
};

#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* earlysuspend module the suspend/resume procedure */
static void gt1x_ts_early_suspend(struct early_suspend *h)
{
	gt1x_suspend();
}

static void gt1x_ts_late_resume(struct early_suspend *h)
{
	gt1x_resume();
}

static struct early_suspend gt1x_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = gt1x_ts_early_suspend,
	.resume = gt1x_ts_late_resume,
};
#endif


static int gt1x_register_powermanger(void)
{
#if   defined(CONFIG_FB)
    INIT_WORK(&fb_notify_work,fb_notify_resume_work);//add by wkh
	gt1x_fb_notifier.notifier_call = gtp_fb_notifier_callback;
	fb_register_client(&gt1x_fb_notifier);
	
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	register_early_suspend(&gt1x_early_suspend);
#endif	
	return 0;
}

static int gt1x_unregister_powermanger(void)
{
#if   defined(CONFIG_FB)
	fb_unregister_client(&gt1x_fb_notifier);
		
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&gt1x_early_suspend);
#endif
	return 0;
}

#ifdef GTP_CONFIG_OF
static const struct of_device_id gt1x_match_table[] = {
		{.compatible = "goodix,gt1x",},
		{ },
};
#endif

static const struct i2c_device_id gt1x_ts_id[] = {
	{GTP_I2C_NAME, 0},
	{}
};

static struct i2c_driver gt1x_ts_driver = {
	.probe = gt1x_ts_probe,
	.remove = gt1x_ts_remove,
	.id_table = gt1x_ts_id,
	.driver = {
		   .name = GTP_I2C_NAME,
		   .owner = THIS_MODULE,
#ifdef GTP_CONFIG_OF
		   .of_match_table = gt1x_match_table,
#endif
#if !defined(CONFIG_FB) && defined(CONFIG_PM)
		   .pm = &gt1x_ts_pm_ops,
#endif
		   },
};

/**   
 * gt1x_ts_init - Driver Install function.
 * Return   0---succeed.
 */
static int __init gt1x_ts_init(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver installing...");
	gt1x_wq = create_singlethread_workqueue("gt1x_wq");
	if (!gt1x_wq) {
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}

	return i2c_add_driver(&gt1x_ts_driver);
}

/**   
 * gt1x_ts_exit - Driver uninstall function.
 * Return   0---succeed.
 */
static void __exit gt1x_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&gt1x_ts_driver);
	if (gt1x_wq) {
		destroy_workqueue(gt1x_wq);
	}
}

module_init(gt1x_ts_init);
module_exit(gt1x_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
