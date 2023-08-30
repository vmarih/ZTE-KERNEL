/***********************
 * file : tpd_fw.c
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include "tpd_sys.h"
//#include "tpd.h"
//#include "tpd_common.h"

//#define TPD_SYS_DMESG(a, arg...) pr_info("tpd" ": " a, ##arg)


DECLARE_RWSEM(tp_firmware_list_lock);
LIST_HEAD(tp_firmware_list);

#define MAX_BUF_SIZE 256 * 1024
#define VENDOR_END 0xff

struct tpvendor_t synaptics_vendor_l[] ={
	{0x31, "TPK"},
	{0x32, "Truly"},
	{0x33, "Success"},
	{0x34, "Ofilm"},
	{0x35, "Lead"},
	{0x36, "Wintek"},
	{0x37, "Laibao"},
	{0x38, "CMI"},
	{0x39, "Ecw"},
	{0x41, "Goworld"},
	{0x42, "BaoMing"},
	{0x43, "Eachopto"},
	{0x44, "Mutto"},
	{0x45, "Junda"},
	{0x46, "BOE"},
	{0x47, "TianMa"},
	{0x48, "Samsung"},
	{0x49, "DiJing"},
	{0x50, "LCE"}, //not define.
	{VENDOR_END, "Unkown"},
};

struct tpvendor_t focal_vendor_l[] ={
	{0x11, "TeMeiKe"},
	{0x15, "ChuangWei"},
	{0x51, "Ofilm"},
	{0x55, "LaiBao"},
	{0x57, "Goworld"},
	{0x5a, "Truly"},
	{0x5c, "TPK"},
	{0x5d, "BaoMing"},
	{0x5f, "Success"},
	{0x60, "Lead"},
	{0x67, "DiJing"},
	{0x80, "Eachopto"},
	{0x82, "HeLiTai"},
	{0x85, "JunDa"},
	{0x87, "LianChuang"},
	{0xda, "DiJingDA"},
	{0xf0, "TongXingDa"},
	{VENDOR_END, "Unkown"},
};

struct tpvendor_t cypress_vendor_l[] ={
	{0x01, "TPK"},
	{0x02, "Truly"},
	{0x03, "Success"},
	{0x04, "Ofilm"},
	{0x05, "Lead"},
	{0x06, "Wintek"},
	{0x07, "Laibao"},
	{0x08, "CMI"},
	{0x09, "Ecw"},
	{0x0a, "Goworld"},
	{0x0b, "BaoMing"},
	{0x0c, "Eachopto"},
	{0x0d, "Mutto"},
	{0x0e, "Junda"},
	{VENDOR_END, "Unkown"},
};

struct tpvendor_t atmel_vendor_l[] ={
	{0x06, "Wintek"},
	{0x07, "Laibao"},
	{0x08, "CMI"},
	{0x09, "Ecw"},
	{VENDOR_END, "Unkown"},
};

struct tpvendor_t goodix_vendor_l[] ={
	{0x00, "Eachopto"},
	{0x01, "Success"},
	{0x02, "TPK"},
	{0x03, "BaoMing"},
	{0x04, "Ofilm"},
	{0x05, "Truly"},
	{0x06, "Wintek"},
	{0x07, "Laibao"},
	{0x08, "CMI"},
	{0x09, "Ecw"},
	{0x0a, "Goworld"},
	{0x0b, "Lead"},
	{0x0c, "TeMeiKe"},
	{0x0d, "Mutto"},
	{0x0e, "Junda"},
	{0x0f, "TianMa"},
	{0x10, "SanXing"},
	{0x11, "GuoXian"},
	{0x12, "BoEn"},
	{VENDOR_END, "Unkown"},
};

struct tpvendor_t mstar_vendor_l[] ={
	{0x01, "FuNaYuanChuang"},
	{0x02, "TeMeiKe"},
	{0x307, "LianChuang"},
	{VENDOR_END, "Unkown"},
};

struct tpvendor_t melfas_vendor_l[] ={
	{VENDOR_END, "Unkown"},
};

int tpd_power_already_on = 0;
struct tpd_classdev_t tpd_fw_cdev;

static struct class *tsp_fw_class;

//static unsigned char filepath[256];

static int tpd_alloc_buffer(struct tpd_classdev_t *cdev, int alloc_size)
{
	if(cdev->tp_fw.data == NULL) {
		cdev->tp_fw.data = (unsigned char *)vmalloc(alloc_size);
		if(!cdev->tp_fw.data) {
			dev_err(cdev->dev, "memory alloc failed\n");
			goto error;
		}
		cdev->tp_fw.length = alloc_size;
		printk("tpd: %s malloc %d byte success.\n", __func__, alloc_size);
	}

	return 0;
error:
	return -1;
}

static int tpd_free_buffer(struct tpd_classdev_t *cdev)
{
	if(cdev->tp_fw.data != NULL) {
		vfree(cdev->tp_fw.data);
		cdev->tp_fw.data = NULL;
		cdev->tp_fw.data_len = 0;
		cdev->tp_fw.length = 0;
		cdev->status = STATUS_OK;
	}

	return 0;
}

static int get_chip_vendor(struct tpvendor_t * vendor_l, int count, int vendor_id, char *vendor_name)
{
	int i = 0;
	printk("%s: count: %d.\n", __func__, count);

	for(i = 0; i < count; i ++) {
		if(vendor_l[i].vendor_id == vendor_id || VENDOR_END == vendor_l[i].vendor_id) {
			strcpy(vendor_name, vendor_l[i].vendor_name);
			break;
		}
	}

	return 0;
}

static void tpd_get_tp_module_name(struct tpd_classdev_t *cdev)
{
	unsigned int vendor_id = 0;
	int size = 0;
	
	printk("%s \n", __func__);

	vendor_id = cdev->ic_tpinfo.module_id;
	
	if(NULL != strstr(cdev->ic_tpinfo.tp_name, "Synaptics")) {
		size = sizeof(synaptics_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(synaptics_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
	} else if (NULL != strstr(cdev->ic_tpinfo.tp_name, "Atmel")) {
		size = sizeof(atmel_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(atmel_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
	} else if (NULL != strstr(cdev->ic_tpinfo.tp_name, "Cyttsp")) {
		size = sizeof(cypress_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(cypress_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
	} else if (NULL != strstr(cdev->ic_tpinfo.tp_name, "Focal"))	{
		size = sizeof(focal_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(focal_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
	} else if (NULL != strstr(cdev->ic_tpinfo.tp_name, "Goodix")) {
		size = sizeof(goodix_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(goodix_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
		vendor_id = 0;
	} else if (NULL != strstr(cdev->ic_tpinfo.tp_name, "Melfas")) {
		size = sizeof(melfas_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(melfas_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
	} else if (NULL != strstr(cdev->ic_tpinfo.tp_name, "Mstar")) {
		size = sizeof(mstar_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(mstar_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
	} else {
		strcpy(cdev->ic_tpinfo.vendor_name, "Unkown.");
	}
	strcpy(cdev->file_tpinfo.vendor_name, cdev->ic_tpinfo.vendor_name);

	printk("fun:%s module name:%s.\n",__func__, cdev->ic_tpinfo.vendor_name);
	
}

static int tpd_upgrade_firmware(struct tpd_classdev_t *cdev, int cmd, int forced)
{
	int retval = 0;

	if(!mutex_trylock(&cdev->upgrade_mutex))
	{
		printk("tpd: %s: Pre func execing.\n", __func__);
		retval = -1;
		goto out;
	}
	
	if(cdev->flash_fw &&( (STATUS_BUF_RDY == cdev->status)||  \
		(STATUS_UPG_FAILED == cdev->status)||(STATUS_UPG_NONEED == cdev->status))) {
		if(NULL == cdev->tp_fw.data || 0 == cdev->tp_fw.data_len) {
			printk("tpd: BUFFER is NULL.\n");
			cdev->status = STATUS_UPG_FAILED;
			retval = -1;
			goto error;
		}
		
		retval = cdev->flash_fw(cdev, cdev->tp_fw.data, cdev->tp_fw.data_len, forced);
		if(retval == 0) {
			cdev->status = STATUS_UPG_SUCC;
		} else if (retval == 0xff) {
			cdev->status = STATUS_UPG_NONEED;
		} else {
			cdev->status = STATUS_UPG_FAILED;
		}
	} else {
		printk("tpd: %s: status:%d\n", __func__, cdev->status);
	}

error:
	mutex_unlock(&cdev->upgrade_mutex);

out:
	return retval;
}

//0: tp file version > ic, need upgrade. other: tp file version <= ic, not upgrade.
static int tpd_start_compare_tpinfo(struct tpd_classdev_t *cdev)
{
	int retval = -1;

	//init file tpinfo.
	strcpy(cdev->file_tpinfo.tp_name, cdev->ic_tpinfo.tp_name);
	cdev->file_tpinfo.chip_model_id = cdev->ic_tpinfo.chip_model_id;
	cdev->file_tpinfo.chip_part_id = cdev->ic_tpinfo.chip_part_id;
	cdev->file_tpinfo.chip_ver = cdev->ic_tpinfo.chip_ver;
	cdev->file_tpinfo.config_ver = cdev->ic_tpinfo.config_ver;
	cdev->file_tpinfo.firmware_ver = cdev->ic_tpinfo.firmware_ver;
	cdev->file_tpinfo.module_id = cdev->ic_tpinfo.module_id;
	cdev->fw_compare_result = 1;

	if(NULL == cdev->tp_fw.data || 0 == cdev->tp_fw.data_len || STATUS_BUF_RDY != cdev->status) {
		printk("%s buffer is NULL, return.\n", __func__);
		return 0;
	}

	if(cdev->compare_tp_version) {
		retval = cdev->compare_tp_version(cdev, cdev->tp_fw.data);
		if(0 == retval) {
			cdev->fw_compare_result = 0;
		} else {
			cdev->fw_compare_result = 1;
		}
		printk("%s  result:%d\ntpd  0: tp file version > ic, need upgrade. other: tp file version <= ic, not upgrade.\n",
			__func__, cdev->fw_compare_result );
	}
		
	return 0;
}

static ssize_t tsp_cmd_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *tsp_fw_cdev = dev_get_drvdata(dev);

	return sprintf(buf, "Current cmd is 0x%x.\n", tsp_fw_cdev->cmd);
}

static ssize_t tsp_cmd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int cmd = 0, param = 0;

	mutex_lock(&cdev->cmd_mutex);
	sscanf(buf, "%d %d", &cmd, &param);

	printk("tpd: %s command:%d, param:%d. \n", __func__, cmd, param);
	
	cdev->cmd = cmd;
	switch(cmd) {
	case CMD_WRITE_IMAGE:
		cdev->status = STATUS_BUF_NULL;
		tpd_free_buffer(cdev);
		break;
	case CMD_WRITE_FINISH:
		if(cdev->tp_fw.data_len == param) {
			cdev->status = STATUS_BUF_RDY;
		} else {
			printk("tpd: %s error: not entire content of image. \n", __func__);
		}
		printk("tpd: %s image write 0x%x bytes, should 0x%x bytes.\n ", __func__, cdev->tp_fw.data_len, param);
		break;
	case CMD_READ_IMAGE:

		break;
	case CMD_PORCE_UPG:
		tpd_upgrade_firmware(cdev, cmd, 1);
		break;
	case CMD_PERFORM_UPG:
		tpd_upgrade_firmware(cdev, cmd, 0);
		break;
	case CMD_CLEAN_BUF:
		tpd_free_buffer(cdev);
		break;
	case CMD_SET_PART_ID:
		cdev->ic_tpinfo.chip_part_id = param;
		break;
	case CMD_COMPARE_FIRMWARE:
		tpd_start_compare_tpinfo(cdev);
		break;
	default:
		printk("tpd: %s Not support this cmd:0x%x\n", __func__, cmd);
		break;
	}
	mutex_unlock(&cdev->cmd_mutex);

	return size;
}
static ssize_t tsp_upg_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);

	memcpy(buf, &cdev->status, sizeof(int));

	return sizeof(int);
}
static ssize_t tsp_upg_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	//struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	
	//memcpy(buf, &cdev->status, sizeof(int));

	return sizeof(int);
}
static ssize_t tsp_fw_ic_tpinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);

	if(cdev->get_tpinfo) {
		cdev->get_tpinfo(cdev);
	}

	return sprintf(buf, "%u 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x %s\n",
		cdev->ic_tpinfo.chip_part_id,   cdev->ic_tpinfo.chip_model_id,
		cdev->ic_tpinfo.chip_ver,        cdev->ic_tpinfo.module_id, 
		cdev->ic_tpinfo.firmware_ver, cdev->ic_tpinfo.config_ver, 
		cdev->ic_tpinfo.i2c_type,        cdev->ic_tpinfo.i2c_addr, 
		cdev->ic_tpinfo.tp_name);
}
static ssize_t tsp_fw_ic_tpinfo_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static ssize_t tsp_get_bsg_value_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 1;

	mutex_lock(&cdev->cmd_mutex);
	if(cdev->get_gesture) {
		retval = cdev->get_gesture(cdev);
	}
	printk("tpd: %s val:%d.\n", __func__, retval);
	//memcpy(buf, &retval, sizeof(int));
	retval = sprintf(buf, "0x%02x\n", retval);
	mutex_unlock(&cdev->cmd_mutex);
	
	//retval = sizeof(int);
	return retval;
}
static ssize_t tsp_get_bsg_value_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

int tpd_get_bsg_enable_status(void)
{
	return tpd_fw_cdev.b_gesture_enable;
}

static ssize_t tsp_gesture_enable_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	mutex_lock(&cdev->cmd_mutex);
	printk("tpd: %s val:%d.\n", __func__, cdev->b_gesture_enable);

	//memcpy(buf, &cdev->b_gesture_enable, sizeof(int));
	retval = sprintf(buf, "0x%02x\n", cdev->b_gesture_enable);
	mutex_unlock(&cdev->cmd_mutex);

	//retval = sizeof(int);
	return retval;
}

static ssize_t tsp_gesture_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int ret = -1;
	char *after;
	unsigned long enable = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
	}
	
	printk("tpd: %s val %ld.\n", __func__, enable);

	mutex_lock(&cdev->cmd_mutex);
	if(cdev->wake_gesture) {
		cdev->wake_gesture(cdev, enable);
	}
	cdev->b_gesture_enable = enable;
	mutex_unlock(&cdev->cmd_mutex);

	return size;
}
static ssize_t tsp_film_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	mutex_lock(&cdev->cmd_mutex);
	printk("tpd: %s val:%d.\n", __func__, cdev->b_film_mode_enable);

	retval = snprintf(buf, 32, "%d,\n", cdev->b_film_mode_enable);
	mutex_unlock(&cdev->cmd_mutex);

	return retval;
}

static ssize_t tsp_film_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;
	
	printk("tpd: %s val %d.\n", __func__, input);

	mutex_lock(&cdev->cmd_mutex);
	if(cdev->wake_film_mode) {
		cdev->wake_film_mode(cdev, input);
	}
	cdev->b_film_mode_enable = input;
	mutex_unlock(&cdev->cmd_mutex);
	
	return count;
}
static ssize_t tsp_fw_file_tpinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	
	return sprintf(buf, "%u 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x %s\n",
		cdev->file_tpinfo.chip_part_id,   cdev->file_tpinfo.chip_model_id,
		cdev->file_tpinfo.chip_ver,        cdev->file_tpinfo.module_id, 
		cdev->file_tpinfo.firmware_ver, cdev->file_tpinfo.config_ver, 
		cdev->file_tpinfo.i2c_type,        cdev->file_tpinfo.i2c_addr, 
		cdev->file_tpinfo.tp_name);
}

static ssize_t tsp_ic_tpinfo_show_for_pv(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);

	if(cdev->get_tpinfo) {
		cdev->get_tpinfo(cdev);
	}
	tpd_get_tp_module_name(cdev);
	
	return sprintf(buf, "%s%u config version:0x%x\nManufacturer:%s firmware_version:0x%x\nLcm:%s\n", 
		cdev->ic_tpinfo.tp_name, cdev->ic_tpinfo.chip_part_id, 
		cdev->ic_tpinfo.config_ver, cdev->ic_tpinfo.vendor_name, cdev->ic_tpinfo.firmware_ver, cdev->lcm_info);
}

static ssize_t tsp_file_tpinfo_show_for_pv(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	tpd_get_tp_module_name(cdev);
	
	return sprintf(buf, "%s%u config version:0x%x\nManufacturer:%s firmware_version:0x%x\nLcm:%s\n", 
		cdev->ic_tpinfo.tp_name, cdev->ic_tpinfo.chip_part_id, 
		cdev->ic_tpinfo.config_ver, cdev->ic_tpinfo.vendor_name, cdev->ic_tpinfo.firmware_ver, cdev->lcm_info);
}

//0: tp file version > ic, need upgrade. 1: tp file version <= ic, not upgrade.
static ssize_t tsp_tpinfo_compare_result_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	unsigned int compare_ret;
	
	if(cdev->fw_compare_result == 0) {
		compare_ret = 0;
	} else {
		compare_ret = 1;
	}

	return sprintf(buf, "%u\n",compare_ret);
}

static unsigned char *i2c_test_buffer = NULL;
static unsigned int i2c_test_size = 0;

static ssize_t tsp_i2c_test_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = i2c_test_size;
	char i2c_test_buf1[64], i2c_test_buf2[64];
	
	snprintf(i2c_test_buf1, 64, "%u 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x %s\n", cdev->ic_tpinfo.chip_part_id, 
		cdev->ic_tpinfo.chip_ver, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.firmware_ver, cdev->ic_tpinfo.config_ver, 
		cdev->ic_tpinfo.i2c_type, cdev->ic_tpinfo.i2c_addr, 
		cdev->ic_tpinfo.tp_name);
	if(cdev->get_tpinfo) {
		cdev->get_tpinfo(cdev);
	}

	snprintf(i2c_test_buf2, 64, "%u 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x %s\n", cdev->ic_tpinfo.chip_part_id, 
		cdev->ic_tpinfo.chip_ver, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.firmware_ver, cdev->ic_tpinfo.config_ver, 
		cdev->ic_tpinfo.i2c_type, cdev->ic_tpinfo.i2c_addr, 
		cdev->ic_tpinfo.tp_name);
	if(strncmp(i2c_test_buf1, i2c_test_buf2, 64) == 0 ) {
		memcpy(buf, i2c_test_buffer, i2c_test_size);
	} else {
		memset(i2c_test_buffer, 0, i2c_test_size);
		memcpy(buf, i2c_test_buffer, i2c_test_size);
	}
	printk("tpd %s", i2c_test_buf1);
	printk("tpd %s", i2c_test_buf2);

	printk("tpd: %s read val:0x%x.\n", __func__, buf[0]);

	retval = i2c_test_size;

	return retval;
}

static ssize_t tsp_i2c_test_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	//struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	//printk("tpd: %s write %d byte.\n", __func__, size);

	if(i2c_test_buffer == NULL) {
		i2c_test_buffer = (unsigned char *)vmalloc(256);
		printk("tpd i2c_test_buffer already vmalloc.\n");
	}
	memcpy(i2c_test_buffer, buf, size > 256 ? 256 : size);
	i2c_test_size = size > 256 ? 256 : size;

	printk("tpd: %s write val:0x%x.\n", __func__, i2c_test_buffer[0]);

	return size > 256 ? 256 : size;
}

static ssize_t tsp_lcminfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);

	return sprintf(buf, "%s", cdev->lcm_info);
}

static unsigned char filepath[256];
static int tpd_read_firmware_from_file(struct tpd_classdev_t *cdev, unsigned char * filepath)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic; 
	off_t fsize; 
	loff_t pos;
	int retval = 0;
	mm_segment_t old_fs;

	TPD_DMESG("%s file:%s\n", __func__, filepath);

	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		retval = -1;
		goto out;
	}
	
	inode = pfile->f_dentry->d_inode; 
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size; 

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	
	if(cdev == NULL || 0 != tpd_alloc_buffer(cdev, fsize)) {
		TPD_DMESG("%s memory alloc failed\n", __func__);
		retval = -1;
		goto out;
	}
	vfs_read(pfile, cdev->tp_fw.data, fsize, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);

	cdev->tp_fw.data_len = fsize;

	cdev->status = STATUS_BUF_RDY;
	retval = fsize;
	
out:
	return retval;
}

static ssize_t tsp_force_upgrade_filepath_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//ruct tpd_classdev_t *cdev = dev_get_drvdata(dev);

	return sprintf(buf, "Current firmware file path:%s .\n", filepath);
}

static ssize_t tsp_force_upgrade_filepath_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);

	mutex_lock(&cdev->cmd_mutex);

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", (unsigned char*)buf);
	filepath[size - 1] = '\0';
	{
		int fw_count = 0;
		tpd_free_buffer(cdev);
		fw_count = tpd_read_firmware_from_file(cdev, filepath);
		if(fw_count > 0) {
			tpd_upgrade_firmware(cdev, cdev->cmd, 1);
		} else if(fw_count <= 0) {
			TPD_DMESG("%s: Read file failed.\n", __func__);
			cdev->status = STATUS_FILE_FAILED;
		}
		tpd_free_buffer(cdev);
	}
	
	mutex_unlock(&cdev->cmd_mutex);

	return size;
}

//for tpd test
static ssize_t tsp_test_save_file_path_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	if(cdev->tpd_test_get_save_filepath) {
		retval = cdev->tpd_test_get_save_filepath(cdev, buf);
	}

	return retval;
}

static ssize_t tsp_test_save_file_path_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	if(cdev->tpd_test_set_save_filepath) {
		retval = cdev->tpd_test_set_save_filepath(cdev, buf);
	}

	return count;
}


static ssize_t tsp_test_ini_file_path_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	if(cdev->tpd_test_get_ini_filepath) {
		retval = cdev->tpd_test_get_ini_filepath(cdev, buf);
	}

	return retval;
}

static ssize_t tsp_test_ini_file_path_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	if(cdev->tpd_test_set_ini_filepath) {
		retval = cdev->tpd_test_set_ini_filepath(cdev, buf);
	}

	return count;
}

static ssize_t tsp_test_filename_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	if(cdev->tpd_test_get_filename) {
		retval = cdev->tpd_test_get_filename(cdev, buf);
	}

	return retval;
}

static ssize_t tsp_test_filename_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	if(cdev->tpd_test_set_filename) {
		retval = cdev->tpd_test_set_filename(cdev, buf);
	}

	return count;
}

static ssize_t tsp_test_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	if(cdev->tpd_test_get_cmd) {
		retval = cdev->tpd_test_get_cmd(cdev, buf);
	}

	return retval;
}

static ssize_t tsp_test_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	if(cdev->tpd_test_set_cmd) {
		retval = cdev->tpd_test_set_cmd(cdev, buf);
	}

	return count;
}

static ssize_t tsp_test_node_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	if(cdev->tpd_test_get_node_data) {
		retval = cdev->tpd_test_get_node_data(cdev, buf);
	}

	return retval;
}

static ssize_t tsp_test_node_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	if(cdev->tpd_test_set_node_data_type) {
		retval = cdev->tpd_test_set_node_data_type(cdev, buf);
	}

	return count;
}
static ssize_t tsp_test_channel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	if(cdev->tpd_test_get_channel_info) {
		retval = cdev->tpd_test_get_channel_info(cdev, buf);
	}

	return retval;
}
static ssize_t tsp_test_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	if(cdev->tpd_test_get_result) {
		retval = cdev->tpd_test_get_result(cdev, buf);
	}

	return retval;
}

static DEVICE_ATTR(cmd, 0644, tsp_cmd_show, tsp_cmd_store);
static DEVICE_ATTR(status, 0644, tsp_upg_status_show, tsp_upg_status_store);
static DEVICE_ATTR(tpinfo, 0644, tsp_fw_ic_tpinfo_show, tsp_fw_ic_tpinfo_store);
static DEVICE_ATTR(fileinfo, 0444, tsp_fw_file_tpinfo_show, NULL);
static DEVICE_ATTR(gesture, 0644, tsp_get_bsg_value_show, tsp_get_bsg_value_store);
static DEVICE_ATTR(gesture_enable, 0644, tsp_gesture_enable_show, tsp_gesture_enable_store);
static DEVICE_ATTR(film_mode_enable, 0644, tsp_film_mode_show, tsp_film_mode_store);
static DEVICE_ATTR(i2c_test, 0644, tsp_i2c_test_show, tsp_i2c_test_store);
static DEVICE_ATTR(lcminfo, 0644, tsp_lcminfo_show, NULL);
static DEVICE_ATTR(upg, 0644, tsp_force_upgrade_filepath_show, tsp_force_upgrade_filepath_store);

//for pv tpinfo compare.
static DEVICE_ATTR(ic_info, 0444, tsp_ic_tpinfo_show_for_pv, NULL );
static DEVICE_ATTR(file_info, 0444, tsp_file_tpinfo_show_for_pv,  NULL);
static DEVICE_ATTR(compare_result, 0444, tsp_tpinfo_compare_result_show, NULL);

//for tpd test
static DEVICE_ATTR(tpd_test_result, S_IRUGO|S_IRUSR, tsp_test_result_show, NULL);
static DEVICE_ATTR(tpd_test_channel_setting, S_IRUGO|S_IRUSR, tsp_test_channel_show, NULL);
static DEVICE_ATTR(tpd_test_save_file_path, S_IRUGO|S_IWUSR, tsp_test_save_file_path_show, tsp_test_save_file_path_store);
static DEVICE_ATTR(tpd_test_ini_file_path, S_IRUGO|S_IWUSR, tsp_test_ini_file_path_show, tsp_test_ini_file_path_store);
static DEVICE_ATTR(tpd_test_filename, S_IRUGO|S_IWUSR, tsp_test_filename_show, tsp_test_filename_store);
static DEVICE_ATTR(tpd_test_cmd, S_IRUGO|S_IWUSR, tsp_test_cmd_show, tsp_test_cmd_store);
static DEVICE_ATTR(tpd_test_node_data, S_IRUGO|S_IWUSR, tsp_test_node_data_show, tsp_test_node_data_store);

static struct attribute *tsp_dev_attrs[] = {
	&dev_attr_cmd.attr,
	&dev_attr_status.attr,
	&dev_attr_tpinfo.attr,
	&dev_attr_fileinfo.attr,
	&dev_attr_gesture.attr,
	&dev_attr_gesture_enable.attr,
	&dev_attr_film_mode_enable.attr,
	&dev_attr_i2c_test.attr,
	&dev_attr_lcminfo.attr,
	&dev_attr_upg.attr,
	
	//for pv tpinfo compare.
	&dev_attr_ic_info.attr,
	&dev_attr_file_info.attr,
	&dev_attr_compare_result.attr,

	//for tpd test
	&dev_attr_tpd_test_filename.attr,
	&dev_attr_tpd_test_node_data.attr,
	&dev_attr_tpd_test_cmd.attr,
	&dev_attr_tpd_test_ini_file_path.attr,
	&dev_attr_tpd_test_save_file_path.attr,
	&dev_attr_tpd_test_channel_setting.attr,
	&dev_attr_tpd_test_result.attr,
	NULL,
};

//static struct bin_attribute *tsp_dev_bin_attributes[] = {
//	&bin_attr_button,
//	&bin_attr_info,
//	NULL,
//};

static const struct attribute_group tsp_dev_attribute_group = {
	.attrs = tsp_dev_attrs,
	//.bin_attrs = tsp_dev_bin_attributes,
};

static const struct attribute_group *tsp_dev_attribute_groups[] = {
	&tsp_dev_attribute_group,
	NULL,
};

static ssize_t tsp_fw_data_read(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buffer, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = -1;

	//printk("tpd: %s offset %lld byte, count %ld byte.\n", __func__, offset, count);
	
	if(offset > MAX_BUF_SIZE || cdev->tp_fw.data == NULL) {
		dev_err(cdev->dev, "[TSP]firmware size overflow\n");
		retval = -1;
		goto error;
	}

	if(offset + count > MAX_BUF_SIZE) {
		count = MAX_BUF_SIZE - offset;
	}

	memcpy(buffer, cdev->tp_fw.data + offset, count);

	retval = count;

error:
	return retval;
}

static ssize_t tsp_fw_data_write(struct file *filp, struct kobject *kobj,
				   struct bin_attribute *bin_attr,
				   char *buffer, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = -1;
	int i = 0;

	//printk("tpd: %s offset %lld byte, count %ld byte.\n", __func__, offset, count);

	if(0 != tpd_alloc_buffer(cdev, MAX_BUF_SIZE)) {
		retval = -1;
		goto error;
	}

	if(offset + count > MAX_BUF_SIZE) {
		dev_err(cdev->dev, "[TSP]firmware size overflow\n");
		retval = -1;
		goto error;
	}
	memcpy(cdev->tp_fw.data + offset, buffer, count);
	cdev->tp_fw.data_len= cdev->tp_fw.data_len+ count;
	retval = count;

	printk("tpd: %s:", __func__);
	for(i = 0; i < 16; i++) {
		printk("tpd: 0x%x ", cdev->tp_fw.data[i]);
	}
	printk("tpd: \n");
	cdev->status = STATUS_BUF_ING;

error:
	return retval;
}

static struct bin_attribute firmware_attr_data = {
	.attr = { .name = "data", .mode = 0644 },
	.size = 0,
	.read = tsp_fw_data_read,
	.write = tsp_fw_data_write,
};

static ssize_t tsp_fw_reg_read(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buffer, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = -1;

	//printk("tpd: %s offset %lld byte, count %ld byte.\n", __func__, offset, count);
	
	if(cdev->read_block) {
		cdev->read_block(cdev, offset, buffer , count);
	}

	retval = count;

	return retval;
}

static ssize_t tsp_fw_reg_write(struct file *filp, struct kobject *kobj,
				   struct bin_attribute *bin_attr,
				   char *buffer, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = -1;

	//printk("tpd: %s offset %lld byte, count %ld byte.\n", __func__, offset, count);

	if(cdev->write_block) {
		cdev->write_block(cdev, offset, buffer, count);
	}

	retval = count;

	return retval;
}

static struct bin_attribute firmware_attr_reg = {
	.attr = { .name = "reg", .mode = 0644 },
	.size = 0,
	.read = tsp_fw_reg_read,
	.write = tsp_fw_reg_write,
};

//for shanghai && xi'an emode interface.
#ifdef XIAN_COMMON_INTERFACE
#ifdef CONFIG_PROC_FS
#define	LCD_PROC_FILE	"driver/lcd_id"
#define	TPD_PROC_FILE	"driver/tsc_id"

static struct proc_dir_entry *lcd_proc_entry;
static struct proc_dir_entry *tpd_proc_entry;
static struct proc_dir_entry *tpd_shanghai_proc_entry;

static int tpd_proc_show(struct seq_file *seq, void *v)
{
	struct tpd_classdev_t *cdev = &tpd_fw_cdev;
	int ret = -1;

	pr_notice("%s", __func__);

	if(cdev->get_tpinfo) {
		cdev->get_tpinfo(cdev);
	}
	tpd_get_tp_module_name(cdev);

	ret = seq_printf(seq, "TP module:%s(0x%x): IC type:%s; I2C address:0x%x; Firmware version:0x%x; Config version:0x%x\n", 
		cdev->ic_tpinfo.vendor_name, cdev->ic_tpinfo.module_id, 
		cdev->ic_tpinfo.tp_name, cdev->ic_tpinfo.i2c_addr,
		cdev->ic_tpinfo.firmware_ver, cdev->ic_tpinfo.config_ver);
	if(ret < 0) {
		pr_notice("%s seq write error.\n", __func__);
	}
	return 0;
}

static int tpd_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, tpd_proc_show, NULL);
}

static struct file_operations tpd_proc_ops = {
	.owner		= THIS_MODULE,
	.open		= tpd_seq_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	//.release	= seq_release_ops,
};

static void create_tpd_proc_entry(void)
{
	tpd_proc_entry = proc_create(TPD_PROC_FILE, 0644, NULL, &tpd_proc_ops);
	if (tpd_proc_entry) {
		printk(KERN_INFO "create proc file sucess!\n");
	} else
		printk(KERN_INFO "create proc file failed!\n");
	//for shanghai emode interface.
	tpd_shanghai_proc_entry = proc_create("touchscreen_info", 0644, NULL, &tpd_proc_ops);
	if (tpd_shanghai_proc_entry) {
		printk(KERN_INFO "create proc file sucess!\n");
	} else
		printk(KERN_INFO "create proc file failed!\n");
}

static int lcd_proc_show(struct seq_file *seq, void *v)
{
	struct tpd_classdev_t *cdev = &tpd_fw_cdev;
	int ret = -1;

	pr_notice("%s", __func__);
	
	ret = seq_printf(seq, "IC:%s; Vendor:%s; Resolution:%d*%d.\n", cdev->lcm_chip_info, cdev->lcm_info, 720, 1280);
	if(ret < 0) {
		pr_notice("%s seq write error.\n", __func__);
	}
	return 0;
}

static int lcd_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, lcd_proc_show, NULL);
}

static const struct file_operations lcd_proc_ops = {
	.owner		= THIS_MODULE,
	.open		= lcd_seq_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	//.release	= seq_release_ops,
};

static void create_lcd_proc_entry(void)
{
	lcd_proc_entry = proc_create(LCD_PROC_FILE, 0644, NULL, &lcd_proc_ops);
	if (lcd_proc_entry) {
		printk(KERN_INFO "create proc file sucess!\n");
	} else
		printk(KERN_INFO "create proc file failed!\n");
}
#endif

static struct kobject *tpd_wake_gesture_kobj;
static ssize_t tpd_wake_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = &tpd_fw_cdev;
	int retval = 0;

	mutex_lock(&cdev->cmd_mutex);
	printk("tpd: %s val:%d.\n", __func__, cdev->b_gesture_enable);

	retval = snprintf(buf, 32, "%d,\n", cdev->b_gesture_enable);
	mutex_unlock(&cdev->cmd_mutex);

	return retval;
}

static ssize_t tpd_wake_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct tpd_classdev_t *cdev = &tpd_fw_cdev;
	
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;
	
	printk("tpd: %s val %d.\n", __func__, input);

	mutex_lock(&cdev->cmd_mutex);
	if(cdev->wake_gesture) {
		cdev->wake_gesture(cdev, input);
	}
	if(cdev->wake_gesture_mode){
		cdev->wake_gesture_mode(cdev, input);
		}
	cdev->b_gesture_enable = input;
	mutex_unlock(&cdev->cmd_mutex);
	
	return count;
}
static ssize_t tpd_film_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = &tpd_fw_cdev;
	int retval = 0;

	mutex_lock(&cdev->cmd_mutex);
	printk("tpd: %s val:%d.\n", __func__, cdev->b_film_mode_enable);

	retval = snprintf(buf, 32, "%d,\n", cdev->b_film_mode_enable);
	mutex_unlock(&cdev->cmd_mutex);

	return retval;
}

static ssize_t tpd_film_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct tpd_classdev_t *cdev = &tpd_fw_cdev;
	
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;
	
	printk("tpd: %s val %d.\n", __func__, input);

	mutex_lock(&cdev->cmd_mutex);
	if(cdev->wake_film_mode) {
		cdev->wake_film_mode(cdev, input);
	}
	cdev->b_film_mode_enable = input;
	mutex_unlock(&cdev->cmd_mutex);
	
	return count;
}
static DEVICE_ATTR(wake_gesture, 0664/*S_IRUGO|S_IWUSR|S_IWGRP*/,  tpd_wake_gesture_show,  tpd_wake_gesture_store);
static DEVICE_ATTR(wake_film_mode, 0664/*S_IRUGO|S_IWUSR|S_IWGRP*/,  tpd_film_mode_show,  tpd_film_mode_store); 
/* add your attr in here*/
static struct attribute *tpd_bsg_attributes[] = {
	&dev_attr_wake_gesture.attr,
	&dev_attr_wake_film_mode.attr,
	NULL
};

static struct attribute_group tpd_bsg_attribute_group = {
	.attrs = tpd_bsg_attributes
};


int tpd_create_wake_gesture_sysfs(void)
{
	int err;

	tpd_wake_gesture_kobj = kobject_create_and_add("tp_wake_gesture", NULL);
	if (!tpd_wake_gesture_kobj) {
		err = -EINVAL;
		printk("%s() - ERROR Unable to create tpd_wake_gesture_kobj.\n", __func__);
		return -EIO;
	}

	err = sysfs_create_group(tpd_wake_gesture_kobj, &tpd_bsg_attribute_group);
	if (0 != err) 
	{
		printk("%s - ERROR sysfs_create_group failed.\n",__func__);
		//sysfs_remove_group(tpd_wake_gesture_kobj, &tpd_bsg_attribute_group);
		kobject_put(tpd_wake_gesture_kobj);
		return -EIO;
	} else {
		printk("%s succeeded.\n",__func__);
	}
	
	return err;
}

int tpd_remove_wake_gesture_sysfs(void)
{
	sysfs_remove_group(tpd_wake_gesture_kobj, &tpd_bsg_attribute_group);
	kobject_put(tpd_wake_gesture_kobj);
	return 0;
}
#endif


/**
 * tpd_classdev_register - register a new object of tpd_classdev_t class.
 * @parent: The device to register.
 * @tsp_fw_cdev: the tpd_classdev_t structure for this device.
 */
int tpd_classdev_register(struct device *parent, struct tpd_classdev_t *tsp_fw_cdev)
{
	int error = 0;
	
	tsp_fw_cdev->dev = device_create(tsp_fw_class, NULL, 0, tsp_fw_cdev,
					  "%s", tsp_fw_cdev->name);
	if (IS_ERR(tsp_fw_cdev->dev))
		return PTR_ERR(tsp_fw_cdev->dev);

	error = device_create_bin_file(tsp_fw_cdev->dev, &firmware_attr_data);
	if (error) {
		dev_err(tsp_fw_cdev->dev, "%s: sysfs_create_bin_file failed\n", __func__);
	}
	error = device_create_bin_file(tsp_fw_cdev->dev, &firmware_attr_reg);
	if (error) {
		dev_err(tsp_fw_cdev->dev, "%s: sysfs_create_bin_file failed\n", __func__);
	}

	/* add to the list of tp_firmware */
	down_write(&tp_firmware_list_lock);
	list_add_tail(&tsp_fw_cdev->node, &tp_firmware_list);
	up_write(&tp_firmware_list_lock);

	mutex_init(&tsp_fw_cdev->upgrade_mutex);
	mutex_init(&tsp_fw_cdev->cmd_mutex);

	tpd_create_wake_gesture_sysfs();
#ifdef CONFIG_PROC_FS
	create_tpd_proc_entry();
#endif

	printk("tpd: Registered tsp_fw device: %s\n",
			tsp_fw_cdev->name);

	return 0;
}
EXPORT_SYMBOL_GPL(tpd_classdev_register);

/**
 * tpd_classdev_unregister - unregisters a object of tsp_fw_properties class.
 * @tsp_fw_cdev: the tsp_fw device to unregister
 *
 * Unregisters a previously registered via tpd_classdev_register object.
 */
void tpd_classdev_unregister(struct tpd_classdev_t *tsp_fw_cdev)
{
	device_unregister(tsp_fw_cdev->dev);

	down_write(&tp_firmware_list_lock);
	list_del(&tsp_fw_cdev->node);
	up_write(&tp_firmware_list_lock);
}
EXPORT_SYMBOL_GPL(tpd_classdev_unregister);

//extern char* mtkfb_find_lcm_driver(void);
static void get_lcm_info(struct tpd_classdev_t *cdev)
{
	char * tmp_name = NULL;
	char lcm_info[16], lcm_chip_info[16];
	
	//tmp_name = mtkfb_find_lcm_driver();
	tmp_name = saved_command_line;
	//printk("check lcm name = %s", tmp_name);
	if(NULL != strstr(tmp_name, "txd") || NULL != strstr(tmp_name, "tongxingda") ){
		strncpy(lcm_info, "TongXingDa", 15);
	} else if(NULL != strstr(tmp_name, "lead") || NULL != strstr(tmp_name, "lide")){
		strncpy(lcm_info, "Lead", 15);
	} else if(NULL != strstr(tmp_name, "tianma")){
		strncpy(lcm_info, "TianMa", 15);
	} else if(NULL != strstr(tmp_name, "sanxing")){
		strncpy(lcm_info, "SanXing", 15);
	} else if(NULL != strstr(tmp_name, "hotech")){
		strncpy(lcm_info, "HeLiTai", 15);
	} else if(NULL != strstr(tmp_name, "dijing")){
		strncpy(lcm_info, "DiJing", 15);
	} else if(NULL != strstr(tmp_name, "yassy")){
		strncpy(lcm_info, "YaShi", 15);
	} else if(NULL != strstr(tmp_name, "mdss_dsi_truly_1080p_video")){
		strncpy(lcm_info, "Boe", 15); //only for boe project
	} else if(NULL != strstr(tmp_name, "chuangwei") || NULL != strstr(tmp_name, "skyworth")){
		strncpy(lcm_info, "ChuangWei", 15);
	} else if(NULL != strstr(tmp_name, "liansi") || NULL != strstr(tmp_name, "lianchuang")){
		strncpy(lcm_info, "LianChuang", 15);
	} else {
		strncpy(lcm_info, "unkown", 15);
	}

	if(NULL != strstr(tmp_name, "8394") ){
		strncpy(lcm_chip_info, "HX8394", 15);
	} else if(NULL != strstr(tmp_name, "9881")){
		strncpy(lcm_chip_info, "ILI9881", 15);
	} else if(NULL != strstr(tmp_name, "8399")){
		strncpy(lcm_chip_info, "HX8399", 15);
	} else if(NULL != strstr(tmp_name, "35532")){
		strncpy(lcm_chip_info, "NT35532", 15);
	} else if(NULL != strstr(tmp_name, "68200")){
		strncpy(lcm_chip_info, "RM68200", 15);
	} else {
		strncpy(lcm_chip_info, "lcd chip", 15);
	}

	strncpy(cdev->lcm_info, lcm_info, 64);
	strncpy(cdev->lcm_chip_info, lcm_chip_info, 64);
}

static int __init tpd_class_init(void)
{
	tsp_fw_class = class_create(THIS_MODULE, "tsp_fw");
	if (IS_ERR(tsp_fw_class))
		return PTR_ERR(tsp_fw_class);
	tsp_fw_class->dev_groups = tsp_dev_attribute_groups;

	tpd_fw_cdev.tp_fw.data = NULL;
	tpd_fw_cdev.tp_fw.data_len = 0;
	tpd_fw_cdev.tp_fw.length = 0;
	tpd_fw_cdev.fw_compare_result = 1;

	get_lcm_info(&tpd_fw_cdev);
#ifdef CONFIG_PROC_FS
	create_lcd_proc_entry();
#endif

	//for tp probe after se init.
	tpd_fw_cdev.name = "touchscreen";
	tpd_fw_cdev.private = NULL;
	tpd_fw_cdev.flash_fw = NULL;
	tpd_fw_cdev.read_block = NULL;
	tpd_fw_cdev.write_block = NULL;
	tpd_fw_cdev.compare_tp_version = NULL;
	tpd_fw_cdev.get_tpinfo = NULL;
	
	tpd_fw_cdev.get_gesture = NULL;
	tpd_fw_cdev.wake_gesture = NULL;
	tpd_fw_cdev.wake_gesture_mode= NULL;
	
	//for tpd test
	tpd_fw_cdev.tpd_test_set_save_filepath = NULL;
	tpd_fw_cdev.tpd_test_get_save_filepath = NULL;
	tpd_fw_cdev.tpd_test_set_ini_filepath = NULL;
	tpd_fw_cdev.tpd_test_get_ini_filepath = NULL;
	tpd_fw_cdev.tpd_test_set_filename = NULL;
	tpd_fw_cdev.tpd_test_get_filename = NULL;
	tpd_fw_cdev.tpd_test_set_cmd = NULL;
	tpd_fw_cdev.tpd_test_get_cmd = NULL;
	tpd_fw_cdev.tpd_test_set_node_data_type = NULL;
	tpd_fw_cdev.tpd_test_get_node_data = NULL;
	tpd_fw_cdev.tpd_test_get_channel_info = NULL;
	tpd_fw_cdev.tpd_test_get_result = NULL;

	tpd_classdev_register(NULL, &tpd_fw_cdev);
	
	return 0;
}

static void __exit tpd_class_exit(void)
{
	tpd_free_buffer(&tpd_fw_cdev);
	class_destroy(tsp_fw_class);
}

subsys_initcall(tpd_class_init);
module_exit(tpd_class_exit);

MODULE_AUTHOR("John Lenz, Richard Purdie");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TSP FW Class Interface");


