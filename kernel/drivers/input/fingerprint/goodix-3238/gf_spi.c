/*
 * TEE driver for goodix fingerprint sensor
 * Copyright (C) 2016 Goodix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)		KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

/*added by chenhui for proc info node begin*/
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
/*added by chenhui for proc info node end*/

#define VER_MAJOR   1
#define VER_MINOR   0
#define PATCH_LEVEL 1

#define GF_SPIDEV_NAME     "goodix,fingerprint"
/*device name after register in charater*/
#define GF_DEV_NAME            "goodix_fp"
#define	GF_INPUT_NAME	    "goodix_fp"	/*"qwerty" */

#define	CHRD_DRIVER_NAME	"goodix_fp_spi"
#define	CLASS_NAME		    "goodix_fp"

#define N_SPI_MINORS		32	/* ... up to 256 */

#define GF_TTW_HOLD_TIME 1000 //Alan, add wakelock

/*added by chenhui for proc info node begin*/
#define PROC_FPDSENSOR_DIR  "fpdsensor"
#define PROC_FPDSENSOR_NAME  "fpdsensor/chip_info"

static struct proc_dir_entry *fpdsensor_proc_dir_entry = NULL;
static ssize_t fpdsensor_config_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos);
static int fpdsensor_config_proc_open(struct inode *inode, struct file *file);
static struct list_head head;

#define MAX_CHIP_INFO_LEN 32
static char chip_info_buffer[MAX_CHIP_INFO_LEN] = {0};

static const struct file_operations info_proc_file_ops = {	
	.owner	 = THIS_MODULE,    
	.open    = fpdsensor_config_proc_open,    
	.read	 = seq_read,
	.write   = fpdsensor_config_proc_write,	
	.llseek	 = seq_lseek,	
	.release = seq_release,	
};
/*added by chenhui for proc info node end*/


static int SPIDEV_MAJOR;

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct gf_dev gf;

static void gf_enable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		pr_warn("IRQ has been enabled.\n");
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_enabled = 1;
	}
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		gf_dev->irq_enabled = 0;
		disable_irq(gf_dev->irq);
	} else {
		pr_warn("IRQ has been disabled.\n");
	}
}

#ifdef AP_CONTROL_CLK
static long spi_clk_max_rate(struct clk *clk, unsigned long rate)
{
	long lowest_available, nearest_low, step_size, cur;
	long step_direction = -1;
	long guess = rate;
	int max_steps = 10;

	cur = clk_round_rate(clk, rate);
	if (cur == rate)
		return rate;

	/* if we got here then: cur > rate */
	lowest_available = clk_round_rate(clk, 0);
	if (lowest_available > rate)
		return -EINVAL;

	step_size = (rate - lowest_available) >> 1;
	nearest_low = lowest_available;

	while (max_steps-- && step_size) {
		guess += step_size * step_direction;
		cur = clk_round_rate(clk, guess);

		if ((cur < rate) && (cur > nearest_low))
			nearest_low = cur;
		/*
		 * if we stepped too far, then start stepping in the other
		 * direction with half the step size
		 */
		if (((cur > rate) && (step_direction > 0))
				|| ((cur < rate) && (step_direction < 0))) {
			step_direction = -step_direction;
			step_size >>= 1;
		}
	}
	return nearest_low;
}

static void spi_clock_set(struct gf_dev *gf_dev, int speed)
{
	long rate;
	int rc;

	rate = spi_clk_max_rate(gf_dev->core_clk, speed);
	if (rate < 0) {
		pr_info("%s: no match found for requested clock frequency:%d",
				__func__, speed);
		return;
	}

	rc = clk_set_rate(gf_dev->core_clk, rate);
}

static int gfspi_ioctl_clk_init(struct gf_dev *data)
{
	pr_debug("%s: enter\n", __func__);

	data->clk_enabled = 0;
	data->core_clk = clk_get(&data->spi->dev, "core_clk");
	if (IS_ERR_OR_NULL(data->core_clk)) {
		pr_err("%s: fail to get core_clk\n", __func__);
		return -EPERM;
	}
	data->iface_clk = clk_get(&data->spi->dev, "iface_clk");
	if (IS_ERR_OR_NULL(data->iface_clk)) {
		pr_err("%s: fail to get iface_clk\n", __func__);
		clk_put(data->core_clk);
		data->core_clk = NULL;
		return -ENOENT;
	}
	return 0;
}

static int gfspi_ioctl_clk_enable(struct gf_dev *data)
{
	int err;

	pr_debug("%s: enter\n", __func__);

	if (data->clk_enabled)
		return 0;

	err = clk_prepare_enable(data->core_clk);
	if (err) {
		pr_err("%s: fail to enable core_clk\n", __func__);
		return -EPERM;
	}

	err = clk_prepare_enable(data->iface_clk);
	if (err) {
		pr_err("%s: fail to enable iface_clk\n", __func__);
		clk_disable_unprepare(data->core_clk);
		return -ENOENT;
	}

	data->clk_enabled = 1;

	return 0;
}

static int gfspi_ioctl_clk_disable(struct gf_dev *data)
{
	pr_debug("%s: enter\n", __func__);

	if (!data->clk_enabled)
		return 0;

	clk_disable_unprepare(data->core_clk);
	clk_disable_unprepare(data->iface_clk);
	data->clk_enabled = 0;

	return 0;
}

static int gfspi_ioctl_clk_uninit(struct gf_dev *data)
{
	pr_debug("%s: enter\n", __func__);

	if (data->clk_enabled)
		gfspi_ioctl_clk_disable(data);

	if (!IS_ERR_OR_NULL(data->core_clk)) {
		clk_put(data->core_clk);
		data->core_clk = NULL;
	}

	if (!IS_ERR_OR_NULL(data->iface_clk)) {
		clk_put(data->iface_clk);
		data->iface_clk = NULL;
	}

	return 0;
}
#endif

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = &gf;
	struct gf_key gf_key;
#if defined(SUPPORT_NAV_EVENT)
	gf_nav_event_t nav_event = GF_NAV_NONE;
	uint32_t nav_input = 0;
#endif
	int retval = 0;
	u8 netlink_route = NETLINK_TEST;
	struct gf_ioc_chip_info info;
	uint32_t key_input = 0;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENODEV;

	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (retval)
		return -EFAULT;

	if (gf_dev->device_available == 0) {
		if ((cmd == GF_IOC_ENABLE_POWER) || (cmd == GF_IOC_DISABLE_POWER)) {
			pr_info("power cmd\n");
		} else {
			pr_info("Sensor is power off currently. \n");
			return -ENODEV;
		}
	}

	switch (cmd) {
	case GF_IOC_INIT:
		pr_info("%s GF_IOC_INIT .\n", __func__);
		if (copy_to_user((void __user *)arg, (void *)&netlink_route, sizeof(u8))) {
			retval = -EFAULT;
			break;
		}
		break;
	case GF_IOC_EXIT:
		pr_info("%s GF_IOC_EXIT .\n", __func__);
		break;
	case GF_IOC_DISABLE_IRQ:
		pr_info("%s GF_IOC_DISABEL_IRQ .\n", __func__);
		gf_disable_irq(gf_dev);
		break;
	case GF_IOC_ENABLE_IRQ:
		pr_info("%s GF_IOC_ENABLE_IRQ .\n", __func__);
		gf_enable_irq(gf_dev);
		break;
	case GF_IOC_RESET:
		pr_info("%s GF_IOC_RESET. \n", __func__);
		gf_hw_reset(gf_dev, 3);
		break;
	case GF_IOC_INPUT_KEY_EVENT:
		if (copy_from_user(&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
			pr_info("Failed to copy input key event from user to kernel\n");
			retval = -EFAULT;
			break;
		}

		if (GF_KEY_HOME == gf_key.key) {
			key_input = GF_KEY_INPUT_HOME;
		} else if (GF_KEY_POWER == gf_key.key) {
			key_input = GF_KEY_INPUT_POWER;
		} else if (GF_KEY_CAMERA == gf_key.key) {
			key_input = GF_KEY_INPUT_CAMERA;
		} else {
			/* add special key define */
			key_input = gf_key.key;
		}
		pr_info("%s: received key event[%d], key=%d, value=%d\n",
				__func__, key_input, gf_key.key, gf_key.value);

		if ((GF_KEY_POWER == gf_key.key || GF_KEY_CAMERA == gf_key.key) && (gf_key.value == 1)) {
			input_report_key(gf_dev->input, key_input, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, key_input, 0);
			input_sync(gf_dev->input);
		}

		if (GF_KEY_HOME == gf_key.key) {
			input_report_key(gf_dev->input, key_input, gf_key.value);
			input_sync(gf_dev->input);
		}

		break;
#if defined(SUPPORT_NAV_EVENT)
	case GF_IOC_NAV_EVENT:
		pr_info("%s nav event\n", __func__);
		if (copy_from_user(&nav_event, (gf_nav_event_t *)arg, sizeof(gf_nav_event_t))) {
			pr_info("Failed to copy nav event from user to kernel\n");
			retval = -EFAULT;
			break;
		}

		switch (nav_event) {
		case GF_NAV_FINGER_DOWN:
			pr_info("%s nav finger down\n", __func__);
			break;

		case GF_NAV_FINGER_UP:
			pr_info("%s nav finger up\n", __func__);
			break;

		case GF_NAV_DOWN:
			//nav_input = GF_NAV_INPUT_DOWN;
#ifdef	CONFIG_FINGERPRINT_GF3238_NAV		
			nav_input = GF_NAV_INPUT_UP;	
#else
			nav_input = GF_NAV_INPUT_DOWN;	
#endif					
			pr_info("%s nav input %d\n", __func__, nav_input);
			break;

		case GF_NAV_UP:
#ifdef	CONFIG_FINGERPRINT_GF3238_NAV		
			nav_input = GF_NAV_INPUT_DOWN;
#else			
			nav_input = GF_NAV_INPUT_UP;
#endif			
			pr_info("%s nav input %d\n", __func__, nav_input);
			break;

		case GF_NAV_LEFT:
#ifdef	CONFIG_FINGERPRINT_GF3238_NAV			
			nav_input = GF_NAV_INPUT_RIGHT;	
#else
			nav_input = GF_NAV_INPUT_LEFT;//GF_NAV_INPUT_RIGHT;
#endif			
			pr_info("%s nav input %d\n", __func__, nav_input);
			break;

		case GF_NAV_RIGHT:
#ifdef	CONFIG_FINGERPRINT_GF3238_NAV			
			nav_input = GF_NAV_INPUT_LEFT;
#else
			nav_input = GF_NAV_INPUT_RIGHT;//GF_NAV_INPUT_LEFT;	
#endif			
			pr_info("%s nav input %d\n", __func__, nav_input);
			break;

		case GF_NAV_CLICK:
			//nav_input = GF_NAV_INPUT_CLICK;
			pr_info("%s nav click \n", __func__);
			break;

		case GF_NAV_HEAVY:
			//nav_input = GF_NAV_INPUT_HEAVY;
			break;

		case GF_NAV_LONG_PRESS:
			//nav_input = GF_NAV_INPUT_LONG_PRESS;
			break;

		case GF_NAV_DOUBLE_CLICK:
			//nav_input = GF_NAV_INPUT_DOUBLE_CLICK;
			break;

		default:
			pr_info("%s: not support nav event nav_event: %d ======\n", __func__, nav_event);
			break;
		}

		if ((nav_event != GF_NAV_FINGER_DOWN) && (nav_event != GF_NAV_FINGER_UP)) {
			input_report_key(gf_dev->input, nav_input, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, nav_input, 0);
			input_sync(gf_dev->input);
		}
		break;
#endif

	case GF_IOC_ENABLE_SPI_CLK:
		pr_info("%s GF_IOC_ENABLE_SPI_CLK. \n", __func__);
#ifdef AP_CONTROL_CLK
		gfspi_ioctl_clk_enable(gf_dev);
#else
		pr_info("Doesn't support control clock.\n");
#endif
		break;
	case GF_IOC_DISABLE_SPI_CLK:
		pr_info("%s GF_IOC_DISABLE_SPI_CLK. \n", __func__);
#ifdef AP_CONTROL_CLK
		gfspi_ioctl_clk_disable(gf_dev);
#else
		pr_info("Doesn't support control clock.\n");
#endif
		break;
	case GF_IOC_ENABLE_POWER:
		pr_info("%s GF_IOC_ENABLE_POWER. \n", __func__);
		if (gf_dev->device_available == 1)
			pr_info("Sensor has already powered-on.\n");
		else
			gf_power_on(gf_dev);
		gf_dev->device_available = 1;
		break;
	case GF_IOC_DISABLE_POWER:
		pr_info("%s GF_IOC_DISABLE_POWER. \n", __func__);
		if (gf_dev->device_available == 0)
			pr_info("Sensor has already powered-off.\n");
		else
			gf_power_off(gf_dev);
		gf_dev->device_available = 0;
		break;
	case GF_IOC_ENTER_SLEEP_MODE:
		pr_info("%s GF_IOC_ENTER_SLEEP_MODE. \n", __func__);
		break;
	case GF_IOC_GET_FW_INFO:
		pr_info("%s GF_IOC_GET_FW_INFO. \n", __func__);
		break;
	case GF_IOC_REMOVE:
		pr_info("%s GF_IOC_REMOVE. \n", __func__);
		break;
	case GF_IOC_CHIP_INFO:
		pr_info("%s GF_IOC_CHIP_INFO. \n", __func__);
		if (copy_from_user(&info, (struct gf_ioc_chip_info *)arg, sizeof(struct gf_ioc_chip_info))) {
			retval = -EFAULT;
			break;
		}
		pr_info(" vendor_id : 0x%x \n", info.vendor_id);
		pr_info(" mode : 0x%x \n", info.mode);
		pr_info(" operation: 0x%x \n", info.operation);
		break;
	default:
		pr_info("Unsupport cmd:0x%x \n", cmd);
		break;
	}

	return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif /*CONFIG_COMPAT*/

static irqreturn_t gf_irq(int irq, void *handle)
{
	struct gf_dev *gf_dev = &gf;
#if defined(GF_NETLINK_ENABLE)
    char temp = GF_NET_EVENT_IRQ;
    sendnlmsg(&temp);
#elif defined (GF_FASYNC)
	struct gf_dev *gf_dev = &gf;
	if (gf_dev->async)
		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif

	wake_lock_timeout(&gf_dev->ttw_wl, msecs_to_jiffies(GF_TTW_HOLD_TIME));//Alan, add wakelock

	return IRQ_HANDLED;
}

static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devt == inode->i_rdev) {
			pr_info("Found\n");
			status = 0;
			break;
		}
	}

	if (status == 0) {
		if (status == 0) {
			gf_dev->users++;
			filp->private_data = gf_dev;
			nonseekable_open(inode, filp);
			pr_info("Succeed to open device. irq = %d\n",
					gf_dev->irq);
			if (gf_dev->users == 1)
				gf_enable_irq(gf_dev);
	        gf_power_on(gf_dev);					
			gf_hw_reset(gf_dev, 3);	
			gf_dev->device_available = 1;
		}
	} else {
		pr_info("No device for minor %d\n", iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
	struct gf_dev *gf_dev = filp->private_data;
	int ret;

	ret = fasync_helper(fd, filp, mode, &gf_dev->async);
	pr_info("ret = %d\n", ret);
	return ret;
}
#endif

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = 0;

	mutex_lock(&device_list_lock);
	gf_dev = filp->private_data;
	filp->private_data = NULL;

	/*last close?? */
	gf_dev->users--;
	if (!gf_dev->users) {

		pr_info("disble_irq. irq = %d\n", gf_dev->irq);
		gf_disable_irq(gf_dev);
		/*power off the sensor*/
		gf_dev->device_available = 0;
		gf_power_off(gf_dev);
	}
	mutex_unlock(&device_list_lock);
	return status;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif /*CONFIG_COMPAT*/
	.open = gf_open,
	.release = gf_release,
#ifdef GF_FASYNC
	.fasync = gf_fasync,
#endif
};

static int goodix_fb_state_chg_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct gf_dev *gf_dev;
	struct fb_event *evdata = data;
	unsigned int blank;
	char temp = 0;

	if (val != FB_EARLY_EVENT_BLANK)
		return 0;
	pr_info("[info] %s go to the goodix_fb_state_chg_callback value = %d\n",
			__func__, (int)val);
	gf_dev = container_of(nb, struct gf_dev, notifier);
	if (evdata && evdata->data && val == FB_EARLY_EVENT_BLANK && gf_dev) {
		blank = *(int *)(evdata->data);
		switch (blank) {
		case FB_BLANK_POWERDOWN:
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 1;
#if defined(GF_NETLINK_ENABLE)
				temp = GF_NET_EVENT_FB_BLACK;
				sendnlmsg(&temp);
#elif defined (GF_FASYNC)
				if (gf_dev->async) {
					kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
				}
#endif
			}
			break;
		case FB_BLANK_UNBLANK:
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 0;
#if defined(GF_NETLINK_ENABLE)
				temp = GF_NET_EVENT_FB_UNBLACK;
				sendnlmsg(&temp);
#elif defined (GF_FASYNC)
				if (gf_dev->async) {
					kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
				}
#endif
			}
			break;
		default:
			pr_info("%s defalut\n", __func__);
			break;
		}
	}
	return NOTIFY_OK;
}

static struct notifier_block goodix_noti_block = {
	.notifier_call = goodix_fb_state_chg_callback,
};

/*added by chenhui for proc node begin*/
static void *fpdsensor_config_proc_start(struct seq_file *m, loff_t *pos)
{	
	return seq_list_start_head(&head, *pos);
}

static void *fpdsensor_config_proc_next(struct seq_file *p, void *v, loff_t *pos)
{	
	return seq_list_next(v, &head, pos);
}

static void fpdsensor_config_proc_stop(struct seq_file *m, void *v)
{

}

static int fpdsensor_config_proc_show(struct seq_file *m, void *v)
{
	pr_err("fpdsensor_config_proc_show time!\n");
    seq_printf(m,"fpd chip info: %s\n", chip_info_buffer); 		   
	return 0;
}


static const struct seq_operations proc_config_ops = {	
	.start = fpdsensor_config_proc_start,	
	.next  = fpdsensor_config_proc_next,	
	.stop  = fpdsensor_config_proc_stop,	
	.show  = fpdsensor_config_proc_show,
};

static int fpdsensor_config_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &proc_config_ops);
}

static ssize_t fpdsensor_config_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
		
	if(NULL == buffer)
	{
		pr_err("buffer is NULL\n");
		return -1;  	
	}

	pr_err("gf chip info!!");	

	if (copy_from_user(chip_info_buffer, buffer, count)) {
		pr_err("copy_from_user mem failed\n");	
        return EFAULT;  
	}
	
    return count;
}

static void remove_procfs_interfaces(void)
{
	remove_proc_entry(PROC_FPDSENSOR_NAME, fpdsensor_proc_dir_entry);
	remove_proc_entry(PROC_FPDSENSOR_DIR, NULL);
}
/*added by chenhui for proc node end*/

static struct class *gf_class;
#if defined(USE_SPI_BUS)
static int gf_probe(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_probe(struct platform_device *pdev)
#endif
{
	struct gf_dev *gf_dev = &gf;
	int status = -EINVAL;
	unsigned long minor;
	int ret;

    printk("%s %d \n", __func__, __LINE__);
	/* Initialize the driver data */
	INIT_LIST_HEAD(&gf_dev->device_entry);
#if defined(USE_SPI_BUS)
	gf_dev->spi = spi;
#elif defined(USE_PLATFORM_BUS)
	gf_dev->spi = pdev;
#endif
	gf_dev->irq_gpio = -EINVAL;
	gf_dev->reset_gpio = -EINVAL;
/*added by chenhui for  pwr, begin*/	
	gf_dev->pwr3v3_gpio = -EINVAL;
	gf_dev->pwrvdd_gpio = -EINVAL;	
/*added by chenhui for  pwr, end*/		
	gf_dev->device_available = 0;
	gf_dev->fb_black = 0;
	gf_dev->irq_enabled = 0;

	if (gf_parse_dts(gf_dev))
		goto error;

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_class, &gf_dev->spi->dev, gf_dev->devt,
				gf_dev, GF_DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&gf_dev->spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}

	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
	} else {
		gf_dev->devt = 0;
	}
	mutex_unlock(&device_list_lock);

	if (status == 0) {
		/*input device subsystem */
		gf_dev->input = input_allocate_device();
		if (gf_dev->input == NULL) {
			pr_info("%s, Failed to allocate input device.\n", __func__);
			status = -ENOMEM;
		}

		__set_bit(EV_KEY, gf_dev->input->evbit);
		__set_bit(GF_KEY_INPUT_HOME, gf_dev->input->keybit);

		__set_bit(GF_KEY_INPUT_MENU, gf_dev->input->keybit);
		__set_bit(GF_KEY_INPUT_BACK, gf_dev->input->keybit);
		__set_bit(GF_KEY_INPUT_POWER, gf_dev->input->keybit);

#if defined(SUPPORT_NAV_EVENT)
		__set_bit(GF_NAV_INPUT_UP, gf_dev->input->keybit);
		__set_bit(GF_NAV_INPUT_DOWN, gf_dev->input->keybit);
		__set_bit(GF_NAV_INPUT_RIGHT, gf_dev->input->keybit);
		__set_bit(GF_NAV_INPUT_LEFT, gf_dev->input->keybit);
		__set_bit(GF_KEY_INPUT_CAMERA, gf_dev->input->keybit);
		__set_bit(GF_NAV_INPUT_CLICK, gf_dev->input->keybit);
		__set_bit(GF_NAV_INPUT_DOUBLE_CLICK, gf_dev->input->keybit);
		__set_bit(GF_NAV_INPUT_LONG_PRESS, gf_dev->input->keybit);
		__set_bit(GF_NAV_INPUT_HEAVY, gf_dev->input->keybit);
#endif
		gf_dev->input->name = GF_INPUT_NAME;
		if (input_register_device(gf_dev->input)) {
			pr_warn("Failed to register GF as input device.\n");
		}
	}
#ifdef AP_CONTROL_CLK
	pr_info("Get the clk resource.\n");
	/* Enable spi clock */
	if (gfspi_ioctl_clk_init(gf_dev))
		goto gfspi_probe_clk_init_failed;

	if (gfspi_ioctl_clk_enable(gf_dev))
		goto gfspi_probe_clk_enable_failed;

	spi_clock_set(gf_dev, 1000000);
#endif

	gf_dev->notifier = goodix_noti_block;
	fb_register_client(&gf_dev->notifier);

      wake_lock_init(&gf_dev->ttw_wl, WAKE_LOCK_SUSPEND, "gf_ttw_wl"); //Alan, add wakelock

	gf_dev->irq = gf_irq_num(gf_dev);
		ret = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
            IRQF_TRIGGER_RISING | IRQF_ONESHOT, "gf", gf_dev);
    //printk("%s %d \n", __func__, __LINE__);
		if (!ret) {
			enable_irq_wake(gf_dev->irq);
		    gf_dev->irq_enabled = 1;
		    gf_disable_irq(gf_dev);
		}
/*added by chenhui for pwr3.3v pull down, begin*/
	gpio_direction_output(gf_dev->reset_gpio, 0);
	gf_power_off(gf_dev);
	gf_dev->device_available = 0;
/*added by chenhui for pwr3.3v pull down, end*/		
	pr_info("version V%d.%d.%02d\n", VER_MAJOR, VER_MINOR, PATCH_LEVEL);

/*added by chenhui for proc info node begin*/
	fpdsensor_proc_dir_entry = proc_mkdir(PROC_FPDSENSOR_DIR, NULL);	

	if (!fpdsensor_proc_dir_entry) {           
		pr_info("fpdsensor_proc_dir_entry is null\n");
	}
	proc_create(PROC_FPDSENSOR_NAME, 0666, NULL, &info_proc_file_ops);               

/*added by chenhui for proc info node end*/		
	
	
	return status;

error:
	gf_cleanup(gf_dev);
	gf_dev->device_available = 0;
	if (gf_dev->devt != 0) {
		pr_info("Err: status = %d\n", status);
		mutex_lock(&device_list_lock);
		list_del(&gf_dev->device_entry);
		device_destroy(gf_class, gf_dev->devt);
		clear_bit(MINOR(gf_dev->devt), minors);
		mutex_unlock(&device_list_lock);
#ifdef AP_CONTROL_CLK
gfspi_probe_clk_enable_failed:
		gfspi_ioctl_clk_uninit(gf_dev);
gfspi_probe_clk_init_failed:
#endif
		if (gf_dev->input != NULL)
			input_unregister_device(gf_dev->input);
	}

	return status;
}

#if defined(USE_SPI_BUS)
static int gf_remove(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_remove(struct platform_device *pdev)
#endif
{
	struct gf_dev *gf_dev = &gf;

	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq)
		free_irq(gf_dev->irq, gf_dev);

	if (gf_dev->input != NULL)
		input_unregister_device(gf_dev->input);
		input_free_device(gf_dev->input);

/*added by chenhui for proc node begin*/	
	remove_procfs_interfaces();
/*added by chenhui for proc node end*/	
	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);	
	clear_bit(MINOR(gf_dev->devt), minors);
	if (gf_dev->users == 0)
        gf_cleanup(gf_dev);

    fb_unregister_client(&gf_dev->notifier);
        mutex_unlock(&device_list_lock);

	return 0;
}

static struct of_device_id gx_match_table[] = {
	{ .compatible = GF_SPIDEV_NAME },
	{},
};

#if defined(USE_SPI_BUS)
static struct spi_driver gf_driver = {
#elif defined(USE_PLATFORM_BUS)
static struct platform_driver gf_driver = {
#endif
	.driver = {
		.name = GF_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gx_match_table,
	},
	.probe = gf_probe,
	.remove = gf_remove,
};

static int __init gf_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
	if (status < 0) {
		pr_warn("Failed to register char device!\n");
		return status;
	}
	SPIDEV_MAJOR = status;
	gf_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gf_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		pr_warn("Failed to create class.\n");
		return PTR_ERR(gf_class);
	}
#if defined(USE_PLATFORM_BUS)
	status = platform_driver_register(&gf_driver);
#elif defined(USE_SPI_BUS)
	status = spi_register_driver(&gf_driver);
#endif
	if (status < 0) {
		class_destroy(gf_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		pr_warn("Failed to register SPI driver.\n");
	}

#ifdef GF_NETLINK_ENABLE
	netlink_init();
#endif
	pr_info(" status = 0x%x\n", status);
	return 0;
}

module_init(gf_init);

static void __exit gf_exit(void)
{
#ifdef GF_NETLINK_ENABLE
	netlink_exit();
#endif
#if defined(USE_PLATFORM_BUS)
	platform_driver_unregister(&gf_driver);
#elif defined(USE_SPI_BUS)
	spi_unregister_driver(&gf_driver);
#endif
	class_destroy(gf_class);
	unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_AUTHOR("Jandy Gou, <gouqingsong@goodix.com>");
MODULE_DESCRIPTION("goodix fingerprint sensor device driver");
MODULE_LICENSE("GPL");
