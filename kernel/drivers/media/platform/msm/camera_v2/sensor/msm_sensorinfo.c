/*
 * drivers/media/video/msm/msm_sensorinfo.c
 *
 * For sensor cfg test
 *
 * Copyright (C) 2009-2010 ZTE Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Created by li.jing19@zte.com.cn
 */
/*-----------------------------------------------------------------------------------------
  when         who      what, where, why                        comment tag
  --------     ----     -------------------------------------   ---------------------------
  2010-06-13   lijing   modify file permission                  LIJING_CAM_20100613
  2010-06-10   lijing   create file                             
------------------------------------------------------------------------------------------*/
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include "msm_sensor.h"
/*-----------------------------------------------------------------------------------------
 *
 * MACRO DEFINITION
 *
 *----------------------------------------------------------------------------------------*/
#define MAX_NAME_LENGTH     40


#define SENSOR_INFO_IMX258_MODEL_ID     0x0258
#define SENSOR_INFO_S5K5E2_MODEL_ID     0x5e20
#define SENSOR_INFO_IMX351_MODEL_ID     0x0351
#define SENSOR_INFO_S5K4H8_MODEL_ID     0x4088
#define SENSOR_INFO_OV13855_MODEL_ID    0xD855
#define SENSOR_INFO_S5K3L8_MODEL_ID     0x30c8
#define SENSOR_INFO_GC2145_MODEL_ID     0x2145



/*-----------------------------------------------------------------------------------------
 *
 * GLOBAL VARIABLE DEFINITION
 *
 *----------------------------------------------------------------------------------------*/
static uint16_t g_back_sensor_id = 0;
static uint16_t g_front_sensor_id = 0;
static uint16_t module_integrator_id = 0;
static uint16_t g_aux_sensor_id = 0;

/*-----------------------------------------------------------------------------------------
 *
 * FUNCTION DECLARATION
 *
 *----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
 *
 * FUNCTION DEFINITION
 *
 *----------------------------------------------------------------------------------------*/

DEFINE_SEMAPHORE(set_back_sensor_id_sem);
void msm_sensorinfo_set_back_sensor_id(uint16_t id)
{
    down(&set_back_sensor_id_sem);
    g_back_sensor_id = id;
    up(&set_back_sensor_id_sem);
}
EXPORT_SYMBOL(msm_sensorinfo_set_back_sensor_id);


DEFINE_SEMAPHORE(set_back_sensor_module_id_sem);
void msm_sensorinfo_set_back_sensor_module_id(uint16_t module_id)
{
    down(&set_back_sensor_module_id_sem);
	module_integrator_id = module_id;
    up(&set_back_sensor_module_id_sem);
}
EXPORT_SYMBOL(msm_sensorinfo_set_back_sensor_module_id);


DEFINE_SEMAPHORE(set_front_sensor_id_sem);
void msm_sensorinfo_set_front_sensor_id(uint16_t id)
{
    down(&set_front_sensor_id_sem);
	if(SENSOR_INFO_GC2145_MODEL_ID == id)
	   g_aux_sensor_id = id;
	else
    g_front_sensor_id = id;
    up(&set_front_sensor_id_sem);   
}
EXPORT_SYMBOL(msm_sensorinfo_set_front_sensor_id);


static int rear_camera_id_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%x\n", g_back_sensor_id);
	return 0;
}
static int rear_camera_name_proc_show(struct seq_file *m, void *v)
{
	char sensor_name[MAX_NAME_LENGTH] = {0x00};	
	switch(g_back_sensor_id)
	{
		case SENSOR_INFO_IMX258_MODEL_ID:			
			if(module_integrator_id==0x15){
			  if(g_aux_sensor_id == SENSOR_INFO_GC2145_MODEL_ID)
			  	sprintf(sensor_name, "IMX258-13M-LITEON-AF&GC2145-2M-FF");
			  else
			  sprintf(sensor_name, "IMX258-13M-LITEON-AF");
			}
			else if(module_integrator_id==0x06){
			  if(g_aux_sensor_id == SENSOR_INFO_GC2145_MODEL_ID)
			  	sprintf(sensor_name, "IMX258-13M-QTECH-AF&GC2145-2M-FF");
			  else
                          sprintf(sensor_name, "IMX258-13M-QTECH-AF");
			}
                      else
			  sprintf(sensor_name, "IMX258-13M-AF");
			break;
		case SENSOR_INFO_IMX351_MODEL_ID:
			  sprintf(sensor_name, "IMX351-16M-AF");
			break;
		case SENSOR_INFO_OV13855_MODEL_ID:
			 if(g_aux_sensor_id == SENSOR_INFO_GC2145_MODEL_ID)
			    sprintf(sensor_name, "OV13855-13M-AF&GC2145-2M-FF");
			 else
			  sprintf(sensor_name, "OV13855-13M-AF");
			break;
		 case SENSOR_INFO_S5K3L8_MODEL_ID:			
			  sprintf(sensor_name, "S5K3L8-13M-AF");
			break;
		  case SENSOR_INFO_S5K4H8_MODEL_ID:			
			  sprintf(sensor_name, "S5K4H8-8M-AF");
		  break;
		default:
			sprintf(sensor_name, "No sensor or error ID!");
			break;
	}
	seq_printf(m, "%s\n", sensor_name);
	return 0;
}

static int front_camera_id_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%x\n", g_front_sensor_id);
	return 0;
}
static int front_camera_name_proc_show(struct seq_file *m, void *v)
{
	char sensor_name[MAX_NAME_LENGTH] = {0x00};	
	switch(g_front_sensor_id)
	{
		case SENSOR_INFO_S5K5E2_MODEL_ID:			
			  sprintf(sensor_name, "S5K5E2-5M-FF");
			break;
		case SENSOR_INFO_IMX258_MODEL_ID:			
			  sprintf(sensor_name, "IMX258-13M-FF");
			break;
		case SENSOR_INFO_S5K4H8_MODEL_ID:			
			  sprintf(sensor_name, "S5K4H8-8M-FF");
			break;
		case SENSOR_INFO_S5K3L8_MODEL_ID:			
			  sprintf(sensor_name, "S5K3L8-13M-FF");
			break;
		default:
			sprintf(sensor_name, "No sensor or error ID!");
			break;
	}
	seq_printf(m, "%s\n", sensor_name);
	return 0;
}


static int rear_camera_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rear_camera_id_proc_show, NULL);
}
static int rear_camera_name_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rear_camera_name_proc_show, NULL);
}

static int front_camera_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, front_camera_id_proc_show, NULL);
}
static int front_camera_name_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, front_camera_name_proc_show, NULL);
}


static const struct file_operations rear_camera_id_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= rear_camera_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static const struct file_operations rear_camera_name_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= rear_camera_name_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static const struct file_operations front_camera_id_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= front_camera_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static const struct file_operations front_camera_name_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= front_camera_name_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
extern const struct file_operations flash_proc_fops;
/*
 * Attention:
 *
 * Path of camera's sysdev: /sys/devices/system/camera/camera0
 */
static int __init sensorinfo_init(void)
{	
	struct proc_dir_entry *dir, *res;

	dir = proc_mkdir("camera", NULL);
	if (!dir)
		return -ENOMEM;

	res = proc_create_data("rear_camera_id", 0, dir,
			       &rear_camera_id_proc_fops, NULL);
	if (!res)
		return -ENOMEM;

	res = proc_create_data("rear_camera_name", 0, dir,
			       &rear_camera_name_proc_fops, NULL);
	if (!res)
		return -ENOMEM;

	res = proc_create_data("front_camera_id", 0, dir,
			       &front_camera_id_proc_fops, NULL);
	if (!res)
		return -ENOMEM;

	res = proc_create_data("front_camera_name", 0, dir,
			       &front_camera_name_proc_fops, NULL);
	if (!res)
		return -ENOMEM;
  return 0;
}
module_init(sensorinfo_init);

