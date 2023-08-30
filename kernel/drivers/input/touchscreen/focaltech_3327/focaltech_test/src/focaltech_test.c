/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, FocalTech Systems, Ltd., all rights reserved.
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
 */

/************************************************************************
*
* File Name: focaltech_test.c
*
* Author:     Software Department, FocalTech
*
* Created: 2016-08-01
*
* Modify:
*
* Abstract: create char device and proc node for  the comm between APK and TP
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <asm/uaccess.h>

#include <linux/i2c.h>//iic
#include <linux/delay.h>//msleep

#include "../../focaltech_core.h"
#include "../include/focaltech_test_main.h"
#include "../include/focaltech_test_ini.h"

#ifdef CONFIG_TPD_FOCAL_TEST
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/time.h>
#include <linux/rtc.h>
#endif

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define IC_TEST_VERSION  "Test version: V1.0.0--2016-12-28, (sync version of FT_MultipleTest: V4.0.0.0 ------ 2016-07-18)"

// Define the configuration file storage directory
#define FTS_INI_FILE_PATH "/mnt/sdcard/"

#define FTS_TEST_BUFFER_SIZE        80*1024
#define FTS_TEST_PRINT_SIZE     128
/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/


/*****************************************************************************
* Static variables
*****************************************************************************/
#ifdef CONFIG_TPD_FOCAL_TEST
static unsigned char g_str_save_file_path[256];
static unsigned char g_str_ini_file_path[256];
static unsigned char g_str_ini_filename[128];

static int g_node_data_type = -1;
#endif
/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
#ifdef CONFIG_TPD_FOCAL_TEST
struct fts_test_buffer g_fts_test_buffer;
int g_int_tptest_result = 0;
#endif
/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int fts_test_get_ini_size(char *config_name);
static int fts_test_read_ini_data(char *config_name, char *config_buf);
static int fts_test_save_test_data(char *file_name, char *data_buf, int iLen);
static int fts_test_get_testparam_from_ini(char *config_name);
static int fts_test_entry(char *ini_file_name);

static int fts_test_i2c_read(unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen);
static int fts_test_i2c_write(unsigned char *writebuf, int writelen);


/*****************************************************************************
* functions body
*****************************************************************************/
#if 0
//  old fts_i2c_read/write function. need to set fts_i2c_client.
extern struct i2c_client* fts_i2c_client;
extern int fts_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
extern int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
#endif
static int fts_test_i2c_read(unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen)
{
    int iret = -1;
#if 1
    //  old fts_i2c_read function. need to set fts_i2c_client.
    //Modify the i2c_read function that is used in this project
    iret = fts_i2c_read(fts_i2c_client, writebuf, writelen, readbuf, readlen);
#else
    iret = fts_i2c_read(writebuf, writelen, readbuf, readlen);
#endif

    return iret;

}

static int fts_test_i2c_write(unsigned char *writebuf, int writelen)
{
    int iret = -1;
#if 1
    //  old fts_i2c_write function.  need to set fts_i2c_client.
    //Modify the i2c_read function that is used in this project
    iret = fts_i2c_write(fts_i2c_client, writebuf, writelen);
#else
    iret = fts_i2c_write(writebuf, writelen);
#endif

    return iret;
}
#ifdef CONFIG_TPD_FOCAL_TEST
int save_failed_node_to_buffer(struct fts_test_buffer *stp_test, char * tmp_buffer, int length)
{
	//TPD_DMESG("length:%d", length);
	
	if(NULL == stp_test->node_failed_buffer || (stp_test->node_failed_buffer_length + length) > TEST_RESULT_LENGTH) {
		TPD_DMESG("warning:buffer is null or buffer overflow, return");
		return -1;
	}

	memcpy(stp_test->node_failed_buffer + stp_test->node_failed_buffer_length, tmp_buffer, length);
	stp_test->node_failed_buffer_length += length;
	stp_test->node_failed_count++;

	return 0;
}

int save_failed_node(char * str)
{
	save_failed_node_to_buffer(&g_fts_test_buffer, str, strlen(str));

	return 0;
}

int fts_save_string_to_buffer(struct fts_test_buffer *stp_test, char * tmp_buffer, int length)
{
	//TPD_DMESG("length:%d", length);
	
	if(NULL == stp_test->procedure_buffer || (stp_test->procedure_length + length) > TEST_RESULT_LENGTH) {
		TPD_DMESG("warning:buffer is null or buffer overflow, return");
		return -1;
	}

	memcpy(stp_test->procedure_buffer + stp_test->procedure_length, tmp_buffer, length);
	stp_test->procedure_length += length;	

	return 0;
}

int save_test_procedure(char * str)
{
	fts_save_string_to_buffer(&g_fts_test_buffer, str, strlen(str));

	return 0;
}

int fts_save_test_result(struct fts_test_buffer *stp_test)
{
	int i_len = 0;
	int result = stp_test->test_result;

	TPD_DMESG("result:0x%x", result);
	i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "show test result: 0x%x\n", result);
	fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);

	if(result < 0) {
		i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "test skip!\n");
		fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
	}else if(result == 0) {
		i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "test pass!\n");
		fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
	} else {
		i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "test fail!\n");
		fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		if ((result & TEST_BEYOND_MAX_LIMIT) != 0) {
			i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "beyond max limit\n");
			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		}
		if ((result & TEST_BEYOND_MIN_LIMIT) != 0){
			i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "beyond min limit\n");
			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		}
		if ((result & TEST_BEYOND_ACCORD_LIMIT) != 0){
			i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "beyond accord limit\n");
			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		}
		if ((result & TEST_BEYOND_JITTER_LIMIT) != 0){
			i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "beyond jitter limit\n");
			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		}
		if ((result & TEST_KEY_BEYOND_MAX_LIMIT) != 0){
			i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "key beyond max limit\n");
			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		}
		if ((result & TEST_KEY_BEYOND_MIN_LIMIT) != 0){
			i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "key beyond min limit\n");
			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		}
		if ((result & TEST_MODULE_TYPE_ERR) != 0){
			i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "module type error\n");
			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		}
		if ((result & TEST_VERSION_ERR) != 0){
			i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "module version error\n");
			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		}
		if ((result & TEST_GT_OPEN) != 0){
			i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "ito channel open\n");
			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		}
		if ((result & TEST_GT_SHORT) != 0){
			i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "ito channel short\n");
			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		}
		if ((result & TEST_BEYOUNT_SCAP_CB_LIMIT) != 0){
			i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "scap cb out of range\n");
			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		}
		if ((result & TEST_BEYOUNT_SCAP_RAWDATA_LIMIT) != 0){
			i_len = snprintf(stp_test->temp_buffer, TEST_TEMP_LENGTH, "scap rawdata out of range\n");
			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
		}
	}

	return i_len;
}

int fts_save_ito_array_data_to_file(struct fts_test_buffer *stp_test, char *title_info, void *data, int length, int size, int i_row, int i_col)
{
	int i_len = 0;
	int i = 0, j = 0;
	int i_value = 0;
	int i_rawdata_min = 65535, i_rawdata_max = 0, i_rawdata_count = 0;
	long i_rawdata_average = 0;
	u8 *p_u8_value = NULL;
	u16 *p_u16_vlaue = NULL;
	u32 *p_u32_vlaue = NULL;
	u64 *p_u64_vlaue = NULL;
	

	p_u8_value = (u8 *)data;
	p_u16_vlaue = (u16 *)data;
	p_u32_vlaue = (u32 *)data;
	p_u64_vlaue = (u64 *)data;

	if(NULL == stp_test->temp_buffer || NULL == stp_test->procedure_buffer) {
		TPD_DMESG("warning:buffer is null, return");
		return -1;
	}

	i_len = sprintf(stp_test->temp_buffer,"%s Col:%d Row:%d\n", title_info, i_row, i_col);
	fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
	
	i_rawdata_min = 65535;
	i_rawdata_max = 0;
	i_rawdata_average = 0;
	i_rawdata_count = 0;
	//Save Data 
	for(i = 0; i < i_row; i++) {
		for(j = 0; j < i_col; j++) {
			if(4 == size) {
				i_value = p_u32_vlaue[i * i_col + j];
			} else if(2 == size){
				i_value = p_u16_vlaue[i * i_col + j];
			} else if(1== size){
				i_value = p_u8_value[i * i_col + j];
			} else if(8 == size){
				i_value = p_u64_vlaue[i * i_col + j];
			} else {
				i_value = p_u8_value[i * i_col + j];
			}
			if(j == (i_col -1))        //The Last Data of the Row, add "\n"
				i_len= sprintf(stp_test->temp_buffer,"%d, \n",  i_value);	
			else
				i_len= sprintf(stp_test->temp_buffer,"%d, ", i_value);	

			fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
			if(i_rawdata_min > i_value) {
				i_rawdata_min = i_value;
			}
			if(i_rawdata_max < i_value) {
				i_rawdata_max = i_value;
			}
			i_rawdata_average += i_value;
			i_rawdata_count++;
		}
	}
	i_len= sprintf(stp_test->temp_buffer,"Data min:%d max:%d average:%ld\n", i_rawdata_min, i_rawdata_max, i_rawdata_average / i_rawdata_count);
	fts_save_string_to_buffer(stp_test, stp_test->temp_buffer, i_len);
	
	return 0;
}
int save_rawdata_to_file(char *title_info, void *data, int length, int node_byte)
{
	return fts_save_ito_array_data_to_file(&g_fts_test_buffer, title_info, data, length, node_byte, g_fts_test_buffer.i_txNum, g_fts_test_buffer.i_rxNum);
}

static int fts_test_get_save_filename(char *filename, int len)
{	
	char* board_idFile = {"/persist/factoryinfo/board_id"};
	struct file *pfile = NULL;
	struct inode *inode = NULL;
	//unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	loff_t pos = 0;
	mm_segment_t old_fs;
	char board_id[20];
	struct timespec ts;
	struct rtc_time tm;

	//get rtc time
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", board_idFile); 

	if (NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}

	if (IS_ERR(pfile)) {
		FTS_TEST_ERROR("error occured while opening file %s.",  filepath);
		snprintf(filename, len, "test_data%04d%02d%02d",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
		return 0;
	}

	inode = pfile->f_dentry->d_inode;
	//magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, board_id, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	//snprintf(filename, len, "TP_test_data%s_%02d%02d%02d", board_id, tm.tm_hour, tm.tm_min, tm.tm_sec);
	snprintf(filename, len, "TP_test_data%s", board_id);
	return 0;
}
#endif
//Gets the configuration file size for allocating memory to read configuration
static int fts_test_get_ini_size(char *config_name)
{
    struct file *pfile = NULL;
    struct inode *inode = NULL;
    //unsigned long magic;
    off_t fsize = 0;
    char filepath[128];

    FTS_TEST_FUNC_ENTER();

    memset(filepath, 0, sizeof(filepath));
#ifdef CONFIG_TPD_FOCAL_TEST
	sprintf(filepath, "%s%s", g_str_ini_file_path, config_name);
#else
    sprintf(filepath, "%s%s", FTS_INI_FILE_PATH, config_name);
#endif
    if (NULL == pfile)
        pfile = filp_open(filepath, O_RDONLY, 0);

    if (IS_ERR(pfile))
    {
        FTS_TEST_ERROR("error occured while opening file %s.",  filepath);
        return -EIO;
    }

    inode = pfile->f_dentry->d_inode;
    //magic = inode->i_sb->s_magic;
    fsize = inode->i_size;
    filp_close(pfile, NULL);

    FTS_TEST_FUNC_ENTER();

    return fsize;
}

//Read configuration to memory
static int fts_test_read_ini_data(char *config_name, char *config_buf)
{
    struct file *pfile = NULL;
    struct inode *inode = NULL;
    //unsigned long magic;
    off_t fsize = 0;
    char filepath[128];
    loff_t pos = 0;
    mm_segment_t old_fs;

    FTS_TEST_FUNC_ENTER();

    memset(filepath, 0, sizeof(filepath));
#ifdef CONFIG_TPD_FOCAL_TEST
	sprintf(filepath, "%s%s", g_str_ini_file_path, config_name);
#else
    sprintf(filepath, "%s%s", FTS_INI_FILE_PATH, config_name);
#endif
    if (NULL == pfile)
    {
        pfile = filp_open(filepath, O_RDONLY, 0);
    }
    if (IS_ERR(pfile))
    {
        FTS_TEST_ERROR("error occured while opening file %s.",  filepath);
        return -EIO;
    }

    inode = pfile->f_dentry->d_inode;
    //magic = inode->i_sb->s_magic;
    fsize = inode->i_size;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_read(pfile, config_buf, fsize, &pos);
    filp_close(pfile, NULL);
    set_fs(old_fs);

    FTS_TEST_FUNC_EXIT();
    return 0;
}

//Save test data to SD card etc.
static int fts_test_save_test_data(char *file_name, char *data_buf, int iLen)
{
    struct file *pfile = NULL;

    char filepath[128];
    loff_t pos;
    mm_segment_t old_fs;

    FTS_TEST_FUNC_ENTER();

    memset(filepath, 0, sizeof(filepath));
#ifdef CONFIG_TPD_FOCAL_TEST
	sprintf(filepath, "%s%s", g_str_save_file_path, file_name);
#else
    sprintf(filepath, "%s%s", FTS_INI_FILE_PATH, file_name);
#endif
    if (NULL == pfile)
    {

        pfile = filp_open(filepath, O_TRUNC|O_CREAT|O_RDWR, 0);
    }
    if (IS_ERR(pfile))
    {
        FTS_TEST_ERROR("error occured while opening file %s.",  filepath);
        return -EIO;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_write(pfile, data_buf, iLen, &pos);
    filp_close(pfile, NULL);
    set_fs(old_fs);

    FTS_TEST_FUNC_EXIT();
    return 0;
}

//Read, parse the configuration file, initialize the test variable
static int fts_test_get_testparam_from_ini(char *config_name)
{
    char *pcfiledata = NULL;
    int ret = 0;
    int inisize = 0;

    FTS_TEST_FUNC_ENTER();

    inisize = fts_test_get_ini_size(config_name);
    FTS_TEST_DBG("ini_size = %d ", inisize);
    if (inisize <= 0)
    {
        FTS_TEST_ERROR("%s ERROR:Get firmware size failed",  __func__);
        return -EIO;
    }

    pcfiledata = fts_malloc(inisize + 1);
    if (NULL == pcfiledata)
    {
        FTS_TEST_ERROR("fts_malloc failed in function:%s",  __func__);
        return -1;
    }

    memset(pcfiledata, 0, inisize + 1);

    if (fts_test_read_ini_data(config_name, pcfiledata))
    {
        FTS_TEST_ERROR(" - ERROR: fts_test_read_ini_data failed" );
        fts_free(pcfiledata);
        pcfiledata = NULL;

        return -EIO;
    }
    else
    {
        FTS_TEST_DBG("fts_test_read_ini_data successful");
    }

    ret = set_param_data(pcfiledata);

    fts_free(pcfiledata);   // lifengshi add. 20160608
    pcfiledata = NULL;

    FTS_TEST_FUNC_EXIT();

    if (ret < 0)
        return ret;

    return 0;
}

/////////////////////////////////
//Test library call entry
///////////////////////////////////
static int fts_test_entry(char *ini_file_name)
{
    /* place holder for future use */
    char cfgname[128];
    char *testdata = NULL;
    char *printdata = NULL;
    int iTestDataLen=0; //The actual length of the test data in the library is used to save the data to the file.
    int ret = 0;
    int icycle = 0, i =0;
    int print_index = 0;


    FTS_TEST_FUNC_ENTER();
    FTS_TEST_DBG("ini_file_name:%s.", ini_file_name);
    /*Used to obtain the test data stored in the library, pay attention to the size of the distribution space.*/
    FTS_TEST_DBG("Allocate memory, size: %d", FTS_TEST_BUFFER_SIZE);
    testdata = fts_malloc(FTS_TEST_BUFFER_SIZE);
    if (NULL == testdata)
    {
        FTS_TEST_ERROR("fts_malloc failed in function:%s",  __func__);
        return -1;
    }
    printdata = fts_malloc(FTS_TEST_PRINT_SIZE);
    if (NULL == printdata)
    {
        FTS_TEST_ERROR("fts_malloc failed in function:%s",  __func__);
        return -1;
    }
    /*Initialize the platform related I2C read and write functions*/

#if 0
    init_i2c_write_func(fts_i2c_write);
    init_i2c_read_func(fts_i2c_read);
#else
    init_i2c_write_func(fts_test_i2c_write);
    init_i2c_read_func(fts_test_i2c_read);
#endif

    /*Initialize pointer memory*/
    ret = focaltech_test_main_init();
    if (ret < 0)
    {
        FTS_TEST_ERROR("focaltech_test_main_init() error.");
        goto TEST_ERR;
    }

    /*Read parse configuration file*/
    memset(cfgname, 0, sizeof(cfgname));
    sprintf(cfgname, "%s", ini_file_name);
    FTS_TEST_DBG("ini_file_name = %s", cfgname);

    fts_test_funcs();

    if (fts_test_get_testparam_from_ini(cfgname) <0)
    {
        FTS_TEST_ERROR("get testparam from ini failure");
        goto TEST_ERR;
    }


    if ((g_ScreenSetParam.iSelectedIC >> 4  != FTS_CHIP_TEST_TYPE >> 4))
    {
        FTS_TEST_ERROR("Select IC and Read IC from INI does not match ");
        goto TEST_ERR;
    }


    /*Start testing according to the test configuration*/
    if (true == start_test_tp())
    {
        TestResultLen += sprintf(TestResult+TestResultLen,"Tp test pass. \n\n");
        FTS_TEST_INFO("tp test pass");
    }

    else
    {
        TestResultLen += sprintf(TestResult+TestResultLen,"Tp test failure. \n\n");
        FTS_TEST_INFO("tp test failure");
    }


    /*Gets the number of tests in the test library and saves it*/
    iTestDataLen = get_test_data(testdata);
    //FTS_TEST_DBG("\n%s", testdata);

    icycle = 0;
    /*Print test data packets */
    FTS_TEST_DBG("print test data: \n");
    for (i = 0; i < iTestDataLen; i++)
    {
        if (('\0' == testdata[i]) //Meet the end
            ||(icycle == FTS_TEST_PRINT_SIZE -2)//Meet the print string length requirements
            ||(i == iTestDataLen-1)//The last character
           )
        {
            if (icycle == 0)
            {
                print_index++;
            }
            else
            {
                memcpy(printdata, testdata + print_index, icycle);
                printdata[FTS_TEST_PRINT_SIZE-1] = '\0';
				//FTS_TEST_DBG("%s", printdata);    // for phone dead
                print_index += icycle;
                icycle = 0;
            }
        }
        else
        {
            icycle++;
        }
    }
    FTS_TEST_DBG("\n");
#ifdef CONFIG_TPD_FOCAL_TEST
	{
		char filename[64];
		char procfilename[64];
		g_fts_test_buffer.test_result = g_int_tptest_result;
		fts_save_test_result(&g_fts_test_buffer);
		fts_test_get_save_filename(filename, 56);
		strncpy(procfilename, filename, 56);
		fts_test_save_test_data(strcat(filename, ".csv"), testdata, iTestDataLen);
		fts_test_save_test_data(strcat(procfilename, "_proc.csv"), g_fts_test_buffer.procedure_buffer, g_fts_test_buffer.procedure_length);
	}
#else
    fts_test_save_test_data("testdata.csv", testdata, iTestDataLen);
    fts_test_save_test_data("testresult.txt", TestResult,TestResultLen);
#endif
    /*Release memory */
    focaltech_test_main_exit();


    //mutex_unlock(&g_device_mutex);
    if (NULL != testdata) fts_free(testdata);
    if (NULL != printdata) fts_free(printdata);

    FTS_TEST_FUNC_EXIT();

    return 0;

TEST_ERR:
    if (NULL != testdata) fts_free(testdata);
    if (NULL != printdata) fts_free(printdata);

    FTS_TEST_FUNC_EXIT();

    return -1;
}

/************************************************************************
* Name: fts_test_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EPERM;
}

/************************************************************************
* Name: fts_test_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char fwname[128] = {0};
    struct i2c_client *client = fts_i2c_client;

    FTS_TEST_FUNC_ENTER();

    memset(fwname, 0, sizeof(fwname));
    sprintf(fwname, "%s", buf);
    fwname[count-1] = '\0';
    FTS_TEST_DBG("fwname:%s.", fwname);

    mutex_lock(&fts_input_dev->mutex);

    disable_irq(client->irq);

#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
    fts_esdcheck_switch(DISABLE);
#endif
    fts_test_entry( fwname);

#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
    fts_esdcheck_switch(ENABLE);
#endif
    enable_irq(client->irq);

    mutex_unlock(&fts_input_dev->mutex);

    FTS_TEST_FUNC_EXIT();

    return count;
}
#ifdef CONFIG_TPD_FOCAL_TEST
extern int tpd_test_get_channel_setting(int *buffer);
extern int tpd_test_get_tp_node_data(int type, char * buffer, int length);

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

	mutex_lock(&fts_input_dev->mutex);

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_save_file_path);

	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static int tpd_test_save_file_path_store(struct tpd_classdev_t *cdev, const char *buf)
{
	memset(g_str_save_file_path, 0, sizeof(g_str_save_file_path));
	snprintf(g_str_save_file_path, 256, "%s", buf);

	dir_path_add_slash(g_str_save_file_path);

	FTS_TEST_DBG("save file path:%s.", g_str_save_file_path);

	return 0;
}

static int tpd_test_ini_file_path_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	mutex_lock(&fts_input_dev->mutex);

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_ini_file_path);

	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static int tpd_test_ini_file_path_store(struct tpd_classdev_t *cdev, const char *buf)
{
	memset(g_str_ini_file_path, 0, sizeof(g_str_ini_file_path));
	snprintf(g_str_ini_file_path, 256, "%s", buf);

	dir_path_add_slash(g_str_ini_file_path);

	FTS_TEST_DBG("ini file path:%s.", g_str_ini_file_path);

	return 0;
}

static int tpd_test_filename_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	mutex_lock(&fts_input_dev->mutex);

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_ini_filename);

	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static int tpd_test_filename_store(struct tpd_classdev_t *cdev, const char *buf)
{
	memset(g_str_ini_filename, 0, sizeof(g_str_ini_filename));
	snprintf(g_str_ini_filename, 128, "%s", buf);

	string_trim_tail(g_str_ini_filename);
	
	FTS_TEST_DBG("fwname:%s.", g_str_ini_filename);

	return 0;
}

static int tpd_test_cmd_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;
	//struct i2c_client *client = fts_i2c_client;
	struct fts_test_buffer * stp_test = &g_fts_test_buffer;
	int buffer_length = 0;
	int i_len = 0;

	mutex_lock(&fts_input_dev->mutex);
	printk("tpd %s [func] in.\n", __func__);
	
	i_len = sprintf(buf, "%d,%d,%d", stp_test->i_txNum, stp_test->i_rxNum, stp_test->node_failed_count);
	printk("tpd %s [func] test resutl:0x%x && rawdata node failed count:0x%x.\n", __func__, stp_test->test_result, stp_test->node_failed_count);
	
	buffer_length = (stp_test->node_failed_buffer_length + 1) > (PAGE_SIZE - i_len) ? (PAGE_SIZE - i_len - 1) : stp_test->node_failed_buffer_length;
	printk("tpd %s [func] failed node string lenght:0x%x, buffer_length:0x%x.\n", __func__, stp_test->node_failed_buffer_length, buffer_length);

	if(stp_test->node_failed_buffer != NULL && buffer_length > 0) {
		memcpy(buf + i_len, stp_test->node_failed_buffer, buffer_length);
		buf[buffer_length + i_len] = '\0';
	}
	
	printk("tpd %s test:%s.", __func__, buf);
	
	num_read_chars = buffer_length + i_len;
	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static int tpd_test_cmd_store(struct tpd_classdev_t *cdev, const char *buf)
{
	struct i2c_client *client = fts_i2c_client;
	unsigned long command = 0;
	int retval = -1;
	static int node_opened = 0;
	struct fts_test_buffer * stp_test = &g_fts_test_buffer;
	
	retval = kstrtoul(buf, 10, &command);
	if (retval) {
		TPD_DMESG("invalid param:%s", buf);
		return 0;
	}
	
	printk("tpd %s [func] command:%ld, ini filename:%s.\n", __func__, command, g_str_ini_filename);

	mutex_lock(&fts_input_dev->mutex);

	//command 1:open node; 2:start test; 3:close node.
	if(1 == command) {    //open test node, alloc space etc...
		if(NULL == stp_test->result_buffer ) {
			int buffer[5] = {0, 0, 0, 0, 0};;
			tpd_test_get_channel_setting(&buffer[0]);
			stp_test->i_txNum = buffer[0];
			stp_test->i_rxNum = buffer[1];
			
			stp_test->result_buffer = (char*)fts_malloc(TEST_RESULT_LENGTH);
			stp_test->result_length = 0;
		}
		if(NULL == stp_test->node_failed_buffer ) {
			stp_test->node_failed_buffer = (char*)fts_malloc(TEST_RESULT_LENGTH);
			stp_test->node_failed_buffer_length = 0;
			stp_test->node_failed_count = 0;
		}
		if(NULL == stp_test->temp_buffer ) {
			stp_test->temp_buffer = (char*)fts_malloc(TEST_TEMP_LENGTH);
		}
		if(NULL == stp_test->procedure_buffer ) {
			stp_test->procedure_buffer = (char*)fts_malloc(TEST_RESULT_LENGTH);
			stp_test->procedure_length = 0;
		}
		node_opened = 1;
	} else if(2 == command) {
		if(1 == node_opened) {   //start test
		
			stp_test->result_length = 0;
			stp_test->node_failed_buffer_length = 0;
			stp_test->node_failed_count = 0;
			stp_test->procedure_length = 0;

			stp_test->test_result = g_int_tptest_result = 0;
			
			disable_irq(client->irq);

#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
			fts_esdcheck_switch(DISABLE);
#endif
			fts_test_entry(g_str_ini_filename);

#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
			fts_esdcheck_switch(ENABLE);
#endif
			enable_irq(client->irq);
		} else {
			//stp_test->test_result = -1;
			TPD_DMESG("command:0x%ld,  open node before start test.", command);
		}
	} else if(3 == command) {    //close test node, free space etc...
		node_opened = 0;
		//free_fts_test_bufferruct(stp_test);
	} else {
		TPD_DMESG("invalid command %ld", command);
	}
	mutex_unlock(&fts_input_dev->mutex);

	return 0;
}

static int tpd_test_node_data_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;
	int iLen = 0;
	
	mutex_lock(&fts_input_dev->mutex);
	init_i2c_write_func(fts_test_i2c_write);
	init_i2c_read_func(fts_test_i2c_read);
	fts_test_funcs();

	iLen = tpd_test_get_tp_node_data(g_node_data_type, buf, 4096);

	num_read_chars = iLen;
	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static int tpd_test_node_data_store(struct tpd_classdev_t *cdev, const char *buf)
{
	int data_type = 0;

	sscanf(buf, "%d", &data_type);

	FTS_TEST_DBG("%s type:%d .", __func__, data_type);
	
	mutex_lock(&fts_input_dev->mutex);

	g_node_data_type = data_type;

	mutex_unlock(&fts_input_dev->mutex);

	return 0;
}
static int tpd_test_channel_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;
	int buffer[5] = {0, 0, 0, 0, 0};
	
	init_i2c_write_func(fts_test_i2c_write);
	init_i2c_read_func(fts_test_i2c_read);
	fts_test_funcs();

	tpd_test_get_channel_setting(&buffer[0]);
	g_fts_test_buffer.i_txNum = buffer[0];
	g_fts_test_buffer.i_rxNum = buffer[1];

	num_read_chars = snprintf(buf, PAGE_SIZE, "%d %d %d %d %d", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);

	return num_read_chars;
}
static int tpd_test_result_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "0x%x", g_int_tptest_result);

	return num_read_chars;
}
#endif

/*  upgrade from app.bin
*    example:echo "***.ini" > fts_test
*/
static DEVICE_ATTR(fts_test, S_IRUGO|S_IWUSR, fts_test_show, fts_test_store);

/* add your attr in here*/
static struct attribute *fts_test_attributes[] =
{
    &dev_attr_fts_test.attr,
    NULL
};

static struct attribute_group fts_test_attribute_group =
{
    .attrs = fts_test_attributes
};


int fts_test_init(struct i2c_client *client)
{
    int err=0;

    FTS_TEST_FUNC_ENTER();

    FTS_TEST_INFO("[focal] %s ",  IC_TEST_VERSION);  //show version


    err = sysfs_create_group(&client->dev.kobj, &fts_test_attribute_group);
    if (0 != err)
    {
        FTS_TEST_ERROR( "[focal] %s() - ERROR: sysfs_create_group() failed.",  __func__);
        sysfs_remove_group(&client->dev.kobj, &fts_test_attribute_group);
        return -EIO;
    }
    else
    {
        FTS_TEST_DBG("[focal] %s() - sysfs_create_group() succeeded.", __func__);
    }
#ifdef CONFIG_TPD_FOCAL_TEST
	strncpy(g_str_save_file_path, FTS_INI_FILE_PATH, 256);
	strncpy(g_str_ini_file_path, FTS_INI_FILE_PATH, 256);
	strncpy(g_str_ini_filename, "test.ini", 128);

	tpd_fw_cdev.tpd_test_set_save_filepath = tpd_test_save_file_path_store;
	tpd_fw_cdev.tpd_test_get_save_filepath = tpd_test_save_file_path_show;
	tpd_fw_cdev.tpd_test_set_ini_filepath = tpd_test_ini_file_path_store;
	tpd_fw_cdev.tpd_test_get_ini_filepath = tpd_test_ini_file_path_show;
	tpd_fw_cdev.tpd_test_set_filename = tpd_test_filename_store;
	tpd_fw_cdev.tpd_test_get_filename = tpd_test_filename_show;
	tpd_fw_cdev.tpd_test_set_cmd = tpd_test_cmd_store;
	tpd_fw_cdev.tpd_test_get_cmd = tpd_test_cmd_show;
	tpd_fw_cdev.tpd_test_set_node_data_type = tpd_test_node_data_store;
	tpd_fw_cdev.tpd_test_get_node_data = tpd_test_node_data_show;
	tpd_fw_cdev.tpd_test_get_channel_info = tpd_test_channel_show;
	tpd_fw_cdev.tpd_test_get_result = tpd_test_result_show;
#endif
    FTS_TEST_FUNC_EXIT();
    //fts_protocol_windows_to_android(client);
    return err;
}
int fts_test_exit(struct i2c_client *client)
{
    FTS_TEST_FUNC_ENTER();
    sysfs_remove_group(&client->dev.kobj, &fts_test_attribute_group);

    FTS_TEST_FUNC_EXIT();
    return 0;
}

