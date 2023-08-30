/**************************************************************************
*  AW9233_ts_3button.c
* 
*  Create Date :
* 
*  Modify Date : 
*
*  Create by   : AWINIC Technology CO., LTD
*
*  Version     : 1.0 , 2016/03/22
* 
**************************************************************************/
//////////////////////////////////////////////////////////////
//  
//  APPLICATION DEFINE :
//
//                   Mobile -    MENU      HOME      BACK
//                   AW9233 -     S3        S2        S1
//
//////////////////////////////////////////////////////////////
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include "AW9233_reg.h"
#include "AW9233_para.h"
//#include <mach/mt_gpt.h>
#include <linux/wakelock.h>
#include <linux/atomic.h>
#include <linux/notifier.h>
#include <linux/fb.h>

#include <linux/leds.h>



//////////////////////////////////////
// Marco
//////////////////////////////////////
#define AW9233_ts_NAME		"AW9233_ts"

////////////////////////////////////////////////////////////////////
// Auto Calibration
////////////////////////////////////////////////////////////////////
#ifdef AW_AUTO_CALI
#define CALI_NUM		4
#define CALI_RAW_MIN	1000
#define CALI_RAW_MAX	3000

unsigned char cali_flag = 0;
unsigned char cali_num = 0;
unsigned char cali_cnt = 0;
unsigned char cali_used = 0;
unsigned char old_cali_dir[6];	//	0: no cali		1: ofr pos cali		2: ofr neg cali
unsigned int old_ofr_cfg[6];
long Ini_sum[6];
#endif

//////////////////////////////////////////////////////////////////////////////////////
// Touch Key Driver
//////////////////////////////////////////////////////////////////////////////////////

struct AW9233_ts_data {
	struct input_dev	*input_dev;
	struct work_struct 	eint_work;	
	struct delayed_work led_work;
	struct device_node *irq_node;
	int irq;
	struct notifier_block fb_notif;

};

struct aw9233_led_button {
	struct i2c_client *client;
	struct led_classdev cdev;
};

struct aw9233_led_HUXI {
	struct i2c_client *client;
	struct led_classdev cdev;
};


struct AW9233_ts_data *AW9233_ts;
static struct i2c_client *aw9233_i2c_client;
struct aw9233_led_button *AW9233_led_button;
struct aw9233_led_HUXI *AW9233_led_huxi;
/***********************led breath code start********************************/
#define ROM_CODE_MAX 255
/*
 * rise_time_ms = 1500
 * hold_time_ms = 500
 * fall_time_ms = 1500
 * off_time_ms = 1500
 */
int led_code1_len = 14;
int led_code1[ROM_CODE_MAX] = {
	0xbf00,0x854,0x900,0x1c0d,0x1d0e,0x81e,0x90b,0x4401,0x9fa,0x4301,0x3c7c,0x4201,0x3cbb,0x9,
};

/*
 * rise_time_ms = 1000
 * hold_time_ms = 1000
 * fall_time_ms = 1000
 * off_time_ms = 2000
 */
int led_code2_len = 14;
int led_code2[ROM_CODE_MAX] = {
	0xbf00,0x854,0x900,0x1c0d,0x1d0e,0x81e,0x907,0x4401,0x9fa,0x4301,0x3c7c,0x4201,0x3cbb,0x9,
};
/**********************************led breath code stop**************************************************/

//////////////////////////////////////////////////////////////////////////////////////
// Touch process variable
//////////////////////////////////////////////////////////////////////////////////////
static unsigned char suspend_flag = 0 ; //0: normal; 1: sleep
static int debug_level=0;
static int WorkMode = 1 ; //1

//////////////////////////////////////////////////////////////////////////////////////
// Function
//////////////////////////////////////////////////////////////////////////////////////
static int AW9233_create_sysfs(struct i2c_client *client);


static int AW9233_i2c_suspend(struct device *dev);

static int AW9233_i2c_resume(struct device *dev);



//////////////////////////////////////////////////////////////////////////////////////////
// PDN power control
//////////////////////////////////////////////////////////////////////////////////////////
struct pinctrl *aw9233ctrl = NULL;
struct pinctrl_state *aw9233_int_pin = NULL;
struct pinctrl_state *aw9233_pdn_high = NULL;
struct pinctrl_state *aw9233_pdn_low = NULL;

int AW9233_gpio_init(struct i2c_client *client)
{
	int ret = 0;

	aw9233ctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(aw9233ctrl)) {
		printk("Cannot find  pinctrl!");
		ret = PTR_ERR(aw9233ctrl);
		printk("%s devm_pinctrl_get fail!\n", __func__);
	}
	aw9233_pdn_high = pinctrl_lookup_state(aw9233ctrl, "aw9233_pdn_high");
	if (IS_ERR(aw9233_pdn_high)) {
		ret = PTR_ERR(aw9233_pdn_high);
		printk("%s : pinctrl err, aw9233_pdn_high\n", __func__);
	}

	aw9233_pdn_low = pinctrl_lookup_state(aw9233ctrl, "aw9233_pdn_low");
	if (IS_ERR(aw9233_pdn_low)) {
		ret = PTR_ERR(aw9233_pdn_low);
		printk("%s : pinctrl err, aw9233_pdn_low\n", __func__);
	}
	printk("%s success\n", __func__);
	return ret;
}

static void AW9233_ts_pwron(void)
{
	
	int ret = 0;
	printk("%s enter\n", __func__);


	if (!IS_ERR_OR_NULL(aw9233_pdn_low) && !IS_ERR_OR_NULL(aw9233ctrl)) {
		ret = pinctrl_select_state(aw9233ctrl, aw9233_pdn_low);
		if (ret) {
			printk("cannot set low pins\n");			
		}
	} else{
      		printk("pin ctrl is not a valide pin ctrl");	
	}
	msleep(5);
	
	if (!IS_ERR_OR_NULL(aw9233_pdn_high) && !IS_ERR_OR_NULL(aw9233ctrl)) {
		ret = pinctrl_select_state(aw9233ctrl, aw9233_pdn_high);
		if (ret) {
			printk("cannot set high pins");			
		}
	} else{
        printk("pin ctrl is not a valide pin ctrl");	
	}
	
	msleep(10);

	printk("%s out\n", __func__);
}


/*
static void AW9233_ts_pwroff(void)
{
	printk("%s enter\n", __func__);
	pinctrl_select_state(aw9233ctrl, aw9233_pdn_low);
	msleep(5);
	printk("%s out\n", __func__);
}
*/

static void AW9233_ts_config_pins(void)
{
	printk("%s enter\n", __func__);
	AW9233_ts_pwron();
	printk("%s out\n", __func__);
}

//////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////
unsigned int aw9233_I2C_write_reg(unsigned char addr, unsigned int reg_data)
{
	int ret;
	u8 wdbuf[512] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= aw9233_i2c_client->addr,
			.flags	= 0,
			.len	= 3,
			.buf	= wdbuf,
		},
	};

	wdbuf[0] = addr;
	wdbuf[2] = (unsigned char)(reg_data & 0x00ff);
	wdbuf[1] = (unsigned char)((reg_data & 0xff00)>>8);

	ret = i2c_transfer(aw9233_i2c_client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return ret;

}


unsigned int aw9233_I2C_read_reg(unsigned char addr)
{
	int ret;
	u8 rdbuf[512] = {0};
	unsigned int getdata;

	struct i2c_msg msgs[] = {
		{
			.addr	= aw9233_i2c_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rdbuf,
		},
		{
			.addr	= aw9233_i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= 2,
			.buf	= rdbuf,
		},
	};

	rdbuf[0] = addr;

	ret = i2c_transfer(aw9233_i2c_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	getdata=rdbuf[0] & 0x00ff;
	getdata<<= 8;
	getdata |=rdbuf[1];

    return getdata;
}


//////////////////////////////////////////////////////////////////////
// AW9233 initial register @ mobile active
//////////////////////////////////////////////////////////////////////
void AW_NormalMode(void)
{
	aw9233_I2C_write_reg(GCR,0x0000);			// disable chip

	///////////////////////////////////////
	// LED config
	///////////////////////////////////////
	aw9233_I2C_write_reg(LER1,N_LER1);			// LED enable
	aw9233_I2C_write_reg(LER2,N_LER2);			// LED enable

	//aw9233_I2C_write_reg(CTRS1,0x0000);		// LED control RAM or I2C
	//aw9233_I2C_write_reg(CTRS2,0x0000);		// LED control RAM or I2C
	aw9233_I2C_write_reg(IMAX1,N_IMAX1);		// LED MAX light setting
	aw9233_I2C_write_reg(IMAX2,N_IMAX2);		// LED MAX light setting
	aw9233_I2C_write_reg(IMAX3,N_IMAX3);		// LED MAX light setting
	aw9233_I2C_write_reg(IMAX4,N_IMAX4);		// LED MAX light setting
	aw9233_I2C_write_reg(IMAX5,N_IMAX5);		// LED MAX light setting

	aw9233_I2C_write_reg(LCR,N_LCR);			// LED effect control
	aw9233_I2C_write_reg(IDLECR,N_IDLECR);		// IDLE time setting

	///////////////////////////////////////
	// cap-touch config
	///////////////////////////////////////
	aw9233_I2C_write_reg(SLPR,N_SLPR);			// touch key enable
	aw9233_I2C_write_reg(SCFG1,N_SCFG1);		// scan time setting
	aw9233_I2C_write_reg(SCFG2,N_SCFG2);		// bit0~3 is sense seting

	aw9233_I2C_write_reg(OFR2,N_OFR2);			// offset
	aw9233_I2C_write_reg(OFR3,N_OFR3);			// offset

	aw9233_I2C_write_reg(THR2, N_THR2);		// S1 press thred setting
	aw9233_I2C_write_reg(THR3, N_THR3);		// S2 press thred setting
	aw9233_I2C_write_reg(THR4, N_THR4);		// S3 press thred setting

	aw9233_I2C_write_reg(SETCNT,N_SETCNT);		// debounce
	aw9233_I2C_write_reg(BLCTH,N_BLCTH);		// base trace rate 

	aw9233_I2C_write_reg(AKSR,N_AKSR);			// AKS 
#ifndef AW_AUTO_CALI
	aw9233_I2C_write_reg(INTER,N_INTER);	 	// signel click interrupt 
#else
	aw9233_I2C_write_reg(INTER,0x0080);	 	// frame interrupt 
#endif

	aw9233_I2C_write_reg(MPTR,N_MPTR);			// Long Press Time 
	aw9233_I2C_write_reg(GDTR,N_GDTR);			// gesture time setting
	aw9233_I2C_write_reg(GDCFGR,N_GDCFGR);		// gesture key select
	aw9233_I2C_write_reg(TAPR1,N_TAPR1);		// double click 1
	aw9233_I2C_write_reg(TAPR2,N_TAPR2);		// double click 2
	aw9233_I2C_write_reg(TDTR,N_TDTR);			// double click time

#ifndef AW_AUTO_CALI
	aw9233_I2C_write_reg(GIER,N_GIER);			// gesture and double click enable
#else
	aw9233_I2C_write_reg(GIER,0x0000);			// gesture and double click disable
#endif	

	///////////////////////////////////////
	aw9233_I2C_write_reg(GCR,N_GCR);			// LED enable and touch scan enable

	WorkMode = 2;
	printk("%s Finish\n", __func__);

}


//////////////////////////////////////////////////////////////////////
// AW9233 initial register @ mobile sleep
//////////////////////////////////////////////////////////////////////
void AW_SleepMode(void)
{
	aw9233_I2C_write_reg(GCR,0x0000);   		// disable chip

    ///////////////////////////////////////
    // LED config
    ///////////////////////////////////////
	aw9233_I2C_write_reg(LER1,S_LER1);			// LED enable
	aw9233_I2C_write_reg(LER2,S_LER2);			// LED enable

	aw9233_I2C_write_reg(LCR,S_LCR);			// LED effect control
	aw9233_I2C_write_reg(IDLECR,S_IDLECR);		// IDLE time setting

	aw9233_I2C_write_reg(IMAX1,S_IMAX1);		// LED MAX light setting
	aw9233_I2C_write_reg(IMAX2,S_IMAX2);		// LED MAX light setting
	aw9233_I2C_write_reg(IMAX3,S_IMAX3);		// LED MAX light setting
	aw9233_I2C_write_reg(IMAX4,S_IMAX4);		// LED MAX light setting
	aw9233_I2C_write_reg(IMAX5,S_IMAX5);		// LED MAX light setting

	///////////////////////////////////////
	// cap-touch config
	///////////////////////////////////////
	aw9233_I2C_write_reg(SLPR,S_SLPR);			// touch key enable

	aw9233_I2C_write_reg(SCFG1,S_SCFG1);		// scan time setting
	aw9233_I2C_write_reg(SCFG2,S_SCFG2);		// bit0~3 is sense seting

	aw9233_I2C_write_reg(OFR2,S_OFR2);			// offset
	aw9233_I2C_write_reg(OFR3,S_OFR3);			// offset

	aw9233_I2C_write_reg(THR2, S_THR2);		// S1 press thred setting
	aw9233_I2C_write_reg(THR3, S_THR3);		// S2 press thred setting
	aw9233_I2C_write_reg(THR4, S_THR4);		// S3 press thred setting

	aw9233_I2C_write_reg(SETCNT,S_SETCNT);		// debounce
	aw9233_I2C_write_reg(IDLECR, S_IDLECR);	// idle mode
	aw9233_I2C_write_reg(BLCTH,S_BLCTH);		// base speed setting

	aw9233_I2C_write_reg(AKSR,S_AKSR);			// AKS

#ifndef AW_AUTO_CALI
	aw9233_I2C_write_reg(INTER,S_INTER);	 	// signel click interrupt 
#else
	aw9233_I2C_write_reg(INTER,0x0080);	 	// signel click interrupt 
#endif

	aw9233_I2C_write_reg(GDCFGR,S_GDCFGR);		// gesture key select
	aw9233_I2C_write_reg(TAPR1,S_TAPR1);		// double click 1
	aw9233_I2C_write_reg(TAPR2,S_TAPR2);		// double click 2

	aw9233_I2C_write_reg(TDTR,S_TDTR);			// double click time
#ifndef AW_AUTO_CALI
	aw9233_I2C_write_reg(GIER,S_GIER);			// gesture and double click enable
#else
	aw9233_I2C_write_reg(GIER,0x0000);			// gesture and double click disable
#endif

	///////////////////////////////////////
	aw9233_I2C_write_reg(GCR, S_GCR);   		// enable chip sensor function

	WorkMode = 1;
	printk("%s Finish\n", __func__);
}




void aw9233_led_breath(void)
{
	unsigned int reg_val;
	unsigned char i;
	
	// Enable LED Module
	reg_val = aw9233_I2C_read_reg(GCR);
	reg_val |= 0x0001;
	aw9233_I2C_write_reg(GCR, reg_val);

	// LED Config
	aw9233_I2C_write_reg(LER1,0x0054);		// LEDx Enable
	aw9233_I2C_write_reg(LCR,0x00A1);		// LEDx Enable
	aw9233_I2C_write_reg(IMAX1,0x0100);		// LED1 Current
	aw9233_I2C_write_reg(IMAX2,0x0101);		// LED2/LED3 Current
	aw9233_I2C_write_reg(CTRS1,0x0000);		// SRAM Program Control

	// Load LED Code
	aw9233_I2C_write_reg(PMR,0x0000);
	aw9233_I2C_write_reg(RMR,0x0000);
	aw9233_I2C_write_reg(WADDR,0x0000);

	for(i=0; i<led_code1_len; i++)
	{
		aw9233_I2C_write_reg(WDATA,led_code1[i]);
	}

	// LED Code Run
	aw9233_I2C_write_reg(SADDR,0x0000);
	aw9233_I2C_write_reg(PMR,0x0001);
	aw9233_I2C_write_reg(RMR,0x0002);
}


void AW9233_LED_OFF(void);
static void aw9233_led_huxi_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	if (!strcmp(led_cdev->name, "bln")) 
	{
		if(value == LED_OFF)
		{
			AW9233_LED_OFF();
		} else if(value == 1){
			aw9233_led_breath();
		}
	} 

}
void AW9233_LED_ON(void)
{
	aw9233_I2C_write_reg(IMAX1,0x3300);		// IMAX
	aw9233_I2C_write_reg(IMAX2,0x0003);		// IMAX
	aw9233_I2C_write_reg(LER1,0x001c);			// LEDx EN
	aw9233_I2C_write_reg(CTRS1,0x00fc);		// I2C Control
	aw9233_I2C_write_reg(CMDR,0xa2ff);			// LED1 ON
	aw9233_I2C_write_reg(CMDR,0xa3ff);			// LED2 ON
	aw9233_I2C_write_reg(CMDR,0xa4ff);			// LED3 ON
	aw9233_I2C_write_reg(GCR,0x0003);			// GCR
}

void AW9233_LED_OFF(void)
{
	aw9233_I2C_write_reg(IMAX1,0x3300);		// IMAX
	aw9233_I2C_write_reg(IMAX2,0x0003);		// IMAX
	aw9233_I2C_write_reg(LER1,0x001c);			// LEDx EN
	aw9233_I2C_write_reg(CTRS1,0x00fc);		// I2C Control
	aw9233_I2C_write_reg(CMDR,0xa200);			// LED1 OFF
	aw9233_I2C_write_reg(CMDR,0xa300);			// LED2 OFF
	aw9233_I2C_write_reg(CMDR,0xa400);			// LED3 OFF
	aw9233_I2C_write_reg(GCR,0x0003);			// GCR
}

////////////////////////////////////////////////////////////////////////
// AW91xx Auto Calibration
////////////////////////////////////////////////////////////////////////
#ifdef AW_AUTO_CALI
unsigned char AW91xx_Auto_Cali(void)
{
	unsigned char i;
	unsigned char cali_dir[6];

	unsigned int buf[6];
	unsigned int ofr_cfg[6];
	unsigned int sen_num;

	if(cali_num == 0){
		ofr_cfg[0] = aw9233_I2C_read_reg(0x13);
		ofr_cfg[1] = aw9233_I2C_read_reg(0x14);
		ofr_cfg[2] = aw9233_I2C_read_reg(0x15);
	}else{
		for(i=0; i<3; i++){
			ofr_cfg[i] = old_ofr_cfg[i];
		}
	}

	aw9233_I2C_write_reg(0x1e,0x3);
	for(i=0; i<6; i++){
		buf[i] = aw9233_I2C_read_reg(0x36+i);
	}
	sen_num = aw9233_I2C_read_reg(0x02);		// SLPR

	for(i=0; i<6; i++) 
		Ini_sum[i] = (cali_cnt==0)? (0) : (Ini_sum[i] + buf[i]);

	if(cali_cnt==4){
		for(i=0; i<6; i++){
			if((sen_num & (1<<i)) == 0)	{	// sensor used
				if((Ini_sum[i]>>2) < CALI_RAW_MIN){
					if((i%2) && ((ofr_cfg[i>>1]&0xFF00)==0x1000)){					// 0x10** -> 0x00**
						ofr_cfg[i>>1] = ofr_cfg[i>>1] & 0x00FF;
						cali_dir[i] = 2;
					}else if((i%2) && ((ofr_cfg[i>>1]&0xFF00)==0x0000)){			// 0x00**	no calibration
						cali_dir[i] = 0;
					}else if (((i%2)==0) && ((ofr_cfg[i>>1]&0x00FF)==0x0010)){		// 0x**10 -> 0x**00
						ofr_cfg[i>>1] = ofr_cfg[i>>1] & 0xFF00;
						cali_dir[i] = 2;
					}else if (((i%2)==0) && ((ofr_cfg[i>>1]&0x00FF)==0x0000)){		// 0x**00 no calibration
						cali_dir[i] = 0;
					}else{
						ofr_cfg[i>>1] = ofr_cfg[i>>1] - ((i%2)? (1<<8):1);
						cali_dir[i] = 2;
					}
				}else if((Ini_sum[i]>>2) > CALI_RAW_MAX){
					if((i%2) && ((ofr_cfg[i>>1]&0xFF00)==0x1F00)){	// 0x1F** no calibration
						cali_dir[i] = 0;
					}else if (((i%2)==0) && ((ofr_cfg[i>>1]&0x00FF)==0x001F)){	// 0x**1F no calibration
						cali_dir[i] = 0;
					}else{
						ofr_cfg[i>>1] = ofr_cfg[i>>1] + ((i%2)? (1<<8):1);
						cali_dir[i] = 1;
					}
				}else{
					cali_dir[i] = 0;
				}

				if(cali_num > 0){
					if(cali_dir[i] != old_cali_dir[i]){
						cali_dir[i] = 0;
						ofr_cfg[i>>1] = old_ofr_cfg[i>>1];
					}
				}
			}
		}

		cali_flag = 0;
		for(i=0; i<6; i++){
			if((sen_num & (1<<i)) == 0)	{	// sensor used
				if(cali_dir[i] != 0){
					cali_flag = 1;
				}
			}
		}
		if((cali_flag==0) && (cali_num==0)){
			cali_used = 0;
		}else{
			cali_used = 1;
		}

		if(cali_flag == 0){
			cali_num = 0;
			cali_cnt = 0;
			return 0;
		}

		aw9233_I2C_write_reg(GCR, 0x0000);
		for(i=0; i<3; i++){
			aw9233_I2C_write_reg(OFR1+i, ofr_cfg[i]);
		}
		aw9233_I2C_write_reg(GCR, 0x0003);

		if(cali_num == (CALI_NUM -1)){	// no calibration
			cali_flag = 0;
			cali_num = 0;
			cali_cnt = 0;
			return 0;
		}

		for(i=0; i<6; i++){
			old_cali_dir[i] = cali_dir[i];
		}

		for(i=0; i<3; i++){
			old_ofr_cfg[i] = ofr_cfg[i];
		}

		cali_num ++;
	}

	if(cali_cnt < 4){
		cali_cnt ++;
	}else{
		cali_cnt = 0;
	}

	return 1;
}
#endif

/////////////////////////////////////////////
// report left slip 
/////////////////////////////////////////////
void AW_left_slip(void)
{
	printk("AW9233 left slip \n");
	input_report_key(AW9233_ts->input_dev, KEY_F1, 1);
	input_sync(AW9233_ts->input_dev);
	input_report_key(AW9233_ts->input_dev, KEY_F1, 0);
	input_sync(AW9233_ts->input_dev);
}


/////////////////////////////////////////////
// report right slip 
/////////////////////////////////////////////
void AW_right_slip(void)
{
	printk("AW9233 right slip \n");
	input_report_key(AW9233_ts->input_dev, KEY_F2, 1);
	input_sync(AW9233_ts->input_dev);
	input_report_key(AW9233_ts->input_dev, KEY_F2, 0);
	input_sync(AW9233_ts->input_dev);
}


/////////////////////////////////////////////
// report MENU-BTN double click 
/////////////////////////////////////////////
void AW_left_double(void)
{
	printk("AW9233 Left double click \n");
	input_report_key(AW9233_ts->input_dev, KEY_PREVIOUSSONG, 1);
	input_sync(AW9233_ts->input_dev);
	input_report_key(AW9233_ts->input_dev, KEY_PREVIOUSSONG, 0);
	input_sync(AW9233_ts->input_dev);
}

/////////////////////////////////////////////
// report HOME-BTN double click 
/////////////////////////////////////////////
void AW_center_double(void)
{
	printk("AW9233 Center double click \n");
	input_report_key(AW9233_ts->input_dev, KEY_F3, 1);
	input_sync(AW9233_ts->input_dev);
	input_report_key(AW9233_ts->input_dev, KEY_F3, 0);
	input_sync(AW9233_ts->input_dev);
}
/////////////////////////////////////////////
// report BACK-BTN double click 
/////////////////////////////////////////////
void AW_right_double(void)
{
	printk("AW9233 Right_double click \n");
	input_report_key(AW9233_ts->input_dev, KEY_NEXTSONG, 1);
	input_sync(AW9233_ts->input_dev);
	input_report_key(AW9233_ts->input_dev, KEY_NEXTSONG, 0);
	input_sync(AW9233_ts->input_dev);
}

/////////////////////////////////////////////
// report MENU-BTN single click 
/////////////////////////////////////////////
void AW_left_press(void)
{
	input_report_key(AW9233_ts->input_dev, KEY_MENU, 1);
	input_sync(AW9233_ts->input_dev);
	printk("AW9233 left press \n");
}

/////////////////////////////////////////////
// report HOME-BTN single click 
/////////////////////////////////////////////
void AW_center_press(void)
{
	printk("AW9233 center press \n");
	input_report_key(AW9233_ts->input_dev, KEY_HOMEPAGE, 1);
	input_sync(AW9233_ts->input_dev);
}
/////////////////////////////////////////////
// report BACK-BTN single click 
/////////////////////////////////////////////
void AW_right_press(void)
{
	input_report_key(AW9233_ts->input_dev, KEY_BACK, 1);
	input_sync(AW9233_ts->input_dev);
	printk("AW9233 right press \n");
}

/////////////////////////////////////////////
// report MENU-BTN single click 
/////////////////////////////////////////////
void AW_left_release(void)
{
	input_report_key(AW9233_ts->input_dev, KEY_MENU, 0);
	input_sync(AW9233_ts->input_dev);
	printk("AW9233 left release\n");
}

/////////////////////////////////////////////
// report HOME-BTN single click 
/////////////////////////////////////////////
void AW_center_release(void)
{
	input_report_key(AW9233_ts->input_dev, KEY_HOMEPAGE, 0);
	input_sync(AW9233_ts->input_dev);
	printk("AW9233 center release \n");

}
/////////////////////////////////////////////
// report BACK-BTN single click 
/////////////////////////////////////////////
void AW_right_release(void)
{
	input_report_key(AW9233_ts->input_dev, KEY_BACK, 0);
	input_sync(AW9233_ts->input_dev);
	printk("AW9233 right release \n");
}


////////////////////////////////////////////////////
//
// Function : Cap-touch main program @ mobile sleep 
//            wake up after double-click/right_slip/left_slip
//
////////////////////////////////////////////////////
void AW_SleepMode_Proc(void)
{
	unsigned int buff1;

	printk("%s Enter\n", __func__);

#ifdef AW_AUTO_CALI
	if(cali_flag){
		AW91xx_Auto_Cali();
		if(cali_flag == 0){	
			if(cali_used){
				aw9233_I2C_write_reg(GCR,0x0000);	 // disable chip
			}
			aw9233_I2C_write_reg(INTER,S_INTER);
			aw9233_I2C_write_reg(GIER,S_GIER);
			if(cali_used){
				aw9233_I2C_write_reg(GCR,S_GCR);	 // enable chip
			}
		}
		return ;
	}
#endif
	
	if(debug_level == 0){
		buff1=aw9233_I2C_read_reg(0x2e);			//read gesture interupt status
		if(buff1 == 0x10){
				AW_center_double();
		}else if(buff1 == 0x01){
				AW_right_slip();
		}else if (buff1 == 0x02){
				AW_left_slip();
		}
	}
}

////////////////////////////////////////////////////
//
// Function : Cap-touch main pragram @ mobile normal state
//            press/release
//
////////////////////////////////////////////////////
void AW_NormalMode_Proc(void)
{
	unsigned int buff1,buff2;

	printk("%s Enter\n", __func__);

#ifdef AW_AUTO_CALI
	printk("AW_NormalMode_Proc cali_flag: %c, cali_used:%c \n",cali_flag,cali_used);
	if(cali_flag){
		AW91xx_Auto_Cali();
		if(cali_flag == 0){	
			if(cali_used){
				aw9233_I2C_write_reg(GCR,0x0000);	 // disable chip
			}
			aw9233_I2C_write_reg(INTER,N_INTER);
			aw9233_I2C_write_reg(GIER,N_GIER);
			if(cali_used){
				aw9233_I2C_write_reg(GCR,N_GCR);	 // enable chip
			}
		}
		return ;
	}
#endif
	if(debug_level == 0){
		buff2=aw9233_I2C_read_reg(0x32);		//read key interupt status
		buff1=aw9233_I2C_read_reg(0x31);

	//	printk("AW9233: reg0x31=0x%x, reg0x32=0x%x\n",buff1,buff2);

		if(buff2 & 0x10){						//S3 click
			if(buff1 == 0x00){
					AW_left_release();
			}else if(buff1 == 0x10){
					AW9233_LED_ON();
					AW_left_press();					
					cancel_delayed_work_sync(&AW9233_ts->led_work);
					schedule_delayed_work(&AW9233_ts->led_work, msecs_to_jiffies(5000));
			}
		}else if(buff2 & 0x08){					//S2 click
			if(buff1 == 0x00){
					AW_center_release();
			}else if (buff1 == 0x08){
					AW9233_LED_ON();
					AW_center_press();					
					cancel_delayed_work_sync(&AW9233_ts->led_work);
					schedule_delayed_work(&AW9233_ts->led_work, msecs_to_jiffies(5000));
			}
		}else if(buff2 & 0x04){					//S1 click
			if(buff1 == 0x00){
				   AW_right_release();
			}else if(buff1 == 0x04){
				AW9233_LED_ON();
				 AW_right_press();				 
				 cancel_delayed_work_sync(&AW9233_ts->led_work);
				 schedule_delayed_work(&AW9233_ts->led_work, msecs_to_jiffies(5000));
			}
		}
	}
}


static int AW9233_ts_clear_intr(struct i2c_client *client) 
{
	int res;
	res = aw9233_I2C_read_reg(0x32);
	//printk("%s: reg0x32=0x%x\n", __func__, res);
	
	return 0;
}

////////////////////////////////////////////////////
//
// Function : Interrupt sub-program
//            work in AW_SleepMode_Proc() or 
//            AW_NormalMode_Proc()
//
////////////////////////////////////////////////////
static void AW9233_ts_eint_work(struct work_struct *work)
{

	switch(WorkMode){
		case 1:
			AW_SleepMode_Proc();
			break;
		case 2:
			AW_NormalMode_Proc();
			break;
		default:
			break;
	}

	AW9233_ts_clear_intr(aw9233_i2c_client);


	enable_irq(AW9233_ts->irq);
}
static void AW9233_led_work(struct work_struct *work)
{
		AW9233_LED_OFF();
}

static irqreturn_t AW9233_ts_eint_func(int irq, void *desc)
{

	disable_irq_nosync(irq);

	if(AW9233_ts == NULL){
		printk("%s: AW9233_ts == NULL", __func__);
		return  IRQ_NONE;
	}

	schedule_work(&AW9233_ts->eint_work);

	return IRQ_HANDLED;
}


int AW9233_ts_setup_eint(void)
{
	int ret = 0;
	u32 ints[2] = {0, 0};

	aw9233_int_pin = pinctrl_lookup_state(aw9233ctrl, "aw9233_int_pin");
	if (IS_ERR(aw9233_int_pin)) {
		ret = PTR_ERR(aw9233_int_pin);
		printk("%s : pinctrl err, aw9233_int_pin\n", __func__);
		return -EINVAL;
	}

	if (AW9233_ts->irq_node) {
		of_property_read_u32_array(AW9233_ts->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(aw9233ctrl, aw9233_int_pin);
		printk("%s ints[0] = %d, ints[1] = %d!!\n", __func__, ints[0], ints[1]);

		AW9233_ts->irq = irq_of_parse_and_map(AW9233_ts->irq_node, 0);
		printk("%s irq = %d\n", __func__, AW9233_ts->irq);
		if (!AW9233_ts->irq) {
			printk("%s irq_of_parse_and_map fail!!\n", __func__);
			return -EINVAL;
		}
		if (request_irq(AW9233_ts->irq, AW9233_ts_eint_func, IRQF_ONESHOT | IRQ_TYPE_LEVEL_LOW, "AW9233-eint", NULL)) {
			printk("%s IRQ LINE NOT AVAILABLE!!\n", __func__);
			return -EINVAL;
		}
	//	enable_irq(AW9233_ts->irq);
	} else {
		printk("null irq node!!\n");
		return -EINVAL;
	}

    return 0;
}

////////////////////////////////////////////////////
//
// Function : AW9233 initial @ mobile goto sleep mode
//            enter SleepMode
//
////////////////////////////////////////////////////
static int AW9233_i2c_suspend(struct device *dev)
{
	if(WorkMode != 1){
		AW_SleepMode();
		suspend_flag = 1;
#ifdef AW_AUTO_CALI
		cali_flag = 1;
		cali_num = 0;
		cali_cnt = 0;
#endif
	}
	printk("%s Finish\n", __func__);

	return 0;
}

////////////////////////////////////////////////////
//
// Function : AW9233 initial @ mobile wake up
//            enter NormalMode 
//
////////////////////////////////////////////////////
static int AW9233_i2c_resume(struct device *dev)
{	
	if(WorkMode != 2){
		AW_NormalMode();
		suspend_flag = 0;
#ifdef AW_AUTO_CALI
		cali_flag = 1;
		cali_num = 0;
		cali_cnt = 0;
#endif
	}
	printk("%s Finish\n", __func__);

	return 0;
}

////////////////////////////////////////////////////
//
// Function : fb_notifier_callback
//
////////////////////////////////////////////////////
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct AW9233_ts_data *a9233_data =
        container_of(self, struct AW9233_ts_data, fb_notif);

    if (evdata && evdata->data && event == FB_EVENT_BLANK &&
        a9233_data && aw9233_i2c_client)
    {
        blank = evdata->data;
        if (*blank == FB_BLANK_UNBLANK)
            AW9233_i2c_resume(&aw9233_i2c_client->dev);
        else if (*blank == FB_BLANK_POWERDOWN)
            AW9233_i2c_suspend(&aw9233_i2c_client->dev);
    }

    return 0;
}

//static SIMPLE_DEV_PM_OPS(AW9233_pm_ops, AW9233_i2c_suspend, AW9233_i2c_resume);
////////////////////////////////////////////////////
//
// Function : AW9233 initial @ mobile power on
//            enter NormalMode directly
//
////////////////////////////////////////////////////

//	ret = AW9233_gpio_init(pdev);


static void aw9233_led_button_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{	
	if (!strcmp(led_cdev->name, "button-backlight")) 
	{
		if(value == LED_OFF)
		{
			AW9233_LED_OFF();
		} else {
			AW9233_LED_ON();
		}
	} 	
	
}



static int AW9233_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct input_dev *input_dev;
	int err = 0;
	int count =0;
	unsigned int reg_value,reg_value1; 

	printk("%s Enter\n", __func__);

	err = AW9233_gpio_init(client);
	if (err != 0) {
		printk("[%s] failed to init AW9233 pinctrl.\n", __func__);
		goto exit_alloc_data_failed;
	} else {
		printk("[%s] Success to init AW9233 pinctrl.\n", __func__);
	}


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	printk("%s: kzalloc\n", __func__);
	AW9233_ts = kzalloc(sizeof(*AW9233_ts), GFP_KERNEL);
	if (!AW9233_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	AW9233_led_button= kzalloc(sizeof(*AW9233_led_button), GFP_KERNEL);
	if (!AW9233_led_button)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	AW9233_led_huxi= kzalloc(sizeof(*AW9233_led_huxi), GFP_KERNEL);
	if (!AW9233_led_huxi)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	AW9233_ts_config_pins();

	client->addr = 0x2c; 	//0x2C;		gaoyuan					// chip  I2C address
	//client->timing= 400;	//gaoyuan
	aw9233_i2c_client = client;
	i2c_set_clientdata(client, AW9233_ts);

	printk("aw9233 I2C addr=%x", client->addr);
	for(count = 0; count < 5; count++){
		reg_value = aw9233_I2C_read_reg(0x00);				//read chip ID
		printk("AW9233 chip ID = 0x%4x\n", reg_value);

		if (reg_value == 0xb223)
			break;

		msleep(20);

		if(count == 4) {
			err = -ENODEV;
			goto exit_create_singlethread;
		}
	}

	AW9233_ts->irq_node = of_find_compatible_node(NULL, NULL, "awinic,aw9233_i2c");	//gaoyuan 


	INIT_WORK(&AW9233_ts->eint_work, AW9233_ts_eint_work);
	INIT_DELAYED_WORK(&AW9233_ts->led_work, AW9233_led_work);

	input_dev = input_allocate_device();
	if (!input_dev){
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	AW9233_ts->input_dev = input_dev;


	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	
	__set_bit(KEY_HOMEPAGE, input_dev->keybit);
	__set_bit(KEY_MENU, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);
	__set_bit(KEY_PREVIOUSSONG, input_dev->keybit);
	__set_bit(KEY_NEXTSONG, input_dev->keybit);
	__set_bit(KEY_F1, input_dev->keybit);
	__set_bit(KEY_F2, input_dev->keybit);
	__set_bit(KEY_F3, input_dev->keybit);
	
	input_dev->name = AW9233_ts_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"AW9233_i2c_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}	

	AW9233_create_sysfs(client);

	AW9233_led_button->cdev.name = "button-backlight";
	AW9233_led_button->client = client;
	AW9233_led_button->cdev.brightness = 0;
	AW9233_led_button->cdev.max_brightness = 0xff;
	AW9233_led_button->cdev.brightness_set= aw9233_led_button_set;
	led_classdev_register(&AW9233_led_button->client->dev, &AW9233_led_button->cdev);

	AW9233_led_huxi->cdev.name = "bln";
	AW9233_led_huxi->client = client;
	AW9233_led_huxi->cdev.brightness = 0;
	AW9233_led_huxi->cdev.max_brightness = 0xff;
	AW9233_led_huxi->cdev.brightness_set= aw9233_led_huxi_set;//aw9233_led_breath;
	led_classdev_register(&AW9233_led_huxi->client->dev, &AW9233_led_huxi->cdev);	

	WorkMode = 2;
	AW_NormalMode();

#ifdef AW_AUTO_CALI
	cali_flag = 1;
	cali_num = 0;
	cali_cnt = 0;
#endif

	reg_value1 = aw9233_I2C_read_reg(0x01);
 	printk("AW9233 GCR = 0x%4x\n", reg_value1);

	AW9233_ts_setup_eint();

	AW9233_ts->fb_notif.notifier_call = fb_notifier_callback;
    	err = fb_register_client(&AW9233_ts->fb_notif);
   	if (err)
       	 printk("[FB]Unable to register fb_notifier: %d", err);


	printk("%s Over\n", __func__);
    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	//free_irq(client->irq, AW9233_ts);
	cancel_work_sync(&AW9233_ts->eint_work);
	cancel_delayed_work_sync(&AW9233_ts->led_work);
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(AW9233_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
/***********************************************************************************************
Name	:	 

Input	:	

Output	:	

function	:

***********************************************************************************************/
static int AW9233_i2c_remove(struct i2c_client *client)
{
	struct AW9233_ts_data *AW9233_ts = i2c_get_clientdata(client);

	printk("%s enter\n", __func__);

	input_unregister_device(AW9233_ts->input_dev);
	kfree(AW9233_ts);
	i2c_set_clientdata(client, NULL);
	
	return 0;
}

static const struct i2c_device_id AW9233_i2c_id[] = {
	{ AW9233_ts_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, AW9233_ts_id);


#ifdef CONFIG_OF
static const struct of_device_id AW9233_of_match[] = {
	{.compatible = "awinic,aw9233_i2c"},
	{},
};
#endif
static struct i2c_driver AW9233_i2c_driver = {
	.probe		= AW9233_i2c_probe,
	.remove		= AW9233_i2c_remove,
	.id_table	= AW9233_i2c_id,
	.driver	= {
		.name	= AW9233_ts_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = AW9233_of_match,
#endif	
	},
};


//////////////////////////////////////////////////////
//
// for adb shell and APK debug
//
//////////////////////////////////////////////////////
static ssize_t AW9233_show_debug(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9233_store_debug(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW9233_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9233_write_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW9233_get_adbBase(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9233_get_rawdata(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9233_get_delta(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9233_get_irqstate(struct device* cd,struct device_attribute *attr, char* buf);

static DEVICE_ATTR(debug, 0664, AW9233_show_debug, AW9233_store_debug);
static DEVICE_ATTR(getreg, 0664, AW9233_get_reg, AW9233_write_reg);
static DEVICE_ATTR(adbbase, 0444, AW9233_get_adbBase, NULL);
static DEVICE_ATTR(rawdata, 0444, AW9233_get_rawdata, NULL);
static DEVICE_ATTR(delta, 0444, AW9233_get_delta, NULL);
static DEVICE_ATTR(getstate, 0444, AW9233_get_irqstate, NULL);

static ssize_t AW9233_show_debug(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;
	
	sprintf(buf, "AW9233 Debug %d\n",debug_level);
	
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t AW9233_store_debug(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	debug_level = on_off;

	printk("%s: debug_level=%d\n",__func__, debug_level);
	
	return len;
}



static ssize_t AW9233_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned int reg_val[1];
	ssize_t len = 0;
	u8 i;
	disable_irq(AW9233_ts->irq);
	for(i=1;i<0x7F;i++) {
		reg_val[0] = aw9233_I2C_read_reg(i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg%2X = 0x%4X, ", i,reg_val[0]);
	}
	enable_irq(AW9233_ts->irq);
	return len;
}

static ssize_t AW9233_write_reg(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{

	unsigned int databuf[2];
	disable_irq(AW9233_ts->irq);
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1])) {
		aw9233_I2C_write_reg((u8)databuf[0],databuf[1]);
	}
	enable_irq(AW9233_ts->irq);
	return len;
}

static ssize_t AW9233_get_adbBase(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned int dataS1,dataS2,dataS3,dataS4,dataS5,dataS6;
	ssize_t len = 0;

	disable_irq(AW9233_ts->irq);
	len += snprintf(buf+len, PAGE_SIZE-len, "base: \n");
	aw9233_I2C_write_reg(MCR,0x0003);

	dataS1=aw9233_I2C_read_reg(0x36);
	dataS2=aw9233_I2C_read_reg(0x37);
	dataS3=aw9233_I2C_read_reg(0x38);
	dataS4=aw9233_I2C_read_reg(0x39);
	dataS5=aw9233_I2C_read_reg(0x3a);
	dataS6=aw9233_I2C_read_reg(0x3b);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS1);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS2);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS3);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS4);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS5);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS6);

	len += snprintf(buf+len, PAGE_SIZE-len, "\n");

	enable_irq(AW9233_ts->irq);
	return len;
}

static ssize_t AW9233_get_rawdata(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned int dataS1,dataS2,dataS3,dataS4,dataS5,dataS6;
	ssize_t len = 0;

	disable_irq(AW9233_ts->irq);
	len += snprintf(buf+len, PAGE_SIZE-len, "base: \n");
	aw9233_I2C_write_reg(MCR,0x0003);

	dataS1=aw9233_I2C_read_reg(0x36);
	dataS2=aw9233_I2C_read_reg(0x37);
	dataS3=aw9233_I2C_read_reg(0x38);
	dataS4=aw9233_I2C_read_reg(0x39);
	dataS5=aw9233_I2C_read_reg(0x3a);
	dataS6=aw9233_I2C_read_reg(0x3b);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS1);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS2);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS3);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS4);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS5);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",dataS6);

	len += snprintf(buf+len, PAGE_SIZE-len, "\n");

	enable_irq(AW9233_ts->irq);
	return len;
}

static ssize_t AW9233_get_delta(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned int deltaS1,deltaS2,deltaS3,deltaS4,deltaS5,deltaS6;
	ssize_t len = 0;

	disable_irq(AW9233_ts->irq);
	len += snprintf(buf+len, PAGE_SIZE-len, "delta: \n");
	aw9233_I2C_write_reg(MCR,0x0001);

	deltaS1=aw9233_I2C_read_reg(0x36);if((deltaS1 & 0x8000) == 0x8000) { deltaS1 = 0; }
	deltaS2=aw9233_I2C_read_reg(0x37);if((deltaS2 & 0x8000) == 0x8000) { deltaS2 = 0; }
	deltaS3=aw9233_I2C_read_reg(0x38);if((deltaS3 & 0x8000) == 0x8000) { deltaS3 = 0; }
	deltaS4=aw9233_I2C_read_reg(0x39);if((deltaS4 & 0x8000) == 0x8000) { deltaS4 = 0; }
	deltaS5=aw9233_I2C_read_reg(0x3a);if((deltaS5 & 0x8000) == 0x8000) { deltaS5 = 0; }
	deltaS6=aw9233_I2C_read_reg(0x3b);if((deltaS6 & 0x8000) == 0x8000) { deltaS6 = 0; }
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaS1);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaS2);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaS3);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaS4);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaS5);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",deltaS6);

	len += snprintf(buf+len, PAGE_SIZE-len, "\n");

	enable_irq(AW9233_ts->irq);
	return len;
}

static ssize_t AW9233_get_irqstate(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned int keytouch,keyS1,keyS2,keyS3,keyS4,keyS5,keyS6;
	unsigned int gesture,slide1,slide2,slide3,slide4,doubleclick1,doubleclick2;
	ssize_t len = 0;

	disable_irq(AW9233_ts->irq);
	len += snprintf(buf+len, PAGE_SIZE-len, "keytouch: \n");

	keytouch=aw9233_I2C_read_reg(0x31);
	if((keytouch&0x1) == 0x1) keyS1=1;else keyS1=0;
	if((keytouch&0x2) == 0x2) keyS2=1;else keyS2=0;
	if((keytouch&0x4) == 0x4) keyS3=1;else keyS3=0;
	if((keytouch&0x8) == 0x8) keyS4=1;else keyS4=0;
	if((keytouch&0x10) == 0x10) keyS5=1;else keyS5=0;
	if((keytouch&0x20) == 0x20) keyS6=1;else keyS6=0;

	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",keyS1);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",keyS2);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",keyS3);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",keyS4);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",keyS5);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",keyS6);
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");

	len += snprintf(buf+len, PAGE_SIZE-len, "gesture: \n");			
	gesture=aw9233_I2C_read_reg(0x2e);
	if(gesture == 0x1) slide1=1;else slide1=0;
	if(gesture == 0x2) slide2=1;else slide2=0;
	if(gesture == 0x4) slide3=1;else slide3=0;
	if(gesture == 0x8) slide4=1;else slide4=0;
	if(gesture == 0x10) doubleclick1=1;else doubleclick1=0;
	if(gesture == 0x200) doubleclick2=1;else doubleclick2=0;

	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",slide1);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",slide2);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",slide3);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",slide4);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",doubleclick1);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",doubleclick2);

	len += snprintf(buf+len, PAGE_SIZE-len, "\n");

	enable_irq(AW9233_ts->irq);
	return len;
}


static int AW9233_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	printk("%s", __func__);
	
	err = device_create_file(dev, &dev_attr_debug);
	err = device_create_file(dev, &dev_attr_getreg);
	err = device_create_file(dev, &dev_attr_adbbase);
	err = device_create_file(dev, &dev_attr_rawdata);
	err = device_create_file(dev, &dev_attr_delta);
	err = device_create_file(dev, &dev_attr_getstate);
	return err;
}

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int __init AW9233_ts_init(void)
{
	int ret;
	printk("%s Enter\n", __func__);

	ret = i2c_add_driver(&AW9233_i2c_driver);
	if (ret) {
		printk("****[%s] Unable to register driver (%d)\n", __func__, ret);
		return ret;
	}		

	printk("%s Exit\n", __func__);

    return ret;
}

static void __exit AW9233_ts_exit(void)
{
	printk("%s Enter\n", __func__);
	i2c_del_driver(&AW9233_i2c_driver);
	printk("%s Exit\n", __func__);
}

late_initcall(AW9233_ts_init);
module_exit(AW9233_ts_exit);

MODULE_AUTHOR("<liweilei@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC AW9233 Touch driver");
MODULE_LICENSE("GPL v2");
