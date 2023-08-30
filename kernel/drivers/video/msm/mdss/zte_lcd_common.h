#ifndef ZTE_LCD_COMMON_H
#define ZTE_LCD_COMMON_H

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/sysfs.h>
#include <linux/proc_fs.h>

#define ZTE_LCD_COVERT_BACKLEVEL
//#define ZTE_GET_BOOT_MODE
#define ZTE_LCD_DEBUG



#define PANEL_NAME_MAX_LENTH 				 80
#define A01_ST7789V_PANEL 					 "zteLEAD(ST7789V)_320*240_2.8Inch"
#define K50_HX8394F_PANEL 					 "hx8394f lead 720p video mode dsi panel"
#define S10_ILI9881C_DIJING_PANEL 			 "ili9881c dijing s10 720p video mode dsi panel"
#define S10_ILI9881C_HOLITECH_PANEL 		 "ili9881c holitech 720p video mode dsi panel"
#define S10_ILI9881C_YASHI_PANEL			 "ili9881c yashi 720p video mode dsi panel"
#define S10_ILI9881C_SKYWORTH_PANEL 		 "ili9881c skyworth 720p video mode dsi panel"
#define S10_TD4100_PANEL					 "td4100 boe 720p video mode dsi panel"
#define F20_JD9161B_DIJING_PANEL			 "jd9161b dijing 480p video mode dsi panel"
#define F20_JD9161B_JINGTAI_PANEL			 "jd9161b jingtai 480p video mode dsi panel"
#define F20_JD9161B_TONGXINGDA_PANEL		 "jd9161b tongxingda 480p video mode dsi panel"
#define F20_ILI98806E_PANEL				 "ili9806e holitech 480p video mode dsi panel"
enum {
    LCD_TYPE_ST7789V = 0,                    //A01
    LCD_TYPE_HX8394F,                         //K50
    LCD_TYPE_ILI9881C_DIJING,           //S10 and F10
    LCD_TYPE_ILI9881C_HOLITECH,
    LCD_TYPE_ILI9881C_YASHI,
    LCD_TYPE_ILI9881C_SKYWORTH,
    LCD_TYPE_TD4100,
    LCD_TYPE_JD9161B_DIJING,            //F20
    LCD_TYPE_JD9161B_JINGTAI,
    LCD_TYPE_JD9161B_TONGXINGDA,
    LCD_TYPE_ILI98806E,
    LCD_TYPE_UNKNOWN
};

enum {
    BOOT_MODE_FTM               = 0,
    BOOT_MODE_RECOVERY,
    BOOT_MODE_FASTBOOT,
    BOOT_MODE_LCD_CYCLE_TEST,
    BOOT_MODE_CHARGER,
    BOOT_MODE_NORMAL,
    BOOT_MODE_FFBM,
    BOOT_MODE_UNKNOWN
};

struct zte_lcd_info{
	//char boot_mode;
	//char lcd_type;
	//char esd_read_reg_num;
	bool is_close_dimming;
	//u32  esd_interval_time;
	char lcd_panel_name[60];	
};

struct zte_lcd_reg_debug{
	bool is_read_mode;// 1 read ,0 write
	bool is_hs_mode;   // 1 hs mode, 0 lp mode
	char dtype;
	unsigned char length;
	char  rbuf[32];
	char wbuf[32];
	char reserved[64];
};

#ifdef ZTE_LCD_COVERT_BACKLEVEL
int  zte_covert_backlevel_function(int level,u32 bl_max);
#endif


#endif
