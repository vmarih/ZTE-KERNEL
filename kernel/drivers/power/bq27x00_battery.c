/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 * http://www.ti.com/product/bq27411-g1
 * http://www.ti.com/product/bq27421-g1
 * http://www.ti.com/product/bq27425-g1
 * http://www.ti.com/product/bq27441-g1
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/wakelock.h>    //added for suspend20170321
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include <linux/power/bq27x00_battery.h>
#include "bqfs_image.h"


//#define CONFIG_BATTERY_BQ27X00_I2C

#define DRIVER_VERSION			"1.2.0"

#define INVALID_REG_ADDR		0xFF

enum bq27xxx_reg_index {
	BQ27XXX_REG_CTRL = 0,
	BQ27XXX_REG_TEMP,
	BQ27XXX_REG_INT_TEMP,
	BQ27XXX_REG_VOLT,
	BQ27XXX_REG_AI,
	BQ27XXX_REG_FLAGS,
	BQ27XXX_REG_TTE,
	BQ27XXX_REG_TTF,
	BQ27XXX_REG_TTES,
	BQ27XXX_REG_TTECP,
	BQ27XXX_REG_NAC,
	BQ27XXX_REG_FCC,
	BQ27XXX_REG_CYCT,
	BQ27XXX_REG_AE,
	BQ27XXX_REG_SOC,
	BQ27XXX_REG_DCAP,
	BQ27XXX_POWER_AVG,
	NUM_REGS
};

enum bq_fg_subcmd {
	FG_SUBCMD_CTRL_STATUS	= 0x0000,
	FG_SUBCMD_PART_NUM		= 0x0001,
	FG_SUBCMD_FW_VER		= 0x0002,
	FG_SUBCMD_DM_CODE		= 0x0004,
	FG_SUBCMD_CHEM_ID		= 0x0008,
	FG_SUBCMD_BAT_INSERT	= 0x000C,
	FG_SUBCMD_BAT_REMOVE	= 0x000D,
	FG_SUBCMD_SET_CFGUPDATE	= 0x0013,
	FG_SUBCMD_SEAL			= 0x0020,
	FG_SUBCMD_CHEM_A		= 0x0030,
	FG_SUBCMD_CHEM_B		= 0x0031,
	FG_SUBCMD_CHEM_C		= 0x0032,
	FG_SUBCMD_HARD_RESET	= 0x0041,
	FG_SUBCMD_SOFT_RESET	= 0x0042,
};

enum {
	SEAL_STATE_FA,
	SEAL_STATE_UNSEALED,
	SEAL_STATE_SEALED,
};
struct batt_chem_id {
	u16 id;
	u16 cmd;
};
static struct batt_chem_id batt_chem_id_arr[] = {
	{3230, FG_SUBCMD_CHEM_A},
	{1202, FG_SUBCMD_CHEM_B},
	{3142, FG_SUBCMD_CHEM_C},
};

/* bq27500 registers */
static u8 bq27500_regs[NUM_REGS] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0xFF,	/* INT TEMP -NA	*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0A,	/* FLAGS	*/
	0x16,	/* TTE		*/
	0x18,	/* TTF		*/
	0x1c,	/* TTES		*/
	0x26,	/* TTECP	*/
	0x0C,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2A,	/* CYCT		*/
	0x22,	/* AE		*/
	0x2C,	/* SOC(RSOC)	*/
	0x3C,	/* DCAP(ILMD)	*/
	0x24,	/* AP		*/
};

/* bq27520 registers */
static u8 bq27520_regs[] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0xFF,	/* INT TEMP - NA*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0A,	/* FLAGS	*/
	0x16,	/* TTE		*/
	0x18,	/* TTF		*/
	0x1c,	/* TTES		*/
	0x26,	/* TTECP	*/
	0x0C,	/* NAC		*/
	0x12,	/* LMD		*/
	0xFF,	/* CYCT - NA	*/
	0x22,	/* AE		*/
	0x2C,	/* SOC(RSOC	*/
	0xFF,	/* DCAP(ILMD) - NA */
	0x24,	/* AP		*/
};

/* bq2753x registers */
static u8 bq2753x_regs[] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0xFF,	/* INT TEMP - NA*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0A,	/* FLAGS	*/
	0x16,	/* TTE		*/
	0xFF,	/* TTF - NA	*/
	0xFF,	/* TTES - NA	*/
	0xFF,	/* TTECP - NA	*/
	0x0C,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2A,	/* CYCT		*/
	0xFF,	/* AE - NA	*/
	0x2C,	/* SOC(RSOC)	*/
	0xFF,	/* DCAP(ILMD) - NA */
	0x24,	/* AP		*/
};

/* bq2754x registers */
static u8 bq2754x_regs[NUM_REGS] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0x28,	/* INT TEMP - NA*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0A,	/* FLAGS	*/
	0x16,	/* TTE		*/
	0xFF,	/* TTF - NA	*/
	0xFF,	/* TTES - NA	*/
	0xFF,	/* TTECP - NA	*/
	0x0C,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2A,	/* CYCT		*/
	0xFF,	/* AE - NA	*/
	0x2C,	/* SOC(RSOC)	*/
	0xFF,	/* DCAP(ILMD) - NA */
	0xFF,	/* AP		*/
};

/* bq27200 registers */
static u8 bq27200_regs[NUM_REGS] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0xFF,	/* INT TEMP - NA	*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0A,	/* FLAGS	*/
	0x16,	/* TTE		*/
	0x18,	/* TTF		*/
	0x1c,	/* TTES		*/
	0x26,	/* TTECP	*/
	0x0C,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2A,	/* CYCT		*/
	0x22,	/* AE		*/
	0x0B,	/* SOC(RSOC)	*/
	0x76,	/* DCAP(ILMD)	*/
	0x24,	/* AP		*/
};

/* bq274xx registers */
static u8 bq274xx_regs[NUM_REGS] = {
	0x00,	/* CONTROL	*/
	0x02,	/* TEMP		*/
	0x1e,	/* INT TEMP	*/
	0x04,	/* VOLT		*/
	0x10,	/* AVG CURR	*/
	0x06,	/* FLAGS	*/
	0xFF,	/* TTE - NA	*/
	0xFF,	/* TTF - NA	*/
	0xFF,	/* TTES - NA	*/
	0xFF,	/* TTECP - NA	*/
	0x08,	/* NAC		*/
	0x0E,	/* FCC		*/
	0xFF,	/* CYCT - NA	*/
	0xFF,	/* AE - NA	*/
	0x1C,	/* SOC		*/
	0xFF,	/* DCAP - NA	*/
	0x18,	/* AP		*/
};

/* bq276xx registers - same as bq274xx except CYCT */
static u8 bq276xx_regs[NUM_REGS] = {
	0x00,	/* CONTROL	*/
	0x02,	/* TEMP		*/
	0x1e,	/* INT TEMP	*/
	0x04,	/* VOLT		*/
	0x10,	/* AVG CURR	*/
	0x06,	/* FLAGS	*/
	0xFF,	/* TTE - NA	*/
	0xFF,	/* TTF - NA	*/
	0xFF,	/* TTES - NA	*/
	0xFF,	/* TTECP - NA	*/
	0x08,	/* NAC		*/
	0x0E,	/* FCC		*/
	0x22,	/* CYCT		*/
	0xFF,	/* AE - NA	*/
	0x1C,	/* SOC		*/
	0x3C,	/* DCAP - NA	*/
	0x18,	/* AP		*/
};

/* for device that does not have a design capacity register */

#define HARDCODED_DCAP 1340
/*
 * SBS Commands for DF access - these are pretty standard
 * So, no need to go in the command array
 */
#define BLOCK_DATA_CLASS		0x3E
#define DATA_BLOCK			0x3F
#define BLOCK_DATA			0x40
#define BLOCK_DATA_CHECKSUM		0x60
#define BLOCK_DATA_CONTROL		0x61

/* bq274xx/bq276xx specific command information */
#define BQ274XX_UNSEAL_KEY		0x80008000
#define BQ274XX_HARD_RESET	0x41
#define BQ274XX_SOFT_RESET		0x42

#define BQ274XX_FLAG_ITPOR				0x20
#define BQ274XX_CTRL_STATUS_INITCOMP	0x80

#define BQ27XXX_FLAG_DSC		BIT(0)
#define BQ27XXX_FLAG_SOCF		BIT(1) /* State-of-Charge threshold final */
#define BQ27XXX_FLAG_SOC1		BIT(2) /* State-of-Charge threshold 1 */
#define BQ27XXX_FLAG_FC			BIT(9)
#define BQ27XXX_FLAG_OTD		BIT(14)
#define BQ27XXX_FLAG_OTC		BIT(15)

/* BQ27000 has different layout for Flags register */
#define BQ27200_FLAG_EDVF		BIT(0) /* Final End-of-Discharge-Voltage flag */
#define BQ27200_FLAG_EDV1		BIT(1) /* First End-of-Discharge-Voltage flag */
#define BQ27200_FLAG_CI			BIT(4) /* Capacity Inaccurate flag */
#define BQ27200_FLAG_FC			BIT(5)
#define BQ27200_FLAG_CHGS		BIT(7) /* Charge state flag */

#define BQ27200_RS			20 /* Resistor sense */
#define BQ27200_POWER_CONSTANT		(256 * 29200 / 1000)

/* Subcommands of Control() */
#define CONTROL_STATUS_SUBCMD		0x0000
#define DEV_TYPE_SUBCMD			0x0001
#define FW_VER_SUBCMD			0x0002
#define DF_VER_SUBCMD			0x001F
#define RESET_SUBCMD			0x0041
#define SET_CFGUPDATE_SUBCMD		0x0013
#define SEAL_SUBCMD			0x0020

/* Location of SEAL enable bit in bq276xx DM */
#define BQ276XX_OP_CFG_B_SUBCLASS	64
#define BQ276XX_OP_CFG_B_OFFSET		2
#define BQ276XX_OP_CFG_B_DEF_SEAL_BIT	(1 << 5)

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(struct bq27x00_device_info *di, u8 reg, int *val, bool single);
	int (*write)(struct bq27x00_device_info *di, u8 reg, int value,
			bool single);
	int (*blk_read)(struct bq27x00_device_info *di, u8 reg, u8 *data,
		u8 sz);
	int (*blk_write)(struct bq27x00_device_info *di, u8 reg, u8 *data,
		u8 sz);
};

enum bq27x00_chip { BQ27200, BQ27500, BQ27520, BQ274XX, BQ276XX, BQ2753X,
	BQ27542, BQ27545};

struct bq27x00_reg_cache {
	int temperature;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int charge_full;
	int cycle_count;
	int capacity;
	int energy;
	int flags;
	int power_avg;
	int health;
};

struct dm_reg {
	u8 subclass;
	u8 offset;
	u8 len;
	u32 data;
};

struct bq27x00_device_info {
	struct device 		*dev;
	int			id;
	struct i2c_client	*client;
	
	enum bq27x00_chip	chip;

	struct bq27x00_reg_cache cache;
	int charge_design_full;

	unsigned long last_update;
	struct delayed_work work;

	struct power_supply	bat;

	struct bq27x00_access_methods bus;

	int irq_gpio;
	
	struct mutex lock;
	struct mutex i2c_rw_lock;
	struct mutex data_lock;
	struct mutex irq_complete_lock;

	int fw_ver;
	int df_ver;
	u8 regs[NUM_REGS];
	struct dm_reg *dm_regs;
	u16 dm_regs_count;
	bool use_internal_dm_regs;

	int current_fg_mode;
	int invalid_count;

	/* status tracking */	
	bool batt_inserted;
	bool batt_fc;
	bool batt_ot;
	bool batt_ut;
	bool batt_soc1;
	bool batt_socf;
	bool batt_dsg;
	bool allow_chg;
	bool cfg_update_mode;
	bool itpor;

	int	seal_state; /* 0 - Full Access, 1 - Unsealed, 2 - Sealed */
	struct wake_lock fg_poll_wake_lock;
	bool resume_completed;
	bool bq_update_waiting;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	bool early_resume_completed;
};

int bq27x00_battery_dump_regs(struct bq27x00_device_info *di);
static void bq27xxx_change_fg_work_mode(struct bq27x00_device_info *di);

/* add supplied to "bms" function */
static char *bq_fg_supplied_to[] = {
	"battery",
	"bcl",
};

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
};

static enum power_supply_property bq27520_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
};

static enum power_supply_property bq2753x_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

static enum power_supply_property bq27542_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

static enum power_supply_property bq27545_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_POWER_AVG,
};


static enum power_supply_property bq274xx_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_SOC_REPORTING_READY,
	POWER_SUPPLY_PROP_BATTERY_INFO,
	POWER_SUPPLY_PROP_BATTERY_INFO_ID,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
};

static enum power_supply_property bq276xx_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

/*
 * Ordering the parameters based on subclass and then offset will help in
 * having fewer flash writes while updating.
 * Customize these values and, if necessary, add more based on system needs.
 */
static struct dm_reg bq274xx_dm_regs[] = {
	{64, 0, 2, 0x6479},	/* ext temp */
	{82, 0, 2, 1000},	/* Qmax */
	{82, 5, 1, 0x81},	/* Load Select */
	{82, 6, 2, 1340},	/* Design Capacity */
	{82, 8, 2, 1161},	/* Design Energy */
	{82, 10, 2, 3250},	/* Terminate Voltage */
	{82, 27, 2, 110},	/* Taper rate */
};

static struct dm_reg bq274xx_external_temp_dm_regs[] = {
	{64, 0, 2, 0x6479},	/* ext temp */
};

static struct dm_reg bq274xx_internal_temp_dm_regs[] = {
	{64, 0, 2, 0x6478},	/* int temp */
};

static struct dm_reg bq274xx_host_temp_dm_regs[] = {
	{64, 0, 2, 0x647A},	/* host temp */
};

static struct dm_reg bq276xx_dm_regs[] = {
	{64, 2, 1, 0x2C},	/* Op Config B */
	{82, 0, 2, 1000},	/* Qmax */
	{82, 2, 1, 0x81},	/* Load Select */
	{82, 3, 2, 1340},	/* Design Capacity */
	{82, 5, 2, 3700},	/* Design Energy */
	{82, 9, 2, 3250},	/* Terminate Voltage */
	{82, 20, 2, 110},	/* Taper rate */
};

//static unsigned int poll_interval = 360;
static unsigned int poll_interval = 15;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
				"0 disables polling");

/*
 * Forward Declarations
 */
static int read_dm_block(struct bq27x00_device_info *di, u8 subclass,
	u8 offset, u8 *data);


/*
 * Common code for BQ27x00 devices
 */
static int bq27xxx_i2c_read_byte(struct bq27x00_device_info *di, u8 reg, u8 *val)
{
	int ret = -1;
	int value = 0;

	mutex_lock(&di->i2c_rw_lock);
	ret = di->bus.read(di, reg, &value, true);
	mutex_unlock(&di->i2c_rw_lock);
	*val = value & 0xff;
	return ret;
}
static int bq27xxx_i2c_write_byte(struct bq27x00_device_info *di, u8 reg, u8 val)
{
	int ret;
	mutex_lock(&di->i2c_rw_lock);
	ret =  di->bus.write(di, reg, val, true);
	mutex_unlock(&di->i2c_rw_lock);
	return ret;
}
static int bq27xxx_i2c_read_word(struct bq27x00_device_info *di, u8 reg, u16 *val)
{
	int ret = -1;
	int value = 0;
	mutex_lock(&di->i2c_rw_lock);
	ret = di->bus.read(di, reg, &value, false);
	mutex_unlock(&di->i2c_rw_lock);
	*val = value & 0xffff;
	return ret;
}
static int bq27xxx_i2c_write_word(struct bq27x00_device_info *di, u8 reg, u16 val)
{
	int ret;
	mutex_lock(&di->i2c_rw_lock);
	ret =  di->bus.write(di, reg, val, false);
	mutex_unlock(&di->i2c_rw_lock);
	return ret;
}
static int bq27xxx_i2c_read_block(struct bq27x00_device_info *di, u8 reg, u8 *data, u8 len)
{
	int ret;
	mutex_lock(&di->i2c_rw_lock);
	ret = di->bus.blk_read(di, reg, data, len);
	mutex_unlock(&di->i2c_rw_lock);
	return ret;
}
static int bq27xxx_i2c_write_block(struct bq27x00_device_info *di, u8 reg, u8 *data, u8 len)
{
	int ret;
	mutex_lock(&di->i2c_rw_lock);
	ret = di->bus.blk_write(di, reg, data, len);
	mutex_unlock(&di->i2c_rw_lock);
	mdelay(5);
	return ret;
}
static inline int bq27xxx_read(struct bq27x00_device_info *di, int reg_index, bool single)
{
	int ret = -1;
	int value = 0;

	/* Reports 0 for invalid/missing registers */
	if (!di || di->regs[reg_index] == INVALID_REG_ADDR)
		return 0;

	ret = di->bus.read(di, di->regs[reg_index], &value, single);

	return value;
}

static inline int bq27xxx_write(struct bq27x00_device_info *di, int reg_index, int value, bool single)
{
	if (!di || di->regs[reg_index] == INVALID_REG_ADDR)
		return -1;

	return di->bus.write(di, di->regs[reg_index], value, single);
}

static int control_cmd_wr(struct bq27x00_device_info *di, u16 cmd)
{
	dev_dbg(di->dev, "%s: cmd - %04x\n", __func__, cmd);

	return bq27xxx_write(di, BQ27XXX_REG_CTRL, cmd, false);
}

static int control_cmd_read(struct bq27x00_device_info *di, u16 cmd)
{
	dev_dbg(di->dev, "%s: cmd - %04x\n", __func__, cmd);

	bq27xxx_write(di, BQ27XXX_REG_CTRL, cmd, false);

	msleep(5);

	return bq27xxx_read(di, BQ27XXX_REG_CTRL, false);
}
/*
 * It is assumed that the gauge is in unsealed mode when this function
 * is called
 */
static int bq276xx_seal_enabled(struct bq27x00_device_info *di)
{
	u8 buf[32];
	u8 op_cfg_b;

	if (!read_dm_block(di, BQ276XX_OP_CFG_B_SUBCLASS,
		BQ276XX_OP_CFG_B_OFFSET, buf)) {
		return 1; /* Err on the side of caution and try to seal */
	}

	op_cfg_b = buf[BQ276XX_OP_CFG_B_OFFSET & 0x1F];

	if (op_cfg_b & BQ276XX_OP_CFG_B_DEF_SEAL_BIT)
		return 1;

	return 0;
}

#define SEAL_UNSEAL_POLLING_RETRY_LIMIT	1000

static inline int sealed(struct bq27x00_device_info *di)
{
	u16 val = 0;
	val = control_cmd_read(di, CONTROL_STATUS_SUBCMD);
	dev_dbg(di->dev, "%s val = 0x%04x\n", __func__, val);
	return val & (1 << 13);
}

static int unseal(struct bq27x00_device_info *di, u32 key)
{
	int i = 0;

	dev_notice(di->dev, "%s: key - %08x\n", __func__, key);

	if (!sealed(di))
		goto out;

	di->bus.write(di, BQ27XXX_REG_CTRL, key & 0xFFFF, false);
	msleep(5);
	di->bus.write(di, BQ27XXX_REG_CTRL, (key & 0xFFFF0000) >> 16, false);
	msleep(5);

	while (i < SEAL_UNSEAL_POLLING_RETRY_LIMIT) {
		i++;
		if (!sealed(di))
			break;
		msleep(10);
	}

out:
	if (i == SEAL_UNSEAL_POLLING_RETRY_LIMIT) {
		dev_err(di->dev, "%s: failed\n", __func__);
		return 0;
	} else {
		return 1;
	}
}

static int seal(struct bq27x00_device_info *di)
{
	int i = 0;
	int is_sealed;

	dev_notice(di->dev, "%s:\n", __func__);

	is_sealed = sealed(di);
	if (is_sealed)
		return is_sealed;

	if (di->chip == BQ276XX && !bq276xx_seal_enabled(di)) {
		dev_dbg(di->dev, "%s: sealing is not enabled\n", __func__);
		return is_sealed;
	}

	di->bus.write(di, BQ27XXX_REG_CTRL, SEAL_SUBCMD, false);

	while (i < SEAL_UNSEAL_POLLING_RETRY_LIMIT) {
		i++;
		is_sealed = sealed(di);
		if (is_sealed)
			break;
		msleep(10);
	}

	if (!is_sealed)
		dev_err(di->dev, "%s: failed\n", __func__);

	return is_sealed;
}


	






	






	




	

	//if (bq->skip_reads) {
	//	*val = 0;
	//	return 0;
	//}



	
	//if (bq->skip_writes) 
	//	return 0;

	

	//if (bq->skip_reads) {
	//	*val = 0;
	//	return 0;
	//}
	//TODO:check little endian


	//if (bq->skip_writes) 
	//	return 0;

	//TODO:check little endian
	

	//if (bq->skip_reads)
	//	return 0;
	
	



	//if (bq->skip_writes)
	//	return 0;
	
		//msleep(200);
		//ret = bq->bus.blk_write(bq, reg, data, len);

	





	



















/*static int fg_seal(struct bq27x00_device_info *bq)
{
	int ret;
	int retry = 0;

	fg_get_seal_state(bq);

	if (bq->seal_state == SEAL_STATE_SEALED)
		return 0;
	
	ret = fg_write_word(bq, bq->regs[BQ27XXX_REG_CTRL], 0x0020);

	if (ret < 0) {
		pr_err("Failed to send seal command\n");
		return ret;
	}

	while (retry++ < 1000) {
		fg_get_seal_state(bq);
		if (bq->seal_state == SEAL_STATE_SEALED)
			return 0;
		msleep(10);
	}

	return -1;
}*/



	




/*static int fg_dm_post_access(struct bq27x00_device_info *bq)
{
	int ret;
	int i;


	ret = fg_write_word(bq, bq->regs[BQ27XXX_REG_CTRL],
						FG_SUBCMD_SOFT_RESET);
	if (ret < 0)
		return ret;
	
	msleep(100);

	while(i++ < CFG_UPDATE_POLLING_RETRY_LIMIT) {
		ret = fg_read_status(bq);
		if (!ret && !bq->cfg_update_mode)
			break;
		msleep(100);
	}
	
	if (i == CFG_UPDATE_POLLING_RETRY_LIMIT) {
		pr_err("Failed to exit cfgupdate mode\n");
		return 1;
	} else {
		return fg_seal(bq);
	}
}*/
#define	FG_FLAGS_OT					BIT(15)
#define	FG_FLAGS_UT					BIT(14)
#define	FG_FLAGS_FC					BIT(9)
#define	FG_FLAGS_CHG				BIT(8)
#define	FG_FLAGS_OCVTAKEN			BIT(7)
#define	FG_FLAGS_ITPOR				BIT(5)
#define	FG_FLAGS_CFGUPMODE			BIT(4)
#define	FG_FLAGS_BAT_DET			BIT(3)
#define	FG_FLAGS_SOC1				BIT(2)
#define	FG_FLAGS_SOCF				BIT(1)
#define	FG_FLAGS_DSG				BIT(0)
static int fg_read_status(struct bq27x00_device_info *bq)
{
	int ret;
	u16 flags;

	ret = bq27xxx_i2c_read_word(bq, bq->regs[BQ27XXX_REG_FLAGS], &flags);
	if (ret < 0) {
		pr_err("err: %d\n", ret);
		return ret;
	}
	

	mutex_lock(&bq->data_lock);
	bq->batt_inserted	= !!(flags & FG_FLAGS_BAT_DET);
	bq->batt_ot			= !!(flags & FG_FLAGS_OT);
	bq->batt_ut			= !!(flags & FG_FLAGS_UT);
	bq->batt_fc			= !!(flags & FG_FLAGS_FC);
	bq->batt_soc1		= !!(flags & FG_FLAGS_SOC1);
	bq->batt_socf		= !!(flags & FG_FLAGS_SOCF);
	bq->batt_dsg		= !!(flags & FG_FLAGS_DSG);
	bq->allow_chg		= !!(flags & FG_FLAGS_CHG);
	bq->cfg_update_mode	= !!(flags & FG_FLAGS_CFGUPMODE);
	bq->itpor			= !!(flags & FG_FLAGS_ITPOR);
	mutex_unlock(&bq->data_lock);
	
	dev_notice(bq->dev, "flags:0x%02x, batt_inserted:%d, batt_ot:%d, batt_ut:%d, batt_fc:%d, batt_soc1:%d, batt_socf:%d, batt_dsg:%d, allow_chg:%d, cfg_update_mode:%d, itpor:%d\n", 
		flags, bq->batt_inserted, bq->batt_ot, bq->batt_ut, bq->batt_fc, bq->batt_soc1, bq->batt_socf, bq->batt_dsg, bq->allow_chg, bq->cfg_update_mode, bq->itpor);

	return 0;
}

static bool fg_check_update_necessary(struct bq27x00_device_info *bq)
{
	int ret;
	
	ret = fg_read_status(bq);
	if (!ret && bq->itpor)
		return true;
	else
		return false;
}

static bool fg_update_bqfs_execute_cmd(struct bq27x00_device_info *bq,
										const bqfs_cmd_t *cmd)
{
	int ret;
	u8	tmp_buf[CMD_MAX_DATA_SIZE];

	switch (cmd->cmd_type) {
	case CMD_R:
		ret = bq27xxx_i2c_read_block(bq, cmd->reg, (u8 *)&cmd->data.bytes, cmd->data_len);
		if (ret < 0)
			return false;
		else
			return true;
		break;
	case CMD_W:
		ret = bq27xxx_i2c_write_block(bq, cmd->reg, (u8 *)&cmd->data.bytes, cmd->data_len);
		if (ret < 0)
			return false;
		else
			return true;
		break;
	case CMD_C:
		if (bq27xxx_i2c_read_block(bq, cmd->reg, tmp_buf, cmd->data_len) < 0)
			return false;
		if (memcmp(tmp_buf, cmd->data.bytes, cmd->data_len)) {
			pr_err("CMD_C failed at line %d\n", cmd->line_num);
			return false;
		}

		return true;
		break;
	case CMD_X:
		mdelay(cmd->data.delay * 2);
		return true;
		break;
	default:
		pr_err("Unsupported command at line %d\n", cmd->line_num);
		return false;
	}

}


#define CFG_UPDATE_POLLING_RETRY_LIMIT 50
static int enter_cfg_update_mode(struct bq27x00_device_info *di)
{
	int i = 0;
	u16 flags;

	dev_notice(di->dev, "%s:\n", __func__);

	if (!unseal(di, BQ274XX_UNSEAL_KEY)) {
		dev_err(di->dev, "%s: failed\n", __func__);
		return 0;
	}

	control_cmd_wr(di, SET_CFGUPDATE_SUBCMD);
	msleep(5);

	while (i < CFG_UPDATE_POLLING_RETRY_LIMIT) {
		i++;
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
		if (flags & (1 << 4))
			break;
		msleep(100);
	}

	if (i == CFG_UPDATE_POLLING_RETRY_LIMIT) {
		dev_err(di->dev, "%s: failed %04x\n", __func__, flags);
		return 0;
	}

	return 1;
}

static int exit_cfg_update_mode(struct bq27x00_device_info *di)
{
	int i = 0;
	u16 flags;

	dev_notice(di->dev, "%s:\n", __func__);

	control_cmd_wr(di, BQ274XX_SOFT_RESET);
	msleep(100);

	while (i < CFG_UPDATE_POLLING_RETRY_LIMIT) {
		i++;
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
		if (!(flags & (1 << 4)))
			break;
		msleep(100);
	}

	if (i == CFG_UPDATE_POLLING_RETRY_LIMIT) {
		dev_err(di->dev, "%s: failed %04x\n", __func__, flags);
		return 0;
	}

	if (seal(di))
		return 1;
	else
		return 0;
}
static u8 checksum(u8 *data)
{
	u16 sum = 0;
	int i;

	for (i = 0; i < 32; i++)
		sum += data[i];

	sum &= 0xFF;

	return 0xFF - sum;
}

static void fg_read_datamemory(struct bq27x00_device_info *bq)
{
	u8	tmp_buf[CMD_MAX_DATA_SIZE];
	u8 reg = 0;
	int i = 0, data_len = 0, offset  = 0;

	reg = 0x68;
	data_len = 32;
	offset  = 32;

	read_dm_block(bq, reg, offset, tmp_buf);

	printk("bq27426, %s , reg:0x%02x, offset:%d : ", __func__, reg, offset);
	for(i = 0; i < data_len; i++)
		printk("0x%02x ", tmp_buf[i]);
	printk("\n");

	offset  = 0;

	read_dm_block(bq, reg, offset, tmp_buf);

	printk("bq27426, %s , reg:0x%02x, offset:%d : ", __func__, reg, offset);
	for(i = 0; i < data_len; i++)
		printk("0x%02x ", tmp_buf[i]);
	printk("\n");
}

static void fg_update_bqfs(struct bq27x00_device_info *bq)
{
	int i;

	if (!fg_check_update_necessary(bq))
		return;

	/* TODO:if unseal, enter cfg update mode cmd sequence are in gmfs file,
	   no need to do explicitly */
	/* fg_dm_pre_access(bq); */
	enter_cfg_update_mode(bq);

	bq->bus.write(bq, BLOCK_DATA_CONTROL, 0, true);

	pr_notice("bq27xxx Start updating...\n");
	for (i = 0; i < ARRAY_SIZE(bqfs_image); i++) {
		if (!fg_update_bqfs_execute_cmd(bq, &bqfs_image[i])) {
			pr_err("Failed at command: %d\n", i);
			goto out;
		}
	}
	pr_notice("bq27xxx update dqfs Done!\n");
		
	/* TODO:exit cfg update mode and seal device if these are not handled in gmfs file */
	/* fg_dm_post_access(bq); */
out:
	fg_read_datamemory(bq);
	//fg_dm_post_access(bq);
	exit_cfg_update_mode(bq);
	return;

}

#ifdef DEBUG
static void print_buf(const char *msg, u8 *buf)
{
	int i;

	printk("\nbq: %s buf: ", msg);
	for (i = 0; i < 32; i++)
		printk("%02x ", buf[i]);

	printk("\n");
}
#else
#define print_buf(a, b)
#endif

static int update_dm_block(struct bq27x00_device_info *di, u8 subclass,
	u8 offset, u8 *data)
{
	u8 buf[32];
	u8 cksum;
	u8 blk_offset = offset >> 5;

	dev_dbg(di->dev, "%s: subclass %d offset %d\n",
		__func__, subclass, offset);

	di->bus.write(di, BLOCK_DATA_CONTROL, 0, true);
	msleep(5);

	di->bus.write(di, BLOCK_DATA_CLASS, subclass, true);
	msleep(5);

	di->bus.write(di, DATA_BLOCK, blk_offset, true);
	msleep(5);

	di->bus.blk_write(di, BLOCK_DATA, data, 32);
	msleep(5);
	print_buf(__func__, data);

	cksum = checksum(data);
	di->bus.write(di, BLOCK_DATA_CHECKSUM, cksum, true);
	msleep(5);

	/* Read back and compare to make sure write is successful */
	di->bus.write(di, DATA_BLOCK, blk_offset, true);
	msleep(5);
	di->bus.blk_read(di, BLOCK_DATA, buf, 32);
	if (memcmp(data, buf, 32)) {
		dev_err(di->dev, "%s: error updating subclass %d offset %d\n",
			__func__, subclass, offset);
		return 0;
	} else {
		return 1;
	}
}

static int read_dm_block(struct bq27x00_device_info *di, u8 subclass,
	u8 offset, u8 *data)
{
	u8 cksum_calc, cksum;
	u8 blk_offset = offset >> 5;

	dev_notice(di->dev, "%s: subclass %d offset %d\n",
		__func__, subclass, offset);

	di->bus.write(di, BLOCK_DATA_CONTROL, 0, true);
	msleep(5);

	di->bus.write(di, BLOCK_DATA_CLASS, subclass, true);
	msleep(5);

	di->bus.write(di, DATA_BLOCK, blk_offset, true);
	msleep(5);

	di->bus.blk_read(di, BLOCK_DATA, data, 32);

	cksum_calc = checksum(data);
	bq27xxx_i2c_read_byte(di, BLOCK_DATA_CHECKSUM, &cksum);
	if (cksum != cksum_calc) {
		dev_err(di->dev, "%s: error reading subclass %d offset %d\n",
			__func__, subclass, offset);
		return 0;
	}

	print_buf(__func__, data);

	return 1;
}

/*
 * Return the battery State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_soc(struct bq27x00_device_info *di)
{
	int soc;

	soc = bq27xxx_read(di, BQ27XXX_REG_SOC, false);

	if (soc < 0)
		dev_dbg(di->dev, "error reading relative State-of-Charge\n");

	return soc;
}

/*
 * Return a battery charge value in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_charge(struct bq27x00_device_info *di, u8 reg)
{
	int charge;

	charge = bq27xxx_read(di, reg, false);
	if (charge < 0) {
		dev_dbg(di->dev, "error reading charge register %02x: %d\n",
			reg, charge);
		return charge;
	}

	if (di->chip == BQ27200)
		charge = charge * 3570 / BQ27200_RS;
	else
		charge *= 1000;

	return charge;
}

/*
 * Return the battery Nominal available capaciy in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_nac(struct bq27x00_device_info *di)
{
	int flags;

	if (di->chip == BQ27200) {
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, true);
		if (flags >= 0 && (flags & BQ27200_FLAG_CI))
			return -ENODATA;
	}

	return bq27x00_battery_read_charge(di, BQ27XXX_REG_NAC);
}

/*
 * Return the battery Last measured discharge in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_fcc(struct bq27x00_device_info *di)
{
	return bq27x00_battery_read_charge(di, BQ27XXX_REG_FCC);
}

/*
 * Return the Design Capacity in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_dcap(struct bq27x00_device_info *di)
{
	int dcap;

	dcap = bq27xxx_read(di, BQ27XXX_REG_DCAP, false);

	if (dcap < 0) {
		dev_dbg(di->dev, "error reading initial last measured discharge, returning hard-coded value\n");
		dcap = HARDCODED_DCAP;
	}

	if (di->chip == BQ27200)
		dcap = dcap * 256 * 3570 / BQ27200_RS;
	else
		dcap *= 1000;

	return dcap;
}

/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_energy(struct bq27x00_device_info *di)
{
	int ae;

	ae = bq27xxx_read(di, BQ27XXX_REG_AE, false);
	if (ae < 0) {
		dev_dbg(di->dev, "error reading available energy\n");
		return ae;
	}

	if (di->chip == BQ27200)
		ae = ae * 29200 / BQ27200_RS;
	else
		ae *= 1000;

	return ae;
}

/*
 * Return the battery temperature in tenths of degree Kelvin
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_temperature(struct bq27x00_device_info *di)
{
	int temp;

	temp = bq27xxx_read(di, BQ27XXX_REG_TEMP, false);
	if (temp < 0) {
		dev_err(di->dev, "error reading temperature\n");
		return temp;
	}

	if (di->chip == BQ27200)
		temp = 5 * temp / 2;

	return temp;
}

/*
 * Return the battery temperature in tenths of degree Kelvin
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_internal_temperature(struct bq27x00_device_info *di)
{
	int temp;

	temp = bq27xxx_read(di, BQ27XXX_REG_INT_TEMP, false);
	if (temp < 0) {
		dev_err(di->dev, "error reading temperature\n");
		return temp;
	}

	if (di->chip == BQ27200)
		temp = 5 * temp / 2;

	return temp;
}

/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_cyct(struct bq27x00_device_info *di)
{
	int cyct;

	cyct = bq27xxx_read(di, BQ27XXX_REG_CYCT, false);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_time(struct bq27x00_device_info *di, u8 reg)
{
	int tval;

	tval = bq27xxx_read(di, reg, false);
	if (tval < 0) {
		dev_dbg(di->dev, "error reading time register %02x: %d\n",
			reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	return tval * 60;
}

/*
 * Read a power avg register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_pwr_avg(struct bq27x00_device_info *di, u8 reg)
{
	int tval;

	tval = bq27xxx_read(di, reg, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading power avg rgister  %02x: %d\n",
			reg, tval);
		return tval;
	}

	if (di->chip == BQ27200)
		return (tval * BQ27200_POWER_CONSTANT) / BQ27200_RS;
	else
		return tval;
}

static int overtemperature(struct bq27x00_device_info *di, u16 flags)
{
	if (di->chip == BQ27520)
		return flags & (BQ27XXX_FLAG_OTC | BQ27XXX_FLAG_OTD);
	else
		return flags & BQ27XXX_FLAG_OTC;
}

/*
 * Read flag register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_health(struct bq27x00_device_info *di)
{
	u16 tval;

	tval = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading flag register:%d\n", tval);
		return tval;
	}

	if ((di->chip == BQ27200)) {
		if (tval & BQ27200_FLAG_EDV1)
			tval = POWER_SUPPLY_HEALTH_DEAD;
		else
			tval = POWER_SUPPLY_HEALTH_GOOD;
		return tval;
	} else {
		if (tval & BQ27XXX_FLAG_SOCF)
			tval = POWER_SUPPLY_HEALTH_DEAD;
		else if (overtemperature(di, tval))
			tval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			tval = POWER_SUPPLY_HEALTH_GOOD;
		return tval;
	}

	return -1;
}

static bool bq27x00_temp_is_normal(int temp)
{
	if((1731  < temp) && (temp < 4731)) {
		return true;
	}
	return false;
}

//static int bq27x00_battery_current(struct bq27x00_device_info *di, union power_supply_propval *val);
static void bq27x00_update(struct bq27x00_device_info *di)
{
	struct bq27x00_reg_cache cache = {0, };
	bool is_bq27200 = di->chip == BQ27200;
	bool is_bq27500 = di->chip == BQ27500;
	bool is_bq274xx = di->chip == BQ274XX;
	bool is_bq276xx = di->chip == BQ276XX;
	bool is_fg_status_changed = false;

	int int_temp = 0, ext_temp = 0;
	bool temp_abnormal = false;
	//union power_supply_propval val;
	//static int is_first = 1;

	//if(is_first == 0) {
	//	bq27x00_battery_current(di, &val);
	//}
	//is_first = 0;
	//printk("%s current is :%d\n", __func__, val.intval);

	if(3 == di->current_fg_mode) {
		bq27xxx_write(di, BQ27XXX_REG_TEMP, 3031, false);
	}
	
	cache.flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, !is_bq27500);
	if (cache.flags >= 0) {
		if (is_bq27200 && (cache.flags & BQ27200_FLAG_CI)) {
			dev_info(di->dev, "battery is not calibrated! ignoring capacity values\n");
			cache.capacity = -ENODATA;
			cache.energy = -ENODATA;
			cache.time_to_empty = -ENODATA;
			cache.time_to_empty_avg = -ENODATA;
			cache.time_to_full = -ENODATA;
			cache.charge_full = -ENODATA;
			cache.health = -ENODATA;
		} else {
			cache.capacity = bq27x00_battery_read_soc(di);
			if (!(is_bq274xx || is_bq276xx)) {
				cache.energy = bq27x00_battery_read_energy(di);
				cache.time_to_empty =
					bq27x00_battery_read_time(di,
							BQ27XXX_REG_TTE);
				cache.time_to_empty_avg =
					bq27x00_battery_read_time(di,
							BQ27XXX_REG_TTECP);
				cache.time_to_full =
					bq27x00_battery_read_time(di,
							BQ27XXX_REG_TTF);
			}
			cache.charge_full = bq27x00_battery_read_fcc(di);
			cache.health = bq27x00_battery_read_health(di);
		}
		cache.temperature = bq27x00_battery_read_temperature(di);
		if (!is_bq274xx)
			cache.cycle_count = bq27x00_battery_read_cyct(di);
		cache.power_avg =
			bq27x00_battery_read_pwr_avg(di, BQ27XXX_POWER_AVG);
		
		ext_temp = cache.temperature;
		if(false == bq27x00_temp_is_normal(ext_temp)) {    //temperature is invalid
			pr_notice("%s temp is out of range.\n", __func__);
			temp_abnormal = true;
		} else if(ext_temp > 3431){    //if temperature great than 70degree, compare internel external temp.
			if(1 == di->current_fg_mode) {
				int_temp = bq27x00_battery_read_internal_temperature(di);
				if(abs(int_temp - ext_temp) > 200) {
					pr_notice("%s temp is out of range differ from internal && external.\n", __func__);
					temp_abnormal = true;
				}
			} else if(2 == di->current_fg_mode) {
				;
			} else {
				;
			}
		}
		if(true == temp_abnormal) {
			di->invalid_count++;
			pr_err("%s temp 0x%x out of range, use previous value 0x%x.\n", __func__, cache.temperature, di->cache.temperature);
			cache.temperature = di->cache.temperature;
			if((cache.capacity <= 0) || (cache.capacity > 100)) {
				cache.capacity = di->cache.capacity;
			}
			if(di->invalid_count > 2) {         //continuous temperature is invlid triple, change mode.
				if(1 == di->current_fg_mode) {
					di->current_fg_mode = 2;
					bq27xxx_change_fg_work_mode(di);
				} else if(2 == di->current_fg_mode) {
					di->current_fg_mode = 3;
					bq27xxx_change_fg_work_mode(di);
					cache.temperature = 3031;
					//bq27xxx_write(di, BQ27XXX_REG_TEMP, 3031, false);
				}
				di->invalid_count = 0;
			}
		} else {
			di->invalid_count = 0;
		}

		if(2 == di->current_fg_mode) {
			cache.temperature = cache.temperature - 50;
		}

		/* We only have to read charge design full once */
		if (di->charge_design_full <= 0) {
			di->charge_design_full = bq27x00_battery_read_dcap(di);
			if (di->charge_design_full <= 0) {
				di->charge_design_full = 2500;
			}
		}
	}

	if(di->cache.capacity != cache.capacity || di->cache.charge_full != cache.charge_full 
		|| di->cache.health != cache.health || di->cache.temperature != cache.temperature 
		|| di->cache.flags != cache.flags) 
		is_fg_status_changed = true;
	if (memcmp(&di->cache, &cache, sizeof(cache)) != 0) {
		di->cache = cache;
		if(true == is_fg_status_changed)
		power_supply_changed(&di->bat);
	}

	di->last_update = jiffies;
}

static void copy_to_dm_buf_big_endian(struct bq27x00_device_info *di,
	u8 *buf, u8 offset, u8 sz, u32 val)
{
	dev_dbg(di->dev, "%s: offset %d sz %d val %d\n",
		__func__, offset, sz, val);

	switch (sz) {
	case 1:
		buf[offset] = (u8) val;
		break;
	case 2:
		put_unaligned_be16((u16) val, &buf[offset]);
		break;
	case 4:
		put_unaligned_be32(val, &buf[offset]);
		break;
	default:
		dev_err(di->dev, "%s: bad size for dm parameter - %d",
			__func__, sz);
		break;
	}
}

static void bq27xxx_change_fg_work_mode(struct bq27x00_device_info *di)
{
	int i;
	u8 subclass, offset;
	u32 blk_number;
	u32 blk_number_prev = 0;
	u8 buf[32];
	bool buf_valid = false;
	struct dm_reg *dm_regs;
	struct dm_reg *dm_reg;
	int dm_regs_count = 0;

	dev_notice(di->dev, "%s fuel gague mode:%d.\n", __func__, di->current_fg_mode);

	if(1 == di->current_fg_mode) {
		dm_regs = bq274xx_external_temp_dm_regs;
		dm_regs_count = ARRAY_SIZE(bq274xx_external_temp_dm_regs);
	} else if(2 == di->current_fg_mode) {
		dm_regs = bq274xx_internal_temp_dm_regs;
		dm_regs_count = ARRAY_SIZE(bq274xx_internal_temp_dm_regs);
	} else if(3 == di->current_fg_mode) {
		dm_regs = bq274xx_host_temp_dm_regs;
		dm_regs_count = ARRAY_SIZE(bq274xx_host_temp_dm_regs);
	} else {
		return;
	}

	enter_cfg_update_mode(di);
	for (i = 0; i < dm_regs_count; i++) {
		dm_reg = &dm_regs[i];
		subclass = dm_reg->subclass;
		offset = dm_reg->offset;

		/*
		 * Create a composite block number to see if the subsequent
		 * register also belongs to the same 32 btye block in the DM
		 */
		blk_number = subclass << 8;
		blk_number |= offset >> 5;

		if (blk_number == blk_number_prev) {
			copy_to_dm_buf_big_endian(di, buf, offset,
				dm_reg->len, dm_reg->data);
		} else {

			if (buf_valid)
				update_dm_block(di, blk_number_prev >> 8,
					(blk_number_prev << 5) & 0xFF , buf);
			else
				buf_valid = true;

			read_dm_block(di, dm_reg->subclass, dm_reg->offset,
				buf);
			copy_to_dm_buf_big_endian(di, buf, offset,
				dm_reg->len, dm_reg->data);
		}
		blk_number_prev = blk_number;
	}

	/* Last buffer to be written */
	if (buf_valid)
		update_dm_block(di, subclass, offset, buf);

	exit_cfg_update_mode(di);

	if(3 == di->current_fg_mode) {
		msleep(20);
		bq27xxx_write(di, BQ27XXX_REG_TEMP, 3031, false);
	}
}

static int rom_mode_gauge_init_completed(struct bq27x00_device_info *di)
{
	u16 status = 0;


	status = control_cmd_read(di, CONTROL_STATUS_SUBCMD);
	dev_dbg(di->dev, "%s: status - 0x%04x\n", __func__, status);

	return status & BQ274XX_CTRL_STATUS_INITCOMP;
		
	//dev_dbg(di->dev, "%s:\n", __func__);
//
//	return control_cmd_read(di, CONTROL_STATUS_SUBCMD) &
//		BQ274XX_CTRL_STATUS_INITCOMP;
}

static bool rom_mode_gauge_dm_initialized(struct bq27x00_device_info *di)
{
	int ret = -1;
	u16 flags = 0;
	static int boot = 0;

	ret = bq27xxx_i2c_read_word(di, di->regs[BQ27XXX_REG_FLAGS], &flags);
	if(ret < 0) {
		dev_err(di->dev, "%s read flag failed.\n", __func__);
		return true;    //i2c failed donot update dm data.
	}

	if (flags & BQ274XX_FLAG_ITPOR) {
		dev_notice(di->dev, "%s config registers have been reset to default values, flags - 0x%04x\n", __func__, flags);
		return false;
	} else {
		if(0 == boot)
			dev_notice(di->dev, "%s: fg dm_initialized, flags - 0x%04x\n", __func__, flags);
		boot = 1;
		return true;
	}
}

#define INITCOMP_TIMEOUT_MS		10000
static void rom_mode_gauge_dm_init(struct bq27x00_device_info *di)
{
	int i;
	int timeout = INITCOMP_TIMEOUT_MS;
	u8 subclass, offset;
	u32 blk_number;
	u32 blk_number_prev = 0;
	u8 buf[32];
	bool buf_valid = false;
	struct dm_reg *dm_reg;

	dev_notice(di->dev, "%s:\n", __func__);

	while (!rom_mode_gauge_init_completed(di) && timeout > 0) {
		msleep(100);
		timeout -= 100;
	}

	if (timeout <= 0) {
		dev_err(di->dev, "%s: INITCOMP not set after %d seconds\n",
			__func__, INITCOMP_TIMEOUT_MS/100);
		return;
	}

	if (!di->dm_regs || !di->dm_regs_count) {
		dev_err(di->dev, "%s: Data not available for DM initialization\n",
			__func__);
		return;
	}
	if(!di->use_internal_dm_regs) {
		fg_update_bqfs(di);
		return;
	}

	enter_cfg_update_mode(di);
	for (i = 0; i < di->dm_regs_count; i++) {
		dm_reg = &di->dm_regs[i];
		subclass = dm_reg->subclass;
		offset = dm_reg->offset;

		/*
		 * Create a composite block number to see if the subsequent
		 * register also belongs to the same 32 btye block in the DM
		 */
		blk_number = subclass << 8;
		blk_number |= offset >> 5;

		if (blk_number == blk_number_prev) {
			copy_to_dm_buf_big_endian(di, buf, offset,
				dm_reg->len, dm_reg->data);
		} else {

			if (buf_valid)
				update_dm_block(di, blk_number_prev >> 8,
					(blk_number_prev << 5) & 0xFF , buf);
			else
				buf_valid = true;

			read_dm_block(di, dm_reg->subclass, dm_reg->offset,
				buf);
			copy_to_dm_buf_big_endian(di, buf, offset,
				dm_reg->len, dm_reg->data);
		}
		blk_number_prev = blk_number;
	}

	/* Last buffer to be written */
	if (buf_valid)
		update_dm_block(di, subclass, offset, buf);

	exit_cfg_update_mode(di);
}

static void bq27x00_battery_poll(struct work_struct *work)
{
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, work.work);
	
	mutex_lock(&di->irq_complete_lock);
	di->bq_update_waiting = false;
	wake_lock_timeout(&di->fg_poll_wake_lock, 1 * HZ);
	if(false == di->resume_completed) {
		di->bq_update_waiting = true;
		mutex_unlock(&di->irq_complete_lock);
		goto out;
	}
	
	if (((di->chip == BQ274XX) || (di->chip == BQ276XX)) &&
		!rom_mode_gauge_dm_initialized(di)) {
		rom_mode_gauge_dm_init(di);
	}

	bq27x00_update(di);
	bq27x00_battery_dump_regs(di);
	
	mutex_unlock(&di->irq_complete_lock);
out:
	if (poll_interval > 0) {
		/* The timer does not have to be accurate. */
		//set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
		schedule_delayed_work(&di->work, poll_interval * HZ);
	}
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int curr;
	int flags;

	curr = bq27xxx_read(di, BQ27XXX_REG_AI, false);
	if (curr < 0) {
		dev_err(di->dev, "error reading current\n");
		return curr;
	}

	if (di->chip == BQ27200) {
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
		if (flags & BQ27200_FLAG_CHGS) {
			dev_dbg(di->dev, "negative current!\n");
			curr = -curr;
		}

		val->intval = curr * 3570 / BQ27200_RS;
	} else {
		/* Other gauges return signed value */
		val->intval = (int)((s16)curr) * 1000;
	}
	val->intval = 0 - val->intval;

	return 0;
}

static int bq27x00_battery_status(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int status;

	if (di->chip == BQ27200) {
		if (di->cache.flags & BQ27200_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27200_FLAG_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else if (power_supply_am_i_supplied(&di->bat))
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		if (di->cache.flags & BQ27XXX_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27XXX_FLAG_DSC)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	}

	val->intval = status;

	return 0;
}

static int bq27x00_battery_capacity_level(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int level;

	if (di->chip == BQ27200) {
		if (di->cache.flags & BQ27200_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27200_FLAG_EDV1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27200_FLAG_EDVF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	} else {
		if (di->cache.flags & BQ27XXX_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27XXX_FLAG_SOC1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27XXX_FLAG_SOCF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}

	val->intval = level;

	return 0;
}

/*
 * Return the battery Voltage in millivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int volt;

	volt = bq27xxx_read(di, BQ27XXX_REG_VOLT, false);
	if (volt < 0) {
		dev_err(di->dev, "error reading voltage\n");
		return volt;
	}

	val->intval = volt * 1000;

	return 0;
}

static int bq27x00_simple_value(int value,
	union power_supply_propval *val)
{

	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}
#ifdef CONFIG_PROC_FS
#define TEMP_BUFFER_LENGTH 4096
static struct proc_dir_entry *fg_std_proc_entry;
static struct proc_dir_entry *fg_dm_proc_entry;
static struct proc_dir_entry *fg_work_mode_proc_entry;
static char g_temp_str_buffer[TEMP_BUFFER_LENGTH];
static int g_length = 0;
static int g_std_regs[] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x18, 0x1C, 0x1E, 0x20, 0x28, 0x2A, 0x2C, 0x2E, 0x30};
static int g_ctrl_reg[] = {0x0, 0x1, 0x2, 0x4, 0x7, 0x8, 0xc, 0xd, 0x13, 0x19, 0x1b, 0x1c, 0x20, 0x23, 0x30, 0x31, 0x32, 0x41, 0x42};
static int g_dm_reg[23][2] = {
	{0x2, 0x0},
	{0x24, 0x0},
	{0x31, 0x0},
	{0x40, 0x0},
	{0x44, 0x0},
	{0x50, 0x0},
	{0x50, 0x1},
	{0x50, 0x2},
	{0x51, 0x0},
	{0x52, 0x0},
	{0x53, 0x0},
	{0x53, 0x1},
	{0x54, 0x0},
	{0x54, 0x1},
	{0x55, 0x0},
	{0x6c, 0x0},
	{0x59, 0x0},
	{0x6d, 0x0},
	{0x68, 0x0},
	{0x68, 0x1},
	{0x69, 0x0},
	{0x6b, 0x0},
	{0x70, 0x0},
};

static void dump_regs(struct bq27x00_device_info *di)
{
	int i = 0, j = 0;
	int val = 0;
	u16 val_16 = 0;
	for(i = 0; i < 18; i ++) {
		if(0 == i) {
			g_length += snprintf(g_temp_str_buffer + g_length, TEMP_BUFFER_LENGTH - g_length, "[control reg]\n");
			for(j = 0; j < 6; j++) {
				val = control_cmd_read(di, g_ctrl_reg[j]);
				g_length += snprintf(g_temp_str_buffer + g_length, TEMP_BUFFER_LENGTH - g_length, "0x%02x = 0x%02x\n",  g_ctrl_reg[j], val);
				printk(g_temp_str_buffer + g_length, TEMP_BUFFER_LENGTH - g_length, "0x%02x = 0x%02x\n", g_ctrl_reg[j], val);
			}
			g_length += snprintf(g_temp_str_buffer + g_length, TEMP_BUFFER_LENGTH - g_length, "[standard commands]\n");
		} else {
			val = bq27xxx_i2c_read_word(di, g_std_regs[i], &val_16);
			g_length += snprintf(g_temp_str_buffer + g_length, TEMP_BUFFER_LENGTH - g_length, "0x%02x = 0x%02x\n", g_std_regs[i], val_16);
		}
	}

	//g_length += snprintf(g_temp_str_buffer + g_length, TEMP_BUFFER_LENGTH - g_length, 
	//	"capacity %d charge_full %d cycle_count %d energy %d flags 0x%x health %d power_avg %d\ntemp %d time_to_empty %d time_to_empty_avg %d time_to_full %d\n", 
	//	di->cache.capacity, di->cache.charge_full, di->cache.cycle_count, di->cache.energy, di->cache.flags, di->cache.health, di->cache.power_avg,
	//	di->cache.temperature, di->cache.time_to_empty, di->cache.time_to_empty_avg, di->cache.time_to_full);
	g_length += snprintf(g_temp_str_buffer + g_length, TEMP_BUFFER_LENGTH - g_length, "BQ27426 20170410\n");
	return;
}
static void dump_dm_regs(struct bq27x00_device_info *di)
{
	int i = 0, j = 0;
	unseal(di, BQ274XX_UNSEAL_KEY);
	for(i = 0; i < 23; i++) {
		u8	tmp_buf[CMD_MAX_DATA_SIZE];
		u8 reg = 0;
		int data_len = 0, offset  = 0;
		reg = g_dm_reg[i][0];
		data_len = 32;
		offset  =  g_dm_reg[i][1] * 32;
		read_dm_block(di, reg, offset, tmp_buf);
		g_length += snprintf(g_temp_str_buffer + g_length, TEMP_BUFFER_LENGTH - g_length, "REG:%02x(%d) offset: %02x\n", reg, reg, offset);
		printk("reg:0x%02x, offset:%d : ", reg, offset);
		for(j = 0; j < data_len; j++) {
			g_length += snprintf(g_temp_str_buffer + g_length, TEMP_BUFFER_LENGTH - g_length, "%02x ", tmp_buf[j]);
			printk("%02x ", tmp_buf[j]);
		}
		g_length += snprintf(g_temp_str_buffer + g_length, TEMP_BUFFER_LENGTH - g_length, "\n");
		printk("\n");
	}
	seal(di);
	return;
}
static int bq_proc_show(struct seq_file *seq, void *v)
{
	struct bq27x00_device_info *di = seq->private;
	int ret = -1;
	int length = 0;
	pr_notice("bq27x00 %s", __func__);
	g_length = 0;
	dump_regs(di);
	if((g_length + 1) >= TEMP_BUFFER_LENGTH) {
		g_temp_str_buffer[g_length-2] = '\n';
		g_temp_str_buffer[g_length-1] = '\0';
		length = g_length;
	} else {
		g_temp_str_buffer[g_length] = '\0';
		length = g_length + 1;
	}
	ret = seq_write(seq, g_temp_str_buffer, length);
	if(ret < 0) {
		pr_notice("bq27x00 %s seq write error.\n", __func__);
	}
	return 0;
}
static int bq_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, bq_proc_show, PDE_DATA(inode));
}
static int bq27xxx_hard_reset(struct bq27x00_device_info *di)
{
	pr_notice("bq27x00 %s", __func__);
	cancel_delayed_work_sync(&di->work);
	unseal(di, BQ274XX_UNSEAL_KEY);
	control_cmd_wr(di, BQ274XX_HARD_RESET);
	seal(di);
	return 0;
}
static int bq27xxx_soft_reset(struct bq27x00_device_info *di)
{
	pr_notice("bq27x00 %s", __func__);
	cancel_delayed_work_sync(&di->work);
	unseal(di, BQ274XX_UNSEAL_KEY);
	control_cmd_wr(di, BQ274XX_SOFT_RESET);
	seal(di);
	return 0;
}
static ssize_t bq_proc_write(struct file *filp,  const char *buff, size_t len, loff_t * off)
{
	struct bq27x00_device_info *di = PDE_DATA(file_inode(filp));
	char message[256];
	int reg, val;
	int copy_len;
	int fgmode = 0;
	if(len > 256)    copy_len = 255;
	if (copy_from_user(message, buff, copy_len))
		return -EFAULT;
	message[copy_len] = '\0';
	pr_notice("bq27x00 %s\n", message);
	if (strstr(message, "write byte")) {
		sscanf(&message[11], "%x %x", &reg, &val);
		bq27xxx_i2c_write_byte(di, reg, val);
		return len;
	} else if (strstr(message, "write word")) {
		sscanf(&message[11], "%x %x", &reg, &val);
		bq27xxx_i2c_write_word(di, reg, val);
		return len;
	} else if(strstr(message, "hard reset")) {
		bq27xxx_hard_reset(di);
	} else if(strstr(message, "soft reset")) {
		bq27xxx_soft_reset(di);
	} else if(strstr(message, "int")) {
		di->use_internal_dm_regs = true;
	} else if(strstr(message, "ext")) {
		di->use_internal_dm_regs = false;
	} else if (strstr(message, "mode")) {
		sscanf(&message[5], "%d", &fgmode);
		di->current_fg_mode = fgmode;
		bq27xxx_change_fg_work_mode(di);
		return len;
	}
	
	return len;
}
static int bq_dm_proc_show(struct seq_file *seq, void *v)
{
	struct bq27x00_device_info *di = seq->private;
	int ret = -1;
	int length = 0;
	pr_notice("bq27x00 %s", __func__);
	g_length = 0;
	dump_dm_regs(di);
	if((g_length + 1) >= TEMP_BUFFER_LENGTH) {
		g_temp_str_buffer[g_length-2] = '\n';
		g_temp_str_buffer[g_length-1] = '\0';
		length = g_length;
	} else {
		g_temp_str_buffer[g_length] = '\0';
		length = g_length + 1;
	}
	ret = seq_write(seq, g_temp_str_buffer, length);
	if(ret < 0) {
		pr_notice("bq27x00 %s seq write error.\n", __func__);
	}
	return 0;
}
static int bq_dm_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, bq_dm_proc_show, PDE_DATA(inode));
}
static int fg_change_chem_id(struct bq27x00_device_info *di, u16 new_id)
{
	int ret;
	u16 old_id;
	int i;
	ret = bq27xxx_i2c_write_word(di, di->regs[BQ27XXX_REG_CTRL], FG_SUBCMD_CHEM_ID);
	if (ret < 0) {
		pr_err("Failed to write chemid subcmd, ret = %d\n", ret);
		return ret;
	}
	msleep(5);
	ret = bq27xxx_i2c_read_word(di, di->regs[BQ27XXX_REG_CTRL], &old_id);
	if (ret < 0) {
		pr_err("Failed to read control status, ret = %d\n", ret);
		return ret;
	}
	if (new_id == old_id) {
		pr_info("new chemid is same as old one, skip change\n");
		return 0;
	}
	for (i = 0; i < ARRAY_SIZE(batt_chem_id_arr); i++) {
		if (new_id == batt_chem_id_arr[i].id) 
			break;
	}
	if (i == ARRAY_SIZE(batt_chem_id_arr)) {
		pr_err("not supported chem_id %d\n", new_id);
		return -1;
	}
	msleep(5);
	ret = enter_cfg_update_mode(di);
	if (ret < 0)
		return ret;
	msleep(5);
	ret = bq27xxx_i2c_write_word(di, di->regs[BQ27XXX_REG_CTRL], batt_chem_id_arr[i].cmd);
	if (ret < 0) {
		pr_err("Failed to send chem_id command, ret=%d\n", ret);
		exit_cfg_update_mode(di);
		return ret;
	}
	msleep(5);
	ret = exit_cfg_update_mode(di);
	if (ret < 0)
		return ret;
	msleep(2000);
	/* Read back checm id to confirm  */
	ret = bq27xxx_i2c_read_word(di, di->regs[BQ27XXX_REG_CTRL], &old_id);
	if (ret < 0) {
		pr_err("Failed to read control status, ret = %d\n", ret);
		return ret;
	}
	if (new_id == old_id) {
		pr_info("chem ID changed successfully\n");
		return 0;
	} else {
		return -1;
	}
}
static ssize_t bq_dm_proc_write(struct file *filp,  const char *buff, size_t len, loff_t * off)
{
	struct bq27x00_device_info *di = PDE_DATA(file_inode(filp));
	char message[256];
	int val = 0;
	int copy_len = 0;
	if(len > 256)    copy_len = 255;
	if (copy_from_user(message, buff, copy_len))
		return -EFAULT;
	message[copy_len] = '\0';
	pr_notice("bq27x00 %s\n", message);
	if (strstr(message, "change chem id")) {
		sscanf(&message[15], "%x", &val);
		fg_change_chem_id(di, val);
		return len;
	} 
	return len;
}

static int bq_work_mode_proc_show(struct seq_file *seq, void *v)
{
	struct bq27x00_device_info *di = seq->private;
	int ret = -1;

	pr_notice("bq27x00 %s", __func__);

	ret = seq_printf(seq, "%d", di->current_fg_mode);
	if(ret < 0) {
		pr_notice("bq27x00 %s seq write error.\n", __func__);
	}
	return 0;
}

static int bq_work_mode_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, bq_work_mode_proc_show, PDE_DATA(inode));
}
static const struct file_operations charge_proc_ops = {
	.owner		= THIS_MODULE,
	.open		= bq_seq_open,
	.read		= seq_read,
	.write		= bq_proc_write,
	.llseek		= seq_lseek,
};
static const struct file_operations bq_dm_proc_ops = {
	.owner = THIS_MODULE,
	.open		= bq_dm_seq_open,
	.read		= seq_read,
	.write		= bq_dm_proc_write,
	.llseek		= seq_lseek,
};

static const struct file_operations bq_work_mode_proc_ops = {
	.owner		= THIS_MODULE,
	.open		= bq_work_mode_seq_open,
	.read		= seq_read,
	.write		= NULL,
	.llseek		= seq_lseek,
	//.release	= seq_release_ops,
};

static void create_charge_proc_entry(struct bq27x00_device_info *data)
{
	fg_std_proc_entry = proc_create_data("driver/bq_regs", 0644, NULL, &charge_proc_ops, data);
	if (fg_std_proc_entry) {
		pr_notice("create proc file sucess!\n");
	} else
		pr_err("create proc file failed!\n");
	
	fg_dm_proc_entry = proc_create_data("driver/bq_dm_regs", 0644, NULL, &bq_dm_proc_ops, data);
	if (fg_dm_proc_entry) {
		pr_notice("create proc file sucess!\n");
	} else
		pr_err("create proc file failed!\n");
	
	fg_work_mode_proc_entry = proc_create_data("driver/fg_work_mode", 0444, NULL, &bq_work_mode_proc_ops, data);
	if (fg_work_mode_proc_entry) {
		pr_notice("create proc file sucess!\n");
	} else
		pr_err("create proc file failed!\n");
}
#endif

static void bq27x00_dm_compare_work(struct bq27x00_device_info *di)
{
//  struct bq27x00_device_info *di = container_of(work, struct bq27x00_device_info, compare_work.work);
	//int i = 0;
	u8 tmp_buf[CMD_MAX_DATA_SIZE];
	u8 reg = 0;
	int data_len = 0, offset  = 0;
	int retval = -1;

	u8 data_array[32] = {0x64, 0x79, 0x0F, 0x96, 0x23, 0x10, 0x00, 0x14, 0x04, 0x00, 0x09, 0x04, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	if (!unseal(di, BQ274XX_UNSEAL_KEY)) {
		dev_err(di->dev, "%s: failed\n", __func__);
		goto out;
	}
	reg = 0x40;
	data_len = 32;
	offset  =  0 * 32;
	retval = read_dm_block(di, reg, offset, tmp_buf);
	if(1 == retval) {
		if(data_array[5] != tmp_buf[5]) {
			pr_notice("%s dm %d != %d is not same, perform hard reset to reload param 00.\n", __func__, data_array[5], tmp_buf[5]);
			control_cmd_wr(di, BQ274XX_HARD_RESET);
		} else {
			pr_notice("%s dm %d == %d same.\n", __func__, data_array[5], tmp_buf[5]);
			if(0x78 == tmp_buf[1]) {
				di->current_fg_mode = 2;
			} else if(0x7A == tmp_buf[1]) {    //use host temperture, retuan mode 1 to recheck.
				di->current_fg_mode = 1;
				bq27xxx_change_fg_work_mode(di);
			}
		}
		//for(i = 0; i < 6; i++) {
		//	if(tmp_buf[i] != data_array[i]) {
		//		pr_notice("%s dm is not same, perform hard reset to reload param.\n", __func__);
		//		control_cmd_wr(di, BQ274XX_HARD_RESET);
		//		break;
		//	}
		//}
	}
	seal(di);

	//if(i >= 5) {
	//	pr_notice("%s dm is same.\n", __func__);
	//}

out:
	return;
}

#define to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	mutex_lock(&di->lock);
	if (time_is_before_jiffies(di->last_update + 5 * HZ)) {
		cancel_delayed_work_sync(&di->work);
		bq27x00_battery_poll(&di->work.work);
	}
	mutex_unlock(&di->lock);

	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27x00_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq27x00_battery_voltage(di, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->cache.flags < 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq27x00_battery_current(di, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = bq27x00_simple_value(di->cache.capacity, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = bq27x00_battery_capacity_level(di, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = bq27x00_simple_value(di->cache.temperature, val);
		if (ret == 0)
			val->intval -=2731;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_empty, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27x00_simple_value(di->cache.time_to_empty_avg, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_full, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27x00_simple_value(bq27x00_battery_read_nac(di), val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27x00_simple_value(di->cache.charge_full, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = bq27x00_simple_value(di->charge_design_full, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = bq27x00_simple_value(di->cache.cycle_count, val);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = bq27x00_simple_value(di->cache.energy, val);
		break;
	case POWER_SUPPLY_PROP_POWER_AVG:
		ret = bq27x00_simple_value(di->cache.power_avg, val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bq27x00_simple_value(di->cache.health, val);
		break;
	case POWER_SUPPLY_PROP_SOC_REPORTING_READY:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_BATTERY_INFO:
		val->intval = 29000;
		break;
	case POWER_SUPPLY_PROP_BATTERY_INFO_ID:
		val->intval = 29000;
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		val->intval = 29000;
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		val->intval = 29000;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void bq27x00_external_power_changed(struct power_supply *psy)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, 0);
}

static void __init set_properties_array(struct bq27x00_device_info *di,
	enum power_supply_property *props, int num_props)
{
	int tot_sz = num_props * sizeof(enum power_supply_property);

	di->bat.properties = devm_kzalloc(di->dev, tot_sz, GFP_KERNEL);

	if (di->bat.properties) {
		memcpy(di->bat.properties, props, tot_sz);
		di->bat.num_properties = num_props;
	} else {
		di->bat.num_properties = 0;
	}
}

//int bq27x00_battery_get_fg_regs(struct power_supply *psy, int *regs)
int bq27x00_battery_dump_regs(struct bq27x00_device_info *di)
{
	static unsigned int nums = 0;
	u16 val = 0;
	int nom_avl_cap = 0, full_avl_cap = 0, rem_cap = 0, full_chg_cap = 0;
	int rem_cap_unfilter = 0, rem_cap_filter = 0, full_chg_cap_unfilter = 0, full_chg_cap_filter = 0, soc = 0, soc_unfilter = 0;
	int temp = 0, int_temp = 0, current_now = 0, voltage = 0;
	int regs_00 = 0, regs_06 = 0, regs_16 = 0, regs_66 = 0, regs_1a = 0, regs_68 = 0, regs_6c = 0, regs_6e = 0, regs_70 = 0;
	//struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	if((nums % 2) == 0) {
		bq27xxx_i2c_read_word(di, 0x02, &val);
		temp = val;
		bq27xxx_i2c_read_word(di, 0x04, &val);
		voltage = val;
		bq27xxx_i2c_read_word(di, 0x10, &val);
		current_now = (s16)val;

		bq27xxx_i2c_read_word(di, 0x08, &val);
		nom_avl_cap = val;
		bq27xxx_i2c_read_word(di, 0x0A, &val);
		full_avl_cap = val;
		bq27xxx_i2c_read_word(di, 0x0C, &val);
		rem_cap = val;
		bq27xxx_i2c_read_word(di, 0x0E, &val);
		full_chg_cap = val;
		
		bq27xxx_i2c_read_word(di, 0x1E, &val);
		int_temp = val;

		bq27xxx_i2c_read_word(di, 0x28, &val);
		rem_cap_unfilter = val;
		bq27xxx_i2c_read_word(di, 0x2A, &val);
		rem_cap_filter = val;
		bq27xxx_i2c_read_word(di, 0x2C, &val);
		full_chg_cap_unfilter = val;
		bq27xxx_i2c_read_word(di, 0x2E, &val);
		full_chg_cap_filter = val;
		bq27xxx_i2c_read_word(di, 0x1C, &val);
		soc = val;
		bq27xxx_i2c_read_word(di, 0x30, &val);
		soc_unfilter = val;
		

		pr_notice("T,%d,INTT,%d,V,%d,I,%d,nomAC,%d,fulAC,%d,remC,%d,FChgC,%d,remCapUF,%d,remCapF,%d,FChgCapUF,%d,FChgCapF,%d,soc,%d,socUF,%d\n", 
			temp, int_temp, voltage, current_now, nom_avl_cap, full_avl_cap, rem_cap, full_chg_cap, 
			rem_cap_unfilter, rem_cap_filter, full_chg_cap_unfilter, full_chg_cap_filter, soc, soc_unfilter);
	}
	if((nums % 10) == 0) {
		regs_00 = val = control_cmd_read(di, 0x00);
		bq27xxx_i2c_read_word(di, 0x06, &val);
		regs_06 = val;
		bq27xxx_i2c_read_word(di, 0x16, &val);
		regs_16 = val;
		bq27xxx_i2c_read_word(di, 0x66, &val);
		regs_66 = val;
		bq27xxx_i2c_read_word(di, 0x1a, &val);
		regs_1a = val;
		bq27xxx_i2c_read_word(di, 0x68, &val);
		regs_68 = val;
		bq27xxx_i2c_read_word(di, 0x6c, &val);
		regs_6c = val;
		bq27xxx_i2c_read_word(di, 0x6e, &val);
		regs_6e = val;
		bq27xxx_i2c_read_word(di, 0x70, &val);
		regs_70 = val;

		pr_notice("00:0x%x,06:0x%x,16:0x%x,66:0x%x,1a:0x%x,68:0x%x,6c:0x%x,6e:0x%x,70:0x%x\n", regs_00, regs_06, regs_16, regs_66, regs_1a, regs_68, regs_6c, regs_6e, regs_70);
	}
	nums ++;
	
	return 0;
}
static int __init bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	int ret;

	di->bat.name		= "bms";
	di->bat.type = POWER_SUPPLY_TYPE_BMS;
	if (di->chip == BQ274XX) {
		set_properties_array(di, bq274xx_battery_props,
			ARRAY_SIZE(bq274xx_battery_props));
	} else if (di->chip == BQ276XX) {
		set_properties_array(di, bq276xx_battery_props,
			ARRAY_SIZE(bq276xx_battery_props));
	} else if (di->chip == BQ27520) {
		set_properties_array(di, bq27520_battery_props,
			ARRAY_SIZE(bq27520_battery_props));
	} else if (di->chip == BQ2753X) {
		set_properties_array(di, bq2753x_battery_props,
			ARRAY_SIZE(bq2753x_battery_props));
	} else if (di->chip == BQ27542) {
		set_properties_array(di, bq27542_battery_props,
			ARRAY_SIZE(bq27542_battery_props));
	} else if (di->chip == BQ27545) {
		set_properties_array(di, bq27545_battery_props,
			ARRAY_SIZE(bq27545_battery_props));
	} else {
		set_properties_array(di, bq27x00_battery_props,
			ARRAY_SIZE(bq27x00_battery_props));
	}
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.external_power_changed = bq27x00_external_power_changed;
	di->bat.properties	= bq274xx_battery_props;
	di->bat.num_properties	= ARRAY_SIZE(bq274xx_battery_props);
	di->bat.supplied_to = bq_fg_supplied_to;
	di->bat.num_supplicants = ARRAY_SIZE(bq_fg_supplied_to);

	INIT_DELAYED_WORK(&di->work, bq27x00_battery_poll);
	mutex_init(&di->lock);

	ret = power_supply_register(di->dev, &di->bat);
	if (ret) {
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	bq27x00_update(di);

	return 0;
}

static void bq27x00_powersupply_unregister(struct bq27x00_device_info *di)
{
	/*
	 * power_supply_unregister call bq27x00_battery_get_property which
	 * call bq27x00_battery_poll.
	 * Make sure that bq27x00_battery_poll will not call
	 * schedule_delayed_work again after unregister (which cause OOPS).
	 */
	poll_interval = 0;

	cancel_delayed_work_sync(&di->work);

	power_supply_unregister(&di->bat);

	mutex_destroy(&di->lock);
}


/* i2c specific code */
#ifdef CONFIG_BATTERY_BQ27X00_I2C

/* If the system has several batteries we need a different name for each
 * of them...
 */
//static DEFINE_IDR(battery_id);
//static DEFINE_MUTEX(battery_mutex);

static int bq27xxx_read_i2c(struct bq27x00_device_info *di, u8 reg, int *val, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		pr_err("%s: reg:0x%x, err:%d\n", __func__, reg, ret);
		return ret;
	}

	if (!single)
		*val = get_unaligned_le16(data);
	else
		*val = data[0];

	return ret;
}

static int bq27xxx_write_i2c(struct bq27x00_device_info *di, u8 reg, int value, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg;
	unsigned char data[4];
	int ret;
	dev_dbg(di->dev, "%s 0x%x <- 0x%x\n", __func__, reg, value);

	if (!client->adapter)
		return -ENODEV;

	data[0] = reg;
	if (single) {
		data[1] = (unsigned char)value;
		msg.len = 2;
	} else {
		put_unaligned_le16(value, &data[1]);
		msg.len = 3;
	}

	msg.buf = data;
	msg.addr = client->addr;
	msg.flags = 0;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0){
		pr_err("%s: reg:0x%x, err:%d\n", __func__, reg, ret);
		return ret;
	}

	return 0;
}

static int bq27xxx_read_i2c_blk(struct bq27x00_device_info *di, u8 reg,
	u8 *data, u8 len)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = 1;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = len;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		pr_err("%s: reg:0x%x, err:%d\n", __func__, reg, ret);
		return ret;
	}

	return ret;
}

static int bq27xxx_write_i2c_blk(struct bq27x00_device_info *di, u8 reg,
	u8 *data, u8 sz)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg;
	int ret;
	u8 buf[33];

	if (!client->adapter)
		return -ENODEV;

	buf[0] = reg;
	memcpy(&buf[1], data, sz);

	msg.buf = buf;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sz + 1;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		pr_err("%s: reg:0x%x, err:%d\n", __func__, reg, ret);
		return ret;
	}

	return 0;
}

static int bq27x00_battery_reset(struct bq27x00_device_info *di)
{
	dev_info(di->dev, "Gas Gauge Reset\n");

	bq27xxx_write(di, BQ27XXX_REG_CTRL, RESET_SUBCMD, false);

	msleep(10);

	return bq27xxx_read(di, BQ27XXX_REG_CTRL, false);
}

static int bq27x00_battery_read_fw_version(struct bq27x00_device_info *di)
{
	bq27xxx_write(di, BQ27XXX_REG_CTRL, FW_VER_SUBCMD, false);

	msleep(10);

	return bq27xxx_read(di, BQ27XXX_REG_CTRL, false);
}

static int bq27x00_battery_read_device_type(struct bq27x00_device_info *di)
{
	bq27xxx_write(di, BQ27XXX_REG_CTRL, DEV_TYPE_SUBCMD, false);

	msleep(10);

	return bq27xxx_read(di, BQ27XXX_REG_CTRL, false);
}

static int bq27x00_battery_read_dataflash_version(struct bq27x00_device_info *di)
{
	bq27xxx_write(di, BQ27XXX_REG_CTRL, DF_VER_SUBCMD, false);

	msleep(10);

	return bq27xxx_read(di, BQ27XXX_REG_CTRL, false);
}

static ssize_t show_firmware_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int ver;

	ver = bq27x00_battery_read_fw_version(di);

	return sprintf(buf, "%d\n", ver);
}

static ssize_t show_dataflash_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int ver;

	ver = bq27x00_battery_read_dataflash_version(di);

	return sprintf(buf, "%d\n", ver);
}

static ssize_t show_device_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int dev_type;

	dev_type = bq27x00_battery_read_device_type(di);

	return sprintf(buf, "%d\n", dev_type);
}

static ssize_t store_reset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int ret;
	unsigned int val;

	ret = sscanf(buf,"%x",&val);
	if (ret == 1 && val == 0x55){
		bq27x00_battery_reset(di);	// reset chip
	}
	return count;
}

static DEVICE_ATTR(fw_version, S_IRUGO, show_firmware_version, NULL);
static DEVICE_ATTR(df_version, S_IRUGO, show_dataflash_version, NULL);
static DEVICE_ATTR(device_type, S_IRUGO, show_device_type, NULL);
static DEVICE_ATTR(reset, S_IWUSR, NULL, store_reset);

static struct attribute *bq27x00_attributes[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_df_version.attr,
	&dev_attr_device_type.attr,
	&dev_attr_reset.attr,
	NULL
};

static const struct attribute_group bq27x00_attr_group = {
	.attrs = bq27x00_attributes,
};

static irqreturn_t bq27x00_irq_handler(int irq, void *dev_id)
{
	struct bq27x00_device_info *di = dev_id;

	pr_notice("%s\n", __func__);

	mutex_lock(&di->irq_complete_lock);
	di->bq_update_waiting = false;
	if(false == di->resume_completed) {
		wake_lock_timeout(&di->fg_poll_wake_lock, 1 * HZ);
		di->bq_update_waiting = true;
		mutex_unlock(&di->irq_complete_lock);
		goto out;
	}
	
	bq27x00_update(di);
	mutex_unlock(&di->irq_complete_lock);

out:
	return IRQ_HANDLED;
}

#if defined(CONFIG_FB)
static int bq27xxx_late_resume(struct bq27x00_device_info *di)
{
	pr_notice("%s \n", __func__);
	di->early_resume_completed = true;
	return 0;
}

static int bq27xxx_early_suspend(struct bq27x00_device_info *di)
{
	pr_notice("%s \n", __func__);
	di->early_resume_completed = false;
	return 0;
}

/*****************************************************************************
*  Name: fb_notifier_smb_callback
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fb_notifier_bq_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct bq27x00_device_info *di = container_of(self, struct bq27x00_device_info, fb_notif);

	if(evdata && evdata->data && di && di->client){
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			bq27xxx_late_resume(di);
		else if (*blank == FB_BLANK_POWERDOWN) {
			bq27xxx_early_suspend(di);
		}
	}
	
	return 0;
}
#endif

static int bq27x00_parse_dt(struct device *dev, struct bq27x00_device_info * di)
{
	struct device_node *np = dev->of_node;
	
	di->irq_gpio  = of_get_named_gpio_flags(np, "qcom,irq-gpio", 0, NULL);

	return 0;
}

static int __init bq27x00_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	int num = 0;
	int retval = 0;
	u8 *regs;

	printk("%s \n", __func__);

	/* Get new ID for the new battery device */
	//idr_preload(GFP_KERNEL);
	//mutex_lock(&battery_mutex);
	//num = idr_alloc(&battery_id, client, 0, 0, GFP_KERNEL);
	//mutex_unlock(&battery_mutex);
	//idr_preload_end();
	//if (num < 0) {
	//	dev_err(&client->dev, "failed to allocate pointer id\n");
	//	return num;
	//}

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	
	bq27x00_parse_dt(&client->dev, di);
	
	di->id = num;
	di->dev = &client->dev;
	di->chip = BQ274XX;    //id->driver_data;
	di->bat.name = name;
	di->bus.read = &bq27xxx_read_i2c;
	di->bus.write = &bq27xxx_write_i2c;
	di->bus.blk_read = bq27xxx_read_i2c_blk;
	di->bus.blk_write = bq27xxx_write_i2c_blk;
	di->dm_regs = NULL;
	di->dm_regs_count = 0;
	di->client = client;
	di->use_internal_dm_regs = false;
	
	di->current_fg_mode = 1;
	di->invalid_count = 0;

	di->early_resume_completed = true;
	di->resume_completed = true;

	di->cache.capacity = 50;
	di->cache.temperature = 3031;

	mutex_init(&di->i2c_rw_lock);
	mutex_init(&di->data_lock);
	mutex_init(&di->irq_complete_lock);
	wake_lock_init(&di->fg_poll_wake_lock, WAKE_LOCK_SUSPEND, "fg_poll_wake");    //added for suspend20170321
	i2c_set_clientdata(client, di);

	if (di->chip == BQ27200)
		regs = bq27200_regs;
	else if (di->chip == BQ27500)
		regs = bq27500_regs;
	else if (di->chip == BQ27520)
		regs = bq27520_regs;
	else if (di->chip == BQ2753X)
		regs = bq2753x_regs;
	else if (di->chip == BQ27542 || di->chip == BQ27545)
		regs = bq2754x_regs;
	else if (di->chip == BQ274XX) {
		regs = bq274xx_regs;
		di->dm_regs = bq274xx_dm_regs;
		di->dm_regs_count = ARRAY_SIZE(bq274xx_dm_regs);
	} else if (di->chip == BQ276XX) {
		/* commands are same as bq274xx, only DM is different */
		regs = bq276xx_regs;
		di->dm_regs = bq276xx_dm_regs;
		di->dm_regs_count = ARRAY_SIZE(bq276xx_dm_regs);
	} else {
		dev_err(&client->dev,
			"Unexpected gas gague: %d\n", di->chip);
		regs = bq27520_regs;
	}

	memcpy(di->regs, regs, NUM_REGS);


	di->fw_ver = bq27x00_battery_read_fw_version(di);
	if(di->fw_ver < 0) {
		dev_err(&client->dev, "i2c op error.\n");
		goto batt_failed_3;
	}
	dev_info(&client->dev, "Gas Guage fw version is 0x%04x\n", di->fw_ver);



	fg_update_bqfs(di);
	bq27x00_dm_compare_work(di);

	retval = bq27x00_powersupply_init(di);
	if (retval)
		goto batt_failed_3;

	/* Schedule a polling after about 1 min */
	schedule_delayed_work(&di->work, 20 * HZ);
	
	if (gpio_is_valid(di->irq_gpio)) {
		int irq = 0;
		int rc = 0;
		
		rc = gpio_request(di->irq_gpio, "bq27x00_irq_gpio");
		if (rc) {
			dev_err(&client->dev,
					"irq gpio request failed, rc=%d", rc);
		} else {
			pr_info("bq27x00_irq_gpio request succes");
			rc = gpio_direction_input(di->irq_gpio);

			irq = gpio_to_irq(di->irq_gpio);
			if (irq < 0) {
				dev_err(&client->dev,
					"Invalid irq_gpio irq = %d\n", irq);
			} else {
				pr_info("gpio_to_irq succes");
				rc = devm_request_threaded_irq(&client->dev, irq, NULL,
						bq27x00_irq_handler,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"bq27x00_irq_handler", di);
				if (rc) {
					dev_err(&client->dev,
						"Failed STAT irq=%d request rc = %d\n",
						irq, rc);
				} else {
					pr_info("devm_request_threaded_irq bq27x00_irq_handler irq success");
				}
			}
		}
		enable_irq_wake(irq);
	}

#if defined(CONFIG_FB)
	di->fb_notif.notifier_call = fb_notifier_bq_callback;
	retval = fb_register_client(&di->fb_notif);
	if (retval)
		pr_err("[FB]Unable to register fb_notifier: %d", retval);
#endif

#ifdef CONFIG_PROC_FS
	create_charge_proc_entry(di);
#endif
	retval = sysfs_create_group(&client->dev.kobj, &bq27x00_attr_group);
	if (retval)
		dev_err(&client->dev, "could not create sysfs files\n");

	return 0;

batt_failed_3:
	printk("%s batt_failed_3 \n", __func__);
	//kfree(di);
batt_failed_2:
	//kfree(name);
batt_failed_1:
	//mutex_lock(&battery_mutex);
	//idr_remove(&battery_id, num);
	//mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27x00_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	bq27x00_powersupply_unregister(di);

	kfree(di->bat.name);

	//mutex_lock(&battery_mutex);
	//idr_remove(&battery_id, di->id);
	//mutex_unlock(&battery_mutex);

	kfree(di);

	return 0;
}

static void bq27xxx_battery_shutdown(struct i2c_client *client)
{
	pr_notice("%s\n", __func__);
	return;
}
static int bq27xxx_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	//pr_notice("%s\n", __func__);

	mutex_lock(&di->irq_complete_lock);
	di->resume_completed = false;
	mutex_unlock(&di->irq_complete_lock);

	return 0;
}
static int bq27xxx_suspend_noirq(struct device *dev)
{
	//pr_notice("%s\n", __func__);
	return 0;
}
static int bq27xxx_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	//pr_notice("%s\n", __func__);
	mutex_lock(&di->irq_complete_lock);
	di->resume_completed = true;
	mutex_unlock(&di->irq_complete_lock);
	
	if(di->bq_update_waiting) {
		pr_notice("%s update work waiting in suspend, need perform\n", __func__);
		bq27x00_update(di);
	}

	return 0;
}
static const struct dev_pm_ops bq27xxx_pm_ops = {
	.suspend	= bq27xxx_suspend,
	.suspend_noirq	= bq27xxx_suspend_noirq,
	.resume		= bq27xxx_resume,
};
static struct of_device_id bq27x00_match_table[] = {
	{ .compatible = "ti,bq27x00-battery",},
	{ },
};

static const struct i2c_device_id bq27x00_id[] = {
	{ "bq27200", BQ27200 },
	{ "bq27500", BQ27500 },
	{ "bq27520", BQ27520 },
	{ "bq274xx", BQ274XX },
	{ "bq276xx", BQ276XX },
	{ "bq2753x", BQ2753X },
	{ "bq27542", BQ27542 },
	{ "bq27545", BQ27545 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27x00_id);

static struct i2c_driver __refdata bq27x00_battery_driver = {
	.driver = {
		.name = "ti,bq27x00-battery",
		.owner		= THIS_MODULE,
		.of_match_table = bq27x00_match_table,
		.pm		= &bq27xxx_pm_ops,
	},
	.probe = bq27x00_battery_probe,
	.remove = bq27x00_battery_remove,
	.shutdown = bq27xxx_battery_shutdown,
	.id_table = bq27x00_id,
};

module_i2c_driver(bq27x00_battery_driver);

//static inline int __init bq27x00_battery_i2c_init(void)
//{
//	int ret = i2c_add_driver(&bq27x00_battery_driver);
//	if (ret)
//		printk(KERN_ERR "Unable to register BQ27x00 i2c driver\n");
//
//	return ret;
//}
//
//static inline void __exit bq27x00_battery_i2c_exit(void)
//{
//	///	i2c_del_driver(&bq27x00_battery_driver);
//
//}


#else

//static inline int bq27x00_battery_i2c_init(void) { return 0; }
//static inline void bq27x00_battery_i2c_exit(void) {};
//
//#endif
//
/* platform specific code */
//#ifdef CONFIG_BATTERY_BQ27X00_PLATFORM

static int bq27000_read_platform(struct bq27x00_device_info *di, u8 reg,
			bool single)
{
	struct device *dev = di->dev;
	struct bq27000_platform_data *pdata = dev->platform_data;
	unsigned int timeout = 3;
	int upper, lower;
	int temp;

	if (!single) {
		/* Make sure the value has not changed in between reading the
		 * lower and the upper part */
		upper = pdata->read(dev, reg + 1);
		do {
			temp = upper;
			if (upper < 0)
				return upper;

			lower = pdata->read(dev, reg);
			if (lower < 0)
				return lower;

			upper = pdata->read(dev, reg + 1);
		} while (temp != upper && --timeout);

		if (timeout == 0)
			return -EIO;

		return (upper << 8) | lower;
	}

	return pdata->read(dev, reg);
}

static int bq27000_battery_probe(struct platform_device *pdev)
{
	struct bq27x00_device_info *di;
	struct bq27000_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform_data supplied\n");
		return -EINVAL;
	}

	if (!pdata->read) {
		dev_err(&pdev->dev, "no hdq read callback supplied\n");
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;
	di->chip = BQ27200;

	di->bat.name = pdata->name ?: dev_name(&pdev->dev);
	di->bus.read = &bq27000_read_platform;

	ret = bq27x00_powersupply_init(di);
	if (ret)
		goto err_free;

	return 0;

err_free:
	kfree(di);

	return ret;
}

static int bq27000_battery_remove(struct platform_device *pdev)
{
	struct bq27x00_device_info *di = platform_get_drvdata(pdev);

	bq27x00_powersupply_unregister(di);

	kfree(di);

	return 0;
}

static struct platform_driver __initdata bq27000_battery_driver = {
	.probe	= bq27000_battery_probe,
	.remove = bq27000_battery_remove,
	.driver = {
		.name = "bq27000-battery",
		.owner = THIS_MODULE,
	},
};

static inline int bq27x00_battery_platform_init(void)
{
	int ret = platform_driver_register(&bq27000_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27200 platform driver\n");

	return ret;
}

static inline void bq27x00_battery_platform_exit(void)
{
	platform_driver_unregister(&bq27000_battery_driver);
}

//#else

//static inline int bq27x00_battery_platform_init(void) { return 0; }
//static inline void bq27x00_battery_platform_exit(void) {};

#endif

/*
 * Module stuff
 */

/*
static int __init bq27x00_battery_init(void)
{
	int ret;

	ret = bq27x00_battery_i2c_init();
	if (ret)
		return ret;

	ret = bq27x00_battery_platform_init();
	if (ret)
		bq27x00_battery_i2c_exit();

	return ret;
}
module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
	bq27x00_battery_platform_exit();
	bq27x00_battery_i2c_exit();
}
module_exit(bq27x00_battery_exit);
*/

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");

