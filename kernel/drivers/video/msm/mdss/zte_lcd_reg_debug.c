#include "mdss_dsi.h"
#include "zte_lcd_common.h"

#ifdef ZTE_LCD_DEBUG

struct zte_lcd_reg_debug zte_lcd_reg_debug;
struct mdss_dsi_ctrl_pdata *g_zte_ctrl_pdata;


#define SYSFS_FOLDER_NAME "lcd_reg_debug"


static void zte_lcd_reg_rw_func(struct mdss_dsi_ctrl_pdata *ctrl,struct zte_lcd_reg_debug *reg_debug)
{
	//int i;
	int read_length;
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc write_lcd_cmd;

	write_lcd_cmd.dchdr.dtype = reg_debug->dtype;
	write_lcd_cmd.dchdr.last = 1;
	write_lcd_cmd.dchdr.vc = 0;
	write_lcd_cmd.dchdr.dlen = reg_debug->length;
	write_lcd_cmd.payload = (char *)reg_debug->wbuf;

	#if 0
	for(i=0;i<reg_debug->length;i++)
		printk("rwbuf[%d]= %x\n",i,reg_debug->wbuf[i]);
	#endif
	
	memset(&cmdreq, 0, sizeof(cmdreq));
	switch(reg_debug->is_read_mode)
	{
		case 1:	
			read_length = reg_debug->wbuf[1];
			reg_debug->wbuf[1] = 0;

			write_lcd_cmd.dchdr.ack = 1;
			write_lcd_cmd.dchdr.wait = 5;//5 ms
			cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
			cmdreq.rbuf = (char*)reg_debug->rbuf;
			cmdreq.rlen = read_length;
			break;
		case 0:
			write_lcd_cmd.dchdr.ack = 0;
			write_lcd_cmd.dchdr.wait = 5;//5 ms
			cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
			cmdreq.rbuf = NULL;
			cmdreq.rlen = 0;
			break;
		default:
			printk("%s:rw error\n",__FUNCTION__);
			break;
	}
	cmdreq.cmds = &write_lcd_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static void get_user_sapce_data(const char *buf, size_t count)
{
	int i,length;
	char lcd_status[64]={"0"};
	
	if (count >= sizeof(lcd_status))
	{
	    printk("count=%ld  sizeof(lcd_status)=%ld\n",count,sizeof(lcd_status));
	    return ;
	}

	strlcpy(lcd_status, buf, count);
	memset(zte_lcd_reg_debug.wbuf,0,32);
	memset(zte_lcd_reg_debug.rbuf,0,32);
	
	#if 0
	for(i=0;i<count;i++)
		printk("lcd_status[%d]=%c  %d\n",i,lcd_status[i],lcd_status[i]);
	#endif
	for(i=0;i<count;i++)
	{
		if (isdigit (lcd_status[i]))
			lcd_status[i]-='0';
		else if (isalpha (lcd_status[i]))
			lcd_status[i]-=(isupper(lcd_status[i]) ? 'A' - 10 : 'a' - 10);
	}

	for(i=0,length=0;i<(count-1);i=i+2,length++)
	{
		zte_lcd_reg_debug.wbuf[length]=lcd_status[i]*16+lcd_status[1+i];
	}

	zte_lcd_reg_debug.length=length; //length is use space write data number

}

static ssize_t sysfs_show_read(struct device *d, struct device_attribute *attr,char *buf)
{
	int i;
	char *s;
	char data_buf[100];
	
	s=&data_buf[0];
        for(i=0;i<zte_lcd_reg_debug.length;i++)
        {
                sprintf(s,"rbuf[%d]=%02x ",i,zte_lcd_reg_debug.rbuf[i]);
                s+=11;
        }

	return sprintf(buf, "read back:%s\n", &data_buf[0]);
}
static ssize_t sysfs_store_dread(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int i,length;
	get_user_sapce_data(buf,count);
	length = zte_lcd_reg_debug.wbuf[1];
	if(length<1)
	{
		printk("%s:read length is 0\n",__FUNCTION__);
		return count;
	}
	
	zte_lcd_reg_debug.is_read_mode = 1; // 1 read ,0 write
	zte_lcd_reg_debug.dtype = DTYPE_DCS_READ;

	printk("[MSM_LCD] dtype = %x read cmd = %x length = %x\n",zte_lcd_reg_debug.dtype,zte_lcd_reg_debug.wbuf[0],length);
	zte_lcd_reg_rw_func(g_zte_ctrl_pdata,&zte_lcd_reg_debug);

	zte_lcd_reg_debug.length = length;
	for(i=0;i<length;i++)
		printk("read zte_lcd_reg_debug.rbuf[%d]=0x%02x\n",i,zte_lcd_reg_debug.rbuf[i]);
	
	return count;
}

static ssize_t sysfs_store_gread(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int i,length;
	get_user_sapce_data(buf,count);
	length = zte_lcd_reg_debug.wbuf[1];
	if(length<1)
	{
		printk("%s:read length is 0\n",__FUNCTION__);
		return count;
	}

	zte_lcd_reg_debug.is_read_mode = 1; // 1 read ,0 write

	if(zte_lcd_reg_debug.wbuf[1]>=3)
		zte_lcd_reg_debug.dtype = DTYPE_GEN_READ2;
	else if(zte_lcd_reg_debug.wbuf[1]==2)
		zte_lcd_reg_debug.dtype = DTYPE_GEN_READ1;
	else
		zte_lcd_reg_debug.dtype = DTYPE_GEN_READ;

	printk("[MSM_LCD] dtype = %x read cmd = %x num = %x\n",zte_lcd_reg_debug.dtype,zte_lcd_reg_debug.wbuf[0],length);
	zte_lcd_reg_rw_func(g_zte_ctrl_pdata,&zte_lcd_reg_debug);

	zte_lcd_reg_debug.length = length;
	for(i=0;i<length;i++)
		printk("read zte_lcd_reg_debug.rbuf[%d]=0x%02x\n",i,zte_lcd_reg_debug.rbuf[i]);
	
	return count;
}

static ssize_t sysfs_store_dwrite(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int length;
	get_user_sapce_data(buf,count);
	length = zte_lcd_reg_debug.length;

	zte_lcd_reg_debug.is_read_mode = 0; // 1 read ,0 write

	if(length>=3)
		zte_lcd_reg_debug.dtype = DTYPE_DCS_LWRITE;
	else if(length==2)
		zte_lcd_reg_debug.dtype = DTYPE_DCS_WRITE1;
	else
		zte_lcd_reg_debug.dtype = DTYPE_DCS_WRITE;

	zte_lcd_reg_rw_func(g_zte_ctrl_pdata,&zte_lcd_reg_debug);
	printk("[MSM_LCD] dtype = 0x%02x write cmd = 0x%02x length = 0x%02x\n",zte_lcd_reg_debug.dtype,zte_lcd_reg_debug.wbuf[0],length);

	return count;
}

static ssize_t sysfs_store_gwrite(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int length;
	get_user_sapce_data(buf,count);
	length = zte_lcd_reg_debug.length;

	zte_lcd_reg_debug.is_read_mode = 0; // 1 read ,0 write

	if(length>=3)
		zte_lcd_reg_debug.dtype = DTYPE_GEN_LWRITE;
	else if(length==2)
		zte_lcd_reg_debug.dtype = DTYPE_GEN_WRITE1;
	else
		zte_lcd_reg_debug.dtype = DTYPE_GEN_WRITE;

	zte_lcd_reg_rw_func(g_zte_ctrl_pdata,&zte_lcd_reg_debug);
	printk("[MSM_LCD] dtype = 0x%02x write cmd = 0x%02x length = 0x%02x\n",zte_lcd_reg_debug.dtype,zte_lcd_reg_debug.wbuf[0],length);

	return count;
}

static ssize_t sysfs_show_reserved(struct device *d, struct device_attribute *attr,char *buf)
{

	return sprintf(buf, "read back:%s\n", zte_lcd_reg_debug.reserved);
}

static ssize_t sysfs_store_reserved(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int i;
	get_user_sapce_data(buf,count);

	for(i=0;i<zte_lcd_reg_debug.length;i++)
		printk("write data [%d]=0x%02x\n",i,zte_lcd_reg_debug.wbuf[i]);

	/******************************* add code here ************************************************/
	
	return count;
}
static DEVICE_ATTR(dread, 0600,sysfs_show_read, sysfs_store_dread);
static DEVICE_ATTR(gread, 0600,sysfs_show_read, sysfs_store_gread);
static DEVICE_ATTR(dwrite, 0600,NULL, sysfs_store_dwrite);
static DEVICE_ATTR(gwrite, 0600,NULL, sysfs_store_gwrite);
static DEVICE_ATTR(reserved, 0600,sysfs_show_reserved, sysfs_store_reserved);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_dread.attr,
	&dev_attr_gread.attr,
	&dev_attr_dwrite.attr,
	&dev_attr_gwrite.attr,
	&dev_attr_reserved.attr,
	NULL,
};

static struct attribute_group sysfs_attr_group = {
	.attrs = sysfs_attrs,
};

void zte_lcd_reg_debug_func(struct mdss_dsi_ctrl_pdata *ctrl_pdata,struct device_node *node)
{
	int ret;
	struct kobject *vkey_obj;

	g_zte_ctrl_pdata = ctrl_pdata;
	vkey_obj = kobject_create_and_add(SYSFS_FOLDER_NAME, NULL);
	if (!vkey_obj) {
		pr_err("%s: unable to create kobject\n",__FUNCTION__);
	}

	ret = sysfs_create_group(vkey_obj, &sysfs_attr_group);
	if (ret) {
		pr_err("%s: failed to create attributes\n",__FUNCTION__);
	}
}
#endif
