/* For MTK android platform.
 *
 * msa.c - Linux kernel modules for 3-Axis Accelerometer
 *
 * Copyright (C) 2007-2016 MEMS Sensing Technology Co., Ltd.
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
//#include <mach/gpio.h>
//#include <mach/board.h> 
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>
#include <linux/syscalls.h>
#include <linux/fs.h>

#include "msa_core.h"
#include "msa_cust.h"
/******************************************************************************/
#define GSENSOR_MIN        					2
#define MSA_PRECISION           				12
#define MSA_RANGE               				32768
#define MSA_BOUNDARY            				(0x1 << (MSA_PRECISION - 1))
#define MSA_GRAVITY_STEP        			(MSA_RANGE/MSA_BOUNDARY)
/******************************************************************************/
#define MSA_DRV_NAME                 			"msa"
#define MSA_INPUT_DEV_NAME     			MSA_DRV_NAME
/******************************************************************************/
static MSA_HANDLE              msa_handle;
extern int                              	Log_level;
static int is_init =0;
/******************************************************************************/
#define MI_DATA(format, ...)            if(DEBUG_DATA&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
#define MI_MSG(format, ...)             if(DEBUG_MSG&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
#define MI_ERR(format, ...)             if(DEBUG_ERR&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
#define MI_FUN                          if(DEBUG_FUNC&Log_level){printk(KERN_ERR MI_TAG "%s is called, line: %d\n", __FUNCTION__,__LINE__);}
#define MI_ASSERT(expr)                 \
	if (!(expr)) {\
		printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);\
	}
/******************************************************************************/
#if MSA_OFFSET_TEMP_SOLUTION
static char OffsetFileName[] = "/data/misc/msaGSensorOffset.txt";
static char OffsetFolerName[] = "/data/misc/";
#define OFFSET_STRING_LEN               26
struct work_info
{
    char        tst1[20];
    char        tst2[20];
    char        buffer[OFFSET_STRING_LEN];
    struct      workqueue_struct *wq;
    struct      delayed_work read_work;
    struct      delayed_work write_work;
    struct      completion completion;
    int         len;
    int         rst; 
};

static struct work_info m_work_info = {{0}};
/******************************************************************************/
static void sensor_write_work( struct work_struct *work )
{
    struct work_info*   pWorkInfo;
    struct file         *filep;
    mm_segment_t                 orgfs;
    int                 ret;   

    orgfs = get_fs();
    set_fs(KERNEL_DS);

    pWorkInfo = container_of((struct delayed_work*)work, struct work_info, write_work);
    if (pWorkInfo == NULL){            
            MI_ERR("get pWorkInfo failed!");       
            return;
    }
    
    filep = filp_open(OffsetFileName, O_RDWR|O_CREAT, 0666);
    if (IS_ERR(filep)){
        MI_ERR("write, sys_open %s error!!.\n", OffsetFileName);
        ret =  -1;
    }
    else
    {   
        filep->f_op->write(filep, pWorkInfo->buffer, pWorkInfo->len, &filep->f_pos);
        filp_close(filep, NULL);
        ret = 0;        
    }
    
    set_fs(orgfs);   
    pWorkInfo->rst = ret;
    complete( &pWorkInfo->completion );
}
/******************************************************************************/
static void sensor_read_work( struct work_struct *work )
{
    mm_segment_t orgfs;
    struct file *filep;
    int ret; 
    struct work_info* pWorkInfo;
        
    printk("--- %s, %d ---\n", __func__, __LINE__);
    orgfs = get_fs();
    printk("--- %s, %d ---\n", __func__, __LINE__);
    set_fs(KERNEL_DS);
    printk("--- %s, %d ---\n", __func__, __LINE__);
    
    pWorkInfo = container_of((struct delayed_work*)work, struct work_info, read_work);
    printk("--- %s, %d ---\n", __func__, __LINE__);
    if (pWorkInfo == NULL){            
        MI_ERR("get pWorkInfo failed!");       
        return;
    }
    printk("--- %s, %d ---\n", __func__, __LINE__);
 
    filep = filp_open(OffsetFileName, O_RDONLY, 0666);
    printk("--- %s, %d ---\n", __func__, __LINE__);
    if (IS_ERR(filep)){
    	printk("--- %s, %d ---\n", __func__, __LINE__);
        MI_ERR("read, sys_open %s error!!.\n",OffsetFileName);
        set_fs(orgfs);
        ret =  -1;
    }
    else{
    	printk("--- %s, %d ---\n", __func__, __LINE__);
        filep->f_op->read(filep, pWorkInfo->buffer,  sizeof(pWorkInfo->buffer), &filep->f_pos);
        filp_close(filep, NULL);    
        set_fs(orgfs);
        ret = 0;
    }

    printk("--- %s, %d ---\n", __func__, __LINE__);
    pWorkInfo->rst = ret;
    printk("--- %s, %d ---\n", __func__, __LINE__);
    complete( &(pWorkInfo->completion) );
    printk("--- %s, %d ---\n", __func__, __LINE__);
}
/******************************************************************************/
static int sensor_sync_read(u8* offset)
{
    int     err;
    int     off[MSA_OFFSET_LEN] = {0};
    struct work_info* pWorkInfo = &m_work_info;
     
    init_completion( &pWorkInfo->completion );
    queue_delayed_work( pWorkInfo->wq, &pWorkInfo->read_work, msecs_to_jiffies(0) );
    err = wait_for_completion_timeout( &pWorkInfo->completion, msecs_to_jiffies( 2000 ) );
    if ( err == 0 ){
        MI_ERR("wait_for_completion_timeout TIMEOUT");
        return -1;
    }

    if (pWorkInfo->rst != 0){
        MI_ERR("work_info.rst  not equal 0");
        return pWorkInfo->rst;
    }
    
    sscanf(m_work_info.buffer, "%x,%x,%x,%x,%x,%x,%x,%x,%x", &off[0], &off[1], &off[2], &off[3], &off[4], &off[5],&off[6], &off[7], &off[8]);

    offset[0] = (u8)off[0];
    offset[1] = (u8)off[1];
    offset[2] = (u8)off[2];
    offset[3] = (u8)off[3];
    offset[4] = (u8)off[4];
    offset[5] = (u8)off[5];
    offset[6] = (u8)off[6];
    offset[7] = (u8)off[7];
    offset[8] = (u8)off[8];
    
    return 0;
}
/******************************************************************************/
static int sensor_sync_write(u8* off)
{
    int err = 0;
    struct work_info* pWorkInfo = &m_work_info;
       
    init_completion( &pWorkInfo->completion );
    
    sprintf(m_work_info.buffer, "%x,%x,%x,%x,%x,%x,%x,%x,%x\n", off[0],off[1],off[2],off[3],off[4],off[5],off[6],off[7],off[8]);
    
    pWorkInfo->len = sizeof(m_work_info.buffer);
        
    queue_delayed_work( pWorkInfo->wq, &pWorkInfo->write_work, msecs_to_jiffies(0) );
    err = wait_for_completion_timeout( &pWorkInfo->completion, msecs_to_jiffies( 2000 ) );
    if ( err == 0 ){
        MI_ERR("wait_for_completion_timeout TIMEOUT");
        return -1;
    }

    if (pWorkInfo->rst != 0){
        MI_ERR("work_info.rst  not equal 0");
        return pWorkInfo->rst;
    }
    
    return 0;
}
/******************************************************************************/
static int check_califolder_exist(void)
{
    mm_segment_t     orgfs;
    struct  file *filep;
        
    orgfs = get_fs();
    set_fs(KERNEL_DS);

    filep = filp_open(OffsetFolerName, O_RDONLY, 0666);
    if (IS_ERR(filep)) {
        MI_ERR("%s read, sys_open %s error!!.\n",__func__,OffsetFolerName);
        set_fs(orgfs);
        return 0;
    }

    filp_close(filep, NULL);    
    set_fs(orgfs); 

    return 1;
}
/******************************************************************************/
static int support_fast_auto_cali(void)
{
#if MSA_SUPPORT_FAST_AUTO_CALI
    return 1;
#else
    return 0;
#endif
}
#endif
/******************************************************************************/
static int get_address(PLAT_HANDLE handle)
{
    if(NULL == handle){
        MI_ERR("chip init failed !\n");
		    return -1;
    }
			
	return ((struct i2c_client *)handle)->addr; 		
}
/******************************************************************************/
int i2c_smbus_read(PLAT_HANDLE handle, u8 addr, u8 *data)
{
    int                 res = 0;
    struct i2c_client   *client = (struct i2c_client*)handle;
    
    *data = i2c_smbus_read_byte_data(client, addr);
    
    return res;
}
/******************************************************************************/
int i2c_smbus_read_block(PLAT_HANDLE handle, u8 addr, u8 count, u8 *data)
{
    int                 res = 0;
    struct i2c_client   *client = (struct i2c_client*)handle;
    
    res = i2c_smbus_read_i2c_block_data(client, addr, count, data);
    
    return res;
}
/******************************************************************************/
int i2c_smbus_write(PLAT_HANDLE handle, u8 addr, u8 data)
{
    int                 res = 0;
    struct i2c_client   *client = (struct i2c_client*)handle;
    
    res = i2c_smbus_write_byte_data(client, addr, data);
    
    return res;
}
/******************************************************************************/
void msdelay(int ms)
{
    mdelay(ms);
}
/******************************************************************************/

#if MSA_OFFSET_TEMP_SOLUTION
MSA_GENERAL_OPS_DECLARE(ops_handle, i2c_smbus_read, i2c_smbus_read_block, i2c_smbus_write, sensor_sync_write, sensor_sync_read, check_califolder_exist,get_address,support_fast_auto_cali,msdelay, printk, sprintf);
#else
MSA_GENERAL_OPS_DECLARE(ops_handle, i2c_smbus_read, i2c_smbus_read_block, i2c_smbus_write, NULL, NULL, NULL,get_address,NULL,msdelay, printk, sprintf);
#endif

/******************************************************************************/
static ssize_t msa_enable_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
    int ret;
    char bEnable;

    MI_FUN;
    printk("--- msa_enable_show ---\n"); 
    ret = msa_get_enable(msa_handle, &bEnable);    
    if (ret < 0){
        ret = -EINVAL;
    }
    else{
        ret = sprintf(buf, "%d\n", bEnable);
    }

    return ret;
}
/******************************************************************************/
static ssize_t msa_enable_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ret;
    bool bEnable;
    unsigned long enable;
	
    if (buf == NULL){
        return -1;
    }

    enable = simple_strtoul(buf, NULL, 10);    
    bEnable = (enable > 0) ? true : false;

    MI_MSG("%s:enable=%d\n",__func__,bEnable);	

    ret = msa_set_enable (msa_handle, bEnable);
    if (ret < 0){
        ret = -EINVAL;
    }
    else{
        ret = count;
    }

    return ret;
}
/******************************************************************************/
static ssize_t msa_axis_data_show(struct device *dev,
           struct device_attribute *attr, char *buf)
{
    int result;
    short x,y,z;
    int count = 0;

    result = msa_read_data(msa_handle, &x, &y, &z);
    if (result == 0)
        count += sprintf(buf+count, "x= %d;y=%d;z=%d\n", x,y,z);
    else
        count += sprintf(buf+count, "reading failed!");

    return count;
}

static ssize_t msa_axis_data_x(struct device *dev,
           struct device_attribute *attr, char *buf)
{
    int result;
    short x,y,z;
    int count = 0;

    result = msa_read_data(msa_handle, &x, &y, &z);
    if (result == 0)
        count += sprintf(buf+count, "%d\n", x);
    else
        count += sprintf(buf+count, "reading failed!");

    return count;
}
static ssize_t msa_axis_data_y(struct device *dev,
           struct device_attribute *attr, char *buf)
{
    int result;
    short x,y,z;
    int count = 0;

    result = msa_read_data(msa_handle, &x, &y, &z);
    if (result == 0)
        count += sprintf(buf+count, "%d\n", y);
    else
        count += sprintf(buf+count, "reading failed!");

    return count;
}
static ssize_t msa_axis_data_z(struct device *dev,
           struct device_attribute *attr, char *buf)
{
    int result;
    short x,y,z;
    int count = 0;

    result = msa_read_data(msa_handle, &x, &y, &z);
    if (result == 0)
        count += sprintf(buf+count, "%d\n", z);
    else
        count += sprintf(buf+count, "reading failed!");

    return count;
}

/******************************************************************************/
static ssize_t msa_reg_data_store(struct device *dev,
           struct device_attribute *attr, const char *buf, size_t count)
{
    int                 addr, data;
    int                 result;
    
    sscanf(buf, "0x%x, 0x%x\n", &addr, &data);
    
    result = msa_register_write(msa_handle, addr, data);
    
    MI_ASSERT(result==0);

    return count;
}
/******************************************************************************/
static ssize_t msa_reg_data_show(struct device *dev,
           struct device_attribute *attr, char *buf)
{
    MSA_HANDLE          handle = msa_handle;
        
    return msa_get_reg_data(handle, buf);
}
/******************************************************************************/
//static ssize_t msa_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
//{
//    ssize_t count = 0;
//    
//    if(bLoad==FILE_EXIST)
//    	count += sprintf(buf,"%s",m_work_info.buffer);   
//    else
//    	count += sprintf(buf,"%s","Calibration file not exist!\n");
//
//    return count;
//}     
/******************************************************************************/
#if FILTER_AVERAGE_ENHANCE
static ssize_t msa_average_enhance_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
    int                             ret = 0;
    struct msa_filter_param_s    param = {0};

    ret = msa_get_filter_param(&param);
    ret |= sprintf(buf, "%d %d %d\n", param.filter_param_l, param.filter_param_h, param.filter_threhold);

    return ret;
}
/******************************************************************************/
static ssize_t msa_average_enhance_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{ 
    int                             ret = 0;
    struct msa_filter_param_s    param = {0};
    
    sscanf(buf, "%d %d %d\n", &param.filter_param_l, &param.filter_param_h, &param.filter_threhold);
    
    ret = msa_set_filter_param(&param);
    
    return count;
}
#endif
/******************************************************************************/
#if MSA_OFFSET_TEMP_SOLUTION
int bCaliResult = -1;
static ssize_t msa_calibrate_show(struct device *dev,struct device_attribute *attr,char *buf)
{
    int ret;       

    ret = sprintf(buf, "%d\n", bCaliResult);   
    return ret;
}
/******************************************************************************/
static ssize_t msa_calibrate_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    s8      z_dir = 0;
    MSA_HANDLE      handle = msa_handle;	
   
    z_dir = simple_strtol(buf, NULL, 10);
    bCaliResult = msa_calibrate(handle,z_dir);
    
    return count;
}
#endif
/******************************************************************************/
static ssize_t msa_log_level_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
    int ret;

    ret = sprintf(buf, "%d\n", Log_level);

    return ret;
}
/******************************************************************************/
static ssize_t msa_log_level_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    Log_level = simple_strtoul(buf, NULL, 10);    

    return count;
}
/******************************************************************************/
static ssize_t msa_primary_offset_show(struct device *dev,
                   struct device_attribute *attr, char *buf){    
    MSA_HANDLE   handle = msa_handle;	
    int x=0,y=0,z=0;
   
    msa_get_primary_offset(handle,&x,&y,&z);

	  return sprintf(buf, "x=%d ,y=%d ,z=%d\n",x,y,z);
}
/******************************************************************************/
static ssize_t msa_version_show(struct device *dev,
                   struct device_attribute *attr, char *buf){    

	return sprintf(buf, "%s_%s\n", DRI_VER, CORE_VER);
}
/******************************************************************************/
static ssize_t msa_vendor_show(struct device *dev,
                   struct device_attribute *attr, char *buf){
	return sprintf(buf, "%s\n", "MsaMEMS");
}
/******************************************************************************/
static DEVICE_ATTR(enable,          0660,  msa_enable_show,             msa_enable_store);
static DEVICE_ATTR(axis_data,       0660,            msa_axis_data_show,          NULL);
static DEVICE_ATTR(axis_data_x,       0664,            msa_axis_data_x,          NULL);
static DEVICE_ATTR(axis_data_y,       0664,            msa_axis_data_y,          NULL);
static DEVICE_ATTR(axis_data_z,       0664,            msa_axis_data_z,          NULL);
static DEVICE_ATTR(reg_data,        0660,  msa_reg_data_show,           msa_reg_data_store);
static DEVICE_ATTR(log_level,       0660,  msa_log_level_show,          msa_log_level_store);
#if MSA_OFFSET_TEMP_SOLUTION
static DEVICE_ATTR(offset,          0660,  msa_offset_show,             NULL);
static DEVICE_ATTR(calibrate_msaGSensor,       	0660,  msa_calibrate_show,          msa_calibrate_store);
#endif
#if FILTER_AVERAGE_ENHANCE
static DEVICE_ATTR(average_enhance, 0660,  msa_average_enhance_show,    msa_average_enhance_store);
#endif
static DEVICE_ATTR(primary_offset,  0660,            msa_primary_offset_show,     NULL);
static DEVICE_ATTR(version,         0660,            msa_version_show,            NULL);
static DEVICE_ATTR(vendor,          0660,            msa_vendor_show,             NULL); 

/******************************************************************************/
static struct attribute *msa_attributes[] = { 
    &dev_attr_enable.attr,
    &dev_attr_axis_data.attr,
    &dev_attr_axis_data_x.attr,
    &dev_attr_axis_data_y.attr,
    &dev_attr_axis_data_z.attr,
    &dev_attr_reg_data.attr,
    &dev_attr_log_level.attr,
#if MSA_OFFSET_TEMP_SOLUTION
    &dev_attr_offset.attr,    
    &dev_attr_calibrate_msaGSensor.attr,
#endif
#if FILTER_AVERAGE_ENHANCE
    &dev_attr_average_enhance.attr,
#endif 
    &dev_attr_primary_offset.attr,
    &dev_attr_version.attr,
    &dev_attr_vendor.attr,
    NULL
};

static const struct attribute_group msa_attr_group = {
    .attrs  = msa_attributes,
};
/******************************************************************************/
static int sensor_init(struct i2c_client *client)
{
    int ret = 0;
   // static int withSysAttr = 1;
	unsigned char chip_id=0;
	unsigned char i=0;
	
    struct sensor_private_data *sensor =(struct sensor_private_data *) i2c_get_clientdata(client);

    MI_FUN;    

    printk("--- %s, %d ---\n", __func__, __LINE__);
    if(is_init)	   
	return 0;	

    printk("--- %s, %d ---\n", __func__, __LINE__);
    sensor->status_cur = SENSOR_OFF;

    printk("--- %s, %d ---\n", __func__, __LINE__);
    if(msa_install_general_ops(&ops_handle)){
        MI_ERR("Install ops failed !\n");
        return -1;
    }

#if MSA_OFFSET_TEMP_SOLUTION
    m_work_info.wq = create_singlethread_workqueue( "oo" );
    if(NULL==m_work_info.wq) {
        MI_ERR("Failed to create workqueue !");
        return -1;
    }
    
    INIT_DELAYED_WORK( &m_work_info.read_work, sensor_read_work );
    INIT_DELAYED_WORK( &m_work_info.write_work, sensor_write_work );
#endif

    	printk("--- %s, %d ---\n", __func__, __LINE__);
	i2c_smbus_read((PLAT_HANDLE) client, NSA_REG_WHO_AM_I, &chip_id);	
    	printk("--- %s, %d chip_id = %d ---\n", __func__, __LINE__, chip_id);
	if(chip_id != 0x13){
        for(i=0;i<5;i++){
			mdelay(5); 
		    i2c_smbus_read((PLAT_HANDLE) client, NSA_REG_WHO_AM_I, &chip_id);
            if(chip_id == 0x13)
                break;				
		}
		if(i == 5)
	        client->addr = 0x62;
	}	
    	printk("--- %s, %d chip_id = %d ---\n", __func__, __LINE__, chip_id);
	
    msa_handle = msa_core_init(client);
    if(NULL == msa_handle){
        MI_ERR("chip init failed !\n");
        return -1;     
    }
    	printk("--- %s, %d ---\n", __func__, __LINE__);

    /*
    if(withSysAttr)
    {
       struct input_dev* pInputDev;
    	printk("--- %s, %d ---\n", __func__, __LINE__);
       pInputDev = input_allocate_device();
	if (!pInputDev) {
		MI_ERR("Failed to allocate input device %s\n", sensor->input_dev->name);
		return -ENOMEM;	
	}

    	printk("--- %s, %d ---\n", __func__, __LINE__);
	pInputDev->name = MSA_INPUT_DEV_NAME;
    	printk("--- %s, %d ---\n", __func__, __LINE__);
	set_bit(EV_ABS, pInputDev->evbit);	
    	printk("--- %s, %d ---\n", __func__, __LINE__);
		
	ret = input_register_device(pInputDev);
	if (ret) {
		MI_ERR("Unable to register input device %s\n", pInputDev->name);
		return -ENOMEM;	
	}	
    	printk("--- %s, %d ---\n", __func__, __LINE__);
    
        MI_MSG("Sys Attribute Register here %s is called for MSA.\n", __func__);
		
        ret = sysfs_create_group(&pInputDev->dev.kobj, &msa_attr_group);
        if (ret) {
    	    printk("--- %s, %d ---\n", __func__, __LINE__);
            MI_ERR("msa_attr_group create Error err=%d..", ret);
            ret = -EINVAL;
        }
		
    	printk("--- %s, %d ---\n", __func__, __LINE__);
        withSysAttr = 0;
    }
	*/

    	printk("--- %s, %d ---\n", __func__, __LINE__);
     is_init =1;	
     
    	printk("--- %s, %d ---\n", __func__, __LINE__);
      return ret;
}
/******************************************************************************/
static int sensor_active(struct i2c_client *client, int enable, int rate)
{
    int result = 0;
	
    MI_MSG("%s. enable=%d.\n", __func__,enable);

    printk("--- sensor_active ---\n"); 
    if(!is_init)     
	return -1;			
	
    mdelay(10);
    if(enable){   
 /*      result = msa_chip_resume(client);	
	if(result) {	
		MI_ERR("sensor_active chip resume fail!!\n");		
		return result;	
	}
*/
	result = msa_set_enable(client, true);        
	if(result){       
		MI_ERR("sensor_active enable  fail!!\n");	
		return result;        
	}	
    }
    else{
	result = msa_set_enable(client, false);        
	if(result){       
		MI_ERR("sensor_active disable  fail!!\n");	
		return result;        
	}
    }
    mdelay(10);	

    return result;
}
/******************************************************************************/
static int sensor_report_value(struct i2c_client *client)
{
    struct sensor_private_data *sensor = (struct sensor_private_data *) i2c_get_clientdata(client);  
    struct sensor_platform_data *pdata = sensor->pdata;
    struct sensor_axis axis;  
    int ret = 0;
    short  x=0,y=0,z=0;
    int  tmp_x=0,tmp_y=0,tmp_z=0;	
	

    if(!is_init)      
	return -1;	
	
     ret = msa_read_data (client,&x, &y, &z);
     if (ret){
         MI_ERR("read data failed!");
         return ret;
     }   

     MI_DATA(" x = %d, y = %d, z = %d\n", x, y, z);
	
     tmp_x = x*MSA_GRAVITY_STEP; 
     tmp_y = y*MSA_GRAVITY_STEP; 
     tmp_z = z*MSA_GRAVITY_STEP;

     MI_DATA(" tmp_x = %d, tmp_y = %d, tmp_z = %d\n", tmp_x, tmp_y, tmp_z);	 
     
     

     axis.x = (pdata->orientation[0])*tmp_x + (pdata->orientation[1])*tmp_y + (pdata->orientation[2])*tmp_z;
     axis.y = (pdata->orientation[3])*tmp_x + (pdata->orientation[4])*tmp_y + (pdata->orientation[5])*tmp_z;	
#if MSA_STK_TEMP_SOLUTION     
     axis.z = (pdata->orientation[6])*tmp_x + (pdata->orientation[7])*tmp_y + (bzstk?1:(pdata->orientation[8]))*tmp_z;
#else
     axis.z = (pdata->orientation[6])*tmp_x + (pdata->orientation[7])*tmp_y + (pdata->orientation[8])*tmp_z;
#endif
     MI_DATA( "map: axis = %d  %d  %d \n", axis.x, axis.y, axis.z);

    if((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) || (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) || (abs(sensor->axis.z - axis.z) > GSENSOR_MIN))
    {
	    //input_report_abs(sensor->input_dev, ABS_X, -1*axis.y);
	    //input_report_abs(sensor->input_dev, ABS_Y, axis.x);
	    //input_report_abs(sensor->input_dev, ABS_Z, axis.z);
	    
        input_report_abs(sensor->input_dev, ABS_X, axis.x);
	    input_report_abs(sensor->input_dev, ABS_Y, -1*axis.y);
	    input_report_abs(sensor->input_dev, ABS_Z, -1*axis.z);
	    input_sync(sensor->input_dev);		
 
           mutex_lock(&(sensor->data_mutex) );
           sensor->axis = axis;
           mutex_unlock(&(sensor->data_mutex) );
    }
    
    return ret;
}
/******************************************************************************/
static int sensor_suspend(struct i2c_client *client)
{
    int result = 0;

    MI_FUN;

    mdelay(10);	
	
    result = msa_set_enable(client, false);        
    if(result){       
	MI_ERR("sensor_suspend disable  fail!!\n");	
	return result;        
    }
	
	mdelay(10);	
	
    return result;   
}

/******************************************************************************/
static int sensor_resume(struct i2c_client *client)
{
    int result = 0;

    MI_FUN;	

    mdelay(10);	 

    /*
    result = msa_chip_resume(client);	
    if(result) {	
		MI_ERR("sensor_resume chip resume fail!!\n");		
		return result;	
    }
    */
    result = msa_set_enable(client, true);        
    if(result){       
		MI_ERR("sensor_resume enable  fail!!\n");	
		return result;        
    }	
	
	 mdelay(10);	
	
    return result;        
}

/******************************************************************************/
struct sensor_operate gsensor_msa_ops = {
    .name           = MSA_DRV_NAME,
    .type           = SENSOR_TYPE_ACCEL,
    .id_i2c         = ACCEL_ID_MSA,
    .read_reg       =-1,
    .read_len       = 0,
    .id_reg         =  -1,
    .id_data            = 0, 
    .precision          = MSA_PRECISION,
    .ctrl_reg           = -1,
    .int_status_reg     = 0x00,
    .range          = {-32768,32768},
    .trig           = IRQF_TRIGGER_LOW|IRQF_ONESHOT,     
    .active         = sensor_active,    
    .init           = sensor_init,
    .report         = sensor_report_value,
    .suspend  =sensor_suspend,
    .resume   =sensor_resume,    
};
/******************************************************************************/
//static struct sensor_operate *gsensor_get_ops(void)
//{
//    return &gsensor_ops;
//}
/******************************************************************************/
static const struct i2c_device_id gsensor_msa_id[] = {
	{"gs_msa", ACCEL_ID_MSA},
	{}
};


static int gsensor_msa_probe(struct i2c_client *client,
				const struct i2c_device_id *devid)
{
	return sensor_register_device(client, NULL, devid, &gsensor_msa_ops);
}
/******************************************************************************/
static int gsensor_msa_remove(struct i2c_client *client)
{
	return sensor_unregister_device(client, NULL, &gsensor_msa_ops);
}
/******************************************************************************/
static struct i2c_driver gsensor_msa_driver = {
	.probe = gsensor_msa_probe,
	.remove = gsensor_msa_remove,
	.shutdown = sensor_shutdown,
	.id_table = gsensor_msa_id,
	.driver = {
		.name = "gsensor_msa",
	#ifdef CONFIG_PM
		.pm = &sensor_pm_ops,
	#endif
	},
};
module_i2c_driver(gsensor_msa_driver);
MODULE_AUTHOR("MEMSING <lctang@memsing.com>");
MODULE_DESCRIPTION("MEMSING 3-Axis Accelerometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

