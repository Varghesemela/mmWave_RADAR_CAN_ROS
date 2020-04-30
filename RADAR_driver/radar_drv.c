#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/kdev_t.h>

#include <linux/time.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include "radar_def.h"


#define mem_size	1024
#define Num_of_radars 10

#define RADAR_ID 	'r'
#define radar0		'0'
#define	radar1		'1'
#define	radar2		'2'
#define read_cmd 	'a'
#define write_cmd	'b'
#define data_cmd	'c'

#define RADAR_RD _IOR(RADAR_ID, read_cmd, int32_t*)
#define RADAR_WR _IOW(RADAR_ID, write_cmd, int32_t*)
#define RADAR_DAT _IOR(RADAR_ID, data_cmd, radar_frame_t *)
#define RADAR_CMD _IOW(RADAR_ID, data_cmd, radar_frame_t *)

int32_t val = 0, i = 0;


dev_t radar = 0;
static struct class *radar_class;
static struct cdev radar_cdev[Num_of_radars];
uint8_t *kernel_buffer;

static int __init radar_driver_init(void);
static void __exit radar_driver_exit(void);
static int  radar_open(struct inode *inode, struct file *file);
static int  radar_release(struct inode *inode, struct file *file);
static ssize_t  radar_read(struct file *pfile, char __user *buff, size_t len, loff_t *offset);
static ssize_t  radar_write(struct file *pfile, const char __user *buff, size_t len, loff_t *offset);
static long radar_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg);

static struct task_struct *etx_thread;

unsigned int minor_num;

int thread_function(void *pv);
 
int thread_function(void *pv)
{
    int i=0;
    while(!kthread_should_stop()) {
        printk(KERN_INFO "In Radar thread function %d\n", i++);
        msleep(1000);
    }
    return 0;
}

static struct  file_operations fops = {
	.owner = THIS_MODULE,
	.open = radar_open,
	.release = radar_release,
	.read = radar_read,
	.write = radar_write,
	.unlocked_ioctl = radar_ioctl,

};

static int  radar_open(struct inode *inode, struct file *file){
	if((kernel_buffer = kmalloc(mem_size, GFP_KERNEL))==0){
		printk(KERN_INFO "Cannot allocate memory in kernel\n");
		return -1;
	}
	minor_num = MINOR(inode->i_rdev);
	printk(KERN_INFO "Radar device file opened\n");
	return 0;

}

static int radar_release(struct inode *inode, struct file *file){
	kfree(kernel_buffer);
	printk(KERN_INFO "Radar device is closed\n");
	return 0;

}

static ssize_t radar_read(struct file *pfile, char __user *buff, size_t len, loff_t *offset){
	copy_to_user(buff, kernel_buffer, mem_size);
	printk(KERN_INFO "%d\n", val);
	printk(KERN_INFO "Data read from radar\n");
	return 0;

}

static ssize_t radar_write(struct file *pfile, const char __user *buff, size_t len, loff_t *offset){
	copy_from_user(kernel_buffer, buff, len);
	printk(KERN_INFO "Data written to radar\n");
	return 0;

}

static long radar_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg){
	struct timespec t;
			
	switch(cmd){
		case RADAR_RD:
			copy_to_user((int32_t*)arg, &val, sizeof(val));
			printk(KERN_INFO "Data read from radar\n");
		break;

		case RADAR_WR:
			copy_from_user(&val, (int32_t*)arg, sizeof(val));
			printk(KERN_INFO "Data written to radar\n");
		break;

		case RADAR_DAT:
			getnstimeofday(&t);
			radar_data.can_id = minor_num;
			radar_data.timeCPUcycles= t.tv_sec;		//seconds; 
			//radar_data.Num_of_Objects = 22;
			copy_to_user((radar_frame_t*)arg, &radar_data, sizeof(radar_data));
			printk(KERN_INFO "Data read from radar\n");
		break;

		case RADAR_CMD:
			getnstimeofday(&t);
			radar_data.timeCPUcycles = t.tv_sec;
			copy_from_user(&radar_data, (radar_frame_t*)arg, sizeof(radar_data));
			printk(KERN_INFO "Data received\n");
		break;

	}
	return 0;

}

int cdev_creation(void){

	/* Creating cdev structure */
	int major = MAJOR(radar);
	dev_t radar_instance = MKDEV(major,i);
	cdev_init(&radar_cdev[i], &fops);
	printk(KERN_INFO "radar no %d", i);
	/*Adding char device to system */
	if((cdev_add(&radar_cdev[i], radar_instance, 1)) < 0){
		printk(KERN_INFO "Error: Device not created\n");
		return -1;
	}
	
	/*Creating device */
	if((device_create(radar_class, NULL, radar_instance, NULL, "radar!radar%d", i))==NULL){
		printk(KERN_INFO "Unable to create the device");
		return -1;
	}
	if(i <= Num_of_radars){
		i++;
	}
	else{
		printk(KERN_INFO "max reached");
		return -1;
	}

	printk(KERN_INFO "Device inserted...\n");
	return 0;

	// r_class:
	// unregister_chrdev_region(radar, 1);
	// return -1;
}

static int __init radar_driver_init(void){

	/* Allocating major numbers */
	if((alloc_chrdev_region(&radar, 0, Num_of_radars, "radar")) < 0){
		printk(KERN_INFO "Cannot allocate major number\n");
		return -1;
	}
	printk(KERN_INFO "Major : %d Minor : %d", MAJOR(radar), MINOR(radar));

	/* Creating struct class */
	radar_class = class_create(THIS_MODULE, "radar_class");

	if(cdev_creation() == -1){
		printk(KERN_INFO "Device creation unsuccessful");
	}
	if(cdev_creation() == -1){
		printk(KERN_INFO "Device creation unsuccessful");
	}
	if(cdev_creation() == -1){
		printk(KERN_INFO "Device creation unsuccessful");
	}
	
	etx_thread = kthread_create(thread_function,NULL,"Thread function");
    if(etx_thread) {
        wake_up_process(etx_thread);
    } else {
        printk(KERN_ERR "Cannot create kthread\n");
        return -1;
    }	
	return 0;

}
int major;
	
static void __exit radar_driver_exit(void){
	kthread_stop(etx_thread);
	major = MAJOR(radar);
	dev_t radar_instance;
	do{
		i--;
		radar_instance = MKDEV(major, i);
		cdev_del(&radar_cdev[i]);
		device_destroy(radar_class, radar_instance);
	}while(i>0);
	class_destroy(radar_class);
	unregister_chrdev_region(radar, Num_of_radars);
	printk(KERN_INFO "Device removed...\n");
}

module_init(radar_driver_init);
module_exit(radar_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sanjay");
MODULE_DESCRIPTION("RADAR device management");
