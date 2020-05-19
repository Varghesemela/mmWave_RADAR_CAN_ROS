#include "radar_drv.h"
#include "radar_def.h"

#include <linux/can.h>
#include <linux/can/raw.h>

#define AUTHOR "Sanjay Varghese"
#define DESCRIPTION "Radar device management"
#define VERSION "1.0"
#define CLASS_NAME "Radars"

#define MEM_SIZE	1024
#define NUM_OF_RADARS 10

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

int32_t val = 0, i = 0, kmem_count = 0;

dev_t radar = 0;
static struct class *radar_class;
static struct cdev radar_cdev[NUM_OF_RADARS];
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
int major;
dev_t radar_instance;

int thread_function(void *pv);
 
int thread_function(void *pv)
{
	int i=0;
	//int status;
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
	minor_num = MINOR(inode->i_rdev);
	if((kernel_buffer = kmalloc(MEM_SIZE, GFP_KERNEL))==0){
		printk(KERN_INFO "Cannot allocate memory in kernel\n");
		return -1;
	}
	kmem_count++;
	printk(KERN_INFO "Radar device file opened\n");
	return 0;

}

static int radar_release(struct inode *inode, struct file *file){
	if(kmem_count > 0){
		if(kernel_buffer){
			kfree(kernel_buffer);
			kernel_buffer = NULL;
		}

		kmem_count--;
	}
	printk(KERN_INFO "Radar device is closed\n");
	return 0;

}

static ssize_t radar_read(struct file *pfile, char __user *buff, size_t len, loff_t *offset){
	int status;
	status = copy_to_user(buff, kernel_buffer, MEM_SIZE);
	printk(KERN_INFO "%d\n", val);
	printk(KERN_INFO "Data read from radar\n");
	return 0;

}

static ssize_t radar_write(struct file *pfile, const char __user *buff, size_t len, loff_t *offset){
	int status;
	status = copy_from_user(kernel_buffer, buff, len);
	printk(KERN_INFO "Data written to radar\n");
	return 0;

}

static long radar_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg){
	int status;
	struct timespec t;
			
	switch(cmd){
		case RADAR_RD:
			status = copy_to_user((int32_t*)arg, &val, sizeof(val));
			printk(KERN_INFO "Data read from radar\n");
		break;

		case RADAR_WR:
			status = copy_from_user(&val, (int32_t*)arg, sizeof(val));
			printk(KERN_INFO "Data written to radar\n");
		break;

		case RADAR_DAT:
			getnstimeofday(&t);
			//radar_data.can_id = minor_num;
			//radar_data.timeCPUcycles= t.tv_sec;		//seconds; 
			//radar_data.Num_of_Objects = 22;
			status = copy_to_user((radar_frame_t*)arg, &radar_data, sizeof(radar_data));
			printk(KERN_INFO "Data read from radar\n");
		break;

		case RADAR_CMD:
			getnstimeofday(&t);
			radar_data.timeCPUcycles = t.tv_sec;
			status = copy_from_user(&radar_data, (radar_frame_t*)arg, sizeof(radar_data));
			printk(KERN_INFO "Data received\n");
		break;

	}
	return 0;

}

int cdev_creation(void){

	/* Creating cdev structure */
	major = MAJOR(radar);
	radar_instance = MKDEV(major,i);
	cdev_init(&radar_cdev[i], &fops);
	printk(KERN_INFO "radar no %d", i);
	/*Adding char device to system */
	if((cdev_add(&radar_cdev[i], radar_instance, 1)) < 0){
		printk(KERN_INFO "Error: Device not created\n");
		return -1;
	}
	
	/*Creating device */
	if((device_create(radar_class, NULL, radar_instance, NULL, "Radars!radar%d", i))==NULL){
		printk(KERN_INFO "Unable to create the device");
		return -1;
	}
	if(i <= NUM_OF_RADARS){
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
	radar_data.Num_of_Objects = 22;
	/* Allocating major numbers */
	if((alloc_chrdev_region(&radar, 0, NUM_OF_RADARS, "radar")) < 0){
		printk(KERN_INFO "Cannot allocate major number\n");
		return -1;
	}
	printk(KERN_INFO "Major : %d Minor : %d", MAJOR(radar), MINOR(radar));

	/* Creating struct class */
	radar_class = class_create(THIS_MODULE, "Radars");

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
		
static void __exit radar_driver_exit(void){
	kthread_stop(etx_thread);
	major = MAJOR(radar);
	do{
		i--;
		radar_instance = MKDEV(major, i);
		cdev_del(&radar_cdev[i]);
		device_destroy(radar_class, radar_instance);
	}while(i>0);
	class_destroy(radar_class);
	unregister_chrdev_region(radar, NUM_OF_RADARS);
	printk(KERN_INFO "Device removed...\n");
}

module_init(radar_driver_init);
module_exit(radar_driver_exit);


MODULE_AUTHOR(AUTHOR);
MODULE_VERSION(VERSION);
MODULE_DESCRIPTION(DESCRIPTION);
MODULE_LICENSE("GPL");
