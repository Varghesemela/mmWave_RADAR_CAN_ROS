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
