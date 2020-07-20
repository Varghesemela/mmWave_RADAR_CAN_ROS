#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

//#include <linux/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>


#define total_radars 2+1	//+1 to make 1 indexed
#define Radar1 1
#define Radar2 2
#define Header_frame 1
#define PCD_frame 2
#define noiseprofile_frame 3
#define ENABLED 1
#define DISABLED 0
#define Radar_obj_max 30



uint32_t Radar_no = 0, Message_type = 0;
uint8_t countObj = {0};
int exception_flag = 0;
struct canfd_frame canframe_read, canframe_sort;


pthread_mutex_t read_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t swap_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t sort_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_cond_t read_cv = PTHREAD_COND_INITIALIZER;
pthread_cond_t swap_cv = PTHREAD_COND_INITIALIZER;
pthread_cond_t sort_cv = PTHREAD_COND_INITIALIZER;
