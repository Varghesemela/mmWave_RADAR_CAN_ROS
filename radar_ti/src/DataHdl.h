#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>


#include <ros/ros.h>
#include "radar_ti/RadarPoints.h"
#include "radar_ti/RadarScan.h"


#define total_radars 2+1	//+1 to make 1 indexed
#define Radar1 1
#define Radar2 2
#define Header_frame 1
#define PCD_frame 2
#define noiseprofile_frame 3
#define ENABLED 1
#define DISABLED 0
#define Radar_obj_max 30


typedef struct DPIF_PointCloudCartesian_t{
	/*! @brief x - coordinate in meters */
	float x;

	/*! @brief y - coordinate in meters */
	float y;

	/*! @brief z - coordinate in meters */
	float z;

	/*! @brief Doppler velocity estimate in m/s. Positive velocity means target
	* is moving away from the sensor and negative velocity means target
	* is moving towards the sensor. */
	float velocity;

}DPIF_PointCloudCartesian;

typedef struct DPIF_PointCloudSideInfo_t{
    /*! @brief snr - CFAR cell to side noise ratio in dB expressed in 0.1 steps of dB */
    int16_t snr;

    /*! @brief y - CFAR noise level of the side of the detected cell in dB expressed in 0.1 steps of dB */
    int16_t noise;
}DPIF_PointCloudSideInfo;

struct radar_frame {
	uint32_t   can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint32_t   timeCPUcycles;
	uint32_t	   Num_of_Objects;
	DPIF_PointCloudCartesian_t    PCD_data[Radar_obj_max];
	DPIF_PointCloudSideInfo_t	  noiseprofile_data[Radar_obj_max];
};

struct canfd_frame canframe_read, canframe_sort, canframe_null = {};
struct radar_frame radar_data[total_radars];
uint32_t Radar_no = 0, Message_type = 0;
uint8_t countObj[total_radars] = {0};

pthread_mutex_t read_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t swap_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t sort_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_cond_t read_cv = PTHREAD_COND_INITIALIZER;
pthread_cond_t swap_cv = PTHREAD_COND_INITIALIZER;
pthread_cond_t sort_cv = PTHREAD_COND_INITIALIZER;

void* readCANData(void* arg);
void* sortandpublishData(void* arg);
void* swapData(void* arg);    
ros::Publisher radar_pub[total_radars];
radar_ti::RadarPoints radar[total_radars];



