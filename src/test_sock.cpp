#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

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
#define Radar_obj_max 20


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

typedef struct DPIF_PointCloudSideInfo_t
{
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



struct canfd_frame canframe;
struct radar_frame radar_data[total_radars];
uint32_t Radar_no = 0, Message_type = 0;
int countObj = 0;

int readCANData(void);
int sortandpublishData();
int flushpublishedData();

int main(void)
{
	readCANData();
	sortandpublishData();
	flushpublishedData();


}


int readCANData(void){

	int cansock_fd, i; 
    int enable_canfd = 1;
	int nbytes;
    struct sockaddr_can sock_addr;
	struct ifreq ifr;
	
	if ((cansock_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("error: socket init");
		return 1;	
	}
    if(setsockopt(cansock_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd))<0){
        perror("error: switching to canfd");
        return 1;
    }

	strcpy(ifr.ifr_name, "can1" );
	ioctl(cansock_fd, SIOCGIFINDEX, &ifr);

	memset(&sock_addr, 0, sizeof(sock_addr));
	sock_addr.can_family = AF_CAN;
	sock_addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(cansock_fd, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) < 0) {
		perror("error: Bind");
		return 1;
	}
    while(1){
	    nbytes = read(cansock_fd, &canframe, sizeof(struct canfd_frame));
        if (nbytes < 0) {
		    perror("error: Read");
		    return 1;
	    }

    }
	if (close(cansock_fd) < 0) {
		perror("error: Close");
		return 1;
	}

	return 0;
}


int sortandpublishData(void){
	uint32_t Radar_no = canframe.can_id >> 4;
	uint32_t Message_type = canframe.can_id & 0x0F;
	int currentframes = 0, currentp = 0;
	
	switch(Message_type){

		case Header_frame:
			memcpy(&radar_data[Radar_no].Num_of_Objects, &canframe.data[28], sizeof(uint32_t));
			printf("Radar no. %d, Num_of_Objects : %d\n",Radar_no, radar_data[Radar_no].Num_of_Objects);
			break;

		case PCD_frame:
			if((radar_data[Radar_no].Num_of_Objects-countObj)<3){
				currentframes = 4;
			}
			else{
				currentframes = radar_data[Radar_no].Num_of_Objects-countObj;
			}

			for(int i = 0; i < currentframes; i++){
				memcpy(&radar_data[Radar_no].PCD_data[countObj].x, &canframe.data[currentp], sizeof(float));
				currentp+=(sizeof(float));
				memcpy(&radar_data[Radar_no].PCD_data[countObj].y, &canframe.data[currentp], sizeof(float));
				currentp+=(sizeof(float));
				memcpy(&radar_data[Radar_no].PCD_data[countObj].z, &canframe.data[currentp], sizeof(float));
				currentp+=(sizeof(float));
				memcpy(&radar_data[Radar_no].PCD_data[countObj].velocity, &canframe.data[currentp], sizeof(float));
				currentp+=(sizeof(float));
				countObj++;
				printf("%f %f %f %f\n", radar_data[Radar_no].PCD_data[countObj].x, radar_data[Radar_no].PCD_data[countObj].y,
						 radar_data[Radar_no].PCD_data[countObj].z, radar_data[Radar_no].PCD_data[countObj].velocity);
			}

			break;

		case noiseprofile_frame:

			//write code here to copy noise profile data if needed
			countObj = 0;

			break;
		default:
			printf("garbage data\n");
			break;
	}

}

int flushpublishedData(void){

	memset(radar_data, 0, sizeof(radar_data));

}