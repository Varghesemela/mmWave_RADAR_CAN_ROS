#include <stdio.h>
#include <stdlib.h>
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



struct can_frame canframe;
struct radar_frame radar_data[total_radars];
uint32_t Radar_no = 0, Message_type = 0;
int SoM[total_radars] = {0}, EoM[total_radars] = {0};

int main(void)
{



}


*void DataCANHandler::readIncomingData(void){

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
	    nbytes = read(cansock_fd, &canframe, sizeof(struct can_frame));
        if (nbytes < 0) {
		    perror("error: Read");
		    return 1;
	    }

     //    printf("CAN ID : %X ", canframe.can_id);
     //    printf("\t");
	    // for (i = 0; i < canframe.can_dlc; i++){
		   //  printf("%02X ",canframe.data[i]);
     //    }
	    // printf("\r\n");
    }
	if (close(cansock_fd) < 0) {
		perror("error: Close");
		return 1;
	}

	return 0;
}


*void DataCANHandler::sortandpublishData(void){
	Radar_no = canframe.can_id >> 4;
	Message_type = canframe.can_id & 0x0F;
	switch(Message_type){

		case Header_frame:

			memcpy(&radar_data[Radar_no].Num_of_Objects, &canframe.data->at(28), sizeof(radar_data[Radar_no][Message_type].Num_of_Objects));
			
			break;

		case PCD_frame:
			
			//if(radar_data[Radar_no][PCD_frame].Num_of_Objects > )
				memcpy(radar_data[Radar_no].PCD_data.x, canframe.data->at(0), sizeof(radar_data[Radar_no].PCD_data.x));
				memcpy(radar_data[Radar_no].PCD_data.y, canframe.data->at(0+4), sizeof(radar_data[Radar_no].PCD_data.y));
				memcpy(radar_data[Radar_no].PCD_data.z, canframe.data->at(), sizeof(radar_data[Radar_no].PCD_data.z));
				memcpy(radar_data[Radar_no].PCD_data.velocity, canframe.data->at(), sizeof(radar_data[Radar_no].PCD_data.velocity));
			
			break;

		case noiseprofile_frame:

			//write code here to copy noise profile data if needed
			EoM[Radar_no] = ENABLED;

			break;
	}
	for(int i = 1; i <= total_radars; i++){
		if(EoM[i] == ENABLED){
			//publish radar data of "i" radar
			EoM[i] == DISABLED;
		}
	}

}

*void DataCANHandler::flushpublishedData(void){

}