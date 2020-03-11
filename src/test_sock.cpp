#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#define total_radars 2
#define Radar1 1
#define Radar2 2
#define Header_frame 1
#define PCD_frame 2
#define noiseprofile_frame 3

struct can_frame canframe, radar_data[total_radars(+1)][3];
uint32_t Radar_no = 0, Message_type = 0;
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
	switch(message_type){

		case Header_frame:


			break;

		case PCD_frame:
			radar_data[Radar_no][PCD_frame]= canframe.data;
			break;

		case noiseprofile_frame:

			break;
	}
}

*void DataCANHandler::flushpublishedData(void){



}