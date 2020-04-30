#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <time.h>

#include "DataHdl.h"

#include "radar_def.h"

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
#define RADAR_CMD _IOW(RADAR_ID, data_cmd, radar_command_t *)

void* readCANData(void* arg);

int main(){
	int fd1, fd2, fd3;
	int32_t val, num;
	printf("DEADBEEF\n");
	fd1 = open("/dev/radar/radar2", O_RDWR);
	
	if(fd1 < 0){
		printf("Cannot open file");
		return -1;
	}

	//radar_data.can_id = 2;
	//radar_data.timeCPUcycles= t.tv_sec;		//seconds; 
	// radar_data.Num_of_Objects = 22;
	// radar_data.PCD_data[0].x = 32;
	// radar_data.PCD_data[0].y = 33;
	// radar_data.PCD_data[0].z = 34;
	// if(ioctl(fd1, RADAR_CMD, (radar_frame_t*)&radar_data) == -1){
 //       	printf("issue in sending data\n");
 //    }
 //    else{
 //    	printf("data sent: %f\n", radar_data.PCD_data[0].x );
 //    }

	if(ioctl(fd1, RADAR_DAT, (radar_frame_t*)&radar_data) == -1){
        	printf("issue in getting data\n");
    	}
	else{
		printf("can_id : %d\n", radar_data.can_id);
		printf("can_time: %d\n", radar_data.timeCPUcycles);
		printf("can_numobj: %d\n", radar_data.Num_of_Objects);
		printf("pcddata: %f %f %f\n", radar_data.PCD_data[0].x, radar_data.PCD_data[0].y, radar_data.PCD_data[0].z);	
	}

	close(fd1);

	fd2 = open("/dev/radar/radar1", O_RDWR);
	
	if(fd2 < 0){
		printf("Cannot open file");
		return -1;
	}


	close(fd2);
			

}


void* readCANData(void* arg){

		int cansock_fd, i; 
        int enable_canfd = 1;                       
	    int nbytes;
        struct sockaddr_can sock_addr;		
	    struct ifreq ifr;
	    sock_addr.can_family = AF_CAN;
	    sock_addr.can_ifindex = ifr.ifr_ifindex;
	
		if ((cansock_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0){
		    perror("error: socket init");	
	    }
        if(setsockopt(cansock_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd))<0){
            perror("error: switching to canfd");             
        }
	    strcpy(ifr.ifr_name, "can1" );
	    ioctl(cansock_fd, SIOCGIFINDEX, &ifr);
	    memset(&sock_addr, 0, sizeof(sock_addr));
	    
	    if (bind(cansock_fd, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) < 0) {
		    perror("error: Bind");		     
	    }    	
        nbytes = read(cansock_fd, &canframe_read, sizeof(struct canfd_frame));
        if (nbytes < 0) {
	        perror("error: Read");	         
        } 

    	if (close(cansock_fd) < 0) {
		    perror("error: Close");		     
	    }

}
