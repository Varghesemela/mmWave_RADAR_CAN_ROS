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
#include <signal.h>

#include "DataHdl.h"
#define Header_frame 1
#define PCD_frame 2
#define noiseprofile_frame 3

#include "radar_def.h"

#define RADAR_ID 	'r'
//#define radar0		'0'
#define	radar1		'1'
#define	radar2		'2'
#define read_cmd 	'a'
#define write_cmd	'b'
#define data_cmd	'c'
#define radar_new 	'd'

#define RADAR_RD _IOR(RADAR_ID, read_cmd, int32_t*)
#define RADAR_WR _IOW(RADAR_ID, write_cmd, int32_t*)
#define RADAR_DAT _IOR(RADAR_ID, data_cmd, radar_frame_t *)
#define RADAR_CMD _IOW(RADAR_ID, data_cmd, radar_frame_t *)
#define RADAR_NEW _IOW(RADAR_ID, radar_new, char *)

void readCANData(void);
int putdata(void);
int fd1, fd2, fd3;
	
int status = 1;

void sig_int_handler (int sig_num )
{
	status = 0;
    printf("Exception %d \n", sig_num);
}


int main(void){

		{
		struct sigaction action = { };
	    action.sa_handler = sig_int_handler;

	    sigaction (SIGHUP, &action, NULL);	// controlling terminal closed, Ctrl-D
	    sigaction (SIGINT, &action, NULL);	// Ctrl-C
	    sigaction (SIGQUIT, &action, NULL);	// Ctrl-\, clean quit with core dump
	    sigaction (SIGABRT, &action, NULL);	// abort() called.
	    sigaction (SIGTERM, &action, NULL);	// kill command
		}

		fd2 = open("/dev/radar/radar1", O_RDWR);
		if(fd2 < 0){
			printf("Cannot open file");
			return -1;
		}
		
		if(ioctl(fd2, RADAR_DAT, (radar_frame_t*)&radar_data) == -1){
	        	printf("issue in getting data\n");
	    	}
		else{
			printf("can_id : %d\n", radar_data.can_id);
			printf("can_time: %d\n", radar_data.timeCPUcycles);
			printf("can_numobj: %d\n", radar_data.Num_of_Objects);
			printf("pcddata: %ld %ld %ld\n", radar_data.PCD_data[0].x, radar_data.PCD_data[0].y, radar_data.PCD_data[0].z);	
		}
		close(fd2);
				
		readCANData();
}


void readCANData(void){

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
	    strcpy(ifr.ifr_name, "can1");
	    ioctl(cansock_fd, SIOCGIFINDEX, &ifr);
	    memset(&sock_addr, 0, sizeof(sock_addr));
	    
	    if (bind(cansock_fd, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) < 0) {
		    perror("error: Bind");		     
	    }   
	    while(status){ 	
	        nbytes = read(cansock_fd, &canframe_read, sizeof(struct canfd_frame));
	        if (nbytes < 0) {
		        perror("error: Read");	         
	        } 
	        putdata();
	    }

    	if (close(cansock_fd) < 0) {
		    perror("error: Close");		     
	    }

}



int putdata(void){

	int32_t val, num;
	int currentp = 0;
	for(int i=0; i<strlen(canframe_read.data); i++){
		printf("%02X", canframe_read.data[i]);
	}
	printf("\n");

	float temp_data;
	// radar_data.can_id = canframe_read.can_id >> 4;
	uint32_t Message_type = canframe_read.can_id & 0x0F;
	switch(Message_type){
				case Header_frame:
					memcpy(&radar_data.Num_of_Objects, &canframe_read.data[28], sizeof(uint32_t));
					printf("Radar no. %d, Num_of_Objects : %d\n",Radar_no, radar_data.Num_of_Objects);
					
					break;

				case PCD_frame:
					if((radar_data.Num_of_Objects-countObj[Radar_no])>3){
						currentframes = 4;
					}
					else{
						currentframes = radar_data.Num_of_Objects-countObj[Radar_no];
					}

					for(int i = 0; i < currentframes; i++){

						memcpy(&temp_data, &canframe_read.data[currentp], sizeof(float));
						radar_data.PCD_data[0].x = temp_data * 100000;
						currentp+=(sizeof(float));
						memcpy(&temp_data, &canframe_read.data[currentp], sizeof(float));
						radar_data.PCD_data[0].y = temp_data * 100000;
						currentp+=(sizeof(float));
						memcpy(&temp_data, &canframe_read.data[currentp], sizeof(float));
						radar_data.PCD_data[0].z = temp_data * 100000;
						currentp+=(sizeof(float));
						memcpy(&temp_data, &canframe_read.data[currentp], sizeof(float));
						radar_data.PCD_data[0].velocity = temp_data * 100000;
						currentp+=(sizeof(float));
					}
					break;

				case noiseprofile_frame:

					fd1 = open("/dev/radar/radar1", O_RDWR);
					if(fd1 < 0){
						printf("Cannot open file");
						return -1;
					}

					if(ioctl(fd1, RADAR_CMD, (radar_frame_t*)&radar_data) == -1){
				        	printf("issue in getting data\n");
				    	}
					else{
						printf("can_id : %d\n", radar_data.can_id);
						printf("can_time: %d\n", radar_data.timeCPUcycles);
						printf("can_numobj: %d\n", radar_data.Num_of_Objects);
						printf("pcddata: %ld %ld %ld\n", radar_data.PCD_data[0].x, radar_data.PCD_data[0].y, radar_data.PCD_data[0].z);	
					}
					close(fd1);
			
					break;


}