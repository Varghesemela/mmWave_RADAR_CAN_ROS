#include "DataHdl.h"

int exception_flag;

int main(int argc, char **argv){
    int thret1, thret2, thret3;
    pthread_t readThread, swapThread, sortThread;
    
    ros::init(argc, argv, "Radar");
    ros::NodeHandle n;
    radar_pub[1] = n.advertise<radar_ti::RadarPoints>("radar1/data", 100);
    radar_pub[2] = n.advertise<radar_ti::RadarPoints>("radar2/data", 100);
    ros::Rate loop_rate(1);

	/*Simple loop without multi-threading and mutex-sync*/
    // while(ros::ok()){   	
    //    readCANData(NULL);
    // 	sortandpublishData(NULL);
    //    ros::spinOnce();  
    // }
	// return 0;
	
	pthread_mutex_init(&read_mutex, NULL);
	pthread_mutex_init(&swap_mutex, NULL);
	pthread_mutex_init(&sort_mutex, NULL);
	pthread_cond_init(&read_cv, NULL);
	pthread_cond_init(&swap_cv, NULL);
	pthread_cond_init(&sort_cv, NULL);

    
	thret1 = pthread_create(&swapThread, NULL, &swapData , NULL);
    if(thret1){
		ROS_INFO("Error creating sortthread code: %d\n",thret2);
		ros::shutdown();
	}
	thret2 = pthread_create(&sortThread, NULL, &sortandpublishData, NULL);
	if(thret2){
		ROS_INFO("Error creating flushthread code: %d\n",thret3);
		ros::shutdown();
	}
	thret3 = pthread_create(&readThread, NULL, &readCANData, NULL);
	if(thret3){
		ROS_INFO("Error creating readthread code: %d\n",thret1);
		ros::shutdown();
	}
	
	//ros::spin();
	
    pthread_join(swapThread, NULL);
    pthread_join(sortThread, NULL);
	pthread_join(readThread, NULL);
    
	pthread_cond_destroy(&read_cv);
	pthread_cond_destroy(&swap_cv);    
	pthread_cond_destroy(&sort_cv);  
	pthread_mutex_destroy(&read_mutex);
    pthread_mutex_destroy(&swap_mutex);
    pthread_mutex_destroy(&sort_mutex);
    ros::shutdown();
    return EXIT_SUCCESS;
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

    while(ros::ok()){
		pthread_mutex_lock(&swap_mutex);   
		pthread_mutex_lock(&read_mutex);   
		canframe_read = canframe_null;
        nbytes = read(cansock_fd, &canframe_read, sizeof(struct canfd_frame));
        if (nbytes < 0) {
	        printf("error: Read");         
        } 
    	// printf("readdata\n");
		pthread_mutex_unlock(&read_mutex);
		pthread_cond_signal(&swap_cv);
		pthread_cond_wait(&read_cv, &swap_mutex);
			
    }

    if (close(cansock_fd) < 0) {
	    perror("error: Close");		     
	}
	
	return NULL;
}

void* swapData(void* arg){
      
	while(ros::ok()){

		pthread_mutex_lock(&swap_mutex);
		pthread_cond_wait(&swap_cv, &swap_mutex);
		pthread_mutex_unlock(&swap_mutex);
		pthread_mutex_lock(&sort_mutex);
		pthread_mutex_lock(&read_mutex);
		struct canfd_frame canframe_temp = canframe_read;
		canframe_read = canframe_sort;
		canframe_sort = canframe_temp;
		// printf("swapdata\n");
		pthread_mutex_unlock(&sort_mutex);
		pthread_mutex_unlock(&read_mutex);
		pthread_cond_signal(&sort_cv);
		pthread_cond_signal(&read_cv);

	}
	return NULL;
}

void* sortandpublishData(void* arg){

    while(ros::ok()){
		pthread_mutex_lock(&swap_mutex);
		pthread_cond_wait(&sort_cv, &swap_mutex);
		pthread_mutex_unlock(&swap_mutex);
		
		pthread_mutex_lock(&sort_mutex);
		
		uint32_t Radar_no = canframe_sort.can_id >> 4;
	    uint32_t Message_type = canframe_sort.can_id & 0x0F;
	    int currentframes = 0, currentp = 0;
		if(Radar_no){	
			switch(Message_type){
				case Header_frame:
					memcpy(&radar_data[Radar_no].Num_of_Objects, &canframe_sort.data[28], sizeof(uint32_t));
					printf("Radar no. %d, Num_of_Objects : %d\n",Radar_no, radar_data[Radar_no].Num_of_Objects);
					
					break;

				case PCD_frame:
					if((radar_data[Radar_no].Num_of_Objects-countObj[Radar_no])>3){
						currentframes = 4;
					}
					else{
						currentframes = radar_data[Radar_no].Num_of_Objects-countObj[Radar_no];
					}

					for(int i = 0; i < currentframes; i++){
						radar_ti::RadarScan temp_radar;
						
						exception_flag = 0;
						memcpy(&radar_data[Radar_no].PCD_data[countObj[Radar_no]].x, &canframe_sort.data[currentp], sizeof(float));
						currentp+=(sizeof(float));
						if(std::isnan(radar_data[Radar_no].PCD_data[countObj[Radar_no]].x))	exception_flag += 1;
						memcpy(&radar_data[Radar_no].PCD_data[countObj[Radar_no]].y, &canframe_sort.data[currentp], sizeof(float));
						currentp+=(sizeof(float));
						if(std::isnan(radar_data[Radar_no].PCD_data[countObj[Radar_no]].y))	exception_flag += 1;
						memcpy(&radar_data[Radar_no].PCD_data[countObj[Radar_no]].z, &canframe_sort.data[currentp], sizeof(float));
						currentp+=(sizeof(float));
						if(std::isnan(radar_data[Radar_no].PCD_data[countObj[Radar_no]].z))	exception_flag += 1;
						memcpy(&radar_data[Radar_no].PCD_data[countObj[Radar_no]].velocity, &canframe_sort.data[currentp], sizeof(float));
						currentp+=(sizeof(float));
						if(std::isnan(radar_data[Radar_no].PCD_data[countObj[Radar_no]].velocity))	exception_flag += 1;
						
						try{
							if(exception_flag){
								throw exception_flag;
							}
							temp_radar.point_id = countObj[Radar_no];
							temp_radar.x = radar_data[Radar_no].PCD_data[countObj[Radar_no]].y;
							temp_radar.y = -radar_data[Radar_no].PCD_data[countObj[Radar_no]].x;
							temp_radar.z = radar_data[Radar_no].PCD_data[countObj[Radar_no]].z;            
							temp_radar.velocity = radar_data[Radar_no].PCD_data[countObj[Radar_no]].velocity;

						}
						catch(int exception_flag){
							temp_radar.point_id = countObj[Radar_no];
							temp_radar.x = 0;
							temp_radar.y = 0;
							temp_radar.z = 0;      
							temp_radar.velocity = 0;

							printf("Got a NaN after object %d, data is %d\n", countObj[Radar_no], exception_flag);
						}
						radar[Radar_no].radarscan.push_back(temp_radar);
						printf("%f %f %f %f\n", temp_radar.x, temp_radar.y, temp_radar.z, temp_radar.velocity);
						countObj[Radar_no]++;
					}
					currentframes = 0;

					//radar_pub[Radar_no].publish(radar[Radar_no]);
					break;

				case noiseprofile_frame:
					//write code here to copy noise profile data if needed
					countObj[Radar_no] = 0;

					break;
				default:
					
					printf("garbage data\n");
					break;
			}
			if(radar_data[Radar_no].Num_of_Objects == countObj[Radar_no]){
			
				radar_pub[Radar_no].publish(radar[Radar_no]);
				radar[Radar_no].radarscan.clear();		
				countObj[Radar_no] = 0;
				memset(&radar_data, 0, sizeof(radar_data));
				//pthread_cond_signal(&read_cv);
			}
			canframe_sort = canframe_null;	
		}
					
		pthread_mutex_unlock(&sort_mutex);
    }
	return NULL;
}
