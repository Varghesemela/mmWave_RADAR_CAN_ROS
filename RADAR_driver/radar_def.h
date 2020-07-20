//#include <stdint.h>
//#include <time.h>

time_t seconds;

#define Radar_obj_max 30

typedef struct DPIF_PointCloudCartesian{
	/*! @brief x - coordinate in meters */
	long x;

	/*! @brief y - coordinate in meters */
	long y;

	/*! @brief z - coordinate in meters */
	long z;

	/*! @brief Doppler velocity estimate in m/s. Positive velocity means target
	* is moving away from the sensor and negative velocity means target
	* is moving towards the sensor. */
	long velocity;

}DPIF_PointCloudCartesian_t;

typedef struct DPIF_PointCloudSideInfo{
    /*! @brief snr - CFAR cell to side noise ratio in dB expressed in 0.1 steps of dB */
    int16_t snr;

    /*! @brief y - CFAR noise level of the side of the detected cell in dB expressed in 0.1 steps of dB */
    int16_t noise;
}DPIF_PointCloudSideInfo_t;

typedef struct radar_frame{
	uint32_t   can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint32_t   timeCPUcycles;
	uint32_t	   Num_of_Objects;
	DPIF_PointCloudCartesian_t    PCD_data[Radar_obj_max];
	DPIF_PointCloudSideInfo_t	  noiseprofile_data[Radar_obj_max];
}radar_frame_t;

// typedef struct radar_command{
// 	uint32_t   can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
// 	uint32_t   timeCPUcycles;
// 	uint32_t	   Num_of_Objects;
// 	DPIF_PointCloudCartesian_t    PCD_data[Radar_obj_max];
// 	DPIF_PointCloudSideInfo_t	  noiseprofile_data[Radar_obj_max];
// }radar_command_t;

radar_frame_t radar_data;
