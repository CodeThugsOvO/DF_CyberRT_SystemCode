#ifndef _LOCATIONIMU_H
#define _LOCATIONIMU_H
#include <sys/types.h>



typedef struct
{
	u_int8_t Error_Code;
	u_int32_t SampleTime;
	u_int16_t GroupCounter;
	
    //StatusWord
    uint8_t have_gnss_time_pulse;
    uint8_t filter_mode;
    uint8_t clip_mag_z;
    uint8_t retransmitted;
    uint8_t clipping_detected;
    uint8_t interpolated;
    uint8_t sync_in;
    uint8_t sync_out;
    uint8_t clip_acc_x;
    uint8_t clip_acc_y;
    uint8_t clip_acc_z;
    uint8_t clip_gyr_x;
    uint8_t clip_gyr_y;
    uint8_t clip_gyr_z;
    uint8_t clip_mag_x;
    uint8_t clip_mag_y;
    uint8_t self_test_ok;
    uint8_t orientation_valid;
    uint8_t gps_valid;
    uint8_t no_rotation;
    uint8_t representative_motion;
    uint8_t external_clock_synced;

    double Q0;
    double Q1;
    double Q2;
    double Q3;

    double Roll;
    double Pitch;
    double Yaw;

    double DeltaVx;
    double DeltaVy;
    double DeltaVz;
    u_int8_t Exponent;

    double gyrX;
    double gyrY;
    double gyrZ;

    double DeltaQ0;
    double DeltaQ1;
    double DeltaQ2;
    double DeltaQ3;

    double accX;
    double accY;
    double accZ;

    double freeAccX;
    double freeAccY;
    double freeAccZ;
}LocationIMU_mp;
#endif
