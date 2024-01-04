#include <iostream>
typedef enum
{
    AD_Off=0,
    AD_Ready=1,
    AD_Run=2,
    AD_SafeRun=3,
}AD_mode;

typedef struct
{
	bool UltrasionicRadarSts;
	bool EoneSts;
	bool LidarECUSts;
	bool ForwardCameraSts;
	bool BMS_Sts;
	bool HCCB_Sts;
	bool MCU_Sts;
	bool TPMS_Sts;
	bool EBS_Sts;
	bool EPS_Sts;
	bool BCM_Sts;
	bool ADCU_POWER_Status;
	bool FishCameraSts;
}System_sts;
