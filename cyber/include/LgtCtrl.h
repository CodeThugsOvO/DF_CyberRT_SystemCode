#ifndef _LGTCTRL_H
#define _LGTCTRL_H
#include <sys/types.h>
/*author: Yuanmu*/
/*date:2021-7-1*/

//#include "Types.h" //需包含数据类型定义头文件
//zxp 0914	注释
// u_int64_t ReverseBits(u_int64_t value, int valueLength)
// {
//     u_int64_t output = 0;
//     for (int i = valueLength - 1; i >= 0; i--)
//     {
//         output |= (value & 1) << i;
//         value >>= 1;
//     }
//     return output;
// }

// typedef struct
// {
// 	int16_t VehDa_prcActuTrq_mp; //EEC1：实际扭矩百分比
// 	int16_t VehDa_prcDrvrDmdTrq_mp; //EEC1：驾驶员需求扭矩百分比
// 	u_int8_t VehDa_stSrcEngCtrl_mp; //EEC1：发动机控制设备源地址
// 	double VehDa_nEngSpd_mp; //EEC1：发动机转速
// 	int16_t VehDa_prcTrqEngNomFric_mp; //EEC2：发动机名义摩擦扭矩百分比
// 	int16_t VehDa_prcTrqEstimdLoss_mp; //EEC2：发动机预估附件损失扭矩百分比
// 	double VehDa_trqEngRef_mp;// EC1:发动机参考扭矩
// 	double VehDa_jEng_mp;//EC1：发动机转动惯量
// 	u_int8_t VehDa_stCluSwt_mp; //CCVS：离合器状态 源地址：00
// 	double VehDa_rAccrPedl_mp;//EEC2:Accelerator pedal (AP) position 2（2021.7.14更新，新增信号）
// } EECU_To_Ctrl;

// typedef struct
// {
// 	//u_int8_t VehDa_stCluSwt_mp; //CCVS：离合器状态

// } VECU_To_Ctrl;

// typedef struct
// {
// 	double VehDa_rTraCurGear_mp; //ETC2：变速箱当前档位传动比
// 	int16_t VehDa_stTraCurGear_mp; //ETC2：变速箱当前档位
// 	int16_t VehDa_stTraSelGear_mp; //ETC2：选择变速箱档位（2021.7.14更新，新增信号）
// 	u_int8_t VehDa_stTraEgd_mp; //ETC1:传动系统结合状态
// 	u_int8_t VehDa_stTraSht_mp; //ECT1:换挡状态
// 	u_int8_t VehDa_stTraRdy4BrkRls_mp; //ETC7：制动允许释放状态
// 	u_int8_t VehDa_stTraTrqLim_mp; //TSC1_TE：Engine Override Control Mode（2021.7.14更新，新增信号）
// 	int16_t VehDa_prcTraTrqLim_mp; //TSC1_TE：Engine Requested Torque/Torque Limit（2021.7.14更新，新增信号）
// } TCU_To_Ctrl;

// typedef struct
// {
// 	double VehDa_rBrkPedl_mp; //EBC1：制动踏板位置百分比
// 	u_int8_t VehDa_stBrkPedl_mp; //EBC1：制动踏板状态
// 	u_int8_t VehDa_stSrcBrk_mp; //EBC1:制动控制设备源地址VehDa_stSrcBrk_mp
// 	double VehDa_vEgoSpd_mp; //EBC2：前轴车速
// 	double VehDa_mWght_mp; //CVW：列车总质量
// 	double VehDa_aEgoLgt_mp; //VDC2：本车纵向加速度（2021.7.14备注，赋值接口VehDa_aEgoAcc_mp）
//     double VehDa_aEgoLat_mp; //VDC2: 本车横向加速度
//     double VehDa_yawRateEgo_mp; //VDC2:自车横摆角速度
// 	double VehDa_pFrntLe_mp; //EBC3:前轴左侧制动压力（2021.7.14备注，赋值接口VehDa_pFrontLeft_mp）
// 	double VehDa_pFrntRi_mp; //EBC3:前轴右侧制动压力
// 	u_int8_t VehDa_stXBR_mp; //EBC5:XBR系统状态
// 	u_int8_t VehDa_stBrkTempWarn_mp; //EBC5:制动系统温度报警状态
// 	u_int8_t VehDa_stXBRActv_mp; //EBC5:XBR激活状态
// 	double VehDa_aXBRLim_mp; //EBC5:XBR减速度限值
// } EBS_To_Ctrl;


// typedef struct 
// {
//     double DE_phiSteerAngle; // msg_ctlinfo1_eps1_kb:方向盘转角
// 	double DE_phiSteerSpd; //msg_ctlinfo1_eps1_kb：方向盘转速
// 	double DE_SteerToruqe; //msg_ctlinfo1_eps1_kb： 0
// 	double DE_HandTorque; // msg_ctlinfo1_eps1_kb: 方向盘手扭矩 steering torque
// 	double DE_EpsMode; //msg_ctlinfo2_eps1_kb：ControlDemandResponse；
// 	double DE_EpsERRCode;//msg_ctlinfo2_eps1_kb  Byte6（故障码）
// } EPS_To_Ctrl;

//typedef struct
//{
//    u_int8_t transmissionmode_3; // TC_1:MODE 3
//} TC_1_To_Ctrl;

// typedef struct
// {
// 	union
// 	{
// 		u_int8_t first_byte;
// 		struct
// 		{
// 			u_int8_t Transmission_Service_Indicator : 2;
// 			u_int8_t not_defined_0 : 2;
// 			u_int8_t not_defined_1 : 2;
// 			u_int8_t not_defined_2 : 2;
// 		};
// 	};//0

// 	union
// 	{
// 		u_int8_t second_byte;
// 		struct
// 		{
// 			u_int8_t Transmission_Ready_for_Brake_Release : 2;
// 			u_int8_t not_defined_3 : 2;
// 			u_int8_t Transmission_Engine_Crank_Enable : 2;
// 			u_int8_t not_defined_4 : 2;
// 		};
// 	};//1

// 	union
// 	{
// 		u_int8_t third_byte;
// 		struct
// 		{
// 			u_int8_t not_defined_5 : 2;
// 			u_int8_t Transmission_Mode_3 : 2;
// 			u_int8_t Transmission_Mode_2 : 2;
// 			u_int8_t Transmission_Mode_1 : 2;
// 		};
// 	};//2

// 	u_int8_t Transmission_Requested_Gear_Feedback; //3
// 	union
// 	{
// 		u_int8_t fifth_byte;
// 		struct
// 		{
// 			u_int8_t Transmission_Mode_5 : 2;
// 			u_int8_t Transmission_Mode_6 : 2;
// 			u_int8_t Transmission_Mode_7 : 2;
// 			u_int8_t Transmission_Mode_8 : 2;
// 		};
// 	};//4

// 	union
// 	{
// 		u_int8_t sixth_byte;
// 		struct{
// 			u_int8_t Transmission_Reverse_Gear_Shift_Inhibit_Status : 2;
// 			u_int8_t Transmission_Warning_Indicator : 2;
// 			u_int8_t Transmission_Mode_9 :2;
// 			u_int8_t Transmission_Mode_10 :2;
// 		};
// 	};//5

// 	union
// 	{
// 		u_int8_t seventh_byte;
// 		struct
// 		{
// 			u_int8_t Transmission_Air_Supply_Pressure : 3;
// 			u_int8_t Transmission_Auto_Neutral_State : 3;
// 			u_int8_t Transmission_Manual_Mode :2;

// 		};
// 	};//6

// 	union
// 	{
// 		u_int8_t eighth_byte;
// 		struct
// 		{
// 			u_int8_t Clutch_Overload_Warning : 2;
// 			u_int8_t Clutch_Wear_Warning : 2;
// 			u_int8_t Transmission_Temperature_Too_High :2;
// 			u_int8_t not_defined_6; //2

// 		};
// 	};//7
// }ETC7_To_Ctrl; //P-CAN

// typedef struct
// {
// 	u_int32_t Can_Id; //报文ID
// 	u_int8_t data[8]; //报文内容
// 	u_int8_t frequency; //发送周期，单位毫秒
// }Ctrl_To_CAN;  //控制指令发送接口

// typedef struct
// {
// 	float PthPln_lTrgLtr;
// 	float PthPln_phiTrgAng;
// 	float PthPln_kTrgCrv;
// 	float SpdPln_vTgtSpd;
// 	float SpdPln_aTgtAcc;
// 	u_int8_t BhvCrdn_numBhvID;
// 	u_int8_t BhvCrdn_CreepMod;
// 	float S;
// 	u_int8_t rounteID;
// 	u_int8_t BhvCrdn_GearReq;
// }Vechicle_CMD;

// typedef struct
// {
// 	u_int8_t Alarm_Level1;
// 	u_int8_t State1;
// 	double Dis1;
// 	u_int8_t Alarm_Level2;
// 	u_int8_t State2;
// 	double Dis2;
// 	u_int8_t Alarm_Level3;
// 	u_int8_t State3;
// 	double Dis3;
// 	u_int8_t Alarm_Level4;
// 	u_int8_t State4;
// 	double Dis4;
// }Radar_To_Ctrl;

// typedef struct
// {
// 	u_int8_t GasHornActivationSwitch;
// 	u_int8_t ElectricHornActivationSwitch;
// 	u_int8_t ADWiperSwitch;
// 	u_int8_t ADWiperControl;
// 	u_int8_t ADHeadlampSwitch;
// 	u_int8_t ADHeadlampControl;
// 	u_int8_t TurnLampControl;
// 	u_int8_t BrakelightCommand;
// 	u_int8_t ADWasherSwitch;
// 	u_int8_t FogLightSwitch;
// 	u_int8_t AmbientLightDisplayRequest;
// 	u_int8_t SeatbeltTightenRequest;
// 	u_int8_t DoorEmergencyUnlocking;
// 	u_int8_t SeatVibrationRequest;
// 	u_int8_t HazardlightSwitch;
// 	u_int8_t RearlightSwitch;
// 	u_int8_t LiftSwitch;
	
// }BodyControl_ADCU_To_Ctrl;


// typedef struct
// {
// 	u_int8_t EPB_ParkBraStatus;
// 	u_int8_t AH_ParkActive;
// 	u_int8_t AH_BraStatus;
// 	u_int8_t EPB_WorkMode;
// 	u_int8_t ChildLockActive;
// 	u_int8_t LowPreRelParkLimit;
// 	u_int8_t ResponsePriority;
// 	u_int8_t IdepedtBraOly;
// 	u_int8_t PrakBraForcTest;
// 	u_int8_t DriveHillStatus;
// 	u_int8_t EPB_BraSwitch;
// 	u_int8_t ParkBtSwitch;
// 	u_int8_t RelsBtSwitch;
// 	u_int8_t AHBtSwitch;
// 	u_int8_t IdepedtBtSwitch;
// 	u_int8_t ForcTestBtSwitch;
// 	u_int8_t SysStatus;
	
// }EPBS1_EPB_To_Ctrl;


// typedef struct
// {
// 	u_int8_t ExternalBrakeControl;
// 	u_int8_t TractorBrakeControl;
// 	u_int8_t TrailerBrakeControl;
// 	u_int8_t ParkingBrakeControl;
// 	u_int8_t AutoholdBrakeControl;
// 	u_int8_t WorkingMode;
// 	u_int8_t CSUM;
	
// }EPBC1_ADU_To_Ctrl;

// typedef struct
// {
// 	u_int8_t EnableSwitch_TransmissionInputShaftPTO_1;
// 	u_int8_t EnableSwitch_TransmissionInputShaftPTO_2;
	
// }PTODE_To_Ctrl;


// typedef struct
// {
// 	u_int8_t EnableSwitch_TransmissionInputShaftPTO_1;
// 	u_int8_t EnableSwitch_TransmissionInputShaftPTO_2;
	
// }PTODE_ADCU_To_Ctrl;

/// ********************new************************
// typedef struct
// {
// 	u_int8_t EPB_ParkBrKSt;
// }EPB1_To_Ctrl;


#endif
