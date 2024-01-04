#ifndef _LGTCTRL_H
#define _LGTCTRL_H
#include <sys/types.h>

//#include "Types.h" //需包含数据类型定义头文件

typedef struct
{
	u_int8_t VehDa_prcActuTrq; //EEC1：实际扭矩百分比
	u_int8_t VehDa_prcDrvrDmdTrq; //EEC1：驾驶员需求扭矩百分比
	u_int8_t VehDa_stSrcEngCtrl; //EEC1：发动机控制设备源地址
	u_int16_t VehDa_nEngSpd; //EEC1：发动机转速
	u_int8_t VehDa_prcTrqEngNomFric; //EEC2：发动机名义摩擦扭矩百分比
	u_int8_t VehDa_prcTrqEstimdLoss; //EEC2：发动机预估附件损失扭矩百分比
	//u_int16_t VehDa_trqEngRef;// EC1:发动机参考扭矩 
	//u_int16_t VehDa_jEng;//EC1：发动机转动惯量
} EECU_To_Ctrl;

typedef struct
{
	//u_int8_t VehDa_stCluSwt; //CCVS：离合器状态 

} VECU_To_Ctrl;

typedef struct
{
	u_int16_t VehDa_rTraCurGear; //ETC2：变速箱当前档位传动比
	u_int8_t VehDa_stTraCurGear; //ETC2：变速箱当前档位
	u_int8_t VehDa_stTraEgd; //ETC1:传动系统结合状态
	u_int8_t VehDa_stSht; //ECT1:换挡状态
	//u_int8_t VehDa_stTraRdy4BrkRls; //ETC7：制动允许释放状态 !empty:0
} TCU_To_Ctrl;

typedef struct
{
	u_int8_t VehDa_rBrkPedl; //EBC1：制动踏板位置百分比
	u_int8_t VehDa_stBrkPedl; //EBC1：制动踏板状态
	u_int8_t VehDa_stSrcBrkCtrl; //EBC1:制动控制设备源地址
	u_int16_t VehDa_vEgoSpd; //EBC2：前轴车速
	//u_int16_t VehDa_mWght; //CVW：列车总质量  !empty:0
	//u_int8_t VehDa_aEgoLgt; //VDC2：本车纵向加速度 !empty:0
    //u_int8_t VehDa_aEgoLat; //VDC2: 本车横向加速度 !empty:0
    //u_int8_t VehDa_yawRateEgo; //VDC2:自车横摆角速度 !empty:0
	//u_int8_t VehDa_pFrntLe; //EBC3:前轴左侧制动压力 !empty:0
	//u_int8_t VehDa_pFrntRi; //EBC3:前轴右侧制动压力 !empty:0
	//u_int8_t VehDa_stXBR; //EBC5:XBR系统状态 !empty:0
	//u_int8_t VehDa_stBrkTempWarn; //EBC5:制动系统温度报警状态 !empty:0
	//u_int8_t VehDa_stXBRActv; //EBC5:XBR激活状态 !empty:0
	//u_int8_t VehDa_aXBRLim; //EBC5:XBR减速度限值 !empty:0
} EBS_To_Ctrl;

typedef struct 
{
    u_int16_t VehDa_agSteer; // msg_ctlinfo1_eps1_kb:方向盘转角
	u_int16_t VehDa_vSteerSpd; //msg_ctlinfo1_eps1_kb：方向盘转速
	u_int16_t VehDa_tSteerTorque; //msg_ctlinfo1_eps1_kb：方向盘手扭矩
	u_int8_t vehDa_stSteerMode; //msg_ctlinfo2_eps1_kb Byte0
} EPS_To_Ctrl;

typedef struct 
{
	u_int32_t Can_Id; //报文ID
	u_int8_t data[8]; //报文内容
	u_int8_t frequency; //发送周期，单位毫秒
}Ctrl_To_CAN;  //控制指令发送接口






#endif
