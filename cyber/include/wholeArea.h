#ifndef WHOLE_AREA_H
#define WHOLE_AREA_H

#define NODE 0
#include "../include/ADCU_header.h"
#include "../include/Ros_To_Can.h"
#include "../include/LgtCtrl.h"
#include "../include/Ros_To_Can.h"
#include "../include/LatCtrlEX.h"
#include "Ros2canfdbus.h"
#include <ara/log/logging.h>
#include "mdc/mdccan/impl_type_canfdbusdataparaml.h"
#include "mdc/mdccan/impl_type_canfdbusdataparams.h"
#include "../include/mcu_can_fd_interface.h"
#include "../include/LocationIMU0601.h"
#include "../include/LocationEx.h"
#include "System_TboxEx.h"
#include "statechange.h"
#include "../include/LatCtrlEX.h"
#include "../include/LgtCtrlEX.h"
#include "../include/LiftingEx.h"
#include "../include/BehaviorEx.h"
#include "../include/MotionEx.h"
#include "Ros2canbus.h"
#include "../include/CAN_TO_CTRL_EBS.h"
#include "../include/CAN_TO_CTRL_HCU.h"
#include "../include/CAN_TO_CTRL_TCU.h"
#include "../include/CAN_TO_CTRL_EPS.h"
#include "../include/CAN_TO_CTRL_EPB1.h"
#include "System_ADModeEx.h"
#include "../include/Radar.h"
#include "../include/BodyControl_ADCU.h"
#include "../include/EPBC1_ADU.h"
#include "../include/PTODE.h"
#include "../include/PTODE_ADCU.h"
#include "../include/EPBS1_EPB.h"
#include "../include/ETC7.h"
#include "../include/dr_time.h"
#include "../include/ins570d.h"
#include "../build/cmake-build-debug-default_toolchain/gcc-linux-aarch64/aboutcom.pb.h"


using namespace mdc::mdccan;
using namespace CanFdMsgHandle;
using namespace ara::log;
using namespace addcommunication;
//using namespace AppSpace_new;//zxp	0906
static ara::log::Logger &logger = {ara::log::CreateLogger("CAN", "CanFdLDataRecieved", ara::log::LogLevel::kVerbose)};
//extern ros::NodeHandle nh_;

extern McuCanFdInterface canab1ApInterfaceNode;
extern McuCanFdInterface radar5ApInterfaceNode;
extern McuCanFdInterface ecu2ApInterfaceNode;	 
extern McuCanFdInterface radar1ApInterfaceNode;	
extern McuCanFdInterface radar2ApInterfaceNode;
extern McuCanFdInterface radar3ApInterfaceNode;
extern McuCanFdInterface gps1ApInterfaceNode;	 
extern McuCanFdInterface ecu0ApInterfaceNode;
extern McuCanFdInterface radar4ApInterfaceNode;
extern McuCanFdInterface radar6ApInterfaceNode;
extern McuCanFdInterface ecu1ApInterfaceNode;	 
extern McuCanFdInterface canab2ApInterfaceNode;

const uint8_t ECU0_CHANNEL_ID   = 0;
const uint8_t ECU1_CHANNEL_ID   = 1;
const uint8_t ECU2_CHANNEL_ID   = 2;
const uint8_t RADAR1_CHANNEL_ID = 3;
const uint8_t RADAR2_CHANNEL_ID = 4;
const uint8_t RADAR3_CHANNEL_ID = 5;
const uint8_t RADAR4_CHANNEL_ID = 6;
const uint8_t RADAR5_CHANNEL_ID = 7;
const uint8_t RADAR6_CHANNEL_ID = 8;
const uint8_t GPS1_CHANNEL_ID   = 9;
const uint8_t CANAB1_CHANNEL_ID = 10;
const uint8_t CANAB2_CHANNEL_ID = 11;

////A车
//const uint8_t A_CAN_CHANNEL_ID = CANAB2_CHANNEL_ID;//ECU0_CHANNEL_ID;
//const uint8_t C_CAN_CHANNEL_ID = RADAR5_CHANNEL_ID;//RADAR3_CHANNEL_ID;
////const uint8_t TBOX_CHANNEL_ID = CANAB1_CHANNEL_ID;//ECU2_CHANNEL_ID;
//const uint8_t TBOX_CHANNEL_ID = RADAR2_CHANNEL_ID;

//一号车
//const uint8_t A_CAN_CHANNEL_ID = RADAR6_CHANNEL_ID;//ECU0_CHANNEL_ID;
//const uint8_t C_CAN_CHANNEL_ID = RADAR3_CHANNEL_ID;//RADAR3_CHANNEL_ID;
////const uint8_t TBOX_CHANNEL_ID = CANAB1_CHANNEL_ID;//ECU2_CHANNEL_ID;
//const uint8_t TBOX_CHANNEL_ID = GPS1_CHANNEL_ID;

////二号车
//const uint8_t A_CAN_CHANNEL_ID = RADAR5_CHANNEL_ID;//ECU0_CHANNEL_ID;
//const uint8_t C_CAN_CHANNEL_ID = RADAR3_CHANNEL_ID;//RADAR3_CHANNEL_ID;
////const uint8_t TBOX_CHANNEL_ID = GPS1_CHANNEL_ID;//ECU2_CHANNEL_ID;
//const uint8_t TBOX_CHANNEL_ID = RADAR6_CHANNEL_ID;

 //三号车
//const uint8_t A_CAN_CHANNEL_ID = RADAR5_CHANNEL_ID;//ECU0_CHANNEL_ID;
//const uint8_t C_CAN_CHANNEL_ID = RADAR3_CHANNEL_ID;//RADAR3_CHANNEL_ID;
//////const uint8_t TBOX_CHANNEL_ID = CANAB1_CHANNEL_ID;//ECU2_CHANNEL_ID;
//const uint8_t TBOX_CHANNEL_ID = GPS1_CHANNEL_ID;

//电车
const uint8_t A_CAN_CHANNEL_ID = RADAR1_CHANNEL_ID;//ECU0_CHANNEL_ID;
//const uint8_t C_CAN_CHANNEL_ID = RADAR3_CHANNEL_ID;//RADAR3_CHANNEL_ID;
////const uint8_t TBOX_CHANNEL_ID = CANAB1_CHANNEL_ID;//ECU2_CHANNEL_ID;
const uint8_t TBOX_CHANNEL_ID = GPS1_CHANNEL_ID;

extern drdtu::Ros2canfdbus can_to_tbox_reply_msg;	//zxp add 0831	T-box事件回复
//extern TBOX_MOVE_STATE tboxMoveState;	//zxp add 0831 TBOX_MOVE_STATE
//extern TBOX_JOINT_STATE tboxJointORAutoState;	//zxp add 0831 TBOX_JOINT_STATE和TBOX_AUTO_START共用
//extern TBOX_DROP_STATE tboxDropState;	//zxp add 0831 TBOX_DROP_STATE
extern bool hasTBoxMove;
extern bool hasTBoxDrop;
extern bool hasTBoxJoint;
extern bool plat_ctrl;  //平台指令/人工驾驶判断的开关
extern u_int8_t auto_state;
extern u_int8_t Ctrl_lift_state;   //反馈平台需要的举升状态
//extern u_int32_t Ctrl_Acceleration_from_Vehicle ;  //控制计算出的车辆实际加速度  2023.1.11
//u_int8_t Move_state;
extern double dis_error;   //平台指令完成后与终点的距离误差
//ros::Time Location_time_now;
//ros::Time Planning_time_now;
//ros::Time LatCtl_time_now;
//ros::Time LgtCtl_time_now;
extern double Location_time_now;
//extern double LatCtl_time_now;
//extern double LgtCtl_time_now;
extern double Planning_time_now;
extern bool node_state;   ///false;  7.20
extern double ros_time,ros_time_start,ros_time_end;
extern double node_ros_time;
// bool task_error = false;
// int last_BhvCrdn_numBhvID=0;
// int TboxExMsg_getLast = 0;
// bool arrive_end_point =false;
// //新增全局变量
// unsigned char engage_mode = 0;	//0 OR 1
// unsigned char enable_mode = 0;	//0 OR 1
// std::queue<double> vel_pos_x;
// std::queue<double> vel_pos_y;
// std::queue<double> vel_pos_z;
// std::queue<double> vel_pos_yaw;
// bool backward = false;
// bool manual = false;
// unsigned char pre_engage_mode = 0; //0 OR 1
// unsigned char now_engage_mode = 0; //0 OR 1
// int tBoxMoveCount = 0;
// int tBoxJointCount = 0;
// int tBoxDropCount = 0;
//zxp protobuf
extern msg_eec1_e *pb_obj_eec1_e;
//extern EEC1_E inter_eec1; //save the newest status of eec1			
extern msg_eec2_e *pb_obj_eec2_e;
//extern EEC2_E inter_eec2; //save the newest status of eec2
extern msg_etc1_tcu *pb_obj_etc1_tcu;
//extern ETC1_TCU inter_etc1;
extern msg_etc2_tcu inter_etc2;
extern msg_ebc1_ebs *pb_obj_ebc1_ebs;
extern msg_ebc2_ebs *pb_obj_ebc2_ebs;
extern msg_ctlinfo1_eps1_kb *pb_obj_ctlinfo1_eps1_kb;
extern msg_ctlinfo2_eps1_kb *pb_obj_ctlinfo2_eps1_kb;
extern msg_state_bcm *pb_obj_state_bcm;
extern msg_tsc1_te *pb_obj_tsc1_te;
extern msg_etc7 *pb_obj_etc7;
///-------------------new-------------
extern msg_ms1_mcu *pb_obj_ms1_mcu;
//extern MS1_MCU inter_MS1_MCU;
extern msg_mc1_hcu *pb_obj_mc1_hcu;
extern msg_eec3_e *pb_obj_eec3_e;
extern msg_ccvs1_hcu *pb_obj_ccvs1_hcu;
extern msg_tsc1_ae *pb_obj_tsc1_ae;
extern msg_cvw_ebs *pb_obj_cvw_ebs;
extern msg_epb1 *pb_obj_epb1;
//MS2_MCU inter_MS2_MCU;
//extern dfcv_mining_msgs::LocationIMU LocationIMU_wpf;
//extern EECU_To_Ctrl EECU_To_Ctrl_;//zxp	0906
extern msg_tcu_to_ctrl *pb_obj_tcu_to_ctrl;
//extern EBS_To_Ctrl EBS_To_Ctrl_;
//extern EPS_To_Ctrl EPS_To_Ctrl_;
extern msg_etc7_to_ctrl *pb_obj_etc7_to_ctrl;
extern msg_vechicle_cmd *pb_obj_vechicle_cmd;
extern msg_tbox_move *pb_obj_tbox_move;
//--------------------------new-----------------------------------------------------
//extern EPB1_To_Ctrl EPB1_To_Ctrl_;


extern ros::NodeHandle nh_;

namespace AppSpace_new{
class App
{
public:	

	class AppCore
	{
	public:
		//App::ComCore cc;
		//TBOX_MOVE plan;
		msg_tbox_joint *pb_obj_tbox_joint;
		//TBOX_JOINT joint;
		msg_tbox_drop *pb_obj_tbox_drop;
		//TBOX_DROP DROP;
		std::vector<u_int32_t> planIds;
		//自动驾驶状态切换
		//unsigned char engage_mode; //0 OR 1
		//unsigned char enable_mode; //0 OR 1
		//unsigned char manual; //0 OR 1
		unsigned char counter;
		bool brake_pressed;
		bool reentered;
		unsigned int exit_counter;
		double vel_pos[3];

		bool switchOn;
		bool adentered; //自动驾驶模式
		AD_mode admode; //自动驾驶模式

		//3.15添加自動駕駛狀態切換
		//任务状态
		unsigned int move_state;
		unsigned int drop_state;
		bool joint_state;
		msg_radar_to_ctrl *pb_obj_radar1_to_ctrl;
		msg_radar_to_ctrl *pb_obj_radar2_to_ctrl;
		msg_radar_to_ctrl *pb_obj_radar3_to_ctrl;
		msg_radar_to_ctrl *pb_obj_radar4_to_ctrl;
		//Radar_To_Ctrl Radar1;
		//Radar_To_Ctrl Radar2;
		//Radar_To_Ctrl Radar3;
		//Radar_To_Ctrl Radar4;		
		//由can输入radar数据
		msg_radar_front *pb_obj_radar_front;
		//RADAR_Front inter_radar1;
		msg_radar_back *pb_obj_radar_back;
		//RADAR_Back inter_radar2;
		msg_radar_side1 *pb_obj_radar_side1;
		//RADAR_Side1 inter_radar3;
		msg_radar_side2 *pb_obj_radar_side2;
		//RADAR_Side2 inter_radar4;		
		//2022.3.10新接入控制信号
		msg_body_control_adcu *pb_obj_body_control_adcu;
		//can输入信号
		//BodyControl_ADCU inter_BodyControl_ADCU;
		msg_epbs1_epb *pb_obj_epbs1_epb;
		//EPBS1_EPB inter_EPBS1_EPB;
		msg_epbc1_adu *pb_obj_epbc1_adu;
		//EPBC1_ADU inter_EPBC1_ADU;
		msg_ptode  *pb_obj_ptode;
		//PTODE inter_PTODE;
		msg_ptode_adcu *pb_obj_ptode_adcu;
		//PTODE_ADCU inter_PTODE_ADCU;		
		msg_bodycontrol_adcu_to_ctrl *pb_obj_bodycontrol_adcu_to_ctrl;
		//BodyControl_ADCU_To_Ctrl BodyControl_ADCU_To_Ctrl_;
		msg_epbs1_epb_to_ctrl *pb_obj_epbs1_epb_to_ctrl;
		//EPBS1_EPB_To_Ctrl EPBS1_EPB_To_Ctrl_;
		msg_epbc1_adu_to_ctrl *pb_obj_epbc1_adu_to_ctrl;
		//EPBC1_ADU_To_Ctrl EPBC1_ADU_To_Ctrl_;
		msg_ptode_to_ctrl *pb_obi_ptode_to_ctrl;
		//PTODE_To_Ctrl PTODE_To_Ctrl_;
		msg_ptode_adcu_to_ctrl *pb_obi_ptode_adcu_to_ctrl;
		//PTODE_ADCU_To_Ctrl PTODE_ADCU_To_Ctrl_;
		dfcv_mining_msgs::System_TboxEx msg;
		dfcv_mining_msgs::System_TboxEx lastMsg;
		//dfcv_mining_msgs::System_TboxEx TboxExMsg;
		auto TboxExMsg = std::make_shared<dfcv_mining_msgs::System_TboxEx>();
		int  start_TboxExMsg;
		int  end_TboxExMsg;
		//bool TboxExMsg_sendState = false;	//zxp 0906
		bool TboxExMsg_sendState;
		int autodrive_counter;
		//int k = 1;	//zxp 0906
		int flag;
		//df_msgs::TBOXMOVE msg;
		//df_msgs::TBOXMOVE lastMsg;
		//df_msgs::TBOXJOINT jointMsg;
		//df_msgs::TBOXJOINT lastJointMsg;
		//df_msgs::TBOXDROP dropMsg;
		//df_msgs::TBOXDROP lastDropMsg;

		AppCore()
		{
			//engage_mode = 0;
			//enable_mode = 1;
			counter = 0;
			brake_pressed = false;
			reentered = false;

			for(int m = 0;m<3;m++)
			{
				vel_pos[m] = 0.0;
			}

			pb_obj_tcu_to_ctrl->set_vehda_sttracurgear_mp(0);
			exit_counter = 0;
			//backward = false;
			//manual = false;
			//pre_engage_mode = 0;
			//now_engage_mode = 0;
			adentered = false;
			admode = AD_Off;
			pb_obj_vechicle_cmd->set_bhvcrdn_numbhvid(15);
			pb_obj_vechicle_cmd->set_bhvcrdn_gearreq(2);
			//pb_obj_etc7_to_ctrl->etc7_to_ctrl_third_byte_1().set_transmission_mode_3(0);
			//pb_obj_etc7_to_ctrl->etc7_to_ctrl_third_byte_1().set_transmission_mode_3(0);
			autodrive_counter=0;
			TboxExMsg.Drop_TaskOperation = 0;
			TboxExMsg.Move_TaskOperation = 0;
			pb_obj_tbox_drop->set_questtype(0);
			pb_obj_tbox_drop->set_questoperation(0);
			//tBoxMoveCount = 0;
			//tBoxJointCount = 0;
			//tBoxDropCount=0;
			switchOn=false;
			//arrive_end_point=false;
			TboxExMsg_sendState = false;
			flag = 1;
			
		};

		void setEEC1_E(msg_eec1_e* eec1_e);	
		void setEEC2_E(msg_eec2_e* eec2_e);
		void setETC1_TCU(msg_etc1_tcu* etc1);
		void setETC2_TCU(msg_etc2_tcu* etc2);
		void setEBC1_EBS(msg_ebc1_ebs* ebc1);
		void setEBC2_EBS(msg_ebc2_ebs* ebc2);
		void setCTLINFO1_EPS1_KB(msg_ctlinfo1_eps1_kb* eps1);
		void setCTLINFO2_EPS1_KB(msg_ctlinfo2_eps1_kb* eps2);
		void setState_BCM(msg_state_bcm* bcm);
		void setTSC1_TE(msg_tsc1_te* tsc1);
		void setETC7(msg_etc7* etc7);
		unsigned char CRC8(uint8_t data[]);
	//------------------new----------------------
		void setMS1_MCU(msg_ms1_mcu*ms1_mcu);
		void setMC1_HCU(msg_mc1_hcu*mc1_hcu);
		void setEEC3_E(msg_eec3_e*eec3_e);
		void setCCVS1_HCU(msg_ccvs1_hcu*ccvs1_hcu);
		void setTSC1_AE(msg_tsc1_ae*tsc1_ae);
		void setCVW_EBS(msg_cvw_ebs*cvw_ebs);
		void setEPB1(msg_epb1*epb1);
		//void setMS2_MCU(MS2_MCU*ms2_mcu);
		//2023.2.6接入IMU
		void setXCID_Error(const uint8_t *src_p);
		void setXCID_SampleTime(const uint8_t *src_p);
		void setXCID_GroupCounter(const uint8_t *src_p);
		void setXCDI_StatusWord(const uint8_t *src_p);
		void setXCDI_Quaternion(const uint8_t *src_p);
		void setXCDI_EulerAngles(const uint8_t *src_p);
		void setXCDI_DeltaV(const uint8_t *src_p);
		void setXCDI_RateOfTurn(const uint8_t *src_p);
		void setXCDI_DeltaQ(const uint8_t *src_p);
		void setXCDI_Acceleration(const uint8_t *src_p);
		void setXCDI_FreeAcceleration(const uint8_t *src_p);
		//2023.6.1接入IMU--MS6111
		void setXCID_GPS_time(const uint8_t *src_p);
		void setXCID_Z_Gyro_Acce(const uint8_t *src_p);
		void setXCID_X_Y_Gyro(const uint8_t *src_p);
		void setXCDI_X_Y_acce(const uint8_t *src_p);
		void setXCDI_Triaxial_attitude(const uint8_t *src_p);
		void setEngageMode();
		//void radar_Pub(const Radar_To_Ctrl& radar_data);
		//void BodyControl_ADCU_Pub(const BodyControl_ADCU_To_Ctrl& BodyControl_ADCU_data);
		//void EPBS1_EPB_Pub(const EPBS1_EPB_To_Ctrl& EPBS1_EPB_data);
		//void EPBC1_ADU_Pub(const EPBC1_ADU_To_Ctrl& EPBC1_ADU_data);
		//void PTODE_Pub(const PTODE_To_Ctrl& PTODE_data);
		//void PTODE_ADCU_Pub(const PTODE_ADCU_To_Ctrl& PTODE_ADCU_data);
		//void ETC7_Pub(const ETC7_To_Ctrl& ETC7_data);
		// void LocationIMU_mp_Pub(const LocationIMU_mp& LocationIMU_data);   //2023.2.6接入IMU
		//void LocationIMU_mp_Pub();   //2023.6.1接入IMU
		void setRADAR_Front(msg_radar_front* radar1);
		void setRADAR_Back(msg_radar_back* radar2);
		void setRADAR_Side1(msg_radar_side1* radar3);
		void setRADAR_Side2(msg_radar_side2* radar4);
		//void ros2tbox();		
		void setBodyControl_ADCU(msg_body_control_adcu* BodyControl_ADCU);
		void setEPBS1_EPB(msg_epbs1_epb* EPBS1_EPB);
		void setEPBC1_ADU(msg_epbc1_adu* EPBC1_ADU);
		void setPTODE(msg_ptode* PTODE);
		void setPTODE_ADCU(msg_ptode_adcu* PTODE_ADCU);
		void SYS_StateChange();
		// void publishTboxMsg();
		void cantest();
		//void setTboxMove(TBOX_MOVE* tBoxMove,TBOX_MOVE_id* *tBoxMoveId);
		//void setTboxJoint(TBOX_JOINT* tBoxJoint);  //4.2
		//void setTboxDrop(TBOX_DROP* tBoxDrop);
		void setTbox(msg_tbox_joint* tBoxJoint,msg_tbox_drop* tBoxDrop,msg_tbox_move *tboxMOve,msg_tbox_move_id* tboxMoveIds); //4.28
		void setAutoState(bool autoState); //4.28
		//void tboxAutoStateReply(u_int8_t state,u_int8_t questtype);//0911
		//void tboxMoveReply(bool hasTboxMove,u_int8_t state,u_int8_t questtype);
		//void tboxJointReply(bool hasTboxMove,u_int8_t state,u_int8_t questtype);
		//void tboxDropReply(bool hasTboxMove,u_int8_t state,u_int8_t questtype);
		//void getLifting_LiftComplishFlag(dfcv_mining_msgs::LiftingEx::ConstPtr msg);//zxp	0913
		//void tboxJointReply(TBOX_JOINT* tBoxJoint,u_int8_t state,u_int8_t questtype);
		//void tboxDropReply(TBOX_DROP* tBoxDrop,u_int8_t state,u_int8_t questtype);
		//zxp add 0830	can通信流程提取函数canMsgHandle
		//void canMsgHandle(dfcv_mining_msgs::LgtCtrlEX::ConstPtr primitiveMsg);
		// template <class T> 
		// void canMsgHandle(T &primitiveMsg);
		//zxp add 0831  can通信流程简化——T-box事件回复函数tboxEventReply
		//void tboxEventReply(uint8_t flagNum, u_int8_t tBoxState, u_int8_t questType);
		// template <class T> 	//zxp	0913
		// //zxp add 0904		//zxp	0913
		// void ros2ToCanHandle(T &primitiveMsg);//zxp	0913
		//void timestamp2bcd(long timestamp, u_int8_t bcd_buf[]);
	};
	class ComCore
	{
	public:
/*zxp	0913 add start*/
		void BodyControl_ADCU_Pub(const msg_bodycontrol_adcu_to_ctrl& BodyControl_ADCU_data);
		void PTODE_Pub(const msg_ptode_to_ctrl& PTODE_data);
		void PTODE_ADCU_Pub(const msg_ptode_adcu_to_ctrl& PTODE_ADCU_data);
		void ETC7_Pub(const msg_etc7_to_ctrl& ETC7_data);
		void LocationIMU_mp_Pub();
		void radar_Pub(const msg_radar_to_ctrl& radar_data);
		void EPBS1_EPB_Pub(const msg_epbs1_epb_to_ctrl& EPBS1_EPB_data);
		void EPBC1_ADU_Pub(const msg_epbc1_adu_to_ctrl& EPBC1_ADU_data);
		void ros2tbox();
		void tboxJointReply(bool hasTboxMove,u_int8_t state,u_int8_t questtype);
		void tboxDropReply(bool hasTboxMove,u_int8_t state,u_int8_t questtype);
		void tboxEventReply(uint8_t flagNum, u_int8_t tBoxState, u_int8_t questType);
		void tboxMoveReply(bool hasTboxMove,u_int8_t state,u_int8_t questtype);
		void publishTboxMsg();
		void tboxAutoStateReply(u_int8_t state,u_int8_t questtype);
		template <class T> 
		void canMsgHandle(T &primitiveMsg);
		//void PositionReceived(dfcv_mining_msgs::LocationEx::ConstPtr pos_msg);	//zxp 0911
		void canmsgsend(const ros::TimerEvent& event);	//zxp 0911
		void BehaviorChange(const ros::TimerEvent& event);	//zxp 0911
		void ros2canbusLgtCtrlEx(dfcv_mining_msgs::LgtCtrlEX::ConstPtr msg);
		void ros2canbusLiftingEX(dfcv_mining_msgs::LiftingEx::ConstPtr msg);
		void getLifting_LiftComplishFlag(dfcv_mining_msgs::LiftingEx::ConstPtr msg);
		void ros2canbusLatCtrlEX(dfcv_mining_msgs::LatCtrlEX::ConstPtr msg);
		void PositionReceived(dfcv_mining_msgs::LocationEx::ConstPtr pos_msg);
		void BehaviorExMsgReceived(dfcv_mining_msgs::BehaviorEx::ConstPtr BehaviorEx_msg);
		void MotionExMsgReceived(dfcv_mining_msgs::MotionEx::ConstPtr MotionEx_msg);
		void timestamp2bcd(long timestamp, u_int32_t bcd_buf[]);
		template <class T> 	//zxp	0913
		//zxp add 0904		//zxp	0913
		void ros2ToCanHandle(T &primitiveMsg);//zxp	0913
		//set timer to upload msg

		apollo::cyber::Timer timer1(100, &App::ComCore::canmsgsend, false);
		apollo::cyber::Timer timer2(500, &App::ComCore::BehaviorChange, false);
		timer1.Start();
        timer2.Start();

		auto Ros2canbus_ad_sub  = 
					nh_->CreateReader("/LgtCtrlEx_ToCan_XBR_Info", &App::ComCore::ros2canbusLgtCtrlEx);
		auto Ros2canbus_ad_EPB_sub  = 
					nh_->CreateReader("/LiftingEX_ToCan_EPB_Info", &App::ComCore::ros2canbusLiftingEX);
		auto Ros2canbus_ad_PTO_sub  = 
					nh_->CreateReader("/LiftingEX_ToCan_PTO_Info", &App::ComCore::ros2canbusLiftingEX);
		
		//2023.3.27举升需求
		auto Ros2canbus_ad_LiftingEX_sub = 
					nh_->CreateReader("/LiftingEX_Info",&App::ComCore::getLifting_LiftComplishFlag);
		auto Ros2canbus_ad_LiftingEX_sub  =
					nh_->CreateReader("/LiftingEX_Info", &App::ComCore::getLifting_LiftComplishFlag);
		auto Ros2canbus_ad_BCM_sub  = 
					nh_->CreateReader("/LiftingEX_ToCan_BCM_Info", &App::ComCore::ros2canbusLiftingEX);
		auto Ros2canbus_ad_Command_ADCU_sub  =
					nh_->CreateReader("/LiftingEX_ToCan_Command_ADCU", &App::ComCore::ros2canbusLiftingEX);
		auto Ros2canbus_ad_TBOX_sub  =
						nh_->CreateReader("/LiftingEX_ToCan_TBOX", &App::ComCore::ros2canbusLiftingEX);
		auto Ros2canbus_ad_DM1_sub  =
					nh_->CreateReader("/LiftingEX_ToCan_DM1", &App::ComCore::ros2canbusLiftingEX);
		auto Ros2canbus_ad_tmp1_sub  =
					nh_->CreateReader("/LiftingEX_ToCan_tmp1", &App::ComCore::ros2canbusLiftingEX);
		auto Ros2canbus_ad_tmp2_sub  =
					nh_->CreateReader("/LiftingEX_ToCan_tmp2", &App::ComCore::ros2canbusLiftingEX,);
		
		auto Ros2canbus_c_sub =
					nh_->CreateReader("/LgtCtrlEx_ToCan_CommandADCU_Info", &App::ComCore::ros2canbusLgtCtrlEx);
		auto Ros2canbus_eps_sub  =
					nh_->CreateReader("/LatCtrlEX_ToCan_LKAS_Info", &App::ComCore::ros2canbusLatCtrlEX);
		auto Ros2canfdbus_tbox_sub  =
					//nh_.subscribe("/CAN_TO_TBOX",800, &App::ros2canfdbus, this, ros::TransportHints().tcpNoDelay(true))};//11111111
					nh_->CreateReader("/CAN_TO_TBOX", &App::ComCore::ros2ToCanHandle);
		auto Adt_position_sub = 
					nh_->CreateReader("/location/adt_position", &App::ComCore::PositionReceived);		
		auto BehaviorEx_Msg_sub  =
					nh_->CreateReader("/BehaviorEx_Info", &App::ComCore::BehaviorExMsgReceived);
		auto MotionEx_Msg_sub =
					nh_->CreateReader("/MotionEx_Info", &App::ComCore::MotionExMsgReceived);

		auto intercan_pub = nh_->CreateWriter<dfcv_mining_msgs::Ros_To_Can>("/CTRL_TO_CAN_C_ADCU");
		auto canbus2ros_pub = nh_->CreateWriter<canbus::Ros2canbus>("/canbus/canbus2ros");

		auto canbus2ros_pub = nh_->CreateWriter<canbus::Ros2canbus>("/canbus/canbus2ros");

		auto canfdbus2ros_pub = nh_->CreateWriter<drdtu::Ros2canfdbus>("/CAN_TO_TBOX");

		auto CAN_TO_CTRL_EBS_pub = nh_->CreateWriter<dfcv_mining_msgs::CAN_TO_CTRL_EBS>("/EBS_To_Ctrl")};
		
		auto CAN_TO_CTRL_HCU_pub = 
					nh_->CreateWriter<dfcv_mining_msgs::CAN_TO_CTRL_HCU>("/HCU_To_Ctrl");
		auto CAN_TO_CTRL_TCU_pub =
					nh_->CreateWriter<dfcv_mining_msgs::CAN_TO_CTRL_TCU>("/TCU_To_Ctrl");
		auto CAN_TO_CTRL_EPS_pub =
					nh_->CreateWriter<dfcv_mining_msgs::CAN_TO_CTRL_EPS>("/EPS_TO_CTRL");		
		auto CAN_TO_CTRL_EPB1_pub =
					nh_->CreateWriter<dfcv_mining_msgs::CAN_TO_CTRL_EPB1>("/EPB1_TO_CTRL");
		//ros::Publisher CAN_TO_CTRL_TC_1_pub{
					// nh_.advertise<dfcv_mining_msgs::SYS_TO_PLAN>("/TC_1_To_Ctrl",100)};
		//	ros::Publisher autostate_pub{
		//		 nh_.advertise<dfcv_mining_msgs::Sys_stADMd_mp>("/autostate", 8)};
		//	ros::Publisher autostate_pub{
		//			 nh_.advertise<dfcv_mining_msgs::System_ADModeEx>("/autostate", 8)};
		auto System_ADModeEx_pub =
					nh_->CreateWriter<dfcv_mining_msgs::System_ADModeEx>("/System_ADModeEx_Info");
		auto RADAR_pub = 
					nh_->CreateWriter<dfcv_mining_msgs::Radar>("/Radar");
		//3.13添加控制所需信號的發布
		auto BodyControl_ADCU_pub =
					nh_->CreateWriter<dfcv_mining_msgs::BodyControl_ADCU>("/BodyControl_ADCU");
		auto EPBS1_EPB_pub =
					nh_->CreateWriter<dfcv_mining_msgs::EPBS1_EPB>("/EPBS1_EPB");
		auto EPBC1_ADU_pub =
					nh_->CreateWriter<dfcv_mining_msgs::EPBC1_ADU>("/EPBC1_ADU");
		auto PTODE_pub =
					nh_->CreateWriter<dfcv_mining_msgs::PTODE>("/PTODE") ;
		auto PTODE_ADCU_pub =
					nh_->CreateWriter<dfcv_mining_msgs::PTODE_ADCU>("/PTODE_ADCU") ;
		auto ETC7_pub =
						nh_->CreateWriter<dfcv_mining_msgs::ETC7>("/msg_etc7_to_ctrl") ;//11.10xinzeng6
		//5.5注释掉
		//ros::Publisher TBoxMove
		//							 nh_.advertise<dfcv_mining_msgs::PTODE_ADCU>("/TBOX_MOVE",100)};
		//ros::Publisher TBoxJoint{
		//							 nh_.advertise<df_msgs::TBOXJOINT>("/TBOX_JOINT",100)};S
		//ros::Publisher TBoxDrop{
		//							nh_.advertise<dfcv_mining_msgs::TBOXDROP>("/TBOX_DROP",100)};
		auto TBoxExMsg_pub =
					nh_->CreateWriter<dfcv_mining_msgs::System_TboxEx>("/System_TboxEx_Info") ;
		// ros::Publisher LocationIMU_mp_pub{
		// 			nh_.advertise<dfcv_mining_msgs::LocationIMUCp>("/LocationIMU_Info",1,true)};     //2023.2.6接入IMU		
		auto LocationIMU_mp_pub =
					nh_->CreateWriter<dfcv_mining_msgs::LocationIMU>("/LocationIMU_Info") ;     //2023.2.6接入IMU
		// void canmsgsend(const ros::TimerEvent& event);	//zxp 0911
		// void BehaviorChange(const ros::TimerEvent& event);	//zxp 0911
		// //set timer to upload msg
		// ros::Timer timer = nh_.createTimer(ros::Duration(0.1), &App::AppCore::canmsgsend,this);	//zxp 0911
		// ros::Timer timer1= nh_.createTimer(ros::Duration(0.5), &App::AppCore::BehaviorChange,this);	//zxp 0911
		//ros::Timer timer_lat = nh_.createTimer(ros::Duration(0.02), &App::ros2canbus,this);

		//auto msg = std::make_shared<Car>()
};

#endif
