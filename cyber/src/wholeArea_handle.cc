#include "../include/wholeArea.h"
//#include "../build/cmake-build-debug-default_toolchain/gcc-linux-aarch64/aboutcom.pb.h"
// #include "Ros_To_Can.h"
// using namespace mdc::mdccan;
// using namespace CanFdMsgHandle;
// using namespace ara::log;
//using namespace AboutComSpace;
using namespace AppSpace_new;

bool hasTBoxMove=false;
bool hasTBoxDrop=false;
bool hasTBoxJoint=false;
bool plat_ctrl=false;  //平台指令/人工驾驶判断的开关
u_int8_t Ctrl_lift_state = 0;   //反馈平台需要的举升状态
double dis_error = 0;   //平台指令完成后与终点的距离误差
bool node_state=false;   ///false;  7.20
//drdtu::Ros2canfdbus can_to_tbox_reply_msg;	//zxp add 0831	T-box事件回复
auto nh_ = apollo::cyber::createNode("can2cyber");

bool task_error = false;
int last_BhvCrdn_numBhvID=0;
int TboxExMsg_getLast = 0;
bool arrive_end_point =false;
//新增全局变量
unsigned char engage_mode = 0;	//0 OR 1
unsigned char enable_mode = 0;	//0 OR 1
std::queue<double> vel_pos_x;
std::queue<double> vel_pos_y;
std::queue<double> vel_pos_z;
std::queue<double> vel_pos_yaw;
bool backward = false;
bool manual = false;
unsigned char pre_engage_mode = 0; //0 OR 1
unsigned char now_engage_mode = 0; //0 OR 1
int tBoxMoveCount = 0;
int tBoxJointCount = 0;
int tBoxDropCount = 0;
u_int8_t auto_state = 0; 

msg_tbox_move_state *pb_obj_tbox_move_state;
//TBOX_MOVE_STATE tboxMoveState;
msg_tbox_joint_state *pb_obj_tbox_joint_state;
//TBOX_JOINT_STATE tboxJointORAutoState;
msg_tbox_drop_state *pb_obj_tbox_drop_state;
//TBOX_DROP_STATE tboxDropState;
u_int32_t Ctrl_Acceleration_from_Vehicle;

//zxp0922
msg_eecu_to_ctrl *pb_obj_eecu_to_ctrl;
//EECU_To_Ctrl EECU_To_Ctrl_;//zxp	0906

msg_tcu_to_ctrl *pb_obj_tcu_to_ctrl;
//TCU_To_Ctrl TCU_To_Ctrl_;
double LgtCtl_time_now;
double LatCtl_time_now;
//dfcv_mining_msgs::LocationIMU LocationIMU_wpf;
auto LocationIMU_wpf = std::make_shared<dfcv_mining_msgs::LocationIMU>();
msg_ebs_to_ctrl *pb_obj_ebs_to_ctrl;
//EBS_To_Ctrl EBS_To_Ctrl_;
msg_eps_to_ctrl *pb_obj_eps_to_ctrl;
//EPS_To_Ctrl EPS_To_Ctrl_;
msg_epb1_to_ctrl *pb_obj_epb1_to_ctrl;
//EPB1_To_Ctrl EPB1_To_Ctrl_;
msg_etc7_to_ctrl *pb_obj_etc7_to_ctrl;
//ETC7_To_Ctrl ETC7_;
msg_vechicle_cmd *pb_obj_vechicle_cmd;
//Vechicle_CMD vechicle_pln;
double ros_time,ros_time_start,ros_time_end;
double node_ros_time;
msg_ebc1_ebs *pb_obj_ebc1_ebs;
//EBC1_EBS inter_ebc1;
msg_ctlinfo2_eps1_kb *pb_obj_ctlinfo2_eps1_kb;
//CTLINFO2_EPS1_KB inter_info2;

msg_eec1_e *pb_obj_eec1_e;
//EEC1_E inter_eec1; //save the newest status of eec1
msg_eec2_e *pb_obj_eec2_e;
//EEC2_E inter_eec2; //save the newest status of eec2
msg_etc1_tcu *pb_obj_etc1_tcu;
msg_etc2_tcu *pb_obj_etc2_tcu;
//ETC2_TCU inter_etc2;
msg_ebc2_ebs *pb_obj_ebc2_ebs;
//EBC2_EBS inter_ebc2;
msg_ctlinfo1_eps1_kb *pb_obj_ctlinfo1_eps1_kb;
//CTLINFO1_EPS1_KB inter_info1;
msg_state_bcm *pb_obj_state_bcm;
//State_BCM inter_State_BCM
msg_tsc1_te *pb_obj_tsc1_te;
//TSC1_TE inter_TSC1_TE;
msg_etc7 *pb_obj_etc7;
//ETC7 inter_ETC7;
msg_ms1_mcu *pb_obj_ms1_mcu;
msg_mc1_hcu *pb_obj_mc1_hcu;
//MC1_HCU inter_MC1_HCU;
msg_eec3_e *pb_obj_eec3_e;
//EEC3_E inter_EEC3_E;
msg_ccvs1_hcu *pb_obj_ccvs1_hcu;
//CCVS1_HCU inter_CCVS1_HCU;
msg_tsc1_ae *pb_obj_tsc1_ae;
//TSC1_AE inter_TSC1_AE;
msg_cvw_ebs *pb_obj_cvw_ebs;
//CVW_EBS inter_CVW_EBS;
msg_epb1 *pb_obj_epb1;
//EPB1 inter_EPB1;
double Location_time_now;
double Planning_time_now;

// McuCanFdInterface canab1ApInterfaceNode;
// McuCanFdInterface radar5ApInterfaceNode;
// McuCanFdInterface ecu2ApInterfaceNode;	 
// McuCanFdInterface radar1ApInterfaceNode;	
// McuCanFdInterface radar2ApInterfaceNode;
// McuCanFdInterface radar3ApInterfaceNode;
// McuCanFdInterface gps1ApInterfaceNode;	 
// McuCanFdInterface ecu0ApInterfaceNode;
// McuCanFdInterface radar4ApInterfaceNode;
// McuCanFdInterface radar6ApInterfaceNode;
// McuCanFdInterface ecu1ApInterfaceNode;	 
// McuCanFdInterface canab2ApInterfaceNode;

template <class T>
void App::ComCore::canMsgHandle(T &primitiveMsg){
	App::AppCore ac;
	dfcv_mining_msgs::Ros_To_Can msgCan;
	//int i = 1;	//标记是否需要发送两个CAN路：P-CAN和C-CAN	
	//while(i){	
	//特殊处理ros2canbusLatCtrlEX()需指定报文id为 0xCEF139E的情况
	if(typeid(primitiveMsg)!= typeid(dfcv_mining_msgs::LatCtrlEX::ConstPtr))
	{
		msgCan.can_id = primitiveMsg->can_id;
	}
	else
	{
		msgCan.can_id = 0xCEF139E;
	}

	msgCan.channel_id = A_CAN_CHANNEL_ID;

	for(int i = 0; i < 8; i ++)
	{
		msgCan.data[i] = primitiveMsg->data[i];
	}

	//i = 0;
	
	//ros2canbusAB(msgCan);	//1111111
	dfcv_mining_msgs::Ros_To_Can *msgCanPtr = &msgCan;//zxp 0904	处理报错	11111
	ros2ToCanHandle(msgCanPtr);//zxp 0904	处理报错	11111

}
// void App::canMsgHandle(dfcv_mining_msgs::LgtCtrlEX::ConstPtr primitiveMsg)
// {	
// 	dfcv_mining_msgs::Ros_To_Can msgCan;
// 	//int i = 1;	//标记是否需要发送两个CAN路：P-CAN和C-CAN
// 	//while(i){	
// 	msgCan.can_id = primitiveMsg->can_id;
// 	msgCan.channel_id = A_CAN_CHANNEL_ID;
// 	for(int i = 0; i < 8; i ++)
// 	{
// 		msgCan.data[i] = primitiveMsg->data[i];
// 	}
// 	//i = 0;
// 	ros2canbusAB(msgCan);
// 	//}	
// 	//msgCan.channel_id = TBOX_CHANNEL_ID;
// 	//ros2canbusAB(msgCan);
// 	//zxp add start
// 	// cout<<"canMsgHandle执行完成！\n";
// 	// cout<<"msgCan.can_id"<<msgCan.can_id<<endl;
// 	// cout<<"msgCan.channel_id"<<msgCan.channel_id<<endl;
// 	// cout<<"i = "<< i <<endl;
// 		
// }
//zxp add end	0830	can通信流程提取函数canMsgHandle

//void timestamp2bcd(long timestamp, u_int8_t bcd_buf[]);	//zxp add 0831

//zxp add start 0831	can通信流程简化——T-box事件回复函数tboxEventReply
void App::ComCore::tboxEventReply(uint8_t flagNum, u_int8_t tBoxState, u_int8_t questType){
	int task_state = 0;
	unsigned char buff1[64];
	//drdtu::Ros2canfdbus can_to_tbox_reply_msg;	//zxp add 0831	T-box事件回复
	auto can_to_tbox_reply_msg = std::make_shared<drdtu::Ros2canfdbus>();

	memset(&can_to_tbox_reply_msg.data, 0, sizeof(can_to_tbox_reply_msg.data));
	// for (unsigned int j = 0; j < 64; j++) {
	// 	can_to_tbox_reply_msg.data[j] = 0;
	// }
	//memset(buff1, 0, sizeof(buff1));
	if(flagNum == 1)	//tboxMoveReply
	{
		can_to_tbox_reply_msg.can_id = 0x19ff569e;
		u_int32_t buf1[6] = {0};
		for (int i = 0; i < 6; i++)
		{
			buf1[i] = pb_obj_tbox_move_state->timestamp(i);
		}
		
		timestamp2bcd(1000, buf1);
		
		for(int n = 0; n < 8; n++)
		{
			pb_obj_tbox_move_state->set_questnum(n, pb_obj_tbox_move->questnum(n));
		}
		//int task_state=0;
		if (questType == 1)
		{
			if((tBoxState == 1) || (tBoxState == 13) || (tBoxState == 5))
			{
				task_state = 0;
			}
			else if ((tBoxState == 11) && (arrive_end_point == true))
			{
				task_state = 1;
				//hasTBoxMove = false;
			}
			else if ((tBoxState == 11) && (arrive_end_point == false) && (task_error == true))
			{
				task_state = 6;
			}
		}
		else if (questType == 2)
		{
			if(tBoxState == 11)
			{
				task_state = 3;
			}
			else 
			{
				task_state = 6;
			}
		}
		else if (questType == 3)
		{
			if((tBoxState == 1) || (tBoxState == 13) || (tBoxState == 5))
			{
				task_state = 4;
			}
			else if ((tBoxState == 11) && (arrive_end_point == true))
			{
				task_state = 1;
				//hasTBoxMove = false;
			}
			else if ((tBoxState == 11) && (arrive_end_point == false) && (task_error == true))
			{
				task_state = 6;
			}
		}
		else if (questType == 4)
		{
			if(tBoxState == 11)
			{
				task_state = 5;
			}
			else
			{
				task_state = 6;
			}
		}

		pb_obj_tbox_move_state->set_queststate(task_state);
		
		//unsigned char buff1[64];
		memcpy(buff1, (unsigned char*)&pb_obj_tbox_move_state, 64);

		for (unsigned int j = 0; j < 64; j++) {
			can_to_tbox_reply_msg.data[j] = buff1[j];
		}

		//canfdbus2ros_pub.publish(can_to_tbox_reply_msg);
		logger.LogInfo()<<"++++++++++++++++===========tboxMoveReply_task_state: ==============+++++++++++++++"<<task_state;

	}else if(flagNum == 2)	//tboxAutoState
	{
		can_to_tbox_reply_msg.can_id = 0x19ff559e;

		uint32_t buf1[6] = {0};
		for (int i = 0; i < 6; i++)
		{
			buf1[i] = pb_obj_tbox_joint_state->timestamp(i);
		}
		
		timestamp2bcd(1000, buf1);

		pb_obj_tbox_joint_state->set_queststate(tBoxState);
		pb_obj_tbox_joint_state->set_slow_feedback(1);
		//timestamp2bcd(1000,tboxJointORAutoState.timestamp);	//zxp 0831 注释掉
		//unsigned char buff1[64];
		//memset(buff1, 0, sizeof(buff1));
		memcpy(buff1, (unsigned char*)&pb_obj_tbox_joint_state, 64);

		for (unsigned int j = 0; j < 64; j++) {
			can_to_tbox_reply_msg.data[j] = buff1[j];
		}
		//canfdbus2ros_pub.publish(can_to_tbox_reply_msg);

	}else if(flagNum == 3)
	{	//tboxJointState
		can_to_tbox_reply_msg.can_id = 0x19ff589e;
		u_int32_t buf1[6] = {0};
		for(int i = 0; i < 6; i++){
			buf1[i] = pb_obj_tbox_joint_state->timestamp(i);
		}
		timestamp2bcd(1000, buf1);
		//int task_state = 0;
		task_state = 0;

		if ((tBoxState == 1) || (tBoxState == 13) || (tBoxState == 5))
		{
			task_state = 0;
		}
		else if (tBoxState == 11)
		{
			task_state = 1;
		}
		//tboxState.questnum = tBoxJoint->questnum;
		pb_obj_tbox_joint_state->set_queststate(task_state);
		//tboxJointORAutoState.questtype = questtype;	//zxp 0831 注释掉
		pb_obj_tbox_joint_state->set_slow_feedback(1);
		//timestamp2bcd(1000,tboxJointORAutoState.timestamp);	//zxp 0831 注释掉
		//unsigned char buff1[64];
		//memset(buff1, 0, sizeof(buff1));
		memcpy(buff1, (unsigned char*)&pb_obj_tbox_joint_state, 64);

		for (unsigned int j = 0; j < 64; j++) {
			can_to_tbox_reply_msg.data[j] = buff1[j];
		}

		//canfdbus2ros_pub.publish(can_to_tbox_reply_msg);
		logger.LogInfo()<<"++++++++++++++++===========tboxJointReply_task_state: ==============+++++++++++++++"<<task_state;

	}else if(flagNum == 4)
	{	//tboxDropState
		can_to_tbox_reply_msg.can_id = 0x19ff579e;
		u_int32_t buf1[6] = {0};
		for(int i = 0; i < 6; i++){
			buf1[i] = pb_obj_tbox_drop_state->timestamp(i);
		}
		timestamp2bcd(1000, buf1);

		//tboxState.questnum = tBoxDrop->questnum;
		pb_obj_tbox_drop_state->set_queststate(tBoxState);
		//tboxDropState.questtype = questtype;	//zxp 0831 注释掉
		//timestamp2bcd(1000,tboxDropState.timestamp);//zxp 0831 注释掉
		//unsigned char buff1[64];
		//memset(buff1, 0, sizeof(buff1));
		memcpy(buff1, (unsigned char*)&pb_obj_tbox_drop_state, 64);
		//memset(can_to_tbox_reply_msg.data, 0, sizeof(can_to_tbox_reply_msg.data));

		for (unsigned int j = 0; j < 64; j++) {
			can_to_tbox_reply_msg.data[j] = buff1[j];
		}
		//canfdbus2ros_pub.publish(can_to_tbox_reply_msg);
	}else
	{
		return;
	}

	pb_obj_tbox_move_state->set_questtype(questType);
	//static drdtu::Ros2canfdbus can_to_tbox_reply_msg;
	can_to_tbox_reply_msg.channel_id = TBOX_CHANNEL_ID;
	can_to_tbox_reply_msg.time_meas = deeproute::now();
	can_to_tbox_reply_msg.time_pub = deeproute::now();
	// for (unsigned int j = 0; j < 64; j++) {
	// 	can_to_tbox_reply_msg.data[j] = 0;
	// }
	//ac.canfdbus2ros_pub.publish(can_to_tbox_reply_msg);
	//canfdbus2ros_pub.publish(can_to_tbox_reply_msg);
	canfdbus2ros_pub->Write(can_to_tbox_reply_msg);

}
//zxp add end 0831	can通信流程简化——T-box事件回复函数tboxEventReply
//zxp add 0904
template <class T>
void App::ComCore::ros2ToCanHandle(T &primitiveMsg){
	CanFdBusDataParam canSendData;
	uint8_t canfdDataMaxLen;

	if(typeid(primitiveMsg) == typeid(drdtu::Ros2canfdbus))
	{
		canfdDataMaxLen = 64;
		static uint8_t fd_seq = 0;	//zxp 0904 修改变量名
		canSendData.canIdType = 3;		
		canSendData.seq = fd_seq;
		fd_seq++;
	}
	else if(typeid(primitiveMsg) == typeid(dfcv_mining_msgs::Ros_To_Can))
	{
		canfdDataMaxLen = 8;
		static uint8_t seq = 0;	//zxp 0904 修改变量名
		canSendData.canIdType = 1;		
		canSendData.seq = seq;
		seq++;
	}
	else
	{
		logger.LogInfo()<<"++++++++++++++++===========App::ros2ToCanHandle函数参数类型传递有误==============+++++++++++++++";
	}

    canSendData.sendType = 0;
    canSendData.validLen = canfdDataMaxLen;
    canSendData.timeStamp.second = 0xFFFFFFFF;
    canSendData.timeStamp.nsecond = 0xFFFFFFFF;
    canSendData.canId = primitiveMsg->can_id;
	// canId 以十进制打印
    //logger.LogInfo()<<"canId: "<<canSendData.canId<<"msg->channel_id"<<msg->channel_id;

	for(int i = 0; i < canfdDataMaxLen; i ++)
	{
		canSendData.data.push_back(primitiveMsg->data[i]);
	}

    switch(primitiveMsg->channel_id)
	{
		case CANAB1_CHANNEL_ID: 
			canab1ApInterfaceNode.CanFdDataMethodSend(CANAB1_CHANNEL_ID, canSendData);
		break;

		case RADAR5_CHANNEL_ID: 
			radar5ApInterfaceNode.CanFdDataMethodSend(RADAR5_CHANNEL_ID, canSendData);
		break;
		
		case ECU2_CHANNEL_ID: 
			ecu2ApInterfaceNode.CanFdDataMethodSend(ECU2_CHANNEL_ID , canSendData);
		break;

		case RADAR1_CHANNEL_ID: 
			radar1ApInterfaceNode.CanFdDataMethodSend(RADAR1_CHANNEL_ID, canSendData);
		break;
		
		case RADAR2_CHANNEL_ID: 
			radar2ApInterfaceNode.CanFdDataMethodSend(RADAR2_CHANNEL_ID, canSendData);
		break;
		
		case RADAR3_CHANNEL_ID: 
			radar3ApInterfaceNode.CanFdDataMethodSend(RADAR3_CHANNEL_ID, canSendData);
		break;
		
		case GPS1_CHANNEL_ID: 
			gps1ApInterfaceNode.CanFdDataMethodSend(GPS1_CHANNEL_ID, canSendData);
		break;
		
		case ECU0_CHANNEL_ID: 
			ecu0ApInterfaceNode.CanFdDataMethodSend(ECU0_CHANNEL_ID, canSendData);
		break;
		
		case RADAR4_CHANNEL_ID: 
			radar4ApInterfaceNode.CanFdDataMethodSend(RADAR4_CHANNEL_ID, canSendData);
		break;
		
		case RADAR6_CHANNEL_ID: 
			radar6ApInterfaceNode.CanFdDataMethodSend(RADAR6_CHANNEL_ID, canSendData);
		break;
		
		case ECU1_CHANNEL_ID: 
			ecu1ApInterfaceNode.CanFdDataMethodSend(ECU1_CHANNEL_ID, canSendData);
		break;
		
		case CANAB2_CHANNEL_ID: 
			canab2ApInterfaceNode.CanFdDataMethodSend(CANAB2_CHANNEL_ID, canSendData);
		break;
		
		default: 
		break;
	}

}

// msg channel_id wich means deeproute channel
void App::ComCore::ros2canbusLgtCtrlEx(dfcv_mining_msgs::LgtCtrlEX::ConstPtr msg){
	App::ComCore cc;
	Ctrl_Acceleration_from_Vehicle = msg->LgtCtrl_AccelerationCalculate;    //车辆实际加速度  2023.1.11加	
	//zxp add  start 0831  分离can通信
// 	//logger.LogInfo()<<"++++++++++++++++===========tboxDrop_Reply_task_state: ==============+++++++++++++++"<<DROP.questoperation;
// 	dfcv_mining_msgs::Ros_To_Can msgCan;
// 	msgCan.can_id = msg->can_id;
// 	msgCan.channel_id = A_CAN_CHANNEL_ID;
// 	//msg->channel_id;
// 	for(int i= 0; i < 8; i ++)
// 	 {
// 		msgCan.data[i]=msg->data[i];
// 	 }
// //	logger.LogInfo()<<"++++++++++++++++===========msgCan.can_id: ==============+++++++++++++++"<<msgCan.can_id;
// //	logger.LogInfo()<<"++++++++++++++++===========msg->data[1]: ==============+++++++++++++++"<<msg->data[1];
// //	logger.LogInfo()<<"++++++++++++++++===========msg->data[2]: ==============+++++++++++++++"<<msg->data[2];
// 	//logger.LogInfo()<<"++++++++++++++++===========msg->LgtCtrl_AccelerationCalculate: ==============+++++++++++++++"<<msg->LgtCtrl_AccelerationCalculate;

// 	//dfcv_mining_msgs::Ros_To_Can::Ptr ptr;
// 	//ptr->can_id=msg->can_id;
// 	//ros2canbus(static_cast<dfcv_mining_msgs::Ros_To_Can::ConstPtr>(ptr));
// 	ros2canbusAB(msgCan);
	canMsgHandle(msg);
	//cc.canMsgHandle(msg);
	//zxp add end 0831 分离can通信
	LgtCtl_time_now = msg->header.stamp.sec + msg->header.stamp.nsec/1000000000;
}
void App::ComCore::getLifting_LiftComplishFlag(dfcv_mining_msgs::LiftingEx::ConstPtr msg){
	Ctrl_lift_state = msg->Lifting_LiftComplishFlag;
	logger.LogInfo()<<"=================++++++++++++++++===========Ctrl_lift_state: ==============+++++++++++++++============"<<Ctrl_lift_state;
}

void App::ComCore::ros2canbusLiftingEX(dfcv_mining_msgs::LiftingEx::ConstPtr msg){
	App::ComCore cc;
	logger.LogInfo()<<"Send to CAN Lifting";
//	dfcv_mining_msgs::Ros_To_Can can_to_can_msg;
//
//		can_to_can_msg.can_id = 0x0CFF649E;
//		can_to_can_msg.channel_id = A_CAN_CHANNEL_ID; //P-CAN ad command
//		can_to_can_msg.time_pub = deeproute::now();
//		can_to_can_msg.time_meas = deeproute::now();
//		for(int i= 0; i < 8; i ++)
	//zxp add  start 0831  分离can通信
// 	dfcv_mining_msgs::Ros_To_Can msgCan;
// 	msgCan.can_id = msg->can_id;
// 	msgCan.channel_id = A_CAN_CHANNEL_ID;
// 	for(int i= 0; i < 8; i ++)
// 	{
// 		msgCan.data[i]=msg->data[i];
// 	}
// 	//dfcv_mining_msgs::Ros_To_Can::ConstPtr ptr=msgCan;
// 	ros2canbusAB(msgCan);
// //	msgCan.channel_id = C_CAN_CHANNEL_ID;
// //	ros2canbusAB(msgCan);
// 	//ros2canbus(msg);
	canMsgHandle(msg);
	//cc.canMsgHandle(msg);
	//zxp add end 0831	分离can通信
}

void App::ComCore::ros2canbusLatCtrlEX(dfcv_mining_msgs::LatCtrlEX::ConstPtr msg){
	App::ComCore cc;
	logger.LogInfo()<<"Send to CAN LatCtrl";
//	dfcv_mining_msgs::Ros_To_Can can_to_can_msg;
//
//		can_to_can_msg.can_id = 0x0CFF649E;
//		can_to_can_msg.channel_id = A_CAN_CHANNEL_ID; //P-CAN ad command
//		can_to_can_msg.time_pub = deeproute::now();
//		can_to_can_msg.time_meas = deeproute::now();
	//zxp add  start 0831  分离can通信
	// dfcv_mining_msgs::Ros_To_Can msgCan;
	// msgCan.can_id = 0xCEF139E;
	// 		//msg->can_id;
	// //msgCan.channel_id = msg->channel_id;
	// msgCan.channel_id = A_CAN_CHANNEL_ID;
	// //A_CAN_CHANNEL_ID;
	// for(int i= 0; i < 8; i ++)
	//  {
	// 	msgCan.data[i]=msg->data[i];
	// 	//logger.LogInfo()<<"139EDATA:"<<msg->data[i];
	//  }
	// ros2canbusAB(msgCan);
	// //ros2canbus(msg);
	canMsgHandle(msg);
	//cc.canMsgHandle(msg);
	//zxp add  end 0831  分离can通信
	LatCtl_time_now=msg->header.stamp.sec+msg->header.stamp.nsec/1000000000;
}
#if NODE
//zxp	0905	start	注释掉ros2canbusAB(dfcv_mining_msgs::Ros_To_Can msg)方法
// void App::ros2canbusAB(dfcv_mining_msgs::Ros_To_Can msg){
// 	//	logger.LogInfo()<<"Send to CAN";
//     uint8_t canDataMaxLen = 8;
//     static uint8_t seq = 0;
//     CanFdBusDataParam canSendData;
//     seq ++;
//     canSendData.seq               = seq;
//     canSendData.sendType          = 0;
//     canSendData.canIdType         = 1;
//     canSendData.validLen          = canDataMaxLen;
//     canSendData.timeStamp.second  = 0xFFFFFFFF;
//     canSendData.timeStamp.nsecond = 0xFFFFFFFF;
//     canSendData.canId             = msg.can_id;

// 	for(int i= 0; i < canDataMaxLen; i ++)
//     {
// 	    canSendData.data.push_back(msg.data[i]);
//     }

// 	/*
// 	logger.LogInfo()<<"Send to CAN" //canIdType: "<<canSendData.canIdType
// 			<<"canId: "<< canSendData.canId
// 	        //		<<", validLen: "<<canSendData.validLen
// 			<<", channelId" << msg->channel_id;
// 	*/
// 	switch(msg.channel_id){
//     case CANAB1_CHANNEL_ID:
// 	    canab1ApInterfaceNode.CanFdDataMethodSend(CANAB1_CHANNEL_ID, canSendData);
//             break;
//     case RADAR5_CHANNEL_ID:
// 	    radar5ApInterfaceNode.CanFdDataMethodSend(RADAR5_CHANNEL_ID, canSendData);
//             break;
//     case ECU2_CHANNEL_ID:
// 	    ecu2ApInterfaceNode.CanFdDataMethodSend(ECU2_CHANNEL_ID  , canSendData);
//             break;
//     case RADAR1_CHANNEL_ID:
// 	    radar1ApInterfaceNode.CanFdDataMethodSend(RADAR1_CHANNEL_ID, canSendData);
//             break;
//     case RADAR2_CHANNEL_ID:
// 	    radar2ApInterfaceNode.CanFdDataMethodSend(RADAR2_CHANNEL_ID, canSendData);
//             break;
//     case RADAR3_CHANNEL_ID:
// 	    radar3ApInterfaceNode.CanFdDataMethodSend(RADAR3_CHANNEL_ID, canSendData);
//             break;
//     case GPS1_CHANNEL_ID:
// 	    gps1ApInterfaceNode.CanFdDataMethodSend(GPS1_CHANNEL_ID  , canSendData);
//             break;
//     case ECU0_CHANNEL_ID:
// 	    ecu0ApInterfaceNode.CanFdDataMethodSend(ECU0_CHANNEL_ID  , canSendData);
//             break;
//     case RADAR4_CHANNEL_ID:
// 	    radar4ApInterfaceNode.CanFdDataMethodSend(RADAR4_CHANNEL_ID, canSendData);
//             break;
//     case RADAR6_CHANNEL_ID:
// 	    radar6ApInterfaceNode.CanFdDataMethodSend(RADAR6_CHANNEL_ID, canSendData);
//             break;
//     case ECU1_CHANNEL_ID:
// 	    ecu1ApInterfaceNode.CanFdDataMethodSend(ECU1_CHANNEL_ID  , canSendData);
//             break;
//     case CANAB2_CHANNEL_ID:
// 	    canab2ApInterfaceNode.CanFdDataMethodSend(CANAB2_CHANNEL_ID, canSendData);
//             break;
// 	default:
// 		break;
// 	}
// }
//zxp	0905	end	注释掉ros2canbusAB(dfcv_mining_msgs::Ros_To_Can msg)方法

//zxp 0904	注释start
// void App::ros2canbus(dfcv_mining_msgs::Ros_To_Can::ConstPtr msg){
// 	//	logger.LogInfo()<<"Send to CAN";
//     uint8_t canDataMaxLen = 8;
//     static uint8_t seq = 0;
//     CanFdBusDataParam canSendData;
//     seq ++;
//     canSendData.seq               = seq;
//     canSendData.sendType          = 0;
//     canSendData.canIdType         = 1;
//     canSendData.validLen          = canDataMaxLen;
//     canSendData.timeStamp.second  = 0xFFFFFFFF;
//     canSendData.timeStamp.nsecond = 0xFFFFFFFF;
//     canSendData.canId             = msg->can_id;

// 	for(int i= 0; i < 8; i ++)
//     {
// 	    canSendData.data.push_back(msg->data[i]);
//     }

// 	/*
// 	logger.LogInfo()<<"Send to CAN" //canIdType: "<<canSendData.canIdType
// 			<<"canId: "<< canSendData.canId
// 	        //		<<", validLen: "<<canSendData.validLen
// 			<<", channelId" << msg->channel_id;
// 	*/
// 	switch(msg->channel_id){        
//     case CANAB1_CHANNEL_ID: 
// 	    canab1ApInterfaceNode.CanFdDataMethodSend(CANAB1_CHANNEL_ID, canSendData);
//             break;
//     case RADAR5_CHANNEL_ID: 
// 	    radar5ApInterfaceNode.CanFdDataMethodSend(RADAR5_CHANNEL_ID, canSendData);
//             break;
//     case ECU2_CHANNEL_ID: 
// 	    ecu2ApInterfaceNode.CanFdDataMethodSend(ECU2_CHANNEL_ID  , canSendData);
//             break;
//     case RADAR1_CHANNEL_ID: 
// 	    radar1ApInterfaceNode.CanFdDataMethodSend(RADAR1_CHANNEL_ID, canSendData);
//             break;
//     case RADAR2_CHANNEL_ID: 
// 	    radar2ApInterfaceNode.CanFdDataMethodSend(RADAR2_CHANNEL_ID, canSendData);
//             break;
//     case RADAR3_CHANNEL_ID: 
// 	    radar3ApInterfaceNode.CanFdDataMethodSend(RADAR3_CHANNEL_ID, canSendData);
//             break;
//     case GPS1_CHANNEL_ID: 
// 	    gps1ApInterfaceNode.CanFdDataMethodSend(GPS1_CHANNEL_ID  , canSendData);
//             break;
//     case ECU0_CHANNEL_ID: 
// 	    ecu0ApInterfaceNode.CanFdDataMethodSend(ECU0_CHANNEL_ID  , canSendData);
//             break;
//     case RADAR4_CHANNEL_ID: 
// 	    radar4ApInterfaceNode.CanFdDataMethodSend(RADAR4_CHANNEL_ID, canSendData);
//             break;
//     case RADAR6_CHANNEL_ID: 
// 	    radar6ApInterfaceNode.CanFdDataMethodSend(RADAR6_CHANNEL_ID, canSendData);
//             break;
//     case ECU1_CHANNEL_ID: 
// 	    ecu1ApInterfaceNode.CanFdDataMethodSend(ECU1_CHANNEL_ID  , canSendData);
//             break;
//     case CANAB2_CHANNEL_ID: 
// 	    canab2ApInterfaceNode.CanFdDataMethodSend(CANAB2_CHANNEL_ID, canSendData);
//             break;
// 	default: 
// 		break;
// 	}

// }
//zxp	0904	注释end	

//zxp	0904	add start	修改方法形参并注释
// void App::ros2canbus(dfcv_mining_msgs::Ros_To_Can msg){
// 	//	logger.LogInfo()<<"Send to CAN";
//     uint8_t canDataMaxLen = 8;
//     static uint8_t seq = 0;
//     CanFdBusDataParam canSendData;
//     seq ++;
//     canSendData.seq               = seq;
//     canSendData.sendType          = 0;
//     canSendData.canIdType         = 1;
//     canSendData.validLen          = canDataMaxLen;
//     canSendData.timeStamp.second  = 0xFFFFFFFF;
//     canSendData.timeStamp.nsecond = 0xFFFFFFFF;
//     canSendData.canId             = msg.can_id;

// 	for(int i= 0; i < 8; i ++)
//     {
// 	    canSendData.data.push_back(msg.data[i]);
//     }

// 	/*
// 	logger.LogInfo()<<"Send to CAN" //canIdType: "<<canSendData.canIdType
// 			<<"canId: "<< canSendData.canId
// 	        //		<<", validLen: "<<canSendData.validLen
// 			<<", channelId" << msg->channel_id;
// 	*/
// 	switch(msg.channel_id){        
//     case CANAB1_CHANNEL_ID: 
// 	    canab1ApInterfaceNode.CanFdDataMethodSend(CANAB1_CHANNEL_ID, canSendData);
//             break;
//     case RADAR5_CHANNEL_ID: 
// 	    radar5ApInterfaceNode.CanFdDataMethodSend(RADAR5_CHANNEL_ID, canSendData);
//             break;
//     case ECU2_CHANNEL_ID: 
// 	    ecu2ApInterfaceNode.CanFdDataMethodSend(ECU2_CHANNEL_ID  , canSendData);
//             break;
//     case RADAR1_CHANNEL_ID: 
// 	    radar1ApInterfaceNode.CanFdDataMethodSend(RADAR1_CHANNEL_ID, canSendData);
//             break;
//     case RADAR2_CHANNEL_ID: 
// 	    radar2ApInterfaceNode.CanFdDataMethodSend(RADAR2_CHANNEL_ID, canSendData);
//             break;
//     case RADAR3_CHANNEL_ID: 
// 	    radar3ApInterfaceNode.CanFdDataMethodSend(RADAR3_CHANNEL_ID, canSendData);
//             break;
//     case GPS1_CHANNEL_ID: 
// 	    gps1ApInterfaceNode.CanFdDataMethodSend(GPS1_CHANNEL_ID  , canSendData);
//             break;
//     case ECU0_CHANNEL_ID: 
// 	    ecu0ApInterfaceNode.CanFdDataMethodSend(ECU0_CHANNEL_ID  , canSendData);
//             break;
//     case RADAR4_CHANNEL_ID: 
// 	    radar4ApInterfaceNode.CanFdDataMethodSend(RADAR4_CHANNEL_ID, canSendData);
//             break;
//     case RADAR6_CHANNEL_ID: 
// 	    radar6ApInterfaceNode.CanFdDataMethodSend(RADAR6_CHANNEL_ID, canSendData);
//             break;
//     case ECU1_CHANNEL_ID: 
// 	    ecu1ApInterfaceNode.CanFdDataMethodSend(ECU1_CHANNEL_ID  , canSendData);
//             break;
//     case CANAB2_CHANNEL_ID: 
// 	    canab2ApInterfaceNode.CanFdDataMethodSend(CANAB2_CHANNEL_ID, canSendData);
//             break;
// 	default: 
// 		break;
// 	}

// }
//zxp	0904	add end	修改方法形参并注释

//zxp 0905	start	注释掉void App::ros2canfdbus(drdtu::Ros2canfdbus::ConstPtr msg)方法
//zxp	0904	add start	修改方法形参
// msg channel_id wich means deeproute channel
// void App::ros2canfdbus(drdtu::Ros2canfdbus::ConstPtr msg){
//     uint8_t canfdDataMaxLen = 64;
//     static uint8_t seq = 0;	//zxp 0904 修改变量名
//     seq ++;
//     CanFdBusDataParam canSendData;
//     canSendData.seq = seq;
//     canSendData.sendType = 0;
//     canSendData.canIdType = 3;
//     canSendData.validLen = canfdDataMaxLen;
//     canSendData.timeStamp.second = 0xFFFFFFFF;
//     canSendData.timeStamp.nsecond = 0xFFFFFFFF;
//     canSendData.canId = msg->can_id;
// 	// canId 以十进制打印
//     //logger.LogInfo()<<"canId: "<<canSendData.canId<<"msg->channel_id"<<msg->channel_id;

// 	for(int i= 0; i < canfdDataMaxLen; i ++)
// 	{
// 		canSendData.data.push_back(msg->data[i]);
// 	}

//     switch(msg->channel_id){        
//     case CANAB1_CHANNEL_ID: 
// 	    canab1ApInterfaceNode.CanFdDataMethodSend(CANAB1_CHANNEL_ID, canSendData);
//             break;
//     case RADAR5_CHANNEL_ID: 
// 	    radar5ApInterfaceNode.CanFdDataMethodSend(RADAR5_CHANNEL_ID, canSendData);
//             break;
//     case ECU2_CHANNEL_ID: 
// 	    ecu2ApInterfaceNode.CanFdDataMethodSend(ECU2_CHANNEL_ID  , canSendData);
//             break;
//     case RADAR1_CHANNEL_ID: 
// 	    radar1ApInterfaceNode.CanFdDataMethodSend(RADAR1_CHANNEL_ID, canSendData);
//             break;
//     case RADAR2_CHANNEL_ID: 
// 	    radar2ApInterfaceNode.CanFdDataMethodSend(RADAR2_CHANNEL_ID, canSendData);
//             break;
//     case RADAR3_CHANNEL_ID: 
// 	    radar3ApInterfaceNode.CanFdDataMethodSend(RADAR3_CHANNEL_ID, canSendData);
//             break;
//     case GPS1_CHANNEL_ID: 
// 	    gps1ApInterfaceNode.CanFdDataMethodSend(GPS1_CHANNEL_ID  , canSendData);
//             break;
//     case ECU0_CHANNEL_ID: 
// 	    ecu0ApInterfaceNode.CanFdDataMethodSend(ECU0_CHANNEL_ID  , canSendData);
//             break;
//     case RADAR4_CHANNEL_ID: 
// 	    radar4ApInterfaceNode.CanFdDataMethodSend(RADAR4_CHANNEL_ID, canSendData);
//             break;
//     case RADAR6_CHANNEL_ID: 
// 	    radar6ApInterfaceNode.CanFdDataMethodSend(RADAR6_CHANNEL_ID, canSendData);
//             break;
//     case ECU1_CHANNEL_ID: 
// 	    ecu1ApInterfaceNode.CanFdDataMethodSend(ECU1_CHANNEL_ID  , canSendData);
//             break;
//     case CANAB2_CHANNEL_ID: 
// 	    canab2ApInterfaceNode.CanFdDataMethodSend(CANAB2_CHANNEL_ID, canSendData);
//             break;
// 	default: 
// 		break;
// 	}
// }
//zxp	0904	add end	修改方法形参
//zxp 0905	end	注释掉void App::ros2canfdbus(drdtu::Ros2canfdbus::ConstPtr msg)方法
#endif

#if 0
// void App::setXCID_Error(const uint8_t *src_p)
// {
// 	struct xsens_imu_error_t msg;
// 	LocationIMU_mp_.Error_Code = xsens_imu_error_unpack(&msg, src_p, XSENS_IMU_ERROR_LENGTH);
	
// 	LocationIMU_mp_Pub(LocationIMU_mp_);
// }

// void App::setXCID_SampleTime(const uint8_t *src_p)
// {
// 	struct  xsens_imu_sample_time_t msg;
// 	xsens_imu_sample_time_unpack(&msg, src_p, XSENS_IMU_SAMPLE_TIME_LENGTH);

// 	LocationIMU_mp_.SampleTime = xsens_imu_sample_time_timestamp_decode(msg.timestamp);
// 	logger.LogInfo()<<"****************LocationIMU_mp_.SampleTime**************************"<<LocationIMU_mp_.SampleTime;
// 	LocationIMU_mp_Pub(LocationIMU_mp_);
// }

// void App::setXCID_GroupCounter(const uint8_t *src_p)
// {
// 	struct xsens_imu_group_counter_t msg;
// 	xsens_imu_group_counter_unpack(&msg, src_p, XSENS_IMU_GROUP_COUNTER_LENGTH);

// 	LocationIMU_mp_.GroupCounter = xsens_imu_group_counter_counter_decode(msg.counter);
// 	LocationIMU_mp_Pub(LocationIMU_mp_);
// }

// void App::setXCDI_StatusWord(const uint8_t *src_p)
// {
// 	struct xsens_imu_status_word_t msg;
// 	xsens_imu_status_word_unpack(&msg, src_p, XSENS_IMU_STATUS_WORD_LENGTH);

// 	LocationIMU_mp_.have_gnss_time_pulse = xsens_imu_status_word_have_gnss_time_pulse_decode(msg.have_gnss_time_pulse);
//     LocationIMU_mp_.filter_mode = xsens_imu_status_word_filter_mode_decode(msg.filter_mode);
//     LocationIMU_mp_.clip_mag_z = xsens_imu_status_word_clip_mag_z_decode(msg.clip_mag_z);
//     LocationIMU_mp_.retransmitted = xsens_imu_status_word_retransmitted_decode(msg.retransmitted);
//     LocationIMU_mp_.clipping_detected = xsens_imu_status_word_clipping_detected_decode(msg.clipping_detected);
//     LocationIMU_mp_.interpolated = xsens_imu_status_word_interpolated_decode(msg.interpolated);
//     LocationIMU_mp_.sync_in = xsens_imu_status_word_sync_in_decode(msg.sync_in);
//     LocationIMU_mp_.sync_out = xsens_imu_status_word_sync_out_decode(msg.sync_out);
//     LocationIMU_mp_.clip_acc_x = xsens_imu_status_word_clip_acc_x_decode(msg.clip_acc_x);
//     LocationIMU_mp_.clip_acc_y = xsens_imu_status_word_clip_acc_y_decode(msg.clip_acc_y);
//     LocationIMU_mp_.clip_acc_z = xsens_imu_status_word_clip_acc_z_decode(msg.clip_acc_z);
//     LocationIMU_mp_.clip_gyr_x = xsens_imu_status_word_clip_gyr_x_decode(msg.clip_gyr_x);
//     LocationIMU_mp_.clip_gyr_y = xsens_imu_status_word_clip_gyr_y_decode(msg.clip_gyr_y);
//     LocationIMU_mp_.clip_gyr_z = xsens_imu_status_word_clip_gyr_z_decode(msg.clip_gyr_z);
//     LocationIMU_mp_.clip_mag_x = xsens_imu_status_word_clip_mag_x_decode(msg.clip_mag_x);
//     LocationIMU_mp_.clip_mag_y = xsens_imu_status_word_clip_mag_y_decode(msg.clip_mag_y);
//     LocationIMU_mp_.self_test_ok = xsens_imu_status_word_self_test_ok_decode(msg.self_test_ok);
//     LocationIMU_mp_.orientation_valid = xsens_imu_status_word_orientation_valid_decode(msg.orientation_valid);
//     LocationIMU_mp_.gps_valid = xsens_imu_status_word_gps_valid_decode(msg.gps_valid);
//     LocationIMU_mp_.no_rotation = xsens_imu_status_word_no_rotation_decode(msg.no_rotation);
//     LocationIMU_mp_.representative_motion = xsens_imu_status_word_representative_motion_decode(msg.representative_motion);
//     LocationIMU_mp_.external_clock_synced = xsens_imu_status_word_external_clock_synced_decode(msg.external_clock_synced);
// 	LocationIMU_mp_Pub(LocationIMU_mp_);
// }

// void App::setXCDI_Quaternion(const uint8_t *src_p)
// {
// 	struct xsens_imu_quaternion_t msg;
// 	xsens_imu_quaternion_unpack(&msg, src_p, XSENS_IMU_QUATERNION_LENGTH);

// 	LocationIMU_mp_.Q0 = xsens_imu_quaternion_q1_decode(msg.q1);
//     LocationIMU_mp_.Q1 = xsens_imu_quaternion_q2_decode(msg.q2);
// 	LocationIMU_mp_.Q2 = xsens_imu_quaternion_q3_decode(msg.q3);
// 	LocationIMU_mp_.Q3 = xsens_imu_quaternion_q4_decode(msg.q4);
// 	LocationIMU_mp_Pub(LocationIMU_mp_);
// }

// void App::setXCDI_EulerAngles(const uint8_t *src_p)
// {
// 	struct xsens_imu_euler_angles_t msg;
// 	xsens_imu_euler_angles_unpack(&msg, src_p, XSENS_IMU_EULER_ANGLES_LENGTH);

// 	LocationIMU_mp_.Roll = xsens_imu_euler_angles_roll_decode(msg.roll);
// 	LocationIMU_mp_.Pitch = xsens_imu_euler_angles_pitch_decode(msg.pitch);
// 	LocationIMU_mp_.Yaw = xsens_imu_euler_angles_yaw_decode(msg.yaw);
// 	logger.LogInfo()<<"****************LocationIMU_mp_.Roll***************************="<<LocationIMU_mp_.Roll;
// 	logger.LogInfo()<<"****************LocationIMU_mp_.Pitch***************************="<<LocationIMU_mp_.Pitch;
// 	logger.LogInfo()<<"****************LocationIMU_mp_.Yaw***************************="<<LocationIMU_mp_.Yaw;
// 	LocationIMU_mp_Pub(LocationIMU_mp_);
// }

// void App::setXCDI_DeltaV(const uint8_t *src_p)
// {
// 	struct xsens_imu_delta_v_t msg;
// 	xsens_imu_delta_v_unpack(&msg, src_p, XSENS_IMU_DELTA_V_LENGTH);	

// 	LocationIMU_mp_.DeltaVx = xsens_imu_delta_v_x_decode(msg.x);
// 	LocationIMU_mp_.DeltaVy = xsens_imu_delta_v_y_decode(msg.y);
// 	LocationIMU_mp_.DeltaVz = xsens_imu_delta_v_z_decode(msg.z);
// 	LocationIMU_mp_.Exponent = xsens_imu_delta_v_exponent_decode(msg.exponent);

// 	//logger.LogInfo()<<"****************LocationIMU_mp_.DeltaVxxxxxxxxxx***************************="<<LocationIMU_mp_.DeltaVx;
// 	LocationIMU_mp_Pub(LocationIMU_mp_);
// }

// void App::setXCDI_RateOfTurn(const uint8_t *src_p)
// {
// 	struct xsens_imu_rate_of_turn_t msg;
// 	xsens_imu_rate_of_turn_unpack(&msg, src_p, XSENS_IMU_RATE_OF_TURN_LENGTH);	

// 	LocationIMU_mp_.gyrX = xsens_imu_rate_of_turn_gyr_x_decode(msg.gyr_x);
// 	LocationIMU_mp_.gyrY = xsens_imu_rate_of_turn_gyr_y_decode(msg.gyr_y);
// 	LocationIMU_mp_.gyrZ = xsens_imu_rate_of_turn_gyr_z_decode(msg.gyr_z);
// 	LocationIMU_mp_Pub(LocationIMU_mp_);
// }

// void App::setXCDI_DeltaQ(const uint8_t *src_p)
// {
// 	struct xsens_imu_delta_q_t msg;
// 	xsens_imu_delta_q_unpack(&msg, src_p, XSENS_IMU_DELTA_Q_LENGTH);

// 	LocationIMU_mp_.DeltaQ0 = xsens_imu_delta_q_delta_q1_decode(msg.delta_q1);
//     LocationIMU_mp_.DeltaQ1 = xsens_imu_delta_q_delta_q2_decode(msg.delta_q2);
// 	LocationIMU_mp_.DeltaQ2 = xsens_imu_delta_q_delta_q3_decode(msg.delta_q3);
// 	LocationIMU_mp_.DeltaQ3 = xsens_imu_delta_q_delta_q4_decode(msg.delta_q4);
// 	LocationIMU_mp_Pub(LocationIMU_mp_);
// }

// void App::setXCDI_Acceleration(const uint8_t *src_p)
// {
// 	struct xsens_imu_acceleration_t msg;
// 	xsens_imu_acceleration_unpack(&msg, src_p, XSENS_IMU_ACCELERATION_LENGTH);

// 	LocationIMU_mp_.accX = xsens_imu_acceleration_acc_x_decode(msg.acc_x);
// 	LocationIMU_mp_.accY = xsens_imu_acceleration_acc_y_decode(msg.acc_y);
// 	LocationIMU_mp_.accZ = xsens_imu_acceleration_acc_z_decode(msg.acc_z);
// 	LocationIMU_mp_Pub(LocationIMU_mp_);
// }

// void App::setXCDI_FreeAcceleration(const uint8_t *src_p)
// {
// 	struct xsens_imu_free_acceleration_t msg;
// 	xsens_imu_free_acceleration_unpack(&msg, src_p, XSENS_IMU_FREE_ACCELERATION_LENGTH);

// 	LocationIMU_mp_.freeAccX = xsens_imu_free_acceleration_free_acc_x_decode(msg.free_acc_x);
// 	LocationIMU_mp_.freeAccY = xsens_imu_free_acceleration_free_acc_y_decode(msg.free_acc_y);
// 	LocationIMU_mp_.freeAccZ = xsens_imu_free_acceleration_free_acc_z_decode(msg.free_acc_z);
// 	LocationIMU_mp_Pub(LocationIMU_mp_);
// }
#endif
// 6.1 新加IMU_MS6111
void App::AppCore::setXCID_GPS_time(const uint8_t *src_p)
{	
	App::ComCore cc;
	ins570d_gps_time_t msg;

	ins570d_gps_time_unpack(&msg, src_p, INS570D_GPS_TIME_LENGTH);
	LocationIMU_wpf.GPS_Week = ins570d_gps_time_gps_week_decode(msg.gps_week);
	LocationIMU_wpf.GPS_Millisecond = ins570d_gps_time_gps_millisecond_decode(msg.gps_millisecond);
	logger.LogInfo()<<"****************gps_Week**************************"<< LocationIMU_wpf.GPS_Week;
	logger.LogInfo()<<"****************gps_millisecond**************************"<< LocationIMU_wpf.GPS_Millisecond;
	cc.LocationIMU_mp_Pub();

}

void App::AppCore::setXCID_Z_Gyro_Acce(const uint8_t *src_p)
{	
	App::ComCore cc;
	ins570d_z_gyro_acce_t msg;

	ins570d_z_gyro_acce_unpack(&msg, src_p, INS570D_Z_GYRO_ACCE_LENGTH);
	LocationIMU_wpf.gyroZ = ins570d_z_gyro_acce_z_gyro_decode(msg.z_gyro);
	LocationIMU_wpf.accZ = ins570d_z_gyro_acce_z_acce_decode(msg.z_acce);
	logger.LogInfo()<<"****************z_gyro**************************"<<LocationIMU_wpf.gyroZ;
	logger.LogInfo()<<"****************z_acce**************************"<<LocationIMU_wpf.accZ;
	cc.LocationIMU_mp_Pub();

}

void App::AppCore::setXCID_X_Y_Gyro(const uint8_t *src_p)
{	
	App::ComCore cc;
	ins570d_x_y_gyro_t msg;

	ins570d_x_y_gyro_unpack(&msg, src_p, INS570D_X_Y_GYRO_LENGTH);
	LocationIMU_wpf.gyroX = ins570d_x_y_gyro_x_gyro_decode(msg.x_gyro);
	LocationIMU_wpf.gyroY = ins570d_x_y_gyro_y_gyro_decode(msg.y_gyro);
	logger.LogInfo()<<"****************x_gyro**************************"<<LocationIMU_wpf.gyroX;
	logger.LogInfo()<<"****************y_gyro**************************"<<LocationIMU_wpf.gyroY;
	cc.LocationIMU_mp_Pub();

}

void App::AppCore::setXCDI_X_Y_acce(const uint8_t *src_p)
{	
	App::ComCore cc;
	ins570d_x_y_acce_t msg;

	ins570d_x_y_acce_unpack(&msg, src_p, INS570D_X_Y_ACCE_LENGTH);
	LocationIMU_wpf.accX = ins570d_x_y_acce_x_acce_decode(msg.x_acce);
	LocationIMU_wpf.accY = ins570d_x_y_acce_y_acce_decode(msg.y_acce);
	logger.LogInfo()<<"****************x_acce**************************"<<LocationIMU_wpf.accX;
	logger.LogInfo()<<"****************y_acce**************************"<<LocationIMU_wpf.accY;
	cc.LocationIMU_mp_Pub();

}

void App::AppCore::setXCDI_Triaxial_attitude(const uint8_t *src_p)
{
	App::ComCore cc;
	ins570d_triaxial_attitude_t msg;

	ins570d_triaxial_attitude_unpack(&msg, src_p, INS570D_TRIAXIAL_ATTITUDE_LENGTH);
	LocationIMU_wpf.roll = ins570d_triaxial_attitude_tra_roll_decode(msg.tra_roll);
	LocationIMU_wpf.pitch = ins570d_triaxial_attitude_tra_pitch_decode(msg.tra_pitch);
	LocationIMU_wpf.yaw = ins570d_triaxial_attitude_tra_course_decode(msg.tra_course);
	logger.LogInfo()<<"****************roll**************************"<<LocationIMU_wpf.roll;
	logger.LogInfo()<<"****************pitch**************************"<<LocationIMU_wpf.pitch;
	logger.LogInfo()<<"****************yaw**************************"<<LocationIMU_wpf.yaw;
	cc.LocationIMU_mp_Pub();

}




/*
void App::radar_Pub(const Radar_To_Ctrl& radar_data)
{
	dfcv_mining_msgs::Radar msg;
	msg.Alarm_Level1 = radar_data.Alarm_Level1;
	msg.State1 = radar_data.State1;
	msg.Dis1 = radar_data.Dis1;
	msg.Alarm_Level2 = radar_data.Alarm_Level2;
	msg.State2 = radar_data.State2;
	msg.Dis2 = radar_data.Dis2;
	msg.Alarm_Level3 = radar_data.Alarm_Level3;
	msg.State3 = radar_data.State3;
	msg.Dis3 = radar_data.Dis3;
	msg.Alarm_Level4 = radar_data.Alarm_Level4;
	msg.State4 = radar_data.State4;
	msg.Dis4 = radar_data.Dis4;

	//RADAR_pub.publish(msg);
}
*/
//3/13發布控制所需信號mvc b
void App::ComCore::BodyControl_ADCU_Pub(const msg_bodycontrol_adcu_to_ctrl& BodyControl_ADCU_data)
{
	//ara::log::Logger& logger = ara::log::LogManager::defaultLogContext();
	//dfcv_mining_msgs::BodyControl_ADCU msg;
	auto msg = std::make_shared<dfcv_mining_msgs::BodyControl_ADCU>();

	msg.GasHornActivationSwitch = BodyControl_ADCU_data.gashornactivationswitch();
	msg.ElectricHornActivationSwitch = BodyControl_ADCU_data.electrichornactivationswitch();
	msg.ADWiperSwitch = BodyControl_ADCU_data.adwiperswitch();
	msg.ADWiperControl = BodyControl_ADCU_data.adwipercontrol();
	msg.ADHeadlampSwitch = BodyControl_ADCU_data.adheadlampswitch();
	msg.ADHeadlampControl = BodyControl_ADCU_data.adheadlampcontrol();
	msg.TurnLampControl = BodyControl_ADCU_data.turnlampcontrol();
	msg.BrakelightCommand = BodyControl_ADCU_data.brakelightcommand();
	msg.ADWasherSwitch = BodyControl_ADCU_data.adwasherswitch();
	msg.AmbientLightDisplayRequest = BodyControl_ADCU_data.ambientlightdisplayrequest();
	msg.SeatbeltTightenRequest= BodyControl_ADCU_data.seatbelttightenrequest();
	msg.DoorEmergencyUnlocking = BodyControl_ADCU_data.dooremergencyunlocking();
	msg.SeatVibrationRequest = BodyControl_ADCU_data.seatvibrationrequest();
	msg.HazardlightSwitch = BodyControl_ADCU_data.hazardlightswitch();
	msg.RearlightSwitch = BodyControl_ADCU_data.rearlightswitch();
	msg.LiftSwitch = BodyControl_ADCU_data.liftswitch();
	BodyControl_ADCU_pub->Write(msg);
}

void App::ComCore::EPBS1_EPB_Pub(const msg_epbs1_epb_to_ctrl& EPBS1_EPB_data)
{
	//App::ComCore cc;
	//ara::log::Logger& logger = ara::log::LogManager::defaultLogContext();
	//dfcv_mining_msgs::EPBS1_EPB msg;
	auto msg = std::make_shared<dfcv_mining_msgs::EPBS1_EPB>();
	
	msg.EPB_ParkBraStatus = EPBS1_EPB_data.epb_parkbrastatus();
	msg.AH_ParkActive = EPBS1_EPB_data.ah_parkactive();
	msg.AH_BraStatus = EPBS1_EPB_data.ah_brastatus();
	msg.EPB_WorkMode = EPBS1_EPB_data.epb_workmode();
	msg.ChildLockActive = EPBS1_EPB_data.childlockactive();
	msg.LowPreRelParkLimit = EPBS1_EPB_data.lowprerelparklimit();
	msg.ResponsePriority = EPBS1_EPB_data.responsepriority();
	msg.IdepedtBraOly = EPBS1_EPB_data.idepedtbraoly();
	msg.PrakBraForcTest = EPBS1_EPB_data.prakbraforctest();
	msg.DriveHillStatus = EPBS1_EPB_data.drivehillstatus();
	msg.EPB_BraSwitch= EPBS1_EPB_data.epb_braswitch();
	msg.ParkBtSwitch= EPBS1_EPB_data.parkbtswitch();
	msg.RelsBtSwitch = EPBS1_EPB_data.relsbtswitch();
	msg.AHBtSwitch = EPBS1_EPB_data.ahbtswitch();
	msg.IdepedtBtSwitch = EPBS1_EPB_data.idepedtbtswitch();
	msg.ForcTestBtSwitch = EPBS1_EPB_data.forctestbtswitch();
	msg.SysStatus = EPBS1_EPB_data.sysstatus();
	//EPBS1_EPB_pub.publish(msg);
	EPBS1_EPB_pub->Write(msg);
}

// void App::ComCore::timestamp2bcd(long timestamp, u_int8_t bcd_buf[])
// {	
// 	struct tm *time;
// 	char buffer[20];

// 	//时区不同.手动加八个小时
// 	timestamp = timestamp + 28800; //北京时间	
// 	time = gmtime(&timestamp);
// 	strftime(buffer, 20, "%y%m%d%H%M%S", time);
// 	bcd_buf[0] = (unsigned char)((buffer[0] - '0')<<4) + (buffer[1] - '0');
// 	bcd_buf[1] = (unsigned char)((buffer[2] - '0')<<4) + (buffer[3] - '0');
// 	bcd_buf[2] = (unsigned char)((buffer[4] - '0')<<4) + (buffer[5] - '0');
// 	bcd_buf[3] = (unsigned char)((buffer[6] - '0')<<4) + (buffer[7] - '0');
// 	bcd_buf[4] = (unsigned char)((buffer[8] - '0')<<4) + (buffer[9] - '0');
// 	bcd_buf[5] = (unsigned char)((buffer[10] - '0')<<4) + (buffer[11] - '0');
// }

void App::ComCore::timestamp2bcd(long timestamp, u_int32_t bcd_buf[])
{	
	struct tm *time;
	char buffer[20];

	//时区不同.手动加八个小时
	timestamp = timestamp + 28800; //北京时间	
	time = gmtime(&timestamp);
	strftime(buffer, 20, "%y%m%d%H%M%S", time);
	bcd_buf[0] = (unsigned char)((buffer[0] - '0')<<4) + (buffer[1] - '0');
	bcd_buf[1] = (unsigned char)((buffer[2] - '0')<<4) + (buffer[3] - '0');
	bcd_buf[2] = (unsigned char)((buffer[4] - '0')<<4) + (buffer[5] - '0');
	bcd_buf[3] = (unsigned char)((buffer[6] - '0')<<4) + (buffer[7] - '0');
	bcd_buf[4] = (unsigned char)((buffer[8] - '0')<<4) + (buffer[9] - '0');
	bcd_buf[5] = (unsigned char)((buffer[10] - '0')<<4) + (buffer[11] - '0');
}

void App::ComCore::EPBC1_ADU_Pub(const msg_epbc1_adu_to_ctrl& EPBC1_ADU_data)
{
	//dfcv_mining_msgs::EPBC1_ADU msg;
	auto msg = std::make_shared<dfcv_mining_msgs::EPBC1_ADU>();

	msg.ExternalBrakeControl = EPBC1_ADU_data.externalbrakecontrol();
	msg.TractorBrakeControl = EPBC1_ADU_data.tractorbrakecontrol();
	msg.TrailerBrakeControl = EPBC1_ADU_data.trailerbrakecontrol();
	msg.ParkingBrakeControl = EPBC1_ADU_data.parkingbrakecontrol();
	msg.AutoholdBrakeControl = EPBC1_ADU_data.autoholdbrakecontrol();
	msg.WorkingMode = EPBC1_ADU_data.workingmode();
	msg.CSUM = EPBC1_ADU_data.csum();

	//EPBC1_ADU_pub.publish(msg);
	EPBC1_ADU_pub->Write(msg);
}

void App::ComCore::PTODE_Pub(const msg_ptode_to_ctrl& PTODE_data)
{
	//dfcv_mining_msgs::PTODE msg;
	auto msg = std::make_shared<dfcv_mining_msgs::PTODE>();
	msg.EnableSwitch_TransmissionInputShaftPTO_1 = PTODE_data.enableswitch_transmissioninputshaftpto_1();
	msg.EnableSwitch_TransmissionInputShaftPTO_2 = PTODE_data.enableswitch_transmissioninputshaftpto_2();
	
	//PTODE_pub.publish(msg);
	PTODE_pub->Write(msg);
}

void App::ComCore::PTODE_ADCU_Pub(const msg_ptode_adcu_to_ctrl& PTODE_ADCU_data)
{
	//dfcv_mining_msgs::PTODE_ADCU msg;
	auto msg = std::make_shared<dfcv_mining_msgs::PTODE_ADCU>();
	msg.EnableSwitch_TransmissionInputShaftPTO_1 = PTODE_ADCU_data.enableswitch_transmissioninputshaftpto_1();
	msg.EnableSwitch_TransmissionInputShaftPTO_2 = PTODE_ADCU_data.enableswitch_transmissioninputshaftpto_2();

	//PTODE_ADCU_pub.publish(msg);
	PTODE_ADCU_pub->Write(msg);
}

void App::ComCore::ETC7_Pub(const msg_etc7_to_ctrl& ETC7_data)
{
	//dfcv_mining_msgs::ETC7 msg;
	auto msg = std::make_shared<dfcv_mining_msgs::ETC7>();
	msg.Transmission_Mode_3 = ETC7_data.etc7_to_ctrl_third_byte_1().transmission_mode_3();
	msg.Transmission_Ready_for_Brake_Release = ETC7_data.etc7_to_ctrl_second_byte_1().transmission_ready_for_brake_release();

	//ETC7_pub.publish(msg);
	ETC7_pub->Write(msg);
}

//2023.2.6接入IMU
// void App::LocationIMU_mp_Pub(const LocationIMU_mp& LocationIMU_data)
// {
// 	dfcv_mining_msgs::LocationIMUCp msg;

// 	msg.Error_Code=LocationIMU_data.Error_Code;
// 	msg.SampleTime=LocationIMU_data.SampleTime;
// 	msg.GroupCounter=LocationIMU_data.GroupCounter;
// 	//msg.StatusWord=LocationIMU_data.StatusWord;
// 	msg.clip_acc_x=LocationIMU_data.clip_acc_x;
// 	msg.clip_acc_y=LocationIMU_data.clip_acc_y;
// 	msg.clip_acc_z=LocationIMU_data.clip_acc_z;
// 	msg.clip_gyr_x=LocationIMU_data.clip_gyr_x;
// 	msg.clip_gyr_y=LocationIMU_data.clip_gyr_y;
// 	msg.clip_gyr_z=LocationIMU_data.clip_gyr_z;
// 	msg.clip_mag_x=LocationIMU_data.clip_mag_x;
// 	msg.clip_mag_y=LocationIMU_data.clip_mag_y;
// 	msg.clip_mag_z=LocationIMU_data.clip_mag_z;
// 	msg.clipping_detected=LocationIMU_data.clipping_detected;
// 	msg.external_clock_synced=LocationIMU_data.external_clock_synced;
// 	msg.filter_mode=LocationIMU_data.filter_mode;
// 	msg.freeAccX=LocationIMU_data.freeAccX;
// 	msg.freeAccY=LocationIMU_data.freeAccY;
// 	msg.freeAccZ=LocationIMU_data.freeAccZ;
// 	msg.gps_valid=LocationIMU_data.gps_valid;
// 	msg.gyrX=LocationIMU_data.gyrX;
// 	msg.gyrY=LocationIMU_data.gyrY;
// 	msg.gyrZ=LocationIMU_data.gyrZ;
// 	msg.have_gnss_time_pulse=LocationIMU_data.have_gnss_time_pulse;
// 	msg.interpolated=LocationIMU_data.interpolated;
// 	msg.no_rotation=LocationIMU_data.no_rotation;
// 	msg.orientation_valid=LocationIMU_data.orientation_valid;
// 	msg.representative_motion=LocationIMU_data.representative_motion;
// 	msg.retransmitted=LocationIMU_data.retransmitted;
// 	msg.self_test_ok=LocationIMU_data.self_test_ok;
// 	msg.sync_in=LocationIMU_data.sync_in;
// 	msg.sync_out=LocationIMU_data.sync_out;
// 	msg.Q0=LocationIMU_data.Q0;
// 	msg.Q1=LocationIMU_data.Q1;
// 	msg.Q2=LocationIMU_data.Q2;
// 	msg.Q3=LocationIMU_data.Q3;
// 	msg.Roll=LocationIMU_data.Roll;
// 	msg.Pitch=LocationIMU_data.Pitch;
// 	msg.Yaw=LocationIMU_data.Yaw;
// 	msg.DeltaVx=LocationIMU_data.DeltaVx;
// 	msg.DeltaVy=LocationIMU_data.DeltaVy;
// 	msg.DeltaVz=LocationIMU_data.DeltaVz;
// 	msg.Exponent=LocationIMU_data.Exponent;
// 	msg.gyrX=LocationIMU_data.gyrX;
// 	msg.gyrY=LocationIMU_data.gyrY;
// 	msg.gyrZ=LocationIMU_data.gyrZ;
// 	msg.DeltaQ0=LocationIMU_data.DeltaQ0;
// 	msg.DeltaQ1=LocationIMU_data.DeltaQ1;
// 	msg.DeltaQ2=LocationIMU_data.DeltaQ2;
// 	msg.DeltaQ3=LocationIMU_data.DeltaQ3;
// 	msg.accX=LocationIMU_data.accX;
// 	msg.accY=LocationIMU_data.accY;
// 	msg.accZ=LocationIMU_data.accZ;
// 	msg.freeAccX=LocationIMU_data.freeAccX;
// 	msg.freeAccY=LocationIMU_data.freeAccY;
// 	msg.freeAccZ=LocationIMU_data.freeAccZ;
// 	LocationIMU_mp_pub.publish(msg);
// }

//2023.6.1接入IMU--MS6111
void App::ComCore::LocationIMU_mp_Pub()
{
	//LocationIMU_mp_pub.publish(LocationIMU_wpf);
	LocationIMU_mp_pub->Write(LocationIMU_wpf);
}

void App::AppCore::setRADAR_Front(msg_radar_front* radar1)
{
	//ara::log::Logger& logger = ara::log::LogManager::defaultLogContext();
	pb_obj_radar_front = radar1;
	//1#radar
	pb_obj_radar1_to_ctrl->set_alarm_level1(pb_obj_radar_front->radar_front_second_byte_1().s_flradardisalarmlevel1() * 1 + 0);
	pb_obj_radar1_to_ctrl->set_state1(pb_obj_radar_front->radar_front_second_byte_1().s_fladarsts1() * 1 + 0);
	pb_obj_radar1_to_ctrl->set_dis1((pb_obj_radar_front->radar_front_first_byte_1().s_flradardis1_low() + (u_int16_t)(pb_obj_radar_front->radar_front_second_byte_1().s_flradardis1_high() << 8)) * 1 + 0);
	//2#radar
	pb_obj_radar1_to_ctrl->set_alarm_level2(pb_obj_radar_front->radar_front_fourth_byte_1().s_fmlradardisalarmlevel2() * 1 + 0);
	pb_obj_radar1_to_ctrl->set_state2(pb_obj_radar_front->radar_front_fourth_byte_1().s_fmladarsts2() * 1 + 0);
	pb_obj_radar1_to_ctrl->set_dis2((pb_obj_radar_front->radar_front_third_byte_1().s_fmlradardis2_low() + (u_int16_t)(pb_obj_radar_front->radar_front_fourth_byte_1().s_fmlradardis2_high() << 8)) * 1 + 0);
	//3#radar
	pb_obj_radar1_to_ctrl->set_alarm_level3(pb_obj_radar_front->radar_front_sixth_byte_1().s_fmrradardisalarmlevel3() * 1 + 0);
	pb_obj_radar1_to_ctrl->set_state3(pb_obj_radar_front->radar_front_sixth_byte_1().s_fmradarsts3() * 1 + 0);
	pb_obj_radar1_to_ctrl->set_dis3((pb_obj_radar_front->radar_front_fifth_byte_1().s_fmrradardis3_low() + (u_int16_t)(pb_obj_radar_front->radar_front_sixth_byte_1().s_fmrradardis3_high() << 8)) * 1 + 0);
	//4#radar
	pb_obj_radar1_to_ctrl->set_alarm_level4(pb_obj_radar_front->radar_front_eighth_byte_1().s_frradardisalarmlevel4() * 1 + 0);
	pb_obj_radar1_to_ctrl->set_state4(pb_obj_radar_front->radar_front_eighth_byte_1().s_frradarsts4() * 1 + 0);
	pb_obj_radar1_to_ctrl->set_dis4((pb_obj_radar_front->radar_front_seventh_byte_1().s_fradardis4_low() + (u_int16_t)(pb_obj_radar_front->radar_front_eighth_byte_1().s_fradardis4_high() << 8)) * 1 + 0);
	//radar_Pub(Radar1);
	//log
	/*
	logger.LogInfo()<<"Radar1: "
								<<", Alarm_Level1: "<< Radar1.Alarm_Level1
								<<",State1: "<< Radar1.State1
								<<",Dis1 "<< Radar1.Dis1
								<<",Alarm_Level2"<< Radar1.Alarm_Level2
								<<",State2 "<< Radar1.State2
								<<",Dis2 "<< Radar1.Dis2
								<<",Alarm_Level3 "<< Radar1.Alarm_Level3
								<<",State3 "<< Radar1.State3
								<<",Dis3 "<< Radar1.Dis3
								<<",Alarm_Level4"<< Radar1.Alarm_Level4
								<<",State4 "<< Radar1.State4
								<<",Dis4 "<< Radar1.Dis4;
								*/
}

void App::AppCore::setRADAR_Back(msg_radar_back* radar2)
{
	pb_obj_radar_back = radar2;
	//1#radar
	pb_obj_radar2_to_ctrl->set_alarm_level1(pb_obj_radar_back->radar_back_second_byte_1().s_blradardisalarmlevel1() * 1 + 0);
	pb_obj_radar2_to_ctrl->set_state1(pb_obj_radar_back->radar_back_second_byte_1().s_bladarsts1() * 1 + 0);
	pb_obj_radar2_to_ctrl->set_dis1((pb_obj_radar_back->radar_back_first_byte_1().s_blradardis1_low() + (u_int16_t)(pb_obj_radar_back->radar_back_second_byte_1().s_blradardis1_high() << 8)) * 1 + 0);
	//2#radar
	pb_obj_radar2_to_ctrl->set_alarm_level2(pb_obj_radar_back->radar_back_fourth_byte_1().s_bmlradardisalarmlevel2() * 1 + 0);
	pb_obj_radar2_to_ctrl->set_state2(pb_obj_radar_back->radar_back_fourth_byte_1().s_bmladarsts2() * 1 + 0);
	pb_obj_radar2_to_ctrl->set_dis2((pb_obj_radar_back->radar_back_third_byte_1().s_bmlradardis2_low() + (u_int16_t)(pb_obj_radar_back->radar_back_fourth_byte_1().s_bmlradardis2_high() << 8)) * 1 + 0);
	//3#radar
	pb_obj_radar2_to_ctrl->set_alarm_level3(pb_obj_radar_back->radar_back_sixth_byte_1().s_bmrradardisalarmlevel3() * 1 + 0);
	pb_obj_radar2_to_ctrl->set_state3(pb_obj_radar_back->radar_back_sixth_byte_1().s_bmradarsts3() * 1 + 0);
	pb_obj_radar2_to_ctrl->set_dis3((pb_obj_radar_back->radar_back_fifth_byte_1().s_bmrradardis3_low() + (u_int16_t)(pb_obj_radar_back->radar_back_sixth_byte_1().s_bmrradardis3_high() << 8)) * 1 + 0);
	//4#radar
	pb_obj_radar2_to_ctrl->set_alarm_level4(pb_obj_radar_back->radar_back_eighth_byte_1().s_brradardisalarmlevel4() * 1 + 0);
	pb_obj_radar2_to_ctrl->set_state4(pb_obj_radar_back->radar_back_eighth_byte_1().s_brradarsts4() * 1 + 0);
	pb_obj_radar2_to_ctrl->set_dis4((pb_obj_radar_back->radar_back_seventh_byte_1().s_bradardis4_low()+ (u_int16_t)(pb_obj_radar_back->radar_back_eighth_byte_1().s_bradardis4_high() << 8)) * 1 + 0);
	//radar_Pub(Radar2);
	//logs
	/*
	ara::log::Logger& logger = ara::log::LogManager::defaultLogContext();
	logger.LogInfo()<<"Radar2: "
								<<", Alarm_Level1: "<< Radar2.Alarm_Level1
								<<",State1: "<< Radar2.State1
								<<",Dis1 "<< Radar2.Dis1
								<<",Alarm_Level2"<< Radar2.Alarm_Level2
								<<",State2 "<< Radar2.State2
								<<",Dis2 "<< Radar2.Dis2
								<<",Alarm_Level3 "<< Radar2.Alarm_Level3
								<<",State3 "<< Radar2.State3
								<<",Dis3 "<< Radar2.Dis3
								<<",Alarm_Level4"<< Radar2.Alarm_Level4
								<<",State4 "<< Radar2.State4
								<<",Dis4 "<< Radar2.Dis4;
	*/
}

void App::AppCore::setRADAR_Side1(msg_radar_side1* radar3)
{
	pb_obj_radar_side1 = radar3;
	//1#radar
	pb_obj_radar3_to_ctrl->set_alarm_level1(pb_obj_radar_side1->radar_side1_second_byte_1().s_sfl_1_radardisalarmlevel1() * 1 + 0);
	pb_obj_radar3_to_ctrl->set_state1(pb_obj_radar_side1->radar_side1_second_byte_1().s_sfl_1_radarsts1() * 1 + 0);
	pb_obj_radar3_to_ctrl->set_dis1((pb_obj_radar_side1->radar_side1_first_byte_1().s_sfl_1_radardis1_low() + (u_int16_t)(pb_obj_radar_side1->radar_side1_second_byte_1().s_sfl_1_radardis1_high() << 8)) * 1 + 0);
	//2#radar
	pb_obj_radar3_to_ctrl->set_alarm_level2(pb_obj_radar_side1->radar_side1_fourth_byte_1().s_sfr_1_radardisalarmlevel2() * 1 + 0);
	pb_obj_radar3_to_ctrl->set_state2(pb_obj_radar_side1->radar_side1_fourth_byte_1().s_sfr_1_radarsts2()* 1 + 0);
	pb_obj_radar3_to_ctrl->set_dis2((pb_obj_radar_side1->radar_side1_third_byte_1().s_sfr_1_radardis2_low() + (u_int16_t)(pb_obj_radar_side1->radar_side1_fourth_byte_1().s_sfr_1_radardis2_high() << 8)) * 1 + 0);
	//3#radar
	pb_obj_radar3_to_ctrl->set_alarm_level3(pb_obj_radar_side1->radar_side1_sixth_byte_1().s_sbl_1_radardisalarmlevel3() * 1 + 0);
	pb_obj_radar3_to_ctrl->set_state3(pb_obj_radar_side1->radar_side1_sixth_byte_1().s_sbl_1_radarsts3() * 1 + 0);
	pb_obj_radar3_to_ctrl->set_dis3((pb_obj_radar_side1->radar_side1_fifth_byte_1().s_sbl_1_radardis3_low() + (u_int16_t)(pb_obj_radar_side1->radar_side1_sixth_byte_1().s_sbl_1_radardis3_high() << 8)) * 1 + 0);
	//4#radar
	pb_obj_radar3_to_ctrl->set_alarm_level4(pb_obj_radar_side1->radar_side1_eighth_byte_1().s_sbr_1_radardisalarmlevel4() * 1 + 0);
	pb_obj_radar3_to_ctrl->set_state4(pb_obj_radar_side1->radar_side1_eighth_byte_1().s_sbr_1_radarsts4() * 1 + 0);
	pb_obj_radar3_to_ctrl->set_dis4((pb_obj_radar_side1->radar_side1_seventh_byte_1().s_sbr_1_radardis4_low() + (u_int16_t)(pb_obj_radar_side1->radar_side1_eighth_byte_1().s_sbr_1_radardis4_high() << 8)) * 1 + 0);
	//radar_Pub(Radar3);
	//log
	/*
	ara::log::Logger& logger = ara::log::LogManager::defaultLogContext();
	logger.LogInfo()<<"Radar3: "
								<<", Alarm_Level1: "<< Radar3.Alarm_Level1
								<<",State1: "<< Radar3.State1
								<<",Dis1 "<< Radar3.Dis1
								<<",Alarm_Level2"<< Radar3.Alarm_Level2
								<<",State2 "<< Radar3.State2
								<<",Dis2 "<< Radar3.Dis2
								<<",Alarm_Level3 "<< Radar3.Alarm_Level3
								<<",State3 "<< Radar3.State3
								<<",Dis3 "<< Radar3.Dis3
								<<",Alarm_Level4"<< Radar3.Alarm_Level4
								<<",State4 "<< Radar3.State4
								<<",Dis4 "<< Radar3.Dis4;
								*/
}

void App::AppCore::setRADAR_Side2(msg_radar_side2* radar4)
{
	pb_obj_radar_side2 = radar4;
	//1#radar
	pb_obj_radar4_to_ctrl->set_alarm_level1(pb_obj_radar_side2->radar_side2_second_byte_1().s_sfl_2_radardisalarmlevel1() * 1 + 0);
	pb_obj_radar4_to_ctrl->set_state1(pb_obj_radar_side2->radar_side2_second_byte_1().s_sfl_2_radarsts1() * 1 + 0);
	pb_obj_radar4_to_ctrl->set_dis1((pb_obj_radar_side2->radar_side2_first_byte_1().s_sfl_2_radardis1_low() + (u_int16_t)(pb_obj_radar_side2->radar_side2_second_byte_1().s_sfl_2_radardis1_high() << 8)) * 1 + 0);
	//2#radar
	pb_obj_radar4_to_ctrl->set_alarm_level2(pb_obj_radar_side2->radar_side2_fourth_byte_1().s_sfr_2_radardisalarmlevel2() * 1 + 0);
	pb_obj_radar4_to_ctrl->set_state2(pb_obj_radar_side2->radar_side2_fourth_byte_1().s_sfr_2_radarsts2() * 1 + 0);
	pb_obj_radar4_to_ctrl->set_dis2((pb_obj_radar_side2->radar_side2_third_byte_1().s_sfr_2_radardis2_low() + (u_int16_t)(pb_obj_radar_side2->radar_side2_fourth_byte_1().s_sfr_2_radardis2_high() << 8)) * 1 + 0);
	//3#radar
	pb_obj_radar4_to_ctrl->set_alarm_level3(pb_obj_radar_side2->radar_side2_sixth_byte_1().s_sbl_2_radardisalarmlevel3() * 1 + 0);
	pb_obj_radar4_to_ctrl->set_state3(pb_obj_radar_side2->radar_side2_sixth_byte_1().s_sbl_2_radarsts3() * 1 + 0);
	pb_obj_radar4_to_ctrl->set_dis3((pb_obj_radar_side2->radar_side2_fifth_byte_1().s_sbl_2_radardis3_low() + (u_int16_t)(pb_obj_radar_side2->radar_side2_sixth_byte_1().s_sbl_2_radardis3_high() << 8)) * 1 + 0);
	//4#radar
	pb_obj_radar4_to_ctrl->set_alarm_level4(pb_obj_radar_side2->radar_side2_eighth_byte_1().s_sbr_2_radardisalarmlevel4() * 1 + 0);
	pb_obj_radar4_to_ctrl->set_state4(pb_obj_radar_side2->radar_side2_eighth_byte_1().s_sbr_2_radarsts4() * 1 + 0);
	pb_obj_radar4_to_ctrl->set_dis4((pb_obj_radar_side2->radar_side2_seventh_byte_1().s_sbr_2_radardis4_low() + (u_int16_t)(pb_obj_radar_side2->radar_side2_eighth_byte_1().s_sbr_2_radardis4_high() << 8)) * 1 + 0);
	//radar_Pub(Radar4);
	//log
	/*
	ara::log::Logger& logger = ara::log::LogManager::defaultLogContext();
	logger.LogInfo()<<"Radar4: "
								<<", Alarm_Level1: "<< Radar4.Alarm_Level1
								<<",State1: "<< Radar4.State1
								<<",Dis1 "<< Radar4.Dis1
								<<",Alarm_Level2"<< Radar4.Alarm_Level2
								<<",State2 "<< Radar4.State2
								<<",Dis2 "<< Radar4.Dis2
								<<",Alarm_Level3 "<< Radar4.Alarm_Level3
								<<",State3 "<< Radar4.State3
								<<",Dis3 "<< Radar4.Dis3
								<<",Alarm_Level4"<< Radar4.Alarm_Level4
								<<",State4 "<< Radar4.State4
								<<",Dis4 "<< Radar4.Dis4;
								*/
}

void App::ComCore::canmsgsend(const ros::TimerEvent& event)
{
	App::AppCore ac;
	//send msg to other modules
	//dfcv_mining_msgs::CAN_TO_CTRL_EBS can_to_ctrl_ebs_msg;
	auto can_to_ctrl_ebs_msg = std::make_shared<dfcv_mining_msgs::CAN_TO_CTRL_EBS>();
	//dfcv_mining_msgs::CAN_TO_CTRL_HCU can_to_ctrl_hcu_msg;
	auto can_to_ctrl_hcu_msg = std::make_shared<dfcv_mining_msgs::CAN_TO_CTRL_HCU>();
	//dfcv_mining_msgs::CAN_TO_CTRL_TCU can_to_ctrl_tcu_msg;
	auto can_to_ctrl_tcu_msg = std::make_shared<dfcv_mining_msgs::CAN_TO_CTRL_TCU>();
	//dfcv_mining_msgs::CAN_TO_CTRL_EPS can_to_ctrl_eps_msg;
	auto can_to_ctrl_eps_msg = std::make_shared<dfcv_mining_msgs::CAN_TO_CTRL_EPS>();
	//dfcv_mining_msgs::CAN_TO_CTRL_EPB1 can_to_ctrl_epb1_msg;
	auto can_to_ctrl_epb1_msg = std::make_shared<dfcv_mining_msgs::CAN_TO_CTRL_EPB1>();
	//dfcv_mining_msgs::System_ADModeEx System_ADModeEx_msg;//4.28
	auto System_ADModeEx_msg = std::make_shared<dfcv_mining_msgs::System_ADModeEx>();
//------------------------CAN_TO_CTRL_EBS-------------------------------
	can_to_ctrl_ebs_msg.VehDa_mWght_mp = pb_obj_ebs_to_ctrl->vehda_mwght_mp(); //EMPTY
	can_to_ctrl_ebs_msg.VehDa_rBrkPedl_mp = pb_obj_ebs_to_ctrl->vehda_rbrkpedl_mp();
	can_to_ctrl_ebs_msg.VehDa_vEgoSpd_mp = pb_obj_ebs_to_ctrl->vehda_vegospd_mp();
	can_to_ctrl_ebs_msg.VehDa_stSrcBrk_mp = pb_obj_ebs_to_ctrl->vehda_stsrcbrk_mp();
	can_to_ctrl_ebs_msg.VehDa_stBrkPedl_mp = pb_obj_ebs_to_ctrl->vehda_stbrkpedl_mp();
//------------------------CAN_TO_CTRL_HCU-----------------------------
	can_to_ctrl_hcu_msg.VehDa_nEngSpd_mp = pb_obj_eecu_to_ctrl->vehda_nengspd_mp();
	can_to_ctrl_hcu_msg.VehDa_prcActuTrq_mp = pb_obj_eecu_to_ctrl->vehda_prcactutrq_mp();
	can_to_ctrl_hcu_msg.VehDa_prcDrvrDmdTrq_mp = pb_obj_eecu_to_ctrl->vehda_prcdrvrdmdtrq_mp();
	can_to_ctrl_hcu_msg.VehDa_prcTrqEngNomFric_mp = pb_obj_eecu_to_ctrl->vehda_prctrqengnomfric_mp();
	can_to_ctrl_hcu_msg.VehDa_prcTrqEstimdLoss_mp = 0; //EECU_To_Ctrl_.VehDa_prcTrqEstimdLoss_mp;
	can_to_ctrl_hcu_msg.VehDa_stCluSwt_mp = pb_obj_eecu_to_ctrl->vehda_stcluswt_mp();
	can_to_ctrl_hcu_msg.VehDa_stSrcEngCtrl_mp = pb_obj_eecu_to_ctrl->vehda_stsrcengctrl_mp();
	can_to_ctrl_hcu_msg.VehDa_rAccrPedl_mp = pb_obj_eecu_to_ctrl->vehda_raccrpedl_mp();
//-----------------------CAN_TO_CTRL_TCU-------------------------------
	can_to_ctrl_tcu_msg.VehDa_rTraCurGear_mp = pb_obj_tcu_to_ctrl->vehda_rtracurgear_mp();
	can_to_ctrl_tcu_msg.VehDa_stTraCurGear_mp = pb_obj_tcu_to_ctrl->vehda_sttracurgear_mp();
	logger.LogInfo()<<"-----------TTTTTTT--------TCU_To_Ctrl_.VehDa_stTraCurGear_mp: ------------TTTTTTT-----------"<<pb_obj_tcu_to_ctrl->vehda_sttracurgear_mp();
	can_to_ctrl_tcu_msg.VehDa_stTraSelGear_mp = pb_obj_tcu_to_ctrl->vehda_sttraselgear_mp();
	can_to_ctrl_tcu_msg.VehDa_stTraTrqLim_mp = pb_obj_tcu_to_ctrl->vehda_sttratrqlim_mp();
	can_to_ctrl_tcu_msg.VehDa_prcTraTrqLim_mp = pb_obj_tcu_to_ctrl->vehda_prctratrqlim_mp();
	can_to_ctrl_tcu_msg.VehDa_stTraSht_mp = pb_obj_tcu_to_ctrl->vehda_sttrasht_mp();
	can_to_ctrl_tcu_msg.VehDa_stTraEgd_mp = pb_obj_tcu_to_ctrl->vehda_sttraegd_mp();
//-----------------------CAN_TO_CTRL_EPS-------------------------------
	can_to_ctrl_eps_msg.DE_phiSteerSpd = pb_obj_eps_to_ctrl->de_phisteerspd();
	can_to_ctrl_eps_msg.DE_phiSteerAngle = pb_obj_eps_to_ctrl->de_phisteerangle();
	can_to_ctrl_eps_msg.DE_SteerToruqe = pb_obj_eps_to_ctrl->de_steertoruqe();
	can_to_ctrl_eps_msg.DE_HandTorque = pb_obj_eps_to_ctrl->de_handtorque();
	can_to_ctrl_eps_msg.DE_EpsMode = pb_obj_eps_to_ctrl->de_epsmode();
	can_to_ctrl_eps_msg.DE_EpsERRCode = pb_obj_eps_to_ctrl->de_epserrcode();///msg no DE_EpsERRCode
//----------------------------CAN_TO_CTRL_EPB1-----------------------
	can_to_ctrl_epb1_msg.EPB_ParkBrKSt = pb_obj_epb1_to_ctrl->epb_parkbrkst();
	//sys_to_plan_msg.TransmissionMode3 = TC_1_.transmissionmode_3;
	System_ADModeEx_msg.System_TransmissionMode3 = pb_obj_etc7_to_ctrl->etc7_to_ctrl_third_byte_1().transmission_mode_3();//4.28
	//logger.LogInfo()<<"-------------------transmissionmode_3: -----------------------"<<TC_1_.transmissionmode_3;
    //engage_mode = 0;
    //enable_mode &= ; //(inter_info2.CtrlModResp_Overridedetection == 0) |
	//engage_mode = (unsigned int)((TCU_To_Ctrl_.VehDa_stTraCurGear_mp > 0) && (TCU_To_Ctrl_.VehDa_stTraCurGear_mp != 126));

	//logger.LogInfo()<<"activated: " << engage_mode << "gear: " << TCU_To_Ctrl_.VehDa_stTraCurGear_mp
	//		<< "break: " << inter_ebc1.EBS_brake_switch << "o: " <<inter_info2.CtrlModResp_Overridedetection;
	ac.SYS_StateChange();

	if(engage_mode == 0)
	{
		plat_ctrl = false;
	}

	//Sys_stADMd_mp_msg.Sys_stADMd_mp = enable_mode * 3; //engage_mode * 2 + enable_mode;
	//Sys_stADMd_mp_msg.Sys_stADMd_mp = engage_mode * 2 + enable_mode;
	System_ADModeEx_msg.System_ADMode = engage_mode * 2 + enable_mode;//4.28

	//CAN_TO_CTRL_EBS_pub.publish(can_to_ctrl_ebs_msg);
	CAN_TO_CTRL_EBS_pub->Write(can_to_ctrl_ebs_msg);
	//CAN_TO_CTRL_HCU_pub.publish(can_to_ctrl_hcu_msg);
	CAN_TO_CTRL_HCU_pub->Write(can_to_ctrl_hcu_msg);
	//CAN_TO_CTRL_TCU_pub.publish(can_to_ctrl_tcu_msg);
	CAN_TO_CTRL_TCU_pub->Write(can_to_ctrl_tcu_msg);
	//CAN_TO_CTRL_EPS_pub.publish(can_to_ctrl_eps_msg);
	CAN_TO_CTRL_EPS_pub->Write(can_to_ctrl_eps_msg);
	//CAN_TO_CTRL_EPB1_pub.publish(can_to_ctrl_epb1_msg);
	CAN_TO_CTRL_EPB1_pub->Write(can_to_ctrl_epb1_msg);
	//System_ADModeEx_pub.publish(System_ADModeEx_msg);  //5.5
	System_ADModeEx_pub->Write(System_ADModeEx_msg);
	//CAN_TO_CTRL_TC_1_pub.publish(sys_to_plan_msg);//5.5
	ac.setEngageMode();
	ros2tbox();
	//cantest();
	//publishTboxMsg();
	//sendTbox
}

void App::ComCore::BehaviorChange(const ros::TimerEvent& event)
{
	//App::ComCore cc;
	App::AppCore ac;
	logger.LogInfo()<<"=========there is no position data=========="<<Adt_position_sub.getNumPublishers();

	if (Adt_position_sub.getNumPublishers() == 0)
	{
		logger.LogInfo()<<"=========there is no position data==========";
	}
	if ((pb_obj_vechicle_cmd->bhvcrdn_numbhvid() == 11) && (last_BhvCrdn_numBhvID != 11))
	{
		ros_time_start = ros::Time::now().toSec();
	}
	if ((pb_obj_vechicle_cmd->bhvcrdn_numbhvid() == 11) && (last_BhvCrdn_numBhvID == 11))
	{
		ros_time_end = ros::Time::now().toSec();
	}

	ros_time = ros_time_end - ros_time_start;

	if ((ros_time >= 30) && (ros_time <= 1000))
	{
		task_error = true;
	}
	else{
		task_error = false;
	}

//	logger.LogInfo()<<"-------------------------------ros_time_start--------------------------------" << ros_time_start;
//	logger.LogInfo()<<"-------------------------------ros_time_end--------------------------------" << ros_time_end;
//	logger.LogInfo()<<"-------------------------------task_error--------------------------------" << task_error;
//	logger.LogInfo()<<"-------------------------------hasTBoxMove--------------------------------" << hasTBoxMove;

	if(hasTBoxMove == true)
	{
		tboxMoveReply(hasTBoxMove, pb_obj_vechicle_cmd->bhvcrdn_numbhvid(), pb_obj_tbox_move->questtype());

		if ((pb_obj_vechicle_cmd->bhvcrdn_numbhvid() == 11) && (arrive_end_point == true))
		{
			hasTBoxMove = false;
		}
	}

	if(hasTBoxJoint == true)
	{
		tboxJointReply(hasTBoxJoint, pb_obj_vechicle_cmd->bhvcrdn_numbhvid(), pb_obj_tbox_move->questtype());
		sleep(2);

		if (pb_obj_vechicle_cmd->bhvcrdn_numbhvid() == 11)
		{
			hasTBoxJoint = false;
		}
	}

	if(hasTBoxDrop == true)
	{
		tboxDropReply(hasTBoxDrop, Ctrl_lift_state, ac.pb_obj_tbox_drop->questtype());
		if (Ctrl_lift_state == 1)
		{
			hasTBoxDrop = false;
		}
	}
	 node_ros_time = ros::Time::now().toSec();
//	 logger.LogInfo()<<"--------node_ros_time ++++++++++++++++++"<<node_ros_time*1000;
//	 logger.LogInfo()<<"--------abs(node_ros_time-Planning_time_now) ++++++++++++++++++"<<abs(node_ros_time-Planning_time_now);
//	 logger.LogInfo()<<"--------abs(node_ros_time-Location_time_now) ++++++++++++++++++"<<abs(node_ros_time-Location_time_now);
//	 logger.LogInfo()<<"--------abs(node_ros_time-LatCtl_time_now) ++++++++++++++++++"<<abs(node_ros_time-LatCtl_time_now);
//	 logger.LogInfo()<<"--------abs(node_ros_time-LgtCtl_time_now) ++++++++++++++++++"<<abs(node_ros_time-LgtCtl_time_now);
	 //	if(((abs(node_ros_time-Location_time_now)<2)) && (abs(node_ros_time-Planning_time_now)<2)&& (abs(node_ros_time-LatCtl_time_now)<2)&& (abs(node_ros_time-LgtCtl_time_now)<2)){
		node_state = true;
//	}else{
//		node_state=false;
//	}
	//logger.LogInfo()<<"--------node_state ++++++++++++++++++"<<node_state;
}

void App::AppCore::cantest()
{
	std::string INPUT_CAN_DATA = "0xCEF189E#11.22.33.44.55.66.77.88";
	uint8_t channelId = A_CAN_CHANNEL_ID;
	uint8_t canSendType = 0;
	radar3ApInterfaceNode.DataParseFromInput(INPUT_CAN_DATA, canSendType);
	radar3ApInterfaceNode.CanFdDataMethodSend(channelId);
}

//3.15添加自動駕駛狀態切換
void App::AppCore::SYS_StateChange()
{
	//AD_mode admode;
	bool UltrasionicRadarSts = true;
	bool EoneSts = true;
	bool LidarECUSts = true;
	bool ForwardCameraSts = true;
	bool BMS_Sts = true;
	bool HCCB_Sts = true;
	bool MCU_Sts = true;
	bool TPMS_Sts = true;
	bool EBS_Sts = true;
	bool EPS_Sts = true;
	bool BCM_Sts = true;
	bool ADCU_POWER_Status = true;
	bool FishCameraSts = true;
	//bool keyAutoDriv=true;
	//bool remoteCtr=false;
	//bool PeopleControl=true;
	//logger.LogInfo()<<"admode" << admode ;
	switch (admode)
	{
		case AD_Off:
			if ((UltrasionicRadarSts == true) && (EoneSts == true) && (LidarECUSts == true)
			&& (ForwardCameraSts == true) && (BMS_Sts == true) && (HCCB_Sts == true)
			&& (MCU_Sts == true) && (TPMS_Sts == true) && (EBS_Sts == true)
			&& (EPS_Sts == true) && (BCM_Sts == true) && (ADCU_POWER_Status == true) && (enable_mode == 1))
			{
				admode = AD_Ready;
			}
			else
			{
				admode = AD_Off;
			}
		break;

		case AD_Ready:
			if (((pb_obj_tcu_to_ctrl->vehda_sttracurgear_mp() != 0) && (node_state == true) && (switchOn))||(auto_state == 1))
			{
				admode = AD_Run;
				engage_mode = 1;
			}
			else if((UltrasionicRadarSts == false) || (EoneSts == false) || (LidarECUSts == false)
			|| (ForwardCameraSts == false) || (BMS_Sts == false) || (HCCB_Sts == false)
			|| (MCU_Sts == false) || (TPMS_Sts == false) || (EBS_Sts == false)
			|| (EPS_Sts == false) || (BCM_Sts == false) || (ADCU_POWER_Status == false))
			{
				admode = AD_Off;
				enable_mode = 0;
			}
			else
			{
				admode = AD_Ready;
			}
		break;

		case AD_Run:
			if((UltrasionicRadarSts == false) || (FishCameraSts == false))
			{
				admode = AD_SafeRun;
			}
			else if((!switchOn) || (pb_obj_ebc1_ebs->ebc1_ebs_first_byte_1().ebs_brake_switch() != 0) || (pb_obj_ctlinfo2_eps1_kb->ctlinfo2_eps1_kb_first_byte_1().ctrlmodresp_overridedetection() == 1) ||(node_state == false))
			{
				admode = AD_Ready;
				//enable_mode=1;
				engage_mode = 0;
				manual = true;
			}
			else if((UltrasionicRadarSts == false) || (EoneSts == false) || (LidarECUSts==false)
					|| (ForwardCameraSts == false) || (BMS_Sts == false) || (HCCB_Sts == false)
					|| (MCU_Sts == false) || (TPMS_Sts == false) || (EBS_Sts == false)
					|| (EPS_Sts == false) || (BCM_Sts == false) || (ADCU_POWER_Status == false))
			{
				admode = AD_Off;
				enable_mode = 0;
			}
			else
			{
				admode = AD_Run;
			}
		break;

		case AD_SafeRun:
			if(true == manual)
			{
				admode = AD_Off;
				engage_mode = 0;
			}
			else
			{
				admode = AD_Off;
				engage_mode = 0;
			}
		break;

		default:
			admode = AD_Off;
		break;
	}
	logger.LogInfo()<<"engage_mode" << engage_mode<<"switdhOn"<<switchOn <<"admode"<<admode<<"VehDa_stTraCurGear_mp"<<pb_obj_tcu_to_ctrl->vehda_sttracurgear_mp();

}

void App::AppCore::setBodyControl_ADCU(msg_body_control_adcu* BodyControl_ADCU)
{
	pb_obj_body_control_adcu = BodyControl_ADCU;
	pb_obj_bodycontrol_adcu_to_ctrl->set_gashornactivationswitch(pb_obj_body_control_adcu->body_control_adcu_first_byte_1().gashornactivationswitch() * 1 + 0);
	pb_obj_bodycontrol_adcu_to_ctrl->set_electrichornactivationswitch(pb_obj_body_control_adcu->body_control_adcu_first_byte_1().electrichornactivationswitch() * 1 + 0);
	pb_obj_bodycontrol_adcu_to_ctrl->set_liftswitch(pb_obj_body_control_adcu->body_control_adcu_fifth_byte_1().liftswitch() * 1 + 0);
	//	logger.LogInfo()<<"BodyControl_ADCU: "
	//								<<", GasHornActivationSwitch: "<< BodyControl_ADCU_To_Ctrl_.GasHornActivationSwitch
	//								<<",ElectricHornActivationSwitch: "<< BodyControl_ADCU_To_Ctrl_.ElectricHornActivationSwitch
	//								<<",LiftSwitch "<< BodyControl_ADCU_To_Ctrl_.LiftSwitch;
}
	
void App::AppCore::setEPBS1_EPB(msg_epbs1_epb* EPBS1_EPB)
{	
	App::ComCore cc;
	pb_obj_epbs1_epb = EPBS1_EPB;
	pb_obj_epbs1_epb_to_ctrl->set_epb_parkbrastatus(pb_obj_epbs1_epb->epbs1_epb_first_byte_1().epb_parkbrastatus() * 1 + 0);
	pb_obj_epbs1_epb_to_ctrl->set_ah_parkactive(pb_obj_epbs1_epb->epbs1_epb_first_byte_1().ah_parkactive() * 1 + 0);
	pb_obj_epbs1_epb_to_ctrl->set_ah_brastatus(pb_obj_epbs1_epb->epbs1_epb_first_byte_1().ah_brastatus() * 1 + 0);
	pb_obj_epbs1_epb_to_ctrl->set_epb_workmode(pb_obj_epbs1_epb->epbs1_epb_first_byte_1().epb_workmode() * 1 + 0);
	pb_obj_epbs1_epb_to_ctrl->set_parkbtswitch(pb_obj_epbs1_epb->epbs1_epb_fourth_byte_1().parkbtswitch() * 1 + 0);
	pb_obj_epbs1_epb_to_ctrl->set_relsbtswitch(pb_obj_epbs1_epb->epbs1_epb_fifth_byte_1().relsbtswitch() * 1 + 0);
	pb_obj_epbs1_epb_to_ctrl->set_ahbtswitch(pb_obj_epbs1_epb->epbs1_epb_fifth_byte_1().ahbtswitch() * 1 + 0);
	pb_obj_epbs1_epb_to_ctrl->set_sysstatus(pb_obj_epbs1_epb->epbs1_epb_sixth_byte_1().sysstatus() * 1 + 0);
	//	logger.LogInfo()<<"EPBS1_EPB: "
	//								<<", EPB_ParkBraStatus: "<< EPBS1_EPB_To_Ctrl_.EPB_ParkBraStatus
	//								<<",AH_ParkActive: "<< EPBS1_EPB_To_Ctrl_.AH_ParkActive
	//								<<",AH_BraStatus "<< EPBS1_EPB_To_Ctrl_.AH_BraStatus
	//								<<",EPB_WorkMode: "<< EPBS1_EPB_To_Ctrl_.EPB_WorkMode
	//								<<",ParkBtSwitch "<< EPBS1_EPB_To_Ctrl_.ParkBtSwitch
	//								<<",RelsBtSwitch: "<< EPBS1_EPB_To_Ctrl_.RelsBtSwitch
	//								<<",AHBtSwitch "<< EPBS1_EPB_To_Ctrl_.AHBtSwitch
	//								<<",SysStatus "<< EPBS1_EPB_To_Ctrl_.SysStatus;
	cc.EPBS1_EPB_Pub(*pb_obj_epbs1_epb_to_ctrl);
}
	
void App::AppCore::setEPBC1_ADU(msg_epbc1_adu* EPBC1_ADU)
{	
	App::ComCore cc;
	pb_obj_epbc1_adu = EPBC1_ADU;
	pb_obj_epbc1_adu_to_ctrl->set_externalbrakecontrol(pb_obj_epbc1_adu->externalbrakecontrol() * 1 + 0);
	pb_obj_epbc1_adu_to_ctrl->set_tractorbrakecontrol(pb_obj_epbc1_adu->tractorbrakecontrol() * 1 + 0);
	pb_obj_epbc1_adu_to_ctrl->set_trailerbrakecontrol(pb_obj_epbc1_adu->trailerbrakecontrol() * 1 + 0);
	pb_obj_epbc1_adu_to_ctrl->set_parkingbrakecontrol(pb_obj_epbc1_adu->parkingbrakecontrol() * 1 + 0);
	pb_obj_epbc1_adu_to_ctrl->set_autoholdbrakecontrol(pb_obj_epbc1_adu->autoholdbrakecontrol() * 1 + 0);
	pb_obj_epbc1_adu_to_ctrl->set_workingmode(pb_obj_epbc1_adu->workingmode() * 1 + 0);
	pb_obj_epbc1_adu_to_ctrl->set_csum(pb_obj_epbc1_adu->csum() * 1 + 0);
	
//	logger.LogInfo()<<"EPBC1_ADU: "
//								<<", ExternalBrakeControl: "<< EPBC1_ADU_To_Ctrl_.ExternalBrakeControl
//								<<",TractorBrakeControl: "<< EPBC1_ADU_To_Ctrl_.TractorBrakeControl
//								<<",TrailerBrakeControl "<< EPBC1_ADU_To_Ctrl_.TrailerBrakeControl
//								<<",ParkingBrakeControl: "<< EPBC1_ADU_To_Ctrl_.ParkingBrakeControl
//								<<",AutoholdBrakeControl "<< EPBC1_ADU_To_Ctrl_.AutoholdBrakeControl
//								<<",WorkingMode: "<< EPBC1_ADU_To_Ctrl_.WorkingMode
//								<<",CSUM "<< EPBC1_ADU_To_Ctrl_.CSUM;
	cc.EPBC1_ADU_Pub(*pb_obj_epbc1_adu_to_ctrl);
}
	
void App::AppCore::setPTODE(msg_ptode* PTODE)
{	
	App::ComCore cc;
	pb_obj_ptode = PTODE;
	pb_obi_ptode_to_ctrl->set_enableswitch_transmissioninputshaftpto_1(pb_obj_ptode->ptode_first_byte_1().enableswitch_transmissioninputshaftpto_1() * 1 + 0);
	pb_obi_ptode_to_ctrl->set_enableswitch_transmissioninputshaftpto_2(pb_obj_ptode->ptode_first_byte_1().enableswitch_transmissioninputshaftpto_2() * 1 + 0);
//	logger.LogInfo()<<"PTODE: "
//								<<", EnableSwitch_TransmissioninputshaftPTO_1: "<< PTODE_To_Ctrl_.EnableSwitch_TransmissionInputShaftPTO_1
	//							<<",EnableSwitch_TransmissioninputshaftPTO_2: "<< PTODE_To_Ctrl_.EnableSwitch_TransmissionInputShaftPTO_2;
	cc.PTODE_Pub(*pb_obi_ptode_to_ctrl);
}
	
void App::AppCore::setPTODE_ADCU(msg_ptode_adcu* PTODE_ADCU)
{	
	App::ComCore cc;
	pb_obj_ptode_adcu = PTODE_ADCU;
	pb_obi_ptode_adcu_to_ctrl->set_enableswitch_transmissioninputshaftpto_1(pb_obj_ptode_adcu->ptode_adcu_first_byte_1().enableswitch_transmissioninputshaftpto_1() * 1 + 0);
	pb_obi_ptode_adcu_to_ctrl->set_enableswitch_transmissioninputshaftpto_2(pb_obj_ptode_adcu->ptode_adcu_first_byte_1().enableswitch_transmissioninputshaftpto_2() * 1 + 0);
//	logger.LogInfo()<<"PTODE_ADCU: "
//								<<", EnableSwitch_TransmissioninputshaftPTO_1: "<< PTODE_ADCU_To_Ctrl_.EnableSwitch_TransmissionInputShaftPTO_1
//								<<",EnableSwitch_TransmissioninputshaftPTO_2: "<< PTODE_ADCU_To_Ctrl_.EnableSwitch_TransmissionInputShaftPTO_2;
	cc.PTODE_ADCU_Pub(*pb_obi_ptode_adcu_to_ctrl);
}

void App::AppCore::setEEC1_E(msg_eec1_e* eec1_e)
{	
	pb_obj_eec1_e = eec1_e;
	//inter_eec1 = *eec1_e;
	//EECU_To_Ctrl_.VehDa_prcActuTrq_mp = inter_eec1.Actual_engine_torque * 1 - 125;
	//EECU_To_Ctrl_.VehDa_prcDrvrDmdTrq_mp = inter_eec1.Driver_demand_engine_torque * 1 -125;
	pb_obj_eecu_to_ctrl->set_vehda_stsrcengctrl_mp(pb_obj_eec1_e->address_of_controlling_device() * 1 + 0);
	//EECU_To_Ctrl_.VehDa_stSrcEngCtrl_mp = inter_eec1.address_of_controlling_device * 1 + 0;
	pb_obj_eecu_to_ctrl->set_vehda_nengspd_mp((pb_obj_eec1_e->engine_speed_low() + (u_int16_t)(pb_obj_eec1_e->engine_speed_high() << 8)) * 0.125 + 0);
}

void App::AppCore::setEEC2_E(msg_eec2_e* eec2_e)
{
	pb_obj_eec2_e = eec2_e;
	pb_obj_eecu_to_ctrl->set_vehda_raccrpedl_mp((double)(pb_obj_eec2_e->accelerator_pedal_position())*0.4 + 0);
}

void App::AppCore::setETC1_TCU(msg_etc1_tcu* etc1)
{
	pb_obj_etc1_tcu = etc1;
	pb_obj_tcu_to_ctrl->set_vehda_sttraegd_mp(pb_obj_etc1_tcu->etc1_tcu_first_byte_1().transmission_driveline_engaged()); //ETC1:传动系统结合状态
	//TCU_To_Ctrl_.VehDa_stTraEgd_mp = pb_obj_etc1_tcu->transmission_driveline_engaged(); //ETC1:传动系统结合状态
	pb_obj_tcu_to_ctrl->set_vehda_sttrasht_mp(pb_obj_etc1_tcu->etc1_tcu_first_byte_1().transmission_shift_in_process()); //ETC1:换挡状态
}

void App::AppCore::setETC2_TCU(msg_etc2_tcu* etc2)
{
	pb_obj_etc2_tcu = etc2;
    //按位翻转
	pb_obj_tcu_to_ctrl->set_vehda_rtracurgear_mp((double)((u_int16_t)pb_obj_etc2_tcu->transmission_actual_gear_ratio_low() + (u_int16_t)(pb_obj_etc2_tcu->transmission_actual_gear_ratio_high() << 8)) * 0.001 ); //ETC2：变速箱当前档位传动比
	pb_obj_tcu_to_ctrl->set_vehda_sttracurgear_mp((int16_t)pb_obj_etc2_tcu->current_gear() - 125); //ETC2：变速箱当前档位
	pb_obj_tcu_to_ctrl->set_vehda_sttraselgear_mp((int16_t)pb_obj_etc2_tcu->selected_gear() - 125); //ETC2：选择变速箱档位（2021.7.14更新，新增信号）
}

void App::AppCore::setEBC1_EBS(msg_ebc1_ebs* ebc1)
{
	pb_obj_ebc1_ebs = ebc1;
	pb_obj_ebs_to_ctrl->set_vehda_rbrkpedl_mp((double)(pb_obj_ebc1_ebs->brake_pedal_position())* 0.4 + 0); //EBC1：制动踏板位置百分比
	pb_obj_ebs_to_ctrl->set_vehda_stbrkpedl_mp(pb_obj_ebc1_ebs->ebc1_ebs_first_byte_1().ebs_brake_switch() * 1 + 0); //EBC1：制动踏板状态
	pb_obj_ebs_to_ctrl->set_vehda_stsrcbrk_mp(pb_obj_ebc1_ebs->source_address_of_controlling_device_for_brake_control() * 1 + 0); //EBC1:制动控制设备源地址
	if(pb_obj_ebc1_ebs->ebc1_ebs_first_byte_1().ebs_brake_switch() != 0)
	{
		switchOn = false;
	}
}

void App::AppCore::setEBC2_EBS(msg_ebc2_ebs* ebc2)
{
	pb_obj_ebc2_ebs = ebc2;
	pb_obj_ebs_to_ctrl->set_vehda_vegospd_mp((double)(pb_obj_ebc2_ebs->front_axle_speed_low() + (u_int16_t)(pb_obj_ebc2_ebs->front_axle_speed_high() << 8)) * 1/256 + 0); //EBC2：前轴车速
}

void App::AppCore::setCTLINFO1_EPS1_KB(msg_ctlinfo1_eps1_kb* eps1)
{
	pb_obj_ctlinfo1_eps1_kb = eps1;
	pb_obj_eps_to_ctrl->set_de_phisteerangle((double)((int16_t)(pb_obj_ctlinfo1_eps1_kb->steeringangle_low() + (u_int16_t)(pb_obj_ctlinfo1_eps1_kb->steeringangle_high() << 8))) * 0.1 + 0); // msg_ctlinfo1_eps1_kb:方向盘转角
	pb_obj_eps_to_ctrl->set_de_phisteerspd((double)((int16_t)(pb_obj_ctlinfo1_eps1_kb->steeringanglespeed_low() + (u_int16_t)(pb_obj_ctlinfo1_eps1_kb->steeringanglespeed_high() << 8))) * 0.1 + 0); //msg_ctlinfo1_eps1_kb：方向盘转速
	pb_obj_eps_to_ctrl->set_de_steertoruqe(((int16_t)(pb_obj_ctlinfo1_eps1_kb->steeringtorque_low() + (u_int16_t)(pb_obj_ctlinfo1_eps1_kb->steeringtorque_high() << 8))) * 0.1 + 0); //msg_ctlinfo1_eps1_kb：方向盘手扭矩
	pb_obj_eps_to_ctrl->set_de_handtorque(0);
	logger.LogInfo()<<"----------------------EPS_To_Ctrl_.DE_phiSteerAngle--------------"<<pb_obj_eps_to_ctrl->de_phisteerangle();
}

void App::AppCore::setCTLINFO2_EPS1_KB(msg_ctlinfo2_eps1_kb* eps2)
{
	pb_obj_ctlinfo2_eps1_kb = eps2;
	pb_obj_eps_to_ctrl->set_de_epsmode((double)(pb_obj_ctlinfo2_eps1_kb->ctlinfo2_eps1_kb_first_byte_1().controldemandresponse() * 1 + 0)); //msg_ctlinfo2_eps1_kb Byte0
	pb_obj_eps_to_ctrl->set_de_epserrcode((double)(pb_obj_ctlinfo2_eps1_kb->not_defiend()));

	if(pb_obj_ctlinfo2_eps1_kb->ctlinfo2_eps1_kb_first_byte_1().ctrlmodresp_overridedetection() == 1)
	{
		switchOn = false;
	}
}

void App::AppCore::setState_BCM(msg_state_bcm* bcm)
{
	//logger.LogInfo()<<"pre_engage_mode: "<<pre_engage_mode<<"now_engage_mode"<<now_engage_mode;
	pre_engage_mode = now_engage_mode;
	pb_obj_state_bcm = bcm;
	//enable_mode = inter_State_BCM.ActiveSwitchOfADCU; //ENABLE ADCU
	now_engage_mode = pb_obj_state_bcm->state_bcm_second_byte_1().poweronswitchofadcu(); //ENGAGE ADCU

	if(pre_engage_mode == 0 && now_engage_mode == 1)
	{
		switchOn =! switchOn;
	}
}

void App::AppCore::setTSC1_TE(msg_tsc1_te* tsc1)
{
	pb_obj_tsc1_te = tsc1;
	pb_obj_tcu_to_ctrl->set_vehda_prctratrqlim_mp(0);
}

void App::AppCore::setETC7(msg_etc7* etc7)
{
	App::ComCore cc;
	pb_obj_etc7 = etc7;

	//ETC7_.Transmission_Mode_3 = pb_obj_etc7->etc7_third_byte_1().transmission_mode_3();
	//ETC7_.Transmission_Ready_for_Brake_Release = pb_obj_etc7->etc7_second_byte_1().transmission_ready_for_brake_release();
	pb_obj_etc7_to_ctrl->mutable_etc7_to_ctrl_third_byte_1()->set_transmission_mode_3(pb_obj_etc7->etc7_third_byte_1().transmission_mode_3());
	pb_obj_etc7_to_ctrl->mutable_etc7_to_ctrl_second_byte_1()->set_transmission_ready_for_brake_release(pb_obj_etc7->etc7_second_byte_1().transmission_ready_for_brake_release());
	//pb_obj_etc7_to_ctrl->set_transmission_mode_3();
	//pb_obj_etc7_to_ctrl->etc7_to_ctrl_third_byte_1().set_transmission_mode_3(pb_obj_etc7->etc7_third_byte_1().transmission_mode_3());
	//u_int32_t i = pb_obj_etc7->etc7_third_byte_1().transmission_mode_3();
	//pb_obj_etc7_to_ctrl->etc7_to_ctrl_third_byte_1().set_transmission_mode_3(1);
	//int i = 2;
	//msg_etc7_to_ctrl etc;
	//etc.mutable_etc7_to_ctrl_third_byte_1().set_transmission_mode_3(i);
	//pb_obj_etc7_to_ctrl->etc7_to_ctrl_third_byte_1().set_transmission_mode_3(i);
	//pb_obj_etc7_to_ctrl->etc7_to_ctrl_third_byte_1().set_transmission_mode_3(1);
	pb_obj_etc7_to_ctrl->etc7_to_ctrl_third_byte_1().transmission_mode_3();
	//pb_obj_etc7_to_ctrl->etc7_to_ctrl_second_byte_1().set_transmission_ready_for_brake_release(pb_obj_etc7->etc7_second_byte_1().transmission_ready_for_brake_release());
	cc.ETC7_Pub(*pb_obj_etc7_to_ctrl);
	//logger.LogInfo()<<"----------------------Transmission_Mode_3--------------"<<TC_1_.transmissionmode_3;
	dfcv_mining_msgs::Ros_To_Can msgCan;
	msgCan.can_id = 0x18FE4A03;
	msgCan.channel_id = A_CAN_CHANNEL_ID;
	memcpy(&msgCan.data, &pb_obj_etc7, sizeof(pb_obj_etc7));//sizeof指针，出来的大小存在潜在风险
//		for(int i= 0; i < 8; i ++)
//		{
//			msgCan.data[i]=msg->data[i];
//		}
	//dfcv_mining_msgs::Ros_To_Can::ConstPtr ptr=msgCan;
	//ros2canbusAB(msgCan);		//zxp 0904 处理报错	111111
	dfcv_mining_msgs::Ros_To_Can *msgCanPtr = &msgCan;	//zxp 0904	处理报错	11111
	cc.ros2ToCanHandle(msgCanPtr);		//zxp 0904	处理报错	11111
//	msgCan.channel_id = C_CAN_CHANNEL_ID;
//	ros2canbusAB(msgCan);
}

/// -----------------new--------------------------------
void App::AppCore::setMS1_MCU(msg_ms1_mcu* ms1_mcu)
{
	pb_obj_ms1_mcu = ms1_mcu;
	pb_obj_eecu_to_ctrl->set_vehda_prcactutrq_mp((int16_t)((u_int16_t)pb_obj_ms1_mcu->ms1_mor_trq_cent_low() + (u_int16_t)(pb_obj_ms1_mcu->ms1_mor_trq_cent_high() << 8)));
}

void App::AppCore::setMC1_HCU(msg_mc1_hcu* mc1_hcu)
{
	pb_obj_mc1_hcu = mc1_hcu;
	pb_obj_eecu_to_ctrl->set_vehda_prcdrvrdmdtrq_mp((int16_t)((u_int16_t)pb_obj_mc1_hcu->mc1_mor_tgt_trq_low() + (u_int16_t)(pb_obj_mc1_hcu->mc1_mor_tgt_trq_high() << 8)));
}

void App::AppCore::setEEC3_E(msg_eec3_e* eec3_e)
{
	pb_obj_eec3_e = eec3_e;
	pb_obj_eecu_to_ctrl->set_vehda_prctrqengnomfric_mp((int16_t)(pb_obj_eec3_e->nominal_friction_percent_torque())*1-125);
	pb_obj_eecu_to_ctrl->set_vehda_prctrqestimdloss_mp((int16_t)pb_obj_eec3_e->estimated_engine_parasitic_losses_percent_torque());
}

void App::AppCore::setCCVS1_HCU(msg_ccvs1_hcu* ccvs1_hcu)
{
	pb_obj_ccvs1_hcu = ccvs1_hcu;
	pb_obj_eecu_to_ctrl->set_vehda_stcluswt_mp(pb_obj_ccvs1_hcu->ccvs1_hcu_fourth_byte_1().clutchswitch());
}

void App::AppCore::setTSC1_AE(msg_tsc1_ae* tsc1_ae)
{
	pb_obj_tsc1_ae = tsc1_ae;
	pb_obj_tcu_to_ctrl->set_vehda_sttratrqlim_mp(pb_obj_tsc1_te->first_byte_1()* 1 + 0);
}

void App::AppCore::setCVW_EBS(msg_cvw_ebs* cvw_ebs)
{
	pb_obj_cvw_ebs = cvw_ebs;
	pb_obj_ebs_to_ctrl->set_vehda_mwght_mp((double)(pb_obj_cvw_ebs->gross_combination_weight_low()+ (u_int16_t)(pb_obj_cvw_ebs->gross_combination_weight_high() << 8))*2+0);
}

void App::AppCore::setEPB1(msg_epb1* epb1)
{
	pb_obj_epb1 = epb1;
	pb_obj_epb1_to_ctrl->set_epb_parkbrkst(pb_obj_epb1->state_of_epb() *1 + 0);
	logger.LogInfo()<<"-----======---EPB1_To_Ctrl_.EPB_ParkBrKSt-----"<<pb_obj_epb1_to_ctrl->epb_parkbrkst();
}

void App::ComCore::PositionReceived(dfcv_mining_msgs::LocationEx::ConstPtr pos_msg)
{
//	vel_pos[0] = pos_msg->LocationEx_X;
//	vel_pos[1] = pos_msg->LocationEx_Y;
//	vel_pos[2] = 0.0;
	Location_time_now = pos_msg->header.stamp.sec + pos_msg->header.stamp.nsec/1000000000;
	if( vel_pos_x.size() <= 8)
	{
		vel_pos_x.push(pos_msg->LocationEx_X);
		vel_pos_y.push(pos_msg->LocationEx_Y);
		vel_pos_z.push(0.0);
		vel_pos_yaw.push(pos_msg->LocationEx_yaw);
	}
	else 
	{
		vel_pos_x.pop();
		vel_pos_y.pop();
		vel_pos_z.pop();
		vel_pos_yaw.pop();
		vel_pos_x.push(pos_msg->LocationEx_X);
		vel_pos_y.push(pos_msg->LocationEx_Y);
		vel_pos_z.push(0.0);
		vel_pos_yaw.push(pos_msg->LocationEx_yaw);
	}
//logger.LogInfo()<<"-----======---pos_msg->LocationEx_Y--------======---- "<<pos_msg->LocationEx_Y;
	double end_x,end_y;
	memcpy(&end_x,&pb_obj_tbox_move->end_x(),sizeof(end_x));
	memcpy(&end_y,&pb_obj_tbox_move->end_y(),sizeof(end_y));
	dis_error = sqrt(pow(end_x-pos_msg->LocationEx_X,2)+pow(end_y-pos_msg->LocationEx_Y,2));
	logger.LogInfo()<<"--------------s--end_x------------ "<<end_x;
	logger.LogInfo()<<"--------------s--end_y------------ "<<end_y;
	logger.LogInfo()<<"--------------s--dis_error------------ "<<dis_error;
	if (dis_error<1.0)
	{
		arrive_end_point=true;
	}
	else
	{
		arrive_end_point=false;
	}

}

void App::AppCore::setAutoState(bool autoState)
{
	switchOn = autoState;
}

void App::AppCore::setTbox(msg_tbox_joint* tBoxJoint,msg_tbox_drop* tBoxDrop,msg_tbox_move *tboxMove,msg_tbox_move_id* planId)
{
	//dfcv_mining_msgs::System_TboxEx TboxExMsg;
	logger.LogInfo()<<"validLen: "<<"output data to plan";
	//memset(&plan,0,sizeof(plan));//20221007
	if(tBoxJoint != NULL)
	{
		u_int32_t buf1[8] = {0};
		for(int i = 0; i < 8; i++){
			buf1[i] = tBoxJoint->questnum(i);
		}
		memcpy(&TboxExMsg.Joint_TaskNo, buf1, sizeof(buf1));

		//TboxExMsg.Joint_TaskNo=tBoxJoint->questnum;
		TboxExMsg.Joint_Tasktype = tBoxJoint->questtype();
		logger.LogInfo()<<"--------tBoxJoint->questtype------ "<<tBoxJoint->questtype();
		TboxExMsg.Joint_SlowGuide = tBoxJoint->slowguide();
		TboxExMsg.Joint_JointPoint = tBoxJoint->joint_point();
		TboxExMsg.Joint_JointMethod = tBoxJoint->joint_method();
	}

	if(tBoxDrop!=NULL)
	{	
		u_int32_t buf1[8] = {0};
		for(int i = 0; i < 8; i++){
			buf1[i] = pb_obj_tbox_drop->questnum(i);
		}
		memcpy(&TboxExMsg.Drop_TaskNo, buf1, sizeof(buf1));
		//TboxExMsg.Drop_TaskNo=tBoxDrop->questnum;
		TboxExMsg.Drop_TaskType = pb_obj_tbox_drop->questtype();
		TboxExMsg.Drop_TaskOperation = pb_obj_tbox_drop->questoperation();
	}

	if(tboxMove != NULL)
	{
		logger.LogInfo()<<"validLen: "<<"333333";
		//TboxExMsg.Move_PassPathId.clear;
		TboxExMsg.Move_PassPathId.clear();

		u_int32_t buf1[8] = {0};
		for(int i = 0; i < 8; i++){
			buf1[i] = pb_obj_tbox_move->questnum(i);
		}
		memcpy(&TboxExMsg.Move_TaskNo, buf1, sizeof(buf1));
		//memcpy(&TboxExMsg.Move_TaskNo, pb_obj_tbox_move->questnum(),  sizeof(u_int64_t));

		//TboxExMsg.Move_TaskNo = tboxMove->questnum;
		TboxExMsg.Move_TaskType = pb_obj_tbox_move->questtype();
		//TboxExMsg.Move_TaskOperation = plan.questoperation;
		TboxExMsg.Move_LimitSpeed =  pb_obj_tbox_move->limitspeed();

		memset(buf1, 0, sizeof(buf1));
		for(int i = 0; i < 8; i++){
			buf1[i] = pb_obj_tbox_move->start_x(i);
		}
		memcpy(&TboxExMsg.Move_Start_X, buf1, sizeof(buf1));
		//memcpy(&TboxExMsg.Move_Start_X, pb_obj_tbox_move->start_X(), 8);

		memset(buf1, 0, sizeof(buf1));
		for(int i = 0; i < 8; i++){
			buf1[i] = pb_obj_tbox_move->start_y(i);
		}
		memcpy(&TboxExMsg.Move_Start_Y, buf1, sizeof(buf1));
		//memcpy(&TboxExMsg.Move_Start_Y, pb_obj_tbox_move->start_Y(), 8);

		memset(buf1, 0, sizeof(buf1));
		for(int i = 0; i < 8; i++){
			buf1[i] = pb_obj_tbox_move->start_z(i);
		}
		memcpy(&TboxExMsg.Move_Start_Z, buf1, sizeof(buf1));
		//memcpy(&TboxExMsg.Move_Start_Z, pb_obj_tbox_move->start_Z(), 8);

		logger.LogInfo()<<"validLen: "<<"44444";
		
		memset(buf1, 0, sizeof(buf1));
		for(int i = 0; i < 8; i++){
			buf1[i] = pb_obj_tbox_move->end_x(i);
		}
		memcpy(&TboxExMsg.Move_End_X, buf1, sizeof(buf1));
		//memcpy(&TboxExMsg.Move_End_X, pb_obj_tbox_move->end_X(), 8);
		
		memset(buf1, 0, sizeof(buf1));
		for(int i = 0; i < 8; i++){
			buf1[i] = pb_obj_tbox_move->end_y(i);
		}
		memcpy(&TboxExMsg.Move_End_Y, buf1, sizeof(buf1));
		//memcpy(&TboxExMsg.Move_End_Y, pb_obj_tbox_move->end_Y(), 8);
		
		memset(buf1, 0, sizeof(buf1));
		for(int i = 0; i < 8; i++){
			buf1[i] = pb_obj_tbox_move->end_z(i);
		}
		memcpy(&TboxExMsg.Move_End_Z, buf1, sizeof(buf1));				
		//memcpy(&TboxExMsg.Move_End_Z, pb_obj_tbox_move->end_Z(), 8);

//		/TboxExMsg.Move_Start_X = tboxMove->start_X;
//		TboxExMsg.Move_Start_Y = tboxMove->start_Y;
//		TboxExMsg.Move_Start_Z = tboxMove->start_Z;
//		TboxExMsg.Move_End_X = tboxMove->end_X;
//		TboxExMsg.Move_End_Y = tboxMove->end_Y;
//		TboxExMsg.Move_End_Z = tboxMove->end_Z;
		//int count;
//				for(int j=0;i<planIds[i]->idlength;j++){
//					logger.LogInfo()<<"j: "<<planIds[i]->idlength;
//					TboxExMsg.Move_PassPathId.push_back(planIds[i]->id[j]);
//					count++;
//				}
//		if ((vechicle_pln.BhvCrdn_numBhvID == 1)||(vechicle_pln.BhvCrdn_numBhvID == 13))
//		{
//			plan.questoperation = 0;
//		}
//		if(vechicle_pln.BhvCrdn_numBhvID == 11)   //10.14
//		{
//			joint.questtype=0;
//		}
		TboxExMsg.Move_TaskOperation = pb_obj_tbox_move->questoperation();
		logger.LogInfo()<<"---------------------+++++++-----pb_obj_tbox_move->questoperation(): ------+++++++++--------------------------"<<pb_obj_tbox_move->questoperation();
		TboxExMsg.Move_PassPathId.insert(TboxExMsg.Move_PassPathId.end(),planIds.begin(),planIds.end());
		TboxExMsg.Move_PathIdLength = planIds.size();
		double x;
		double y;
		double z;
		double ex;
		double ey;
		double ez;
		memcpy(&x, &TboxExMsg.Move_Start_X, 8);
		memcpy(&y, &TboxExMsg.Move_Start_Y, 8);
		memcpy(&z, &TboxExMsg.Move_Start_Z, 8);
		memcpy(&ex, &TboxExMsg.Move_End_X, 8);
		memcpy(&ey, &TboxExMsg.Move_End_Y, 8);
		memcpy(&ez, &TboxExMsg.Move_End_Z, 8);
		logger.LogInfo()<<"============================TboxExMsg: "<<TboxExMsg.Move_TaskNo<<"Move_TaskType:"<<TboxExMsg.Move_TaskType<<
				"Move_TaskOperation"<<TboxExMsg.Move_TaskOperation<<"Move_LimitSpeed"<<TboxExMsg.Move_LimitSpeed
				<<"x:"<<x<<"y:"<<y<<"z:"<<z<<"ex:"<<ex<<"ey:"<<ey<<"ez:"<<ez;
				;
	}
	logger.LogInfo()<<"-------------+++++++++=======-------------TboxExMsg.Move_TaskOperation ----------++++++++++=======----------------------"<<TboxExMsg.Move_TaskOperation;
	//TBoxExMsg_pub.publish(TboxExMsg);
}

void App::ComCore::publishTboxMsg(){
	App::AppCore ac;
	//dfcv_mining_msgs::System_TboxEx TboxExMsg;
	auto TboxExMsg = std::make_shared<dfcv_mining_msgs::System_TboxEx>();
	logger.LogInfo()<<"hasTBoxMove: "<<hasTBoxMove;
	if(hasTBoxMove)
	{
		if(tBoxMoveCount == 10 && ac.lastMsg.Move_TaskNo == ac.msg.Move_TaskNo)
		{
			ac.msg.Move_TaskType = 0;
			//TBoxExMsg_pub.publish(TboxExMsg);
			TBoxExMsg_pub->Write(TboxExMsg);
		}else
		{
			//TBoxExMsg_pub.publish(TboxExMsg);
			TBoxExMsg_pub->Write(TboxExMsg);
			if(ac.lastMsg.Move_TaskNo != ac.msg.Move_TaskNo)
			{
				tBoxMoveCount = 0;
			}
		}
		ac.lastMsg = ac.msg;
		tBoxMoveCount++;
	}
	if(hasTBoxJoint)
	{
			if(tBoxJointCount == 10 && ac.lastMsg.Joint_TaskNo == ac.msg.Joint_TaskNo)
			{
				ac.msg.Joint_Tasktype = 0;
				//TBoxExMsg_pub.publish(TboxExMsg);
				TBoxExMsg_pub->Write(TboxExMsg);
			}else
			{
				//TBoxExMsg_pub.publish(TboxExMsg);
				TBoxExMsg_pub->Write(TboxExMsg);
				if(ac.lastMsg.Joint_TaskNo != ac.msg.Joint_TaskNo)
				{
					tBoxJointCount = 0;
				}
			}
			ac.lastMsg = ac.msg;
			tBoxMoveCount++;
	}
	if(hasTBoxDrop)
	{
			if(tBoxDropCount == 10 && ac.lastMsg.Drop_TaskNo == ac.msg.Drop_TaskNo)
			{
				ac.msg.Drop_TaskType = 0;
				//TBoxExMsg_pub.publish(TboxExMsg);
				TBoxExMsg_pub->Write(TboxExMsg);
			}else
			{
				//TBoxExMsg_pub.publish(TboxExMsg);
				TBoxExMsg_pub->Write(TboxExMsg);
				if(ac.lastMsg.Drop_TaskNo != ac.msg.Drop_TaskNo)
				{
					tBoxDropCount = 0;
				}
			}
			ac.lastMsg = ac.msg;
			tBoxDropCount++;
		}
}

void App::ComCore::BehaviorExMsgReceived(dfcv_mining_msgs::BehaviorEx::ConstPtr BehaviorEx_msg)
{
	last_BhvCrdn_numBhvID = pb_obj_vechicle_cmd->bhvcrdn_numbhvid();
	pb_obj_vechicle_cmd->set_bhvcrdn_creepmod(BehaviorEx_msg->BehaviorCoordinate_CreepMode);
	pb_obj_vechicle_cmd->set_bhvcrdn_gearreq(BehaviorEx_msg->BehaviorCoordinate_GearRequest);
	pb_obj_vechicle_cmd->set_bhvcrdn_numbhvid(BehaviorEx_msg->BehaviorCoordinate_NumberBehaviorID);
//	logger.LogInfo()<<"--------vechicle_pln.BhvCrdn_numBhvID "<<vechicle_pln.BhvCrdn_numBhvID;
	if(BehaviorEx_msg->BehaviorCoordinate_NumberBehaviorID != 13)
	{
		backward = false;
	}
	else
	{
		backward = true;
	}
	if(dis_error < 1.0)
	{
		pb_obj_tbox_move->set_questoperation(0);
		//plan.questoperation = 0;
	}
	else if((pb_obj_vechicle_cmd->bhvcrdn_numbhvid() == 11) && (pb_obj_tbox_move->questoperation() != 1))   //10.13
	{
		pb_obj_tbox_move->set_questoperation(0);
		//plan.questoperation = 0;
	}
	else if (((pb_obj_vechicle_cmd->bhvcrdn_numbhvid()== 1) || (pb_obj_vechicle_cmd->bhvcrdn_numbhvid() == 13)) && (pb_obj_tbox_move->questoperation() != 2))
	{
		pb_obj_tbox_move->set_questoperation(0);
		//plan.questoperation = 0;
	}
	logger.LogInfo()<<"--------pb_obj_tbox_move->questoperation "<<pb_obj_tbox_move->questoperation();

	Planning_time_now = BehaviorEx_msg->header.stamp.sec + BehaviorEx_msg->header.stamp.nsec/1000000000;
//	logger.LogInfo()<<"--------Planning_time_now ++++++++++++++++++"<<Planning_time_now;
}

void App::ComCore::MotionExMsgReceived(dfcv_mining_msgs::MotionEx::ConstPtr MotionEx_msg)
{
	pb_obj_vechicle_cmd->set_pthpln_ktrgcrv(MotionEx_msg->PathPlan_TargetCurvature);
	pb_obj_vechicle_cmd->set_pthpln_ltrgltr(MotionEx_msg->PathPlan_LateralOffset);
	pb_obj_vechicle_cmd->set_pthpln_phitrgang(MotionEx_msg->PathPlan_YawOffset);
	pb_obj_vechicle_cmd->set_spdpln_atgtacc(MotionEx_msg->SpeedPlan_TargetAcceleration);
	pb_obj_vechicle_cmd->set_spdpln_vtgtspd(MotionEx_msg->SpeedPlan_TargetSpeed);
	pb_obj_vechicle_cmd->set_rounteid(MotionEx_msg->RoutePlan_CurrentRouteID);
	pb_obj_vechicle_cmd->set_s(MotionEx_msg->RoutePlan_CurrentRouteRemainderDistance);

}

//ADCU状态上传TBOX
void App::ComCore::ros2tbox()
{
	App::ComCore cc;
	//drdtu::Ros2canfdbus can_to_tbox_msg1;
	auto can_to_tbox_msg1 = std::make_shared<drdtu::Ros2canfdbus>();
	//drdtu::Ros2canfdbus can_to_tbox_msg2;
	auto can_to_tbox_msg2 = std::make_shared<drdtu::Ros2canfdbus>(); 
	//drdtu::Ros2canfdbus can_to_tbox_msg3;
	auto can_to_tbox_msg3 = std::make_shared<drdtu::Ros2canfdbus>();
	//drdtu::Ros2canfdbus can_to_tbox_msg4;    //2023.1.11
	auto can_to_tbox_msg4 = std::make_shared<drdtu::Ros2canfdbus>();

//	drdtu::Ros2canfdbus can_to_tbox_move_msg;
//	drdtu::Ros2canfdbus can_to_tbox_drop_msg;
//	drdtu::Ros2canfdbus can_to_tbox_joint_msg;

	can_to_tbox_msg1.channel_id = TBOX_CHANNEL_ID;  //tbox
	can_to_tbox_msg2.channel_id = TBOX_CHANNEL_ID;  //tbox
	can_to_tbox_msg3.channel_id = TBOX_CHANNEL_ID;  //tbox
	can_to_tbox_msg4.channel_id = TBOX_CHANNEL_ID;  //tbox

//	can_to_tbox_move_msg.channel_id = TBOX_CHANNEL_ID;
//	can_to_tbox_drop_msg.channel_id = TBOX_CHANNEL_ID;
//	can_to_tbox_joint_msg.channel_id = TBOX_CHANNEL_ID;
	msg_adcu_to_tbox_1 *pb_obj_adcu_to_tbox_1;
	//ADCU_TO_TBOX_1 adcu_to_tbox_1;
	msg_adcu_to_tbox_2 *pb_obj_adcu_to_tbox_2;
	//ADCU_TO_TBOX_2 adcu_to_tbox_2;
	msg_adcu_to_tbox_3 *pb_obj_adcu_to_tbox_3;
	//ADCU_TO_TBOX_3 adcu_to_tbox_3;
	msg_adcu_to_tbox_4 *pb_obj_adcu_to_tbox_4;
	//ADCU_TO_TBOX_4 adcu_to_tbox_4;

	for (unsigned int j = 0; j < 64; j++)
	{
		can_to_tbox_msg1.data[j] = 0;
		can_to_tbox_msg2.data[j] = 0;
		can_to_tbox_msg3.data[j] = 0;
		can_to_tbox_msg4.data[j] = 0;
	}

	// fulfill the data
	pb_obj_adcu_to_tbox_1->set_adcu_ctrl_mode(enable_mode+engage_mode);//0x02;
	pb_obj_adcu_to_tbox_1->set_scene_type(0x00);
	pb_obj_adcu_to_tbox_1->set_action_type((unsigned int)(backward) * 0x12);
	pb_obj_adcu_to_tbox_1->set_error_code(0x03);
	//x\y\z 移位操作    //2023.2.22
	pb_obj_adcu_to_tbox_1->set_x_1(int64_t(vel_pos_x.front()*1000) & 0xFFFFFFFF);
	pb_obj_adcu_to_tbox_1->set_x_2(int64_t(vel_pos_x.front()*1000) & (0xFFFFFFFF << 32));
	pb_obj_adcu_to_tbox_1->set_y_1(int64_t(vel_pos_y.front()*1000) & 0xFFFFFFFF);
	pb_obj_adcu_to_tbox_1->set_y_2(int64_t(vel_pos_y.front()*1000) & (0xFFFFFFFF << 32));
	pb_obj_adcu_to_tbox_1->set_z_1(int64_t(vel_pos_z.front()*1000) & 0xFFFFFFFF);
	pb_obj_adcu_to_tbox_1->set_z_2(int64_t(vel_pos_z.front()*1000) & (0xFFFFFFFF << 32));
	logger.LogInfo()<<"--------------location-x--------------- "<<pb_obj_adcu_to_tbox_1->x_1();
	//logger.LogInfo()<<"-------------- vel_pos_y.front()*1000--------------- "<< vel_pos_y.front()*1000;
	logger.LogInfo()<<"--------------location-y--------------- "<<pb_obj_adcu_to_tbox_1->y_1();
//	for(int i = 0; i < 8; i++)
//	{
//		adcu_to_tbox_1.x[i] = (u_int64_t(vel_pos_x.front()) & (0xFF<<8*i))>>8*i;
//		adcu_to_tbox_1.y[i] = (u_int64_t(vel_pos_y.front()) & (0xFF<<8*i))>>8*i;
//		adcu_to_tbox_1.z[i] = (u_int64_t(vel_pos_z.front()) & (0xFF<<8*i))>>8*i;
//	}
//	logger.LogInfo()<<"--------------location-x--------------- "<<adcu_to_tbox_1.x;
//	logger.LogInfo()<<"-------------- vel_pos_y.front()*1000--------------- "<< vel_pos_y.front()*1000;
//	logger.LogInfo()<<"--------------location-y--------------- "<<adcu_to_tbox_1.y;
	pb_obj_adcu_to_tbox_1->set_speed(pb_obj_eecu_to_ctrl->vehda_nengspd_mp() * 3.6);
//	adcu_to_tbox_1.direction_angle_range = vel_pos_yaw.front()*100;		  	        //方向盘转角,左转角为正/右转角为负
//	adcu_to_tbox_1.course_angle = EPS_To_Ctrl_.DE_phiSteerAngle;					//车辆航向角
	pb_obj_adcu_to_tbox_1->set_direction_angle_range(pb_obj_eps_to_ctrl->de_phisteerangle());       //方向盘转角,左转角为正/右转角为负    //2023.2.1
	pb_obj_adcu_to_tbox_1->set_course_angle(vel_pos_yaw.front()*100);                      //车辆航向角                      //2023.2.1
	pb_obj_adcu_to_tbox_1->set_sys_state(engage_mode + enable_mode *2);
	pb_obj_adcu_to_tbox_1->set_manual_state(manual);
	pb_obj_adcu_to_tbox_1->set_lidar_state(0);
	pb_obj_adcu_to_tbox_1->set_camera_state(0);
	pb_obj_adcu_to_tbox_1->set_acc_speed(0);
	pb_obj_adcu_to_tbox_1->set_adcu_power_state(0);
	pb_obj_adcu_to_tbox_3->set_lateral_deviation(0);
	pb_obj_adcu_to_tbox_3->set_global_course_angle(pb_obj_vechicle_cmd->pthpln_phitrgang());
	pb_obj_adcu_to_tbox_3->set_curvature(pb_obj_vechicle_cmd->pthpln_ktrgcrv());
	pb_obj_adcu_to_tbox_3->set_target_speed(pb_obj_vechicle_cmd->spdpln_vtgtspd());
	pb_obj_adcu_to_tbox_3->set_target_acc(pb_obj_vechicle_cmd->spdpln_atgtacc());
	pb_obj_adcu_to_tbox_3->set_bev_id(pb_obj_vechicle_cmd->bhvcrdn_numbhvid());
	//adcu_to_tbox_3.dis_to_target = 20;//vechicle_pln.PthPln_lTrgLtr;
	//adcu_to_tbox_3.path_id = vechicle_pln.rounteID;  2023.1.11
	pb_obj_adcu_to_tbox_3->set_rev1(0);   //2023.1.11
	pb_obj_adcu_to_tbox_3->set_request_gear(pb_obj_vechicle_cmd->bhvcrdn_gearreq());
	for (int i = 0; i< 4; i++)//2023.4.7路径🆔id
	{
		pb_obj_adcu_to_tbox_3->set_target_obj_id(i, 0);
		pb_obj_adcu_to_tbox_3->set_target_obj_dis(i, 0);
		pb_obj_adcu_to_tbox_3->set_target_obj_speed(i, 0);
		pb_obj_adcu_to_tbox_3->set_plan_track_point(i, 0);
	}
	//dis_to_target 移位操作   //2023.2.22
	for (int j = 0; j < 4; j++)
	{
		pb_obj_adcu_to_tbox_3->set_dis_to_target(j, (int32_t(pb_obj_vechicle_cmd->pthpln_ltrgltr()) & (0x000000FF << 8*j)) >> 8*j);
			//adcu_to_tbox_3.dis_to_target[1] = (int32_t(10000000) & (0x000000FF << 8)) >> 8;
			//adcu_to_tbox_3.dis_to_target[2] = (int32_t(10000000) & (0x000000FF << 16)) >> 16;
			//adcu_to_tbox_3.dis_to_target[3] = (int32_t(10000000) & (0x000000FF << 24)) >> 24;
	}
	pb_obj_adcu_to_tbox_3->set_creep_state(0);//vechicle_pln.BhvCrdn_CreepMod;
	pb_obj_adcu_to_tbox_3->set_lift_state(5);//vechicle_pln.;
	//adcu_to_tbox_3.path_id = vechicle_pln.rounteID;
	//vechicle_pln.rounteID = 100;
	for (int j = 0; j < 4; j++)//2023.4.7路径id
	{
		pb_obj_adcu_to_tbox_3->set_path_id(j, (((int32_t(pb_obj_vechicle_cmd->rounteid())) & (0x000000FF<<8*j))>>8*j));
	}
	pb_obj_adcu_to_tbox_4->set_acceleration_from_vehicle(Ctrl_Acceleration_from_Vehicle*1000 + 10);  //车辆实际加速度 2023.1.11

	unsigned char buff1[64];
	unsigned char buff2[64];
	unsigned char buff3[64];
	unsigned char buff4[64];
	memcpy(buff1, (unsigned char*)&pb_obj_adcu_to_tbox_1, 64);
	memcpy(buff2, (unsigned char*)&pb_obj_adcu_to_tbox_2, 64);
	memcpy(buff3, (unsigned char*)&pb_obj_adcu_to_tbox_3, 64);
	memcpy(buff4, (unsigned char*)&pb_obj_adcu_to_tbox_4, 64);
	//put the structs in the msgs
	for (unsigned int j = 0; j < 64; j++) 
	{
		can_to_tbox_msg1.data[j] = buff1[j];
		can_to_tbox_msg2.data[j] = buff2[j];
		can_to_tbox_msg3.data[j] = buff3[j];
		can_to_tbox_msg4.data[j] = buff4[j];
	}

	can_to_tbox_msg1.time_meas = deeproute::now();
	can_to_tbox_msg1.time_pub = deeproute::now();
	can_to_tbox_msg1.can_id = 0x19FF509E;

	can_to_tbox_msg2.time_meas = deeproute::now();
	can_to_tbox_msg2.time_pub = deeproute::now();
	can_to_tbox_msg2.can_id = 0x19FF599E;

	can_to_tbox_msg3.time_meas = deeproute::now();
	can_to_tbox_msg3.time_pub = deeproute::now();
	can_to_tbox_msg3.can_id = 0x19FF609E;

	can_to_tbox_msg4.time_meas = deeproute::now();
	can_to_tbox_msg4.time_pub = deeproute::now();
	can_to_tbox_msg4.can_id = 0x19FF619E;

	cc.canfdbus2ros_pub->Write(can_to_tbox_msg1);
	cc.canfdbus2ros_pub->Write(can_to_tbox_msg2);
	cc.canfdbus2ros_pub->Write(can_to_tbox_msg3);
	cc.canfdbus2ros_pub->Write(can_to_tbox_msg4);
}

void App::ComCore::tboxMoveReply(bool hasTboxMove, u_int8_t state, u_int8_t questtype)
{
	//App::ComCore cc;
	//zxp add start 0831	can通信流程简化——T-box事件回复
	// drdtu::Ros2canfdbus can_to_tbox_move_msg;
	// can_to_tbox_move_msg.channel_id = TBOX_CHANNEL_ID;
	// can_to_tbox_move_msg.time_meas = deeproute::now();
	// can_to_tbox_move_msg.time_pub = deeproute::now();
	// can_to_tbox_move_msg.can_id = 0x19ff569e;
	// for (unsigned int j = 0; j < 64; j++) {
	// 	can_to_tbox_move_msg.data[j] = 0;
	// }
	//drdtu::Ros2canfdbus can_to_tbox_reply_msg;	
	tboxEventReply(1, state, questtype);
	//can_to_tbox_reply_msg.can_id = 0x19ff569e;	
	//TBOX_MOVE_STATE tboxState;
	// for(int n = 0; n < 8; n++)
	// {
	// 	tboxMoveState.questnum[n] = plan.questnum[n];
	// }
	// int task_state=0;
	// if (questtype==1)
	// {
	// 	if((state==1)||(state==13)||(state==5))
	// 	{
	// 		task_state=0;
	// 	}
	// 	else if ((state==11)&&(arrive_end_point==true))
	// 	{
	// 		task_state=1;
	// 		//hasTBoxMove = false;
	// 	}
	// 	else if ((state==11)&&(arrive_end_point==false)&&(task_error==true))
	// 	{
	// 		task_state=6;
	// 	}
	// }
	// else if (questtype==2)
	// {
	// 	if(state==11){
	// 		task_state=3;
	// 	}
	// 	else {
	// 		task_state=6;
	// 	}
	// }
	// else if (questtype==3)
	// {
	// 	if((state==1)||(state==13)||(state==5)){
	// 		task_state=4;
	// 	}
	// 	else if ((state==11)&&(arrive_end_point==true))
	// 	{
	// 		task_state=1;
	// 		//hasTBoxMove = false;
	// 	}
	// 	else if ((state==11)&&(arrive_end_point==false)&&(task_error==true))
	// 	{
	// 		task_state=6;
	// 	}
	// }
	// else if (questtype==4)
	// {
	// 	if(state==11)
	// 	{
	// 		task_state=5;
	// 	}
	// 	else
	// 	{
	// 		task_state=6;
	// 	}
	// }
	// tboxMoveState.queststate = task_state;
	//zxp add end 0831	can通信流程简化——T-box事件回复
	//tboxMoveState.questtype = questtype;	//zxp 0831 注释掉
	//timestamp2bcd(1000,tboxMoveState.timestamp);	//zxp 0831 注释掉
	// unsigned char buff1[64];
	// memcpy(buff1,(unsigned char*)&tboxMoveState,64);
	// for (unsigned int j = 0; j < 64; j++) {
	// 	can_to_tbox_reply_msg.data[j] = buff1[j];
	// }
	// canfdbus2ros_pub.publish(can_to_tbox_reply_msg);
	// logger.LogInfo()<<"++++++++++++++++===========tboxMoveReply_task_state: ==============+++++++++++++++"<<task_state;
}

void App::ComCore::tboxAutoStateReply(u_int8_t state, u_int8_t questtype){
	//App::ComCore cc;
	//zxp add start 0831	can通信流程简化——T-box事件回复
	// drdtu::Ros2canfdbus can_to_tbox_joint_msg;
	// can_to_tbox_joint_msg.channel_id = TBOX_CHANNEL_ID;
	// can_to_tbox_joint_msg.time_meas = deeproute::now();
	// can_to_tbox_joint_msg.time_pub = deeproute::now();
	// can_to_tbox_joint_msg.can_id = 0x19ff559e;
	// for (unsigned int j = 0; j < 64; j++) {
	// 	can_to_tbox_joint_msg.data[j] = 0;
	// }
	tboxEventReply(2, state, questtype);
	//can_to_tbox_reply_msg.can_id = 0x19ff559e;	
	//TBOX_JOINT_STATE tboxState;
	//zxp add end 0831	can通信流程简化——T-box事件回复
	//tboxState.questnum = tBoxJoint->questnum;
	//tboxJointORAutoState.queststate = state;
	//tboxJointORAutoState.questtype = questtype;	//zxp 0831 注释掉
	// tboxJointORAutoState.slow_feedback = 1;
	// //timestamp2bcd(1000,tboxJointORAutoState.timestamp);	//zxp 0831 注释掉
	// unsigned char buff1[64];
	// memcpy(buff1,(unsigned char*)&tboxJointORAutoState,64);
	// for (unsigned int j = 0; j < 64; j++) {
	// 	can_to_tbox_reply_msg.data[j] = buff1[j];
	// }
	// canfdbus2ros_pub.publish(can_to_tbox_reply_msg);
}

void App::ComCore::tboxJointReply(bool hasTboxMove, u_int8_t state, u_int8_t questtype){
	//App::AppCore ac;
	//zxp add start 0831	can通信流程简化——T-box事件回复
	// drdtu::Ros2canfdbus can_to_tbox_joint_msg;
	// can_to_tbox_joint_msg.channel_id = TBOX_CHANNEL_ID;
	// can_to_tbox_joint_msg.time_meas = deeproute::now();
	// can_to_tbox_joint_msg.time_pub = deeproute::now();
	// can_to_tbox_joint_msg.can_id = 0x19ff589e;
	// for (unsigned int j = 0; j < 64; j++) {
	// 	can_to_tbox_joint_msg.data[j] = 0;
	// }
	tboxEventReply(3, state, questtype);
	//can_to_tbox_reply_msg.can_id = 0x19ff589e;	
	//zxp add end 0831	can通信流程简化——T-box事件回复
	//TBOX_JOINT_STATE tboxState;
	// int task_state = 0;
	// if ((state==1)||(state==13)||(state==5))
	// {
	// 	task_state = 0;
	// }
	// else if (state==11)
	// {
	// 	task_state = 1;
	// }
	// //tboxState.questnum = tBoxJoint->questnum;
	// tboxJointORAutoState.queststate = task_state;
	// //tboxJointORAutoState.questtype = questtype;	//zxp 0831 注释掉
	// tboxJointORAutoState.slow_feedback = 1;
	// //timestamp2bcd(1000,tboxJointORAutoState.timestamp);	//zxp 0831 注释掉
	// unsigned char buff1[64];
	// memcpy(buff1,(unsigned char*)&tboxJointORAutoState,64);
	// for (unsigned int j = 0; j < 64; j++) {
	// 	can_to_tbox_reply_msg.data[j] = buff1[j];
	// }
	// canfdbus2ros_pub.publish(can_to_tbox_reply_msg);
	// logger.LogInfo()<<"++++++++++++++++===========tboxJointReply_task_state: ==============+++++++++++++++"<<task_state;
}

void App::ComCore::tboxDropReply(bool hasTboxMove, u_int8_t state, u_int8_t questtype){
	//App::AppCore ac;
	//zxp add start 0831	can通信流程简化——T-box事件回复
	// drdtu::Ros2canfdbus can_to_tbox_drop_msg;
	// can_to_tbox_drop_msg.channel_id = TBOX_CHANNEL_ID;
	// can_to_tbox_drop_msg.time_meas = deeproute::now();
	// can_to_tbox_drop_msg.time_pub = deeproute::now();
	// can_to_tbox_drop_msg.can_id = 0x19ff579e;
	// for (unsigned int j = 0; j < 64; j++) {
	// 	can_to_tbox_drop_msg.data[j] = 0;
	// }
	tboxEventReply(4, state, questtype);
	//can_to_tbox_reply_msg.can_id = 0x19ff579e;			
	//TBOX_DROP_STATE tboxState;
	//zxp add end 0831	can通信流程简化——T-box事件回复
	// //tboxState.questnum = tBoxDrop->questnum;
	// tboxDropState.queststate = state;
	// //tboxDropState.questtype = questtype;	//zxp 0831 注释掉
	// //timestamp2bcd(1000,tboxDropState.timestamp);//zxp 0831 注释掉
	// unsigned char buff1[64];
	// memcpy(buff1,(unsigned char*)&tboxDropState,64);
	// for (unsigned int j = 0; j < 64; j++) {
	// 	can_to_tbox_reply_msg.data[j] = buff1[j];
	// }
	// canfdbus2ros_pub.publish(can_to_tbox_reply_msg);
}

unsigned char App::AppCore::CRC8(uint8_t data[])
{
	unsigned char checkSum;
	int byteIndex;
	unsigned char bitIndex;
	unsigned char CRC_POLYNOM;

	checkSum = 0xFF;
	CRC_POLYNOM = 0x2f;

	for(int i = 0; i < 8; i++)
	{
		logger.LogInfo()<<"=====================CRC8.data["<<i<<"]==========="<<data[i];
	}
	for(byteIndex = 1; byteIndex < 8; byteIndex++)
	{
		if(byteIndex != 8)
		{
			checkSum ^= data[byteIndex]; //按位异或运算符。参与运算的两个值，如果两个相应位相同，则结果为0，否则为1
			for(bitIndex = 0 ; bitIndex < 8; bitIndex++)
			{
				if((checkSum & 0x80) != 0)
				{
					checkSum = (unsigned char)((unsigned char)(checkSum <<1)^CRC_POLYNOM);
				}
				else
				{
					checkSum = (unsigned char)(checkSum<<1);
				}
			}
		}

	}
	checkSum = (~checkSum);
	return (unsigned char)checkSum;

}

void App::AppCore::setEngageMode()
{	
	App::ComCore cc;
	//dfcv_mining_msgs::Ros_To_Can can_to_can_msg;
	auto can_to_can_msg = std::make_shared<dfcv_mining_msgs::Ros_To_Can>();
	//dfcv_mining_msgs::Ros_To_Can msgCan;

	can_to_can_msg.can_id = 0xCFF649E;
	can_to_can_msg.channel_id = A_CAN_CHANNEL_ID; //P-CAN ad command
	can_to_can_msg.time_pub = deeproute::now();
	can_to_can_msg.time_meas = deeproute::now();

	for(int i= 0; i < 8; i ++)
	{
		can_to_can_msg.data[i] = 0;
	}

	autodrive_counter++;
	autodrive_counter = autodrive_counter%15;
	can_to_can_msg.data[1] = autodrive_counter;
	uint8_t a[8] = {};

	for(int i = 0 ; i < 8; i++)
	{
		a[i] = can_to_can_msg.data[i];
	}

	//unsigned char test_admode = 1;
	can_to_can_msg.data[2] = (engage_mode << 4) + enable_mode;
	a[2] = can_to_can_msg.data[2];
	can_to_can_msg.data[0] = (uint8_t)CRC8(a);
//	uint8_t cksum = 0;

	for(int i= 0; i < 8; i ++)
	{
		logger.LogInfo()<<"can_to_can_msg.data["<<i<<"] is"<<can_to_can_msg.data[i];
	}
//	cksum = cksum + (counter & 0x0f) + 0x0C + 0xff + 0x64 + 0x9E;
//	cksum = ((cksum >> 4) + cksum) & 0x0f;
//	can_to_can_msg.data[7] = (uint8_t)((uint8_t)(counter & 0x0f) | (cksum << 4));
//	counter++;
	//ara::log::Logger& logger = ara::log::LogManager::defaultLogContext();
	//logger.LogInfo()<<"engage_mode: "<<engage_mode<<"enable_mode"<<enable_mode;
	cc.intercan_pub->Write(can_to_can_msg);
	//ros2canbusAB(can_to_can_msg);	//11111111111
	dfcv_mining_msgs::Ros_To_Can *can_to_can_msgPtr = &can_to_can_msg;//zxp 0904	处理报错	11111
	cc.ros2ToCanHandle(can_to_can_msgPtr);//zxp 0904	处理报错	11111
//	can_to_can_msg.channel_id = C_CAN_CHANNEL_ID; //P-CAN ad command
//	ros2canbusAB(can_to_can_msg);
    TboxExMsg.Move_TaskOperation = pb_obj_tbox_move->questoperation();
    TboxExMsg.Joint_Tasktype = pb_obj_tbox_joint->questtype();
    TboxExMsg.Drop_TaskOperation = pb_obj_tbox_drop->questoperation();

    if(Ctrl_lift_state == 1)
    {
    	pb_obj_tbox_drop->set_questoperation(0);
      	pb_obj_tbox_drop->set_questtype(0);
      	TboxExMsg.Drop_TaskOperation = 0;
      	//hasTBoxDrop=false;
    }
    if(plat_ctrl == true)
    {
    	/* 0603 TboxExMsg 发送10s*/
    	//    	TBoxExMsg_pub.publish(TboxExMsg);
    	   //app.DROP.questoperation = 1;
		if(pb_obj_tbox_drop->questoperation() == 1 || TboxExMsg_sendState == true)//接下来连续10s，都发送的是1
		{
			TboxExMsg_sendState = true;
			//发10s，之后职为false
			if(flag == 1)
			{
				start_TboxExMsg = ros::Time::now().toSec();
				flag = 0;
			}
			TboxExMsg.Drop_TaskOperation = 1;
			cc.TBoxExMsg_pub->Write(TboxExMsg);
			end_TboxExMsg = ros::Time::now().toSec();
			logger.LogInfo()<<"++++++++++++++++===========time: ==============+++++++++++++++"<<end_TboxExMsg - start_TboxExMsg;
			if(end_TboxExMsg - start_TboxExMsg > 10)//大于10s 退出发送1
			{
				TboxExMsg_sendState = false;
				flag = 1;
				pb_obj_tbox_drop->set_questoperation(0);
			}
		}
		else
		{
			TboxExMsg.Drop_TaskOperation = 0;
			cc.TBoxExMsg_pub->Write(TboxExMsg);
		};//发送的不是1，直接发送
		logger.LogInfo()<<"++++++++++++++++===========pb_obj_tbox_drop->questoperation(): ==============+++++++++++++++"<<pb_obj_tbox_drop->questoperation();
    }
    else if (plat_ctrl==false)
	{
    	TboxExMsg.Move_TaskOperation = 0;
    	TboxExMsg.Joint_Tasktype = 0;
    	TboxExMsg.Drop_TaskOperation = 0;
    	TboxExMsg.Move_PassPathId.clear();
    	TboxExMsg.Move_PathIdLength = 0;
    	cc.TBoxExMsg_pub->Write(TboxExMsg);
	}
//	drdtu::Ros2canfdbus canfd_msg;
//	canfd_msg.can_id = 0x19FF509E;//0xCFF649E;
//	canfd_msg.channel_id = RADAR6_CHANNEL_ID; //P-CAN ad command
//	canfd_msg.time_meas = deeproute::now();
//	for(int i= 0; i < 64; i ++)
//	{
//		canfd_msg.data[i] = 0;
//	}
//	canfdbus2ros_pub.publish(canfd_msg);

	/*
	pub_and_sub::CTRL_TO_CAN can_to_test_msg;
	can_to_test_msg.can_id = 0x18271463;
	can_to_test_msg.channel_id = 3;
	can_to_test_msg.time_pub = deeproute::now();
	can_to_test_msg.time_meas = deeproute::now();
	for(int i= 0; i < 8; i ++)
	{
		can_to_test_msg.data[i] = i* 10 + i;
	}
	//test_pub.publish(can_to_test_msg);
	*/
    //logger.LogInfo()<<"----------------------EPS_To_Ctrl_.DE_phiSteerAngle--------------"<<EPS_To_Ctrl_.DE_phiSteerAngle;
}
