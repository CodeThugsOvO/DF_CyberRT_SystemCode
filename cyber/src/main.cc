/*
* Copyright (c) Huawei Technologies Co., Ltd. 2012-2020. All rights reserved.
* Description: MDC CANFD消息实例化demo样例代码（基于ARXML)，需要基于不同硬件连线和ARXML配置进行修改调测，不可直接商用
* Author: 
* Create: 2019-12-23
 */
#include <thread>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <unistd.h>
#include <functional>
#include "cyber/include/wholeArea.h"

#include "cyber/examples/proto/examples.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"


using namespace mdc::mdccan;
using namespace CanFdMsgHandle;
using namespace ara::log;
using namespace AppSpace_new;//zxp	0910

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::examples::proto::Chatter;


void CanFdLDataRecieved(App::AppCore& app, uint8_t channelId, const CanFdBusDataParamL &canBusDataL);
void CanFdSDataRecieved(App::AppCore& app, uint8_t channelId, const CanFdBusDataParamS &canBusDataS);

McuCanFdInterface canab1ApInterfaceNode;
McuCanFdInterface radar5ApInterfaceNode;
McuCanFdInterface ecu2ApInterfaceNode;	 
McuCanFdInterface radar1ApInterfaceNode;	
McuCanFdInterface radar2ApInterfaceNode;
McuCanFdInterface radar3ApInterfaceNode;
McuCanFdInterface gps1ApInterfaceNode;	 
McuCanFdInterface ecu0ApInterfaceNode;
McuCanFdInterface radar4ApInterfaceNode;
McuCanFdInterface radar6ApInterfaceNode;
McuCanFdInterface ecu1ApInterfaceNode;	 
McuCanFdInterface canab2ApInterfaceNode;
msg_tbox_move *pb_obj_tbox_move;
//TBOX_MOVE plan;
int main(int argc, char *argv[])
{
    ara::log::InitLogging("CAN", "can2cyber v0.0.1", LogLevel::kVerbose, (LogMode::kRemote | LogMode::kConsole));

	apollo::cyber::Init(argv[0]);
  	// create talker node
  	auto talker_node = apollo::cyber::CreateNode("talker");
  	// create talker
  	//auto talker = talker_node->CreateWriter<Chatter>("channel/chatter");
	AINFO << "can to cyber rt version v0.0.1";

    //AppSpace_old::App app;
	App::AppCore app;

    logger.LogInfo()<<"can2cyber version v0.0.1 ";

    //canab1ApInterfaceNode.Init(CANAB1_CHANNEL_ID);
    //radar5ApInterfaceNode.Init(RADAR5_CHANNEL_ID);
    //ecu2ApInterfaceNode .Init(ECU2_CHANNEL_ID);
    radar1ApInterfaceNode.Init(RADAR1_CHANNEL_ID);
    //radar2ApInterfaceNode.Init(RADAR2_CHANNEL_ID);
    //radar3ApInterfaceNode.Init(RADAR3_CHANNEL_ID);
    gps1ApInterfaceNode.Init(GPS1_CHANNEL_ID);
    //ecu0ApInterfaceNode.Init(ECU0_CHANNEL_ID);
    radar4ApInterfaceNode.Init(RADAR4_CHANNEL_ID);
    radar6ApInterfaceNode.Init(RADAR6_CHANNEL_ID);
    //ecu1ApInterfaceNode.Init(ECU1_CHANNEL_ID);
    //canab2ApInterfaceNode.Init(CANAB2_CHANNEL_ID);
    //sleep(10);
  	// 注册CAN帧处理回调函数
    canab1ApInterfaceNode.RegisterRxEventCallBackFunc(
        std::bind(&CanFdSDataRecieved, std::ref(app), std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanFdLDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2));
    radar5ApInterfaceNode.RegisterRxEventCallBackFunc(
        std::bind(&CanFdSDataRecieved, std::ref(app), std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanFdLDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2));
    ecu2ApInterfaceNode .RegisterRxEventCallBackFunc(
        std::bind(&CanFdSDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanFdLDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2));
    radar1ApInterfaceNode.RegisterRxEventCallBackFunc(
        std::bind(&CanFdSDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanFdLDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2));
    radar2ApInterfaceNode.RegisterRxEventCallBackFunc(
        std::bind(&CanFdSDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanFdLDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2));
    radar3ApInterfaceNode.RegisterRxEventCallBackFunc(
        std::bind(&CanFdSDataRecieved, std::ref(app), std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanFdLDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2));
    gps1ApInterfaceNode.RegisterRxEventCallBackFunc(
        std::bind(&CanFdSDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanFdLDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2));
    ecu0ApInterfaceNode.RegisterRxEventCallBackFunc(
        std::bind(&CanFdSDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanFdLDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2));
    radar4ApInterfaceNode.RegisterRxEventCallBackFunc(
        std::bind(&CanFdSDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanFdLDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2));
    radar6ApInterfaceNode.RegisterRxEventCallBackFunc(
        std::bind(&CanFdSDataRecieved, std::ref(app), std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanFdLDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2));
    ecu1ApInterfaceNode.RegisterRxEventCallBackFunc(
        std::bind(&CanFdSDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanFdLDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2));
    canab2ApInterfaceNode.RegisterRxEventCallBackFunc(
        std::bind(&CanFdSDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanFdLDataRecieved, std::ref(app),  std::placeholders::_1, std::placeholders::_2));

    Rate rate(1.0);

    while (apollo::cyber::OK()) {
    	rate.Sleep();
    }
    return 0;

}

/*****************************************************************************
 函 数 名  : CanFdSDataRecieved
 功能描述  : 处理从MCU收到的CANFD短包函数 需基于业务需求自行实现
 输入参数  : uint8_t channelId 0~11 CanFdBusDataParamS& canBusDataS
 返 回 值  : 无
 修改历史  : NA
*****************************************************************************/

void CanFdSDataRecieved(App::AppCore& app, uint8_t channelId, const CanFdBusDataParamS &canBusDataS)
{
    // 入参判断
    if (channelId < 0 || channelId >= CAN_CHANNEL_NUM) 
	{
        return;
    }

    canbus::Ros2canbus msg;
	
    msg.channel_id = channelId;
    u_int8_t rev_data[8] ={0};
    // 打印接收的CANFd 短包
    for (unsigned int i = 0; i < canBusDataS.elementList.size(); i++) 
	{

    	/*logger.LogInfo()<<"Rev from CAN: canIdType: "<<canBusDataS.elementList[i].canIdType
      		<<", canId: "<< canBusDataS.elementList[i].canId
        	<<", validLen: "<< canBusDataS.elementList[i].validLen
			<<", channelId" << channelId;*/

    	// logger.LogInfo()<<"**************** canId:***************************="<<canBusDataS.elementList[i].canId;
        for (unsigned int j = 0; j < canBusDataS.elementList[i].validLen; j++) 
		{
            rev_data[j] = canBusDataS.elementList[i].data[j];
            //printf("%i ",rev_data[j]);
        }

        //switch id for different msg
        switch(canBusDataS.elementList[i].canId)
        {

			case 0x0CF00300: //EEC2
			{
				msg_eec2_e* eec2 =  reinterpret_cast<msg_eec2_e*>((msg_eec2_e*)rev_data);
				app.setEEC2_E(eec2);
			}
			break;

			case 0x0CF00400: //EEC1
			{	
				msg_eec1_e* eec1 = reinterpret_cast<msg_eec1_e*>((msg_eec1_e*)rev_data);
				//EEC1_E* eec1 =  reinterpret_cast<EEC1_E*>((EEC1_E*)rev_data);
				app.setEEC1_E(eec1);
				/*
				for (unsigned int j = 0; j < canBusDataS.elementList[i].validLen; j++) {
					//logger.LogInfo() << rev_data[j];// = canBusDataS.elementList[i].data[j];

				                }
				logger.LogInfo()<<"EEC1_E: "
							<<", Engine_torque_mode: "<< eec1->Engine_torque_mode
								<<", Actual_engine_torque_high_resolution: "<<eec1->Actual_engine_torque_high_resolution
								<<",Driver_demand_engine_torque "<< eec1->Driver_demand_engine_torque
								<<",Actual_engine_torque"<< eec1->Actual_engine_torque
								<<",Engine_speed "<< eec1->Engine_speed_low + (u_int16_t)(eec1->Engine_speed_high << 8)
								<<",address_of_controlling_device "<< eec1->address_of_controlling_device
								<<",not_defined_6 "<< eec1->not_defined_6
								<<",engine_demand_percent_torque "<< eec1->engine_demand_percent_torque;
				*/
			}
			break;

			case 0x0CF00203: //ETC1
			{
				msg_etc1_tcu* etc1 =  reinterpret_cast<msg_etc1_tcu*>((msg_etc1_tcu*)rev_data);
				app.setETC1_TCU(etc1);
			}
			break;

			case 0x18F00503: //ETC2
			{
				msg_etc2_tcu* etc2 =  reinterpret_cast<msg_etc2_tcu*>((msg_etc2_tcu*)rev_data);
				app.setETC2_TCU(etc2);
			}
			break;

			case 0x18FEBF0B: //EBC2
			{
				msg_ebc2_ebs* ebc2 =  reinterpret_cast<msg_ebc2_ebs*>((msg_ebc2_ebs*)rev_data);
				app.setEBC2_EBS(ebc2);
			}
			break;

			case 0x18F0010B: //EBC1
			{
				msg_ebc1_ebs* ebc1 =  reinterpret_cast<msg_ebc1_ebs*>((msg_ebc1_ebs*)rev_data);
				app.setEBC1_EBS(ebc1);
			}
			break;

			case 0x18FF4713: //INFO1
			{
				msg_ctlinfo1_eps1_kb* info1 =  reinterpret_cast<msg_ctlinfo1_eps1_kb*>((msg_ctlinfo1_eps1_kb*)rev_data);
				app.setCTLINFO1_EPS1_KB(info1);
			}
			break;

			case 0x18FF4813: //INFO2
			{
				msg_ctlinfo2_eps1_kb* info2 =  reinterpret_cast<msg_ctlinfo2_eps1_kb*>((msg_ctlinfo2_eps1_kb*)rev_data);
				app.setCTLINFO2_EPS1_KB(info2);
//				logger.LogInfo()<<"msg_ctlinfo2_eps1_kb: "
//											<<", CtrlModResp_Overridedetection: "<< info2->CtrlModResp_Overridedetection;
			}
			break;

			case 0x18FF6832: //INFO2
			{
				msg_state_bcm* bcm =  reinterpret_cast<msg_state_bcm*>((msg_state_bcm*)rev_data);
				app.setState_BCM(bcm);
			}
			break;

			case 0x0C000003: //tsc1_te
			{
				msg_tsc1_te* tsc1 =  reinterpret_cast<msg_tsc1_te*>((msg_tsc1_te*)rev_data);
				app.setTSC1_TE(tsc1);
			}
			break;

            //			case 0x18FE4A03: //0x0C010305: //tc1
            //			{
            //				TC_1* tc1 =  reinterpret_cast<TC_1*>((TC_1*)rev_data);
            //				app.setTC_1(tc1);
            //			}
			case 0x18FE4A03:  //old tc1
			{
			    msg_etc7* etc7 =  reinterpret_cast<msg_etc7*>((msg_etc7*)rev_data);
				app.setETC7(etc7);
			}
			break;

			case 0x19D2E2D6: //RADAR_Front
			{
				msg_radar_front* radar1 = reinterpret_cast<msg_radar_front*>((msg_radar_front*)rev_data);
				app.setRADAR_Front(radar1);
			}
			break;

			case 0x19D2E3D6: //RADAR_Back
			{
				msg_radar_back* radar2 = reinterpret_cast<msg_radar_back*>((msg_radar_back*)rev_data);
				app.setRADAR_Back(radar2);
			}
			break;

			case 0x19D2E4D6: //RADAR_Side1
			{
				msg_radar_side1* radar3 = reinterpret_cast<msg_radar_side1*>((msg_radar_side1*)rev_data);
				app.setRADAR_Side1(radar3);
			}
			break;

			case 0x19D2E5D6: //RADAR_Side2
			{
				msg_radar_side2* radar4 = reinterpret_cast<msg_radar_side2*>((msg_radar_side2*)rev_data);
				app.setRADAR_Side2(radar4);
			}
			break;

			case 0x18FF669E: //BodyControl_ADCU
			{
				msg_body_control_adcu* pb_obj_body_control_adcu = reinterpret_cast<msg_body_control_adcu*>((msg_body_control_adcu*)rev_data);
				app.setBodyControl_ADCU(pb_obj_body_control_adcu);
			}
			break;

			case 0x18FE1264: //EPBS1_EPB
			{
				msg_epbs1_epb* pb_obj_epbs1_epb = reinterpret_cast<msg_epbs1_epb*>((msg_epbs1_epb*)rev_data);
				app.setEPBS1_EPB(pb_obj_epbs1_epb);
			}
			break;

			case 0x18EA64EF: //EPBC1_ADU
			{
				msg_epbc1_adu* pb_obj_epbc1_adu = reinterpret_cast<msg_epbc1_adu*>((msg_epbc1_adu*)rev_data);
				app.setEPBC1_ADU(pb_obj_epbc1_adu);
			}
			break;

			case 0x18FDA403: //PTODE
			{
				msg_ptode* pb_obj_ptode = reinterpret_cast<msg_ptode*>((msg_ptode*)rev_data);
				app.setPTODE(pb_obj_ptode);
			}
			break;

			case 0x18FDA49E: //PTODE
			{
				msg_ptode_adcu* pb_obj_ptode_adcu = reinterpret_cast<msg_ptode_adcu*>((msg_ptode_adcu*)rev_data);
				app.setPTODE_ADCU(pb_obj_ptode_adcu);
			}
			break;

			//-------------------------------------------new------------------------------
			case 0x0CFF11EF: //MS1_MCU
			{
				msg_ms1_mcu* inter_MS1_MCU1 = reinterpret_cast<msg_ms1_mcu*>((msg_ms1_mcu*)rev_data);
				app.setMS1_MCU(inter_MS1_MCU1);
			}
			break;

			case 0x0CFF0A31: //MC1_HCU
			{
				msg_mc1_hcu* inter_MC1_HCU1 = reinterpret_cast<msg_mc1_hcu*>((msg_mc1_hcu*)rev_data);
				app.setMC1_HCU(inter_MC1_HCU1);
			}
			break;

			case 0x18FEDF00: //EEC3_E
			{
				msg_eec3_e* inter_EEC3_E1 = reinterpret_cast<msg_eec3_e*>((msg_eec3_e*)rev_data);
				app.setEEC3_E(inter_EEC3_E1);
			}
			break;

			case 0x18FEF100: //CCVS1_HCU
			{
				msg_ccvs1_hcu* inter_CCVS1_HCU1 = reinterpret_cast<msg_ccvs1_hcu*>((msg_ccvs1_hcu*)rev_data);
				app.setCCVS1_HCU(inter_CCVS1_HCU1);
			}
			break;

			case 0x0C00000B: //TSC1_AE
			{
				msg_tsc1_ae* inter_tsc1_ae = reinterpret_cast<msg_tsc1_ae*>((msg_tsc1_ae*)rev_data);
				app.setTSC1_AE(inter_tsc1_ae);
			}
			break;

			case 0x18FE700B: //CVW_EBS
			{
				msg_cvw_ebs* inter_cvw_ebs = reinterpret_cast<msg_cvw_ebs*>((msg_cvw_ebs*)rev_data);
				app.setCVW_EBS(inter_cvw_ebs);
			}
			break;

			case 0x18FF3C50: //EPB1
			{
				msg_epb1* inter_epb1 = reinterpret_cast<msg_epb1*>((msg_epb1*)rev_data);
				app.setEPB1(inter_epb1);
			}
			break;

			case 0x09B:		//GPS_time
			{
				app.setXCID_GPS_time(&rev_data[0]);
			}
			break; 

			case 0x60B:     // xy_gyro
			{
				app.setXCID_X_Y_Gyro(&rev_data[0]);
			}
			break;

			case 0x50B:    //xy_acc
			{
				app.setXCDI_X_Y_acce(&rev_data[0]);
			}
			break;

			case 0x70B:		// z_gyro_acc
			{
				app.setXCID_Z_Gyro_Acce(&rev_data[0]);
			}
			break;

			case 0x10B:		//Triaxial_attitude
			{
				app.setXCDI_Triaxial_attitude(&rev_data[0]);///mistake
			}
			break;
			
			default: 
			break;
		}

        /*
        msg.time_meas = deeproute::now();
        msg.time_pub = deeproute::now();
	    msg.can_id = canBusDataS.elementList[i].canId;
        app.canbus2ros_pub.publish(msg);
        */
    }
    return;
}

/*****************************************************************************
 函 数 名  : CanFdLDataRecieved
 功能描述  : 处理从MCU收到的CANFD长包函数 需基于业务需求自行实现
 输入参数  : uint8_t channelId 0~11 CanFdBusDataParamS& canBusDataL
 返 回 值  : 无
 修改历史  : NA
****************************************************************************/
void CanFdLDataRecieved(App::AppCore& app, uint8_t channelId, const CanFdBusDataParamL &canBusDataL)
{
	App::ComCore cc;
    // 入参判断
    if (channelId < 0 || channelId >= CAN_CHANNEL_NUM) 
	{
        return;
    }

    drdtu::Ros2canfdbus msg;
    logger.LogInfo()<<"validLen: "<<"tttt";
    // 打印接收的CANFd 长包
    for (unsigned int i = 0; i < canBusDataL.elementList.size(); i++) 
	{
        //logger.LogInfo()<<"canIdType: "<<canBusDataL.elementList[i].canIdType;
         //canId 以十进制打印
        logger.LogInfo()<<"canId: "<<canBusDataL.elementList[i].canId;
        logger.LogInfo()<<"validLen: "<<canBusDataL.elementList[i].validLen;
        u_int8_t rev_data[64] = {0};
        msg.can_id = canBusDataL.elementList[i].canId;

        for (unsigned int j = 0; j < canBusDataL.elementList[i].validLen; j++) {
        	rev_data[j] = canBusDataL.elementList[i].data[j];
        	//logger.LogInfo()<<"rev_data: "<<rev_data[j];
        }

        msg.channel_id = channelId;
        logger.LogInfo()<<"msg.can_id: "<<msg.can_id;
        switch(msg.can_id)
        {
			case 0x19FF51DE:
			{
				auto_state = msg.data[0];
				if(auto_state==1)
				{
					//app.setAutoState(true);
				}else
				{
					//app.setAutoState(false);
				}
				//app.tboxAutoStateReply(auto_state,1);//zxp 0911 注释
				cc.tboxAutoStateReply(auto_state,1);
				//drdtu::Ros2canfdbus can_to_tbox_autostate_msg;
				//can_to_tbox_autostate_msg.time_meas = deeproute::now();
				//can_to_tbox_autostate_msg.time_pub = deeproute::now();
				//can_to_tbox_autostate_msg.can_id = 0x19ff559e;
				//can_to_tbox_autostate_msg.channel_id = TBOX_CHANNEL_ID;
				//canfdbus2ros_pub.publish(can_to_tbox_autostate_msg);
			}
			break;

			case 0x19FF52DE:
			{
				logger.LogInfo()<<"validLen: "<<"0x19FF52DE";

				for (int k=0;k<64;k++) 
				{
					logger.LogInfo()<<"rev_data: "<<rev_data[k];
				}
				if(rev_data[1]==1)
				{
					app.planIds.clear();
					msg_tbox_move* planTemp = reinterpret_cast<msg_tbox_move*>((msg_tbox_move*)rev_data);
					pb_obj_tbox_move->set_packnum(planTemp->packnum());
					pb_obj_tbox_move->set_packseq(planTemp->packseq());

					for(int i = 0; i < 8; i++){
						pb_obj_tbox_move->set_questnum(i, planTemp->questnum(i));
					}
					//memcpy(&plan.questnum, planTemp->questnum(), sizeof(u_int64_t));

					pb_obj_tbox_move->set_questoperation(planTemp->questoperation());
					pb_obj_tbox_move->set_questtype(planTemp->questtype());
					pb_obj_tbox_move->set_limitspeed(planTemp->limitspeed());

					for(int i = 0; i < 8; i++){
						pb_obj_tbox_move->set_start_x(i, planTemp->start_x(i));
					}
					//memcpy(&pb_obj_tbox_move->start_X(), planTemp->start_X, 8);

					for(int i = 0; i < 8; i++){
						pb_obj_tbox_move->set_start_y(i, planTemp->start_y(i));
					}
					//memcpy(&pb_obj_tbox_move->start_Y(), planTemp->start_Y,  8);

					for(int i = 0; i < 8; i++){
						pb_obj_tbox_move->set_start_z(i, planTemp->start_z(i));
					}					
					//memcpy(&pb_obj_tbox_move->start_Z(), planTemp->start_Z,  8);

					for(int i=0;i<8;i++)
					{
						logger.LogInfo()<<"planTemp->start_y "<<planTemp->start_y(i) ;
					}
					for(int i=0;i<8;i++){
						pb_obj_tbox_move->set_end_x(i, planTemp->end_x(i));
					}
					//memcpy(&pb_obj_tbox_move->end_X(), planTemp->end_x,  8);
					for(int i=0;i<8;i++){
						pb_obj_tbox_move->set_end_y(i, planTemp->end_y(i));
					}
					//memcpy(&pb_obj_tbox_move->end_Y(), planTemp->end_y,  8);
					for(int i=0;i<8;i++){
						pb_obj_tbox_move->set_end_z(i, planTemp->end_z(i));
					}
					//memcpy(&pb_obj_tbox_move->end_Z(), planTemp->end_z,  8);
					logger.LogInfo()<<"planTemp->packnum "<<planTemp->packnum();
					logger.LogInfo()<<"planTemp->packseq "<<planTemp->packseq();
					//logger.LogInfo()<<"planTemp->questnum "<<planTemp->questnum();
					//logger.LogInfo()<<"planTemp->start_x "<<planTemp->start_x();
					//logger.LogInfo()<<"planTemp->start_y "<<planTemp->start_y();
					//logger.LogInfo()<<"pb_obj_tbox_move.questnum "<<pb_obj_tbox_move->questnum();
					logger.LogInfo()<<"planTemp->questtype "<<planTemp->questtype();
					logger.LogInfo()<<"----======---planTemp->questoperation------========--- "<<planTemp->questoperation();
					logger.LogInfo()<<"pb_obj_tbox_move.questoperation "<<pb_obj_tbox_move->questoperation();
					logger.LogInfo()<<"planTemp->limitspeed "<<planTemp->limitspeed();
					logger.LogInfo()<<"planTemp"<<planTemp;
				}
				else if(rev_data[1] != 1)
				{
					msg_tbox_move_id* planId = reinterpret_cast<msg_tbox_move_id*>((msg_tbox_move_id*)rev_data);
					//TBOX_MOVE_id* planId = reinterpret_cast<TBOX_MOVE_id*>((TBOX_MOVE_id*)rev_data);
					logger.LogInfo()<<"planId->idlength "<<planId->idlength();
					//for(int l=0;l<planId->idlength;l++){
					for(int l = 0; l < 13; l++)
					{                                               //12.10下矿
						logger.LogInfo()<<"planId->id[i] "<<planId->id(l);
						if(planId->id(l) != 0)
						{                                            //12.10下矿
							app.planIds.push_back(planId->id(l));
						}
					}
					if(rev_data[1] == rev_data[0])
					{
						logger.LogInfo()<<"validLen: "<<"2222233333333";
						app.setTbox(NULL, NULL, pb_obj_tbox_move, planId);
						logger.LogInfo()<<"validLen: "<<"1212121";
						//app.plan.questoperation = 1;
						hasTBoxMove = true;

						if (pb_obj_tbox_move->questoperation() == 2)
						{
							hasTBoxJoint = true;
						}
						plat_ctrl = true;
						//app.publishTboxMsg();
						//app.tboxMoveReply(true,0,1);
					}
				}
			}
			break;

			case 0x19FF53DE:
			{
				logger.LogInfo()<<"validLen: "<<"0x19FF53DE";
				msg_tbox_drop* drop = reinterpret_cast<msg_tbox_drop*>((msg_tbox_drop*)rev_data);
				//app.DROP.packnum = drop->packnum;
				//u_int32_t buf1[8] = {0};
				for(int i = 0;i < 8; i++){
					//buf1[i] = drop->questnum[i];
					//app.pb_obj_tbox_drop->questnum(i) = drop->questnum(i);	
					app.pb_obj_tbox_drop->set_questnum(i, drop->questnum(i));				
				}				
				//memcpy(&app.DROP.questnum, drop->questnum, 64);

				app.pb_obj_tbox_drop->set_questtype(drop->questtype());
				//app.DROP.questoperation=drop->questoperation;

				if(drop->questoperation() == 1)
				{
					app.pb_obj_tbox_drop->set_questoperation(1);
				}
				//app.setTboxDrop(drop);
				app.setTbox(NULL, drop, NULL, NULL);
				//Ctrl_lift_state = 0;
				hasTBoxDrop = true;
				plat_ctrl = true;
				//app.publishTboxMsg();
				//app.tboxDropReply(drop,0,1);
			}
			break;

			case 0x19FF54DE:
			{
				logger.LogInfo()<<"validLen: "<<"0x19FF54DE";
				msg_tbox_joint* joint = reinterpret_cast<msg_tbox_joint*>((msg_tbox_joint*)rev_data);
				app.pb_obj_tbox_joint->set_questtype(joint->questtype());
				logger.LogInfo()<<"++++++++++++joint->questtype++++++++++++"<<joint->questtype();
				//app.setTboxJoint(joint);
				app.setTbox(joint, NULL, NULL, NULL);
				hasTBoxJoint = true;
				plat_ctrl = true;
				//app.publishTboxMsg();
				//app.tboxJointReply(joint,0,1);
			}
			break;

			default:
			break;
		}
    }
    return;
}

