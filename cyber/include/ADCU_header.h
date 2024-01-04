#include <sys/types.h>
#ifndef _ADCU_HEADER_H
#define _ADCU_HEADER_H

// typedef struct
// {
// 	union
// 	{
//         u_int8_t first_byte;
//         struct
//         {
//         	u_int8_t  Engine_torque_mode : 4;
//         	u_int8_t  Actual_engine_torque_high_resolution : 4;
//         };

// 	};
// 	u_int8_t Driver_demand_engine_torque ; //1 125
// 	u_int8_t Actual_engine_torque; //2 140
// 	u_int8_t Engine_speed_low ; //3 20
// 	u_int8_t Engine_speed_high ; // 4 20
// 	u_int8_t address_of_controlling_device ; //5 255
// 	union
// 	{
//         u_int8_t sixth_byte;
//         struct
//         {
//         	u_int8_t  engine_starter_mode : 4;
//         	u_int8_t  rev1 : 4;
//         };

// 	}; //6 148
// 	u_int8_t engine_demand_percent_torque ; //7 0
// }EEC1_E; //P-CAN

// typedef struct
// {
// 	union
// 	{
//         u_int8_t first_byte;
//         struct
//         {
//         	u_int8_t  AP_low_idle_switch : 2;
//         	u_int8_t  AP_kickdown_switch : 2;
//         	u_int8_t  Road_speed_limit_status  : 4;
//         };
// 	};//0
// 	u_int8_t Accelerator_pedal_position  ; //1
// 	u_int8_t Percent_load_at_current_speed ; //2
// 	u_int8_t Remote_accelerator ; //3
// 	u_int8_t Accelerator_pedal_position_2; //4
// 	u_int8_t not_defined_5 ; //5
// 	u_int8_t Actual_Maximum_Available_Engine ; //6
// 	u_int8_t not_defined_7 ; //7
// }EEC2_E; //P-CAN

// typedef struct
// {
// 	union
// 	{
//         u_int8_t first_byte;
//         struct
//         {
//         	u_int8_t	Transmission_driveline_engaged : 2;
//         	u_int8_t	Transmission_Torque_Converter_Lockup_Engaged : 2;
//         	u_int8_t	Transmission_shift_in_process : 2;
//         	u_int8_t	Transmission_Torque_Converter_Lockup_Transition : 2;

//         };


// 	};//0
// 	u_int16_t	Transmission_Output_Shaft_Speed; //1,2
// 	u_int8_t	Percent_clutch_slip; //3
// 	union
// 		{
// 	        u_int8_t fifth_byte;
// 	        struct
// 	        {
// 	        	u_int8_t	Engine_Momentary_Overspeed_Enable : 2;
// 	        	u_int8_t	Progressive_Shift_Disable : 2;
// 	        	u_int8_t	Momentary_Engine_Maximum_Power_Enable : 2;
// 	        	u_int8_t	not_defined_5_6 : 2;

// 	        };


// 		};
// 	u_int16_t	Transmission_input_shaft_speed; //5,6
// 	u_int8_t	Source_address_of_controlling_device_for_transmission_control; //7

// }ETC1_TCU; //P-CAN  0x0CF00203

// typedef struct
// {
// 	u_int8_t	selected_gear; //0
// 	u_int8_t	Transmission_Actual_Gear_Ratio_low; //1
// 	u_int8_t	Transmission_Actual_Gear_Ratio_high; //2
// 	u_int8_t	Current_Gear; //3
// 	/*u_int8_t	not_defined_4; //4
// 	u_int8_t	current_range; //5
// 	u_int8_t	not_defined_6; //6
// 	u_int8_t	not_defined_7; //7*/

// }ETC2_TCU; //P-CAN   0x18F00503

// typedef struct
// {

// 	union
// 	{
// 		u_int8_t first_byte;
// 		struct
// 		{
// 			u_int8_t ASR_Engine_Control_Active :2;
// 			u_int8_t ASR_Brake_Control_Active:2;
// 			u_int8_t Antilock_braking:2;
// 			u_int8_t EBS_brake_switch:2;

// 		};


// 	};//0


// 	u_int8_t	Brake_Pedal_Position;// 1
// 	union
// 	{
// 		u_int8_t third_byte;
// 		struct
// 		{
// 			u_int8_t	ABS_Offroad_Switch : 2;
// 			u_int8_t	ASR_Offroad_Switch : 2;
// 			u_int8_t	ASR_hill_holder_Switch: 2;
// 			u_int8_t	Traction_Control_Override_Switch : 2;

// 		};

// 	};//2
// 	union
// 	{
// 		u_int8_t fourth_byte;
// 		struct
// 		{
// 			u_int8_t	Accelerator_Interlock_Switch : 2;
// 			u_int8_t	Engine_Derate_Switch : 2;
// 			u_int8_t	Auxiliary_Engine_Shutdown_Switch : 2;
// 			u_int8_t	Remote_Accelerator_Enable_Switch : 2;

// 		};

// 	};//3

// 	u_int8_t	Engine_Retarder_Selection;		//4
// 	union
// 	{
// 		u_int8_t sixth_byte;
// 		struct
// 		{
// 			u_int8_t	ABS_Fully_Operational : 2;
// 			u_int8_t	EBS_Red_Warning_Lamp_State : 2;
// 			u_int8_t	ABS_EBS_Amber_Warning_State : 2;
// 			u_int8_t	ATC_ASR_Lamp_State : 2;

// 		};

// 	};//5
	
// 	u_int8_t	Source_Address_of_Controlling_Device_for_Brake_Control;// 6

// 	u_int8_t	not_defined_7; //7

// }EBC1_EBS; //C-CAN

//2023.2.6感知定位增加IMU
/* typedef struct
{
	u_int8_t Error_Code ; //
	u_int8_t res1 ; //
	u_int8_t res2 ; //
	u_int8_t res3 ; // 
	u_int8_t res4 ; //
	u_int8_t res5 ; //
	u_int8_t res6 ; //
	u_int8_t res7 ; //
}XCID_Error; //IMU-19FF0001

typedef struct
{
	u_int32_t SampleTime ; //
	u_int8_t res1 ; //
	u_int8_t res2 ; //
	u_int8_t res3 ; // 
 	u_int8_t res4 ; //
	u_int8_t res5 ; //
	u_int8_t res6 ; //
	u_int8_t res7 ; // 
}XCID_SampleTime; //IMU-19FF0005

typedef struct
{
	u_int16_t GroupCounter ; //
	u_int8_t res1 ; //
	u_int8_t res2 ; //
	u_int8_t res3 ; // 
	u_int8_t res4 ; //
	u_int8_t res5 ; //
	u_int8_t res6 ; //
 	u_int8_t res7 ; // 
}XCID_GroupCounter; //IMU-19FF0006

typedef struct
{
	u_int32_t StatusWord;
	u_int8_t res1 ; //
	u_int8_t res2 ; //
	u_int8_t res3 ; // 
	u_int8_t res4 ; //
 	u_int8_t res5 ; //
	u_int8_t res6 ; //
	u_int8_t res7 ; // 
}XCDI_StatusWord; //IMU-19FF0011

typedef struct
{
	u_int16_t Q0 ; //
	u_int16_t Q1 ; //
	u_int16_t Q2 ; //
	u_int16_t Q3 ; //
	u_int8_t res4 ; //
	u_int8_t res5 ; //
	u_int8_t res6 ; //
	u_int8_t res7 ; // 
}XCDI_Quaternion; //IMU-19FF0021

typedef struct
{
	u_int16_t Roll ; //
	u_int16_t Pitch ; //
	u_int16_t Yaw ; //
	u_int8_t res3 ; //
	u_int8_t res4 ; //
	u_int8_t res5 ; //
	u_int8_t res6 ; //
	u_int8_t res7 ; //
}XCDI_EulerAngles; //IMU-19FF0022

typedef struct
{
	u_int16_t DeltaVx ; //
	u_int16_t DeltaVy ; //
	u_int16_t DeltaVz ; //
	u_int8_t Exponent ; //
	u_int8_t res4 ; //
 	u_int8_t res5 ; //
	u_int8_t res6 ; //
	u_int8_t res7 ; // 
}XCDI_DeltaV; //IMU-19FF0031

typedef struct
{
	u_int16_t gyrX ; //
	u_int16_t gyrY ; //
	u_int16_t gyrZ ; //
	u_int8_t res3 ; //
	u_int8_t res4 ; //
 	u_int8_t res5 ; //
    u_int8_t res6 ; //
	u_int8_t res7 ; // 
}XCDI_RateOfTurn; //IMU-19FF0032

typedef struct
{
	u_int16_t DeltaQ0 ; //
	u_int16_t DeltaQ1 ; //
	u_int16_t DeltaQ2 ; //
	u_int16_t DeltaQ3 ; //
 	u_int8_t res4 ; //
	u_int8_t res5 ; //
	u_int8_t res6 ; //
	u_int8_t res7 ; // 
}XCDI_DeltaQ; //IMU-19FF0033

typedef struct
{
	u_int16_t accX ; //
	u_int16_t accY ; //
	u_int16_t accZ ; //
	u_int8_t res3 ; //
	u_int8_t res4 ; //
 	u_int8_t res5 ; //
	u_int8_t res6 ; //
	u_int8_t res7 ; // 
}XCDI_Acceleration; //IMU-19FF0034

typedef struct
{
	u_int16_t freeAccX ; //
	u_int16_t freeAccY ; //
	u_int16_t freeAccZ ; //
	u_int8_t res3 ; //
	u_int8_t res4 ; //
 	u_int8_t res5 ; //
	u_int8_t res6 ; //
	u_int8_t res7 ; // 
}XCDI_FreeAcceleration; //IMU-19FF0035
 */

// typedef struct
// {
// 	union
// 	{
// 		u_int8_t first_byte;
// 		struct
// 		{
// 			u_int8_t    GasHornActivationSwitch:2;
// 			u_int8_t	ElectricHornActivationSwitch : 2;
// 			u_int8_t	ADWiperSwitch : 2;
// 			u_int8_t	ADWiperControl: 2;

// 		};

// 	};  //1
// 	union
// 	{
// 		u_int8_t second_byte;
// 		struct
// 		{
// 			u_int8_t    ADHeadlampSwitch:2;
// 			u_int8_t	ADHeadlampControl : 2;
// 			u_int8_t	TurnLampControl : 2;
// 			u_int8_t	BrakelightCommand: 2;
//         };
// 	};  //2
// 	union
// 	{
// 		u_int8_t third_byte;
// 		struct
// 		{
// 			u_int8_t    ADWasherSwitch:2;
// 			u_int8_t	FogLightSwitch : 2;
// 			u_int8_t	AmbientLightDisplayRequest : 4;
// 		};
// 	};  //3
// 	union
// 	{
// 		u_int8_t fourth_byte;
// 		struct
// 		{
// 			u_int8_t    SeatbeltTightenRequest:2;
// 			u_int8_t	DoorEmergencyUnlocking : 2;
// 			u_int8_t	SeatVibrationRequest : 3;
// 			u_int8_t    not_defined_1: 1;
// 		};
// 	};  //4
// 	union
// 	{
// 		u_int8_t fifth_byte;
// 		struct
// 		{
// 			u_int8_t    HazardlightSwitch:2;
// 			u_int8_t	RearlightSwitch : 2;
// 			u_int8_t	LiftSwitch : 2;
// 			u_int8_t    not_defined_2: 2;
// 		};
// 	};  //5
// }BodyControl_ADCU;


// typedef struct
// {
// 	union
// 	{
// 		u_int8_t first_byte;
// 		struct
// 		{
// 			u_int8_t	EPB_ParkBraStatus : 2;
// 			u_int8_t	AH_ParkActive : 2;
// 			u_int8_t	AH_BraStatus: 2;
// 			u_int8_t	EPB_WorkMode : 2;
// 		};

// 	};   //1
// 	u_int8_t ErrCode;   //2
// 	union
// 	{
// 		 u_int8_t third_byte;
// 		 struct
// 			{
// 					u_int8_t	ChildLockActive : 2;
// 					u_int8_t	LowPreRelParkLimit : 2;
// 					u_int8_t	ResponsePriority: 2;
// 					u_int8_t	IdepedtBraOly : 2;
// 			};

// 	};   //3
// 	union
// 	{
// 		u_int8_t fourth_byte;
// 		struct
// 		{
// 				u_int8_t	PrakBraForcTest : 2;
// 				u_int8_t	DriveHillStatus : 2;
// 				u_int8_t	EPB_BraSwitch: 2;
// 				u_int8_t	ParkBtSwitch : 2;
// 		};

// 	};   //4
// 	union
// 	{
// 		u_int8_t fifth_byte;
// 		struct
// 		{
// 				u_int8_t	RelsBtSwitch : 2;
// 				u_int8_t	AHBtSwitch : 2;
// 				u_int8_t	IdepedtBtSwitch: 2;
// 				u_int8_t	ForcTestBtSwitch : 2;
// 		};

// 	};    //5
// 	union
// 	{
// 		u_int8_t sixth_byte;
// 		struct
// 		{
// 				u_int8_t	SysStatus : 2;
// 				u_int8_t	not_defined_1 : 2;
// 				u_int8_t	not_defined_2: 2;
// 				u_int8_t	not_defined_3 : 2;
// 		};

// 	};	//6

// }EPBS1_EPB;

// typedef struct
// {
//     u_int8_t    ExternalBrakeControl;//0
//     u_int8_t	TractorBrakeControl;//1
// 	u_int8_t	TrailerBrakeControl; //2
// 	u_int8_t	ParkingBrakeControl; //3
// 	u_int8_t	AutoholdBrakeControl;//4
// 	u_int8_t	WorkingMode;//5
// 	u_int8_t	CSUM;//6

// }EPBC1_ADU;

// typedef struct
// {
// 	union
// 	{
// 		u_int8_t first_byte;
// 		struct
// 		{
// 			u_int8_t	not_defined_1:6;
// 		    u_int8_t	EnableSwitch_TransmissionInputShaftPTO_1 : 2;
// 			u_int8_t	not_defined_2:8;
// 			u_int8_t	not_defined_3:6;
// 			u_int8_t	EnableSwitch_TransmissionInputShaftPTO_2 : 2;
// 		};

// 	};   //1
// }PTODE;

// typedef struct
// {
// 	union
// 	{
// 		u_int8_t first_byte;
// 		struct
// 		{
// 			u_int8_t	not_defined_1:6;
// 			u_int8_t	EnableSwitch_TransmissionInputShaftPTO_1 : 2;
// 			u_int8_t	not_defined_2:8;
// 			u_int8_t	not_defined_3:6;
// 			u_int8_t	EnableSwitch_TransmissionInputShaftPTO_2 : 2;
// 		};

// 	};   //1
// }PTODE_ADCU;

// typedef struct
// {
// 	u_int8_t	Front_Axle_Speed_low;//0
// 	u_int8_t	Front_Axle_Speed_high;//1
// 	u_int8_t	Relative_Speed_Front_Axle_Left_Wheel;//2
// 	u_int8_t	Relative_Speed_Front_Axle_Right_Wheel;//3
// 	u_int8_t	Relative_Speed_Rear_Axle_1_Left_Wheel;//4
// 	u_int8_t	Relative_Speed_Rear_Axle_1_Right_Wheel;//5
// 	u_int8_t	Relative_Speed_Rear_Axle_2_Left_Wheel;//6
// 	u_int8_t	Relative_Speed_Rear_Axle_2_Right_Wheel;//7


// }EBC2_EBS; //C-CAN 0x18FEBF0B


// typedef struct
// {
// 	u_int8_t	SteeringAngle_low;//0
// 	u_int8_t	SteeringAngle_high;//1
// 	u_int8_t	SteeringAngleSpeed_low; //2
// 	u_int8_t	SteeringAngleSpeed_high; //3
// 	u_int8_t	SteeringTorque_low;//4
// 	u_int8_t	SteeringTorque_high;//5
// 	u_int8_t	not_defiend;//6
// 	union
// 	{
// 		u_int8_t eighth_byte;
// 		struct
// 		{
// 			u_int8_t Counter : 4;
// 			u_int8_t CSUM : 4;
// 		};
// 	};//7
// }CTLINFO1_EPS1_KB; //P-CAN

// typedef struct
// {
// 	union
// 		{
// 			u_int8_t first_byte;
// 			struct
// 			{
// 				u_int8_t ControlDemandResponse :2;
// 				u_int8_t CtrlModResp_strAsistExcptActRetu:1;
// 				u_int8_t CtrlModResp_Activereturn:1;
// 				u_int8_t CtrlModResp_Overridedetection:1;
// 				u_int8_t CtrlModResp_TorqueDemand:1;
// 				u_int8_t CtrlModResp_AngleDemand:1;
// 				u_int8_t OutputLimitation:1;
// 			};
// 		};//0
// 	u_int8_t	CurrentConsumption;//1
// 	u_int8_t	SteeringAssistTorque_low; //2
// 	u_int8_t	SteeringAssistTorque_high; //2
// 	u_int8_t	TotalTorque_low;//4 5
// 	u_int8_t	TotalTorque_high;//4 5
// 	u_int8_t	not_defiend;//6
// 	union
// 	{
// 		u_int8_t eighth_byte;
// 		struct
// 		{
// 			u_int8_t Counter : 4;
// 			u_int8_t CSUM : 4;
// 		};
// 	};//7
// }CTLINFO2_EPS1_KB; //P-CAN

// typedef struct
// {
// 	u_int8_t	RLS_Brightness_FW;//0
// 	union
// 	{
// 		u_int8_t second_byte;
// 		struct
// 		{
// 			u_int8_t RLS_Rain_Intensity :4;
// 			u_int8_t ADModeOfBCM:2;
// 			u_int8_t PowerOnSwitchOfADCU:2;
// 		};
// 	};//1
// 	union
// 	{
// 		u_int8_t third_byte;
// 		struct
// 		{
// 			u_int8_t ActiveSwitchOfADCU :2;
// 			u_int8_t not_defined_2:2;
// 			u_int8_t AmbientLightStatus:4;
// 		};
// 	};//2
// 	union
// 	{
// 		u_int8_t fourth_byte;
// 		struct
// 		{
// 			u_int8_t SeatVibrationDrive :3;
// 			u_int8_t not_defined_3:1;
// 			u_int8_t Left_Turn_Switch:2;
// 			u_int8_t Right_Turn_Switch:2;
// 		};
// 	};//3
// 	union
// 	{
// 		u_int8_t fifth_byte;
// 		struct
// 		{
// 			u_int8_t	HazardLightSwitch:2; //
// 			u_int8_t	FogLightOpenSwitch:2; //
// 			u_int8_t	FogLightCloseSwitch:2; //
// 			u_int8_t	LiftSwitch:2;//
// 		};
// 	};//4

// 	u_int8_t	not_defined_5;//5
// 	u_int8_t	not_defined_6;//6
// 	u_int8_t	not_defined_7; ;//7

// }State_BCM; //P-CAN


// typedef struct
// {
// 	union
// 	{
// 		u_int8_t first_byte;
// 		struct
// 		{
// 			u_int8_t Engine_Override_Control_Mode :2;
// 			u_int8_t Requested_speed_control_conditions :2;
// 			u_int8_t Override_Control_Mode_Priority :2;
// 			u_int8_t not_defined_0:2;
// 		};
// 	};//0
// 	u_int8_t	Engine_Requested_Speed_low; //1
// 	u_int8_t	Engine_Requested_Speed_high; //2
// 	u_int8_t	Engine_Requested_Torque; //3
// 	union
// 	{
// 		u_int8_t fifth_byte;
// 		struct
// 		{
// 			u_int8_t TSC1_Transmission_Rate : 3;
// 			u_int8_t TSC1_Control_Purpose : 5;
// 		};
// 	};//4
// 	union
// 	{
// 		u_int8_t sixth_byte;
// 		struct
// 		{
// 			u_int8_t Engine_Requested_Torque_High_Resolution : 4;
// 			u_int8_t not_defined_5 : 5;
// 		};
// 	};//5
// 	u_int8_t	not_defined_6; //6
// 	union
// 	{
// 		u_int8_t eighth_byte;
// 		struct
// 		{
// 			u_int8_t Message_counter : 4;
// 			u_int8_t Message_checksum : 4;
// 		};
// 	};//7

// }TSC1_TE; //P-CAN

//typedef struct
//{
//	u_int8_t	first_byte; //0
//	u_int8_t	second_byte; //1
//	union
//	{
//		u_int8_t third_byte;
//		struct
//		{
//			u_int8_t Transmission_Mode_1 : 2;
//			u_int8_t Transmission_Mode_2 : 2;
//			u_int8_t Transmission_Mode_3 : 2;
//			u_int8_t Transmission_Mode_4 : 2;
//		};
//
//	};//2
//
//	u_int8_t	fourth_byte; //3
//	u_int8_t	fifth_byte; //4
//	u_int8_t 	sixth_byte; //5
//	u_int8_t	seventh_byte; //6
//	u_int8_t	eighth_byte; //7
//
//}TC_1; //P-CAN


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
// }ETC7; //P-CAN


// typedef struct
// {
// 	u_int8_t adcu_ctrl_mode; //0
// 		u_int8_t scene_type;     //1
// 		u_int8_t action_type;    //2
// 		u_int8_t error_code;     //3

// 		//	int32_t x;               //2023.1.11
// 		//	u_int32_t rev8;
// 		//	int32_t y;               //2023.1.11
// 		//	u_int32_t rev9;
// 	    //int64_t x;               //2023.2.2
// 	    //int64_t y;               //2023.2.2
// 	    //int64_t z;               //2023.2.2

// 	    int32_t x_1;               //2023.2.22
// 	    int32_t x_2;               //2023.2.22
// 	    int32_t y_1;               //2023.2.22
// 	    int32_t y_2;               //2023.2.22
// 	    int32_t z_1;               //2023.2.22
// 	    int32_t z_2;               //2023.2.22

// 		u_int32_t speed;
// 		//u_int32_t course_angle;
// 		int32_t direction_angle_range;   //2023.1.11
// 		u_int32_t course_angle;         //2023.2.1

// 		u_int8_t sys_state;
// 		u_int8_t manual_state;
// 		u_int8_t lidar_state;
// 		u_int8_t sonar_state;
// 		u_int8_t camera_state;
// 		u_int8_t acc_speed;
// 		u_int8_t ADCU_power_state;
// }ADCU_TO_TBOX_1;

// typedef struct
// {
// 	u_int8_t first_obj_id;
// 	u_int8_t first_obj_type;
// 	u_int32_t first_obj_speed;
// 	u_int32_t first_obj_dis;
// 	u_int8_t second_obj_id;
// 	u_int8_t second_obj_type;
// 	u_int32_t second_obj_speed;
// 	u_int32_t second_obj_dis;
// 	u_int8_t third_obj_id;
// 	u_int8_t third_obj_type;
// 	u_int32_t third_obj_speed;
// 	u_int32_t third_obj_dis;
// 	u_int8_t fourth_obj_id;
// 	u_int8_t fourth_obj_type;
// 	u_int32_t fourth_obj_speed;
// 	u_int32_t fourth_obj_dis;
// 	u_int8_t fifth_obj_id;
// 	u_int8_t fifth_obj_type;
// 	u_int32_t fifth_obj_speed;
// 	u_int32_t fifth_obj_dis;
// 	u_int8_t sixth_obj_id;
// 	u_int8_t sixth_obj_type;
// 	u_int32_t sixth_obj_speed;
// 	u_int32_t sixth_obj_dis;
// 	u_int8_t seventh_obj_id;
// 	u_int8_t seventh_obj_type;
// 	u_int32_t seventh_obj_speed;
// 	u_int32_t seventh_obj_dis;
// 	u_int8_t eighth_obj_id;
// 	u_int8_t eighth_obj_type;
// 	u_int32_t eighth_obj_speed;
// 	u_int32_t eighth_obj_dis;

// }ADCU_TO_TBOX_2;

// typedef struct
// {
// 	u_int32_t lateral_deviation;//4
// 		u_int32_t global_course_angle;//8
// 		u_int32_t curvature;//12
// 		u_int32_t target_speed;//16
// 		u_int32_t target_acc;//20
// 		u_int8_t bev_id;//21
// 		u_int8_t dis_to_target[4];//25
// 		//u_int8_t path_id;//2023.1.11
// 		u_int8_t rev1;//26
// 		u_int8_t request_gear;//27
// 		u_int8_t plan_track_point[4];//31
// 		u_int8_t target_obj_id[4];//35
// 		u_int8_t target_obj_dis[4];//39
// 		u_int8_t target_obj_speed[4];//43
// 		//u_int32_t course_angle_off;
// 		u_int8_t creep_state;//44
// 		u_int8_t lift_state;//45
// 		//u_int32_t path_id;
// 		u_int8_t path_id[4];//2023.4.7 id//49
// 		u_int8_t rev;//50
// }ADCU_TO_TBOX_3;

// typedef struct
// {
// 	u_int32_t Acceleration_from_Vehicle;  //车辆实际加速度  2023.1.11加

// }ADCU_TO_TBOX_4;

// typedef struct
// {
// 	u_int8_t packnum;
// 	u_int8_t packseq;
// 	u_int8_t questnum[8];
// 	u_int8_t questtype;
// 	u_int8_t questoperation;
// 	u_int16_t limitspeed;
// 	u_int8_t start_X[8];
// 	u_int8_t start_Y[8];
// 	u_int8_t start_Z[8];
// 	u_int8_t end_X[8];
// 	u_int8_t end_Y[8];
// 	u_int8_t end_Z[8];
// 	u_int8_t rev;
// 	u_int8_t rev2;
// }TBOX_MOVE; //19ff52de

// typedef struct
// {
// 	u_int8_t packnum;
// 	u_int8_t packseq;
// 	u_int8_t questnum[8];
// 	u_int8_t idlength;
// 	u_int32_t id[13];
// 	u_int8_t revv;
// }TBOX_MOVE_id; //19ff52de

// typedef struct
// {
// 	u_int8_t questnum[8];
// 	u_int8_t questtype;
// 	u_int8_t questoperation;
// 	u_int64_t rev1;
// 	u_int64_t rev2;
// 	u_int32_t rev3;
// 	u_int64_t rev4;
// 	u_int64_t rev5;
// 	u_int64_t rev6;
// 	u_int64_t rev7;
// 	u_int64_t rev8;
// 	u_int8_t rev9;
// }TBOX_DROP; //19ff53de

// typedef struct
// {
// 	u_int8_t questnum[8];
// 	u_int8_t questtype;
// 	u_int8_t slowguide;
// 	u_int16_t joint_point;
// 	u_int8_t joint_method;
// 	u_int8_t rev1;
// 	u_int16_t rev2;
// 	u_int64_t rev3;
// 	u_int64_t rev4;
// 	u_int64_t rev5;
// 	u_int64_t rev6;
// 	u_int64_t rev7;
// 	u_int64_t rev8;
// 	u_int8_t rev9;
// 	u_int64_t rev10;
// }TBOX_JOINT; //19ff54de

// typedef struct
// {
// 	u_int64_t questnum;
// 	u_int8_t questtype;
// 	u_int8_t slowguide;
// 	u_int16_t joint_point;
// 	u_int8_t joint_method;
// 	u_int8_t rev1;
// 	u_int16_t rev2;
// 	u_int64_t rev3;
// 	u_int64_t rev4;
// 	u_int64_t rev5;
// 	u_int64_t rev6;
// 	u_int64_t rev7;
// 	u_int64_t rev8;
// }TBOX_AUTOMODESET; //19ff559e

// typedef struct
// {
// 	u_int8_t questnum[8];
// 	u_int8_t timestamp[6];
// 	u_int8_t questtype;
// 	u_int8_t queststate;
// 	u_int64_t rev3;
// 	u_int64_t rev4;
// 	u_int64_t rev5;
// 	u_int64_t rev6;
// 	u_int64_t rev7;
// 	u_int64_t rev8;
// }TBOX_MOVE_STATE; //19ff569e

// typedef struct
// {
// 	u_int8_t questnum[8];
// 	u_int8_t timestamp[6];
// 	u_int8_t questtype;
// 	u_int8_t queststate;
// 	u_int8_t slow_feedback;
// 	u_int8_t rev1;
// 	u_int16_t rev2;
// 	u_int32_t rev3;
// 	u_int64_t rev4;
// 	u_int64_t rev5;
// 	u_int64_t rev6;
// 	u_int64_t rev7;
// 	u_int64_t rev8;

// }TBOX_DROP_STATE; //19ff579e

// typedef struct
// {
// 	u_int8_t questnum[8];
// 	u_int8_t timestamp[6];
// 	u_int8_t questtype;
// 	u_int8_t queststate;
// 	u_int8_t slow_feedback;
// 	u_int8_t rev1;
// 	u_int16_t rev2;
// 	u_int32_t rev3;
// 	u_int64_t rev4;
// 	u_int64_t rev5;
// 	u_int64_t rev6;
// 	u_int64_t rev7;
// 	u_int64_t rev8;

// }TBOX_JOINT_STATE; //19ff589e

// typedef struct
// {
// 	union
// 		{
// 			u_int8_t first_byte;
// 			struct
// 			{
// 				u_int8_t S_FLRadarDis1_low : 8;
// 			};
// 		};//0
// 	union
// 		{
// 			u_int8_t second_byte;
// 			struct
// 			{
// 				u_int8_t S_FLRadarDis1_high : 1;
// 				u_int8_t not_defined_1 : 1;
// 				u_int8_t S_FLadarSts1 : 2;
// 				u_int8_t S_FLRadarDisAlarmLevel1 : 4;
// 			};
// 		};//1
// 	union
// 		{
// 			u_int8_t third_byte;
// 			struct
// 			{
// 				u_int8_t S_FMLRadarDis2_low : 8;
// 			};
// 		};//2
// 	union
// 		{
// 			u_int8_t fourth_byte;
// 			struct
// 			{
// 				u_int8_t S_FMLRadarDis2_high : 1;
// 				u_int8_t not_defined_2 : 1;
// 				u_int8_t S_FMLadarSts2 : 2;
// 				u_int8_t S_FMLRadarDisAlarmLevel2 : 4;	
// 			};
// 		};//3
// 	union
// 		{
// 			u_int8_t fifth_byte;
// 			struct
// 			{
// 				u_int8_t S_FMRRadarDis3_low : 8;
// 			};
// 		};//4
// 	union
// 		{
// 			u_int8_t sixth_byte;
// 			struct
// 			{
// 				u_int8_t S_FMRRadarDis3_high : 1;
// 				u_int8_t not_defined_3 : 1;
// 				u_int8_t S_FMRadarSts3 : 2;
// 				u_int8_t S_FMRRadarDisAlarmLevel3 : 4;
// 			};
// 		};//5
// 	union
// 		{
// 			u_int8_t seventh_byte;
// 			struct
// 			{
// 				u_int8_t S_FRadarDis4_low : 8;
// 			};
// 		};//6
// 	union
// 		{
// 			u_int8_t eighth_byte;
// 			struct
// 			{
// 				u_int8_t S_FRadarDis4_high : 1;
// 				u_int8_t not_defined_4 : 1;
// 				u_int8_t S_FRRadarSts4 : 2;
// 				u_int8_t S_FRRadarDisAlarmLevel4 : 4;
// 			};
// 		};//7
// }RADAR_Front; //P-CAN

// typedef struct
// {
// 	union
// 		{
// 			u_int8_t first_byte;
// 			struct
// 			{
// 				u_int8_t S_BLRadarDis1_low : 8;
// 			};
// 		};//0
// 	union
// 		{
// 			u_int8_t second_byte;
// 			struct
// 			{
// 				u_int8_t S_BLRadarDis1_high : 1;
// 				u_int8_t not_defined_1 : 1;
// 				u_int8_t S_BLadarSts1 : 2;
// 				u_int8_t S_BLRadarDisAlarmLevel1 : 4;
// 			};
// 		};//1
// 	union
// 		{
// 			u_int8_t third_byte;
// 			struct
// 			{
// 				u_int8_t S_BMLRadarDis2_low : 8;
// 			};
// 		};//2
// 	union
// 		{
// 			u_int8_t fourth_byte;
// 			struct
// 			{
// 				u_int8_t S_BMLRadarDis2_high : 1;
// 				u_int8_t not_defined_2 : 1;
// 				u_int8_t S_BMLadarSts2 : 2;
// 				u_int8_t S_BMLRadarDisAlarmLevel2 : 4;
// 			};
// 		};//3
// 	union
// 		{
// 			u_int8_t fifth_byte;
// 			struct
// 			{
// 				u_int8_t S_BMRRadarDis3_low : 8;
// 			};
// 		};//4
// 	union
// 		{
// 			u_int8_t sixth_byte;
// 			struct
// 			{
// 				u_int8_t S_BMRRadarDis3_high : 1;
// 				u_int8_t not_defined_3 : 1;
// 				u_int8_t S_BMRadarSts3 : 2;
// 				u_int8_t S_BMRRadarDisAlarmLevel3 : 4;
// 			};
// 		};//5
// 	union
// 		{
// 			u_int8_t seventh_byte;
// 			struct
// 			{
// 				u_int8_t S_BRadarDis4_low : 8;
// 			};
// 		};//6
// 	union
// 		{
// 			u_int8_t eighth_byte;
// 			struct
// 			{
// 				u_int8_t S_BRadarDis4_high : 1;
// 				u_int8_t not_defined_4 : 1;
// 				u_int8_t S_BRRadarSts4 : 2;
// 				u_int8_t S_BRRadarDisAlarmLevel4 : 4;
// 			};
// 		};//7
// }RADAR_Back; //P-CAN

// typedef struct
// {
// 	union
// 		{
// 			u_int8_t first_byte;
// 			struct
// 			{
// 				u_int8_t S_SFL_1_RadarDis1_low : 8;
// 			};
// 		};//0
// 	union
// 		{
// 			u_int8_t second_byte;
// 			struct
// 			{
// 				u_int8_t S_SFL_1_RadarDis1_high : 1;
// 				u_int8_t not_defined_1 : 1;
// 				u_int8_t S_SFL_1_RadarSts1 : 2;
// 				u_int8_t S_SFL_1_RadarDisAlarmLevel1 : 4;
// 			};
// 		};//1
// 	union
// 		{
// 			u_int8_t third_byte;
// 			struct
// 			{
// 				u_int8_t S_SFR_1_RadarDis2_low : 8;
// 			};
// 		};//2
// 	union
// 		{
// 			u_int8_t fourth_byte;
// 			struct
// 			{
// 				u_int8_t S_SFR_1_RadarDis2_high : 1;
// 				u_int8_t not_defined_2 : 1;
// 				u_int8_t S_SFR_1_RadarSts2 : 2;
// 				u_int8_t S_SFR_1_RadarDisAlarmLevel2 : 4;
// 			};
// 		};//3
// 	union
// 		{
// 			u_int8_t fifth_byte;
// 			struct
// 			{
// 				u_int8_t S_SBL_1_RadarDis3_low : 8;
// 			};
// 		};//4
// 	union
// 		{
// 			u_int8_t sixth_byte;
// 			struct
// 			{
// 				u_int8_t S_SBL_1_RadarDis3_high : 1;
// 				u_int8_t not_defined_3 : 1;
// 				u_int8_t S_SBL_1_RadarSts3 : 2;
// 				u_int8_t S_SBL_1_RadarDisAlarmLevel3 : 4;
// 			};
// 		};//5
// 	union
// 		{
// 			u_int8_t seventh_byte;
// 			struct
// 			{
// 				u_int8_t S_SBR_1_RadarDis4_low : 8;
// 			};
// 		};//6
// 	union
// 		{
// 			u_int8_t eighth_byte;
// 			struct
// 			{
// 				u_int8_t S_SBR_1_RadarDis4_high : 1;
// 				u_int8_t not_defined_4 : 1;
// 				u_int8_t S_SBR_1_RadarSts4 : 2;
// 				u_int8_t S_SBR_1_RadarDisAlarmLevel4 : 4;
// 			};
// 		};//7
// }RADAR_Side1; //P-CAN

// typedef struct
// {
// 	union
// 		{
// 			u_int8_t first_byte;
// 			struct
// 			{
// 				u_int8_t S_SFL_2_RadarDis1_low : 8;
// 			};
// 		};//0
// 	union
// 		{
// 			u_int8_t second_byte;
// 			struct
// 			{
// 				u_int8_t S_SFL_2_RadarDis1_high : 1;
// 				u_int8_t not_defined_1 : 1;
// 				u_int8_t S_SFL_2_RadarSts1 : 2;
// 				u_int8_t S_SFL_2_RadarDisAlarmLevel1 : 4;
// 			};
// 		};//1
// 	union
// 		{
// 			u_int8_t third_byte;
// 			struct
// 			{
// 				u_int8_t S_SFR_2_RadarDis2_low : 8;
// 			};
// 		};//2
// 	union
// 		{
// 			u_int8_t fourth_byte;
// 			struct
// 			{
// 				u_int8_t S_SFR_2_RadarDis2_high : 1;
// 				u_int8_t not_defined_2 : 1;
// 				u_int8_t S_SFR_2_RadarSts2 : 2;
// 				u_int8_t S_SFR_2_RadarDisAlarmLevel2 : 4;
// 			};
// 		};//3
// 	union
// 		{
// 			u_int8_t fifth_byte;
// 			struct
// 			{
// 				u_int8_t S_SBL_2_RadarDis3_low : 8;
// 			};
// 		};//4
// 	union
// 		{
// 			u_int8_t sixth_byte;
// 			struct
// 			{
// 				u_int8_t S_SBL_2_RadarDis3_high : 1;
// 				u_int8_t not_defined_3 : 1;
// 				u_int8_t S_SBL_2_RadarSts3 : 2;
// 				u_int8_t S_SBL_2_RadarDisAlarmLevel3 : 4;
// 			};
// 		};//5
// 	union
// 		{
// 			u_int8_t seventh_byte;
// 			struct
// 			{
// 				u_int8_t S_SBR_2_RadarDis4_low : 8;
// 			};
// 		};//6
// 	union
// 		{
// 			u_int8_t eighth_byte;
// 			struct
// 			{
// 				u_int8_t S_SBR_2_RadarDis4_high : 1;
// 				u_int8_t not_defined_4 : 1;
// 				u_int8_t S_SBR_2_RadarSts4 : 2;
// 				u_int8_t S_SBR_2_RadarDisAlarmLevel4 : 4;
// 			};
// 		};//7
// }RADAR_Side2; //P-CAN
//-------------------new----------------
// typedef struct
// {
// 	u_int8_t MS1_Mor_Curr_AC_low;//0
// 	u_int8_t MS1_Mor_Curr_AC_high;//1
// 	u_int8_t MS1_Mor_Volta_AC_low;//2
// 	u_int8_t MS1_Mor_Volta_AC_high;//3
// 	u_int8_t MS1_Mor_Spd_Cent_low;//4
// 	u_int8_t MS1_Mor_Spd_Cent_high;//5
// 	u_int8_t MS1_Mor_Trq_Cent_low;//6
// 	u_int8_t MS1_Mor_Trq_Cent_high;//7
// }MS1_MCU; //A-CAN

// typedef struct
// {
// 	union
// 		{
// 			u_int8_t first_byte;
// 			struct
// 			{
// 				u_int8_t MC1_MCU_Enable : 2;
// 				u_int8_t MC1_Mor_Work_Mode : 3;
// 				u_int8_t MC1_Mor_Con_Mode : 2;
// 				u_int8_t MC1_QuickOFF : 1;
// 			};
// 		};//0
// 	u_int8_t rev1;//1
// 	u_int8_t MC1_Mor_Tgt_Spd_low;//2
// 	u_int8_t MC1_Mor_Tgt_Spd_high;//3
// 	u_int8_t MC1_Mor_Tgt_Trq_low;//4
// 	u_int8_t MC1_Mor_Tgt_Trq_high;//5
// 	u_int8_t MC1_CycMsg_Num;//6
// 	u_int8_t MC1_Accped;//7
// }MC1_HCU; //A-CAN


// typedef struct
// {
// 	union
// 		{
// 			u_int8_t first_byte;
// 			struct
// 			{
// 				u_int8_t TwoSpeedAxleSwitch : 2;
// 				u_int8_t ParkingBrakeSwitch : 2;
// 				u_int8_t CruiseCtrlPauseSwitch : 2;
// 				u_int8_t ParkBrakeReleaseInhibitRq : 2;
// 			};
// 		};//0
// 	u_int8_t WheelBasedVehicleSpeed_low;//1
// 	u_int8_t WheelBasedVehicleSpeed_high;//2
// 	union
// 		{
// 			u_int8_t fourth_byte;
// 			struct
// 			{
// 				u_int8_t CruiseCtrlActive : 2;
// 				u_int8_t CruiseCtrlEnableSwitch : 2;
// 				u_int8_t BrakeSwitch : 2;
// 				u_int8_t ClutchSwitch : 2;
// 			};
// 		};//3
// 	union
// 		{
// 			u_int8_t fifth_byte;
// 			struct
// 			{
// 				u_int8_t CruiseCtrlSetSwitch : 2;
// 				u_int8_t CruiseCtrlCoastSwitch : 2;
// 				u_int8_t CruiseCtrlResumeSwitch : 2;
// 				u_int8_t CruiseCtrlAccelerateSwitch : 2;
// 			};
// 		};//4
// 	u_int8_t CruiseCtrlSetSpeed;//5
// 	union
// 		{
// 			u_int8_t seventh_byte;
// 			struct
// 			{
// 				u_int8_t PTOGovernorState : 5;
// 				u_int8_t CruiseCtrlStates : 3;
// 			};
// 		};//6
// 	union
// 		{
// 			u_int8_t eighth_byte;
// 			struct
// 			{
// 				u_int8_t EngIdleIncrementSwitch : 2;
// 				u_int8_t EngIdleDecrementSwitch : 2;
// 				u_int8_t EngTestModeSwitch : 2;
// 				u_int8_t EngShutdownOverrideSwitch : 2;
// 			};
// 		};//7
// }CCVS1_HCU; //A-CAN 0x18FEF100

// typedef struct
// {
// 	u_int8_t Nominal_friction_percent_torque ;//0
// 	u_int8_t Engine_desired_operating_speed_low ;//1,2
// 	u_int8_t Engine_desired_operating_speed_high;//1.2
// 	u_int8_t Engine_operating_speed_asymmetry_adjustment ;//3
// 	u_int8_t Estimated_Engine_Parasitic_Losses_Percent_Torque;//4
// 	u_int8_t Exhaust_gas_mass_flow_low;//5.6
// 	u_int8_t Exhaust_gas_mass_flow_high;//5.6
// }EEC3_E; //0x18FEDF00


// typedef struct
// {
// 	union
// 		{
// 			u_int8_t first_byte;
// 			struct
// 			{
// 				u_int8_t EngOverrideCtrlMode_21 : 2;
// 				u_int8_t EngRqedSpeedCtrlConditions_21 : 2;
// 				u_int8_t OverrideCtrlModePriority_21 : 2;
// 				u_int8_t rev : 2;
// 			};
// 		};//0
// 	u_int16_t EngRqedSpeed_SpeedLimit_21;//1,2
// 	u_int8_t EngRqedTorque_TorqueLimit_21;//3
// 	union
// 		{
// 			u_int8_t fifth_byte;
// 			struct
// 			{
// 				u_int8_t TransmissionRate_21 : 3;
// 				u_int8_t ControlPurpose_21 : 5;
// 			};
// 		};//4
// 	union
// 		{
// 			u_int8_t sixth_byte;
// 			struct
// 			{
// 				u_int8_t Engine_Requested_Torque  : 4;
// 				u_int8_t not_defined : 4;
// 			};
// 		};//5
// 	u_int8_t rev1;//6
// 	union
// 		{
// 			u_int8_t eighth_byte;
// 			struct
// 			{
// 				u_int8_t MessageCounter_21 : 4;
// 				u_int8_t MessageChecksum_21 : 4;
				
// 			};
// 		};//7
// }TSC1_AE; //A-CAN 0x0C00000B
// typedef struct
// {
// 	u_int8_t Powered_Vehicle_Weight_low ;//0
// 	u_int8_t Powered_Vehicle_Weight_high ;//1
// 	u_int8_t Gross_Combination_Weight_low ;//2
// 	u_int8_t Gross_Combination_Weight_high;//3
// 	u_int8_t not_defined_1;//4
// 	u_int8_t not_defined_2;//5
// 	u_int8_t not_defined_3;//6
// 	u_int8_t not_defined_4;//7
// }CVW_EBS; //0x18FE700B

// typedef struct
// {
// 	union
// 		{
// 			u_int8_t first_byte;
// 			struct
// 			{
// 				u_int8_t ParkBtSwitch : 2;
// 				u_int8_t RelsBtSwitch : 2;
// 				u_int8_t AH_BraStatus : 2;
// 				u_int8_t EPB_WorkMode : 2;
// 			};
// 		};//0
// 	union
// 		{
// 			u_int8_t second_byte;
// 			struct
// 			{
// 				u_int8_t Reason_for_inhibition_EPB_release : 4;
// 				u_int8_t AHBtSwitch : 2;
// 				u_int8_t IdepedtBtSwitch  : 2;	
// 			};
// 		};//1
// 	union
// 		{
// 			u_int8_t third_byte;
// 			struct
// 			{
// 				u_int8_t Auto_Hold : 2;
// 				u_int8_t ForcTestBtSwitch : 2;
// 				u_int8_t WarmingStatus : 2;
// 				u_int8_t ResponsePriority : 2;
// 			};
// 		};//2
// 	union
// 		{
// 			u_int8_t fourth_byte;
// 			struct
// 			{
// 				u_int8_t IdepedtBraOly  : 2;
// 				u_int8_t PrakBraForcTest : 2;
// 				u_int8_t SysStatus : 2;
// 				u_int8_t EPB_BraSwitch : 2;
// 			};
// 		};//3	
	
// 	u_int8_t State_of_EPB;//4
// 	u_int8_t PTposition;//5
// 	u_int8_t ParkBraPres;//6
// 	union
// 		{
// 			u_int8_t eighth_byte;
// 			struct
// 			{
// 				u_int8_t not_defined : 4;
// 				u_int8_t EPBADModeStatePS : 4;
// 			};
// 		};//7
// }EPB1; //0x18FF3C50
// typedef struct
// {
// 	union
// 		{
// 			u_int8_t first_byte;
// 			struct
// 			{
// 				u_int8_t MS2_Mor_State_Feed : 4;
// 				u_int8_t MS2_Mor_Work_Mode : 4;
// 			};
// 		};//0
// 	u_int8_t MS2_Mor_MaxTrq_Cent ;//1
// 	u_int8_t MS2_Mor_MinTrq_Cent ;//2
// 	u_int8_t MS2_Mor_MaxSpd_Cent;//3
// 	u_int8_t MS2_MorCavity_Temp ;//4
// 	u_int8_t MS2_MorCtrl_Temp ;//5
// 	union
// 		{
// 			u_int8_t seventh_byte;
// 			struct
// 			{
// 				u_int8_t MS2_MorCtrl_Requ : 2;
// 				u_int8_t not_defined : 2;
// 				u_int8_t MS2_CycMsg_Num : 4;
// 			};
// 		};//6
// 	u_int8_t MS2_CAN_Protocol_Version;//7
// }MS2_MCU; //0x18FF12EF




#endif
