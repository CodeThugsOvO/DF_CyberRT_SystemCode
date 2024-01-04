/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: DC canFd event switch下发demo样例代码（基于ARXML)，需要基于ARXML配置进行修改调测
 * Author:
 * Create: 2021-6-5
 */
#include <cstdint>
#include <iomanip>
#include <cstdlib>
#include <exception>
#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include "cyber/include/can_fd_event_switch.h"

namespace CanFdMsgHandle {
using namespace ara::log;
using namespace ara::core;
using namespace mdc::mdccan;
const int CANSEND_METHOD_PERIOD = 400;
const int DATA_CENTER_SERVICE_ID = 0x0300;
const int SERVICE_ACTION = 0x0100;
const unsigned int INSTANCE_ID = 1U;
/*****************************************************************************
 函 数 名  : CanFdEventSwitch::Init
 功能描述  : CanFdEventSwitch下发实例初始化
 输入参数  : 无
 返 回 值  : 无
 修改历史  : NA
*****************************************************************************/
void CanFdEventSwitch::Init()
{
    m_eventSwitchLogger.LogInfo() << "CanFdEventSwitch start init";
    std::string instanceIdStr = std::to_string(INSTANCE_ID);
    // 注册服务发现的回调函数，当发现服务的时候，会回调该函数
    CanFdEventSwitchProxy::StartFindService(
        [this](ara::com::ServiceHandleContainer<CanFdEventSwitchProxy::HandleType> handles, 
        ara::com::FindServiceHandle handler) {
            CanFdEventSwitch::ServiceAvailabilityCallback(std::move(handles), handler);
        },
        ara::com::InstanceIdentifier(StringView(instanceIdStr.c_str())));
    m_eventSwitchLogger.LogInfo() << "CanFdEventSwitch init success";
}

/*****************************************************************************
 函 数 名  : CanFdEventSwitch::ServiceAvailabilityCallback
 功能描述  : 基于ARXML配置信息提供Service和实例的消息实时接收函数注册功能(需基于不同的ARXML配置重新调测)
 输入参数  : 服务发现句柄
 返 回 值  : 无
 修改历史  : NA
*****************************************************************************/
void CanFdEventSwitch::ServiceAvailabilityCallback(
    ara::com::ServiceHandleContainer<CanFdEventSwitchProxy::HandleType> handles, ara::com::FindServiceHandle handler)
{
    if (handles.size() > 0) {
        for (unsigned int i = 0; i < handles.size(); i++) {
            std::string instanceIdStr = std::to_string(INSTANCE_ID);
            if (handles[i].GetInstanceId().ToString() != StringView(instanceIdStr.c_str())) {
                continue;
            }
            if (m_eventSwitchProxy == nullptr) {
                // 注册接收controlData回调函数
                m_eventSwitchProxy = std::make_unique<CanFdEventSwitchProxy>(handles[i]);
                m_eventSwitchLogger.LogInfo() << "Created m_eventSwitchProxy from handle with instance: " <<
                    handles[i].GetInstanceId().ToString();
            }
        }
    }
}
/*****************************************************************************
 函 数 名  : CanFdEventSwitch::EventSwitchSend
 功能描述  : eventSwitch下发 举例如下：
            // 关闭CanChannle Event开关的举例配置
            // ServiceId：3（DataCenterServiceId），小端，应配置成0x0300
            // ServiceAction：1（CycleSentMessageSwitch），小端，应配置成0x0100
            // ChannelID：1（Channel 1），应配置成0x01
            // SwitchSts：0（OFF），应配置成0x00
 输入参数  : 无
 返 回 值  : 无
 修改历史  : NA
*****************************************************************************/
void CanFdEventSwitch::EventSwitchSend(uint8_t switchValue, uint8_t channleId)
{
    if (m_eventSwitchProxy == nullptr) {
        m_eventSwitchLogger.LogError()<<"m_eventSwitchProxy is nullptr";
        return;
    }
    HostServiceRequestParam eventSwitchSendData;
    eventSwitchSendData.serviceId = DATA_CENTER_SERVICE_ID;
    eventSwitchSendData.serviceAction = SERVICE_ACTION;
    eventSwitchSendData.channelId = channleId;
    eventSwitchSendData.switchSts = switchValue;
    // Method发送
    auto handle = m_eventSwitchProxy->EventSwitchDataSetMethod(eventSwitchSendData);
    auto sendResult = handle.GetResult();
    // 判断返回的是正常值还是错误码并做校验和打印
    if (sendResult.HasValue()) {
        // 获取值
        auto value = sendResult.Value();
        m_eventSwitchLogger.LogInfo() << "success send the value, reply value is: " <<
            static_cast<int>(value.serviceResult);
    } else {
        // 获取错误码
        auto error = sendResult.Error();
        m_eventSwitchLogger.LogError() << "method send error, the error is: " << std::string(error.Message().data()) <<
            "domain is: " << error.Domain().Id() << " error code is " << error.Value();
    }
    // Method 发送返回值 auto output = handle.get();
    // output.serviceResult 为返回结果 OK：0， ERROR：1
    // output.reserve 为保留字段
    std::this_thread::sleep_for(std::chrono::milliseconds(CANSEND_METHOD_PERIOD));
}
}  // namespace CanFdMsgHandle
