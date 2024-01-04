/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: MDC CANFD消息解析demo样例头文件，代码引用ARXML代码自动生成类，需要基于不同硬件连线和ARXML配置进行修改调测
 * Author:
 * Create: 2021-6-5
 */
#ifndef CANFD_EVENT_SWITCH_H
#define CANFD_EVENT_SWITCH_H
#include "mdc/mdccan/eventswitchserviceinterface_common.h"
#include "mdc/mdccan/eventswitchserviceinterface_proxy.h"

#include "mdc/mdccan/impl_type_hostservicerequestparam.h"
#include "mdc/mdccan/impl_type_eventswitchsetdataresult.h"
#include "ara/log/logging.h"

namespace CanFdMsgHandle {
class CanFdEventSwitch {
public:
    using CanFdEventSwitchProxy = mdc::mdccan::proxy::EventSwitchServiceInterfaceProxy;

    // CM初始化
    void Init();
    // 服务发现回调
    void ServiceAvailabilityCallback(ara::com::ServiceHandleContainer<CanFdEventSwitchProxy::HandleType> handles,
                                     ara::com::FindServiceHandle handler);
    // switch 下发
    void EventSwitchSend(uint8_t switchValue, uint8_t channleId);

private:
    // 客户端接收
    std::unique_ptr<CanFdEventSwitchProxy> m_eventSwitchProxy = nullptr;
    ara::log::Logger& m_eventSwitchLogger{ara::log::CreateLogger("CAN", "event switch example",
                                                                 ara::log::LogLevel::kVerbose)};
};
}  // namespace CanFdMsgHandle

#endif
