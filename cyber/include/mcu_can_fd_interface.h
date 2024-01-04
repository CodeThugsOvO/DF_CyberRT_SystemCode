/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: MDC CANFD消息解析demo样例头文件，代码引用ARXML代码自动生成类，需要基于不同硬件连线和ARXML配置进行修改调测
 * Author:
 * Create: 2021-6-5
 */
#ifndef CAN_FD_H
#define CAN_FD_H

#include "mdc/mdccanrx/canrxserviceinterface_common.h"
#include "mdc/mdccanrx/canrxserviceinterface_proxy.h"

#include "mdc/mdccan/impl_type_canfdsetdataresult.h"
#include "mdc/mdccan/impl_type_canfdbusdataparaml.h"
#include "mdc/mdccan/impl_type_canfdbusdataparams.h"
#include "ara/log/logging.h"
#include <string>

const unsigned int CAN_CHANNEL_NUM = 12;
const int SLEEP_TIME = 10000;

namespace CanFdMsgHandle {
class McuCanFdInterface {
public:
    using CanFdRxProxy = mdc::mdccanrx::proxy::CanRxServiceInterfaceProxy;

    // CM初始化
    void Init(uint8_t channelId);
    // 服务发现回调
    void ServiceAvailabilityCallback(ara::com::ServiceHandleContainer<CanFdRxProxy::HandleType> handles,
                                     ara::com::FindServiceHandle handler);
    // canFd 短包接收数据回调
    void CanFdDataSEventCallback(uint8_t channelId);
    // canFd 长包接收数据回调
    void CanFdDataLEventCallback(uint8_t channelId);
    // CANFD短包设置接收回调接口
    using dataScallbackFunc = std::function<void (int, const mdc::mdccan::CanFdBusDataParamS &)>;
    // CANFD长包设置接收回调接口
    using dataLcallbackFunc = std::function<void (int, const mdc::mdccan::CanFdBusDataParamL &)>;

    // 注册回调函数
    void RegisterRxEventCallBackFunc(dataScallbackFunc handlerS, dataLcallbackFunc handlerL);
    // 命令端CanFd 数据解析
    bool DataParseFromInput(std::string inputData, uint8_t canSendType);
    // canFd 格式的canData内容解析
    bool CanFdDataParse(std::string &canDataStr, uint32_t &canId, uint8_t &canSendType);
    // can 格式的canData内容解析
    bool CanDataParse(std::string &canDataStr, uint32_t &canId, uint8_t &canSendType);

    // 判断命令行输入的CanFd数据格式是否正确
    bool IsCanFdFormatHexNum(std::string &tempStr, uint8_t &maxSize) const;
    // canfd method下发函数
    void CanFdDataMethodSend(uint8_t &channelId) const;

    void CanFdDataMethodSend(uint8_t channelId,  const mdc::mdccan::CanFdBusDataParam& test);

    // 配置数据信号
    void SetSendSign(const uint8_t &startSendSign)
    {
        m_startSendSign = startSendSign;
    }
    uint8_t GetSendSign()
    {
        return m_startSendSign;
    }
private:
    // canbus_config.json中的ChannelId 范围（0~11）
    int m_channelId = 1;
    // instance ID
    int m_instance = 0;
    // canFd 短包读取锁
    std::mutex m_canfdDataSReadMutex;
    // canFd 长包读取锁
    std::mutex m_canfdDataLReadMutex;

    // 客户端接收
    std::unique_ptr<CanFdRxProxy> m_proxy[CAN_CHANNEL_NUM];
    // canFd 短包接收回调句柄
    dataScallbackFunc CanFdDataSRecievedHandler = nullptr;
    // canFd 长包接收回调句柄
    dataLcallbackFunc CanFdDataLRecievedHandler = nullptr;
    // canfd 下发数据包
    mdc::mdccan::CanFdBusDataParam m_canFdSendData;
    // this class own logger
    ara::log::Logger& m_canFdLogger{ara::log::CreateLogger("CAN", "can data communication example",
                                    ara::log::LogLevel::kVerbose)};
    // 下发数据信号
    uint8_t m_startSendSign = 0;
};
}  // namespace CanFdMsgHandle
#endif
