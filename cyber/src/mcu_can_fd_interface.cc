/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: MDC CANFD消息实例化demo样例代码（基于ARXML)，需要基于不同硬件连线和ARXML配置进行修改调测
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
#include "../include/mcu_can_fd_interface.h"

namespace CanFdMsgHandle {
using namespace ara::log;
using namespace ara::core;
using namespace mdc::mdccan;
// 数据缓存区大小
const int BUFFER_DEP = 1000;
const int CANSEND_METHOD_PERIOD = 0;
/*****************************************************************************
 函 数 名  : McuCanFdInterface::Init
 功能描述  : 消息接收实例初始化（当前代码支持初始化一路，多路需要修改、调测）
            (默认channelId = 1，可通过命令行输入)
 输入参数  : uint8_t channelId 0~11
 返 回 值  : 无
 修改历史  : NA
*****************************************************************************/
void McuCanFdInterface::Init(uint8_t channelId)
{
    m_canFdLogger.LogInfo() <<"McuCanFdInterface start init";
    // 入参判断
    if (channelId >= CAN_CHANNEL_NUM) {
        return;
    }
    // 物理通道
    m_channelId = channelId;
    // instance ID, instance比物理通道多1
    m_instance = channelId + 1;
    std::string instanceIdStr = std::to_string(m_instance);
    // 注册服务发现的回调函数，，当发现服务的时候，会回调该函数
    CanFdRxProxy::StartFindService(
        [this](ara::com::ServiceHandleContainer<CanFdRxProxy::HandleType> handles, 
        ara::com::FindServiceHandle handler) {
            McuCanFdInterface::ServiceAvailabilityCallback(std::move(handles), handler);
        },
        ara::com::InstanceIdentifier(StringView(instanceIdStr.c_str())));
    m_canFdLogger.LogInfo() <<"McuCanFdInterface init success";
}

/*****************************************************************************
 函 数 名  : McuCanFdInterface::ServiceAvailabilityCallback
 功能描述  : 基于ARXML配置信息提供Service和实例的消息实时接收函数注册功能(需基于不同的ARXML配置重新调测)
 输入参数  : 服务发现句柄
 返 回 值  : 无
 修改历史  : NA
*****************************************************************************/
void McuCanFdInterface::ServiceAvailabilityCallback(ara::com::ServiceHandleContainer<CanFdRxProxy::HandleType> handles,
    ara::com::FindServiceHandle handler)
{
    if (handles.size() > 0) {
        for (unsigned int i = 0; i < handles.size(); i++) {
            int channelId = m_channelId;
            std::string instanceIdStr = std::to_string(m_instance);
            if (handles[i].GetInstanceId().ToString() != StringView(instanceIdStr.c_str())) {
                continue;
            }
            if (m_proxy[channelId] == nullptr) {
                // 注册接收MCU上传CANFD帧的回调函数
                m_proxy[channelId] = std::make_unique<CanFdRxProxy>(handles[i]);
                // 注册结束，可启动数据下发
                SetSendSign(1);
                m_canFdLogger.LogInfo() << "Created m_proxy from handle with instance: " << instanceIdStr;
                // canFd 短包订阅与回调
                m_proxy[channelId]->CanFdDataSRxEvent.Subscribe(BUFFER_DEP);
                m_proxy[channelId]->CanFdDataSRxEvent.SetReceiveHandler(
                    [this, channelId]() { McuCanFdInterface::CanFdDataSEventCallback(channelId); });

                // canFd 长包订阅与回调
                m_proxy[channelId]->CanFdDataLRxEvent.Subscribe(BUFFER_DEP);
                m_proxy[channelId]->CanFdDataLRxEvent.SetReceiveHandler(
                    [this, channelId]() { McuCanFdInterface::CanFdDataLEventCallback(channelId); });
            }
        }
    }
}
/*****************************************************************************
 函 数 名  : McuCanFdInterface::CanFdDataSEventCallback
 功能描述  : 基于ARXML配置信息提供Service和实例的消息实时接收函数(需基于不同的ARXML配置重新调测)
            CanFd 短包接收回调
 输入参数  : uint8_t channelId
 返 回 值  : 无
 修改历史  : NA
*****************************************************************************/
void McuCanFdInterface::CanFdDataSEventCallback(uint8_t channelId)
{
    // channelId 范围为（0~11)
    if (channelId >= CAN_CHANNEL_NUM) {
        return;
    }

    if (m_proxy[channelId] == nullptr) {
        return;
    }

    // 加锁防止重入
    std::unique_lock<std::mutex> lockread(m_canfdDataSReadMutex);
    // 接收CAN帧
    m_proxy[channelId]->CanFdDataSRxEvent.GetNewSamples([this](ara::com::SamplePtr<CanFdBusDataParamS const> ptr) {
        auto sample = ptr.Get(); // sample为收到数据
        // 接收转入canFd 短包处理回调函数
        CanFdDataSRecievedHandler(m_channelId, *sample);
    });
    // 解锁
    lockread.unlock();
}

/*****************************************************************************
 函 数 名  : McuCanFdInterface::CanFdDataLEventCallback
 功能描述  : 基于ARXML配置信息提供Service和实例的消息实时接收函数(需基于不同的ARXML配置重新调测)
            CanFd 长包接收回调
 输入参数  : uint8_t channelId
 返 回 值  : 无
 修改历史  : NA
*****************************************************************************/
void McuCanFdInterface::CanFdDataLEventCallback(uint8_t channelId)
{
    // channelId 范围为（0~11)
    if (channelId >= CAN_CHANNEL_NUM) {
        return;
    }

    if (m_proxy[channelId] == nullptr) {
        return;
    }

    // 加锁防止重入
    std::unique_lock<std::mutex> lockread(m_canfdDataLReadMutex);
    // 接收CAN帧
    m_proxy[channelId]->CanFdDataLRxEvent.GetNewSamples([this](ara::com::SamplePtr<CanFdBusDataParamL const> ptr) {
        auto sample = ptr.Get(); // sample为收到数据
        // 接收转入canFd 长包处理回调函数
        CanFdDataLRecievedHandler(m_channelId, *sample);
    });
    // 解锁
    lockread.unlock();
}

/*****************************************************************************
 函 数 名  : McuCanFdInterface::CanFdDataMethodSend
 功能描述  : 应用层以method方式下发 canFd 包给MCU
 输入参数  : para1: uint8_t channelId 0~11
 返 回 值  :  无
 修改历史  : NA
*****************************************************************************/
void McuCanFdInterface::CanFdDataMethodSend(uint8_t &channelId) const
{
    // channelId 范围为（0~11)
    if (channelId >= CAN_CHANNEL_NUM) {
        return;
    }

    if (m_proxy[channelId] == nullptr) {
        m_canFdLogger.LogError()<<"CanFdDataMethodSend m_proxy == nullptr";
        return;
    }
    // Method发送
    auto handle = m_proxy[channelId]->CanFdDataSetMethod(m_canFdSendData);
    std::this_thread::sleep_for(std::chrono::milliseconds(CANSEND_METHOD_PERIOD));
}

//8.23新加 
void McuCanFdInterface::CanFdDataMethodSend(uint8_t channelId, const CanFdBusDataParam& test)
{
    // channelId 范围为（0~11)
    if (channelId < 0 || channelId >= CAN_CHANNEL_NUM) {
        return;
    }
    
    if (m_proxy[channelId] == nullptr) {
        m_canFdLogger.LogError()<<"CanFdDataMethodSend m_proxy == nullptr";
        return;
    }
	//printf("---------> %d \n", test.canId);
    // Method发送
    auto handle = m_proxy[channelId]->CanFdDataSetMethod(test);
  //  if (handle.wait_for(std::chrono::milliseconds(CANSEND_METHOD_PERIOD)) != ara::core::FutureStatus::kReady) {
  //      m_canFdLogger.LogError() <<"CanFdDataMethodSend fail";
  //      return;
  //  }
  //  // Method 发送返回值获取方式： auto output = handle.get(); 返回值为output.result OK：0， ERROR：1
    std::this_thread::sleep_for(std::chrono::milliseconds(CANSEND_METHOD_PERIOD));
}

/*****************************************************************************
函 数 名  : McuCanFdInterface::RegisterRxEventCallBackFunc
功能描述  : 提供给应用层的注册接口
                        完成消息处理函数注册 示例如下
        McuCanFdInterface::RegisterRxEventCallBackFunc(
            std::bind(&CanFdSDataRecieved, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CanFdLDataRecieved, std::placeholders::_1, std::placeholders::_2));

 输入参数  : 参见示例
 返 回 值    : 无
 修改历史  : NA
*****************************************************************************/
void McuCanFdInterface::RegisterRxEventCallBackFunc(dataScallbackFunc handlerS, dataLcallbackFunc handlerL)
{
    // 入参判断
    if ((handlerS == nullptr) || (handlerL == nullptr)) {
        return;
    }
    CanFdDataSRecievedHandler = handlerS;
    CanFdDataLRecievedHandler = handlerL;
}
/*****************************************************************************
 函 数 名  : McuCanFdInterface::DataParseFromInput
 功能描述  : 从命令端口获取Can 数据
 输入参数  : para1: std::string inputData(命令行输入数据格式为16进制，canId以#号和canData分开，每个canData以'.'做为分隔符)
 返 回 值  : bool型
 修改历史  : NA
*****************************************************************************/
bool McuCanFdInterface::DataParseFromInput(std::string inputData, uint8_t canSendType)
{
    std::string::size_type posCanId;
    posCanId = inputData.find('#', 0);
    if (posCanId == inputData.npos) {
        m_canFdLogger.LogError() <<"error canData format";
        return false;
    }
    std::string canIdStr = inputData.substr(0, posCanId);
    // 以16进制转换
    uint32_t canId = std::stoi(canIdStr, nullptr, 16);
    bool ret = false;
    // 提取出 canData 字符串内容
    std::string canDataStr = inputData.substr(posCanId + 1, inputData.size() - posCanId);
    // canSendType下发类型  0：CAN标准帧，1：CAN扩展帧，2：CANFD标准帧，3：CANFD扩展帧
    if (canSendType > 1) {
        // 发送类型为canFd
        ret = CanFdDataParse(canDataStr, canId, canSendType);
    } else {
        // 发送类型为can
        ret = CanDataParse(canDataStr, canId, canSendType);
    }
    return ret;
}

/*****************************************************************************
 函 数 名  : McuCanFdInterface::CanFdDataParse
 功能描述  : 从命令端口获取CanFd 数据格式解析
 输入参数  : para1: std::string canDataStr(命令行输入数据格式为16进制，每个canData以'.'做为分隔符)
 返 回 值  : bool型
 修改历史  : NA
*****************************************************************************/
bool McuCanFdInterface::CanFdDataParse(std::string &canDataStr, uint32_t &canId, uint8_t &canSendType)
{
    uint8_t canfdDataMaxLen = 64;
    m_canFdSendData.seq++;
    m_canFdSendData.sendType = 0;
    m_canFdSendData.canIdType = canSendType;
    m_canFdSendData.timeStamp.second = 0xFFFFFFFF;
    m_canFdSendData.timeStamp.nsecond = 0xFFFFFFFF;
    m_canFdSendData.canId = canId;
    // 16进制
    int hexBase = 16;
    int lastSplitpos = 0;
    std::string::size_type nowSplitpos;
    uint32_t dataSize = 0;
    while (1) {
        nowSplitpos = canDataStr.find_first_of('.', lastSplitpos);
        std::string tempStr = canDataStr.substr(lastSplitpos, nowSplitpos - lastSplitpos);
        // 每个can数据，位数限制为2位
        uint8_t canDataMaxSize = 2;
        if (IsCanFdFormatHexNum(tempStr, canDataMaxSize)) {
            // 以16进制转换
            uint8_t tempData = stoi(tempStr, nullptr, hexBase);
            m_canFdSendData.data.push_back(tempData);
            dataSize++;
        } else {
            m_canFdLogger.LogError() <<"wrong canFd format";
            return false;
        }
        if (nowSplitpos == std::string::npos) {
            break;
        }
        if (dataSize >= canfdDataMaxLen) {
            break;
        }
        lastSplitpos = nowSplitpos + 1;
    }
    m_canFdSendData.validLen = dataSize;
    return true;
}

/*****************************************************************************
 函 数 名  : McuCanFdInterface::CanDataParse
 功能描述  : 从命令端口获取Can 数据格式解析，与CanFd解析过程一样，便于区分can和canFd，可合成一个解析函数
 输入参数  : para1: std::string canDataStr(命令行输入数据格式为16进制，每个canData以'.'做为分隔符)
 返 回 值  :  无
 修改历史  : NA
*****************************************************************************/
bool McuCanFdInterface::CanDataParse(std::string &canDataStr, uint32_t &canId, uint8_t &canSendType)
{
    uint8_t canDataMaxLen = 8;
    m_canFdSendData.seq++;
    m_canFdSendData.sendType = 0;
    m_canFdSendData.canIdType = canSendType;
    m_canFdSendData.timeStamp.second = 0xFFFFFFFF;
    m_canFdSendData.timeStamp.nsecond = 0xFFFFFFFF;
    m_canFdSendData.canId = canId;
    // 16进制
    constexpr int hexBase = 16;
    int lastSplitPos = 0;
    std::string::size_type nowSplitpos;
    int32_t datasize = 0;
    while (1) {
        nowSplitpos = canDataStr.find_first_of('.', lastSplitPos);
        std::string tempStr = canDataStr.substr(lastSplitPos, nowSplitpos - lastSplitPos);
        // 每个can数据，位数限制为2位
        uint8_t canDataMaxSize = 2;
        if (IsCanFdFormatHexNum(tempStr, canDataMaxSize)) {
            uint8_t tempData = stoi(tempStr, nullptr, hexBase);
            m_canFdSendData.data.push_back(tempData);
            datasize++;
        } else {
            m_canFdLogger.LogError() <<"wrong can format";
            return false;
        }
        if (nowSplitpos == std::string::npos) {
            break;
        }
        if (datasize >= canDataMaxLen) {
            break;
        }
        lastSplitPos = nowSplitpos + 1;
    }
    m_canFdSendData.validLen = datasize;
    return true;
}

/*****************************************************************************
 函 数 名  : McuCanFdInterface::IsCanFdFormatHexNum
 功能描述  : 判断命令行输入的CanFd数据格式是否正确
 输入参数  : para1: std::string inputData
            para2: uint8_t maxSize
 返 回 值  :  无
 修改历史  : NA
*****************************************************************************/
bool McuCanFdInterface::IsCanFdFormatHexNum(std::string &tempStr, uint8_t &maxSize) const
{
    if ((tempStr.size() > maxSize) || (tempStr.empty())) {
        return false;
    }
    for (uint8_t i = 0; i < tempStr.size(); i++) {
        if (!isxdigit(tempStr[i])) {
            return false;
        }
    }
    return true;
}
}  // namespace CanFdMsgHandle
