
#ifndef CAMERA_NODE_H
#define CAMERA_NODE_H

// Include necessary libraries
#include "rclcpp/rclcpp.hpp"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "opencv2/opencv.hpp"
#include "MvCameraControl.h"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
// Define the CameraNode class
/*
    创建一个类节点，名字叫做CameraNode,继承自Node.
*/
class CameraNode : public rclcpp::Node
{

public:
    //打印相机信息
    bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("The Pointer of pstMVDevInfo is NULL!\n");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
            // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
            printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
            printf("UserDefinedName: %s\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
            printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
            printf("Device Number: %d\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }
    //读取rosparam
    int getParam()
    {
        this->declare_parameter("camera_height", 540); /*声明参数*/
        this->get_parameter("camera_height", camera_height); /*获取参数*/
        this->declare_parameter("camera_width", 720); /*声明参数*/
        this->get_parameter("camera_width", camera_width); /*获取参数*/
        this->declare_parameter("camera_framerate", 200); /*声明参数*/
        this->get_parameter("camera_framerate", camera_framerate); /*获取参数*/
        this->declare_parameter("camera_exp", 4000.0); /*声明参数*/
        this->get_parameter("camera_exp", camera_exp); /*获取参数*/
        this->declare_parameter("camera_gain", 12.0); /*声明参数*/
        this->get_parameter("camera_gain", camera_gain); /*获取参数*/
        this->declare_parameter("camera_name", "camera"); /*声明参数*/
        this->get_parameter("camera_name", camera_name); /*获取参数*/
        this->declare_parameter("camera_roi_offset_x", 0); /*声明参数*/
        this->get_parameter("camera_roi_offset_x", camera_roi_offset_x); /*获取参数*/
        this->declare_parameter("camera_roi_offset_y", 0); /*声明参数*/
        this->get_parameter("camera_roi_offset_y", camera_roi_offset_y); /*获取参数*/
        this->declare_parameter("camera_auto_exp", 0); /*声明参数*/
        this->get_parameter("camera_auto_exp", camera_auto_exp); /*获取参数*/
        this->declare_parameter("camera_auto_gain", 0); /*声明参数*/
        this->get_parameter("camera_auto_gain", camera_auto_gain); /*获取参数*/
        this->declare_parameter("camera_auto_whitebalance", 0); /*声明参数*/
        this->get_parameter("camera_auto_whitebalance", camera_auto_whitebalance); /*获取参数*/
        this->declare_parameter("camera_auto_maxexp", 4500); /*声明参数*/
        this->get_parameter("camera_auto_maxexp", camera_auto_maxexp); /*获取参数*/
        this->declare_parameter("camera_auto_minexp", 100); /*声明参数*/
        this->get_parameter("camera_auto_minexp", camera_auto_minexp); /*获取参数*/
        this->declare_parameter("camera_auto_maxgain", 17.0); /*声明参数*/
        this->get_parameter("camera_auto_maxgain", camera_auto_maxgain); /*获取参数*/
        this->declare_parameter("camera_auto_mingain", 0.0); /*声明参数*/
        this->get_parameter("camera_auto_mingain", camera_auto_mingain); /*获取参数*/
    }
    //从相机内部获取参数，如果与要求不一致，则进行修改
    int getAndSetCameraParam()
    {
        int nRet = MV_OK;
        int state = MV_OK;
        // 获取高度信息
        // get IInteger variable
        MVCC_INTVALUE stHeight = {0};
        nRet = MV_CC_GetIntValue(handle, "Height", &stHeight);
        if (MV_OK == nRet)
        {
            if(camera_height > stHeight.nMax)
            {
                camera_height = stHeight.nMax;
            }
            else if(camera_height < stHeight.nMin)
            {
                camera_height = stHeight.nMin;
            }
            printf("camera_height current value:%d, max value:%d, min value:%d, increment value:%d\n", stHeight.nCurValue, stHeight.nMax, stHeight.nMin, stHeight.nInc);
        }
        else
        {
            printf("get height failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置高度
        // set IInteger variable
        if(camera_height != stHeight.nCurValue)
        {
            nRet = MV_CC_SetIntValue(handle, "Height", camera_height);
            if (MV_OK == nRet)
            {
                printf("set height %d\n", camera_height);
            }
            else
            {
                printf("set height failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        // 获取宽度信息
        // get IInteger variable
        MVCC_INTVALUE stWidth = {0};
        nRet = MV_CC_GetIntValue(handle, "Width", &stWidth);
        if (MV_OK == nRet)
        {
            if(camera_width > stWidth.nMax)
            {
                camera_width = stWidth.nMax;
            }
            else if(camera_width < stWidth.nMin)
            {
                camera_width = stWidth.nMin;
            }
            printf("camera_width current value:%d, max value:%d, min value:%d, increment value:%d\n", stWidth.nCurValue, stWidth.nMax, stWidth.nMin, stWidth.nInc);
        }
        else
        {
            printf("get camera_width failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置宽度
        // set IInteger variable
        if(camera_width != stWidth.nCurValue)
        {
            nRet = MV_CC_SetIntValue(handle, "Width", camera_width);
            if (MV_OK == nRet)
            {
                printf("set camera_width %d\n", camera_width);
            }
            else
            {
                printf("set camera_width failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }

        // 获取X偏移信息
        // get IInteger variable
        MVCC_INTVALUE stOffsetX = {0};
        nRet = MV_CC_GetIntValue(handle, "OffsetX", &stOffsetX);
        if (MV_OK == nRet)
        {
            if(camera_roi_offset_x > stOffsetX.nMax)
            {
                camera_roi_offset_x = stOffsetX.nMax;
            }
            else if(camera_roi_offset_x < stOffsetX.nMin)
            {
                camera_roi_offset_x = stOffsetX.nMin;
            }
            printf("camera_OffsetX current value:%d, max value:%d, min value:%d, increment value:%d\n", stOffsetX.nCurValue, stOffsetX.nMax, stOffsetX.nMin, stOffsetX.nInc);
        }
        else
        {
            printf("get camera_OffsetX failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置X偏移
        // set IInteger variable
        if(camera_roi_offset_x != stOffsetX.nCurValue)
        {
            nRet = MV_CC_SetIntValue(handle, "OffsetX", camera_roi_offset_x);
            if (MV_OK == nRet)
            {
                printf("set camera_OffsetX %d\n", camera_roi_offset_x);
            }
            else
            {
                printf("set camera_OffsetX failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        // 获取Y偏移信息
        // get IInteger variable
        MVCC_INTVALUE stOffsetY = {0};
        nRet = MV_CC_GetIntValue(handle, "OffsetY", &stOffsetY);
        if (MV_OK == nRet)
        {
            if(camera_roi_offset_y > stOffsetY.nMax)
            {
                camera_roi_offset_y = stOffsetY.nMax;
            }
            else if(camera_roi_offset_y < stOffsetY.nMin)
            {
                camera_roi_offset_y = stOffsetY.nMin;
            }
            printf("camera_OffsetY current value:%d, max value:%d, min value:%d, increment value:%d\n", stOffsetY.nCurValue, stOffsetY.nMax, stOffsetY.nMin, stOffsetY.nInc);
        }
        else
        {
            printf("get camera_OffsetY failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置Y偏移
        // set IInteger variable
        if(camera_roi_offset_y != stOffsetY.nCurValue)
        {
            nRet = MV_CC_SetIntValue(handle, "OffsetY", camera_roi_offset_y);
            if (MV_OK == nRet)
            {
                printf("set camera_OffsetY %d\n", camera_roi_offset_y);
            }
            else
            {
                printf("set camera_OffsetY failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        // 获取自动曝光信息
        MVCC_ENUMVALUE stExposureAuto = {0};
        nRet = MV_CC_GetEnumValue(handle, "ExposureAuto", &stExposureAuto);
        if (MV_OK == nRet)
        {
            if(camera_auto_exp != 0 && camera_auto_exp != 1 && camera_auto_exp != 2)
            {
                camera_auto_exp = stExposureAuto.nCurValue;
            }
            printf("auto exposure current value:%d\n", stExposureAuto.nCurValue);
        }
        else
        {
            printf("get auto exposure failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置自动曝光
        if(camera_auto_exp != stExposureAuto.nCurValue)
        {
            nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", camera_auto_exp);
            if (MV_OK == nRet)
            {
                printf("set auto exposure %d\n", camera_auto_exp);
            }
            else
            {
                printf("set auto exposure failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        // 获取自动增益信息
        MVCC_ENUMVALUE stGainAuto = {0};
        nRet = MV_CC_GetEnumValue(handle, "GainAuto", &stGainAuto);
        if (MV_OK == nRet)
        {
            if(camera_auto_gain != 0 && camera_auto_gain != 1 && camera_auto_gain != 2)
            {
                camera_auto_gain = stGainAuto.nCurValue;
            }
            printf("auto gain current value:%d\n", stGainAuto.nCurValue);
        }
        else
        {
            printf("get auto gain failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置自动增益
        if(camera_auto_gain != stGainAuto.nCurValue)
        {
            nRet = MV_CC_SetEnumValue(handle, "GainAuto", camera_auto_gain);
            if (MV_OK == nRet)
            {
                printf("set auto gain %d\n", camera_auto_gain);
            }
            else
            {
                printf("set auto gain failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        // 获取自动白平衡信息
        MVCC_ENUMVALUE stBalanceWhiteAuto = {0};
        nRet = MV_CC_GetEnumValue(handle, "BalanceWhiteAuto", &stBalanceWhiteAuto);
        if (MV_OK == nRet)
        {
            if(camera_auto_whitebalance != 0 && camera_auto_whitebalance != 1 && camera_auto_whitebalance != 2)
            {
                camera_auto_whitebalance = stBalanceWhiteAuto.nCurValue;
            }
            printf("auto whitebalance current value:%d\n", stBalanceWhiteAuto.nCurValue);
        }
        else
        {
            printf("get auto whitebalance failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置自动白平衡
        if(camera_auto_whitebalance != stBalanceWhiteAuto.nCurValue)
        {
            nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", camera_auto_whitebalance);
            if (MV_OK == nRet)
            {
                printf("set auto whitebalance %d\n", camera_auto_whitebalance);
            }
            else
            {
                printf("set auto whitebalance failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        // 获取图像格式信息
        MVCC_ENUMVALUE stPixelFormat = {0};
        nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &stPixelFormat);
        if (MV_OK == nRet)
        {
            printf("PixelFormat current value:%x\n", stPixelFormat.nCurValue);
        }
        else
        {
            printf("get PixelFormat failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置图像格式
        if(pixel_format != stPixelFormat.nCurValue)
        {
            nRet = MV_CC_SetEnumValue(handle, "PixelFormat", pixel_format);
            if (MV_OK == nRet)
            {
                printf("set PixelFormat %x\n", pixel_format);
            }
            else
            {
                printf("set PixelFormat failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        // 获取曝光信息
        // get IFloat variable
        MVCC_FLOATVALUE stExposureTime = {0};
        nRet = MV_CC_GetFloatValue(handle, "ExposureTime", &stExposureTime);
        if (MV_OK == nRet)
        {
            if(camera_exp > stExposureTime.fMax)
            {
                camera_exp = stExposureTime.fMax;
            }
            else if(camera_exp < stExposureTime.fMin)
            {
                camera_exp = stExposureTime.fMin;
            }
            printf("exposure time current value:%f, max value:%f, min value:%f\n", stExposureTime.fCurValue, stExposureTime.fMax, stExposureTime.fMin);
        }
        else
        {
            printf("get exposure time failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置曝光
        // set IFloat variable
        if(abs(camera_exp - stExposureTime.fCurValue) > 0.00001)
        {
            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", camera_exp);
            if (MV_OK == nRet)
            {
                printf("set exposure time %f\n", camera_exp);
            }
            else
            {
                printf("set exposure time failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        // 获取自动最小曝光信息
        // get IFloat variable
        MVCC_INTVALUE stAutoExposureTimeLowerLimit = {0};
        nRet = MV_CC_GetIntValue(handle, "AutoExposureTimeLowerLimit", &stAutoExposureTimeLowerLimit);
        if (MV_OK == nRet)
        {
            if(camera_auto_minexp > stAutoExposureTimeLowerLimit.nMax)
            {
                camera_auto_minexp = stAutoExposureTimeLowerLimit.nMax;
            }
            else if(camera_auto_minexp < stAutoExposureTimeLowerLimit.nMin)
            {
                camera_auto_minexp = stAutoExposureTimeLowerLimit.nMin;
            }
            printf("camera_auto_minexp current value:%d, max value:%d, min value:%d\n", stAutoExposureTimeLowerLimit.nCurValue, stAutoExposureTimeLowerLimit.nMax, stAutoExposureTimeLowerLimit.nMin);
        }
        else
        {
            printf("get camera_auto_minexp failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置自动最小曝光
        // set IFloat variable
        if(camera_auto_minexp != stAutoExposureTimeLowerLimit.nCurValue)
        {
            nRet = MV_CC_SetIntValue(handle, "AutoExposureTimeLowerLimit", camera_auto_minexp);
            if (MV_OK == nRet)
            {
                printf("set camera_auto_minexp %d\n", camera_auto_minexp);
            }
            else
            {
                printf("set camera_auto_minexp failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        // 获取自动最大曝光信息
        // get IFloat variable
        MVCC_INTVALUE stAutoExposureTimeUpperLimit = {0};
        nRet = MV_CC_GetIntValue(handle, "AutoExposureTimeUpperLimit", &stAutoExposureTimeUpperLimit);
        if (MV_OK == nRet)
        {
            if(camera_auto_maxexp > stAutoExposureTimeUpperLimit.nMax)
            {
                camera_auto_maxexp = stAutoExposureTimeUpperLimit.nMax;
            }
            else if(camera_auto_maxexp < stAutoExposureTimeUpperLimit.nMin)
            {
                camera_auto_maxexp = stAutoExposureTimeUpperLimit.nMin;
            }
            printf("camera_auto_maxexp current value:%d, max value:%d, min value:%d\n", stAutoExposureTimeUpperLimit.nCurValue, stAutoExposureTimeUpperLimit.nMax, stAutoExposureTimeUpperLimit.nMin);
        }
        else
        {
            printf("get camera_auto_maxexp failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置自动最大曝光
        // set IFloat variable
        if(camera_auto_maxexp != stAutoExposureTimeUpperLimit.nCurValue)
        {
            nRet = MV_CC_SetIntValue(handle, "AutoExposureTimeUpperLimit", camera_auto_maxexp);
            if (MV_OK == nRet)
            {
                printf("set camera_auto_maxexp %d\n", camera_auto_maxexp);
            }
            else
            {
                printf("set camera_auto_maxexp failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        // 获取增益信息
        // get IFloat variable
        MVCC_FLOATVALUE stGain = {0};
        nRet = MV_CC_GetFloatValue(handle, "Gain", &stGain);
        if (MV_OK == nRet)
        {
            if(camera_gain > stGain.fMax)
            {
                camera_gain = stGain.fMax;
            }
            else if(camera_gain < stGain.fMin)
            {
                camera_gain = stGain.fMin;
            }
            printf("camera_gain current value:%f, max value:%f, min value:%f\n", stGain.fCurValue, stGain.fMax, stGain.fMin);
        }
        else
        {
            printf("get camera_gain failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置增益
        // set IFloat variable
        if(abs(camera_gain - stGain.fCurValue) > 0.01)
        {
            nRet = MV_CC_SetFloatValue(handle, "Gain", camera_gain);
            if (MV_OK == nRet)
            {
                printf("set camera_gain %f\n", camera_gain);
            }
            else
            {
                printf("set camera_gain failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        // 获取自动最大增益信息
        // get IFloat variable
        MVCC_FLOATVALUE stAutoGainUpperLimit = {0};
        nRet = MV_CC_GetFloatValue(handle, "AutoGainUpperLimit", &stAutoGainUpperLimit);
        if (MV_OK == nRet)
        {
            if(camera_auto_maxgain > stAutoGainUpperLimit.fMax)
            {
                camera_auto_maxgain = stAutoGainUpperLimit.fMax;
            }
            else if(camera_auto_maxgain < stAutoGainUpperLimit.fMin)
            {
                camera_auto_maxgain = stAutoGainUpperLimit.fMin;
            }
            printf("camera_auto_maxgain current value:%f, max value:%f, min value:%f\n", stAutoGainUpperLimit.fCurValue, stAutoGainUpperLimit.fMax, stAutoGainUpperLimit.fMin);
        }
        else
        {
            printf("get camera_auto_maxgain failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置自动最大增益
        // set IFloat variable
        if(abs(camera_auto_maxgain - stAutoGainUpperLimit.fCurValue) > 0.01)
        {
            nRet = MV_CC_SetFloatValue(handle, "AutoGainUpperLimit", camera_auto_maxgain);
            if (MV_OK == nRet)
            {
                printf("set camera_auto_maxgain %f\n", camera_auto_maxgain);
            }
            else
            {
                printf("set camera_auto_maxgain failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        // 获取自动最小增益信息
        // get IFloat variable
        MVCC_FLOATVALUE stAutoGainLowerLimit = {0};
        nRet = MV_CC_GetFloatValue(handle, "AutoGainLowerLimit", &stAutoGainLowerLimit);
        if (MV_OK == nRet)
        {
            if(camera_auto_mingain > stAutoGainLowerLimit.fMax)
            {
                camera_auto_mingain = stAutoGainLowerLimit.fMax;
            }
            else if(camera_auto_mingain < stAutoGainLowerLimit.fMin)
            {
                camera_auto_mingain = stAutoGainLowerLimit.fMin;
            }
            printf("camera_auto_mingain current value:%f, max value:%f, min value:%f\n", stAutoGainLowerLimit.fCurValue, stAutoGainLowerLimit.fMax, stAutoGainLowerLimit.fMin);
        }
        else
        {
            printf("get camera_auto_mingain failed! nRet [%x]\n", nRet);
            state = -1;
        }
        // 设置自动最小增益
        // set IFloat variable
        if(abs(camera_auto_mingain - stAutoGainLowerLimit.fCurValue) > 0.01)
        {
            nRet = MV_CC_SetFloatValue(handle, "AutoGainLowerLimit", camera_auto_mingain);
            if (MV_OK == nRet)
            {
                printf("set camera_auto_mingain %f\n", camera_auto_mingain);
            }
            else
            {
                printf("set camera_auto_mingain failed! nRet [%x]\n", nRet);
                state = -1;
            }
        }
        return state;
    }
    //枚举相机
    int enum_devices()
    {
        int nRet = MV_OK;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            return nRet;
        }
        //打印相机信息
        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    continue;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Find No Devices!\n");
            return MV_E_NODATA;
        }
        return nRet;
    }
    //选择相机，创建句柄
    int select_device()
    {
        int nRet = MV_OK;
        //默认选择第一个枚举到的相机
        //如果发现多个相机，根据自定义名称选择相机连接
        if (stDeviceList.nDeviceNum > 1)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    continue;
                }
                if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE)
                {
                    std::string target_camera_name((char *) pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
                    if (target_camera_name == camera_name)
                    {
                        nIndex = i;
                        break;
                    }
                }
                else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
                {
                    std::string target_camera_name((char *) pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
                    if (target_camera_name == camera_name)
                    {
                        nIndex = i;
                        break;
                    }
                }
            }
        }
        // ch:选择设备并创建句柄 | en:Select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        return nRet;
    }
    //打开相机
    int open_device()
    {
        int nRet = MV_OK;
        // ch:打开设备 | en:Open device
        nRet = MV_CC_OpenDevice(handle);
        return nRet;
    }
    //探测网络最佳包大小(只对GigE相机有效)
    int detection_network_optimal_package_size()
    {
        int nRet = MV_OK;
        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK) {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
            }
        }
        return nRet;
    }
    //设置触发模式
    int set_trigger_mode()
    {
        int nRet = MV_OK;
        // 设置触发模式为on
        // set trigger mode as on
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        return nRet;
    }
    //设置触发源
    int set_trigger_source()
    {
        int nRet = MV_OK;
        // 设置触发源
        // set trigger source
        nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
        return nRet;
    }
    //获取数据包大小
    int get_package_size()
    {
        int nRet = MV_OK;
        // ch:获取数据包大小 | en:Get payload size
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        return nRet;
    }
    //开始取流
    int start_grabbing()
    {
        int nRet = MV_OK;
        // ch:开始取流 | en:Start grab image
        nRet = MV_CC_StartGrabbing(handle);
        return nRet;
    }
    //停止取流
    int stop_grabbing()
    {
        int nRet = MV_OK;
        // ch:停止取流 | en:Stop grab image
        nRet = MV_CC_StopGrabbing(handle);
        return nRet;
    }
    //关闭设备
    int close_device()
    {
        int nRet = MV_OK;
        // ch:关闭设备 | en:Close device
        nRet = MV_CC_CloseDevice(handle);
        return nRet;
    }
    //销毁句柄
    int destroy_handle()
    {
        int nRet = MV_OK;
        // ch:销毁句柄 | en:Destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        return nRet;
    }
    //获取帧率
    int get_framerate()
    {
        return camera_framerate;
    }
    //获取图像
    int get_image()
    {
        int nRet = MV_OK;
        nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
        return nRet;
    }
    //格式转换
    int convert_pixel_format()
    {
        int nRet = MV_OK;
        pDataForBGR = (unsigned char*)malloc(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 4 + 2048);
        if (NULL == pDataForBGR)
        {
            return MV_E_NODATA;
        }
        // 像素格式转换
        // convert pixel format
        MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam = {0};
        // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
        // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
        // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format,
        // destination pixel format, output data buffer, provided output buffer size
        stConvertParam.nWidth = stOutFrame.stFrameInfo.nWidth;
        stConvertParam.nHeight = stOutFrame.stFrameInfo.nHeight;
        stConvertParam.pSrcData = stOutFrame.pBufAddr;
        stConvertParam.nSrcDataLen = stOutFrame.stFrameInfo.nFrameLen;
        stConvertParam.enSrcPixelType = stOutFrame.stFrameInfo.enPixelType;
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        stConvertParam.pDstBuffer = pDataForBGR;
        stConvertParam.nDstBufferSize = stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight *  4 + 2048;
        nRet = MV_CC_ConvertPixelTypeEx(handle, &stConvertParam);
        return nRet;
    }
    //构造并发布消息
    int publish_message()
    {
        int nRet = MV_OK;
        //构造消息
        uint64_t camera_time_int = stOutFrame.stFrameInfo.nDevTimeStampHigh; //获取相机内部时间高32位
        camera_time_int <<= 32;
        camera_time_int |= stOutFrame.stFrameInfo.nDevTimeStampLow; //获取相机内部时间低32位
        rclcpp::Time camera_time_ros(camera_time_int * 10); //构造ros::Time,乘十是因为相机内部时间为10e-8s
        image_header.stamp = camera_time_ros; //时间戳的零点为相机上电时间
        image_header.stamp = camera_time_ros;
        image_header.frame_id = "hik_camera";
        // cv::Mat image(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC3);
        // //将char*数组转换为Mat
        // for(int j = 0; j < stOutFrame.stFrameInfo.nHeight; j++)
        // {
        //     unsigned char* data = image.ptr<unsigned char>(j);
        //     unsigned char* pSubDataForRGB = stOutFrame.pBufAddr + (j + 1) * stOutFrame.stFrameInfo.nWidth * 3;
        //     memcpy(data, pSubDataForRGB, stOutFrame.stFrameInfo.nWidth * 3);
        // }
        //使用cv_bridge将Mat转为sensor_msgs::msg::Image类型
        // cv_bridge::CvImage cv_image;
        // cv_image.encoding = "bgr8";
        // cv_image.header = image_header;
        // cv_image.image = image;
        image_msg.height = stOutFrame.stFrameInfo.nHeight;
        image_msg.width = stOutFrame.stFrameInfo.nWidth;
        image_msg.encoding = "bgr8";
        image_msg.header = image_header;
        image_msg.step = stOutFrame.stFrameInfo.nWidth * 3;
        image_msg.data = std::vector<unsigned char>(stOutFrame.pBufAddr, stOutFrame.pBufAddr + stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3);
        //cv_image.toImageMsg(image_msg);
        image_pub->publish(image_msg);
        //cv::imshow("rgb", image);
        //cv::waitKey(1);
        // if (pDataForBGR)
        // {
        //     free(pDataForBGR);
        //     pDataForBGR = NULL;
        // }
        return nRet;
    }
    //释放图像缓存
    int free_image_buffer()
    {
        int nRet = MV_OK;
        if(NULL != stOutFrame.pBufAddr)
        {
            nRet = MV_CC_FreeImageBuffer(handle, &stOutFrame);
        }
        return nRet;
    }
    //参数监控回调
    
    // 构造函数,有一个参数为节点名称
    CameraNode(std::string name) : Node(name)
    {
        // 打印一句
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.",name.c_str());
        image_pub = this->create_publisher<sensor_msgs::msg::Image>("hik_camera/rgb", 10);
        memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
        height_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        width_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        framerate_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        exp_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        gain_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        roi_offset_x_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        roi_offset_y_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto_exp_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto_gain_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto_whitebalance_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto_maxexp_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto_minexp_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto_maxgain_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto_mingain_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto height_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            {
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to height \"%s\" of type %s: \"%ld\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_int());
                camera_height = p.as_int();
                param_change = true;
            }
        };
        auto width_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            {
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to width \"%s\" of type %s: \"%ld\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_int());
                camera_width = p.as_int();
                param_change = true;
            }
        };
        auto framerate_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            {
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to framerate \"%s\" of type %s: \"%ld\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_int());
                camera_framerate = p.as_int();
                param_change = true;
            }
        };
        auto exp_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            {
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to exp \"%s\" of type %s: \"%f\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_double());
                camera_exp = p.as_double();
                param_change = true;
            }
        };
        auto gain_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            {
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to gain \"%s\" of type %s: \"%f\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_double());
                camera_gain = p.as_double();
                param_change = true;
            }
        };
        auto roi_offset_x_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            {
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to roi_offset_x \"%s\" of type %s: \"%ld\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_int());
                camera_roi_offset_x = p.as_int();
                param_change = true;
            }
        };
        auto roi_offset_y_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            {
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to roi_offset_y \"%s\" of type %s: \"%ld\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_int());
                camera_roi_offset_y = p.as_int();
                param_change = true;
            }
        };
        auto auto_exp_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            {
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to auto_exp \"%s\" of type %s: \"%ld\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_int());
                camera_auto_exp = p.as_int();
                param_change = true;
            }
        };
        auto auto_gain_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            {
                first_done_flag = false; 
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to auto_gain \"%s\" of type %s: \"%ld\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_int());
                camera_auto_gain = p.as_int();
                param_change = true;
            }
        };
        auto auto_whitebalance_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            { 
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to auto_whitebalance \"%s\" of type %s: \"%ld\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_int());
                camera_auto_whitebalance = p.as_int();
                param_change = true;
            }
        };
        auto auto_maxexp_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            { 
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to auto_maxexp \"%s\" of type %s: \"%f\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_double());
                camera_auto_maxexp = p.as_double();
                param_change = true;
            }
        };
        auto auto_minexp_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            { 
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to auto_minexp \"%s\" of type %s: \"%f\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_double());
                camera_auto_minexp = p.as_double();
                param_change = true;
            }
        };
        auto auto_maxgain_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            { 
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to auto_maxgain \"%s\" of type %s: \"%f\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_double());
                camera_auto_maxgain = p.as_double();
                param_change = true;
            }
        };
        auto auto_mingain_callback = [this](const rclcpp::Parameter & p) 
        {
            static bool first_done_flag = true;
            if(first_done_flag)
            { 
                first_done_flag = false;
            }
            else
            {
                RCLCPP_INFO(
                this->get_logger(), "Received an update to auto_mingain \"%s\" of type %s: \"%f\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_double());
                camera_auto_mingain = p.as_double();
                param_change = true;
            }
        };
        height_cb_handle = height_subscriber->add_parameter_callback("camera_height", height_callback);
        width_cb_handle = width_subscriber->add_parameter_callback("camera_width", width_callback);
        framerate_cb_handle = framerate_subscriber->add_parameter_callback("camera_framerate", framerate_callback);
        exp_cb_handle = exp_subscriber->add_parameter_callback("camera_exp", exp_callback);
        gain_cb_handle = gain_subscriber->add_parameter_callback("camera_gain", gain_callback);
        roi_offset_x_cb_handle = roi_offset_x_subscriber->add_parameter_callback("camera_roi_offset_x", roi_offset_x_callback);
        roi_offset_y_cb_handle = roi_offset_y_subscriber->add_parameter_callback("camera_roi_offset_y", roi_offset_y_callback);
        auto_exp_cb_handle = auto_exp_subscriber->add_parameter_callback("camera_auto_exp", auto_exp_callback);
        auto_gain_cb_handle = auto_gain_subscriber->add_parameter_callback("camera_auto_gain", auto_gain_callback);
        auto_whitebalance_cb_handle = auto_whitebalance_subscriber->add_parameter_callback("camera_auto_whitebalance", auto_whitebalance_callback);
        auto_maxexp_cb_handle = auto_maxexp_subscriber->add_parameter_callback("camera_auto_maxexp", auto_maxexp_callback);
        auto_minexp_cb_handle = auto_minexp_subscriber->add_parameter_callback("camera_auto_minexp", auto_minexp_callback);
        auto_maxgain_cb_handle = auto_maxgain_subscriber->add_parameter_callback("camera_auto_maxgain", auto_maxgain_callback);
        auto_mingain_cb_handle = auto_mingain_subscriber->add_parameter_callback("camera_auto_mingain", auto_mingain_callback);
        getParam();
    }
    int restart_flag = 10;
    bool param_change = false;
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    std_msgs::msg::Header image_header;
    sensor_msgs::msg::Image image_msg;
    int camera_height = 1080;
    int camera_width = 1440;
    int camera_framerate = 200;
    int camera_roi_offset_x = 0;
    int camera_roi_offset_y = 0;
    int camera_auto_exp = 0;
    int camera_auto_gain = 0;
    int camera_auto_whitebalance = 0;
    int camera_auto_maxexp = 4500;
    int pixel_format = 0x02180014;
    int camera_auto_minexp = 100;
    float camera_auto_maxgain = 17;
    float camera_auto_mingain = 0;
    float camera_exp = 4000;
    float camera_gain = 5;
    std::string camera_name="camera";
    unsigned char *pDataForBGR = NULL;
    unsigned int nIndex = 0;
    void* handle = NULL;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    MVCC_INTVALUE stParam;
    MV_FRAME_OUT stOutFrame = {0};
    std::shared_ptr<rclcpp::ParameterEventHandler> height_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> height_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> width_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> width_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> framerate_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> framerate_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> exp_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> exp_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> gain_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> gain_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> roi_offset_x_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> roi_offset_x_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> roi_offset_y_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> roi_offset_y_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> auto_exp_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> auto_exp_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> auto_gain_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> auto_gain_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> auto_whitebalance_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> auto_whitebalance_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> auto_maxexp_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> auto_maxexp_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> auto_minexp_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> auto_minexp_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> auto_maxgain_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> auto_maxgain_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> auto_mingain_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> auto_mingain_cb_handle;
};

#endif // CAMERA_NODE_H
