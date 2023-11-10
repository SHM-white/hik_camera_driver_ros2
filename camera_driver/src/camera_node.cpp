
#include "rclcpp/rclcpp.hpp"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "opencv2/opencv.hpp"
#include "MvCameraControl.h"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"



/*
    创建一个类节点，名字叫做CameraNode,继承自Node.
*/
class CameraNode : public rclcpp::Node
{

public:
    /*!
     * 打印相机信息
     * @param pstMVDevInfo
     * @return
     */
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
            printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
            printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
            printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }
    /*!
     * 从相机内部获取参数，如果与要求不一致，则进行修改
     * @param pHandle 相机句柄
     * @param width 宽度
     * @param height 高度
     * @param framerate 帧率
     * @param exp 曝光时间，单位us
     * @param gain 增益，单位db
     * @return 修改状态
     */
    int getAndSetCameraParam()
    {
        int nRet = MV_OK;
        int state = MV_OK;
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
            if(camera_height != stHeight.nMax && camera_height != stHeight.nMin && (camera_height % 16))
            {
                camera_height -= (camera_height % 16);
            }
            printf("height current value:%d\n", stHeight.nCurValue);
            printf("height max value:%d\n", stHeight.nMax);
            printf("height min value:%d\n", stHeight.nMin);
            printf("height increment value:%d\n\n", stHeight.nInc);
        }
        else
        {
            printf("get height failed! nRet [%x]\n\n", nRet);
            state = -1;
        }
        // 设置高度
        // set IInteger variable
        // 宽高设置时需考虑步进(16)，即设置宽高需16的倍数
        // Step (16) should be considered when setting camera_width and height, that is the camera_width and height should be a multiple of 16
        if(camera_height != stHeight.nCurValue)
        {
            nRet = MV_CC_SetIntValue(handle, "Height", camera_height);
            if (MV_OK == nRet)
            {
                printf("set height %d\n\n", camera_height);
            }
            else
            {
                printf("set height failed! nRet [%x]\n\n", nRet);
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
            if(camera_width != stWidth.nMax && camera_width != stWidth.nMin && (camera_width % 16))
            {
                camera_width -= (camera_width % 16);
            }
            printf("camera_width current value:%d\n", stWidth.nCurValue);
            printf("camera_width max value:%d\n", stWidth.nMax);
            printf("camera_width min value:%d\n", stWidth.nMin);
            printf("camera_width increment value:%d\n\n", stWidth.nInc);
        }
        else
        {
            printf("get camera_width failed! nRet [%x]\n\n", nRet);
            state = -1;
        }
        // 设置宽度
        // set IInteger variable
        // 宽高设置时需考虑步进(16)，即设置宽高需16的倍数
        // Step (16) should be considered when setting camera_width and height, that is the camera_width and height should be a multiple of 16
        if(camera_width != stWidth.nCurValue)
        {
            nRet = MV_CC_SetIntValue(handle, "Width", camera_width);
            if (MV_OK == nRet)
            {
                printf("set camera_width %d\n\n", camera_width);
            }
            else
            {
                printf("set camera_width failed! nRet [%x]\n\n", nRet);
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
            printf("exposure time current value:%f\n", stExposureTime.fCurValue);
            printf("exposure time max value:%f\n", stExposureTime.fMax);
            printf("exposure time min value:%f\n\n", stExposureTime.fMin);
        }
        else
        {
            printf("get exposure time failed! nRet [%x]\n\n", nRet);
            state = -1;
        }
        // 设置曝光
        // set IFloat variable
        if(abs(camera_exp - stExposureTime.fCurValue) > 0.00001)
        {
            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", camera_exp);
            if (MV_OK == nRet)
            {
                printf("set exposure time %f\n\n", camera_exp);
            }
            else
            {
                printf("set exposure time failed! nRet [%x]\n\n", nRet);
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
            printf("camera_gain current value:%f\n", stGain.fCurValue);
            printf("camera_gain time max value:%f\n", stGain.fMax);
            printf("camera_gain time min value:%f\n\n", stGain.fMin);
        }
        else
        {
            printf("get camera_gain failed! nRet [%x]\n\n", nRet);
            state = -1;
        }
        // 设置增益
        // set IFloat variable
        if(abs(camera_gain - stGain.fCurValue) > 0.01)
        {
            nRet = MV_CC_SetFloatValue(handle, "Gain", camera_gain);
            if (MV_OK == nRet)
            {
                printf("set camera_gain %f\n\n", camera_gain);
            }
            else
            {
                printf("set camera_gain failed! nRet [%x]\n\n", nRet);
                state = -1;
            }
        }
        // 创建定时器，定时发布
        //timer = this->create_wall_timer(std::chrono::nanoseconds (1000000000 / camera_framerate), std::bind(&CameraNode::timer_callback, this));
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
        cv::Mat image(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC3);
        //将char*数组转换为Mat
        for(int j = 0; j < stOutFrame.stFrameInfo.nHeight; j++)
        {
            unsigned char* data = image.ptr<unsigned char>(j);
            unsigned char* pSubDataForRGB = pDataForBGR + (j + 1) * stOutFrame.stFrameInfo.nWidth * 3;
            memcpy(data, pSubDataForRGB, stOutFrame.stFrameInfo.nWidth * 3);
        }
        //使用cv_bridge将Mat转为sensor_msgs::msg::Image类型
        cv_bridge::CvImage cv_image;
        cv_image.encoding = "bgr8";
        cv_image.header = image_header;
        cv_image.image = image;
        cv_image.toImageMsg(image_msg);
        image_pub->publish(image_msg);
        //cv::imshow("rgb", image);
        //cv::waitKey(1);
        if (pDataForBGR)
        {
            free(pDataForBGR);
            pDataForBGR = NULL;
        }
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
    // 构造函数,有一个参数为节点名称
    CameraNode(std::string name) : Node(name)
    {
        // 打印一句
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.",name.c_str());
        image_pub = this->create_publisher<sensor_msgs::msg::Image>("hik_camera/rgb", 1);
        memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
    }
    int restart_flag = 10;
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    std_msgs::msg::Header image_header;
    sensor_msgs::msg::Image image_msg;
    int camera_height = 540, camera_width = 720, camera_framerate = 200;
    float camera_exp = 4000, camera_gain = 5;
    std::string camera_name="camera";
    unsigned char *pDataForBGR = NULL;
    unsigned int nIndex = 0;
    void* handle = NULL;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    MVCC_INTVALUE stParam;
    MV_FRAME_OUT stOutFrame = {0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>("camera_node");
    int nRet = MV_OK;
    do
    {
        // ch:枚举设备 | en:Enum device
        nRet = node->enum_devices();
        if (MV_OK != nRet)
        {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:选择设备并创建句柄 | en:Select device and create handle


        nRet = node->select_device();
        if (MV_OK != nRet)
        {
            printf("Create Handle fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:打开设备 | en:Open device
        nRet = node->open_device();
        if (MV_OK != nRet)
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            break;
        }
        //修改相机参数
        node->getAndSetCameraParam();
        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        nRet = node->detection_network_optimal_package_size();
        // ch:设置触发模式为on | en:Set trigger mode as on
        nRet = node->set_trigger_mode();
        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:设置触发源 | en:Set trigger source
        //nRet = node->set_trigger_source();
        if (MV_OK != nRet)
        {
            printf("Set Trigger Source fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:获取数据包大小 | en:Get payload size
        nRet = node->get_package_size();
        if (MV_OK != nRet)
        {
            printf("Get Payload Size fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:开始取流 | en:Start grab image
        nRet = node->start_grabbing();
        if (MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }
        rclcpp::WallRate loop_rate(node->get_framerate());
        while (rclcpp::ok())
        {
            nRet = node->get_image();
            if (nRet == MV_OK)
            {
                nRet = node->convert_pixel_format();
                if (MV_OK != nRet)
                {
                    printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
                }
                nRet = node->publish_message();
                if (MV_OK != nRet)
                {
                    printf("publish_message fail!\n", nRet);
                }
                node->restart_flag = 10;
            }
            else
            {
                printf("No data! nRet [0x%x], time to restart %d\n", nRet, node->restart_flag);
                node->restart_flag--;
            }
            nRet = node->free_image_buffer();
            if(nRet != MV_OK)
            {
                printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
            }
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }
        // ch:停止取流 | en:Stop grab image
        nRet = node->stop_grabbing();
        if (MV_OK != nRet)
        {
            printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:关闭设备 | en:Close device
        nRet = node->close_device();
        if (MV_OK != nRet)
        {
            printf("Close Device fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:销毁句柄 | en:Destroy handle
        nRet = node->destroy_handle();
        if (MV_OK != nRet)
        {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
            break;
        }
    }while (0);
    printf("相机驱动节点安全退出\n");
    rclcpp::shutdown();
    return 0;
}

