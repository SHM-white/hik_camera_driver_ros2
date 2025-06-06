#include "camera.h"
bool CameraNode::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
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
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
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
// 读取rosparam
int CameraNode::getParam()
{
    this->declare_parameter("camera_height", 540);                             /*声明参数*/
    this->get_parameter("camera_height", camera_height);                       /*获取参数*/
    this->declare_parameter("camera_width", 720);                              /*声明参数*/
    this->get_parameter("camera_width", camera_width);                         /*获取参数*/
    this->declare_parameter("camera_framerate", 200);                          /*声明参数*/
    this->get_parameter("camera_framerate", camera_framerate);                 /*获取参数*/
    this->declare_parameter("camera_exp", 4000.0);                             /*声明参数*/
    this->get_parameter("camera_exp", camera_exp);                             /*获取参数*/
    this->declare_parameter("camera_gain", 12.0);                              /*声明参数*/
    this->get_parameter("camera_gain", camera_gain);                           /*获取参数*/
    this->declare_parameter("camera_name", "camera");                          /*声明参数*/
    this->get_parameter("camera_name", camera_name);                           /*获取参数*/
    this->declare_parameter("camera_roi_offset_x", 0);                         /*声明参数*/
    this->get_parameter("camera_roi_offset_x", camera_roi_offset_x);           /*获取参数*/
    this->declare_parameter("camera_roi_offset_y", 0);                         /*声明参数*/
    this->get_parameter("camera_roi_offset_y", camera_roi_offset_y);           /*获取参数*/
    this->declare_parameter("camera_auto_exp", 0);                             /*声明参数*/
    this->get_parameter("camera_auto_exp", camera_auto_exp);                   /*获取参数*/
    this->declare_parameter("camera_auto_gain", 0);                            /*声明参数*/
    this->get_parameter("camera_auto_gain", camera_auto_gain);                 /*获取参数*/
    this->declare_parameter("camera_auto_whitebalance", 0);                    /*声明参数*/
    this->get_parameter("camera_auto_whitebalance", camera_auto_whitebalance); /*获取参数*/
    this->declare_parameter("camera_auto_maxexp", 4500);                       /*声明参数*/
    this->get_parameter("camera_auto_maxexp", camera_auto_maxexp);             /*获取参数*/
    this->declare_parameter("camera_auto_minexp", 100);                        /*声明参数*/
    this->get_parameter("camera_auto_minexp", camera_auto_minexp);             /*获取参数*/
    this->declare_parameter("camera_auto_maxgain", 17.0);                      /*声明参数*/
    this->get_parameter("camera_auto_maxgain", camera_auto_maxgain);           /*获取参数*/
    this->declare_parameter("camera_auto_mingain", 0.0);                       /*声明参数*/
    this->get_parameter("camera_auto_mingain", camera_auto_mingain);           /*获取参数*/
    std::vector<double> camera_matrix_vector;
    this->declare_parameter("camera_matrix", std::vector<double>(9, 0.0));     /*声明参数*/
    this->get_parameter("camera_matrix", camera_matrix_vector);                /*获取参数*/
    for (int i = 0; i < 9; i++)
    {
        camera_matrix[i] = camera_matrix_vector[i];
    }
    std::vector<double> camera_projection_vector;
    this->declare_parameter("camera_projection", std::vector<double>(12, 0.0));/*声明参数*/
    this->get_parameter("camera_projection", camera_projection_vector);        /*获取参数*/
    for (int i = 0; i < 12; i++)
    {
        camera_projection[i] = camera_projection_vector[i];
    }
    this->declare_parameter("camera_distortion", std::vector<double>(5, 0.0)); /*声明参数*/
    this->get_parameter("camera_distortion", camera_distortion);               /*获取参数*/
    std::vector<double> camera_rectification_vector;
    this->declare_parameter("camera_rectification", std::vector<double>(9, 0.0));/*声明参数*/
    this->get_parameter("camera_rectification", camera_rectification_vector);  /*获取参数*/
    for (int i = 0; i < 9; i++)
    {
        camera_rectification[i] = camera_rectification_vector[i];
    }
    return MV_OK;
}
// 枚举相机
int CameraNode::enum_devices()
{
    int nRet = MV_OK;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("Enum Devices fail! nRet [0x%x]\n", nRet);
        return nRet;
    }
    // 打印相机信息
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
// 选择相机，创建句柄
int CameraNode::select_device()
{
    int nRet = MV_OK;
    // 默认选择第一个枚举到的相机
    // 如果发现多个相机，根据自定义名称选择相机连接
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
                std::string target_camera_name((char *)pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
                if (target_camera_name == camera_name)
                {
                    nIndex = i;
                    break;
                }
            }
            else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
            {
                std::string target_camera_name((char *)pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
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
// 打开相机
int CameraNode::open_device()
{
    int nRet = MV_OK;
    // ch:打开设备 | en:Open device
    nRet = MV_CC_OpenDevice(handle);
    return nRet;
}
// 探测网络最佳包大小(只对GigE相机有效)
int CameraNode::detection_network_optimal_package_size()
{
    int nRet = MV_OK;
    // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
    if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
    {
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
        if (nPacketSize > 0)
        {
            nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
            if (nRet != MV_OK)
            {
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
// 设置触发模式
int CameraNode::set_trigger_mode()
{
    int nRet = MV_OK;
    // 设置触发模式为on
    // set trigger mode as on
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    return nRet;
}
// 设置触发源
int CameraNode::set_trigger_source()
{
    int nRet = MV_OK;
    // 设置触发源
    // set trigger source
    nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
    return nRet;
}
// 获取数据包大小
int CameraNode::get_package_size()
{
    int nRet = MV_OK;
    // ch:获取数据包大小 | en:Get payload size
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    return nRet;
}
// 开始取流
int CameraNode::start_grabbing()
{
    int nRet = MV_OK;
    // ch:开始取流 | en:Start grab image
    nRet = MV_CC_StartGrabbing(handle);
    return nRet;
}
// 停止取流
int CameraNode::stop_grabbing()
{
    int nRet = MV_OK;
    // ch:停止取流 | en:Stop grab image
    nRet = MV_CC_StopGrabbing(handle);
    return nRet;
}
// 关闭设备
int CameraNode::close_device()
{
    int nRet = MV_OK;
    // ch:关闭设备 | en:Close device
    nRet = MV_CC_CloseDevice(handle);
    return nRet;
}
// 销毁句柄
int CameraNode::destroy_handle()
{
    int nRet = MV_OK;
    // ch:销毁句柄 | en:Destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    return nRet;
}
// 获取帧率
int CameraNode::get_framerate()
{
    return camera_framerate;
}
// 获取图像
int CameraNode::get_image()
{
    int nRet = MV_OK;
    nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
    return nRet;
}
// 构造并发布消息
int CameraNode::publish_message()
{
    int nRet = MV_OK;
    // 构造消息
    uint64_t camera_time_int = stOutFrame.stFrameInfo.nDevTimeStampHigh; // 获取相机内部时间高32位
    camera_time_int <<= 32;
    camera_time_int |= stOutFrame.stFrameInfo.nDevTimeStampLow; // 获取相机内部时间低32位
    rclcpp::Time camera_time_ros(camera_time_int * 10);         // 构造ros::Time,乘十是因为相机内部时间为10e-8s
    image_header.stamp = camera_time_ros;                       // 时间戳的零点为相机上电时间
    image_header.stamp = camera_time_ros;
    image_header.frame_id = "hik_camera";
    image_msg.height = stOutFrame.stFrameInfo.nHeight;
    image_msg.width = stOutFrame.stFrameInfo.nWidth;
    image_msg.encoding = "rgb8";
    image_msg.header = image_header;
    image_msg.step = stOutFrame.stFrameInfo.nWidth * 3;
    image_msg.data = std::vector<unsigned char>(stOutFrame.pBufAddr, stOutFrame.pBufAddr + stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3);
    image_pub->publish(image_msg);
    camera_info_msg.header = image_header;
    camera_info_msg.height = stOutFrame.stFrameInfo.nHeight;
    camera_info_msg.width = stOutFrame.stFrameInfo.nWidth;
    camera_info_msg.distortion_model = "plumb_bob";
    camera_info_msg.d = camera_distortion;
    camera_info_msg.k = camera_matrix;
    camera_info_msg.r = camera_rectification;
    camera_info_msg.p = camera_projection;
    camera_info_pub->publish(camera_info_msg);
    return nRet;
}
// 释放图像缓存
int CameraNode::free_image_buffer()
{
    int nRet = MV_OK;
    if (NULL != stOutFrame.pBufAddr)
    {
        nRet = MV_CC_FreeImageBuffer(handle, &stOutFrame);
    }
    return nRet;
}
// 构造函数,有一个参数为节点名称
CameraNode::CameraNode(std::string name) : Node(name)
{
    // 打印一句
    RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
    image_pub = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    set_camera_info_srv = this->create_service<sensor_msgs::srv::SetCameraInfo>("/camera/set_camera_info", std::bind(&CameraNode::set_camera_info_callback, this, std::placeholders::_1, std::placeholders::_2));
    param_event_srv = this->create_service<camera_interfaces::srv::ParamEvent>("param_event", std::bind(&CameraNode::param_event_callback, this, std::placeholders::_1, std::placeholders::_2));
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
    camera_namespace = this->get_namespace();
    camera_namespace.erase(0,1);
    getParam();
}
void CameraNode::set_camera_info_callback(const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request, std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received a request on topic \"%s\"", "set_camera_info");
    response->success = true;
    response->status_message = "success";
    camera_distortion = request->camera_info.d;
    camera_matrix = request->camera_info.k;
    camera_rectification = request->camera_info.r;
    camera_projection = request->camera_info.p;
    yaml_config = YAML::LoadFile(std::string(ROOT_DIR) + std::string("yaml/camera.yaml"));
    for(int i=0;i<9;i++)
    {
        yaml_config[camera_namespace]["camera_driver_node"]["ros__parameters"]["camera_matrix"][i] = camera_matrix[i];
        std::string astring = yaml_config[camera_namespace]["camera_driver_node"]["ros__parameters"]["camera_matrix"][i].as<std::string>();
        if(astring.find(".") == std::string::npos)
        {
            astring += ".0";
        }
        yaml_config[camera_namespace]["camera_driver_node"]["ros__parameters"]["camera_matrix"][i] = astring;
    }
    for(int i=0;i<5;i++)
    {
        yaml_config[camera_namespace]["camera_driver_node"]["ros__parameters"]["camera_distortion"][i] = camera_distortion[i];
        std::string astring = yaml_config[camera_namespace]["camera_driver_node"]["ros__parameters"]["camera_distortion"][i].as<std::string>();
        if(astring.find(".") == std::string::npos)
        {
            astring += ".0";
        }
        yaml_config[camera_namespace]["camera_driver_node"]["ros__parameters"]["camera_distortion"][i] = astring;
    }
    for(int i=0;i<9;i++)
    {
        yaml_config[camera_namespace]["camera_driver_node"]["ros__parameters"]["camera_rectification"][i] = camera_rectification[i];
        std::string astring = yaml_config[camera_namespace]["camera_driver_node"]["ros__parameters"]["camera_rectification"][i].as<std::string>();
        if(astring.find(".") == std::string::npos)
        {
            astring += ".0";
        }
        yaml_config[camera_namespace]["camera_driver_node"]["ros__parameters"]["camera_rectification"][i] = astring;
    }
    for(int i=0;i<12;i++)
    {
        yaml_config[camera_namespace]["camera_driver_node"]["ros__parameters"]["camera_projection"][i] = camera_projection[i];
        std::string astring = yaml_config[camera_namespace]["camera_driver_node"]["ros__parameters"]["camera_projection"][i].as<std::string>();
        if(astring.find(".") == std::string::npos)
        {
            astring += ".0";
        }
        yaml_config[camera_namespace]["camera_driver_node"]["ros__parameters"]["camera_projection"][i] = astring;
    }
    std::ofstream yaml_out;
    yaml_out.open(std::string(ROOT_DIR) + std::string("yaml/camera.yaml"));
    yaml_out << yaml_config;
    yaml_out.close();
    RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", (long int)response->success);
}
