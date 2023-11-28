
#ifndef CAMERA_H
#define CAMERA_H
// Include necessary libraries
#include "rclcpp/rclcpp.hpp"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "opencv2/opencv.hpp"
#include "MvCameraControl.h"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
// Define the CameraNode class
/*
    创建一个类节点，名字叫做CameraNode,继承自Node.
*/
enum CameraParams
{
    ALL_PARAMS,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    CAMERA_FRAMERATE,
    CAMERA_ROI_OFFSET_X,
    CAMERA_ROI_OFFSET_Y,
    CAMERA_AUTO_EXP,
    CAMERA_AUTO_GAIN,
    CAMERA_AUTO_WHITEBALANCE,
    CAMERA_AUTO_MAXEXP,
    CAMERA_AUTO_MINEXP,
    CAMERA_AUTO_MAXGAIN,
    CAMERA_AUTO_MINGAIN,
    CAMERA_EXP,
    CAMERA_GAIN,
    NONE
};
class CameraNode : public rclcpp::Node
{

public:
    //打印相机信息
    bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
    //读取rosparam
    int getParam();
    //从相机内部获取参数，如果与要求不一致，则进行修改
    int getAndSetCameraParam(CameraParams which_param = CameraParams::ALL_PARAMS);
    int getAndSetCameraHeight();
    int getAndSetCameraWidth();
    int getAndSetCameraFramerate();
    int getAndSetCameraRoiOffsetX();
    int getAndSetCameraRoiOffsetY();
    int getAndSetCameraAutoExp();
    int getAndSetCameraAutoGain();
    int getAndSetCameraAutoWhitebalance();
    int getAndSetCameraAutoMaxexp();
    int getAndSetCameraAutoMinexp();
    int getAndSetCameraAutoMaxgain();
    int getAndSetCameraAutoMingain();
    int getAndSetCameraExp();
    int getAndSetCameraGain();
    int getAndSetCameraPixelFormat();
    //枚举相机
    int enum_devices();
    //选择相机，创建句柄
    int select_device();
    //打开相机
    int open_device();
    //探测网络最佳包大小(只对GigE相机有效)
    int detection_network_optimal_package_size();
    //设置触发模式
    int set_trigger_mode();
    //设置触发源
    int set_trigger_source();
    //获取数据包大小
    int get_package_size();
    //开始取流
    int start_grabbing();
    //停止取流
    int stop_grabbing();
    //关闭设备
    int close_device();
    //销毁句柄
    int destroy_handle();
    //获取帧率
    int get_framerate();
    //获取图像
    int get_image();
    //构造并发布消息
    int publish_message();
    //释放图像缓存
    int free_image_buffer();   
    // 构造函数,有一个参数为节点名称
    CameraNode(std::string name);
    int restart_flag = 10;
    CameraParams param_change = NONE;
    YAML::Node yaml_config;
    std::string camera_namespace; 
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    cv::VideoCapture cap;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
    rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_srv;
    void set_camera_info_callback(const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
                            std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response);
    std_msgs::msg::Header image_header;
    sensor_msgs::msg::Image image_msg;
    sensor_msgs::msg::CameraInfo camera_info_msg;
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
    std::array<double, 9> camera_matrix;
    std::vector<double> camera_distortion;
    std::array<double, 9> camera_rectification;
    std::array<double, 12> camera_projection;
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

#endif // CAMERA_H
