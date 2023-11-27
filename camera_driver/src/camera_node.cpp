#include"camera.h"
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>("camera_node");
    int nRet = MV_OK;
    do
    {
        // ch:枚举设备 | en:Enum device
        RESTART:
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
        node->getAndSetCameraParam(CameraParams::ALL_PARAMS);
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
                if(node->restart_flag <= 0)
                {
                    break;
                }
            }
            nRet = node->free_image_buffer();
            if(nRet != MV_OK)
            {
                printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
            }
            rclcpp::spin_some(node);
            if(node->param_change != NONE)
            {
                node->stop_grabbing();
                node->getAndSetCameraParam(node->param_change);
                node->start_grabbing();
                node->param_change = NONE;
            }  
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

