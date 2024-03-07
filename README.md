# hik_camera_driver_ros2

ROS2下的海康相机驱动

## 目前已经实现的功能

- 兼容USB3.0和GIGE协议的相机
- 从yaml参数服务器中读取并设置相机参数
- 使用namespace和相机的用户自定义名称实现多相机启动
- 参数更改，参数改变后自动停止取流重新加载参数
- 参数查询
- 使用camera_calibration包进行标定后，自动保存参数到yaml文件

## 目前存在的问题

- 目前仅写了必要参数的加载，还有一部分参数没有写进驱动

## 依赖

- [MVS v2.1.2](https://www.hikrobotics.com/cn/machinevision/service/download?module=0)
- [工业相机Runtime组件](https://www.hikrobotics.com/cn/machinevision/service/download?module=0)
- OpenCV 4
- ROS2

## 使用说明

### 部署

#### 确保你已经正确安装ROS2

#### 安装MVS

从[海康机器人官网](https://www.hikrobotics.com/cn/machinevision/service/download?module=0)下载工业相机客户端MVS安装包，并解压，进入目录。

使用`uname -a`查询电脑架构，根据电脑架构使用对应的安装包。例如：

`sudo dpkg -i MVS-2.1.2_x86_64_20231011.deb`

#### 安装工业相机SDKRuntime组件包

从[海康机器人官网](https://www.hikrobotics.com/cn/machinevision/service/download?module=0)下载工业相机SDKRuntime组件包安装包，并解压，进入目录。

使用`uname -a`查询电脑架构，根据电脑架构使用对应的安装包。例如：

`sudo dpkg -i MvCamCtrlSDK_Runtime-4.1.2_x86_64_20231011.deb`

#### 网口相机建议开启巨帧

将`ifconfig eth0 mtu 9000`语句写到配置文件/etc/profile中，重启生效。

>注意：eth0代表网卡名字，不同系统名字有可能不一样；9000代表最大接收包的大小，一些较老的网卡最大接收包或许无法达到，需根据实际来设置，并且相机侧需将GEVSCPSPacketSize节点设置成相应大小。

#### 编译

新建一个工作空间、拉取代码、编译

`mkdir -p camera_driver_ws/src`

`cd camera_driver_ws/src`

`git clone git@github.com:nuaa-rm/hik_camera_driver_ros2.git`

`cd ..`

`colcon build --symlink-install`

>`--symlink-install`参数的作用：编译时如果install中的文件已经存在于src或者build文件夹中，就用超链接指向该文件，实现同步更新（修改yaml和launch文件后无需编译），也避免浪费空间。

### 参数文件camera.yaml说明

camera.yaml:

    camera1: #与命名空间对应，使用的命名空间决定了读入的参数
        camera_driver_node:
            ros__parameters:
                camera_name: camera1 #相机名称，用于区分不同相机，从相机客户端中修改
                camera_height: 128 #使用裁剪实现的，数值大小会影响相机视野
                camera_width: 1440 #使用裁剪实现的，数值大小会影响相机视野
                camera_exp: 2000.0 #曝光时间，曝光时间与帧率尽量匹配
                camera_gain: 15.0 #增益，增益越大，图像越亮，但噪点越明显、动态范围越小，应该在亮度允许的条件下尽量小
                camera_framerate: 200 #帧率，帧率与曝光时间尽量匹配
                camera_roi_offset_x: 0 #裁剪图像的偏移量
                camera_roi_offset_y: 0 #裁剪图像的偏移量
                camera_auto_exp: 0 #是否使用自动曝光 0:关闭 1:单次 2:连续
                camera_auto_maxexp: 4500 #自动曝光的最大值
                camera_auto_minexp: 100 #自动曝光的最小值
                camera_auto_gain: 0 #是否使用自动增益 0:关闭 1:单次 2:连续
                camera_auto_maxgain: 17.0 #自动增益的最大值
                camera_auto_mingain: 0.0 #自动增益的最小值
                camera_auto_whitebalance: 0 #是否使用自动白平衡 0:关闭 1:单次 2:连续
                camera_matrix: [1549.8015577176407, 0.0, 774.50694325965446, 0.0, 1546.0969313119535, 353.35181301418544, 0.0, 0.0, 1.0]
                #相机内参矩阵K
                camera_distortion: [0.1405344646814807, -0.8867151879475168, -0.012452148491565052, 0.004768478116866451, 0.0]
                #相机畸变参数D
                camera_rectification: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                #相机矫正参数R
                camera_projection: [1524.743408203125, 0.0, 787.72717800308601, 0.0, 0.0, 1555.3603515625, 349.04974594277155, 0.0, 0.0, 0.0, 1.0, 0.0]
                #相机的投影矩阵P

> 注意，在相机标定完成后，会调用`/camera/set_camera_info`服务，这个服务会将标定内参写入yaml文件，这个过程会将整个yaml文件重写，文件中的注释会被清除。这也是yaml文件在此处进行详细解释的原因。

> 在CmakeList.txt文件中可以看到，yaml文件夹和launch文件夹都被安装（拷贝）到了install文件夹下。使用`ros2 launch`命令调用的launch文件和在启动时加载的yaml文件都来自于install文件夹下。所以，在改变了yaml文件和launch文件后，需要重新编译，使新的yaml和launch文件拷贝过去。或者如上文所说，也可以在`colcon build`命令后添加`--symlink-install`参数。此时不再是简单的拷贝，而是生成超链接，在src文件夹下修改，install文件夹也会同步更新。


### 接口camera_interfaces/srv/ParamEvent介绍

这个接口用来实现参数修改服务，在`camera_interfaces` 中定义，如果需要调用该服务来实现修改和查询相机参数，请依赖该包，在编译时先编译该包并且在编译其他包时source该包。

ParamEvent.srv:
    
    # 以下15个有初值的参数作用类似于枚举类，他们指出了1~15代表的实际含义，请不要修改他们的值
    #当param参数等于CAMERA_HEIGHT~CAMERA_GAIN时，服务会将对应的参数修改为value的值，并返回所有参数的值。当param等于CHECK时，不执行任何修改操作，只返回当前参数值。
    int8 CAMERA_HEIGHT = 1
    int8 CAMERA_WIDTH = 2
    int8 CAMERA_FRAMERATE = 3
    int8 CAMERA_ROI_OFFSET_X = 4
    int8 CAMERA_ROI_OFFSET_Y = 5
    int8 CAMERA_AUTO_EXP = 6
    int8 CAMERA_AUTO_GAIN = 7
    int8 CAMERA_AUTO_WHITEBALANCE = 8
    int8 CAMERA_AUTO_MAXEXP = 9
    int8 CAMERA_AUTO_MINEXP = 10
    int8 CAMERA_AUTO_MAXGAIN = 11
    int8 CAMERA_AUTO_MINGAIN = 12
    int8 CAMERA_EXP = 13
    int8 CAMERA_GAIN = 14
    int8 CHECK = 15
    int8 param_name
    float64 value
    ---
    bool success
    string status_message
    int32 camera_height
    int32 camera_width
    float64 camera_exp
    float64 camera_gain
    int32 camera_framerate
    int32 camera_roi_offset_x
    int32 camera_roi_offset_y
    int32 camera_auto_exp
    int32 camera_auto_maxexp
    int32 camera_auto_minexp
    int32 camera_auto_gain
    float64 camera_auto_maxgain
    float64 camera_auto_mingain
    int32 camera_auto_whitebalance

### 运行

#### 单相机节点启动

连接好相机，进入工作空间

`source install/setup.bash`

`ros2 launch camera_driver camera.launch.py`

#### 参数查询与修改

参数查询与修改是调用`param_event`服务实现的，接口为`camera_interfaces/srv/ParamEvent`。下面分别在命令行和C++程序中给出示例：

##### C++实现

修改CmakeList.txt文件，使你的包依赖`camera_interfaces`包。

`find_package(camera_interfaces REQUIRED)`
`ament_target_dependencies(executable_name camera_interfaces other_dependences)`

下面给出一个请求函数的示例

    void send_request()
    {
        // Wait for the server to be available
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        // Call the service
        auto request = std::make_shared<camera_interfaces::srv::ParamEvent::Request>();
        // Set the request data here
        request->param_name = request->CHECK;
        // 如上所示，没有给出param_name的具体值，而是使用request中CHECK的初值。这种写法等价于request->param_name = 15，但是显然这种写法更加直观。
        client_->async_send_request(request, std::bind(&MyClientNode::result_callback_, this, std::placeholders::_1));
    }

##### 命令行实现

先启动相机节点。

新建一个终端，source工作空间

`ros2 service call /namespace/param_event camera_interfaces/srv/ParamEvent "{param_name: 1,value: 540}"`

这里作者还不会如何在命令行中使用类似于C++中`request->param_name = request->CHECK;`的用法，所以param_name只能给出具体值了:(。

>修改参数后，若想将参数永久保存，并在下一次启动时自动加载，需将参数写入yaml文件中。

>再次强调，ROS2中，由于编译时源码中的yaml和launch文件夹被拷贝到了install文件夹中，所以在修改了yaml和launch文件后，需要重新编译，下一次启动才能生效，除非在`colcon build`命令后添加`--symlink-install`参数。

#### 相机标定

安装相关包

`sudo apt install ros-<ros2-distro>-camera-calibration-parsers`

`sudo apt install ros-<ros2-distro>-camera-info-manager`

`sudo apt install ros-<ros2-distro>-launch-testing-ament-cmake`

安装camera_calibration

`sudo apt install ros-<ros2-distro>-camera-calibration`

启动相机标定节点

`ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.02 --ros-args -r image:=/my_camera/image_raw -p camera:=/my_camera`

--size 7x9：标定板的黑白块交点个数（而不是黑白格子的个数，实际上是黑白格子个数-1）。

--square 0.02：格子的边长，单位为m。

image:=/my_camera/image_raw：你的图片topic。

camera:=/my_camera：相机名称，在当前场景下无用。

图片采集完成后，点击`calibrate`按钮，等待标定完成后，点击`commit`按钮，程序会自动调用`set_camera_info`服务，将标定得到的K、D、P、R矩阵写入yaml文件，`/namespace/camera_info`话题下发布的相机内参信息也会更新。

>再再次强调，如果你没有使用`--symlink-install`参数，请在标定完成之后重新编译。

#### 多相机节点启动

多相机节点启动时，必须在客户端修改每一个相机的名称，并且不重复，这是区分相机的依据。

修改yaml文件和launch文件，两套参数按照命名空间区分。

## 常见问题

运行某些软件时报错：`undefined symbol: libusb_set_option`

问题原因：MVS在安装时，会安装一个libusb库，他的版本和系统自带的不同，导致报错。

解决方案：目前作者就直接删除了/opt/MVS/lib/64/libusb-1.0.so，目前还没发现问题。

#### by:[@DoveJH](https://github.com/DoveJH) 1358446393@qq.com

