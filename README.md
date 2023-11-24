# hik_camera_driver_ros2

ROS2下的海康相机驱动

## 目前已经实现的功能

- 兼容USB3.0和GIGE协议的相机
- 从yaml参数服务器中读取并设置相机参数
- 使用namespace和相机的用户自定义名称实现多相机启动
- 参数监视，参数改变后自动销毁句柄重新加载参数

## 目前存在的问题

- 性能开销较大
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

将`ifconfig eth0 mtu 9000`语句写到配置文件/etc/profile中，重启生效

>注意：eth0代表网卡名字，不同系统名字有可能不一样；9000代表最大接收包的大小，一些较老的网卡最大接收包或许无法达到，需根据实际来设置，并且相机侧需将GEVSCPSPacketSize节点设置成相应大小。

#### 编译

新建一个工作空间、拉取代码、编译，例如

`mkdir -p camera_driver_ws/src`

`cd camera_driver_ws/src`

`git clone git@github.com:nuaa-rm/hik_camera_driver_ros2.git`

`cd ..`

`colcon build`

### 运行

#### 单相机节点

连接好相机，进入工作空间

`source install/setup.bash`

`ros2 launch camera_driver camera.launch.py`

#### 参数监视

先启动相机节点

调整参数，需使用命令行`ros2 param set`操作，修改参数后，程序会自动停止取流，将新的参数写入相机。

>修改参数后，若想将参数永久保存，并在下一次启动时自动加载，需将参数写入yaml文件中。

>ROS2中，由于编译时源码中的yaml和launch文件夹被拷贝到了install文件夹中，所以在修改了yaml和launch文件后，需要重新编译，下一次启动才能生效。

#### 多相机节点

多相机节点启动时，必须在客户端修改每一个相机的名称，并且不重复，这是区分相机的依据

修改yaml文件和launch文件，两套参数按照命名空间区分。

### 常见问题

运行某些软件时报错： "undefined symbol: libusb_set_option"

问题原因：MVS在安装时，会安装一个libusb库，他的版本和系统自带的不同，导致报错。

解决方案：目前我就直接删除了/opt/MVS/lib/64/libusb-1.0.so，目前还没发现问题。

#### by:[@DoveJH](https://github.com/DoveJH) 1358446393@qq.com

