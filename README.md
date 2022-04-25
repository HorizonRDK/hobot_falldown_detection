# hobot_falldown_detection

## Intro

hobot_falldown_detection package是订阅ai msg，接收到body_kps数据后判断人体是否摔倒并发布摔倒事件的人体摔倒检测算法示例。
body_kps数据来源于订阅到的ai msg。
摔倒事件使用自定义的hobot ai msg发布出去，发布topic名为“falldown_event”。用户可以订阅此topic的ai msg用于应用开发。

## Build

### Dependency

ros package：

- ai_msgs

ai_msgs为自定义的消息格式，用于接收body_kps数据，发布推理结果，ai_msgs pkg定义在hobot_msgs中。

### 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

### 编译

支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

#### 编译选项

#### X3 Ubuntu系统上编译

1、编译环境确认

- 当前编译终端已设置ROS环境变量：`source /opt/ros/foxy/setup.bash`。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon，需要手动安装colcon。colcon安装命令：`apt update; apt install python3-colcon-common-extensions`
- 已安装ai_msgs

2、编译

- 编译命令：`colcon build --packages-select hobot_falldown_detection`

#### docker交叉编译

1、编译环境确认

- 在docker中编译，并且docker中已经安装好tros。docker安装、交叉编译说明、tros编译和部署说明：http://gitlab.hobot.cc/robot_dev_platform/robot_dev_config/blob/dev/README.md
- 已安装ai_msgs

2、编译

- 编译命令：

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

  colcon build --packages-select hobot_falldown_detection \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
  ```


## Usage

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh

# 运行：使用订阅到的ai msg进行摔倒检测，并设置log级别为warn
ros2 run hobot_falldown_detection hobot_falldown_detection --ros-args --log-level warn

# 运行参数配置：灵敏度paramSensivity默认为3(0:ExLow, 1:Low, 2:Middle, 3:High),订阅的ksp_point的topic默认为hobot_mono2d_body_detection,发布智能结果的topic默认为falldown_event.可通过-p选项更改默认
ros2 run hobot_falldown_detection hobot_falldown_detection --ros-args --log-level warn -p paramSensivity:=3 -p body_kps_topic_name:=hobot_mono2d_body_detection -p pub_smart_topic_name:=smart_topic

# web端展示渲染效果: hobot_falldown_detection的检测结果可以通过hobot_websocket查看web端渲染效果，需要在启动hobot_websocket的时候将订阅智能结果topic(smart_topic)与hobot_falldown_detection的发布智能结果的topic(pub_smart_topic_name)保持一致。查看上一条运行参数配置对hobot_falldown_detection的参数进行配置，hobot_websocket的参数配置请查看hobot_websocket/README.md。运行hobot_falldown_detection后，web端展示流程请查看hobot_websocket/README.md。
```
![image](./falldown.jpg)
