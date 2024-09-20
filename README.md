  ## 环境要求
  linux ubuntu ROS2

  支持多种运行模式：Raspi（基本完成，未仿真测试）、ARM PC（开发中）、Jetson NX（开发中）

  ## 安装Micro-XRCE-DDS

  Micro-XRCE-DDS适用于ROS2环境的可用于与PX4 1.14以上版本进行分布式通信的方式，也是MavROS的替代

  ```shell
  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  cd Micro-XRCE-DDS-Agent
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  sudo ldconfig /usr/local/lib/
  ```
  ## 测试Micro-XRCE-DDS

  `MicroXRCEAgent udp4 -p 8888`

  wsl1中运行
  `sudo MicroXRCEAgent serial --dev /dev/ttyS6 -b 921600`
  
  Raspi中使用GPIO UART运行
  `sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600`
  
  Raspi中使用USB UART运行
  `sudo MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600`
  
  Xavier-NX中使用GPIO UART运行
  `sudo MicroXRCEAgent serial --dev /dev/ttyTHS0 -b 921600`

  ## 使用Micro-XRCE-DDS

  安装PX4消息定义，在src目录下
  
  `git clone https://github.com/PX4/px4_msgs.git`

  
  > 也可以设置为submodule，git submodule add https://github.com/PX4/px4_msgs.git src/px4_msgs

  `colcon build`编译即可
  
  安装测试例子，在src目录下

  ```sh
  git clone https://github.com/PX4/px4_ros_com.git
  ```
  
  clone后需要修改文件src/px4_ros_com/launch/sensor_combined_listener.launch.py的对应部分为
  
  ```python
  def generate_launch_description():

    micro_ros_agent = ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600'
        ]],
        shell=True
    )

    sensor_combined_listener_node = Node(
        package='px4_ros_com',
        executable='sensor_combined_listener',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        micro_ros_agent,
        sensor_combined_listener_node
    ])
  ```

  再次执行编译
  `colcon build --packages-select px4_ros_com`

  运行

  ```sh
  source install/local_setup.bash
  ros2 launch px4_ros_com sensor_combined_listener.launch.py
  ```

  ## 安装MavROS

  安装ROS2 foxy对应的MavROS(ubuntu 22.04)
  ```sh
  sudo apt-get install ros-foxy-mavros 
  sudo apt-get install ros-foxy-mavros-extras
  ```

  安装ROS2 humble对应的MavROS(ubuntu 22.04)
  ```sh
  sudo apt-get install ros-humble-mavros 
  sudo apt-get install ros-humble-mavros-extras
  ```

  安装ROS2 jazzy对应的MavROS(ubuntu 24.04)
  ```sh
  sudo apt-get install ros-jazzy-mavros 
  sudo apt-get install ros-jazzy-mavros-extras
  ```
  
  安装依赖
  ```sh
  git clone -b ros2 https://github.com/mavlink/mavros.git
  cd mavros/mavros/scripts
  sudo ./install_geographiclib_datasets.sh
  ```

  参考：[ROS2安装MavROS](https://blog.csdn.net/sinat_16643223/article/details/136144717)

  运行：
  ```sh
  ros2 run mavros mavros_node --ros-args --params-file ./mavros/mavros_param_1.yaml
  ```

  ## 启用树莓派串口设备
  参考连接：
  https://www.raspberrypi.com/documentation/computers/configuration.html#configure-uarts

  对于树莓派5，要启用经典位置的串口需要在config.txt中添加以下配置
  dtoverlay=uart0-pi5
  uart0对应linux下的/dev/ttyAMA0，GPIO14和GPIO15。而默认的/dev/ttyAMA10是树莓派5主板上的专用串口。
  不同的串口设备的默认GPIO号可以参考：https://github.com/raspberrypi/linux/blob/rpi-6.1.y/arch/arm/boot/dts/overlays/README

  同时需要在/boot/firmware/cmdline.txt删除串口终端部分的内容（暂时不确定该串口终端是否对应主板上的专用串口）

  ### 运行dtoverlay -a查看系统可用的串口设备
  
  ```shell
  uart0
  uart0-pi5
  uart1
  uart1-pi5
  uart2
  uart2-pi5
  uart3
  uart3-pi5
  uart4
  uart4-pi5
  uart5
  ```

## 启用树莓派的spi设备
参考连接：
https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#spi-overview
```shell
dtparam=spi=on
dtoverlay=spi0-2cs
dtoverlay=spi1-2cs
```