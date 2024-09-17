  ## 环境要求
  linux ubuntu ROS2

  支持多种运行模式：Raspi（基本完成，未仿真测试）、ARM PC（开发中）、Jetson NX（开发中）

  ## 依赖仓库

  git clone https://github.com/PX4/px4_msgs.git


  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
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

  MicroXRCEAgent udp4 -p 8888
  sudo MicroXRCEAgent serial --dev /dev/ttyS6 -b 921600

  ## 安装MavROS
  humble/foxy
  sudo apt-get install ros-humble-mavros 
  sudo apt-get install ros-humble-mavros-extras

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