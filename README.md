# vrpn_client_ros2

Ros2 package for transport vrpn data to ros2 topic




## Quick start

1. **安装VRPN库**
   
   执行以下命令安装VRPN库：

   ```bash
      git clone https://github.com/vrpn/vrpn.git
      mkdir -p vrpn/build
      cd vrpn/build
      cmake ..
      make 
      sudo make install
   ```

2. **拉取vrpn_client_ros2代码，并配置VRPN server参数**
   
   执行以下命令拉取vrpn_client_ros2代码：

   ```bash
      git clone https://github.com/efc-robot/vrpn_client_ros2
   ```

   拉取完成后，编辑代码中的配置文件 `vrpn_client_ros2/src/vrpn_listener/config/params.yaml`：

   ``` yaml
      /vrpn_listener:
         ros__parameters:
            server: 192.168.3.41              # VRPN server在局域网内的IP
            port: 3883                        # VRPN server的服务端口
            frame_id: "world"                 # reference frame参数
            mainloop_frequency: 100.0         # 主循环频率，频率越高，越不容易丢失VRPN数据包，但同时计算负载越大
            refresh_trackers_frequency: 1.0   # 追踪器刷新频率，频率越高，发现新的VRPN tracker的实时性越强
            tracker_mainloop_frequency: 100.0 # 追踪器主循环频率，频率越高，越不容易丢失VRPN数据包，但同时计算负载越大
   ```

   * 根据动捕软件的数据发送设置，配置 `server` 和 `port` 参数
   * 不推荐修改其余配置项，默认即可

3. **编译package**

   执行以下命令编译vrpn_client_ros2,其中 `${ROS_DISTRO}` 替换为实际使用的ROS2版本：

   ```bash
      source /opt/ros/${ROS_DISTRO}/setup.bash
      cd vrpn_client_ros2/src
      colcon build
   ```

4. **启动vrpn_client_ros2**
   
   执行以下命令运行vrpn_client_ros2：

   ```bash
      cd vrpn_client_ros2/src
      source install/setup.bash
      ros2 launch vrpn_listener sync_entity_state.launch
   ```

   执行该指令后，VRPN动捕数据将被转发为ROS2话题