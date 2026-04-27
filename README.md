# RosTo32_serial_protocol
ros系统控制32主控凌霄四轴飞行器的底层串口转换程序


速度话题(优先级 1)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.2}, angular: {z: 0.3}}"

降落命令(优先级 2)
服务路径：/serial_protocol_node/land
ros2 service call /serial_protocol_node/land std_srvs/srv/Trigger

解锁命令(优先级 3)
服务路径：/serial_protocol_node/arm
服务类型：std_srvs/srv/Trigger
ros2 service call /serial_protocol_node/arm std_srvs/srv/Trigger

锁定命令(优先级 4)
服务路径：/serial_protocol_node/disarm
ros2 service call /serial_protocol_node/disarm std_srvs/srv/Trigger

紧急停机命令（最高优先级 5）
服务路径：/serial_protocol_node/emergency
ros2 service call /serial_protocol_node/emergency std_srvs/srv/Trigger