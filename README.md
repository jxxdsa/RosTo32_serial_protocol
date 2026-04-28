# 🚁 RosTo32_serial_protocol

> ROS 2 底层串口协议转换节点 —— 专为 **凌霄四轴飞行器（STM32 主控）** 设计

[![ROS2](https://img.shields.io/badge/ROS-2_Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/platform-Ubuntu%2022.04-lightgrey)]()

---

## 📖 目录

- [概述](#概述)
- [通信接口](#通信接口)
  - [🔹 速度控制话题（优先级 1）](#-速度控制话题优先级-1)
  - [🔸 降落服务（优先级 2）](#-降落服务优先级-2)
  - [🔸 解锁服务（优先级 3）](#-解锁服务优先级-3)
  - [🔸 锁定服务（优先级 4）](#-锁定服务优先级-4)
  - [🔴 紧急停机服务（优先级 5 - 最高）](#-紧急停机服务优先级-5---最高)
- [快速开始](#快速开始)
- [依赖项](#依赖项)
- [许可证](#许可证)

---

## 概述

`RosTo32_serial_protocol` 是 ROS 2 节点，负责将上层控制指令（如速度指令、飞行模式切换、安全指令）转换成飞行控制板（STM32）可识别的串口数据帧，并通过 UART 下发。  
节点内部实现优先级仲裁，确保**紧急停机**等安全指令能够优先发送。

---

## 通信接口

> **优先级策略**：数字越高越优先抢占串口发送权

| 优先级 | 接口类型 | 名称 | 用途 |
|:---:|:---:|:---|:---|
| 1 | 话题 | `/cmd_vel` | 速度控制 |
| 2 | 服务 | `/serial_protocol_node/land` | 降落 |
| 3 | 服务 | `/serial_protocol_node/arm` | 解锁电机 |
| 4 | 服务 | `/serial_protocol_node/disarm` | 锁定电机 |
| 5 | 服务 | `/serial_protocol_node/emergency` | 紧急停机 |

---

### 🔹 速度控制话题（优先级 1）

**话题**：`/cmd_vel`  
**消息类型**：`geometry_msgs/msg/Twist`  

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.2}, angular: {z: 0.3}}"
  
linear.x – 前后方向速度 (m/s)

linear.y – 左右方向速度 (m/s)

linear.z – 升降速度 (m/s)

angular.z – 自旋角速度 (rad/s)

🔸 降落服务（优先级 2）
服务：/serial_protocol_node/land
类型：std_srvs/srv/Trigger

bash
ros2 service call /serial_protocol_node/land std_srvs/srv/Trigger
🔸 解锁服务（优先级 3）
服务：/serial_protocol_node/arm
类型：std_srvs/srv/Trigger

bash
ros2 service call /serial_protocol_node/arm std_srvs/srv/Trigger
🔸 锁定服务（优先级 4）
服务：/serial_protocol_node/disarm
类型：std_srvs/srv/Trigger

bash
ros2 service call /serial_protocol_node/disarm std_srvs/srv/Trigger
🔴 紧急停机服务（优先级 5 - 最高）
服务：/serial_protocol_node/emergency
类型：std_srvs/srv/Trigger

bash
ros2 service call /serial_protocol_node/emergency std_srvs/srv/Trigger
⚠️ 调用后将立刻切断电机输出，不可恢复（需重新上锁/解锁）。

快速开始
bash
# 1. 克隆并编译
cd ~/ros2_ws/src
git clone <your-repo-url>
cd ..
colcon build --packages-select ros_to_32_serial_protocol

# 2. 配置串口权限
sudo chmod 666 /dev/ttyUSB0

# 3. 运行节点
ros2 run ros_to_32_serial_protocol serial_protocol_node --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200

# 4. 发送飞行指令（先解锁）
ros2 service call /serial_protocol_node/arm std_srvs/srv/Trigger
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.2}, angular: {z: 0.3}}"
依赖项
ROS 2 Humble（或更高版本）

geometry_msgs

std_srvs

rclcpp

支持串口通信的 Linux 环境（如 Ubuntu 22.04）

许可证
本项目采用 MIT License。