# Serial Protocol Node

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green)](./LICENSE)

ROS 2 节点的自定义串口协议发送器，将高层指令（速度、展示数据、服务请求）打包成 20 字节定长帧，通过物理串口发送给飞控或执行器。内置 **优先级调度、超时降级、节点退出安全停机** 功能。

---

## 📦 功能一览

| 功能 | 说明 |
|------|------|
| 🛩️ **速度控制** | 订阅 `cmd_vel` 话题，10 Hz 持续发送 VELOCITY 指令 |
| 🖥️ **展示数据传输** | 接收 16 字节原始数据，作为 `DISPLAY` 指令发送 |
| 🎮 **无人机服务** | `arm`、`disarm`、`land`、`emergency` 四个触发式服务 |
| ⚖️ **优先级抢占** | 高优先级命令立即覆盖低优先级，紧急停止最高 |
| ⏱️ **超时保护** | 速度 / 展示数据超时后自动逐步降级为心跳包 |
| 🛡️ **安全退出** | 节点析构时发送两次零速度指令确保停机 |

---

## 📡 订阅 & 服务

### 订阅话题

| 话题名 | 消息类型 | 用途 |
|--------|---------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 接收速度指令 (linear.x/y/z, angular.z) |
| `display_data` | `std_msgs/ByteMultiArray` | 接收展示模块原始数据（**必须 16 字节**） |

### 服务

| 服务名 | 类型 | 用途 |
|--------|------|------|
| `~/arm` | `std_srvs/Trigger` | 解锁电机 |
| `~/disarm` | `std_srvs/Trigger` | 锁定电机 |
| `~/land` | `std_srvs/Trigger` | 降落 |
| `~/emergency` | `std_srvs/Trigger` | 紧急停止 |

---

## 🧭 命令码与优先级

| 命令码 | 宏定义 | 优先级 (int) | 描述 |
|-------|--------|:-----------:|------|
| `0x00` | `CMD_HEARTBEAT` | 0 | 心跳包，空闲默认发送 |
| `0x03` | `CMD_VELOCITY` | 1 | 速度控制 |
| `0x07` | `CMD_DISPLAY` | 2 | 展示数据包 |
| `0x05` | `CMD_LAND` | 3 | 降落指令 |
| `0x01` | `CMD_ARM` | 4 | 解锁 |
| `0x02` | `CMD_DISARM` | 5 | 锁定 |
| `0x06` | `CMD_EMERGENCY` | 6 | 紧急停止（最高） |

---

## 📦 串口帧格式

```text
Byte |  0  |  1  |  2..5  |  6..9  | 10..13 | 14..17 |  18  |  19
-----+-----+-----+--------+--------+--------+--------+------+-----
     |0xA5 | cmd | float1 | float2 | float3 | float4 | XOR  |0x5B
     |帧头 |命令 |  数据1  |  数据2  |  数据3  |  数据4  |校验 |帧尾
- **帧头 (Header)**：固定为 `0xA5`
- **命令 (Command)**：1 字节指令码（见上方命令码表）
- **数据 (Data)**：4 个单精度浮点数（IEEE 754，小端序），共 16 字节
- **校验 (Checksum)**：前 18 字节（帧头 + 命令 + 16 字节数据）的**异或和**
- **帧尾 (Footer)**：固定为 `0x5B`
```
---

## 💻 快速开始

### 1. 编译

```bash
cd ros2_ws/src
git clone https://github.com/your-org/serial_protocol_node.git
cd ..
colcon build --packages-select serial_protocol_node
source install/setup.bash
```
### 2. 参数配置

节点支持以下 ROS 2 参数，可在启动时通过命令行或 YAML 文件覆盖：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `port` | string | `/dev/ttyS3` | 串口设备路径 |
| `baud` | int | `115200` | 波特率 |
| `send_rate_hz` | double | `10.0` | 主循环发送频率（Hz） |
| `vel_timeout` | double | `0.2` | 速度 / 展示数据超时时间（秒） |

### 3. 运行

```bash
# 使用默认参数启动
ros2 run serial_protocol_node serial_protocol_node

# 覆盖参数示例
ros2 run serial_protocol_node serial_protocol_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baud:=921600 \
  -p send_rate_hz:=20.0
```
### 4. 发送速度指令

```bash
# 持续发布 0.5 m/s 前向速度，0.1 rad/s 偏航角速度
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.1}}"
停止发布后，节点将在 `vel_timeout` 秒后自动执行零速降级。
```
### 5. 发送展示数据（16 字节十六进制）

例如需要向飞控发送四个浮点数 `[1.0, 2.0, 3.0, 4.0]`，其 IEEE 754 小端表示为  
`00 00 80 3F  00 00 00 40  00 00 40 40  00 00 80 40`

使用以下命令发布（**推荐方式一**）：

```bash
# 方式一：Python 单行自动转换十六进制字符串
ros2 topic pub /display_data std_msgs/msg/ByteMultiArray \
  "{data: $(python3 -c "print([int(b,16) for b in '00 00 80 3F 00 00 00 40 00 00 40 40 00 00 80 40'.split()])")}"

# 方式二：YAML !hex 标记（ROS 2 Humble 及以上）
ros2 topic pub /display_data std_msgs/msg/ByteMultiArray \
  "{data: [!hex 0x00, !hex 0x00, !hex 0x80, !hex 0x3F, !hex 0x00, !hex 0x00, !hex 0x00, !hex 0x40, !hex 0x00, !hex 0x00, !hex 0x40, !hex 0x40, !hex 0x00, !hex 0x00, !hex 0x80, !hex 0x40]}"
一旦收到数据，节点将以 10 Hz 持续发送 `CMD_DISPLAY` 帧，直至超时或被更高优先级命令抢占。
```
### 6. 调用服务

四个服务均为一次性触发，调用后立即生效并发送对应帧一次：

```bash
# 解锁
ros2 service call /serial_protocol_node/arm std_srvs/srv/Trigger

# 锁定
ros2 service call /serial_protocol_node/disarm std_srvs/srv/Trigger

# 降落
ros2 service call /serial_protocol_node/land std_srvs/srv/Trigger

# 紧急停止
ros2 service call /serial_protocol_node/emergency std_srvs/srv/Trigger
```

---

## 🔁 超时降级策略

- **速度指令超时**（`/cmd_vel` 停止超过 `vel_timeout`）  
  1. 立即发送 **一次性零速包**（`CMD_VELOCITY` + 全零数据）  
  2. 下一周期降级为 **心跳包**（`CMD_HEARTBEAT`）

- **展示数据超时**（`display_data` 停止超过 `vel_timeout`）  
  → 若当前命令仍为 `CMD_DISPLAY`，直接降级为 **心跳包**

---

## 🧠 优先级抢占示例

假设当前正在发送 `CMD_VELOCITY`（优先级 1），此时收到展示数据：

- 展示数据优先级 2 > 速度 1 → **立即切换**为 `CMD_DISPLAY`，并持续发送。
- 之后若调用 `emergency` 服务（优先级 6）→ **立刻覆盖**为紧急停止帧。
- 紧急帧发送一次后（one-shot），自动回退到 `CMD_DISPLAY`（若仍有效）或心跳。

---

## 🛡️ 安全退出

- 节点收到 `SIGINT` 或 `rclcpp::shutdown()` 时，析构函数会 **连续发送两次零速指令**（间隔 100 ms），确保飞行器进入静止状态。
- 紧急停止服务可随时打断任何指令，优先级最高。

---

## 📂 项目结构

```text
serial_protocol_node/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── serial_protocol/
│       └── serial_port.hpp
└── src/
    └── serial_protocol_node.cpp
```

---

## ⚠️ 注意事项

- 展示数据必须严格 **16 字节**（`ByteMultiArray.data.size() == 16`），否则报文将被丢弃并打印错误日志。
- 串口设备需提前存在并有读写权限（可能需要 `sudo` 或加入 `dialout` 组）。
- 修改 `send_rate_hz` 时请确保飞控端能匹配该频率，避免丢帧或缓冲区溢出。

---

## 📄 License

本项目基于 MIT License 开源，详见 [LICENSE](./LICENSE) 文件。

