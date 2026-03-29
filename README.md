# Robot Arm Teaching System

基于 ESP32-S3 的 6 轴机械臂示教控制系统，支持 Web 控制、示教录制回放、IMU 体感遥控和动作持久化存储。

## 功能特性

- **Web 控制** - 通过浏览器拖动滑杆实时控制 6 个关节，实时反馈当前位置
- **示教录制/回放** - 关闭扭矩手动拖动示教，50Hz 采样录制，支持单次/循环回放
- **动作持久化** - 录制的动作可保存到 Flash（SPIFFS），断电不丢失，支持命名、加载、删除
- **IMU 体感遥控** - 通过另一块 ESP32-S3 + MPU6050 的体感控制器，经 ESP-NOW 无线控制机械臂
- **正/逆运动学** - 5 自由度运动学求解，支持末端位置计算
- **回原点** - 一键回到预设 Home 位置，带渐进扭矩释放

## 硬件要求

| 组件 | 型号 |
|------|------|
| 主控 | ESP32-S3 (16MB Flash) |
| 舵机 | STS3215 x6（UART 总线舵机，1Mbps） |
| 通信 | UART1 (TX=GPIO17, RX=GPIO18) |
| IMU 控制器（可选） | ESP32-S3 + MPU6050 + 摇杆 + 按钮 |

### 关节配置

| 关节 | 舵机 ID | 范围 | 初始位置 | 说明 |
|------|---------|------|----------|------|
| J1 | 1 | 1900-4000 | 3000 | 底座旋转 |
| J2 | 2 | 1000-2500 | 2180 | 肩部 |
| J3 | 3 | 1550-3000 | 1550 | 肘部 |
| J4 | 4 | 400-4000 | 1970 | 腕部 |
| J5 | 5 | 2200-4000 | 3000 | 手腕 |
| J6 | 6 | 2000-4000 | 2048 | 夹爪 |

## 项目结构

```
robot-arm-esp32-main/
├── main/
│   ├── main.c            # 入口，初始化各模块
│   ├── robot.c/h         # 机械臂控制核心（模式管理、录制回放、50Hz 伺服循环）
│   ├── servo.c/h         # STS3215 舵机 UART 驱动（自动波特率检测）
│   ├── kinematics.c/h    # 5DOF 正/逆运动学（DH 参数）
│   ├── web_server.c/h    # HTTP + WebSocket 服务器
│   ├── web_page.h        # 内嵌 HTML/CSS/JS 控制页面
│   ├── wifi.c/h          # WiFi STA 模式连接
│   ├── espnow_rx.c/h     # ESP-NOW 接收（IMU 数据包）
│   ├── action_store.c/h  # SPIFFS 动作存储（保存/加载/删除/列表）
│   └── CMakeLists.txt
├── imu_controller/       # IMU 体感控制器固件（独立项目）
│   └── main/main.c       # MPU6050 + ESP-NOW 发送端
├── partitions.csv        # 自定义分区表（2MB App + 2MB SPIFFS）
├── sdkconfig             # ESP-IDF 配置
└── CMakeLists.txt
```

## 编译与烧录

### 环境要求

- [ESP-IDF v5.1.x](https://docs.espressif.com/projects/esp-idf/en/v5.1.2/esp32s3/get-started/)
- 目标芯片：ESP32-S3

### 构建

```bash
idf.py set-target esp32s3
idf.py build
```

### 烧录并监控

```bash
idf.py flash monitor
```

首次烧录会自动格式化 SPIFFS 分区，日志中会出现：

```
W (xxx) SPIFFS: mount failed, -10025. formatting...
I (xxx) STORE: SPIFFS mounted. Total: xxxx KB, Used: 0 KB, Free: xxxx KB
```

这是正常行为，仅首次发生。

### WiFi 配置

在 `main/wifi.c` 顶部修改 WiFi SSID 和密码：

```c
#define WIFI_SSID     "your_ssid"
#define WIFI_PASS     "your_password"
```

## 使用方法

1. 烧录后，查看串口日志获取 IP 地址（如 `192.168.1.103`）
2. 在浏览器中访问该 IP
3. 页面功能：
   - **滑杆控制** - 拖动 J1-J6 滑杆实时控制各关节
   - **示教** - 点击后舵机扭矩释放，手动拖动机械臂演示动作
   - **停止** - 停止录制/回放，恢复扭矩
   - **播放 / 循环播放** - 回放录制的动作
   - **回原点** - 机械臂回到 Home 位置
   - **动作管理** - 输入名称保存动作到 Flash，或从列表加载/删除已保存的动作

### 操作流程示例

```
示教录制 → 停止 → 输入名称"点头" → 保存
                                      ↓
      加载"点头" → 播放（或循环播放）← 下次开机从列表选择
```

## IMU 体感控制器（可选）

`imu_controller/` 目录包含体感遥控器的固件，运行在独立的 ESP32-S3 上。

### 接线

| 组件 | 引脚 |
|------|------|
| MPU6050 SCL | GPIO9 |
| MPU6050 SDA | GPIO8 |
| 摇杆 VRX | GPIO1 (ADC) |
| 摇杆 VRY | GPIO2 (ADC) |
| 按钮 SW | GPIO4 |

### 使用

1. 将 `imu_controller/` 作为独立 ESP-IDF 项目编译烧录到第二块 ESP32-S3
2. 确保两块 ESP32 连接同一 WiFi 信道
3. 在 Web 页面点击"IMU 遥控"按钮启用体感控制模式
4. 按住按钮（死人开关）→ 倾斜控制器 → 机械臂跟随运动

## 分区表

| 分区 | 类型 | 偏移 | 大小 |
|------|------|------|------|
| nvs | NVS | 0x9000 | 24 KB |
| phy_init | PHY | 0xF000 | 4 KB |
| factory | App | 0x10000 | 1984 KB |
| storage | SPIFFS | 0x200000 | 2048 KB |

## 技术细节

- **通信协议**: WebSocket JSON（浏览器 ↔ ESP32），ESP-NOW 二进制包（IMU 控制器 → ESP32）
- **伺服循环**: 50Hz FreeRTOS 任务，mutex 保护共享数据
- **舵机驱动**: STS3215 UART 协议，1Mbps，自动波特率检测
- **运动学参数**: d1=65mm, a2=105mm, a3=105mm, d5=80mm
- **录制容量**: 内存最大 6000 帧（约 2 分钟 @50Hz），Flash 可存多个动作文件

## 致谢

- [微雪科技 (Waveshare)](https://www.waveshare.net/) - 提供 STS3215 舵机适配机械爪的 3D 建模图，本项目的机械臂结构基于其模型完成
- [罗大富Bigrich](https://space.bilibili.com/475498258) - B站 Fusion 360 建模教学视频，帮助完成机械臂的结构设计
- [艾谷科技](https://space.bilibili.com/3546572271498137) - B站 ESP32 入门教学视频，提供了嵌入式开发的基础指导
- [黑马程序员](https://space.bilibili.com/37974444) - B站零基础具身智能课程，提供了具身智能开发的系统学习资源

## License

MIT
