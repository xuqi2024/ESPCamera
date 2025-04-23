# ESP32-S3 Camera UDP视频流项目

这是一个基于ESP32-S3和OV3660摄像头的实时视频流项目。该项目通过UDP协议在WiFi网络上传输JPEG格式的视频数据。

## 功能特点

- 支持VGA分辨率（640x480）
- JPEG格式视频流传输
- UDP广播模式，支持多客户端接收
- 实时FPS统计
- 自适应传输控制
- 丢包率统计和反馈机制

## 硬件要求

- ESP32-S3-N16R8开发板
- OV3660摄像头模块
- 可用的WiFi网络

## 引脚连接

摄像头与ESP32-S3的连接方式：
```
PWDN  -> 不使用
RESET -> 不使用
XCLK  -> GPIO15
SIOD  -> GPIO4
SIOC  -> GPIO5
D7    -> GPIO16
D6    -> GPIO17
D5    -> GPIO18
D4    -> GPIO12
D3    -> GPIO10
D2    -> GPIO8
D1    -> GPIO9
D0    -> GPIO11
VSYNC -> GPIO6
HREF  -> GPIO7
PCLK  -> GPIO13
```

## 软件依赖

- ESP-IDF (推荐使用最新版本)
- Python 3.6+
- OpenCV-Python
- NumPy

## 配置说明

### ESP32端配置

在`main/main.c`文件中：

1. WiFi配置：
```c
#define WIFI_SSID "你的WiFi名称"
#define WIFI_PASS "你的WiFi密码"
```

2. UDP配置：
```c
#define UDP_PORT 8888            // UDP端口
#define PACKET_SIZE 1024         // 包大小
#define BROADCAST_ADDR "255.255.255.255"  // 广播地址
```

3. 相机配置：
```c
s->set_framesize(s, FRAMESIZE_VGA);    // 设置分辨率
s->set_quality(s, adaptive_ctrl.jpeg_quality);  // JPEG质量（动态调整）
```

### Python客户端配置

在`pc_client.py`文件中：

1. UDP配置：
```python
UDP_IP = "0.0.0.0"  # 监听所有网络接口
UDP_PORT = 8888     # 与ESP32端相同的端口
```

## 编译和运行

### ESP32端

1. 进入项目目录：
```bash
cd ESP32-S3-N16R8-CAM-OV3660
```

2. 编译项目：
```bash
idf.py build
```

3. 烧录程序：
```bash
idf.py -p [PORT] flash monitor
```

### Python客户端

1. 安装依赖：
```bash
pip install opencv-python numpy
```

2. 运行客户端：
```bash
python pc_client.py
```

## 工作原理

1. 数据流程：
   - ESP32摄像头捕获图像
   - 直接输出JPEG格式数据
   - 通过UDP广播发送数据包
   - Python客户端接收并重组数据包
   - 解码JPEG数据并显示图像

2. 自适应控制：
   - 监控丢包率
   - 动态调整传输参数
   - 包括帧率、JPEG质量等

3. 反馈机制：
   - 客户端每秒发送统计数据
   - 包括丢包数和总包数
   - ESP32根据反馈调整参数

## 性能优化

1. 传输优化：
   - 使用1024字节的包大小
   - UDP广播模式减少延迟
   - 包间延迟控制

2. 内存优化：
   - 使用ESP32的PSRAM
   - 优化缓冲区管理
   - 及时释放不用的内存

3. 图像优化：
   - JPEG格式直接传输
   - 动态质量调整
   - 自适应帧率控制

## 故障排除

1. 无法连接WiFi：
   - 检查WiFi配置是否正确
   - 确认WiFi信号强度
   - 查看ESP32日志输出

2. 视频流不稳定：
   - 检查网络环境
   - 调整包大小和发送间隔
   - 观察丢包率统计

3. 图像质量问题：
   - 调整JPEG质量参数
   - 检查光照条件
   - 验证摄像头配置

## 注意事项

1. 网络环境：
   - 建议在局域网内使用
   - 避免网络拥堵
   - 保持稳定的WiFi连接

2. 资源使用：
   - 监控ESP32内存使用
   - 注意Python客户端的CPU占用
   - 及时释放资源

3. 安全性：
   - UDP广播无加密
   - 仅建议在可信网络使用
   - 注意保护WiFi密码

## 许可证

MIT License

## 贡献

欢迎提交Issue和Pull Request来改进项目。
