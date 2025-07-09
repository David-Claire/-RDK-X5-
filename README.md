# 基于RDK X5的机器视觉与自主避障智能灭火车

![License](https://img.shields.io/badge/License-MIT-blue.svg)
![Platform](https://img.shields.io/badge/Platform-RDK%20X5-orange.svg)
![Language](https://img.shields.io/badge/Language-Python-green.svg)
![Status](https://img.shields.io/badge/Status-Stable-brightgreen.svg)

一款基于RDK X5平台的智能灭火机器人，集成机器视觉、自主避障与智能灭火功能，专为室内初期火灾快速响应和危险环境灭火作业而设计。

## 🌟 项目概述

本项目打造了一款融合多传感器的智能灭火车，利用RDK X5的强大算力（10 TOPS）实现：
- 🔥 **智能火源识别**：基于YOLOv5模型的实时火焰检测
- 🚧 **自主避障导航**：激光雷达SLAM + MPPI路径规划
- 💧 **精准灭火执行**：水泵驱动的柱状/雾状水灭火
- 🔄 **模块化设计**：支持快速功能扩展与设备更换

## 📋 目录

- [功能特性](#功能特性)
- [技术架构](#技术架构)
- [硬件需求](#硬件需求)
- [安装部署](#安装部署)
- [使用指南](#使用指南)
- [性能指标](#性能指标)
- [应用场景](#应用场景)
- [开发指南](#开发指南)
- [贡献指南](#贡献指南)
- [许可证](#许可证)

## ✨ 功能特性

### 🎯 核心功能
- **多模态感知融合**：激光雷达 + RGB-D相机，贝叶斯方法数据融合
- **动态地图构建**：实时2D SLAM建图，栅格地图动态更新
- **智能路径规划**：MPPI算法优化路径，自主避障导航
- **精准火源定位**：YOLOv5深度学习模型，±10cm检测精度
- **自动灭火执行**：GPIO控制水泵，2m有效射程

### 🔧 技术亮点
- **低成本方案**：功放+水泵轻量化设计，成本可控
- **模块化架构**：标准化快拆接口，支持功能扩展
- **实时响应**：传感器时空对齐精度±100微秒
- **环境适应**：适配狭窄空间、复杂地形作业

## 🏗️ 技术架构

```
┌─────────────────────────────────────────────┐
│                 RDK X5 主控                  │
│            (旭日5芯片 10 TOPS)              │
└─────────────────┬───────────────────────────┘
                  │
    ┌─────────────┼─────────────┐
    │             │             │
┌───▼───┐    ┌───▼───┐    ┌───▼───┐
│激光雷达│    │RGB相机│    │灭火模块│
│2D SLAM│    │YOLOv5 │    │水泵控制│
└───────┘    └───────┘    └───────┘
    │             │             │
    └─────────────┼─────────────┘
                  │
        ┌─────────▼─────────┐
        │   贝叶斯数据融合   │
        │   MPPI路径规划    │
        └───────────────────┘
```

## 🛠️ 硬件需求

### 必需硬件
- **主控板**：RDK X5 开发板 (旭日5芯片)
- **传感器**：
  - 激光雷达 (测距范围25m)
  - RGB-D相机 (高分辨率)
- **执行器**：
  - 麦克纳姆轮 × 4
  - 水泵 + 功放模块
  - 继电器控制电路

### 推荐配置
- **结构**：双层车型轮式底盘
- **材料**：防火材料外壳
- **电源**：高容量锂电池组
- **通信**：Wi-Fi/5G通信模块

## 🚀 安装部署

### 环境准备

```bash
# 系统要求
- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+
- OpenCV 4.5+
- PyTorch 1.9+
```

### 克隆项目

```bash
git clone https://github.com/username/intelligent-fire-robot.git
cd intelligent-fire-robot
```

### 安装依赖

```bash
# 安装ROS依赖
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full

# 安装Python依赖
pip install -r requirements.txt

# 编译工作空间
catkin_make
source devel/setup.bash
```

### 硬件连接

```bash
# 激光雷达连接
sudo chmod 666 /dev/ttyUSB0

# 相机设备检查
ls /dev/video*

# GPIO权限设置
sudo usermod -a -G gpio $USER
```

## 📖 使用指南

### 快速启动

```bash
# 启动核心节点
roslaunch fire_robot fire_robot.launch

# 启动SLAM建图
roslaunch fire_robot slam_mapping.launch

# 启动火源检测
roslaunch fire_robot fire_detection.launch

# 启动灭火执行
roslaunch fire_robot fire_fighting.launch
```

### 参数配置

```yaml
# config/robot_config.yaml
robot:
  max_speed: 0.5        # 最大速度 m/s
  detection_range: 25   # 检测范围 m
  fire_threshold: 0.8   # 火源识别阈值

camera:
  resolution: "640x480"
  fps: 30
  
lidar:
  range: 25            # 激光雷达范围
  angle_resolution: 1  # 角度分辨率
```

### 操作流程

1. **系统初始化**
   ```bash
   rosrun fire_robot init_system.py
   ```

2. **环境建图**
   ```bash
   rosrun fire_robot slam_node.py
   ```

3. **火源巡检**
   ```bash
   rosrun fire_robot patrol_mode.py
   ```

4. **自动灭火**
   ```bash
   rosrun fire_robot auto_fire_fighting.py
   ```

## 📊 性能指标

| 指标项 | 技术参数 | 测试条件 |
|--------|----------|----------|
| 障碍物检测准确率 | 98.7% | 复杂室内环境 |
| 激光雷达测距范围 | 25m | 标准环境(25℃, 50%湿度) |
| 路径优化长度 | 提升15% | 复杂避障场景 |
| 时空对齐精度 | ±100微秒 | 多传感器融合 |
| 火源检测精度 | ±10cm | 标准光照条件 |
| 灭火有效射程 | 2m | 柱状水模式 |

## 🏭 应用场景

### 室内场所
- 🏠 **家庭住宅**：厨房、客厅初期火灾
- 🏢 **办公楼宇**：办公区、机房火灾
- 🏬 **商业场所**：商场、图书馆火灾

### 工业环境
- ⚗️ **化工厂**：易燃易爆环境灭火
- 🔋 **锂电池厂**：高温电池火灾
- 🏭 **食品加工厂**：粉尘爆炸预防

### 特殊场景
- 🚇 **地下空间**：车库、隧道、管道井
- 🏗️ **建筑工地**：狭窄空间作业
- 🚨 **灾后救援**：复燃检测、人员搜救

## 🔧 开发指南

### 项目结构

```
fire_robot/
├── src/
│   ├── detection/          # 火源检测模块
│   ├── navigation/         # 导航避障模块
│   ├── control/           # 控制执行模块
│   └── fusion/            # 多传感器融合
├── launch/                # ROS启动文件
├── config/                # 配置文件
├── scripts/               # 脚本文件
└── docs/                  # 文档资料
```

### 核心算法

#### 火源检测 (YOLOv5)
```python
class FireDetector:
    def __init__(self, model_path):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', 
                                   path=model_path)
    
    def detect_fire(self, image):
        results = self.model(image)
        return self.parse_results(results)
```

#### SLAM建图
```python
class SLAMMapper:
    def __init__(self):
        self.map = OccupancyGrid()
        self.particles = self.init_particles()
    
    def update_map(self, laser_data, odom_data):
        self.particle_filter(laser_data, odom_data)
        self.update_occupancy_grid()
```

#### 路径规划 (MPPI)
```python
class MPPIPlanner:
    def __init__(self, cost_function):
        self.cost_fn = cost_function
        self.samples = 1000
    
    def plan_path(self, start, goal, obstacles):
        return self.sample_based_planning(start, goal, obstacles)
```

### 自定义开发

#### 添加新传感器
```python
# 1. 创建传感器驱动
class NewSensor:
    def __init__(self, config):
        self.config = config
    
    def read_data(self):
        # 实现数据读取
        pass

# 2. 注册到融合系统
fusion_system.register_sensor(NewSensor(config))
```

#### 扩展灭火模块
```python
class CustomFireModule:
    def __init__(self):
        self.fire_type = "foam"  # 泡沫灭火
    
    def execute_fire_fighting(self, target_pos):
        # 实现自定义灭火逻辑
        pass
```

## 🤝 贡献指南

我们欢迎所有形式的贡献！

### 贡献类型
- 🐛 **Bug修复**：修复代码缺陷
- ✨ **功能增强**：新增功能模块
- 📝 **文档完善**：改进文档质量
- 🔧 **性能优化**：提升系统性能

### 贡献流程
1. Fork 项目仓库
2. 创建功能分支 (`git checkout -b feature/new-feature`)
3. 提交更改 (`git commit -m '添加新功能'`)
4. 推送分支 (`git push origin feature/new-feature`)
5. 创建 Pull Request

### 开发规范
- 遵循 PEP 8 Python编码规范
- 添加单元测试覆盖新功能
- 更新相关文档说明
- 通过CI/CD检查

## 🛡️ 安全提醒

⚠️ **重要安全事项**：
- 本设备仅适用于初期小火扑灭
- 不可用于电气火灾或化学品火灾
- 使用前请确保人员撤离安全区域
- 定期检查设备状态和水源充足
- 遵循当地消防法规要求

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

```
MIT License

Copyright (c) 2024 Intelligent Fire Robot Team

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
```

## 📞 联系我们

- **项目主页**：https://github.com/username/intelligent-fire-robot
- **问题反馈**：https://github.com/username/intelligent-fire-robot/issues
- **技术讨论**：https://github.com/username/intelligent-fire-robot/discussions
- **邮箱联系**：fire.robot.team@example.com

## 🙏 致谢

感谢以下项目和团队的支持：
- [RDK X5](https://developer.horizon.cc/) - 提供强大的硬件平台
- [YOLOv5](https://github.com/ultralytics/yolov5) - 目标检测算法
- [ROS](https://ros.org/) - 机器人操作系统
- [OpenCV](https://opencv.org/) - 计算机视觉库

---

**如果这个项目对您有帮助，请给它一个 ⭐️ 支持！**

> 🔥 让技术守护生命，让创新更有温度
