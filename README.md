# 基于RDK X5的机器视觉与自主避障智能灭火车

## 项目概述

本项目开发了一款基于RDK X5平台的智能灭火机器人，融合了机器视觉、自主避障和智能灭火功能。该机器人能够在复杂环境中自主导航，准确识别火源并执行灭火任务，为火灾救援提供创新性、智能化的解决方案。

## 核心特性

### 🔥 火源识别
- 基于YOLOv5深度学习模型的火源检测
- RGB相机实时视觉识别
- 复杂光照条件下的高精度识别

### 🚗 自主导航
- 激光雷达环境感知
- 基于SLAM的2D地图构建
- MPPI算法优化路径规划
- 麦克纳姆轮全向运动控制

### 💧 智能灭火
- 高压水泵灭火装置
- 模块化设计，支持多种灭火模式
- GPIO控制的自动灭火系统
- 精确的水流喷射控制

## 系统架构

```
智能灭火车系统
├── 硬件系统
│   ├── RDK X5 主控板
│   ├── STM32F407 运动控制
│   ├── 激光雷达
│   ├── RGB相机
│   ├── 麦克纳姆轮驱动系统
│   └── 灭火装置
└── 软件系统
    ├── ROS2 框架
    ├── YOLOv5 视觉识别
    ├── SLAM建图
    ├── MPPI路径规划
    └── GPIO控制模块
```

## 技术规格

| 指标项 | 技术参数 | 测试条件 |
|--------|----------|----------|
| 障碍物检测准确率 | 98.7% | 复杂环境 |
| 激光雷达测距范围 | 25m | 标准环境 |
| 路径优化率 | 15% | 复杂环境 |
| 视觉检测精度 | ±10cm | 标准环境 |
| 灭火装置射程 | 2m | 标准环境 |

## 应用场景

### 工业环境
- 化工仓储火灾应急处理
- 锂电池工厂火灾防护
- 食品加工厂粉尘爆炸防护

### 室内环境
- 家庭、办公楼初期火灾响应
- 商场、图书馆火灾防护
- 地下车库、隧道等狭窄空间

## 安装与部署

### 硬件要求
- RDK X5开发板
- STM32F407控制板
- 激光雷达传感器
- RGB相机
- 麦克纳姆轮驱动系统
- 水泵灭火装置

### 软件环境
```bash
# 安装ROS2
sudo apt update
sudo apt install ros-humble-desktop

# 克隆项目
git clone [项目地址]
cd intelligent-fire-fighting-robot

# 安装依赖
pip install -r requirements.txt
```

### YOLOv5模型部署
```bash
# 克隆YOLOv5仓库
git clone https://github.com/ultralytics/yolov5
cd yolov5

# 准备训练数据集
python train.py --data fire_dataset.yaml --weights yolov5s.pt --epochs 100
```

## 使用说明

### 启动系统
```bash
# 启动ROS2节点
ros2 launch fire_robot fire_robot.launch.py

# 启动SLAM建图
ros2 run slam_toolbox sync_slam_toolbox_node

# 启动火源检测
ros2 run fire_detection yolo_detector
```

### 控制命令
```bash
# 开始自主巡航
ros2 topic pub /robot_mode std_msgs/String "data: 'auto_patrol'"

# 手动控制
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# 执行灭火
ros2 topic pub /fire_extinguish std_msgs/Bool "data: true"
```

## 核心算法

### 火源检测算法
基于YOLOv5的实时火源检测：
- 多尺度训练增强检测精度
- 自适应锚框计算
- Mosaic数据增强

### 路径规划算法
MPPI（Model Predictive Path Integral）算法：
- 高斯分布随机采样
- 代价函数优化
- 动态障碍物避让

### 传感器融合
贝叶斯方法多传感器融合：
- 激光雷达点云数据
- RGB相机视觉信息
- 递归置信度更新

## 项目结构

```
intelligent-fire-fighting-robot/
├── src/
│   ├── fire_detection/          # 火源检测模块
│   ├── slam_mapping/            # SLAM建图模块
│   ├── path_planning/           # 路径规划模块
│   ├── motion_control/          # 运动控制模块
│   └── fire_extinguish/         # 灭火控制模块
├── launch/                      # 启动文件
├── config/                      # 配置文件
├── models/                      # 训练模型
├── datasets/                    # 数据集
└── docs/                       # 文档
```

## 创新点

1. **多模态感知融合**：创新性地结合激光雷达与RGB相机，通过贝叶斯方法实现数据融合
2. **动态地图构建**：基于递归贝叶斯方法动态更新栅格地图
3. **低成本灭火方案**：利用功放和水泵实现轻量化、低功耗灭火
4. **模块化设计**：标准化快拆接口，支持多种功能模块切换

## 性能展示

### 火源检测效果
- 实时火源识别准确率达到98.7%
- 支持复杂光照条件下的稳定检测
- 动态火焰特征分析降低误检率

### 自主导航能力
- 25米激光雷达测距范围
- 厘米级定位精度
- 15%路径优化率提升

### 灭火效果
- 2米有效射程
- 支持雾状和柱状水流切换
- 模块化灭火装置设计

## 扩展功能

### 传感器增强
- 红外摄像头/热成像仪
- 气体传感器（CO、VOCs）
- IMU防倾覆系统

### 智能算法升级
- D* Lite动态路径规划
- 深度学习火势预测
- 多机协作系统

### 执行机构优化
- 干粉喷射模块
- 高压水雾系统
- 灭火弹发射装置

## 贡献指南

1. Fork项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启Pull Request

## 许可证

本项目采用MIT许可证，详情请参阅 [LICENSE](LICENSE) 文件。

## 联系我们

- 项目主页：[项目链接]
- 问题反馈：[Issues页面]
- 技术支持：[联系邮箱]

## 致谢

感谢所有为本项目做出贡献的开发者和研究人员。特别感谢RDK X5平台、ROS2社区以及YOLOv5开源项目的支持。

---

**注意**：本项目仅供学习和研究使用，在实际应用中请确保符合相关安全规范和法律法规。
