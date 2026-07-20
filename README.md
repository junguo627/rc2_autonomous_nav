# RC2026 navigation Project

 湖南农业大学26赛季视觉项目主仓库，该项目使用fastlio2进行里程计定点导航，同时配备梅林路径决策


## 一、项目结构

```
.
│
├── main.py (代码主文件)
│
├── livox_ros_driver2 (雷达驱动包)
│
├── Livox-SDK2 (雷达驱动文件)
│
├── FASTLIO2_ROS2（lio算法包）
│
├──path_planner.py (决策算法)
```

## 二、环境

### 1. 基础
- Ubuntu 22.04
- ROS2 Humble
- mid360驱动

### 2. 配置
参考https://github.com/liangheming/FASTLIO2_ROS2

## 三、编译与运行

```bash
# 编译
cd ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
# 运行
source install/setup.bash
红方
python3 main.py --red
蓝方
python3 main.py --blue
```
## 致谢
- [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2)FASTLIO2_ROS2是本项目的基础
