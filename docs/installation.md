# 安装与可复现构建

基线：Ubuntu 22.04、ROS 2 Humble、Python 3.10。原生 Linux 为实机目标；WSL 的 USB 转发、图形和时序需要独立验证。仓库不包含 ROS、厂商 SDK、控制器固件或已获再分发许可的完整模型。

## 0. 获取代码

项目仓库：[ros2-visual-manipulator](https://github.com/chao142857/ros2-visual-manipulator)。
Repository URL 与公开维护邮箱均已确认；发布状态见 [发布清单](../RELEASE_CHECKLIST.md)。
private staging 阶段克隆需要仓库访问权限。

```bash
git clone https://github.com/chao142857/ros2-visual-manipulator
cd ros2-visual-manipulator
```

后续命令均从这个仓库根运行。软件候选不携带现场标定和未获授权模型，完整九包的
本机构建与公开候选可复现的子集分开验收，见 [本次报告](audit_2026-09-20.md)。

## 1. ROS 与系统依赖

先按 [ROS 2 Humble Ubuntu deb 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 配好官方 apt 源。以下命令是新机器的安装步骤，本次整理没有在用户主机上执行系统安装：

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-moveit \
  ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs \
  ros-humble-urdfdom-py python3-colcon-common-extensions python3-rosdep \
  python3-venv python3-pip python3-pyqt5 python3-serial \
  libgl1 libglib2.0-0
source /opt/ros/humble/setup.bash

# 仅在 /etc/ros/rosdep/sources.list.d 尚未初始化时执行 sudo rosdep init。
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y \
  --skip-keys 'astra_camera python3-open3d'
```

`astra_camera` 没有可用的标准 rosdep 解析项，只有旧 Astra 硬件路径需要它。`python3-open3d` 在下面的虚拟环境中用明确版本安装；这两项跳过不是“依赖全部满足”。若使用源码 RealSense overlay，可额外跳过 `realsense2_camera realsense2_camera_msgs`，并显式配置该 overlay。

只验证公开候选三个基础包时，可以把上述全仓 rosdep 步骤缩小为：

```bash
rosdep install --from-paths src/arm_driver src/gripper_driver src/camera_perception \
  --ignore-src --rosdistro humble -r -y
rosdep check --from-paths src/arm_driver src/gripper_driver src/camera_perception \
  --ignore-src --rosdistro humble
```

两个串口 driver 已正确声明 `python3-serial`，该 rosdep key 在 Jammy 解析为 apt
`python3-serial`；Python 导入名为 `serial`，PyPI 名为 `pyserial`，不要混改。
本机先前 rosdep 失败的原因是 apt 未安装该包；venv 可 import 不等于 rosdep 已满足。
新机让 rosdep 安装它，不跳过此项，也不复用作者的 venv。本轮未安装系统依赖，
完整新机器安装仍为 NOT TESTED。相机/模型相关可选依赖不能靠无条件全仓 rosdep 自动补齐。

## 2. 仓库内 Python 环境

在仓库根目录运行；不要创建嵌套源码工作区或复制旧 build/install。

```bash
export PYTHONNOUSERSITE=1
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements-dev.txt
export ARM_PYTHON_ENV="$PWD/.venv"
.venv/bin/python -m pip check
```

`--system-site-packages` 用于 ROS apt 的 rclpy、cv_bridge 和 PyQt5 ABI。Open3D 为 `open3d-cpu==0.19.0`，NumPy `<2`，SciPy `<1.15`，setuptools `69.5.1` 与 Humble 的 editable install 兼容。旧 AprilTag 使用 `cv2.aruco.ArucoDetector`，系统 OpenCV 4.5.4 不足；公共开发依赖固定 `opencv-contrib-python==4.11.0.86`。不要额外混装多个不同版本的 cv2 wheel。

requirements 是直接依赖约束，不是全平台哈希锁文件。apt 滚动更新与 inherited user-site 包仍可能影响环境；完整冷安装、跨机 ABI 和软件供应链锁定是后续复现任务。`pip check` 发现已有第三方包冲突时，保存输出并定位依赖，不要直接升级系统 Python/ROS 或 NumPy 2。

运行这些 Python 命令前设置 `export PYTHONNOUSERSITE=1`，避免混入用户目录的包。依赖文件中的 cffi 是对 Ubuntu ROS 开发环境继承的 PyNaCl 依赖补齐，只安装在虚拟环境，不更改系统库。

## 3. 相机厂商来源

| 依赖 | 当前证据 | 公开安装策略 |
| --- | --- | --- |
| RealSense ROS | 本地源码版本 4.56.1，commit `53169f3265266d07e88c025b950aa4ccaa03e671` | 优先 Humble deb；精确源码复现从 [上游](https://github.com/IntelRealSense/realsense-ros) 选择并核对该 commit，单独构建 |
| librealsense | 由 RealSense 安装提供 | 跟随兼容的厂商安装步骤；本项目不修改相机固件 |
| Astra ROS | 本地 vendor 副本无 Git 历史，无法追溯精确 commit | 可选，先核实型号、原厂来源/版本，再独立构建；不复制未知来源副本到本仓库 |

使用已有厂商 overlay 时，显式指定真实路径。变量未设置时脚本不会猜测相邻目录：

```bash
# 以下路径按本机填写；使用 apt 版不需要这些变量。
export ARM_REALSENSE_SETUP=/path/to/realsense_ws/install/local_setup.bash
export ARM_ASTRA_SETUP=/path/to/astra_ws/install/local_setup.bash
```

不要把模板路径原样 source。Astra 是可选运行依赖；全仓 rosdep 检查可能仍报告其缺失。三个基础包 CI 不启动相机，也不需要 Astra SDK。

## 4. Build 与 test

从没有 source 旧应用 overlay 的新终端执行；若使用 venv/vendor，重新设置上面的变量：

```bash
bash scripts/build.sh
source scripts/env.sh
bash scripts/test.sh functional
bash scripts/test.sh all
python3 scripts/check_structure.py
```

脚本以仓库位置为准，默认写入 `build-release/`、`install-release/`、`log-release/`；不会隐式加载旧 `install/` 或 `install-tabletop/`。`env.sh` 必须 source 才能保留环境。支持 `--help`，build 接受 colcon 的选包参数。

功能测试与样式检查分开；`all` 中的历史 lint 失败不会被当作功能通过。测试固定使用域 194、localhost、Qt offscreen，不启动真实相机或控制器。CMake/pytest 结果可能具有父子汇总，不应把 colcon 总数当作独立 pytest case 数。

公开候选因模型/标定被排除，只承诺以下明确范围：

```bash
# 在未 source 应用 overlay 的新终端运行。
bash scripts/ci.sh
```

它构建并测试 arm_driver、gripper_driver、camera_perception 三包，并离线解析 launch 权限传递；静态检查标记缺失资源为 pending。若希望运行纯几何 demo，需要构建 `tabletop_grasp_interfaces tabletop_grasp` 并加载 overlay，随后 `python scripts/demo_geometry.py`。这不需要真实模型，但完整 MoveIt 演示需要 [合法模型资源](../models/README.md)。

在干净的新终端、完成上述依赖安装后，纯几何演示的完整构建步骤为：

```bash
export ARM_PYTHON_ENV="$PWD/.venv"
bash scripts/build.sh --packages-select tabletop_grasp_interfaces tabletop_grasp \
  --packages-ignore arm_description arm_console wrist_camera_bringup
source scripts/env.sh
python3 scripts/demo_geometry.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/tabletop_grasp/test/test_geometry_grasps.py \
  src/tabletop_grasp/test/test_inverse_brown_rgbd.py
```

这里显式忽略三个未构建的运行期集成包，限定为纯软件几何演示；否则 colcon 会要求它们的安装环境。此命令没有准备完整机器人或相机 launch 所需的包，不能当作完整 bringup 构建。

反投影测试中的 `pyrealsense2` 是可选 SDK 数学对照依赖，基础开发依赖不强制安装。未安装时三个 SDK 对照用例会跳过，其他数学测试仍执行；SDK/驱动的实机安装与相机精度验证是另一层检查。

本地已有模型的完整九包结果、公开候选结果与环境限制见 [RELEASE_READINESS.md](RELEASE_READINESS.md)。
