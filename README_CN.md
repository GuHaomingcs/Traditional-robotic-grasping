# Traditional-robotic-grasping

## 项目概述

本项目展示了传统机器人抓取和操作方法，使用了：
- **Franka Panda 7自由度机械臂**配备2指夹持器
- **逆运动学**求解器，实现精确的末端执行器定位
- **PID**，提供稳定的关节空间控制
- **任务序列**，固定顺序任务（你可以自己修改）
- **MuJoCo仿真**

执行完整的抓取-放置操作：从一个位置抓取箱子，旋转它，然后将其放置到不同位置并进行90度旋转。

## 环境要求

```
pip install -r requirements.txt
```

## 项目结构

```
├── README.md
├── README_CN.md
├── requirements.txt
├── mani.py              # 入口
├── ik.py               # 逆运动学求解
├── src/                # 核心实现模块
│   ├── ik_module.py    # 逆运动学求解器
│   ├── PID.py          # PID控制器类
│   ├── mujoco_parser.py 
│   └── util.py         
└── asset/              # 资源
    ├── panda/          # Franka Panda机器人模型
    │   ├── franka_panda_ghm.xml
    │   ├── franka_panda_w_objs.xml
    │   ├── meshes/     
    │   └── assets/     
    └── common_arena/   
        └── simple_plane.xml
```

## 资源文件

`asset/` 目录包含所有必要的仿真资源：

**更多资源文件可在 https://github.com/sjchoi86/yet-another-mujoco-tutorial 获取**

### 机器人模型 (`asset/panda/`)
- **franka_panda_ghm.xml**: 操作任务的主要机器人配置文件
- **franka_panda_w_objs.xml**: 包含不同物体设置的替代配置
- **meshes/**: 用于机器人组件视觉和碰撞表示的STL/OBJ网格文件
- **assets/**: 模块化XML组件，包括：
  - 机器人本体定义
  - 执行器配置
  - 物体定义
  - 材料属性

### 环境 (`asset/common_arena/`)
- **simple_plane.xml**: 用于操作任务的基础地平面环境

### 核心模块 (`src/`)

**代码基于 https://github.com/sjchoi86/yet-another-mujoco-tutorial**

- **`ik_module.py`**: 使用基于雅可比方法的数值逆运动学求解器
- **`PID.py`**: PID控制器
- **`mujoco_parser.py`**
- **`util.py`**

### 主要脚本

- **`mani.py`**: 程序入口
- **`ik.py`**: 逆运动学

## 方法论

### 1. 逆运动学求解器

项目实现了使用雅可比伪逆方法的数值逆运动学求解器：

```python
# 基于雅可比的逆运动学求解器
J_p, J_R, J_full = get_J_body(env.model, env.data, body_name, rev_joint_idxs=env.rev_joint_idxs)
p_err = (p_trgt - p_curr)  # 位置误差
R_err = np.linalg.solve(R_curr, R_trgt)  # 旋转误差
w_err = R_curr @ r2w(R_err)  # 角速度误差
err = np.concatenate((p_err, w_err))
dq = np.linalg.solve(a=(J.T@J) + eps*np.eye(J.shape[1]), b=J.T@err)
```

## 使用方法

1. **初始化环境：**
   ```python
   python mani.py
   ```

2. **仿真：**
   - 在MuJoCo中加载Franka Panda机器人
   - 计算所有路径点的逆运动学解
   - 执行完整的操作序列
   - 显示实时可视化

3. **控制参数可在`mani.py`中调整：**
   - 不同控制特性的PID增益
   - 任务序列时序和路径点

## 警告

**如果使用新的资源文件，请注意在`ik.py`中，`pre_grasp_q`是硬编码的预抓取姿态。**
