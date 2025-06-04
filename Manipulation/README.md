# Traditional-robotic-grasping

## Overview

This project demonstrates a traditional approach to robotic grasping and manipulation using:
- **Franka Panda 7-DOF robotic arm** with 2-finger gripper
- **Inverse kinematics** solver for precise end-effector positioning
- **PID** for stable joint-space control
- **Task sequence** for fixed sequential tasks(u can modify it yourself in mani.py .)
- **MuJoCo simulation**

The system performs a complete pick-and-place operation: grasping a box from one location, rotating it, and placing it at a different position with 90-degree rotation.

## Requirements

```
pip install -r requirements.txt
```

## Project Structure

```
├── README.md
├── README_CN.md
├── requirements.txt
├── mani.py              # Entry point
├── ik.py               # Inverse kinematics solver
├── src/                # Core implementation modules
│   ├── ik_module.py    # IK solver
│   ├── PID.py          # PID controller class
│   ├── mujoco_parser.py 
│   └── util.py         
└── asset/              # Assets
    ├── panda/          # Franka Panda robot models
    │   ├── franka_panda_ghm.xml
    │   ├── franka_panda_w_objs.xml
    │   ├── meshes/     
    │   └── assets/     
    └── common_arena/   
        └── simple_plane.xml
```

## Assets

The `asset/` directory contains all necessary simulation resources:

**Here will be more assets at https://github.com/sjchoi86/yet-another-mujoco-tutorial**

### Robot Models (`asset/panda/`)
- **franka_panda_ghm.xml**: Main robot configuration file for the manipulation task
- **franka_panda_w_objs.xml**: Alternative configuration with different object setups
- **meshes/**: STL/OBJ mesh files for visual and collision representation of robot components
- **assets/**: Modular XML components including:
  - Robot body definitions
  - Actuator configurations
  - Object definitions
  - Material properties

### Environment (`asset/common_arena/`)
- **simple_plane.xml**: Basic ground plane environment for manipulation tasks

### Core Modules (`src/`)

**Code fully based on https://github.com/sjchoi86/yet-another-mujoco-tutorial**

- **`ik_module.py`**: Numerical inverse kinematics solver using Jacobian-based methods
- **`PID.py`**: PID controller
- **`mujoco_parser.py`**
- **`util.py`**

### Main Scripts

- **`mani.py`**: Program entry point
- **`ik.py`**: Inverse kinematics

## Methodology

### 1. Inverse Kinematics Solver

The project implements a numerical IK solver using the Jacobian pseudo-inverse method:

```python
# Jacobian-based IK solver
J_p, J_R, J_full = get_J_body(env.model, env.data, body_name, rev_joint_idxs=env.rev_joint_idxs)
p_err = (p_trgt - p_curr)  # Position error
R_err = np.linalg.solve(R_curr, R_trgt)  # Rotation error
w_err = R_curr @ r2w(R_err)  # Angular velocity error
err = np.concatenate((p_err, w_err))
dq = np.linalg.solve(a=(J.T@J) + eps*np.eye(J.shape[1]), b=J.T@err)
```

## Usage

1. **Initialize the environment:**
   ```python
   python mani.py
   ```

2. **The simulation:**
   - Load the Franka Panda robot in MuJoCo
   - Calculate IK solutions for all waypoints
   - Execute the complete manipulation sequence
   - Display real-time visualization

3. **Control parameters can be adjusted in `mani.py`:**
   - PID gains for different control characteristics
   - Task sequence timing and waypoints

## Warning

**If you are using new assets, please note that in `ik.py`, the `pre_grasp_q` is hardcoded with the pre-grasp pose.**