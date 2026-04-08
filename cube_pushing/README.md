# Mobile Robotics Assessment 1 – Python Package

This package provides solutions to the cube pushing problem across:
- Level 1  
- Level 2  
- Experimental Level 3 (incomplete)

---

## Prerequisites

Ensure the following are set up before running the package:

- A configured repository derived from the module template repository:  
  https://github.com/UoL-SoCS/cmp3103-ws  
- Ubuntu Operating System or WSL  
- ROS 2 installed and configured  
- This package placed inside your ROS 2 workspace (`src/` folder)

---

## Installation and Build

Install dependencies, build the package, and source the workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cube_pushing
source install/setup.bash
```

## Running the Tasks

# Level 1

```bash
ros2 launch cube_pushing task_1.launch.py
```

# Level 2

```bash
ros2 launch cube_pushing task_2.launch.py
```

# Level 3

```bash
ros2 launch cube_pushing task_3.launch.py
```

