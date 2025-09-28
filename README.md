# ROS_Turtel_function
# Assignment 1 – ROS 2 Research Track I

This package demonstrates a simple ROS 2 application where:
1. A second turtle (`turtle2`) is spawned in the **turtlesim** environment.
2. The user can control either `turtle1` or `turtle2` via a textual interface.
3. The node publishes a **custom message** containing the name and pose (x, y, theta) of `turtle2`.

---

## Package Overview

**Package name:** `assignment1_rt`  
**ROS 2 version:** Humble (compatible with Foxy/Galactic)  
**Main node:** `ui_node.py`  
**Custom message:** `msg/TurtleInfo.msg`

### Custom Message Definition
```text
string name
float32 x
float32 y
float32 theta
