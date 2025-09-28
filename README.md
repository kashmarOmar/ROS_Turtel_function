# ROS / ROS 2 – Research Track Assignments

This repository contains the solutions for two exercises from **Research Track I**,  
implemented using **ROS 2 Humble** and the `turtlesim` simulator.

---

## 📚 Assignment Descriptions (from the Course)

### ✅ Assignment 1  – ROS (or ROS 2)

> **Goal:** Create a new package named `assignment1_rt` with one or two nodes that:
> * **Spawn** a new turtle in the environment: `turtle2`.
> * Implement a **simple textual UI** where the user can:
>   * Select which robot to control (`turtle1` or `turtle2`).
>   * Set the velocity (linear, angular, or both).
> * Send the command for **1 second**, then stop the robot and prompt again.
> * **Publish a custom message** containing:
>   * A string `"turtle2"`.
>   * The current position of `turtle2`.

---

### ✅ Assignment 2 – ROS 2 Only

> **Goal:** Repeat the first exercise in ROS 2 with these differences:
> * **Call the `/kill` service** to remove `turtle1` and **`/spawn`** to add a new turtle.
> * Create a **publisher** and **subscriber** for the new turtle (`cmd_vel` and `pose` topics).
> * Check the **message types** and add the needed dependencies.
> * Publish a certain velocity in the subscriber **callback**.
> * Provide a **launch file** to start both `turtlesim` and your controller.

---

## 🐢 Implementation Overview

### Assignment 1 (`assignment1_rt`)

* **Main Node:** `ui_node.py`
* Spawns `turtle2`, lets the user choose which turtle to control and which velocity to send.
* Publishes a **custom message** `TurtleInfo.msg`:
  ```text
  string name
  float32 x
  float32 y
  float32 theta
