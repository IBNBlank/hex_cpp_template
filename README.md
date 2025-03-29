# **hex_cpp_template**

## **Overview**

This **hex_cpp_template** repository provides a unified C++ template for ROS1 and ROS2.

### **License**

This project is licensed under the terms of the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

### **Maintainer**

**[Dong Zhaorui](https://github.com/IBNBlank)**

### **Supported Platform**

- [x] **x64**
- [ ] **Jetson Orin Nano**
- [x] **Jetson Orin NX**
- [ ] **Jetson AGX Orin**
- [ ] **Horizon RDK X5**

### **Supported ROS Version**

- [x] **ROS Noetic**
- [x] **ROS Humble**

---

## **Public APIs**

### **Publish**

| Topic         | Msg Type            | Description   |
| ------------- | ------------------- | ------------- |
| `/string_out` | `std_msgs/String`   | String output |
| `/odom`       | `nav_msgs/Odometry` | Odometry test |

### **Subscribe**

| Topic        | Msg Type          | Description  |
| ------------ | ----------------- | ------------ |
| `/string_in` | `std_msgs/String` | String input |

### **Parameters**

| Name                  | Data Type             | Description                              |
| --------------------- | --------------------- | ---------------------------------------- |
| `string_prefix`       | `std::string`         | String prefix                            |
| `odom_frame`          | `std::string`         | Odom frame                               |
| `odom_child_frame`    | `std::string`         | Odom child frame                         |
| `odom_vel_lin`        | `std::vector<double>` | Odom linear velocity                     |
| `odom_vel_ang`        | `std::vector<double>` | Odom angular velocity                    |
| `odom_child_in_frame` | `std::vector<double>` | Transform that child frame in odom frame |

---

## **Getting Started**

### **Dependencies**

- **[hex_cpp_utils](https://github.com/hexfellow/hex_cpp_utils)**
- **ROS Noetic** or **ROS Humble**

### **Install**

1. Create a workspace `catkin_ws` and get into the `src`.

   ```shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Clone this repo.

   ```shell
   git clone git@github.com:hexfellow/hex_cpp_template.git
   ```

3. Go to `catkin_ws` directory and build the repo.

   ```shell
   cd ../

   # For ROS1
   catkin_make -DCMAKE_BUILD_TYPE=Release

   # For ROS2
   colcon build -DCMAKE_BUILD_TYPE=Release
   ```

4. Source the `setup.bash` and run the test blow

   ```shell
   # For ROS1
   source devel/setup.bash --extend

   # For ROS2
   source install/setup.bash --extend
   ```

### **Usage**

- **ROS1**

  1. Open a terminal and run the following command:

     ```shell
     roslaunch hex_cpp_template hex_cpp_template.launch
     ```

  2. Open a new terminal and run the following command:

     ```shell
     rostopic pub /string_in std_msgs/String "data: 'Hello, World'" -r 10
     ```

  3. Open a new terminal and run the following command:

     ```shell
     rostopic echo /string_out
     ```

     You should see output like this:

     ```shell
     data: "hex_cpp_template: Hello, World;"
     ---
     ```

  4. Open a new terminal and run the following command:

     ```shell
     rostopic echo /odom
     ```

     You should see output like this:

     ```shell
     header:
       seq: 6840
       stamp:
         secs: 1743256841
         nsecs: 627260024
       frame_id: "odom"
     child_frame_id: "base_link"
     pose:
       pose:
         position:
           x: 1.01
           y: 0.55
           z: 2.02
         orientation:
           x: 0.0
           y: 0.0
           z: 0.0
           w: 1.0
       covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
     twist:
       twist:
         linear:
           x: 1.0
           y: 0.5
           z: 2.0
         angular:
           x: 0.1
           y: 0.05
           z: 0.0
       covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
     ---
     ```

- **ROS2**

  1. Open a terminal and run the following command:

     ```shell
     ros2 launch hex_cpp_template hex_cpp_template.launch.py
     ```

  2. Open a new terminal and run the following command:

     ```shell
     ros2 topic pub /string_in std_msgs/String "data: 'Hello, World'" -r 10
     ```

  3. Open a new terminal and run the following command:

     ```shell
     ros2 topic echo /string_out
     ```

     You should see output like this:

     ```shell
     data: "hex_cpp_template: Hello, World;"
     ---
     ```

  4. Open a new terminal and run the following command:

     ```shell
     ros2 topic echo /odom
     ```

     You should see output like this:

     ```shell
     header:
       seq: 6840
       stamp:
         secs: 1743256841
         nsecs: 627260024
       frame_id: "odom"
     child_frame_id: "base_link"
     pose:
       pose:
         position:
           x: 1.01
           y: 0.55
           z: 2.02
         orientation:
           x: 0.0
           y: 0.0
           z: 0.0
           w: 1.0
       covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
     twist:
       twist:
         linear:
           x: 1.0
           y: 0.5
           z: 2.0
         angular:
           x: 0.1
           y: 0.05
           z: 0.0
       covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
     ---
     ```

