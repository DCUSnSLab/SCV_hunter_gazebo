```bash
mkdir -p ~/hunter_gazebo_ws/src
cd ~/hunter_gazebo_ws/src

git clone --recursive https://github.com/DCUSnSLab/SCV_hunter_gazebo scv_robot_gazebo

cd ~/hunter_gazebo_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### 2. 실행

```bash
ros2 launch scv_robot_gazebo hunter_test.launch.py
```

### 3. 키보드 제어

```bash
source ~/hunter_gazebo_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ackermann_like_controller/cmd_vel
```

**조작키:**
- `i` - 전진 | `k` - 정지 | `,` - 후진
- `j` - 좌회전 | `l` - 우회전
- `q/z` - 속도 조절
