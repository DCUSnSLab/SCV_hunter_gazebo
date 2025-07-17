```bash
mkdir -p ~/hunter_gazebo_ws/src
cd ~/hunter_gazebo_ws/src

git clone --recursive <repository-url> scv_robot_gazebo
```

```bash
cd scv_robot_gazebo
git submodule update --init --recursive
```

```bash
cd ~/hunter_gazebo_ws
rosdep install --from-paths src --ignore-src -r -y
```

```bash
colcon build
source install/setup.bash
```

```bash
ros2 launch scv_robot_gazebo hunter_test.launch.py
```