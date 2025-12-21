# ROS Competition

## Usage

1. Install dependencies

    ```bash
    cd ~/template_ws    # your workspace folder
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble
    ```

2. Build the project

    ```bash
    colcon build
    ```

3. Source the workspace

    ```bash
    . ~/template_ws/install/setup.bash
    ```

4. Запуск карты и робота по соревам

    ```bash
    ros2 launch robot_bringup autorace_2025.launch.py
    ```

5. Запуск алгоритма для сорев

    ```bash
    ros2 launch autorace_core_ros_sos autorace_core.launch.py
    ```