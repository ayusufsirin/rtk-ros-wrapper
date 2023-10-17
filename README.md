# rtk-ros-wrapper
RTK ROS wrapper and Python NTRIP client

## Setup

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

```bash
pip install -r requirements.txt
```

## Running

```bash
roslaunch rtk_wrapper RTK.launch
```
