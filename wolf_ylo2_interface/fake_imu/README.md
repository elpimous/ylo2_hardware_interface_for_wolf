# fake_imu
publish fake imu messages where Z=gravity in ros

## how to use

```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone https://github.com/SimoSilvere/fake_imu.git
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
roslaunch fake_imu fake_imu.launch
```
