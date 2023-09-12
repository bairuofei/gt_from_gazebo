# gt_from_gazebo
A ROS node that gets ground truth pose of the robot in the Gazebo simulator, and publish the robot GT trajectory.

The node subscribes `/gazebo/model_states` topic published by Gazebo, and publishes `/path_gt`.


## Usage
```
cd catkin_ws/src
git clone git@github.com:bairuofei/gt_from_gazebo.git
cd ..
catkin_make
source ./devel/setup.bash

rosrun gt_from_gazebo gt_gazebo
```

Set the parameter `model_name` as `robot` in a terminal:
```
rosparam set model_name robot
```

Or you can launch the node `gt_gazebo` in a launch file:
```
<node pkg="gt_from_gazebo" type="gt_gazebo" name="gt_gazebo" output="screen">
    <param name="model_name" type="str" value="robot"/>
</node>
```
