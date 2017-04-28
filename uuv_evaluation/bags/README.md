Default folder for rosbags generated with the demo launch files in uuv_gazebo.
To record the topics from the demo launch files included in `uuv_gazebo/launch/controller_demos`, run the launch file with the `record` flag set to `true` as in the example:

```
roslaunch uuv_gazebo start_pid_demo.launch record:=true
```

You can generate plots and KPIs by running:

```
roslaunch uuv_evaluation evaluate_trajectory.launch
```
