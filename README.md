# ld06_sim

## Usage
```sh
ros2 launch ld06_sim ld06_sim.launch.py
```

Add some models in Gazebo.

### Results
<img src=assets/gazebo.png width=50%><img src=assets/rviz.png width=50%>


## Notes
ROS1と同様にrobot_state_publisherを用いて，rviz2にld06のモデルを表示したが，うまく表示されない．これは`/robot_description` topicがpublishされていないことが原因だと考えられる．