# ld06_sim

## Usage
```sh
ros2 launch ld06_sim ld06_sim.launch.py
```

Add some models in Gazebo.

### Results
#### gazebo
![](assets/gazebo.png)

#### rviz
![](assets/rviz.png)

## TODO
- Gazeboに生成するモデルへのテクスチャの設定
- LiDARによる計測対象の障害物をlaunch時に自動でGazebo環境中に生成

## References
- [LD06 datasheets](https://www.inno-maker.com/wp-content/uploads/2020/11/LDROBOT_LD06_Datasheet.pdf)
- [LD06 LiDAR 3DCAD model](https://grabcad.com/library/ldrobot-ld06-360-lidar-module-raspberry-pi-mounting-bracket-1)