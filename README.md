# pcd点云图转化为pgm栅格地图

## 编译
```
colcon build --symlink-install
```
## 运行
```
ros2 launch pcdtogrid pcdtogrid.launch.py
```
## 保存栅格地图
```
ros2 run nav2_map_server map_saver_cli -f YOUR_MAP_NAME
```
## 参数文件：
* config/params.yaml:
```
ptgm:
  ros__parameters:
    file_name: "/home/hy/wust_nav/src/bringup_nav/pcd/simulation/rmul_2024.pcd" #pcd文件路径
    min_z: -1.5 #直通滤波最小高度（默认z轴）
    max_z: 0.5  #直通滤波最大高度（默认z轴）
    radius: 0.5 #半径滤波探索半径
    min_pts: 10 #半径滤波最小点数
    use_sim_time: false #是否使用sim_time
```
