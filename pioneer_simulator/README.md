# pioneer_simulator

#### 介绍
双轮移动机器人pioneer在Gazebo仿真环境中的实现，可用于测试导航算法。配备30m@180°单线激光雷达，定位方法默认使用amcl

先已增加双车模式，分别是双车室内模式和双车室外模式。双车模式下一辆是leader车可以由键盘或者手柄控制，另一辆是自主车，可用于算法验证。

自主车辆配有单线激光雷达、相机。

双车室外模式用于测试f1f2比赛。


#### 安装教程

1. 使用此packet前，需要先安装以下packet：amcl,p2os-urdf,joy,map-server,gazebo-ros。
2. 安装方法为"sudo apt install ros-kinetic- **packet_name** "。
3. (无手柄情况)键盘控制安装方法为`sudo apt-get install ros-kinetic-teleop-twist-keyboard`。
可以用以下方式双开节点。
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py __name:=my_node2
rosrun teleop_twist_keyboard teleop_twist_keyboard.py __name:=my_node1 cmd_vel:=/Leader/cmd_vel
```
4.  [gazebo模型下载1](https://gitee.com/csc105/dashboard/attach_files/338158/download) [gazebo模型下载2](https://gitee.com/csc105/dashboard/attach_files/338157/download) [gazebo模型下载0](https://gitee.com/csc105/dashboard/attach_files/338156/download)。下载后的三个文件放在同一目录,通过`cat models.zip0* > models.zip`完成合并,之后解压到这个目录`~/.gazebo/models`

#### 双车室外模式使用说明

执行`roslaunch pioneer_simulator pioneer_double_car_outdoor.launch`

leader车辆初始位置为（0,0），自主车辆（-3,0）

Leader车的速度控制topic为"/Leader/cmd_vel"；自主跟随车的速度控制topic为"/cmd_vel"。

自主跟随车会发布单线激光点云的pointCloud2（10Hz）和图像数据（20Hz）。

#### 注意事项

第一次开启gazebo，会黑屏，等待几分钟就行，看电脑性能。之后打开会比较快。

双车模式中，leader车辆的碰撞模型经过修改，实际高度有1米，但是视觉模型上方是透明的。为的是让跟随车辆在加减速发生姿态变化时依旧能检测到前方车辆。

建议自行查看TF，rviz的TF报warinng不要紧。

室外的地图下载地址
[pcd](https://gitee.com/csc105/dashboard/attach_files/330458/download)
[pgm](https://gitee.com/csc105/dashboard/attach_files/330457/download)
[yaml](https://gitee.com/csc105/dashboard/attach_files/330456/download)



