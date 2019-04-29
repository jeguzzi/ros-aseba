This [ROS] stack is a bridge to access an [Aseba] network from [ROS].
For compilation instructions, see the README.md in the `asebaros` directory.

[Aseba]: http://aseba.wikidot.com
[ROS]: http://www.ros.org

This branch allows to connect multiple Aseba node to the ROS network.

Compilation
-----------

Clone the repository in your catkin [tools] workspace:

```bash
cd <YOUR_CATKIN_WS>/src
git --recursive -b multi  clone git://github.com/jeguzzi/ros-aseba.git
catkin config --blacklist thymio_navigation ethzasl_aseba
```

Apply a patch to only build the necessary components of Aseba:

```
cd <YOUR_CATKIN_WS>/src/ros-aseba/aseba/upstream
git apply ../aseba.patch
```

Build:

```
catkin build
```
