# rviz_plugin_examples
Examples for rviz plugin for `melodic` environment.

# Build
```bash
$ mkdir <catkin_ws>/src
$ cd <catkin_ws>/src
$ git clone -b melodic-devel https://github.com/masakifujiwara1/rviz_plugin_goalradius.git
$ cd <catkin_ws>
$ rosdep install -i -y -r --from-paths src
$ catkin build
$ source devel/setup.bash
```

$ Execute
Do not forget to set path as below.
```bash
$ cd <catkin_ws>
$ source devel/setup.bash
```

