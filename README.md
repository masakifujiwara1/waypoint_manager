# waypoint_manager

<img width="736" alt="Screen Shot 2022-01-19 at 18 19 39" src="https://user-images.githubusercontent.com/18626482/150101087-60d64f9e-ca0b-46d5-82cf-49a279f38a61.png">

## Releases
https://github.com/masakifujiwara1/waypoint_manager/releases

## Usage

```shell
roslaunch waypoint_server waypoint_server.launch
```
or 
```shell
roslaunch waypoint_server waypoint_server.launch config_file:=<your confg>
```
### when using options
```
roslaunch waypoint_server waypoint_server_options.launch config_file:=<your confg> use_check_robot_moving:=<true or false> use_waypoint_reconfigure:=<true or false>
```

## Options
### check_robot_moving
https://github.com/masakifujiwara1/waypoint_manager/releases/tag/v2.0.1

### waypoint_reconfigure
https://github.com/masakifujiwara1/waypoint_manager/releases/tag/v2.1.1-beta

## TODO

+ Add an edit panel for waypoint properties
+ Support actionlib
+ Support ros2
+ Refactor code
