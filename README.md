# Map My World

### Build

```bash
$ catkin_make
$ source develop/setup.bash
```

### Launch Gazebo and RViz with Robot

```bash
$ roslaunch my_robot world.launch
```

### Launch RTAB

```bash
$ roslaunch my_robot mapping.launch
```

### Launch Teleop

```bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard
```

### Map database
After driving the robot around the environment twice, the rtab database was created.  It can be downloaded at https://drive.google.com/drive/folders/1NpjdGH9Ii9YlLezm7eTVJUmgCUT0TEir?usp=drive_link

