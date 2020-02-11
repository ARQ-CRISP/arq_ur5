# ARQ UR5 Package

ROS programs to use UR5 in ARQ lab.

## Moveit and RViz

First, the computer should be [connected to the robot](docs/connecting.md).

After ensuring connection, an appropriate UR5 driver should be launched (For example, ```roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=177.22.22.11```). Then, MoveIt controller for UR5 with RViz GUI can be started as follows:

```bash
roslaunch arq_ur5 moveit_rviz
```

If Allegro Hand is mounted, the corresponding argument should be set:

```bash
roslaunch arq_ur5 moveit_rviz allegro:=true
```

Collision objects aren't created by default. Be careful before executing plans!

## Adding collision objects

Simply run the ```load_scene.launch``` file to add predefined collision objects into the scene.
```bash
roslaunch arq_ur5 load_scene.launch scene:=my_scene
```

Here, _my\_scene_ is the name of the **.scene** file in the _arq\_ur5/scenes_ folder. If you want to use the default scene file for the bimanual platform, you can omit the scene parameter as follows:
```bash
roslaunch arq_ur5 load_scene.launch
```

This will load the _arq\_ur5/scenes/bimanual.scene_ by default.

## Cartesian movement

You can start the Cartesian controller node using:
```bash
rosrun arq_ur5 cartesian_move_node
```

Cartesian node prompts the user for desired position changes. It finds a straight Cartesian path to the desired position and executes it, if possible.

## Running with Allegro hand

You need the [*allegro-hand-ros (experimental branch)*](https://github.com/gokhansolak/allegro-hand-ros/tree/experimental), [allegro_hand_kdl](https://github.com/ARQ-CRISP/allegro_hand_kdl) and [ur5_allegro_moveit](https://github.com/ARQ-CRISP/ur5_allegro_moveit) packages. The following set of commands will connect to the real robot and start the drivers, launch MoveIt and RViz and add the collision objects to scene. It is your responsibility to confirm the configuration of collision objects, because the default scene may be outdated. These commands should be run in separate terminal instances:

```bash
roslaunch ur5_allegro_moveit ur5_allegro_bringup.launch robot_ip:=177.22.22.11
roslaunch arq_ur5 moveit_rviz.launch allegro:=true
roslaunch arq_ur5 load_scene.launch scene:=bimanual_computers
```

Then, you can run your own moveit control program. MoveIt move group names for different parts of the system are "ur5_arm", "allegro_finger0", "allegro_finger1", "allegro_finger2", "allegro_finger3".

For safety, it is recommended to decrease the maximum velocity parameter of MoveIt. In C++:
```c++
move_group->setMaxVelocityScalingFactor(0.05);
```
In Python:
```python
move_group.set_max_velocity_scaling_factor(0.05);
```

**Optionally**, you can load the robot without the optoforce fingertips using `optoforce` arg:

```bash
roslaunch ur5_allegro_moveit ur5_allegro_bringup.launch robot_ip:=177.22.22.11 optoforce:=false
```


### Simulation with Allegro Hand

In order to run MoveIt and Allegro Hand KDL controllers together in a simulation, you can run the following:

```bash
roslaunch allegro_hand_kdl allegro_torque.launch sim:=true RVIZ:=false
roslaunch ur5_allegro_moveit demo.launch
roslaunch arq_ur5 load_scene.launch scene:=bimanual_computers
```

Simulation doesn't include physics. It is just RViz and fake controllers.
