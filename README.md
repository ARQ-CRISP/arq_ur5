# ARQ UR5 Package

ROS programs to use UR5 in ARQ lab.

## Moveit and RViz

You can use MoveIt and RViz in real or in simulation.

### Running with Allegro hand

First, the computer should be [connected to the robot](docs/connecting.md).

You need the [allegro-hand-ros](https://github.com/gokhansolak/allegro-hand-ros), [allegro_hand_kdl](https://github.com/ARQ-CRISP/allegro_hand_kdl) and [ur5_allegro_moveit](https://github.com/ARQ-CRISP/ur5_allegro_moveit) packages. The following set of commands will connect to the real robot and start the drivers, launch MoveIt and RViz and add the collision objects to scene. **It is your responsibility to confirm the configuration of collision objects, the default scene may be outdated**.

The following starts the drivers and loads some necessary parameters for *MoveIt*:

```bash
roslaunch ur5_allegro_moveit ur5_allegro_bringup.launch robot_ip:=177.22.22.11
```
Then you can start MoveIt and RViz as below:

> Tip: You can create a custom launch file for your needs at this step

```bash
roslaunch arq_ur5 moveit_rviz.launch allegro:=true
```

Finally, the collision object must be added to work safely on the real robot. An example collision scene can be loaded as follows:

```bash
roslaunch arq_ur5 load_scene.launch scene:=bimanual_computers
```

Then, you can run your own moveit control program. MoveIt move group names for different parts of the system are `ur5_arm`, `allegro_finger0`, `allegro_finger1`, `allegro_finger2`, `allegro_finger3`.

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

#### Simulation with Allegro Hand

In order to run MoveIt and Allegro Hand KDL controllers together in a simulation, you can run the following instead of the above commands:

```bash
roslaunch allegro_hand_kdl allegro_torque.launch sim:=true RVIZ:=false
roslaunch ur5_allegro_moveit demo.launch
roslaunch arq_ur5 load_scene.launch scene:=bimanual_computers
```

Simulation doesn't include physics. It is just RViz and fake controllers.

### Adding collision objects

Simply run the ```load_scene.launch``` file to add predefined collision objects into the scene.
```bash
roslaunch arq_ur5 load_scene.launch scene:=my_scene
```

Here, _my\_scene_ is the name of the **.scene** file in the _arq\_ur5/scenes_ folder. If you want to use the default scene file for the bimanual platform, you can omit the scene parameter as follows:
```bash
roslaunch arq_ur5 load_scene.launch
```

This will load the _arq\_ur5/scenes/bimanual.scene_ by default.
