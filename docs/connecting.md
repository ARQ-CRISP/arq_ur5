### Connecting to UR5 robot

[Check this link for details](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial)

UR5 IP Addresses are
- 177.22.22.11 for right arm
- 177.22.22.12 for left arm (**update needed**)


Connect the ethernet cable, click network on the status bar.
_Edit connections > select UR5 > edit > IPv4 tab_

Make sure the address is something other than robot's and other computer's addresses, starting with 177.22.22...

|||
|---|---|
| Subnet mask | 255.255.255.0 |
| Gateway | 177.22.22.1 |
| DNS servers | 177.22.22.1 |

These three are the same for both the robot and the computer.

Save and close. Test connectivity by pinging the robot, for the right arm:
```
	ping 177.22.22.11
```

Then you can run the ur5 driver and moveit planner etc.
