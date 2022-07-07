#The OS at a glance

The above diagram shows a simplified control flow diagram of the MuadQuad’s OS (featuring the MIT Controller, but is generalizable to any typical controller). The core of the robot’s operation is a feedback loop (shown in red) between the robot’s state estimator and leg controller.

Most controllers will take two inputs: the current state estimation of the robot, and gamepad inputs from the user. Using an algorithm, controllers will determine the necessary leg commands needed to achieve the behavior asked by the user (for example, to locomote forward), given that current state estimation.

In the case of the MIT Controller, the algorithm is a combination of Whole Body Control and Convex Model-Predictive-Control. These two in tandem issue leg commands to the leg controller, which are then sent to the robot server, which issues the low-level commands to the motors. The execution of these commands will inevitably alter the state of the on-board IMU, which will change the state estimation.

When running the MuadQuad in the lab, we run 4 separate terminals (shown by bolded arrows): the controller/support code, the robot server, the gamepad handler, and the control panel.
