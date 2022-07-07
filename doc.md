# The OS at a glance

The above diagram shows a simplified control flow diagram of the MuadQuad’s OS (featuring the MIT Controller, but is generalizable to any typical controller). The core of the robot’s operation is a feedback loop (shown in red) between the robot’s state estimator and leg controller.

Most controllers will take two inputs: the current state estimation of the robot, and gamepad inputs from the user. Using an algorithm, controllers will determine the necessary leg commands needed to achieve the behavior asked by the user (for example, to locomote forward), given that current state estimation.

In the case of the MIT Controller, the algorithm is a combination of Whole Body Control and Convex Model-Predictive-Control. These two in tandem issue leg commands to the leg controller, which are then sent to the robot server, which issues the low-level commands to the motors. The execution of these commands will inevitably alter the state of the on-board IMU, which will change the state estimation.

When running the MuadQuad in the lab, we run 4 separate terminals (shown by bolded arrows): the controller/support code, the robot server, the gamepad handler, and the control panel.


# Support code
Code that is not part of the actual robot controller is referred to as support code. The support code acts as a framework for implementing controllers by providing functionality for state estimation, leg control, and a floating-base model of the robot that user controllers can use in their implementations.

## `RobotRunner` & `PeriodicTask`: core execution
`PeriodicTask` is a parent class with two pure-virtual functions `run()` and `init()`. Any class that inherits from `PeriodicTask` should implement these two functions. If this is done, the child class can call `PeriodicTask::start()` which will begin executing its `run()` function at a desired frequency inside a new thread.

`RobotRunner` is the most important child class of `PeriodicTask`. Because of the described behavior, `RobotRunner::run()` is called periodically, and any function calls made from within this function will also have periodic execution. For example, `RobotRunner::run()` calls `RobotController::runController()` which is how the periodic controller execution is propagated. `RobotRunner` is also responsible for initializating and periodically executing the major components of the support code, including the state estimator and leg controller.

## State estimation
The aptly-named `StateEstimatorContainer` acts as a container for different state estimators. `RobotRunner` initializes these estimators and adds them to the container. There are currently 2 types of estimators: position/velocity and orientation estimator. There is also a cheater mode version of each for a total of 4 unique estimators. At any given time, only the 2 relevant estimators (whether you are or are not in cheater mode) are used.

When running the physical robot, these estimators receive information from the robot’s sensors, namely the IMU,  to create their estimates. When running the simulator, the `Simulation` instead will send fake IMU sensor data to the state estimators via shared memory.

Controllers can call `StateEstimatorContainer::getResult()` to get an estimate of the robot’s overall state.

## Static robot data
`RobotType` is an enumerated type that identifies which robot you are using (ie `RobotType::CHEETAH_3`, `RobotType::MUADQUAD`, etc). 

`Quadruped` contains constant parameters about the robot (link lengths, gear ratios, inertias...). Using this information, `Quadruped` can generate a floating-base model for the robot.

## Dynamic robot data
`FloatingBaseModel` is used in physical motion dynamics. Controllers can read and write the model as necessary. For example, MIT’s WBC calls a `_UpdateModel()` function every iteration. 

`LegController` is the interface for sending mid-level hardware commands to the actuators. When a hardware command is issued to the leg controller, it is sent via LCM to the robot server. 

## Control panel
`SimControlPanel` is run as its own program and serves as a GUI for controlling the robot while it is active. Despite its name, it is used for both simulation mode and hardware mode. If the controller is run in hardware mode, the control panel must also be run in hardware mode, conversely for simulation they must both be in sim mode.

In simulation mode, the panel initializes a `Simulator` object that handles flow of information to a virtual version of the robot. On each iteration, the `Simulation` will call `Simulation::lowLevelControl()` and `Simulation::highLevelControl()`. `lowLevelControl()` updates the simulated robot server board data and runs its controls. `highLevelControl()` updates the simulated IMU data and sends it to the robot, sends joystick commands to the robot, and finally allows the robot controller to run.

In hardware mode, the panel performs no extra math for the sake of simulation. The `SimControlPanel` will initialize a `RobotInterface` which is responsible for sending user parameters from the panel to the robot. It does allocate a `DynamicsSimulator`, but it is kinematic and used solely for visualization. Like `RobotRunner`, `RobotInterface` inherits from `PeriodicTask` and has periodic execution. This execution 1) sends gamepad commands to LCM and 2) updates the visualizer. 

_This is a point of confusion for jack atm… i thought the other program that spawns a GameController is responsible for sending inputs over LCM when youre in hardware mode, but RobotRunner::run() sends them over the same LCM channel? Why then do we need the other program?

## Gamepad input
The robot is commanded by a Logitech video game controller. Inputs from the gamepad are handled by up to three simple objects: a `GameController`, a `GamepadCommand`, and a `DesiredStateCommand`. How exactly they send inputs to the robot changes based on the mode.

In hardware mode, all gamepad inputs are handled by a separate executable. This separate program creates a `GameController` object which handles an LCM channel for sending inputs to the robot. Over this LCM channel, a `GamepadCommand` object is published, which acts as a container for the current state of all the gamepad buttons, triggers, and joysticks.

In simulation mode, there is no `GameController` object and no use of LCM. Instead, `Simulation::highLevelControl()` will update the `GamepadCommand` object directly via shared memory.

In both modes, `DesiredStateCommand` is initialized by `RobotRunner` and acts as a simple wrapper for the gamepad joystick inputs. CMPCL will interact with the `DesiredStateCommand` to get the joystick position and convert it into a desired locomotion speed for the robot. Simple, non-joystick inputs, like buttons, are not read through `DesiredStateCommand` but directly accessed by controllers through `GamepadCommand`.

To summarize:
- `GamepadCommand`: contains current state of the gamepad controller
- `DesiredStateCommand`: utility for converting `GamepadCommand`’s joystick state into a form usable by locomotion controllers
- `GameController`: hardware-only utility for sending `GamepadCommand` object to robot via LCM. In simulation mode, `GamepadCommand` is sent to robot via shared memory instead (no `GameController` needed)

## Control parameters
In the robot’s entire control stack, there are three types of parameters: generic `ControlParameters`, `RobotControlParameters`, and `SimulatorControlParameters`. The `ControlParameters` are often called user parameters in the code, and are used as inputs for the user controller. The contents of the `ControlParameters` will vary with the controller used. The `RobotControlParameters` describe attributes specific to the robot’s hardware, such as sensor noise. The `SimulatorControlParameters` describe parameters specific to the execution of the simulation, like simulation speed.

All three types of parameters are loaded from `.yaml` files. When the control program is run from the command line, one of the arguments is whether the parameters are loaded from a file. If this argument is set, the `HardwareBridge` will initialize the `ControlParameters` and the `RobotControlParameters` from the file specified in `MuadQuadHardwareBridge::run()`. If this argument is not set, the parameters will instead be loaded from a `.yaml` by the `SimControlPanel` and sent via LCM. The `SimulatorControlParameters` are always loaded from a file by the `SimControlPanel`.
