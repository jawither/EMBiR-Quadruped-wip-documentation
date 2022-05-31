# Controller case study: `JPos_Controller`
To better help understand the codebase and the implementation of an inherited `RobotController`, this mid-level guide will walk through the implementation and execution of `JPos_Controller`, a basic provided controller for joint position control on the MuadQuad.

### Contents
1. [Code entry and initialization](#code-entry-and-initialization)
2. [`RobotRunner` and `PeriodicTask`](#robotrunner-and-periodictask)

# Code entry and initialization
### `int main(int argc, char** argv)`
The `main` function of any controller will pass command line arguments and a newly allocated instance of the desired controller type to `main_helper()`.
```cpp
int main(int argc, char** argv) {
  main_helper(argc, argv, new JPos_Controller());
  return 0;
}
```

### `int main_helper(int argc, char** argv, RobotController* ctrl)`
`main_helper` will parse the command line arguments, which are as follows:
- `argc[1]`: the type of Robot
  - `e` for MuadQuad
- `argc[2]`: whether you are running the simulation or actual robot
  - `s` or `r`
- `argc[3]`(optional): whether parameters are loaded from file
  - `f` for load from file, if no argument, parameters will be loaded from network

Once `main_helper` has parsed the arguments, depending on whether you are running the robot simulation or the actual robot, it will initialize either a `SimulationBridge` or a `HardwareBridge` and call `run()`.

In this guide, we will assume we are running the controller on the MuadQuad, on the actual robot.

```cpp
...
else if (gMasterConfig._robot == RobotType::MUADQUAD) {
      MuadQuadHardwareBridge hw(ctrl, gMasterConfig.load_from_file);
      hw.run();
}
```
### `void MuadQuadHardwareBridge::run()`
This function mainly serves to initialize a dynamically allocated instance of `RobotRunner`.
```cpp
_robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");   
_robotRunner->LCMCommand = &_LCMCommand;
_robotRunner->robServData = &_robServData;
_robotRunner->robServCommand = &_robServCommand;
...
```

```cpp
_robotRunner->start();
```

# `RobotRunner` and `PeriodicTask`
`RobotRunner` inherits from `PeriodicTask`, allowing its `run` function to be executed in a looping thread. This looping `run` is the core of the controller execution. Here is how `RobotRunner::init()` and `RobotRunner::run()` are called from `MuadQuadHardwareBridge::run()`:

```cpp
_robotRunner->start();
```

```cpp
void PeriodicTask::start() {
  ...
  init(); // pure virtual, calls RobotRunner::init()
  ...
  _thread = std::thread(&PeriodicTask::loopFunction, this);
}
```

```cpp
void PeriodicTask::loopFunction() {
...
  while (_running) {
    ...
    run(); // pure virtual, calls RobotRunner::run()
    ...
...
}
```

Any module that inherits from `PeriodicTask` should implement the pure virtual functions `PeriodicTask::init()` and `PeriodicTask::run()` and be executed in the same way.

### `void RobotRunner::init()`
`RobotRunner::init()` sets up LCM subscriptions and initializes the controller and its members.

```cpp
_responseLCM.subscribe("robot_server_response", &RobotRunner::handleresponseLCM, this);
...
_responselcmthread = std::thread(&RobotRunner::handlelcm, this);
```

```
_robot_ctrl->_model = &_model;
_robot_ctrl->_quadruped = &_quadruped;
_robot_ctrl->_legController = _legController;
_robot_ctrl->_stateEstimator = _stateEstimator;
_robot_ctrl->_stateEstimate = &_stateEstimate;
_robot_ctrl->_visualizationData= visualizationData;
_robot_ctrl->_robotType = robotType;
_robot_ctrl->_driverCommand = driverCommand;
_robot_ctrl->_controlParameters = controlParameters;
_robot_ctrl->_desiredStateCommand = _desiredStateCommand;

_robot_ctrl->initializeController();
  ```
