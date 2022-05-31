# Controller case study: `JPos_Controller`
To better help understand the codebase and the implementation of an inherited `RobotController`, this mid-level guide will walk through the implementation and execution of `JPos_Controller`, a basic provided controller for joint position control on the MuadQuad.

### Contents
1. [Code entry and initialization](#code-entry-and-initialization)
2. [`RobotRunner` and `PeriodicTask`](#robotrunner-and-periodictask)
3. [`JPos_Controller`](#jpos_controller)

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
`RobotRunner::init()` subscribes to the robot server response LCM channel and initializes the controller and its members.

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
  ...
  _robot_ctrl->initializeController();
```

### `void RobotRunner::run()`
`RobotRunner::run()` dispatches the core function calls for the execution of the robot. Recall that `RobotRunner` inherits from `PeriodicTask`, so `RobotRunner::run()` is executed periodically.

On every iteration, `run()` will step the state estimator and clear the previous iteration's visualization data. The iteration count is tracked by `count_ini`.

```cpp
void RobotRunner::run() {
  _stateEstimator->run();
  visualizationData->clear();
  setupStep();

  static int count_ini(0);
  ++count_ini;
  ...
}
```

To prevent the effects of race conditions, the leg controller is disabled for the first 50 iterations.

```cpp
  ...
  if (count_ini < 10) {
    _legController->setEnabled(false);
  } else if (20 < count_ini && count_ini < 30) {
    _legController->setEnabled(false);
  } else if (40 < count_ini && count_ini < 50) {
    _legController->setEnabled(false);
  } else {
    _legController->setEnabled(true);
  ...
```
From the 50th iteration onwards, core behavior of the function is enabled.

First, emergency stop is checked.
```cpp
    ...
    if( (rc_control.mode == RC_mode::OFF) && controlParameters->use_rc ) {
      for (int leg = 0; leg < 4; leg++) {
        _legController->commands[leg].zero();
      }
      _robot_ctrl->Estop();
    }
    ...
```
Next, the leg controller is initialized.

```cpp
      ...
      if (!_jpos_initializer->IsInitialized(_legController)) {
        Mat3<float> kpMat;
        Mat3<float> kdMat;
        ...
        else if (robotType == RobotType::MUADQUAD){
          //Need to redefine these for MUADQUAD
          kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
          kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
        }
        ...
        for (int leg = 0; leg < 4; leg++) {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
      }
      ...
```
Finally, the controller and visualizer are executed.
```cpp
      ...
      else {
        _robot_ctrl->runController();
        cheetahMainVisualization->p = _stateEstimate.position;
        ...
  finalizeStep();
}
```

> **Warning**
> setupStep() and finalizeStep() might not be worth going over. they may be too low-level for the purpose of this guide, maybe will remove them.

### `void RobotRunner::setupStep()`
This function is run every iteration **before** the user code in `RobotController::run()` is executed. Mainly, it sets up the leg controller for the next iteration. It also handles some simulator-only behavior for the state estimator.

```cpp
void RobotRunner::setupStep() {
  ...
  else if (robotType == RobotType::MUADQUAD) {
    _legController->updateData(robServData);
  }

  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  _legController->setEnabled(true);
  _legController->setMaxTorqueCheetah3(208.5);

  // state estimator checks (for simulator only)
  if (!_cheaterModeEnabled && controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning to Cheater Mode...\n");
    initializeStateEstimator(true);
    _cheaterModeEnabled = true;
  }
  if (_cheaterModeEnabled && !controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning from Cheater Mode...\n");
    initializeStateEstimator(false);
    _cheaterModeEnabled = false;
  }

  get_rc_control_settings(&rc_control);
}
```

### `void RobotRunner::finalizeStep()`
Conversely, this function is run every iteration **after** user code in `RobotController::run()` runs. It sends leg commands to the robot server, updates the state estimator, and publishes debug data via LCM.

> **Warning**
> TODO difference between leg_control_data and leg_control_command?

```cpp
void RobotRunner::finalizeStep() {
  ...
  else if (robotType == RobotType::MUADQUAD) {
    _legController->updateCommand(robServCommand);

    robot_server_command_lcmt LCMCommandfix;

    for(int leg = 0; leg < 4; leg++) {
      for(int axis = 0; axis < 3; axis++) {
        int idx = leg*3 + axis;
        int mq_idx = muadquad_leg_reordering[idx];
        int sign = 1;
        if (leg%2 == 0)
          sign = -1;
        LCMCommandfix.tau_ff[mq_idx] = sign*robServCommand->tau_ff[idx];
        LCMCommandfix.q_des[mq_idx]  = sign*(robServCommand->q_des[idx] - muadquad_angle_offsets[idx]);
        LCMCommandfix.qd_des[mq_idx] = sign*robServCommand->qd_des[idx];
        LCMCommandfix.kp_joint[mq_idx] = robServCommand->kp_joint[idx];
        LCMCommandfix.kd_joint[mq_idx] = robServCommand->kd_joint[idx];
      }
    }

    _lcm.publish("robot_server_command", &LCMCommandfix);
    
  } else {
    assert(false);
  }
  _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
  _stateEstimate.setLcm(state_estimator_lcm);
  _lcm.publish("leg_control_command", &leg_control_command_lcm);
  _lcm.publish("leg_control_data", &leg_control_data_lcm);
  _lcm.publish("state_estimator", &state_estimator_lcm);
  _iterations++;
}
```
> **Note**
> **Everything up until this point is universal behavior of the support code and will be the same for all controllers. Beyond this point is documentation specifically for the `JPos_Controller` controller.**

# `JPos_Controller`
Recall that `JPos_Controller` and all other controllers inherit from `RobotController` and implement its pure virtual functions `initializeController()`, `runController()` and `updateVisualization()`. `JPos_Controller` is a basic controller that only implements `RobotController::runController()`.

### `JPos_Controller::runController()`
