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


# The MIT Controller
The support code allows for execution of user controllers through inheritance of the `RobotController` class. Any class that inherits from `RobotController` will be integrated into the system by calls to `RobotController::initializeController()` and `RobotController::runController()`. The MIT controller is an example of such a controller. The controller was originally written by MIT for their MiniCheetah quadruped and has been adapted for the MuadQuad.

## `MIT_Controller::initializeController()` & `MIT_Controller::runController()`
As is the case with any controller, `initializeController()` will be called once upon startup, and `runController()` will be called every iteration. For the MIT controller, initialization allocates a `ControlFSM` object. `runController()` 1) updates the `DesiredStateCommand`, and 2) runs the `ControlFSM`.

## The MIT FSM
The core of the MIT Controller is a finite state machine that expresses the current state of the robot’s behavior. Each state in the FSM is itself a class object inherited from `FSM_State`. When the `ControlFSM` object is created by `MIT_Controller::initializeController()`, its constructor will create all of the state objects. For the purpose of the MuadQuad, the four states used are `FSM_State_Passive`, `FSM_State_StandUp`, `FSM_State_BalanceStand`, and `FSM_State_Locomotion`.

`ControlFSM` is responsible for interacting with the `GamepadCommand` object to determine which control mode to activate, and thus to which state the FSM needs to transition. Each state inherits from the `FSM_State` parent class and implements its virtual member functions. At a high level, each iteration, `ControlFSM` will check the state of the gamepad to see if the human user has requested a change in state, and if so, call the appropriate state member functions to execute the transition. If no change in state is requested by the user, `ControlFSM` will execute the normal behavior of that state for the iteration.

`ControlFSM` also performs safety checks at various points in the control flow of the controller, specifically that the desired positions of the feet and the desired feedforward forces are all safe.

## FSM State functions
The `FSM_State` parent class offers several virtual functions that `Control_FSM` uses to polymorphically interact with the different states. Each state class should implement these functions.

### `FSM_State::checkTransition()`
After `Control_FSM` determines the user requested a change in state from the gamepad, it will call the current state’s`checkTransition()` function. This function will determine whether the current state is able to transition into the requested state, as well as how long the transition will take. In the case of a successful transition, `checkTransition()` returns the name of the new state after transitioning. If the transition is unsuccessful, it returns the name of the current state.

For example, you cannot transition from BalanceStand into StandUp (you’re already standing), so `FSM_State_BalanceStand::checkTransition()` will not verify the transition if the requested state is StandUp.

### `FSM_State::transition()`
This function is called after `checkTransition()` successfully verifies the transition. It handles any actions necessary to perform the actual transition itself. What this means specifically varies based on the transition. For most transitions, this function does nothing. 

_For transitioning from BalanceStand to Locomotion, the function will request the BalanceStand algorithm runs for extra iterations before finishing the transition. (Really, it requests the alg be run for 1000*transition_duration more iterations, but the transition_duration is 0… so i dont see the reason for adding that bit of logic to the code… (¬‿¬) )

### `FSM_State::onExit()`
Once `transition()` verifies the transition has completed, `Control_FSM` will call the current state’s `onExit()` function. This allows the state to clean up any necessary data before it completely exits. For most states, this function does nothing.

After `onExit()` returns, it is at this point that `Control_FSM` updates its `currentState` member variable to be enumerated type associated with the new state.

### `FSM_State::onEnter()`
The last step in the state transition is calling the new state’s `onEnter()` function. The main purpose of `onEnter()` is to zero out any data associated with the state, including data about the previous transition (which is now complete). Also, many states have dynamically allocated data that needs to be reinitialized when the state is entered.

In the case of a state transition, all of the `FSM_State` functions listed up until this point are each called once, consecutively, in the same iteration. After, `onEnter()` executes, however, the iteration finishes, and any further calls (namely to `run()`, or if another transition is triggered) will take place on the next iteration.

### `FSM_State::run()`
If `Control_FSM` determines that no transition is taking place on the current iteration, it will call the current state’s `run()` function, which triggers the default behavior of the robot when it is in that state.

Here, the differences between states are most visible, as each state’s implementation of the `run()` function varies significantly. `FSM_State_Passive::run()`, for example, does nothing, which reflects the lifeless nature of the robot when it’s in the passive state. Most other states’ `run()` functions will call a uninherited class-specific “step” function that handles the low-level details of the robot behavior in that state.

## WBC
Whole body control, or WBC, describes a feedback optimization technique that tracks multiple motion trajectories and solves conflicts using priorities in robots possibly under multiple contacts (L Sentis, 2021).

 [this is where i need help describing what exactly WBC is, how it works, and how much detail i should go into]. 

Any state that uses WBC (in the case of MuadQuad, every state except Passive) will allocate a `LocomotionCtrl` object, which inherits from the `WBC_Ctrl` class, and use it to call `WBC_Ctrl::run()` each iteration from within `FSM_State::run()`. From within this object, WBC will calculate optimized mid-level hardware commands and send them to the leg controller.

## `ConvexMPCLocomotion`
The most interesting FSM state in the MIT controller is `FSM_State_Locomotion`. In addition to using WBC, which runs every iteration, Locomotion also uses an implementation of model-predictive-control that simplifies the control into a convex optimization problem. CMPCL runs at a lower frequency, roughly 20-30hz. 

It’s from within CMPCL that we define the different gaits for the robot. CMPCL will predict the GRFs required to enact the selected gait and, like WBC, send commands to the leg controller.

 [like wbc i will need a bit of guidance with what to say here exactly]. 

## Gaits
All robot gaits are performed within the Locomotion state. Gaits are listed in `ConvexMPCLocomotion.h` and initialized by the CMPCL constructor. There are two types of gaits, `OffsetDurationGait` and `MixedFrequencyGait`.

### `OffsetDurationGait`
These are the most common type of gait. They are represented by a cycle length in timesteps and two four-dimensional vectors of integers. Each dimension of the vectors corresponds to one leg of the robot. The first vector represents the number of timesteps into each cycle that the legs make contact. The second vector represents the duration of each leg’s contact. Each leg can only have one contact per cycle.

### `MixedFrequencyGait`
The second kind of gait is a mixed-frequency gait, which allows each leg to have its own period independent from the other legs. TODO explain further


# Robot server
The robot server acts as an interface between the mid-level hardware commands passed to the leg controller and the low-level hardware of the actual robot. Because of the close relationship between it and the microcontroller firmware, the robot server runs on its own pi that is separate from the controller pi to allow greater flexibility of the controller and its hardware.

The robot server receives leg controller commands from the control pi via LCM and parses them into a byte stream that is usable by the pi3hat’s microcontrollers. These microcontrollers then dispatch the necessary low-level commands to the moteus drivers, which finally move the motors. As a last step, the moteus drivers send a response back with information about the state of the motors which is used by the robot server to do data logging. This data is sent back to the control pi via LCM.

# Vestigial code
The following pieces of code are present in the repository but are not used.

## Vestigial for MuadQuad
- FSM States: BackFlip, FrontJump, ImpedanceControl, JointPD, RecoveryStand, Vision
  - The MuadQuad only uses Passive, StandUp, BalanceStand, and Locomotion
- `VisionMPC`
  - Only used in the Vision state

## Truly vestigial
- `BalanceController`
  - `runBalanceController()` is called from `FSM_State::runControls()`, but `runControls()` is never called anywhere
- `GaitScheduler`
  - Only accessed by the balance controller
- `ContactEstimator`
  - An empty `.cpp`
- `SolverMPC`
  - Included indirectly by `ConvexMPCLocomotion` but not used
- `convexMPC_util`
  - Not included anywhere
- `OsqpTriples`
  - Not included anywhere other than a test file
- `GraphSearch`, `FootstepPlanner`, `FootstepCost`
  - Included by `ConvexMPCLocomotion` but not used
- `filters.cpp`
  - Not included anywhere other than a test file
- `save_file.cpp`
  - Not included anywhere other than a test file

