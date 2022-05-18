# MuadQuad controller documentation
To add your own robot controller, you should add a folder under `Cheetah-Software/user`, and add the folder to the `CMakeLists.txt` in `user`. Your `.cpp` and `.hpp` files are the actual controller, which should extend `RobotController`.
## `RobotController` members and their accesses
### `Quadruped<float>* _quadruped`
Contains constant parameters about the robot (link lengths, gear ratios, inertias...). The `getHipLocation` function returns the location of the "hip" in the body coordinate system. The x-axis points forward, y-axis to the left, and z-axis up. 
#### Modules that modify `_quadruped`
- `RobotRunner`
  - `init()`
- `RobotInterface`
  - default constructor
- `Simulation`
  - default constructor
#### Modules that access `_quadruped` without modifying
- `LegController`
  - default constructor
- `LegControllerData` struct (`LegController.h`)
- `computeLegJacobianAndPosition()` (`LegController.h`)

### `FloatingBaseModel<float>* _model`
A dynamics model of the robot. This can be used to compute forward kinematics, Jacobians, etc..
#### Modules that modify `_model`
- `RobotRunner`
  - `init()`
- `RobotInterface`
  - default constructor
- `Simulation`
  - default constructor
- `DynamicsSimulator`
  - default constructor, `runABA()`, `forwardsKinematics()`, `setState()`, `setAllExternalForces()`, `step()`
- `ContactImpulse`
  - `_UpdateVelocity()`
- `ContactSpringDamper`
  - `_UpdateExternalForces()`, `groundContactWithOffset()`
#### Modules that access `_model` without modifying
- `ContactConstraint`
  - default constructor, `_CheckContact()`
- `ContactImpulse`
  - default constructor
- `ContactSpringDamper`
  - default constructor
- `DynamicsSimulator`
  - `getNumBodies()`, `getTotalNumGC()`, `getModel()`, `updateCollisions()`

### `LegController<float>* _legController`
Interface to the robot's legs. This data is syncronized with the hardware at around 700 Hz. There are multiple ways to control the legs, and the result from all the controllers are added together.
#### `_legController` sub members
- `commands[leg_id].tauFeedForward` : Leg torque (Nm, at the joint). Order is ab/ad, hip, knee.
- `commands[leg_id].forceFeedForward` : Force to apply at foot (N), in hip frame. (Same orientation as body frame, origin is the hip)
- `commands[leg_id].qDes` : Desired joint position for joint PD controller (radians). Order is ab/ad, hip, knee. (0,0,0) is leg pointing straight down.
- `commands[leg_id].qdDes` : Desired joint velocity for joint PD controller (rad/sec).
- `commands[leg_id].pDes`, `vDes` : Desired foot position/velocity for cartesian PD controller (meters, hip frame)
- `commands[leg_id].kpCartesian`, `kdCartesian`, `kpJoint`, `kdJoint` : Gains for PD controllers (3x3 matrix). Use the diagonal entries only.
- `datas[leg_id].q` : Leg joint encoder (radians). Order is ab/ad, hip, knee. (0,0,0) is leg pointing straight down.
- `datas[leg_id].qd` : Leg joint velocity (radians/sec). Same order as q.
- `datas[leg_id].p` : Foot cartesian position, in hip frame. (Same orientation as body frame, origin is the hip)
- `datas[leg_id].v` : Foot cartesian velocity, in hip frame.
- `datas[leg_id].tau` : Estimate of motor torque from combination of all controllers. The joint PD control actually runs at 40 kHz on the motor controllers.

#### Modules that modify `_legController`
- `commands[leg_id].kpJoint` and `commands[leg_id].kdJoint` written by `RobotRunner::run()`
- freed by `RobotRunner::~RobotRunner()`
#### Modules that access `_legController` without modifying
- `datas[leg_id].q` read by `RobotRunner::run()`

### `StateEstimate<float>* _stateEstimate`, `StateEstimatorContainer<float>* _stateEstimatorContainer`
The result and interface for the provided state estimator. If you provide the contact state of the robot (which feet are touching the ground), it will determine the robot's position/velocity in the world.

Accesses to the state estimator are particularly confusing because of the onion-like layering of the data structure and the different it’s accessed by different modules. “Reads” of `_stateEstimatorContainer` occur through an access of `_stateEstimate`, which usually happens through a call to `StateEstimatorContainer::getResult()`. An exception to this is in `GenericEstimator` and other inherited estimators, which have a `_stateEstimatorData` member, which itself points to the same memory as `_stateEstimatorContainer::_data.result` and `_stateEstimatorContainer::getResult`. Thus accesses to the state estimator from within the estimators themselves look a bit different than access from, say, `RobotRunner` or the MIT controllers.

The only direct modifications to the state estimator made by controllers should be calls to `StateEstimatorContainer::setContactPhase()`. `RobotRunner::initializeStateEstimator()` will also modify it by adding the relevant estimators to the container. Other than these exceptions, all writes to the state estimator should come from the periodic calls to the actual estimators’ `run()` functions.

#### Modules that modify `_stateEstimatorContainer`
- `RobotRunner`
  - `init()`, `initializeStateEstimator()`, destructor
- `CheaterOrientationEstimator`
  - `run()`
- `CheaterPositionVelocityEstimator`
  - `run()`
 - `VectorNavOrientationEstimator`
   - `run()`
 - `LinearKFPositionVelocityEstimator`
   - `run()`
 - `ContactEstimator`
   - `run()`
 #### Modules that access `_stateEstimatorContainer` without modifying
 - `LinearKFPositionVelocityEstimator`
   - `setup()`

### `GamepadCommand* _driverCommand`
Inputs from the game pad.
#### Modules that modify `_driverCommand`
- `MiniCheetahHardwareBridge`
  - `run()`
- `Cheetah3HardwareBridge`
  - `run()`
- `HardwareBridge`
  - `handleGamepadLCM()`
- `GameController`
  - `updateGameCommand()`

### `RobotControlParameters* _controlParameters`
Values from the center robot control parameters panel.
#### Modules that modify `_controlParameters`
- `RobotInterface`
  - default constructor
#### Modules that access `_controlParameters` without modifying
- `RobotInterface`
  - `startInterface()`

### `VisualizationData* _visualizationData`
Interface to add debugging visualizations to the simulator window.
#### Modules that modify `_visulizationData`
- `RobotInterface`
  - default constructor
- `Simulation`
  - default constructor
#### Modules that access `_visualizationData` without modifying
- `Graphics3D`
  - `_AdditionalDrawing()`

### `RobotType _robotType`
Whether you are the mini Cheetah or Cheetah 3 robot.
#### Modules that modify `_robotType`
- `RobotInterface`
  - default constructor

## `RobotController` member accesses from within controllers
If you are interested in the design of the provided controllers in `user`, or are struggling to get them working, you can consult their memory accesses here.

### `Quadruped<float>* _quadruped`
#### No writes
#### Reads
- `ControlFSM`
  - default constructor
- `struct ControlFSMData` (`ControlFSMData.h`)
- `ConvexMPCLocomotion`
  - `_SetupCommand()`, `run()`
- `VisionMPCLocomotion`
  - `run()`
- `FSM_State_BackFlip`
  - default constructor
- `FSM_State_BalanceStand`
  - default constructor, `onEnter()`
- `FSM_State_FrontJump`
  - default constructor
- `FSM_State_Locomotion`
  - default constructor
- `FSM_State_StandUp`
  - default constructor
- `FSM_State_Vision`
  - default constructor
- `FSM_State`
  - `runControls()`, `runBalanceController()`
- `SafetyChecker`
  - `checkPDesFoot()`, `checkForceFeedForward()`

### `FloatingBaseModel<float>* _model`
#### Writes
- `Leg_InvDyn_Controller`
  - `runController()`
- `WBC_Ctrl`
  - `_UpdateModel()`
#### Reads
- `WBC_Ctrl`
  - default constructor
- `LocomotionCtrl`
  - default constructor, `_LCM_PublishData()`

## `LegController<float>* _legController`
### `Vec3<T> commands[leg_id].tauFeedForward`
#### Writes
- `JPosInitializer`
  - `IsInitialized()`
- `Leg_InvDyn_Controller`
  - `runController()`
- `JPos_Controller`
  - `runController()`
- `VisionMPCLocomotion`
  - `run()`
- `FSM_State_BackFlip`
  - `_Initialization()`, `_SafeCommand()`
- `FSM_State_FrontJump`
  - `_Initialization()`, `_SafeCommand()`
#### No reads

### `Vec3<T> commands[leg_id].forceFeedForward`
#### Writes
- `ConvexMPCLocomotion`
  - `run()`
- `VisionMPCLocomotion`
  - `run()`
- `SafetyChecker`
  - `checkForceFeedForward()`
#### No reads

### `Vec3<T> commands[leg_id].qDes`
#### Writes
- `JPosInitializer`
  - `IsInitialized()`
- `JPos_Controller`
  - `runController()`
- `Leg_InvDyn_Controller`
  - `runController()`
- `MiniCheetahSpi_Controller`
  - `runController()`
- `FSM_State_BackFlip`
  - `_Initialization()`, `_SafeCommand()`
- `FSM_State_FrontJump`
  - `_Initialization()`, `_SafeCommand()`
- `FSM_State`
  - `jointPDControl()`
#### No reads

### `Vec3<T> commands[leg_id].qdDes`
#### Writes
- `JPosInitializer`
  - `IsInitialized()`
- `JPos_Controller`
  - `runController()`
- `Leg_InvDyn_Controller`
  - `runController()`
- `FSM_State_BackFlip`
  - `_Initialization()`, `_SafeCommand()`
- `FSM_State_FrontJump`
  - `_Initialization()`, `_SafeCommand()`
- `FSM_State`
  - `jointPDControl()`
#### No reads

### `Vec3<T> commands[leg_id].pDes`
#### Writes
- `ConvexMPCLocomotion`
  - `run()`
- `VisionMPCLocomotion`
  - `run()`
- `FSM_State_StandUp`
  - `run()`
- `FSM_State`
  - `cartesianImpedanceControl()`
- `SafetyChecker`
  - `checkPDesFoot()`
#### Reads
- `FSM_State_Locomotion`
  - `LocomotionControlStep()`
### `Vec3<T> commands[leg_id].vDes`
#### Writes
- `ConvexMPCLocomotion`
  - `run()`
- `VisionMPCLocomotion`
  - `run()`
- `FSM_State_Locomotion`
  - `LocomotionControlStep()`
- `FSM_State`
  - `cartesianImpedanceControl()`
#### No reads

### `Mat3<T> commands[leg_id].kpCartesian`
#### Writes
- `ConvexMPCLocomotion`
  - `run()`
- `VisionMPCLocomotion`
  - `run()`
- `FSM_State_StandUp`
  - `run()`
- `FSM_State`
  - `cartesianImpedanceControl()`
#### Reads
- `FSM_State_Locomotion`
  - `LocomotionControlStep()`

### `Mat3<T> commands[leg_id].kdCartesian`
#### Writes
- `ConvexMPCLocomotion`
  - `run()`
- `VisionMPCLocomotion`
  - `run()`
- `FSM_State_Locomotion`
  - `LocomotionControlStep()`
- `FSM_State_StandUp`
  - `run()`
- `FSM_State`
  - `cartesianImpedanceControl()`
#### No reads

### `Mat3<T> commands[leg_id].kpJoint`
#### Writes
- `Leg_InvDyn_Controller`
  - `runController()`
- `JPos_Controller`
  - `runController()`
- `MiniCheetahSpi_Controller`
  - `runController()`
- `FSM_State_BackFlip`
  - `_Initialization()`
- `FSM_State_FrontJump`
  - `_Initialization()`
- `FSM_State`
  - `jointPDControl()`
#### No reads

### `Mat3<T> commands[leg_id].kdJoint`
#### Writes
- `Leg_InvDyn_Controller`
  - `runController()`
- `JPos_Controller`
  - `runController()`
- `MiniCheetahSpi_Controller`
  - `runController()`
- `ConvexMPCLocomotion`
  - `run()`
- `VisionMPCLocomotion`
  - `run()`
- `FSM_State_BackFlip`
  - `_Initialization()`
- `FSM_State_FrontJump`
  - `_Initialization()`
- `FSM_State`
  - `jointPDControl()`
#### No reads
### `Vec3<T> datas[leg_id].q`
#### No writes
#### Reads
- `JPosInitializer`
  - `_UpdateInitial()`
- `JPos_Controller`
  - `runController()`
- `Leg_InvDyn_Controller`
  - `runController()`
- `VisionMPCLocomotion`
  - `run()`
- `WBC_Ctrl`
  - `_UpdateLegCMD()`
- `FSM_State_BackFlip`
  - `onEnter()`, `_SafeCommand()`
- `FSM_State_FrontJump`
  - `onEnter()`, `_SafeCommand()`
- `FSM_State_JointPD`
  - `onEnter()`
- `FSM_State_RecoveryStand`
  - `onEnter()`, `_StandUp()`
- `FSM_State`
  - `runBalanceController()`
### `Vec3<T> datas[leg_id].qd`
#### No writes
#### Reads
- `Leg_InvDyn_Controller`
  - `runController()`
### `Vec3<T> datas[leg_id].p`
#### No writes
#### Reads
- `ConvexMPCLocomotion`
  - `run()`
- `VisionMPCLocomotion`
  - `run()`
- `FSM_State_Locomotion`
  - `locomotionSafe()`
- `FSM_State_StandUp`
  - `onEnter()`
- `FSM_State`
  - `runBalanceController()`

### `Vec3<T> datas[leg_id].v`
#### No writes
#### Reads
- `FSM_State_Locomotion`
  - `locomotionSafe()`
### `Vec3<T> datas[leg_id].tauEstimate`
#### No accesses from controllers
