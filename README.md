# MuadQuad controller documentation
To add your own robot controller, you should add a folder under `Cheetah-Software/user`, and add the folder to the `CMakeLists.txt` in `user`. Your `.cpp` and `.hpp` files are the actual controller, which should extend `RobotController`.
## `RobotController` members and their accesses
### `_quadruped`
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

### `_model`
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

### `_legController`
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

hey
