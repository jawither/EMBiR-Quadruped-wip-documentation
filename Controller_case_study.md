# Controller case study
To better help understand the codebase and the implementation of an inherited `RobotController`, this guide will walk through the implementation and execution of `JPos_Controller`, a simple provided controller for joint position control on the MuadQuad.

## Code entry and initialization
#### `int main(int argc, char** argv)`
The `main` function of any controller will pass command line arguments and a newly allocated instance of the desired controller type to `main_helper()`.
```cpp
int main(int argc, char** argv) {
  main_helper(argc, argv, new JPos_Controller());
  return 0;
}
```

#### `int main_helper(int argc, char** argv, RobotController* ctrl)`
`main_helper` will parse the command line arguments, which are as follows:
- `argc[1]`: the type of Robot
  - `e` for MuadQuad
- `argc[2]`: whether you are running the simulation or actual robot
  - `s` or `r`
- `argc[3]`(optional): whether parameters are loaded from file
  - `f` for load from file, if no argument, parameters will be loaded from network

In this guide, we will assume we are running the controller on the MuadQuad, on the actual robot (not the simulation).

Once `main_helper` has parsed the arguments, it will initialize either a `SimulationBridge` or a `HardwareBridge` and call `run()`.

```cpp
...
else if (gMasterConfig._robot == RobotType::MUADQUAD) {
      MuadQuadHardwareBridge hw(ctrl, gMasterConfig.load_from_file);
      hw.run();
}
```

