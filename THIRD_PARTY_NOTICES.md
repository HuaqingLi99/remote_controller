# Third-party materials

The MIT license in `LICENSE` applies to the original code and documentation
authored by Huaqing Li. Third-party materials remain subject to their own
licenses and copyright notices.

## Panda robot model assets

The repository includes the following robot model files:

- `src/remote_controller/assets/panda/panda_arm.urdf`
- `src/remote_controller/assets/meshes/visual/*.dae`
- `src/remote_controller/assets/meshes/collision/*.stl`

### Verified Franka materials

The collision meshes `hand.stl` and `link1.stl` through `link7.stl` are
byte-for-byte identical to files in Franka Robotics' `franka_description`
package, in the official `franka_ros` repository at revision
`ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98`:

https://github.com/frankarobotics/franka_ros/tree/ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98/franka_description

The upstream package credits Franka Robotics GmbH and declares Apache-2.0.
The upstream license is included in
[`licenses/Franka-APACHE-2.0.txt`](licenses/Franka-APACHE-2.0.txt).
These eight collision meshes are unchanged.

### Generated and modified model files

The URDF header identifies it as generated from
`robots/panda/panda.urdf.xacro`. The URDF, the two other collision meshes,
and the visual meshes differ from the official files checked above. The exact
historical sources and modification history of these copies were not recorded
in the supplied source snapshot and have not been independently established.
Their embedded metadata is retained. None of the robot model assets is
covered by Huaqing Li's MIT license grant.

## Dependencies

External dependencies, including libfranka, Eigen, XML-RPC C/C++, NumPy,
Pinocchio, Sphinx, and Furo, retain their own licenses. This project's MIT
license does not replace those licenses.
