# Source and attribution

Joint topology, origins, axes and limits adapted from ROS-Industrial motoman_gp7_support:
https://github.com/ros-industrial/motoman/tree/846fbbc57c60223f12bf4795ea6404982d57709e/motoman_gp7_support
Branch retrieved: noetic-devel. Revision reported by GitHub: 846fbbc57c60223f12bf4795ea6404982d57709e.
The source package names Eric Marcil as author and declares BSD-3-Clause.
The original macro and package manifest are preserved in this directory.
The source cites YASKAWA MOTOMAN-GP8,-GP7 INSTRUCTIONS HW1486547, 194559-1CD rev 4
for joint limits and velocities. These are default variant values, not independent
verification of the user's machine or assignment.

All seven meshes in meshes/ are exact byte copies of the user's uploaded STLs.
No upstream reference meshes are distributed in this package.
CAD/mesh rights remain with their respective owners; this package does not grant
new redistribution rights for the CAD meshes.

Alignment: full-size upstream visual meshes were used as comparison geometry.
Numerical registration identified cardinal rotations. Final mesh origins were
snapped to those rotations and geometrically consistent offsets. This is an
estimate, not extraction of SolidWorks assembly mates. Fine CAD detail and
internal surfaces differ from the simplified upstream meshes. The registration
residuals therefore are not a measurement of joint accuracy.

## Physics and control update

Mass properties: user-provided SolidWorks text in this conversation; see mass_properties_input.json.
Plugin API and configuration checked against ROS Controls Humble documentation:
https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html
https://github.com/ros-controls/gz_ros2_control/tree/humble
World format: https://gazebosim.org/docs/fortress/building_robot/
Positive tensor notation: https://help.solidworks.com/2024/English/SolidWorks/sldworks/HIDD_MEASURE_OPTIONS.htm
Runtime has not been tested in the preparation environment.
