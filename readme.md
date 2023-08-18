This is used for visualizing transforms and vectors in rviz using tf and makers.

## Usage
Launch the tf_viz.launch file
```
roslaunch tf_viz tf_viz.launch
```

## Features
**AXESMarker:** This is used to visualize the axes of a frame.

Input: 
- ns: represent the child frame, also the name of this axes marker 
- frame: represent the father frame
- translation: the translation from "frame" frame to "ns" frame
- rotation: the rotation from "frame" frame to "ns" frame

Output: an axes marker and corresponding TF from "frame" frame to "ns" frame

The axes are colored as follows:
- x-axis: red
- y-axis: green
- z-axis: blue

**VecMarker:** This is used to visualize a vector in a frame.

Input:
- ns: the name of this vector
- id: a unique id for this vector in this ns
- frame: the frame where this vector is represented
- p_end: the end point of this vector. (The start point is set to (0,0,0) by default.)

## Example
To add a transform from world to camera frame, add the following code.
```c++
// Set the translation and rotation
std::vector<double> t_CinW = {10.0, 10.0, 0.0};
Eigen::Matrix3d R_WtoC;
R_WtoC << 0, -1, 0,
      0, 0, 1,
      1, 0, 0;
Eigen::Quaterniond q_WtoC;
q_WtoC = Eigen::Quaterniond(R_WtoC);

// Set the cam frame axes marker 
AXESMarker cam_axes("camera", "world", t_CinW, q_WtoC);
markers.markers.insert(markers.markers.end(), {cam_axes.get_axes().markers.at(0),
                                             cam_axes.get_axes().markers.at(1),
                                             cam_axes.get_axes().markers.at(2),
                                             cam_axes.get_axes().markers.at(3)});
// Pub the corresponding TF
static_broadcaster.sendTransform(cam_axes.get_tf_msg());
```