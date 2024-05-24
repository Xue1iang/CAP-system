# Confined Space Underwater Positioning Using Collaborative Robots


## Description of the data

The experiments were conducted at RAICo Lab in West Cumbria. The data were real-time data collected during experiments to validate the CAP system. The uploaded file consists of nine data sets. The nine subsets are: Lawnmower1-3, Random1-3, and Square1-3.

Lawnmower: The underwater robot moves according to a preset lawnmower pattern, while the surface robot automatically follows.

Random: ...

Square: ...

Each pattern folder contains a ROS bag (`.bag`) file to facilitate ROS users. It can be used with a script to run the CAP system to view and reproduce our work. The ROS bag includes the following rostopics:

*   `/imu/data`: Raw IMU data
*   `/cap_cd`: Real-time output of CAP-CD, coordinates of the underwater robot
*   `/cap_pnp`: Real-time output of CAP-CPnP, coordinates of the underwater robot
*   `/bluerov/mavros/global_position/rel_alt`: Depth sensor data from the underwater robot
*   `/qualisys/bluerov_8_points/pose`: Qualisys tracking underwater robot (Ground truth)
*   `/slam_out_pose`: Output from Hector SLAM of the surface robot, x, y coordinates and yaw (in quaternion form)
*   `/tag_detections`: EPnP solved pose of AprilTag in the camera
*   `/tag_detections_image`: Camera view (images)
*   `/tag_detections_raw`: Pixel coordinates of the four corners of the detected AprilTag

To facilitate plotting, rostopics other than images have also been converted into `.csv` format files.

*   `/imu/data` -> imu.csv
*   `/cap_cd` -> capcd_online.csv
*   `/cap_pnp` -> cappnp_online.csv
*   `/bluerov/mavros/global_position/rel_alt` -> depth_sensor.csv
*   `/qualisys/bluerov_8_points/pose` -> qualisys.csv
*   `/slam_out_pose` -> slam.csv
*   `/tag_detections` -> apriltag_pnp.csv
*   `/tag_detections_raw` -> corners.csv

## Code/Software

The code was written in Python and uploaded in the format of a ROS package. The following steps are required:

1.  Compile `cap_code`.
2.  Download the dataset.
3.  Play the rosbag with the command `rosbag play /path_to_rosbags/***.bag` and also execute `roslaunch cap_codes localisation_1_3.launch`. (You can also integrate the `rosbag play` command into the launch file, depending on user preference.)
4.  You can monitor `/cap_cd` or `/cap_pnp` through PlotJuggler or by any other means.

