# LEMUR CoLo-D

## Synopsis

CoLo-D provides 2d datasets for studying problem like like cooperative localization (with only a team robots), cooperative localization with a known map, and cooperative simultaneous localization and mapping (SLAM). It includes datasets that are compatible with CoLo-AT, a software analysis tool for localization algorithms. These datases can be collected by CoLo-PE, the physical experiment of CoLo.


## CoLo Compatible Datasets:

CoLo Datasets:
4 individual dataset with 3 robots
- official_dataset1
- official_dataset2
- official_dataset3
- official_dataset4


UTIAS dataset (created by the Autonomous Space Robotics Lab (ASRL) at the University of Toronto):
9 individual dataset with 5 robots
- MRCLAM_Dataset1
- MRCLAM_Dataset2
- MRCLAM_Dataset3
- MRCLAM_Dataset4
- MRCLAM_Dataset5
- MRCLAM_Dataset6
- MRCLAM_Dataset7
- MRCLAM_Dataset8
- MRCLAM_Dataset9
details info: [UTIAS Dataset](http://asrl.utias.utoronto.ca/datasets/mrclam/).

## CoLo Dataset format

For a team of 3 robots, there are:
3 odometry files (filename: Robot[subject #]_Odometry.dat ), one for each robot:
[time [s] | linear velocity [m/s] | angular velocity [rad/s]]

3 measurement files (filename: Robot[subject id]_Measurement.dat ), one for each robot with respect to its camera
[time [s] | ArUco Code | range [m] | bearing [rad]]

3 measurement_x files (filename: Robot[subject id]_Measurement_x.dat ), one for each robot with respect to its center and covert ArUco Code to subject id
[time [s] |subject id | range [m] | bearing [rad]]

3 groudntruth files (filename: Robot[subject id]_Groundtruth.dat ), one for each robot
[time [s] | x [m] | y [m] | orientation [rad]] 

1 landmark groundtruth files (file name: Landmark_Groundtruth.dat ), for conversion between subject id and landmark locations
[subject id | x [m] | y [m]]

Note: for landmark, subject id is the same as its AruCo code; for robots subject id is from 0 to 5. 

## Authors
Shengkang Chen


