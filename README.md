# Pepper Localization and Navigation

The automatic navigation function is mainly divided into two steps: First step: Exploration and Mapping. Second step: Localization and Navigation.

First step: Exploration and Mapping: During the exploration and mapping, the Pepper uses Gmapping technology to construct a 2D map. Gmapping is an open-source SLAM algorithm based on the filtering SLAM framework. To simplify operation, we can use the Exploration API instead of the complicated algorithm.

Second step: Localization and Navigation: This part includes Marker recognition, Kalman filter and Navigation. By the inverse perspective transform, we can calculate the robot position from the known aruco marker's position. By the Kalman filter, we can reduce the errors. Finally, we can use the Navigation API to complete the Navigation.
