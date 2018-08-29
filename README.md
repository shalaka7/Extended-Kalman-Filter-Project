# Extended Kalman Filter Project 


In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]
  

RUBIC POINTS :

* Accuracy: The EKF accuracy was:
             Dataset 1 : RMSE <= [0.0973, 0.0855, 0.4513, 0.4399]
             Dataset 2 : RMSE <= [0.0726, 0.0965, 0.4216, 0.4932]

* The Kalman filter implementation can be found src/kalman_filter.cpp and it is used to predict at src/FusionEKF.cpp prediction section(line no. 100 to 131) and to update in  update section(line no. 137 to 149).

* The first measurement is handled at src/FusionEKF.cpp in Initialization section(line no. 51 to line 93)

* The predict operation could be found at src/FusionEKF.cpp line no.131 and the update operation from line no.139 to 149 of the same file.

* The calculation optimization is when the Q matrix is calculated src/FusionEKF.cpp line 110 to line 128.