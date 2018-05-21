# FCND-Term1-P4-Estimation

## Project Rubric

#### 1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

The process of determine the standard deviation of the measurement noise consists of the following steps

1. First turning on the log tracing by adding the following two commands in the config file (06_SensorNoise.txt) of the scenario.
```
Commands += AddGraph1.LogToFile
Commands += AddGraph2.LogToFile
```
Run scenario 06_SensorNoise, the we will got two log files. One is the measurement of GPS x location and another is acceleometer x velocity.

2. Write a data processing script (./scripts/process_log.py) 
3. Run the data processing script, I can easily figure out the standard deviation of the measurements: MeasuredStdDev_GPSPosXY is around 0.71, and MeasuredStdDev_AccelXY is around 0.49.

#### 2. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.

Steps of integrating the body rates into new Euler angles are as follows.
1. First, Creating a quaternion from the current euler angle estimate with the handy FromEuler123_RPY function.
2. Then integrated the body rate ((rollEst, pitchEst and ekfState(6))) into the quaternion.
3. Finally, convert the quaternion back to the new euler angle.

#### 3. Implement all of the elements of the prediction step for the estimator.
Breakdown of the prediction step are as follows,

1. First, Run scenario 08_PredictState with the default code with a perfect IMU (QuadEstimatorEKF.attitudeTau = 100, only an IMU), Which will predict the state based on the acceleration measurement. We see the estimated state does not follow the true state.
Predict the current state forward by time dt using current accelerations and body rates as input. 
2. Second, We implement the state prediction step in the PredictState() functon in QuadEstimatorEKF.cpp. Then run scenario 08_PredictState, we see the estimator state track the actual state, with only reasonably slow drift.
3. Third, Run scenario 09_PredictionCov, this time we use  a realistic IMU, one with noise. we see the estimated covariance (white bounds) currently do not capture the growing errors.
4. Finally, We calculate the partial derivative of the body-to-global rotation matrix ( `RbgPrime`) in the function GetRbgPrime(), `RbgPrime` is then used in the calculation of Jacobian matrix, the key for performing covariace prediction in EKF.
Rerun scenario 09_PredictionCov, we can see the estimated covariance (white bounds) grows with errors. 

#### 4. Implement the magnetometer update.

The Implement of the magnetometer update has the follows steps.
1. Run scenario 10_MagUpdate with default code £¨without the magnetometer update). We see the estimate yaw is drifting away from the real value (and the estimated standard deviation is also increasing). 
2. Next,We implement the magnetometer update with the equations from section `7.3.2 Magnetometer` from the [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/) paper.
. After implement the update, then call `QuadEstimatorEKF::Update` to do the state update and covariance update. Please note the difference of measurement and estimated yaw must be normalized before the magnetometer update.
3. Rerun scenario 10_MagUpdate with the magnetometer update code. We can see the estimated standard deviation that accurately captures the yaw error and maintain an error of less than 0.1rad in heading for at least 10 seconds of the simulation. 

#### 5. Implement the GPS update.

The Implement of the GPS update has the follows steps.
1. Run scenario 11_GPSUpdate without the GPS update, but using both an ideal estimator and and ideal IMU. watch the position and velocity errors (bottom right). We see they are drifting away.
2. Change to using my estimator and the realistic IMU. Rerun the scenario without the GPS update, We see the drone goes wild from time to time as well, due to 
errors of position and velocity grows rapidly.
3. Implement the EKF GPS Update in the function `UpdateFromGPS()` with the equations from section `7.3.1 GPS` from the [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/) paper.
4. Rerun the scenario with the GPS update, We can see the position error, and sigma decreased.