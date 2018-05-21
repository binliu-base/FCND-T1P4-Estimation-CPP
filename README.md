# FCND-Term1-P4-Estimation

### 1. Project Overview

The goal of this project is to design and build the estimation portion of the controller used in the CPP simulator, which will control the quadrotors to fly the desired trajectory in the 3D environment with your estimator and your custom controller (from the previous project). 


### 2. Breakdown of Implement Estimator (Project Rubric)

#### 1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

The process of determine the standard deviation of the measurement noise consists of the following steps

1. Data collection, I turn on log tracing by adding the following two commands in the config file (06_SensorNoise.txt) of the scenario.
```
Commands += AddGraph1.LogToFile
Commands += AddGraph2.LogToFile
```
Next,  I launch a run for the scenario (Sensor Noise) in the simulator and got two log files. One of log file is the measurement of GPS x location, another is acceleometer x velocity.

2. Write a data processing script (./scripts/process_log.py) 
3. With the data processing script, I can easily figure out the standard deviation of the measurements: MeasuredStdDev_GPSPosXY should be around 0.71, and MeasuredStdDev_AccelXY should be around 0.49.

#### 2. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.

#### 3. Implement all of the elements of the prediction step for the estimator.

#### 4. Implement the magnetometer update.

#### 5. Implement the GPS update.