# Estimation Project #

Welcome to the estimation project.  In this project, a simulated quad will be flying with our developed estimator and custom controller.

## Implement Estimator ##

The estimator is built up in pieces.  At each step, there will be a set of success criteria that will be displayed both in the plots and in the terminal output.

### Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data. ###

A quick python program was written to read the csv files `config/log/Graph1.txt` (GPS X data) and `config/log/Graph2.txt` (Accelerometer X data) using the built-in function `np.loadtxt`. A while loop was implemented to ensure that there were at least 99 GPS data points in a stored array before using the built-in `np.std` to calculate the standard deviation. I experimented with Scenario 6 to determine 99 GPS points and the corresponding number of IMU points are the maximum provided before the scenario restarts. Calculated values for `MeasuredStdDev_GPSPosXY` and `MeasuredStdDev_AccelXY` are `0.71` and `0.49`, respectively. 

### Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function. ###

In Section 7.1.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) a non-linear complimentary filter for attitude using quarternions is described. It is implemented in `Lines 96-101` of `QuadEstimatorEKF.cpp` in the function `UpdateFromIMU()`. The `Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6))` function is used on `Line 96` to create the quarternion of the current estimated state and the following line (`q_t.IntegrateBodyRate(gyro, dtIMU)`) implements the integration using the body rate measurements from the gyro. This implementation reduced attitude errors to get within 0.1 rad for each of the Euler angles, as shown in the screenshot below.

![attitude example](images/attitude-screenshot.png)

In the screenshot above the attitude estimation using linear scheme (left) and using the improved nonlinear scheme (right). Note that Y axis on error is much greater on left.

### Implement all of the elements of the prediction step for the estimator. ###

In `QuadEstimatorEKF.cpp`, the state prediction step in the `PredictState()` functon is implemented in `Lines 163-169`. An attitude quarternion that was created for us using the current vehicle estimated attitude is used to rotate the accelerometer measurements from body frame to inertial frame (`attitude.Rotate_BtoI(accel)`). At this point the first 6 variables of the predicted state vector are updated and gravity was accounted for when updating the vertical velocity. Scenario `08_PredictState` will then show the estimator state track the actual state, with only reasonably slow drift, as shown in the figure below:

![predict drift](images/predict-slow-drift.png)

In `QuadEstimatorEKF.cpp`, calculation for the partial derivative of the body-to-global rotation matrix in the function `GetRbgPrime()` is implemented in `Lines 195-197`. The partial derivatives were written as described in section 7.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj). The state covariance is predicted forward in `Predict()` which is implemented in `Lines 243-255` in accordance with the classic EKF prediction equations. Scenario `09_PredictionCov` is then ran with `QPosXYStd = 0.1` and `QVelXYStd = 0.18` process parameters in `QuadEstimatorEKF.txt` to capture the magnitude of the error. The solution looks as follows:

![good covariance](images/predict-good-cov.png)

Looking at this result, you can see that in the first part of the plot, our covariance (the white line) grows very much like the data.

### Implement the magnetometer update. ###

Up until now we've only used the accelerometer and gyro for our state estimation.  In this step, you will be adding the information from the magnetometer to improve your filter's performance in estimating the vehicle's heading.

1. Run scenario `10_MagUpdate`.  This scenario uses a realistic IMU, but the magnetometer update hasn’t been implemented yet. As a result, you will notice that the estimate yaw is drifting away from the real value (and the estimated standard deviation is also increasing).  Note that in this case the plot is showing you the estimated yaw error (`quad.est.e.yaw`), which is drifting away from zero as the simulation runs.  You should also see the estimated standard deviation of that state (white boundary) is also increasing.

2. Tune the parameter `QYawStd` (`QuadEstimatorEKF.txt`) for the QuadEstimatorEKF so that it approximately captures the magnitude of the drift, as demonstrated here:

![mag drift](images/mag-drift.png)

3. Implement magnetometer update in the function `UpdateFromMag()`.  Once completed, you should see a resulting plot similar to this one:

![mag good](images/mag-good-solution.png)

***Success criteria:*** *Your goal is to both have an estimated standard deviation that accurately captures the error and maintain an error of less than 0.1rad in heading for at least 10 seconds of the simulation.*

**Hint: after implementing the magnetometer update, you may have to once again tune the parameter `QYawStd` to better balance between the long term drift and short-time noise from the magnetometer.**

**Hint: see section 7.3.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for a refresher on the magnetometer update.**


### Implement the GPS update. ###

1. Run scenario `11_GPSUpdate`.  At the moment this scenario is using both an ideal estimator and and ideal IMU.  Even with these ideal elements, watch the position and velocity errors (bottom right). As you see they are drifting away, since GPS update is not yet implemented.

2. Let's change to using your estimator by setting `Quad.UseIdealEstimator` to 0 in `config/11_GPSUpdate.txt`.  Rerun the scenario to get an idea of how well your estimator work with an ideal IMU.

3. Now repeat with realistic IMU by commenting out these lines in `config/11_GPSUpdate.txt`:
```
#SimIMU.AccelStd = 0,0,0
#SimIMU.GyroStd = 0,0,0
```

4. Tune the process noise model in `QuadEstimatorEKF.txt` to try to approximately capture the error you see with the estimated uncertainty (standard deviation) of the filter.

5. Implement the EKF GPS Update in the function `UpdateFromGPS()`.

6. Now once again re-run the simulation.  Your objective is to complete the entire simulation cycle with estimated position error of < 1m (you’ll see a green box over the bottom graph if you succeed).  You may want to try experimenting with the GPS update parameters to try and get better performance.

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated position error of < 1m.*

**Hint: see section 7.3.1 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for a refresher on the GPS update.**

At this point, congratulations on having a working estimator!

### Step 6: Adding Your Controller ###

Up to this point, we have been working with a controller that has been relaxed to work with an estimated state instead of a real state.  So now, you will see how well your controller performs and de-tune your controller accordingly.

1. Replace `QuadController.cpp` with the controller you wrote in the last project.

2. Replace `QuadControlParams.txt` with the control parameters you came up with in the last project.

3. Run scenario `11_GPSUpdate`. If your controller crashes immediately do not panic. Flying from an estimated state (even with ideal sensors) is very different from flying with ideal pose. You may need to de-tune your controller. Decrease the position and velocity gains (we’ve seen about 30% detuning being effective) to stabilize it.  Your goal is to once again complete the entire simulation cycle with an estimated position error of < 1m.

**Hint: you may find it easiest to do your de-tuning as a 2 step process by reverting to ideal sensors and de-tuning under those conditions first.**

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated position error of < 1m.*
