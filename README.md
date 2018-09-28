# **Model Predictive Control** 
---

This project is the fifth project of Udacity Self-driving Car Nanodegree Term2. The goals / steps of this project are the following:

* Use a model predictive controller to control the handling and throttle of the car on the simulator
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[video1]: ./results/MPC_80mph.mp4 "MPC controller"

---
### Describe the model in detail

#### 1. the states
The states include the x position(x), the y position(y), the orientation(psi), the velocity(v), the cross track error(cte) and the orientation error(epsi).
Those elements are all reference to car perspective coordinate.

#### 2. the actuators
The actuators include the steering angle(delta) and the throttle(or the brake), namely the acceleration(a).

#### 3. the update equations
The update equations are as the following. Notice that "dt" is the duration and "LF" is the distance between the front of the vehicle and its center of gravity.

```
x1 = x0 + v0 * cos(psi0) * dt

y1 = y0 + v0 * sin(psi0) * dt

psi1 = psi0 + v0 * delta0 / Lf * dt

v1 = v0 + a0 * dt;

cte1 = cte0 + v0 * sin(epsi0) * dt

epsi1 = epsi0 + v0 * delta0 / Lf * dt
```

### How the N (timestep length) and dt (elapsed duration between timesteps) values were chosen
The N and dt combination decides how fast the car can drive around the track safely. If speed of 30 mph is desired, there could many options we could choose. But my target is the speed limit, 100 mph. 
#### 1. N (timestep length)
This parameter decides how much time would be consumed by the solver. On my computer, if N is set as 6, the consumed time is about 0.08 seconds. If N is set as 20, the consumed time would be about 0.025 seconds. But if N is set as 30, the consumed time would skyrocket to 0.1 seconds, which will let the car very unstable and uncontrollable. But N less than 6 can not control the car. For speed of 30 mph and dt of value 0.1, N of value from 6 to 20 all have good performance. But for speed of 90 mph, I have the only one option, which is N set as 6 and dt set as 0.1 seconds. Unfortunately I can not find a combination for 100 mph.

#### 2. dt (elapsed duration between timesteps)
Too large dt will cause the predicted car positions to deviate from the desired trajectory. But too small dt will cause the car to response to the environment change too slowly. For speed of 30mph, dt of value larger than 0.3 seconds would generate deviation obviously. Conversely, dt of value less than 0.05 seconds would let the car ride on the curbs easily. But for speed of 90 mph, similar to the choosing of N, I also have very narrow span of the dt I could choose, that is about 0.08 to 0.12 seconds. 


### Polynomial Fitting and MPC Preprocessing
Before starting the execution of the solver each time when the new states and way points are received, the following steps will be done.

First, transform the way points to those reference to the current car perspective coordinate.

Second, use a 3 order polynomial to fit the way points.

Third, reference to the current car perspective coordinate, calculate the state of the car and then pass the state and the 3 order polynomial to the solver.

### Model Predictive Control with Latency
As mentioned third step above, according to the current state and the delay time, the state would be updated and then passed to the solver. Again, the state is reference to the current car perspective coordinate.

```
px_delay = v * TIME_DELAY

py_delay = 0;

psi_delay = v * delta / LF * TIME_DELAY

v_delay = v + a * TIME_DELAY;

epsi_delay = epsi + v * delta / LF * TIME_DELAY

cte_delay = cte + v * sin(epsi) * TIME_DELAY
```

About cte and epsi, considering the fitting polynomial

```
y(x) = coeffs[3]*x^3 + coeffs[2]*x^2 + coeffs[1]*x + coeffs[0]

epsi = 0 - atan(coeffs[1])

cte = y(0) - 0
```

The final result is driving a lap around the track at the speed controlled at 80 mph, as the video.
[video1]





