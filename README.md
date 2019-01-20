# CarND-Controls-MPC
## Control the vehicle actuators using a Model Predictive Controller to drive a car around a track.

---

[//]: # (Image References)

[image1]: ./support/SimulatorStartup.png "Simulator Startup Window"
[image2]: ./support/MPC_simulator.png " MPC Simulator"
[image3]: ./support/MPC_running.png " MPC Running"
[image4]: ./support/MPC.gif " MPC Video Clip"
[image5]: ./support/HarshSteering1.png " Steering Cost Function = 1"
[image6]: ./support/SmoothSteering100.png " Steering Cost Function = 100"


## Overview
Implementation of a MPC controller in C++, which drives a vehicle around the lake race track in the Udacity simulator. Using a MPC controller the acceleration and steering actuators are controlled to manoeuvre the vehicle to follow the target ideal road line.

The following information passed from the simulator:  

    + Current Position in map coordinate system: vehicle x_position, vehicle y_position, vehicle heading angle psi.
    + Current velocity of vehicle
    + Current steering and throttle actuator values
    + A vector of points to indicate x and y positions on map of ideal road line

The MPC receives this information in the form of two vectors which describe the vehicle's target position and current position. 
Cost functions are assigned to states and actuator values. The cost function's bias the controller calculations to influence the output lateral and longitudinal corrections required to move the vehicle to the target position.
These corrections are converted into steering and accelerator actuator values and fed back into the Simulator.

---

## Installation steps

To run this code the following downloads are required:

1. Make a project directory `mkdir project_udacity && cd project_udacity`
2. Clone this repository into the project_udacity directory. `git clone https://github.com/nutmas/CarND-MPC-Project.git`
3. Setup environment. `cd CarND-MPC-Project\` and launch the script to install uWebSocketIO `./install-mac.sh`. Alternatively for Ubuntu installs launch `./install-ubuntu.sh`. The environment will be installed with these scripts.
4. Download Term 2 Simulator: 
      * Download from here: [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases).
      * Place the extracted folder into the project_MPC directory. 
      * Within the folder run simulator `term2_sim`; if successful the following window should appear:
      ![alt text][image1]

---

## Other Important Dependencies

* cmake >= 3.5
* make >= 4.1 
* gcc/g++ >= 5.4
* Ipopt 3.12.8
    - In order to obtain a successful Ipopt with openblas on Mac follow these instructions: From terminal
        + `brew tap nutmas/CarND-MPC-Project https://github.com/nutmas/CarND-MPC-Project`
        + `brew install ipopt --with-openblas`
        + `brew untap nutmas/CarND-MPC-Project`
* CppAD: `brew install cppad`

---

## Build the code

1. From the project_udacity folder change to folder created from cloning `cd CarND-MPC-Project`
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 

---

## Usage

After running the script, confirming the operation of the simulator and building the code the MPC program is ready to be run.

1. From terminal window; change to build folder of project `cd ~/project_udacity/CarND-MPC-Project/build/`
2. Run MPC: `./mpc `
3. Message appears `Listening to port 4567` the MPC is now running and waiting for connection from the simulator
4. Run the simulator `term2_sim`
5. In the simulator window, press 'Next' arrow four times to navigate to MPC Controller then press 'Select'.

![alt text][image2]

6. In the MPC terminal window `Connected!!!` will now appear.
7. The MPC is now fully operational, the vehicle will proceed to navigate around the track, controlling its own speed and steering autonomously.

The Animation will start and the vehicle can be seen moving around the screen.

![alt text][image3]

---

## Model Preparation

The information received from the simulator required preparation before being passed to the MPC.

#### MPC Input data preparation steps:
**Transform from Map coordinates to vehicle coordinates**.
The target path from the simulator is used to understand where the vehicle is in relation to each of those points and build a target path of points in a coordinate system reference to the vehicle position. `Lines 132-159 in main.cpp`

**Error Calculation**
The transformed points are fitted with a 3rd order polyline. The results of this function are used to calculate the vehicle cross track (cte) and heading (epsi) errors from the target path. `Lines 174-180 in main.cpp`

**State Vector**
The vehicle positional information (x,y,psi), velocity (v) and errors (cte, epsi) are added to the state vector. They are updated to calculate the next state using a bicycle model. The algorithms are included for reference in `Lines 184-194 in main.cpp` The state vector is updated for each state: `Lines 197-214 in main.cpp`

**MPC Solver**
The MPC function is called at `Line 220 in main.cpp`. The state vector and polynomial of target path is passed into the solver.

#### MPC Tuning:  
Three areas of the MPC required 'calibration' in order to produce the required results.  

  - **Define length of trajectory** - this has 2 parameters N & dt. N denotes the number of time steps and dt is the interval between steps. The product of these two values determines the horizon point or how far ahead the MPC is looking. I finally chose N = 10, dt = 0.05; which produces: `T = 10 * 0.05 = 0.5 second` This sets the MPC to have a horizon 0.5 second into the future. The guideline to find a tradeoff for these values is to make dt as small as possible, while making T as large as possible. However the larger N the slower the simulation will run. `Lines 9-10 in MPC.cpp`
  - **Define limitations** - restricting the actuator upper and lower bounds. It was set quite simply to:
    + Steering:  -25 to +25 degrees (-/+0.436 RAD) - which defines the limits the steering can move between. `Lines 215-220 in MPC.cpp`
    + Acceleration: -1.0 to +1.0 - setting -1.0 as full brake and +1.0 as max acceleration. `Lines 222-226 in MPC.cpp`
  - **Define Cost Functions** - The cost functions serve to influence the MPC to allow it to trade parameter values to find an optimum solution. In `Lines 74-99 in MPC.cpp` There are 7 algorithms that can set the costs the MPC must consider. The tuning of these functions can be set by a set of variables present in `Lines 28-35 in MPC.cpp`.

---

## Reflection

**N Time length and dt Interval **  

I experimented with some combinations of N and dt. I found that extending the total horizon time would lead to the MPC being very unstable as it was looking too far ahead, and the vehicle would often leave the track. I spent a lot of time using N=10, dt=0.1 to give a horizon of 1 second. Lots of my tuning was performed using these values, however over 60mph instability returned. I reduced the 1 second horizon by reducing the interval dt, putting 10 intervals into the 0.5 second horizon and this simple change allowed the vehicle to travel round the track at 80mph without changing any of the cost function parameter which were set for 60mph. I finally settled on these values for this project as they improved stability through the whole range of speeds form 30mph to 80mph.  *Final settings:* `N=10, dt=0.05`

**Latency**

To compensate for the latency in the system a variable is create in `Line 129 in main.cpp`. This is set at 0.1(100ms) to match the simulated latency in this simulator system. In `Lines 203-213 in main.cpp` the variable is calculated as the time interval with each state before being passed to the MPC.

When the ideal path and vehicle position was available form the simulator, the vehicle would be ahead of one or more points; this caused instability in the MPC. To ease the MPC calculation I checked to see how far the vehicle was ahead of the target, and any target point that was already passed I set to 0.0 as the vehicle coordinate system. `Line 149 in main.cpp` sets a variable to check if any transformed point would be behind the vehicle. `Lines 152-159 in main.cpp` will transform the point if it is in front of the vehicle otherwise set it to 0.0. This stabilised the MPC trajectory planning by only using forward points to predict the path.

**MPC Tuning Process**

Most of the tuning of the MPC is achieved by influencing the cost function variables in `Lines 29-35 in MPC.cpp`. 

To find a range of values that worked over a range of speeds I started at 20mph with all these parameters set to 1.0.

From the MPC class study it was found to get a smooth steering movement, the steer variation set to 100 gave good results. This was the first cost function parameter to be set and produced a result which would move the vehicle around the track. At very slow speed this worked.

**Steering Cost Function = 1** Transition has a 'sharp' transitions which results in severe steering movement changes.
![alt text][image5]
**Steering Cost Function = 100** Transition has a 'continuous' transition which results in smooth movement changes.
![alt text][image6]

In order to achieve higher speeds and better results from the model, I incremented the speed, tested the result in the simulator, and then modified the cost parameters where needed to keep the project requirements.

The end result is a model which can drive the vehicle around the track with a target speed set to 80mph, and drives at a stable 76mph.

Earlier I found if I set the cte and heading cost parameters very high along with the steer variation then the target speed could be set at 200mph and 91mph can be realised on the straight. However this forces the MPC to put all of its focus on perfectly matching the target path and therefore a lot of braking is applied slowing the vehicle to 50mph around the bends.
Some of the successful cost parameters used are shown as comments in `Lines 37-47 in MPC.cpp`.

I preferred the solution where the vehicle has a fairly consistent speed around the whole circuit and stays within the limits of the track.



---

## Results

The vehicle successfully navigates around the track. 
Criteria for success:

 `No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe.` 

 The following video shows a short duration of the vehicle driving around the track being controlled by the MPC. 
 The yellow line indicates the target ideal path, and the green line shows the trajectory the vehicle will follow with the actuator values set by MPC output.

![alt text][image4]

A video of the full lap can be viewed by downloading: [MPC_Video](./support/MPC_Video_80_480.mov)



---

## License

For License information please see the [LICENSE](./LICENSE) file for details

---

