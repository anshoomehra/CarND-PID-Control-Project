# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

PID Controllers

A proportional–integral–derivative controller (PID controller) is a control loop feedback mechanism (controller) commonly used in industrial control systems. A PID controller continuously calculates an error value {\displaystyle e(t)} e(t) as the difference between a desired setpoint and a measured process variable and applies a correction based on proportional, integral, and derivative terms (sometimes denoted P, I, and D respectively) which give their name to the controller type.

In this project, we are attempting to make use of PID controller to effectively compute steering measurements to drive the simulation vehcile developed in Unity to stay on track. Simulator produces the error signal as the distance between the actual car position on the road and a reference trajectory, known as cross-track error (cte). The PID controller is designed to minimize the distance to this reference trajectory.

P - Proportional Gain

The proportional term computes an output proportional to the cross-track error. P - controller alone may not give desired result and will oscillates about the target, as meeting linear target to reference will either overshoot or stay under. In this project, the proportional gain contributes a control output to the steering angle of the form -K_p cte with a positive constant K_p.

D - Differential Gain

The oscillations effect by P_conroller can be mitigated by a term proportional to the derivative of the cross-track error. The derivative gain contributes a control output of the form -K_d d/dt cte, with a positive constant K_d.

I - Integral Gain

There could be possible biases in real-life scenarios like wheel drift, miss alignment becuase of road conditions and/or mechanical faults. In real life, we steer harder/counter on these situations to keep vehicle on track, in our project third contribution of integral gain can help offset these sitiation. This approach sums up the cross-track error over time, & corresponding contribution to the steering angle is given by -K_i sum(cte).

Hyperparameter Tuning

All parameters were tuned manually to have better understanding of how each parameter affect the outcomes. Approach followed was simple and organic evolution of PID params as stated below:

* Started with PID as 0,0,0, to see cte produced & behaviour.
* Started playing with just P, keeping I,D as zeros.
* Tried aggresive value of 0.5 starts to drive vehicle but soon catch into bad osciallates and being driven out of track.
* After few tries, P of 0.1 seems to have been giving decent result and vehcile to osciallte on sharp turns.
* At this stage, introduced D to dampen oscialltions, started with values in contrast of P, .1  then .001 and then .0001, .0001 gave a decent stable performance and also indicactor of proportinality these parameters to be dealt with.
* At this stage, vehicle was running fairly well, but will go out of track at 2 very sharp turns (I have not used braking logic, was trying to keep low speed and attempt to manage control with param optimizations.
* It was perhaps logical to introduce I-gain, to offset error introduced by change in trajectory on sharp turns. After some trial-error value of 0.6 made the vehcile run full loop.
* This was fairly confident stage to expriment with params to smoothen out run and try to keep vehicle within bounds. Below is the summary of my exploration .. 

  // TODO: Initialize the pid variable.
  // First sucess run manual observations: pid.Init(0.1, 0.0001,  0.5);
     // Almost smooth @ pid.Init(0.1, 0.0001,  0.6);
      // Getting there @ pid.Init(0.1, 0.0001,  0.7);
        // Perhaps we can improve corners a bit more @ pid.Init(0.1, 0.0002,  0.9);
          // I guess I am hapy with  this !! pid.Init(0.09, 0.0002,  1.5);
          // And this : pid.Init(0.1, 0.0002,  1.5);
          // VStill touching yellow lines -- pid.Init(0.15, 0.0002,  2.0);
          // Now with in yellow lines : pid.Init(0.20, 0.0002,  3.0);
          // Bit jerky on sharp turns: pid.Init(0.20, 0.0003,  3.0);
  // Let's stop here .. :-) & try Twiddle !!
  pid.Init(0.20, 0.0003,  3.5);

Observation:
When I tried to create video of outcome for submission, I realized that, high CPU consumption impacted performance and subsequently outcomes, i.e vehicle was not as smooth as prior run. I end up using my phone to record, but something to keep in mind that same params may or mat noy work on different computers and when designed for real use, CPU clock rate must be kept in consideratios for optimizations.

Opportunities to further improve:
* Twiddle, I did not use it becuase of time contraint but would like to try and see how best it can help optimize. I am really impressed with the idea though. 
* Increasing speed will be good challenge and will require adpative throttle, braking as in real-life.
* Smooth control on sharp turns, I think above two may significantly help & even more so have additoonal bias introduction based on error may help .. 

## Video Capture: https://youtu.be/pU-hx5PEhwc

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
