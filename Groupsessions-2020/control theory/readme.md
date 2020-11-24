# Control Theory

## Independent Joint Model
* Controlling a whole manipulator is fairly difficult - we focus on controlling only one joint instead
* All interaction between joints (dynamic coupling) will be classified as noise

## Laplace Transform
* Takes time-dependent functions (for example, electric signals) from **time domain** to **frequency domain**
* Equations in time domain are often **differential**, they become normal algebraic equations after transformation
* Differentiation in time -> multiplication in Laplace
* Integration in time -> division in Laplace

![Alt text](laplace_transform.png?raw=true)

## Transfer functions
* **Y(s) = H(s)X(s)**
* Y(s) - system output
* X(s) - system input
* H(s) - **transfer function**, transforms/"transfers" input into output
* Rearrange to find the transfer function: **H(s) = Y(s)/X(s)**
* If numerator (Y) is set to 0, can get **poles** of the system by solving for **s**
* If denominator (X) is set to 0, can get **zeros** of the system by solving for **s**

## Poles and zeros

## Dynamic Model of a Robot
![Alt text](dynamic_model.png?raw=true)
* **Not** the same notations as in velocity kinematics: **J is not Jacobian, D matrix is not the one from dynamics**
* Inertia and inertial forces are **not** the same thing, they are loosely related, if at all
* Coriolis and centrifugal forces are often classified as inertial (fictitious) forces

## Block Diagrams
* Confusing at first, but helpful in visualizing equations and feedback/feedforward loops
* Block diagram for the generic system outlined above:
![Alt text](basic_block.png?raw=true)
* On the left side, we start with the full torque equation denoted as **U** (called **control effort**)
* As the signal proceeds through the blocks, we start "losing" terms one by one, until we end up with only &theta;
* The whole diagram represents a robotic system - it is used as a building block in larger diagrams with controller blocks

## Setpoint Controllers
* Controllers that drive a robot to a set point
* Current angle: <strong>&theta;</strong>
* Desired angle: <strong>&theta;<sub>d</sub></strong>
* **Error:** <strong>e(t) = &theta;<sub>d</sub> - &theta;</strong>
* Controllers use the error to calculate **control effort U** - the output torque/force
* Controllers try to reduce error to 0
* Types of setpoint controllers:
  * **P (proportional)** controllers
  * **PD (proportional derivative)** controllers
  * **PI (proportional integral)** controllers
  * **PID (proportional integral derivative)** controllers

## P Controller
* <strong>U(t) = K<sub>p</sub>e(t)</strong>
* Control effort **P**roportional to the error
* Laplace transform: <strong>U(s) = K<sub>p</sub>E(s)</strong>
* Block diagram:
![Alt text](p_block.png?raw=true)
* Increased K<sub>p</sub> gives:
  * Faster response
  * Decrease in steady state error
  * Increased oscillations
* Proportional term by itself will not eliminate the error

## PD Controller
* Builds up on P controllers by adding a **D**erivative term
* Derivative term represents how fast the error changes - used to "predict" future error
* <strong>U(t) = K<sub>p</sub>e(t) + K<sub>d</sub>e&#775;(t)</strong>
* Laplace transform: <strong>U(s) = K<sub>p</sub>(&theta;<sub>d</sub> - &theta;) - K<sub>d</sub>s&theta;</strong>
* Block diagram:
![Alt text](pd_block.png?raw=true)
* Increased K<sub>d</sub> reduces oscillations, but can make the robot stop before reaching the desired point

## PID Controller
* Builds up on PD controllers by adding an **I**ntegral term
* Integral term "accumulates" past errors over time
* <strong>U(t) = K<sub>p</sub>e(t) + K<sub>d</sub>e&#775;(t) + K<sub>i</sub> &#8747;e(t)dt</strong>
* Laplace transform: <strong>U(s) = (K<sub>p</sub> + K<sub>d</sub>s + K<sub>i</sub>*1/s)E(s)</strong>
* Block diagram:
![Alt text](pid_block.png?raw=true)
* Good choice of K<sub>i</sub> gives the necessary push to prevent early stopping due to the derivative term, eliminating the error
* If K<sub>i</sub> is too big, expect an overshoot, oscillations and instability

## Schedule for CT session 1
* Dynamics example: exam 2018 task 3.
* Work with assignment 3.
* BREAK!
* Assignment 3.
* Gentle Introduction to Control Theory
