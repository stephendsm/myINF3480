
# Dynamics
*Read ![dynamics_overview.pdf](https://github.uio.no/INF3480/Groupsessions/blob/2020/dynamics/dynamics_overview.pdf) for a gentle introduction to manipulator dynamics!*

## Prerequisites
* Newton's second law (general): **F = ma**
* Kinetic energy (general): <strong>K = 1/2*mv<sup>2</sup></strong>
* Potential energy due to Earth's gravity (general): **P = mgh**
* Double dot over a variable - second order time derivative: **&#7821;** - acceleration in x-direction
* Joint acceleration vector: **q&#776;** (contains a **d&#776;** for every prismatic joint and a **&theta;&#776;** for every revolute joint)
* Differentiation rules:

![Alt text](diff_rules.png?raw=true)

## Kinetic Energy
is energy associated with motion. 

Kinetic energy is usually a sum of two terms: Translational and Rotational Kinetic Energy. See formula below.
![Alt text](images/kinetic-energy.png?raw=true)
, where *v* is translational velocity and *omega* is rotational velocity.

### Kinetic Energy for an n-Link manipulator robot
We want the only variables of our kinetic energy equation to be the joint variables (q-vector).
This is done by including the velocity kinematics Jacobian. See formula below.
![Alt text](images/kinetic-nLink.png?raw=true)

*I* is the constant moment-of-inertia matrix. It is found by doing triple integrals over a coordinate frame with origin in the mass-center of the rigid body that is the respective link, this frame is called the **body-attached-frame**.
The body-attached-frame need to be represented in the intertial(base) frame, and this is done with *R<sub>i</sub>(q)*. 

#### R<sub>i</sub>(q)
The rotational matrix need to account for the change in **all** joints before the respective link-*i*. 
If the body-attached-frame is aligned with the frame of joint-*i-1* then the forward kinematics up to joint-*i-1* can be used, and only the rotation in joint-*i-1* need to be accounted for. <br>
   *Example: T<sup>0</sup><sub>i-1</sub>R<sub>z<sub>i-1</sub>,theta<sub>i-1</sub></sub>* <br>
Similarly the forward kinematics up to joint-*i* could be used alone if that frame also is aligned with the body-attached-frame. This can be used alone as the rotation in joint-*i-1* is already accounted for in T<sup>0</sup><sub>i</sub>



## Schedule for second group session of dynamics
* Present solution to assignment 2.
* Get started on assignment 3. 
    * Read page 255 to 262. 
