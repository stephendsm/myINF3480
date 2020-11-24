# Velocity Kinematics

## Prerequisite Knowledge
* Two types of velocities: linear and angular. Linear velocity is velocity along a line (e.g. 2 m/s in z-direction), angular velocity is a change in angle (e.g. 1 rad/s)
* **q** - vector of joint variables, has a <strong>&theta;</strong> for every revolute joint and a <strong>d</strong> for every prismatic joint 
* Dot above a variable denotes time derivative of that variable. This means that <strong>x&#775;</strong> is velocity in x-direction
* <strong>q&#775;</strong> - vector of joint velocities, has a <strong>&theta;&#775;</strong> for every revolute joint and a <strong>&#7691;</strong> for every prismatic joint
* Calculating cross product:
![Alt text](cross_product.jpg?raw=true)

### Velocity
* The velocity of an object is the rate of change of its position with respect to a frame of reference, and is a function of time.[<sup>1</sup>](https://en.wikipedia.org/wiki/Velocity) In other words: It is distance over time.
![explanation of rigid body velocity](https://github.uio.no/INF3480/Groupsessions/blob/2019/velocity%20kinematics/rigid%20body%20velocity.png)

## End-effector velocity
* <strong>&xi; = Jq&#775;</strong> is the main formula, relating velocities of joints to velocities of the end-effector
* <strong>&xi;</strong> (xi) - vector of end-effector velocities, it breaks down into linear velocities (three elements, one for each axis) and angular velocities (three more elements). Number of joints does not matter - <strong>&xi;</strong> is always of size six
* <strong>J</strong> - the Jacobian matrix. Works roughly the same way as the one from vector calculus. In this course, it is a transformation from joint velocities to end-effector velocities
* The Jacobian can also be broken down into linear component/matrix (<strong>3 x Number of joints</strong>) and angular component/matrix (again, <strong>3 x Number of joints</strong>). The total size is always <strong>6 x Number of joints</strong>.

## Deriving Jacobian
![Alt text](jacobian_calc.jpg?raw=true)

Below is a figure that can be helpful for understanding why we use the cross product when finding the Jacobian.
![Figure to explain how we get the formulas for Jacobian](https://github.uio.no/INF3480/Groupsessions/blob/2019/velocity%20kinematics/figure_explain_jacobian.png)
## Singularities
* Singularities are configuartions from which motion in some directions is impossible. In these configuartions, bounded **end-effector** velocities/forces may correspond to unbounded **joint** velocities/forces
* Does not mean that moving the tip will cause joints to fly off into the stratosphere at superluminal speeds, but the joints might strain and break.
  * Link to vizualisation at Studywolf: [https://studywolf.wordpress.com/2013/10/10/robot-control-part-6-handling-singularities/](https://studywolf.wordpress.com/2013/10/10/robot-control-part-6-handling-singularities/)
  * Link to vizualisation on a real robot: [https://youtu.be/1zTDmiDjDOA](https://youtu.be/1zTDmiDjDOA)
* Mathematically, singularities are configurations for which the **rank** of **J(q)** is less than maximum (when it has linearly dependent vectors) - This is from linear algebra, and a full understanding is not needed for this course. 
  * Link to a great explanation on rank and zero-determinants, by 3Blue1Brown: [https://youtu.be/uQhTuRlWMxw](https://youtu.be/uQhTuRlWMxw) 
  * Maximum rank of a matrix:
    * Number of rows, if there are more rows than columns
    * Number of columns, if there are more columns than rows
  * Actual rank of a matrix:
    * Number of linearly **independent** rows, if there are more rows than columns (any linearly **dependent** rows - singularity)
    * Number of linearly **independent** columns, if there are more columns than rows (any linearly **dependent** columns - singularity)
* Find singularities by solving |J| = 0 for joint variables
* Decoupling of singularities - the principle that allows separate calculation of arm singularities and wrist singularities
  * Practical consequence: <strong>arm singularities can be calculated by solving only |J<sub>v</sub>| = 0</strong>

### Chapter 4.9. Page 142
* The five points on this page have to be handled by our code. Partly by restrictions from creating points at outer boundary or inside inner boundary, and partly by collision detection. If you create a trajectory function which does not consider the inner boundary, you'll have to make sure you handle the singularity caused by theta3 = 180 degrees. 

## Example
Task 2d from the exam of 2014 is used as an example. Below are the task and the solution.
![Task 2d exam 2014](https://github.uio.no/INF3480/Groupsessions/blob/2019/velocity%20kinematics/exam2014_velocity_kinematics_task_2d.PNG)
![Solution 2d exam 2014](https://github.uio.no/INF3480/Groupsessions/blob/2019/velocity%20kinematics/exam2014_velocity_kinematics_solution.PNG)


## Schedule for session 2
* Present parts of solution to assignment 1.
* Work on assignment 2.
  * Present details about assignment 2 theory?
* In-depth theory about singularities.
  * Chapter 4.9 Page 142
