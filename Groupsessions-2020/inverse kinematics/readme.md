## Repetition of Forward Kinematics using Denavit-Hartenberg convention approach
  * Work with an example task from a past exam
  
## Inverse Kinematics
* **Overview:**
  * The counterpart of the Forward Kinematics problems: given the link lengths and the position of the end-effector, find the joint variables
  * Arguably more useful than Forward Kinematics: we often have an idea about where we want the end-effector to be, but do not know what joint angles and extensions would place it there
  * More difficult than Forward Kinematics: the approaches are not straight-forward, often require to think out of the box
  
* **Important Considerations (number of solutions):**
  * Inverse Kinematics solutions are not always unique
  * Often, both an *elbow-up* and an *elbow-down* configuration is valid - yielding **two** solutions
  * Sometimes other configurations can exist when a manipulator has a revolute joint at the base, which lets it:
    * Simply reach forwards and get to the desired point
    * Turn away (do a 180 at the base) and reach bakwards to that same point
  * In certain cases (articulated manipulators), all combinations work: forwards/elbow-up, forwards/elbow-down, backwards/elbow-up, backwards/elbow-down - **four** solutions
  * *Singular configurations* or *singularities* occur for some points. There are **infinite** solutions for these points.
  * If a point is out of reach or violates constraints, there are **zero** solutions.
  
* **Required knowledge (trigonometry/geometry):**
  * Law of cosines: <span><strong>c<sup>2</sup> = a<sup>2</sup> + b<sup>2</sup> - 2ab*cos(&alpha;)</strong></span>
  * Sine (sin), cosine (cos), *tangent* (tan) and the corresponding inverse functions: arcsine, arccosine and arctangent - know the meaning, definitions and values for commonly used angles/lengths
  * tan(&alpha;) = opposite/adjacent = sin(&alpha;)/cos(&alpha;)
  * Pythagorean identity: sin<sup>2</sup>(&alpha;) + cos<sup>2</sup>(&alpha;) = 1  ----->  sin(&alpha;) = sqrt(1 - cos<sup>2</sup>(&alpha;))
  * Arctangent (arctan or <span>tan<sup>-1</sup></span>) is replaced by **atan2** function
  * **atan2(x,y)** is *multi-valued arctangent*, gives different results depending on whether **x** and **y** are positive or negative
  * If **x** and **y** are both positive, **atan2(x,y) = arctan(y/x) [ONLY IN THE BOOK/SLIDES]**
  * Different programming languages have different implementations. **Python implementation: atan2(x,y) = arctan(x/y)**
  * Be careful, check documentation and specify what implementation you are using
  * Upon finding cos of an angle, **DO NOT** use cos<sup>-1</sup> to get the angle. **Always use atan2(&plusmn;sin/cos)** to get all solutions (find sin from cos using Pythagorean identity).
  
* **Approaches to Inverse Kinematics:**
  * Inverse kinematics problems can be solved using several approaches. The main ones are
  * **Geometric approach** (projecting the manipulator onto various planes)
    * Look at the manipulator from the *side view* (normally **X0,Z0 plane**), pretend that it is two-dimensional. From there, identify as many equations as you can
    * Look at the manipulator from the *top view* (normally **X0,Y0 plane**) and repeat the procedure
    * Solve the equations for the joint variables: for revolute joints, *law of cosines* and *atan2* are often enough to find the angles; for prismatic joints, basic geometry and common sense get the job done
    * Be on look out for places to use trigonometric identites, introduce new angles/variables when needed
    * Common choices for new variables:
      * <span><strong>s = z - L<sub>1</sub></strong></span> (height of the end-effector without the first link)
      * <span><strong>r<sup>2</sup> = x<sup>2</sup> + y<sup>2</sup></strong></span>
  * **Analytic/algebraic approach** (reusing forward kinematics)
    * Take the transformation matrix from the base to the end-effector and break it down into a set of equations, solve for the joint variables. [INSERT EXAMPLE]
    * General procedure for a 3 DOF manipulator:
      ![Alt text](algebraic_random.png?raw=true)
    * The resulting system is often difficult to solve. Possible ways to simplify things: 
      * trigonometric identities
      * cos<sup>2</sup>(&theta;) + sin<sup>2</sup>(&theta;) = 1 
      * summing two equations
      * subtracting one from another
      * summing AND squaring
      * multiplying an equation by a constant (cos or sin of angle, for example)
      * packing terms into new variables
      * any combination of the above
  
## Schedule for IK session 1:
* Menti to track progress (40 02 02)
* [Exam 2018, 2a](https://github.uio.no/INF3480/Groupsessions/blob/2019/forward%20kinematics/exam2018_task2a.PNG)
* [Co-op task 2016/2011](https://github.uio.no/INF3480/Groupsessions/blob/2019/forward%20kinematics/)
* BREAK
* Overview of inverse kinematics
* Task 3.11 and 3.12

## Schedule for IK session 2:
* Overview of approaches (2 min)
* Algebraic approach (20 min)
* [2018 exam, task 2d) , algebraic solution](https://github.uio.no/INF3480/Groupsessions/blob/2019/inverse%20kinematics/algebraic-solution_inverse-kinematics_2018.pdf)
* Geometric approach (10 min)
* BREAK
* Task 4 of assignment 1 (40 min)
* Questions for assignment 1 (?? min)
