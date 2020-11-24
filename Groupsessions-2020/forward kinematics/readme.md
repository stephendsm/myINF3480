# Forward Kinematics

## Schedule:
* Introduction to the Denavit-Hartenberg convention
* Rules for setting up coordinate systems (including the gripper frame) as per the DH convention
  * Explanation of the rules
  * Example: cylindrical manipulator
  * Task: set up coordinate systems for a manipulator from 2018 exam
* The four DH parameters
  * Explanation of the parameters
  * Example: parameter table for cylindrical manipulator
  * Task: write up the parameter table for the exam 2018 manipulator
* ------ BREAK ------
* The special matrix for forward kinematics (p.77 in the book)
  * Introduction of the matrix and its composition
  * Example: finish calculating forward kinematics of the cylindrical manipulator (**two first joints only**)
  * Task: finish calculating forward kinematics of the manipulator from exam 2018 (**two first joints only**)
* Work with Task 2 in Assignment 1 and general help

## Information:
* **Introduction to DH convention (what/how/why)**
  - A way to calculate forward kinematics
  - The approach requires correctly setting up coordinate frames and extracting parameters

* **Rules for setting up coordinate frames (0 - current frame, 1 - next frame):**
  1. **Z** is along the axis of joint action (rotation for revolute, extension for prismatic)
  2. **X1** is perpendicular to both **Z0** and **Z1**.
  3. **X1** intersects **Z0** (move the origin if it doesn't).
  4. **Y** follows the right hand rule.
  
* **DH parameters (0 - current frame , 1 - next frame):**
  1. **a**: link length, distance between **Z0** and **Z1** along **X1** (rotate the **current** frame, **NOT** the next frame).
  2. **α**: link twist, angle between **Z0** and **Z1** about **X1** (rotate the **current** frame, **NOT** the next frame).
  3. **d**: link offset, distance between **X0** and **X1** along **Z0** (+ joint variable if prismatic).
  4. **θ**: joint angle, angle between **X0** and **X1** about **Z0** (+ joint variable if revolute).
  
* **Practice**
  - Use a custom example or a specific task

* **Forward Kinematics matrix with DH parameters**
  - Special matrix to paste DH parameters into (*formula booklet or p. 77 in the book*)

## DH Practice example: Exam 2018, task 2a 
  ![exam2018 manipulator](https://github.uio.no/INF3480/Groupsessions/blob/2019/forward%20kinematics/exam2018_task2a.PNG)
  **This text is written as a step-by-step guide on how to use the Denavit-Hartenberg convention with vizualisation to solve forward kinematics problems. Pick up a pen and paper and follow the instructions.** 
  
  *(Visualization trick: While following the steps, try to visualize that your right hand is moving along the configuration's lines, and that you are only allowed to move using a sequence of four steps: 
    1. rotation about your thumb, 
    2. translation along your thumb, 
    3. translation along your index finger and 
    4. rotation along your index finger. <br/>
    Your goal is to move from the base of the robot to the tip of the robot, aligning your thumb with the rotational/translational axes at least once for each joint along the way.)*
  
  We want to assign coordinate frames to the manipulator, and values to the DH-parameters in the DH-table. 
  1. Start by assigning z-axes (subscript still unknown) to all joints. **Remember: z-axis is supposed to lie along the rotational/translational axis!**
  [INSERT FIGURE]
  
  2. First iteration of DH! Place your right hand in the same orientation as the first frame of reference. In this case, the first frame is coordinate-frame zero. **Remember the right-hand-rule.** <br/>
   Now we want to get the next z-axis (z<sub>1</sub>) aligned with the rotational axis of joint 2 by doing transformations in the order of the DH-convention (**Rotation about z, Translation about z, Tx, Rx**).  <br/>
  **Note:** If we don't get the next z-axis aligned with the rotational axis of joint 2 we will have to do another iteration of the DH-convention( Rz,Tz,Tx,Rx).  <br/> <br/>
   To align z<sub>1</sub> with rotational axis of joint 2, the easiest choice of parameter values are:
      1. Recognize that in order to orient the z-axis corretly later on, we will need to have the x-axis in the same orientation as x<sub>0</sub>: We do 0 degrees rotation about the z-axis.
      2. Translate the length L1 along the z<sub>0</sub>.
      3. Translate the length 0 along the new x-axis. (note: here new x equals old x) 
      4. -90 degrees rotation about new x. 
     (**Note:** We don't move z<sub>1</sub> (and coordinate frame 1) into the position of joint 2, because we've already reached or goal of aligning z<sub>1</sub> with the rotational axis of joint 2. Getting to the correct position (AND the correct orientation) would require extra iterations.)
    [INSERT FIGURE] <br/><br/>
  This yields the first line in the DH-table: <br/>  
  
     | | &theta; <sub>i</sub>      | d<sub>i</sub> | a<sub>i</sub> | &alpha;<sub>i</sub> |
     |--- | --- | --- | --- | --- |
     | A1 | &theta;<sub>1</sub>  | L1            | 0             | -&pi;/2 |            


  3. Second iteration: Our goal is to get the z<sub>2</sub>-axis aligned with the translational axis of joint 3. 
    We do transformations following the DH-convention:
      1. Recognize that we want to move up L2 after moving along Loff. To do this in one iteration we have to use both translations. So we align the next x-axis along L2: Rotate -90 degrees about z<sub>2</sub>.
      2. Translate the length Loff along the z-axis. 
      3. Translate the length L2 along the new x-axis. 
      4. We reach our goal of aligning z<sub>2</sub> with the translational axis by rotating -90 degrees about the new x-axis. 
          [INSERT FIGURE] <br/><br/>
  This yields the second line in the DH-table: <br/>    
  
     | | &theta; <sub>i</sub>      | d<sub>i</sub> | a<sub>i</sub> | &alpha;<sub>i</sub> |
     |--- | --- | --- | --- | --- |
     | A2 | &theta;<sub>2</sub> - &pi;/2  | Loff            | L2             | -&pi;/2 |   
     
  4. Third iteration: Our goal is to get the next frame of reference aligned with the predefined frame of reference for the tip of the robot. 
    We do transformations following the DH-convention:
      1. Rotate 90 degrees about z<sub>2</sub> to align the new x-axis with the defined x<sub>t</sub>.
      2. Translate the length L3 along the z-axis. 
      3. Translate the length 0 along the new x-axis. 
      4. Rotate 0 degrees about new x<sub>2</sub>.
         <br/> [INSERT FIGURE] <br/><br/>
  This yields the third line in the DH-table: <br/>  
  
     | | &theta; <sub>i</sub>      | d<sub>i</sub> | a<sub>i</sub> | &alpha;<sub>i</sub> |
     |--- | --- | --- | --- | --- |
     | A3 |  &pi;/2  | d3 + L3            | 0             | 0 |   

We now have a result with only three A-matrices, which is the least amount we can have while including all lengths, angles and joint variables of the configuration. We got to this result using only an intuitive understanding of our goals and the steps we are allowed to take to get there. This is a quick method to solve or check forward kinematics solutions. 

**Bonus exercise:** Try to double check the solution by checking the rules for the frames and parameters (found above). 


Youtube videos explaining the DH convention:

* ![Intro by Angela Sodemann](https://www.youtube.com/watch?v=bzoppKB6sWI)

* ![Visualization](https://www.youtube.com/watch?v=rA9tm0gTln8&t=1s)

      
    
   
    
