# Transformations

## Schedule:
  * Rotation directions (+/-)
  * Rotation by angle about x,y,z axes
  * Rotations about current and fixed frames
  * Brief mention of arbitrary axis rotations ("parameterization of rotations").
  * Task 2.12
  * ------ BREAK ------
  * Structure of the Homogeneous Transformation matrix
  * Chaining and inverting transformations
  * Assignment 1: Task 1

## Information:
* **Positive/Negative rotation directions:**
  * Looking from behind an axis, clockwise direction is **+**, counter-clockwise is **-**.

* **Commonly used matrices:**

  For the derivation of a 2 dimensional matrix of rotation, read **chapter 2.2.1: Rotation in the Plane**.
  ![Matrices](common_matrices.png?raw=true "Common Matrices")
  
* **Methods of rotation about arbitrary axes (low priority, look up in the book):**
  1. Euler angles approach
  2. Roll-Pitch-Yaw approach
  3. Axis/Angle approach (*with matrix on p. 58 in the course book*)

* **Rotation about fixed/current frames (pre- & post-multiplication)**
  * Rotation about current frame: **post**-multiply
  * Rotation about fixed frame: **pre**-multiply
  * Practice with a task from the book (any of 2.10-2.13 will do)

* **Setting up transformation chains (going from one frame to the others):**
  * Analogy: transformation from one frame to another - going between two places (example: home -> school)
  * Different transformation paths possible. Example:
    * Want to go from school to the store, but direct path is unknown.
    * However, know these paths: school -> home **and** home -> store
  * Transformations between frames are similar, all paths are valid:
    * Frame 1 -> Frame 3 **OR** Frame 1 -> Frame 2 -> Frame 3
  * "Inverting" known transformation is possible:
    * Apply opposite rotation, then opposite translation
    
## Weekly tasks:
* 2.10-2.13
* 2.38
* 2.39

  
