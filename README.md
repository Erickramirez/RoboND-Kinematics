## Project: Kinematics Pick & Place

**Steps to complete the project:**  

1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.
The [kr210.urdf.xacro](/kuka_arm/urdf/kr210.urdf.xacro) file contains all the robot specific information like links, joints, actuators, etc. The urdf file is an XML format used in ROS for representing a robot model. URDF can only describe a robot with rigid links connected by joints in a chain or tree-like structure. It is usefult to extract the DH parameters (using joins and their types, axis, links and so on) 
The [kr210.urdf.xacro](/kuka_arm/urdf/kr210.urdf.xacro)  contains robot specific information like link lengths and joint offsets, it is the only file you need to derive DH parameters and create transform matrices. Since urdf (and xacro) files are basically XML, they use tags to define robot geometry and properties. 

Denavit-Hartenberg Parameters:
*In general, each transform would require six independent parameters to describe frame i relative to i-1, three for position and three for orientation. In 1955, Jacques Denavit and Richard Hartenberg proposed a systematic method of attaching reference frames to the links of a manipulator that simplified the homogeneous transforms. Their method only requires four parameters to describe the position and orientation of neighboring reference frames. The four parameters are the following:*

![Alt text](/misc_images/DH-parameter.png)
* source: Udacity lesson.

Also, this is a representation of the robot model:
![Alt text](/misc_images/robot_model.png)

![Alt text](/misc_images/URDF_relation.png)


For instance for the link offset, in the joint_2 and joint_1 we have values for z = to 0.33 and 0.42 respectivetly, they will be part of the d(i-1) for the link 0->1 wich is d1 = 0.33 + 0.42 = 0.75

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0|0|0.75|q1|
1->2 | -pi/2|0.35|0|pi/2 -q2|
2->3 | 0|1.25|0|q3|
3->4 | -pi/2|-0.054|1.5|q4|
4->5 | pi/2|0|0|q5|
5->6 | -pi/2|0|0|q6|
6->EE |0|0|0.303|0

Homogeneous transform matrix from base_link to gripper_link using only the position and orientation of the gripper_link:



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
Inverse kinematics consist to transform, in this case,  the position of the object (cartesian space -xyz axis) to determinate the joint space (the robot's joins), also in this case we can get more than one solution.  (i.e., position and orientation) into  angles. 
![Alt text](/misc_images/forward-kinematics-01.png)

In this case we have 6 revolute joins,  such a design is called a spherical wrist and the common point of intersection is called the wrist center. The advantage of such a design is that it kinematically decouples the position and orientation of the end effector. Mathematically, this means that instead of solving twelve nonlinear equations simultaneously (one equation for each term in the first three rows of the overall homogeneous transform matrix), it is now possible to independently solve two simpler problems: **first, the Cartesian coordinates of the wrist center**, and then **the composition of rotations to orient the end effector**. Physically speaking, a six degree of freedom serial manipulator with a spherical wrist would use the first three joints to control the position of the wrist center while the last three joints would orient the end effector as needed.

The first 3 joints theta1, theta2, theta3 have the a lot of impact on the location of the end effector, or gripper. The last 3 joints: theta4, theta5, theta6 make up the spherical wrist, with `theta5` being the Wrist Center.


**Homogeneous transform** 
![Alt text](/misc_images/homogeneous_transform.png)
it has been implemented in the function:
```
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[cos(q), -sin(q), 0, a],
                 [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                 [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                 [0, 0, 0, 1]])
    return TF
```

It was necessary to correct for discrepancies between frames in Gazebo:
```
Rotation_Error = Rotation_yaw.subs(y, radians(180)) * Rotation_pitch.subs(p, radians(-90))
```
The composition of rotation  has been defined as: **Rotation_EE**  and each Rotation_xxx  are the rotration matrix from the transformation matrix.
```
Rotation_EE = Rotation_yaw * Rotation_pitch * Rotation_roll
Rotation_EE = Rotation_EE * Rotation_Error #It has been corrected with Gazebo
```
##### Wrist Center definition:
```
WC = EE - 0.303 * Rotation_EE[:, 2]
x = WC[0]
y = WC[1]
z = WC[2]
```
##### derive the equations to calculate all individual joint angles:
We will use again the following image:
![Alt text](/misc_images/robot_model.png)
###### theta1
It is the first joint angle, and we can determine it using Arc Tangent:
```
theta1 = atan2(WC[0], WC[1]) #atan2(x, y)
```
###### theta2
Check the following image for theta2 and theta3:
![Alt text](/misc_images/joint_relation.png)
It is the second angle, and it is determiated using the same triangle expressed previous image, and the Arc Tangent is the the angle from x axis to side_b

```
        side_a = sqrt((0.054) ** 2 + (0.96 + 0.54) ** 2) #it is the hypotenuse of a3 and d4
        side_b = sqrt((WC[2] - 0.75) ** 2 + (sqrt(WC[0] ** 2 + WC[1] ** 2) - 0.35) ** 2) #it is the hypotenuse of a3 and d4
        side_c = 1.25
        ## it is the application of c**2 = a**2 + b**2 - 2*a*b*cos(C) where C is the desired angle.
        angle_a = acos((side_b ** 2 + side_c ** 2 - side_a ** 2) / (2 * side_b * side_c))
        angle_b = acos((side_a ** 2 + side_c ** 2 - side_b ** 2) / (2 * side_a * side_c))

        theta2 = np.pi / 2.0 - (angle_a + atan2((WC[2] - 0.75), sqrt((WC[0] - 0.35) ** 2 + WC[1] ** 2)))
        # (angle_a + atan2((WC[2] - 0.75), sqrt((WC[0] - 0.35) ** 2 + WC[1] ** 2)) is the angle obtained if I draw a triangle from join 2, 3 and 5. this will be the external angle from x to the adjacent, that is why it is removing 0.75 of d1 and d2 in their axis
        #theta2 = pi/2 - angle_a - previos angle and 
```
###### theta3
It is the third angle, and it is determiated using the same triangle expressed previous image, and the Arc Tangent is the the angle from x axis to side_a
```
theta3 = np.pi / 2.0 - (angle_b + atan2(0.054, 0.96 + 0.54))
```
###### theta4, theta5 and theta6
theta4-6 will be determined by Euler Angles from a Rotation Matrix. We want to isolate the final 3 thetas, so we'll solve for what we know. The first three Rotations from the base frame to frame3:


![Alt text](/misc_images/R3_6.png)
```
        R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
        R3_6 = R0_3.T * Rotation_EE  # Use tranpose instead of inv("LU") [R0_3.inv("LU")]
```
Note: R0_3  cancels on the RHS of the equation, and then is why it is oly with: `R3_6 = R0_3.T * Rotation_EE`

Remember the following (Consider the extrinsic (i.e., fixed axis) X-Y-Z rotation sequence. The composite rotation matrix is,):
![Alt text](/misc_images/Rxyz.png)
It is possible to find beta, by recognizing:
![Alt text](/misc_images/beta.png)
and for gamma and alpha:
![Alt text](/misc_images/gamma.gif)
![Alt text](/misc_images/alpha.gif)
```
        theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
        theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
        theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 
![Alt text](/misc_images/final_result.png)



#### Conclusion
Using diagrams helps a lot to figure out the DH parameter table. I have an issue with the types of the theta values to convert them to float. A Linux native will be better than the VM used for the project.
