# Robotic arm - Pick & Place project

[DH]: ./misc_images/DHParameters.png
[dh-transform]: ./misc_images/dh-transform.png
[dh-transform-matrix]: ./misc_images/dh-transform-matrix.png
[l21-l-inverse-kinematics-new-design-fixed]: ./misc_images/l21-l-inverse-kinematics-new-design-fixed.png

## Project Writeup

### Derive KR210's DH parameters

Below is the image shows the coordination of the robotic arm.

![Arm image][DH]

To build the DH parameter table, we need to figure out the following variables.

**Link Twist alpha**: alpha(i-1) is the angle from axis Z(i-1) to Z(i) via X(i-1) axis using right hand rule.

**Link Length a**: a(i-1) is the distance between Z(i-1) and Z(i) along X(i-1) axis.

**Link Offset d**: d(i) is the distance between Z(i-1) - Z(i) along X(i-1) axis.

**Joint Angle theta**: theta(i) is the angle from axis X(i-1) to X(i) along Z(i) using the right and rule. All joint angles will be zero at initial Robot state in KR210 except joint 2 which has a -90 degree constant offset between X(1) and X(2).

So the DH parameters table according to the above image and `kr210.urdf.xacro` for the arm shows below:

| Links | alpha(i-1) | a(i-1) | d(i) | theta(i) |
|-------|------------|--------|------|----------|
| 0->1  | 0          | 0      | 0.75 | q1
| 1->2  | -pi/2      | 0.35   | 0    | -pi/2 + q2
| 2->3  | 0          | 1.25   | 0    | q3
| 3->4  | -pi/2      | -0.054 | 1.50 | q4
| 4->5  | pi/2       | 0      | 0    | q5
| 5->6  | -pi/2      | 0      | 0    | q6
| 6->EE | 0          | 0      | 0.303| q7

Where `q(i)` in the table represent the input angle to the joints.

### Create individual transformation matrices about each joint

The DH convention uses four individual transforms,

![1][dh-transform]

to describe the relative translation and orientation of link (i-1) to link (i). In matrix form, this transform is,

![2][dh-transform-matrix]

Python code related to the equation above:

```python
def TF_Matrix(alpha, a, d, q):
    TF = Matrix(
        [
            [cos(q), -sin(q), 0, a],
            [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
            [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
            [0, 0, 0, 1]
        ]
    )
    return TF
# Transform to end effector
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
```

To get the end effector position relative to base:

```python
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

### Inverse Kinematics

Since the last three joints (Joint_4, Joint_5, and Joint_6) in KUKA KR210 robot arm are revolute and their joint axes intersect at a single point (Joint_5), we have a case of spherical wrist with joint_5 being the common intersection point; the wrist center (WC). This allows us to kinematically decouple the IK problem into Inverse Position and Inverse Orientation problems and been solved analytically.

#### Inverse Position and Joints 1 to 3

Since we have the case of a spherical wrist involving joints 4,5,6, the position of the wrist center is governed by the first three joints. We can obtain the position of the wrist center by using the complete transformation matrix we derived in the last section based on the end-effector pose.

theta(1) is pretty straight forward, just the angle from X(0) to the line between O(0) and wrist center WC.

```python
# y = WC[1], x = WC[0]
theta1 = atan2(WC[1], WC[0])
```

For theta(2) and theta(3):

![3][l21-l-inverse-kinematics-new-design-fixed]

The labels 2, 3 and WC are Joint 2, Joint 3, and the Wrist Center, respectively. We can obtain, or rather visualize, the triangle between the three if you project the joints onto the z-y plane corresponding to the world reference frame.

Below is the computing steps according to the image above.

```python
A = 1.501
B = 1.25
C = sqrt(pow(sqrt((WC[0] ** 2 + WC[1] ** 2) - 0.35), 2) + pow((WC[2] - 0.75), 2))

a = acos((B ** 2 + C ** 2 - A ** 2) / (2 * B * C))
b = acos((A ** 2 + C ** 2 - B ** 2) / (2 * A * C))
c = acos((A ** 2 + B ** 2 - C ** 2) / (2 * A * B))

theta2 = pi / 2 - a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
theta3 = pi / 2 - (b + 0.036)  # 0.036 accounts for sag in link4 of -0.054m
```

#### Inverse Orientation and Joints 4 to 6

Using the individual DH transforms we can obtain the resultant transform and hence resultant rotation by:

```python
R0_6 = R0_1 * R1_2 * R2_3 * R3_4 * R4_5 * R5_6
```

Since the overall RPY (Roll Pitch Yaw) rotation between base_link and gripper_link must be equal to the product of individual rotations between respective links, following holds true:

```python
R0_6 = Rrpy
```

where,

`Rrpy` = Homogeneous RPY rotation between base_link and gripper_link as calculated above.

We can substitute the values we calculated for joints 1 to 3 in their respective individual rotation matrices and pre-multiply both sides of the above equation by inv(R0_3) which leads to:

```python
R3_6 = inv(R0_3) * Rrpy
```

The resultant matrix on the RHS (Right Hand Side of the equation) does not have any variables after substituting the joint angle values, and hence comparing LHS (Left Hand Side of the equation) with RHS will result in equations for joint 4, 5, and 6.

```python
Extract rotation matrix R0_3 from transformation matrix T0_3 the substitute angles q1-3
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

R3_6 = R0_3.inv("LU") * ROT_EE

# Compute theta 4 - 6
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0, 2] ** 2 + R3_6[2, 2] ** 2), R3_6[1,2])
theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```

## Setup Environment and Run

[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step

Check the version of gazebo installed on your system using a terminal:

```sh
gazebo --version
```

To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:

```sh
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo7
```

Once again check if the correct version was installed:

```sh
gazebo --version
```

### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:

```sh
cd ~/catkin_ws/src
git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
sudo chmod +x target_spawn.py
sudo chmod +x IK_server.py
sudo chmod +x safe_spawner.sh
```

Build the project:

```sh
cd ~/catkin_ws
catkin_make
```

Add following to your .bashrc file

```sh
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by

```sh
cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:

```sh
cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
rosrun kuka_arm IK_server.py
```

Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

    - Robot

    - Shelf

    - Blue cylindrical target in one of the shelves

    - Dropbox right next to the robot

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully.

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location.

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.
