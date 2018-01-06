# Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc2.png

## 1. Introduction 

The assignment of this project was to solve the inverse kinematics problem of the KUKA R210 robot arm. The problem has been solved by calculating angles for 6 joints of the robot arm to achieve given end effector position. 

## 2. Kinematic Analysis
### 1. KR210 robot DH parameters.

From the kr210.urdf.xacro file I was able to extract the following DH parameters:

| Links | alpha(i-1) | a(i-1) | d(i)  | theta(i)   |
| ----- | ---------- | ------ | ----- | ---------- |
| 0->1  | 0          | 0      | .75   | qi         |
| 1->2  | - pi/2     | 0.35   | 0     | -pi/2 + q2 |
| 2->3  | 0          | 1.25   | 0     | qi         |
| 3->4  | - pi/2     | -0.054 | 1.501 | qi         |
| 4->5  | pi/2       | 0      | 0     | qi         |
| 5->6  | - pi/2     | 0      | 0     | qi         |
| 6->EE | 0          | 0      | 0.303 | 0          |

Annotated angles can be found on the image bellow (source: Udacity lectures):

![alt text][image2]


### 2. Individual transformation matrices about each joint

With the following code individual transformation matrices are generated:
```
    T0_1 = Matrix([[cos(q1),            -sin(q1),            0,              a0],
                   [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -
                    sin(alpha0), -sin(alpha0) * d1],
                   [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0),
                    cos(alpha0),  cos(alpha0) * d1],
                   [0,                   0,            0,               1]])
    T0_1 = T0_1.subs(s)
    T1_2 = Matrix([[cos(q2),            -sin(q2),            0,              a1],
                   [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -
                    sin(alpha1), -sin(alpha1) * d2],
                   [sin(q2) * sin(alpha1), cos(q2) * sin(alpha1),
                    cos(alpha1),  cos(alpha1) * d2],
                   [0,                   0,            0,               1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[cos(q3),            -sin(q3),            0,              a2],
                   [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -
                    sin(alpha2), -sin(alpha2) * d3],
                   [sin(q3) * sin(alpha2), cos(q3) * sin(alpha2),
                    cos(alpha2),  cos(alpha2) * d3],
                   [0,                   0,            0,               1]])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[cos(q4),            -sin(q4),            0,              a3],
                   [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -
                    sin(alpha3), -sin(alpha3) * d4],
                   [sin(q4) * sin(alpha3), cos(q4) * sin(alpha3),
                    cos(alpha3),  cos(alpha3) * d4],
                   [0,                   0,            0,               1]])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[cos(q5),            -sin(q5),            0,              a4],
                   [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -
                    sin(alpha4), -sin(alpha4) * d5],
                   [sin(q5) * sin(alpha4), cos(q5) * sin(alpha4),
                    cos(alpha4),  cos(alpha4) * d5],
                   [0,                   0,            0,               1]])
    T4_5 = T4_5.subs(s)

    T5_6 = Matrix([[cos(q6),            -sin(q6),            0,              a5],
                   [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -
                    sin(alpha5), -sin(alpha5) * d6],
                   [sin(q6) * sin(alpha5), cos(q6) * sin(alpha5),
                    cos(alpha5),  cos(alpha5) * d6],
                   [0,                   0,            0,               1]])
    T5_6 = T5_6.subs(s)

    T6_G = Matrix([[cos(q7),            -sin(q7),            0,              a6],
                   [sin(q7) * cos(alpha6), cos(q7) * cos(alpha6), -
                    sin(alpha6), -sin(alpha6) * d7],
                   [sin(q7) * sin(alpha6), cos(q7) * sin(alpha6),
                    cos(alpha6),  cos(alpha6) * d7],
                   [0,                   0,            0,               1]])
    T6_G = T6_G.subs(s)

```

Transformation matrices are always the same, we only change DH parameter symbols for each joint and insert them with **subs(s)** command from the previously calculated DH parameter table.

### 3. Generalized homogeneous transform between base_link and gripper_link by using only end-effector(gripper) pose

Then  I have calculated individual transform matrices from base_link to gripper_link by matrix multiplication and use the **simplify** method to simplify matrices programmatically.

```
    # Create individual transformation matrices
    T0_2 = simplify(T0_1 * T1_2)
    T0_3 = simplify(T0_2 * T2_3)
    T0_4 = simplify(T0_3 * T3_4)
    T0_5 = simplify(T0_4 * T4_5)
    T0_6 = simplify(T0_5 * T5_6)
    T0_G = simplify(T0_6 * T6_G)
```

### 4. Inverse Position Kinematics 

To find a wrist center position (position of the 5th joint) from a given gripper position I have first added previously calculated T0_G matrix to the correction matrix as defined bellow:
```
    # Correction difference between definition of gripper_link in URDF vs DH convention
    R_z = Matrix([[cos(np.pi),  -sin(np.pi),     0,    0],
                  [sin(np.pi),   cos(np.pi),     0,    0],
                  [0,        0,       1,    0],
                  [0,        0,       0,    1]])

    R_y = Matrix([[cos(-np.pi / 2),        0, sin(-np.pi / 2),   0],
                  [0,        1,          0,   0],
                  [-sin(-np.pi / 2),        0, cos(-np.pi / 2),   0],
                  [0,        0,          0,   1]])
    R_corr = simplify(R_z * R_y)

    T_total = simplify(T0_G + R_corr)
```

After the differences between URDF and DH conventions have been addressed, I have extracted R_corr_rot rotation matrix:
```
R_corr_rot = R_corr[0:3, 0:3]
```

I have defined rotation matrices around x, y and z axes as well:
```
def rot_x(q):
    R_x = Matrix([[1,              0,        0],
                  [0,         cos(q),  -sin(q)],
                  [0,         sin(q),  cos(q)]])

    return R_x


def rot_y(q):
    R_y = Matrix([[cos(q),        0,  sin(q)],
                  [0,        1,       0],
                  [-sin(q),        0, cos(q)]])

    return R_y


def rot_z(q):
    R_z = Matrix([[cos(q),  -sin(q),       0],
                  [sin(q),   cos(q),       0],
                  [0,        0,       1]])
    return R_z
```

Then I have created Rrpy matrix in a way that I have rotated and multiplied previously defined rot_x,  rot_y and rot_z matrices for yaw, pitch and roll angles respectively given by the path planner. At the end I have multiplied the rotation matrices by the previously defined R_corr_rot matrix to address discrepancies between DH parameter notation and Gazebo notation:
```
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    # Your IK code here
    # Compensate for rotation discrepancy between DH parameters and Gazebo

    Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr_rot
```

After I made those calculations I had everything I needed to calculate wrist center position based on the formula from lectures:
```
    wx = px - (d6 + d7) * Rrpy.row(0).col(2)[0]
    wy = py - (d6 + d7) * Rrpy.row(1).col(2)[0]
    wz = pz - (d6 + d7) * Rrpy.row(2).col(2)[0]
    wx = wx.subs(s)
    wy = wy.subs(s)
    wz = wz.subs(s)
```

### 5. inverse Orientation Kinematics

I have calculated first three joints by using inverse tangent function and the law of cosines. This was fairly straightforward and it involved using wrist centre positions combined with the parameters from DH table.

theta1 = atan2(wy, wx)
r1 = sqrt(wx * wx + wy * wy) - a1
r2 = wz - d1
phi2 = atan2(r2, r1)
r3 = sqrt(r1 * r1 + r2 * r2)
phi1 = acos((d4 * d4 - a2 * a2 - r3 * r3) / (-2 * a2 * r3))
theta2 = pi / 2 - (phi1 + phi2)
theta2 = theta2.subs(s)
phi3 = acos((r3 * r3 - a2 * a2 - d4 * d4) / (-2 * a2 * d4))
phi4 = -atan2(a3, d4)
theta3 = pi / 2 - phi3 - phi4
theta3 = theta3.subs(s)

Once I had wrist centre position and angles for the first three joints, I have calculated individual joint angles for the last three joints with the help of R3_6_symbol and R3_6 matrices that I defined as described bellow.

I have calculated R0_3 matrix by multiplying previously defined transformation matrices and extracting only the rotation part. In a same way I have calculated R3_6_symbol matrix as well.

R0_3 = simplify(T0_1 * T1_2 * T2_3)[:3, :3]
R3_6_symbol = simplify(T3_4 * T4_5 * T5_6)[:3, :3]

Then I have calculated R3_6 matrix with values, I did it by taking inverse matrix of R0_3 and multiplying it by Rrpy. To get specific values I needed to insert previously calculated angles for the first three joints with help of **evalf** method as well.

R3_6 = R0_3.inv("LU") * Rrpy
R3_6 = R3_6.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

By extracting equations from R3_6_symbol matrix and inserting values from the R3_6 matrix I was able to calculate joints for the last three angles.

R3_6_symbol matrix:
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6) , -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5) , -sin(q5)*cos(q4)]
[sin(q5)*cos(q6)                            , -sin(q5)*sin(q6)                           , cos(q5)]
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4) , sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6)  , sin(q4)*sin(q5)]

Calculation for the last three joints:

r13 = R3_6.row(0).col(2)[0]
r21 = R3_6.row(1).col(0)[0]
r22 = R3_6.row(1).col(1)[0]
r23 = R3_6.row(1).col(2)[0]
r33 = R3_6.row(2).col(2)[0]

theta5 = atan2(sqrt(r13**2 + r33**2), r23)
if sin(theta5) < 0:
    theta4 = atan2(-r33, r13)
    theta6 = atan2(r22, -r21)
else:
    theta4 = atan2(r33, -r13)
    theta6 = atan2(-r22, r21)

### 6. Conclusion
This was very interesting project where it was needed to apply knowledge of Python Sympy package, ROS and trigonometry. I found the project very engaging and challenging at the same time, which was great. Things that could still be improved is the speed of calculations, as well as precalculated trajectories for specific trajectories. There is still much room for improvement in the trajectory planning as well, but that would be out of scope of this project.