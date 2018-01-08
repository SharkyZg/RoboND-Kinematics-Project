#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sympy import *
from sympy.matrices import Matrix
import os
import pickle

# Define functions for Rotation Matrices about x, y, and z given specific angle.


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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        if not (os.path.exists("T6_G.p") or os.path.exists("R_corr_rot.p") or os.path.exists("R0_3.p")):
            # Your FK code here
            # Create symbols
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta_i
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols(
                'alpha0:7')

            # Create Modified DH parameters
            s = {alpha0:        0,   a0:       0,   d1:   .75,
                alpha1: -pi / 2,   a1:    0.35,   d2:     0, q2: q2 - pi / 2,
                alpha2:     0,   a2:    1.25,   d3:     0,
                alpha3: -pi / 2,   a3: -0.054,   d4:  1.501,
                alpha4:  pi / 2,   a4:       0,   d5:     0,
                alpha5: -pi / 2,   a5:       0,   d6:     0,
                alpha6:     0,   a6:     0.0,   d7: 0.303,   q7: 0}

            # Define Modified DH Transformation matrix
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

            # T3_4 = Matrix([[cos(q4),            -sin(q4),            0,              a3],
            #                [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -
            #                 sin(alpha3), -sin(alpha3) * d4],
            #                [sin(q4) * sin(alpha3), cos(q4) * sin(alpha3),
            #                 cos(alpha3),  cos(alpha3) * d4],
            #                [0,                   0,            0,               1]])
            # T3_4 = T3_4.subs(s)

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


            # Create individual transformation matricess
            # T0_2 = simplify(T0_1 * T1_2)
            # T0_3 = simplify(T0_2 * T2_3)
            # T0_4 = simplify(T0_3 * T3_4)
            # T0_5 = simplify(T0_4 * T4_5)
            # T0_6 = simplify(T0_5 * T5_6)
            # T0_G = simplify(T0_6 * T6_G)

            # Correction difference between definition of gripper_link in URDF vs DH convetion
            R_z = Matrix([[cos(pi),  -sin(pi),     0,    0],
                        [sin(pi),   cos(pi),     0,    0],
                        [0,        0,       1,    0],
                        [0,        0,       0,    1]])

            R_y = Matrix([[cos(-pi / 2),        0, sin(-pi / 2),   0],
                        [0,        1,          0,   0],
                        [-sin(-pi / 2),        0, cos(-pi / 2),   0],
                        [0,        0,          0,   1]])
            R_corr = simplify(R_z * R_y)

            # T_total = simplify(T0_G + R_corr)

            # Extract rotation matrices from the transformation matrices
            R_corr_rot = R_corr[0:3, 0:3]
            R0_3 = simplify(T0_1 * T1_2 * T2_3)[:3, :3]
            # R3_6_symbol = simplify(T3_4 * T4_5 * T5_6)[:3, :3]

            pickle.dump(T6_G, open("T6_G.p", "wb"))
            pickle.dump(R_corr_rot, open("R_corr_rot.p", "wb"))  
            pickle.dump(R0_3, open("R0_3.p", "wb"))            
                      
        else:
            T6_G = pickle.load(open("T6_G.p", "rb"))
            R_corr_rot = pickle.load(open("R_corr_rot.p", "rb"))
            R0_3 = pickle.load(open("R0_3.p", "rb"))

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo

            Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr_rot

            wx = px - (d6 + d7) * Rrpy.row(0).col(2)[0]
            wy = py - (d6 + d7) * Rrpy.row(1).col(2)[0]
            wz = pz - (d6 + d7) * Rrpy.row(2).col(2)[0]
            wx = wx.subs(s)
            wy = wy.subs(s)
            wz = wz.subs(s)

            print("WC")
            print("wx: ", wx)
            print("wy: ", wy)
            print("wz: ", wz)
            
            # Calculate joint angles using Geometric IK method
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

            R3_6 = R0_3.inv("LU") * Rrpy
            R3_6 = R3_6.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            # r11 = R3_6.row(0).col(0)[0]
            r13 = R3_6.row(0).col(2)[0]

            r21 = R3_6.row(1).col(0)[0]
            r22 = R3_6.row(1).col(1)[0]
            r23 = R3_6.row(1).col(2)[0]
            # r31 = R3_6.row(2).col(0)[0]
            # r32 = R3_6.row(2).col(1)[0]
            r33 = R3_6.row(2).col(2)[0]

            theta5 = atan2(sqrt(r13**2 + r33**2), r23)
            if sin(theta5) < 0:
                theta4 = atan2(-r33, r13)
                theta6 = atan2(r22, -r21)
            else:
                theta4 = atan2(r33, -r13)
                theta6 = atan2(-r22, r21)
            print("Angles: ", theta1, theta2, theta3, theta4, theta5, theta6)

        # Populate response for the IK request
        # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [
                theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" %
                      len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
