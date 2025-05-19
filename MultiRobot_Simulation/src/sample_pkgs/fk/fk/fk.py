#!/usr/bin/env python3

import numpy as np
from math import cos, sin, pi

class OpenManipulatorFK:
    def __init__(self):
        # OpenMANIPULATOR-X link lengths (in meters)
        self.l1 = 0.077  # Distance from base to joint1
        self.l2 = 0.128  # Length of link1
        self.l3 = 0.124  # Length of link2
        self.l4 = 0.126  # Length of link3
        
    def transform_matrix(self, theta, d, a, alpha):
        """Calculate transformation matrix based on DH parameters"""
        T = np.array([
            [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0,          sin(alpha),             cos(alpha),             d           ],
            [0,          0,                      0,                      1           ]
        ])
        return T

    def forward_kinematics(self, joint_angles):
        """
        Calculate forward kinematics for OpenMANIPULATOR-X
        Args:
            joint_angles: List of 4 joint angles [theta1, theta2, theta3, theta4] in radians
        Returns:
            End effector position (x, y, z) and rotation matrix
        """
        # DH Parameters for OpenMANIPULATOR-X [theta, d, a, alpha]
        dh_params = [
            [joint_angles[0], self.l1,      0,    pi/2],  # Joint 1
            [joint_angles[1], 0,       self.l2,    0  ],  # Joint 2
            [joint_angles[2], 0,       self.l3,    0  ],  # Joint 3
            [joint_angles[3], 0,       self.l4,    0  ],  # Joint 4
        ]
        
        # Calculate transformation matrices
        T01 = self.transform_matrix(*dh_params[0])
        T12 = self.transform_matrix(*dh_params[1])
        T23 = self.transform_matrix(*dh_params[2])
        T34 = self.transform_matrix(*dh_params[3])
        
        # Calculate final transformation matrix
        T04 = T01 @ T12 @ T23 @ T34
        
        # Extract position and rotation
        position = T04[:3, 3]
        rotation = T04[:3, :3]
        
        return position, rotation

def main():
    # Create FK solver
    fk_solver = OpenManipulatorFK()
    
    # Test some joint angles (in radians)
    test_angles = [
        [0, 0, 0, 0],                    # Home position
        [pi/4, 0, 0, 0],                 # Base rotation 45 degrees
        [0, pi/4, pi/4, 0],             # Arm extended
        [pi/4, pi/4, pi/4, pi/4]        # Combined movement
    ]
    
    # Calculate FK for each test case
    for i, angles in enumerate(test_angles):
        position, rotation = fk_solver.forward_kinematics(angles)
        print(f"\nTest case {i + 1}")
        print(f"Joint angles: {[round(angle * 180/pi, 2) for angle in angles]} degrees")
        print(f"End effector position (x, y, z): {position.round(3)} meters")
        print("Rotation matrix:")
        print(rotation.round(3))

if __name__ == "__main__":
    main()
