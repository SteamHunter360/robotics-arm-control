import math


def forward_kinematics(theta1_deg, theta2_deg, L1=1.0, L2=0.75):
    """
    Calculate joint and end-effector positions for a 2-link planar robotic arm.

    Angles are entered in degrees.
    Returns base, joint 1, and end-effector coordinates.
    """

    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)

    x0, y0 = 0, 0

    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)

    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)

    return {
        "base": (x0, y0),
        "joint_1": (x1, y1),
        "end_effector": (x2, y2),
    }