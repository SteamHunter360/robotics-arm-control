import pytest
from src.kinematics import forward_kinematics


def test_forward_kinematics_known_position():
    points = forward_kinematics(theta1_deg=45, theta2_deg=30, L1=1.0, L2=0.75)

    x, y = points["end_effector"]

    assert x == pytest.approx(0.901, abs=0.001)
    assert y == pytest.approx(1.431, abs=0.001)