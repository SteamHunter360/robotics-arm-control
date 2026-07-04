import pytest
from src.forward_kinematics import forward_kinematics


def test_forward_kinematics_zero_angles():
    points = forward_kinematics(0, 0)

    x, y = points["end_effector"]

    assert x == pytest.approx(1.75)
    assert y == pytest.approx(0.0)


def test_forward_kinematics_vertical_arm():
    points = forward_kinematics(90, 0)

    x, y = points["end_effector"]

    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(1.75)


def test_forward_kinematics_folded_arm():
    points = forward_kinematics(0, 180)

    x, y = points["end_effector"]

    assert x == pytest.approx(0.25)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_forward_kinematics_known_position():
    points = forward_kinematics(
        theta1_deg=45,
        theta2_deg=30,
        L1=1.0,
        L2=0.75,
    )

    x, y = points["end_effector"]

    assert x == pytest.approx(0.901221, abs=1e-6)
    assert y == pytest.approx(1.431551, abs=1e-6)