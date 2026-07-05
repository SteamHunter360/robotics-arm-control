import pytest

from src.forward_kinematics import forward_kinematics
from src.inverse_kinematics import inverse_kinematics


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


def test_inverse_kinematics_reaches_target():
    target_x = 0.9
    target_y = 1.2

    theta1, theta2 = inverse_kinematics(target_x, target_y)
    points = forward_kinematics(theta1, theta2)

    x, y = points["end_effector"]

    assert x == pytest.approx(target_x, abs=1e-6)
    assert y == pytest.approx(target_y, abs=1e-6)


def test_inverse_kinematics_rejects_unreachable_target():
    with pytest.raises(ValueError):
        inverse_kinematics(3.0, 0.0)