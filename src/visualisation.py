import os
import matplotlib.pyplot as plt

from src.forward_kinematics import forward_kinematics


def plot_robot_arm(theta1_deg=45, theta2_deg=30, L1=1.0, L2=0.75):
    """
    Plot and save the configuration of a 2-link planar robotic arm.
    """

    points = forward_kinematics(
        theta1_deg=theta1_deg,
        theta2_deg=theta2_deg,
        L1=L1,
        L2=L2,
    )

    x0, y0 = points["base"]
    x1, y1 = points["joint_1"]
    x2, y2 = points["end_effector"]

    fig, ax = plt.subplots(figsize=(8, 6))

    ax.plot(
        [x0, x1],
        [y0, y1],
        marker="o",
        linewidth=3,
        label="Link 1",
    )

    ax.plot(
        [x1, x2],
        [y1, y2],
        marker="o",
        linewidth=3,
        label="Link 2",
    )

    ax.annotate(
        f"Base\n({x0:.2f}, {y0:.2f})",
        xy=(x0, y0),
        xytext=(-0.35, -0.15),
        arrowprops=dict(arrowstyle="->"),
    )

    ax.annotate(
        f"Joint 1\n({x1:.2f}, {y1:.2f})",
        xy=(x1, y1),
        xytext=(x1 + 0.15, y1 + 0.10),
        arrowprops=dict(arrowstyle="->"),
    )

    ax.annotate(
        f"End Effector\n({x2:.2f}, {y2:.2f})",
        xy=(x2, y2),
        xytext=(x2 + 0.15, y2 + 0.10),
        arrowprops=dict(arrowstyle="->"),
    )

    ax.set_title("2-Link Robot Arm Forward Kinematics")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.grid(True)
    ax.axis("equal")
    ax.legend()

    os.makedirs("images", exist_ok=True)

    fig.savefig(
        "images/robot_arm_visualisation.png",
        dpi=300,
        bbox_inches="tight",
    )

    plt.show()


if __name__ == "__main__":
    plot_robot_arm()