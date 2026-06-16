🤖 Robotics Arm Control

A Python-based robotics project demonstrating forward kinematics, inverse kinematics, and trajectory tracking for a planar robotic arm.

This project focuses on building a working, testable simulation rather than just theoretical models.

import math

def forward_kinematics(theta1, theta2, L1, L2):
    """
    Calculates the end-effector position of a 2-link planar robotic arm.

    theta1, theta2: joint angles in degrees
    L1, L2: link lengths
    """

    theta1 = math.radians(theta1)
    theta2 = math.radians(theta2)

    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)

    return x, y

```text
if __name__ == "__main__":
    x, y = forward_kinematics(theta1=45, theta2=30, L1=1.0, L2=0.75)

    print(f"End-effector position: x = {x:.3f}, y = {y:.3f}")
   ``` 

## Forward Kinematics

Forward kinematics calculates the position of the robot arm's end effector from the known joint angles and link lengths.

For my two-link planar robotic arm:

```text
x = L1 cos(theta1) + L2 cos(theta1 + theta2)

y = L1 sin(theta1) + L2 sin(theta1 + theta2) 

```
Example Input 

```text
L1 = 1.0
L2 = 0.75
theta1 = 45 degrees
theta2 = 30 degrees
```
Example Output

```text
End-effector position: x = 0.901, y = 1.431
```



🚧 Current Status

This project is currently under active development.

Progress:

✅ Project structure defined
🔄 Forward kinematics implementation 
🔄 Inverse kinematics solver (in progress)
🔄 Simulation and plotting (in progress)

Demo and results will be added as core functionality is completed.

🚀 Demo

Coming soon – simulation currently in development

📊 Results

Plots and simulation output will be added here once implementation is complete

Planned Example Output:

Target Position: (0.5, 0.3)
Computed Joint Angles
End-effector trajectory plot
Final position error

⚙️ Features (Planned)
Forward kinematics (FK) for planar robotic arm
Numerical inverse kinematics (IK) solver
Target position tracking
Trajectory generation and visualization
Joint angle and error plotting


## 📁 Project Structure
robotics-arm-control/
│── src/
│   ├── kinematics.py        # FK + IK implementations
│   ├── controller.py        # trajectory / control logic
│   └── simulator.py         # simulation + visualization
│
│── tests/
│   └── test_kinematics.py   # FK/IK validation
│
│── results/
│   ├── gifs/
│   └── plots/
│
│── main.py                  # run simulation
│── requirements.txt
│── README.md

🛠️ How to Run
git clone https://github.com/SteamHunter360/robotics-arm-control.git
cd robotics-arm-control

pip install -r requirements.txt
python main.py

Expected Behaviour:

A simulation window opens showing the robotic arm

The arm moves toward a target position

Trajectory and joint plots are saved in /results/plots

## 🧪 Testing

Run validation tests:

pytest tests/

Tests will verify:

Forward kinematics correctness
Inverse kinematics accuracy
End-effector error within tolerance


## 🧠 What I Learned
Implementing kinematic models from first principles
Solving inverse kinematics numerically
Debugging control and convergence behaviour
Structuring engineering code for clarity and testing
🔜 Next Steps
Extend to 3DOF / 6DOF robotic arm
Add PID-based joint control
Integrate with ROS / Gazebo
Compare different IK solving methods

## 📌 Notes

This project is intentionally focused on building a working system with visible outputs, rather than over-engineering complexity.

## 👤 Author

Mechanical Engineering student building at the intersection of robotics, software, and control systems.
