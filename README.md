🤖 Robotics Arm Control

A Python-based robotics project demonstrating forward kinematics, inverse kinematics, and trajectory tracking for a planar robotic arm.

This project focuses on building a working, testable simulation rather than just theoretical models.

🚧 Current Status

This project is currently under active development.

Progress:

✅ Project structure defined
🔄 Forward kinematics implementation (in progress)
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
📁 Project Structure
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

🧪 Testing

Run validation tests:

pytest tests/

Tests will verify:

Forward kinematics correctness
Inverse kinematics accuracy
End-effector error within tolerance


🧠 What I Learned
Implementing kinematic models from first principles
Solving inverse kinematics numerically
Debugging control and convergence behaviour
Structuring engineering code for clarity and testing
🔜 Next Steps
Extend to 3DOF / 6DOF robotic arm
Add PID-based joint control
Integrate with ROS / Gazebo
Compare different IK solving methods

📌 Notes

This project is intentionally focused on building a working system with visible outputs, rather than over-engineering complexity.

👤 Author

Mechanical Engineering student building at the intersection of robotics, software, and control systems.
