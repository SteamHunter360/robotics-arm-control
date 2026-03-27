# 🤖 2. Tech Project (Robotics Arm Control)

#🤖  Robotics Arm Control

Robotics Arm Control

A Python-based robotics project demonstrating forward kinematics, inverse kinematics, and trajectory tracking for a planar robotic arm.

This project focuses on building a working, testable simulation rather than just theoretical models.

🚀 Demo

## 🚀 Demo
*Coming soon – currently implementing simulation*

## 📈 Results
*Plots and simulation output will be added here*


📊 Example Output

*Output plots will be added after simulation is complete*

Once implemented, the system will:
- Compute joint angles for a target position  
- Simulate arm movement toward the target  
- Generate trajectory and joint angle plots  

Example (planned):
- Target: (0.5, 0.3)
- Output: joint angles + trajectory plot

## 🚧 Current Status

This project is currently under active development.

Current progress:
- [x] Project structure defined  
- [ ] Forward kinematics implementation  
- [ ] Inverse kinematics solver  
- [ ] Simulation + plotting  

Demo and results will be added once core functionality is complete.


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

Run basic validation tests:

pytest tests/

Tests verify:

Forward kinematics correctness
IK solution accuracy
End-effector error within tolerance
🧠 What I Learned
Implementing kinematic models from first principles
Numerical solving for inverse kinematics
Debugging control and convergence issues
Structuring engineering code for clarity and testing


🔜 Next Steps
Extend to 3DOF / 6DOF robotic arm
Add PID-based joint control
Integrate with ROS / Gazebo
Compare different IK solving methods
📌 Notes

This project is intentionally focused on building a working system with visible outputs rather than over-engineering complexity.

👤 Author

Mechanical Engineering student building at the intersection of robotics, software, and control systems.
