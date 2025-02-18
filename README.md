#Obstacle Avoidance using Modified Particle Swarm Optimization
🚀 Real-Time Path Planning & Obstacle Avoidance using a modified Particle Swarm Optimization (PSO) algorithm. This project enables an autonomous agent to navigate optimally while avoiding obstacles in real-time using camera-based perception.

🔹 Features
✅ Optimal Path Planning – Uses an enhanced PSO algorithm for efficient route optimization.
✅ Real-Time Obstacle Avoidance – Dynamically detects and avoids obstacles using live camera data.
✅ Computer Vision Integration – Processes visual input for intelligent decision-making.
✅ Fast & Adaptive – Designed for real-world environments with quick response times.

🔹 Technologies Used
🔸 Python | OpenCV | TensorFlow (if ML-based perception is used) | ROS (if robotic system)
🔸 Modified PSO Algorithm for adaptive navigation

🔹 How It Works
Camera Perception: Captures real-time environment data.
Obstacle Detection: Identifies obstacles using image processing.
Path Planning: Runs a modified PSO to find the optimal collision-free path.
Navigation Execution: Guides the agent along the computed trajectory.

🔹 Installation & Usage
git clone https://github.com/ahmedlamiiri/obstacle-avoidance-pso.git
cd obstacle-avoidance-pso
pip install -r requirements.txt
python main.py

🔹 Future Improvements
✅ Integration with LiDAR / Depth Sensors
✅ Optimization for Multi-Agent Systems
✅ Hardware Deployment (Drones, Robots)
