# Drone-Telemetry-Flight-Estimator
# Autonomous Drone Flight Telemetry & State Estimator

![C++17](https://img.shields.io/badge/Language-C%2B%2B17-blue.svg)
![Domain](https://img.shields.io/badge/Domain-Robotics%20%26%20Embedded%20Systems-green.svg)
![Developer](https://img.shields.io/badge/Developer-Ayoub%20Lahmar-brightgreen.svg)

A lightweight real-time C++ telemetry simulator and sensor fusion pipeline for autonomous unmanned aerial vehicles (UAVs). Designed to process noisy IMU, Gyroscope, and Barometer data using a **Complementary Filter** and maintain flight stability with a **PID Altitude Control System**.

Designed and implemented by **Ayoub Lahmar** ([@Redayoub-lang](https://github.com/Redayoub-lang)) as part of my technical portfolio for admission into the **Bachelor’s in Computer Science & Technology** program at **Jiangsu University (JSU)**.

🚀 Key FeaturesSensor Fusion Pipeline: Implements a Complementary Filter combining high-pass filtered Gyroscope inputs and low-pass filtered Accelerometer data to eliminate drift and noise.Realistic Sensor Noise Model: Uses Gaussian normal distribution to simulate raw sensor variance in real-world flight conditions.Closed-Loop Altitude PID Control: Implements Proportional-Integral-Derivative control logic to calculate real-time thrust output for stable hover altitude.Real-time CLI Telemetry Streaming: Modular simulation pipeline rendering pitch, roll, and altitude status at 10 Hz frequency.📐 Mathematical FormulationComplementary Filter$$\theta_{est} = \alpha \cdot (\theta_{prev} + \omega_{gyro} \cdot \Delta t) + (1 - \alpha) \cdot \theta_{accel}$$Where:$\alpha = 0.95$ (Weight parameter balancing high-frequency gyro data and low-frequency accel data)$\omega_{gyro}$ = Angular velocity from Gyroscope$\theta_{accel}$ = Pitch/Roll calculated from Accelerometer gravity vector💻 How to Build & RunPrerequisitesC++17 compliant compiler (g++ or clang++)Compilation via TerminalBash# Clone repository
git clone [https://github.com/Redayoub-lang/Drone-Telemetry-Flight-Estimator.git](https://github.com/Redayoub-lang/Drone-Telemetry-Flight-Estimator.git)
cd Drone-Telemetry-Flight-Estimator

# Compile
g++ -std=c++17 main.cpp -o DroneEstimator

# Run
./DroneEstimator
🎯 Academic Background & ObjectiveWith my practical foundation in Software Engineering and Application Development (DTS), I developed this system to bridge hardware control with high-level software engineering. At Jiangsu University, my objective is to deepen my knowledge in Robotics Operating System (ROS 2), Embedded Control, and Autonomous Aerial Robotics.📜 LicenseThis project is open-source under the MIT License.
