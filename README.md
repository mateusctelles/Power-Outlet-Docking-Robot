# Real Time Power Outlet Detection Applied to Differential Robot

## Project Overview
This project focuses on integrating a real-time object detection system into a differential robot using the YOLO v8 Nano architecture. The main objective is to autonomously detect and navigate towards power outlets for charging, enhancing the robot's operational autonomy across different environments.

## Features
- **Object Detection**: Employs YOLO v8 Nano for high-efficiency and accuracy in real-time power outlet detection.
- **Proportional Control Algorithm**: Implements proportional controllers for navigation based on detection outputs.
- **Simulation and Real Environment Testing**: Validates the model's effectiveness through extensive video simulations and real-environment tests.

## Technologies Used
- **YOLO v8 Nano**: Chosen for its speed and efficiency, particularly in environments with constrained computational resources.
- **ROS 2**: Manages robot operations and communication, facilitating real-time processing.
- **GoPiGo & Raspberry Pi**: Provides the physical platform and computing hardware for the robot.

## Setup and Installation
1. **Clone the Repository**
   ```bash
   git clone <repository-url>
2. **Install ROS 2**
   - Install ROS 2 on your Raspberry Pi. Follow the official ROS 2 documentation to ensure it is configured properly to interact with GoPiGo.
   - Ensure all dependencies and ROS 2 packages needed for the project are installed.

3. **Assemble the GoPiGo Robot**
   - Attach the camera module to the GoPiGo robot. Ensure it is securely mounted and positioned to capture the environment at the robot’s operating height.
   - Connect the Raspberry Pi to the GoPiGo robot and verify all hardware connections are correct.

## Usage
To run the detection and navigation system:
```bash
python3 robot_control.py
```
This script activates the robot's camera and starts a continuous loop for real-time detection of power outlets. Detected outlets trigger navigational commands to approach and dock for recharging.

## Detailed Workflow

### Data Collection
- **Phase 1**: Used a pre-annotated dataset for initial model training on general power outlet recognition.
- **Phase 2**: Custom images captured from the test environment were used to refine and tune the model to specific conditions and challenges found in the actual deployment environment.

### Model Training
The training process included two main stages:
- **Stage 1**: General feature learning using the broader dataset.
- **Stage 2**: Fine-tuning with a custom dataset specific to the deployment environment to enhance accuracy and reliability.

### Model Performance Testing
- **Video Simulations**: Conducted to test detection capabilities under various simulated conditions to ensure robustness.
- **Real Environment Testing**: Implemented in a controlled room setup mimicking potential operational scenarios to validate real-world effectiveness.

## Results and Improvements
The model displayed high accuracy in well-lit conditions and adequate performance under artificial lighting. Future improvements will focus on:
- **Enhancing the dataset**: Adding images under varied lighting conditions.
- **Refining the control system**: Implementing a PID control system to improve navigational smoothness and response accuracy.

## Future Work
- **Dataset Expansion**: To include more diverse environmental scenarios.
- **Control System Upgrade**: Introducing integral and derivative controls to the existing proportional control setup.
- **Adaptability Tests**: Expanding the application to different robot types and operational contexts.

## Contributing
Contributors are welcome! Please fork this repository, make your changes, and submit a pull request with your improvements.

## License
This project is released under the MIT License. See the `LICENSE` file for more information.
