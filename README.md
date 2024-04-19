# Project Overview
This project focuses on integrating a real-time object detection system into a differential robot using ROS (Robot Operation System 2) and the YOLO v8 Nano architecture. The main objective is to autonomously detect and navigate towards available power outlets for charging, enhancing the robot's operational autonomy across different environments.

<p align="center">
<img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/OutletSeekr_480.gif?raw=true" alt="final_gif" width="900"/>

                   Figure 1: Working project. Robot autonomously searching and pursuing available power outlet for docking.
</p>

# Features
- **Object Detection**: Employs YOLO v8 Nano for high-efficiency and accuracy in real-time power outlet detection.
- **Proportional Control Algorithm**: Implements proportional controllers for navigation based on detection outputs.
- **Simulation and Real Environment Testing**: Validates the model's effectiveness through extensive video simulations and real-environment tests.

# Technologies Used
- **YOLO v8 Nano**: Chosen for its speed and efficiency, particularly in environments with constrained computational resources.
- **ROS 2**: Manages robot operations and communication, facilitating real-time processing.
- **GoPiGo & Raspberry Pi**: Provides the physical platform and computing hardware for the robot.

# Detailed Workflow

## 1. Data Collection
- **Phase 1**: Used a pre-annotated dataset for initial model training on general power outlet recognition.

<p align="center">
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/rflow1.png?raw=true" alt="rflow1" width="500"/>
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/rflow11.jpg?raw=true" alt="final_gif" width="300"/>
 
                
</p>

- **Phase 2**: Custom images captured from the test environment were used to refine and tune the model to specific conditions and challenges found in the actual deployment environment.
<p align="center">
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/sala1.png?raw=true" alt="sala1" width="200"/>
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/sala12.png?raw=true" alt="sala12" width="200"/>
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/sala13.png?raw=true" alt="sala13" width="200"/>
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/sala14.png?raw=true" alt="sala14" width="200"/>
 
    
</p>

## 2. Data Annotation
The data annotation was performed in the cvat.ai website. The outlets in use by another devices were purposefully ignored, so that they would not be detected / considered as a goal.

<p align="center">
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/annotation_gif.gif?raw=true" alt="sala1" width="500"/>
  
            Figure: Sequence of annotated images in cvat.ai website. Here it can be seen how the annotation was manually made.
</p>


## 3. Model Training

### Model Architecture
The model architecture selected for this project was YOLO v8 Nano, which is a variant of the standard YOLO v8 optimized for environments where computational resources are limited. Here’s more on the architecture:

- **YOLO v8 Nano**: Designed for speed and efficiency, it uses deep convolutional neural networks (CNNs) but with a simplified structure that maintains performance while being resource-efficient.
- **Features**: YOLO v8 architectures are known for their ability to perform object detection in real-time by analyzing the entire image in one pass, predicting both bounding boxes and class probabilities simultaneously. This is crucial for applications like autonomous robotic navigation where quick processing is essential.


### Training Parameters and Execution
The model's training involved two main stages, each tailored to leverage the datasets prepared in the earlier phases:

#### First Stage of Training
- **Dataset**: Used the 1003 annotated images from RoboFlow.
- **Split**: Data was divided into 80% for training and 20% for testing.
- **Epochs**: 150 epochs were set to allow sufficient learning without overfitting.
- **Batch Size**: 16, to balance between memory constraints and efficient gradient estimation.
- **Patience Parameter**: Set at 50, to prevent overfitting and stop training if the model doesn’t improve on the validation set.

#### Second Stage of Training
- **Dataset**: The 312 newly captured and annotated images were used, focusing specifically on the test room environment.
- **Split**: Due to the smaller dataset size, a 90% training and 10% test split was used.
- **Epochs**: Reduced to 34, considering the specific adaptation required and the smaller dataset.
- **Batch Size**: Slightly decreased to 15, to adjust for the dataset's characteristics.
- **Patience Parameter**: Maintained at 50, for consistency in handling potential overfitting.

### Evaluation and Adjustments
- **Performance Monitoring**: Throughout the training process, the model’s performance was regularly evaluated using the validation set, allowing for adjustments in parameters if necessary.
- **Fine-Tuning**: The two-stage training approach ensured that the model was not only familiar with general features of power outlets but also finely adjusted to recognize the specific outlets it would encounter in its operational environment.

This structured approach to training ensures that the YOLO v8 Nano model is robust, efficient, and highly functional for the specific task of detecting power outlets in real-time, which is critical for the autonomous operations of the differential robot.


## 4. Model Performance Testing
- **Video Simulations**: Conducted to test detection capabilities under various simulated conditions to ensure robustness.
- **Real Environment Testing**: Implemented in a controlled room setup mimicking potential operational scenarios to validate real-world effectiveness.

## 5. Results and Improvements
The model displayed high accuracy in well-lit conditions and adequate performance under artificial lighting. Future improvements will focus on:
- **Enhancing the dataset**: Adding images under varied lighting conditions.
- **Refining the control system**: Implementing a PID control system to improve navigational smoothness and response accuracy.

## Future Work
- **Dataset Expansion**: To include more diverse environmental scenarios.
- **Control System Upgrade**: Introducing integral and derivative controls to the existing proportional control setup.
- **Adaptability Tests**: Expanding the application to different robot types and operational contexts.

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

## Contributing
Contributors are welcome! Please fork this repository, make your changes, and submit a pull request with your improvements.

## License
This project is released under the MIT License. See the `LICENSE` file for more information.
