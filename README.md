## Project Overview
This project focuses on integrating a real-time object detection system into a differential robot using ROS (Robot Operation System 2) and the YOLO v8 Nano architecture. The main objective is to autonomously detect and navigate towards available power outlets for charging, enhancing the robot's operational autonomy across different environments.

<p align="center">
<img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/OutletSeekr_480.gif?raw=true" alt="final_gif" width="900"/>

                   Figure 1: Working project. Robot autonomously searching and pursuing available power outlet for docking.
</p>

## Features
- **Object Detection**: Employs YOLO v8 Nano for high-efficiency and accuracy in real-time power outlet detection.
- **Proportional Control Algorithm**: Implements proportional controllers for navigation based on detection outputs.
- **Simulation and Real Environment Testing**: Validates the model's effectiveness through extensive video simulations and real-environment tests.

## Technologies Used
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
The data annotation of the second set of images, which contained the power outlet images from the room where the robot would operate, was performed in the cvat.ai website. The outlets in use by another devices were purposefully ignored, so that they would not be detected / considered as a goal.

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

<p align="center">
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/Training12.png?raw=true" alt="training1" width="430"/>
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/Training13.png?raw=true" alt="training1" width="330"/>
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/Training1.png?raw=true" alt="training1" width="650"/>
</p>

#### Second Stage of Training
- **Dataset**: The 312 newly captured and annotated images were used, focusing specifically on the test room environment.
- **Split**: Due to the smaller dataset size, a 90% training and 10% test split was used.
- **Epochs**: Reduced to 34, considering the specific adaptation required and the smaller dataset.
- **Batch Size**: Slightly decreased to 15, to adjust for the dataset's characteristics.
- **Patience Parameter**: Maintained at 50, for consistency in handling potential overfitting.

<p align="center">
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/Trainin21.png?raw=true" alt="training22" width="550"/>
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/Training23.png?raw=true" alt="training23" width="330"/>
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/Training2.png?raw=true" alt="training2" width="650"/>
</p>

### Evaluation and Adjustments
- **Performance Monitoring**: Throughout the training process, the model’s performance was regularly evaluated using the validation set, allowing for adjustments in parameters if necessary.
- **Fine-Tuning**: The two-stage training approach ensured that the model was not only familiar with general features of power outlets but also finely adjusted to recognize the specific outlets it would encounter in its operational environment.

This structured approach to training ensures that the YOLO v8 Nano model is robust, efficient, and highly functional for the specific task of detecting power outlets in real-time, which is critical for the autonomous operations of the differential robot.


## 4. Model Performance Testing
- **Video Simulation**: Conducted by running the trained model on a video recorded hovering the iPhone camera at the robot's camera height through the operating environment as if it was the actual robot.
<p align="center">
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/Simulation.gif?raw=true" alt="sim" width="650"/>
</p>

- The results from testing in the video proved the model to be working very well, being potentially ready to be implemented in the robot, where the data from the bounding boxes would be processed to feed the control system.
  
## 5. Processing YoloV8 results to perform Control Actions

The effective navigation of the differential robot towards power outlets heavily relies on the accurate interpretation of visual data captured by its camera. This section elaborates on the bounding box centroid and area calculation,  and their relationship to the center of the camera’s field of view, which are critical for the control system's functionality.

### Bounding Box Centroid Calculation
- **Purpose**: The centroid of a bounding box represents the central point of the detected object (in this case, a power outlet). It is critical for determining the object's position relative to the robot.
- **Calculation**: The centroid (`Cx, Cy`) is calculated as the average of the bounding box's corners:
  - `Cx = (x_min + x_max) / 2`
  - `Cy = (y_min + y_max) / 2`
- **Usage**: The centroid's coordinates are used to compute the yaw control, specifically adjusting the robot's heading to align with the power outlet.

### Area Calculation of the Bounding Box
- **Purpose**: The area of the bounding box is used as a proxy for the distance between the robot and the power outlet. Larger areas suggest the outlet is closer, while smaller areas indicate it is farther away.
- **Calculation**: The area is calculated by multiplying the width and height of the bounding box:
  - `Area = (x_max - x_min) * (y_max - y_min)`
- **Usage**: This measure helps determine the robot’s speed—the closer the object, the slower the robot moves to allow for precise maneuvers.

### Relationship with the Center of the Camera’s Field of View
- **Importance**: Aligning the centroid of the bounding box with the center of the camera's field of view is crucial for direct navigation towards the power outlet.
- **Center of the Camera**: Defined typically as the middle point of the image frame dimensions:
  - `Center_x = Frame Width / 2`
  - `Center_y = Frame Height / 2`
- **Control Adjustments**: The robot uses the relative position of the centroid to the center of the camera to adjust its yaw (rotational movement) and linear speed (forward movement). If the centroid is off-center, the robot adjusts its path to center the outlet in its view before advancing.

### Control System Integration
Using these calculations, the robot's control system dynamically adjusts its orientation and speed based on real-time input from its vision system. This ensures accurate and efficient navigation towards the target, essential for autonomous docking and recharging operations.

The integration of these visual computations into the robot’s navigational controls allows for a sophisticated and responsive system capable of operating autonomously in complex environments.


## 6. Control System Implementation

The control system of the differential robot uses data from the vision system to navigate towards detected power outlets efficiently. This section explains how the yaw and distance control strategies are informed by bounding box calculations from the vision system, with a focus on the mathematical foundations.

### Yaw Control (Orientation Adjustment)
- **Objective**: To ensure the robot is precisely aligned with the power outlet, optimizing its orientation for effective docking.
- **Implementation Details**:
  - **Angular Error**: Defined as the difference between the centroid's angle relative to the camera's central axis.
  - **Control Strategy**: The robot adjusts its yaw by applying a correction that is proportional to the negative of the angular error. This is mathematically expressed as:
    - `Angular Velocity = -K_ang × Angular Error`
  - **Angular Gain (`K_ang`)**: A predefined constant that modulates how responsive the robot is to angular discrepancies, facilitating smooth and accurate adjustments.

### Distance Control (Speed Adjustment)
- **Objective**: To manage the robot's approach speed based on its estimated distance from the power outlet, which is inferred from the area of the bounding box.
- **Implementation Details**:
  - **Distance Estimation**: Utilizes the area of the bounding box as an inverse measure of distance. A larger bounding box indicates closer proximity.
  - **Control Strategy**: The forward speed of the robot is adjusted to be inversely proportional to the distance, modified by the square of the angular error to refine the approach based on alignment accuracy. The relationship is given by:
    - `Forward Speed = K_lin × (1 / Distance) / (α × Angular Error)^2`
    - Here, α is a scaling factor to adjust the influence of angular error on speed. This is meant to incentivize the robot to move really slow if the direction it is pointing is not towards the power outlet. So once it aligns, it goes faster toward the goal.
  - **Linear Gain (`K_lin`)**: Influences the deceleration rate, enhancing control precision during the final approach. Speed is capped to prevent overshooting and ensure safe docking.

### Safe Stopping Mechanism
- **Objective**: Ensure the robot stops at a safe distance from the power outlet without losing visibility.
- **Implementation Details**:
  - **Stop Condition**: The robot monitors the area of the bounding box to determine its proximity to the outlet. When the area reaches or exceeds a predefined threshold, this indicates that the robot is sufficiently close to the outlet.
  - **Tolerance Parameter**: A specific area value (`tol`) is set as the threshold to stop the robot. This parameter ensures that the robot maintains a safe distance where the outlet remains within the camera's field of view, preventing overshooting or misalignment during docking.
  - **Control Adjustment**: Upon reaching the tolerance area, the robot reduces its forward speed to zero and ceases angular adjustments, effectively halting its movement while keeping the outlet in view.

### Integration and Real-Time Adjustments
- **Dynamic Adjustments**: Both yaw and distance controls are dynamically updated based on real-time visual inputs, allowing the robot to adjust its path and speed instantaneously.
- **Safety and Robustness**: Includes mechanisms for capping the maximum speed and robust error handling to ensure reliable operation under various environmental conditions.
- **Feedback Mechanism**: Continuous feedback from the vision system ensures that the robot can correct its orientation and speed based on the latest data, maintaining precise navigation towards the target.

### Summary of Control System Features
- **Responsive**: Adapts in real-time to changes in the detected outlet's position and orientation.
- **Precise**: Maintains exact alignment with the outlet for effective docking, using finely tuned control parameters.
- **Safe**: Implements safety measures such as velocity limits and comprehensive error management.

This control system design leverages bounding box data effectively to guide the differential robot towards power outlets, balancing precision, adaptability, and safety in its autonomous docking procedures.



## 7. Results and Improvements
The model displayed high accuracy in well-lit conditions and adequate performance under artificial lighting. 

- **First video (Left):** Robot working during the natural daylight, which is the condition that the training data was obtained. The YoloV8 model showed superior performance, which allowed a faster and smoother control of the robot.
- **Second video (Right):** Robot working during the night, using artificial lamp light. There was no training data with the power outlets captured under this kind of lightning. It resulted in a less smooth and slower control of the robot. 

<p align="center">
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/daylight_evalGif.gif?raw=true" alt="goodlight" width="220"/>
  <img src="https://github.com/mateusctelles/Power-Outlet-Docking-Robot/blob/main/media/OutletSeekr_480.gif?raw=true" alt="badlight" width="550"/>
</p>

## Future Work
- **Dataset Expansion**: To include more diverse environmental scenarios.
- **Control System Upgrade**: Introducing integral and derivative controls to the existing proportional control setup.
- **Adaptability Tests**: Expanding the application to different robot types and operational contexts.

## Setup and Installation
1. **Clone the Repository**
   ```bash
   git clone <https://github.com/mateusctelles/Power-Outlet-Docking-Robot.git>
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
