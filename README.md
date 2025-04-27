# LEGO EV3 Line Robot - Group 9 DML


## Lego EV3 Line Robot 
## Project Summary 

The Lego Line Follower Robot project is designed to showcase the application of Object Oriented Programming (OOP) principls in a tangible, real-world scenario using the Lego EV3 robotics kit and the LeJOS Java Library. This project involves building a robot capable of autonomously following a line on a track, detecting obstacles and avoiding them while staying on the path. 

The Project integrates several key functionalities, including the use of ultrasonic sensors for object detection, color sensors, for line tracking, and motors for movement control. The robot is programmed to follow the track using precise timing techniques, handle obsracles by performing avoidance maneuvers, and return to the track seamlessly after detouring around obstracles. 

in addition to the core functionality, the project emphasizes the importance of using threads for concurrent task management, allowing for smooth sensor data collection and motor control in parallel. The communication between these threads is implemented following best practices for thread synchronization. 

The project is developed following programming practises, including clear code structure, modularity, and JavaDoc documentation. The group also have focused on documenting the process throght Jira and Confluence. 

Overall, this project aims to reinforce key concepts of Object Oriented programming through practicle application, while also offering a fun and interactive way to learn about robotics and sensor integration. 

## Team Member roles
> Madu - Scrum Master, Developer
    Contact infor - amk1006121@student.hamk.fi <br>
> Lien - Developer, Tester 
    Contact infor - amk1002343@student.hamk.fi <br>
> Dima - Developer , Tester 
    Contact infor - amk1002944@student.hamk.fi <br>

## Technology Used 

> LeJOS Java Library - For programming Lego EV3 robot.<br>
> Lego EV3 Hardware  - Motors, Sensors, and the Lego EV3 brick. <br>
> Java               - Object Oriented Programming for controlling the robot. <br>

## Setup and Installation 

1.  Install LeJOS Follow the Installation guide. https://hameenamk-my.sharepoint.com/:v:/g/personal/s1239_hamk_fi/EShje9UUMipPuZU8tGLntksBxGJsVStVtNOfhpuicXmSvQ?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=hlY1UY    <br> 
2. Clone the repository <br> 
```bash
git clone https://github.com/LIENPHAM-HAMK/LineRobotDML.git         
```                                    
3. Connect Lego EV3 to your computer <br>
4. Complie and upload code to EV3 using the LeJOS development tools. <br> 

## Usage

1. Make sure your Lego EV3 robot is powered on. <br>
2. Run the complie.bat and process.bat in VS code <br>
3. Upload the .jar file in the console <br>
4. Run the uploaded .jar file <br>
5. The robot will start following the track and detecting the objects, and perform the avoidance maneuvers. <br>

## Sensors and Motors 

> Ultrasonic Sensors : used for Object Detection <br>
> Color Sensor       : Detects the line and changes in light intensity <br>
> Motors             : Control the movement of the Robot <br> 

### Line Following 

Using a light sensor, the robot drives along a black line. Using the PID controller: the robot sticks to the right edge of the black stripe so that it scans both white and black at the same time. Added color filtering and error calculation to make the movement very smooth and not jerky.  

### Object Detection & Avoidance

Using the Ultrasonic Sensor, the robot can detect  obtracle in its path within a radius of 30 degrees. The robot begins to slow down gradually as it approaches the abject by 25cm. Approaching 20cm, the robot turns and begins to move in a arc around the obstracle. After leaving the black line, he starts constantly looking for it.  

### Thread Communication  

The robot has three threads. <br>
Distance Thread continuously reads the distance using the Ulltrasonic Sensor. It uses Deceleration, rotation, and volume when needed. <br>
Light thread constantly scans scans colors using the Light Sensor. Detects a Black line and switches th robot to follow mode. <br>
Drive thread is responsible for controlling the speed of the motors. Uses the PID controller while following the line. During the passage of an obstacle, it controls the sped of the wheel, regulates deceleration /acceleration. <br>
Threads are using shared variables to immediately see changes. 

## JavaDoc Documentation

The generated documentation will be available in the `docs` folder.

## Folder Structure

The workspace contains two folders by default, where:

- `src`: the folder to maintain sources
- `lib`: the folder to maintain dependencies

Meanwhile, the compiled output files will be generated in the `bin` folder by default.

## Project Time Line

![sd_group_9_dml_2025-04-16_06 42pm](https://github.com/user-attachments/assets/7a898c34-cb2b-425c-b4ac-4027b5753b97)

## Project Flow Chart

![Robot Flow Chart (1)](https://github.com/user-attachments/assets/e9f78626-e34e-4705-a019-6d9c18c42f62)

