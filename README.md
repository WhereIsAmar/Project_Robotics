# Project_Robotics

Abstract
--------
We are going to be programming a robot arm (Sawyer Arm) to be able to dynamically interact with a human. Our interactions will range from simple high fives to a creative handshake that we create. The purpose of this project is to help us create a “social” interaction between a human and robotic mechanism. Aspects about the Sawyer arm that will allow us to make it very realistic are the digital screen and the wide variety joints within the arm to alter positions. Additionally, the Sawyer arm has a camera which will be receiving input of the scene with a human. We will be designing a program that is able to control and visualize where the robot arm should be relative to its current state. The program will be written using ROS and will contain the setup functions and various controllers for the desired behaviors. Overall, this project will be unique because we will be learning about the patterns of human movement and effectively apply concepts from the course.


Equipment 
--------
- A Sawyer (Camera is included)
- A laptop with ROS

Deliverables and Implementation Plan
----
- ROS and Python <br>
Lead: Jon McMillan  
Deadline: 11/20
    - [x] Install ROS on individual machines
    - [x] Implement a Python Script that loops over joint configurations
    - [x] Make sure the Python code actually runs the arm
    - [x] Make sure it is able to record and do playback
    - [ ] Tailor the speed or the iterations of the loop to determine the arm sequence

- Record and replay joint angles <br>
Lead: Amar Patel  
Deadline: 11/20
    - [x] Install ROS on individual machines
    - [x] Start brainstorming how to move the arm
    - [x] Verify the simulator is running and working
    - [x] See if Jon's python script has suitable playback 
    - [x] Setting up our GitHub environment to manage everyone's contributions
    - [ ] Create a way for the CV data to communicate with Jon's Python script
    - [ ] Verify that CV data actually triggers the joint sequence

- Website Controller <br>
Lead: Bandar Albegmi  
Deadline: 11/30  
  - [x] Build a basic website that has Start/Stop using Bootstrap
  - [ ]  Determine how to communicate over ROS and send data to Simulator
  - [ ] Add images of our faces and enhance the design
 
- Create a vision controller <br>
Lead: Abdullah Baijkhaif  
Deadline: 12/4
  - [x] Install OpenCV Python Package & Verify it works with a webcam
  - [ ]  Recognize an individual from our group
  - [ ] If an individual is recognized, perform a special handshake.
  
Demo
--------
We will be sharing our project through a video that we will produce. The video will be composed of scenes that show each of us interacting with the robotic arm. We are restricted to a video rather than a live demo because the robot is currently only accessible in a lab. There will be a demonstration of the vision system and the performance of special handshake. Ideally, we will want to have different handshakes for each of the individuals within our group. In the video, we want to highlight the creative handshakes we came up with and our interactive environment with Sawyer.

  

