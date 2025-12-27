# BE_Robotique
Ce projet est réalisé par: 

Ahmed Mayekhav, 

Khalil Rezgui, 

Mohamed Yassin Ghomrasni 

Wajdi Dridi.


**Robotic Arm Control using ROS 2 (Humble), Raspberry Pi & Arduino 📌 Project Overview
**


This project implements the control of a 4-DOF robotic arm using a Raspberry Pi for high-level control and an Arduino for low-level servo actuation.

Raspberry Pi: trajectory planning, vision input processing, ROS 2 nodes execution

Arduino: direct control of servo motors via PWM

Middleware: ROS 2 Humble running inside a Docker container

Programming language: Python (ROS 2 nodes), Arduino C++

🧠 System Architecture Ball Pose (x, y, z) │ ▼ trajectory_planner.py │ publishes ▼ /servo_cmd ───► servo_node.py │ ▼ servo_control.py │ (Serial) ▼ Arduino │ ▼ 4 Servo Motors

📂 ROS 2 Workspace Structure

The ROS 2 workspace contains the following essential Python nodes:

1️⃣ trajectory_planner.py

Subscribes to: /ball_pose

Message format:

{x: ..., y: ..., z: ...}

Publishes to: /servo_cmd

Computes the required joint angles based on the ball position.

2️⃣ servo_node.py

Subscribes to: /servo_cmd

Message format:

i:angle

where:

i → servo index (1 to 4)

angle → desired angle in degrees

3️⃣ servo_control.py

Sends the servo commands to the Arduino via serial communication.

Acts as a bridge between ROS 2 and the Arduino board.

⚙️ Arduino Configuration

Servo pins:

Servo 1 → Pin 3

Servo 2 → Pin 5

Servo 3 → Pin 6

Servo 4 → Pin 9

The Arduino code is minimal and only handles:

Serial command reception

PWM generation for servo motors

🔒 Joint Constraints

The following joint limits are enforced in the control logic:

Joint 2:

[130°, 180°]

Joint 3:

[30°, 100°] (inverse direction)

🐳 Docker & ROS 2 (Humble) Useful Docker Commands <container_name> = ros2_humble in our case

List existing Docker containers
docker ps -a

Start a Docker container
docker start <container_name>

Execute a container
docker exec -it <container_name> bash

Build ROS 2 Workspace cd ~/ros2_ws colcon build source install/setup.bash

📡 ROS 2 Useful Commands

List all topics
ros2 topic list

Display messages from a topic
ros2 topic echo /servo_cmd

Get information about a topic
ros2 topic info /ball_pose

🔌 Arduino Serial Port

To identify the Arduino serial port:

ls /dev/tty*

Most likely:

/dev/ttyUSB0

✏️ Editing Python Scripts

⚠️ Important To modify Python scripts inside the Docker/container environment, you must open the editor with administrator privileges:

sudo geany &

🌐 Raspberry Pi Connection Instructions Wi-Fi Configuration

The Raspberry Pi is already configured to connect to:

SSID: BEE_WIFI

Password: Sou1919@1908Hail

To connect:

Configure your PC or phone hotspot with the same SSID and password

Power on the Raspberry Pi

Remote Access (VNC)

Install VNC Viewer

In the search bar, enter:

pi.local

Name : pi Password: 0000

Once connected, you can execute scripts directly or via SSH.

📁 File Transfer (PC ⇄ Raspberry Pi)

Sometimes copy/paste may not work correctly. Recommended solution:

Install WinSCP

Use it to copy files from your PC to the Raspberry Pi

Then move or paste them into your ROS workspace on the Pi
