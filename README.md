# Brownian Motion Robot Simulation

This project implements a Brownian Motion simulation for a robot in a square arena. The robot is modeled as a point and starts from the center of the arena, moving straight. Upon reaching any boundary, the robot performs a smooth, random rotation before resuming its motion.

## Overview

The simulation demonstrates:
- **Brownian Motion Behavior:** The robot moves in a straight line and, upon collision with the arena boundaries, rotates by a random angle (between 90° and 180°) over several frames.
- **Trajectory Visualization:** The robot’s path is dynamically plotted using Matplotlib's animation module.
- **Simple and Modular Design:** The project is structured as a self-contained Python module.

## Features

- **Adjustable Parameters:** Easily change parameters such as arena size and robot speed.
- **Randomized Collisions:** On hitting a boundary, a new random rotation (both angle and duration) is applied.
- **Visual Output:** The simulation uses Matplotlib to animate the robot’s movement and display its trajectory.

## How To Use

### 1. Clone the Repository

Start by cloning the repository to your local machine:

```bash
git clone https://github.com/mntadros/fluid-robot-sim.git
```

### 2. Navigate to the Project Folder

Once cloned, navigate to the project directory:

```bash
cd fluid-robot-sim
```

### 3. Run the Simulation

To run the simulation, execute the Python script:

```bash
python brownian_robot_sim.py
```

Alternatively, you can use the executable file if available:

- **Windows:** Run `brownian_robot.exe`
  
This will launch the Brownian motion simulation of the robot.

## Demo

![Demo of Brownian Motion Robot Simulation](/assets/brownian_motion.gif)
