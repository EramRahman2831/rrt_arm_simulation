# RRT Arm Simulation

This project is a Python-based Rapidly-exploring Random Tree (RRT) simulation for a robotic arm. The program demonstrates how an arm can search for a collision-free path from a starting position to a goal position using randomized sampling and path planning.

## Description

The simulation runs from `main.py` and visualizes the RRT path planning process for a robotic arm. RRT works by randomly generating possible positions, connecting valid points together, and gradually building a tree until it reaches the target goal. This approach is useful in robotics because it helps solve motion planning problems in spaces where direct paths may not be possible due to obstacles or movement constraints.

## Purpose

The purpose of this project is to demonstrate basic robotic motion planning using the RRT algorithm. It can be used as a simple educational example for understanding how path planning works in robotics simulations.

## How to Run

Create and activate a Python virtual environment, install the required packages from `requirements.txt`, and run:

```bash
python main.py