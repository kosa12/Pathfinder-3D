
# 3D Pathfinding with Obstacles

This repository contains code for solving the problem of finding the shortest path on a 3D terrain grid while navigating around obstacles. The solution utilizes the A* search algorithm and provides visualizations of the optimal path in both heatmap and 3D plot formats.

# Problem Description

The problem involves navigating from a start point to a goal point on a 3D terrain grid. Each point on the grid represents a location with a specific height, and certain points are designated as obstacles. The objective is to find the shortest path from the start to the goal point while avoiding these obstacles.


# Solution Overview

The solution employs the A* search algorithm, a popular pathfinding algorithm known for its efficiency and accuracy in finding the shortest path in a graph. A heuristic function is used to guide the search towards the goal while considering the terrain's topology and the presence of obstacles.

# The following dependencies are required to run the code:

- matplotlib==3.4.3
- numpy==1.21.2

# Getting Started

To run the pathfinding algorithm and visualize the results:

1. Clone this repository to your local machine.
2. Ensure you have Python installed.
3. Install the required dependencies by running `pip install -r requirements.txt`.
4. Run the `pathfinder.py` script using Python.


# Screenshots

![App Screenshot](https://github.com/kosa12/Pathfinder-3D/blob/main/100x100png)
![App Screenshot](https://github.com/kosa12/Pathfinder-3D/blob/main/100x100_2png)
