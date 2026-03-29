# A-Star Pathfinding Robot (MATLAB & Python)

This project contains two versions (MATLAB and Python) of the A* Search Algorithm. The goal is to find the shortest and most efficient path for a robot in a 10×10 grid while avoiding various obstacles.

## What does this code do?

The code automatically tests 5 different map scenarios. For each case, it:

- Sets up a start point, a goal point, and several obstacles  
- Uses the A* algorithm to calculate the best path (balancing the distance already traveled and the estimated distance to the goal)  
- Measures the computation time  
- Draws a graph showing the final path  

## Project Workflow

- **Initialize Map**: Defines the 10×10 grid and places obstacles  
- **Pathfinding**: The algorithm explores nodes using the formula  
  `f(n) = g(n) + h(n)`  
- **Safety Check**: Prevents the robot from "cutting corners" through diagonal walls  
- **Visualization**: Creates a plot showing the robot's path  
- **Performance Table**: Prints the success status and timing for every case in the console  

## How to use it

### MATLAB
- Open `astar_robot.m` in MATLAB  
- Press Run  
- Check the Command Window for the timing table and view the generated figures  

### Python
- Ensure you have `numpy` and `matplotlib` installed  
- Run the script:
  ```bash
  python astar_robot.py

