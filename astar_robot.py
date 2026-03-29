import numpy as np
import matplotlib.pyplot as plt
import time

# Helper Functions 
def calculateDistance(n1, n2):
    return np.sqrt((n1[0] - n2[0])**2 + (n1[1] - n2[1])**2)

def getNeighbors(node, rows, cols, grid):
    r, c = node
    moves = [[-1,0], [1,0], [0,-1], [0,1], [-1,-1], [-1,1], [1,-1], [1,1]]
    neighbors = []
    for dr, dc in moves:
        nr, nc = r + dr, c + dc
        if 1 <= nr <= rows and 1 <= nc <= cols:
            # Map index conversion (1-based to 0-based for numpy)
            if grid[nr-1, nc-1] != -1:
                # Safety corner check
                if abs(dr) + abs(dc) == 2:
                    if grid[r + dr - 1, c - 1] == -1 or grid[r - 1, c + dc - 1] == -1:
                        continue
                neighbors.append([nr, nc])
    return neighbors

def isNodeInList(node, search_list):
    for i, entry in enumerate(search_list):
        if entry[0] == node[0] and entry[1] == node[1]:
            return True, i
    return False, -1

def reconstructPath(closedList, startNode, goalNode):
    path = [goalNode]
    curr = goalNode
    while list(curr) != list(startNode):
        _, idx = isNodeInList(curr, closedList)
        parent = [closedList[idx][5], closedList[idx][6]]
        path.insert(0, parent)
        curr = parent
    return np.array(path)

# Case Configuration 
cases = [
    {
        "start": [1, 1], "goal": [10, 10], 
        "obs": [[1,5], [2,5], [3,5], [3,9], [4,2], [4,5], [4,7], [4,8], [5,2], [5,3], [5,7], [5,9], [6,3], [6,4], [6,5], [6,8], [6,9], [7,5], [7,9], [8,6], [8,7], [8,8], [9,5], [9,9], [10,9]]
    },
    {
        "start": [1, 10], "goal": [10, 10], 
        "obs": [[1,5], [2,5], [3,5], [4,5], [5,5], [6,5], [7,5], [8,5], [8,6], [9,5], [9,6], [10,5]]
    },
    {
        "start": [1, 1], "goal": [10, 10], 
        "obs": [[1,4], [1,5], [1,7], [1,8], [2,3], [2,4], [2,8], [3,6], [3,8], [4,3], [4,6], [4,8], [5,3], [5,4], [5,6], [5,9], [6,3], [6,4], [6,7], [7,3], [7,4], [7,7], [7,9], [8,4], [8,5], [8,9], [9,2], [9,6], [9,7], [10,4], [10,9]]
    },
    {
        "start": [1, 1], "goal": [10, 10], 
        "obs": [[2,1], [2,2], [2,3], [2,4], [2,5], [2,6], [2,7], [2,8], [4,3], [4,4], [4,5], [4,6], [4,7], [4,8], [4,9], [4,10], [7,9], [8,4], [8,5], [8,9], [9,2], [9,6], [9,7], [10,4], [10,9]]
    },
    {
        "start": [1, 1], "goal": [10, 10], 
        "obs": [[1,5], [2,5], [2,6], [2,7], [2,8], [3,3], [3,4], [3,5], [3,9], [4,2], [4,3], [4,4], [4,5], [4,6], [4,7], [4,8], [4,9], [4,10], [5,5], [6,6], [7,7], [7,9], [8,4], [8,8]]
    }
]

max_x, max_y = 10, 10

max_x, max_y = 10, 10

print("   Case   |   Status   |   Computation Time (s) ")

#  Main Automation Loop 
for i, case_data in enumerate(cases):
    case_id = i + 1
    # Use perf_counter for high resolution timing
    start_time = time.perf_counter() 
    
    # Setup Map
    grid = 2 * np.ones((max_x, max_y))
    start_node = case_data["start"]
    goal_node = case_data["goal"]
    obs_list = case_data["obs"]
    
    for obs in obs_list:
        grid[obs[0]-1, obs[1]-1] = -1
        
    # Initialize Lists
    open_list = []
    closed_list = []
    
    # Pre-fill obstacles in closed_list
    for obs in obs_list:
        closed_list.append([obs[0], obs[1], 0, 0, 0, 0, 0])
        
    f_start = calculateDistance(start_node, goal_node)
    open_list.append([start_node[0], start_node[1], f_start, 0, f_start, start_node[0], start_node[1]])
    
    # Search Loop
    final_path = []
    while open_list:
        open_list.sort(key=lambda x: x[2])
        current = open_list.pop(0)
        curr_pos = [current[0], current[1]]
        closed_list.append(current)
        
        if curr_pos == goal_node:
            final_path = reconstructPath(closed_list, start_node, goal_node)
            break
            
        for neighbor in getNeighbors(curr_pos, max_x, max_y, grid):
            in_closed, _ = isNodeInList(neighbor, closed_list)
            if in_closed: continue
            
            g_neighbor = current[3] + calculateDistance(curr_pos, neighbor)
            h_neighbor = calculateDistance(neighbor, goal_node)
            f_neighbor = g_neighbor + h_neighbor
            
            in_open, open_idx = isNodeInList(neighbor, open_list)
            if not in_open or g_neighbor < open_list[open_idx][3]:
                entry = [neighbor[0], neighbor[1], f_neighbor, g_neighbor, h_neighbor, curr_pos[0], curr_pos[1]]
                if in_open:
                    open_list[open_idx] = entry
                else:
                    open_list.append(entry)

    # Stop timing here
    time_taken = time.perf_counter() - start_time
    status = "Success" if len(final_path) > 0 else "Failed "
    print(f"    {case_id:02d}    |  {status}   |       {time_taken:.6f}")
    
    # Visualization
    plt.figure(case_id)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.title(f"Case {case_id}: Start {start_node} Time: {time_taken:.4f}s")
    
    # Plot obstacles
    for obs in obs_list:
        plt.plot(obs[0]+0.5, obs[1]+0.5, 'ro')
        
    # Plot Start/Goal
    plt.plot(start_node[0]+0.5, start_node[1]+0.5, 'bs', markersize=10)
    plt.plot(goal_node[0]+0.5, goal_node[1]+0.5, 'gd', markersize=10)
    
    if len(final_path) > 0:
        plt.plot(final_path[:,0]+0.5, final_path[:,1]+0.5, 'k-', linewidth=2)
        
    plt.xticks(np.arange(1, 12))
    plt.yticks(np.arange(1, 12))
    plt.xlim(1, 11)
    plt.ylim(1, 11)

plt.show()