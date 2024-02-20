# Team Member

- 2059003 - Dinh Nguyet Anh
- 2159015 - Luu Hoang Thuy Van

# Sokoban Solver

This repository contains a Sokoban solver that applies various search algorithms to solve Sokoban puzzles. 
Sokoban is a classic puzzle game that involves pushing boxes to designated target locations. 
Our solver implements 
  + Breadth-First Search (BFS),
  + Depth-First Search (DFS),
  + Uniform-Cost Search (UCS),
  + Greedy Best-First Search (GBFS),
  + A* Search,
  + and Iterative Deepening A* (IDA*)
to compare their performance on these puzzles.

## System Requirements

- MacBook Pro (M2 chip)
- 16 GB RAM
- 512 GB SSD

## Algorithms Implemented

- BFS: Ensures the shortest path in a breadth-wise manner.
- DFS: Explores possible paths deeply before backtracking.
- UCS: Expands the least costly paths to ensure optimal solutions.
- GBFS: Uses heuristics to find a solution quickly.
- A*: Combines heuristics with cost for efficient search.
- IDA*: Applies A* in an iterative deepening manner for space efficiency.

 # Try out each algorithm

Uncomment each algorithm you'd like to try

    # strategy = 'bfs'
    # strategy = 'dfs'
    # strategy = 'dfs_limited_depth'
    # strategy = 'astar'
    # strategy = 'ucs'
    # strategy = 'greedy'
    strategy = 'custom'

then cd the path to the project --> run `python3 main.py`

# Performance Comparison
The performance of each algorithm has been evaluated based on several metrics:

- Number of states generated
- Number of expanded nodes
- Number of moves to reach the target state
- Running time
  
The detailed performance comparison can be found in our [report](https://drive.google.com/file/d/1SMGKIV4hZhO1iQh1qw-GYAKvbyohXOF4/view?usp=sharing).

