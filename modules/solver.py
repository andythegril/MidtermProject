# Solver for sokuban game using following search strategies:
# - Breadth-first search
# - Depth-first search
# - A* search
# - Uniform-cost search
# - Greedy search
# - Custom strategy
# The solver class has the following methods:
# - solve(): solve the game
# """

import time
import heapq
from collections import deque


class Solver(object):
    def __init__(self, initial_state, strategy):
        self.initial_state = initial_state
        self.strategy = strategy
        self.solution = None
        self.time = None
        self.states_generated = 0
        self.expanded_nodes = 0
        self.moves_to_target = 0

    def solve(self):
        start_time = time.time()
        if self.strategy == 'bfs':
            self.solution = self.bfs()
        elif self.strategy == 'dfs':
            self.solution = self.dfs()
        elif self.strategy == 'dfs_limited_depth':
            self.solution = self.dfs_limited_depth()
        elif self.strategy == 'astar':
            self.solution = self.astar()
        elif self.strategy == 'ucs':
            self.solution = self.ucs()
        elif self.strategy == 'greedy':
            self.solution = self.greedy()
        elif self.strategy == 'custom':
            self.solution = self.custom()
        else:
            raise Exception('Invalid strategy')
        self.time = time.time() - start_time
        self.print_solution()

    def print_solution(self):
        if self.solution is not None:
            print("Solution found:", self.solution)
            print("Number of states generated:", self.states_generated)
            print("Number of expanded nodes:", self.expanded_nodes)
            print("Number of moves to reach the target state:", self.moves_to_target)
            print("Running time to find the solution:", self.time, "seconds")
        else:
            print("No solution found.")

    # ref from W4-Lab revision from Mr.Le
    def bfs(self):
        print("Starting BFS")
        queue = deque([(self.initial_state, [])])
        visited = set()
        print(f"Initial queue: {queue}")
        while queue:
            state, solution = queue.popleft()
            print(f"Exploring state with solution {solution}")
            if state.check_solved():
                self.solution = solution
                self.moves_to_target = len(solution)
                return solution
            for direction in ['U', 'D', 'L', 'R']:
                print(f"Generated new state for direction {direction}")
                new_state = state.move(direction)
                self.states_generated += 1

                # Check if the move results in a valid state
                if new_state not in visited:
                    print(f"Adding new state to queue with solution {solution + [direction]}")
                    visited.add(new_state)
                    queue.append((new_state, solution + [direction]))
                    self.expanded_nodes = len(visited)
        return None

    def dfs(self):
        print("Starting DFS")
        stack = [(self.initial_state, [])]
        visited = set()
        self.states_generated = 0
        self.expanded_nodes = 0
        print(f"Initial stack: {stack}")
        while stack:
            state, path = stack.pop()
            self.expanded_nodes += 1
            print(f"Exploring state with solution {path}")
            if state.check_solved():
                self.solution = path
                self.moves_to_target = len(path)
                return path
            visited.add(state)
            for direction in ['U', 'D', 'L', 'R']:
                new_state = state.move(direction)
                self.states_generated += 1
                # double check on valid state, not yet visited
                if new_state is not state and new_state not in visited:
                    stack.append((new_state, path + [direction]))
        return None

    def dfs_limited_depth(self, max_depth=10):
        stack = [(self.initial_state, [], 0)]
        visited = set()
        self.states_generated = 0
        self.expanded_nodes = 0
        while stack:
            state, path, depth = stack.pop()
            self.expanded_nodes += 1
            print(f"Exploring state with solution {path} at depth {depth}")

            if depth > max_depth:
                continue

            if state.check_solved():
                self.solution = path
                self.moves_to_target = len(path)
                return path

            visited.add(state)
            for direction in ['U', 'D', 'L', 'R']:
                new_state = state.move(direction)
                self.states_generated += 1
                if new_state is not state and new_state not in visited:
                    stack.append((new_state, path + [direction], depth + 1))
        return None

    # ref from https://stackoverflow.com/questions/62531674/speed-up-a-implementation-in-python
    def astar(self):
        open_list = []
        heapq.heappush(open_list, (0 + self.initial_state.get_heuristic(), 0, self.initial_state, []))
        visited = set()
        self.states_generated = 0
        self.expanded_nodes = 0
        while open_list:
            # ignore unpacked values https://python.oneoffcoder.com/tuple-ignoring.html
            # ignore total_cost of the current value as it does not impact future decisions
            _, cost, state, path = heapq.heappop(open_list)
            self.expanded_nodes += 1
            if state.check_solved():
                self.moves_to_target = len(path)
                return path
            if state in visited:
                continue
            visited.add(state)

            for direction in ['U', 'D', 'L', 'R']:
                new_state = state.move(direction)
                self.states_generated += 1
                if new_state not in visited:
                    new_cost = cost + 1
                    total_cost = new_cost + new_state.get_heuristic()
                    heapq.heappush(open_list, (total_cost, new_cost, new_state, path + [direction]))
        return None

    def ucs(self):
        open_list = []
        heapq.heappush(open_list, (0, self.initial_state, []))
        visited = set()
        self.states_generated = 0
        self.expanded_nodes = 0

        while open_list:
            cost, state, path = heapq.heappop(open_list)
            self.expanded_nodes += 1

            if state in visited and state != self.initial_state:
                continue

            visited.add(state)

            if state.check_solved():
                self.moves_to_target = len(path)
                return path

            for direction in ['U', 'D', 'L', 'R']:
                new_state = state.move(direction)
                self.states_generated += 1

                if new_state not in visited or new_state == self.initial_state:
                    new_cost = cost + 1
                    new_path = path + [direction]
                    heapq.heappush(open_list, (new_cost, new_state, new_path))
        return None

    def greedy(self):
        open_list = []
        heapq.heappush(open_list, (self.initial_state.get_heuristic(), self.initial_state, []))
        visited = set()
        self.states_generated = 0
        self.expanded_nodes = 0

        while open_list:
            heuristic, state, path = heapq.heappop(open_list)
            self.expanded_nodes += 1

            if state in visited:
                continue

            visited.add(state)

            if state.check_solved():
                self.moves_to_target = len(path)
                return path

            for direction in ['U', 'D', 'L', 'R']:
                new_state = state.move(direction)
                self.states_generated += 1

                if new_state not in visited:
                    new_heuristic = new_state.get_heuristic()
                    heapq.heappush(open_list, (new_heuristic, new_state, path + [direction]))

    # Genetic Algorithms
    def custom(self):
        return ['U', 'U',]

    def get_solution(self):
        return self.solution
