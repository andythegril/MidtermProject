import argparse

from modules.game_state import GameState
from modules.game_visualization import GameVisualization
from modules.solver import Solver


def load_map(map_path):
    """Load the map from the given path"""
    f = open(map_path, 'r')
    map = []
    for line in f:
        map.append(list(line.strip()))
    f.close()
    return map


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', help='The map file', default='maps/sokoban_extra1.txt')
    parser.add_argument('--strategy', help='The strategy to solve the game', default='testing')
    args = parser.parse_args()

    map = load_map(args.map)

    game_state = GameState(map)
    # strategy = args.strategy
    # strategy = 'bfs'
    # strategy = 'dfs'
    # strategy = 'dfs_limited_depth'
    strategy = 'astar'
    # strategy = 'ucs'
    # strategy = 'greedy'
    # strategy = 'custom'
    # strategy = 'testing'
    print(f"Using strategy: {strategy}")
    solver = Solver(game_state, strategy)
    solver.solve()
    solution = solver.get_solution()

    game_visualization = GameVisualization(game_state, solution)
    game_visualization.start()
