"""
Sokuban game state class
The state of the game consists the map which is a 2D array of characters. There are 6 types of characters:
- ' ': empty space
- '#': wall
- '$': box
- '.': target
- '@': player
- '+': player on target
- '*': box on target
The game state class keeps track of the map.
The game state also keeps track of the player and box positions, and whether the game is solved or not.
The game state class has the following methods:
- find_player(): find the player in the map and return its position
- find_boxes(): find all the boxes in the map and return their positions
- find_targets(): find all the targets in the map and return their positions  
- generate_next_state(direction): generate the next game state by moving the player to the given direction
- check_solved(): check if the game is solved
"""


class GameState:
    def __init__(self, map, current_cost=0):
        self.map = map
        self.player = self.find_player()
        self.boxes = self.find_boxes()
        self.targets = self.find_targets()
        self.is_solved = self.check_solved()
        self.current_cost = current_cost
        self.height = len(self.map)
        self.width = len(self.map[0])

    def __eq__(self, other):
        return isinstance(other, type(self)) and self.map == other.map and self.player == other.player

    # error Answer by Michal Polovka: heapq push TypeError: '<' not supported between instances
    # https://stackoverflow.com/questions/53554199/heapq-push-typeerror-not-supported-between-instances
    def __lt__(self, other):
        return (self.current_cost + self.get_heuristic()) < (other.current_cost + other.get_heuristic())

    def __hash__(self):
        return hash((tuple(tuple(row) for row in self.map), self.player, tuple(self.boxes)))

    # ------------------------------------------------------------------------------------------------------------------
    # The following methods are used to find the player, boxes, and targets in the map
    # The positions are tuples (row, column)
    # ------------------------------------------------------------------------------------------------------------------

    def find_player(self):
        """Find the player in the map and return its position"""
        # TODO: implement this method
        for row_index, row in enumerate(self.map):
            for col_index, cell in enumerate(row):
                if cell == '@' or cell == '+':
                    return row_index, col_index

    def find_boxes(self):
        """Find all the boxes in the map and return their positions"""
        boxes = []
        for row_index, row in enumerate(self.map):
            for col_index, cell in enumerate(row):
                if cell == '$' or cell == '*':
                    boxes.append((row_index, col_index))
        return boxes

    def find_targets(self):
        """Find all the targets in the map and return their positions"""
        targets = []
        for row_index, row in enumerate(self.map):
            for col_index, cell in enumerate(row):
                if cell == '.' or cell == '*' or cell == '+':
                    targets.append((row_index, col_index))
        return targets

    # ------------------------------------------------------------------------------------------------------------------
    # The following methods are used to check if a position is a wall, box, target, or empty space
    # The position is a tuple (row, column)
    # ------------------------------------------------------------------------------------------------------------------

    def copy_map_with_player_position(self, new_x, new_y):
        new_map = [row[:] for row in self.map]
        new_map[self.player[0]][self.player[1]] = ' '

        if self.is_target((self.player[0], self.player[1])):
            new_map[self.player[0]][self.player[1]] = '.'
        else:
            new_map[self.player[0]][self.player[1]] = ' '

        if self.is_target((new_x, new_y)):
            new_map[new_x][new_y] = '+'
        else:
            new_map[new_x][new_y] = '@'
        return new_map

    def copy_map_with_player_and_box_position(self, new_x, new_y, beyond_x, beyond_y):
        new_map = [row[:] for row in self.map]

        if self.is_target((self.player[0], self.player[1])):
            new_map[self.player[0]][self.player[1]] = '.'
        else:
            new_map[self.player[0]][self.player[1]] = ' '

        if self.is_target((new_x, new_y)):
            new_map[new_x][new_y] = '+'
        else:
            new_map[new_x][new_y] = '@'

        if self.is_target((beyond_x, beyond_y)):
            new_map[beyond_x][beyond_y] = '*'
        else:
            new_map[beyond_x][beyond_y] = '$'

        return new_map

    def is_wall(self, position):
        """Check if the given position is a wall"""
        row, col = position
        return self.map[row][col] == '#'

    def is_box(self, position):
        """Check if the given position is a box
            Note: the box can be on "$" or "*" (box on target)
        """
        row, col = position
        return self.map[row][col] in ['$', '*']

    def is_target(self, position):
        """Check if the given position is a target
            Note: the target can be "." or "*" (box on target)
        """
        row, col = position
        return self.map[row][col] in ['.', '*', '+']

    def is_empty(self, position):
        """Check if the given position is empty"""
        row, col = position
        return self.map[row][col] in [' ', '.']

    # ------------------------------------------------------------------------------------------------------------------
    # The following methods get heuristics for the game state (for informed search strategies)
    # ------------------------------------------------------------------------------------------------------------------

    def get_heuristic(self):
        """Get the heuristic for the game state
            Note: the heuristic is the sum of the distances from all the boxes to their nearest targets
        """
        # check if target is populated/retained
        if not self.targets:
            return float('inf')

        heuristic = 0
        for box in self.boxes:
            nearest_target_distance = min(abs(box[0] - target[0]) + abs(box[1] - target[1]) for target in self.targets)
            heuristic += nearest_target_distance
        return heuristic

    def get_total_cost(self):
        """Get the cost for the game state
            Note: the cost is the number of moves from the initial state to the current state + the heuristic
        """
        return self.get_current_cost() + self.get_heuristic()

    def get_current_cost(self):
        """Get the current cost for the game state
            Note: the current cost is the number of moves from the initial state to the current state
        """
        return self.current_cost

    # ------------------------------------------------------------------------------------------------------------------
    # The following methods are used to generate the next game state and check if the game is solved
    # ------------------------------------------------------------------------------------------------------------------

    def move(self, direction):
        """Generate the next game state by moving the player to the given direction. 
            The rules are as follows:
            - The player can move to an empty space
            - The player can move to a target
            - The player can push a box to an empty space (the box moves to the empty space, the player moves to the box's previous position)
            - The player can push a box to a target (the box moves to the target, the player moves to the box's previous position)
            - The player cannot move to a wall
            - The player cannot push a box to a wall
            - The player cannot push two boxes at the same time
        """
        # TODO: implement this method
        dx, dy = 0, 0
        if direction == 'U':
            dx = -1
        elif direction == 'D':
            dx = 1
        elif direction == 'L':
            dy = -1
        elif direction == 'R':
            dy = 1

        new_x, new_y = self.player[0] + dx, self.player[1] + dy

        if self.is_wall((new_x, new_y)):
            print("Move invalid: Wall")
            return self
        print(f"Moving {direction}, New State Generated")

        # if self.is_empty((new_x, new_y)) or self.is_target((new_x, new_y)):
        if self.is_empty((new_x, new_y)):
            new_map = self.copy_map_with_player_position(new_x, new_y)

            return GameState(new_map, self.current_cost + 1)

        elif self.is_box((new_x, new_y)):
            beyond_x, beyond_y = new_x + dx, new_y + dy
            # if self.is_empty((beyond_x, beyond_y)) or self.is_target((beyond_x, beyond_y)):            if self.is_empty((beyond_x, beyond_y)) or self.is_target((beyond_x, beyond_y)):
            if self.is_empty((beyond_x, beyond_y)):
                print(f"Pushing box from ({new_x}, {new_y}) to ({beyond_x}, {beyond_y})")
                new_map = self.copy_map_with_player_and_box_position(new_x, new_y, beyond_x, beyond_y)
                return GameState(new_map, self.current_cost + 1)
            else:
                print("Box push invalid: Blocked")
                pass
        return self

    def check_solved(self):
        """Check if the game is solved"""
        for box in self.boxes:
            x, y = box
            if self.map[x][y] != "*":
                return False
        return True
