class Node:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.neighbors = []
        self.start = False
        self.end = False
        self.obstacle = False
        self.closed = False
        self.path = False
        self.h = 0

    def get_pos(self):
        return (self.row, self.col)

    def is_obstacle(self):
        return self.obstacle

    def is_start(self):
        return self.start

    def is_end(self):
        return self.end

    def make_obstacle(self):
        self.obstacle = True

    def make_start(self):
        self.start = True

    def make_end(self):
        self.end = True

    def update_neighbors(self, grid):
        self.neighbors = []

        if self.row < len(grid) - 1 and not grid[self.row + 1][self.col].is_obstacle(): # DOWN
            self.neighbors.append((grid[self.row + 1][self.col], False))

        if self.row > 0 and not grid[self.row - 1][self.col].is_obstacle(): # UP
            self.neighbors.append((grid[self.row - 1][self.col], False))

        if self.col > 0 and not grid[self.row][self.col - 1].is_obstacle(): # LEFT
            self.neighbors.append((grid[self.row][self.col - 1], False))

        if self.col < len(grid[0]) - 1 and not grid[self.row][self.col + 1].is_obstacle(): # RIGHT
            self.neighbors.append((grid[self.row][self.col + 1], False))


        if self.row < len(grid) - 1 and self.col > 0: # DOWN-LEFT
            if not grid[self.row + 1][self.col - 1].is_obstacle():
                self.neighbors.append((grid[self.row + 1][self.col - 1], True))

        if self.row < len(grid) - 1 and self.col < len(grid[0]) - 1: # DOWN-RIGHT
            if not grid[self.row + 1][self.col + 1].is_obstacle():
                self.neighbors.append((grid[self.row + 1][self.col + 1], True))

        if self.row > 0 and self.col > 0: # UP-LEFT
            if not grid[self.row - 1][self.col - 1].is_obstacle():
                self.neighbors.append((grid[self.row - 1][self.col - 1], True))

        if self.row > 0 and self.col < len(grid[0]) - 1: # UP-RIGHT
            if not grid[self.row - 1][self.col + 1].is_obstacle():
                self.neighbors.append((grid[self.row - 1][self.col + 1], True))


