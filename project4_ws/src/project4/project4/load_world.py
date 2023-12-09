import yaml

class World:
    def __init__(self, world_file):
        with open(world_file, 'r') as file:
            self.world_data = yaml.safe_load(file)
        
        self.resolution = self.world_data['resolution']
        self.goal_resolution = 0.1
        self.resolution_translation = int(self.resolution/self.goal_resolution)
        self.initial_pose = self.world_data['initial_pose']
        self.raw_map = self.world_data['map'].strip().split('\n')

        self.width = 0
        self.height = 0
        self.occupancy_grid = self.create_occupancy_grid(self.raw_map)
        self.grid_2d = self.create_grid_2d(self.raw_map)

    def create_occupancy_grid(self, raw_map):
        """
        Convert the ASCII map into an occupancy grid.
        # is considered an obstacle and . is free space.
        """
        occupancy_grid = []
        for line in raw_map[::-1]:
            self.height += 1
            self.width = 0
            for char in line:
                self.width += 1
                if char == '#': # Obstacle
                    occupancy_grid.append(100) # Occupied space is denoted by 100
                elif char == '.': # Free space
                    occupancy_grid.append(0) # Free space is denoted by 0
        return occupancy_grid
    
    def create_grid_2d(self, raw_map): # GOAL RESOLUTION: 0.1
        grid_2d = []
        for line in raw_map[::-1]:
            row = []
            for char in line:
                for i in range(self.resolution_translation):
                    if char == '#':
                        row.append(100)
                    elif char == '.':
                        row.append(0)
            for i in range(self.resolution_translation):
                grid_2d.append(row)
        return grid_2d
                    
    def find_obstacles(self, resolution):
        obstacles = []
        rows = self.raw_map

        for y, row in enumerate(rows):
            for x, cell in enumerate(row):
                if cell == '#':
                    # Calculate the world coordinates of the obstacle's corners
                    top_left = (x * resolution, (len(rows) - y) * resolution)
                    top_right = ((x + 1) * resolution, (len(rows) - y) * resolution)
                    bottom_left = (x * resolution, (len(rows) - y - 1) * resolution)
                    bottom_right = ((x + 1) * resolution, (len(rows) - y - 1) * resolution)

                    obstacles.append([top_left, top_right, bottom_right, bottom_left])

        return obstacles

    def get_resolution(self):
        return self.resolution

    def get_goal_resolution(self):
        return self.goal_resolution

    def get_resolution_translation(self):
        return self.resolution_translation

    def get_initial_pose(self):
        return self.initial_pose
    
    def get_dimensions(self):
        return self.width, self.height

    def get_occupancy_grid(self):
        return self.occupancy_grid
    
    def get_grid_2d(self):
        return self.grid_2d

    def get_obstacles(self):
        return self.find_obstacles(self.resolution)

if __name__ == '__main__':
    world = World('brick.world')
    print("Initial Pose:", world.get_initial_pose())
    print("Occupancy Grid:")
    grid = world.get_occupancy_grid()
    index = 0
    for row in range(world.get_dimensions()[1]):
        for col in range(world.get_dimensions()[0]):
            print(grid[index], end='')
            index += 1
        print()

    for row in world.get_grid_2d():
        print(row)
