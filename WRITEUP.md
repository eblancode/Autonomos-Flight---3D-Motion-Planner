# 3D Motion Planning
## Writeup

## Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

### Test that motion_planning.py is a modified version of backyard_flyer_solution.py for simple path planning. Verify that both scripts work. Then, compare them side by side and describe in words how each of the modifications implemented in motion_planning.py is functioning.

RDF: There is no Planning Step or method in backyard flier. There is a Planning Step in the motion planner.

The steps are Manual -> Arming -> Planning (Only Motion Planner) -> Takeoff -> Waypoint -> Landing -> Disarming -> (Back to Manual)

### In the starter code, we assume that the home position is where the drone first initializes, but in reality you need to be able to start planning from anywhere. Modify your code to read the global home location from the first line of the colliders.csv file and set that position as global home (self.set_home_position())

    def read_file(filename):
        with open(filename) as f:
            for line in islice(f, 1):
                read_pos = line

        read_pos = read_pos.replace(",", "")  # remove ','
        read_pos = read_pos.split()  # split string

        return float(read_pos[1]), float(read_pos[3])
    
    # TODO: read lat0, lon0 from colliders into floating point values
    lat0, lon0 = read_file('colliders.csv')
    
    # TODO: set home position to (lon0, lat0, 0)
    self.set_home_position(lat0, lon0, 0)

### In the starter code, we assume the drone takes off from map center, but you'll need to be able to takeoff from anywhere. Retrieve your current position in geodetic coordinates from self._latitude, self._longitude and self._altitude. Then use the utility function global_to_local() to convert to local position (using self.global_home as well, which you just set)

    # TODO: retrieve current global position
    print("Current global position")
    local_pos = self.global_position

    # TODO: convert to current local position using global_to_local()
    print('Global Position {0}, Global Home {1}', self.global_position, self.global_home)
    local_north, local_east, local_alt = global_to_local(self.global_position, self.global_home)

    print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                        self.local_position))

### In the starter code, the start point for planning is hardcoded as map center. Change this to be your current local position.

    # TODO: convert start position to current position rather than map center
    grid_start = ( int(np.ceil(local_north - north_offset)), int(np.ceil(local_east - east_offset)) )
    print ("Grid_Start:",grid_start)

### In the starter code, the goal position is hardcoded as some location 10 m north and 10 m east of map center. Modify this to be set as some arbitrary position on the grid given any geodetic coordinates (latitude, longitude)

    def __init__(self, connection, goal_pos=(10,0,5)): # set values as parameter

        # set a goal position 
        self.global_goal_position = goal_pos

        # TODO: adapt to set goal as latitude / longitude position
        local_goal_north, local_goal_east, _alt = global_to_local(self.global_goal_position, self.global_home)

### Write your search algorithm. Minimum requirement here is to add diagonal motions to the A* implementation provided, and assign them a cost of sqrt(2). However, you're encouraged to get creative and try other methods from the lessons and beyond!

Class Action in planning_utils.py

    # Assume all actions cost the same.
    class Action(Enum):
        """
        An action is represented by a 3 element tuple.

        The first 2 values are the delta of the action relative
        to the current grid position. The third and final value
        is the cost of performing the action.
        """

        WEST = (0, -1, 1)
        EAST = (0, 1, 1)
        NORTH = (-1, 0, 1)
        SOUTH = (1, 0, 1)

        # add diagonal motions to the A* implementation provided
        SOUTH_EAST = (1, 1, np.sqrt(2))
        NORTH_EAST = (-1, 1, np.sqrt(2))
        SOUTH_WEST = (1, -1, np.sqrt(2))
        NORTH_WEST = (-1, -1, np.sqrt(2))

Function valid_actions in planning_utils.py

    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)

### Cull waypoints from the path you determine using search.

def prune_path(path, epsi=1e-6):
    def point(p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity(p1, p2, p3):
        return abs(np.linalg.det(np.concatenate((p1, p2, p3), 0))) < epsi

    pruned_path = [p for p in path]

    i = 0
    while i < len(pruned_path) - 2:
        collinear = collinearity(point(pruned_path[i]), point(pruned_path[i + 1]), point(pruned_path[i + 2]))
        if collinear:
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1

    return pruned_path
