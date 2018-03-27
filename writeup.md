## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], 
where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

---
### Explain the Starter Code

These `planning_utils.py` and `motion_planning.py` scripts contain a basic path planning implementation.
 
Whereas `backyard_flyer_solution.py` calculates waypoints after takeoff once the drone has reached the target altitude,
`motion_planning.py` introduces a new PLANNING flight phase: once the drone is armed and before takeoff, a fligth plan 
is computed by:
- setting the target altitude
- building the config space grid from the CSV data and target altitude
- plan a simple path from the center of the grid to a position 10m north and 10m east of the start position.
- send the computed to the simulator to visualize the planned trajectory.

Once the plan is computed, drone transitions to the TAKEOFF phase and once it has reached its target altitude, it proceeds 
by going to the next waypoints, once it is within 1m from the target waypoing, it proceed to the next and so on
until the final waypoint is reached. The drone then land.

`planning_utils.py` contains the functions called from `motion_planning.py`:
- the `create_grid` function creates a grid (1m resolution) where each cells contain 0 if the cell is feasible for the 
given target altitude or 1 if the cell is entirely of partially occupied by an obstacle, taking into account an extra
safety distance.
- the `a_start` function do a simple A* path planning with the given grid. 

The provided code only allows north/south and east/west movements leading the staircase effect visible on the trajectory.
![staircase](.misc/staircase.png)


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

We read the "grid anchor" global position from the `colliders.csv` file in the 
`motion_planning.py#plan_path` method and then set the home position of the drone:
```python
# Read the obstacle map 'anchor position' and set as home position.
# Setting the home position fixes the local NED frame
grid_anchor_global_pos = read_grid_anchor_global_pos('colliders.csv')
self.set_home_position(*grid_anchor_global_pos)
```

#### 2. Set your current local position

Once we set the home position and hence fix the local NED frame, we can compute the drone's local coordinates:
```python
# Convert the current global position of the drone into a (north,east) offset from the grid center
local_pos = global_to_local(self.global_position, self.global_home)
```


#### 3. Set grid start position from local position

Before we can compute the grid start position from the local position, we need to create the grid in order to 
get the grid offset, i.e. the local coordinates of south west corner of the grid cell (grid.row-1, 0):
```python
# Read the obstacle data and create a config space grid
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
```

#### 4. Set grid goal position from geodetic coords

In the current code the goal position in global (aka geodetic) coordinates is arbitrarily hard-coded to 
lon = -122.400150 deg, lat = 37.796005.

To convert those coordinates into a grid position, we first convert them to local coordinates, using the `global_to_local`
utility function from `frame_utils`, then convert the local coordinates to grid position by rounding to int and applying 
the gird offset in the new `motion_planning.py#local_to_grid` utility function:
```python
goal_global_pos = np.array([-122.400150, 37.796005, 0])
goal_local_pos = global_to_local(goal_global_pos, self.global_home)
goal_grid_pos = local_to_grid(goal_local_pos, north_offset, east_offset)
```

We are now ready to plan a path.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

To include diagonal actions, we add the following values to the Action enum:
```python
NORTH_WEST = (-1, -1, math.sqrt(2))
NORTH_EAST = (-1, 1, math.sqrt(2))
SOUTH_EAST = (1, 1, math.sqrt(2))
SOUTH_WEST = (1, -1, math.sqrt(2))
```

We also modify the `planning_utils.py#valid_actions` function by adding the following cases:
```python
if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTH_WEST)
if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTH_EAST)
if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTH_WEST)
if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTH_EAST)
```

The rest of the algorithm works the same as before.

#### 6. Cull waypoints 

We remove waypoints by checking 3 successive points in `planning_utils.py#prune_path`:
- if the 3 points are collinear(`collinearity_check`), remove the middle point
- if not, we check if a direct path if feasible (`direct_path_check`)

```python
def prune_path(path, grid):
    pruned_path = [p for p in path]

    i = 0
    while i < len(pruned_path) - 2:
        print("Path length = {}".format(len(pruned_path)))
        p1 = pruned_path[i]
        p2 = pruned_path[i+1]
        p3 = pruned_path[i+2]
        if collinearity_check(p1, p2, p3):
            del pruned_path[i+1]
        elif direct_path_check(p1, p2, p3, grid):
            del pruned_path[i+1]
        else:
            i = i + 1
```

Without the `direct_path_check` the path is reduced to 27 cells. With the extra check we reduce the path further to 19 cells.
Still the presence of some points in the path eludes me...

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


