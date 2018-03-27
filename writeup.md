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

We read the home position from the `colliders.csv` file in the `motion_planning.py#read_home_position` method.
We open the file, read the first line, split around space, remove comma and then convert the lon and lat tokens into 
float:
```python
def read_home_position(filename):
    with open(filename) as input:
        home = next(input)
        _, lat_str, _, lon_str = home.split(" ")
        return float(lat_str.strip(',')), float(lon_str)
```

We then call the drone `set_home_position` method of the drone.


#### 2. Set your current local position

Here as long as you successfully determine your local position relative to global home you'll be all set. 
Explain briefly how you accomplished this in your code.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


