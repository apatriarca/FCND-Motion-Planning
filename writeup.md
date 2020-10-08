## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

The `motion_planning.py` script implements an event driven high-level controller for the drone. It is mostly based on
the solution to the *Backyard Flyer* project, but includes an additional method to create a collision-free motion plan 
between two location in the map. The plan is created using the methods in the `planning_utils.py` script. In particular,
it is using a grid representation of the map and it is using the A* algorithm to find the path. In the basic code there 
are only horizontal and vertical motions and the euclidean distance is used as heuristic.
 
The obstacles are described in the `colliders.csv` file as 3D boxes. Since the entire flight is planned at a constant 
altitude, the grid takes this into account and only show obstacles at that altitude. An additional safety distance is
added to each obstacle to make sure the drone stay far enough from them.

In the basic implementation the start and goal position are hardcoded and all the intermediate grid positions are used
as waypoints.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

I added a new method inside the `planning_utils.py` to read the csv file called `read_obstacles`. This method uses
a *regex* to parse the first row and then reads the rest of the file using the usual `loadtxt` numpy method. The
implementation is included in the `update_positions` method.

#### 2. Set your current local position

I used the `global_to_local` method in the `udacidrone.frame_utils` module. I have then set the local position by 
setting the `_north`, `_east` and `_down` attributes of the drone. The implementation is included in the 
`update_positions` method.

#### 3. Set grid start position from local position

I set the grid start position by subtracting the grid offsets to the local position and then casting the result to
integer coordinates. This is abstracted in the `_path_start_location`.

#### 4. Set grid goal position from geodetic coords

Here I set the goal position by randomly sampling between the minimum and maximum geodetic coordinates of the map. These
are computed from the local coordinates of the map extrema by using the `local_to_glocal` method in the 
`udacidrone.frame_utils` module. The drone's method `_path_goal_location` then take these location and create the 
corresponding grid coordinates using the `global_to_local` method. The random position is sampled until a obstacle free
one is found.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

I have included diagonal movements with cost `sqrt(2)` to A*. The only changes are in the `Action` enum and the 
`valid_actions` function.

#### 6. Cull waypoints 

I have used both the collinearity and the bresenham algorithm for culling waypoints. The culling is implemented in the 
`reduce_path` function in the `planning_utils.py` script and the bresenham algorithm in the `bres` method in the same
script. I am first testing for collinearity since it is much cheaper to compute.


### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


