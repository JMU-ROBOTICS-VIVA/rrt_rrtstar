# rrt_rrtstar

A programming assignment to introduce the ideas of probabilistic
planning using the RRT and RRT* adaptions.  A robotic arm (py_arm) is
utilized to explore planning problems that can not be efficiently
solved by deterministic planners such as A*.

## Install

You will need to install the python package py_arm.  To do this, go to
the [py_arm repository](https://github.com/JMU-ROBOTICS-VIVA/py_arm)
and clone it into a directory.  Then, from within that directory, run
pip via the following command:

 ```
 pip install py-arm
 ```
   
## Files

- **astar\_planner_arm.py** A complete and functional A\* planner that
  is setup to interact with `py_arm`. Accepts a command line argument
  so that you can create an arm with the desired degrees of freedom
  (DOFs) and run the A\* planner.
- **rrt.py** A template for building an RRT and RRT* planner (**needs
  methods to be completed before it is functional**).  Once complete,
  a main method exists that will call RRT and RRT* on a straight line
  (2d) problem and illustrate the tree and solution path.
- **rrt\_problems.py** Defines an abstract problem class that is used
  to define specific problem specific functions that are required by
  RRT.
- **rrt\_problems\_py\_arm.py** Extends the problem class so support
  py_arm.
- **test\_rrt.py** Unit tests for RRT and RRT*.
- **spatial\_map.py** Implements methods for identifying nearest
  neighbors.


## Demos

### A* on Py-Arm

Once py\_arm is installed, you can experiment with using A* to solve
planning problems with it.  A command line argument, `--dof`, controls
the number of *links*.  **Caution**: 3 links takes about 5 minutes for
A* to solve on my computer.  So, testing your configuration with 2
links first is recommended, as shown in the following code block.

```
python astar_planner_arm.py --dof 2
```


## Helpful Hints

*spatial_map.py* has functions in it to determine the nearest
neighbors, specifically the function ```within_delta```
