# rrt_rrtstar
A programming assignment to introduce the ideas of probabilistic planning using the RRT and RRT* adaptions.  A robotic arm (py_arm) is utilized to explore planning problems that can not be effifiently solved by deterministic planners such as A*.  

## Install
You will need to install the python package py_arm.  To do this, go to the [py_arm respository](https://github.com/JMU-ROBOTICS-VIVA/py_arm) and clone it into a directory.  Then, from within that directory, run pip via the following command:
 ```pip install py-arm```
   

## Files
- **astar\_planner_arm.py** A complete and functional A* planner that is setup to interact with py*arm. Accepts a command line argument so that you can create an arm with the desired degrees of freedom (DOFs) and run the A\* planner. 
- **rrt.py** A template for building an RRT and RRT* planner (**needs methods to be completed before it is functional**).  Once complete, a main method exists that will call RRT and RRT* on a straightline (2d) problem and illustrate the tree and solution path.
- **rrt\_planner.py** Setup the py_arm and run RRT.  This method sets the seed such that the problem should solve in approximately 4 minutes for a 4 DOF arm.
- **rrt\_problems.py** Defines an abstract problem class that is used to define specific problem specific functions that are required by RRT. 
- **rrt\_problems\_py\_arm.py** Extends the problem class so support py_arm.
- **rrt\_test.py** Unittests for RRT and RRT*.
- **spatial\_map.py** Implements methods for identifying nearest neighbors.


## Demos

### A* on Py-Arm
Once py-arm is installed, you can experiment with using A* to solve planning problems with it.  A command line argument, *--armDOFs*, controls the number of *links*.  **Caution**: 3 links takes about 5 minutes for A* to solve on my computer.  So, testing your configuration with 2 links first is recommended, as shown in the following code block.
```
python astar_planner_arm.py --armDOFs 2
```

### RRT and RRT* on 2d StraightLine

## Helpful Hints
*spatial_map.py* has  functions in it to determine the nearest neighbors, specifically the function ```within_delta```

