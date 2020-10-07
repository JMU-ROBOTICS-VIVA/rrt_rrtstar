"""Pure python implementation of the Rapidly Exploring Random Tree
algorithm

author: Nathan Sprague, Kevin Molloy,  and ...

"""
import numpy as np
import rrt_problems
from spatial_map import SpatialMap


class RRTNode(object):
    """ RRT Tree node. """

    def __init__(self, parent, u, x):
        """ Constructor.
        Arguments:
            parent - Parent node in the tree.
            u      - control signal that moves from the parents state to x.
            x      - state associated with this node.
        """
        self.parent = parent
        self.u = u
        self.x = x

    def __repr__(self):
        """ String representation for debugging purposes. """
        return "<u: {}, x: {}>".format(self.u, self.x)

class Tree:
    """ Rapidly exploring rapid tree data structure. """

    def __init__(self, q_init, dist_metric=None):
        """ q_init - root, d-dimensional numpy array """
        self.sm = SpatialMap(dim=q_init.size, dist_metric=dist_metric)
        self.root = RRTNode(None, None, q_init)
        self.sm.add(q_init, self.root)
      

    def add_node(self, q_new):
        node = RRTNode(None, None, q_new)
        return self.sm.add(q_new, node)

    def add_edge(self, q_near, q_new, u):
        new_node = self.sm.get_value(q_new)
        near_node = self.sm.get_value(q_near)
        assert np.array_equal(new_node.x, q_new)
        assert np.array_equal(near_node.x, q_near)
        new_node.parent = near_node
        new_node.u = u

    """
        Get distance from x to y 
    """
    def get_distance(self, x, y):
        return self.sm.get_distance(x, y)
    """
        Get cost to node x from the root
    """
    def get_cost(self, node):
        cost = 0
        while node.parent is not None:
            cost += self.sm.get_distance(node.x, node.parent.x)
            node = node.parent

        return cost

    def num_nodes(self):
        return self.sm.num_values

    def __iter__(self):
        return self.sm.__iter__()

def rrt(problem, q_init, q_goal, max_tree_size=float('inf'), goal_bias=0.0):
    """ Path search using RRT.

    Args:
        problem:   a problem instance that provides three methods:

            problem.random_state() -
               Return a randomly generated configuration
            problem.select_input(q_rand, q_near) -
               Return an action that would move the robot from
               q_near in the direction of q_rand
            problem.new_state(q_near, u) -
               Return the state that would result from taking
               action u in state q_near
            problem.close_enough(q, q_goal) -
               Return true if q is close enough to q_goal to be 
               considered the termination of a successful plan

        q_init:         the initial state
        q_goal:         the goal state
        max_tree_size:  the maxmimum number of nodes to add to the tree
        goal_bias:      probability of using q_goal for q_rand   

    Returns:
       (path, tree) tuple

    """
    # UNFINISHED!


    tree = Tree(q_init, problem.dist_fn)  # Make the start state the root
                                              # of the tree

    raise NotImplementedError

def rrtstar(problem, q_init, q_goal, max_tree_size=int(10000), goal_bias = 0.0):
    """ Path search using RRTStar.

    Args:
        problem:   a problem instance that provides three methods:

            problem.random_state() -
               Return a randomly generated configuration
            problem.select_input(q_rand, q_near, [step_size]) -
               Return an action that would move the robot from
               q_near in the direction of q_rand
            problem.new_state(q_near, u) -
               Return the state that would result from taking
               action u in state q_near
            problem.close_enough(q, q_goal) -
               Return true if q is close enough to q_goal to be
               considered the termination of a successful plan
            problem.dist_fn -
               Function reference. First argument is a
               nparray of configurations (n x d) and the second
               is a single configuration (1 x d).  Returns a 1d
               numpy array of distances between the n configurations
               in  the first argument and the single configuration in the
               second array.

            problem.step_size -
               Attribute of how far to move in one RRT extension

            problem.rrtstar_step_size -
               Attribute of how far to connect in one RRTStar rewire
              

        q_init:         the initial state
        q_goal:         the goal state
        max_tree_size:  the maxmimum number of nodes to add to the tree
        goal_bias:      probability of using q_goal for q_rand   
    Returns:
       (path, tree) tuple

    """
    # UNFINISHED!

    tree = Tree(q_init, problem.dist_fn)      # Make the start state the root
                                              # of the tree
    raise NotImplementedError

def draw_tree(rrt):
    """ Draw a full RRT using matplotlib. """
    import matplotlib.pyplot as plt
    for node in rrt.sm:
        if node.parent is not None:
            plt.plot([node.parent.x[0], node.x[0]],
                     [node.parent.x[1], node.x[1]],
                     'r.-')


def test_simple_rrt():
    """ Demo the rrt algorithm on a simple 2d search task. """
    import matplotlib.pyplot as plt
    x_start = np.array([.5, .5])
    x_goal = np.array([.9, .9])
    lines = rrt_problems.RRTStraightLines(.03)
    path, tree = rrt(lines, x_start, x_goal)
    print(path)
    result = np.zeros((len(path), 2))
    for i in range(len(path)):
        result[i, :] = path[i].x
    draw_tree(tree)
    plt.plot(result[:, 0], result[:, 1], '.-')
    plt.show()
    plt.clf()
  
    path, tree = rrtstar(lines, x_start, x_goal, 300)
    result = np.zeros((len(path), 2))
    for i in range(len(path)):
        result[i, :] = path[i].x
    draw_tree(tree)
    plt.plot(result[:, 0], result[:, 1], '.-')
    plt.show()
    plt.clf()



if __name__ == "__main__":
    test_simple_rrt()
