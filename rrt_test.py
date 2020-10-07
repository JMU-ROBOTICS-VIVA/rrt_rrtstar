""" Unit tests for rrt.py with rrtstar

Author: Nathan Sprague, Kevin Molloy
Version: Oct 2020
"""

import unittest
import numpy as np
from numpy.testing import assert_array_almost_equal
from shapely.geometry import Point
from py_arm.arm_planning import ArmProblem

from rrt_problems_py_arm import RRTArm

from rrt_problems import RRTProblem

import rrt
import rrt_problems
import rrt_problems

class RRTTests(unittest.TestCase):

    def test_find_2dof_rrtstar(self):
        self.test_find_2dof_rrtpath(run_rrtstar=True)

    def test_find_2dof_rrtpath(self, run_rrtstar=False):
        q_start = [0] * 2
        q_goal  = [90, 0]

        obs1 = Point(-40, 60).buffer(10)

        prob = ArmProblem(q_start, q_goal, goal_tolerance=5.,
                      obstacles=[obs1])

        rrt_prob = RRTArm(prob)

        print('Starting RRT Run:')

        max_tree = 1500
        if run_rrtstar:
            path, tree = rrt.rrtstar(rrt_prob, prob.start(), prob.goal(),
                                     max_tree_size=max_tree)
        else:
            path, tree = rrt.rrt(rrt_prob, prob.start(), prob.goal(),
                                 max_tree_size=max_tree)

        # check that the path starts in the right place.
        cur_x = q_start
        assert_array_almost_equal(cur_x, path[0].x)
        result = [path[0].x]
        for node in path[1::]:
            cur_x += node.u
            result.append(node.x)
            assert_array_almost_equal(cur_x, node.x)

        ## check that path is collision free
        prob.plan_ok(result)

    def test_find_2dline_rrtpath(self):
        print('line')
        x_start = np.array([.4, .4])
        x_goal = np.array([.7, .7])
        problem = rrt_problems.RRTStraightLines(.03, .04)
        path, _ = rrt.rrt(problem, x_start, x_goal)

        # check that the path ends in the right place.
        end_x = path[-1].x
        self.assertTrue(np.linalg.norm(end_x - x_goal) < .05)

        # check that the path starts in the right place.
        cur_x = x_start
        assert_array_almost_equal(cur_x, path[0].x)
        # check that each step is valid.
        for node in path[1::]:
            cur_x += node.u
            assert_array_almost_equal(cur_x, node.x)


if __name__ == "__main__":
    unittest.main()
