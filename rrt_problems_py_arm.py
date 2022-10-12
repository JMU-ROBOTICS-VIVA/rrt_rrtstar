import numpy as np
import py_arm.geom_util
from py_arm.geom_util import angle_diff
from rrt_problems import RRTProblem


def angle_diffs(x, y):
    # https://stackoverflow.com/questions/1878907/
    # the-smallest-difference-between-2-angles
    x = np.deg2rad(x)
    y = np.deg2rad(y)
    diffs = np.arctan2(np.sin(x - y), np.cos(x - y))
    diffs = np.rad2deg(diffs)
    return diffs


def angle_metric_l2(x, y):
    diffs = angle_diffs(x, y)
    return np.sqrt(np.sum(diffs ** 2, axis=1))


class RRTArm(RRTProblem):
    def __init__(self, arm_problem, dist_fn=angle_metric_l2, step_size=30, tolerance=5):
        self.arm_problem = arm_problem
        self.step_size = step_size
        self.rrtstar_step_size = step_size * 1.25
        self.tolerance = tolerance
        self.dist_fn = dist_fn

    def random_state(self):
        return (np.random.random((self.arm_problem.arm.num_links(),))
                * 360. - 180.0)

    def select_input(self, x_rand, x_near, step_size=None):
        if step_size is None:
            step_size = self.step_size

        seq = py_arm.geom_util.angle_sequence(x_near, x_rand, step_size)
        if len(seq) == 0:
            return None

        u = angle_diffs(seq[0], x_near)
        x_new = self.new_state(x_near, u)
        if self.arm_problem.step_ok(x_near, x_new):
            return u
        else:
            return None

    def new_state(self, x_near, u):
        new = x_near + u
        less = new < -180.
        new[less] = 360. + new[less]
        more = new > 180.
        new[more] = new[more] - 360.
        return new

    def close_enough(self, x, x_goal):
        return self.arm_problem.at_goal(x)
