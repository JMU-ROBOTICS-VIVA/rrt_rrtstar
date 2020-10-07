"""
This module contains the abstract superclass for RRT planning problems, as
well as one sample problem.

Author: Nathan Sprague
Version: 9/25/2020
"""
from abc import ABC, abstractmethod
import numpy as np


class RRTProblem(ABC):
    """Extend this class in order to apply RRT to a particular planning
    problem.

    """

    @abstractmethod
    def random_state(self):
        """Return a random point (numpy array) in the configuration space.

        """
        pass

    @abstractmethod
    def select_input(self, q_rand, q_near, step_size=None):
        """
        Select an action that moves q_near toward q_rand.

        q_rand - numpy array representing a random point in the config space
        q_near - numpy array representing a coniguration corresponding
                 to a node in the existing tree.
        step_size - max step

        Returns: An action in whatever format makes sense for the problem
        """
        pass

    @abstractmethod
    def new_state(self, q_near, u):
        """Return the state reached by starting in q_near and taking action u.

        """
        pass

    @abstractmethod
    def close_enough(self, q, q_goal):
        """Return true if q is close enough to q_goal to be considered the
        termination of a successful plan.

        """
        pass


class RRTStraightLines(RRTProblem):
    """RRT problem class for searching in straight line steps directly
        toward intermediate target locations.  All searching happens
        within the unit square.

    """

    def __init__(self, step_size=.03, tolerance=.02):
        self.step_size = step_size
        self.rrtstar_step_size = step_size * 1.25
        self.tolerance = tolerance
        self.dist_fn = lambda xs, y: np.sqrt(np.sum((xs -
                                                         y)**2, axis=1))
    def random_state(self):
        return np.random.random((2,))

    def select_input(self, q_rand, q_near, step_size=None):
        if step_size is None:
            step_size = self.step_size

        u = q_rand - q_near
        length = np.linalg.norm(u)
        if length > self.step_size:
            u = u / length * self.step_size
        return u

    def new_state(self, q_near, u):
        return q_near + u

    def close_enough(self, q, q_goal):
        return np.linalg.norm(q - q_goal) < self.tolerance
