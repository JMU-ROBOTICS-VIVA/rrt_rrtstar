import numpy as np
from py_arm.arm_planning import ArmProblem
import py_arm.geom_util
import rrt
import argparse

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



class RRTArm:
    def __init__(self, arm_problem, dist_fn, step_size=30, tolerance=5):
        self.arm_problem = arm_problem
        self.step_size = step_size
        self.rrtstar_step_size = step_size * 1.25
        self.tolerance = tolerance
        self.dist_fn = dist_fn



    def random_state(self):
        return (np.random.random((self.arm_problem.arm.num_links(),)) *
                360. - 180.0)
        
    def select_input(self, x_rand, x_near, step_size = None):
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

        """  Instructor remove
        if self.arm_problem.at_goal(x):
            print(x_goal)

            q_goal = self.arm_problem.goal();
            print(q_goal)
            angles_delta = np.abs(angle_diffs(x, q_goal))
            print(angles_delta)
            max_angle = np.max(np.abs(angle_diffs(x, q_goal)))
            print('exit')
        """


def draw_tree(tree, ax=None, linewidth=.5, markersize=3):
    """ Draw a full RRT using matplotlib. """
    import matplotlib.pyplot as plt
    if not ax:
        ax = plt.gca()
    for node in tree:
        if node.parent is not None:
            ax.plot([node.parent.x[0], node.x[0]], [node.parent.x[1],
                                                    node.x[1]], 'k.-')


def parse_args():
    parser = argparse.ArgumentParser(description='A* Planner Args')


    parser.add_argument('--armDOFs', action='store', type=int,
                        dest='arm_dofs', default=2, required=False,
                        help='number of DOFs arms')

    parser.add_argument('--maxTreeSize', action='store', type=int,
                        dest='max_tree_size', default=10000, required=False,
                        help='treesize (max for RRT, size for RRT*)')

    parser.add_argument('--planner', action='store',
                        dest='planner', default='rrt', required=False,
                        help='rrt or rrtstar')


    return parser.parse_args()

def test_planning():
    from shapely.geometry import Point
    from py_arm.arm_plotting import draw_c_space
    from py_arm import arm_plotting
    import matplotlib.pyplot as plt

    args = parse_args()

    obs1 = Point(-40, 60).buffer(10)
    obs2 = Point(40, 60).buffer(10)

    q_start = [0.] * args.arm_dofs
    q_goal  = [0.] * args.arm_dofs
    q_goal[0] = 90.

    ## goal_tolerance is the maximum number of degrees that
    ## any of the angles can differ from the goal configuration (l-inf)
    prob = ArmProblem(q_start, q_goal, goal_tolerance=5.,
                      obstacles=[obs1, obs2])

    rrt_prob = RRTArm(prob)

    if args.planner == 'rrt':
        print('Starting RRT Run maxsize: ', args.max_tree_size)

        path, tree = rrt.rrt(rrt_prob, prob.start(), prob.goal(),
                             max_tree_size=args.max_tree_size)
    elif args.planner == 'rrtstar':
        print('Starting RRT Run maxsize: ', args.max_tree_size)

        path, tree = rrt.rrtstar(rrt_prob, prob.start(), prob.goal(),
                                 max_tree_size=args.max_tree_size)

    ax = plt.gca()
    draw_tree(tree, ax)
    plt.show()
        
    
    result = []
    if path is not None:
        for step in path:
            result.append(step.x)
        print("path found")
        print(prob.plan_ok(result))
        print(tree.num_nodes())
        print('cost:', tree.get_cost(path[-1]))

        plan_animator = arm_plotting.PlanAnimator(prob.arm,
                                                  prob.obstacles, 10)
        plan_animator.animate_plan(prob.start(), prob.goal(), result)
    else:
        print('path not found')
        

    
if __name__ == "__main__":
    test_planning()
