import numpy as np
import argparse
from py_arm.arm_planning import ArmProblem
import rrt
from rrt_problems_py_arm import RRTArm
from shapely.geometry import Point
from py_arm import arm_plotting
import matplotlib.pyplot as plt

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
    parser = argparse.ArgumentParser(description='pyarm Planner Args')

    parser.add_argument('--dof', action='store', type=int,
                        default=2, required=False,
                        help='Arm degrees of freedom')

    parser.add_argument('--max-tree-size', action='store', type=int,
                        dest='max_tree_size', default=10000, required=False,
                        help='treesize (max for RRT, size for RRT*)')

    parser.add_argument('--planner', action='store',
                        dest='planner', default='rrt', required=False,
                        help='rrt or rrtstar')

    parser.add_argument('--goal-bias', action='store',
                        dest='goal_bias', default='0.1', type=float,
                        required=False,
                        help='percentage of the time to assign q_goal to q_rand')

    return parser.parse_args()


def test_planning():

    args = parse_args()

    # Only use random seed 0 for testing this MUST be removed to
    # achieve any variance in your solutions.
    np.random.seed(0)

    obs1 = Point(-40, 60).buffer(10)
    obs2 = Point(40, 60).buffer(10)
    obs3 = Point(20, 20).buffer(10)
    obs4 = Point(-20, 60).buffer(10)

    print('Making an arm with', args.dof, 'degrees of freedom.')

    q_start = [0.] * args.dof
    q_goal = [0.] * args.dof
    q_goal[0] = 90.

    ## goal_tolerance is the maximum number of degrees that
    ## any of the angles can differ from the goal configuration (l-inf)

    prob = ArmProblem(q_start, q_goal, goal_tolerance=5.,
                      obstacles=[obs1, obs2])
    #                  obstacles=[obs1, obs2, obs3, obs4])

    rrt_prob = RRTArm(prob)


    if args.planner == 'rrt':
        print('Starting RRT Run maxsize: ', args.max_tree_size)

        path, tree = rrt.rrt(rrt_prob, prob.start(), prob.goal(),
                             max_tree_size=args.max_tree_size,
                             goal_bias=args.goal_bias)
    elif args.planner == 'rrtstar':
        print('Starting RRT* grow tree until size: ', args.max_tree_size)

        path, tree = rrt.rrtstar(rrt_prob, prob.start(), prob.goal(),
                                 max_tree_size=args.max_tree_size,
                                 goal_bias=args.goal_bias)

    print("Path cost: ", tree.get_cost(path[-1]))

    ax = plt.gca()
    draw_tree(tree, ax)
    plt.show()

    result = []
    if path is None:
        print('No solution found.')
    else:
        for step in path:
            result.append(step.x)
        print(prob.plan_ok(result))
        print(tree.num_nodes())

        plan_animator = arm_plotting.PlanAnimator(prob.arm,
                                                  prob.obstacles, 10)
        plan_animator.animate_plan(prob.start(), prob.goal(), result)



if __name__ == "__main__":

    #import timeit
    #print(timeit.timeit(test_planning,number=3))

    test_planning()
