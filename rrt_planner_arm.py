import numpy as np
import argparse
from py_arm.arm_planning import ArmProblem
import py_arm.geom_util
import rrt
from rrt_problems_py_arm import draw_tree, RRTArm, angle_metric_l2


    
def parse_args():
    parser = argparse.ArgumentParser(description='pyarm Planner Args')


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

    ## only use random seed 0 for testing
    ## this MUST be removed to achieve any variance in your
    ## solutions.

    np.random.seed(0)

    obs1 = Point(-40, 60).buffer(10)
    obs2 = Point(40, 60).buffer(10)

    q_start = [0.] * args.arm_dofs
    q_goal  = [0.] * args.arm_dofs
    q_goal[0] = 90.

    ## goal_tolerance is the maximum number of degrees that
    ## any of the angles can differ from the goal configuration (l-inf)

    prob = ArmProblem([0, 0, 0, 0], [90, 0, 0, 0], goal_tolerance=5.,
                      obstacles=[obs1, obs2])

    rrt_prob = RRTArm(prob, angle_metric_l2)

        
    if args.planner == 'rrt':
        print('Starting RRT Run maxsize: ', args.max_tree_size)

        path, tree = rrt.rrt(rrt_prob, prob.start(), prob.goal(),
                             max_tree_size=args.max_tree_size)
    elif args.planner == 'rrtstar':
        print('Starting RRT* grow tree until size: ', args.max_tree_size)

        path, tree = rrt.rrtstar(rrt_prob, prob.start(), prob.goal(),
                                 max_tree_size=args.max_tree_size)

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
