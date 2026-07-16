"""Plan with the bow_connect bindings and visualize the result.

Run from this directory (map paths in the config are relative to it):
    cd python && python3 example.py [--output traj.png]
"""
import argparse
import sys

sys.path.append("../build")  # bow_connect module
sys.path.append("..")        # traj_viewer module

import bow_connect
from traj_viewer import TrajectoryVisualizer


def main():
    parser = argparse.ArgumentParser(description='Plan and visualize a trajectory')
    parser.add_argument('--env', default="../test/ugv/sr_clutter_01.yaml",
                        help='Environment YAML file')
    parser.add_argument('--solver-time', type=float, default=30.0,
                        help='Planning budget in seconds')
    parser.add_argument('--no-optimize', action='store_true',
                        help='Skip trajectory post-optimization')
    parser.add_argument('--output', default=None,
                        help='Save the plot to this image file instead of showing it')
    args = parser.parse_args()

    viz = TrajectoryVisualizer(args.env)

    planner = bow_connect.BOWConnectPlanner(args.env)
    traj = planner.plan(
        start=viz.start,               # [x, y, theta] from the config
        goal=viz.goal[:2],             # [x, y] from the config
        solver_time=args.solver_time,
        optimize=not args.no_optimize,
    )

    # traj is a numpy array of shape (N, 5): [x, y, theta, v, omega] per row
    print(f"trajectory: {traj.shape[0]} states")
    viz.show(traj, output=args.output)


if __name__ == '__main__':
    main()
