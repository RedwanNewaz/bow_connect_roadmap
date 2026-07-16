"""Build a roadmap by running the BOWConnect planner repeatedly.

Each plan() call returns one collision-free trajectory. Mirroring
src/planner/MotionTree.cpp, every state is snapped to a grid cell
(cell size = robot_radius by default, like MotionTree::hashState), so
overlapping trajectories share nodes; consecutive states become edges.
The combined networkx graph is a roadmap of the free space, on which we
also query the shortest start-to-goal path.

Run from this directory (map paths in the config are relative to it):
    cd python && python3 roadmap_example.py -n 10 [--output roadmap.png]
"""
import argparse
import sys

sys.path.append("../build")  # bow_connect module
sys.path.append("..")        # traj_viewer module

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.patches import Rectangle

import bow_connect
from traj_viewer import TrajectoryVisualizer


class RoadmapBuilder:
    """Combine trajectories into a graph, python analogue of bow::MotionTree.

    States are hashed into grid cells (MotionTree::hashState); the first
    state seen in a cell becomes the node's representative state, and
    consecutive trajectory states become weighted edges.
    """

    def __init__(self, origin_x, origin_y, grid_size):
        self.origin = np.array([origin_x, origin_y])
        self.grid_size = grid_size
        self.graph = nx.Graph()

    def cell(self, state):
        """Equivalent of MotionTree::hashState: world (x, y) -> grid cell."""
        idx = np.round((np.asarray(state[:2]) - self.origin) / self.grid_size)
        return int(idx[0]), int(idx[1])

    def add_trajectory(self, traj):
        """Equivalent of repeated MotionTree::addState along one trajectory."""
        prev = None
        for state in np.asarray(traj, dtype=float):
            c = self.cell(state)
            if c not in self.graph:
                self.graph.add_node(c, state=state, visits=0)
            self.graph.nodes[c]["visits"] += 1

            if prev is not None and prev != c:
                dist = float(np.linalg.norm(
                    self.graph.nodes[prev]["state"][:2] - state[:2]))
                if self.graph.has_edge(prev, c):
                    edge = self.graph.edges[prev, c]
                    edge["weight"] = min(edge["weight"], dist)
                    edge["count"] += 1
                else:
                    self.graph.add_edge(prev, c, weight=dist, count=1)
            prev = c

    def nearest_node(self, xy):
        """Equivalent of MotionTree::findNearestNode."""
        xy = np.asarray(xy[:2], dtype=float)
        return min(
            self.graph.nodes,
            key=lambda n: float(np.sum((self.graph.nodes[n]["state"][:2] - xy) ** 2)),
        )

    def shortest_path(self, start_xy, goal_xy):
        """Shortest node sequence between the cells nearest to start/goal."""
        try:
            return nx.shortest_path(
                self.graph,
                self.nearest_node(start_xy),
                self.nearest_node(goal_xy),
                weight="weight",
            )
        except nx.NetworkXNoPath:
            return None


def draw_roadmap(viz, roadmap, path=None, output=None):
    """Draw the roadmap on the environment (map image or world coordinates)."""
    graph = roadmap.graph
    config = viz.config
    ax, fig = viz.ax, viz.fig
    pos_world = {n: graph.nodes[n]["state"][:2] for n in graph.nodes}

    if 'map_image_path' in config:
        img = plt.imread(config['map_image_path'])
        img_h = img.shape[0]

        def to_img(pts):
            return TrajectoryVisualizer.world_to_image(
                np.atleast_2d(pts), config['origin'],
                config['map_resolution'], img_h)

        ax.imshow(img, origin='upper')
        pos = {n: to_img(p)[0] for n, p in pos_world.items()}
        start = to_img(viz.start[:2])[0]
        goal = to_img(viz.goal[:2])[0]
        ax.grid(False)
    else:
        pos = pos_world
        start, goal = viz.start[:2], viz.goal[:2]
        for obs in viz.obstacles:
            w = h = 0.5
            ax.add_patch(Rectangle((obs[0] - w / 2, obs[1] - h / 2), w, h,
                                   color='black'))
        ax.grid(True)

    # Edges
    segs = [np.array([pos[u], pos[v]]) for u, v in graph.edges]
    ax.add_collection(LineCollection(segs, colors='tab:blue',
                                     linewidths=1.0, alpha=0.5))

    # Nodes, colored by how many trajectories visited their cell
    xy = np.array([pos[n] for n in graph.nodes])
    visits = np.array([graph.nodes[n]["visits"] for n in graph.nodes])
    sc = ax.scatter(xy[:, 0], xy[:, 1], c=visits, cmap='viridis',
                    s=14, zorder=4)
    fig.colorbar(sc, ax=ax, label='Visits')

    # Shortest path through the roadmap
    if path is not None:
        pxy = np.array([pos[n] for n in path])
        ax.plot(pxy[:, 0], pxy[:, 1], 'r-', linewidth=2.5, zorder=5,
                label='Shortest path')

    ax.scatter(start[0], start[1], c='green', s=200, zorder=6, label='Start')
    ax.scatter(goal[0], goal[1], c='red', s=200, zorder=6, label='Goal')
    ax.set_aspect('equal')
    ax.legend()
    ax.set_title(f"Roadmap - {config.get('name', '')} "
                 f"({graph.number_of_nodes()} nodes, "
                 f"{graph.number_of_edges()} edges)")
    plt.tight_layout()

    if output:
        plt.savefig(output, dpi=150)
        print(f"Saved to {output}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Build and visualize a roadmap from repeated plans')
    parser.add_argument('--env', default="../test/ugv/sr_clutter_01.yaml",
                        help='Environment YAML file')
    parser.add_argument('-n', '--num-plans', type=int, default=10,
                        help='Number of planner runs to combine')
    parser.add_argument('--solver-time', type=float, default=10.0,
                        help='Planning budget per run in seconds')
    parser.add_argument('--grid-size', type=float, default=None,
                        help='Roadmap cell size (default: robot_radius from config)')
    parser.add_argument('--output', default=None,
                        help='Save the plot to this image file instead of showing it')
    args = parser.parse_args()

    viz = TrajectoryVisualizer(args.env)
    planner = bow_connect.BOWConnectPlanner(args.env)

    origin = viz.config['origin']
    grid_size = args.grid_size or viz.config['robot_radius']
    roadmap = RoadmapBuilder(origin[0], origin[1], grid_size)

    successes = 0
    for i in range(args.num_plans):
        try:
            traj = planner.plan(
                start=viz.start,               # [x, y, theta] from the config
                goal=viz.goal[:2],             # [x, y] from the config
                solver_time=args.solver_time,
                optimize=False,
            )
        except RuntimeError as e:
            print(f"[{i + 1}/{args.num_plans}] plan failed: {e}")
            continue
        successes += 1
        roadmap.add_trajectory(traj)
        print(f"[{i + 1}/{args.num_plans}] {traj.shape[0]} states -> roadmap: "
              f"{roadmap.graph.number_of_nodes()} nodes, "
              f"{roadmap.graph.number_of_edges()} edges")

    if successes == 0:
        sys.exit("No plan succeeded; nothing to visualize.")

    path = roadmap.shortest_path(viz.start, viz.goal)
    if path is None:
        print("No start-goal path in the roadmap graph.")
    else:
        length = nx.path_weight(roadmap.graph, path, weight="weight")
        print(f"Shortest roadmap path: {len(path)} nodes, length {length:.2f} m")

    draw_roadmap(viz, roadmap, path, args.output)


if __name__ == '__main__':
    main()
