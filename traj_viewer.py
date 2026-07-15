#!/usr/bin/env python3
"""
Visualize robot trajectory in:
1) World coordinates (regular obstacles)
2) Image/map coordinates (overlay on map image)
Automatically switches based on presence of map_image_path in YAML.
"""

import argparse
import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import Polygon
from matplotlib.collections import LineCollection


class TrajectoryVisualizer:
    def __init__(self, env):
        self.config = self.read_yaml_config(env)


        self.start = self.config['start']
        self.goal = self.config['goal']
        self.obstacles = np.array(self.config.get('obstacles', []))

        self.fig, self.ax = plt.subplots(figsize=(12, 8))

    # ------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------
    @staticmethod
    def read_yaml_config(filename):
        with open(filename, 'r') as f:
            data = yaml.safe_load(f)
        return data if data else {}

    @staticmethod
    def load_trajectory(filename):
        # Load trajectory
        traj = []
        with open(filename, 'r') as f:
            for line in f:
                vals = [float(x) for x in line.strip().split(',') if x]
                while len(vals) < 5:
                    vals.append(0.0)
                traj.append(vals[:5])
        traj = np.array(traj)
        return traj

    def __call__(self, args, **kwds):

        config = self.config
        traj = self.load_trajectory(args.result)
        # Auto-switch visualization mode
        if 'map_image_path' in config:
            self.visualize_image(
                traj,
                self.start,
                self.goal,
                config['map_image_path'],
                config['map_resolution'],
                config['origin']
            )
        else:
            self.viz_triangles()
            self.visualize_world(
                traj,
                self.start,
                self.goal,
                self.obstacles
            )

        self.ax.set_title(f"Trajectory - {config.get('name', '')}")
        plt.tight_layout()

        if args.output:
            plt.savefig(args.output, dpi=150)
            print(f"Saved to {args.output}")
        else:
            plt.show()


    # ------------------------------------------------------------
    # Visualization: WORLD
    # ------------------------------------------------------------

    def viz_triangles(self):

        data = self.config.get('triangles', [])

        if not data:
            return

        vertices = np.array(data).reshape((-1, 3, 2))
        color='green'
        alpha=0.6
        for poly in vertices:
            # Create a Polygon patch
            polygon = Polygon(poly, closed=True, color=color, alpha=alpha)

            # Add the polygon to the axes
            self.ax.add_patch(polygon)

    def visualize_world(self, traj, start, goal, obstacles):
        """World-coordinate visualization (no image)."""

        # Trajectory
        if traj.shape[1] > 3:
            pts = traj[:, :2].reshape(-1, 1, 2)
            segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
            lc = LineCollection(segs, cmap='rainbow', linewidth=3, alpha=0.8)
            lc.set_array(traj[:-1, 3])
            self.ax.add_collection(lc)
            self.fig.colorbar(lc, ax=self.ax, label='Velocity')
        else:
            self.ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=2)

        # Obstacles
        for obs in obstacles:
            if len(obs) > 2:
                w, h = obs[3] * 2, obs[4] * 2
                rect = Rectangle(
                    (obs[1] - w / 2, obs[2] - h / 2),
                    w, h,
                    color='black'
                )
            else:
                w = h = 0.5
                rect = Rectangle(
                    (obs[0] - w / 2, obs[1] - h / 2),
                    w, h,
                    color='black'
                )

            self.ax.add_patch(rect)

        # Start / Goal
        self.ax.scatter(start[0], start[1], c='green', s=200, zorder=5, label='Start')
        self.ax.scatter(goal[0], goal[1], c='red', s=200, zorder=5, label='Goal')

        # ax.set_xlim(boundary[0], boundary[1])
        # ax.set_ylim(boundary[2], boundary[3])
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True)
        self.ax.legend()


    # ------------------------------------------------------------
    # Visualization: IMAGE
    # ------------------------------------------------------------
    @staticmethod
    def world_to_image(points, origin, resolution, img_height, flip_y=False):
        """
        World (x,y) -> image (u,v)

        flip_y = True  : ROS / PNG / imshow(origin='upper')  ✅ (default)
        flip_y = False : World-aligned images / origin='lower'
        """
        pts = np.asarray(points)

        x = (pts[:, 0] - origin[0]) / resolution
        y = (pts[:, 1] - origin[1]) / resolution

        if flip_y:
            y = img_height - y

        return np.column_stack((x, y))

    def visualize_image(self, traj, start, goal,
                        img_path, resolution, origin):
        """Image/map visualization with world->image transform."""

        img = plt.imread(img_path)
        img_h, img_w = img.shape[:2]

        # Transform data
        traj_img =  self.world_to_image(traj[:, :2], origin, resolution, img_h)
        start_img = self.world_to_image([start[:2]], origin, resolution, img_h)[0]
        goal_img =  self.world_to_image([goal[:2]], origin, resolution, img_h)[0]


        # Map
        self.ax.imshow(img, origin='upper')

        # Trajectory
        pts = traj_img.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc = LineCollection(segs, cmap='rainbow', linewidth=3, alpha=0.8)
        lc.set_array(traj[:-1, 3])
        self.ax.add_collection(lc)
        self.fig.colorbar(lc, ax=self.ax, label='Velocity')


        # Start / Goal
        self.ax.scatter(start_img[0], start_img[1], c='green', s=200, zorder=5, label='Start')
        self.ax.scatter(goal_img[0], goal_img[1], c='red', s=200, zorder=5, label='Goal')

        # ax.set_xlim(0, img_w)
        # ax.set_ylim(img_h, 0)
        self.ax.set_aspect('equal')
        self.ax.grid(False)
        self.ax.legend()


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Visualize robot trajectory')
    parser.add_argument('env', help='Environment YAML file')
    parser.add_argument('--result', default='build/trajectory.csv',
                        help='Trajectory CSV file')
    parser.add_argument('--output', default=None,
                        help='Output image file')
    args = parser.parse_args()
    visualizer = TrajectoryVisualizer(args.env)
    visualizer(args)

if __name__ == '__main__':
    main()