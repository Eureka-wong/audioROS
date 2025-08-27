#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wall_mapper.py: 
"""

import rclpy
from rclpy.action import ActionServer

import numpy as np

from audio_gtsam.wall_backend import WallBackend
from audio_interfaces.srv import StateMachine
from audio_interfaces.msg import Distribution, PoseRaw
from audio_interfaces_py.messages import read_pose_raw_message
from audio_interfaces_py.node_with_params import NodeWithParams
from audio_stack.topic_synchronizer import TopicSynchronizer
from crazyflie_demo.wall_detection import State

import matplotlib
matplotlib.use("Agg")  # Use non-interactive backend for saving plots   
import matplotlib.pyplot as plt
from pathlib import Path
import threading
import time


class WallMapper(NodeWithParams):
    """ Node to map out the walls of a room using audio signals  """

    PARAMS_DICT = {}

    def __init__(self):
        super().__init__("wall_mapper")

        self._subscription_dist_moving = self.create_subscription(
            Distribution, "results/distribution_moving", self.listener_callback_dist, 10
        )
        self.pose_synch = TopicSynchronizer(10, n_buffer=200)
        self._subscription_pose = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.pose_synch.listener_callback, 10,
        )

        self._service_wall = self.create_service(
            StateMachine, "check_wall", self.check_wall_callback
        )
        # initialize ISAM
        self.wall_backend = WallBackend()

        # add SLAM visualization
        self.enable_viz = True
        self.visualization_counter = 0
        self.plot_interval = 10
        self.save_plots = True
        self.plot_dir = Path("/audioROS/AUDIOROS/SLAM_VIS_3")
        self.plot_dir.mkdir(exist_ok=True)
        

    def listener_callback_dist(self, msg_dist):
        # extract the current probability distribution

        # creates lag... need to u
        msg_pose = self.pose_synch.get_latest_message(msg_dist.timestamp)
        if msg_pose is None:
            return

        # self.get_logger().warn(f"treating distribution with timestamp {msg_dist.timestamp} and pose {msg_pose.timestamp}")

        r_world, v_world, yaw_deg, yaw_rate = read_pose_raw_message(msg_pose)

        # add pose factor
        self.wall_backend.add_pose(
            r_world, yaw_deg / 180 * np.pi, verbose=True, logger=self.get_logger()
        )

        # add plane factor
        distances = np.array(msg_dist.values)
        probs = np.array(msg_dist.probabilities)
        # self.get_logger().warn(f"distribution: {distances[0]}...{distances[-1]}, {probs[0]}, {probs[-1]}")

        self.wall_backend.add_plane_from_distances(
            distances, probs, verbose=True, logger=self.get_logger()
        )
        '''
        # add visualization functions
        self.visualization_counter += 1
        if self.enable_viz and (self.visualization_counter % self.plot_interval == 0):
            self.call_visualizer()
        '''

    def check_wall_callback(self, request, response):
        # find which enum the state corresponds to
        state_by_server = State(request.state)
        self.get_logger().info(
            f"Action received: {request.state} which corresponds to {state_by_server}"
        )
        if state_by_server == State.WAIT_DISTANCE:
            if self.wall_backend.check_wall():
                response.flag = State.AVOID_DISTANCE.value
            else:
                response.flag = State.WAIT_DISTANCE.value
        else:
            response.flag = -1
        self.get_logger().warn(f'Action response: {response}')
        return response
    
    def call_visualizer(self):
        viz_thread = threading.Thread(target=self.create_slam_plot)
        viz_thread.daemon = True
        viz_thread.start()

    def create_slam_plot(self):
        try:
            if self.wall_backend.pose_index < 0:
                self.get_logger().warn("No poses available yet for SLAM visualization")
                return
            fig, ax = plt.subplots(figsize=(10, 10))
            self.wall_backend.plot(
                ax=ax,
                fig=fig,
                n_poses=20,
                live_update=False,
                final=True,
            )

            if self.save_plots:
                filename = f"slam_{self.visualization_counter:04d}.png"
                plt.savefig(self.plot_dir / filename)
                self.get_logger().warn(f"Saved SLAM plot to {self.plot_dir / filename}")

            plt.close(fig)  # Close the figure to free memory
        except Exception as e:
            self.get_logger().error(f"Error in create_slam_plot: {e}")


def main(args=None):
    rclpy.init(args=args)
    wall_mapper = WallMapper()
    rclpy.spin(wall_mapper)
    wall_mapper.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
