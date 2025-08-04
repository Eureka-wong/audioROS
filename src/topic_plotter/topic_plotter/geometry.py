#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
geometry.py: 
"""

from copy import deepcopy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import numpy as np

from audio_interfaces.msg import PoseRaw, DoaEstimates, GroundTruth
from audio_interfaces_py.messages import (
    read_pose_raw_message,
    read_pose_message,
    convert_stamp_to_ms,
)
from audio_stack.topic_synchronizer import TopicSynchronizer

# consider publishing these to a topic instead of this bad dependency.
from audio_simulation.geometry import (
    SPEAKER_POSITION,
    ROOM_DIM,
    STARTING_POS,
    STARTING_YAW_DEG,
)

from .live_plotter import LivePlotter

MAX_LENGTH = 1000  # number of positions to plot

XLABEL = "x [m]"
YLABEL = "y [m]"

Z_THRESHOLD_M = 0  # 0.3  # do not plot poses below this threshold

# PLOT_POSE = False
PLOT_POSE = True  # 需要启用pose订阅以支持DOA可视化
PLOT_RAW = True
PLOT_IMU = False

# data is matrix where each column contains x, y, yaw.
POSE_DICT = {
    "index": -1,
    "data": np.full((3, MAX_LENGTH), None),
    "start_time": None,
    "time": np.full(MAX_LENGTH, None),
}


def plot_room(ax):
    x, y = ROOM_DIM[:2]
    ax.plot([0, 0], [0, y], color="black")
    ax.plot([x, x], [0, y], color="black")
    ax.plot([0, x], [0, 0], color="black")
    ax.plot([0, x], [y, y], color="black")


def plot_source(ax):
    ax.scatter(SPEAKER_POSITION[0], SPEAKER_POSITION[1], color="black")


def add_pose(dict_, position, time=None):
    dict_["index"] = (dict_["index"] + 1) % MAX_LENGTH
    dict_["data"][:, dict_["index"]] = position
    if time is not None:
        dict_["time"][dict_["index"]] = time


def get_latest_pose(pose_dict):
    return pose_dict["data"][:, pose_dict["index"]]


def get_latest_time(pose_dict):
    return pose_dict["time"][pose_dict["index"]]


class GeometryPlotter(Node):
    def __init__(self):
        super().__init__("geometry_plotter")

        self.plotter_dict = {}
        self.pose_dict = {}
        
        # 使用相对时间戳，与DOA时间戳保持一致
        self.start_time = None  # 记录开始时间，用于计算相对时间戳

        if PLOT_RAW or PLOT_IMU:
            self.subscription_pose_raw = self.create_subscription(
                PoseRaw, "geometry/pose_raw", self.listener_callback_pose_raw, 10
            )

        if PLOT_POSE:
            self.subscription_pose = self.create_subscription(
                PoseStamped, "geometry/pose", self.listener_callback_pose, 10
            )
            # need no starting position for pose as it has absolute positions
            self.pose_dict["pose"] = deepcopy(POSE_DICT)

        if PLOT_RAW:
            self.pose_dict["raw"] = deepcopy(POSE_DICT)

        if PLOT_IMU:
            self.pose_dict["imu"] = deepcopy(POSE_DICT)
            add_pose(
                self.pose_dict["imu"], [*STARTING_POS[:2], STARTING_YAW_DEG], time=0
            )

        self.subscription_doa = self.create_subscription(
            DoaEstimates, "geometry/doa_estimates", self.listener_callback_doa, 10,
        )

        # for error calculations
        self.source_direction_deg = None
        self.error_list = []
        self.ground_truth_synch = TopicSynchronizer(20)
        self.subscription = self.create_subscription(
            GroundTruth,
            "geometry/ground_truth",
            self.ground_truth_synch.listener_callback,
            10,
        )
        # 禁用自动关闭定时器，以便持续运行
        # self.shutdown_timer = self.create_timer(5.0, self.shutdown_callback)
        # self.get_logger().info("geometry Estimator will shutdown in 5 seconds")

    def shutdown_callback(self):
        """10秒后自动关闭节点"""
        self.get_logger().info("5 seconds elapsed, shutting down geometry estimator node")
        rclpy.shutdown()

    
    def init_plotter(self, name, xlabel="x", ylabel="y", equal=True):
        if not (name in self.plotter_dict.keys()):
            self.plotter_dict[name] = LivePlotter(
                np.inf, -np.inf, label=name, log=False, logger=self.get_logger()
            )
            self.plotter_dict[name].ax.set_xlabel(xlabel)
            self.plotter_dict[name].ax.set_ylabel(ylabel)
            if equal:
                self.plotter_dict[name].ax.axis("equal")
            self.plotter_dict[name].ax.grid()
            # plot_room(self.plotter_dict[name].ax)
            # plot_source(self.plotter_dict[name].ax)

    def update_plotter(
        self, name, pose_dict, yaw_deg=False, source_direction_deg=None, timestamp=None
    ):
        self.init_plotter(name, xlabel=XLABEL, ylabel=YLABEL)

        pose_data = pose_dict["data"]
        latest_pose = get_latest_pose(pose_dict)

        self.plotter_dict[name].update_scatter(
            pose_data[0, :], pose_data[1, :], label=name, color="C0", timestamp=timestamp
        )

        if yaw_deg:
            self.plotter_dict[name].update_arrow(latest_pose[:2], latest_pose[2])

        if source_direction_deg is not None:
            self.plotter_dict[name].update_arrow(
                latest_pose[:2], source_direction_deg, label="source direction"
            )
        self.plotter_dict[name].ax.legend(loc="upper right")
        self.plotter_dict[name].fig.canvas.draw()
        self.plotter_dict[name].reset_xlim()
        self.plotter_dict[name].reset_ylim()

    def listener_callback_pose_raw(self, msg_pose_raw):
        """Plot the latest poses, calculated from the velocity estimates."""
        r_world, v_world, yaw, yaw_rate = read_pose_raw_message(msg_pose_raw)
        if r_world[2] < Z_THRESHOLD_M:
            return
        timestamp = msg_pose_raw.timestamp

        if PLOT_IMU:
            if self.pose_dict["imu"]["start_time"] is None:
                self.pose_dict["imu"]["start_time"] = timestamp

            previous_time = get_latest_time(self.pose_dict["imu"])
            delta_sec = (timestamp - previous_time) / 1000.0

            delta_yaw = yaw_rate * delta_sec  # degrees
            delta_pos = v_world * delta_sec  # (x, y)
            pose_imu = get_latest_pose(self.pose_dict["imu"]) + [*delta_pos, delta_yaw]

            add_pose(self.pose_dict["imu"], pose_imu, time=timestamp)
            self.update_plotter(
                "pose imu", self.pose_dict["imu"], yaw_deg=True,
            )

        if PLOT_RAW:
            pose_raw = [*r_world[:2], yaw]
            add_pose(self.pose_dict["raw"], pose_raw, time=timestamp)
            self.update_plotter(
                "pose raw", self.pose_dict["raw"], yaw_deg=True,
            )

    def listener_callback_pose(self, msg_pose):
        """Plot the latest poses."""
        new_position, yaw, pitch, roll = read_pose_message(msg_pose)
        if new_position[2] < Z_THRESHOLD_M:
            return

        # 使用相对时间戳，与DOA时间戳保持一致
        ros_timestamp = convert_stamp_to_ms(msg_pose.header.stamp)
        
        # 计算相对时间戳
        if self.start_time is None:
            self.start_time = ros_timestamp
            self.get_logger().info(f"Set start time: {self.start_time}")
        
        # 转换为相对时间戳（毫秒）
        timestamp = ros_timestamp - self.start_time

        assert pitch == 0, pitch
        assert roll == 0, roll
        add_pose(self.pose_dict["pose"], [*new_position[:2], yaw], time=timestamp)

        # 清除timestamp（只有DOA可视化时才设置timestamp）
        if "pose" in self.plotter_dict:
            self.plotter_dict["pose"]._current_timestamp = None
        
        self.update_plotter("pose", self.pose_dict["pose"], yaw_deg=True, timestamp=timestamp)
        self.get_logger().info(f"Pose updated with relative timestamp: {timestamp}ms")

    def listener_callback_doa(self, msg_doa):
        """Plot the estimated DOA directions on the most recent pose."""
        self.get_logger().info("=== DOA CALLBACK TRIGGERED ===") 
        self.init_plotter("position", xlabel=XLABEL, ylabel=YLABEL)

        doa_estimates = list(msg_doa.doa_estimates_deg)
        doa_timestamp = msg_doa.timestamp  # 直接使用DOA消息的音频时间戳
        
        self.get_logger().info(f"DOA visualization using audio timestamp: {doa_timestamp}ms")
        
       # 添加详细的DOA分析日志
        self.get_logger().info(f"Raw DOA estimates: {doa_estimates}")

       # 计算理论DOA角度进行对比
        if "pose" in self.pose_dict and self.pose_dict["pose"]["index"] >= 0:
            latest_pose = get_latest_pose(self.pose_dict["pose"])
            drone_pos = np.array([latest_pose[0], latest_pose[1], 1.0])  # 假设z=1.0
            speaker_pos = np.array(SPEAKER_POSITION)
            
            # 计算理论方向向量
            direction_vector = speaker_pos - drone_pos
            theoretical_angle = np.degrees(np.arctan2(direction_vector[1], direction_vector[0]))
            theoretical_angle = theoretical_angle % 360  # 确保0-360度范围
            
            self.get_logger().info(f"Drone position: [{drone_pos[0]:.2f}, {drone_pos[1]:.2f}, {drone_pos[2]:.2f}]")
            # self.get_logger().info(f"Speaker position: [{speaker_pos[0]:.2f}, {speaker_pos[1]:.2f}, {speaker_pos[2]:.2f}]")
            self.get_logger().info(f"Direction vector: [{direction_vector[0]:.2f}, {direction_vector[1]:.2f}]")
            # self.get_logger().info(f"Theoretical DOA angle: {theoretical_angle:.1f}°")
            
            
            # 计算估计误差
            if doa_estimates:
                error = abs(theoretical_angle - doa_estimates[0])
                if error > 180:
                    error = 360 - error  # 处理角度环绕
                self.get_logger().info(f"DOA estimation error: {error:.1f}° (estimated: {doa_estimates[0]:.1f}°, theoretical: {theoretical_angle:.1f}°)")
        
        self.get_logger().info(f"-------------------------")

        # 记录无人机当前姿态
        if "pose" in self.pose_dict and self.pose_dict["pose"]["index"] >= 0:
            latest_pose = get_latest_pose(self.pose_dict["pose"])
            drone_yaw = latest_pose[2]  # yaw角度
            self.get_logger().info(f"Current drone yaw: {drone_yaw:.1f}°")
        
        # 检查是否有可用的姿态数据
        if "pose" not in self.pose_dict:
            self.get_logger().warn("No pose data available for DOA visualization")
            return
        
        # 检查姿态字典是否为空
        if self.pose_dict["pose"]["index"] == -1:
            self.get_logger().warn("No pose messages received yet")
            return

        # 设置当前时间戳到plotter，用于生成包含时间戳的文件名
        if "pose" in self.plotter_dict:
            self.plotter_dict["pose"].set_timestamp(doa_timestamp)
            self.get_logger().info(f"Set DOA timestamp {doa_timestamp} for geometry visualization filename")

        for i, doa_estimate in enumerate(doa_estimates):
            latest_pose = get_latest_pose(self.pose_dict["pose"])
            if "pose" not in self.plotter_dict:
                self.update_plotter("pose", self.pose_dict["pose"], yaw_deg=True)
            
            self.plotter_dict["pose"].update_arrow(
                latest_pose[:2], doa_estimate, label=f"doa {i}"
            )
            self.plotter_dict["pose"].fig.canvas.draw()

        # calculate the current error
        message = self.ground_truth_synch.get_latest_message(
            msg_doa.timestamp, self.get_logger()
        )
        if message is not None:
            self.source_direction_deg = message.source_direction_deg

        if self.source_direction_deg is not None:
            error = abs(self.source_direction_deg - doa_estimates[0])
            self.error_list.append(error)
            avg_error = np.mean(self.error_list)
            self.get_logger().info(
                f"Current error: {error}, current average: {avg_error}"
            )


def main(args=None):
    rclpy.init(args=args)

    plotter = GeometryPlotter()

    rclpy.spin(plotter)

    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
