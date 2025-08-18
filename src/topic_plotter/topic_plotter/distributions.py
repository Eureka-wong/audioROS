#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
distributions.py: 
"""

import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib.pylab as plt
import os
from datetime import datetime

from audio_interfaces.msg import Distribution
from geometry_msgs.msg import PoseStamped
from audio_interfaces_py.messages import read_distribution_message

from .live_plotter import LivePlotter

N_TIMES = 30
LOG = True
N_LABELS = 10
YLABEL = "distance [cm]"
PROB_THRESHOLD = 0.0001  # 只显示概率大于此值的数据
# YLABEL = "angle [deg]"


class DistributionsPlotter(Node):
    def __init__(self):
        super().__init__("distributions_plotter")

        self.x_labels = {}
        self.y_labels = {}
        self.results_matrix = {}
        self.plotter_dict = {}

        # 存储当前无人机位置信息
        self.current_pose = None
        self.current_position_text = "Position: Unknown"
        self.position_text_objects = []  # 存储位置文本对象

        # 创建保存目录
        self.save_dir = self._create_save_directory()

        distributions = ["raw", "moving"]
        self.fig, self.axs = plt.subplots(1, len(distributions))
        self.fig.set_size_inches(5 * len(distributions), 5)

        # 自动保存计数器和间隔设置
        self.save_counter = 0
        self.save_interval = 5  # 每5次更新保存一次图片

        topic = "results/distribution_raw"
        self._subscription_distribution_raw = self.create_subscription(
            Distribution, topic, self.listener_callback_distribution_raw, 20
        )
        self.get_logger().info(f"subscribed to {topic}")
        topic = "results/distribution_moving"
        self._subscription_distribution_moving = self.create_subscription(
            Distribution, topic, self.listener_callback_distribution_moving, 20
        )
        self.get_logger().info(f"subscribed to {topic}")

        # 订阅无人机位置信息
        self._subscription_pose = self.create_subscription(
            PoseStamped, "geometry/pose", self.listener_callback_pose, 10
        )
        self.get_logger().info("subscribed to geometry/pose")

        for i, name in enumerate(distributions):
            self.plotter_dict[name] = LivePlotter(
                label=name, log=LOG, fig=self.fig, ax=self.axs[i],
            )
            self.x_labels[name] = np.zeros(N_TIMES)
            self.y_labels[name] = None
            self.axs[i].set_xlabel("timestamp [s]")
            self.axs[i].set_ylabel(YLABEL)
            self.axs[i].set_title(name)


    def listener_callback_distribution_raw(self, msg_dist):
        return self.listener_callback_distribution(msg_dist, "raw")

    def listener_callback_distribution_moving(self, msg_dist):
        return self.listener_callback_distribution(msg_dist, "moving")

    def listener_callback_pose(self, msg_pose):
        """接收无人机位置信息"""
        x = msg_pose.pose.position.x
        y = msg_pose.pose.position.y
        z = msg_pose.pose.position.z
        
        # 更新位置文本
        self.current_position_text = f"Drone Position: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m"
        self.current_pose = msg_pose
        
        # 更新图片上的位置显示
        self._update_position_display()

    def _update_position_display(self):
        """更新图片上的位置信息显示"""
        if hasattr(self, 'position_text_objects'):
            # 移除旧的文本
            for text_obj in self.position_text_objects:
                text_obj.remove()
        
        # 添加新的位置文本到每个子图
        self.position_text_objects = []
        for i, ax in enumerate(self.axs):
            # 基本位置信息
            text_obj = ax.text(0.02, 0.98, self.current_position_text, 
                             transform=ax.transAxes, 
                             fontsize=8, 
                             verticalalignment='top',
                             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
            self.position_text_objects.append(text_obj)
            
            # 如果有分布数据，显示最可能的距离
            dist_name = ["raw", "moving"][i]
            if dist_name in self.results_matrix and self.results_matrix[dist_name] is not None:
                latest_probs = self.results_matrix[dist_name][:, -1]  # 最新一列的概率
                if np.any(latest_probs > 0):
                    max_prob_idx = np.argmax(latest_probs)
                    max_prob_distance = self.y_labels[dist_name][max_prob_idx]
                    max_prob_value = latest_probs[max_prob_idx]
                    
                    prob_text = f"Most Likely Distance: {max_prob_distance:.1f}cm (P={max_prob_value:.4f})"
                    text_obj2 = ax.text(0.02, 0.88, prob_text, 
                                      transform=ax.transAxes, 
                                      fontsize=7, 
                                      verticalalignment='top',
                                      bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
                    self.position_text_objects.append(text_obj2)

    def listener_callback_distribution(self, msg_dist, name="raw"):
        # self.get_logger().info(f"callback {name}")
        distances, probs = read_distribution_message(msg_dist)
        
        # 过滤概率值：只保留大于阈值的概率值
        probs_filtered = np.where(probs > PROB_THRESHOLD, probs, 0.0)
        
        # 记录过滤信息（可选）
        n_filtered = np.sum(probs <= PROB_THRESHOLD)
        if n_filtered > 0:
            self.get_logger().debug(f"Filtered {n_filtered} small probability values (< {PROB_THRESHOLD}) for {name}")
        
        if self.y_labels[name] is None:
            self.y_labels[name] = distances
            self.results_matrix[name] = np.zeros((len(distances), N_TIMES), dtype=float)

        self.x_labels[name] = np.r_[
            self.x_labels[name][1:], round(msg_dist.timestamp * 1e-3, 1)
        ]

        assert np.allclose(self.y_labels[name], distances)
        self.results_matrix[name] = np.c_[
            self.results_matrix[name][:, 1:], probs_filtered.reshape(-1, 1)
        ]
        self.plotter_dict[f"{name}"].update_mesh(
            self.results_matrix[name],
            x_labels=self.x_labels[name],
            y_labels=self.y_labels[name],
            n_labels=N_LABELS,
            colorbar=True,
            logger=self.get_logger(),
        )
        
        # 更新位置信息显示
        self._update_position_display()
        
        self.fig.canvas.draw()
        
        # 自动保存图片
        self.save_counter += 1
        if self.save_counter % self.save_interval == 0:
            self.save_current_plot_auto()
            self.get_logger().warn(f"Auto-saved plot after {self.save_counter} updates")

    def _create_save_directory(self):
        """创建保存图片的目录"""
        # 获取项目根目录路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # 向上查找到audioROS根目录
        project_root = current_dir
        while os.path.basename(project_root) != "audioROS" and project_root != os.path.dirname(project_root):
            project_root = os.path.dirname(project_root)
        
        save_dir = os.path.join(project_root, "AUDIOROS", "dist-4")
        os.makedirs(save_dir, exist_ok=True)
        self.get_logger().info(f"Distribution plots will be saved to: {save_dir}")
        return save_dir

    def save_current_plot_auto(self):
        """自动保存当前的分布图"""
        try:
            # 生成带时间戳的文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"distribution_plot_{timestamp}.png"
            filepath = os.path.join(self.save_dir, filename)
            
            # 为图片添加总标题，包含位置信息和时间戳
            title_text = f"Distance Distribution - {self.current_position_text} - {timestamp}"
            self.fig.suptitle(title_text, fontsize=12, y=0.95)
            
            # 保存图片，设置高DPI以获得高质量图像
            self.fig.savefig(filepath, dpi=300, bbox_inches='tight', 
                           facecolor='white', edgecolor='none')
            
            # 也保存一个最新的副本（覆盖之前的）
            latest_filepath = os.path.join(self.save_dir, "distribution_latest.png")
            self.fig.savefig(latest_filepath, dpi=300, bbox_inches='tight',
                           facecolor='white', edgecolor='none')
            
        except Exception as e:
            self.get_logger().error(f"Failed to auto-save plot: {str(e)}")

    def _auto_save_callback(self):
        """自动保存回调函数"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"auto_save_{timestamp}.png"
        filepath = os.path.join(self.save_dir, filename)
        
        try:
            self.fig.savefig(filepath, dpi=150, bbox_inches='tight',
                           facecolor='white', edgecolor='none')
            self.get_logger().info(f"Auto-saved plot: {filepath}")
        except Exception as e:
            self.get_logger().error(f"Auto-save failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    plotter = DistributionsPlotter()
    
    # 提示用户自动保存功能
    # plotter.get_logger().info("Distribution plotter started with auto-save!")
    # plotter.get_logger().info(f"Plots will be automatically saved every {plotter.save_interval} updates")
    # plotter.get_logger().info("Plots will be saved to AUDIOROS/dist/")
    
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        plotter.get_logger().info("Shutting down distributions plotter...")
    finally:
        plotter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
