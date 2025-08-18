import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib.pylab as plt
import os
from datetime import datetime

from audio_interfaces.msg import SignalsFreq
from geometry_msgs.msg import PoseStamped

from audio_interfaces_py.messages import (
    read_signals_freq_message,
    convert_sec_nanosec_to_ms,
)
from audio_stack.topic_synchronizer import TopicSynchronizer

from .live_plotter import LivePlotter

FREQ_HZ = 3000  # frequency bin to use. if not given, we use the max snr one.
MAX_FREQS_TO_PLOT = 4  # maximum frequencies to plot, less than 10
FIG_SIZE = (10, 10)
XMIN = 0
XMAX = 1000  # x轴范围改为0-10m (1000cm)
N_MICS = 4


class WallApproachPlotter(Node):
    def __init__(self):
        super().__init__("wall_plotter")

        # 创建保存目录
        self.save_dir = self._create_save_directory()
        
        # 存储当前无人机位置信息
        self.current_position_text = "Position: Unknown"
        self.position_text_objects = []  # 存储位置文本对象
        self.save_counter = 0
        self.save_interval = 10  # 每10次更新保存一次

        # self.signals_f_synch = TopicSynchronizer(10)
        # self.subscription_signals_f = self.create_subscription(
        #     SignalsFreq,
        #     "audio/signals_f",
        #     self.signals_f_synch.listener_callback,
        #     10,
        # )

        # 直接储存最新的音频消息，不用TopicSynchronizer同步
        self.latest_audio_message = None
        self.audio_message_count = 0
        self.subscription_signals_f = self.create_subscription(
            SignalsFreq,
            "audio/signals_f",
            self.listener_callback_audio,
            10,
        )

        self.subscription_pose = self.create_subscription(
            PoseStamped, "geometry/pose", self.listener_callback_pose, 10
        )

        self.plotter_dict = {}
        self.frequency_colors = {}

        self.fig, self.axs = plt.subplots(N_MICS, sharey=True, sharex=True)
        self.fig.set_size_inches(*FIG_SIZE)
        for i in range(N_MICS):
            self.plotter_dict[i] = LivePlotter(
                label=f"mic{i}",
                max_xlim=XMAX,
                min_xlim=XMIN,
                ax=self.axs[i],
                fig=self.fig,
            )
            self.plotter_dict[i].ax.set_xlabel("x coordinate [cm]")  # 改为x坐标
            self.plotter_dict[i].ax.set_title(f"mic{i}")
        self.plotter_dict[0].ax.set_ylabel("loudness")

    def _create_save_directory(self):
        """创建保存图片的目录"""
        # 获取项目根目录路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # 向上查找到audioROS根目录
        project_root = current_dir
        while os.path.basename(project_root) != "audioROS" and project_root != os.path.dirname(project_root):
            project_root = os.path.dirname(project_root)
        
        save_dir = os.path.join(project_root, "AUDIOROS", "wall-4")
        os.makedirs(save_dir, exist_ok=True)
        self.get_logger().info(f"Wall plots will be saved to: {save_dir}")
        return save_dir

    def _update_position_display(self):
        """更新图片上的位置信息显示"""
        if hasattr(self, 'position_text_objects'):
            # 移除旧的文本
            for text_obj in self.position_text_objects:
                text_obj.remove()
        
        # 添加新的位置文本到第一个子图
        self.position_text_objects = []
        text_obj = self.axs[0].text(0.02, 0.98, self.current_position_text, 
                         transform=self.axs[0].transAxes, 
                         fontsize=10, 
                         verticalalignment='top',
                         bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        self.position_text_objects.append(text_obj)

    def save_current_plot_auto(self):
        """自动保存当前的音频信号图"""
        try:
            # 生成带时间戳的文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"wall_audio_plot_{timestamp}.png"
            filepath = os.path.join(self.save_dir, filename)
            
            # 为图片添加总标题，包含位置信息和时间戳
            title_text = f"Audio Signals vs X-Position - {self.current_position_text} - {timestamp}"
            self.fig.suptitle(title_text, fontsize=12, y=0.95)
            
            # 保存图片，设置高DPI以获得高质量图像
            self.fig.savefig(filepath, dpi=300, bbox_inches='tight', 
                           facecolor='white', edgecolor='none')
            
            # 也保存一个最新的副本（覆盖之前的）
            latest_filepath = os.path.join(self.save_dir, "wall_audio_latest.png")
            self.fig.savefig(latest_filepath, dpi=300, bbox_inches='tight',
                           facecolor='white', edgecolor='none')
            
            self.get_logger().info(f"Wall audio plot saved to: {filepath}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to auto-save wall plot: {str(e)}")

    def listener_callback_audio(self, msg):
        """音频消息回调函数"""
        self.latest_audio_message = msg
        self.audio_message_count += 1
        if self.audio_message_count % 50 == 0:  # 每50个消息记录一次
            self.get_logger().info(f"Received audio message #{self.audio_message_count}, timestamp: {msg.timestamp}")

    def listener_callback_pose(self, msg_pose):
        timestamp = convert_sec_nanosec_to_ms(
            msg_pose.header.stamp.sec, msg_pose.header.stamp.nanosec
        )

        # 更新位置信息
        x = msg_pose.pose.position.x
        y = msg_pose.pose.position.y
        z = msg_pose.pose.position.z
        self.current_position_text = f"Drone Position: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m"

        # 检查是否有最新的音频消息
        if self.latest_audio_message is not None:
            self.get_logger().info(f"Processing pose {timestamp} with latest audio message (timestamp: {self.latest_audio_message.timestamp})")
            msg_signals_f = self.latest_audio_message
        else:
            self.get_logger().info("No audio message available yet")
            self.fig.canvas.draw()
            return

        # msg_signals_f = self.signals_f_synch.get_latest_message(
        #     timestamp, self.get_logger()
        # )
        if msg_signals_f is not None:
            self.get_logger().info(f"Processing pose {timestamp}")
            __, signals_f, freqs = read_signals_freq_message(msg_signals_f)
            # remove zero frequencies
            signals_f = signals_f[freqs > 0, :]  # n_freqs x n_mics
            freqs = freqs[freqs > 0]

            if FREQ_HZ is None:
                f_idx = np.argmax(np.sum(np.abs(signals_f), axis=1))
            else:
                f_idx = np.argmin(np.abs(freqs - FREQ_HZ))

            xdata = msg_pose.pose.position.x * 100  # 改为x坐标，转换为厘米
            ydata = np.abs(signals_f[f_idx, :])
            self.get_logger().info(
                f"x coordinate [cm]: {xdata:.0f}, frequency [Hz]: {freqs[f_idx]}"
            )

            if f_idx not in self.frequency_colors.keys():
                if len(self.frequency_colors.keys()) >= MAX_FREQS_TO_PLOT:
                    return
                color_idx = len(self.frequency_colors.keys()) % 10
                color = f"C{color_idx}"
                label = f"{freqs[f_idx]}Hz"
                self.frequency_colors[f_idx] = color
            else:
                color = self.frequency_colors[f_idx]
                label = None

            for i in range(N_MICS):
                self.plotter_dict[i].ax.scatter(
                    xdata, ydata[i], color=color, label=label
                )
            self.plotter_dict[0].ax.legend(loc="upper left")
            
            # 更新位置信息显示
            self._update_position_display()
            
            # 自动保存图片
            self.save_counter += 1
            if self.save_counter % self.save_interval == 0:
                self.save_current_plot_auto()
                self.get_logger().info(f"Auto-saved wall plot after {self.save_counter} updates")
                
        else:
            self.get_logger().warn(
                f"No valid signals message for pose {timestamp}"
            )

        self.fig.canvas.draw()


def main(args=None):
    rclpy.init(args=args)

    plotter = WallApproachPlotter()

    rclpy.spin(plotter)

    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
