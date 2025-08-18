"""
linear_pose_publisher.py: Publish linear translational movement of Crazyflie inside a room. 
"""

import time
import os

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped
from audio_interfaces.msg import PoseRaw

from audio_interfaces_py.messages import (
    create_pose_message_from_arrays,
    create_pose_message,
    create_pose_raw_message,
)
from audio_simulation.geometry import (
    ROOM_DIM,
    STARTING_POS,
    STARTING_YAW_DEG,
)

VELOCITY = np.array([0.04, 0.0, 0.0])  # m/s, in drone coordinates
# 修改为x方向前进，匹配wall_detection的前向运动
EPS = 0.10  # m, safety margin from walls
MAX_Y = None


class LinearPosePublisher(Node):
    def __init__(
        self,
        starting_position=STARTING_POS,
        constant_velocity=VELOCITY,
        starting_yaw_deg=STARTING_YAW_DEG,
        max_y=MAX_Y,
    ):
        super().__init__("linear_pose_publisher")

        self.publisher_pose = self.create_publisher(PoseStamped, "geometry/pose", 10)
        self.publisher_pose_raw = self.create_publisher(PoseRaw, "geometry/pose_raw", 10)

        self.rot = R.from_euler("z", starting_yaw_deg, degrees=True)
        self.constant_velocity = constant_velocity
        self.position = starting_position
        self.max_y = max_y
        assert np.all(
            (EPS <= self.position) | (self.position <= ROOM_DIM - EPS)
        ), "starting position not inside room!"

        # start movement
        self.timer_period = 0.5  # seconds
        # we do not need to wait before starting the timer
        # because the timer first waits for self.timer_period.
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 初始化相对时间戳计数器
        self.start_time = time.time()
        self.timestamp_counter = 0

        # # 添加停止标志
        # self.should_stop = False

    def get_time_ms(self):
        # 使用简单的计数器方法，每次调用递增
        self.timestamp_counter += int(self.timer_period * 1000)  # 转换为毫秒
        # 确保在uint32范围内
        return self.timestamp_counter % (2**32)

    def shutdown_system(self, reason="boundary reached"):
        """关闭整个系统"""
        self.get_logger().warn(f"Shutting down system: {reason}")
        self.get_logger().warn(f"Final position: X={self.position[0]:.3f}, Y={self.position[1]:.3f}, Z={self.position[2]:.3f}")
        
        # 设置停止标志
        self.should_stop = True
        
        # 停止定时器
        if hasattr(self, 'timer'):
            self.timer.cancel()
        
         # 关闭ROS节点
        try:
            self.get_logger().info("Destroying node...")
            self.destroy_node()
            
            # 关闭整个ROS系统
            rclpy.shutdown()
            
            # 强制退出程序
            self.get_logger().info("Exiting program...")
            os._exit(0)
            
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
            # 强制终止
            os._exit(1)

    def timer_callback(self):
        # if self.should_stop:
        #     return
        
        delta = self.rot.apply(self.timer_period * self.constant_velocity)
        new_position = self.position + delta

        # if we hit the wall in x-direction, revert x-direction  
        if (new_position[0] > ROOM_DIM[0] - EPS) or (new_position[0] < EPS):
            self.get_logger().warn("touched wall in x-direction, turn direction")
            # self.get_logger().warn("touched wall in x-direction, stop moving!")
            self.constant_velocity[0] = -self.constant_velocity[0]
            new_position = self.position
            # self.shutdown_system("Reached x boundary, shut down system")
            # # 重新计算新位置，使用反向速度
            # delta = self.rot.apply(self.timer_period * self.constant_velocity)
            # new_position = self.position + delta

        # if we hit the wall, revert y-direction
        if (new_position[1] > ROOM_DIM[1] - EPS) or (new_position[1] < EPS):
            self.get_logger().warn("touched wall, turn direction")
            self.constant_velocity[1] = -self.constant_velocity[1]
            new_position = self.position
            # # 重新计算新位置，使用反向速度
            # delta = self.rot.apply(self.timer_period * self.constant_velocity)
            # new_position = self.position + delta

        # if we hit the ceiling/floor, revert z-direction
        if (new_position[2] > ROOM_DIM[2] - EPS) or (new_position[2] < EPS):
            self.get_logger().warn("touched ceiling/floor, turn direction")
            self.constant_velocity[2] = -self.constant_velocity[2]
            new_position = self.position
            # # 重新计算新位置，使用反向速度
            # delta = self.rot.apply(self.timer_period * self.constant_velocity)
            # new_position = self.position + delta
            
        # if we reached max_y, revert direction.
        if (self.max_y is not None) and (new_position[1] > self.max_y):
            self.get_logger().warn(f"reached {self.max_y}, turn direction")
            self.constant_velocity[1] = -self.constant_velocity[1]
            new_position = self.position
            # # 重新计算新位置，使用反向速度
            # delta = self.rot.apply(self.timer_period * self.constant_velocity)
            # new_position = self.position + delta

        self.position = new_position
        quat = self.rot.as_quat()

        timestamp = self.get_time_ms()
        msg = create_pose_message_from_arrays(quat, self.position, timestamp=timestamp)
        self.publisher_pose.publish(msg)

        # Publish raw pose message
        yaw_deg = self.rot.as_euler("zyx", degrees=True)[0]  # 提取z轴旋转角度(yaw)
        msg_raw = create_pose_raw_message(
            x=self.position[0], y=self.position[1], z=self.position[2],
            yaw_deg=yaw_deg,
            vx=self.constant_velocity[0], vy=self.constant_velocity[1],
            yaw_rate_deg=0.0,  # 假设角速度为0
            timestamp=timestamp,
        )
        self.publisher_pose_raw.publish(msg_raw)
        self.get_logger().info(f"PoseRaw has been published at time {timestamp}")

def main(args=None):
    rclpy.init(args=args)

    linear_pub = LinearPosePublisher()

    rclpy.spin(linear_pub)

    linear_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
