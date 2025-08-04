# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

import numpy as np

from audio_interfaces.msg import Spectrum, DoaEstimates
from audio_interfaces_py.messages import read_spectrum_message, create_doa_message
from audio_stack.beam_former import combine_rows, normalize_rows

N_ESTIMATES = 3  # number of peaks to detect
COMBINATION_METHOD = "sum"  # way to combine across frequencies
NORMALIZE = "zero_to_one"


class DoaEstimator(Node):
    def __init__(self):
        super().__init__("doa_estimator")

        self.subscription_spectrum = self.create_subscription(
            Spectrum, "audio/spectrum_combined", self.listener_callback_spectrum, 10
        )

        self.publisher_doa = self.create_publisher(
            DoaEstimates, "geometry/doa_estimates", 10
        )
        
        # 禁用自动关闭定时器，以便持续运行
        # self.shutdown_timer = self.create_timer(5.0, self.shutdown_callback)
        # self.get_logger().info("DOA Estimator will shutdown in 5 seconds")
        
        # 备选方案：记录开始时间，在回调中检查
        # import time
        # self.start_time = time.time()
        # self.run_duration = 10.0  # 运行10秒

    def shutdown_callback(self):
        """10秒后自动关闭节点"""
        self.get_logger().info("10 seconds elapsed, shutting down DOA estimator node")
        rclpy.shutdown()

    def listener_callback_spectrum(self, msg_spec):
        # 备选方案：在每次回调中检查运行时间
        # import time
        # if time.time() - self.start_time > self.run_duration:
        #     self.get_logger().info("10 seconds elapsed, shutting down DOA estimator node")
        #     rclpy.shutdown()
        #     return
        
        spectrum, *_ = read_spectrum_message(msg_spec)
        
        # calculate and publish doa estimates
        final_spectrum = combine_rows(spectrum, COMBINATION_METHOD, keepdims=True)
        final_spectrum = normalize_rows(final_spectrum, NORMALIZE)
        
        angles = np.linspace(0, 360, msg_spec.n_angles)
        sorted_indices = np.argsort(final_spectrum.flatten())

        '''
        #### 添加调试：显示最强的几个角度
        top_angles = angles[sorted_indices[-10:][::-1]]  # 最强的10个角度
        top_powers = final_spectrum.flatten()[sorted_indices[-10:][::-1]]
        
        self.get_logger().info(f"Top 10 angles and powers:")
        for i, (angle, power) in enumerate(zip(top_angles, top_powers)):
            self.get_logger().info(f"  {i+1}: {angle:.1f}° (power: {power:.3f})")
        #### 
        '''
        sorted_indices = np.argsort(
            final_spectrum.flatten()
        )  # sorts in ascending order
        doa_estimates = angles[sorted_indices[-N_ESTIMATES:][::-1]]

        msg_doa = create_doa_message(doa_estimates, msg_spec.timestamp)
        self.publisher_doa.publish(msg_doa)
        self.get_logger().info(f"Published DOA estimates: {doa_estimates} at timestamp {msg_spec.timestamp}ms")
        # self.get_logger().info(f"msg_doa: {msg_doa}")



def main(args=None):
    rclpy.init(args=args)

    estimator = DoaEstimator()

    rclpy.spin(estimator)

    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
