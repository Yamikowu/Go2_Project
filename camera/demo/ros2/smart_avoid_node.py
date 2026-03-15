#!/usr/bin/env python3
"""
🧠 Smart Avoid Node - 聰明閃避節點

比 safety_layer 更聰明：會轉彎繞過障礙物，不只後退。

原理：
  把畫面分成 左/中/右 三個區域，
  根據三邊距離決定要左轉、右轉、後退還是直走。

使用方式:
  ros2 run go2_safety smart_avoid_node

行為表:
  左=遠, 中=近, 右=遠  →  後退 (前方太窄)
  左=遠, 中=近, 右=近  →  左轉
  左=近, 中=近, 右=遠  →  右轉
  左=近, 中=近, 右=近  →  後退 (都太近)
  左=遠, 中=遠, 右=遠  →  不干預 (安全)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import json
from cv_bridge import CvBridge
import time


class SmartAvoidNode(Node):
    """
    聰明閃避節點 - 會轉彎的 Safety Layer
    """
    
    def __init__(self):
        super().__init__('smart_avoid_node')
        
        # 參數
        self.declare_parameter('danger_distance', 0.4)   # 危險距離 (公尺)
        self.declare_parameter('safe_distance', 0.8)     # 安全距離 (公尺)
        self.declare_parameter('max_distance', 2.0)      # 最大偵測距離
        self.declare_parameter('backup_speed', 0.25)     # 後退速度
        self.declare_parameter('turn_speed', 0.5)        # 轉彎角速度
        self.declare_parameter('enable', True)           # 是否啟用
        
        self.danger_distance = self.get_parameter('danger_distance').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.backup_speed = self.get_parameter('backup_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.enable = self.get_parameter('enable').value
        
        self.bridge = CvBridge()
        
        # 訂閱深度影像
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # 發布控制指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/smart_avoid_status', 10)
        
        self.frame_count = 0
        self.last_action = ""
        
        self.get_logger().info('🧠 Smart Avoid Node 啟動')
        self.get_logger().info(f'   危險距離: {self.danger_distance}m')
        self.get_logger().info(f'   安全距離: {self.safe_distance}m')
    
    def depth_callback(self, msg):
        """處理深度影像"""
        if not self.enable:
            return
        
        start_time = time.time()
        
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = depth_image.shape
            
            # 分成左/中/右三個區域
            third = w // 3
            
            left_roi = depth_image[:, :third]
            center_roi = depth_image[:, third:2*third]
            right_roi = depth_image[:, 2*third:]
            
            # 取得各區域距離
            left_dist = self._get_distance(left_roi)
            center_dist = self._get_distance(center_roi)
            right_dist = self._get_distance(right_roi)
            
            # 決定動作
            action, cmd = self._decide_action(left_dist, center_dist, right_dist)
            
            # 發布指令
            if action != "CLEAR" and action != self.last_action:
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info(
                    f'🚨 {action} | 左:{left_dist:.2f}m 中:{center_dist:.2f}m 右:{right_dist:.2f}m'
                )
            
            # 發布狀態
            self._publish_status(left_dist, center_dist, right_dist, action, cmd)
            
            self.frame_count += 1
            self.last_action = action
            
            # 效能監控
            if self.frame_count % 30 == 0:
                reaction_time = (time.time() - start_time) * 1000
                self.get_logger().info(f'📊 反應時間: {reaction_time:.1f}ms')
        
        except Exception as e:
            self.get_logger().error(f'❌ 錯誤: {str(e)}')
    
    def _get_distance(self, roi):
        """
        取得區域的代表距離 (中位數)
        """
        valid = roi[roi > 0]
        if len(valid) == 0:
            return self.max_distance + 1  # 視為很遠
        
        # 假設深度單位是 mm，轉 m
        return np.median(valid) / 1000.0
    
    def _decide_action(self, left, center, right):
        """
        根據左/中/右距離決定動作
        
        Returns:
            (action_name, Twist)
        """
        cmd = Twist()
        
        # 判斷各區是否「危險」
        left_danger = left < self.danger_distance
        center_danger = center < self.danger_distance
        right_danger = right < self.danger_distance
        
        left_safe = left > self.safe_distance
        center_safe = center > self.safe_distance
        right_safe = right > self.safe_distance
        
        # ===== 決策邏輯 =====
        
        # 全部安全 → 不干預
        if left_safe and center_safe and right_safe:
            return "CLEAR", cmd
        
        # 中間危險，但左邊安全 → 右轉 (往左邊空間)
        # 注意：angular_z 正值是逆時針 (左轉)，負值是順時針 (右轉)
        if center_danger and left_safe and not right_safe:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed  # 左轉
            return "TURN_LEFT", cmd
        
        # 中間危險，但右邊安全 → 左轉 (往右邊空間)
        if center_danger and right_safe and not left_safe:
            cmd.linear.x = 0.0
            cmd.angular.z = -self.turn_speed  # 右轉
            return "TURN_RIGHT", cmd
        
        # 中間危險，左右都安全 → 選一邊 (預設左轉)
        if center_danger and left_safe and right_safe:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed  # 左轉
            return "TURN_LEFT", cmd
        
        # 三邊都危險 → 後退
        if left_danger and center_danger and right_danger:
            cmd.linear.x = -self.backup_speed
            cmd.angular.z = 0.0
            return "BACKUP", cmd
        
        # 中間安全但左或右危險 → 閃一下
        if not center_danger:
            if left_danger and not right_danger:
                cmd.linear.x = 0.0
                cmd.angular.z = -self.turn_speed * 0.5  # 微右轉
                return "DODGE_RIGHT", cmd
            if right_danger and not left_danger:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed * 0.5  # 微左轉
                return "DODGE_LEFT", cmd
        
        # 其他情況 → 後退保守
        cmd.linear.x = -self.backup_speed
        cmd.angular.z = 0.0
        return "BACKUP", cmd
    
    def _publish_status(self, left, center, right, action, cmd):
        """發布狀態 JSON"""
        status = {
            "distances": {
                "left": round(left, 2),
                "center": round(center, 2),
                "right": round(right, 2)
            },
            "action": action,
            "cmd_vel": {
                "linear_x": cmd.linear.x,
                "angular_z": cmd.angular.z
            },
            "frame": self.frame_count
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SmartAvoidNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 Smart Avoid 節點關閉')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
