#!/usr/bin/env python3
"""
🚧 避障模組 (Obstacle Avoidance Module)
輸出 JSON 格式，相容於 MCP 流程

輸出格式與 find_object (YOLO) 相容，可直接整合到現有 MCP 控制流程

使用方式 (本地測試):
python obstacle_json.py

使用方式 (作為模組):
from obstacle_json import ObstacleDetector
detector = ObstacleDetector()
result = detector.detect(distance_m=0.35)
print(result)
"""
import json
import time
from datetime import datetime
from typing import Optional, Dict, Any

class ObstacleDetector:
    """避障偵測器 - 輸出 JSON 格式指令"""
    
    def __init__(
        self,
        close_threshold: float = 0.4,    # 小於此距離 = 後退
        far_threshold: float = 0.8,       # 大於此距離 = 可前進
        max_distance: float = 2.0,        # 最大有效偵測距離
        backup_speed: float = 0.3,        # 後退速度
        forward_speed: float = 0.3,       # 前進速度
    ):
        self.close_threshold = close_threshold
        self.far_threshold = far_threshold
        self.max_distance = max_distance
        self.backup_speed = backup_speed
        self.forward_speed = forward_speed
    
    def detect(self, distance_m: float, direction: str = "前方") -> Dict[str, Any]:
        """
        根據距離產生避障指令
        
        Args:
            distance_m: 偵測到的距離 (公尺)
            direction: 障礙物方向 ("前方", "左側", "右側")
        
        Returns:
            相容於 MCP 的 JSON 格式結果
        """
        timestamp = datetime.now().isoformat()
        
        # 無效距離 (太遠或無法偵測)
        if distance_m <= 0 or distance_m > self.max_distance:
            return {
                "success": True,
                "obstacle_detected": False,
                "distance_m": None,
                "direction": None,
                "cmd_vel": {
                    "linear_x": 0,
                    "angular_z": 0
                },
                "action": "SEARCH",
                "message": "未偵測到前方障礙物，待機中",
                "timestamp": timestamp
            }
        
        # 太近 - 需要後退
        if distance_m < self.close_threshold:
            return {
                "success": True,
                "obstacle_detected": True,
                "distance_m": round(distance_m, 2),
                "direction": direction,
                "cmd_vel": {
                    "linear_x": -self.backup_speed,  # 負值 = 後退
                    "angular_z": 0
                },
                "action": "BACK",
                "message": f"⚠️ 障礙物在{direction}，距離 {distance_m:.2f} 公尺，後退中",
                "timestamp": timestamp
            }
        
        # 安全距離內 - 待命
        if distance_m <= self.far_threshold:
            return {
                "success": True,
                "obstacle_detected": True,
                "distance_m": round(distance_m, 2),
                "direction": direction,
                "cmd_vel": {
                    "linear_x": 0,
                    "angular_z": 0
                },
                "action": "STAY",
                "message": f"✋ 障礙物在{direction}，距離 {distance_m:.2f} 公尺，保持待命",
                "timestamp": timestamp
            }
        
        # 較遠 - 可以前進
        return {
            "success": True,
            "obstacle_detected": True,
            "distance_m": round(distance_m, 2),
            "direction": direction,
            "cmd_vel": {
                "linear_x": self.forward_speed,  # 正值 = 前進
                "angular_z": 0
            },
            "action": "FORWARD",
            "message": f"✅ 障礙物在{direction}，距離 {distance_m:.2f} 公尺，可安全前進",
            "timestamp": timestamp
        }
    
    def to_move_service(self, result: Dict[str, Any], duration: float = 1.0) -> Dict[str, Any]:
        """
        將避障結果轉換為 MCP call_service 格式
        
        Args:
            result: detect() 的輸出結果
            duration: 移動持續時間 (秒)
        
        Returns:
            可直接用於 MCP call_service 的 JSON
        """
        return {
            "service_name": "/move_for_duration",
            "service_type": "go2_interfaces/srv/MoveForDuration",
            "request": {
                "linear_x": result["cmd_vel"]["linear_x"],
                "angular_z": result["cmd_vel"]["angular_z"],
                "duration": duration
            },
            "timeout": None
        }


def demo():
    """示範模式 - 模擬不同距離的偵測結果"""
    print("=" * 60)
    print("🚧 避障模組 Demo")
    print("=" * 60)
    
    detector = ObstacleDetector()
    
    # 測試不同距離
    test_cases = [
        (0.3, "前方"),   # 太近 - 後退
        (0.5, "前方"),   # 安全距離 - 待命
        (1.0, "前方"),   # 較遠 - 可前進
        (3.0, "前方"),   # 超出範圍 - 搜尋
    ]
    
    for distance, direction in test_cases:
        print(f"\n📏 測試距離: {distance}m")
        print("-" * 40)
        
        result = detector.detect(distance, direction)
        print(f"Action: {result['action']}")
        print(f"Message: {result['message']}")
        print(f"cmd_vel: {result['cmd_vel']}")
        
        # 轉換為 MCP service 格式
        service_cmd = detector.to_move_service(result, duration=1.5)
        print(f"\n📤 MCP Service 格式:")
        print(json.dumps(service_cmd, indent=2, ensure_ascii=False))
    
    print("\n" + "=" * 60)
    print("✅ Demo 完成")


if __name__ == "__main__":
    demo()
