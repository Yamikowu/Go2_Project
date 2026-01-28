#!/usr/bin/env python3
"""
🧪 Smart Avoid 本地測試版 (不需要 ROS2)

在 Mac 上用 RealSense 測試避障邏輯，不需要 ROS2 也不需要狗。
你就是狗！看著畫面，感受指令。

使用方式:
sudo /Users/yamiko/Documents/VsCode/Go2_Project/.venv/bin/python /Users/yamiko/Documents/VsCode/Go2_Project/camera/demo/smart_avoid.py

操作:
- 把手放在相機前面左/中/右區域
- 觀察終端機輸出的動作和反應時間
- 按 'q' 離開
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import json
import sys
import os

# 加入 utils 路徑
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from utils.perf_monitor import PerfMonitor, print_system_info

# ===== 參數設定 =====
DANGER_DISTANCE = 0.4   # 危險距離 (公尺)
SAFE_DISTANCE = 0.8     # 安全距離 (公尺)
MAX_DISTANCE = 2.0      # 最大偵測距離
BACKUP_SPEED = 0.25     # 後退速度 (模擬)
TURN_SPEED = 0.5        # 轉彎角速度 (模擬)


def get_distance(roi):
    """取得區域的中位數距離 (公尺)"""
    # RealSense 深度單位是 mm，先轉成 m
    roi_m = roi.astype(np.float32) / 1000.0
    
    # 過濾無效值：太近 (<0.1m) 或太遠 (>MAX_DISTANCE) 都不算
    valid = roi_m[(roi_m > 0.1) & (roi_m < MAX_DISTANCE)]
    
    if len(valid) == 0:
        return MAX_DISTANCE + 1  # 沒有有效資料，視為很遠
    
    return float(np.median(valid))


def decide_action(left, center, right):
    """
    根據左/中/右距離決定動作
    
    距離定義:
    - < 0.4m = 危險 (DANGER)
    - 0.4~0.8m = 注意 (CAUTION) 
    - > 0.8m = 安全 (SAFE)
    
    Returns:
        (action, linear_x, angular_z, color)
    """
    left_danger = left < DANGER_DISTANCE
    center_danger = center < DANGER_DISTANCE
    right_danger = right < DANGER_DISTANCE
    
    left_safe = left > SAFE_DISTANCE
    center_safe = center > SAFE_DISTANCE
    right_safe = right > SAFE_DISTANCE
    
    # === 優先級 1: 中間危險 (<0.4m)，必須閃避 ===
    if center_danger:
        # 左邊安全，往左轉
        if left_safe:
            return "TURN_LEFT", 0, TURN_SPEED, (255, 255, 0)
        # 右邊安全，往右轉
        if right_safe:
            return "TURN_RIGHT", 0, -TURN_SPEED, (255, 255, 0)
        # 都不安全，只能後退
        return "BACKUP", -BACKUP_SPEED, 0, (0, 0, 255)
    
    # === 優先級 2: 側邊危險 (<0.4m)，微調 ===
    if left_danger:
        return "DODGE_RIGHT", 0, -TURN_SPEED * 0.5, (255, 200, 0)
    if right_danger:
        return "DODGE_LEFT", 0, TURN_SPEED * 0.5, (255, 200, 0)
    
    # === 優先級 3: 中間在注意區 (0.4~0.8m)，減速但可繼續 ===
    if not center_safe:  # 0.4 <= center <= 0.8
        return "CAUTION", 0, 0, (255, 165, 0)  # 橙色
    
    # === 全部安全 (>0.8m) ===
    return "CLEAR", 0, 0, (0, 255, 0)


def draw_dashboard(frame, left, center, right, action, linear_x, angular_z, reaction_ms, memory_mb, color):
    """繪製儀表板"""
    h, w = frame.shape[:2]
    
    # 分隔線
    third = w // 3
    cv2.line(frame, (third, 0), (third, h), (100, 100, 100), 2)
    cv2.line(frame, (2*third, 0), (2*third, h), (100, 100, 100), 2)
    
    # 各區域距離文字
    cv2.putText(frame, f"L:{left:.2f}m", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"C:{center:.2f}m", (third + 10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"R:{right:.2f}m", (2*third + 10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # 動作指示 (大字)
    cv2.putText(frame, action, (w//2 - 80, h//2), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 3)
    
    # 模擬指令
    cmd_text = f"cmd_vel: linear_x={linear_x:.2f}, angular_z={angular_z:.2f}"
    cv2.putText(frame, cmd_text, (10, h - 55), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    # 反應時間
    cv2.putText(frame, f"Reaction: {reaction_ms:.1f}ms", (10, h - 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    # 記憶體用量
    cv2.putText(frame, f"Memory: {memory_mb:.1f}MB", (10, h - 8), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    # 邊框顏色
    cv2.rectangle(frame, (5, 5), (w-5, h-5), color, 3)
    
    return frame


def main():
    # 先印出系統資訊
    print_system_info()
    
    print("=" * 60)
    print("🧪 Smart Avoid 本地測試")
    print("=" * 60)
    print(f"危險距離: < {DANGER_DISTANCE}m")
    print(f"安全距離: > {SAFE_DISTANCE}m")
    print("=" * 60)
    print("把手放在相機前面測試，按 'q' 離開")
    print("=" * 60)
    
    # 效能監控
    monitor = PerfMonitor("SmartAvoid")
    monitor.start()
    
    # 初始化相機
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    try:
        pipeline.start(config)
        print("✅ 相機啟動成功！")
        
        # 跳過前幾幀
        for _ in range(30):
            pipeline.wait_for_frames()
        
        reaction_times = []
        
        while True:
            start_time = time.time()
            
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue
            
            depth_image = np.asanyarray(depth_frame.get_data())
            h, w = depth_image.shape
            
            # 分成左/中/右
            third = w // 3
            left_roi = depth_image[:, :third]
            center_roi = depth_image[:, third:2*third]
            right_roi = depth_image[:, 2*third:]
            
            # 計算距離
            left_dist = get_distance(left_roi)
            center_dist = get_distance(center_roi)
            right_dist = get_distance(right_roi)
            
            # 決定動作
            action, linear_x, angular_z, color = decide_action(
                left_dist, center_dist, right_dist
            )
            
            reaction_ms = (time.time() - start_time) * 1000
            reaction_times.append(reaction_ms)
            
            # 取得記憶體用量
            perf = monitor.get_current()
            memory_mb = perf["memory_mb"]
            
            # 轉換為彩色圖顯示
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )
            
            # 繪製儀表板
            dashboard = draw_dashboard(
                depth_colormap, 
                left_dist, center_dist, right_dist,
                action, linear_x, angular_z, reaction_ms, memory_mb, color
            )
            
            cv2.imshow('Smart Avoid Test (Press Q to Exit)', dashboard)
            
            # 每秒輸出一次統計 (包含距離和動作)
            if len(reaction_times) % 30 == 0:
                print(f"📊 左:{left_dist:.2f}m 中:{center_dist:.2f}m 右:{right_dist:.2f}m | "
                      f"動作:{action} | 記憶體:{memory_mb:.1f}MB | 反應:{reaction_ms:.1f}ms")
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except RuntimeError as e:
        print(f"❌ 錯誤: {e}")
    
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        
        # 最終統計
        if reaction_times:
            print("\n" + "=" * 60)
            print("📈 測試報告")
            print("=" * 60)
            print(f"總幀數: {len(reaction_times)}")
            print(f"平均反應時間: {np.mean(reaction_times):.1f}ms")
            print(f"最快: {np.min(reaction_times):.1f}ms")
            print(f"最慢: {np.max(reaction_times):.1f}ms")
        
        # 效能報告
        monitor.report()


if __name__ == "__main__":
    main()

