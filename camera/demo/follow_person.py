#!/usr/bin/env python3
"""
🚶‍♂️ 人物跟隨模擬器 (Follow Person Simulator)

基於深度攝影機的跟隨功能：
- 鎖定畫面中最近的物體（假設是人）
- 太近 → 停下
- 太遠 → 跟上
- 左右偏移 → 轉向修正 
使用方式:
sudo /Users/yamiko/Documents/VsCode/Go2_Project/.venv/bin/python /Users/yamiko/Documents/VsCode/Go2_Project/camera/demo/follow_person.py
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import os

# 加入 utils 路徑
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from utils.perf_monitor import PerfMonitor, print_system_info

# ===== 跟隨參數 =====
STOP_DISTANCE = 0.6      # 小於 60cm = 太近，停下
FOLLOW_DISTANCE = 1.2    # 大於 120cm = 太遠，跟上
IDEAL_DISTANCE = 0.9     # 理想距離 90cm（甜蜜點中心）
MAX_DISTANCE = 3.0       # 最大偵測距離

# ===== 轉向參數 =====
CENTER_DEADZONE = 30     # 中心死區（像素），在這範圍內不轉向 (降低=更靈敏)
MAX_ANGULAR_Z = 0.6      # 最大轉向速度 (rad/s)
ANGULAR_GAIN = 0.005     # 轉向增益 (提高=更快反應)

# ===== 偵測參數 =====
ROI_WIDTH = 300          # 偵測區域寬度 (加大=更容易追蹤)
ROI_HEIGHT = 350         # 偵測區域高度（縱向較大，適合偵測人）
MIN_BLOB_SIZE = 3000     # 最小物體面積（過濾雜訊，降低=更靈敏）
SEARCH_MARGIN = 100      # 搜尋範圍邊距（避開畫面邊緣雜訊）


def find_target(depth_image, depth_scale):
    """
    找到畫面中央區域內最近的物體（假設是人）
    
    策略：
    1. 優先看畫面中央區域
    2. 用加權方式，讓越靠近中央的物體優先被選中
    
    Returns:
        center_x: 目標中心 X 座標（畫面座標）
        center_y: 目標中心 Y 座標
        distance: 目標距離（公尺）
        blob_size: 物體大小（像素數）
    """
    h, w = depth_image.shape
    cx, cy = w // 2, h // 2
    
    # 轉換成公尺
    depth_meters = depth_image.astype(np.float32) * depth_scale
    
    # 只看有效距離內的物體（排除太近和太遠）
    valid_mask = (depth_meters > 0.2) & (depth_meters < MAX_DISTANCE)
    
    # 排除畫面邊緣（通常是牆壁）
    edge_mask = np.ones_like(valid_mask)
    edge_mask[:SEARCH_MARGIN, :] = False  # 上邊
    edge_mask[-SEARCH_MARGIN:, :] = False  # 下邊
    edge_mask[:, :SEARCH_MARGIN] = False   # 左邊
    edge_mask[:, -SEARCH_MARGIN:] = False  # 右邊
    valid_mask = valid_mask & edge_mask
    
    if not np.any(valid_mask):
        return None, None, 0, 0
    
    # 建立「中央偏好」權重圖
    # 越靠近中央權重越高，這樣即使旁邊有更近的牆壁，也會優先選中央的人
    y_coords, x_coords = np.mgrid[0:h, 0:w]
    dist_to_center = np.sqrt((x_coords - cx)**2 + (y_coords - cy)**2)
    center_weight = 1.0 - (dist_to_center / (np.sqrt(cx**2 + cy**2)))  # 0~1
    center_weight = np.clip(center_weight, 0.3, 1.0)  # 最小權重 0.3
    
    # 計算「綜合分數」= 距離近 + 靠近中央
    # 分數越低越好
    score = np.where(valid_mask,
                     depth_meters / center_weight,  # 距離 / 權重
                     np.inf)
    
    # 找分數最低的區域（最近且最靠近中央）
    threshold_score = np.percentile(score[valid_mask], 15)  # 最佳 15%
    best_mask = (score <= threshold_score) & valid_mask
    
    # 過濾太小的區域
    blob_size = np.sum(best_mask)
    if blob_size < MIN_BLOB_SIZE:
        return None, None, 0, 0
    
    # 計算重心
    target_y_coords, target_x_coords = np.where(best_mask)
    center_x = int(np.mean(target_x_coords))
    center_y = int(np.mean(target_y_coords))
    
    # 計算目標距離（該區域的中位數距離）
    target_distance = np.median(depth_meters[best_mask])
    
    return center_x, center_y, target_distance, blob_size


def get_follow_command(distance, offset_x, frame_width):
    """
    根據距離和偏移量決定跟隨指令
    
    Args:
        distance: 目標距離（公尺）
        offset_x: 目標相對於畫面中心的偏移（像素，正=右，負=左）
        frame_width: 畫面寬度
    
    Returns:
        linear_x: 前進速度 (正=前進, 負=後退, 0=停止)
        angular_z: 轉向速度 (正=左轉, 負=右轉)
        action: 動作名稱
        message: 顯示訊息
        color: UI 顏色
    """
    # === 前進/停止邏輯 ===
    if distance <= 0 or distance > MAX_DISTANCE:
        linear_x = 0
        action = "SEARCH"
        message = "🔍 搜尋目標中..."
        color = (128, 128, 128)  # 灰色
    elif distance < STOP_DISTANCE:
        linear_x = 0
        action = "STOP"
        message = f"🛑 太近了！停下 ({distance:.2f}m)"
        color = (0, 0, 255)  # 紅色
    elif distance > FOLLOW_DISTANCE:
        # 距離越遠，速度越快（但有上限）
        linear_x = min(0.5, (distance - IDEAL_DISTANCE) * 0.3)
        action = "FOLLOW"
        message = f"🏃 跟上！({distance:.2f}m)"
        color = (0, 255, 0)  # 綠色
    else:
        linear_x = 0
        action = "KEEP"
        message = f"✅ 保持距離 ({distance:.2f}m)"
        color = (255, 165, 0)  # 橙色
    
    # === 轉向邏輯 ===
    if abs(offset_x) < CENTER_DEADZONE:
        angular_z = 0
    else:
        # 偏右 → 負值（右轉追過去）
        # 偏左 → 正值（左轉追過去）
        angular_z = -offset_x * ANGULAR_GAIN
        angular_z = np.clip(angular_z, -MAX_ANGULAR_Z, MAX_ANGULAR_Z)
    
    # 轉向時更新訊息
    if angular_z != 0:
        direction = "⬅️ 左轉" if angular_z > 0 else "➡️ 右轉"
        message += f" | {direction}"
    
    return linear_x, angular_z, action, message, color


def draw_follow_ui(frame, target_x, target_y, distance, blob_size, 
                   linear_x, angular_z, action, message, color):
    """繪製跟隨 UI"""
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    
    # 繪製目標追蹤框
    if target_x is not None:
        # 根據 blob_size 估算框大小
        box_size = int(np.sqrt(blob_size) * 0.5)
        box_size = max(50, min(200, box_size))
        
        cv2.rectangle(frame, 
                      (target_x - box_size // 2, target_y - box_size // 2),
                      (target_x + box_size // 2, target_y + box_size // 2),
                      color, 3)
        
        # 畫線連接目標和畫面中心
        cv2.line(frame, (cx, cy), (target_x, target_y), color, 2)
        
        # 標記目標中心
        cv2.circle(frame, (target_x, target_y), 8, color, -1)
    
    # 畫面中心十字
    cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 1)
    cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 1)
    
    # 死區範圍
    cv2.rectangle(frame, 
                  (cx - CENTER_DEADZONE, 50),
                  (cx + CENTER_DEADZONE, h - 50),
                  (100, 100, 100), 1)
    
    # === 左側：距離條 ===
    bar_x, bar_w, bar_h = 30, 30, h - 100
    bar_y = 50
    
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (50, 50, 50), -1)
    
    if distance > 0 and distance <= MAX_DISTANCE:
        fill_ratio = 1.0 - (distance / MAX_DISTANCE)
        fill_h = int(bar_h * fill_ratio)
        cv2.rectangle(frame, (bar_x, bar_y + bar_h - fill_h),
                      (bar_x + bar_w, bar_y + bar_h), color, -1)
    
    # 標記閾值
    stop_y = bar_y + bar_h - int(bar_h * (STOP_DISTANCE / MAX_DISTANCE))
    follow_y = bar_y + bar_h - int(bar_h * (FOLLOW_DISTANCE / MAX_DISTANCE))
    cv2.line(frame, (bar_x - 5, stop_y), (bar_x + bar_w + 5, stop_y), (0, 0, 255), 2)
    cv2.line(frame, (bar_x - 5, follow_y), (bar_x + bar_w + 5, follow_y), (0, 255, 0), 2)
    
    # === 右側：速度指示 ===
    # 前進速度箭頭
    arrow_x = w - 60
    arrow_cy = h // 2
    if linear_x > 0:
        arrow_len = int(linear_x * 100)
        cv2.arrowedLine(frame, (arrow_x, arrow_cy), (arrow_x, arrow_cy - arrow_len),
                        (0, 255, 0), 3, tipLength=0.3)
        cv2.putText(frame, f"{linear_x:.2f}", (arrow_x - 25, arrow_cy - arrow_len - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # 轉向箭頭
    if angular_z != 0:
        turn_x = w - 120
        turn_len = int(abs(angular_z) * 80)
        if angular_z > 0:  # 左轉
            cv2.arrowedLine(frame, (turn_x, arrow_cy), (turn_x - turn_len, arrow_cy),
                            (255, 255, 0), 3, tipLength=0.3)
        else:  # 右轉
            cv2.arrowedLine(frame, (turn_x, arrow_cy), (turn_x + turn_len, arrow_cy),
                            (255, 255, 0), 3, tipLength=0.3)
    
    # === 底部資訊（人讀） ===
    # 主訊息（含箭頭）- 大字
    # 因為 OpenCV 不支援 emoji，用文字代替
    if angular_z > 0:
        turn_text = "<< LEFT"
    elif angular_z < 0:
        turn_text = "RIGHT >>"
    else:
        turn_text = ""
    
    # 動作訊息
    if action == "SEARCH":
        action_text = "SEARCHING..."
    elif action == "STOP":
        action_text = f"STOP! Too close ({distance:.2f}m)"
    elif action == "FOLLOW":
        action_text = f"FOLLOW! ({distance:.2f}m)"
    else:
        action_text = f"KEEP ({distance:.2f}m)"
    
    # 顯示主訊息
    cv2.putText(frame, action_text, (w // 2 - 120, h - 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    
    # 顯示轉向
    if turn_text:
        cv2.putText(frame, turn_text, (w // 2 + 100, h - 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    # 機器數據（小字，給開發者參考）
    cv2.putText(frame, f"x:{linear_x:.2f} z:{angular_z:.2f}",
                (w // 2 - 60, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
    
    # 頂部標題
    cv2.putText(frame, "Follow Person Simulator", (w // 2 - 130, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    
    return frame


def main():
    print("=" * 50)
    print("🚶‍♂️ 人物跟隨模擬器 (Follow Person Simulator)")
    print("=" * 50)
    print(f"📏 距離設定:")
    print(f"   < {STOP_DISTANCE}m = 停下 (STOP)")
    print(f"   {STOP_DISTANCE}m ~ {FOLLOW_DISTANCE}m = 保持 (KEEP)")
    print(f"   > {FOLLOW_DISTANCE}m = 跟上 (FOLLOW)")
    print(f"🎯 轉向死區: ±{CENTER_DEADZONE} 像素")
    print("=" * 50)
    
    print_system_info()
    monitor = PerfMonitor("FollowPerson")
    monitor.start()
    
    # 初始化相機
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    try:
        profile = pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        
        print(f"✅ 相機啟動成功！深度單位: {depth_scale}")
        print("👉 站在相機前面試試看！")
        print("👉 按 'q' 離開")
        
        # 跳過前幾幀讓相機穩定
        for _ in range(30):
            pipeline.wait_for_frames()
        
        frame_count = 0
        
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue
            
            depth_image = np.asanyarray(depth_frame.get_data())
            h, w = depth_image.shape
            
            # 找目標
            target_x, target_y, distance, blob_size = find_target(depth_image, depth_scale)
            
            # 計算偏移量
            if target_x is not None:
                offset_x = target_x - (w // 2)
            else:
                offset_x = 0
            
            # 決定指令
            linear_x, angular_z, action, message, color = get_follow_command(
                distance, offset_x, w
            )
            
            # 視覺化
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )
            
            # 繪製 UI
            ui_frame = draw_follow_ui(
                depth_colormap, target_x, target_y, distance, blob_size,
                linear_x, angular_z, action, message, color
            )
            
            cv2.imshow('Follow Person (Press Q to Exit)', ui_frame)
            
            # 終端機輸出（機器可讀格式）
            print(f"\r[{action}] dist:{distance:.2f} x:{linear_x:.2f} z:{angular_z:.2f} blob:{blob_size}", end="", flush=True)
            
            # 效能監控
            frame_count += 1
            monitor.log(interval=60)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n👋 離開中...")
                break
    
    except RuntimeError as e:
        print(f"❌ 錯誤: {e}")
    
    finally:
        try:
            pipeline.stop()
        except:
            pass
        cv2.destroyAllWindows()
        monitor.report()
        print("程式結束")


if __name__ == "__main__":
    main()
