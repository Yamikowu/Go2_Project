#!/usr/bin/env python3
"""
🎮 自動避障（簡易版，以框框為主）

使用方式:
sudo /Users/yamiko/Documents/VsCode/Go2_Project/.venv/bin/python /Users/yamiko/Documents/VsCode/Go2_Project/camera/demo/obstacle.py
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import time

# ===== 參數設定 =====
CLOSE_THRESHOLD = 0.4    # 小於 40cm = 太近，後退
FAR_THRESHOLD = 0.8      # 大於 80cm = 太遠，前進
MAX_DISTANCE = 2.0       # 最大偵測距離 (超過視為「沒偵測到」)
ROI_SIZE = 100           # 偵測區域大小 (畫面中央的正方形)

def get_command(distance):
    """根據距離決定機器狗應該做什麼"""
    if distance <= 0 or distance > MAX_DISTANCE:
        return "SEARCH", "🔍 搜尋中...", (128, 128, 128)  # 灰色
    elif distance < CLOSE_THRESHOLD:
        return "BACK", "⬅️ 後退！太近了！", (0, 0, 255)  # 紅色
    elif distance > FAR_THRESHOLD:
        return "FORWARD", "➡️ 前進！跟上！", (0, 255, 0)  # 綠色
    else:
        return "STAY", "🛑 待命 (甜蜜點)", (255, 165, 0)  # 橙色

def draw_dashboard(frame, distance, command, message, color):
    """繪製儀表板 UI"""
    h, w = frame.shape[:2]
    
    # 繪製中央偵測區域框
    cx, cy = w // 2, h // 2
    half = ROI_SIZE // 2
    cv2.rectangle(frame, (cx - half, cy - half), (cx + half, cy + half), color, 3)
    
    # 繪製距離條 (左側)
    bar_x = 30
    bar_w = 30
    bar_h = h - 100
    bar_y = 50
    
    # 背景條
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (50, 50, 50), -1)
    
    # 填充條 (根據距離)
    if distance > 0 and distance <= MAX_DISTANCE:
        fill_ratio = 1.0 - (distance / MAX_DISTANCE)
        fill_h = int(bar_h * fill_ratio)
        cv2.rectangle(frame, (bar_x, bar_y + bar_h - fill_h), 
                      (bar_x + bar_w, bar_y + bar_h), color, -1)
    
    # 標記閾值線
    close_y = bar_y + bar_h - int(bar_h * (CLOSE_THRESHOLD / MAX_DISTANCE))
    far_y = bar_y + bar_h - int(bar_h * (FAR_THRESHOLD / MAX_DISTANCE))
    cv2.line(frame, (bar_x - 5, close_y), (bar_x + bar_w + 5, close_y), (0, 0, 255), 2)
    cv2.line(frame, (bar_x - 5, far_y), (bar_x + bar_w + 5, far_y), (0, 255, 0), 2)
    
    # 繪製文字資訊 (底部)
    # 距離
    dist_text = f"Distance: {distance:.2f} m" if distance > 0 else "Distance: ---"
    cv2.putText(frame, dist_text, (w // 2 - 100, h - 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    # 指令
    cv2.putText(frame, f"Command: {command}", (w // 2 - 100, h - 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    
    # 頂部標題
    cv2.putText(frame, "Force Control Simulator", (w // 2 - 150, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    
    return frame

def main():
    print("=" * 50)
    print("🎮 原力控制模擬器 (Force Control Simulator)")
    print("=" * 50)
    print(f"📏 距離閾值:")
    print(f"   < {CLOSE_THRESHOLD}m = 後退 (BACK)")
    print(f"   {CLOSE_THRESHOLD}m ~ {FAR_THRESHOLD}m = 待命 (STAY)")
    print(f"   > {FAR_THRESHOLD}m = 前進 (FORWARD)")
    print("=" * 50)
    
    # 初始化相機
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    try:
        pipeline.start(config)
        print("✅ 相機啟動成功！")
        print("👉 把手放在相機前面試試看！")
        print("👉 按 'q' 離開")
        
        # 跳過前幾幀讓相機穩定
        for _ in range(30):
            pipeline.wait_for_frames()
        
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue
            
            # 轉換成 numpy array
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # 取得畫面中央區域的平均距離
            h, w = depth_image.shape
            cx, cy = w // 2, h // 2
            half = ROI_SIZE // 2
            roi = depth_image[cy - half:cy + half, cx - half:cx + half]
            
            # 過濾掉 0 值 (無效讀數)
            valid_depths = roi[roi > 0]
            if len(valid_depths) > 0:
                # 取中位數而不是平均，避免雜訊影響
                distance = np.median(valid_depths) * depth_frame.get_units()
            else:
                distance = 0
            
            # 決定指令
            command, message, color = get_command(distance)
            
            # 轉換成彩色圖顯示
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            
            # 繪製儀表板
            dashboard = draw_dashboard(depth_colormap, distance, command, message, color)
            
            # 顯示
            cv2.imshow('Force Control (Press Q to Exit)', dashboard)
            
            # 終端機也印出來
            print(f"\r{message} | {distance:.2f}m", end="", flush=True)
            
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
        print("程式結束")

if __name__ == "__main__":
    main()
