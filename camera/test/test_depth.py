#!/usr/bin/env python3
"""
簡單深度測試 - 確認相機正常運作
sudo /Users/yamiko/Documents/VsCode/Go2_Project/.venv/bin/python /Users/yamiko/Documents/VsCode/Go2_Project/camera/test/test_depth.py
"""
import pyrealsense2 as rs
import numpy as np
import cv2

print("🔄 測試深度相機...")

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

try:
    pipeline.start(config)
    print("✅ 深度相機正常！按 'q' 離開")
    
    while True:
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        if not depth:
            continue
        
        depth_image = np.asanyarray(depth.get_data())
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), 
            cv2.COLORMAP_JET
        )
        
        # 顯示中心距離
        center_dist = depth.get_distance(320, 240)
        cv2.putText(depth_colormap, f"Center: {center_dist:.2f}m", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        cv2.imshow('Depth Test (Press Q)', depth_colormap)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except RuntimeError as e:
    print(f"❌ 錯誤: {e}")

finally:
    try:
        pipeline.stop()
    except:
        pass
    cv2.destroyAllWindows()
    print("測試結束")
