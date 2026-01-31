#!/usr/bin/env python3
"""
📏 物件體積測量器 (Object Sizer)

使用 RealSense D435 深度相機測量物件的 寬 × 高 × 深度 (公分)。

使用方式:
sudo /Users/yamiko/Documents/VsCode/Go2_Project/.venv/bin/python /Users/yamiko/Documents/VsCode/Go2_Project/camera/demo/object_sizer.py

控制鍵:
- R: 重設測量
- +: 增加最大距離
- -: 減少最大距離
- Q: 離開
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import os

# 加入 utils 路徑
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from utils.perf_monitor import PerfMonitor, print_system_info

# ===== 參數設定 =====
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
FPS = 30

MIN_DISTANCE = 0.20   # 最小偵測距離 (公尺)
MAX_DISTANCE = 0.80   # 最大偵測距離 (公尺)
DISTANCE_STEP = 0.05  # 距離調整步長

MIN_CONTOUR_AREA = 2000  # 最小輪廓面積 (像素)


class ObjectSizer:
    def __init__(self):
        self.pipeline = None
        self.intrinsics = None
        self.min_dist = MIN_DISTANCE
        self.max_dist = MAX_DISTANCE
        
        # 濾鏡
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.spatial.set_option(rs.option.filter_smooth_delta, 20)
        
        self.temporal = rs.temporal_filter()
        
        # 效能監控
        self.monitor = PerfMonitor("ObjectSizer")
    
    def setup_camera(self):
        """初始化相機"""
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FPS)
        
        print("🎥 啟動深度相機...")
        profile = self.pipeline.start(config)
        
        # 取得相機內參 (用於像素轉公分)
        depth_stream = profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
        
        print(f"📐 相機焦距: fx={self.intrinsics.fx:.1f}, fy={self.intrinsics.fy:.1f}")
        
        # 預熱
        print("⏳ 相機預熱中...")
        for _ in range(30):
            self.pipeline.wait_for_frames()
        
        print("✅ 相機準備完成！")
    
    def process_frame(self, depth_frame):
        """處理深度幀，回傳物件測量結果"""
        # 套用濾鏡
        filtered = self.spatial.process(depth_frame)
        filtered = self.temporal.process(filtered)
        
        # 轉成 numpy
        depth_image = np.asanyarray(filtered.get_data())
        depth_meters = depth_image * depth_frame.get_units()
        
        # 建立遮罩：只取範圍內的深度
        mask = (depth_meters > self.min_dist) & (depth_meters < self.max_dist)
        mask = (mask * 255).astype(np.uint8)
        
        # 消除雜訊
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 找輪廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        measurement = None
        bbox = None
        
        if contours:
            # 找最大輪廓
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            if area > MIN_CONTOUR_AREA:
                # 取得邊界框
                x, y, w, h = cv2.boundingRect(largest)
                bbox = (x, y, w, h)
                
                # 計算物件區域內的深度
                roi_mask = np.zeros_like(mask)
                cv2.drawContours(roi_mask, [largest], -1, 255, -1)
                
                valid_depths = depth_meters[(roi_mask > 0) & (depth_meters > 0)]
                
                if len(valid_depths) > 100:
                    avg_dist = np.median(valid_depths)
                    min_depth = np.percentile(valid_depths, 5)
                    max_depth = np.percentile(valid_depths, 95)
                    
                    # 像素轉公分
                    width_cm = (w * avg_dist) / self.intrinsics.fx * 100
                    height_cm = (h * avg_dist) / self.intrinsics.fy * 100
                    depth_cm = (max_depth - min_depth) * 100
                    
                    measurement = {
                        'width': width_cm,
                        'height': height_cm,
                        'depth': depth_cm,
                        'distance': avg_dist,
                        'area': area
                    }
        
        return depth_image, mask, bbox, measurement
    
    def draw_ui(self, depth_image, mask, bbox, measurement):
        """繪製 UI"""
        # 深度圖轉彩色
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), 
            cv2.COLORMAP_JET
        )
        
        h, w = depth_colormap.shape[:2]
        
        # 畫邊界框
        if bbox:
            x, y, bw, bh = bbox
            cv2.rectangle(depth_colormap, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
        
        # 測量結果
        if measurement:
            text = f"{measurement['width']:.1f} x {measurement['height']:.1f} x {measurement['depth']:.1f} cm"
            
            # 背景框
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
            tx = w // 2 - tw // 2
            ty = h - 80
            cv2.rectangle(depth_colormap, (tx - 10, ty - th - 10), (tx + tw + 10, ty + 10), (0, 0, 0), -1)
            cv2.putText(depth_colormap, text, (tx, ty), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            # 距離
            dist_text = f"Distance: {measurement['distance']:.2f}m"
            cv2.putText(depth_colormap, dist_text, (20, h - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        else:
            cv2.putText(depth_colormap, "No object detected", (w // 2 - 100, h - 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 距離範圍
        range_text = f"Range: {self.min_dist:.2f}m - {self.max_dist:.2f}m"
        cv2.putText(depth_colormap, range_text, (w - 250, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # 標題
        cv2.putText(depth_colormap, "Object Sizer", (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # 控制說明
        controls = "[R] Reset  [+/-] Range  [Q] Quit"
        cv2.putText(depth_colormap, controls, (20, h - 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return depth_colormap
    
    def run(self):
        """主迴圈"""
        print_system_info()
        
        print("\n" + "=" * 60)
        print("📏 物件體積測量器")
        print("=" * 60)
        print("📖 使用說明:")
        print(f"   1. 將物件放在相機前 {self.min_dist:.0f}-{self.max_dist:.0f}cm 處")
        print("   2. 系統會自動偵測並測量尺寸")
        print("=" * 60)
        print("控制鍵:")
        print("   R  : 重設")
        print("   +  : 增加最大距離")
        print("   -  : 減少最大距離")
        print("   Q  : 離開")
        print("=" * 60)
        
        try:
            self.setup_camera()
            self.monitor.start()
            
            while True:
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                
                if not depth_frame:
                    continue
                
                # 處理
                depth_image, mask, bbox, measurement = self.process_frame(depth_frame)
                
                # 繪製 UI
                display = self.draw_ui(depth_image, mask, bbox, measurement)
                
                # 顯示
                cv2.imshow('Object Sizer (Press Q to Exit)', display)
                
                # 效能監控
                self.monitor.log(interval=30)
                
                # 處理按鍵
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    print("\n👋 離開中...")
                    break
                elif key == ord('r') or key == ord('R'):
                    print("\n🔄 重設")
                elif key == ord('+') or key == ord('='):
                    self.max_dist = min(2.0, self.max_dist + DISTANCE_STEP)
                    print(f"\n📏 最大距離: {self.max_dist:.2f}m")
                elif key == ord('-') or key == ord('_'):
                    self.max_dist = max(self.min_dist + 0.1, self.max_dist - DISTANCE_STEP)
                    print(f"\n📏 最大距離: {self.max_dist:.2f}m")
        
        except RuntimeError as e:
            print(f"\n❌ 相機錯誤: {e}")
            print("💡 提示: 請確認相機已連接，並使用 sudo 執行此程式")
            sys.exit(1)
        
        except KeyboardInterrupt:
            print("\n\n⚠️ 使用者中斷")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理資源"""
        self.monitor.report()
        
        # 先關閉所有 OpenCV 視窗
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # 讓視窗關閉生效
        
        if self.pipeline:
            try:
                self.pipeline.stop()
            except:
                pass
        print("✅ 程式結束")


def main():
    app = ObjectSizer()
    app.run()


if __name__ == "__main__":
    main()
