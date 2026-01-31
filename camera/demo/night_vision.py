#!/usr/bin/env python3
"""
🌑 Night Vision Mode - RealSense D435

展示 RealSense D435 的夜視能力：
- 雙紅外線鏡頭並排顯示
- 即時控制 Emitter (雷射發射器)
- 在完全黑暗環境下測試

使用方式:
sudo /Users/yamiko/Documents/VsCode/Go2_Project/.venv/bin/python /Users/yamiko/Documents/VsCode/Go2_Project/camera/demo/night_vision.py

控制鍵:
- E: 開關 Emitter
- +: 增加功率
- -: 減少功率
- Q: 離開
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import sys

# ===== 參數設定 =====
IR_WIDTH = 640
IR_HEIGHT = 480
FPS = 30
DEFAULT_POWER = 150  # 預設功率 (0-360 範圍)
POWER_STEP = 30      # 每次調整的功率增量

class NightVisionMode:
    def __init__(self):
        self.pipeline = None
        self.depth_sensor = None
        self.emitter_enabled = True
        self.laser_power = DEFAULT_POWER
        
    def setup_camera(self):
        """初始化相機與雙 IR 串流"""
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # 開啟雙紅外線鏡頭
        config.enable_stream(rs.stream.infrared, 1, IR_WIDTH, IR_HEIGHT, rs.format.y8, FPS)
        config.enable_stream(rs.stream.infrared, 2, IR_WIDTH, IR_HEIGHT, rs.format.y8, FPS)
        
        print("🎥 啟動雙紅外線串流...")
        profile = self.pipeline.start(config)
        
        # 取得深度感測器以控制 Emitter
        device = profile.get_device()
        self.depth_sensor = device.first_depth_sensor()
        
        # 設定初始功率
        try:
            self.depth_sensor.set_option(rs.option.emitter_enabled, 1)
            self.depth_sensor.set_option(rs.option.laser_power, self.laser_power)
            print(f"✅ Emitter 已啟動，功率: {self.laser_power}")
        except Exception as e:
            print(f"⚠️ 無法控制 Emitter: {e}")
            self.depth_sensor = None
        
        # 跳過前幾幀讓相機穩定
        print("⏳ 相機預熱中...")
        for _ in range(30):
            self.pipeline.wait_for_frames()
        
        print("✅ 相機準備完成！")
    
    def toggle_emitter(self):
        """切換 Emitter 開關"""
        if not self.depth_sensor:
            print("⚠️ Emitter 控制不可用")
            return
        
        self.emitter_enabled = not self.emitter_enabled
        try:
            self.depth_sensor.set_option(rs.option.emitter_enabled, 1 if self.emitter_enabled else 0)
            status = "開啟" if self.emitter_enabled else "關閉"
            print(f"\n🔦 Emitter {status}")
        except Exception as e:
            print(f"\n❌ 切換失敗: {e}")
    
    def adjust_power(self, delta):
        """調整 Emitter 功率"""
        if not self.depth_sensor or not self.emitter_enabled:
            return
        
        self.laser_power = max(0, min(360, self.laser_power + delta))
        try:
            self.depth_sensor.set_option(rs.option.laser_power, self.laser_power)
            print(f"\n⚡ 功率: {self.laser_power} ({self.laser_power/360*100:.0f}%)")
        except Exception as e:
            print(f"\n❌ 調整失敗: {e}")
    
    def draw_ui(self, ir1_image, ir2_image):
        """繪製雙畫面 UI"""
        h, w = ir1_image.shape
        
        # 轉換成彩色格式以便繪製彩色文字
        ir1_color = cv2.cvtColor(ir1_image, cv2.COLOR_GRAY2BGR)
        ir2_color = cv2.cvtColor(ir2_image, cv2.COLOR_GRAY2BGR)
        
        # 在左側畫面標記
        cv2.putText(ir1_color, "IR Camera 1 (Left)", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 在右側畫面標記
        cv2.putText(ir2_color, "IR Camera 2 (Right)", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 左右並排
        combined = np.hstack((ir1_color, ir2_color))
        
        # 底部資訊區
        info_height = 80
        info_panel = np.zeros((info_height, combined.shape[1], 3), dtype=np.uint8)
        
        # Emitter 狀態
        if self.emitter_enabled:
            status_text = "Emitter: ON"
            status_color = (0, 255, 0)  # 綠色
            power_ratio = self.laser_power / 360.0
            power_text = f"Power: {int(power_ratio * 100)}%"
        else:
            status_text = "Emitter: OFF"
            status_color = (0, 0, 255)  # 紅色
            power_ratio = 0
            power_text = "Power: ---"
        
        cv2.putText(info_panel, status_text, (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        cv2.putText(info_panel, power_text, (250, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # 功率條
        if self.emitter_enabled:
            bar_x = 250
            bar_y = 45
            bar_w = 200
            bar_h = 20
            # 背景
            cv2.rectangle(info_panel, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (50, 50, 50), -1)
            # 填充
            fill_w = int(bar_w * power_ratio)
            cv2.rectangle(info_panel, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h), (0, 255, 0), -1)
        
        # 控制說明
        controls = "[E] Toggle  [+/-] Power  [Q] Quit"
        cv2.putText(info_panel, controls, (combined.shape[1] - 400, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        # 標題
        title_panel = np.zeros((40, combined.shape[1], 3), dtype=np.uint8)
        cv2.putText(title_panel, "Night Vision Mode - RealSense D435", 
                    (combined.shape[1]//2 - 250, 28), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        
        # 組合所有部分
        final = np.vstack((title_panel, combined, info_panel))
        
        return final
    
    def run(self):
        """主迴圈"""
        print("\n" + "=" * 60)
        print("🌑 夜視模式啟動")
        print("=" * 60)
        print("📖 控制說明:")
        print("   E  : 開關 Emitter")
        print("   +  : 增加功率")
        print("   -  : 減少功率")
        print("   Q  : 離開")
        print("=" * 60)
        print("\n💡 提示: 關燈後可以看到紅外線的威力！")
        print("        試著用手遮住 Emitter (最左邊的孔) 看看差異\n")
        
        try:
            self.setup_camera()
            
            while True:
                frames = self.pipeline.wait_for_frames()
                
                # 取得雙 IR 畫面
                ir1_frame = frames.get_infrared_frame(1)
                ir2_frame = frames.get_infrared_frame(2)
                
                if not ir1_frame or not ir2_frame:
                    continue
                
                # 轉換成 numpy array
                ir1_image = np.asanyarray(ir1_frame.get_data())
                ir2_image = np.asanyarray(ir2_frame.get_data())
                
                # 繪製 UI
                display = self.draw_ui(ir1_image, ir2_image)
                
                # 顯示
                cv2.imshow('Night Vision Mode (Press Q to Exit)', display)
                
                # 處理按鍵
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    print("\n👋 離開中...")
                    break
                elif key == ord('e') or key == ord('E'):
                    self.toggle_emitter()
                elif key == ord('+') or key == ord('='):
                    self.adjust_power(POWER_STEP)
                elif key == ord('-') or key == ord('_'):
                    self.adjust_power(-POWER_STEP)
        
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
        if self.pipeline:
            try:
                self.pipeline.stop()
            except:
                pass
        cv2.destroyAllWindows()
        print("✅ 程式結束")

def main():
    app = NightVisionMode()
    app.run()

if __name__ == "__main__":
    main()
