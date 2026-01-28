import pyrealsense2 as rs
import numpy as np
import cv2
import time
import sys

def show_stream_robust():
    print("🔄 正在初始化相機系統...")

    # 1. 嘗試強制重置相機 (防止上次沒關好)
    try:
        ctx = rs.context()
        if len(ctx.query_devices()) > 0:
            dev = ctx.query_devices()[0]
            print(f"-> 偵測到裝置: {dev.get_info(rs.camera_info.name)}")
            # 如果是卡住的狀態，這個重置或許能救回來
            # dev.hardware_reset() 
            # print("-> 已發送重置訊號 (若等下失敗，請拔線重插)")
        else:
            print("❌ 電腦找不到相機！請拔掉 USB 線，等 10 秒再插回去！")
            return
    except:
        pass

    pipeline = rs.pipeline()
    config = rs.config()
    
    # 使用自動格式 (RGB 在 Mac M4 上可能不穩定，如遇問題請註解掉 color 那行)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.any, 30)

    is_streaming = False

    try:
        print("🎥 正在啟動影像串流...")
        pipeline.start(config)
        is_streaming = True
        print("✅ 影像啟動成功！")
        print("🔴 操作說明：")
        print("   1. 請點選跳出來的「畫面視窗」讓它變成作用中")
        print("   2. 按 'q' 鍵離開")
        print("   3. 或者在終端機按 Ctrl+C 也能安全離開")

        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # 轉檔
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # 確保顏色正確 (RGB -> BGR)
            if len(color_image.shape) == 3 and color_image.shape[2] == 3:
                # 簡單判斷，通常 mac 讀進來是 RGB，OpenCV 需要 BGR
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            # 深度圖上色
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # 顯示
            cv2.imshow('RealSense (Click here and press q to exit)', color_image)
            cv2.imshow('Depth', depth_colormap)

            # 偵測按鍵 'q'
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                print("👋 偵測到 'q' 鍵，正在關閉...")
                break

    except KeyboardInterrupt:
        print("\n⚠️ 偵測到 Ctrl+C，正在強制停止相機...")
    
    except RuntimeError as e:
        print(f"\n❌ 發生相機錯誤: {e}")
        print("💡 建議：拔掉 USB 線，重新開機 Mac。")

    finally:
        # 無論如何都會執行這裡，確保相機被關閉
        if is_streaming:
            try:
                pipeline.stop()
                print("✅ 相機已安全關閉。")
            except:
                print("⚠️ 相機關閉時發生輕微異常 (不影響下次使用)")
        
        cv2.destroyAllWindows()
        print("程式結束。")

if __name__ == "__main__":
    show_stream_robust()