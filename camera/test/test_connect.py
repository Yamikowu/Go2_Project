import pyrealsense2 as rs
import time

def test_hardware():
    print("----------------------------------------")
    print("正在搜尋 Intel RealSense D435...")
    print("----------------------------------------")

    try:
        # 1. 建立管線
        pipeline = rs.pipeline()
        config = rs.config()
        
        # 2. 設定要抓取的資料 (距離深度圖)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # 3. 啟動相機
        pipeline.start(config)
        print("✅ 成功連線！(硬體偵測正常)")
        print("正在讀取數據 (按 Ctrl+C 停止)...")
        print("----------------------------------------")

        # 4. 讀取 50 幀數據來測試
        for i in range(50):
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue

            # 測量畫面正中間的距離 (x=320, y=240)
            dist = depth_frame.get_distance(320, 240)
            print(f"第 {i+1:02d} 幀 | 中心點距離: {dist:.3f} 公尺")
            time.sleep(0.1)

        print("----------------------------------------")
        print("🎉 測試成功！你的 D435 是好的，可以拿去做專題了！")

    except RuntimeError as e:
        print(f"❌ 找不到相機: {e}")
        print("-> 請檢查 USB 線是否插緊")
        print("-> 請確認你是插在 Type-C 孔")
    except Exception as e:
        print(f"❌ 發生其他錯誤: {e}")
    finally:
        # 關閉相機，釋放資源
        try:
            pipeline.stop()
        except:
            pass

if __name__ == "__main__":
    test_hardware()