# camera_server.py (Windows 端)
# 功能：傳送影像給 Mac，並接收辨識結果顯示在視窗上

import pyrealsense2 as rs
import socket
import pickle
import struct
import numpy as np
import cv2
import threading
import time

# ========== 設定 ==========
HOST = '0.0.0.0'
PORT = 9999
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
COLOR_WIDTH = 640
COLOR_HEIGHT = 480
FPS = 30
# ==========================

# 全域變數：用來存 Mac 傳回來的辨識結果
current_message = "Waiting for Mac..."
lock = threading.Lock()

def receive_from_mac(conn):
    """獨立執行緒：專門聽 Mac 說話"""
    global current_message
    buffer = ""
    try:
        while True:
            data = conn.recv(1024).decode('utf-8')
            if not data: break
            
            buffer += data
            if '\n' in buffer:
                lines = buffer.split('\n')
                # 取最後一句完整的訊息
                last_msg = lines[-2] 
                buffer = lines[-1]
                
                if last_msg.startswith("DETECTED:"):
                    names = last_msg.split(":")[1]
                    with lock:
                        if names == "None":
                            current_message = "No Face"
                        elif names == "":
                            current_message = "Unknown Face"
                        else:
                            current_message = f"Hi, {names}!"
    except:
        pass

def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, rs.format.bgr8, FPS)
    
    print("正在啟動 RealSense...")
    pipeline.start(config)
    print("✅ RealSense 啟動成功！等待 Mac 連線...")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server.bind((HOST, PORT))
    except OSError:
        print(f"⚠️ Port {PORT} 被佔用，請稍後再試或是更換 Port")
        return

    server.listen(1)
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    print(f"📡 串流伺服器 IP: {local_ip}")

    try:
        while True:
            print("⏳ 等待連線...")
            conn, addr = server.accept()
            print(f"✅ Mac 已連線: {addr}")
            
            # 開一個小精靈去聽 Mac 說話
            t = threading.Thread(target=receive_from_mac, args=(conn,))
            t.daemon = True
            t.start()
            
            try:
                frame_count = 0
                while True:
                    frames = pipeline.wait_for_frames()
                    depth_frame = frames.get_depth_frame()
                    color_frame = frames.get_color_frame()
                    
                    if not depth_frame or not color_frame:
                        continue
                    
                    depth_image = np.asanyarray(depth_frame.get_data())
                    color_image = np.asanyarray(color_frame.get_data())
                    
                    # === 顯示 Mac 傳回來的結果 ===
                    with lock:
                        display_text = current_message
                    
                    # 在畫面下方畫一個黑底白字的條
                    cv2.rectangle(color_image, (0, 440), (640, 480), (0, 0, 0), -1)
                    color = (0, 255, 0) if "Hi" in display_text else (0, 255, 255)
                    cv2.putText(color_image, display_text, (20, 470), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                    
                    cv2.imshow('Windows Preview (With Mac Result)', color_image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        raise KeyboardInterrupt
                    # ==============================
                    
                    _, color_encoded = cv2.imencode('.jpg', color_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    
                    data = pickle.dumps({
                        'depth': depth_image,
                        'color': color_encoded,
                        'frame': frame_count
                    })
                    
                    size = struct.pack('>I', len(data))
                    conn.sendall(size + data)
                    frame_count += 1

            except (ConnectionResetError, BrokenPipeError):
                print(f"⚠️ Mac 斷線，等待重新連線...")
                conn.close()
                cv2.destroyAllWindows()

    except KeyboardInterrupt:
        print("\n⏹️ 停止串流...")
    finally:
        pipeline.stop()
        server.close()
        cv2.destroyAllWindows()
        print("👋 伺服器已關閉")

if __name__ == "__main__":
    main()