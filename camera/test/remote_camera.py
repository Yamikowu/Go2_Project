# remote_camera.py - Mac 遠端相機接收端
# 用法：python3 /Users/yamiko/Documents/VsCode/Go2_Project/camera/remote_camera.py

import socket
import pickle
import struct
import cv2
import numpy as np

# ========== 設定 ==========
WINDOWS_IP = "192.168.0.14"    # TODO: 改成 Windows 的 IP 
PORT = 9999
# ==========================

def recv_exact(sock, size):
    """確保收到完整資料"""
    data = b''
    while len(data) < size:
        packet = sock.recv(min(size - len(data), 65536))
        if not packet:
            return None
        data += packet
    return data

def main():
    print(f"正在連線到 Windows ({WINDOWS_IP}:{PORT})...")

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((WINDOWS_IP, PORT))
    print("✅ 已連線到 Windows 相機伺服器！")
    print("按 'q' 結束")

    try:
        while True:
            # 接收資料大小 (4 bytes)
            raw_size = recv_exact(client, 4)
            if not raw_size:
                print("⚠️ 連線中斷")
                break
            size = struct.unpack('>I', raw_size)[0]

            # 接收實際資料
            data = recv_exact(client, size)
            if not data:
                break

            frames = pickle.loads(data)

            # 解壓縮彩色影像
            color_image = cv2.imdecode(frames['color'], cv2.IMREAD_COLOR)
            depth_image = frames['depth']

            # 深度轉彩色顯示
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )

            # 合併顯示
            combined = np.hstack((color_image, depth_colormap))
            cv2.imshow('Remote RealSense (Windows -> Mac)', combined)

            # 按 q 結束
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        client.close()
        cv2.destroyAllWindows()
        print("👋 已斷線")

if __name__ == "__main__":
    main()
