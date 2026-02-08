# Windows 遠端相機開發指南

> **目的**：借用 Windows 電腦跑 RealSense D435，Mac 遠端接收影像開發
> **建立日期**：2026-02-03

---

## ✅ Phase 1：測試 Windows 能不能跑相機 (已完成)

> **測試日期**：2026-02-03
> **測試結果**：✅ 通過！Depth 和 RGB 都正常
> **安裝版本**：Intel RealSense SDK (非 Beta 版)

### 最簡單的測試方式：使用官方 Viewer

1. **下載 Intel RealSense Viewer**

   - 網址：https://github.com/IntelRealSense/librealsense/releases
   - 找到最新版本，下載 `Intel.RealSense.Viewer.exe` (Windows installer)
   - 或者下載 `.msi` 安裝檔
2. **安裝並執行**

   - 安裝過程直接下一步即可
   - 插上 RealSense D435 (用 USB 3.0 孔，藍色的那個)
3. **確認相機正常**

   - 開啟 Intel RealSense Viewer
   - 左邊應該會看到 D435 裝置
   - 點擊 Depth 和 RGB 的開關，確認兩個都能顯示畫面
   - ✅ 如果都能顯示 = Windows 可以跑相機！
4. **移除 (測試完畢後)**

   - 控制台 → 程式和功能 → Intel RealSense SDK 2.0 → 解除安裝

---

## 📊 可開發的功能分類

### ✅ 純 RGB 就夠的功能

| 功能                      | 說明               | 技術                    |
| ------------------------- | ------------------ | ----------------------- |
| **人臉辨識**        | 辨認是誰           | face_recognition / dlib |
| **人臉偵測**        | 偵測有沒有人臉     | OpenCV / MediaPipe      |
| **表情辨識**        | 開心/難過/生氣     | FER / DeepFace          |
| **物體辨識 (YOLO)** | 這是人、狗、杯子   | YOLOv8 / COCO           |
| **顏色追蹤**        | 追紅球、找藍色物體 | OpenCV HSV              |
| **OCR 文字辨識**    | 讀招牌、標籤       | Tesseract / EasyOCR     |

### 🔀 需要 RGB + Depth 一起用的功能

| 功能                          | 為什麼需要兩個？                             | 重要程度  |
| ----------------------------- | -------------------------------------------- | --------- |
| **人物追蹤 + 保持距離** | RGB 鎖定「特定的人」，Depth 控制「距離多遠」 | ⭐⭐⭐ 高 |
| **手勢辨識 (進階)**     | RGB 辨認手的形狀，Depth 判斷距離/推拉        | ⭐⭐ 中   |
| **物體尺寸測量**        | RGB 框出物體，Depth 算實際大小               | ⭐ 低     |
| **3D 人體追蹤**         | 結合骨架辨識 + 深度                          | ⭐ 低     |

### 💡 開發策略

1. **先開發純 RGB 功能** (人臉辨識)
2. **之後整合 Depth** (跟隨主人時保持距離)
3. **部署到 Jetson** (RGB + Depth 都完整支援)

---

## 📋 Phase 2：設定遠端串流環境 (測試通過後再做)

### 方案 A：使用 WinPython (推薦，不污染系統)

#### Step 1：下載 WinPython

- 網址：https://winpython.github.io/
- 選擇 Python 3.10 或 3.11 版本
- 下載後解壓縮到桌面或隨身碟

#### Step 2：開啟 WinPython 命令列

- 進入 WinPython 資料夾
- 雙擊 `WinPython Command Prompt.exe`

#### Step 3：安裝必要套件

```bash
pip install pyrealsense2 opencv-python numpy
```

#### Step 4：建立串流資料夾

```
WinPython資料夾/
└── camera_server/
    └── camera_server.py    # 從 Mac 複製過來
```

---

### 方案 B：正常安裝 Python (需要之後清除)

#### Step 1：安裝 Python

- 網址：https://www.python.org/downloads/
- 下載 Python 3.10+ 的 Windows installer
- **重要**：安裝時勾選 "Add Python to PATH"

#### Step 2：安裝套件 (在 cmd 或 PowerShell)

```bash
pip install pyrealsense2 opencv-python numpy
```

#### Step 3：建立工作資料夾

```
C:\Users\使用者\Desktop\camera_server\
└── camera_server.py
```

---

## 📂 camera_server.py (串流伺服器程式碼)

這個檔案放在 Windows 上執行，負責把相機畫面透過網路傳到 Mac。

```python
# camera_server.py - Windows RealSense 串流伺服器
# 用法：python camera_server.py

import pyrealsense2 as rs
import socket
import pickle
import struct
import numpy as np
import cv2

# ========== 設定 ==========
HOST = '0.0.0.0'      # 監聽所有網卡
PORT = 9999           # 串流 port
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
COLOR_WIDTH = 640
COLOR_HEIGHT = 480
FPS = 30
# ==========================

def main():
    # 設定 RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, rs.format.bgr8, FPS)

    print("正在啟動 RealSense...")
    pipeline.start(config)
    print("✅ RealSense 啟動成功！")

    # 建立 TCP Server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    # 顯示本機 IP
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    print(f"\n📡 串流伺服器已啟動！")
    print(f"   IP: {local_ip}")
    print(f"   Port: {PORT}")
    print(f"\n⏳ 等待 Mac 連線...")

    try:
        while True:
            conn, addr = server.accept()
            print(f"✅ Mac 已連線: {addr}")

            try:
                frame_count = 0
                while True:
                    frames = pipeline.wait_for_frames()
                    depth_frame = frames.get_depth_frame()
                    color_frame = frames.get_color_frame()

                    if not depth_frame or not color_frame:
                        continue

                    # 轉成 numpy array
                    depth_image = np.asanyarray(depth_frame.get_data())
                    color_image = np.asanyarray(color_frame.get_data())

                    # 壓縮彩色影像 (減少頻寬)
                    _, color_encoded = cv2.imencode('.jpg', color_image, [cv2.IMWRITE_JPEG_QUALITY, 80])

                    # 打包資料
                    data = pickle.dumps({
                        'depth': depth_image,
                        'color': color_encoded,
                        'frame': frame_count
                    })

                    # 送出
                    size = struct.pack('>I', len(data))
                    conn.sendall(size + data)

                    frame_count += 1
                    if frame_count % 100 == 0:
                        print(f"   已傳送 {frame_count} 幀...")

            except (ConnectionResetError, BrokenPipeError):
                print(f"⚠️ Mac 斷線，等待重新連線...")
                conn.close()

    except KeyboardInterrupt:
        print("\n⏹️ 停止串流...")
    finally:
        pipeline.stop()
        server.close()
        print("👋 伺服器已關閉")

if __name__ == "__main__":
    main()
```

---

## 📂 remote_camera.py (Mac 接收端)

這個檔案放在 Mac 上執行，接收 Windows 串流過來的影像。

```python
# remote_camera.py - Mac 遠端相機接收端
# 用法：python remote_camera.py

import socket
import pickle
import struct
import cv2
import numpy as np

# ========== 設定 ==========
WINDOWS_IP = "192.168.x.x"    # TODO: 改成 Windows 的 IP
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
```

---

## 🧹 完整清除步驟 (還電腦時)

### 如果用 WinPython：

```
直接刪除整個 WinPython 資料夾 → 完成！
```

### 如果用正常安裝：

```powershell
# 1. 移除 pip 套件
pip uninstall pyrealsense2 opencv-python numpy -y

# 2. 清除 pip 快取
pip cache purge

# 3. 解除安裝 Python
#    控制台 → 程式和功能 → Python 3.x → 解除安裝

# 4. 解除安裝 Intel RealSense (如果有裝 Viewer)
#    控制台 → 程式和功能 → Intel RealSense SDK → 解除安裝

# 5. 刪除工作資料夾
#    刪除 C:\Users\使用者\Desktop\camera_server\
```

---

## 📝 測試 Checklist

- [X] Phase 1：Windows 能開啟 Intel RealSense Viewer ✅ 2026-02-03
- [X] Phase 1：Viewer 中 Depth 畫面正常 ✅
- [X] Phase 1：Viewer 中 RGB 畫面正常 ✅
- [X] Phase 2：WinPython 安裝完成
- [X] Phase 2：pip 套件安裝成功
- [X] Phase 2：camera_server.py 可以執行
- [X] Phase 2：Mac 可以連線並收到影像

---

## ❓ 常見問題

### Q: RGB 畫面是黑的？

- 確認環境有光
- 在 Viewer 裡手動調整曝光
- 切換2D, 3D(2D v)

### Q: Mac 連不上 Windows？

- 確認兩台電腦在同一個 Wi-Fi / 網路
- Windows 防火牆可能需要允許 Python 通過
- 在 Windows cmd 執行 `ipconfig` 確認 IP
