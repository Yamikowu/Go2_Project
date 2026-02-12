---
name: RealSense D435 Camera Development
description: 在各平台開發 Intel RealSense D435 相機 (深度/RGB) 的技巧與限制
https://github.com/realsenseai/librealsense/blob/master/doc/installation_osx.md
---
# Intel RealSense D435 開發指南

## 📚 Resources (詳細參考文件)

| 文件                                                              | 說明                                                |
| ----------------------------------------------------------------- | --------------------------------------------------- |
| [realsense_d435_features.md](./resources/realsense_d435_features.md) | D435 硬體規格、串流功能、濾鏡設定、平台相容性       |
| [gesture_recognition.md](./resources/gesture_recognition.md)         | 手勢辨識技術方案 (MediaPipe, YOLO, Depth Threshold) |

---

## 平台相容性總覽

| 平台                     | RGB 彩色相機 | 深度相機 | Intel 官方支援 | 適合進階開發 |
| ------------------------ | ------------ | -------- | -------------- | ------------ |
| **Windows**        | ✅ 穩定      | ✅ 穩定  | ✅ 完整        | ✅ 最佳      |
| **Linux (Ubuntu)** | ✅ 穩定      | ✅ 穩定  | ✅ 完整        | ✅ 最佳      |
| **Mac M4**         | ⚠️ 不穩定  | ✅ 穩定  | ❌ 只有社群版  | ⚠️ 有限    |

> [IMPORTANT]
> **如果需要 RGB 彩色影像 (人臉辨識、顏色追蹤等)，請使用 Windows 或 Linux 開發。**

---

## Mac M4 開發 (社群版)

### ✅ 成功方案 (The Golden Path)

這是目前在 **Mac mini M4** 上開發 Intel RealSense D435 的唯一穩定解法。

#### 1. 硬體配置

- **電腦**：Mac mini M4 (Apple Silicon)
- **相機**：Intel RealSense D435
- **線材**：**USB4 線** (或高品質 USB 3.0 線)
  - _關鍵：_ 必須直插 Mac 本體，確認系統報告顯示速度為 **5 Gbps (5000 Mb)**，不能是 480 Mb

#### 2. 軟體環境 (VSCode)

- **開發工具**：VSCode
- **環境管理**：Python 虛擬環境 (.venv)
- **必要套件**：
  - **核心驅動**：`pyrealsense2-macosx` (社群修改版，專門支援 Apple Silicon)
  - **影像處理**：`opencv-python`
  - **運算**：`numpy`

```bash
pip install pyrealsense2-macosx opencv-python numpy
```

#### 3. 執行方式 (關鍵權限)

由於 macOS 的安全性限制，VSCode 的終端機預設權限不足以控制 USB 裝置，必須使用 **sudo** 執行。

```bash
sudo <虛擬環境的Python路徑> <你的程式碼路徑>
# 範例：
sudo /Users/yamiko/Documents/VsCode/Go2_Project/.venv/bin/python /Users/yamiko/Documents/VsCode/Go2_Project/camera/show_camera.py
```

#### 4. 程式碼重點

- 不強制指定 BGR8 格式，改用 `rs.format.any`，讓 Mac 自動協調
- 顯示畫面時使用 `cv2.cvtColor(img, cv2.COLOR_RGB2BGR)` 修正顏色
- 加入 `try...finally` 區塊，確保程式結束時能正確關閉相機

---

### ⚠️ Mac M4 已知問題

| 症狀                                                   | 原因                                      |
| ------------------------------------------------------ | ----------------------------------------- |
| 開啟 RGB 彩色串流時報錯：`failed to set power state` | `pyrealsense2-macosx` 對 RGB 支援不完整 |
| 只開啟深度串流則正常                                   | 深度是明確支援的功能                      |

**解法**：只用深度串流

```python
# ✅ 只用深度 — 可行
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# ❌ 加上彩色 — 會導致 power state 錯誤
# config.enable_stream(rs.stream.color, 640, 480, rs.format.any, 30)
```

---

### ❌ 嘗試過但失敗的方法

| 方法                                          | 失敗原因                               | 症狀                               |
| --------------------------------------------- | -------------------------------------- | ---------------------------------- |
| **Intel RealSense Viewer (App)**        | GPU 架構不相容，舊版 OpenGL 與 M4 衝突 | 閃退、Segmentation fault           |
| **Homebrew 安裝驅動**                   | 同上，圖形功能崩潰                     | `realsense-viewer` 接相機後崩潰  |
| **官方 Python 套件** (`pyrealsense2`) | 只有 x86_64 版，沒有 ARM64             | `No matching distribution found` |
| **USB 2.0 線材**                        | 頻寬不足 (480Mbps)                     | `No device connected`            |
| **不加 sudo 執行**                      | 權限不足                               | `failed to set power state`      |
| **在 Mac 上更新韌體**                   | USB 處理機制嚴格，驅動衝突             | 更新失敗、`mutex lock failed`    |

---

## 功能可行性分析

| 功能            | 需要 RGB?   | Mac M4 可行? |
| --------------- | ----------- | ------------ |
| 距離測量 / 避障 | ❌ 只需深度 | ✅ 可以      |
| 人臉辨識        | ✅ 需要     | ❌ 不行      |
| 物體追蹤 (顏色) | ✅ 需要     | ❌ 不行      |
| 3D 點雲         | ❌ 只需深度 | ✅ 可以      |
| SLAM / 建圖     | ❌ 只需深度 | ✅ 可以      |

---

## 🛡️ Mac 上只用深度 (Depth Only) 可以做什麼？

深度資訊在機器人開發中比色彩更重要！RGB 是給人看的，Depth 是給機器看「空間」的。

| 功能                        | 原理                   | 應用               |
| --------------------------- | ---------------------- | ------------------ |
| **自動避障** 🚧       | 偵測前方 0.5m 內障礙物 | 讓機器狗不撞牆     |
| **防跌落偵測** 📉     | 偵測地面高度劇烈變化   | 防止摔下樓梯       |
| **跟隨模式** 🚶‍♂️ | 鎖定最近物體保持距離   | 機器狗跟隨主人     |
| **手勢控制** ✋       | 偵測手的距離/揮動變化  | 推/拉控制機器狗    |
| **3D 建圖** 🗺️      | 深度數據轉點雲         | 掃描房間、導航地圖 |
| **隱私監控** 🕵️     | 只監控變化，無人臉辨識 | 隱私場域監控       |

---

## 開發建議

| 你的情況                     | 建議                                        |
| ---------------------------- | ------------------------------------------- |
| 有 Windows 電腦              | 👉 用 Windows 開發，官方完整支援            |
| 最終部署到 Jetson/樹莓派     | 👉 Mac 先寫深度邏輯，進階功能到 Jetson 再做 |
| 只有 Mac，短期不會用其他設備 | 👉 先用深度做基本避障，等社群版更新         |

---

## 🚀 下一步：連接到 Unitree Go2 Pro (機器狗)

現在「視覺開發」環境已在 Mac 上打通，接下來：

1. **在 Mac 上開發邏輯**：

   - 繼續用 `sudo python` 方式寫演算法（例：偵測紅球、測距小於 0.5m 則停止）
2. **移植到機器狗外掛電腦**：

   - 拿到 **Jetson Orin Nano Super** 後插上相機
   - 複製 Mac 上寫好的 `.py` 檔
   - 在 Linux 上直接用 `pip install pyrealsense2`（官方支援）
   - 通常**不需要** `sudo`，設定一次 USB 規則即可

---

## 🏗️ 專題系統架構

### 團隊設備配置

| 角色                    | 設備                                    | 連接方式   |
| ----------------------- | --------------------------------------- | ---------- |
| **開發者 (你)**   | Mac mini M4 + RealSense D435            | USB 有線   |
| **測試者 (朋友)** | Mac + Go2 Pro                           | Wi-Fi 無線 |
| **未來部署**      | Jetson Orin Nano Super + RealSense D435 | USB 有線   |

### 軟體架構

┌─────────────────────────────────────────────────────────────┐
│ 慢系統 (Cloud / API) │
│ LLM (大腦), VLA(視覺理解) │
└──────────────────────────┬──────────────────────────────────┘
│ 字串 / JSON (Wifi)
▼
┌─────────────────────────────────────────────────────────────┐
│ 快系統 (Jetson Orin Nano) │
│ ┌─────────────────────────────────────────────────────┐ │
│ │ 感知與通訊 (Input/Output) │ │
│ │ • STT (語音轉文字) [API] │ │
│ │ • TTS (文字轉語音) [API] │ │
│ │ • YOLO / COCO (視覺辨識) [Local] │ │
│ │ • Safety Layer (避障/反射) [Local/Depth] │ │
│ │ • SLAM + Nav2 (雷達導航) [Local/LiDAR] │ │
│ └─────────────────────────────────────────────────────┘ │
│ │
│ ROS2 / CycloneDDS (<1ms) │
│ ▼ │
│ Unitree Go2 Pro (MCU) │
│ 執行實際動作 │
└─────────────────────────────────────────────────────────────┘

```
┌─────────────────────────────────────────────────────────────┐
│                   學校雲端 GPU / 外部 API                     │
|    黃（大腦）   楊（視覺理解）                                  |
│  ┌─────────┐  ┌─────────┐                                   │
│  │   LLM   │  │   VLA   │                                   │
│  └────┬────┘  └────┬────┘                                   │
└───────┼────────────┼────────────────────────────────────────┘
        ^            ^
        |            |
        │ Text       │ Image/Prompt
        │            │
        ▼            ▼
┌─────────────────────────────────────────────────────────────┐
│              Jetson Orin Nano Super (現場)                   │
│  ┌───────────────────────┐   ┌───────────────────────────┐  │
│  │   聽說 (Audio)    陳   │   │  視覺 (Vision) 鄔（基本辨識）│  │
│  │  STT (Mic -> Text)    │   │  RealSense -> YOLO (Local)│  │
│  │  TTS (Text -> Spk)    │   │  RealSense -> Depth (Safe)│  │
│  │      (Usage: API)     │   │                           │  │
│  └──────────┬────────────┘   └─────────────┬─────────────┘  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │   導航 (Navigation) - 使用 Go2 內建 LiDAR               │  │
│  │   SLAM (slam_toolbox) + Nav2 [Local]     盧（雷達控制） │  │
│  └───────────────────────────────────────────────────────┘  │
│             │                              │                │
└─────────────┼──────────────────────────────┼────────────────┘
              │ ROS2 / WebRTC                │
              ▼                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   Unitree Go2 Pro                           │
│             (移動控制 / 姿態控制 / 執行端)                      │
└─────────────────────────────────────────────────────────────┘
```

### Go2 ROS2 SDK (非官方)

**GitHub**: https://github.com/abizovnuralem/go2_ros2_sdk

**主要功能**:

| 功能                | 狀態 | 說明              |
| ------------------- | ---- | ----------------- |
| URDF 機器人模型     | ✅   | 用於 RViz 視覺化  |
| 即時關節狀態        | ✅   | Joint states sync |
| IMU 資料            | ✅   | 慣性測量          |
| 搖桿控制            | ✅   | Xbox 手把控制移動 |
| LIDAR 點雲          | ✅   | 內建光達資料      |
| 彩色相機串流        | ✅   | 前置攝影機        |
| SLAM 建圖           | ✅   | slam_toolbox      |
| Nav2 導航           | ✅   | 自主導航          |
| COCO 物件偵測       | ✅   | 人/動物辨識       |
| WebRTC 無線連接     | ✅   | Wi-Fi 控制        |
| CycloneDDS 有線連接 | ✅   | 乙太網路控制      |
| 多機器人支援        | ✅   | 多狗協作          |

**連接方式**:

```bash
# Wi-Fi 模式
export ROBOT_IP="192.168.x.x"
export CONN_TYPE="webrtc"
ros2 launch go2_robot_sdk robot.launch.py

# 乙太網路模式
export CONN_TYPE="cyclonedds"
```

**WebRTC 指令發送**:

```bash
ros2 topic pub /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{api_id: 1016, topic: 'rt/api/sport/request'}" --once
```

### AI 視覺與導航架構 (Vision & Navigation Architecture)

這是專題的核心邏輯，分為快系統與慢系統：

| 層級                | 技術               | 回答的問題                   | 部署位置      | 特性                                                        |
| ------------------- | ------------------ | ---------------------------- | ------------- | ----------------------------------------------------------- |
| **1. 感知層** | RealSense 深度     | 「前面多遠？會撞到嗎？」     | Jetson (本地) | **<50ms (即時)**`<br>`不需光線、絕對距離            |
| **2. 辨識層** | YOLO / COCO        | 「前面是什麼物體？」         | Jetson (本地) | **~30ms (即時)**`<br>`標準物件分類 (人、椅、貓)     |
| **3. 導航層** | SLAM + Nav2        | 「我在哪裡？要怎麼走過去？」 | Jetson (本地) | **~100ms** `<br>`使用 Go2 內建 LiDAR 建圖與路徑規劃 |
| **4. 理解層** | VLM (視覺語言模型) | 「這個場景代表什麼意思？」   | 雲端 GPU      | **~1-2s (慢)**`<br>`情境推理、開放詞彙、異常偵測    |

### 未來 AI 整合計畫

| 模組                   | 部署位置                 | 功能       | 負責內容                                     |
| ---------------------- | ------------------------ | ---------- | -------------------------------------------- |
| **LLM**          | 學校雲端 / API           | 大腦       | 負責思考、對話邏輯、多模態決策               |
| **VLM**          | 學校雲端 / API           | 眼睛(理解) | 複雜場景分析 (例如: "這裡發生什麼事?")       |
| **TTS**          | **Jetson (API)**   | 嘴巴       | 呼叫 API 將文字轉語音 (OpenAI/Edge-TTS)      |
| **STT**          | **Jetson (API)**   | 耳朵       | 呼叫 API 將語音轉文字 (Whisper)              |
| **YOLO**         | **Jetson (Local)** | 眼睛(辨識) | 快速辨識物體 (人、球、指示牌)                |
| **SLAM + Nav2**  | **Jetson (Local)** | 腳 (導航)  | 使用 Go2 LiDAR 建圖、路徑規劃、自主移動      |
| **Safety Layer** | **Jetson (Local)** | 反射神經   | **RealSense 避障模組**、防撞、緊急停止 |

### 開發中的模組

#### 避障模組 (Safety Layer)

- **檔案**: `camera/modules/obstacle_json.py`
- **功能**: 接收距離數據，輸出 JSON 格式指令 (相容於 MCP/YOLO 格式)
- **定位**: 這是 Safety Layer 的一部分，確保在 LLM 思考或 VLM 理解時，狗不會撞牆。

```json
{
  "success": true,
  "obstacle_detected": true,
  "distance_m": 0.35,
  "direction": "前方",
  "cmd_vel": { "linear_x": -0.3, "angular_z": 0 },
  "action": "BACK",
  "message": "⚠️ 障礙物在前方，距離 0.35 公尺，後退中"
}
```

---

## 🛠️ Demo 功能展示與比較 (Demo Showcase)

以下是目前已開發完成的 RealSense D435 功能模組，可直接在 Mac M4 上執行：

| 腳本名稱                         | 功能描述                      | 關鍵技術                        | 應用場景                       |
| :------------------------------- | :---------------------------- | :------------------------------ | :----------------------------- |
| **`obstacle.py`**        | **原力控制** (基礎避障) | Depth Threshold, Center ROI     | 測試相機距離感測、除錯         |
| **`smart_avoid.py`**     | **智慧避障** (多區偵測) | Multi-zone Depth, Logic         | 機器狗的「反射神經」，防止撞牆 |
| **`follow_person.py`**   | **人員跟隨**            | Depth Blob Tracking             | 讓狗跟著走、保持固定距離       |
| **`face_rec_remote.py`** | **遠端人臉辨識**        | RGB Stream, InsightFace, Socket | 辨識主人、防盜、個人化互動     |
| **`object_sizer.py`**    | **物件量測**            | Depth Contour, Intrinsics       | 測量包裹尺寸、估算物體大小     |
| **`pointcloud.py`**      | **3D 點雲掃描**         | Deprojection (2D->3D), Open3D   | 建立房間 3D 模型、SLAM 基礎    |
| **`night_vision.py`**    | **夜視模式**            | IR Stream, Emitter Control      | 在全黑環境下導航、監控         |

---

## 📁 專案檔案結構

```
Go2_Project/
├── camera/
│   ├── modules/
│   │   └── obstacle_json.py   # MCP 相容避障模組 (JSON 輸出)
│   ├── demo/
│   │   ├── obstacle.py        # 避障/原力控制模擬器
│   │   ├── smart_avoid.py     # 智慧避障 (多區偵測)
│   │   ├── follow_person.py   # 人員跟隨模擬器
│   │   ├── face_rec_remote.py # 遠端人臉辨識 (需搭配 Windows)
│   │   ├── object_sizer.py    # 物件體積測量器
│   │   ├── pointcloud.py      # 3D 點雲掃描器
│   │   └── night_vision.py    # 夜視模式 Demo
│   ├── test/
│   │   └── test_depth.py      # 深度相機測試
│   └── show_camera.py         # RGB+Depth 顯示 (Linux 用)
├── report/
│   └── integration_and_features.md # 整合策略與新功能建議 (New!)
├── docs/
│   ├── obstacle.md            # 避障模擬器說明
│   └── pointcloud.md          # 點雲掃描器說明
└── .agent/skills/Go2/
    ├── SKILL.md               # 本文件 (主入口)
    └── resources/
        ├── realsense_d435_features.md  # D435 硬體功能詳細參考
        └── gesture_recognition.md      # 手勢辨識技術方案
```
