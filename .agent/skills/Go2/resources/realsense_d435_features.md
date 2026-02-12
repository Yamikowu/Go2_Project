# Intel RealSense D435 完整功能參考

本文件記錄 Intel RealSense D435 深度相機的所有可用功能，包含基礎功能與進階/隱藏功能。

---

## 目錄

1. [硬體規格總覽](#硬體規格總覽)
2. [影像串流功能](#影像串流功能)
3. [硬體控制功能](#硬體控制功能)
4. [影像處理功能](#影像處理功能)
5. [手勢辨識應用](#手勢辨識應用)
6. [平台相容性](#平台相容性)
7. [應用場景](#應用場景)

---

## 大綱

1. **影像輸出 (看到什麼？)**
   RGB 彩色圖：一般相機畫面 (1080p)，用於辨識顏色、人臉。
   Depth 深度圖：地形圖般的畫面，每個像素都有精準的「距離數據」。
   IR 紅外線圖：雙目黑白畫面，用於夜視或在全黑環境下看清物體。
   RGB-D 對齊：將「彩色」與「深度」畫面無損疊加，知道畫面上代表「誰」的物體距離「多遠」。
2. **核心硬體技術 (強在哪？)**
   Global Shutter (全域快門)：機器狗快速轉頭時，深度畫面不會變形。
   Active IR (主動雷射)：內建發射器，即使是對著「白牆」也能測出距離。
   Hardware Sync (硬體同步)：支援多台相機同時觸發，不互相干擾。
3. **內建影像優化 (怎麼變漂亮？)**
   **視覺預設 (Presets)：**
   High Density：填滿所有黑洞（適合避障）。
   High Accuracy：追求極致精準（適合 3D 掃描）。
   Hand：針對近距離手部輪廓優化（適合手勢）。
   **後處理濾鏡 (Filters)：**
   空間濾鏡：美肌磨皮（平滑深度圖邊緣）。
   時間濾鏡：消除閃爍（讓畫面更穩定）。
   補洞濾鏡：強行把測不到的黑點填滿。
4. **機器人應用實務**
   避障/跌落監測：利用深度圖判斷前方是否有障礙物或樓梯。
   3D 點雲建圖：將空間轉為 3D 模型。
   手勢互動：利用「虛擬空間觸發器」（推/拉/揮手）。

---

## 硬體規格總覽

Intel RealSense D435 的正面外觀看起來有 **4 個小圓孔** ，但精確來說，它包含了 **3 顆鏡頭** 和 **1 個發射器** 。

從正面看（面對相機），由左至右分別是：

```
相機正面 (面對你的視角)
┌─────────────────────────────────────┐
│   ●      ◉      ●      ○           │
│  中孔    大孔    中孔    小孔        │
│ (左IR)  (Emitter) (右IR)  (RGB)     │
└─────────────────────────────────────┘
```

### 1. 左紅外線鏡頭 (Infrared Camera 1) — _最左邊 (中孔)_

- **性質** ：全域快門 (Global Shutter) 黑白鏡頭。
- **功能** ：捕捉左側視角的紅外線影像。
- **作用** ：與右鏡頭配合計算「視差」，它是產生 **Depth（深度圖）** 的關鍵核心。

### 2. 紅外線投影器 (IR Emitter) — _左二 (大孔)_

- **性質** ：它不是鏡頭，而是一個「雷射投影機」。
- **功能** ：發射出人類肉眼看不見的「散斑圖案 (Speckle Pattern)」。
- **作用** ：當相機對著「白牆」或「全黑環境」時，它會把圖案打在物體上，讓旁邊的紅外線鏡頭能抓到特徵點，進而計算出距離。
- **測試方式** ：遮住這個孔，IR 畫面會明顯變暗！

### 3. 右紅外線鏡頭 (Infrared Camera 2) — _右二 (中孔)_

- **性質** ：全域快門 (Global Shutter) 黑白鏡頭。
- **功能** ：捕捉右側視角的紅外線影像。
- **作用** ：與左鏡頭一起構成「立體視覺 (Stereo Vision)」。因為 D435 是雙目設計，所以這兩顆鏡頭必須同時運算。

### 4. 彩色鏡頭 (RGB Camera) — _最右邊 (小孔)_

- **性質** ：滾動快門 (Rolling Shutter) 彩色鏡頭。
- **功能** ：捕捉一般的彩色影像 (1080p)。
- **作用** ：用於人臉辨識、顏色偵測，或是與深度圖對齊後產生「彩色點雲」。

| 項目                   | 規格                                        |
| ---------------------- | ------------------------------------------- |
| **深度技術**     | 主動式紅外線立體視覺 (Active IR Stereo)     |
| **深度解析度**   | 最高 1280 x 720 @ 90fps                     |
| **彩色解析度**   | 最高 1920 x 1080 @ 30fps                    |
| **深度範圍**     | 0.1m ~ 10m (室內最佳 0.3m ~ 3m)             |
| **視野角 (FOV)** | 深度: 87° x 58°, 彩色: 69° x 42°        |
| **快門類型**     | 深度: Global Shutter, 彩色: Rolling Shutter |
| **連接介面**     | USB 3.0 Type-C (建議 USB 3.1 Gen 1 以上)    |

---

## 影像串流功能

### 1. 🎨 RGB 彩色相機 (Color Stream)

標準的彩色影像串流，用於一般電腦視覺任務。

| 屬性               | 說明                                  |
| ------------------ | ------------------------------------- |
| **串流類型** | `rs.stream.color`                   |
| **格式**     | RGB8, BGR8, YUYV, MJPEG 等            |
| **解析度**   | 1920x1080, 1280x720, 960x540, 640x480 |
| **幀率**     | 6, 15, 30 fps (依解析度而定)          |

**用途**：

- 人臉辨識
- 物體顏色追蹤
- 一般攝影/錄影
- YOLO / COCO 物件偵測的輸入來源

```python
# 開啟彩色串流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
```

---

### 2. 📏 深度相機 (Depth Stream)

透過雙紅外線鏡頭 + 雷射投影計算每個像素的距離。

| 屬性               | 說明                                |
| ------------------ | ----------------------------------- |
| **串流類型** | `rs.stream.depth`                 |
| **格式**     | Z16 (16-bit 整數，單位: 公釐)       |
| **解析度**   | 1280x720, 848x480, 640x480, 480x270 |
| **幀率**     | 6, 15, 30, 60, 90 fps               |

**用途**：

- 障礙物偵測與避障
- 距離測量
- 3D 點雲建構
- SLAM 建圖與導航

```python
# 開啟深度串流
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 取得特定點的距離 (單位: 公尺)
depth_frame = frames.get_depth_frame()
distance = depth_frame.get_distance(x, y)
```

---

### 3. 🟥 紅外線雙目相機 (Infrared Streams)

D435 內建兩個紅外線鏡頭（左眼 / 右眼），這是深度計算的基礎，但也可以獨立使用。

| 屬性               | 說明                                  |
| ------------------ | ------------------------------------- |
| **串流類型** | `rs.stream.infrared` (index 1 或 2) |
| **格式**     | Y8 (8-bit 灰階)                       |
| **解析度**   | 與深度相同                            |
| **快門類型** | **Global Shutter** ⭐           |

**核心優勢**：

| 特性                 | 說明                                     |
| -------------------- | ---------------------------------------- |
| **夜視能力**   | 搭配 Emitter，在全黑環境也能清楚看到物體 |
| **抗光干擾**   | 不受室內燈光顏色影響，比 RGB 穩定        |
| **無動態模糊** | Global Shutter 避免快速移動時的果凍效應  |
| **VSLAM 首選** | 視覺定位系統偏好使用 IR 而非 RGB         |

```python
# 開啟紅外線串流 (左眼)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)

# 開啟紅外線串流 (右眼)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
```

**夜視模式應用**：

```python
# 夜間避障：只用 IR + Depth，不需要 RGB
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
```

---

### 4. 📐 RGB-D 對齊 (RGB + Depth Alignment)

D435 有兩組鏡頭（RGB 和 IR/Depth），它們的位置不同，所以看到的視野 (FOV) 也不一樣。如果直接將深度圖疊在彩色圖上，會發現邊緣對不準。**Alignment (對齊)** 功能可以解決這個問題。

| 功能                | 說明                                            |
| ------------------- | ----------------------------------------------- |
| **原本狀態**  | Depth 鏡頭解析度/視角 與 RGB 鏡頭不同 (有視差)  |
| **Alignment** | 將深度圖的每個像素，投影到 RGB 攝影機的座標系上 |
| **效果**      | 深度圖會被裁切並縮放，與 RGB 畫面像素點對點重合 |

**為什麼需要對齊？**

- **知道「誰」在「多遠」**：當 YOLO 在 RGB 畫面上框出一個人時，你需要查詢深度圖對應位置的距離。如果沒對齊，你會查到背景的距離。
- **彩色點雲 (Color Point Cloud)**：3D 掃描需要幫每個空間點上色。

```python
# 建立對齊物件：將其他串流對齊到 Color 串流
align = rs.align(rs.stream.color)

# 在迴圈中處理
frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)

# 取得對齊後的深度與彩色圖
depth_frame_aligned = aligned_frames.get_depth_frame()
color_frame = aligned_frames.get_color_frame()
```

---

## 硬體控制功能

### 5. 🔦 Emitter 雷射結構光發射器

相機正面左側的紅外線投影機，發射人眼不可見的「散斑圖案 (Speckle Pattern)」。

| 屬性           | 說明                           |
| -------------- | ------------------------------ |
| **位置** | 相機正面左二大孔（最大的那個） |
| **波長** | 850nm 近紅外線                 |
| **功能** | 在無紋理表面 (白牆) 也能測距   |

**可調整參數**：

| 參數                    | 說明                     | API                            |
| ----------------------- | ------------------------ | ------------------------------ |
| **開關 (On/Off)** | 關閉以節省電力或減少干擾 | `RS2_OPTION_EMITTER_ENABLED` |
| **功率 (Power)**  | 調整雷射強度 (0.0 ~ 1.0) | `RS2_OPTION_LASER_POWER`     |
| **頻閃模式**      | 交替開關以支援多機同步   | `RS2_OPTION_EMITTER_ON_OFF`  |

```python
# 取得深度感測器
depth_sensor = profile.get_device().first_depth_sensor()

# 關閉雷射發射器
depth_sensor.set_option(rs.option.emitter_enabled, 0)

# 調整雷射功率 (0-360)
depth_sensor.set_option(rs.option.laser_power, 150)
```

**使用建議**：

| 場景              | 建議設定                   |
| ----------------- | -------------------------- |
| 白牆 / 無紋理表面 | 開啟 Emitter，功率調高     |
| 反光 / 鏡面       | 關閉或降低功率，避免過曝   |
| 多台相機同時使用  | 使用頻閃模式避免干擾       |
| 純 RGB 拍攝       | 可關閉以減少紅外線光斑污染 |

---

### 6. 📷 Global Shutter vs Rolling Shutter

D435 的紅外線/深度鏡頭使用 **Global Shutter**，這是它成為機器人首選的關鍵。

| 快門類型                  | 特性                             | D435 對應         |
| ------------------------- | -------------------------------- | ----------------- |
| **Global Shutter**  | 整張畫面同時曝光，無動態變形     | 紅外線 / 深度鏡頭 |
| **Rolling Shutter** | 逐行曝光，快速移動會產生果凍效應 | 彩色鏡頭          |

**影響**：

- 機器狗快速轉頭時，深度畫面不會變形 ✅
- VSLAM 定位使用 IR 比 RGB 更穩定 ✅
- 彩色影像在高速移動時可能歪斜 ⚠️

---

### 7. 🎛️ 預設模式 (Presets)

相機晶片內建不同的演算法參數組，可快速切換。

| 預設模式                 | 特性                                 | 適用場景           |
| ------------------------ | ------------------------------------ | ------------------ |
| **High Accuracy**  | 濾掉不確定雜訊，畫面有黑洞但數據準確 | 精密測量、點雲     |
| **High Density**   | 盡量填滿畫面，覆蓋率優先             | 避障偵測、安全防護 |
| **Medium Density** | 平衡模式                             | 一般用途           |
| **Hand**           | 針對近距離手部追蹤優化               | 手勢控制           |

```python
depth_sensor = profile.get_device().first_depth_sensor()

# 設定為高密度模式 (避障推薦)
depth_sensor.set_option(rs.option.visual_preset, 4)  # High Density

# 設定為手部追蹤模式 (手勢辨識推薦)
depth_sensor.set_option(rs.option.visual_preset, 2)  # Hand
```

> [!TIP]
> **避障應用建議使用 High Density**：寧可誤判有障礙物，也不要漏看。

---

### 8. 🔄 Hardware Sync (多機硬體同步)

D435 支援多台相機同步運作，避免雷射互相干擾。

| 功能           | 說明                              |
| -------------- | --------------------------------- |
| **接口** | 9-pin 同步接口 (部分型號需拆外殼) |
| **模式** | Master / Slave                    |
| **用途** | 多機環景深度、避免雷射干擾        |

```python
# 設定為 Master 模式
depth_sensor.set_option(rs.option.inter_cam_sync_mode, 1)

# 設定為 Slave 模式
depth_sensor.set_option(rs.option.inter_cam_sync_mode, 2)
```

---

## 影像處理功能

### 9. ⚙️ 硬體級後處理濾鏡 (Post-Processing Filters)

SDK 內建多種濾鏡，可大幅提升深度圖品質。這些是純軟體運算，**Mac M4 完全可用**。

#### Decimation Filter (降取樣濾鏡)

| 功能           | 降低解析度，減少雜訊，加快運算速度 |
| -------------- | ---------------------------------- |
| **參數** | `filter_magnitude` (2, 4, 8)     |
| **效果** | 848x480 → 424x240 (magnitude=2)   |

```python
decimation = rs.decimation_filter()
decimation.set_option(rs.option.filter_magnitude, 2)
filtered = decimation.process(depth_frame)
```

#### Spatial Filter (空間濾鏡)

| 功能           | 平滑邊緣，填補小破洞 (類似磨皮效果)              |
| -------------- | ------------------------------------------------ |
| **參數** | `filter_smooth_alpha`, `filter_smooth_delta` |

```python
spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
spatial.set_option(rs.option.filter_smooth_delta, 20)
filtered = spatial.process(depth_frame)
```

#### Temporal Filter (時間濾鏡)

| 功能           | 利用前幾幀修補當前幀，減少閃爍 |
| -------------- | ------------------------------ |
| **注意** | 移動太快會產生殘影             |

```python
temporal = rs.temporal_filter()
filtered = temporal.process(depth_frame)
```

#### Hole Filling Filter (補洞濾鏡)

| 功能           | 用鄰近像素填滿測不到距離的黑洞         |
| -------------- | -------------------------------------- |
| **模式** | 0=最遠, 1=最近, 2=farthest from around |

```python
hole_filling = rs.hole_filling_filter()
filtered = hole_filling.process(depth_frame)
```

#### 濾鏡組合建議

```python
# 建議的處理順序
decimation = rs.decimation_filter()
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()
hole_filling = rs.hole_filling_filter()

# 依序處理
frame = decimation.process(depth_frame)
frame = spatial.process(frame)
frame = temporal.process(frame)
frame = hole_filling.process(frame)
```

---

## 手勢辨識應用

### 10. ✋ 手勢辨識能力

D435 可透過其 RGB 和 Depth 串流支援多種手勢辨識方案。

#### 支援的辨識方式

| 方式                         | 使用串流 | 說明                               |
| ---------------------------- | -------- | ---------------------------------- |
| **Depth Thresholding** | Depth    | 偵測手的距離與位置，適合簡單互動   |
| **骨架追蹤 (21 點)**   | RGB      | 使用 MediaPipe 等 AI 追蹤手指關節  |
| **物件偵測 (YOLO)**    | RGB      | 直接辨識手勢類別（如：握拳、比讚） |

#### Hand Preset

D435 內建專為手部追蹤優化的預設模式：

```python
depth_sensor.set_option(rs.option.visual_preset, 2)  # Hand Preset
```

> [!NOTE]
> 手勢辨識技術涉及多種開源模型與開發策略，詳細說明請參閱：
> **[gesture_recognition.md](./gesture_recognition.md)** (同資料夾)

---

## 平台相容性

### 功能支援矩陣

| 功能                 | Windows | Linux (Ubuntu) | Mac M4 (社群版) |
| -------------------- | ------- | -------------- | --------------- |
| RGB 彩色串流         | ✅ 完整 | ✅ 完整        | ⚠️ 不穩定     |
| Depth 深度串流       | ✅ 完整 | ✅ 完整        | ✅ 穩定         |
| Infrared 紅外線串流  | ✅ 完整 | ✅ 完整        | ✅ 可行         |
| Emitter 控制         | ✅ 完整 | ✅ 完整        | ✅ 可行         |
| Post-Processing 濾鏡 | ✅ 完整 | ✅ 完整        | ✅ 完全可用     |
| Presets 模式切換     | ✅ 完整 | ✅ 完整        | ⚠️ 部分可用   |
| Hardware Sync        | ✅ 完整 | ✅ 完整        | ❓ 未測試       |
| RealSense Viewer     | ✅ 完整 | ✅ 完整        | ❌ 無法使用     |
| 韌體更新             | ✅ 完整 | ✅ 完整        | ❌ 無法使用     |

### Mac M4 注意事項

> [!WARNING]
> Mac M4 使用社群版 `pyrealsense2-macosx`，部分功能受限。

- **可用**：Depth, Infrared, Emitter 控制, 所有濾鏡
- **不穩定**：RGB 彩色串流 (容易報 `failed to set power state` 錯誤)
- **無法使用**：RealSense Viewer, 韌體更新

**建議開發流程**：

1. 在 Mac 上開發純深度相關功能 (避障、測距、夜視)
2. 需要 RGB 的功能 (人臉辨識、顏色追蹤) 使用替代相機開發，之後移植到 Jetson

---

## 應用場景

### 各功能應用對照表

| 應用             | 需要功能             | Mac M4 可行?    |
| ---------------- | -------------------- | --------------- |
| 自動避障 🚧      | Depth                | ✅ 可行         |
| 夜間避障 🌙      | Depth + IR + Emitter | ✅ 可行         |
| 防跌落偵測 📉    | Depth                | ✅ 可行         |
| 物體跟隨 🚶      | Depth                | ✅ 可行         |
| 手勢控制 ✋      | Depth + IR           | ✅ 可行         |
| 3D 點雲掃描 🗺️ | Depth                | ✅ 可行         |
| SLAM 建圖        | Depth + IR           | ✅ 可行         |
| 人臉辨識 👤      | RGB                  | ⚠️ 用替代相機 |
| 顏色追蹤 🎨      | RGB                  | ⚠️ 用替代相機 |
| YOLO 物件偵測    | RGB                  | ⚠️ 用替代相機 |

### 機器狗專題建議配置

針對 Unitree Go2 Pro 專題，建議的功能配置：

| 功能模組     | 使用功能                       | 優先級  |
| ------------ | ------------------------------ | ------- |
| Safety Layer | Depth + Filters (High Density) | 🔴 最高 |
| 夜間模式     | Depth + IR + Emitter           | 🟡 中等 |
| 點雲建圖     | Depth + Filters                | 🟢 次要 |

---

## 相關檔案

- [SKILL.md](../SKILL.md) - 主入口，開發環境設定指南
- [gesture_recognition.md](./gesture_recognition.md) - 手勢辨識技術方案
