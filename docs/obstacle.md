# 避障模擬器 (obstacle.py) 使用說明

## 📌 這是什麼？

這是一個**原力控制模擬器 (Force Control Simulator)**，用手的距離來模擬機器狗的前進/後退/待命指令。

即使機器狗不在身邊，你也可以在桌上開發「機器人的大腦」邏輯。

---

## 🚀 使用方式

```bash
sudo /Users/yamiko/Documents/VsCode/Go2_Project/.venv/bin/python /Users/yamiko/Documents/VsCode/Go2_Project/camera/demo/obstacle.py
```

> ⚠️ **必須使用 `sudo`**：macOS 需要管理員權限才能存取 USB 裝置。

---

## 🎮 操作說明

1. 執行程式後，把**手**放在相機前面
2. 調整手的距離，觀察畫面上的指令變化
3. 按 `q` 離開

---

## 📏 距離閾值設定

| 距離範圍      | 顯示狀態 | 模擬指令               |
| ------------- | -------- | ---------------------- |
| < 0.4m (40cm) | 🔴 紅框  | **BACK** (後退)        |
| 0.4m ~ 0.8m   | 🟠 橙框  | **STAY** (待命/甜蜜點) |
| > 0.8m        | 🟢 綠框  | **FORWARD** (前進)     |
| 偵測不到      | ⚪ 灰框  | **SEARCH** (搜尋中)    |

---

## 🖥️ 畫面元素說明

```
┌──────────────────────────────────────┐
│  Force Control Simulator              │
│ ┌──┐                                  │
│ │  │  距離條 (類似油量表)              │
│ │██│  - 紅線: 40cm 閾值                │
│ │██│  - 綠線: 80cm 閾值                │
│ │  │                                  │
│ └──┘     ┌────────┐                   │
│          │ 偵測框 │ ← 中央 100x100 像素 │
│          └────────┘                   │
│                                       │
│  Distance: 0.52 m                     │
│  Command: STAY                        │
└──────────────────────────────────────┘
```

---

## ⚙️ 運作原理

### 1. 擷取深度影像

使用 RealSense D435 的深度串流 (640x480, 30fps)。

### 2. 計算中央區域距離

只分析畫面**正中央 100x100 像素**的區域，使用**中位數**計算距離以避免雜訊干擾：

```python
roi = depth_image[cy - 50:cy + 50, cx - 50:cx + 50]
distance = np.median(valid_depths) * depth_frame.get_units()
```

### 3. 判斷指令

根據距離值套用簡單的閾值邏輯：

```python
if distance < 0.4:
    command = "BACK"
elif distance > 0.8:
    command = "FORWARD"
else:
    command = "STAY"
```

### 4. UI 視覺化

使用 OpenCV 繪製儀表板，包含：

- 偵測框 (根據狀態變色)
- 距離條 (類似油量表)
- 文字顯示距離和指令

---

## 🔧 參數調整

打開 `obstacle.py`，修改最上方的常數：

```python
CLOSE_THRESHOLD = 0.4    # 太近的閾值 (公尺)
FAR_THRESHOLD = 0.8      # 太遠的閾值 (公尺)
MAX_DISTANCE = 2.0       # 最大偵測距離
ROI_SIZE = 100           # 偵測區域大小 (像素)
```

---

## 🤖 未來擴展：連接機器狗

當你有機器狗時，只需要把 `print()` 替換成實際的控制指令：

```python
# 原本
if command == "BACK":
    print("後退！")

# 改成
if command == "BACK":
    robot.move_backward(speed=0.3)
```
