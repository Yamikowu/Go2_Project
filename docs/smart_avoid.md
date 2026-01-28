# Smart Avoid - 聰明閃避避障程式

## 📌 概述

`smart_avoid.py` 是一個本地測試版的避障程式，使用 RealSense D435 深度攝影機偵測前方障礙物，並根據左/中/右三個區域的距離決定閃避動作。

**不需要 ROS2**，可直接在 Mac 上測試演算法邏輯和反應時間。

---

## 🚀 使用方式

```bash
sudo /Users/yamiko/Documents/VsCode/Go2_Project/.venv/bin/python /Users/yamiko/Documents/VsCode/Go2_Project/camera/demo/smart_avoid.py
```

- 按 `q` 離開
- 把手放在相機前面不同位置測試

---

## ⚙️ 運作原理

### 1. 深度影像分割

將 640x480 的深度影像分成**左/中/右**三個區域：

```
┌─────────────────────────────────────┐
│     左 (0~213)  │  中 (213~426)  │  右 (426~640)  │
│                 │                │                │
│   get_distance  │  get_distance  │  get_distance  │
│      ↓          │       ↓        │       ↓        │
│   left_dist     │  center_dist   │   right_dist   │
└─────────────────────────────────────┘
```

### 2. 距離計算

每個區域取**中位數**距離（避免離群值影響）：

```python
def get_distance(roi):
    valid = roi[roi > 0]               # 過濾無效值
    return np.median(valid) / 1000.0   # mm → m
```

### 3. 決策邏輯

根據三區距離與閾值判斷動作：

| 左側 | 中間 | 右側 | 動作            | 說明                 |
| :--: | :--: | :--: | --------------- | -------------------- |
|  ✅  |  ✅  |  ✅  | **CLEAR**       | 全部安全，不干預     |
|  ✅  |  ❌  |  ❌  | **TURN_LEFT**   | 左邊安全，左轉       |
|  ❌  |  ❌  |  ✅  | **TURN_RIGHT**  | 右邊安全，右轉       |
|  ✅  |  ❌  |  ✅  | **TURN_LEFT**   | 左右都安全，預設左轉 |
|  ❌  |  ❌  |  ❌  | **BACKUP**      | 全部危險，後退       |
|  ❌  |  ✅  |  ✅  | **DODGE_LEFT**  | 左邊危險，微右轉     |
|  ✅  |  ✅  |  ❌  | **DODGE_RIGHT** | 右邊危險，微左轉     |

（✅ = 距離 > 0.8m，❌ = 距離 < 0.4m）

### 4. 輸出指令

每個動作對應 `cmd_vel` 指令：

| 動作        | linear_x | angular_z |
| ----------- | -------- | --------- |
| CLEAR       | 0        | 0         |
| TURN_LEFT   | 0        | +0.5      |
| TURN_RIGHT  | 0        | -0.5      |
| BACKUP      | -0.25    | 0         |
| DODGE_LEFT  | 0        | +0.25     |
| DODGE_RIGHT | 0        | -0.25     |

---

## 📊 畫面說明

```
┌──────────────────────────────────────────┐
│ L:1.2m      │ C:0.35m      │ R:0.9m      │  ← 三區距離
│             │              │             │
│             │  TURN_LEFT   │             │  ← 決策動作
│             │              │             │
│ cmd_vel: linear_x=0.00, angular_z=0.50   │  ← 模擬指令
│ Reaction: 12.3ms                         │  ← 反應時間
│ Memory: 145.2MB                          │  ← 記憶體用量
└──────────────────────────────────────────┘
```

邊框顏色代表狀態：

- 🟢 **綠色** = CLEAR (安全)
- 🟡 **黃色** = TURN (轉彎)
- 🔴 **紅色** = BACKUP (後退)

---

## 📈 效能監控

程式整合了 `perf_monitor` 工具，會顯示：

- **Reaction time**: 每幀處理時間（目標 < 50ms）
- **Memory**: 程式記憶體用量
- **結束報告**: 初始/峰值/平均記憶體

---

## 🔧 參數調整

修改檔案開頭的常數：

```python
DANGER_DISTANCE = 0.4   # 危險距離 (公尺)
SAFE_DISTANCE = 0.8     # 安全距離 (公尺)
BACKUP_SPEED = 0.25     # 後退速度
TURN_SPEED = 0.5        # 轉彎角速度
```

---

## 🔗 相關檔案

| 檔案                               | 說明                     |
| ---------------------------------- | ------------------------ |
| `camera/ros2/smart_avoid_node.py`  | ROS2 版本（用於 Jetson） |
| `camera/ros2/safety_layer_node.py` | 簡易版（只後退）         |
| `camera/utils/perf_monitor.py`     | 效能監控工具             |
| `docs/safety_layer_ros2.md`        | ROS2 版說明文件          |
