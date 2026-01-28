# ROS2 Safety Layer Node 使用說明

## 📌 為什麼需要這個節點？

### 問題：MCP + YOLO 太慢

你們之前的架構：

```
相機 → YOLO → MCP → LLM → 思考 → MCP → Go2
❌ 延遲: 500ms ~ 1s，狗早就撞上了
```

這就是為什麼需要 **Safety Layer（安全層）** — 一個不經過 LLM 思考的「反射神經」。

### 解決方案：快系統避障

```
RealSense → Safety Layer Node → /cmd_vel → Go2
✅ 延遲: < 50ms，即時反應
```

---

## 🏗️ 架構定位

Safety Layer 是 **快系統 (Fast System)** 的一部分：

| 系統             | 速度  | 負責內容                                         |
| ---------------- | ----- | ------------------------------------------------ |
| **慢系統** | 1~2s  | LLM 思考、VLM 理解、MCP 問答                     |
| **快系統** | <50ms | Nav2 導航、**Safety Layer 避障**、緊急停止 |

---

## 🚀 安裝與使用

### 1. 前置需求

```bash
# 確認 ROS2 已安裝
ros2 --version

# 安裝依賴
sudo apt install ros-$ROS_DISTRO-cv-bridge
pip3 install opencv-python numpy
```

### 2. 複製檔案到 ROS2 工作空間

```bash
# 假設你的 ROS2 workspace 在 ~/ros2_ws
cd ~/ros2_ws/src/go2_robot_sdk
mkdir -p go2_safety/go2_safety
cp safety_layer_node.py ~/ros2_ws/src/go2_robot_sdk/go2_safety/go2_safety/

# 建立 setup.py (如果還沒有)
cd ~/ros2_ws/src/go2_robot_sdk/go2_safety
```

### 3. 編譯

```bash
cd ~/ros2_ws
colcon build --packages-select go2_safety
source install/setup.bash
```

### 4. 執行

```bash
# 基本執行
ros2 run go2_safety safety_layer_node

# 自訂參數
ros2 run go2_safety safety_layer_node --ros-args \
  -p close_threshold:=0.3 \
  -p far_threshold:=1.0 \
  -p enable_safety:=true
```

---

## ⚙️ 參數設定

| 參數                | 預設值 | 說明                     |
| ------------------- | ------ | ------------------------ |
| `close_threshold` | 0.4    | 小於此距離（公尺）則後退 |
| `far_threshold`   | 0.8    | 大於此距離可前進         |
| `max_distance`    | 2.0    | 最大偵測距離             |
| `roi_width`       | 100    | 偵測區域寬度（像素）     |
| `roi_height`      | 100    | 偵測區域高度（像素）     |
| `enable_safety`   | true   | 是否啟用 Safety Layer    |
| `backup_speed`    | 0.3    | 後退速度                 |
| `forward_speed`   | 0.2    | 前進速度                 |

---

## 📡 ROS2 Topics

### 訂閱 (Subscribed)

| Topic                       | Type                  | 說明     |
| --------------------------- | --------------------- | -------- |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度影像 |

### 發布 (Published)

| Topic              | Type                    | 說明                 |
| ------------------ | ----------------------- | -------------------- |
| `/cmd_vel`       | `geometry_msgs/Twist` | 緊急避障指令         |
| `/safety_status` | `std_msgs/String`     | 狀態訊息 (JSON 格式) |

---

## 🔍 狀態訊息格式

`/safety_status` 發布的 JSON：

```json
{
  "timestamp": 1737478000,
  "distance_m": 0.35,
  "action": "BACK",
  "cmd_vel": {
    "linear_x": -0.3,
    "angular_z": 0
  },
  "frame_count": 1234
}
```

---

## 🎯 整合到現有系統

### 與 go2_ros2_sdk 整合

在 `robot.launch.py` 中加入 Safety Layer：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ... 其他 nodes ...

        # Safety Layer
        Node(
            package='go2_safety',
            executable='safety_layer_node',
            name='safety_layer',
            parameters=[{
                'close_threshold': 0.4,
                'far_threshold': 0.8,
                'enable_safety': True
            }]
        ),
    ])
```

### 與 MCP 共存

Safety Layer 和 MCP 可以**同時運作**：

- MCP 負責高階指令（「去廚房」、「跟著我」）
- Safety Layer 負責緊急煞車（「前面有牆！」）

Safety Layer 的 `/cmd_vel` 指令優先級更高（直接發布），確保安全。

---

## 📊 效能監控

節點會每 30 幀輸出一次反應時間統計：

```
[INFO] 📊 反應時間: 23.5ms (過去30幀平均)
```

目標：保持在 **50ms 以下**。

---

## 🛠️ 常見問題

### Q: 深度影像的單位是什麼？

**A:** 預設假設是 **毫米 (mm)**，程式會自動轉成公尺。如果你的深度影像已經是公尺，請修改 `_get_center_distance()` 中的轉換部分。

### Q: 如何暫時停用 Safety Layer？

**A:** 設定參數 `enable_safety:=false`：

```bash
ros2 run go2_safety safety_layer_node --ros-args -p enable_safety:=false
```

### Q: Safety Layer 會和 Nav2 衝突嗎？

**A:** 不會。Safety Layer 只在偵測到**緊急障礙**時才發布指令。Nav2 的路徑規劃仍然正常運作。

---

## 🧠 Smart Avoid Node (聰明閃避版)

除了基本的 `safety_layer_node` (只後退) 之外，還有進階版 `smart_avoid_node` **會轉彎繞過障礙物**。

### 檔案位置

`camera/ros2/smart_avoid_node.py`

### 原理：三區域偵測

```
┌─────────────────────────┐
│   左   │   中   │   右   │
└─────────────────────────┘
```

| 左 | 中 | 右 | 動作           |
| -- | -- | -- | -------------- |
| ✅ | ❌ | ✅ | 後退 (太窄)    |
| ✅ | ❌ | ❌ | **左轉** |
| ❌ | ❌ | ✅ | **右轉** |
| ❌ | ❌ | ❌ | 後退 (都太近)  |
| ✅ | ✅ | ✅ | 不干預 (安全)  |

（✅ = 安全，❌ = 危險）

### 執行

```bash
ros2 run go2_safety smart_avoid_node

# 自訂參數
ros2 run go2_safety smart_avoid_node --ros-args \
  -p danger_distance:=0.3 \
  -p safe_distance:=1.0 \
  -p turn_speed:=0.6
```

### 參數

| 參數                | 預設值 | 說明            |
| ------------------- | ------ | --------------- |
| `danger_distance` | 0.4    | 危險距離 (公尺) |
| `safe_distance`   | 0.8    | 安全距離 (公尺) |
| `backup_speed`    | 0.25   | 後退速度        |
| `turn_speed`      | 0.5    | 轉彎角速度      |
| `enable`          | true   | 是否啟用        |

### 狀態訊息

`/smart_avoid_status` 發布的 JSON：

```json
{
  "distances": {
    "left": 1.2,
    "center": 0.3,
    "right": 0.8
  },
  "action": "TURN_LEFT",
  "cmd_vel": {
    "linear_x": 0,
    "angular_z": 0.5
  },
  "frame": 1234
}
```

### 兩版比較

| 節點                  | 行為   | 適用場景               |
| --------------------- | ------ | ---------------------- |
| `safety_layer_node` | 只後退 | 最保守，確保不撞牆     |
| `smart_avoid_node`  | 會轉彎 | 更聰明，繞過障礙繼續走 |

---

## 🔮 未來擴展

- [X] ~~支援左右閃避（不只後退）~~ ✅ smart_avoid_node
- [ ] 整合 IMU 資料（偵測跌落）
- [ ] 多方向偵測（前後左右，360度）
- [ ] 動態調整速度（距離越近，後退越快）
- [ ] 與 Nav2 Costmap 整合
