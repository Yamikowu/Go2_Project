# 人臉辨識技術方案比較

> **目的**：為機器狗專題選擇適合的人臉辨識方案
> **需求**：真正的「辨識」功能（認出是誰），不只是「偵測」（找到人臉位置）
> **建立日期**：2026-02-07

---

## 📊 功能區分

| 功能層級               | 說明                   | 範例輸出                      |
| ---------------------- | ---------------------- | ----------------------------- |
| **偵測 (Detection)**   | 找到畫面中的人臉位置   | `[x=100, y=50, w=200, h=200]` |
| **追蹤 (Tracking)**    | 持續追蹤同一張臉的移動 | `Face ID #1 移動到 (150, 60)` |
| **辨識 (Recognition)** | 判斷這張臉是誰         | `這是「小明」，相似度 95%`    |

**你需要的是「辨識」層級。**

---

## 🛠️ 技術方案比較

### 方案 1：face_recognition (推薦入門)

| 項目            | 說明                                         |
| --------------- | -------------------------------------------- |
| **GitHub**      | https://github.com/ageitgey/face_recognition |
| **底層技術**    | dlib + HOG / CNN                             |
| **精度**        | ⭐⭐⭐⭐ 高（LFW 99.38%）                    |
| **速度**        | ⭐⭐⭐ 中等（CPU 約 2-5 FPS）                |
| **易用性**      | ⭐⭐⭐⭐⭐ 極簡單                            |
| **Mac M4 相容** | ✅ 支援（需要額外設定 dlib）                 |

**優點**：

- API 非常簡單，幾行程式碼就能用
- 文件豐富，社群大
- 可以直接比對「已知人臉」資料庫

**缺點**：

- dlib 在 Mac M4 上安裝稍麻煩（需要 cmake）
- 速度不是最快

**安裝**：

```bash
pip install cmake
pip install dlib
pip install face_recognition
```

**範例程式碼**：

```python
import face_recognition

# 載入已知人臉
known_image = face_recognition.load_image_file("xiaoming.jpg")
known_encoding = face_recognition.face_encodings(known_image)[0]

# 載入待辨識的圖片
unknown_image = face_recognition.load_image_file("unknown.jpg")
unknown_encodings = face_recognition.face_encodings(unknown_image)

# 比對
for unknown_encoding in unknown_encodings:
    results = face_recognition.compare_faces([known_encoding], unknown_encoding)
    if results[0]:
        print("這是小明！")
```

---

### 方案 2：DeepFace (功能最全)

| 項目            | 說明                                        |
| --------------- | ------------------------------------------- |
| **GitHub**      | https://github.com/serengil/deepface        |
| **底層技術**    | VGG-Face, Facenet, ArcFace, Dlib 等多種可選 |
| **精度**        | ⭐⭐⭐⭐⭐ 最高（可選 ArcFace 達 99.8%）    |
| **速度**        | ⭐⭐ 較慢（首次載入模型需時間）             |
| **易用性**      | ⭐⭐⭐⭐ 簡單                               |
| **Mac M4 相容** | ✅ 支援                                     |

**優點**：

- 支援多種 AI 模型，可以選最準的
- 除了辨識，還支援「年齡」「性別」「情緒」分析
- 一行程式碼就能比對兩張臉

**缺點**：

- 套件較大（會下載模型檔）
- 首次執行較慢

**安裝**：

```bash
pip install deepface
```

**範例程式碼**：

```python
from deepface import DeepFace

# 比對兩張臉是否為同一人
result = DeepFace.verify("img1.jpg", "img2.jpg")
print(f"是同一人: {result['verified']}, 距離: {result['distance']}")

# 從資料庫中找出是誰
result = DeepFace.find(img_path="unknown.jpg", db_path="./faces_db/")
print(result)

# 分析年齡、性別、情緒
analysis = DeepFace.analyze(img_path="face.jpg", actions=['age', 'gender', 'emotion'])
print(analysis)
```

---

### 方案 3：InsightFace (商業級精度)

| 項目            | 說明                                       |
| --------------- | ------------------------------------------ |
| **GitHub**      | https://github.com/deepinsight/insightface |
| **底層技術**    | ArcFace, RetinaFace                        |
| **精度**        | ⭐⭐⭐⭐⭐ 頂級（業界標準）                |
| **速度**        | ⭐⭐⭐⭐ 快（有 ONNX 優化）                |
| **易用性**      | ⭐⭐⭐ 中等                                |
| **Mac M4 相容** | ⚠️ 需要 onnxruntime                        |

**優點**：

- 精度非常高，商業產品等級
- 速度經過優化
- 支援人臉對齊、特徵點偵測

**缺點**：

- 設定較複雜
- 文件相對少

**安裝**：

```bash
pip install insightface onnxruntime
```

---

### 方案 4：MediaPipe + 自訂辨識

| 項目            | 說明                          |
| --------------- | ----------------------------- |
| **官網**        | https://mediapipe.dev/        |
| **底層技術**    | Google BlazeFace              |
| **精度**        | ⭐⭐⭐ 偵測準，但本身不含辨識 |
| **速度**        | ⭐⭐⭐⭐⭐ 極快（30+ FPS）    |
| **易用性**      | ⭐⭐⭐⭐ 簡單                 |
| **Mac M4 相容** | ✅ 完美支援                   |

**說明**：
MediaPipe 本身只提供「偵測」和「特徵點」，不提供「辨識」。
如果要辨識，需要自己接上其他模型（例如 FaceNet）。

**適合場景**：

- 需要極快的偵測速度
- 願意自己整合辨識模型

---

## 📋 方案選擇建議

| 你的情況                       | 推薦方案                   |
| ------------------------------ | -------------------------- |
| **剛開始學，想快速有成果**     | 👉 **face_recognition**    |
| **想要最準，願意等載入時間**   | 👉 **DeepFace (ArcFace)**  |
| **追求商業級精度**             | 👉 **InsightFace**         |
| **需要極速偵測 + 自己接辨識**  | 👉 **MediaPipe + FaceNet** |
| **需要同時分析情緒/年齡/性別** | 👉 **DeepFace**            |

---

## 🖥️ 邊緣設備部署考量 (Jetson Orin Nano Super)

既然最終要部署到 **Jetson Orin Nano Super**，我們需要從「能在板子上跑」的角度重新評估。

### Jetson 的特點

| 項目       | 規格                  |
| ---------- | --------------------- |
| **GPU**    | NVIDIA Ampere (CUDA)  |
| **記憶體** | 8GB (共享)            |
| **加速器** | TensorRT、CUDA、cuDNN |
| **功耗**   | 7-25W                 |

### 各方案在 Jetson 上的表現

| 方案                 | Jetson 相容      | TensorRT 加速 | 即時性  | 推薦度            |
| -------------------- | ---------------- | ------------- | ------- | ----------------- |
| **InsightFace**      | ✅ 完美          | ✅ 有 ONNX    | 🚀 快   | ⭐⭐⭐⭐⭐        |
| **Facenet-PyTorch**  | ✅ 好            | ✅ 可轉換     | 🚀 快   | ⭐⭐⭐⭐          |
| **DeepFace**         | ⚠️ 可用但肥      | ❌ 較難       | 🐢 慢   | ⭐⭐              |
| **face_recognition** | ⚠️ dlib 編譯麻煩 | ❌ 無         | 🐢 慢   | ⭐⭐              |
| **MediaPipe**        | ✅ 官方支援      | ✅ 內建       | 🚀 極快 | ⭐⭐⭐⭐ (僅偵測) |

---

## ⭐ 最終推薦：InsightFace

**理由**：

1. **ONNX 格式** — 可以直接用 TensorRT 加速，在 Jetson 上跑超快
2. **ArcFace 模型** — 精度業界頂級 (LFW 99.8%)
3. **RetinaFace 偵測** — 偵測速度快且準
4. **輕量版可選** — 有專門為邊緣設備優化的小模型

### InsightFace 在 Jetson 上的預期性能

| 模型              | 解析度  | FPS (預估) | 記憶體 |
| ----------------- | ------- | ---------- | ------ |
| buffalo_l (大)    | 640x640 | ~15 FPS    | ~2GB   |
| buffalo_s (小)    | 640x640 | ~25 FPS    | ~1GB   |
| buffalo_sc (超小) | 640x640 | ~30 FPS    | ~0.5GB |

---

## 🛠️ 開發策略

### 階段 1：Mac 開發 (現在)

**用 InsightFace**，這樣程式碼直接能搬到 Jetson：

```bash
pip install insightface onnxruntime
```

### 階段 2：Jetson 部署 (之後)

**換成 onnxruntime-gpu**，自動使用 TensorRT 加速：

```bash
pip install insightface onnxruntime-gpu
```

_程式碼完全不用改！_

---

## 📦 InsightFace 快速入門

### 安裝 (Mac)

```bash
pip install insightface onnxruntime opencv-python
```

### 基本使用

```python
import insightface
import cv2

# 初始化模型 (首次會自動下載)
app = insightface.app.FaceAnalysis(name='buffalo_l')
app.prepare(ctx_id=0)  # 0 = 第一個 GPU，-1 = CPU

# 載入圖片
img = cv2.imread("test.jpg")

# 偵測 + 取得特徵
faces = app.get(img)
for face in faces:
    print(f"人臉位置: {face.bbox}")
    print(f"特徵向量: {face.embedding.shape}")  # 512 維向量
```

### 人臉比對

```python
import numpy as np

def compare_faces(embedding1, embedding2, threshold=0.4):
    """計算兩張臉的相似度"""
    # 計算餘弦相似度
    similarity = np.dot(embedding1, embedding2) / (
        np.linalg.norm(embedding1) * np.linalg.norm(embedding2)
    )
    return similarity > threshold, similarity

# 比對兩張臉
is_same, score = compare_faces(face1.embedding, face2.embedding)
print(f"是同一人: {is_same}, 相似度: {score:.2%}")
```

---

## 📂 人臉資料庫設計

不管用哪個方案，你都需要建立「已知人臉資料庫」：

```
Go2_Project/
├── faces_db/
│   ├── xiaoming/
│   │   ├── 1.jpg
│   │   ├── 2.jpg
│   │   └── 3.jpg
│   ├── xiaohong/
│   │   ├── 1.jpg
│   │   └── 2.jpg
│   └── owner/          # 主人
│       ├── front.jpg
│       ├── left.jpg
│       └── right.jpg
```

**建議**：

- 每個人準備 3-5 張不同角度的照片
- 照片要清晰，臉部佔畫面 30% 以上
- 可以用相機直接拍，或從影片截圖

---

## ❓ 下一步

1. [ ] 選擇一個方案
2. [ ] 安裝對應套件
3. [ ] 建立人臉資料庫（至少放「主人」的照片）
4. [ ] 整合到 `remote_camera.py`，讓串流畫面即時辨識
