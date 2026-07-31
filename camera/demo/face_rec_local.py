# camera/demo/face_rec_local.py
# 用法：python camera/demo/face_rec_local.py （需先 activate 虛擬環境）source .venv/bin/activate
# 或使用絕對路徑：/Users/yamiko/Code/Archive/School/Go2_Project/.venv/bin/python camera/demo/face_rec_local.py
# 功能：
# 1. 使用 Mac 內建相機 或 iPhone 接續互通相機 (Continuity Camera)
# 2. 使用 InsightFace 進行人臉偵測與辨識
# 3. 按 's' 存人臉 / 'd' 刪除人臉 / 'q' 離開

import cv2
import numpy as np
import insightface
import os
import time

# ========== 設定 ==========
CAMERA_INDEX = 0          # 0 = 預設相機 (Mac 內建 / iPhone 接續互通)
                          # 如果 iPhone 沒被選到，試 1 或 2
FACES_DB_DIR = "camera/faces_db"
MODELS_DIR = "camera/models"
SCREENSHOTS_DIR = "camera/screenshots" # 新增：截圖資料夾
DET_SIZE = (640, 640)
FRAME_SKIP = 2
# ==========================

# 確保目錄存在
for d in [MODELS_DIR, SCREENSHOTS_DIR]:
    if not os.path.exists(d):
        os.makedirs(d)

# 初始化 InsightFace 模型
print(f"正在載入輕量級 InsightFace 模型 (路徑: {MODELS_DIR})...")
app = insightface.app.FaceAnalysis(name='buffalo_s', root=MODELS_DIR, providers=['CPUExecutionProvider'])
app.prepare(ctx_id=0, det_size=DET_SIZE)
print("✅ 模型載入完成！")

# 載入已知人臉資料庫
known_faces = []
known_names = []
loaded_files = set()

def capture_screen(img):
    """純截圖功能：包含畫面上所有的框框與標籤"""
    timestamp = int(time.time())
    filepath = os.path.join(SCREENSHOTS_DIR, f"cap_{timestamp}.jpg")
    cv2.imwrite(filepath, img)
    print(f"📸 [截圖已存檔] -> {filepath}")

def load_faces_db(force_reload=False):
    global known_faces, known_names, loaded_files
    
    if force_reload:
        known_faces = []
        known_names = []
        loaded_files = set()
        print("🔄 強制重新載入資料庫...")

    if not os.path.exists(FACES_DB_DIR):
        os.makedirs(FACES_DB_DIR)
        return

    print("正在增量更新人臉資料庫...")
    new_count = 0
    for name in os.listdir(FACES_DB_DIR):
        person_dir = os.path.join(FACES_DB_DIR, name)
        if os.path.isdir(person_dir):
            for filename in os.listdir(person_dir):
                if filename.lower().endswith(('.jpg', '.jpeg', '.png')):
                    filepath = os.path.join(person_dir, filename)
                    
                    if filepath in loaded_files:
                        continue
                        
                    img = cv2.imread(filepath)
                    if img is None:
                        continue
                    
                    faces = app.get(img)
                    if len(faces) > 0:
                        face = max(faces, key=lambda x: (x.bbox[2]-x.bbox[0]) * (x.bbox[3]-x.bbox[1]))
                        known_faces.append(face.embedding)
                        known_names.append(name)
                        loaded_files.add(filepath)
                        new_count += 1
                        print(f"已新增: {name} ({filename})")
    
    if new_count > 0:
        print(f"✅ 資料庫更新完成，這次新增了 {new_count} 筆，目前總計 {len(known_names)} 筆")
    else:
        print(f"✅ 資料庫已是最新，共 {len(known_names)} 筆資料")

def compare_faces(embedding, threshold=0.45):
    """比對人臉特徵，返回 (名字, 分數)"""
    if not known_faces:
        return "Unknown", 0.0
        
    embeddings_matrix = np.array(known_faces)
    scores = np.dot(embeddings_matrix, embedding) / (
        np.linalg.norm(embeddings_matrix, axis=1) * np.linalg.norm(embedding)
    )
    
    idx = np.argmax(scores)
    max_score = scores[idx]
    
    if max_score > threshold:
        return known_names[idx], max_score
    return "Unknown", max_score

def save_current_face(img, faces):
    """儲存當前畫面最大的人臉"""
    if len(faces) == 0:
        print("❌ 畫面中沒有人臉，無法儲存")
        return

    main_face = max(faces, key=lambda x: (x.bbox[2]-x.bbox[0]) * (x.bbox[3]-x.bbox[1]))
    name, score = compare_faces(main_face.embedding)
    timestamp = int(time.time())

    if name == "Unknown":
        folder_name = f"User_{timestamp}"
        print(f"🆕 發現新面孔，建立新使用者: {folder_name}")
    else:
        folder_name = name
        print(f"📸 更新使用者資料: {name}")

    save_dir = os.path.join(FACES_DB_DIR, folder_name)
    os.makedirs(save_dir, exist_ok=True)
    
    filename = f"{folder_name}_{timestamp}.jpg"
    filepath = os.path.join(save_dir, filename)
    cv2.imwrite(filepath, img)
    print(f"✅ 已儲存照片到: {filepath}")
    
    load_faces_db()

def delete_current_face(faces):
    """刪除當前辨識到的使用者"""
    if len(faces) == 0:
        print("❌ 畫面中沒有人臉")
        return

    main_face = max(faces, key=lambda x: (x.bbox[2]-x.bbox[0]) * (x.bbox[3]-x.bbox[1]))
    name, score = compare_faces(main_face.embedding)
    
    if name == "Unknown":
        print("❌ 無法刪除未知使用者")
        return
        
    dir_path = os.path.join(FACES_DB_DIR, name)
    if os.path.exists(dir_path):
        import shutil
        shutil.rmtree(dir_path)
        print(f"🗑️ 已刪除使用者: {name}")
        load_faces_db(force_reload=True)
    else:
        print(f"⚠️ 資料夾不存在: {dir_path}")

def main():
    load_faces_db()
    
    # 開啟本地相機
    print(f"正在開啟相機 (index: {CAMERA_INDEX})...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    if not cap.isOpened():
        print("❌ 無法開啟相機！")
        print("   提示：如果要用 iPhone 接續互通，請確認：")
        print("   1. iPhone 和 Mac 登入同一個 Apple ID")
        print("   2. iPhone 和 Mac 都有開啟 Wi-Fi 和藍牙")
        print("   3. 試試改 CAMERA_INDEX 為 1 或 2")
        return

    print("✅ 相機已開啟！")
    print("--------------------------------")
    print("操作說明：")
    print("   'q' - 離開")
    print("   's' - 儲存當前人臉 (存檔/強化)")
    print("   'd' - 刪除當前使用者 (慎用！)")
    print("   'c' - 畫面截圖 (Demo 用)")
    print("--------------------------------")

    frame_count = 0
    last_faces = []

    try:
        while True:
            ret, color_image = cap.read()
            if not ret:
                print("⚠️ 無法讀取相機畫面")
                break
            
            frame_count += 1
            
            # 每 FRAME_SKIP 幀才做一次人臉偵測
            if frame_count % FRAME_SKIP == 0:
                last_faces = app.get(color_image)

            # 繪製結果
            for face in last_faces:
                bbox = face.bbox.astype(int)
                name, score = compare_faces(face.embedding)
                
                color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)
                cv2.rectangle(color_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
                cv2.putText(color_image, f"{name} ({score:.2f})", (bbox[0], bbox[1]-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            cv2.putText(color_image, "Local Camera Mode", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow('Face Recognition (Local Camera)', color_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            elif key == ord('s'): save_current_face(color_image, last_faces)
            elif key == ord('d'): delete_current_face(last_faces)
            elif key == ord('c'): capture_screen(color_image)

    except KeyboardInterrupt: pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("👋 相機已關閉")

if __name__ == "__main__":
    main()
