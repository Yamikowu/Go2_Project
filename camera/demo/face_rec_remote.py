
# camera/demo/face_rec_remote.py
# 用法：/Users/yamiko/Documents/VsCode/Go2_Project/.venv/bin/python camera/demo/face_rec_remote.py
# 功能：
# 1. 連線到 Windows 接收 RGB 串流
# 2. 使用 InsightFace 進行人臉偵測與辨識
# 3. 按 's' 鍵可以將當前畫面中最大的人臉存入資料庫

import socket
import pickle
import struct
import cv2
import numpy as np
import insightface
import os
import time

# ========== 設定 ==========
WINDOWS_IP = "192.168.0.14"    # 請確認這是 Windows 目前的 IP
PORT = 9999
FACES_DB_DIR = "camera/faces_db"
MODELS_DIR = "camera/models"  # 強制將模型存放在專案內部
DET_SIZE = (640, 640)
FRAME_SKIP = 2
# ==========================

# 確保模型目錄存在
if not os.path.exists(MODELS_DIR):
    os.makedirs(MODELS_DIR)

# 初始化 InsightFace 模型
# 指定 root=MODELS_DIR，這樣下載的模型就會乖乖待在專案裡，而不是跑到 User Home 去
print(f"正在載入輕量級 InsightFace 模型 (路徑: {MODELS_DIR})...")
app = insightface.app.FaceAnalysis(name='buffalo_s', root=MODELS_DIR, providers=['CPUExecutionProvider'])
app.prepare(ctx_id=0, det_size=DET_SIZE)
print("✅ 模型載入完成！")

# 載入已知人臉資料庫
known_faces = []
known_names = []
loaded_files = set() # 用來記錄哪些檔案已經載入過

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
                    
                    # 如果已經載入過就跳過，這就是解決卡頓的關鍵！
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

def compare_faces(embedding, threshold=0.45): # 調低一點門檻增加容錯率
    """比對人臉特徵，返回 (名字, 分數)"""
    if not known_faces:
        return "Unknown", 0.0
        
    # 使用矩陣運算一次比完所有特徵，速度快 100 倍
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

    # 找最大的臉
    main_face = max(faces, key=lambda x: (x.bbox[2]-x.bbox[0]) * (x.bbox[3]-x.bbox[1]))
    
    # 先辨識看看是誰
    name, score = compare_faces(main_face.embedding)
    timestamp = int(time.time())

    if name == "Unknown":
        # 如果不認識，就創一個新使用者
        folder_name = f"User_{timestamp}"
        print(f"🆕 發現新面孔，建立新使用者: {folder_name}")
    else:
        # 如果認識，就存到他的資料夾裡 (增強辨識率)
        folder_name = name
        print(f"📸 更新使用者資料: {name}")

    save_dir = os.path.join(FACES_DB_DIR, folder_name)
    os.makedirs(save_dir, exist_ok=True)
    
    # 存檔 (檔名加上時間戳記以免覆蓋)
    filename = f"{folder_name}_{timestamp}.jpg"
    filepath = os.path.join(save_dir, filename)
    cv2.imwrite(filepath, img)
    print(f"✅ 已儲存照片到: {filepath}")
    
    # 重新載入資料庫
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
        
    # 確認刪除
    dir_path = os.path.join(FACES_DB_DIR, name)
    if os.path.exists(dir_path):
        import shutil
        shutil.rmtree(dir_path) # 刪除整個資料夾
        print(f"🗑️ 已刪除使用者: {name}")
        
        # 重新載入資料庫 (強制刷新)
        load_faces_db(force_reload=True)
    else:
        print(f"⚠️ 資料夾不存在: {dir_path}")

def recv_exact(sock, size):
    data = b''
    while len(data) < size:
        packet = sock.recv(min(size - len(data), 65536))
        if not packet:
            return None
        data += packet
    return data

def main():
    load_faces_db()
    
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect((WINDOWS_IP, PORT))
        print("✅ 已連線到 Windows 相機伺服器！")
    except Exception as e:
        print(f"❌ 連線失敗: {e}")
        return
        
    print("--------------------------------")
    print("操作說明：")
    print("   'q' - 離開")
    print("   's' - 儲存當前人臉 (存檔/強化)")
    print("   'd' - 刪除當前使用者 (慎用！)")
    print("--------------------------------")

    frame_count = 0
    last_faces = [] # 儲存上一幀的偵測結果
    fps = 0

    try:
        while True:
            # 接收資料大小
            raw_size = recv_exact(client, 4)
            if not raw_size: break
            size = struct.unpack('>I', raw_size)[0]
            
            # 接收資料
            data = recv_exact(client, size)
            if not data: break
            
            frames = pickle.loads(data)
            color_image = cv2.imdecode(frames['color'], cv2.IMREAD_COLOR)
            
            frame_count += 1
            
            # 每 FRAME_SKIP 幀才做一次人臉偵測
            if frame_count % FRAME_SKIP == 0:
                t_start = time.time()
                last_faces = app.get(color_image)
                fps = 1 / (time.time() - t_start) * (1/FRAME_SKIP) # 粗略估算

            # 繪製結果 (每一幀都畫，讓畫面不閃爍)
            detected_names = []
            for face in last_faces:
                bbox = face.bbox.astype(int)
                name, score = compare_faces(face.embedding)
                if name != "Unknown":
                    detected_names.append(name)
                
                color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)
                cv2.rectangle(color_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
                cv2.putText(color_image, f"{name} ({score:.2f})", (bbox[0], bbox[1]-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # === 新增：回傳辨識結果給 Windows ===
            try:
                if detected_names:
                    msg = f"DETECTED:{','.join(detected_names)}"
                else:
                    msg = "DETECTED:None"
                
                # 傳送文字訊息 (簡單的字串，非常快)
                client.sendall(msg.encode('utf-8') + b'\n') # 加個換行當結束符號
            except Exception as e:
                print(f"⚠️ 回傳訊息失敗: {e}")
            # =================================

            cv2.putText(color_image, f"Status: Tracking faces...", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow('Remote Face Recognition (Optimized)', color_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            elif key == ord('s'): save_current_face(color_image, last_faces)
            elif key == ord('d'): delete_current_face(last_faces)

    except KeyboardInterrupt: pass
    finally:
        client.close()
        cv2.destroyAllWindows()
        print("👋 已斷線")

if __name__ == "__main__":
    main()
