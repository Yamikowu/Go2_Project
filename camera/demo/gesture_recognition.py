import cv2
import mediapipe as mp
import math
from collections import deque

class GestureDetector:
    def __init__(self, max_hands=1, detection_con=0.7, track_con=0.7):
        """
        手勢辨識器 - 支援靜態手勢 + 動態手勢（揮手）
        """
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_hands,
            min_detection_confidence=detection_con,
            min_tracking_confidence=track_con
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # 定義指尖的節點 ID (大拇指 4、食指 8、中指 12、無名指 16、小拇指 20)
        self.tip_ids = [4, 8, 12, 16, 20]
        
        # ==========================================
        # 動態手勢用：記錄手掌中心 X 座標的歷史軌跡
        # ==========================================
        # 用 deque 存最近 45 幀的掌心 X 座標（大約 1.5 秒，能抓到較慢的揮手）
        self.palm_x_history = deque(maxlen=45)
        # 記錄最近 45 幀的靜態手勢（用來判斷揮手時手是否張開）
        self.fingers_history = deque(maxlen=45)

    # =================================================================
    # 主要 API：process_frame
    # =================================================================
    def process_frame(self, frame, draw=True):
        """
        處理一幀影像，回傳：
        - frame: 畫上骨架的影像
        - gesture: 辨識結果字串
        - mode: 對應的陪伴模式字串
        """
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)
        
        gesture = "None"
        mode = "Idle"
        
        if self.results.multi_hand_landmarks:
            for hand_landmarks in self.results.multi_hand_landmarks:
                if draw:
                    self.mp_draw.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                    )
                
                landmarks = hand_landmarks.landmark
                
                # 1. 取得手指伸直狀態
                fingers_up = self._get_fingers_up(landmarks)
                
                # 2. 記錄掌心 X 座標（用於動態手勢判斷）
                palm_x = landmarks[9].x  # 9 號點是中指根部，接近掌心中央
                self.palm_x_history.append(palm_x)
                self.fingers_history.append(fingers_up)
                
                # 3. 先檢查靜態手勢（靜態優先，避免比 OK 時手晃動被誤判為揮手）
                gesture, mode = self._recognize_static_gesture(fingers_up, landmarks)
                
                # 4. 只有當靜態手勢是 Unknown 或 Palm 時，才檢查動態揮手
                #    (因為揮手時手也是張開的，跟 Palm 會衝突)
                if gesture in ("Unknown", "Palm") and self._is_waving():
                    gesture = "Wave"
                    mode = "Greeting"
        else:
            # 沒偵測到手，清空歷史紀錄
            self.palm_x_history.clear()
            self.fingers_history.clear()
                
        return frame, gesture, mode

    # =================================================================
    # 靜態手勢判斷
    # =================================================================
    def _get_fingers_up(self, landmarks):
        """
        判斷五根手指是伸直(1)還是彎曲(0)
        使用距離判斷法，不受手部旋轉角度影響
        """
        fingers = []
        wrist = landmarks[0]
        
        def dist(p1, p2):
            return math.hypot(p1.x - p2.x, p1.y - p2.y)
        
        # --- 大拇指 ---
        # 大拇指尖端(4)離小拇指根部(17)越遠 = 越張開
        if dist(landmarks[4], landmarks[17]) > dist(landmarks[3], landmarks[17]):
            fingers.append(1)
        else:
            fingers.append(0)

        # --- 食指、中指、無名指、小拇指 ---
        # 指尖離手腕越遠 = 伸直
        for id_ in range(1, 5):
            tip = landmarks[self.tip_ids[id_]]
            mid = landmarks[self.tip_ids[id_] - 2]
            if dist(tip, wrist) > dist(mid, wrist):
                fingers.append(1)
            else:
                fingers.append(0)
                
        return fingers

    def _recognize_static_gesture(self, fingers, landmarks):
        """
        根據 [大拇指, 食指, 中指, 無名指, 小拇指] 回傳 (手勢名稱, 對應模式)
        
        陪伴模式對應表：
        ┌──────────┬──────────────┬──────────────────────────────┐
        │ 手勢      │ 模式          │ 說明                          │
        ├──────────┼──────────────┼──────────────────────────────┤
        │ Palm 🖐️  │ Pause        │ 暫停互動，讓狗狗安靜守候       │
        │ Fist 👊  │ Mute         │ 立即靜音（狗狗閉嘴）           │
        │ Thumb 👍 │ Happy        │ 正面回饋（狗狗開心搖尾巴）     │
        │ Index ☝️ │ Listen       │ 開啟聆聯模式（ASR 開始收音）   │
        │ OK 👌    │ Confirm      │ 確認指令（取代口頭「好」）     │
        │ Peace ✌️ │ Relax        │ 放鬆模式（播放輕音樂等）      │
        └──────────┴──────────────┴──────────────────────────────┘
        """
        count = sum(fingers)
        
        # --- 先檢查 OK 手勢 👌 (用距離判斷，優先於 finger array) ---
        # OK 手勢 = 大拇指指尖(4) 跟食指指尖(8) 靠得很近 + 其他三指伸直
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]
        thumb_index_dist = math.hypot(thumb_tip.x - index_tip.x, thumb_tip.y - index_tip.y)
        
        # 計算手掌大小作為相對參考（手腕到中指根部的距離）
        wrist = landmarks[0]
        middle_base = landmarks[9]
        palm_size = math.hypot(wrist.x - middle_base.x, wrist.y - middle_base.y)
        
        # 如果大拇指跟食指的距離 < 手掌大小的 30%，且中指、無名指、小拇指至少 2 根伸直
        if palm_size > 0 and (thumb_index_dist / palm_size) < 0.3 and sum(fingers[2:5]) >= 2:
            return "OK", "Confirm"
        
        # 五指全開 🖐️ → 暫停互動
        if count == 5:
            return "Palm", "Pause"
        
        # 五指全縮 👊 → 靜音
        elif count == 0:
            return "Fist", "Mute"
            
        # 只有大拇指 👍 → 正面回饋
        elif fingers == [1, 0, 0, 0, 0]:
            return "Thumb", "Happy"
            
        # 只有食指 ☝️ → 聆聽模式
        elif fingers == [0, 1, 0, 0, 0]:
            return "Index", "Listen"
            
        # 食指 + 中指 ✌️ → 放鬆模式
        elif fingers == [0, 1, 1, 0, 0]:
            return "Peace", "Relax"
            
        return "Unknown", "Idle"

    # =================================================================
    # 動態手勢判斷：揮手 (Waving)
    # =================================================================
    def _is_waving(self):
        """
        判斷是否正在揮手（張開手掌左右擺動）
        
        原理：
        1. 先確認手掌大部分時間是「張開」的
        2. 確認手有足夠大的移動範圍（不是手抖）
        3. 用平滑後的相鄰幀比較，偵測左右方向的來回轉折
        4. 轉折次數 >= 2 就判定為揮手
        """
        # 需要至少累積 20 幀的資料才開始判斷（更嚴格，避免亂觸發）
        if len(self.palm_x_history) < 20:
            return False
        
        # --- 條件 1：手掌必須是張開的（提高到 70%）---
        open_count = sum(1 for f in self.fingers_history if sum(f) >= 4)
        if open_count < len(self.fingers_history) * 0.7:
            return False
        
        x_list = list(self.palm_x_history)
        
        # --- 條件 2：移動範圍要夠大（不是手抖） ---
        x_range = max(x_list) - min(x_list)
        if x_range < 0.10:  # 至少要移動畫面寬度的 10%（提高，避免小晃動誤判）
            return False
        
        # --- 條件 3：用平滑後的相鄰幀，偵測方向轉折 ---
        # 先做 3 幀的移動平均，消除雜訊抖動
        window = 3
        smoothed = []
        for i in range(len(x_list) - window + 1):
            avg = sum(x_list[i:i + window]) / window
            smoothed.append(avg)
        
        # 比較相鄰的平滑值，判斷每一步的移動方向
        direction_changes = 0
        last_direction = 0  # 0=未知, 1=右, -1=左
        
        for i in range(1, len(smoothed)):
            diff = smoothed[i] - smoothed[i - 1]
            
            # 移動量太小就跳過（避免靜止時的雜訊）
            if abs(diff) < 0.003:
                continue
                
            current_direction = 1 if diff > 0 else -1
            
            if last_direction != 0 and current_direction != last_direction:
                direction_changes += 1
            
            last_direction = current_direction
        
        # 至少 3 次轉折 = 完整的「左右左右」才算揮手（更嚴格）
        return direction_changes >= 3


# =========================================================================
# 測試區：直接在 Mac 上執行
# =========================================================================
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    detector = GestureDetector()
    
    # 防抖動陣列
    gesture_history = deque(maxlen=10)
    mode_history = deque(maxlen=10)
    
    print("=" * 50)
    print("  手勢辨識系統 v2.0 (靜態 + 動態)")
    print("=" * 50)
    print("靜態手勢：")
    print("  🖐️ Palm   → Pause  (暫停互動)")
    print("  👊 Fist   → Mute   (靜音)")
    print("  👍 Thumb  → Happy  (正面回饋)")
    print("  ☝️ Index  → Listen (聆聽模式)")
    print("  👌 OK     → Confirm(確認指令)")
    print("  ✌️ Peace  → Relax  (放鬆模式)")
    print("動態手勢：")
    print("  👋 Wave   → Greeting (打招呼)")
    print("=" * 50)
    print("按 'q' 鍵退出")
    print()
    
    while True:
        success, img = cap.read()
        if not success:
            print("無法獲取影像，請確認相機權限！")
            break
            
        img = cv2.flip(img, 1)
        
        # ------ 使用 API ------
        img, current_gesture, current_mode = detector.process_frame(img)
        # ----------------------
        
        # --- 防抖動 ---
        gesture_history.append(current_gesture)
        mode_history.append(current_mode)
        
        stable_gesture = max(set(gesture_history), key=gesture_history.count)
        stable_mode = max(set(mode_history), key=mode_history.count)
        
        # --- 畫面顯示 ---
        # 手勢名稱（左上角）
        cv2.putText(img, f'Gesture: {stable_gesture}', (10, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        
        # 對應模式（左上角第二行）
        mode_color = (0, 255, 255)  # 預設黃色
        if stable_mode == "Listen":
            mode_color = (255, 100, 0)   # 藍色
        elif stable_mode == "Mute":
            mode_color = (0, 0, 255)     # 紅色
        elif stable_mode == "Happy":
            mode_color = (0, 255, 0)     # 綠色
        elif stable_mode == "Greeting":
            mode_color = (255, 0, 255)   # 紫色
        elif stable_mode == "Confirm":
            mode_color = (0, 200, 200)   # 橘色
            
        cv2.putText(img, f'Mode: {stable_mode}', (10, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, mode_color, 2)
                    
        cv2.imshow("Gesture Recognition v2.0", img)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
