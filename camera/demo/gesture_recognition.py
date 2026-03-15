import cv2
import mediapipe as mp

class GestureDetector:
    def __init__(self, max_hands=1, detection_con=0.7, track_con=0.7):
        """
        初始化手勢辨識器
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

    def process_frame(self, frame, draw=True):
        """
        處理影像，回傳加上骨架的影像以及目前的手勢
        這是準備給其他組員未來呼叫的主要 API
        """
        # 1. BGR 轉 RGB (MediaPipe只接受RGB)
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)
        
        gesture = "None"
        
        # 如果有偵測到手
        if self.results.multi_hand_landmarks:
            for hand_landmarks in self.results.multi_hand_landmarks:
                if draw:
                    self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # 2. 判斷手指是否伸直
                fingers_up = self._get_fingers_up(hand_landmarks.landmark)
                
                # 3. 根據伸直的手指陣列決定手勢名稱
                gesture = self._recognize_gesture(fingers_up)
                
        return frame, gesture

    def _get_fingers_up(self, landmarks):
        """
        判斷五根手指是伸直(1)還是彎曲(0)
        """
        fingers = []
        
        # --- 判斷大拇指 (比較特殊，看 X 軸) ---
        # 這裡為了簡化，先判斷右手的邏輯：指尖(4)的 X 是否大於關節(3)
        if landmarks[self.tip_ids[0]].x > landmarks[self.tip_ids[0] - 1].x:
            fingers.append(1)
        else:
            fingers.append(0)

        # --- 判斷其他四根手指 (看 Y 軸) ---
        # OpenCV 影像的 Y 軸越往下數值越大，所以「小於」代表在畫面的「上方」(伸直)
        for id_ in range(1, 5):
            if landmarks[self.tip_ids[id_]].y < landmarks[self.tip_ids[id_] - 2].y:
                fingers.append(1) # 伸直
            else:
                fingers.append(0) # 彎曲
                
        return fingers

    def _recognize_gesture(self, fingers):
        """
        根據 [大, 食, 中, 無名, 小] 伸直的陣列回傳動作名稱
        """
        count = sum(fingers)
        
        # 五指全開
        if count == 5:
            return "Paper (Stop)"
        
        # 五指全縮
        elif count == 0:
            return "Fist (Ready)"
            
        # 只有大拇指
        elif fingers == [1, 0, 0, 0, 0]:
            return "Thumbs Up (Good)"
            
        # 只有食指
        elif fingers == [0, 1, 0, 0, 0]:
            return "Index Point"
            
        # 食指 + 中指 (YA)
        elif fingers == [0, 1, 1, 0, 0]:
            return "Scissors (Peace)"
            
        return "Unknown"

# =========================================================================
# 【重點】以下是測試區，也就是你自己在 Mac 上開發驗證的地方
# =========================================================================
if __name__ == "__main__":
    # 當你是直接執行這支檔案 (python gesture_recognition.py) 時，才會跑這裡。
    # 當其他組員 import 你的 GestureDetector 時，這裡的程式碼「不會」被執行，非常乾淨！
    
    # 開啟 Mac 內建鏡頭 (若閃退或沒畫面，可把 0 改成 1 試試)
    cap = cv2.VideoCapture(0)
    
    detector = GestureDetector()
    
    # 防抖動陣列
    gesture_history = []
    
    print("啟動攝影機... 按 'q' 鍵退出")
    
    while True:
        success, img = cap.read()
        if not success:
            print("無法獲取影像，請確認相機權限！")
            break
            
        # 翻轉影像 (像鏡子一樣，比較直覺)
        img = cv2.flip(img, 1)
        
        # ------ 使用你寫好的 API ------
        img, current_gesture = detector.process_frame(img)
        # ------------------------------
        
        # --- 防抖動邏輯實作 (必須連續穩定偵測到才算數) ---
        gesture_history.append(current_gesture)
        if len(gesture_history) > 10:  # 記錄大概最近 0.3 秒內的 10 幀畫面
            gesture_history.pop(0)
            
        # 找出這 10 幀中最常出現的手勢當作最終結果，避免畫面閃爍
        stable_gesture = max(set(gesture_history), key=gesture_history.count)
        
        # 畫在螢幕上給你看
        cv2.putText(img, f'Gesture: {stable_gesture}', (10, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                    
        cv2.imshow("Mac Gesture Test", img)
        
        # 按 'q' 關閉
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
