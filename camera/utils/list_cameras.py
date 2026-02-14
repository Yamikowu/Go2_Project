
# camera/utils/list_cameras.py
# 功能：偵測目前 Mac 上所有可用的相機與其對應的 Index

import cv2

def list_cameras():
    index = 0
    arr = []
    print("正在搜尋相機，請稍後...")
    
    # 掃描 index 0 到 10
    while index < 5:
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:
            # 在 Mac 上 OpenCV 有時拿不到相機名稱，但我們能確認它是否可用
            print(f"✅ 找到相機! [Index: {index}]")
            arr.append(index)
            cap.release()
        else:
            print(f"❌ Index {index} 無法開啟")
        index += 1
    
    if not arr:
        print("\n😱 找不到任何可用的相機！請檢查 Camo 是否已連線。")
    else:
        print(f"\n🎉 搜尋完畢！可用的 Index 有: {arr}")
        print("建議：先在 face_rec_local.py 試試最前面的 Index，如果畫面不是 Camo 再往後換。")

if __name__ == "__main__":
    list_cameras()
