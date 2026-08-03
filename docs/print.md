# Mac 與 Windows 跨平台列印架構說明

## 架構設計原因
由於部分硬體印表機不支援中文字庫，直接發送純文字(Text)會導致中文無法正確印出。
為了解決這個問題，本系統採用**「前端完全影像化」**的架構：
1. **Mac 端 (前端)** 負責所有複雜的排版工作（如中英文對齊、字體大小、產生 QR Code）。
2. 將排版好的畫面轉為「圖片」。
3. **Windows 端 (後端)** 僅負責接收圖片並將原始像素轉換給硬體印表機。

這樣可以達到「所見即所得」，無論什麼字體或圖案都能完美印出。

---

## 系統角色與職責

### 1. Mac 端 (前端 Web 介面)
- **檔案位置**：`print/` 資料夾 (`index.html`, `app.js`, `styles.css`, `printer-api.js`)
- **職責**：操作介面與收據排版
- **運作流程**：
  1. 使用者在網頁上填寫收據內容（品項、金額、QR Code 網址、Logo 等）。
  2. 前端利用 HTML5 Canvas 2D API 進行精確排版，將所有內容「畫」成一張完整的收據圖片。
  3. 透過 `canvas.toDataURL("image/png")` 將畫好的圖片轉換成 Base64 字串。
  4. 讀取使用者勾選的列印選項（如自動切紙 `cut`、列印份數 `copies`、開錢箱 `openDrawer`）。
  5. 將圖片 Base64 與選項包裝成 JSON 結構，透過 HTTP POST 傳送給 Windows 伺服器。

**發送的 JSON 結構範例**：
```json
{
  "text": "",
  "image_base64": "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAA...",
  "options": {
    "cut": true,
    "copies": 1,
    "openDrawer": false
  }
}
```

### 2. Windows 端 (Go API 伺服器)
- **職責**：硬體驅動與列印控制
- **運作流程**：
  1. 啟動 HTTP 伺服器 (預設 Port 8080)，並允許跨域請求 (CORS Middleware)，讓 Mac 瀏覽器可以合法發送請求。
  2. 接收來自 Mac 端的 `POST /printer/print` 請求，並對應到 Go 的 struct 解析 JSON 資料。
  3. 讀取 JSON 中的 `image_base64` 欄位，去除 Data URL 前綴 (`data:image/png;base64,`) 後，將 Base64 解碼還原成 PNG 圖片位元組 (bytes)。
  4. 使用 ESC/POS 的 `GS v 0` (列印光柵點陣圖) 等相關指令，將圖片的像素資料傳送給實體印表機。
  5. 讀取 JSON 中的 `options` 欄位，根據設定額外發送「切紙」或「開錢箱」等 ESC/POS 控制指令。

---

## 啟動與測試方式

**步驟 1：啟動 Windows 後端伺服器**
在 Windows 電腦的終端機執行：
```bash
go run .
```
> 確保伺服器成功啟動，終端機會顯示類似 `HTTP API server listening on http://0.0.0.0:8080` 的訊息。並請查明這台 Windows 電腦的區網 IP。

**步驟 2：啟動 Mac 前端測試工具**
在 Mac 上開啟 `print/index.html`。由於有使用到 ES6 Module，需透過 Local Server 啟動：
- **方法 A**：在 VS Code 中安裝並點擊 **Live Server**。
- **方法 B**：在終端機執行 `python3 -m http.server 8000` 並在瀏覽器開啟 `http://localhost:8000`。

**步驟 3：進行測試**
1. 在網頁最上方的「API Base URL」填入 Windows 電腦的 IP (例如 `http://192.168.137.1:8080`)。
2. 點擊「測試連線」確認雙方網路相通。
3. 在下方隨意新增品項、上傳 Logo 或填寫 QR Code 網址，並確認右下角的預覽畫面正確。
4. 點擊「🖨️ 傳送列印」，等待印表機成功印出美美的收據！
