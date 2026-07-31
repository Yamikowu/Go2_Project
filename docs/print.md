windows: go run .

mac: open live server 


Mac 瀏覽器 (前端)                         Windows (Go API)
─────────────────                         ─────────────────

1. 使用者填寫收據內容
2. Canvas 畫出完整收據 (含中文、QR Code)
3. canvas.toDataURL("image/png")
   產生 PNG 圖片的 Base64 字串
4. 包成 JSON 發送 ──────────────────────→  5. 收到 JSON
   6. 從 image_base64 取出圖片
   POST /printer/print                     7. Base64 解碼成圖片 bytes
   {                                       8. 用 GS v 0 送給印表機
   "image_base64": "data:image/png;base64,iVBOR..."
   }
