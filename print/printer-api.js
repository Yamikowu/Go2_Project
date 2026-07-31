export class PrinterAPI {
    constructor(baseUrl) {
        // 移除網址結尾的斜線以確保一致性
        this.baseUrl = baseUrl.replace(/\/$/, '');
    }

    async checkHealth() {
        try {
            const response = await fetch(`${this.baseUrl}/health`);
            if (!response.ok) {
                throw new Error(`HTTP 錯誤! 狀態碼: ${response.status}`);
            }
            const text = await response.text();
            return { success: true, message: `連線成功！(${text})` };
        } catch (error) {
            return { success: false, message: `連線失敗: ${error.message}` };
        }
    }

    /**
     * 傳送收據圖片與列印選項給印表機 API
     * @param {string} imageBase64 - PNG Data URL (data:image/png;base64,...)
     * @param {object} options - 列印選項
     * @param {boolean} options.cut - 列印後自動切紙
     * @param {number}  options.copies - 列印份數
     * @param {boolean} options.openDrawer - 開啟錢箱
     */
    async printReceipt(imageBase64, options = {}) {
        try {
            const payload = {
                text: "",
                image_base64: imageBase64,
                options: {
                    cut: options.cut ?? true,
                    copies: options.copies ?? 1,
                    openDrawer: options.openDrawer ?? false
                }
            };

            const response = await fetch(`${this.baseUrl}/printer/print`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(payload)
            });

            if (!response.ok) {
                const errorData = await response.text();
                throw new Error(`列印失敗 (狀態碼: ${response.status}): ${errorData}`);
            }
            
            return { success: true, message: '列印指令已成功送出！' };
        } catch (error) {
            return { success: false, message: `錯誤: ${error.message}` };
        }
    }
}
