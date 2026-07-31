import { PrinterAPI } from './printer-api.js';

// ==================== 資料 ====================
let items = [
    { name: '經典珍珠奶茶', qty: 2, price: 50 },
    { name: '四季春青茶', qty: 1, price: 35 }
];

let logoImage = null; // 使用者上傳的 Logo (Image 物件)

// ==================== Canvas 收據渲染器 ====================
const CANVAS_W = 544;         // 熱感應紙寬度 (px)
const PADDING = 24;           // 左右內邊距
const CONTENT_W = CANVAS_W - PADDING * 2; // 可用內容寬度
const LINE_HEIGHT = 32;       // 文字行高
const FONT_TITLE = 'bold 32px "Microsoft YaHei", "PingFang TC", "Noto Sans TC", sans-serif';
const FONT_NORMAL = '24px "Microsoft YaHei", "PingFang TC", "Noto Sans TC", sans-serif';
const FONT_BOLD = 'bold 24px "Microsoft YaHei", "PingFang TC", "Noto Sans TC", sans-serif';
const FONT_SMALL = '20px "Microsoft YaHei", "PingFang TC", "Noto Sans TC", sans-serif';

/**
 * 在 Canvas 上繪製完整收據，並回傳 PNG Data URL
 */
function renderReceiptCanvas() {
    const canvas = document.getElementById('receipt-canvas');
    const ctx = canvas.getContext('2d');

    const storeName = document.getElementById('store-name').value || '未命名店家';
    const orderNum = document.getElementById('order-num').value || '未填寫';
    const notes = document.getElementById('notes').value || '';
    const qrUrl = document.getElementById('qr-url').value.trim();

    // --- 第一遍：計算整體所需高度 ---
    let y = PADDING;

    // Logo
    if (logoImage) {
        const logoH = Math.round(logoImage.height * (200 / logoImage.width));
        y += logoH + 16;
    }

    // 店名
    y += 40 + 16; // title font size + gap
    // 分隔線
    y += 12;
    // 訂單編號
    y += LINE_HEIGHT + 8;
    // 日期時間
    y += LINE_HEIGHT + 8;
    // 分隔線
    y += 12;
    // 表頭
    y += LINE_HEIGHT + 4;
    // 品項
    y += items.length * (LINE_HEIGHT + 4);
    // 分隔線
    y += 12;
    // 總金額
    y += LINE_HEIGHT + 16;
    // 備註
    if (notes.trim()) {
        // 估算備註可能需要多少行
        const noteLines = wrapText(ctx, `備註: ${notes}`, FONT_NORMAL, CONTENT_W);
        y += noteLines.length * LINE_HEIGHT + 8;
    }
    // 分隔線
    y += 12;
    // 感謝文字
    y += LINE_HEIGHT + 16;
    // QR Code
    if (qrUrl) {
        y += 180 + 16; // QR Code 150px + 白邊 + gap
    }
    // 底部留白 (撕紙安全距離)
    y += 120;

    // --- 設定 Canvas 高度並開始繪製 ---
    canvas.width = CANVAS_W;
    canvas.height = y;

    // 白色背景
    ctx.fillStyle = '#FFFFFF';
    ctx.fillRect(0, 0, CANVAS_W, y);
    ctx.fillStyle = '#000000';

    let cursorY = PADDING;

    // --- Logo ---
    if (logoImage) {
        const maxLogoW = 200;
        const scale = maxLogoW / logoImage.width;
        const drawW = Math.round(logoImage.width * scale);
        const drawH = Math.round(logoImage.height * scale);
        const logoX = Math.round((CANVAS_W - drawW) / 2);
        ctx.drawImage(logoImage, logoX, cursorY, drawW, drawH);
        cursorY += drawH + 16;
    }

    // --- 店家名稱 (置中) ---
    ctx.font = FONT_TITLE;
    ctx.textAlign = 'center';
    ctx.fillText(storeName, CANVAS_W / 2, cursorY + 32);
    cursorY += 40 + 16;

    // --- 雙線分隔 ---
    drawDoubleLine(ctx, cursorY);
    cursorY += 12;

    // --- 訂單編號 ---
    ctx.font = FONT_NORMAL;
    ctx.textAlign = 'left';
    ctx.fillText(`訂單編號: ${orderNum}`, PADDING, cursorY + 24);
    cursorY += LINE_HEIGHT + 8;

    // --- 日期時間 ---
    const now = new Date();
    const dateStr = `${now.getFullYear()}-${String(now.getMonth() + 1).padStart(2, '0')}-${String(now.getDate()).padStart(2, '0')} ${String(now.getHours()).padStart(2, '0')}:${String(now.getMinutes()).padStart(2, '0')}`;
    ctx.fillText(`日期: ${dateStr}`, PADDING, cursorY + 24);
    cursorY += LINE_HEIGHT + 8;

    // --- 虛線分隔 ---
    drawDashedLine(ctx, cursorY);
    cursorY += 12;

    // --- 表頭 ---
    ctx.font = FONT_BOLD;
    ctx.textAlign = 'left';
    ctx.fillText('品項', PADDING, cursorY + 24);
    ctx.textAlign = 'right';
    ctx.fillText('數量', PADDING + CONTENT_W * 0.65, cursorY + 24);
    ctx.fillText('單價', PADDING + CONTENT_W * 0.82, cursorY + 24);
    ctx.fillText('小計', PADDING + CONTENT_W, cursorY + 24);
    cursorY += LINE_HEIGHT + 4;

    // --- 品項列表 ---
    ctx.font = FONT_NORMAL;
    let total = 0;
    items.forEach(item => {
        const itemTotal = item.qty * item.price;
        total += itemTotal;

        ctx.textAlign = 'left';
        ctx.fillText(item.name || '未命名', PADDING, cursorY + 24);
        ctx.textAlign = 'right';
        ctx.fillText(String(item.qty), PADDING + CONTENT_W * 0.65, cursorY + 24);
        ctx.fillText(`$${item.price}`, PADDING + CONTENT_W * 0.82, cursorY + 24);
        ctx.fillText(`$${itemTotal}`, PADDING + CONTENT_W, cursorY + 24);
        cursorY += LINE_HEIGHT + 4;
    });

    // --- 虛線分隔 ---
    drawDashedLine(ctx, cursorY);
    cursorY += 12;

    // --- 總金額 ---
    ctx.font = FONT_BOLD;
    ctx.textAlign = 'left';
    ctx.fillText('總金額', PADDING, cursorY + 24);
    ctx.textAlign = 'right';
    ctx.font = 'bold 28px "Microsoft YaHei", "PingFang TC", "Noto Sans TC", sans-serif';
    ctx.fillText(`$${total}`, PADDING + CONTENT_W, cursorY + 28);
    cursorY += LINE_HEIGHT + 16;

    // 更新 HTML 上的總金額
    document.getElementById('total-amount').textContent = total;

    // --- 備註 ---
    if (notes.trim()) {
        ctx.font = FONT_SMALL;
        ctx.textAlign = 'left';
        const noteLines = wrapText(ctx, `備註: ${notes}`, FONT_SMALL, CONTENT_W);
        noteLines.forEach(line => {
            ctx.fillText(line, PADDING, cursorY + 20);
            cursorY += LINE_HEIGHT;
        });
        cursorY += 8;
    }

    // --- 虛線分隔 ---
    drawDashedLine(ctx, cursorY);
    cursorY += 12;

    // --- 感謝文字 ---
    ctx.font = FONT_NORMAL;
    ctx.textAlign = 'center';
    ctx.fillText('謝謝光臨，歡迎下次再來', CANVAS_W / 2, cursorY + 24);
    cursorY += LINE_HEIGHT + 16;

    // --- QR Code (像素級精確繪製，不模糊縮放) ---
    if (qrUrl) {
        drawQRCode(ctx, qrUrl, CANVAS_W, cursorY);
    }

    // 回傳 PNG Data URL
    return canvas.toDataURL('image/png');
}

/**
 * 使用 qrcode-generator 產生 QR 矩陣，
 * 逐像素繪製到 Canvas 上 (黑白分明，保留白邊，無模糊)
 */
function drawQRCode(ctx, text, canvasWidth, startY) {
    // 產生 QR 矩陣 (typeNumber 0 = 自動選擇最小)
    const qr = qrcode(0, 'L');
    qr.addData(text);
    qr.make();

    const moduleCount = qr.getModuleCount();
    const targetSize = 150;                          // QR Code 目標尺寸 (px)
    const cellSize = Math.floor(targetSize / moduleCount); // 每個 module 的像素大小 (整數，避免模糊)
    const actualSize = cellSize * moduleCount;
    const quietZone = cellSize * 2;                  // 白邊 = 2 個 module 寬度
    const totalSize = actualSize + quietZone * 2;

    const offsetX = Math.floor((canvasWidth - totalSize) / 2);
    const offsetY = startY;

    // 先畫整塊白底 (包含白邊)
    ctx.fillStyle = '#FFFFFF';
    ctx.fillRect(offsetX, offsetY, totalSize, totalSize);

    // 逐 module 畫黑色像素
    ctx.fillStyle = '#000000';
    for (let row = 0; row < moduleCount; row++) {
        for (let col = 0; col < moduleCount; col++) {
            if (qr.isDark(row, col)) {
                ctx.fillRect(
                    offsetX + quietZone + col * cellSize,
                    offsetY + quietZone + row * cellSize,
                    cellSize,
                    cellSize
                );
            }
        }
    }

    // 恢復填充色
    ctx.fillStyle = '#000000';
}

/** 繪製雙實線 */
function drawDoubleLine(ctx, y) {
    ctx.strokeStyle = '#000000';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(PADDING, y);
    ctx.lineTo(CANVAS_W - PADDING, y);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(PADDING, y + 5);
    ctx.lineTo(CANVAS_W - PADDING, y + 5);
    ctx.stroke();
}

/** 繪製虛線 */
function drawDashedLine(ctx, y) {
    ctx.strokeStyle = '#000000';
    ctx.lineWidth = 1;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    ctx.moveTo(PADDING, y);
    ctx.lineTo(CANVAS_W - PADDING, y);
    ctx.stroke();
    ctx.setLineDash([]);
}

/** 自動換行 (中英文混排) */
function wrapText(ctx, text, font, maxWidth) {
    ctx.font = font;
    const lines = [];
    let currentLine = '';

    for (let i = 0; i < text.length; i++) {
        const testLine = currentLine + text[i];
        const metrics = ctx.measureText(testLine);
        if (metrics.width > maxWidth && currentLine.length > 0) {
            lines.push(currentLine);
            currentLine = text[i];
        } else {
            currentLine = testLine;
        }
    }
    if (currentLine) lines.push(currentLine);
    return lines;
}

// ==================== 顯示狀態訊息 ====================
function showStatus(elementId, success, message) {
    const el = document.getElementById(elementId);
    el.textContent = message;
    el.className = 'status ' + (success ? 'success' : 'error');
    
    setTimeout(() => {
        if (el.textContent === message) {
            el.textContent = '';
            el.className = 'status';
        }
    }, 5000);
}

// ==================== 品項表格 UI ====================
function renderItems() {
    const tbody = document.getElementById('items-tbody');
    tbody.innerHTML = '';
    let total = 0;

    items.forEach((item, index) => {
        const itemTotal = item.qty * item.price;
        total += itemTotal;

        const tr = document.createElement('tr');
        tr.innerHTML = `
            <td><input type="text" class="item-name" value="${item.name}" placeholder="品項名稱" data-index="${index}"></td>
            <td><input type="number" class="item-qty" value="${item.qty}" min="1" data-index="${index}"></td>
            <td><input type="number" class="item-price" value="${item.price}" min="0" data-index="${index}"></td>
            <td class="item-subtotal">${itemTotal}</td>
            <td><button class="btn-delete" data-index="${index}">刪除</button></td>
        `;
        tbody.appendChild(tr);
    });

    document.getElementById('total-amount').textContent = total;
    renderReceiptCanvas();
}

// ==================== 事件綁定 ====================
document.addEventListener('DOMContentLoaded', () => {
    // 初始渲染
    renderItems();

    // 監聽一般輸入改變 → 重繪 Canvas
    const inputIds = ['store-name', 'order-num', 'notes', 'qr-url'];
    inputIds.forEach(id => {
        document.getElementById(id).addEventListener('input', () => renderReceiptCanvas());
    });

    // 監聽 Logo 上傳
    document.getElementById('logo-file').addEventListener('change', (e) => {
        const file = e.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (ev) => {
            const img = new Image();
            img.onload = () => {
                logoImage = img;
                document.getElementById('btn-clear-logo').style.display = 'inline-flex';
                renderReceiptCanvas();
            };
            img.src = ev.target.result;
        };
        reader.readAsDataURL(file);
    });

    // 移除 Logo
    document.getElementById('btn-clear-logo').addEventListener('click', () => {
        logoImage = null;
        document.getElementById('logo-file').value = '';
        document.getElementById('btn-clear-logo').style.display = 'none';
        renderReceiptCanvas();
    });

    // 監聽表格內的輸入改變 (局部更新，避免焦點跳脫)
    document.getElementById('items-tbody').addEventListener('input', (e) => {
        if (e.target.tagName === 'INPUT') {
            const index = e.target.dataset.index;
            if (e.target.classList.contains('item-name')) {
                items[index].name = e.target.value;
            } else if (e.target.classList.contains('item-qty')) {
                items[index].qty = parseInt(e.target.value) || 0;
            } else if (e.target.classList.contains('item-price')) {
                items[index].price = parseInt(e.target.value) || 0;
            }
            
            // 局部更新小計
            const itemTotal = items[index].qty * items[index].price;
            const tr = e.target.closest('tr');
            if (tr) {
                tr.querySelector('.item-subtotal').textContent = itemTotal;
            }

            // 更新總金額
            let total = 0;
            items.forEach((item) => {
                total += item.qty * item.price;
            });
            document.getElementById('total-amount').textContent = total;
            renderReceiptCanvas();
        }
    });

    // 刪除品項
    document.getElementById('items-tbody').addEventListener('click', (e) => {
        if (e.target.classList.contains('btn-delete')) {
            const index = e.target.dataset.index;
            items.splice(index, 1);
            renderItems();
        }
    });

    // 新增品項
    document.getElementById('btn-add-item').addEventListener('click', () => {
        items.push({ name: '', qty: 1, price: 0 });
        renderItems();
    });

    // 測試連線
    document.getElementById('btn-test').addEventListener('click', async () => {
        const url = document.getElementById('api-url').value;
        if (!url) {
            showStatus('status-msg', false, '請輸入 API Base URL');
            return;
        }
        
        showStatus('status-msg', true, '測試連線中...');
        const api = new PrinterAPI(url);
        const result = await api.checkHealth();
        showStatus('status-msg', result.success, result.message);
    });

    // 列印按鈕 → 取得 Canvas PNG → 傳給 API
    document.getElementById('btn-print').addEventListener('click', async () => {
        const url = document.getElementById('api-url').value;
        if (!url) {
            showStatus('print-msg', false, '請先在上方設定 API Base URL');
            return;
        }

        const btn = document.getElementById('btn-print');
        const originalText = btn.innerHTML;
        btn.innerHTML = '⏳ 傳送中...';
        btn.disabled = true;

        try {
            // 先重新渲染一次確保最新內容，取得 PNG Data URL
            const pngDataUrl = renderReceiptCanvas();

            // 讀取列印選項
            const options = {
                cut: document.getElementById('opt-cut').checked,
                copies: parseInt(document.getElementById('opt-copies').value) || 1,
                openDrawer: document.getElementById('opt-drawer').checked
            };

            const api = new PrinterAPI(url);
            const result = await api.printReceipt(pngDataUrl, options);
            
            showStatus('print-msg', result.success, result.message);
        } finally {
            btn.innerHTML = originalText;
            btn.disabled = false;
        }
    });
});
