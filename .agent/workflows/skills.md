---
description: use SKILL
---

Read @.agent/skills/Camera/SKILL.md
import @utils 工具模組加入記憶體監控到之後的py
使用方式：

from utils.perf_monitor import PerfMonitor, print_system_info

monitor = PerfMonitor("程式名稱")
monitor.start()

# 在主迴圈中
monitor.log(interval=30)  # 每 30 幀印一次

# 結束時
monitor.report()