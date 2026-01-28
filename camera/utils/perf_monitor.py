#!/usr/bin/env python3
"""
📊 效能監控工具 (Performance Monitor)

共用工具模組，讓其他程式可以輕鬆監控記憶體和 CPU 用量。

使用方式:
    from utils.perf_monitor import PerfMonitor
    
    monitor = PerfMonitor()
    monitor.start()
    
    # ... 你的程式 ...
    
    monitor.log()  # 印出當前用量
    monitor.report()  # 印出完整報告
"""
import os
import time
import psutil
from typing import Optional, List, Dict


class PerfMonitor:
    """效能監控器"""
    
    def __init__(self, name: str = "Process"):
        self.name = name
        self.process = psutil.Process(os.getpid())
        self.start_time = None
        self.start_memory = None
        self.memory_samples: List[float] = []
        self.cpu_samples: List[float] = []
        self.sample_count = 0
    
    def start(self):
        """開始監控"""
        self.start_time = time.time()
        self.start_memory = self._get_memory_mb()
        print(f"📊 [{self.name}] 效能監控開始")
        print(f"   初始記憶體: {self.start_memory:.1f} MB")
    
    def sample(self):
        """取樣一次 (通常在主迴圈中呼叫)"""
        self.memory_samples.append(self._get_memory_mb())
        self.cpu_samples.append(self.process.cpu_percent(interval=0))
        self.sample_count += 1
    
    def log(self, interval: int = 30):
        """
        每 N 次取樣後印出一次 (預設每 30 次)
        回傳 True 表示有印出
        """
        self.sample()
        
        if self.sample_count % interval == 0:
            current_mem = self.memory_samples[-1]
            avg_cpu = sum(self.cpu_samples[-interval:]) / min(len(self.cpu_samples), interval)
            elapsed = time.time() - self.start_time
            
            print(f"📊 [{self.name}] "
                  f"記憶體: {current_mem:.1f}MB | "
                  f"CPU: {avg_cpu:.1f}% | "
                  f"運行: {elapsed:.0f}s")
            return True
        return False
    
    def get_current(self) -> Dict:
        """取得當前狀態 (用於 JSON 輸出)"""
        return {
            "memory_mb": round(self._get_memory_mb(), 1),
            "cpu_percent": round(self.process.cpu_percent(interval=0), 1),
            "elapsed_s": round(time.time() - self.start_time, 1) if self.start_time else 0
        }
    
    def report(self):
        """印出完整報告"""
        if not self.memory_samples:
            print("📊 沒有取樣資料")
            return
        
        elapsed = time.time() - self.start_time
        current_mem = self.memory_samples[-1]
        peak_mem = max(self.memory_samples)
        avg_mem = sum(self.memory_samples) / len(self.memory_samples)
        avg_cpu = sum(self.cpu_samples) / len(self.cpu_samples)
        
        print("\n" + "=" * 60)
        print(f"📊 [{self.name}] 效能報告")
        print("=" * 60)
        print(f"運行時間: {elapsed:.1f} 秒")
        print(f"取樣次數: {len(self.memory_samples)}")
        print("-" * 60)
        print(f"記憶體 (MB):")
        print(f"  初始: {self.start_memory:.1f}")
        print(f"  當前: {current_mem:.1f}")
        print(f"  峰值: {peak_mem:.1f}")
        print(f"  平均: {avg_mem:.1f}")
        print(f"  增長: {current_mem - self.start_memory:+.1f}")
        print("-" * 60)
        print(f"CPU 平均: {avg_cpu:.1f}%")
        print("=" * 60)
    
    def _get_memory_mb(self) -> float:
        """取得當前記憶體用量 (MB)"""
        return self.process.memory_info().rss / (1024 * 1024)


# 快速函數 (不需要建立物件)
def get_memory_mb() -> float:
    """快速取得當前記憶體 (MB)"""
    return psutil.Process(os.getpid()).memory_info().rss / (1024 * 1024)


def get_system_memory() -> Dict:
    """取得系統記憶體狀態"""
    mem = psutil.virtual_memory()
    return {
        "total_gb": round(mem.total / (1024**3), 1),
        "available_gb": round(mem.available / (1024**3), 1),
        "used_gb": round(mem.used / (1024**3), 1),
        "percent": mem.percent
    }


def print_system_info():
    """印出系統資訊"""
    import platform
    mem = get_system_memory()
    
    print("=" * 60)
    print("🖥️ 系統資訊")
    print("=" * 60)
    print(f"平台: {platform.system()} {platform.machine()}")
    print(f"Python: {platform.python_version()}")
    print(f"總記憶體: {mem['total_gb']:.1f} GB")
    print(f"可用記憶體: {mem['available_gb']:.1f} GB ({100 - mem['percent']:.0f}%)")
    print(f"CPU 核心數: {psutil.cpu_count()}")
    print("=" * 60)


# 測試
if __name__ == "__main__":
    print_system_info()
    
    monitor = PerfMonitor("測試")
    monitor.start()
    
    # 模擬一些工作
    import numpy as np
    for i in range(100):
        _ = np.random.rand(1000, 1000)
        monitor.log(interval=10)
        time.sleep(0.05)
    
    monitor.report()
