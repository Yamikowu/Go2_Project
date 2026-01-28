#!/usr/bin/env python3
"""
🌐 房間點雲掃描器 (Room Point Cloud Scanner)
使用 RealSense D435 掃描並顯示 3D 點雲

使用方式:
sudo /Users/yamiko/Documents/VsCode/Go2_Project/.venv/bin/python /Users/yamiko/Documents/VsCode/Go2_Project/camera/demo/pointcloud.py

操作:
- 按 's' 儲存點雲到 .ply 檔案
- 按 'r' 重置視角
- 按 'q' 離開
- 滑鼠拖曳旋轉視角
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import time
import os
from datetime import datetime

def create_point_cloud(depth_frame, intrinsics):
    """將深度幀轉換為點雲 (無 RGB 版本)"""
    depth_image = np.asanyarray(depth_frame.get_data())
    h, w = depth_image.shape
    
    # 建立網格座標
    u = np.arange(w)
    v = np.arange(h)
    u, v = np.meshgrid(u, v)
    
    # 取得深度值 (轉換為公尺)
    z = depth_image * depth_frame.get_units()
    
    # 放寬過濾範圍，確保能抓到資料
    valid = (z > 0.05) & (z < 10.0)  # 0.05m ~ 10m
    
    # 計算 3D 座標
    fx = intrinsics.fx
    fy = intrinsics.fy
    cx = intrinsics.ppx
    cy = intrinsics.ppy
    
    # 公式修正：Y 軸和 X 軸可能需要翻轉
    # RealSense 原生: X向右, Y向下, Z向前
    # Open3D 顯示習慣: Y向上
    x = (u - cx) * z / fx
    y = -(v - cy) * z / fy  # 加負號翻轉 Y 軸
    x = -x                  # 加負號做鏡像 (像照鏡子一樣)
    
    # 組合點雲
    points = np.stack([x[valid], y[valid], z[valid]], axis=-1)
    
    # --- 用深度值來產生假顏色 (Rainbow) ---
    z_valid = z[valid]
    # 正規化到 0-1 (假設最大距離 4m)
    norm_z = np.clip(z_valid / 4.0, 0, 1)
    
    # 漸層色 (近=紅, 遠=藍)
    colors = np.zeros((len(z_valid), 3))
    colors[:, 0] = 1 - norm_z  # Red: 近的多
    colors[:, 1] = np.sin(norm_z * np.pi) * 0.5  # Green: 中間的多
    colors[:, 2] = norm_z      # Blue: 遠的多
    
    return points, colors

def main():
    print("=" * 50)
    print("🌐 純幾何點雲掃描器 (無 RGB)")
    print("=" * 50)
    
    # 初始化相機 (只開深度！)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    try:
        profile = pipeline.start(config)
        
        # 取得內參
        depth_stream = profile.get_stream(rs.stream.depth)
        intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
        
        print("✅ 相機啟動成功！(Depth Only Mode)")
        print("📋 操作說明:")
        print("   ⚠️  請確保「2D Preview」視窗被點選（有藍色邊框）")
        print("   's' = 儲存點雲 (.ply)")
        print("   'q' = 離開")
        print("   滑鼠拖曳 3D 視窗 = 旋轉視角")
        print("=" * 50)
        
        # 建立 Open3D 視窗
        vis = o3d.visualization.Visualizer()
        vis.create_window("Point Cloud (Depth Color)", width=1280, height=720)
        pcd = o3d.geometry.PointCloud()
        vis.add_geometry(pcd)
        
        # 加入座標軸 (幫助理解方向)
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        vis.add_geometry(coord_frame)
        
        # 設定視角
        opt = vis.get_render_option()
        opt.point_size = 2.0
        opt.background_color = np.array([0, 0, 0])
        opt.show_coordinate_frame = True
        
        frame_count = 0
        first_update = True
        
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue
            
            frame_count += 1
            if frame_count % 5 == 0:
                points, colors = create_point_cloud(depth_frame, intrinsics)
                
                if len(points) > 0:
                    pcd.points = o3d.utility.Vector3dVector(points)
                    pcd.colors = o3d.utility.Vector3dVector(colors)
                    vis.update_geometry(pcd)
                    
                    # 第一次更新後自動調整視角
                    if first_update:
                        vis.reset_view_point(True)
                        first_update = False
                        print("🎯 視角已自動對焦到點雲！")
                    
                    # Debug: 顯示點數
                    if frame_count % 30 == 0:  # 每 30 幀印一次
                        print(f"📊 點雲點數: {len(points):,}")
            
            if not vis.poll_events():
                break
            vis.update_renderer()
            
            # 2D 預覽
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            
            # 在 2D 畫面上顯示點數
            cv2.putText(depth_colormap, f"Points: {len(pcd.points):,}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(depth_colormap, "Press 's' to save, 'q' to quit", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow('2D Preview', depth_colormap)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                o3d.io.write_point_cloud(f"scan_{timestamp}.ply", pcd)
                print(f"💾 已儲存！")
            elif key == ord('r'):
                vis.reset_view_point(True)
            elif key == ord('f'):
                # 上下翻轉標記 (雖然這裡沒用到全域變數，但使用者知道有按到就好，實際上 Open3D 視角操作比較快)
                print("🔄 建議直接用滑鼠旋轉視角調整 (RealSense 原生資料 Y 軸通常朝下)")
    
    except RuntimeError as e:
        print(f"❌ 錯誤: {e}")
    
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        vis.destroy_window()

if __name__ == "__main__":
    main()
