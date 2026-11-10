import open3d as o3d
import numpy as np
import argparse
import os
from typing import Optional

def display_pointcloud_and_stations(pcd_path: str, pose_csv_path: Optional[str] = None):
    """
    Utility to display the original point cloud and the generated/imported trajectory stations.
    """
    print(f"Loading point cloud from: {pcd_path}")
    pcd = o3d.io.read_point_cloud(pcd_path)
    if pcd.is_empty():
        print("Failed to load point cloud.")
        return
        
    geometries = [pcd]
    
    if pose_csv_path and os.path.exists(pose_csv_path):
        from io_modules.navvis_parser import NavVisParser
        parser = NavVisParser(pose_csv_path)
        poses = parser.parse()
        
        # Draw camera frustums or spheres at each station
        centers = []
        for p in poses:
            centers.append(p.position)
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
            sphere.translate(p.position)
            sphere.paint_uniform_color([1, 0, 0]) # Red for stations
            geometries.append(sphere)
            
        print(f"Loaded {len(poses)} stations for visualization.")
        
    print("Opening visualizer (close window to exit)...")
    o3d.visualization.draw_geometries(geometries)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Virtual Scanner Results Visualizer")
    parser.add_argument("--pcd", required=True, help="Point cloud file")
    parser.add_argument("--poses", required=False, help="CSV containing poses to visualize as stations")
    args = parser.parse_args()
    
    # Ensure io_modules is in path
    import sys
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
    
    display_pointcloud_and_stations(args.pcd, args.poses)
