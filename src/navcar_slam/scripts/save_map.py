#!/usr/bin/env python3

import os
import sys
import subprocess
from datetime import datetime

def save_map(map_name=None):
    """Save the current map using cartographer's finish_trajectory and write_state services"""
    
    if map_name is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        map_name = f"map_{timestamp}"
    
    # Create maps directory if it doesn't exist
    maps_dir = os.path.expanduser("~/maps")
    os.makedirs(maps_dir, exist_ok=True)
    
    map_file_path = os.path.join(maps_dir, f"{map_name}.pbstream")
    
    print(f"Saving map to: {map_file_path}")
    
    try:
        # Finish the trajectory
        print("Finishing trajectory...")
        result = subprocess.run([
            "ros2", "service", "call", 
            "/finish_trajectory", 
            "cartographer_ros_msgs/srv/FinishTrajectory",
            "{trajectory_id: 0}"
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode != 0:
            print(f"Error finishing trajectory: {result.stderr}")
            return False
            
        # Write state to file
        print("Writing state to file...")
        result = subprocess.run([
            "ros2", "service", "call",
            "/write_state",
            "cartographer_ros_msgs/srv/WriteState",
            f"{{filename: '{map_file_path}'}}"
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode != 0:
            print(f"Error writing state: {result.stderr}")
            return False
            
        print(f"Map saved successfully to: {map_file_path}")
        print(f"To use this map for localization, run:")
        print(f"ros2 launch navcar_slam localization.launch.py map_file:={map_file_path}")
        
        return True
        
    except subprocess.TimeoutExpired:
        print("Timeout while saving map. Make sure cartographer is running.")
        return False
    except Exception as e:
        print(f"Error saving map: {e}")
        return False

if __name__ == "__main__":
    map_name = sys.argv[1] if len(sys.argv) > 1 else None
    
    if save_map(map_name):
        sys.exit(0)
    else:
        sys.exit(1)
