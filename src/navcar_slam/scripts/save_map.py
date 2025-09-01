#!/usr/bin/env python3

import os
import sys
import subprocess
import time
from datetime import datetime

def save_map(map_name=None, convert_pgm=True):
    """Save the current map using cartographer's finish_trajectory and write_state services
    
    Args:
        map_name (str): Name for the map file. If None, uses timestamp.
        convert_pgm (bool): Whether to convert to PGM format for compatibility.
    """
    
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
        
        # Convert to PGM format for compatibility with other navigation systems
        if convert_pgm:
            print("Converting to PGM format...")
            pgm_success = convert_to_pgm(map_file_path, map_name, maps_dir)
            
            if pgm_success:
                print(f"Map also saved in PGM format:")
                print(f"  - {os.path.join(maps_dir, f'{map_name}.pgm')}")
                print(f"  - {os.path.join(maps_dir, f'{map_name}.yaml')}")
            else:
                print("Warning: Failed to convert to PGM format")
        
        print(f"To use this map for localization, run:")
        print(f"ros2 launch navcar_slam localization.launch.py map_file:={map_file_path}")
        
        
        return True
        
    except subprocess.TimeoutExpired:
        print("Timeout while saving map. Make sure cartographer is running.")
        return False
    except Exception as e:
        print(f"Error saving map: {e}")
        return False

def convert_to_pgm(pbstream_file, map_name, maps_dir):
    """Convert pbstream file to PGM format using cartographer_pbstream_to_ros_map or map_server"""
    
    pgm_file_path = os.path.join(maps_dir, f"{map_name}.pgm")
    yaml_file_path = os.path.join(maps_dir, f"{map_name}.yaml")
    
    try:
        print("Running cartographer_pbstream_to_ros_map...")
        
        # Method 1: Use cartographer's pbstream_to_ros_map tool
        result = subprocess.run([
            "ros2", "run", "cartographer_ros", "cartographer_pbstream_to_ros_map",
            "-pbstream_filename", pbstream_file,
            "-map_filestem", os.path.join(maps_dir, map_name)
        ], capture_output=True, text=True, timeout=60)
        
        if result.returncode == 0:
            # Check if files were created
            if os.path.exists(pgm_file_path) and os.path.exists(yaml_file_path):
                return True
        
        print("cartographer_pbstream_to_ros_map failed, trying alternative method...")
        
        # Method 2: Use map_server to save current occupancy grid
        print("Saving current occupancy grid using map_server...")
        result = subprocess.run([
            "ros2", "run", "nav2_map_server", "map_saver_cli",
            "-f", os.path.join(maps_dir, map_name)
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode == 0:
            # Check if files were created
            if os.path.exists(pgm_file_path) and os.path.exists(yaml_file_path):
                return True
        
        print(f"Error converting to PGM: {result.stderr}")
        return False
            
    except subprocess.TimeoutExpired:
        print("Timeout while converting to PGM format")
        return False
    except Exception as e:
        print(f"Error converting to PGM: {e}")
        return False

if __name__ == "__main__":
    map_name = None
    convert_pgm = True
    
    # Parse command line arguments
    for i, arg in enumerate(sys.argv[1:], 1):
        if arg == "--no-pgm":
            convert_pgm = False
        elif arg == "--help" or arg == "-h":
            print("Usage: python3 save_map.py [map_name] [--no-pgm]")
            print("  map_name: Optional name for the map (default: map_YYYYMMDD_HHMMSS)")
            print("  --no-pgm: Skip conversion to PGM format")
            sys.exit(0)
        elif not arg.startswith("--"):
            map_name = arg
    
    if save_map(map_name, convert_pgm):
        sys.exit(0)
    else:
        sys.exit(1)
