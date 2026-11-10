import pye57
import argparse
import os

def analyze_e57(file_path: str):
    print(f"Analyzing structured E57 dataset: {file_path}")
    if not os.path.exists(file_path):
        print("File does not exist.")
        return
        
    try:
        e57 = pye57.E57(file_path)
        scan_count = e57.scan_count
        print(f"Total Scan Stations Found: {scan_count}")
        
        total_points = 0
        for i in range(scan_count):
            # Read header first
            try:
                data = e57.read_scan(i)
                pts = len(data.get("cartesianX", []))
            except Exception as e:
                print(f"  Scan_[{i:04d}] - Failed to read: {e}")
                continue
                
            print(f"  Scan_[{i:04d}] - Assigned Points: {pts}")
            total_points += pts
            
        print(f"Total points mapped across all structured standalone scans: {total_points}")
    except Exception as e:
        print(f"Error reading E57 file: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Virtual Scanner Diagnostics")
    parser.add_argument("--e57", required=True, help="Path to generated E57 file")
    args = parser.parse_args()
    analyze_e57(args.e57)
