import pye57
import argparse
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from io_modules.e57_validator import validate_exported_e57


def analyze_e57(file_path: str):
    print(f"Analyzing structured E57 dataset: {file_path}")
    if not os.path.exists(file_path):
        print("File does not exist.")
        return

    res = validate_exported_e57(file_path, expect_panoramas=False)
    print(f"Total Scan Stations Found: {res['scan_count']}")
    print(f"Total Embedded 2D Panoramas Found: {res['image_count']}")

    for s in res["scans"]:
        ib = s["index_bounds"]
        print(
            f"  Scan_[{s['scan_index']:04d}] ({s['guid']}) - Assigned Points: {s['point_count']:,} | "
            f"Structured Grid: rows=[{ib['rowMinimum']}..{ib['rowMaximum']}], "
            f"cols=[{ib['columnMinimum']}..{ib['columnMaximum']}] | "
            f"RGB={s['has_rgb']} | Pose={s['has_pose']}"
        )

    for img in res["images"]:
        print(
            f"  Pano_[{img['image_index']:04d}] ({img['guid']}) -> Linked Scan GUID: {img['associatedData3DGuid']} | "
            f"Resolution: {img['width']}x{img['height']} | Pose Consistent: {img['pose_consistent']}"
        )

    print(f"Total points mapped across all structured standalone scans: {res['total_points_read']:,}")
    print(f"Structural Validation Status: {'PASSED' if res['valid'] else 'FAILED'}")
    if res["errors"]:
        for err in res["errors"]:
            print(f"  [ERROR] {err}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Virtual Scanner Diagnostics")
    parser.add_argument("--e57", required=True, help="Path to generated E57 file")
    args = parser.parse_args()
    analyze_e57(args.e57)

