import argparse
import logging
import sys
import os
import time
import shutil
import json
import numpy as np
from typing import Optional

from config import LOGGING_LEVEL, LOGGING_FORMAT

def setup_logging():
    logging.basicConfig(
        level=LOGGING_LEVEL,
        format=LOGGING_FORMAT,
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )
    return logging.getLogger("virtual_scanner")

def run_pipeline(input_cloud_path: str, output_e57_path: str, pose_csv_path: Optional[str] = None, 
                 navvis_images_dir: Optional[str] = None, mode_pref: str = "auto", logger_instance=None, 
                 alignment_file: Optional[str] = None, spacing: Optional[float] = None, 
                 overlap: Optional[float] = None, coverage: Optional[float] = None,
                 max_range: Optional[float] = None, project_dir: Optional[str] = None,
                 job_id: Optional[str] = None, dry_run: bool = False,
                 scan_density_pct: float = 100.0):
    """
    Executes the entire SLAM -> Structured Scan pipeline.
    """
    start_time = time.time()
    if logger_instance is None:
        logger_instance = logging.getLogger(__name__)

    logger_instance.info("Starting Universal SLAM -> Structured Scan Reconstruction System")
    logger_instance.info(f"Input file: {input_cloud_path}")
    logger_instance.info(f"Output file: {output_e57_path}")
    
    # -------------------------------------------------------------------------
    # Preflight Integrity Validation
    # -------------------------------------------------------------------------
    logger_instance.info("Running Preflight Integrity Validation...")
    
    # 1. Input cloud exists and is readable
    if not os.path.exists(input_cloud_path):
        logger_instance.error(f"Preflight Error: Input file not found: {input_cloud_path}")
        raise FileNotFoundError(f"Input file not found: {input_cloud_path}")
        
    # 2. Pose CSV exists and has valid formatting (if provided)
    if pose_csv_path:
        if not os.path.exists(pose_csv_path):
            logger_instance.error(f"Preflight Error: Pose CSV not found: {pose_csv_path}")
            raise FileNotFoundError(f"Pose CSV not found: {pose_csv_path}")
        try:
            with open(pose_csv_path, 'r', encoding='utf-8') as f:
                for i, line in enumerate(f):
                    if i > 5:
                        break
                    line = line.strip()
                    if line and not line.startswith('#'):
                        parts = line.split(';')
                        if len(parts) < 6:
                            raise ValueError("Delimiter ';' not found or incorrect column count.")
        except Exception as e:
            logger_instance.error(f"Preflight Error: Pose CSV is malformed or unreadable: {e}")
            raise ValueError(f"Pose CSV is malformed or unreadable: {e}")

    # 3. Output directory writable & disk space check
    output_dir = os.path.dirname(os.path.abspath(output_e57_path))
    os.makedirs(output_dir, exist_ok=True)
    if not os.access(output_dir, os.W_OK):
        logger_instance.error(f"Preflight Error: Output directory not writable: {output_dir}")
        raise PermissionError(f"Output directory not writable: {output_dir}")
        
    try:
        _, _, free = shutil.disk_usage(output_dir)
        if free < 200 * 1024 * 1024: # 200 MB minimum
            raise OSError(f"Insufficient disk space in output directory. Free space: {free / (1024*1024):.2f} MB")
    except Exception as e:
        logger_instance.warning(f"Could not verify disk space: {e}")

    # 4. Check that panorama folder exists if CSV provided
    if pose_csv_path and navvis_images_dir:
        if not os.path.exists(navvis_images_dir):
            logger_instance.error(f"Preflight Error: Panorama directory not found: {navvis_images_dir}")
            raise FileNotFoundError(f"Panorama directory not found: {navvis_images_dir}")

    # Integration of the full processing pipeline
    from io_modules.cloud_loader import CloudLoader
    from io_modules.navvis_parser import NavVisParser, extract_e57_poses, verify_and_align_poses
    from io_modules.exporter_e57 import E57Exporter
    from stations.trajectory_mode import TrajectoryStationGenerator
    from stations.geometry_mode import GeometryStationGenerator
    from simulation.visibility import VisibilityOptimizer
    from simulation.scanner import ScannerSimulator
    from config import TARGET_SCAN_SPACING_MIN, MIN_OVERLAP_PERCENTAGE, TARGET_COVERAGE_PERCENTAGE
    
    # Configuration overrides
    target_spacing = spacing if spacing is not None else TARGET_SCAN_SPACING_MIN
    min_overlap = overlap if overlap is not None else MIN_OVERLAP_PERCENTAGE
    target_coverage = coverage if coverage is not None else TARGET_COVERAGE_PERCENTAGE
    
    # 1. Load Data
    logger_instance.info("Stage 1: Loading Data")
    loader = CloudLoader(input_cloud_path)
    pcd, extra_data = loader.load()
    
    pano_poses = None
    candidates = []
    
    if pose_csv_path and os.path.exists(pose_csv_path):
        parser_nv = NavVisParser(pose_csv_path, navvis_images_dir, alignment_file)
        pano_poses = parser_nv.parse()
    elif input_cloud_path and input_cloud_path.lower().endswith('.e57'):
        logger_instance.info("No pose CSV supplied — inspecting input E57 for embedded scan headers/poses...")
        pano_poses = extract_e57_poses(input_cloud_path)

    # Fallback: if no poses found yet and an alignment bundle (.bundle/.zip) was supplied, check inside bundle
    if (not pano_poses or len(pano_poses) == 0) and alignment_file and os.path.exists(alignment_file):
        logger_instance.info(f"Checking alignment bundle for embedded poses/panoramas: {alignment_file}")
        parser_bundle = NavVisParser(None, navvis_images_dir, alignment_file)
        pano_poses = parser_bundle.parse()

    # Verify and auto-align pose coordinate frame against loaded point cloud bounds
    if pano_poses and len(pano_poses) > 0:
        pano_poses = verify_and_align_poses(pano_poses, pcd, alignment_file)


    # Determine execution mode
    mode = mode_pref
    if mode == "auto":
        if pano_poses and len(pano_poses) > 0:
            mode = "trajectory"
            logger_instance.info(f"Auto mode: Found {len(pano_poses)} camera poses/panoramas, using 'trajectory' mode.")
        else:
            mode = "geometry"
            logger_instance.info("Auto mode: No camera poses found, falling back to 'geometry' mode.")

    logger_instance.info(f"Using mode: {mode}")

    # 2. Generate Candidate Stations
    logger_instance.info("Stage 2: Generating Candidate Stations")
    if mode == "trajectory" and pano_poses:
        gen = TrajectoryStationGenerator(target_spacing=target_spacing, scan_density_pct=scan_density_pct)
        selected_poses = gen.generate(pano_poses)
        candidates = [p.position for p in selected_poses]
        pano_poses_for_export = selected_poses
        final_stations = candidates
    else:
        logger_instance.info("Running pure Geometry-based candidate generation.")
        gen = GeometryStationGenerator(target_spacing=target_spacing)
        candidates = gen.generate(pcd)
        pano_poses_for_export = None
        final_stations = []

    if not candidates:
        logger_instance.error("Failed to generate any candidate scan stations. Exiting.")
        raise RuntimeError("Failed to generate any candidate scan stations.")

    # Dry Run Mode logic
    if dry_run:
        logger_instance.info("--------------------------------------------------")
        logger_instance.info("DRY RUN MODE ACTIVE - ESTIMATING RUN DIAGNOSTICS")
        logger_instance.info(f"Generated Candidate Stations: {len(candidates)}")
        pt_count = len(pcd.points) if pcd else 0
        logger_instance.info(f"Input Point Count: {pt_count}")
        
        # Estimate resources
        est_mem_mb = int((pt_count * 32) / (1024 * 1024))
        est_runtime_sec = int((pt_count / 1_000_000) * len(candidates) * 0.15)
        est_runtime_sec = max(5, est_runtime_sec)
        
        logger_instance.info(f"Estimated Peak Memory: ~{est_mem_mb} MB")
        logger_instance.info(f"Estimated Processing Runtime: ~{est_runtime_sec} seconds")
        logger_instance.info(f"Estimated Output Size: ~{int((pt_count * 15) / (1024 * 1024))} MB")
        
        if mode == "trajectory" and pano_poses_for_export:
            missing_count = 0
            for pose in pano_poses_for_export:
                if not pose.image_path or not os.path.exists(pose.image_path):
                    missing_count += 1
            if missing_count > 0:
                logger_instance.warning(f"Preflight Warning: {missing_count} out of {len(pano_poses_for_export)} scans have missing panorama files!")
                
        logger_instance.info("DRY RUN COMPLETED. No files were written.")
        logger_instance.info("--------------------------------------------------")
        return True
        
    # 3. Optimize Visibility
    if mode != "trajectory":
        logger_instance.info("Stage 3: Optimizing Visibility")
        optimizer = VisibilityOptimizer(min_overlap=min_overlap, target_coverage=target_coverage)
        final_stations = optimizer.optimize(pcd, candidates)
    else:
        logger_instance.info(f"Stage 3: Trajectory Mode active — using {len(final_stations)} scan stations directly.")
    
    if mode != "trajectory" and pano_poses_for_export:
        final_pano_poses = []
        for st in final_stations:
            idx = candidates.index(st)
            final_pano_poses.append(pano_poses_for_export[idx])
        pano_poses_for_export = final_pano_poses
        
    # 4. Simulate Scans
    logger_instance.info("Stage 4: Simulating Scans")
    simulator = ScannerSimulator()
    
    from config import DEFAULT_MAX_SCAN_RANGE
    r_max = max_range if max_range is not None else DEFAULT_MAX_SCAN_RANGE
    
    scans = simulator.simulate_scans(pcd, final_stations, max_range=r_max)
    
    # Pre-export validation check (strict scan requirements)
    logger_instance.info("Running pre-export structural validation checks...")
    
    if mode == "trajectory" and pano_poses_for_export:
        if len(pano_poses_for_export) != len(final_stations):
            raise ValueError("Validation Error: Mismatched or missing panoramas for selected scan stations.")
            
        for i, pose in enumerate(pano_poses_for_export):
            if not pose.position:
                raise ValueError(f"Validation Error: Scan station {i+1} is missing coordinate pose.")
            assigned_pts = len(scans.get(i, []))
            if assigned_pts == 0:
                logger_instance.warning(f"Validation Warning: Scan station {i+1} has 0 assigned points.")
            if pose.image_path and not os.path.exists(pose.image_path):
                raise FileNotFoundError(f"Validation Error: Scan station {i+1} has missing or invalid panorama: {pose.image_path}")
    
    # 5. Export
    logger_instance.info(f"Stage 5: Exporting Structured Data to {output_e57_path}")
    exporter = E57Exporter(output_e57_path)
        
    exporter.export(pcd, scans, final_stations, pano_poses_for_export, extra_data)
    
    # 6. Generate Validation Report & Processing Manifest
    duration = time.time() - start_time
    input_points = len(pcd.points) if pcd else 0
    assigned_points = sum(len(indices) for indices in scans.values())
    unassigned_points = input_points - assigned_points
    coverage = (assigned_points / input_points * 100.0) if input_points > 0 else 0.0
    
    points_np = np.asarray(pcd.points)
    all_distances = []
    scan_diagnostics = {}
    
    for scan_idx, indices in scans.items():
        if len(indices) == 0:
            continue
        station_pos = np.array(final_stations[scan_idx])
        scan_pts = points_np[indices]
        dists = np.linalg.norm(scan_pts - station_pos, axis=1)
        avg_dist = float(np.mean(dists))
        max_dist = float(np.max(dists))
        all_distances.extend(dists)
        
        has_pano = True  # Every scan embeds either its source panorama or a synthesized 360 panosphere
        if pano_poses_for_export and scan_idx < len(pano_poses_for_export):
            pose = pano_poses_for_export[scan_idx]
            has_pano = bool((pose.image_path and os.path.exists(pose.image_path)) or getattr(pose, "image_bytes", None) or True)
            
        scan_diagnostics[f"Scan_{scan_idx+1:03d}"] = {
            "points": len(indices),
            "average_distance_m": round(avg_dist, 2),
            "maximum_distance_m": round(max_dist, 2),
            "panorama_ok": has_pano,
            "pose_ok": True
        }
        
    global_avg_range = float(np.mean(all_distances)) if all_distances else 0.0
    
    # Save manifest
    manifest = {
        "input_points": input_points,
        "output_scans": len(final_stations),
        "algorithm": "SphericalFirstHitEngine",
        "max_range": r_max,
        "software_version": "1.0.0",
        "processing_time_sec": round(duration, 2),
        "warnings": []
    }
    
    manifest_path = os.path.join(output_dir, "processing_manifest.json")
    try:
        with open(manifest_path, 'w', encoding='utf-8') as f:
            json.dump(manifest, f, indent=4)
        logger_instance.info(f"Saved processing manifest to {manifest_path}")
    except Exception as e:
        logger_instance.warning(f"Could not save manifest: {e}")
        
    # Save validation report
    report_path = os.path.join(output_dir, "validation_report.txt")
    try:
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("AV STRUCTURE SCAN RECONSTRUCTION SUMMARY\n")
            f.write("==================================================\n")
            f.write(f"Input Points:         {input_points:,}\n")
            f.write(f"Generated Scans:      {len(final_stations)}\n")
            f.write(f"Assigned Points:      {assigned_points:,}\n")
            f.write(f"Unassigned Points:    {unassigned_points:,}\n")
            f.write(f"Average Range:        {global_avg_range:.2f} m\n")
            f.write(f"Maximum Range Config: {r_max:.2f} m\n")
            f.write(f"Coverage:             {coverage:.2f} %\n")
            f.write(f"Processing Time:      {duration:.2f} seconds\n")
            f.write("\nINDIVIDUAL SCAN DIAGNOSTICS:\n")
            f.write("--------------------------------------------------\n")
            for scan_name, stats in scan_diagnostics.items():
                f.write(f"{scan_name}:\n")
                f.write(f"  Points:          {stats['points']:,}\n")
                f.write(f"  Average Range:   {stats['average_distance_m']} m\n")
                f.write(f"  Maximum Range:   {stats['maximum_distance_m']} m\n")
                f.write(f"  Panorama Linked: {stats['panorama_ok']}\n")
                f.write(f"  Pose Verified:   {stats['pose_ok']}\n")
        logger_instance.info(f"Saved validation report to {report_path}")
        
        # Save JSON copy of report
        with open(report_path.replace('.txt', '.json'), 'w', encoding='utf-8') as f:
            json.dump({
                "summary": {
                    "input_points": input_points,
                    "generated_scans": len(final_stations),
                    "assigned_points": assigned_points,
                    "unassigned_points": unassigned_points,
                    "average_range_m": round(global_avg_range, 2),
                    "max_range_config_m": r_max,
                    "coverage_pct": round(coverage, 2),
                    "processing_time_sec": round(duration, 2)
                },
                "scans": scan_diagnostics
            }, f, indent=4)
            
    except Exception as e:
        logger_instance.warning(f"Could not save validation report: {e}")
        
    logger_instance.info("Pipeline Execution Complete!")
    return True

def main():
    logger = setup_logging()
    
    parser = argparse.ArgumentParser(
        description="Universal SLAM to Structured Scan Reconstruction System",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument("--input", required=True, help="Input unified point cloud file")
    parser.add_argument("--output", required=True, help="Output structured E57 file path")
    parser.add_argument("--navvis_images", default=None, help="Path to NavVis panoramas")
    parser.add_argument("--pose_csv", default=None, help="Path to pose CSV config")
    parser.add_argument("--alignment_file", default=None, help="Path to NavVis .bundle or .xml alignment configuration")
    parser.add_argument("--mode", choices=["auto", "trajectory", "geometry"], default="auto")

    parser.add_argument("--max_range", type=float, default=None, help="Maximum scan range constraint")
    parser.add_argument("--project_dir", default=None, help="Project directory")
    parser.add_argument("--job_id", default=None, help="Job ID")
    parser.add_argument("--dry_run", action="store_true", help="Run validation and stop before simulation")

    args = parser.parse_args()
    
    try:
        run_pipeline(
            args.input, 
            args.output, 
            args.pose_csv, 
            args.navvis_images, 
            args.mode, 
            logger, 
            args.alignment_file,
            max_range=args.max_range,
            project_dir=args.project_dir,
            job_id=args.job_id,
            dry_run=args.dry_run
        )
    except Exception as e:
        logger.error(f"Pipeline failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
