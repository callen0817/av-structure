import argparse
import logging
import sys
import os
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
                 overlap: Optional[float] = None, coverage: Optional[float] = None):
    """
    Executes the entire SLAM -> Structured Scan pipeline.
    """
    if logger_instance is None:
        logger_instance = logging.getLogger(__name__)

    logger_instance.info("Starting Universal SLAM -> Structured Scan Pipeline")
    logger_instance.info(f"Input file: {input_cloud_path}")
    logger_instance.info(f"Output file: {output_e57_path}")
    
    # Validation checks
    if not os.path.exists(input_cloud_path):
        logger_instance.error(f"Input file not found: {input_cloud_path}")
        raise FileNotFoundError(f"Input file not found: {input_cloud_path}")

    # Determine execution mode
    mode = mode_pref
    if mode == "auto":
        if pose_csv_path and os.path.exists(pose_csv_path):
            mode = "trajectory"
            logger_instance.info("Auto mode: Found pose_csv, using 'trajectory' mode.")
        else:
            mode = "geometry"
            logger_instance.info("Auto mode: No pose_csv found, falling back to 'geometry' mode.")

    logger_instance.info(f"Using mode: {mode}")

    # Integration of the full processing pipeline
    from io_modules.cloud_loader import CloudLoader
    from io_modules.navvis_parser import NavVisParser
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
        
    # 2. Generate Candidate Stations
    logger_instance.info("Stage 2: Generating Candidate Stations")
    if mode == "trajectory" and pano_poses:
        gen = TrajectoryStationGenerator(target_spacing=target_spacing)
        selected_poses = gen.generate(pano_poses)
        candidates = [p.position for p in selected_poses]
        pano_poses_for_export = selected_poses
    else:
        logger_instance.info("Running pure Geometry-based candidate generation.")
        gen = GeometryStationGenerator(target_spacing=target_spacing)
        candidates = gen.generate(pcd)
        pano_poses_for_export = None
        
    if not candidates:
        logger_instance.error("Failed to generate any candidate scan stations. Exiting.")
        raise RuntimeError("Failed to generate any candidate scan stations.")
        
    # 3. Optimize Visibility
    logger_instance.info("Stage 3: Optimizing Visibility")
    optimizer = VisibilityOptimizer(min_overlap=min_overlap, target_coverage=target_coverage)
    final_stations = optimizer.optimize(pcd, candidates)
    
    if pano_poses_for_export:
        final_pano_poses = []
        for st in final_stations:
            idx = candidates.index(st)
            final_pano_poses.append(pano_poses_for_export[idx])
        pano_poses_for_export = final_pano_poses
        
    # 4. Simulate Scans
    logger_instance.info("Stage 4: Simulating Scans")
    simulator = ScannerSimulator()
    scans = simulator.simulate_scans(pcd, final_stations)
    
    # 5. Export
    logger_instance.info(f"Stage 5: Exporting Structured Data to {output_e57_path}")
    exporter = E57Exporter(output_e57_path)
        
    exporter.export(pcd, scans, final_stations, pano_poses_for_export, extra_data)
    
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

    args = parser.parse_args()
    
    try:
        run_pipeline(args.input, args.output, args.pose_csv, args.navvis_images, args.mode, logger, args.alignment_file)
    except Exception as e:
        logger.error(f"Pipeline failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
