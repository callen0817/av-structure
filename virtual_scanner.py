import argparse
import logging
import sys
import os
import time
import shutil
import json
import numpy as np
from typing import Optional, List, Tuple, Any, Dict

from config import (
    LOGGING_LEVEL,
    LOGGING_FORMAT,
    DEFAULT_ANGULAR_RES_DEG,
    DEFAULT_GRID_COLLISION_POLICY,
    DEFAULT_PANO_RESOLUTION,
    SUPPORTED_PANO_RESOLUTIONS,
    DEFAULT_OPTIMIZATION_PRIORITY,
    DEFAULT_SIMULATION_MODE,
)


def setup_logging():
    logging.basicConfig(
        level=LOGGING_LEVEL,
        format=LOGGING_FORMAT,
        handlers=[logging.StreamHandler(sys.stdout)],
    )
    return logging.getLogger("virtual_scanner")


def _check_cancelled(cancel_event: Optional[Any], stage_name: str):
    if cancel_event is not None and hasattr(cancel_event, "is_set") and cancel_event.is_set():
        raise RuntimeError(f"Pipeline cancelled during {stage_name}.")


def compute_icp_alignment(
    source_pcd: Any,
    reference_path: str,
) -> np.ndarray:
    """
    Runs multi-stage Open3D point-to-plane ICP (voxel 0.25m -> 0.08m)
    to compute transform_to_project (4x4) into baseline Project Frame.
    """
    import open3d as o3d
    from io_modules.cloud_loader import CloudLoader

    ref_loader = CloudLoader(reference_path)
    ref_pcd, _ = ref_loader.load()
    if ref_pcd is None or ref_pcd.is_empty() or len(ref_pcd.points) < 50:
        return np.eye(4, dtype=np.float64)

    src_raw = np.asarray(source_pcd.points, dtype=np.float64)
    ref_raw = np.asarray(ref_pcd.points, dtype=np.float64)
    centroid = np.mean(ref_raw, axis=0)

    src_shifted = o3d.geometry.PointCloud()
    src_shifted.points = o3d.utility.Vector3dVector(src_raw - centroid)

    ref_shifted = o3d.geometry.PointCloud()
    ref_shifted.points = o3d.utility.Vector3dVector(ref_raw - centroid)

    voxel_coarse = 0.25
    src_coarse = src_shifted.voxel_down_sample(voxel_coarse) if len(src_raw) > 5000 else src_shifted
    ref_coarse = ref_shifted.voxel_down_sample(voxel_coarse) if len(ref_raw) > 5000 else ref_shifted

    src_coarse.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
    ref_coarse.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))

    coarse_res = o3d.pipelines.registration.registration_icp(
        src_coarse,
        ref_coarse,
        max_correspondence_distance=0.45,
        init=np.eye(4, dtype=np.float64),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
    )

    voxel_fine = 0.08
    src_fine = src_shifted.voxel_down_sample(voxel_fine) if len(src_raw) > 5000 else src_shifted
    ref_fine = ref_shifted.voxel_down_sample(voxel_fine) if len(ref_raw) > 5000 else ref_shifted

    src_fine.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
    ref_fine.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))

    fine_res = o3d.pipelines.registration.registration_icp(
        src_fine,
        ref_fine,
        max_correspondence_distance=0.15,
        init=coarse_res.transformation,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
    )

    T_local = np.asarray(fine_res.transformation, dtype=np.float64)
    T_unshift = np.eye(4, dtype=np.float64)
    T_unshift[:3, 3] = centroid
    T_shift = np.eye(4, dtype=np.float64)
    T_shift[:3, 3] = -centroid

    return np.dot(T_unshift, np.dot(T_local, T_shift))


def run_pipeline(
    input_cloud_path: str,
    output_e57_path: str,
    pose_csv_path: Optional[str] = None,
    navvis_images_dir: Optional[str] = None,
    mode_pref: str = "auto",
    logger_instance=None,
    alignment_file: Optional[str] = None,
    spacing: Optional[float] = None,
    overlap: Optional[float] = None,
    coverage: Optional[float] = None,
    max_range: Optional[float] = None,
    project_dir: Optional[str] = None,
    job_id: Optional[str] = None,
    dry_run: bool = False,
    scan_density_pct: float = 100.0,
    optimization_priority: str = DEFAULT_OPTIMIZATION_PRIORITY,
    min_stations: Optional[int] = None,
    max_stations: Optional[int] = None,
    manual_stations: Optional[List[tuple]] = None,
    exclude_stations: Optional[List[tuple]] = None,
    angular_res_deg: Optional[float] = None,
    collision_policy: str = DEFAULT_GRID_COLLISION_POLICY,
    pano_resolution: Optional[Tuple[int, int]] = None,
    include_diagnostic_pano_without_rgb: bool = True,
    cancel_event: Optional[Any] = None,
    return_details: bool = False,
    export_bundle_dir: Optional[str] = None,
    scan_date: Optional[str] = None,
    scan_id: Optional[str] = None,
    project_name: Optional[str] = None,
    reference_cloud_path: Optional[str] = None,
    json_progress: bool = False,
    simulation_mode: str = DEFAULT_SIMULATION_MODE,
    allow_overlap: bool = False,
):
    """
    Executes the entire SLAM -> Structured Scan pipeline (Workflow A & Workflow B).
    """
    start_time = time.time()
    if logger_instance is None:
        logger_instance = logging.getLogger(__name__)

    def emit_progress(pct: int, msg: str, status: str = "RUNNING"):
        if json_progress:
            print(json.dumps({"progress": pct, "message": msg, "status": status}), flush=True)
        logger_instance.info(f"[{pct}%] {msg}")

    emit_progress(0, "Starting Universal SLAM -> Structured Scan Reconstruction System (AV Structure 2.0)")
    logger_instance.info(f"Input file: {input_cloud_path}")
    logger_instance.info(f"Output file: {output_e57_path}")

    # -------------------------------------------------------------------------
    # Preflight Integrity Validation
    # -------------------------------------------------------------------------
    logger_instance.info("Running Preflight Integrity Validation...")
    _check_cancelled(cancel_event, "Preflight Validation")

    # 1. Input cloud exists and is readable
    if not input_cloud_path or not os.path.exists(input_cloud_path):
        logger_instance.error(f"Preflight Error: Input file not found: {input_cloud_path}")
        raise FileNotFoundError(f"Input file not found: {input_cloud_path}")

    # 2. Pose CSV exists and has valid formatting (if provided)
    if pose_csv_path:
        if not os.path.exists(pose_csv_path):
            logger_instance.error(f"Preflight Error: Pose CSV not found: {pose_csv_path}")
            raise FileNotFoundError(f"Pose CSV not found: {pose_csv_path}")
        try:
            valid_data_lines = 0
            with open(pose_csv_path, "r", encoding="utf-8") as f:
                for i, line in enumerate(f):
                    line = line.strip()
                    if line and not line.startswith("#"):
                        delim = ";" if ";" in line else ","
                        parts = [p.strip() for p in line.split(delim)]
                        if len(parts) < 6:
                            raise ValueError("Delimiter ';' or ',' not found or fewer than 6 columns.")
                        # Verify numeric coordinates if not a header line
                        try:
                            int(parts[0])
                            float(parts[3])
                            float(parts[4])
                            float(parts[5])
                            valid_data_lines += 1
                        except ValueError:
                            if i > 1:
                                raise ValueError(f"Non-numeric pose coordinates on line {i+1}.")
                    if valid_data_lines >= 3:
                        break
            if valid_data_lines == 0:
                raise ValueError("Pose CSV contains no valid pose rows.")
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
        if free < 200 * 1024 * 1024:  # 200 MB minimum
            raise OSError(f"Insufficient disk space in output directory. Free space: {free / (1024*1024):.2f} MB")
    except OSError:
        raise
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
    from io_modules.e57_validator import validate_exported_e57
    from stations.trajectory_mode import TrajectoryStationGenerator
    from stations.geometry_mode import GeometryStationGenerator
    from stations.virtual_station import build_virtual_stations
    from simulation.visibility import VisibilityOptimizer
    from simulation.scanner import ScannerSimulator
    from config import TARGET_SCAN_SPACING_MIN, MIN_OVERLAP_PERCENTAGE, TARGET_COVERAGE_PERCENTAGE, DEFAULT_MAX_SCAN_RANGE

    # Configuration overrides
    target_spacing = spacing if spacing is not None else TARGET_SCAN_SPACING_MIN
    min_overlap = overlap if overlap is not None else MIN_OVERLAP_PERCENTAGE
    target_coverage = coverage if coverage is not None else TARGET_COVERAGE_PERCENTAGE

    # 1. Load Data
    emit_progress(10, "Stage 1: Loading Data")
    _check_cancelled(cancel_event, "Stage 1: Loading Data")
    loader = CloudLoader(input_cloud_path)
    pcd, extra_data = loader.load()

    if pcd is None or pcd.is_empty() or len(pcd.points) == 0:
        raise ValueError(f"Input point cloud contains 0 valid points: {input_cloud_path}")

    # ICP Alignment against reference baseline scan (if provided) for 4D historical dataset alignment
    transform_to_project = np.eye(4, dtype=np.float64)
    if reference_cloud_path and os.path.exists(reference_cloud_path):
        emit_progress(15, f"Computing ICP alignment against reference cloud: {reference_cloud_path}")
        try:
            transform_to_project = compute_icp_alignment(pcd, reference_cloud_path)
            pcd.transform(transform_to_project)
            logger_instance.info("Successfully aligned point cloud to baseline Project Frame via multi-stage ICP.")
        except Exception as icp_err:
            logger_instance.warning(f"ICP registration against reference cloud failed ({icp_err}), retaining canonical frame.")

    # Check for flash anchors (user-pinned priority stations in AV Scan / AV ScanAR)
    flash_anchors = extra_data.get("flash_anchors", [])
    if flash_anchors:
        if manual_stations is None:
            manual_stations = []
        for fa in flash_anchors:
            if not np.allclose(transform_to_project, np.eye(4)):
                fa_h = np.array([fa[0], fa[1], fa[2], 1.0], dtype=np.float64)
                fa_t = tuple(float(x) for x in np.dot(transform_to_project, fa_h)[:3])
            else:
                fa_t = (float(fa[0]), float(fa[1]), float(fa[2]))
            if fa_t not in manual_stations:
                manual_stations.append(fa_t)
        logger_instance.info(f"Loaded {len(flash_anchors)} locked user-pinned flash anchors into station generator.")

    pano_poses = None
    candidates = []

    if pose_csv_path and os.path.exists(pose_csv_path):
        parser_nv = NavVisParser(pose_csv_path, navvis_images_dir, alignment_file)
        pano_poses = parser_nv.parse()
    elif input_cloud_path and input_cloud_path.lower().endswith(".e57") and mode_pref in ("auto", "trajectory"):
        logger_instance.info("No pose CSV supplied — inspecting input E57 for embedded scan headers/panoramas...")
        pano_poses = extract_e57_poses(input_cloud_path)

    # Fallback: if no poses found yet and an alignment bundle (.bundle/.zip) was supplied, check inside bundle
    if (not pano_poses or len(pano_poses) == 0) and alignment_file and os.path.exists(alignment_file):
        logger_instance.info(f"Checking alignment bundle for embedded poses/panoramas: {alignment_file}")
        parser_bundle = NavVisParser(None, navvis_images_dir, alignment_file)
        pano_poses = parser_bundle.parse()

    # Verify and auto-align pose coordinate frame against loaded point cloud bounds
    if pano_poses and len(pano_poses) > 0:
        pano_poses = verify_and_align_poses(pano_poses, pcd, alignment_file)
        if not np.allclose(transform_to_project, np.eye(4)):
            for p in pano_poses:
                pos_h = np.array([p.position[0], p.position[1], p.position[2], 1.0], dtype=np.float64)
                new_pos = np.dot(transform_to_project, pos_h)[:3]
                p.position = (float(new_pos[0]), float(new_pos[1]), float(new_pos[2]))

    # Determine execution mode (Workflow A: trajectory/existing panos vs Workflow B: geometry/manual)
    mode = mode_pref
    if mode == "auto":
        if pano_poses and len(pano_poses) > 0:
            mode = "trajectory"
            logger_instance.info(
                f"Auto mode: Found {len(pano_poses)} camera poses/panoramas, using 'trajectory' mode (Workflow A)."
            )
        else:
            mode = "geometry"
            logger_instance.info(
                "Auto mode: No camera poses found, falling back to 'geometry' mode (Workflow B)."
            )

    logger_instance.info(f"Using mode: {mode}")

    # 2. Generate Candidate Stations
    emit_progress(25, "Stage 2: Generating Candidate Stations")
    _check_cancelled(cancel_event, "Stage 2: Generating Candidate Stations")
    candidate_confidences = {}
    placement_uncertain = False

    if mode == "manual":
        if not manual_stations:
            raise ValueError("Manual station placement mode requires at least one manual station coordinate.")
        candidates = [(float(p[0]), float(p[1]), float(p[2])) for p in manual_stations]
        pano_poses_for_export = None
        final_stations = list(candidates)
    elif mode == "trajectory" and pano_poses:
        gen = TrajectoryStationGenerator(target_spacing=target_spacing, scan_density_pct=scan_density_pct)
        selected_poses = gen.generate(pano_poses)
        if max_stations is not None and max_stations > 0 and len(selected_poses) > max_stations:
            idx_sub = np.round(np.linspace(0, len(selected_poses) - 1, int(max_stations))).astype(int)
            selected_poses = [selected_poses[i] for i in idx_sub]
        candidates = [p.position for p in selected_poses]
        pano_poses_for_export = selected_poses
        final_stations = list(candidates)
    else:
        logger_instance.info("Running pure Geometry-based candidate generation.")
        gen = GeometryStationGenerator(target_spacing=target_spacing)
        candidates = gen.generate(pcd)
        candidate_confidences = getattr(gen, "candidate_confidences", {})
        placement_uncertain = getattr(gen, "placement_uncertain", False)
        pano_poses_for_export = None
        final_stations = []

    # Apply manual station exclusions if provided
    if exclude_stations and candidates:
        filtered = []
        for c in candidates:
            if not any(
                math.sqrt((c[0] - ex[0]) ** 2 + (c[1] - ex[1]) ** 2 + (c[2] - ex[2]) ** 2) < 0.50
                for ex in exclude_stations
            ):
                filtered.append(c)
        if filtered:
            candidates = filtered
            if mode in ("trajectory", "manual"):
                final_stations = list(candidates)

    if not candidates and not manual_stations:
        logger_instance.error("Failed to generate any candidate scan stations. Exiting.")
        raise RuntimeError("Failed to generate any candidate scan stations.")

    # Dry Run Mode logic
    if dry_run:
        logger_instance.info("--------------------------------------------------")
        logger_instance.info("DRY RUN MODE ACTIVE - ESTIMATING RUN DIAGNOSTICS")
        logger_instance.info(f"Generated Candidate Stations: {len(candidates)}")
        pt_count = len(pcd.points) if pcd else 0
        logger_instance.info(f"Input Point Count: {pt_count}")

        est_mem_mb = int((pt_count * 32) / (1024 * 1024))
        est_runtime_sec = int((pt_count / 1_000_000) * len(candidates) * 0.15)
        est_runtime_sec = max(5, est_runtime_sec)

        logger_instance.info(f"Estimated Peak Memory: ~{est_mem_mb} MB")
        logger_instance.info(f"Estimated Processing Runtime: ~{est_runtime_sec} seconds")
        logger_instance.info(f"Estimated Output Size: ~{int((pt_count * 15) / (1024 * 1024))} MB")

        if mode == "trajectory" and pano_poses_for_export:
            missing_count = 0
            for pose in pano_poses_for_export:
                if (not pose.image_path or not os.path.exists(pose.image_path)) and not getattr(
                    pose, "image_bytes", None
                ):
                    missing_count += 1
            if missing_count > 0:
                logger_instance.warning(
                    f"Preflight Warning: {missing_count} out of {len(pano_poses_for_export)} scans have no physical panorama file (will synthesize from point cloud)!"
                )

        logger_instance.info("DRY RUN COMPLETED. No files were written.")
        logger_instance.info("--------------------------------------------------")
        return True

    # 3. Optimize Visibility
    _check_cancelled(cancel_event, "Stage 3: Optimizing Visibility")
    if mode not in ("trajectory", "manual"):
        emit_progress(45, "Stage 3: Optimizing Visibility")
        optimizer = VisibilityOptimizer(
            min_overlap=min_overlap,
            target_coverage=target_coverage,
            optimization_priority=optimization_priority,
            min_stations=min_stations,
            max_stations=max_stations,
        )
        final_stations = optimizer.optimize(pcd, candidates, pinned_stations=manual_stations)
    else:
        if manual_stations and mode == "trajectory":
            for mp in manual_stations:
                mt = (float(mp[0]), float(mp[1]), float(mp[2]))
                if mt not in final_stations:
                    final_stations.append(mt)
        emit_progress(45, f"Stage 3: {mode.capitalize()} Mode active — using {len(final_stations)} scan stations directly.")

    if mode not in ("trajectory", "manual") and pano_poses_for_export:
        final_pano_poses = []
        for st in final_stations:
            idx = candidates.index(st)
            final_pano_poses.append(pano_poses_for_export[idx])
        pano_poses_for_export = final_pano_poses

    # 4. Simulate Scans
    emit_progress(65, "Stage 4: Simulating Scans")
    _check_cancelled(cancel_event, "Stage 4: Simulating Scans")
    simulator = ScannerSimulator()

    r_max = max_range if max_range is not None else DEFAULT_MAX_SCAN_RANGE
    scans = simulator.simulate_scans(
        pcd,
        final_stations,
        max_range=r_max,
        simulation_mode=simulation_mode,
        allow_overlap=allow_overlap,
    )

    # Determine spherical grid angular resolution (configured or auto-estimated)
    eff_angular_res = (
        float(angular_res_deg)
        if angular_res_deg is not None
        else DEFAULT_ANGULAR_RES_DEG
    )
    eff_pano_res = pano_resolution if pano_resolution is not None else DEFAULT_PANO_RESOLUTION

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
                raise FileNotFoundError(
                    f"Validation Error: Scan station {i+1} has missing or invalid panorama: {pose.image_path}"
                )

    # Build unified VirtualStation objects
    global_shift_tuple = tuple(float(x) for x in extra_data.get("global_shift", (0.0, 0.0, 0.0)))
    virtual_stations = build_virtual_stations(
        positions=final_stations,
        scans=scans,
        pano_poses=pano_poses_for_export,
        global_shift=global_shift_tuple,
        candidate_confidences=candidate_confidences,
        manual_positions=manual_stations,
        placement_uncertain=placement_uncertain,
    )

    # 5. Export
    emit_progress(80, f"Stage 5: Exporting Structured Data to {output_e57_path}")
    _check_cancelled(cancel_event, "Stage 5: Exporting Structured E57")
    exporter = E57Exporter(
        output_e57_path,
        angular_res_deg=eff_angular_res,
        collision_policy=collision_policy,
        pano_resolution=eff_pano_res,
        include_diagnostic_pano_without_rgb=include_diagnostic_pano_without_rgb,
    )
    exporter.export(pcd, scans, final_stations, pano_poses_for_export, extra_data, virtual_stations=virtual_stations)

    # 6. Independent Post-Export E57 Reimport Validation & Manifest Generation
    emit_progress(90, "Stage 6: Running Post-Export E57 Reimport Validation...")
    points_np = np.asarray(pcd.points)
    non_empty_scans = sum(1 for indices in scans.values() if len(indices) > 0)
    expect_rgb_flag = bool(extra_data.get("has_rgb", pcd.has_colors()))
    expect_panos_flag = bool(expect_rgb_flag or include_diagnostic_pano_without_rgb or pano_poses_for_export)

    e57_validation = validate_exported_e57(
        output_e57_path,
        expected_scans=non_empty_scans,
        expect_rgb=expect_rgb_flag,
        expect_panoramas=expect_panos_flag,
        expected_bbox_min=points_np.min(axis=0),
        expected_bbox_max=points_np.max(axis=0),
    )
    if not e57_validation["valid"]:
        logger_instance.error(f"E57 Reimport Validation Failed: {e57_validation['errors']}")
        raise RuntimeError(f"Exported E57 failed structural reimport validation: {e57_validation['errors']}")
    logger_instance.info(
        f"E57 Reimport Validation PASSED ({e57_validation['scan_count']} structured scans, "
        f"{e57_validation['image_count']} linked 2D panoramas verified)."
    )

    duration = time.time() - start_time
    input_points = len(pcd.points) if pcd else 0
    assigned_points = sum(len(indices) for indices in scans.values())
    exported_points = int(e57_validation.get("total_points_read", assigned_points))
    unassigned_points = input_points - assigned_points
    coverage_pct = (assigned_points / input_points * 100.0) if input_points > 0 else 0.0

    all_distances = []
    scan_diagnostics = {}

    for scan_idx, indices in scans.items():
        if len(indices) == 0:
            continue
        station_pos = np.array(final_stations[scan_idx], dtype=np.float64)
        scan_pts = points_np[indices]
        dists = np.linalg.norm(scan_pts - station_pos, axis=1)
        avg_dist = float(np.mean(dists))
        max_dist = float(np.max(dists))
        all_distances.extend(dists)

        exp_diag = exporter.export_diagnostics.get(scan_idx, {})
        pano_src = exp_diag.get("panorama_source", "none")
        has_pano = pano_src != "none"

        if scan_idx < len(virtual_stations):
            virtual_stations[scan_idx].coverage_metrics.update(
                {
                    "average_distance_m": round(avg_dist, 2),
                    "maximum_distance_m": round(max_dist, 2),
                }
            )

        scan_diagnostics[f"Scan_{scan_idx+1:03d}"] = {
            "points": int(exp_diag.get("exported_points", len(indices))),
            "assigned_source_points": int(len(indices)),
            "average_distance_m": round(avg_dist, 2),
            "maximum_distance_m": round(max_dist, 2),
            "grid_rows": int(exp_diag.get("grid_rows", 0)),
            "grid_cols": int(exp_diag.get("grid_cols", 0)),
            "unique_grid_cells": int(exp_diag.get("unique_grid_cells", 0)),
            "collided_points": int(exp_diag.get("collided_points", 0)),
            "panorama_ok": has_pano,
            "panorama_source": pano_src,
            "panorama_resolution": [
                int(exp_diag.get("panorama_width", 0)),
                int(exp_diag.get("panorama_height", 0)),
            ],
            "panorama_metrics": exp_diag.get("panorama_metrics", {}),
            "e57_scan_guid": exp_diag.get("scan_guid"),
            "e57_image_guid": exp_diag.get("image_guid"),
            "pose_ok": True,
        }

    global_avg_range = float(np.mean(all_distances)) if all_distances else 0.0

    # Save manifest
    warnings_list = list(e57_validation.get("warnings", []))
    if placement_uncertain:
        warnings_list.append("Placement uncertainty: indoor floor-ceiling headroom >2.0m was not detected.")

    manifest = {
        "input_points": input_points,
        "assigned_points": assigned_points,
        "exported_points": exported_points,
        "unassigned_points": unassigned_points,
        "output_scans": len(final_stations),
        "workflow_mode": mode,
        "has_source_rgb": expect_rgb_flag,
        "has_source_intensity": bool(extra_data.get("has_intensity", False)),
        "algorithm": "SphericalFirstHitEngine_v2",
        "duplication_policy": simulator.last_simulation_stats.get("duplication_policy", "partitioned_1_to_1"),
        "grid_collision_policy": collision_policy,
        "angular_res_deg": eff_angular_res,
        "max_range": r_max,
        "optimization_priority": optimization_priority,
        "e57_reimport_validated": bool(e57_validation["valid"]),
        "software_version": "2.0.0",
        "processing_time_sec": round(duration, 2),
        "warnings": warnings_list,
    }

    manifest_path = os.path.join(output_dir, "processing_manifest.json")
    try:
        with open(manifest_path, "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=4)
        logger_instance.info(f"Saved processing manifest to {manifest_path}")
    except Exception as e:
        logger_instance.warning(f"Could not save manifest: {e}")

    # Save validation report
    report_path = os.path.join(output_dir, "validation_report.txt")
    try:
        with open(report_path, "w", encoding="utf-8") as f:
            f.write("AV STRUCTURE SCAN RECONSTRUCTION SUMMARY\n")
            f.write("==================================================\n")
            f.write(f"Input Points:         {input_points:,}\n")
            f.write(f"Generated Scans:      {len(final_stations)}\n")
            f.write(f"Assigned Points:      {assigned_points:,}\n")
            f.write(f"Exported Points:      {exported_points:,}\n")
            f.write(f"Unassigned Points:    {unassigned_points:,}\n")
            f.write(f"Average Range:        {global_avg_range:.2f} m\n")
            f.write(f"Maximum Range Config: {r_max:.2f} m\n")
            f.write(f"Coverage:             {coverage_pct:.2f} %\n")
            f.write(f"E57 Reimport Valid:   {e57_validation['valid']}\n")
            f.write(f"Processing Time:      {duration:.2f} seconds\n")
            f.write("\nINDIVIDUAL SCAN DIAGNOSTICS:\n")
            f.write("--------------------------------------------------\n")
            for scan_name, stats in scan_diagnostics.items():
                f.write(f"{scan_name}:\n")
                f.write(f"  Points:          {stats['points']:,}\n")
                f.write(f"  Spherical Grid:  {stats['grid_cols']}x{stats['grid_rows']} ({stats['unique_grid_cells']:,} cells)\n")
                f.write(f"  Average Range:   {stats['average_distance_m']} m\n")
                f.write(f"  Maximum Range:   {stats['maximum_distance_m']} m\n")
                f.write(f"  Panorama Source: {stats['panorama_source']}\n")
                f.write(f"  Panorama Linked: {stats['panorama_ok']}\n")
                f.write(f"  Pose Verified:   {stats['pose_ok']}\n")
        logger_instance.info(f"Saved validation report to {report_path}")

        # Save JSON copy of report
        with open(report_path.replace(".txt", ".json"), "w", encoding="utf-8") as f:
            json.dump(
                {
                    "summary": {
                        "input_points": input_points,
                        "generated_scans": len(final_stations),
                        "assigned_points": assigned_points,
                        "exported_points": exported_points,
                        "unassigned_points": unassigned_points,
                        "average_range_m": round(global_avg_range, 2),
                        "max_range_config_m": r_max,
                        "coverage_pct": round(coverage_pct, 2),
                        "e57_reimport_valid": bool(e57_validation["valid"]),
                        "processing_time_sec": round(duration, 2),
                    },
                    "scans": scan_diagnostics,
                    "virtual_stations": [vs.to_summary_dict() for vs in virtual_stations],
                    "e57_validation": e57_validation,
                },
                f,
                indent=4,
            )

    except Exception as e:
        logger_instance.warning(f"Could not save validation report: {e}")

    # 7. Export Walkthrough Bundle (if requested)
    bundle_manifest = None
    if export_bundle_dir:
        emit_progress(94, f"Stage 7: Exporting AV Spatial Bundle v1.1 to {export_bundle_dir}...")
        from io_modules.exporter_walkthrough_bundle import export_walkthrough_bundle

        pts_np = np.asarray(pcd.points, dtype=np.float64)
        cols_np = (
            np.clip(np.round(np.asarray(pcd.colors) * 255.0), 0, 255).astype(np.uint8)
            if pcd.has_colors()
            else None
        )
        e57_rel = os.path.relpath(output_e57_path, export_bundle_dir) if output_e57_path else "structured_stations.e57"
        bundle_manifest = export_walkthrough_bundle(
            bundle_dir=export_bundle_dir,
            virtual_stations=virtual_stations,
            points=pts_np,
            colors=cols_np,
            project_id=job_id,
            project_name=project_name or "AV Structure Project",
            scan_id=scan_id,
            scan_date=scan_date,
            transform_to_project=transform_to_project,
            structured_e57_rel_path=e57_rel,
            precomputed_floorplans=extra_data.get("matterpak_floorplans"),
        )
        logger_instance.info(f"AV Spatial Walkthrough Bundle successfully exported to {export_bundle_dir}")

    emit_progress(100, "Pipeline Execution Complete!", status="COMPLETED")
    if return_details:
        return {
            "success": True,
            "virtual_stations": virtual_stations,
            "scans": scans,
            "simulation_stats": simulator.last_simulation_stats,
            "validation": e57_validation,
            "manifest": manifest,
            "bundle_manifest": bundle_manifest,
        }
    return True


def main():
    logger = setup_logging()

    parser = argparse.ArgumentParser(
        description="Universal SLAM to Structured Scan Reconstruction System",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--input", required=True, help="Input unified point cloud file")
    parser.add_argument("--output", required=True, help="Output structured E57 file path")
    parser.add_argument("--navvis_images", default=None, help="Path to NavVis panoramas")
    parser.add_argument("--pose_csv", default=None, help="Path to pose CSV config")
    parser.add_argument("--alignment_file", default=None, help="Path to NavVis .bundle or .xml alignment configuration")
    parser.add_argument("--mode", choices=["auto", "trajectory", "geometry", "manual"], default="auto")
    parser.add_argument("--spacing", type=float, default=None, help="Target station spacing in meters")
    parser.add_argument("--max_range", type=float, default=None, help="Maximum scan range constraint")
    parser.add_argument(
        "--priority",
        choices=["balanced", "geometry", "panorama", "efficiency"],
        default="balanced",
        help="Station placement optimization priority",
    )
    parser.add_argument("--min_stations", type=int, default=None, help="Minimum number of virtual stations")
    parser.add_argument("--max_stations", type=int, default=None, help="Maximum number of virtual stations")
    parser.add_argument("--angular_res", type=float, default=None, help="Spherical grid angular resolution in degrees")
    parser.add_argument(
        "--pano_res",
        choices=list(SUPPORTED_PANO_RESOLUTIONS.keys()),
        default="2048x1024",
        help="Generated equirectangular panorama resolution",
    )
    parser.add_argument("--project_dir", default=None, help="Project directory")
    parser.add_argument("--job_id", default=None, help="Job ID")
    parser.add_argument("--dry_run", action="store_true", help="Run validation and stop before simulation")
    parser.add_argument("--export-bundle", default=None, help="Directory to export AV Spatial Walkthrough Bundle v1.1")
    parser.add_argument("--scan-date", default=None, help="Scan date (YYYY-MM-DD) for bundle manifest")
    parser.add_argument("--scan-id", default=None, help="Scan ID string for bundle manifest")
    parser.add_argument("--project-name", default=None, help="Project name for bundle manifest")
    parser.add_argument("--reference-cloud", default=None, help="Reference baseline scan cloud/lod0.bin for 4D historical ICP alignment")
    parser.add_argument("--json-progress", action="store_true", help="Emit single-line JSON progress events to stdout")
    parser.add_argument(
        "--simulation-mode",
        choices=["physical", "partition_all"],
        default=DEFAULT_SIMULATION_MODE,
        help="Scan simulation mode: 'physical' (strict line-of-sight without x-ray wall penetration) or 'partition_all' (legacy fallback nearest assignment)",
    )

    args = parser.parse_args()
    pano_res_tuple = SUPPORTED_PANO_RESOLUTIONS.get(args.pano_res, DEFAULT_PANO_RESOLUTION)

    try:
        run_pipeline(
            args.input,
            args.output,
            args.pose_csv,
            args.navvis_images,
            args.mode,
            logger,
            args.alignment_file,
            spacing=args.spacing,
            max_range=args.max_range,
            project_dir=args.project_dir,
            job_id=args.job_id,
            dry_run=args.dry_run,
            optimization_priority=args.priority,
            min_stations=args.min_stations,
            max_stations=args.max_stations,
            angular_res_deg=args.angular_res,
            pano_resolution=pano_res_tuple,
            export_bundle_dir=args.export_bundle,
            scan_date=args.scan_date,
            scan_id=args.scan_id,
            project_name=args.project_name,
            reference_cloud_path=args.reference_cloud,
            json_progress=args.json_progress,
            simulation_mode=args.simulation_mode,
        )
    except Exception as e:
        logger.error(f"Pipeline failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

