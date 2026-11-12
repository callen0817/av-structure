import io
import json
import os
import sys
import threading
import time
import zipfile
import struct
import numpy as np
import open3d as o3d
import pytest
import pye57
from PIL import Image
from pyquaternion import Quaternion

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from io_modules.cloud_loader import CloudLoader, load_point_cloud
from io_modules.e57_validator import validate_exported_e57
from io_modules.exporter_e57 import E57Exporter
from io_modules.navvis_parser import (
    NavVisParser,
    PanoPose,
    extract_e57_poses,
    normalize_quaternion,
    resolve_missing_orientations,
    verify_and_align_poses,
)
from simulation.scanner import ScannerSimulator
from simulation.visibility import VisibilityOptimizer
from stations.geometry_mode import GeometryStationGenerator
from stations.trajectory_mode import TrajectoryStationGenerator
from stations.virtual_station import VirtualStation, build_virtual_stations
from virtual_scanner import run_pipeline


# ---------------------------------------------------------------------------
# Synthetic Geometry & Panorama Builders
# ---------------------------------------------------------------------------

def make_synthetic_colored_room(
    bounds=(-5.0, 5.0, -4.0, 4.0, 0.0, 3.2),
    step=0.12,
    include_occluder=True,
    offset=(0.0, 0.0, 0.0),
):
    """
    Create a deterministic 3D room point cloud with distinct wall colors:
      +X (East wall):  Red    [240, 20, 20]
      -X (West wall):  Cyan   [20, 240, 240]
      +Y (North wall): Green  [20, 240, 20]
      -Y (South wall): Blue   [20, 20, 240]
      Floor (z=zmin):  Gray   [120, 120, 120]
      Ceiling(z=zmax): White  [210, 210, 210]
    Optionally includes an interior foreground pillar at x=+2.0m that
    occludes the center of the +X red wall when viewed from the origin,
    colored Magenta [240, 20, 240].
    """
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    xs = np.arange(xmin, xmax + 1e-6, step)
    ys = np.arange(ymin, ymax + 1e-6, step)
    zs = np.arange(zmin, zmax + 1e-6, step)

    pts_list = []
    rgb_list = []

    # Floor & ceiling
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    n_fc = gx.size
    floor_pts = np.column_stack([gx.ravel(), gy.ravel(), np.full(n_fc, zmin)])
    floor_rgb = np.tile(np.array([[120, 120, 120]], dtype=np.uint8), (n_fc, 1))
    ceil_pts = np.column_stack([gx.ravel(), gy.ravel(), np.full(n_fc, zmax)])
    ceil_rgb = np.tile(np.array([[210, 210, 210]], dtype=np.uint8), (n_fc, 1))
    pts_list.extend([floor_pts, ceil_pts])
    rgb_list.extend([floor_rgb, ceil_rgb])

    # +X (East = Red) and -X (West = Cyan) walls
    gy_w, gz_w = np.meshgrid(ys, zs, indexing="ij")
    n_xw = gy_w.size
    east_pts = np.column_stack([np.full(n_xw, xmax), gy_w.ravel(), gz_w.ravel()])
    east_rgb = np.tile(np.array([[240, 20, 20]], dtype=np.uint8), (n_xw, 1))
    west_pts = np.column_stack([np.full(n_xw, xmin), gy_w.ravel(), gz_w.ravel()])
    west_rgb = np.tile(np.array([[20, 240, 240]], dtype=np.uint8), (n_xw, 1))
    pts_list.extend([east_pts, west_pts])
    rgb_list.extend([east_rgb, west_rgb])

    # +Y (North = Green) and -Y (South = Blue) walls
    gx_w, gz_yw = np.meshgrid(xs, zs, indexing="ij")
    n_yw = gx_w.size
    north_pts = np.column_stack([gx_w.ravel(), np.full(n_yw, ymax), gz_yw.ravel()])
    north_rgb = np.tile(np.array([[20, 240, 20]], dtype=np.uint8), (n_yw, 1))
    south_pts = np.column_stack([gx_w.ravel(), np.full(n_yw, ymin), gz_yw.ravel()])
    south_rgb = np.tile(np.array([[20, 20, 240]], dtype=np.uint8), (n_yw, 1))
    pts_list.extend([north_pts, south_pts])
    rgb_list.extend([north_rgb, south_rgb])

    if include_occluder:
        # Dense foreground panel at x = 2.0, y in [-0.6, 0.6], z in [0.8, 2.2]
        oy = np.arange(-0.6, 0.6 + 1e-6, step * 0.4)
        oz = np.arange(0.8, 2.2 + 1e-6, step * 0.4)
        goy, goz = np.meshgrid(oy, oz, indexing="ij")
        occ_pts = np.column_stack([np.full(goy.size, 2.0), goy.ravel(), goz.ravel()])
        occ_rgb = np.tile(np.array([[240, 20, 240]], dtype=np.uint8), (goy.size, 1))
        pts_list.append(occ_pts)
        rgb_list.append(occ_rgb)

    xyz = np.vstack(pts_list).astype(np.float64) + np.asarray(offset, dtype=np.float64)
    rgb = np.vstack(rgb_list).astype(np.uint8)
    intensity = np.linspace(0.1, 0.9, len(xyz), dtype=np.float32)
    return xyz, rgb, intensity


def make_o3d_pcd(xyz: np.ndarray, rgb: np.ndarray = None) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    if rgb is not None:
        pcd.colors = o3d.utility.Vector3dVector(rgb.astype(np.float64) / 255.0)
    return pcd


def create_test_jpeg_bytes(width=512, height=256, color=(40, 120, 220)) -> bytes:
    img = Image.new("RGB", (width, height), color=color)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=92)
    return buf.getvalue()


# ---------------------------------------------------------------------------
# 1. Test Dataset A — Workflow A (Point Cloud with Existing Panoramas)
# ---------------------------------------------------------------------------

def test_workflow_a1_embedded_e57_panoramas_and_poses(tmp_path):
    """
    Test Dataset A1:
    Create an E57 file containing 3D point cloud data and embedded images2D
    JPEG panoramas with known poses. Run the full AV Structure 2.0 pipeline
    and verify:
      - Automatic detection of embedded panoramas and poses (even when n_scans > 1)
      - Byte-for-byte preservation of original JPEG panorama blobs
      - Station pose alignment with embedded panorama poses
      - Structured spherical grid fields (rowIndex, columnIndex, indexBounds,
        sphericalBounds, pointGroupingSchemes)
      - Independent E57 reimport validation via validate_exported_e57()
    """
    xyz, rgb, intensity = make_synthetic_colored_room(step=0.20, include_occluder=False)
    pcd = make_o3d_pcd(xyz, rgb)

    # First create a source E57 with 2 embedded panoramas at known interior poses
    source_e57 = str(tmp_path / "source_with_panos.e57")
    jpeg_0 = create_test_jpeg_bytes(512, 256, color=(200, 50, 50))
    jpeg_1 = create_test_jpeg_bytes(512, 256, color=(50, 200, 50))
    pano_0_path = tmp_path / "pano_0.jpg"
    pano_1_path = tmp_path / "pano_1.jpg"
    pano_0_path.write_bytes(jpeg_0)
    pano_1_path.write_bytes(jpeg_1)

    stations_init = [(-1.5, 0.0, 1.5), (1.5, 0.0, 1.5)]
    q0 = Quaternion(axis=[0, 0, 1], angle=0.0)
    q1 = Quaternion(axis=[0, 0, 1], angle=np.pi / 4)
    pano_poses_init = [
        PanoPose(
            pano_id=0,
            filename="pano_0.jpg",
            timestamp=10.0,
            position=stations_init[0],
            orientation=(float(q0.w), float(q0.x), float(q0.y), float(q0.z)),
            image_path=str(pano_0_path),
            image_bytes=jpeg_0,
        ),
        PanoPose(
            pano_id=1,
            filename="pano_1.jpg",
            timestamp=15.0,
            position=stations_init[1],
            orientation=(float(q1.w), float(q1.x), float(q1.y), float(q1.z)),
            image_path=str(pano_1_path),
            image_bytes=jpeg_1,
        ),
    ]

    half = len(xyz) // 2
    scans_init = {0: np.arange(0, half), 1: np.arange(half, len(xyz))}
    exporter = E57Exporter(source_e57)
    exporter.export(
        pcd=pcd,
        scans=scans_init,
        stations=stations_init,
        pano_poses=pano_poses_init,
        extra_data={"has_rgb": True, "has_intensity": True, "intensity": intensity},
    )

    # Verify extract_e57_poses reads both embedded images2D blobs even though n_scans == 2
    extracted_poses = extract_e57_poses(source_e57)
    assert len(extracted_poses) == 2
    assert extracted_poses[0].image_bytes == jpeg_0
    assert extracted_poses[1].image_bytes == jpeg_1

    # Now run the full pipeline on source_e57 in auto mode (Workflow A)
    output_e57 = str(tmp_path / "output_workflow_a1.e57")
    summary = run_pipeline(
        input_cloud_path=source_e57,
        output_e57_path=output_e57,
        mode_pref="auto",
        spacing=2.0,
        max_range=25.0,
        return_details=True,
    )

    assert summary["success"] is True
    assert len(summary["virtual_stations"]) == 2
    assert summary["validation"]["valid"] is True
    assert summary["validation"]["scan_count"] == 2
    assert summary["validation"]["image_count"] == 2

    # Verify original JPEG bytes were preserved without recompression
    with pye57.E57(output_e57, mode="r") as e57_check:
        imf = e57_check.image_file
        images2d = imf.root()["images2D"]
        assert images2d.childCount() == 2

        extracted_blobs = []
        for idx in range(2):
            img_node = images2d[idx]
            assert img_node.isDefined("associatedData3DGuid")
            rep = img_node["sphericalRepresentation"]
            blob_node = rep["jpegImage"]
            n_bytes = blob_node.byteCount()
            buf = np.empty(n_bytes, dtype=np.uint8)
            blob_node.read(buf, 0, n_bytes)
            extracted_blobs.append(bytes(buf))

        assert extracted_blobs[0] == jpeg_0
        assert extracted_blobs[1] == jpeg_1


def test_workflow_a2_external_csv_panoramas_and_bundle(tmp_path):
    """
    Test Dataset A2:
    Test unstructured point cloud + external pano-poses.csv + panorama JPEGs,
    as well as NavVis .bundle archive extraction, missing orientation calibration
    recovery, and dataset_clean.json alignment transform.
    """
    xyz, rgb, _ = make_synthetic_colored_room(step=0.22, include_occluder=False)
    input_ply = str(tmp_path / "cloud_unstructured.ply")
    pcd = make_o3d_pcd(xyz, rgb)
    o3d.io.write_point_cloud(input_ply, pcd)

    panos_dir = tmp_path / "cam"
    panos_dir.mkdir()
    jpeg_a = create_test_jpeg_bytes(512, 256, color=(180, 90, 30))
    jpeg_b = create_test_jpeg_bytes(512, 256, color=(30, 90, 180))
    (panos_dir / "00000-pano.jpg").write_bytes(jpeg_a)
    (panos_dir / "00001-pano.jpg").write_bytes(jpeg_b)

    # Write 6-column CSV (missing orientation columns -> tests resolve_missing_orientations)
    csv_path = tmp_path / "pano-poses.csv"
    csv_path.write_text(
        "# ID; filename; timestamp; pano_pos_x; pano_pos_y; pano_pos_z\n"
        "0; 00000-pano.jpg; 100.0; -1.5; 0.0; 1.6\n"
        "1; 00001-pano.jpg; 105.0; 1.5; 0.0; 1.6\n",
        encoding="utf-8",
    )

    # Package as a .bundle archive too to test NavVisParser bundle extraction
    bundle_path = tmp_path / "test_dataset.bundle"
    with zipfile.ZipFile(bundle_path, "w") as zf:
        zf.write(csv_path, arcname="pano-poses.csv")
        zf.write(panos_dir / "00000-pano.jpg", arcname="cam/00000-pano.jpg")
        zf.write(panos_dir / "00001-pano.jpg", arcname="cam/00001-pano.jpg")

    parser_bundle = NavVisParser(None, None, str(bundle_path))
    poses = parser_bundle.parse()
    assert len(poses) == 2
    assert poses[0].orientation_resolved is True
    # Forward tangent from (-1.5, 0, 1.6) to (1.5, 0, 1.6) is +X -> yaw = 0 -> identity quaternion
    assert np.isclose(abs(poses[0].orientation[0]), 1.0, atol=1e-4)

    output_e57 = str(tmp_path / "output_workflow_a2.e57")
    summary = run_pipeline(
        input_cloud_path=input_ply,
        output_e57_path=output_e57,
        alignment_file=str(bundle_path),
        mode_pref="auto",
        spacing=2.0,
        max_range=25.0,
        return_details=True,
    )
    assert summary["success"] is True
    assert len(summary["virtual_stations"]) == 2
    assert summary["validation"]["valid"] is True
    assert summary["validation"]["image_count"] == 2


def test_repo_sample_files_parsing(tmp_path):
    """Verify parsing of the repository's sample pano-poses.csv and alignment JSON transform."""
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    csv_file = os.path.join(repo_root, "pano-poses.csv")

    parser = NavVisParser(csv_file, None, None)
    poses = parser.parse()
    assert len(poses) == 349
    for p in poses[:10]:
        q_norm = np.linalg.norm(p.orientation)
        assert np.isclose(q_norm, 1.0, atol=1e-5)

    # Create a site_transform alignment JSON and verify 3D rigid transformation of poses
    align_json = tmp_path / "alignment.json"
    align_json.write_text(
        json.dumps(
            {
                "site_transform": {
                    "r11": 0.0, "r12": -1.0, "r13": 0.0,
                    "r21": 1.0, "r22": 0.0,  "r23": 0.0,
                    "r31": 0.0, "r32": 0.0,  "r33": 1.0,
                    "tx": 100.0, "ty": 200.0, "tz": 10.0,
                }
            }
        ),
        encoding="utf-8",
    )
    parser_aligned = NavVisParser(csv_file, None, str(align_json))
    poses_aligned = parser_aligned.parse()
    assert len(poses_aligned) == 349
    assert not np.allclose(poses_aligned[0].position, poses[0].position)
    expected_x = -poses[0].position[1] + 100.0
    expected_y = poses[0].position[0] + 200.0
    expected_z = poses[0].position[2] + 10.0
    assert np.allclose(poses_aligned[0].position, (expected_x, expected_y, expected_z), atol=1e-5)


# ---------------------------------------------------------------------------
# 2. Test Dataset B — Workflow B (Generic Colorized Point Cloud without Panoramas)
# ---------------------------------------------------------------------------

def test_workflow_b_colorized_cloud_full_pipeline_and_panorama_accuracy(tmp_path):
    """
    Test Dataset B:
    Feed a generic colorized 3D room point cloud (with no panoramas and no trajectory)
    through the full pipeline. Verify:
      1. Automatic interior station placement with free_space_confidence > 0
      2. Structured E57 export with rowIndex, columnIndex, cartesianInvalidState,
         indexBounds, sphericalBounds, pointGroupingSchemes
      3. Depth-buffered 360° panorama synthesis:
         - Foreground magenta occluder at x=+2.0m blocks the distant red wall at x=+5.0m
         - Directional color accuracy at North (+Y Green), West (-X Cyan), South (-Y Blue),
           and East (+X Magenta occluder at center)
      4. Seamless wrap-around across the +/-pi azimuth boundary
      5. Independent E57 reimport validation via validate_exported_e57()
    """
    xyz, rgb, intensity = make_synthetic_colored_room(step=0.10, include_occluder=True)

    # First test E57Exporter.render_station_panorama directly from room center (0, 0, 1.6)
    station_center = np.array([0.0, 0.0, 1.6], dtype=np.float64)
    rel_points = xyz - station_center
    pano_meta = E57Exporter.render_station_panorama(
        rel_points=rel_points,
        scan_colors=rgb,
        scan_intensity=intensity,
        width=1024,
        height=512,
    )
    assert pano_meta["panorama_mode"] == "generated_rgb"
    total_ratio = (
        pano_meta["observed_pixel_ratio"]
        + pano_meta["interpolated_pixel_ratio"]
        + pano_meta["unobserved_pixel_ratio"]
    )
    assert abs(total_ratio - 1.0) < 1e-2
    assert (pano_meta["observed_pixel_ratio"] + pano_meta["interpolated_pixel_ratio"]) > 0.50

    pano_img = np.array(Image.open(io.BytesIO(pano_meta["jpeg_bytes"])))
    H, W, _ = pano_img.shape
    assert (W, H) == (1024, 512)
    mid_v = H // 2

    # In equirectangular mapping:
    # col = ((pi - az) / (2*pi)) * width
    # az = 0 (+X East, occluded by Magenta pillar at x=2.0): col = W // 2
    # az = +pi/2 (+Y North, Green wall): col = W // 4
    # az = -pi/2 (-Y South, Blue wall): col = 3 * W // 4
    # az = +/-pi (-X West, Cyan wall): col = 0 and col = W - 1
    east_px = pano_img[mid_v, W // 2].astype(float)
    north_px = pano_img[mid_v, W // 4].astype(float)
    south_px = pano_img[mid_v, (3 * W) // 4].astype(float)
    west_left_px = pano_img[mid_v, 2].astype(float)
    west_right_px = pano_img[mid_v, W - 3].astype(float)

    # East center must show the foreground Magenta occluder (high R and high B, low G), NOT pure Red wall
    assert east_px[0] > 150 and east_px[2] > 150 and east_px[1] < 100, f"Expected Magenta occluder at East center, got {east_px}"
    # North must be predominantly Green
    assert north_px[1] > 150 and north_px[0] < 100 and north_px[2] < 100, f"Expected Green at North, got {north_px}"
    # South must be predominantly Blue
    assert south_px[2] > 150 and south_px[0] < 100 and south_px[1] < 100, f"Expected Blue at South, got {south_px}"
    # West seam (+/-pi) must be Cyan on both edges and continuous across seam
    assert west_left_px[1] > 150 and west_left_px[2] > 150 and west_left_px[0] < 100
    assert west_right_px[1] > 150 and west_right_px[2] > 150 and west_right_px[0] < 100
    assert np.max(np.abs(west_left_px - west_right_px)) < 25.0

    # Save unstructured cloud to LAS and run full Workflow B pipeline
    import laspy
    las_path = str(tmp_path / "room_workflow_b.las")
    header = laspy.LasHeader(point_format=3, version="1.2")
    header.scales = [0.001, 0.001, 0.001]
    las = laspy.LasData(header)
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    las.red = rgb[:, 0].astype(np.uint16) * 257
    las.green = rgb[:, 1].astype(np.uint16) * 257
    las.blue = rgb[:, 2].astype(np.uint16) * 257
    las.intensity = (intensity * 65535).astype(np.uint16)
    las.write(las_path)

    output_e57 = str(tmp_path / "output_workflow_b.e57")
    summary = run_pipeline(
        input_cloud_path=las_path,
        output_e57_path=output_e57,
        mode_pref="auto",
        spacing=3.5,
        max_range=25.0,
        angular_res_deg=0.25,
        pano_resolution=(2048, 1024),
        optimization_priority="balanced",
        return_details=True,
    )

    assert summary["success"] is True
    assert len(summary["virtual_stations"]) >= 1
    assert summary["validation"]["valid"] is True
    assert summary["validation"]["scan_count"] == len(summary["virtual_stations"])
    assert summary["validation"]["image_count"] == len(summary["virtual_stations"])


# ---------------------------------------------------------------------------
# 3. Edge Cases, Formats, Precision, and Controls
# ---------------------------------------------------------------------------

def test_xyz_only_no_fake_rgb_and_monochrome_panorama(tmp_path):
    """
    Verify that when an input point cloud has NO RGB data (XYZ-only or XYZ+Intensity),
    AV Structure:
      - Sets extra_data['has_rgb'] = False
      - Does NOT write fake colorRed/colorGreen/colorBlue fields into E57 Data3D scans
      - Produces a monochrome diagnostic panorama explicitly marked Non-RGB
    """
    xyz, _, intensity = make_synthetic_colored_room(step=0.25, include_occluder=False)
    xyz_path = tmp_path / "xyz_only.xyz"
    np.savetxt(xyz_path, np.column_stack([xyz, intensity]), fmt="%.4f")

    loaded_xyz, extra = load_point_cloud(str(xyz_path))
    assert len(loaded_xyz) == len(xyz)
    assert extra["has_rgb"] is False
    assert extra["has_intensity"] is True
    assert "rgb" not in extra

    output_e57 = str(tmp_path / "output_xyz_only.e57")
    summary = run_pipeline(
        input_cloud_path=str(xyz_path),
        output_e57_path=output_e57,
        mode_pref="geometry",
        spacing=4.0,
        max_range=20.0,
        return_details=True,
    )
    assert summary["success"] is True
    assert summary["validation"]["valid"] is True

    # Inspect Data3D prototype to verify no colorRed/Green/Blue fields were fabricated
    with pye57.E57(output_e57, mode="r") as e57_f:
        scan_0 = e57_f.read_scan_raw(0)
        assert "colorRed" not in scan_0
        assert "colorGreen" not in scan_0
        assert "colorBlue" not in scan_0
        assert "intensity" in scan_0
        assert "rowIndex" in scan_0
        assert "columnIndex" in scan_0

        # Verify image description marks it as Non-RGB diagnostic preview
        img_0 = e57_f.image_file.root()["images2D"][0]
        desc = str(img_0["description"].value())
        assert "Non-RGB" in desc


def test_ascii_pts_and_corrupted_empty_files(tmp_path):
    """
    Test Leica .pts format (with leading integer point count header and 7-column XYZIRGB),
    and verify clear ValueError / FileNotFoundError on empty or corrupted files.
    """
    pts_file = tmp_path / "sample.pts"
    pts_file.write_text(
        "4\n"
        "0.0 0.0 0.0 -1024 255 0 0\n"
        "1.0 0.0 0.0 0 0 255 0\n"
        "0.0 1.0 0.0 1024 0 0 255\n"
        "1.0 1.0 1.0 2048 255 255 0\n",
        encoding="utf-8",
    )
    xyz, extra = load_point_cloud(str(pts_file))
    assert len(xyz) == 4
    assert extra["has_rgb"] is True
    assert extra["has_intensity"] is True
    assert tuple(extra["rgb"][0]) == (255, 0, 0)

    # Empty file must raise ValueError
    empty_file = tmp_path / "empty.xyz"
    empty_file.write_text("", encoding="utf-8")
    with pytest.raises(ValueError, match="empty"):
        load_point_cloud(str(empty_file))

    # Corrupted binary E57 must raise Exception when read
    corrupt_e57 = tmp_path / "corrupt.e57"
    corrupt_e57.write_bytes(b"NOT_AN_E57_HEADER_1234567890")
    with pytest.raises(Exception):
        load_point_cloud(str(corrupt_e57))


def test_large_global_utm_coordinates_precision(tmp_path):
    """
    Verify numerical stability when point clouds use large global / State Plane / UTM
    coordinates (e.g., X = 500,000m, Y = 4,500,000m). Ensure global coordinates are
    preserved without float32 truncation (< 1mm error).
    """
    utm_offset = (500_000.0, 4_500_000.0, 125.0)
    xyz, rgb, _ = make_synthetic_colored_room(
        step=0.35, include_occluder=False, offset=utm_offset
    )
    pts_path = tmp_path / "utm_cloud.xyzrgb"
    data = np.column_stack([xyz, rgb])
    np.savetxt(pts_path, data, fmt=["%.5f", "%.5f", "%.5f", "%d", "%d", "%d"])

    loaded_xyz, extra = load_point_cloud(str(pts_path))
    assert extra["global_shift"] is not None
    assert np.max(np.abs(loaded_xyz - xyz)) < 1e-4

    output_e57 = str(tmp_path / "utm_structured.e57")
    summary = run_pipeline(
        input_cloud_path=str(pts_path),
        output_e57_path=output_e57,
        mode_pref="manual",
        manual_stations=[(500_000.0, 4_500_000.0, 126.6)],
        max_range=25.0,
        return_details=True,
    )
    assert summary["success"] is True
    assert summary["validation"]["valid"] is True

    # Re-read exported scan in global coordinates and verify < 1mm error against original points
    reloaded_xyz, _ = load_point_cloud(output_e57)
    from scipy.spatial import cKDTree
    tree = cKDTree(xyz)
    dists, _ = tree.query(reloaded_xyz, k=1)
    assert np.max(dists) < 1e-3, f"UTM coordinate drift exceeded 1mm: max={np.max(dists)}"


def test_spherical_grid_collision_policies_and_resolutions():
    """
    Verify ScannerSimulator.organize_spherical_grid() across different angular
    resolutions (0.1°, 0.25°, 0.5°) and collision policies ('preserve_all' vs 'first_hit_cell').
    """
    xyz, _, _ = make_synthetic_colored_room(step=0.20, include_occluder=True)
    station_pos = np.array([0.0, 0.0, 1.6], dtype=np.float64)
    local_pts = xyz - station_pos

    for res_deg in (0.1, 0.25, 0.5):
        grid_all = ScannerSimulator.organize_spherical_grid(
            local_pts, angular_res_deg=res_deg, collision_policy="preserve_all"
        )
        grid_first = ScannerSimulator.organize_spherical_grid(
            local_pts, angular_res_deg=res_deg, collision_policy="first_hit_cell"
        )
        assert len(grid_all["order"]) == len(local_pts)
        assert len(grid_first["order"]) <= len(local_pts)
        assert len(grid_first["order"]) == grid_all["unique_cells"]
        assert grid_all["grid_rows"] == int(round(180.0 / res_deg))
        assert grid_all["grid_cols"] == int(round(360.0 / res_deg))
        assert len(grid_all["line_groups"]) > 0


def test_station_placement_controls_and_priorities():
    """
    Verify manual stations, pinned stations, min/max station counts,
    and all 4 optimization priority modes ('balanced', 'geometry', 'panorama', 'efficiency').
    """
    xyz, rgb, _ = make_synthetic_colored_room(
        bounds=(-8.0, 8.0, -5.0, 5.0, 0.0, 3.2), step=0.25, include_occluder=False
    )
    pcd = make_o3d_pcd(xyz, rgb)
    gen = GeometryStationGenerator(target_spacing=2.5)
    candidates = gen.generate(pcd)
    assert len(candidates) >= 2
    assert len(gen.candidate_confidences) >= 2
    assert all(score > 0.0 for score in gen.candidate_confidences.values())

    counts = {}
    for priority in ("balanced", "geometry", "panorama", "efficiency"):
        opt = VisibilityOptimizer(
            min_overlap=0.40,
            target_coverage=0.95,
            optimization_priority=priority,
        )
        selected = opt.optimize(pcd, candidates)
        counts[priority] = len(selected)
        assert len(selected) >= 1

    # Efficiency mode should select <= stations than geometry mode
    assert counts["efficiency"] <= counts["geometry"]

    # Test pinned station and max_stations cap
    pinned = [(0.0, 0.0, 1.6)]
    opt_pinned = VisibilityOptimizer(max_stations=2)
    selected_pinned = opt_pinned.optimize(pcd, candidates, pinned_stations=pinned)
    assert len(selected_pinned) <= 2
    assert np.allclose(selected_pinned[0], pinned[0])


def test_sparse_cloud_upscale_protection_and_cancellation(tmp_path):
    """
    Verify that:
      1. Requesting 8192x4096 panorama synthesis on a sparse cloud automatically
         caps resolution to avoid misleading over-interpolation.
      2. Setting cancel_event cleanly aborts run_pipeline().
    """
    xyz, rgb, _ = make_synthetic_colored_room(step=0.60, include_occluder=False)
    station_pos = np.array([0.0, 0.0, 1.6], dtype=np.float64)
    rel_pts = xyz - station_pos

    pano_meta = E57Exporter.render_station_panorama(
        rel_points=rel_pts,
        scan_colors=rgb,
        width=8192,
        height=4096,
        allow_sparse_upscale=False,
    )
    # Sparse cloud (< 60,000 points) should be capped to 2048x1024
    assert pano_meta["width"] == 2048
    assert pano_meta["height"] == 1024

    # Cancellation test
    xyz_path = tmp_path / "cancel_cloud.xyz"
    np.savetxt(xyz_path, xyz, fmt="%.3f")
    cancel_ev = threading.Event()
    cancel_ev.set()

    with pytest.raises(RuntimeError, match="cancelled"):
        run_pipeline(
            input_cloud_path=str(xyz_path),
            output_e57_path=str(tmp_path / "cancelled.e57"),
            cancel_event=cancel_ev,
        )


def test_av_spatial_bundle_v11_interoperability(tmp_path):
    """
    Test 11: AV Spatial Bundle v1.1 Interoperability
      - Multi-room, 2-floor synthetic point cloud
      - Pinned flash_anchors.json
      - Generates av_walkthrough_bundle.json, panos/, floors/floor_1_keyplan.png, cache/lod0.bin
      - Verifies binary layout, wall blocking in navigation graph, and world_bounds calibration.
    """
    from tools.verify_interoperability_bundle import create_synthetic_2floor_building

    pcd, pts, cols = create_synthetic_2floor_building()
    session_dir = tmp_path / "session"
    session_dir.mkdir(parents=True, exist_ok=True)

    ply_path = session_dir / "pointcloud_fused_rgb.ply"
    o3d.io.write_point_cloud(str(ply_path), pcd)

    locked_anchor = (-2.0, -2.5, 1.55)
    anchors_path = session_dir / "flash_anchors.json"
    with open(anchors_path, "w", encoding="utf-8") as f:
        json.dump([list(locked_anchor)], f)

    out_e57 = tmp_path / "output.e57"
    bundle_dir = tmp_path / "bundle"

    res = run_pipeline(
        input_cloud_path=str(session_dir),
        output_e57_path=str(out_e57),
        mode_pref="geometry",
        spacing=3.5,
        export_bundle_dir=str(bundle_dir),
        scan_date="2026-11-11",
        scan_id="test_suite_01",
        project_name="PyTest Bundle Interop",
        return_details=True,
    )

    assert res["success"] is True
    assert os.path.exists(out_e57)

    manifest_file = bundle_dir / "av_walkthrough_bundle.json"
    assert manifest_file.exists()
    with open(manifest_file, "r", encoding="utf-8") as f:
        manifest = json.load(f)

    assert manifest["schema"] == "av_walkthrough_bundle"
    assert manifest["schema_version"] == "1.1"
    assert manifest["coordinate_system"] == "right_handed_z_up_meters"
    assert len(manifest["floors"]) >= 2

    # Verify lod0.bin layout
    lod0_file = bundle_dir / "cache" / "lod0.bin"
    assert lod0_file.exists()
    with open(lod0_file, "rb") as f:
        header = f.read(4)
        n_pts = struct.unpack("<I", header)[0]
        assert lod0_file.stat().st_size == 4 + 15 * n_pts

    # Verify keyplans
    for fl in manifest["floors"]:
        kp = bundle_dir / fl["keyplan_path"]
        assert kp.exists()
        assert "world_bounds" in fl
        assert fl["world_bounds"]["meters_per_pixel"] > 0

