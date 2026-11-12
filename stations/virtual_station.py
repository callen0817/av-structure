from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
import numpy as np
from io_modules.navvis_parser import PanoPose, normalize_quaternion


@dataclass
class VirtualStation:
    """
    Unified Virtual Scan Station object for AV Structure 2.0.
    Couples station identity, global pose, structured spherical point organization,
    depth/visibility map, associated 360° panorama, and coverage/confidence intelligence.
    """

    station_id: str
    index: int
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    global_shift: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    # Assigned point indices into the source point cloud
    point_indices: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.int64))

    # Structured spherical scan grid representation (in station-local coordinate system)
    grid_rows: int = 0
    grid_cols: int = 0
    angular_res_deg: float = 0.15
    row_indices: Optional[np.ndarray] = None  # uint16
    col_indices: Optional[np.ndarray] = None  # uint16
    invalid_states: Optional[np.ndarray] = None  # int8 (0=valid, 2=invalid)
    spherical_bounds: Optional[Dict[str, float]] = None

    # Panorama association
    pano_pose: Optional[PanoPose] = None
    panorama_source: str = "none"  # 'existing', 'generated_rgb', 'generated_intensity', 'generated_depth', 'none'
    panorama_jpeg_bytes: Optional[bytes] = None
    panorama_width: int = 0
    panorama_height: int = 0
    panorama_coverage_mask: Optional[np.ndarray] = None  # uint8: 2=observed, 1=interpolated, 0=unobserved

    # Depth & confidence intelligence
    depth_map: Optional[np.ndarray] = None
    free_space_confidence: float = 1.0
    is_manual: bool = False
    placement_uncertain: bool = False
    e57_scan_guid: Optional[str] = None
    e57_image_guid: Optional[str] = None
    coverage_metrics: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        self.position = (float(self.position[0]), float(self.position[1]), float(self.position[2]))
        norm_q = normalize_quaternion(self.orientation)
        self.orientation = norm_q if norm_q is not None else (1.0, 0.0, 0.0, 0.0)

    def to_local_coordinates(self, global_points: np.ndarray) -> np.ndarray:
        """
        Transforms global 3D coordinates (N, 3) into this station's local coordinate system
        using float64 subtraction first to preserve precision on large UTM/State Plane coordinates.
        """
        if len(global_points) == 0:
            return np.empty((0, 3), dtype=np.float64)
        st_pos = np.array(self.position, dtype=np.float64)
        rel = np.asarray(global_points, dtype=np.float64) - st_pos
        if self.orientation != (1.0, 0.0, 0.0, 0.0):
            from pyquaternion import Quaternion

            q_inv = Quaternion(self.orientation).inverse
            rel = np.dot(rel, q_inv.rotation_matrix.T)
        return rel

    def to_global_coordinates(self, local_points: np.ndarray) -> np.ndarray:
        """
        Transforms station-local 3D coordinates (N, 3) back into global point-cloud coordinates.
        """
        if len(local_points) == 0:
            return np.empty((0, 3), dtype=np.float64)
        pts = np.asarray(local_points, dtype=np.float64)
        if self.orientation != (1.0, 0.0, 0.0, 0.0):
            from pyquaternion import Quaternion

            q = Quaternion(self.orientation)
            pts = np.dot(pts, q.rotation_matrix.T)
        return pts + np.array(self.position, dtype=np.float64)

    def to_summary_dict(self) -> Dict[str, Any]:
        return {
            "station_id": self.station_id,
            "index": self.index,
            "position": list(self.position),
            "orientation": list(self.orientation),
            "point_count": int(len(self.point_indices)),
            "grid_rows": int(self.grid_rows),
            "grid_cols": int(self.grid_cols),
            "angular_res_deg": round(float(self.angular_res_deg), 4),
            "panorama_source": self.panorama_source,
            "panorama_resolution": [int(self.panorama_width), int(self.panorama_height)],
            "free_space_confidence": round(float(self.free_space_confidence), 3),
            "is_manual": bool(self.is_manual),
            "placement_uncertain": bool(self.placement_uncertain),
            "e57_scan_guid": self.e57_scan_guid,
            "e57_image_guid": self.e57_image_guid,
            "coverage_metrics": self.coverage_metrics,
        }


def build_virtual_stations(
    positions: List[Tuple[float, float, float]],
    scans: Dict[int, np.ndarray],
    pano_poses: Optional[List[PanoPose]] = None,
    global_shift: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    candidate_confidences: Optional[Dict[Tuple[float, float, float], float]] = None,
    manual_positions: Optional[List[Tuple[float, float, float]]] = None,
    placement_uncertain: bool = False,
) -> List[VirtualStation]:
    """
    Constructs a list of unified VirtualStation objects from station positions,
    assigned point indices, and optional PanoPose objects.
    """
    stations: List[VirtualStation] = []
    manual_set = set()
    if manual_positions:
        for mp in manual_positions:
            manual_set.add((round(float(mp[0]), 3), round(float(mp[1]), 3), round(float(mp[2]), 3)))

    for idx, pos in enumerate(positions):
        pos_tuple = (float(pos[0]), float(pos[1]), float(pos[2]))
        pt_indices = scans.get(idx, np.empty(0, dtype=np.int64))

        matched_pose = None
        if pano_poses and len(pano_poses) > 0:
            if len(pano_poses) == len(positions) and pano_poses[idx] is not None:
                matched_pose = pano_poses[idx]
            else:
                st_np = np.array(pos_tuple, dtype=np.float64)
                min_d = float("inf")
                for p in pano_poses:
                    if p is not None and p.position is not None:
                        d = float(np.linalg.norm(st_np - np.array(p.position, dtype=np.float64)))
                        if d < min_d:
                            min_d = d
                            matched_pose = p

        ori = matched_pose.orientation if matched_pose is not None else (1.0, 0.0, 0.0, 0.0)
        conf = 1.0
        if candidate_confidences:
            key = (round(pos_tuple[0], 3), round(pos_tuple[1], 3), round(pos_tuple[2], 3))
            conf = candidate_confidences.get(key, 1.0)

        is_man = (round(pos_tuple[0], 3), round(pos_tuple[1], 3), round(pos_tuple[2], 3)) in manual_set

        st = VirtualStation(
            station_id=f"Scan_{idx:04d}",
            index=idx,
            position=pos_tuple,
            orientation=ori,
            global_shift=global_shift,
            point_indices=np.asarray(pt_indices, dtype=np.int64),
            pano_pose=matched_pose,
            free_space_confidence=conf,
            is_manual=is_man,
            placement_uncertain=placement_uncertain,
        )
        stations.append(st)

    return stations
