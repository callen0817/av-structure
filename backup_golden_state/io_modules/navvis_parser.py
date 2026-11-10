import csv
import logging
from dataclasses import dataclass
from typing import List, Optional

logger = logging.getLogger(__name__)

@dataclass
class PanoPose:
    pano_id: int
    filename: str
    timestamp: float
    position: tuple # (x, y, z)
    orientation: tuple # (w, x, y, z) quaternion
    image_path: Optional[str] = None

class NavVisParser:
    def __init__(self, pose_csv_path: str, images_dir: Optional[str] = None, alignment_file: Optional[str] = None):
        """
        Initializes the parser for NavVis trajectory CSV.
        """
        self.pose_csv_path = pose_csv_path
        self.images_dir = images_dir
        self.alignment_file = alignment_file
        self.R, self.T = self._load_alignment()
        
    def _load_alignment(self):
        if not self.alignment_file:
            return None, None
            
        import os
        if not os.path.exists(self.alignment_file):
            logger.warning(f"Alignment file not found: {self.alignment_file}")
            return None, None
            
        try:
            import json
            import zipfile
            import numpy as np
            
            ext = os.path.splitext(self.alignment_file)[1].lower()
            data = None
            
            if ext in ['.bundle', '.zip']:
                with zipfile.ZipFile(self.alignment_file, 'r') as z:
                    for filename in z.namelist():
                        if filename.endswith('alignment.json'):
                            data = json.loads(z.read(filename).decode('utf-8'))
                            break
            elif ext == '.json':
                with open(self.alignment_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    
            if data:
                # Prefer site_transform (aligned to BIM/Site), fallback to basic transform
                transform = data.get('site_transform') or data.get('transform')
                if transform:
                    R = np.array([
                        [transform.get('r11', 1), transform.get('r12', 0), transform.get('r13', 0)],
                        [transform.get('r21', 0), transform.get('r22', 1), transform.get('r23', 0)],
                        [transform.get('r31', 0), transform.get('r32', 0), transform.get('r33', 1)]
                    ])
                    T = np.array([transform.get('tx', 0), transform.get('ty', 0), transform.get('tz', 0)])
                    logger.info(f"Loaded NavVis site alignment matrix from {self.alignment_file}")
                    return R, T
            
            logger.warning(f"No valid alignment transform found inside {self.alignment_file}")
        except Exception as e:
            logger.error(f"Failed to load alignment file {self.alignment_file}: {e}")
            
        return None, None
        
    def parse(self) -> List[PanoPose]:
        """
        Parses the NavVis pose CSV.
        Returns a list of PanoPose objects, transformed if an alignment file is provided.
        """
        poses = []
        with open(self.pose_csv_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                
                parts = [p.strip() for p in line.split(';')]
                if len(parts) >= 10:
                    try:
                        pano_id = int(parts[0])
                        filename = parts[1]
                        timestamp = float(parts[2])
                        pos = (float(parts[3]), float(parts[4]), float(parts[5]))
                        ori = (float(parts[6]), float(parts[7]), float(parts[8]), float(parts[9]))
                        
                        import numpy as np
                        from pyquaternion import Quaternion
                        
                        # Apply Coordinate Transformation
                        if self.R is not None and self.T is not None:
                            pos_np = np.array(pos)
                            pos_np = np.dot(self.R, pos_np) + self.T
                            pos = (pos_np[0], pos_np[1], pos_np[2])
                            
                            # Apply rotation to the orientation quaternion
                            q_site = Quaternion(matrix=self.R)
                            q_pose = Quaternion(ori)
                            q_new = q_site * q_pose
                            ori = (q_new.w, q_new.x, q_new.y, q_new.z)
                        
                        image_path = None
                        if self.images_dir is not None:
                            import os
                            potential_path = os.path.join(self.images_dir, filename)
                            if os.path.exists(potential_path):
                                image_path = potential_path
                                
                        poses.append(PanoPose(
                            pano_id=pano_id,
                            filename=filename,
                            timestamp=timestamp,
                            position=pos,
                            orientation=ori,
                            image_path=image_path
                        ))
                    except ValueError as e:
                        logger.warning(f"Failed to parse line: {line}. Error: {e}")
        
        logger.info(f"Parsed {len(poses)} NavVis poses.")
        return poses
