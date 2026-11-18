"""Independent physics-contact acceptance; never supplies navigation commands."""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from pxr import PhysicsSchemaTools
from omni.physx import get_physx_simulation_interface

GROUND_COLLIDERS = ('/World/Floor', '/World/Ground', '/World/SyntheticUnevenFloor')


class ContactAudit:
    def __init__(self, folder, beds=()):
        self.folder = Path(folder)
        self.time = 0.
        self.last_saved = -1.
        self.pairs = {}
        self.forbidden = None
        self.beds = list(beds)
        self.foot_contacts = 0
        self.minimum_toe_clearance = float('inf')
        robot = ET.parse(Path(__file__).parents[1]/'assets/as2_so101_fixed_description/as2_so101_fixed.urdf').getroot()
        radii = [float(link.find('collision/geometry/sphere').get('radius'))
                 for link in robot.findall('link') if link.get('name', '').endswith('_foot')]
        if len(radii) != 4:
            raise RuntimeError('Expected four modeled spherical soles for footprint audit')
        self.toe_radius = max(radii)
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self.record)

    def record(self, headers, data):
        for h in headers:
            if not h.num_contact_data:
                continue
            paths = [str(PhysicsSchemaTools.intToSdfPath(h.collider0)),
                     str(PhysicsSchemaTools.intToSdfPath(h.collider1))]
            if all(p.startswith('/World/AS2/') for p in paths):
                continue
            key = ' | '.join(paths)
            pair = self.pairs.setdefault(key, dict(count=0, first_t=self.time,
                last_t=self.time, max_impulse=0., min_separation=1.))
            pair['count'] += 1
            pair['last_t'] = self.time
            for i in range(h.contact_data_offset, h.contact_data_offset+h.num_contact_data):
                point = data[i]
                impulse = float(np.linalg.norm(point.impulse))
                separation = float(point.separation)
                pair['max_impulse'] = max(pair['max_impulse'], impulse)
                pair['min_separation'] = min(pair['min_separation'], separation)
                pair['last_position'] = list(point.position)
                # Contact-report generation distance is not physical penetration.
                actual = impulse > .001 or separation < -.0005
                other = next((p for p in paths if not p.startswith('/World/AS2/')), '')
                if self.time > 3.5 and actual and self.beds and other in GROUND_COLLIDERS and any('_foot/' in p for p in paths):
                    self.foot_contacts += 1
                    x, y = float(point.position[0]), float(point.position[1])
                    distance = min(math.hypot(max(x0-x, 0., x-x1), max(y0-y, 0., y-y1))
                                   for x0, x1, y0, y1 in self.beds)
                    clearance = distance-self.toe_radius
                    self.minimum_toe_clearance = min(self.minimum_toe_clearance, clearance)
                    if clearance < 0:
                        self.forbidden = dict(t=self.time, reason='FOOTPRINT_OVER_BED',
                            pair=paths, toe_clearance_m=clearance, position=list(point.position))
                # Synthetic uneven terrain is a legitimate ground collider. Foot
                # contacts are still footprint-audited above; torso/leg contact
                # with it remains forbidden, as does contact with any structure.
                if self.time > 3.5 and actual and other and other not in GROUND_COLLIDERS:
                    self.forbidden = dict(t=self.time, pair=paths, impulse=impulse,
                                          separation=separation, position=list(point.position))

    def save(self, stamp, force=False):
        self.time = stamp
        if force or stamp-self.last_saved >= 1.:
            self.last_saved = stamp
            (self.folder/'contact_audit.json').write_text(json.dumps(
                dict(forbidden_contact=self.forbidden, pairs=self.pairs,
                     foot_ground_contact_samples=self.foot_contacts,
                     modeled_sole_radius_m=self.toe_radius,
                     minimum_measured_toe_clearance_m=(self.minimum_toe_clearance if math.isfinite(self.minimum_toe_clearance) else None),
                     scope='Rigid stems/beds/structures and foot contacts on flat or synthetic terrain; body-leaf light contacts are filtered by explicit scene model'),
                indent=2), encoding='utf-8')
