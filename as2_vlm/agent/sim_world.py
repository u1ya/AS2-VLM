"""Hidden ground truth for MockAs2Adapter: apartment geometry, LiDAR ray casting and a
synthetic RGB-D camera.

The robot's *beliefs* live in the semantic map; this module is what the mock sensors
observe. Frame: metres, x east, y north, yaw counter-clockwise from +x (REP-103).
"""
import copy
import io
import math
import zlib
from dataclasses import dataclass

CEILING_M = 2.5
DOOR_TOP_M = 2.1


@dataclass
class Door:
    id: str
    label: str
    segment: tuple
    state: str = "open"
    exterior: bool = False

    @property
    def center(self):
        x1, y1, x2, y2 = self.segment
        return (x1 + x2) / 2, (y1 + y2) / 2


@dataclass
class SimObject:
    id: str
    label: str
    category: str
    x: float
    y: float
    z: float                 # centre height
    height: float
    width: float
    color: tuple
    state: str = "normal"
    footprint: tuple = None  # (half_x, half_y) when the object blocks motion
    cue_z: float = None      # height of the visual state cue (knob, lock, display)
    aliases: tuple = ()
    door_id: str = None
    standoff: float = 0.9    # preferred inspection distance from the object centre


ROOMS = {
    "living room": (0.0, 0.0, 6.0, 4.5),
    "kitchen": (6.0, 0.0, 10.0, 4.5),
    "hallway": (0.0, 4.5, 10.0, 5.7),
    "bedroom": (0.0, 5.7, 5.0, 8.0),
    "utility room": (5.0, 5.7, 10.0, 8.0),
}

WALLS = [
    (0, 0, 10, 0), (10, 0, 10, 8), (0, 8, 8.0, 8), (9.0, 8, 10, 8), (0, 0, 0, 8),
    (6, 0, 6, 1.3), (6, 2.4, 6, 4.5),                              # living | kitchen
    (0, 4.5, 2.0, 4.5), (3.2, 4.5, 7.5, 4.5), (8.6, 4.5, 10, 4.5),  # rooms | hallway
    (0, 5.7, 2.6, 5.7), (3.7, 5.7, 6.1, 5.7), (7.2, 5.7, 10, 5.7),  # hallway | rooms
    (5, 5.7, 5, 8),                                                # bedroom | utility
]

DOORS = [
    Door("kitchen_door", "kitchen door", (6.0, 1.3, 6.0, 2.4)),
    Door("hall_kitchen_door", "hallway kitchen door", (7.5, 4.5, 8.6, 4.5)),
    Door("bedroom_door", "bedroom door", (2.6, 5.7, 3.7, 5.7)),
    Door("utility_door", "utility room door", (6.1, 5.7, 7.2, 5.7)),
    Door("back_door", "back door", (8.0, 8.0, 9.0, 8.0), state="locked", exterior=True),
]

OBJECTS = [
    SimObject("dock", "charging dock", "infrastructure", 0.8, 0.8, 0.05, 0.1, 0.6, (40, 140, 90),
              state="idle", aliases=("dock", "charger", "charging station", "home base"), standoff=0.0),
    SimObject("sofa", "sofa", "furniture", 3.0, 0.45, 0.4, 0.8, 2.0, (120, 90, 160),
              footprint=(1.0, 0.4), aliases=("couch",), standoff=1.2),
    SimObject("coffee_table", "coffee table", "furniture", 3.0, 1.6, 0.22, 0.45, 1.0, (150, 110, 70),
              footprint=(0.5, 0.3), aliases=("table",)),
    SimObject("tv", "television", "electronics", 0.12, 2.6, 1.0, 0.65, 1.1, (30, 30, 40),
              state="off", footprint=(0.08, 0.55), cue_z=1.0, aliases=("tv",), standoff=1.2),
    SimObject("extinguisher", "fire extinguisher", "safety", 5.8, 0.3, 0.35, 0.6, 0.22, (210, 30, 30),
              state="normal", footprint=(0.12, 0.12), cue_z=0.55, aliases=("extinguisher",)),
    SimObject("plant", "potted plant", "furniture", 5.5, 4.1, 0.5, 1.0, 0.5, (50, 140, 60),
              footprint=(0.2, 0.2), aliases=("plant",)),
    SimObject("stove", "stove", "appliance", 9.65, 2.0, 0.45, 0.9, 0.8, (200, 200, 205),
              state="off", footprint=(0.35, 0.4), cue_z=0.9, aliases=("cooker", "oven", "cooktop", "hob"),
              standoff=0.95),
    SimObject("fridge", "refrigerator", "appliance", 9.6, 3.9, 0.9, 1.8, 0.8, (235, 235, 240),
              state="closed", footprint=(0.4, 0.35), cue_z=1.0, aliases=("fridge",), standoff=1.1),
    SimObject("sink", "kitchen sink", "appliance", 7.6, 0.3, 0.45, 0.9, 0.8, (170, 180, 190),
              state="off", footprint=(0.4, 0.3), cue_z=0.9, aliases=("sink", "tap", "faucet")),
    SimObject("bed", "bed", "furniture", 1.4, 7.1, 0.3, 0.6, 2.0, (90, 120, 170),
              footprint=(1.0, 0.8), standoff=1.3),
    SimObject("window", "bedroom window", "window", 3.6, 7.97, 1.5, 1.0, 1.2, (170, 210, 240),
              state="closed", cue_z=1.5, aliases=("window",), standoff=1.2),
    SimObject("washer", "washing machine", "appliance", 9.6, 6.4, 0.43, 0.85, 0.6, (225, 225, 230),
              state="off", footprint=(0.3, 0.3), cue_z=0.85, aliases=("washer",)),
]

WALL_TINTS = {"living room": (232, 224, 208), "kitchen": (212, 229, 222), "hallway": (224, 224, 232),
              "bedroom": (227, 217, 233), "utility room": (220, 220, 212)}
DOOR_COLOR = (146, 98, 58)
BOX_COLOR = (186, 140, 86)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def ray_segment(ox, oy, dx, dy, x1, y1, x2, y2):
    """Distance along the ray (o + t*d) to the segment, or None."""
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
    return t if t > 1e-6 and -1e-9 <= u <= 1 + 1e-9 else None


def point_segment_distance(px, py, x1, y1, x2, y2):
    ex, ey = x2 - x1, y2 - y1
    length2 = ex * ex + ey * ey
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / length2))
    return math.hypot(px - (x1 + t * ex), py - (y1 + t * ey))


def rect_edges(cx, cy, hx, hy):
    a, b, c, d = (cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)
    return [(*a, *b), (*b, *c), (*c, *d), (*d, *a)]


class SimWorld:
    def __init__(self):
        self.bounds = (0.0, 0.0, 10.0, 8.0)
        self.rooms = dict(ROOMS)
        self.doors = {d.id: copy.copy(d) for d in DOORS}
        self.objects = {o.id: copy.copy(o) for o in OBJECTS}
        for door in self.doors.values():
            cx, cy = door.center
            inset = -0.03 if door.exterior else 0.0
            self.objects[door.id] = SimObject(door.id, door.label, "door", cx, cy + inset, 1.0, 2.0, 1.05,
                                              DOOR_COLOR, cue_z=1.0, aliases=("door",), door_id=door.id,
                                              standoff=1.0)
        self.boxes = {}
        self._original_doors = {d.id: d.state for d in self.doors.values()}
        self._original_states = {o.id: o.state for o in self.objects.values()}
        self.version = 0
        self._segments = None

    # ---- truth queries -------------------------------------------------------------
    def object_state(self, obj):
        return self.doors[obj.door_id].state if obj.door_id else obj.state

    def _changed(self):
        self.version += 1
        self._segments = None

    def segments(self):
        """(x1, y1, x2, y2, kind, ref, height) of everything that blocks motion or light."""
        if self._segments is None:
            segs = [(*w, "wall", None, CEILING_M) for w in WALLS]
            for door in self.doors.values():
                if door.state != "open":
                    segs.append((*door.segment, "door", door.id, DOOR_TOP_M))
            for obj in self.objects.values():
                if obj.footprint:
                    segs.extend((*e, "furniture", obj.id, obj.z + obj.height / 2)
                                for e in rect_edges(obj.x, obj.y, *obj.footprint))
            for box_id, (bx, by, half) in self.boxes.items():
                segs.extend((*e, "box", box_id, 0.6) for e in rect_edges(bx, by, half, half))
            self._segments = segs
        return self._segments

    def raycast(self, x, y, angle, max_range=8.0, min_height=0.0):
        dx, dy = math.cos(angle), math.sin(angle)
        best, kind, ref = max_range, None, None
        for x1, y1, x2, y2, k, r, h in self.segments():
            if h < min_height:
                continue
            t = ray_segment(x, y, dx, dy, x1, y1, x2, y2)
            if t is not None and t < best:
                best, kind, ref = t, k, r
        return best, kind, ref

    def collides(self, x, y, radius):
        x0, y0, x1, y1 = self.bounds
        if not (x0 + radius <= x <= x1 - radius and y0 + radius <= y <= y1 - radius):
            return ("wall", None)
        for sx1, sy1, sx2, sy2, kind, ref, _ in self.segments():
            if point_segment_distance(x, y, sx1, sy1, sx2, sy2) < radius:
                return (kind, ref)
        return None

    def room_of(self, x, y):
        for name, (x1, y1, x2, y2) in self.rooms.items():
            if x1 <= x <= x2 and y1 <= y <= y2:
                return name
        return None

    def label_for(self, kind, ref):
        if kind == "door":
            return self.doors[ref].label
        if kind == "furniture":
            return self.objects[ref].label
        if kind == "box":
            return "cardboard box"
        return "wall"

    def state_for(self, kind, ref):
        if kind == "door":
            return self.doors[ref].state
        if kind == "box":
            return "blocking"
        return "static"

    # ---- fault injection -----------------------------------------------------------
    def set_door(self, door_id, state):
        if door_id not in self.doors or state not in {"open", "closed", "locked"}:
            raise ValueError("Unknown door or state")
        self.doors[door_id].state = state
        self._changed()

    def add_box(self, x, y, half=0.3):
        box_id = f"box_{len(self.boxes) + 1}"
        self.boxes[box_id] = (x, y, half)
        self._changed()
        return box_id

    def set_object_state(self, obj_id, state):
        self.objects[obj_id].state = state
        self._changed()

    def clear_anomalies(self):
        for door_id, state in self._original_doors.items():
            self.doors[door_id].state = state
        for obj_id, state in self._original_states.items():
            self.objects[obj_id].state = state
        self.boxes.clear()
        self._changed()

    def anomalies(self):
        active = [f"{d.label} {d.state}" for d in self.doors.values() if d.state != self._original_doors[d.id]]
        active += [f"{o.label} {o.state}" for o in self.objects.values()
                   if not o.door_id and o.state != self._original_states[o.id]]
        active += [f"cardboard box at ({x:.1f}, {y:.1f})" for x, y, _ in self.boxes.values()]
        return active

    # ---- prior knowledge handed to the robot ---------------------------------------
    def static_map(self):
        """Floor plan the robot is allowed to know (like a SLAM map): no door states, no boxes."""
        return {
            "bounds": list(self.bounds),
            "rooms": [{"name": n, "bounds": list(b)} for n, b in self.rooms.items()],
            "walls": [list(w) for w in WALLS],
            "doors": [{"id": d.id, "label": d.label, "segment": list(d.segment), "exterior": d.exterior}
                      for d in self.doors.values()],
            "furniture": [{"id": o.id, "label": o.label, "center": [o.x, o.y], "half": list(o.footprint)}
                          for o in self.objects.values() if o.footprint],
        }

    def truth_overlay(self):
        return {"doors": {d.id: d.state for d in self.doors.values()},
                "boxes": [{"id": i, "center": [x, y], "half": h} for i, (x, y, h) in self.boxes.items()],
                "anomalies": self.anomalies()}

    def seed_nodes(self):
        """A slightly inaccurate prior semantic map; states are unknown until observed."""
        nodes = []
        for obj in self.objects.values():
            jitter = zlib.crc32(obj.id.encode())
            dx, dy = (jitter % 17 - 8) / 100, ((jitter >> 8) % 17 - 8) / 100
            extra = {"standoff": obj.standoff}
            if obj.door_id:
                door = self.doors[obj.door_id]
                extra.update(door_id=door.id, segment=list(door.segment), exterior=door.exterior)
                dx = dy = 0.0
            nodes.append({"label": obj.label, "category": obj.category,
                          "coords": [round(obj.x + dx, 2), round(obj.y + dy, 2), obj.z],
                          "confidence": 0.6, "state": "unknown", "aliases": list(obj.aliases),
                          "source": "seed", "extra": extra})
        return nodes

    # ---- sensors -------------------------------------------------------------------
    def scan(self, pose, beams=180, max_range=8.0):
        x, y, _, yaw = pose
        step = 2 * math.pi / beams
        angles = [-math.pi + i * step for i in range(beams)]
        ranges = [self.raycast(x, y, yaw + a, max_range, min_height=0.05)[0] for a in angles]
        return angles, ranges

    def blocking_ahead(self, pose, radius, half_width, max_range=2.5, fan_deg=30):
        """Nearest thing in the central camera fan (what a VLM would see blocking the way)."""
        x, y, _, yaw = pose
        best = None
        for k in range(-6, 7):
            angle = yaw + math.radians(fan_deg) * k / 6
            t, kind, ref = self.raycast(x, y, angle, max_range, 0.05)
            if kind and (best is None or t < best[0] - 0.05 or (abs(t - best[0]) <= 0.05 and kind == "door")):
                best = (t, kind, ref)
        if best is None:
            return None
        t, kind, ref = best
        return {"kind": kind, "ref": ref, "label": self.label_for(kind, ref), "state": self.state_for(kind, ref),
                "distance_m": round(max(0.0, t - radius), 2)}

    def render(self, pose, pitch, camera, draw=True, quality=80):
        """Ray-cast first-person view. Returns (jpeg|None, truth, depth_fn)."""
        x, y, z, yaw = pose
        width, height = int(camera.width), int(camera.height)
        hfov = math.radians(camera.hfov_deg)
        f = (width / 2) / math.tan(hfov / 2)
        off_f, off_l, off_z = camera.offset_xyz
        cx = x + math.cos(yaw) * off_f - math.sin(yaw) * off_l
        cy = y + math.sin(yaw) * off_f + math.cos(yaw) * off_l
        cz = z + off_z
        up = -pitch  # REP-103: negative pitch tilts the nose (and camera) up

        def v_of(h_rel, depth):
            return height / 2 - f * math.tan(math.atan2(h_rel, depth) - up)

        rays = width // 2
        col_w = width / rays
        columns = []
        for i in range(rays):
            a = math.atan((width / 2 - (i + 0.5) * col_w) / f)
            t, kind, ref = self.raycast(cx, cy, yaw + a, 12.0, min_height=1.5)
            columns.append((t * math.cos(a), t, kind, ref, a))

        items = []
        for obj in self.objects.values():
            items.append((obj.id, obj.label, obj.category, self.object_state(obj), obj.x, obj.y, obj.z,
                          obj.height, obj.width, obj.color, obj.cue_z, obj.door_id))
        for box_id, (bx, by, half) in self.boxes.items():
            items.append((box_id, "cardboard box", "obstacle", "blocking", bx, by, 0.3, 0.6, 2 * half,
                          BOX_COLOR, None, None))

        candidates = []
        for item in items:
            oid, label, category, state, ox, oy, oz, oh, ow, color, cue_z, door_id = item
            dx, dy = ox - cx, oy - cy
            fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
            left = -dx * math.sin(yaw) + dy * math.cos(yaw)
            if fwd < 0.15 or fwd > 9:
                continue
            uc = width / 2 - f * left / fwd
            half_px = f * (ow / 2) / fwd
            if uc + half_px < 0 or uc - half_px > width:
                continue
            candidates.append((fwd, uc, half_px, item))
        candidates.sort(key=lambda c: c[0])  # near first, so nearer objects occlude farther ones

        cover = [[] for _ in range(rays)]
        drawn, truth = [], []
        for fwd, uc, half_px, item in candidates:
            oid, label, category, state, ox, oy, oz, oh, ow, color, cue_z, door_id = item
            u1, u2 = uc - half_px, uc + half_px
            top, bot = v_of(oz + oh / 2 - cz, fwd), v_of(oz - oh / 2 - cz, fwd)
            if bot <= 0 or top >= height:
                continue
            span_top, span_bot = max(top, 0), min(bot, height)
            solid = not (door_id and state == "open")

            def hidden(i):
                return any(t <= span_top + 2 and b >= span_bot - 2 for t, b in cover[i])
            visible_cols = [i for i in range(rays)
                            if u1 <= (i + 0.5) * col_w <= u2 and columns[i][0] > fwd - 0.12 and not hidden(i)]
            if not visible_cols:
                continue
            if solid:
                for i in visible_cols:
                    cover[i].append((span_top, span_bot))
            drawn.append((fwd, oid, label, category, state, visible_cols, top, bot, color, door_id))
            frac_h = len(visible_cols) * col_w / max(u2 - u1, 1e-6)
            frac_v = (span_bot - span_top) / max(bot - top, 1e-6)
            visible = min(1.0, frac_h) * max(0.0, frac_v)
            if visible < 0.2 or fwd > 7.5:
                continue
            centre_col = int(min(max(uc / col_w, 0), rays - 1))
            cue_visible = False
            if cue_z is not None and 0 <= uc <= width and centre_col in visible_cols:
                cue_v = v_of(cue_z - cz, fwd)
                cue_visible = 0.04 * height <= cue_v <= 0.96 * height
            bx1 = max(visible_cols[0] * col_w, 0) / width
            bx2 = min((visible_cols[-1] + 1) * col_w, width) / width
            by1, by2 = span_top / height, span_bot / height
            if bx2 - bx1 < 0.005 or by2 - by1 < 0.005:
                continue
            truth.append({"id": oid, "label": label, "category": category, "state": state,
                          "bbox": [round(bx1, 4), round(by1, 4), round(bx2, 4), round(by2, 4)],
                          "visible_fraction": round(visible, 3), "depth_m": round(fwd, 3),
                          "cue_visible": cue_visible})
        drawn.sort(key=lambda d: -d[0])

        def depth_at(u, v):
            px, py = u * width, v * height
            col = int(min(max(px / col_w, 0), rays - 1))
            for fwd, _, _, _, _, cols, top, bot, _, _ in reversed(drawn):  # nearest first
                if col in cols and top <= py <= bot:
                    return fwd
            depth = columns[col][0]
            if py > height / 2 + f * math.tan(up):  # floor below the horizon: intersect the ground plane
                angle = math.atan2(py - height / 2, f) + up
                if angle > 1e-3:
                    depth = min(depth, cz / math.tan(angle))
            return depth if depth < 10 else None

        jpeg = self._draw(width, height, f, up, cx, cy, cz, yaw, columns, col_w, drawn, truth, quality) if draw else None
        return jpeg, truth, depth_at

    def _draw(self, width, height, f, up, cx, cy, cz, yaw, columns, col_w, drawn, truth, quality):
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError:
            return None
        image = Image.new("RGB", (width, height), (214, 218, 222))
        draw = ImageDraw.Draw(image)
        horizon = height / 2 + f * math.tan(up)
        draw.rectangle([0, max(0, horizon), width, height], fill=(176, 168, 154))
        try:
            font = ImageFont.load_default(size=15)
        except TypeError:
            font = ImageFont.load_default()

        def v_of(h_rel, depth):
            return height / 2 - f * math.tan(math.atan2(h_rel, depth) - up)

        def shade(color, distance, factor=1.0):
            k = factor * (0.5 + 0.5 * max(0.0, 1 - distance / 10))
            return tuple(int(c * k) for c in color)

        for i, (depth, t, kind, ref, a) in enumerate(columns):
            if kind is None:
                continue
            hx, hy = cx + math.cos(yaw + a) * (t - 0.05), cy + math.sin(yaw + a) * (t - 0.05)
            tint = WALL_TINTS.get(self.room_of(hx, hy), (225, 225, 225))
            x0, x1 = i * col_w, (i + 1) * col_w
            top, bot = v_of(CEILING_M - cz, depth), v_of(-cz, depth)
            draw.rectangle([x0, top, x1, bot], fill=shade(tint, t))
            if kind == "door":
                door_top = v_of(DOOR_TOP_M - cz, depth)
                draw.rectangle([x0, door_top, x1, bot], fill=shade(DOOR_COLOR, t))
            floor_line = max(0, min(height, bot))
            draw.line([x0, floor_line, x1, floor_line], fill=(120, 112, 100))

        for fwd, oid, label, category, state, cols, top, bot, color, door_id in drawn:
            runs, start = [], cols[0]
            for prev, cur in zip(cols, cols[1:] + [None]):
                if cur != prev + 1:
                    runs.append((start, prev))
                    start = cur
            outline = shade(color, fwd, 0.6)
            for c1, c2 in runs:
                box = [c1 * col_w, top, (c2 + 1) * col_w, bot]
                if door_id and state == "open":
                    draw.rectangle(box, outline=outline, width=max(2, int(12 / fwd)))
                else:
                    draw.rectangle(box, fill=shade(color, fwd), outline=outline, width=2)
            if category == "appliance" and oid == "stove" and state == "on":
                fx = (cols[0] + cols[-1]) / 2 * col_w
                for k in (-1, 1):
                    r = max(4, 40 / fwd)
                    draw.ellipse([fx + k * 2 * r - r, top - r * 0.6, fx + k * 2 * r + r, top + r * 0.6],
                                 fill=(255, 120, 20))

        for item in truth:  # labels only for objects that are actually visible
            label, state = item["label"], item["state"]
            tag = label if state in (None, "normal", "unknown", "static") else f"{label} [{state.upper()}]"
            tx, ty = item["bbox"][0] * width + 3, item["bbox"][1] * height + 3
            draw.rectangle([tx - 2, ty - 1, tx + 8 * len(tag) + 4, ty + 17], fill=(255, 255, 255))
            draw.text((tx, ty), tag, fill=(20, 20, 20), font=font)

        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=quality)
        return buffer.getvalue()
