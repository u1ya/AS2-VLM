"""3D semantic scene graph: persistent memory of rooms, objects, doors and hazards.

VLM detections (normalized bbox) + depth + the robot pose at capture time are projected
into world coordinates and fused with existing nodes. Storage is SQLite (one JSON row per
node) mirrored in memory; edges (in_room, near) are derived on demand.
"""
import json
import math
import re
import sqlite3
import threading
import time
from pathlib import Path

from .models import SemanticNode

STOP_WORDS = {"the", "a", "an", "of", "to", "my", "our", "your", "please", "this", "that"}
MERGE_RADIUS = {"door": 0.9, "furniture": 1.0, "appliance": 0.8, "obstacle": 0.6, "person": 0.8}
DETECTION_STATE_TTL_S = 60.0


def normalize(text):
    text = re.sub(r"[_\-]+", " ", str(text).lower())
    text = re.sub(r"[^\w\s]", " ", text)
    return " ".join(w for w in text.split() if w not in STOP_WORDS)


def _tokens(text):
    return {w[:-1] if len(w) > 3 and w.endswith("s") else w for w in normalize(text).split()}


def label_similarity(a, b):
    na, nb = normalize(a), normalize(b)
    if not na or not nb:
        return 0.0
    if na == nb:
        return 1.0
    if re.search(rf"\b{re.escape(na)}\b", nb) or re.search(rf"\b{re.escape(nb)}\b", na):
        return 0.85
    ta, tb = _tokens(a), _tokens(b)
    return 0.8 * len(ta & tb) / len(ta | tb) if ta and tb else 0.0


def project_detection(bbox, depth, pose, width, height, hfov_rad, camera_offset, body_pitch=0.0):
    """Normalized bbox centre + z-depth (m) -> world (x, y, z). Camera axes: forward/left/up."""
    if depth is None or not 0.2 <= depth <= 9.0:
        return None
    u, v = (bbox[0] + bbox[2]) / 2 * width, (bbox[1] + bbox[3]) / 2 * height
    f = (width / 2) / math.tan(hfov_rad / 2)
    forward, left, up = depth, depth * (width / 2 - u) / f, depth * (height / 2 - v) / f
    tilt = -body_pitch  # REP-103: negative pitch tilts the camera up
    forward, up = forward * math.cos(tilt) - up * math.sin(tilt), forward * math.sin(tilt) + up * math.cos(tilt)
    bx, by = camera_offset[0] + forward, camera_offset[1] + left
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return (pose.x + bx * c - by * s, pose.y + bx * s + by * c, pose.z + camera_offset[2] + up)


class SemanticSceneGraph:
    def __init__(self, db_path=":memory:"):
        self._lock = threading.RLock()
        self._nodes = {}
        self._rooms = {}
        self.version = 0
        if db_path != ":memory:":
            Path(db_path).parent.mkdir(parents=True, exist_ok=True)
        self._db = sqlite3.connect(db_path, check_same_thread=False)
        self._db.execute("CREATE TABLE IF NOT EXISTS nodes (id TEXT PRIMARY KEY, data TEXT NOT NULL)")
        self._db.execute("CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT NOT NULL)")
        self._db.commit()
        for (data,) in self._db.execute("SELECT data FROM nodes"):
            node = SemanticNode.model_validate_json(data)
            self._nodes[node.id] = node
            if node.category == "room":
                self._rooms[node.label] = node.extra["bounds"]
        row = self._db.execute("SELECT value FROM meta WHERE key='counter'").fetchone()
        self._counter = int(row[0]) if row else 0

    # ---- persistence ------------------------------------------------------------------
    def _save(self, node):
        self._db.execute("INSERT OR REPLACE INTO nodes (id, data) VALUES (?, ?)", (node.id, node.model_dump_json()))
        self._db.execute("INSERT OR REPLACE INTO meta (key, value) VALUES ('counter', ?)", (str(self._counter),))
        self._db.commit()
        self.version += 1

    def close(self):
        with self._lock:
            self._db.close()

    def export_json(self, path=None):
        payload = {"rooms": self._rooms, "nodes": [n.model_dump() for n in self.get_all_nodes()],
                   "edges": self.get_edges()}
        if path:
            Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return payload

    # ---- rooms --------------------------------------------------------------------------
    def add_room(self, name, bounds):
        x1, y1, x2, y2 = bounds
        with self._lock:
            self._rooms[name] = list(bounds)
            node = SemanticNode(id="room_" + normalize(name).replace(" ", "_"), label=name, category="room",
                                coords=[(x1 + x2) / 2, (y1 + y2) / 2, 0.0], confidence=1.0,
                                last_seen=time.time(), state="known", room=name, source="seed",
                                extra={"bounds": list(bounds)})
            self._nodes[node.id] = node
            self._save(node)
            return node

    def room_of(self, x, y):
        for name, (x1, y1, x2, y2) in self._rooms.items():
            if x1 <= x <= x2 and y1 <= y <= y2:
                return name
        return None

    def rooms(self):
        return dict(self._rooms)

    # ---- nodes --------------------------------------------------------------------------
    def is_empty(self):
        return not self._nodes

    def add_node(self, label, category, coords, confidence, state="unknown", source="vlm", aliases=(),
                 extra=None, notes=""):
        with self._lock:
            self._counter += 1
            node = SemanticNode(id=f"obj_{self._counter:02d}", label=normalize(label) or str(label).lower(),
                                category=category, coords=[round(float(c), 3) for c in coords],
                                confidence=round(float(confidence), 3), last_seen=time.time(), state=state,
                                room=self.room_of(coords[0], coords[1]), aliases=list(aliases), source=source,
                                notes=notes, extra=dict(extra or {}))
            node.extra.setdefault("state_source", source)
            self._nodes[node.id] = node
            self._save(node)
            return node

    def seed(self, rooms, nodes):
        """Populate an empty map with a prior (floor plan rooms + known objects)."""
        with self._lock:
            if not self.is_empty():
                return False
            for name, bounds in rooms.items():
                self.add_room(name, bounds)
            for item in nodes:
                self.add_node(item["label"], item["category"], item["coords"], item["confidence"],
                              item.get("state", "unknown"), item.get("source", "seed"), item.get("aliases", ()),
                              item.get("extra"))
            return True

    def get_node(self, node_id):
        return self._nodes.get(node_id)

    def get_all_nodes(self, include_rooms=True):
        with self._lock:
            return [n for n in self._nodes.values() if include_rooms or n.category != "room"]

    def remove_node(self, node_id):
        with self._lock:
            if self._nodes.pop(node_id, None) is None:
                return False
            self._db.execute("DELETE FROM nodes WHERE id = ?", (node_id,))
            self._db.commit()
            self.version += 1
            return True

    def update_node_state(self, node_id, state, confidence=None, note=None, source="manual"):
        with self._lock:
            node = self._nodes.get(node_id)
            if node is None:
                raise KeyError(node_id)
            node.state = str(state).strip().lower()[:40] or "unknown"
            node.extra["state_source"] = source
            node.extra["state_time"] = time.time()
            if confidence is not None:
                node.extra["state_confidence"] = round(float(confidence), 3)
            if note:
                node.notes = str(note)[:300]
            node.last_seen = time.time()
            self._save(node)
            return node

    def similarity(self, node, query):
        scores = [label_similarity(node.label, query)]
        scores += [min(0.95, label_similarity(alias, query)) for alias in node.aliases]
        return max(scores)

    def query_map(self, label_or_room, limit=10):
        """Nodes matching a label, alias, room name or category, best first."""
        query = normalize(label_or_room)
        if not query:
            return []
        with self._lock:
            nodes = list(self._nodes.values())
            room_hits = [n for n in nodes if n.category == "room" and self.similarity(n, query) >= 0.99]
            if room_hits:
                room = room_hits[0].label
                members = sorted((n for n in nodes if n.room == room and n.category != "room"),
                                 key=lambda n: -n.confidence)
                return (room_hits[:1] + members)[:limit]
            category = query[:-1] if query.endswith("s") else query
            scored = []
            for node in nodes:
                score = self.similarity(node, query)
                if node.category == category and node.category != "room":
                    score = max(score, 0.6)
                if score >= 0.5:
                    scored.append((score, node.confidence, node.last_seen, node))
            scored.sort(key=lambda s: (-s[0], -s[1], -s[2]))
            return [s[3] for s in scored[:limit]]

    def resolve_target(self, name, min_score=0.6):
        hits = self.query_map(name, limit=1)
        if not hits:
            return None
        node = hits[0]
        return node if node.category == "room" or self.similarity(node, name) >= min_score else None

    def nearest(self, x, y, category=None, max_distance=math.inf):
        best = None
        for node in self.get_all_nodes(include_rooms=False):
            if category and node.category != category:
                continue
            d = math.hypot(node.coords[0] - x, node.coords[1] - y)
            if d <= max_distance and (best is None or d < best[0]):
                best = (d, node)
        return None if best is None else best[1]

    # ---- perception fusion --------------------------------------------------------------
    def ingest_detection(self, detection, frame):
        """Project one detection and merge it. Returns (node, created) or None if not projectable."""
        if frame.meta.get("calibrated") is False:
            return None
        depth = detection.depth_m
        if depth is None:
            u = (detection.bbox[0] + detection.bbox[2]) / 2
            v = (detection.bbox[1] + detection.bbox[3]) / 2
            depth = frame.depth_at(u, v)
        point = project_detection(detection.bbox, depth, frame.pose, frame.width, frame.height,
                                  frame.hfov_rad, frame.camera_offset, frame.body_pitch)
        if point is None:
            return None
        return self.fuse(detection.label, detection.category, point, detection.confidence, detection.state)

    def ingest_detections(self, detections, frame):
        return [r for r in (self.ingest_detection(d, frame) for d in detections) if r is not None]

    def fuse(self, label, category, point, confidence, state=None):
        with self._lock:
            radius = MERGE_RADIUS.get(category, 0.7)
            best = None
            for node in self._nodes.values():
                if node.category == "room":
                    continue
                distance = math.hypot(node.coords[0] - point[0], node.coords[1] - point[1])
                if distance > radius:
                    continue
                score = self.similarity(node, label)
                if node.category == category:
                    score = max(score, 0.5 if category in ("door", "obstacle") else score)
                if score >= 0.5 and (best is None or (score, -distance) > best[0]):
                    best = ((score, -distance), node)
            if best is None:
                node = self.add_node(label, category, point, confidence,
                                     state if state else "unknown", source="detection")
                if category in ("obstacle", "person"):
                    node.extra["radius"] = 0.4
                    if state in (None, "unknown"):
                        node.state = "blocking"
                    self._save(node)
                return node, True
            node = best[1]
            weight_old = min(node.observations, 8) * max(node.confidence, 0.1)
            weight_new = max(confidence, 0.05)
            total = weight_old + weight_new
            node.coords = [round((o * weight_old + n * weight_new) / total, 3) for o, n in zip(node.coords, point)]
            node.confidence = round(min(0.99, 1 - (1 - node.confidence) * (1 - 0.5 * confidence)), 3)
            node.observations += 1
            node.last_seen = time.time()
            node.room = self.room_of(node.coords[0], node.coords[1]) or node.room
            if (state and state != "unknown" and confidence >= 0.5
                    and (node.extra.get("state_source") in (None, "seed", "detection") or node.state == "unknown"
                         or time.time() - node.extra.get("state_time", 0) > DETECTION_STATE_TTL_S)):
                node.state = state.lower()[:40]
                node.extra["state_source"] = "detection"
                node.extra["state_time"] = time.time()
            normalized = normalize(label)
            if normalized and normalized != node.label and normalized not in node.aliases and len(node.aliases) < 8:
                node.aliases.append(normalized)
            self._save(node)
            return node, False

    # ---- graph view ---------------------------------------------------------------------
    def get_edges(self, near_m=1.5):
        nodes = self.get_all_nodes(include_rooms=False)
        edges = [{"source": n.id, "target": "room_" + normalize(n.room).replace(" ", "_"), "relation": "in_room"}
                 for n in nodes if n.room]
        for i, a in enumerate(nodes):
            for b in nodes[i + 1:]:
                if math.hypot(a.coords[0] - b.coords[0], a.coords[1] - b.coords[1]) <= near_m:
                    edges.append({"source": a.id, "target": b.id, "relation": "near"})
        return edges

    def summary_for_prompt(self, max_nodes=60):
        lines = []
        for node in sorted(self.get_all_nodes(), key=lambda n: (n.category != "room", n.id))[:max_nodes]:
            x, y, z = node.coords
            lines.append(f"{node.id} | {node.label} | {node.category} | room={node.room or '-'} | "
                         f"xyz=({x:.2f}, {y:.2f}, {z:.2f}) | state={node.state} | conf={node.confidence:.2f}")
        return "\n".join(lines)
