"""Validated data contracts shared by the HAL, semantic map, planner, engine and UI."""
import math
from typing import Literal, Optional

from pydantic import BaseModel, ConfigDict, Field, field_validator

CATEGORIES = ("door", "window", "safety", "appliance", "furniture", "electronics", "infrastructure",
              "hazard", "obstacle", "person", "room", "other")


class Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


def _finite(values):
    if any(not math.isfinite(v) for v in values):
        raise ValueError("Non-finite number")
    return values


def _check_bbox(box):
    if box is None:
        return box
    x1, y1, x2, y2 = _finite(box)
    if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
        raise ValueError("bbox must be normalized [x1, y1, x2, y2] with x1<x2, y1<y2")
    return box


class Pose(Strict):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


class Telemetry(Strict):
    source: str
    connected: bool
    pose: Pose
    velocity: list[float]          # body frame vx, vy, vyaw
    body_euler: list[float]        # commanded roll, pitch, yaw
    battery_pct: Optional[float] = None
    gait_mode: str = "unknown"
    fsm_mode: str = "unknown"
    standing: bool = True
    blocked: bool = False          # contact/collision reported by the robot or simulator
    motion_enabled: bool = False
    armed: bool = False
    emergency_latched: bool = False
    timestamp: float = 0.0
    notes: list[str] = Field(default_factory=list)


class Detection(Strict):
    label: str = Field(min_length=1, max_length=60)
    category: str = "other"
    bbox: tuple[float, float, float, float]
    confidence: float = Field(ge=0, le=1)
    state: Optional[str] = Field(default=None, max_length=40)
    depth_m: Optional[float] = None

    @field_validator("bbox")
    @classmethod
    def _valid_bbox(cls, value):
        return _check_bbox(value)

    @field_validator("category")
    @classmethod
    def _category(cls, value):
        return value if value in CATEGORIES else "other"


class SemanticNode(Strict):
    id: str
    label: str
    category: str
    coords: list[float] = Field(min_length=3, max_length=3)
    confidence: float = Field(ge=0, le=1)
    last_seen: float
    state: str = "unknown"
    room: Optional[str] = None
    aliases: list[str] = Field(default_factory=list)
    observations: int = 1
    source: str = "vlm"
    notes: str = ""
    extra: dict = Field(default_factory=dict)

    @field_validator("coords")
    @classmethod
    def _valid_coords(cls, value):
        return _finite(value)


class VisualVerdict(Strict):
    answer: Literal["yes", "no", "uncertain"]
    state: str = Field(max_length=40)
    confidence: float = Field(ge=0, le=1)
    evidence: str = Field(max_length=300)
    bbox: Optional[tuple[float, float, float, float]] = None

    @field_validator("bbox")
    @classmethod
    def _valid_bbox(cls, value):
        return _check_bbox(value)


class BlockageDiagnosis(Strict):
    cause: Literal["door_closed", "temporary_obstacle", "permanent_obstacle", "person_in_way", "unknown"]
    blocking_label: str = Field(max_length=60)
    description: str = Field(max_length=300)
    requires_human: bool
    confidence: float = Field(ge=0, le=1)
