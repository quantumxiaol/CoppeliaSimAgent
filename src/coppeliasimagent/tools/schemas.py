"""Pydantic input schemas for tool functions."""

from __future__ import annotations

import math
from enum import Enum
from typing import Any

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class ToolInputModel(BaseModel):
    """Strict base model shared by all tool input schemas."""

    model_config = ConfigDict(extra="forbid")


class PrimitiveType(str, Enum):
    CUBOID = "cuboid"
    SPHERE = "sphere"
    CYLINDER = "cylinder"


class ColorComponent(str, Enum):
    AMBIENT_DIFFUSE = "ambient_diffuse"
    DIFFUSE = "diffuse"
    SPECULAR = "specular"
    EMISSION = "emission"
    TRANSPARENCY = "transparency"


class SceneObjectType(str, Enum):
    SHAPE = "shape"
    DUMMY = "dummy"
    JOINT = "joint"
    CAMERA = "camera"
    LIGHT = "light"
    FORCE_SENSOR = "force_sensor"


def _validate_vec3(value: list[float], *, field_name: str) -> list[float]:
    if len(value) != 3:
        raise ValueError(f"{field_name} must contain exactly 3 numbers")

    normalized = [float(v) for v in value]
    for v in normalized:
        if not math.isfinite(v):
            raise ValueError(f"{field_name} contains non-finite numbers")
    return normalized


def _validate_color(value: list[float], *, field_name: str) -> list[float]:
    normalized = _validate_vec3(value, field_name=field_name)
    for channel in normalized:
        if channel < 0.0 or channel > 1.0:
            raise ValueError(f"{field_name} must be in [0, 1]")
    return normalized


class SpawnPrimitiveInput(ToolInputModel):
    primitive: PrimitiveType
    size: list[float] = Field(min_length=3, max_length=3)
    position: list[float] = Field(min_length=3, max_length=3)
    color: list[float] = Field(default_factory=lambda: [0.8, 0.8, 0.8], min_length=3, max_length=3)
    dynamic: bool = True
    relative_to: int = -1

    @field_validator("size")
    @classmethod
    def validate_size(cls, value: list[float]) -> list[float]:
        normalized = _validate_vec3(value, field_name="size")
        if any(v <= 0.0 for v in normalized):
            raise ValueError("size values must be > 0")
        return normalized

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: list[float]) -> list[float]:
        return _validate_vec3(value, field_name="position")

    @field_validator("color")
    @classmethod
    def validate_color(cls, value: list[float]) -> list[float]:
        return _validate_color(value, field_name="color")


class SpawnCuboidInput(ToolInputModel):
    size: list[float] = Field(min_length=3, max_length=3)
    position: list[float] = Field(min_length=3, max_length=3)
    color: list[float] = Field(default_factory=lambda: [0.8, 0.8, 0.8], min_length=3, max_length=3)
    dynamic: bool = True
    relative_to: int = -1

    @field_validator("size")
    @classmethod
    def validate_size(cls, value: list[float]) -> list[float]:
        normalized = _validate_vec3(value, field_name="size")
        if any(v <= 0.0 for v in normalized):
            raise ValueError("size values must be > 0")
        return normalized

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: list[float]) -> list[float]:
        return _validate_vec3(value, field_name="position")

    @field_validator("color")
    @classmethod
    def validate_color(cls, value: list[float]) -> list[float]:
        return _validate_color(value, field_name="color")


class SetObjectPoseInput(ToolInputModel):
    handle: int
    position: list[float] | None = Field(default=None, min_length=3, max_length=3)
    orientation_deg: list[float] | None = Field(default=None, min_length=3, max_length=3)
    relative_to: int = -1

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: list[float] | None) -> list[float] | None:
        if value is None:
            return None
        return _validate_vec3(value, field_name="position")

    @field_validator("orientation_deg")
    @classmethod
    def validate_orientation(cls, value: list[float] | None) -> list[float] | None:
        if value is None:
            return None
        return _validate_vec3(value, field_name="orientation_deg")

    @model_validator(mode="after")
    def require_position_or_orientation(self) -> "SetObjectPoseInput":
        if self.position is None and self.orientation_deg is None:
            raise ValueError("either position or orientation_deg must be provided")
        return self


class DuplicateObjectInput(ToolInputModel):
    handle: int
    position: list[float] | None = Field(default=None, min_length=3, max_length=3)
    offset: list[float] | None = Field(default=None, min_length=3, max_length=3)
    relative_to: int = -1

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: list[float] | None) -> list[float] | None:
        if value is None:
            return None
        return _validate_vec3(value, field_name="position")

    @field_validator("offset")
    @classmethod
    def validate_offset(cls, value: list[float] | None) -> list[float] | None:
        if value is None:
            return None
        return _validate_vec3(value, field_name="offset")

    @model_validator(mode="after")
    def validate_move_args(self) -> "DuplicateObjectInput":
        if self.position is not None and self.offset is not None:
            raise ValueError("position and offset cannot both be provided")
        return self


class RenameObjectInput(ToolInputModel):
    handle: int
    new_alias: str = Field(min_length=1)

    @field_validator("new_alias")
    @classmethod
    def validate_new_alias(cls, value: str) -> str:
        text = value.strip()
        if not text:
            raise ValueError("new_alias cannot be blank")
        return text


class SetObjectColorInput(ToolInputModel):
    handle: int
    color: list[float] = Field(min_length=3, max_length=3)
    color_name: str | None = None
    color_component: ColorComponent = ColorComponent.AMBIENT_DIFFUSE

    @field_validator("color")
    @classmethod
    def validate_color(cls, value: list[float]) -> list[float]:
        return _validate_color(value, field_name="color")

    @field_validator("color_name")
    @classmethod
    def validate_color_name(cls, value: str | None) -> str | None:
        if value is None:
            return None
        text = value.strip()
        return text if text else None


class RemoveObjectInput(ToolInputModel):
    handle: int


class LoadModelInput(ToolInputModel):
    model_path: str = Field(min_length=1)
    position: list[float] = Field(min_length=3, max_length=3)
    orientation_deg: list[float] | None = Field(default=None, min_length=3, max_length=3)
    relative_to: int = -1

    @field_validator("model_path")
    @classmethod
    def validate_path(cls, value: str) -> str:
        value = value.strip()
        if not value:
            raise ValueError("model_path cannot be empty")
        return value

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: list[float]) -> list[float]:
        return _validate_vec3(value, field_name="position")

    @field_validator("orientation_deg")
    @classmethod
    def validate_orientation(cls, value: list[float] | None) -> list[float] | None:
        if value is None:
            return None
        return _validate_vec3(value, field_name="orientation_deg")


class SetParentChildInput(ToolInputModel):
    child_handle: int
    parent_handle: int
    keep_in_place: bool = True


class GetSceneGraphInput(ToolInputModel):
    include_types: list[SceneObjectType] = Field(
        default_factory=lambda: [SceneObjectType.SHAPE, SceneObjectType.DUMMY]
    )
    round_digits: int = Field(default=3, ge=0, le=8)


class FindObjectsInput(ToolInputModel):
    name_query: str | None = None
    exact_name: bool = False
    include_types: list[SceneObjectType] = Field(
        default_factory=lambda: [SceneObjectType.SHAPE, SceneObjectType.DUMMY]
    )
    round_digits: int = Field(default=3, ge=0, le=8)
    limit: int = Field(default=20, ge=1, le=200)

    @field_validator("name_query")
    @classmethod
    def validate_name_query(cls, value: str | None) -> str | None:
        if value is None:
            return None
        text = value.strip()
        if not text:
            raise ValueError("name_query cannot be blank")
        return text


class CheckCollisionInput(ToolInputModel):
    entity1: int
    entity2: int


class SpawnWaypointInput(ToolInputModel):
    position: list[float] = Field(min_length=3, max_length=3)
    size: float = Field(default=0.02, gt=0.0)
    relative_to: int = -1

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: list[float]) -> list[float]:
        return _validate_vec3(value, field_name="position")


class SetupIKLinkInput(ToolInputModel):
    base_handle: int
    tip_handle: int
    target_handle: int
    constraints_mask: int | None = None


class MoveIKTargetInput(ToolInputModel):
    environment_handle: int
    group_handle: int
    target_handle: int
    position: list[float] = Field(min_length=3, max_length=3)
    relative_to: int = -1
    steps: int = Field(default=1, ge=1, le=200)

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: list[float]) -> list[float]:
        return _validate_vec3(value, field_name="position")


class ActuateGripperInput(ToolInputModel):
    signal_name: str = Field(min_length=1)
    closed: bool

    @field_validator("signal_name")
    @classmethod
    def validate_signal_name(cls, value: str) -> str:
        value = value.strip()
        if not value:
            raise ValueError("signal_name cannot be empty")
        return value


def as_payload(model: ToolInputModel) -> dict[str, Any]:
    """Convert model to JSON-compatible payload for tool dispatching."""
    return model.model_dump(mode="json")
