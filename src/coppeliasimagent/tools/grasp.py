"""Simple grasp/release abstractions."""

from __future__ import annotations

from pydantic import ValidationError

from ..core.connection import get_sim
from ..core.exceptions import ToolValidationError
from .kinematics import actuate_gripper, actuate_youbot_gripper
from .schemas import AttachObjectToGripperInput, DetachObjectInput, GraspObjectInput, ReleaseObjectInput


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def attach_object_to_gripper(
    object_handle: int,
    gripper_handle: int,
    keep_in_place: bool = True,
) -> dict[str, object]:
    """Parent an object under a gripper/tip handle."""
    try:
        payload = AttachObjectToGripperInput.model_validate(
            {
                "object_handle": object_handle,
                "gripper_handle": gripper_handle,
                "keep_in_place": keep_in_place,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    sim.setObjectParent(payload.object_handle, payload.gripper_handle, payload.keep_in_place)
    return {
        "object_handle": payload.object_handle,
        "gripper_handle": payload.gripper_handle,
        "keep_in_place": payload.keep_in_place,
    }


def detach_object(
    object_handle: int,
    parent_handle: int = -1,
    keep_in_place: bool = True,
) -> dict[str, object]:
    """Detach an object to world or another parent."""
    try:
        payload = DetachObjectInput.model_validate(
            {
                "object_handle": object_handle,
                "parent_handle": parent_handle,
                "keep_in_place": keep_in_place,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    sim.setObjectParent(payload.object_handle, payload.parent_handle, payload.keep_in_place)
    return {
        "object_handle": payload.object_handle,
        "parent_handle": payload.parent_handle,
        "keep_in_place": payload.keep_in_place,
    }


def grasp_object(
    object_handle: int,
    gripper_handle: int,
    signal_name: str | None = None,
    robot_path: str | None = None,
    close_gripper: bool = True,
    attach: bool = True,
    keep_in_place: bool = True,
) -> dict[str, object]:
    """Close a gripper command path if provided, then optionally attach the object."""
    try:
        payload = GraspObjectInput.model_validate(
            {
                "object_handle": object_handle,
                "gripper_handle": gripper_handle,
                "signal_name": signal_name,
                "robot_path": robot_path,
                "close_gripper": close_gripper,
                "attach": attach,
                "keep_in_place": keep_in_place,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    command_result: object | None = None
    if payload.close_gripper:
        if payload.robot_path:
            command_result = actuate_youbot_gripper(robot_path=payload.robot_path, closed=True)
        elif payload.signal_name:
            command_result = actuate_gripper(signal_name=payload.signal_name, closed=True)

    attach_result = None
    if payload.attach:
        attach_result = attach_object_to_gripper(
            object_handle=payload.object_handle,
            gripper_handle=payload.gripper_handle,
            keep_in_place=payload.keep_in_place,
        )

    return {
        "object_handle": payload.object_handle,
        "gripper_handle": payload.gripper_handle,
        "closed": payload.close_gripper,
        "attached": payload.attach,
        "command_result": command_result,
        "attach_result": attach_result,
    }


def release_object(
    object_handle: int,
    signal_name: str | None = None,
    robot_path: str | None = None,
    parent_handle: int = -1,
    open_gripper: bool = True,
    detach: bool = True,
    keep_in_place: bool = True,
) -> dict[str, object]:
    """Open a gripper command path if provided, then optionally detach the object."""
    try:
        payload = ReleaseObjectInput.model_validate(
            {
                "object_handle": object_handle,
                "signal_name": signal_name,
                "robot_path": robot_path,
                "parent_handle": parent_handle,
                "open_gripper": open_gripper,
                "detach": detach,
                "keep_in_place": keep_in_place,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    detach_result = None
    if payload.detach:
        detach_result = detach_object(
            object_handle=payload.object_handle,
            parent_handle=payload.parent_handle,
            keep_in_place=payload.keep_in_place,
        )

    command_result: object | None = None
    if payload.open_gripper:
        if payload.robot_path:
            command_result = actuate_youbot_gripper(robot_path=payload.robot_path, closed=False)
        elif payload.signal_name:
            command_result = actuate_gripper(signal_name=payload.signal_name, closed=False)

    return {
        "object_handle": payload.object_handle,
        "detached": payload.detach,
        "opened": payload.open_gripper,
        "detach_result": detach_result,
        "command_result": command_result,
    }
