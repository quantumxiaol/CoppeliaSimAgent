"""Registry to expose validated Python tools to LLM function-calling."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping

from pydantic import BaseModel

from ..tools.kinematics import (
    actuate_gripper,
    actuate_youbot_gripper,
    drive_youbot_base,
    get_joint_position,
    move_ik_target,
    set_joint_position,
    set_joint_target_position,
    set_joint_target_velocity,
    set_youbot_base_locked,
    set_youbot_wheel_velocities,
    stop_youbot_base,
    setup_ik_link,
    setup_youbot_arm_ik,
    spawn_waypoint,
)
from ..tools.models import load_model, set_parent_child
from ..tools.primitives import (
    duplicate_object,
    remove_object,
    rename_object,
    set_object_color,
    set_object_pose,
    spawn_cuboid,
    spawn_primitive,
)
from ..tools.scene import check_collision, find_objects, get_scene_graph
from ..tools.schemas import (
    ActuateGripperInput,
    ActuateYouBotGripperInput,
    CheckCollisionInput,
    DriveYouBotBaseInput,
    DuplicateObjectInput,
    FindObjectsInput,
    GetJointPositionInput,
    GetSceneGraphInput,
    LoadModelInput,
    MoveIKTargetInput,
    RenameObjectInput,
    RemoveObjectInput,
    SetObjectColorInput,
    SetObjectPoseInput,
    SetJointPositionInput,
    SetJointTargetPositionInput,
    SetJointTargetVelocityInput,
    SetParentChildInput,
    SetYouBotBaseLockedInput,
    StopYouBotBaseInput,
    SetYouBotWheelVelocitiesInput,
    SetupIKLinkInput,
    SetupYouBotArmIKInput,
    SpawnCuboidInput,
    SpawnPrimitiveInput,
    SpawnWaypointInput,
)


@dataclass(frozen=True)
class ToolDefinition:
    """One callable tool with a strict pydantic input model."""

    name: str
    description: str
    input_model: type[BaseModel]
    handler: Callable[..., Any]

    def validate_and_call(self, payload: Mapping[str, Any]) -> Any:
        parsed = self.input_model.model_validate(dict(payload))
        return self.handler(**parsed.model_dump())

    def as_openai_tool(self) -> dict[str, Any]:
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": self.input_model.model_json_schema(),
            },
        }


TOOL_REGISTRY: dict[str, ToolDefinition] = {
    "get_scene_graph": ToolDefinition(
        name="get_scene_graph",
        description="Read scene graph states (name, handle, pose) from CoppeliaSim.",
        input_model=GetSceneGraphInput,
        handler=get_scene_graph,
    ),
    "find_objects": ToolDefinition(
        name="find_objects",
        description="Find scene objects by name query and object type filters.",
        input_model=FindObjectsInput,
        handler=find_objects,
    ),
    "check_collision": ToolDefinition(
        name="check_collision",
        description="Run collision check between two entities/collections.",
        input_model=CheckCollisionInput,
        handler=check_collision,
    ),
    "spawn_primitive": ToolDefinition(
        name="spawn_primitive",
        description="Spawn a primitive shape with color/size/position.",
        input_model=SpawnPrimitiveInput,
        handler=spawn_primitive,
    ),
    "spawn_cuboid": ToolDefinition(
        name="spawn_cuboid",
        description="Spawn a cuboid helper with strict metric units.",
        input_model=SpawnCuboidInput,
        handler=spawn_cuboid,
    ),
    "set_object_pose": ToolDefinition(
        name="set_object_pose",
        description="Set object position and/or orientation (degrees).",
        input_model=SetObjectPoseInput,
        handler=set_object_pose,
    ),
    "remove_object": ToolDefinition(
        name="remove_object",
        description="Remove an object by handle from the scene.",
        input_model=RemoveObjectInput,
        handler=remove_object,
    ),
    "duplicate_object": ToolDefinition(
        name="duplicate_object",
        description="Duplicate one object by handle with optional reposition/offset.",
        input_model=DuplicateObjectInput,
        handler=duplicate_object,
    ),
    "rename_object": ToolDefinition(
        name="rename_object",
        description="Rename one object alias by handle.",
        input_model=RenameObjectInput,
        handler=rename_object,
    ),
    "set_object_color": ToolDefinition(
        name="set_object_color",
        description="Set one shape object's color by handle.",
        input_model=SetObjectColorInput,
        handler=set_object_color,
    ),
    "load_model": ToolDefinition(
        name="load_model",
        description="Load a .ttm model and place it in the scene.",
        input_model=LoadModelInput,
        handler=load_model,
    ),
    "set_parent_child": ToolDefinition(
        name="set_parent_child",
        description="Create parent-child assembly relation between two handles.",
        input_model=SetParentChildInput,
        handler=set_parent_child,
    ),
    "spawn_waypoint": ToolDefinition(
        name="spawn_waypoint",
        description="Create a Dummy waypoint to be used as IK target.",
        input_model=SpawnWaypointInput,
        handler=spawn_waypoint,
    ),
    "get_joint_position": ToolDefinition(
        name="get_joint_position",
        description="Read one joint's current position.",
        input_model=GetJointPositionInput,
        handler=get_joint_position,
    ),
    "set_joint_position": ToolDefinition(
        name="set_joint_position",
        description="Set one joint's immediate position.",
        input_model=SetJointPositionInput,
        handler=set_joint_position,
    ),
    "set_joint_target_position": ToolDefinition(
        name="set_joint_target_position",
        description="Set one joint's target position with optional motion profile.",
        input_model=SetJointTargetPositionInput,
        handler=set_joint_target_position,
    ),
    "set_joint_target_velocity": ToolDefinition(
        name="set_joint_target_velocity",
        description="Set one joint's target velocity with optional motion profile.",
        input_model=SetJointTargetVelocityInput,
        handler=set_joint_target_velocity,
    ),
    "set_youbot_wheel_velocities": ToolDefinition(
        name="set_youbot_wheel_velocities",
        description="Set the four youBot wheel joint target velocities explicitly.",
        input_model=SetYouBotWheelVelocitiesInput,
        handler=set_youbot_wheel_velocities,
    ),
    "drive_youbot_base": ToolDefinition(
        name="drive_youbot_base",
        description="Map youBot forward/lateral/yaw commands to wheel target velocities.",
        input_model=DriveYouBotBaseInput,
        handler=drive_youbot_base,
    ),
    "stop_youbot_base": ToolDefinition(
        name="stop_youbot_base",
        description="Zero all four youBot wheel joint target velocities.",
        input_model=StopYouBotBaseInput,
        handler=stop_youbot_base,
    ),
    "set_youbot_base_locked": ToolDefinition(
        name="set_youbot_base_locked",
        description="Toggle a fixed-base mode for the youBot platform and wheels.",
        input_model=SetYouBotBaseLockedInput,
        handler=set_youbot_base_locked,
    ),
    "setup_ik_link": ToolDefinition(
        name="setup_ik_link",
        description="Create IK environment/group and bind base-tip-target chain.",
        input_model=SetupIKLinkInput,
        handler=setup_ik_link,
    ),
    "setup_youbot_arm_ik": ToolDefinition(
        name="setup_youbot_arm_ik",
        description="Create/reuse youBot arm tip-target dummies and bind an IK chain.",
        input_model=SetupYouBotArmIKInput,
        handler=setup_youbot_arm_ik,
    ),
    "move_ik_target": ToolDefinition(
        name="move_ik_target",
        description="Move IK target and solve IK for a number of steps.",
        input_model=MoveIKTargetInput,
        handler=move_ik_target,
    ),
    "actuate_gripper": ToolDefinition(
        name="actuate_gripper",
        description="Control gripper open/close through int signal.",
        input_model=ActuateGripperInput,
        handler=actuate_gripper,
    ),
    "actuate_youbot_gripper": ToolDefinition(
        name="actuate_youbot_gripper",
        description="Open/close the two-jaw youBot gripper via its finger joints.",
        input_model=ActuateYouBotGripperInput,
        handler=actuate_youbot_gripper,
    ),
}


def list_tool_names() -> list[str]:
    return sorted(TOOL_REGISTRY)


def get_tool(name: str) -> ToolDefinition:
    if name not in TOOL_REGISTRY:
        available = ", ".join(list_tool_names())
        raise KeyError(f"Unknown tool '{name}'. Available: {available}")
    return TOOL_REGISTRY[name]


def invoke_tool(name: str, payload: Mapping[str, Any]) -> Any:
    return get_tool(name).validate_and_call(payload)


def get_openai_tools() -> list[dict[str, Any]]:
    return [TOOL_REGISTRY[name].as_openai_tool() for name in list_tool_names()]
