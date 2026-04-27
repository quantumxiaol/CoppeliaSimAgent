"""Registry to expose validated Python tools to LLM function-calling."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping

from pydantic import BaseModel

from ..tools.dynamics import get_object_velocity, reset_dynamic_object, set_shape_dynamics
from ..tools.grasp import attach_object_to_gripper, detach_object, grasp_object, release_object
from ..tools.kinematics import (
    actuate_gripper,
    actuate_youbot_gripper,
    configure_abb_arm_drive,
    drive_youbot_base,
    get_joint_dyn_ctrl_mode,
    get_joint_force,
    get_joint_mode,
    get_joint_position,
    get_joint_target_force,
    move_ik_target,
    set_joint_dyn_ctrl_mode,
    set_joint_mode,
    set_joint_position,
    set_joint_target_force,
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
from ..tools.point_cloud import (
    create_point_cloud_surface_from_shape,
    execute_polishing_path,
    get_point_cloud_stats,
    insert_points_into_point_cloud,
    remove_points_near_tool,
    simulate_polishing_step,
)
from ..tools.primitives import (
    duplicate_object,
    remove_object,
    rename_object,
    set_object_color,
    set_object_pose,
    spawn_cuboid,
    spawn_primitive,
)
from ..tools.runtime import step_simulation, wait_seconds, wait_until_object_pose_stable, wait_until_state
from ..tools.scene import check_collision, find_objects, get_object_pose, get_relative_pose, get_scene_graph
from ..tools.sensors import check_collision_monitor, get_vision_sensor_image, read_force_sensor, read_proximity_sensor
from ..tools.simulation import (
    get_plugin_status,
    get_simulation_state,
    pause_simulation,
    start_simulation,
    stop_simulation,
)
from ..tools.trajectory import execute_cartesian_waypoints, execute_joint_trajectory
from ..tools.verification import (
    verify_force_threshold,
    verify_joint_positions_reached,
    verify_object_moved,
    verify_object_velocity_below,
)
from ..tools.schemas import (
    ActuateGripperInput,
    ActuateYouBotGripperInput,
    AttachObjectToGripperInput,
    CheckCollisionInput,
    CheckCollisionMonitorInput,
    ConfigureAbbArmDriveInput,
    CreatePointCloudSurfaceFromShapeInput,
    DetachObjectInput,
    DriveYouBotBaseInput,
    DuplicateObjectInput,
    ExecuteCartesianWaypointsInput,
    ExecuteJointTrajectoryInput,
    ExecutePolishingPathInput,
    FindObjectsInput,
    GetVisionSensorImageInput,
    GetObjectVelocityInput,
    GetObjectPoseInput,
    GetPointCloudStatsInput,
    GetPluginStatusInput,
    GetJointDynCtrlModeInput,
    GetJointForceInput,
    GetJointModeInput,
    GetJointPositionInput,
    GetRelativePoseInput,
    GetSceneGraphInput,
    GetSimulationStateInput,
    GetJointTargetForceInput,
    GraspObjectInput,
    InsertPointsIntoPointCloudInput,
    LoadModelInput,
    MoveIKTargetInput,
    PauseSimulationInput,
    ReadForceSensorInput,
    ReadProximitySensorInput,
    RenameObjectInput,
    RemoveObjectInput,
    RemovePointsNearToolInput,
    ReleaseObjectInput,
    ResetDynamicObjectInput,
    SetObjectColorInput,
    SetObjectPoseInput,
    SetShapeDynamicsInput,
    SetJointDynCtrlModeInput,
    SetJointModeInput,
    SetJointPositionInput,
    SetJointTargetForceInput,
    SetJointTargetPositionInput,
    SetJointTargetVelocityInput,
    SetParentChildInput,
    SetYouBotBaseLockedInput,
    SimulatePolishingStepInput,
    StepSimulationInput,
    StartSimulationInput,
    StopYouBotBaseInput,
    StopSimulationInput,
    SetYouBotWheelVelocitiesInput,
    SetupIKLinkInput,
    SetupYouBotArmIKInput,
    SpawnCuboidInput,
    SpawnPrimitiveInput,
    SpawnWaypointInput,
    VerifyForceThresholdInput,
    VerifyJointPositionsReachedInput,
    VerifyObjectMovedInput,
    VerifyObjectVelocityBelowInput,
    WaitSecondsInput,
    WaitUntilObjectPoseStableInput,
    WaitUntilStateInput,
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
    "get_simulation_state": ToolDefinition(
        name="get_simulation_state",
        description="Read CoppeliaSim simulation lifecycle state.",
        input_model=GetSimulationStateInput,
        handler=get_simulation_state,
    ),
    "get_plugin_status": ToolDefinition(
        name="get_plugin_status",
        description="Read availability of simulator-side plugin namespaces such as simIK/simOMPL.",
        input_model=GetPluginStatusInput,
        handler=get_plugin_status,
    ),
    "start_simulation": ToolDefinition(
        name="start_simulation",
        description="Start or resume CoppeliaSim simulation execution.",
        input_model=StartSimulationInput,
        handler=start_simulation,
    ),
    "pause_simulation": ToolDefinition(
        name="pause_simulation",
        description="Pause CoppeliaSim simulation execution.",
        input_model=PauseSimulationInput,
        handler=pause_simulation,
    ),
    "stop_simulation": ToolDefinition(
        name="stop_simulation",
        description="Stop CoppeliaSim simulation execution.",
        input_model=StopSimulationInput,
        handler=stop_simulation,
    ),
    "step_simulation": ToolDefinition(
        name="step_simulation",
        description="Advance CoppeliaSim by deterministic remote stepping ticks when supported.",
        input_model=StepSimulationInput,
        handler=step_simulation,
    ),
    "wait_seconds": ToolDefinition(
        name="wait_seconds",
        description="Wait in the external agent process for a number of seconds.",
        input_model=WaitSecondsInput,
        handler=wait_seconds,
    ),
    "wait_until_state": ToolDefinition(
        name="wait_until_state",
        description="Poll simulation lifecycle state until a target label or numeric value appears.",
        input_model=WaitUntilStateInput,
        handler=wait_until_state,
    ),
    "wait_until_object_pose_stable": ToolDefinition(
        name="wait_until_object_pose_stable",
        description="Wait until an object's pose remains stable within tolerances.",
        input_model=WaitUntilObjectPoseStableInput,
        handler=wait_until_object_pose_stable,
    ),
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
    "get_object_pose": ToolDefinition(
        name="get_object_pose",
        description="Read one object's pose in world or another reference frame.",
        input_model=GetObjectPoseInput,
        handler=get_object_pose,
    ),
    "get_relative_pose": ToolDefinition(
        name="get_relative_pose",
        description="Read target pose expressed in source object's reference frame.",
        input_model=GetRelativePoseInput,
        handler=get_relative_pose,
    ),
    "check_collision": ToolDefinition(
        name="check_collision",
        description="Run collision check between two entities/collections.",
        input_model=CheckCollisionInput,
        handler=check_collision,
    ),
    "check_collision_monitor": ToolDefinition(
        name="check_collision_monitor",
        description="Monitor a collision pair for a duration and report whether contact occurred.",
        input_model=CheckCollisionMonitorInput,
        handler=check_collision_monitor,
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
    "get_object_velocity": ToolDefinition(
        name="get_object_velocity",
        description="Read object linear/angular velocity and speed.",
        input_model=GetObjectVelocityInput,
        handler=get_object_velocity,
    ),
    "reset_dynamic_object": ToolDefinition(
        name="reset_dynamic_object",
        description="Reset dynamic state for an object or model.",
        input_model=ResetDynamicObjectInput,
        handler=reset_dynamic_object,
    ),
    "set_shape_dynamics": ToolDefinition(
        name="set_shape_dynamics",
        description="Set shape static/respondable flags and optional mass/friction.",
        input_model=SetShapeDynamicsInput,
        handler=set_shape_dynamics,
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
    "get_joint_mode": ToolDefinition(
        name="get_joint_mode",
        description="Read one joint's operation mode.",
        input_model=GetJointModeInput,
        handler=get_joint_mode,
    ),
    "set_joint_mode": ToolDefinition(
        name="set_joint_mode",
        description="Set one joint's operation mode.",
        input_model=SetJointModeInput,
        handler=set_joint_mode,
    ),
    "get_joint_dyn_ctrl_mode": ToolDefinition(
        name="get_joint_dyn_ctrl_mode",
        description="Read one joint's dynamic control mode.",
        input_model=GetJointDynCtrlModeInput,
        handler=get_joint_dyn_ctrl_mode,
    ),
    "set_joint_dyn_ctrl_mode": ToolDefinition(
        name="set_joint_dyn_ctrl_mode",
        description="Set one joint's dynamic control mode.",
        input_model=SetJointDynCtrlModeInput,
        handler=set_joint_dyn_ctrl_mode,
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
    "get_joint_target_force": ToolDefinition(
        name="get_joint_target_force",
        description="Read one joint's maximum force or torque setting.",
        input_model=GetJointTargetForceInput,
        handler=get_joint_target_force,
    ),
    "set_joint_target_force": ToolDefinition(
        name="set_joint_target_force",
        description="Set one joint's maximum force or torque.",
        input_model=SetJointTargetForceInput,
        handler=set_joint_target_force,
    ),
    "get_joint_force": ToolDefinition(
        name="get_joint_force",
        description="Read one joint's currently applied force or torque.",
        input_model=GetJointForceInput,
        handler=get_joint_force,
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
    "configure_abb_arm_drive": ToolDefinition(
        name="configure_abb_arm_drive",
        description="Configure ABB IRB4600 joints for dynamic drive with force and control mode settings.",
        input_model=ConfigureAbbArmDriveInput,
        handler=configure_abb_arm_drive,
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
    "execute_joint_trajectory": ToolDefinition(
        name="execute_joint_trajectory",
        description="Execute joint-space waypoints using immediate or target-position commands.",
        input_model=ExecuteJointTrajectoryInput,
        handler=execute_joint_trajectory,
    ),
    "execute_cartesian_waypoints": ToolDefinition(
        name="execute_cartesian_waypoints",
        description="Move an IK target through Cartesian waypoints and solve each segment.",
        input_model=ExecuteCartesianWaypointsInput,
        handler=execute_cartesian_waypoints,
    ),
    "verify_joint_positions_reached": ToolDefinition(
        name="verify_joint_positions_reached",
        description="Check whether joints are within tolerance of requested positions.",
        input_model=VerifyJointPositionsReachedInput,
        handler=verify_joint_positions_reached,
    ),
    "verify_object_moved": ToolDefinition(
        name="verify_object_moved",
        description="Check whether an object moved at least a minimum distance from a start position.",
        input_model=VerifyObjectMovedInput,
        handler=verify_object_moved,
    ),
    "verify_object_velocity_below": ToolDefinition(
        name="verify_object_velocity_below",
        description="Check whether object linear/angular velocity is below thresholds.",
        input_model=VerifyObjectVelocityBelowInput,
        handler=verify_object_velocity_below,
    ),
    "verify_force_threshold": ToolDefinition(
        name="verify_force_threshold",
        description="Check whether any listed joint reports at least a requested force magnitude.",
        input_model=VerifyForceThresholdInput,
        handler=verify_force_threshold,
    ),
    "attach_object_to_gripper": ToolDefinition(
        name="attach_object_to_gripper",
        description="Parent an object under a gripper/tip handle.",
        input_model=AttachObjectToGripperInput,
        handler=attach_object_to_gripper,
    ),
    "detach_object": ToolDefinition(
        name="detach_object",
        description="Detach an object to world or another parent.",
        input_model=DetachObjectInput,
        handler=detach_object,
    ),
    "grasp_object": ToolDefinition(
        name="grasp_object",
        description="Close a gripper command path and optionally attach an object.",
        input_model=GraspObjectInput,
        handler=grasp_object,
    ),
    "release_object": ToolDefinition(
        name="release_object",
        description="Open a gripper command path and optionally detach an object.",
        input_model=ReleaseObjectInput,
        handler=release_object,
    ),
    "read_proximity_sensor": ToolDefinition(
        name="read_proximity_sensor",
        description="Read one proximity sensor.",
        input_model=ReadProximitySensorInput,
        handler=read_proximity_sensor,
    ),
    "read_force_sensor": ToolDefinition(
        name="read_force_sensor",
        description="Read one force sensor.",
        input_model=ReadForceSensorInput,
        handler=read_force_sensor,
    ),
    "get_vision_sensor_image": ToolDefinition(
        name="get_vision_sensor_image",
        description="Read a vision sensor image or metadata.",
        input_model=GetVisionSensorImageInput,
        handler=get_vision_sensor_image,
    ),
    "create_point_cloud_surface_from_shape": ToolDefinition(
        name="create_point_cloud_surface_from_shape",
        description="Create a point-cloud surface sampled from a shape.",
        input_model=CreatePointCloudSurfaceFromShapeInput,
        handler=create_point_cloud_surface_from_shape,
    ),
    "insert_points_into_point_cloud": ToolDefinition(
        name="insert_points_into_point_cloud",
        description="Insert explicit world-frame points into a point cloud.",
        input_model=InsertPointsIntoPointCloudInput,
        handler=insert_points_into_point_cloud,
    ),
    "remove_points_near_tool": ToolDefinition(
        name="remove_points_near_tool",
        description="Remove point-cloud points near a tool object's current position.",
        input_model=RemovePointsNearToolInput,
        handler=remove_points_near_tool,
    ),
    "get_point_cloud_stats": ToolDefinition(
        name="get_point_cloud_stats",
        description="Return known point-cloud count and simulator options when available.",
        input_model=GetPointCloudStatsInput,
        handler=get_point_cloud_stats,
    ),
    "simulate_polishing_step": ToolDefinition(
        name="simulate_polishing_step",
        description="Remove point-cloud surface points near the polishing tool.",
        input_model=SimulatePolishingStepInput,
        handler=simulate_polishing_step,
    ),
    "execute_polishing_path": ToolDefinition(
        name="execute_polishing_path",
        description="Move an IK target along a path and run a polishing point-removal step at each waypoint.",
        input_model=ExecutePolishingPathInput,
        handler=execute_polishing_path,
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
