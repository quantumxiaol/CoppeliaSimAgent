"""MCP server exposing CoppeliaSim tools."""

from __future__ import annotations

import argparse
from typing import Any

from mcp.server.fastmcp import FastMCP

from ..tools.diagnostics import collect_remote_api_diagnostics
from ..tools.dynamics import get_object_velocity, reset_dynamic_object, set_shape_dynamics
from ..tools.grasp import attach_object_to_gripper, detach_object, grasp_object, release_object
from ..tools.kinematics import (
    actuate_gripper,
    actuate_youbot_gripper,
    configure_abb_arm_drive,
    drive_youbot_base,
    find_robot_joints,
    get_joint_dyn_ctrl_mode,
    get_joint_force,
    get_joint_mode,
    get_joint_position,
    get_joint_target_force,
    move_ik_target,
    move_ik_target_checked,
    set_joint_dyn_ctrl_mode,
    set_joint_mode,
    set_joint_position,
    set_joint_target_force,
    set_joint_target_position,
    set_joint_target_velocity,
    set_youbot_base_locked,
    set_youbot_wheel_velocities,
    stop_youbot_base,
    setup_abb_arm_ik,
    setup_ik_link,
    setup_youbot_arm_ik,
    spawn_waypoint,
)
from ..tools.models import load_model, set_parent_child
from ..tools.point_cloud import (
    create_point_cloud_pottery_cylinder,
    create_point_cloud_surface_from_shape,
    execute_polishing_groove,
    execute_polishing_path,
    get_point_cloud_stats,
    insert_points_into_point_cloud,
    remove_points_near_tool,
    simulate_polishing_contact,
    simulate_polishing_step,
)
from ..tools.primitives import (
    duplicate_object,
    remove_object,
    rename_object,
    set_object_color,
    set_object_pose,
    set_object_visibility,
    spawn_cuboid,
    spawn_composite_object,
    spawn_physics_proxy,
    spawn_primitive,
    spawn_visual_cylinder,
    spawn_visual_primitive,
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
from ..tools.task_skills import create_pusher_tool_for_abb, create_tabletop_push_scene, push_object_with_abb
from ..tools.trajectory import execute_cartesian_waypoints, execute_joint_trajectory, execute_stepped_ik_path_checked
from ..tools.verification import (
    verify_force_threshold,
    verify_joint_positions_reached,
    verify_object_moved,
    verify_object_velocity_below,
)


def create_mcp_server(*, host: str = "127.0.0.1", port: int = 7777, debug: bool = False) -> FastMCP:
    """Create MCP server and register all CoppeliaSim tools."""
    mcp = FastMCP(
        name="CoppeliaSimAgent MCP Server",
        instructions=(
            "MCP server for controlling CoppeliaSim scenes. "
            "All positions use [x, y, z] in meters, and z is the height axis."
        ),
        host=host,
        port=port,
        debug=debug,
        sse_path="/sse",
        message_path="/messages/",
        streamable_http_path="/mcp",
        json_response=True,
        stateless_http=True,
    )

    @mcp.tool(name="get_scene_graph", description="Get scene graph (name, handle, pose).")
    def get_scene_graph_tool(
        include_types: list[str] | None = None,
        round_digits: int = 3,
    ) -> dict[str, dict[str, object]]:
        return get_scene_graph(include_types=include_types, round_digits=round_digits)

    @mcp.tool(name="get_simulation_state", description="Read simulation lifecycle state.")
    def get_simulation_state_tool() -> dict[str, int | str | bool]:
        return get_simulation_state()

    @mcp.tool(name="get_plugin_status", description="Read simulator-side plugin availability.")
    def get_plugin_status_tool(
        plugin_names: list[str] | None = None,
        refresh: bool = False,
    ) -> dict[str, object]:
        return get_plugin_status(plugin_names=plugin_names, refresh=refresh)

    @mcp.tool(name="collect_remote_api_diagnostics", description="Collect Remote API failure diagnostics.")
    def collect_remote_api_diagnostics_tool(
        host: str | None = None,
        port: int | None = None,
        timeout_s: float = 3.0,
        plugin_names: list[str] | None = None,
        include_scene_sample: bool = True,
        object_name_queries: list[str] | None = None,
        scene_sample_limit: int = 40,
        include_process_probe: bool = True,
        probe_step: bool = False,
        stale_toolcli_min_age_s: float = 30.0,
        cleanup_stale_toolcli: bool = False,
        record_log: bool = True,
        log_path: str | None = "/tmp/coppeliasimagent_remote_api_diagnostics.jsonl",
    ) -> dict[str, object]:
        return collect_remote_api_diagnostics(
            host=host,
            port=port,
            timeout_s=timeout_s,
            plugin_names=plugin_names,
            include_scene_sample=include_scene_sample,
            object_name_queries=object_name_queries,
            scene_sample_limit=scene_sample_limit,
            include_process_probe=include_process_probe,
            probe_step=probe_step,
            stale_toolcli_min_age_s=stale_toolcli_min_age_s,
            cleanup_stale_toolcli=cleanup_stale_toolcli,
            record_log=record_log,
            log_path=log_path,
        )

    @mcp.tool(name="start_simulation", description="Start or resume simulation execution.")
    def start_simulation_tool() -> dict[str, int | str | bool]:
        return start_simulation()

    @mcp.tool(name="pause_simulation", description="Pause simulation execution.")
    def pause_simulation_tool() -> dict[str, int | str | bool]:
        return pause_simulation()

    @mcp.tool(name="stop_simulation", description="Stop simulation execution.")
    def stop_simulation_tool() -> dict[str, int | str | bool]:
        return stop_simulation()

    @mcp.tool(name="find_objects", description="Find scene objects by name query and type filters.")
    def find_objects_tool(
        name_query: str | None = None,
        exact_name: bool = False,
        include_types: list[str] | None = None,
        round_digits: int = 3,
        limit: int = 20,
    ) -> dict[str, Any]:
        items = find_objects(
            name_query=name_query,
            exact_name=exact_name,
            include_types=include_types,
            round_digits=round_digits,
            limit=limit,
        )
        return {"count": len(items), "items": items}

    @mcp.tool(name="get_object_pose", description="Read one object's pose in a reference frame.")
    def get_object_pose_tool(
        handle: int,
        relative_to: int = -1,
        round_digits: int = 3,
    ) -> dict[str, object]:
        return get_object_pose(handle=handle, relative_to=relative_to, round_digits=round_digits)

    @mcp.tool(name="get_relative_pose", description="Read target pose expressed in source object's frame.")
    def get_relative_pose_tool(
        source_handle: int,
        target_handle: int,
        round_digits: int = 3,
    ) -> dict[str, object]:
        return get_relative_pose(
            source_handle=source_handle,
            target_handle=target_handle,
            round_digits=round_digits,
        )

    @mcp.tool(name="check_collision", description="Check collision between two entities.")
    def check_collision_tool(entity1: int, entity2: int) -> dict[str, object]:
        collides = check_collision(entity1=entity1, entity2=entity2)
        return {"entity1": entity1, "entity2": entity2, "collides": collides}

    @mcp.tool(name="spawn_primitive", description="Spawn a primitive shape.")
    def spawn_primitive_tool(
        primitive: str,
        size: list[float],
        position: list[float],
        color: list[float] | None = None,
        dynamic: bool = True,
        relative_to: int = -1,
    ) -> dict[str, int]:
        handle = spawn_primitive(
            primitive=primitive,
            size=size,
            position=position,
            color=color,
            dynamic=dynamic,
            relative_to=relative_to,
        )
        return {"handle": handle}

    @mcp.tool(name="spawn_visual_primitive", description="Spawn a visual-only primitive.")
    def spawn_visual_primitive_tool(
        primitive: str,
        size: list[float],
        position: list[float],
        color: list[float] | None = None,
        relative_to: int = -1,
        alias: str | None = None,
        visible: bool = True,
    ) -> dict[str, Any]:
        return spawn_visual_primitive(
            primitive=primitive,
            size=size,
            position=position,
            color=color,
            relative_to=relative_to,
            alias=alias,
            visible=visible,
        )

    @mcp.tool(name="spawn_visual_cylinder", description="Spawn a true visual cylinder without physics substitution.")
    def spawn_visual_cylinder_tool(
        radius: float,
        height: float,
        position: list[float],
        color: list[float] | None = None,
        relative_to: int = -1,
        alias: str | None = None,
        visible: bool = True,
    ) -> dict[str, Any]:
        return spawn_visual_cylinder(
            radius=radius,
            height=height,
            position=position,
            color=color,
            relative_to=relative_to,
            alias=alias,
            visible=visible,
        )

    @mcp.tool(name="spawn_physics_proxy", description="Spawn a respondable physics proxy shape.")
    def spawn_physics_proxy_tool(
        proxy_type: str,
        size: list[float],
        position: list[float],
        color: list[float] | None = None,
        dynamic: bool = True,
        respondable: bool = True,
        visible: bool = False,
        relative_to: int = -1,
        alias: str | None = None,
        mass: float | None = None,
        friction: float | None = None,
    ) -> dict[str, Any]:
        return spawn_physics_proxy(
            proxy_type=proxy_type,
            size=size,
            position=position,
            color=color,
            dynamic=dynamic,
            respondable=respondable,
            visible=visible,
            relative_to=relative_to,
            alias=alias,
            mass=mass,
            friction=friction,
        )

    @mcp.tool(name="spawn_composite_object", description="Create visual geometry parented to a physics proxy.")
    def spawn_composite_object_tool(
        visual_primitive: str = "cylinder",
        proxy_type: str = "cylinder_proxy",
        size: list[float] | None = None,
        position: list[float] | None = None,
        visual_color: list[float] | None = None,
        proxy_color: list[float] | None = None,
        dynamic: bool = True,
        visible_proxy: bool = False,
        relative_to: int = -1,
        alias: str = "composite_object",
        mass: float | None = None,
        friction: float | None = None,
    ) -> dict[str, Any]:
        return spawn_composite_object(
            visual_primitive=visual_primitive,
            proxy_type=proxy_type,
            size=size,
            position=position,
            visual_color=visual_color,
            proxy_color=proxy_color,
            dynamic=dynamic,
            visible_proxy=visible_proxy,
            relative_to=relative_to,
            alias=alias,
            mass=mass,
            friction=friction,
        )

    @mcp.tool(name="spawn_cuboid", description="Spawn a cuboid.")
    def spawn_cuboid_tool(
        size: list[float],
        position: list[float],
        color: list[float] | None = None,
        dynamic: bool = True,
        relative_to: int = -1,
    ) -> dict[str, int]:
        handle = spawn_cuboid(
            size=size,
            position=position,
            color=color,
            dynamic=dynamic,
            relative_to=relative_to,
        )
        return {"handle": handle}

    @mcp.tool(name="set_object_pose", description="Set object pose with optional orientation (degrees).")
    def set_object_pose_tool(
        handle: int,
        position: list[float] | None = None,
        orientation_deg: list[float] | None = None,
        relative_to: int = -1,
    ) -> dict[str, int]:
        out_handle = set_object_pose(
            handle=handle,
            position=position,
            orientation_deg=orientation_deg,
            relative_to=relative_to,
        )
        return {"handle": out_handle}

    @mcp.tool(name="remove_object", description="Remove one object by handle.")
    def remove_object_tool(handle: int) -> dict[str, Any]:
        remove_object(handle=handle)
        return {"status": "success", "handle": handle}

    @mcp.tool(name="duplicate_object", description="Duplicate one object by handle with optional move.")
    def duplicate_object_tool(
        handle: int,
        position: list[float] | None = None,
        offset: list[float] | None = None,
        relative_to: int = -1,
    ) -> dict[str, int]:
        out_handle = duplicate_object(
            handle=handle,
            position=position,
            offset=offset,
            relative_to=relative_to,
        )
        return {"source_handle": handle, "new_handle": out_handle, "handle": out_handle}

    @mcp.tool(name="rename_object", description="Rename one object alias by handle.")
    def rename_object_tool(handle: int, new_alias: str) -> dict[str, Any]:
        alias = rename_object(handle=handle, new_alias=new_alias)
        return {"handle": handle, "new_alias": alias}

    @mcp.tool(name="set_object_color", description="Set one shape object color by handle.")
    def set_object_color_tool(
        handle: int,
        color: list[float],
        color_name: str | None = None,
        color_component: str = "ambient_diffuse",
    ) -> dict[str, Any]:
        out_handles = set_object_color(
            handle=handle,
            color=color,
            color_name=color_name,
            color_component=color_component,
        )
        return {
            "target_handle": handle,
            "applied_handles": out_handles,
            "handle": out_handles[0] if out_handles else handle,
            "color": color,
            "color_name": color_name,
            "color_component": color_component,
        }

    @mcp.tool(name="set_object_visibility", description="Show or hide an object, optionally including descendants.")
    def set_object_visibility_tool(
        handle: int,
        visible: bool,
        include_descendants: bool = False,
    ) -> dict[str, Any]:
        return set_object_visibility(
            handle=handle,
            visible=visible,
            include_descendants=include_descendants,
        )

    @mcp.tool(name="load_model", description="Load a .ttm model and place it.")
    def load_model_tool(
        model_path: str,
        position: list[float],
        orientation_deg: list[float] | None = None,
        relative_to: int = -1,
    ) -> dict[str, int]:
        handle = load_model(
            model_path=model_path,
            position=position,
            orientation_deg=orientation_deg,
            relative_to=relative_to,
        )
        return {"handle": handle}

    @mcp.tool(name="set_parent_child", description="Set parent-child relation between two handles.")
    def set_parent_child_tool(child_handle: int, parent_handle: int, keep_in_place: bool = True) -> dict[str, Any]:
        set_parent_child(
            child_handle=child_handle,
            parent_handle=parent_handle,
            keep_in_place=keep_in_place,
        )
        return {
            "status": "success",
            "child_handle": child_handle,
            "parent_handle": parent_handle,
            "keep_in_place": keep_in_place,
        }

    @mcp.tool(name="spawn_waypoint", description="Spawn a Dummy waypoint for IK target.")
    def spawn_waypoint_tool(position: list[float], size: float = 0.02, relative_to: int = -1) -> dict[str, int]:
        handle = spawn_waypoint(position=position, size=size, relative_to=relative_to)
        return {"handle": handle}

    @mcp.tool(name="get_joint_position", description="Read one joint's current position.")
    def get_joint_position_tool(handle: int) -> dict[str, Any]:
        position = get_joint_position(handle=handle)
        return {"handle": handle, "position": position}

    @mcp.tool(name="get_joint_mode", description="Read one joint's operation mode.")
    def get_joint_mode_tool(handle: int) -> dict[str, Any]:
        mode = get_joint_mode(handle=handle)
        return {"handle": handle, "joint_mode": mode}

    @mcp.tool(name="set_joint_mode", description="Set one joint's operation mode.")
    def set_joint_mode_tool(handle: int, joint_mode: str) -> dict[str, Any]:
        applied = set_joint_mode(handle=handle, joint_mode=joint_mode)
        return {"handle": handle, "joint_mode": joint_mode, "joint_mode_value": applied}

    @mcp.tool(name="get_joint_dyn_ctrl_mode", description="Read one joint's dynamic control mode.")
    def get_joint_dyn_ctrl_mode_tool(handle: int) -> dict[str, Any]:
        mode = get_joint_dyn_ctrl_mode(handle=handle)
        return {"handle": handle, "dyn_ctrl_mode": mode}

    @mcp.tool(name="set_joint_dyn_ctrl_mode", description="Set one joint's dynamic control mode.")
    def set_joint_dyn_ctrl_mode_tool(handle: int, dyn_ctrl_mode: str) -> dict[str, Any]:
        applied = set_joint_dyn_ctrl_mode(handle=handle, dyn_ctrl_mode=dyn_ctrl_mode)
        return {"handle": handle, "dyn_ctrl_mode": dyn_ctrl_mode, "dyn_ctrl_mode_value": applied}

    @mcp.tool(name="set_joint_position", description="Set one joint's immediate position.")
    def set_joint_position_tool(handle: int, position: float) -> dict[str, Any]:
        applied = set_joint_position(handle=handle, position=position)
        return {"handle": handle, "position": applied}

    @mcp.tool(name="set_joint_target_position", description="Set one joint's target position.")
    def set_joint_target_position_tool(
        handle: int,
        target_position: float,
        motion_params: list[float] | None = None,
    ) -> dict[str, Any]:
        applied = set_joint_target_position(
            handle=handle,
            target_position=target_position,
            motion_params=motion_params,
        )
        return {"handle": handle, "target_position": applied}

    @mcp.tool(name="get_joint_target_force", description="Read one joint's maximum force or torque setting.")
    def get_joint_target_force_tool(handle: int) -> dict[str, Any]:
        force_or_torque = get_joint_target_force(handle=handle)
        return {"handle": handle, "force_or_torque": force_or_torque}

    @mcp.tool(name="set_joint_target_force", description="Set one joint's maximum force or torque.")
    def set_joint_target_force_tool(
        handle: int,
        force_or_torque: float,
        signed_value: bool = True,
    ) -> dict[str, Any]:
        applied = set_joint_target_force(
            handle=handle,
            force_or_torque=force_or_torque,
            signed_value=signed_value,
        )
        return {"handle": handle, "force_or_torque": applied, "signed_value": signed_value}

    @mcp.tool(name="get_joint_force", description="Read one joint's currently applied force or torque.")
    def get_joint_force_tool(handle: int) -> dict[str, Any]:
        force_or_torque = get_joint_force(handle=handle)
        return {"handle": handle, "force_or_torque": force_or_torque}

    @mcp.tool(name="set_joint_target_velocity", description="Set one joint's target velocity.")
    def set_joint_target_velocity_tool(
        handle: int,
        target_velocity: float,
        motion_params: list[float] | None = None,
    ) -> dict[str, Any]:
        applied = set_joint_target_velocity(
            handle=handle,
            target_velocity=target_velocity,
            motion_params=motion_params,
        )
        return {"handle": handle, "target_velocity": applied}

    @mcp.tool(name="set_youbot_wheel_velocities", description="Set the four youBot wheel joint target velocities explicitly.")
    def set_youbot_wheel_velocities_tool(
        robot_path: str = "/youBot",
        wheel_velocities: list[float] | None = None,
        motion_params: list[float] | None = None,
    ) -> dict[str, Any]:
        return set_youbot_wheel_velocities(
            robot_path=robot_path,
            wheel_velocities=wheel_velocities,
            motion_params=motion_params,
        )

    @mcp.tool(name="drive_youbot_base", description="Map youBot forward/lateral/yaw commands to wheel target velocities.")
    def drive_youbot_base_tool(
        robot_path: str = "/youBot",
        forward_velocity: float = 0.0,
        lateral_velocity: float = 0.0,
        yaw_velocity: float = 0.0,
        motion_params: list[float] | None = None,
    ) -> dict[str, Any]:
        return drive_youbot_base(
            robot_path=robot_path,
            forward_velocity=forward_velocity,
            lateral_velocity=lateral_velocity,
            yaw_velocity=yaw_velocity,
            motion_params=motion_params,
        )

    @mcp.tool(name="stop_youbot_base", description="Zero all four youBot wheel joint target velocities.")
    def stop_youbot_base_tool(
        robot_path: str = "/youBot",
        motion_params: list[float] | None = None,
    ) -> dict[str, Any]:
        return stop_youbot_base(robot_path=robot_path, motion_params=motion_params)

    @mcp.tool(name="set_youbot_base_locked", description="Toggle a fixed-base mode for the youBot platform and wheels.")
    def set_youbot_base_locked_tool(
        robot_path: str = "/youBot",
        locked: bool = True,
        base_shape_paths: list[str] | None = None,
        zero_wheels: bool = True,
        reset_dynamics: bool = True,
        motion_params: list[float] | None = None,
    ) -> dict[str, Any]:
        return set_youbot_base_locked(
            robot_path=robot_path,
            locked=locked,
            base_shape_paths=base_shape_paths,
            zero_wheels=zero_wheels,
            reset_dynamics=reset_dynamics,
            motion_params=motion_params,
        )

    @mcp.tool(
        name="configure_abb_arm_drive",
        description="Configure ABB IRB4600 joints for dynamic drive with force and control mode settings.",
    )
    def configure_abb_arm_drive_tool(
        robot_path: str = "/IRB4600",
        joint_mode: str = "dynamic",
        dyn_ctrl_mode: str = "position",
        max_force_or_torque: float = 2000.0,
        signed_value: bool = True,
        include_aux_joint: bool = False,
        reset_dynamics: bool = True,
    ) -> dict[str, Any]:
        return configure_abb_arm_drive(
            robot_path=robot_path,
            joint_mode=joint_mode,
            dyn_ctrl_mode=dyn_ctrl_mode,
            max_force_or_torque=max_force_or_torque,
            signed_value=signed_value,
            include_aux_joint=include_aux_joint,
            reset_dynamics=reset_dynamics,
        )

    @mcp.tool(name="create_pusher_tool_for_abb", description="Create/reuse a visible respondable ABB pusher tool.")
    def create_pusher_tool_for_abb_tool(
        robot_path: str = "/IRB4600",
        parent_path: str | None = None,
        alias: str = "pusher_tip",
        shape: str = "cuboid",
        size: list[float] | None = None,
        radius: float = 0.02,
        offset: list[float] | None = None,
        color: list[float] | None = None,
        static: bool = True,
        respondable: bool = True,
        visible: bool = True,
        reuse_existing: bool = True,
    ) -> dict[str, Any]:
        return create_pusher_tool_for_abb(
            robot_path=robot_path,
            parent_path=parent_path,
            alias=alias,
            shape=shape,
            size=size,
            radius=radius,
            offset=offset,
            color=color,
            static=static,
            respondable=respondable,
            visible=visible,
            reuse_existing=reuse_existing,
        )

    @mcp.tool(name="create_tabletop_push_scene", description="Create a table and can in an ABB-reachable push region.")
    def create_tabletop_push_scene_tool(
        robot_path: str = "/IRB4600",
        table_size: list[float] | None = None,
        table_height: float | None = None,
        object_radius: float = 0.05,
        object_height: float = 0.12,
        object_mass: float = 0.08,
        object_friction: float | None = 0.45,
        preferred_workspace: list[float] | None = None,
        push_direction: list[float] | None = None,
        contact_height_ratio: float = 0.55,
        pre_contact_clearance: float = 0.05,
        contact_margin: float = 0.005,
        push_distance: float = 0.08,
        max_tip_error: float = 0.02,
        ik_steps_per_waypoint: int = 12,
        alias_prefix: str = "push_test",
        constraint_policy: str = "position_only",
    ) -> dict[str, Any]:
        return create_tabletop_push_scene(
            robot_path=robot_path,
            table_size=table_size,
            table_height=table_height,
            object_radius=object_radius,
            object_height=object_height,
            object_mass=object_mass,
            object_friction=object_friction,
            preferred_workspace=preferred_workspace,
            push_direction=push_direction,
            contact_height_ratio=contact_height_ratio,
            pre_contact_clearance=pre_contact_clearance,
            contact_margin=contact_margin,
            push_distance=push_distance,
            max_tip_error=max_tip_error,
            ik_steps_per_waypoint=ik_steps_per_waypoint,
            alias_prefix=alias_prefix,
            constraint_policy=constraint_policy,
        )

    @mcp.tool(name="push_object_with_abb", description="Push an object with ABB using checked IK and verification.")
    def push_object_with_abb_tool(
        robot_path: str = "/IRB4600",
        object_handle: int | None = None,
        push_direction: list[float] | None = None,
        push_distance: float = 0.10,
        contact_height_ratio: float = 0.5,
        pre_contact_clearance: float = 0.04,
        contact_margin: float = 0.005,
        table_handle: int | None = None,
        pusher_tool_handle: int | None = None,
        max_tip_error: float = 0.015,
        simulation_steps_per_waypoint: int = 5,
        ik_steps_per_waypoint: int = 10,
        object_mass: float = 0.1,
        object_friction: float | None = None,
        min_moved_distance: float = 0.01,
        settle_steps: int = 20,
        constraint_policy: str = "position_only",
        preflight_only: bool = False,
        auto_create_pusher: bool = True,
        release_stepping_on_finish: bool = True,
    ) -> dict[str, Any]:
        return push_object_with_abb(
            robot_path=robot_path,
            object_handle=object_handle,
            push_direction=push_direction,
            push_distance=push_distance,
            contact_height_ratio=contact_height_ratio,
            pre_contact_clearance=pre_contact_clearance,
            contact_margin=contact_margin,
            table_handle=table_handle,
            pusher_tool_handle=pusher_tool_handle,
            max_tip_error=max_tip_error,
            simulation_steps_per_waypoint=simulation_steps_per_waypoint,
            ik_steps_per_waypoint=ik_steps_per_waypoint,
            object_mass=object_mass,
            object_friction=object_friction,
            min_moved_distance=min_moved_distance,
            settle_steps=settle_steps,
            constraint_policy=constraint_policy,
            preflight_only=preflight_only,
            auto_create_pusher=auto_create_pusher,
            release_stepping_on_finish=release_stepping_on_finish,
        )

    @mcp.tool(name="find_robot_joints", description="Find joint handles below a robot model path.")
    def find_robot_joints_tool(
        robot_path: str = "/IRB4600",
        include_aux_joint: bool = False,
    ) -> dict[str, Any]:
        return find_robot_joints(robot_path=robot_path, include_aux_joint=include_aux_joint)

    @mcp.tool(name="setup_ik_link", description="Set up IK chain base-tip-target.")
    def setup_ik_link_tool(
        base_handle: int,
        tip_handle: int,
        target_handle: int,
        constraints_mask: int | None = None,
        constraint_policy: str | None = None,
    ) -> dict[str, Any]:
        return setup_ik_link(
            base_handle=base_handle,
            tip_handle=tip_handle,
            target_handle=target_handle,
            constraints_mask=constraints_mask,
            constraint_policy=constraint_policy,
        )

    @mcp.tool(name="setup_youbot_arm_ik", description="Create/reuse youBot arm tip-target dummies and bind an IK chain.")
    def setup_youbot_arm_ik_tool(
        robot_path: str = "/youBot",
        base_path: str | None = None,
        tip_parent_path: str | None = None,
        tip_dummy_name: str = "youBotArmTip",
        target_dummy_name: str = "youBotArmTarget",
        tip_offset: list[float] | None = None,
        target_offset: list[float] | None = None,
        constraints_mask: int | None = None,
        reuse_existing: bool = True,
    ) -> dict[str, Any]:
        return setup_youbot_arm_ik(
            robot_path=robot_path,
            base_path=base_path,
            tip_parent_path=tip_parent_path,
            tip_dummy_name=tip_dummy_name,
            target_dummy_name=target_dummy_name,
            tip_offset=tip_offset,
            target_offset=target_offset,
            constraints_mask=constraints_mask,
            reuse_existing=reuse_existing,
        )

    @mcp.tool(name="setup_abb_arm_ik", description="Set up ABB IRB4600 IK and optionally verify tip motion.")
    def setup_abb_arm_ik_tool(
        robot_path: str = "/IRB4600",
        base_path: str | None = None,
        tip_path: str = "/IRB4600/IkTip",
        target_path: str = "/IRB4600/IkTarget",
        constraints_mask: int | None = None,
        constraint_policy: str | None = None,
        verify_motion: bool = True,
        test_offset: list[float] | None = None,
        restore_target: bool = True,
        detach_target_to_world: bool = True,
    ) -> dict[str, Any]:
        return setup_abb_arm_ik(
            robot_path=robot_path,
            base_path=base_path,
            tip_path=tip_path,
            target_path=target_path,
            constraints_mask=constraints_mask,
            constraint_policy=constraint_policy,
            verify_motion=verify_motion,
            test_offset=test_offset,
            restore_target=restore_target,
            detach_target_to_world=detach_target_to_world,
        )

    @mcp.tool(name="move_ik_target", description="Move IK target and solve IK.")
    def move_ik_target_tool(
        environment_handle: int,
        group_handle: int,
        target_handle: int,
        position: list[float],
        relative_to: int = -1,
        steps: int = 1,
    ) -> dict[str, Any]:
        move_ik_target(
            environment_handle=environment_handle,
            group_handle=group_handle,
            target_handle=target_handle,
            position=position,
            relative_to=relative_to,
            steps=steps,
        )
        return {
            "status": "success",
            "environment_handle": environment_handle,
            "group_handle": group_handle,
            "target_handle": target_handle,
        }

    @mcp.tool(name="move_ik_target_checked", description="Move IK target and return explicit residual diagnostics.")
    def move_ik_target_checked_tool(
        environment_handle: int,
        group_handle: int,
        target_handle: int,
        tip_handle: int,
        position: list[float],
        orientation_deg: list[float] | None = None,
        relative_to: int = -1,
        steps: int = 10,
        max_position_error: float = 0.01,
        max_orientation_error_deg: float = 5.0,
        record_joint_handles: list[int] | None = None,
        collision_pairs: list[list[int]] | None = None,
    ) -> dict[str, Any]:
        return move_ik_target_checked(
            environment_handle=environment_handle,
            group_handle=group_handle,
            target_handle=target_handle,
            tip_handle=tip_handle,
            position=position,
            orientation_deg=orientation_deg,
            relative_to=relative_to,
            steps=steps,
            max_position_error=max_position_error,
            max_orientation_error_deg=max_orientation_error_deg,
            record_joint_handles=record_joint_handles,
            collision_pairs=collision_pairs,
        )

    @mcp.tool(name="actuate_gripper", description="Set int signal for gripper open/close.")
    def actuate_gripper_tool(signal_name: str, closed: bool) -> dict[str, Any]:
        signal_value = actuate_gripper(signal_name=signal_name, closed=closed)
        return {"status": "success", "signal_name": signal_name, "value": signal_value}

    @mcp.tool(name="actuate_youbot_gripper", description="Open/close the two-jaw youBot gripper via its finger joints.")
    def actuate_youbot_gripper_tool(
        robot_path: str = "/youBot",
        closed: bool = True,
        command_mode: str = "target_position",
        joint1_open: float = 0.025,
        joint1_closed: float = 0.0,
        joint2_open: float = -0.05,
        joint2_closed: float = 0.0,
        motion_params: list[float] | None = None,
    ) -> dict[str, Any]:
        return actuate_youbot_gripper(
            robot_path=robot_path,
            closed=closed,
            command_mode=command_mode,
            joint1_open=joint1_open,
            joint1_closed=joint1_closed,
            joint2_open=joint2_open,
            joint2_closed=joint2_closed,
            motion_params=motion_params,
        )

    @mcp.tool(name="step_simulation", description="Advance simulation by remote stepping ticks when supported.")
    def step_simulation_tool(
        steps: int = 1,
        start_if_stopped: bool = True,
        keep_stepping_enabled: bool = True,
    ) -> dict[str, Any]:
        return step_simulation(
            steps=steps,
            start_if_stopped=start_if_stopped,
            keep_stepping_enabled=keep_stepping_enabled,
        )

    @mcp.tool(name="wait_seconds", description="Wait in the external agent process.")
    def wait_seconds_tool(seconds: float) -> dict[str, float]:
        return wait_seconds(seconds=seconds)

    @mcp.tool(name="wait_until_state", description="Poll simulation lifecycle state until target state.")
    def wait_until_state_tool(
        target_state: str,
        timeout_s: float = 5.0,
        poll_interval_s: float = 0.05,
    ) -> dict[str, Any]:
        return wait_until_state(
            target_state=target_state,
            timeout_s=timeout_s,
            poll_interval_s=poll_interval_s,
        )

    @mcp.tool(name="wait_until_object_pose_stable", description="Wait until an object's pose is stable.")
    def wait_until_object_pose_stable_tool(
        handle: int,
        position_tolerance: float = 0.001,
        orientation_tolerance_deg: float = 0.1,
        stable_duration_s: float = 0.25,
        timeout_s: float = 5.0,
        poll_interval_s: float = 0.05,
        relative_to: int = -1,
    ) -> dict[str, Any]:
        return wait_until_object_pose_stable(
            handle=handle,
            position_tolerance=position_tolerance,
            orientation_tolerance_deg=orientation_tolerance_deg,
            stable_duration_s=stable_duration_s,
            timeout_s=timeout_s,
            poll_interval_s=poll_interval_s,
            relative_to=relative_to,
        )

    @mcp.tool(name="execute_joint_trajectory", description="Execute joint-space waypoints.")
    def execute_joint_trajectory_tool(
        joint_handles: list[int],
        waypoints: list[list[float]],
        mode: str = "target_position",
        dwell_seconds: float = 0.0,
        motion_params: list[float] | None = None,
    ) -> dict[str, Any]:
        return execute_joint_trajectory(
            joint_handles=joint_handles,
            waypoints=waypoints,
            mode=mode,
            dwell_seconds=dwell_seconds,
            motion_params=motion_params,
        )

    @mcp.tool(name="execute_cartesian_waypoints", description="Move an IK target through Cartesian waypoints.")
    def execute_cartesian_waypoints_tool(
        environment_handle: int,
        group_handle: int,
        target_handle: int,
        waypoints: list[list[float]],
        relative_to: int = -1,
        steps_per_waypoint: int = 1,
        dwell_seconds: float = 0.0,
    ) -> dict[str, Any]:
        return execute_cartesian_waypoints(
            environment_handle=environment_handle,
            group_handle=group_handle,
            target_handle=target_handle,
            waypoints=waypoints,
            relative_to=relative_to,
            steps_per_waypoint=steps_per_waypoint,
            dwell_seconds=dwell_seconds,
        )

    @mcp.tool(name="execute_stepped_ik_path_checked", description="Execute checked IK waypoints with simulation stepping.")
    def execute_stepped_ik_path_checked_tool(
        environment_handle: int,
        group_handle: int,
        target_handle: int,
        tip_handle: int,
        waypoints: list[list[float]],
        orientation_deg: list[float] | None = None,
        relative_to: int = -1,
        ik_steps_per_waypoint: int = 10,
        simulation_steps_per_waypoint: int = 1,
        start_simulation: bool = True,
        keep_stepping_enabled: bool = True,
        max_position_error: float = 0.01,
        max_orientation_error_deg: float = 5.0,
        record_joint_handles: list[int] | None = None,
        record_handles: list[int] | None = None,
        record_every: int = 1,
        stop_on_failure: bool = True,
        collision_pairs: list[list[int]] | None = None,
    ) -> dict[str, Any]:
        return execute_stepped_ik_path_checked(
            environment_handle=environment_handle,
            group_handle=group_handle,
            target_handle=target_handle,
            tip_handle=tip_handle,
            waypoints=waypoints,
            orientation_deg=orientation_deg,
            relative_to=relative_to,
            ik_steps_per_waypoint=ik_steps_per_waypoint,
            simulation_steps_per_waypoint=simulation_steps_per_waypoint,
            start_simulation=start_simulation,
            keep_stepping_enabled=keep_stepping_enabled,
            max_position_error=max_position_error,
            max_orientation_error_deg=max_orientation_error_deg,
            record_joint_handles=record_joint_handles,
            record_handles=record_handles,
            record_every=record_every,
            stop_on_failure=stop_on_failure,
            collision_pairs=collision_pairs,
        )

    @mcp.tool(name="get_object_velocity", description="Read object linear/angular velocity.")
    def get_object_velocity_tool(handle: int) -> dict[str, Any]:
        return get_object_velocity(handle=handle)

    @mcp.tool(name="reset_dynamic_object", description="Reset dynamic state for an object/model.")
    def reset_dynamic_object_tool(handle: int, include_model: bool = True) -> dict[str, Any]:
        return reset_dynamic_object(handle=handle, include_model=include_model)

    @mcp.tool(name="set_shape_dynamics", description="Set shape dynamic flags and optional mass/friction.")
    def set_shape_dynamics_tool(
        handle: int,
        static: bool | None = None,
        respondable: bool | None = None,
        mass: float | None = None,
        friction: float | None = None,
    ) -> dict[str, Any]:
        return set_shape_dynamics(
            handle=handle,
            static=static,
            respondable=respondable,
            mass=mass,
            friction=friction,
        )

    @mcp.tool(name="verify_joint_positions_reached", description="Verify joints reached target positions.")
    def verify_joint_positions_reached_tool(
        joint_handles: list[int],
        target_positions: list[float],
        tolerance: float = 0.01,
    ) -> dict[str, Any]:
        return verify_joint_positions_reached(
            joint_handles=joint_handles,
            target_positions=target_positions,
            tolerance=tolerance,
        )

    @mcp.tool(name="verify_object_moved", description="Verify an object moved from a start position.")
    def verify_object_moved_tool(
        handle: int,
        start_position: list[float],
        min_distance: float = 0.01,
        relative_to: int = -1,
    ) -> dict[str, Any]:
        return verify_object_moved(
            handle=handle,
            start_position=start_position,
            min_distance=min_distance,
            relative_to=relative_to,
        )

    @mcp.tool(name="verify_object_velocity_below", description="Verify object velocity is below thresholds.")
    def verify_object_velocity_below_tool(
        handle: int,
        max_linear_speed: float = 0.01,
        max_angular_speed: float = 0.01,
    ) -> dict[str, Any]:
        return verify_object_velocity_below(
            handle=handle,
            max_linear_speed=max_linear_speed,
            max_angular_speed=max_angular_speed,
        )

    @mcp.tool(name="verify_force_threshold", description="Verify joint force threshold.")
    def verify_force_threshold_tool(joint_handles: list[int], min_abs_force: float = 0.1) -> dict[str, Any]:
        return verify_force_threshold(joint_handles=joint_handles, min_abs_force=min_abs_force)

    @mcp.tool(name="attach_object_to_gripper", description="Parent an object under a gripper/tip handle.")
    def attach_object_to_gripper_tool(
        object_handle: int,
        gripper_handle: int,
        keep_in_place: bool = True,
    ) -> dict[str, Any]:
        return attach_object_to_gripper(
            object_handle=object_handle,
            gripper_handle=gripper_handle,
            keep_in_place=keep_in_place,
        )

    @mcp.tool(name="detach_object", description="Detach an object to world or another parent.")
    def detach_object_tool(
        object_handle: int,
        parent_handle: int = -1,
        keep_in_place: bool = True,
    ) -> dict[str, Any]:
        return detach_object(object_handle=object_handle, parent_handle=parent_handle, keep_in_place=keep_in_place)

    @mcp.tool(name="grasp_object", description="Close a gripper command path and optionally attach an object.")
    def grasp_object_tool(
        object_handle: int,
        gripper_handle: int,
        signal_name: str | None = None,
        robot_path: str | None = None,
        close_gripper: bool = True,
        attach: bool = True,
        keep_in_place: bool = True,
    ) -> dict[str, Any]:
        return grasp_object(
            object_handle=object_handle,
            gripper_handle=gripper_handle,
            signal_name=signal_name,
            robot_path=robot_path,
            close_gripper=close_gripper,
            attach=attach,
            keep_in_place=keep_in_place,
        )

    @mcp.tool(name="release_object", description="Open a gripper command path and optionally detach an object.")
    def release_object_tool(
        object_handle: int,
        signal_name: str | None = None,
        robot_path: str | None = None,
        parent_handle: int = -1,
        open_gripper: bool = True,
        detach: bool = True,
        keep_in_place: bool = True,
    ) -> dict[str, Any]:
        return release_object(
            object_handle=object_handle,
            signal_name=signal_name,
            robot_path=robot_path,
            parent_handle=parent_handle,
            open_gripper=open_gripper,
            detach=detach,
            keep_in_place=keep_in_place,
        )

    @mcp.tool(name="read_proximity_sensor", description="Read one proximity sensor.")
    def read_proximity_sensor_tool(handle: int) -> dict[str, Any]:
        return read_proximity_sensor(handle=handle)

    @mcp.tool(name="read_force_sensor", description="Read one force sensor.")
    def read_force_sensor_tool(handle: int) -> dict[str, Any]:
        return read_force_sensor(handle=handle)

    @mcp.tool(name="get_vision_sensor_image", description="Read one vision sensor image or metadata.")
    def get_vision_sensor_image_tool(
        handle: int,
        grayscale: bool = False,
        metadata_only: bool = True,
    ) -> dict[str, Any]:
        return get_vision_sensor_image(handle=handle, grayscale=grayscale, metadata_only=metadata_only)

    @mcp.tool(name="check_collision_monitor", description="Monitor a collision pair for a duration.")
    def check_collision_monitor_tool(
        entity1: int,
        entity2: int,
        duration_s: float = 0.0,
        poll_interval_s: float = 0.05,
    ) -> dict[str, Any]:
        return check_collision_monitor(
            entity1=entity1,
            entity2=entity2,
            duration_s=duration_s,
            poll_interval_s=poll_interval_s,
        )

    @mcp.tool(name="create_point_cloud_surface_from_shape", description="Create a point-cloud surface from a shape.")
    def create_point_cloud_surface_from_shape_tool(
        shape_handle: int,
        grid_size: float = 0.02,
        point_size: float = 0.01,
        color: list[float] | None = None,
        hide_source_shape: bool = False,
        remove_source_shape: bool = False,
    ) -> dict[str, Any]:
        return create_point_cloud_surface_from_shape(
            shape_handle=shape_handle,
            grid_size=grid_size,
            point_size=point_size,
            color=color,
            hide_source_shape=hide_source_shape,
            remove_source_shape=remove_source_shape,
        )

    @mcp.tool(name="create_point_cloud_pottery_cylinder", description="Create a point-cloud pottery cylinder.")
    def create_point_cloud_pottery_cylinder_tool(
        radius: float = 0.11,
        height: float = 0.45,
        center: list[float] | None = None,
        grid_size: float = 0.015,
        point_size: float = 0.008,
        color: list[float] | None = None,
        alias: str = "point_cloud_pottery_cylinder",
        keep_source_shape: bool = False,
        layers: int = 1,
        wall_thickness: float = 0.0,
        angular_step_deg: float | None = None,
        include_caps: bool = True,
        use_explicit_points: bool = True,
    ) -> dict[str, Any]:
        return create_point_cloud_pottery_cylinder(
            radius=radius,
            height=height,
            center=center,
            grid_size=grid_size,
            point_size=point_size,
            color=color,
            alias=alias,
            keep_source_shape=keep_source_shape,
            layers=layers,
            wall_thickness=wall_thickness,
            angular_step_deg=angular_step_deg,
            include_caps=include_caps,
            use_explicit_points=use_explicit_points,
        )

    @mcp.tool(name="insert_points_into_point_cloud", description="Insert points into a point cloud.")
    def insert_points_into_point_cloud_tool(
        point_cloud_handle: int,
        points: list[list[float]],
        color: list[float] | None = None,
    ) -> dict[str, Any]:
        return insert_points_into_point_cloud(point_cloud_handle=point_cloud_handle, points=points, color=color)

    @mcp.tool(name="remove_points_near_tool", description="Remove point-cloud points near a tool.")
    def remove_points_near_tool_tool(
        point_cloud_handle: int,
        tool_handle: int,
        radius: float,
        tolerance: float = 0.0,
    ) -> dict[str, Any]:
        return remove_points_near_tool(
            point_cloud_handle=point_cloud_handle,
            tool_handle=tool_handle,
            radius=radius,
            tolerance=tolerance,
        )

    @mcp.tool(name="get_point_cloud_stats", description="Read known point-cloud stats.")
    def get_point_cloud_stats_tool(point_cloud_handle: int) -> dict[str, Any]:
        return get_point_cloud_stats(point_cloud_handle=point_cloud_handle)

    @mcp.tool(name="simulate_polishing_step", description="Remove surface points near the polishing tool.")
    def simulate_polishing_step_tool(
        tool_handle: int,
        surface_cloud_handle: int,
        contact_radius: float,
        removal_depth: float = 0.0,
    ) -> dict[str, Any]:
        return simulate_polishing_step(
            tool_handle=tool_handle,
            surface_cloud_handle=surface_cloud_handle,
            contact_radius=contact_radius,
            removal_depth=removal_depth,
        )

    @mcp.tool(name="simulate_polishing_contact", description="Remove point-cloud points around an explicit contact position.")
    def simulate_polishing_contact_tool(
        surface_cloud_handle: int,
        tool_position: list[float],
        contact_radius: float,
        removal_depth: float = 0.0,
    ) -> dict[str, Any]:
        return simulate_polishing_contact(
            surface_cloud_handle=surface_cloud_handle,
            tool_position=tool_position,
            contact_radius=contact_radius,
            removal_depth=removal_depth,
        )

    @mcp.tool(name="execute_polishing_groove", description="Cut a visible point-cloud groove along a line segment.")
    def execute_polishing_groove_tool(
        surface_cloud_handle: int,
        start_position: list[float],
        end_position: list[float],
        contact_radius: float,
        removal_depth: float = 0.0,
        steps: int = 12,
    ) -> dict[str, Any]:
        return execute_polishing_groove(
            surface_cloud_handle=surface_cloud_handle,
            start_position=start_position,
            end_position=end_position,
            contact_radius=contact_radius,
            removal_depth=removal_depth,
            steps=steps,
        )

    @mcp.tool(name="execute_polishing_path", description="Move an IK target and polish at each waypoint.")
    def execute_polishing_path_tool(
        environment_handle: int,
        group_handle: int,
        target_handle: int,
        tool_handle: int,
        surface_cloud_handle: int,
        waypoints: list[list[float]],
        contact_radius: float,
        removal_depth: float = 0.0,
        relative_to: int = -1,
        steps_per_waypoint: int = 1,
        dwell_seconds: float = 0.0,
    ) -> dict[str, Any]:
        return execute_polishing_path(
            environment_handle=environment_handle,
            group_handle=group_handle,
            target_handle=target_handle,
            tool_handle=tool_handle,
            surface_cloud_handle=surface_cloud_handle,
            waypoints=waypoints,
            contact_radius=contact_radius,
            removal_depth=removal_depth,
            relative_to=relative_to,
            steps_per_waypoint=steps_per_waypoint,
            dwell_seconds=dwell_seconds,
        )

    return mcp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run CoppeliaSimAgent MCP server")
    parser.add_argument(
        "--transport",
        choices=["stdio", "sse", "streamable-http"],
        default="stdio",
        help="MCP transport to use",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Host for HTTP transports")
    parser.add_argument("--port", type=int, default=7777, help="Port for HTTP transports")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    server = create_mcp_server(host=args.host, port=args.port, debug=args.debug)
    server.run(transport=args.transport)


if __name__ == "__main__":
    main()
