"""MCP server exposing CoppeliaSim tools."""

from __future__ import annotations

import argparse
from typing import Any

from mcp.server.fastmcp import FastMCP

from ..tools.kinematics import (
    actuate_gripper,
    actuate_youbot_gripper,
    get_joint_position,
    move_ik_target,
    set_joint_position,
    set_joint_target_position,
    set_joint_target_velocity,
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

    @mcp.tool(name="setup_ik_link", description="Set up IK chain base-tip-target.")
    def setup_ik_link_tool(
        base_handle: int,
        tip_handle: int,
        target_handle: int,
        constraints_mask: int | None = None,
    ) -> dict[str, Any]:
        return setup_ik_link(
            base_handle=base_handle,
            tip_handle=tip_handle,
            target_handle=target_handle,
            constraints_mask=constraints_mask,
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
