from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from coppeliasimagent.core.exceptions import ToolValidationError
from coppeliasimagent.tools import kinematics, models, primitives, scene


class FakeSim:
    primitiveshape_cuboid = 0
    primitiveshape_sphere = 1
    primitiveshape_cylinder = 2
    colorcomponent_ambient_diffuse = 10
    colorcomponent_diffuse = 11
    colorcomponent_specular = 12
    colorcomponent_emission = 13
    colorcomponent_transparency = 14
    shapeintparam_static = 20
    shapeintparam_respondable = 21
    handleflag_model = 0x040000

    handle_scene = -2
    objintparam_type = 4000
    object_shape_type = 1
    object_dummy_type = 2
    object_joint_type = 3
    object_camera_type = 4
    object_light_type = 5
    object_forcesensor_type = 6

    def __init__(self) -> None:
        self.next_handle = 100
        self.calls: list[tuple[str, tuple]] = []
        self.object_types = {
            1: self.object_shape_type,
            2: self.object_dummy_type,
            3: self.object_joint_type,
            4: self.object_dummy_type,
            5: self.object_shape_type,
            10: self.object_dummy_type,
            11: self.object_joint_type,
            12: self.object_shape_type,
            13: self.object_joint_type,
            14: self.object_joint_type,
            15: self.object_joint_type,
            16: self.object_joint_type,
            17: self.object_joint_type,
            18: self.object_joint_type,
            19: self.object_shape_type,
            20: self.object_shape_type,
        }
        self.object_names = {
            1: "shape_a",
            2: "dummy_a",
            3: "joint_a",
            4: "model_base",
            5: "model_shape",
            10: "youBot",
            11: "youBotArmJoint0",
            12: "Rectangle7",
            13: "youBotGripperJoint1",
            14: "youBotGripperJoint2",
            15: "rollingJoint_rr",
            16: "rollingJoint_rl",
            17: "rollingJoint_fr",
            18: "rollingJoint_fl",
            19: "ME_Platfo2_sub1",
            20: "wheel_respondable_rr",
        }
        self.object_positions = {
            1: [0.123456, 1.0, 2.0],
            2: [0.0, 0.0, 0.2],
            3: [5.0, 5.0, 5.0],
            4: [0.2, 0.2, 0.2],
            5: [0.2, 0.2, 0.2],
            10: [1.0, 0.0, 0.1],
            11: [1.0, 0.0, 0.2],
            12: [1.1, 0.0, 0.25],
            13: [1.11, 0.03, 0.22],
            14: [1.11, -0.03, 0.22],
            15: [1.0, 0.15, 0.05],
            16: [1.0, -0.15, 0.05],
            17: [1.2, 0.15, 0.05],
            18: [1.2, -0.15, 0.05],
            19: [1.0, 0.0, 0.05],
            20: [1.0, 0.15, 0.05],
        }
        self.object_orientations = {
            1: [0.0, math.pi / 2, 0.0],
            2: [0.1, 0.2, 0.3],
            3: [0.0, 0.0, 0.0],
            4: [0.0, 0.0, 0.0],
            5: [0.0, 0.0, 0.0],
            10: [0.0, 0.0, 0.0],
            11: [0.0, 0.0, 0.0],
            12: [0.0, 0.0, 0.0],
            13: [0.0, 0.0, 0.0],
            14: [0.0, 0.0, 0.0],
            15: [0.0, 0.0, 0.0],
            16: [0.0, 0.0, 0.0],
            17: [0.0, 0.0, 0.0],
            18: [0.0, 0.0, 0.0],
            19: [0.0, 0.0, 0.0],
            20: [0.0, 0.0, 0.0],
        }
        self.object_parents = {
            1: -1,
            2: -1,
            3: -1,
            4: -1,
            5: 4,
            10: -1,
            11: 10,
            12: 11,
            13: 12,
            14: 12,
            15: 10,
            16: 10,
            17: 10,
            18: 10,
            19: 10,
            20: 10,
        }
        self.collision_map = {(1, 2): 1, (1, 3): 0}
        self.joint_positions = {
            3: 0.0,
            11: 0.1,
            13: 0.025,
            14: -0.05,
            15: 0.0,
            16: 0.0,
            17: 0.0,
            18: 0.0,
        }
        self.joint_target_positions: dict[int, float] = {}
        self.joint_target_velocities: dict[int, float] = {}
        self.joint_intervals = {
            3: (False, [-1.0, 1.0]),
            11: (False, [-math.pi, math.pi]),
            13: (False, [0.0, 0.025]),
            14: (False, [-0.05, 0.05]),
            15: (False, [-10.0, 20.0]),
            16: (False, [-10.0, 20.0]),
            17: (False, [-10.0, 20.0]),
            18: (False, [-10.0, 20.0]),
        }
        self.linked_dummies: dict[int, int] = {}
        self.object_int_params: dict[tuple[int, int], int] = {}
        self.reset_dynamic_calls: list[int] = []

    def _children_of(self, parent: int) -> list[int]:
        return [handle for handle, value in self.object_parents.items() if value == parent]

    def _descendants_of(self, root: int) -> list[int]:
        out = [root]
        for child in self._children_of(root):
            out.extend(self._descendants_of(child))
        return out

    def _matches_under(self, root: int | None, alias: str) -> list[int]:
        candidates = self.object_names if root is None else {
            handle: self.object_names[handle] for handle in self._descendants_of(root)
        }
        return [handle for handle, name in candidates.items() if name == alias]

    def createPrimitiveShape(self, primitive: int, size: list[float]) -> int:  # noqa: N802
        handle = self.next_handle
        self.next_handle += 1
        self.calls.append(("createPrimitiveShape", (primitive, size)))
        self.object_types[handle] = self.object_shape_type
        self.object_names[handle] = f"Cuboid#{handle}"
        self.object_positions[handle] = [0.0, 0.0, 0.0]
        self.object_orientations[handle] = [0.0, 0.0, 0.0]
        self.object_parents[handle] = -1
        return handle

    def setObjectPosition(self, handle: int, relative_to: int, position: list[float]) -> None:  # noqa: N802
        self.calls.append(("setObjectPosition", (handle, relative_to, position)))
        if relative_to in (-1, self.handle_scene):
            self.object_positions[handle] = [float(v) for v in position]
            return

        ref = self.object_positions[relative_to]
        self.object_positions[handle] = [float(ref[i]) + float(position[i]) for i in range(3)]

    def setShapeColor(self, handle: int, component: object, color_comp: int, color: list[float]) -> None:  # noqa: N802
        self.calls.append(("setShapeColor", (handle, component, color_comp, color)))

    def setObjectInt32Param(self, handle: int, param: int, value: int) -> None:  # noqa: N802
        self.calls.append(("setObjectInt32Param", (handle, param, value)))
        self.object_int_params[(handle, param)] = value

    def setObjectOrientation(self, handle: int, relative_to: int, orientation: list[float]) -> None:  # noqa: N802
        self.calls.append(("setObjectOrientation", (handle, relative_to, orientation)))
        if relative_to in (-1, self.handle_scene):
            self.object_orientations[handle] = [float(v) for v in orientation]
            return

        ref = self.object_orientations[relative_to]
        self.object_orientations[handle] = [float(ref[i]) + float(orientation[i]) for i in range(3)]

    def removeObject(self, handle: int) -> None:  # noqa: N802
        self.calls.append(("removeObject", (handle,)))

    def removeObjects(self, handles: list[int]) -> None:  # noqa: N802
        self.calls.append(("removeObjects", (handles,)))

    def loadModel(self, model_path: str) -> int:  # noqa: N802
        self.calls.append(("loadModel", (model_path,)))
        handle = self.next_handle
        self.next_handle += 1
        self.object_parents[handle] = -1
        return handle

    def setObjectParent(self, child: int, parent: int, keep_in_place: bool) -> None:  # noqa: N802
        self.calls.append(("setObjectParent", (child, parent, keep_in_place)))
        self.object_parents[child] = parent

    def copyPasteObjects(self, handles: list[int], options: int = 0) -> list[int]:  # noqa: N802
        self.calls.append(("copyPasteObjects", (handles, options)))
        src = handles[0]
        out = self.next_handle
        self.next_handle += 1
        self.object_types[out] = self.object_types[src]
        self.object_names[out] = f"{self.object_names[src]}_copy"
        self.object_positions[out] = list(self.object_positions[src])
        self.object_orientations[out] = list(self.object_orientations[src])
        self.object_parents[out] = self.object_parents.get(src, -1)
        return [out]

    def getObjectsInTree(self, root: int) -> list[int]:  # noqa: N802
        self.calls.append(("getObjectsInTree", (root,)))
        if root == self.handle_scene:
            return sorted(self.object_names)
        if root in self.object_names:
            return self._descendants_of(root)
        return sorted(self.object_names)

    def getObjectInt32Param(self, handle: int, param: int) -> int:  # noqa: N802
        return self.object_types[handle]

    def getObjectType(self, handle: int) -> int:  # noqa: N802
        return self.object_types[handle]

    def getObject(self, object_path: str, options: dict | None = None) -> int:  # noqa: N802
        options = options or {}
        parts = [part for part in object_path.split("/") if part]
        if not parts:
            if options.get("noError"):
                return -1
            raise RuntimeError(f"invalid object path: {object_path}")

        current_matches = self._matches_under(None, parts[0])
        for part in parts[1:]:
            next_matches: list[int] = []
            for handle in current_matches:
                next_matches.extend(self._matches_under(handle, part))
            current_matches = next_matches

        if not current_matches:
            if options.get("noError"):
                return -1
            raise RuntimeError(f"object not found: {object_path}")
        return current_matches[0]

    def getObjectName(self, handle: int) -> str:  # noqa: N802
        return self.object_names[handle]

    def getObjectAlias(self, handle: int) -> str:  # noqa: N802
        return self.object_names[handle]

    def setObjectAlias(self, handle: int, alias: str) -> None:  # noqa: N802
        self.calls.append(("setObjectAlias", (handle, alias)))
        self.object_names[handle] = alias

    def getObjectPosition(self, handle: int, relative_to: int) -> list[float]:  # noqa: N802
        return self.object_positions[handle]

    def getObjectOrientation(self, handle: int, relative_to: int) -> list[float]:  # noqa: N802
        return self.object_orientations[handle]

    def getObjectParent(self, handle: int) -> int:  # noqa: N802
        return self.object_parents.get(handle, -1)

    def getObjectChild(self, handle: int, index: int) -> int:  # noqa: N802
        children = self._children_of(handle)
        if index < 0 or index >= len(children):
            return -1
        return children[index]

    def checkCollision(self, entity1: int, entity2: int) -> int:  # noqa: N802
        return self.collision_map.get((entity1, entity2), 0)

    def createDummy(self, size: float) -> int:  # noqa: N802
        self.calls.append(("createDummy", (size,)))
        handle = self.next_handle
        self.next_handle += 1
        self.object_types[handle] = self.object_dummy_type
        self.object_names[handle] = f"Dummy#{handle}"
        self.object_positions[handle] = [0.0, 0.0, 0.0]
        self.object_orientations[handle] = [0.0, 0.0, 0.0]
        self.object_parents[handle] = -1
        return handle

    def setInt32Signal(self, signal_name: str, value: int) -> None:  # noqa: N802
        self.calls.append(("setInt32Signal", (signal_name, value)))

    def getJointPosition(self, handle: int) -> float:  # noqa: N802
        return self.joint_positions[handle]

    def setJointPosition(self, handle: int, position: float) -> None:  # noqa: N802
        self.calls.append(("setJointPosition", (handle, position)))
        self.joint_positions[handle] = float(position)

    def setJointTargetPosition(self, handle: int, target: float, motion_params: list[float] | None = None) -> None:  # noqa: N802
        self.calls.append(("setJointTargetPosition", (handle, target, motion_params)))
        self.joint_target_positions[handle] = float(target)
        self.joint_positions[handle] = float(target)

    def setJointTargetVelocity(self, handle: int, target: float, motion_params: list[float] | None = None) -> None:  # noqa: N802
        self.calls.append(("setJointTargetVelocity", (handle, target, motion_params)))
        self.joint_target_velocities[handle] = float(target)

    def getJointTargetVelocity(self, handle: int) -> float:  # noqa: N802
        return self.joint_target_velocities.get(handle, 0.0)

    def getJointInterval(self, handle: int) -> tuple[bool, list[float]]:  # noqa: N802
        cyclic, interval = self.joint_intervals[handle]
        return cyclic, list(interval)

    def setLinkDummy(self, dummy_handle: int, linked_dummy_handle: int) -> None:  # noqa: N802
        self.calls.append(("setLinkDummy", (dummy_handle, linked_dummy_handle)))
        self.linked_dummies[dummy_handle] = linked_dummy_handle

    def resetDynamicObject(self, handle: int) -> int:  # noqa: N802
        self.calls.append(("resetDynamicObject", (handle,)))
        self.reset_dynamic_calls.append(handle)
        return 1


class FakeSimIK:
    constraint_x = 1
    constraint_y = 2
    constraint_z = 4
    constraint_alpha_beta = 8
    constraint_gamma = 16

    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple]] = []

    def createEnvironment(self) -> int:  # noqa: N802
        self.calls.append(("createEnvironment", ()))
        return 500

    def createGroup(self, env_handle: int) -> int:  # noqa: N802
        self.calls.append(("createGroup", (env_handle,)))
        return 501

    def addElementFromScene(
        self,
        env_handle: int,
        group_handle: int,
        base_handle: int,
        tip_handle: int,
        target_handle: int,
        constraints_mask: int,
    ) -> tuple[int, dict[int, int], dict[int, int]]:  # noqa: N802
        self.calls.append(
            (
                "addElementFromScene",
                (env_handle, group_handle, base_handle, tip_handle, target_handle, constraints_mask),
            )
        )
        return 600, {1: 2}, {2: 1}

    def handleGroup(self, env_handle: int, group_handle: int, opts: dict | None = None) -> int:  # noqa: N802
        self.calls.append(("handleGroup", (env_handle, group_handle, opts)))
        return 0


class TestTools(unittest.TestCase):
    def test_spawn_cuboid_and_pose(self) -> None:
        sim = FakeSim()
        with patch("coppeliasimagent.tools.primitives.get_sim", return_value=sim):
            handle = primitives.spawn_cuboid(size=[0.1, 0.2, 0.3], position=[0, 0, 1])
            primitives.set_object_pose(handle=handle, orientation_deg=[90, 0, 180])

        self.assertIsInstance(handle, int)
        orientation_calls = [c for c in sim.calls if c[0] == "setObjectOrientation"]
        self.assertEqual(len(orientation_calls), 1)
        _, (_, _, orientation_rad) = orientation_calls[0]
        self.assertAlmostEqual(orientation_rad[0], math.pi / 2, places=6)
        self.assertAlmostEqual(orientation_rad[2], math.pi, places=6)

    def test_spawn_cuboid_invalid_color(self) -> None:
        with self.assertRaises(ToolValidationError):
            primitives.spawn_cuboid(size=[0.1, 0.1, 0.1], position=[0, 0, 0], color=[1.5, 0, 0])

    def test_load_model_and_parent(self) -> None:
        sim = FakeSim()
        with patch("coppeliasimagent.tools.models.get_sim", return_value=sim):
            base = models.load_model("robot.ttm", [0, 0, 0], orientation_deg=[0, 90, 0])
            models.set_parent_child(child_handle=10, parent_handle=base, keep_in_place=True)

        self.assertIsInstance(base, int)
        self.assertIn(("loadModel", ("robot.ttm",)), sim.calls)
        self.assertIn(("setObjectParent", (10, base, True)), sim.calls)

    def test_remove_object_prefers_remove_objects(self) -> None:
        sim = FakeSim()
        with patch("coppeliasimagent.tools.primitives.get_sim", return_value=sim):
            primitives.remove_object(42)

        self.assertIn(("removeObjects", ([42],)), sim.calls)
        self.assertNotIn(("removeObject", (42,)), sim.calls)

    def test_scene_graph_and_collision(self) -> None:
        sim = FakeSim()
        with patch("coppeliasimagent.tools.scene.get_sim", return_value=sim):
            graph = scene.get_scene_graph()
            collides = scene.check_collision(1, 2)
            no_collide = scene.check_collision(1, 3)

        self.assertIn("shape_a", graph)
        self.assertIn("dummy_a", graph)
        self.assertNotIn("joint_a", graph)
        self.assertEqual(graph["shape_a"]["position"][0], 0.123)
        self.assertTrue(collides)
        self.assertFalse(no_collide)

    def test_find_objects_by_name(self) -> None:
        sim = FakeSim()
        with patch("coppeliasimagent.tools.scene.get_sim", return_value=sim):
            items = scene.find_objects(name_query="shape_a", include_types=["shape"], exact_name=True, limit=10)

        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["name"], "shape_a")
        self.assertEqual(items[0]["handle"], 1)

    def test_duplicate_object_with_offset(self) -> None:
        sim = FakeSim()
        with patch("coppeliasimagent.tools.primitives.get_sim", return_value=sim):
            out = primitives.duplicate_object(handle=1, offset=[0.1, 0.0, 0.0])

        self.assertIsInstance(out, int)
        self.assertIn(("copyPasteObjects", ([1], 0)), sim.calls)
        self.assertAlmostEqual(sim.object_positions[out][0], 0.223456, places=6)

    def test_duplicate_object_rejects_both_position_and_offset(self) -> None:
        with self.assertRaises(ToolValidationError):
            primitives.duplicate_object(handle=1, position=[0.0, 0.0, 0.0], offset=[0.1, 0.0, 0.0])

    def test_rename_object(self) -> None:
        sim = FakeSim()
        with patch("coppeliasimagent.tools.primitives.get_sim", return_value=sim):
            alias = primitives.rename_object(handle=1, new_alias="jar_lid_copy")

        self.assertEqual(alias, "jar_lid_copy")
        self.assertIn(("setObjectAlias", (1, "jar_lid_copy")), sim.calls)
        self.assertEqual(sim.getObjectName(1), "jar_lid_copy")

    def test_set_object_color(self) -> None:
        sim = FakeSim()
        with patch("coppeliasimagent.tools.primitives.get_sim", return_value=sim):
            out = primitives.set_object_color(
                handle=1,
                color=[0.1, 0.2, 0.9],
                color_component="ambient_diffuse",
            )

        self.assertEqual(out, [1])
        self.assertIn(("setShapeColor", (1, None, sim.colorcomponent_ambient_diffuse, [0.1, 0.2, 0.9])), sim.calls)

    def test_set_object_color_falls_back_to_descendant_shape(self) -> None:
        sim = FakeSim()
        with patch("coppeliasimagent.tools.primitives.get_sim", return_value=sim):
            out = primitives.set_object_color(handle=4, color=[0.3, 0.7, 0.1], color_component="ambient_diffuse")

        self.assertEqual(out, [5])
        self.assertIn(("setShapeColor", (5, None, sim.colorcomponent_ambient_diffuse, [0.3, 0.7, 0.1])), sim.calls)

    def test_rename_object_invalid_alias(self) -> None:
        with self.assertRaises(ToolValidationError):
            primitives.rename_object(handle=1, new_alias="   ")

    def test_kinematics_functions(self) -> None:
        sim = FakeSim()
        simik = FakeSimIK()

        with patch("coppeliasimagent.tools.kinematics.get_sim", return_value=sim), patch(
            "coppeliasimagent.tools.kinematics.get_simik", return_value=simik
        ):
            target = kinematics.spawn_waypoint([0.1, 0.0, 0.2])
            ik_info = kinematics.setup_ik_link(base_handle=1, tip_handle=2, target_handle=target)
            kinematics.move_ik_target(
                environment_handle=ik_info["environment_handle"],
                group_handle=ik_info["group_handle"],
                target_handle=target,
                position=[0.2, 0.0, 0.3],
                steps=3,
            )
            out = kinematics.actuate_gripper("gripper_cmd", closed=True)

        self.assertEqual(out, 1)
        handle_group_calls = [c for c in simik.calls if c[0] == "handleGroup"]
        self.assertEqual(len(handle_group_calls), 3)

    def test_joint_control_functions(self) -> None:
        sim = FakeSim()
        with patch("coppeliasimagent.tools.kinematics.get_sim", return_value=sim):
            current = kinematics.get_joint_position(11)
            direct = kinematics.set_joint_position(11, 0.25)
            target = kinematics.set_joint_target_position(11, 0.5, motion_params=[1.0, 2.0, 3.0])
            velocity = kinematics.set_joint_target_velocity(11, 1.5, motion_params=[2.0, 3.0])

        self.assertEqual(current, 0.1)
        self.assertEqual(direct, 0.25)
        self.assertEqual(target, 0.5)
        self.assertEqual(velocity, 1.5)
        self.assertEqual(sim.joint_positions[11], 0.5)
        self.assertEqual(sim.joint_target_velocities[11], 1.5)

    def test_setup_youbot_arm_ik_and_gripper(self) -> None:
        sim = FakeSim()
        simik = FakeSimIK()

        with patch("coppeliasimagent.tools.kinematics.get_sim", return_value=sim), patch(
            "coppeliasimagent.tools.kinematics.get_simik", return_value=simik
        ):
            ik_info = kinematics.setup_youbot_arm_ik(robot_path="/youBot")
            close_out = kinematics.actuate_youbot_gripper(
                robot_path="/youBot",
                closed=True,
                command_mode="position",
            )
            open_out = kinematics.actuate_youbot_gripper(
                robot_path="/youBot",
                closed=False,
                command_mode="target_position",
            )

        self.assertEqual(ik_info["base_handle"], 10)
        self.assertEqual(ik_info["tip_parent_handle"], 12)
        self.assertIn(("setLinkDummy", (ik_info["tip_handle"], ik_info["target_handle"])), sim.calls)
        self.assertEqual(close_out["joint1_position"], 0.0)
        self.assertEqual(close_out["joint2_position"], 0.0)
        self.assertEqual(open_out["joint1_target"], 0.025)
        self.assertEqual(open_out["joint2_target"], -0.05)

    def test_youbot_base_drive_and_lock(self) -> None:
        sim = FakeSim()
        with patch("coppeliasimagent.tools.kinematics.get_sim", return_value=sim):
            drive = kinematics.drive_youbot_base(
                robot_path="/youBot",
                forward_velocity=1.0,
                lateral_velocity=0.2,
                yaw_velocity=-0.1,
            )
            stop = kinematics.stop_youbot_base(robot_path="/youBot")
            lock = kinematics.set_youbot_base_locked(robot_path="/youBot", locked=True)

        self.assertEqual([round(v, 3) for v in drive["wheel_velocities"]], [-1.3, -0.7, -0.9, -1.1])
        self.assertEqual(stop["wheel_velocities"], [0.0, 0.0, 0.0, 0.0])
        self.assertEqual(sim.joint_target_velocities[15], 0.0)
        self.assertEqual(sim.joint_target_velocities[18], 0.0)
        self.assertTrue(lock["locked"])
        self.assertIn(10 | sim.handleflag_model, sim.reset_dynamic_calls)
        self.assertEqual(sim.object_int_params[(19, sim.shapeintparam_static)], 1)
        self.assertEqual(sim.object_int_params[(20, sim.shapeintparam_static)], 1)


if __name__ == "__main__":
    unittest.main()
