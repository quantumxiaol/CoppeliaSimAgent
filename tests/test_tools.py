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
        }
        self.object_names = {
            1: "shape_a",
            2: "dummy_a",
            3: "joint_a",
            4: "model_base",
            5: "model_shape",
        }
        self.object_positions = {
            1: [0.123456, 1.0, 2.0],
            2: [0.0, 0.0, 0.2],
            3: [5.0, 5.0, 5.0],
            4: [0.2, 0.2, 0.2],
            5: [0.2, 0.2, 0.2],
        }
        self.object_orientations = {
            1: [0.0, math.pi / 2, 0.0],
            2: [0.1, 0.2, 0.3],
            3: [0.0, 0.0, 0.0],
            4: [0.0, 0.0, 0.0],
            5: [0.0, 0.0, 0.0],
        }
        self.collision_map = {(1, 2): 1, (1, 3): 0}
        self.subtree_map = {
            self.handle_scene: [1, 2, 3, 4, 5],
            4: [4, 5],
        }

    def createPrimitiveShape(self, primitive: int, size: list[float]) -> int:  # noqa: N802
        handle = self.next_handle
        self.next_handle += 1
        self.calls.append(("createPrimitiveShape", (primitive, size)))
        self.object_types[handle] = self.object_shape_type
        self.object_names[handle] = f"Cuboid#{handle}"
        self.object_positions[handle] = [0.0, 0.0, 0.0]
        self.object_orientations[handle] = [0.0, 0.0, 0.0]
        return handle

    def setObjectPosition(self, handle: int, relative_to: int, position: list[float]) -> None:  # noqa: N802
        self.calls.append(("setObjectPosition", (handle, relative_to, position)))
        self.object_positions[handle] = [float(v) for v in position]

    def setShapeColor(self, handle: int, component: object, color_comp: int, color: list[float]) -> None:  # noqa: N802
        self.calls.append(("setShapeColor", (handle, component, color_comp, color)))

    def setObjectInt32Param(self, handle: int, param: int, value: int) -> None:  # noqa: N802
        self.calls.append(("setObjectInt32Param", (handle, param, value)))

    def setObjectOrientation(self, handle: int, relative_to: int, orientation: list[float]) -> None:  # noqa: N802
        self.calls.append(("setObjectOrientation", (handle, relative_to, orientation)))
        self.object_orientations[handle] = [float(v) for v in orientation]

    def removeObject(self, handle: int) -> None:  # noqa: N802
        self.calls.append(("removeObject", (handle,)))

    def removeObjects(self, handles: list[int]) -> None:  # noqa: N802
        self.calls.append(("removeObjects", (handles,)))

    def loadModel(self, model_path: str) -> int:  # noqa: N802
        self.calls.append(("loadModel", (model_path,)))
        handle = self.next_handle
        self.next_handle += 1
        return handle

    def setObjectParent(self, child: int, parent: int, keep_in_place: bool) -> None:  # noqa: N802
        self.calls.append(("setObjectParent", (child, parent, keep_in_place)))

    def copyPasteObjects(self, handles: list[int], options: int = 0) -> list[int]:  # noqa: N802
        self.calls.append(("copyPasteObjects", (handles, options)))
        src = handles[0]
        out = self.next_handle
        self.next_handle += 1
        self.object_types[out] = self.object_types[src]
        self.object_names[out] = f"{self.object_names[src]}_copy"
        self.object_positions[out] = list(self.object_positions[src])
        self.object_orientations[out] = list(self.object_orientations[src])
        return [out]

    def getObjectsInTree(self, root: int) -> list[int]:  # noqa: N802
        self.calls.append(("getObjectsInTree", (root,)))
        return list(self.subtree_map.get(root, [root] if root in self.object_names else sorted(self.object_names)))

    def getObjectInt32Param(self, handle: int, param: int) -> int:  # noqa: N802
        return self.object_types[handle]

    def getObjectName(self, handle: int) -> str:  # noqa: N802
        return self.object_names[handle]

    def setObjectAlias(self, handle: int, alias: str) -> None:  # noqa: N802
        self.calls.append(("setObjectAlias", (handle, alias)))
        self.object_names[handle] = alias

    def getObjectPosition(self, handle: int, relative_to: int) -> list[float]:  # noqa: N802
        return self.object_positions[handle]

    def getObjectOrientation(self, handle: int, relative_to: int) -> list[float]:  # noqa: N802
        return self.object_orientations[handle]

    def checkCollision(self, entity1: int, entity2: int) -> int:  # noqa: N802
        return self.collision_map.get((entity1, entity2), 0)

    def createDummy(self, size: float) -> int:  # noqa: N802
        self.calls.append(("createDummy", (size,)))
        handle = self.next_handle
        self.next_handle += 1
        return handle

    def setInt32Signal(self, signal_name: str, value: int) -> None:  # noqa: N802
        self.calls.append(("setInt32Signal", (signal_name, value)))


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


if __name__ == "__main__":
    unittest.main()
