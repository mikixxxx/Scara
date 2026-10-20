"""Run from distribution directory: python -m unittest discover -s tests -v."""
import copy
import json
import math
import tempfile
import unittest
from pathlib import Path
from scara_kinematics import tool_to_flange, flange_to_tool

from scara_core import (
    new_project, pose, reachability_error, validate_project, generate_gcode,
    circle_from_3_points, load_project, save_project, path_samples, normalize_project,
    active_tool, machine_pose, normalize_tools,
    create_tool_profile, rename_tool_profile, remove_tool_profile,
)


def move(kind, x, y, z=100, r=0, feed=300, **extra):
    return {"type":kind,"x":float(x),"y":float(y),"z":float(z),
            "r":float(r),"feed":float(feed),**extra}


class TestScaraCore(unittest.TestCase):
    def sample(self):
        project = new_project()
        project["operations"] = [
            move("linear", 455, 5),
            move("arc", 455, 25, mx=460.0, my=15.0),
            {"type":"delay","seconds":0.5},
            move("rapid", 450, 0),
        ]
        return project

    def test_default_start_reachable(self):
        self.assertIsNone(reachability_error(pose(new_project()["start"])))

    def test_sample_has_no_offline_errors(self):
        self.assertEqual(validate_project(self.sample()), [])

    def test_arc_geometry(self):
        start = (455., 5., 100., 0.)
        end = (455., 25., 100., 0.)
        arc = circle_from_3_points(start, (460., 15.), end)
        self.assertEqual(arc["code"], "G3")
        self.assertAlmostEqual(arc["radius"], 12.5)
        self.assertTrue(0 < arc["mid_fraction"] < 1)
        self.assertEqual(path_samples(start, move("arc", *end, mx=460, my=15))[-1], end)

    def test_generate_uses_actual_supported_commands(self):
        code = generate_gcode(self.sample())
        self.assertIn("--machine-coordinates", code)
        self.assertIn("G1 X455.0000 Y5.0000", code)
        self.assertIn("G3 X455.0000 Y25.0000 I-7.50000 J10.00000", code)
        self.assertIn("G4 P0.500", code)
        self.assertIn("G0 X450.0000", code)
        self.assertNotIn("M66", code)
        self.assertEqual(code.strip().splitlines()[-1], "M2")

    def test_actual_ptp_is_not_exported_as_linear_g0(self):
        project = self.sample()
        project["operations"].append(move("ptp", 455, 0))
        with self.assertRaisesRegex(ValueError, "PTP"):
            generate_gcode(project)

    def test_wait_input_is_not_exported(self):
        project = self.sample()
        project["operations"].append({"type":"wait_input","channel":611,"level":1})
        with self.assertRaisesRegex(ValueError, "WAIT INPUT"):
            generate_gcode(project)

    def test_arc_z_and_r_must_not_change(self):
        project = self.sample()
        project["operations"][1]["z"] = 105.
        with self.assertRaises(ValueError):
            generate_gcode(project)

    def test_linear_segment_crossing_unreachable_hole_is_rejected(self):
        p = new_project()
        p["start"].update(x=0, y=250)
        p["operations"] = [move("linear", 0, -250)]
        self.assertTrue(validate_project(p))

    def test_positive_x_over_500_is_reachable(self):
        self.assertIsNone(reachability_error((550.0, 0.0, 100.0, 0.0)))

    def test_positive_y_over_500_is_reachable(self):
        self.assertIsNone(reachability_error((0.0, 550.0, 100.0, 0.0)))

    def test_negative_x_over_500_abs_is_reachable_when_ik_allows(self):
        self.assertIsNone(reachability_error((-520.0, 250.0, 100.0, 0.0)))

    def test_negative_y_over_500_abs_is_reachable(self):
        self.assertIsNone(reachability_error((550.0, -20.0, 100.0, 0.0)))
        self.assertIsNone(reachability_error((0.0, -550.0, 100.0, 0.0)))

    def test_radius_590_limit_remains(self):
        self.assertIsNone(reachability_error((590.0, 0.0, 100.0, 0.0)))
        self.assertIn("590", reachability_error((590.01, 0.0, 100.0, 0.0)))

    def test_long_linear_segment_beyond_old_square_is_exportable(self):
        p = new_project()
        p["operations"] = [move("linear", 550.0, 0.0)]
        self.assertEqual(validate_project(p), [])
        self.assertIn("G1 X550.0000 Y0.0000", generate_gcode(p))

    def test_pure_z_segment_beyond_old_square_is_exportable(self):
        p = new_project()
        p["start"]["x"] = 550.0
        p["operations"] = [move("linear", 550.0, 0.0, z=110.0)]
        self.assertEqual(validate_project(p), [])
        self.assertIn("G1 X550.0000 Y0.0000 Z110.0000", generate_gcode(p))

    def test_xy_ik_limits_still_reject_unreachable_points(self):
        self.assertIsNotNone(reachability_error((-550.0, 0.0, 100.0, 0.0)))

    def test_bad_z_rejected(self):
        p = new_project()
        p["operations"] = [move("linear", 460, 0, z=300)]
        self.assertTrue(validate_project(p))

    def test_roundtrip(self):
        p = self.sample()
        with tempfile.TemporaryDirectory() as td:
            dest = Path(td) / "job.scara"
            save_project(dest, p)
            self.assertEqual(load_project(dest), p)

    def test_legacy_v1_ptp_is_migrated_to_rapid_g0(self):
        p = self.sample()
        p["schema"] = 1
        p["operations"][-1]["type"] = "ptp"
        converted = normalize_project(p)
        self.assertEqual(converted["schema"], 4)
        self.assertEqual(converted["operations"][-1]["type"], "rapid")
        self.assertIn("G0 X450.0000", generate_gcode(converted))


    def test_move_to_start_disabled_by_default(self):
        p = new_project()
        p["operations"] = [move("linear", 455.0, 5.0)]
        self.assertFalse(p["move_to_start"])
        gcode = generate_gcode(p)
        self.assertIn("MOVE TO START DISABLED", gcode)
        self.assertFalse(any(line.startswith("G0 ") for line in gcode.splitlines()))

    def test_move_to_start_enabled_adds_one_initial_g0_before_sketch(self):
        p = new_project()
        p["move_to_start"] = True
        p["start"].update(x=550.0, y=0.0, z=100.0, r=0.0)
        p["operations"] = [move("linear", 540.0, 0.0)]
        gcode_lines = generate_gcode(p).splitlines()
        g0 = [i for i, line in enumerate(gcode_lines) if line.startswith("G0 ")]
        g1 = [i for i, line in enumerate(gcode_lines) if line.startswith("G1 ")]
        self.assertEqual(len(g0), 1)
        self.assertEqual(len(g1), 1)
        self.assertLess(g0[0], g1[0])
        self.assertEqual(gcode_lines[g0[0]], "G0 X550.0000 Y0.0000 Z100.0000 R0.0000")
        self.assertIn("--machine-coordinates", '\n'.join(gcode_lines))
        self.assertIn("NOT checked offline", '\n'.join(gcode_lines))

    def test_move_to_start_enabled_with_empty_sketch(self):
        p = new_project()
        p["move_to_start"] = True
        lines = generate_gcode(p).splitlines()
        self.assertEqual(sum(line.startswith("G0 ") for line in lines), 1)
        self.assertEqual(lines[-1], "M2")

    def test_move_to_start_setting_is_persistent(self):
        p = self.sample()
        p["move_to_start"] = True
        with tempfile.TemporaryDirectory() as td:
            dest = Path(td) / "start.scara"
            save_project(dest, p)
            self.assertTrue(load_project(dest)["move_to_start"])
            self.assertEqual(load_project(dest), p)

    def test_old_v2_project_defaults_to_no_start_move(self):
        p = self.sample()
        p["schema"] = 2
        del p["move_to_start"]
        converted = normalize_project(p)
        self.assertEqual(converted["schema"], 4)
        self.assertFalse(converted["move_to_start"])
        # The legacy sketch already contains one RAPID operation; no *extra* start G0.
        self.assertEqual(sum(line.startswith("G0 ") for line in generate_gcode(converted).splitlines()), 1)

    def test_invalid_start_still_blocks_export_when_g0_enabled(self):
        p = new_project()
        p["move_to_start"] = True
        p["start"]["x"] = 650.0
        with self.assertRaisesRegex(ValueError, "START"):
            generate_gcode(p)

    def test_reject_non_boolean_start_move(self):
        p = new_project()
        p["move_to_start"] = "True"
        with self.assertRaisesRegex(ValueError, "true/false"):
            normalize_project(p)

    def test_collinear_arc_rejected(self):
        with self.assertRaisesRegex(ValueError, "primce"):
            circle_from_3_points((450,0), (460,0), (470,0))


    def test_tool_transform_xy_and_z(self):
        tool = {"name": "Jehla", "x": 50., "y": 10., "z": 20., "r": 0.}
        flange = tool_to_flange((450., 0., 100., 0.), tool)
        self.assertEqual(flange, (400., -10., 80., 0.))
        for v, goal in zip(flange_to_tool(flange, tool), (450, 0, 100, 0)):
            self.assertAlmostEqual(v, goal)

    def test_tool_transform_rotates_with_r(self):
        tool = {"name": "Jehla", "x": 50., "y": 10., "z": 0., "r": 0.}
        flange = tool_to_flange((450., 0., 100., 90.), tool)
        self.assertAlmostEqual(flange[0], 460.)
        self.assertAlmostEqual(flange[1], -50.)
        self.assertAlmostEqual(flange[3], 90.)

    def test_tool_angle_is_offset_from_flange_orientation(self):
        tool = {"name": "Kamera", "x": 50., "y": 0., "z": 0., "r": 90.}
        self.assertAlmostEqual(tool_to_flange((450., 0., 100., 90.), tool)[0], 400.)
        self.assertAlmostEqual(tool_to_flange((450., 0., 100., 90.), tool)[3], 0.)

    def test_offset_reachability_checks_flange_not_tcp(self):
        tool = {"name": "Dlouhy", "x": -50., "y": 0., "z": 0., "r": 0.}
        # TCP 550 is within workspace radius; flange 600 is not.
        self.assertIn("590", reachability_error((550., 0., 100., 0.), tool))
        # TCP 600 can still be reached with a flange at 550.
        tool["x"] = 50.
        self.assertIsNone(reachability_error((600., 0., 100., 0.), tool))

    def test_nonzero_tool_offset_requires_explicit_compensation(self):
        project = new_project()
        project["tools"][0]["x"] = 30.
        self.assertTrue(validate_project(project))
        with self.assertRaisesRegex(ValueError, "kompenzace"):
            generate_gcode(project)

    def test_nonzero_tool_offset_compensates_g0_g1(self):
        project = new_project()
        project["tcp_compensate"] = True
        project["tools"][0].update(x=30., y=5., z=10., r=0.)
        project["move_to_start"] = True
        project["operations"] = [move("linear", 470., 0.)]
        self.assertEqual(validate_project(project), [])
        lines = generate_gcode(project)
        self.assertIn("G0 X420.0000 Y-5.0000 Z90.0000 R0.0000", lines)
        self.assertIn("G1 X440.0000 Y-5.0000 Z90.0000 R0.0000", lines)

    def test_arc_machine_endpoint_shift_and_ij_relative_remains(self):
        project = self.sample()
        project["tcp_compensate"] = True
        project["tools"][0].update(x=40., y=-8., z=5., r=0.)
        self.assertEqual(validate_project(project), [])
        code = generate_gcode(project)
        self.assertIn("G1 X415.0000 Y13.0000 Z95.0000", code)
        self.assertIn("G3 X415.0000 Y33.0000 I-7.50000 J10.00000", code)
        self.assertIn("G0 X410.0000 Y8.0000 Z95.0000", code)

    def test_side_offset_and_orientation_change_block_export(self):
        project = new_project()
        project["tcp_compensate"] = True
        project["tools"][0]["x"] = 50.
        project["operations"] = [move("linear", 455., 0., r=20.)]
        with self.assertRaisesRegex(ValueError, "bocnim TCP offsetem"):
            generate_gcode(project)

    def test_no_side_offset_can_rotate_r_during_linear(self):
        project = new_project()
        project["tcp_compensate"] = True
        project["tools"][0].update(z=20., r=5.)
        project["operations"] = [move("linear", 455., 0., r=10.)]
        self.assertEqual(validate_project(project), [])
        self.assertIn("Z80.0000 R5.0000", generate_gcode(project))

    def test_roundtrip_new_tool_project_and_legacy_3(self):
        project = new_project()
        project["tools"].append({"name": "Kamera", "x": 12., "y": 10., "z": 6., "r": 8.})
        project["active_tool"] = "Kamera"
        project["tcp_compensate"] = True
        with tempfile.TemporaryDirectory() as tmp:
            filename = Path(tmp) / "tools.scara"
            save_project(filename, project)
            self.assertEqual(load_project(filename), project)
        old = self.sample()
        old["schema"] = 3
        old.pop("tools")
        old.pop("active_tool")
        old.pop("tcp_compensate")
        converted = normalize_project(old)
        self.assertEqual(converted["schema"], 4)
        self.assertEqual(converted["tools"][0]["x"], 0.0)
        self.assertFalse(converted["tcp_compensate"])

    def test_duplicate_or_invalid_tool_profiles_rejected(self):
        with self.assertRaisesRegex(ValueError, "nazev"):
            normalize_tools([dict(name="X", x=0,y=0,z=0,r=0),
                             dict(name="x", x=0,y=0,z=0,r=0)])
        with self.assertRaisesRegex(ValueError, "mimo"):
            normalize_tools([dict(name="Too long", x=600,y=0,z=0,r=0)])


    def test_tool_create_and_change_active(self):
        project = new_project()
        project["tools"][0]["x"] = 30.0
        create_tool_profile(project, "Kamera")
        self.assertEqual(len(project["tools"]), 2)
        self.assertEqual(project["active_tool"], "Kamera")
        self.assertEqual(active_tool(project)["x"], 30.0)
        self.assertEqual(normalize_tools(project["tools"]), project["tools"])

    def test_tool_create_duplicate_blank_or_too_many_does_not_change_project(self):
        project = new_project()
        create_tool_profile(project, "Kamera")
        before = copy.deepcopy(project)
        for name in ("", "  ", "kamera", "a" * 81):
            with self.assertRaises(ValueError, msg=repr(name)):
                create_tool_profile(project, name)
            self.assertEqual(project, before)
        project["tools"] = [dict(name=f"T{i}", x=0, y=0, z=0, r=0) for i in range(100)]
        project["active_tool"] = "T0"
        before = copy.deepcopy(project)
        with self.assertRaises(ValueError):
            create_tool_profile(project, "extra")
        self.assertEqual(project, before)

    def test_tool_rename_and_delete_are_atomic(self):
        project = new_project()
        create_tool_profile(project, "Kamera")
        rename_tool_profile(project, "Jehla")
        self.assertEqual(project["active_tool"], "Jehla")
        before = copy.deepcopy(project)
        with self.assertRaises(ValueError):
            rename_tool_profile(project, "Bez nastroje")
        self.assertEqual(project, before)
        remove_tool_profile(project)
        self.assertEqual(len(project["tools"]), 1)
        self.assertEqual(project["active_tool"], "Bez nastroje")
        before = copy.deepcopy(project)
        with self.assertRaises(ValueError):
            remove_tool_profile(project)
        self.assertEqual(project, before)

    def test_saved_tool_profiles_roundtrip_with_global_library_format(self):
        project = new_project()
        create_tool_profile(project, "Jehla")
        active_tool(project)["x"] = 35.0
        with tempfile.TemporaryDirectory() as tmp:
            library_file = Path(tmp) / "scara_tools.json"
            library_file.write_text(json.dumps({"schema": 1, "tools": normalize_tools(project["tools"])}), encoding="utf-8")
            loaded = json.loads(library_file.read_text(encoding="utf-8"))
            self.assertEqual(loaded["schema"], 1)
            self.assertEqual(normalize_tools(loaded["tools"]), project["tools"])

    def test_tool_rename_then_project_roundtrip(self):
        project = new_project()
        create_tool_profile(project, "Kamera")
        rename_tool_profile(project, "Kamera 2")
        with tempfile.TemporaryDirectory() as tmp:
            filepath = Path(tmp) / "job.scara"
            save_project(filepath, project)
            self.assertEqual(load_project(filepath), project)


if __name__ == "__main__":
    unittest.main()
