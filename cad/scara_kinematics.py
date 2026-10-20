"""Offline SR6 IK geometry derived from uploaded rho4_driver.py.

The driver's Cartesian +/-500 mm constants are intentionally not replicated:
those were used only by driver.jog_start(), not by inverse kinematics.
This module NEVER connects to a robot and neither imports nor distributes the
production driver or runner.

Source driver (SHA-256):
821d81fb95cf7c9a8d198dd8160e599f4f0df721052fc9a9c9233d5b6647c091
"""
from __future__ import annotations
import math


class SR6Geometry:
    # Geometry and limits copied from the supplied rho4_driver.py.
    ARM_L1 = 330.0
    ARM_L2 = 270.0
    A1_MIN, A1_MAX = -140.0, 140.0
    A2_MIN, A2_MAX = -150.0, 150.0
    A4_MIN, A4_MAX = -180.0, 180.0
    Z_MIN, Z_MAX = 5.0, 195.0
    R_MIN, R_MAX = -175.0, 175.0
    XY_RADIUS_MAX = 590.0
    # No arbitrary Cartesian +/-500-mm clipping: XY reachability is
    # determined by IK and the radial safety limit in scara_core.py.

    def inverse_xy(self, x, y):
        l1, l2 = self.ARM_L1, self.ARM_L2
        r2 = x*x + y*y
        cos_a2 = (r2 - l1*l1 - l2*l2) / (2.0*l1*l2)
        if cos_a2 < -1.0 or cos_a2 > 1.0:
            return []
        cos_a2 = max(-1.0, min(1.0, cos_a2))
        a2_abs = math.acos(cos_a2)
        solutions = []
        for a2 in (a2_abs, -a2_abs):
            k1 = l1 + l2 * math.cos(a2)
            k2 = l2 * math.sin(a2)
            a1 = math.atan2(y, x) - math.atan2(k2, k1)
            a1 = math.atan2(math.sin(a1), math.cos(a1))
            solutions.append((math.degrees(a1), math.degrees(a2)))
        return solutions

    def check_xy_kinematics(self, x, y):
        valid = []
        for a1, a2 in self.inverse_xy(x, y):
            if self.A1_MIN <= a1 <= self.A1_MAX and self.A2_MIN <= a2 <= self.A2_MAX:
                valid.append((a1, a2))
        return valid

    def check_target(self, x, y, z, r):
        solutions = self.inverse_xy(float(x), float(y))
        if not solutions:
            raise ValueError(f"Target X={x:.3f} Y={y:.3f} je mimo geometricky dosah robota")
        valid = []
        for a1, a2 in solutions:
            a4 = float(r) - a1 - a2  # Verified SR6: R = A1 + A2 + A4.
            if not (self.A1_MIN <= a1 <= self.A1_MAX):
                continue
            if not (self.A2_MIN <= a2 <= self.A2_MAX):
                continue
            if not (self.A4_MIN <= a4 <= self.A4_MAX):
                continue
            valid.append({"a1": a1, "a2": a2, "a3": float(z), "a4": a4})
        if not valid:
            raise ValueError(
                f"Target X={x:.3f} Y={y:.3f} Z={z:.3f} R={r:.3f} "
                "nema zadnou konfiguraci v limitech A1/A2/A4"
            )
        return valid


def tool_to_flange(tcp, tool):
    """Convert desired TCP XYZR to controller/flange XYZR.

    x/y/z offsets are measured in the flange's local axes at flange R=0.
    tool['r'] is the angular difference TCP orientation minus flange R.
    This is an OFFLINE assumption, not proof of any RHO4 controller TCP setting.
    """
    x, y, z, r_tcp = (float(v) for v in tcp)
    r_flange = r_tcp - float(tool['r'])
    angle = math.radians(r_flange)
    dx = float(tool['x'])
    dy = float(tool['y'])
    return (x - dx*math.cos(angle) + dy*math.sin(angle),
            y - dx*math.sin(angle) - dy*math.cos(angle),
            z - float(tool['z']), r_flange)


def flange_to_tool(flange, tool):
    """Convert controller/flange XYZR to expected TCP XYZR (inverse mapping)."""
    x, y, z, r_flange = (float(v) for v in flange)
    angle = math.radians(r_flange)
    dx = float(tool['x'])
    dy = float(tool['y'])
    return (x + dx*math.cos(angle) - dy*math.sin(angle),
            y + dx*math.sin(angle) + dy*math.cos(angle),
            z + float(tool['z']), r_flange + float(tool['r']))
