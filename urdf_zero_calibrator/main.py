import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Tuple, List, Dict, Optional


# ---------- Math helpers (rotation) ----------
def clamp_unit(x: float) -> float:
    return max(-1.0, min(1.0, x))


def matmul(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
    return [
        [A[0][0]*B[0][j] + A[0][1]*B[1][j] + A[0][2]*B[2][j] for j in range(3)],
        [A[1][0]*B[0][j] + A[1][1]*B[1][j] + A[1][2]*B[2][j] for j in range(3)],
        [A[2][0]*B[0][j] + A[2][1]*B[1][j] + A[2][2]*B[2][j] for j in range(3)],
    ]


def rot_x(a: float) -> List[List[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rot_y(a: float) -> List[List[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_z(a: float) -> List[List[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def rpy_to_R(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    # URDF: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return matmul(rot_z(yaw), matmul(rot_y(pitch), rot_x(roll)))


def R_to_rpy(R: List[List[float]]) -> Tuple[float, float, float]:
    pitch = math.asin(clamp_unit(-R[2][0]))
    cp = math.cos(pitch)
    if abs(cp) > 1e-8:
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        # gimbal lock
        roll = 0.0
        yaw = math.atan2(-R[0][1], R[1][1])
    return roll, pitch, yaw


def normalize(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, z = v
    n = math.sqrt(x*x + y*y + z*z)
    if n < 1e-12:
        raise ValueError(f"Axis vector too small: {v}")
    return x/n, y/n, z/n


def axis_angle_R(axis: Tuple[float, float, float], angle: float) -> List[List[float]]:
    ax, ay, az = normalize(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return [
        [t*ax*ax + c,     t*ax*ay - s*az,  t*ax*az + s*ay],
        [t*ax*ay + s*az,  t*ay*ay + c,     t*ay*az - s*ax],
        [t*ax*az - s*ay,  t*ay*az + s*ax,  t*az*az + c],
    ]


# ---------- URDF helpers ----------
def parse_xyz(s: Optional[str], default: Tuple[float, float, float]) -> Tuple[float, float, float]:
    if not s:
        return default
    parts = s.strip().split()
    if len(parts) != 3:
        raise ValueError(f"Expected 3 numbers, got: {s}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def fmt_xyz(v: Tuple[float, float, float]) -> str:
    return f"{v[0]:.9g} {v[1]:.9g} {v[2]:.9g}"


def find_or_create_origin(joint: ET.Element) -> ET.Element:
    origin = joint.find("origin")
    if origin is None:
        origin = ET.SubElement(joint, "origin")
        origin.set("xyz", "0 0 0")
        origin.set("rpy", "0 0 0")
    else:
        if origin.get("xyz") is None:
            origin.set("xyz", "0 0 0")
        if origin.get("rpy") is None:
            origin.set("rpy", "0 0 0")
    return origin


def apply_zero_offset_to_joint(joint: ET.Element, offset_deg: float, shift_limits: bool = False) -> None:
    """
    Apply a "zero calibration" offset to a revolute/continuous joint by baking it into <origin rpy>.

    Convention used here:
      - You enter offset_deg as "+N / -N" degrees.
      - We implement q' = q - offset, which is equivalent to:
            origin_rotation_new = origin_rotation_old * Rot(axis, +offset)
      - This makes the joint's new '0' correspond to the old pose rotated by +offset along the joint axis.

    If shift_limits=True and joint has <limit lower/upper>, we also do:
      lower' = lower - offset_rad, upper' = upper - offset_rad
    """
    jtype = (joint.get("type") or "").strip().lower()
    if jtype not in ("revolute", "continuous"):
        raise ValueError(f"Joint type '{jtype}' is not supported for zero offset baking.")

    offset_rad = math.radians(offset_deg)

    origin = find_or_create_origin(joint)
    roll, pitch, yaw = parse_xyz(origin.get("rpy"), (0.0, 0.0, 0.0))
    R0 = rpy_to_R(roll, pitch, yaw)

    axis_elem = joint.find("axis")
    axis = (1.0, 0.0, 0.0)  # URDF default
    if axis_elem is not None:
        axis = parse_xyz(axis_elem.get("xyz"), axis)

    R_off = axis_angle_R(axis, offset_rad)

    # Bake offset into origin rotation: R_new = R_old * R(axis, +offset)
    Rn = matmul(R0, R_off)
    r2, p2, y2 = R_to_rpy(Rn)
    origin.set("rpy", fmt_xyz((r2, p2, y2)))

    if shift_limits and jtype == "revolute":
        limit = joint.find("limit")
        if limit is not None:
            if limit.get("lower") is not None:
                lower = float(limit.get("lower"))
                limit.set("lower", f"{(lower - offset_rad):.9g}")
            if limit.get("upper") is not None:
                upper = float(limit.get("upper"))
                limit.set("upper", f"{(upper - offset_rad):.9g}")


# ---------- UI helpers ----------
def print_joint_table(joints: List[ET.Element]) -> None:
    print("\nJoints in URDF:")
    print("------------------------------------------------------------")
    print(f"{'#':>3}  {'name':<40}  {'type':<12}")
    print("------------------------------------------------------------")
    for i, j in enumerate(joints, start=1):
        name = j.get("name", "(no-name)")
        jtype = j.get("type", "(no-type)")
        print(f"{i:>3}  {name:<40}  {jtype:<12}")
    print("------------------------------------------------------------")


def ask(prompt: str) -> str:
    return input(prompt).strip()


def default_output_path(input_path: Path) -> Path:
    return input_path.with_name(input_path.stem + "_zero_calibrated.urdf")


def main():
    if len(sys.argv) < 2:
        print("Usage: python urdf_zero_calibrator.py path/to/robot.urdf")
        sys.exit(1)

    urdf_path = Path(sys.argv[1]).expanduser().resolve()
    if not urdf_path.exists():
        print(f"File not found: {urdf_path}")
        sys.exit(1)

    tree = ET.parse(str(urdf_path))
    root = tree.getroot()

    joints = root.findall("joint")
    if not joints:
        print("No <joint> found in this URDF.")
        sys.exit(1)

    changed: Dict[str, float] = {}
    print_joint_table(joints)

    while True:
        sel = ask("Select joint by #, name, comma-separated, or 'all': ")
        if not sel:
            print("No selection provided. Exiting.")
            sys.exit(1)

        sel = sel.strip()
        selected: List[ET.Element] = []

        if sel.lower() == "all":
            selected = joints[:]
        else:
            tokens = [t.strip() for t in sel.split(",") if t.strip()]
            for t in tokens:
                if t.isdigit():
                    idx = int(t)
                    if idx < 1 or idx > len(joints):
                        print(f"Invalid joint index: {t}")
                        sys.exit(1)
                    selected.append(joints[idx - 1])
                else:
                    match = None
                    for j in joints:
                        if j.get("name") == t:
                            match = j
                            break
                    if match is None:
                        print(f"Joint name not found: {t}")
                        sys.exit(1)
                    selected.append(match)

        shift_limits = ask("Shift limits too? (y/N): ").lower() in ("y", "yes")

        for idx, j in enumerate(selected):
            name = j.get("name", "(no-name)")
            while True:
                offset_str = ask(f"Offset degrees for joint '{name}': ")
                try:
                    offset_deg = float(offset_str)
                    break
                except ValueError:
                    print("Please enter a valid number.")
            apply_zero_offset_to_joint(j, offset_deg, shift_limits=shift_limits)
            changed[name] = offset_deg

            if idx < len(selected) - 1:
                cont = ask("Continue to next joint? (Y/n): ").lower()
                if cont in ("n", "no"):
                    break

        more = ask("Modify more joints? (y/N): ").lower()
        if more not in ("y", "yes"):
            break

    out_default = default_output_path(urdf_path)
    out_str = ask(f"Output path [default: {out_default}]: ")
    out_path = Path(out_str).expanduser().resolve() if out_str else out_default

    tree.write(str(out_path), encoding="utf-8", xml_declaration=True)

    print("\nDone. Updated joints:")
    for name, off in changed.items():
        print(f"- {name}: {off} deg")
    print(f"Saved to: {out_path}")


if __name__ == "__main__":
    main()
