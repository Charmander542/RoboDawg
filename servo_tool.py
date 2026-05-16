"""
RoboDawg Servo Controller — pygame GUI

Features
--------
  1. Individual servo sliders (0–270°) for all 12 servos
  2. Save / load rest positions via SERVOSAVE / SERVOPOS
  3. Trajectory keyframe editor: capture poses, reorder, set per-step
     duration, and play back with smooth lerp interpolation

Serial commands used
--------------------
  SERVO ch angle      – drive a servo
  SERVOPOS ch         – read current angle from firmware
  SERVOSAVE ch        – persist current angle as the rest position

Controls
--------
  Sliders       drag to set angle, or click to jump
  Save Rest     save all current angles to NVS
  Load Rest     read saved rest angles from NVS and apply
  Add KF        capture current slider state as a keyframe
  Delete KF     remove selected keyframe
  Move Up/Down  reorder selected keyframe
  Play / Stop   play trajectory (lerps between keyframes)
  Loop          toggle looping playback
  Duration +/-  adjust selected keyframe's hold+transition time
  Export/Import save/load trajectory to JSON file
"""

import json
import os
import re
import subprocess
import sys
from time import sleep, time
from math import acos, atan2, cos, degrees, pi, radians, sin, sqrt

import pygame
import serial
from pygame import Vector2, Vector3

# ── serial port detection ────────────────────────────────────────────────────


def get_devices() -> list[str]:
    output = subprocess.check_output(["pio", "device", "list"]).decode()
    return re.findall(r"^(\/.+)+", output, flags=re.MULTILINE)


PORT = os.environ.get("PORT", next(
    (dev for dev in get_devices() if "Bluetooth" not in dev), None))
if not PORT:
    print("Error: no serial device found", file=sys.stderr)
    sys.exit(1)

BAUD = 115_200

# ── servo definitions ────────────────────────────────────────────────────────

MIN_DEG = 0.0
MAX_DEG = 270.0
NEUTRAL_DEG = 135.0

SERVOS = [
    ("FR Hip",   0),
    ("FR Thigh", 1),
    ("FR Shin",  2),
    ("FL Hip",   4),
    ("FL Thigh", 5),
    ("FL Shin",  6),
    ("BR Hip",   8),
    ("BR Thigh", 9),
    ("BR Shin",  10),
    ("BL Hip",   12),
    ("BL Thigh", 13),
    ("BL Shin",  14),
]

SERVO_CHANNELS = [ch for _, ch in SERVOS]

# Map channel → leg name (for filtering playback by enabled legs)
CHANNEL_TO_LEG: dict[int, str] = {}
for _name, _chs, _, _ in [
    ("FR", (0, 1, 2), True, False),
    ("FL", (4, 5, 6), True, True),
    ("BR", (8, 9, 10), False, False),
    ("BL", (12, 13, 14), False, True),
]:
    for _ch in _chs:
        CHANNEL_TO_LEG[_ch] = _name

# ── colours ──────────────────────────────────────────────────────────────────

C_BG = (30, 30, 30)
C_PANEL = (45, 45, 48)
C_SLIDER_BG = (60, 60, 65)
C_SLIDER_FG = (80, 180, 220)
C_SLIDER_KNOB = (220, 230, 240)
C_TEXT = (210, 210, 210)
C_TEXT_DIM = (130, 130, 130)
C_ACCENT = (80, 180, 220)
C_BTN = (65, 65, 70)
C_BTN_HOVER = (85, 85, 92)
C_BTN_ACTIVE = (80, 180, 220)
C_KF_BG = (55, 55, 60)
C_KF_SEL = (80, 180, 220)
C_KF_PLAY = (60, 200, 120)
C_RED = (220, 80, 80)
C_GREEN = (80, 200, 120)
C_YELLOW = (220, 200, 80)
C_HEADER = (100, 100, 105)

# leg group colours
C_LEG_FR = (100, 180, 255)
C_LEG_FL = (255, 160, 80)
C_LEG_BR = (120, 220, 160)
C_LEG_BL = (220, 130, 220)

LEG_COLORS = {
    0: C_LEG_FR, 1: C_LEG_FR, 2: C_LEG_FR,
    4: C_LEG_FL, 5: C_LEG_FL, 6: C_LEG_FL,
    8: C_LEG_BR, 9: C_LEG_BR, 10: C_LEG_BR,
    12: C_LEG_BL, 13: C_LEG_BL, 14: C_LEG_BL,
}

# ── kinematics ───────────────────────────────────────────────────────────────

THIGH_LENGTH_MM = 60
SHIN_LENGTH_MM = 120
FROM_HIP_EXTRUDE_MM = 25

# Leg definitions: (name, (hip_ch, thigh_ch, shin_ch), is_front, is_left)
LEGS = [
    ("FR", (0, 1, 2),    True,  False),
    ("FL", (4, 5, 6),    True,  True),
    ("BR", (8, 9, 10),   False, False),
    ("BL", (12, 13, 14), False, True),
]

GAIT_PHASES = {"FR": 0.00, "BR": 0.25, "FL": 0.50, "BL": 0.75}


def polar(r: float, phi: float):
    return r * Vector2(cos(phi), sin(phi))


def leg_fk_2d(thigh_rad: float, shin_rad: float) -> Vector2:
    return polar(THIGH_LENGTH_MM, thigh_rad) + polar(SHIN_LENGTH_MM, shin_rad)


def back_leg_fk(hip_rad: float, thigh_rad: float, shin_rad: float) -> Vector3:
    lx, ly = leg_fk_2d(thigh_rad, shin_rad)
    y, z = Vector2(FROM_HIP_EXTRUDE_MM, ly).rotate_rad(hip_rad)
    return Vector3(lx, y, z)


def front_leg_fk(hip_rad: float, thigh_rad: float, shin_rad: float) -> Vector3:
    x, y, z = back_leg_fk(hip_rad, thigh_rad, shin_rad)
    return Vector3(-x, y, z)


def leg_fk(hip_rad: float, thigh_rad: float, shin_rad: float,
           is_front: bool, is_left: bool) -> Vector3:
    """Unified FK: returns foot position relative to hip in body frame."""
    hip_input = -hip_rad if is_left else hip_rad
    if is_front:
        return front_leg_fk(hip_input, thigh_rad, shin_rad)
    return back_leg_fk(hip_input, thigh_rad, shin_rad)


def leg_ik(x: float, y: float, z: float,
           is_front: bool = False,
           is_left: bool = False) -> tuple[float, float, float] | None:
    """Inverse kinematics: foot position → (hip_rad, thigh_rad, shin_rad).

    Inputs/outputs are joint-space radians. Returns None if unreachable.
    Coordinate frame: +x forward, +y right, +z up, relative to hip joint.
    """
    # back_leg_fk always uses +FROM_HIP_EXTRUDE internally;
    # left/right is handled purely by negating the hip angle.
    hip_extrude = FROM_HIP_EXTRUDE_MM
    lx = -x if is_front else x

    # Step 1: solve hip angle from (y, z) plane
    d_yz_sq = y * y + z * z
    if d_yz_sq < hip_extrude * hip_extrude:
        return None  # too close to hip axis

    ly = -sqrt(d_yz_sq - hip_extrude * hip_extrude)  # negative = foot below hip
    hip_internal = atan2(z, y) - atan2(ly, hip_extrude)

    # Step 2: 2-link planar IK for (thigh, shin) from (lx, ly)
    L_t = THIGH_LENGTH_MM
    L_s = SHIN_LENGTH_MM
    R = sqrt(lx * lx + ly * ly)

    if R > L_t + L_s or R < abs(L_t - L_s):
        return None  # unreachable

    phi = atan2(ly, lx)

    # Solve for thigh using elimination of shin
    k = (lx * lx + ly * ly + L_t * L_t - L_s * L_s) / (2.0 * L_t)
    cos_arg = k / R
    if abs(cos_arg) > 1.0:
        return None

    offset = acos(cos_arg)

    # Try both elbow configurations, pick valid one
    for sign in (+1, -1):
        thigh = phi + sign * offset
        shin = atan2(ly - L_t * sin(thigh), lx - L_t * cos(thigh))
        # Verify by FK round-trip (should match within tolerance)
        check = leg_fk_2d(thigh, shin)
        if abs(check.x - lx) < 1.0 and abs(check.y - ly) < 1.0:
            # hip_internal is what back_leg_fk receives;
            # leg_fk negates it for left legs, so invert back
            hip_joint = -hip_internal if is_left else hip_internal
            return (hip_joint, thigh, shin)

    return None

# ── layout constants ─────────────────────────────────────────────────────────
WIN_W, WIN_H = 1280, 720
SLIDER_X = 20
SLIDER_W = 320
SLIDER_H = 22
SLIDER_PAD = 6
LABEL_W = 90

KF_PANEL_X = 620
KF_PANEL_W = 640
KF_ITEM_H = 36
KF_LIST_Y = 120
KF_LIST_H = 440

BTN_H = 32
BTN_W = 90


# ── serial helpers ───────────────────────────────────────────────────────────

class SerialConn:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=1.0)
        sleep(2.0)
        self.ser.reset_input_buffer()

    def send(self, msg: str):
        """Fire-and-forget: send command, don't wait for response."""
        self.ser.write((msg.strip() + "\n").encode())

    def query(self, msg: str) -> str:
        """Send command and block until matching OK/ERR response line."""
        # Let in-flight fire-and-forget responses arrive, then drain them
        sleep(0.05)
        self.ser.reset_input_buffer()
        self.ser.write((msg.strip() + "\n").encode())
        # Extract command name to match against response
        cmd = msg.strip().split()[0].upper()
        lines: list[str] = []
        while True:
            raw = self.ser.readline()
            if not raw:
                break  # timeout
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            lines.append(line)
            # Only stop on OK/ERR that matches our command
            if (line.startswith("OK") or line.startswith("ERR")) \
               and cmd in line.upper():
                break
        return "\n".join(lines)

    def servo(self, ch: int, angle: float):
        angle = max(MIN_DEG, min(MAX_DEG, angle))
        self.send(f"SERVO {ch} {angle:.1f}")

    def servo_pos(self, ch: int) -> float | None:
        resp = self.query(f"SERVOPOS {ch}")
        for line in resp.splitlines():
            line = line.strip()
            if line.startswith("OK") or line.startswith("ERR"):
                continue
            try:
                return float(line)
            except ValueError:
                continue
        return None

    def servo_save(self, ch: int) -> str:
        return self.query(f"SERVOSAVE {ch}")

    def rest_get(self, ch: int) -> float | None:
        """Read NVS-saved rest position for channel."""
        resp = self.query(f"RESTGET {ch}")
        for line in resp.splitlines():
            line = line.strip()
            if line.startswith("OK") or line.startswith("ERR"):
                continue
            if line == "NONE":
                return None
            try:
                return float(line)
            except ValueError:
                continue
        return None

    def zero_set(self, ch: int, angle: float):
        self.send(f"ZERO {ch} {angle:.1f}")

    def zero_get(self, ch: int) -> float | None:
        resp = self.query(f"ZEROGET {ch}")
        for line in resp.splitlines():
            line = line.strip()
            if line.startswith("OK") or line.startswith("ERR"):
                continue
            try:
                return float(line)
            except ValueError:
                continue
        return None

    def zero_save(self, ch: int):
        self.query(f"ZEROSAVE {ch}")

    def sign_get(self, ch: int) -> int | None:
        resp = self.query(f"SIGNGET {ch}")
        for line in resp.splitlines():
            line = line.strip()
            if line.startswith("OK") or line.startswith("ERR"):
                continue
            try:
                return int(line)
            except ValueError:
                continue
        return None

    def sign_set(self, ch: int, sign: int):
        self.send(f"SIGN {ch} {sign}")

    def sign_save(self, ch: int):
        self.query(f"SIGNSAVE {ch}")

    def close(self):
        self.ser.close()


# ── keyframe data ────────────────────────────────────────────────────────────

class Keyframe:
    def __init__(self, angles: dict[int, float], duration: float = 1.0,
                 label: str = ""):
        self.angles = dict(angles)      # {channel: degrees}
        self.duration = duration         # seconds to lerp TO this keyframe
        self.label = label

    def to_dict(self) -> dict:
        return {
            "angles": {str(k): v for k, v in self.angles.items()},
            "duration": self.duration,
            "label": self.label,
        }

    @staticmethod
    def from_dict(d: dict) -> "Keyframe":
        return Keyframe(
            angles={int(k): float(v) for k, v in d["angles"].items()},
            duration=float(d.get("duration", 1.0)),
            label=d.get("label", ""),
        )


# ── UI helpers ───────────────────────────────────────────────────────────────

def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def draw_text(surf, text, x, y, font, color=C_TEXT):
    s = font.render(text, True, color)
    surf.blit(s, (x, y))
    return s.get_rect(topleft=(x, y))


def draw_button(surf, rect, text, font, hover=False, active=False,
                color=None):
    bg = color or (C_BTN_ACTIVE if active else C_BTN_HOVER if hover else C_BTN)
    pygame.draw.rect(surf, bg, rect, border_radius=4)
    pygame.draw.rect(surf, C_TEXT_DIM, rect, 1, border_radius=4)
    ts = font.render(text, True, C_TEXT)
    tx = rect.x + (rect.w - ts.get_width()) // 2
    ty = rect.y + (rect.h - ts.get_height()) // 2
    surf.blit(ts, (tx, ty))


def point_in_rect(pos, rect):
    return rect.collidepoint(pos)


# ── main application ─────────────────────────────────────────────────────────

class App:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
        pygame.display.set_caption("RoboDawg Servo Tool")
        self.clock = pygame.time.Clock()

        self.font = pygame.font.SysFont("menlo,consolas,monospace", 13)
        self.font_sm = pygame.font.SysFont("menlo,consolas,monospace", 11)
        self.font_lg = pygame.font.SysFont("menlo,consolas,monospace", 16,
                                           bold=True)

        self.conn = SerialConn(PORT, BAUD)

        # slider state: {channel: current_angle}
        self.angles: dict[int, float] = {
            ch: NEUTRAL_DEG for _, ch in SERVOS
        }
        self.dragging_slider: int | None = None  # channel being dragged

        # keyframes
        self.keyframes: list[Keyframe] = []
        self.kf_selected: int = -1

        # playback
        self.playing = False
        self.looping = False
        self.play_idx = 0       # current target keyframe index
        self.play_t = 0.0       # progress 0..1 within current segment
        self.play_start_angles: dict[int, float] = {}
        self.last_tick = 0.0
        self.last_servo_write = 0.0  # throttle servo writes

        # log
        self.log_lines: list[str] = []

        # button rects (computed in draw)
        self.buttons: dict[str, pygame.Rect] = {}

        # zero offsets per channel
        self.zeros: dict[int, float] = {ch: NEUTRAL_DEG for _, ch in SERVOS}

        # sign per channel (+1 or -1)
        self.signs: dict[int, int] = {ch: 1 for _, ch in SERVOS}

        # per-servo button rects
        self.zero_btn_rects: dict[int, pygame.Rect] = {}
        self.save_zero_btn_rects: dict[int, pygame.Rect] = {}
        self.sign_btn_rects: dict[int, pygame.Rect] = {}

        # gait parameters
        self.gait_direction = 0.0    # degrees, 0 = forward (+x)
        self.gait_stride = 40.0      # mm
        self.gait_height = 30.0      # mm, step lift height
        self.gait_cycle_time = 4.0   # seconds for full cycle
        self.gait_legs: dict[str, bool] = {
            "FR": True, "FL": True, "BR": True, "BL": True
        }

        # load positions, zeros, and signs from firmware
        self._load_positions()
        self._load_zeros()
        self._load_signs()

    def _load_positions(self):
        """Read current servo positions from firmware."""
        for _, ch in SERVOS:
            pos = self.conn.servo_pos(ch)
            if pos is not None and 0 <= pos <= 270:
                self.angles[ch] = pos

    def _load_zeros(self):
        """Read zero offsets from firmware."""
        for _, ch in SERVOS:
            val = self.conn.zero_get(ch)
            if val is not None:
                self.zeros[ch] = val

    def _load_signs(self):
        """Read sign flips from firmware."""
        for _, ch in SERVOS:
            val = self.conn.sign_get(ch)
            if val is not None:
                self.signs[ch] = val

    def raw_to_joint(self, ch: int) -> float:
        """Convert current raw servo angle to joint-space radians."""
        return radians(self.signs[ch] * (self.angles[ch] - self.zeros[ch]))

    def joint_to_raw(self, ch: int, joint_rad: float) -> float:
        """Convert joint-space radians to raw servo angle (clamped 0-270)."""
        raw = self.signs[ch] * degrees(joint_rad) + self.zeros[ch]
        return max(MIN_DEG, min(MAX_DEG, raw))

    def _generate_gait(self):
        """Generate simple 4-step rectangular gait from current stance.

        Foot positions per step (XZ plane, relative to home):
          Step 0 (fwd):     (+stride/2, 0, 0)
          Step 1 (back):    (-stride/2, 0, 0)
          Step 2 (up-back): (-stride/2, 0, +height)
          Step 3 (up-fwd):  (+stride/2, 0, +height)

        All enabled legs move in phase (same trajectory simultaneously).
        """
        stride = self.gait_stride
        height = self.gait_height
        dir_rad = radians(self.gait_direction)
        dx = cos(dir_rad)
        dy = sin(dir_rad)

        # 4 foot offsets: (dx_mult, dy_mult, dz)
        step_labels = ["fwd", "back", "up-back", "up-fwd"]
        offsets = [
            (+stride / 2 * dx, +stride / 2 * dy, 0.0),
            (-stride / 2 * dx, -stride / 2 * dy, 0.0),
            (-stride / 2 * dx, -stride / 2 * dy, +height),
            (+stride / 2 * dx, +stride / 2 * dy, +height),
        ]

        # Compute home foot positions from current angles via FK
        home_positions: dict[str, Vector3] = {}
        home_joints: dict[str, tuple[float, float, float]] = {}
        for name, (hip_ch, thigh_ch, shin_ch), is_front, is_left in LEGS:
            hip_j = self.raw_to_joint(hip_ch)
            thigh_j = self.raw_to_joint(thigh_ch)
            shin_j = self.raw_to_joint(shin_ch)
            home_joints[name] = (hip_j, thigh_j, shin_j)
            home_positions[name] = leg_fk(hip_j, thigh_j, shin_j,
                                          is_front, is_left)

        keyframes: list[Keyframe] = []
        kf_duration = self.gait_cycle_time / 4.0

        # Print header
        enabled = [n for n in ["FR", "FL", "BR", "BL"] if self.gait_legs[n]]
        print(f"\n{'='*60}")
        print(f"4-Step Gait  dir={self.gait_direction:.0f}° "
              f"stride={stride:.0f}mm h={height:.0f}mm")
        print(f"Enabled legs: {enabled}")
        print(f"{'='*60}")

        for step_i, (off_x, off_y, off_z) in enumerate(offsets):
            angles: dict[int, float] = {}
            step_info: list[str] = []

            for name, (hip_ch, thigh_ch, shin_ch), is_front, is_left in LEGS:
                if not self.gait_legs[name]:
                    continue

                home = home_positions[name]
                target = Vector3(home.x + off_x, home.y + off_y, home.z + off_z)

                result = leg_ik(target.x, target.y, target.z,
                                is_front, is_left)
                if result is None:
                    hip_j, thigh_j, shin_j = home_joints[name]
                    step_info.append(f"  {name}[IK FAIL - using home]")
                else:
                    hip_j, thigh_j, shin_j = result

                raw_hip = self.joint_to_raw(hip_ch, hip_j)
                raw_thigh = self.joint_to_raw(thigh_ch, thigh_j)
                raw_shin = self.joint_to_raw(shin_ch, shin_j)

                angles[hip_ch] = raw_hip
                angles[thigh_ch] = raw_thigh
                angles[shin_ch] = raw_shin

                step_info.append(
                    f"  {name}[hip={raw_hip:.1f} thigh={raw_thigh:.1f} "
                    f"shin={raw_shin:.1f}]")

            keyframes.append(Keyframe(angles, duration=kf_duration,
                                      label=f"step {step_i} ({step_labels[step_i]})"))

            # Print step angles to terminal
            print(f"Step {step_i+1} ({step_labels[step_i]}):")
            for info in step_info:
                print(info)

        print(f"{'='*60}\n")

        self.keyframes = keyframes
        self.kf_selected = 0
        self._log(f"OK 4-step gait: see terminal for angles")

    def _log(self, msg: str):
        self.log_lines.append(msg)
        if len(self.log_lines) > 100:
            self.log_lines.pop(0)

    # ── slider geometry ──────────────────────────────────────────────────────

    def _slider_rect(self, idx: int) -> pygame.Rect:
        y = 60 + idx * (SLIDER_H + SLIDER_PAD)
        return pygame.Rect(SLIDER_X + LABEL_W, y, SLIDER_W - LABEL_W, SLIDER_H)

    def _angle_from_slider(self, idx: int, mx: int) -> float:
        r = self._slider_rect(idx)
        t = (mx - r.x) / r.w
        t = max(0.0, min(1.0, t))
        return round(MIN_DEG + t * (MAX_DEG - MIN_DEG), 1)

    # ── draw ─────────────────────────────────────────────────────────────────

    def draw(self):
        W, H = self.screen.get_size()
        self.screen.fill(C_BG)

        mx, my = pygame.mouse.get_pos()

        # ── title ────────────────────────────────────────────────────────────
        draw_text(self.screen, "RoboDawg Servo Tool", 20, 12, self.font_lg,
                  C_ACCENT)
        draw_text(self.screen, f"port: {PORT}", 240, 16, self.font_sm,
                  C_TEXT_DIM)

        # ── servo sliders ────────────────────────────────────────────────────
        draw_text(self.screen, "Servos", SLIDER_X, 40, self.font, C_ACCENT)

        for i, (label, ch) in enumerate(SERVOS):
            y = 60 + i * (SLIDER_H + SLIDER_PAD)
            leg_col = LEG_COLORS.get(ch, C_TEXT)

            # label
            draw_text(self.screen, f"{label}", SLIDER_X, y + 2, self.font_sm,
                      leg_col)

            # slider track
            r = self._slider_rect(i)
            pygame.draw.rect(self.screen, C_SLIDER_BG, r, border_radius=3)

            # filled portion
            t = (self.angles[ch] - MIN_DEG) / (MAX_DEG - MIN_DEG)
            fill_w = int(t * r.w)
            if fill_w > 0:
                fill_r = pygame.Rect(r.x, r.y, fill_w, r.h)
                pygame.draw.rect(self.screen, C_SLIDER_FG, fill_r,
                                 border_radius=3)

            # knob
            kx = r.x + fill_w
            pygame.draw.circle(self.screen, C_SLIDER_KNOB,
                               (kx, r.y + r.h // 2), 8)

            # angle text
            info_x = r.x + r.w + 8
            draw_text(self.screen, f"{self.angles[ch]:5.1f}\u00b0",
                      info_x, y + 2, self.font_sm, C_TEXT)

            # channel
            draw_text(self.screen, f"ch{ch:02d}",
                      info_x + 52, y + 2, self.font_sm, C_TEXT_DIM)

            # joint angle display
            joint = degrees(self.raw_to_joint(ch))
            draw_text(self.screen, f"j:{joint:+.1f}\u00b0",
                      info_x + 96, y + 2, self.font_sm, C_TEXT_DIM)

            # "Set 0" button
            btn_x = info_x + 160
            btn_set0 = pygame.Rect(btn_x, y, 42, SLIDER_H)
            self.zero_btn_rects[ch] = btn_set0
            draw_button(self.screen, btn_set0, "Set0", self.font_sm,
                        point_in_rect((mx, my), btn_set0))

            # "Save" button (saves zero)
            btn_save = pygame.Rect(btn_x + 46, y, 42, SLIDER_H)
            self.save_zero_btn_rects[ch] = btn_save
            draw_button(self.screen, btn_save, "Save", self.font_sm,
                        point_in_rect((mx, my), btn_save))

            # Sign toggle button (±)
            sign_label = "+" if self.signs[ch] > 0 else "-"
            btn_sign = pygame.Rect(btn_x + 92, y, 26, SLIDER_H)
            self.sign_btn_rects[ch] = btn_sign
            sign_col = C_GREEN if self.signs[ch] > 0 else C_RED
            draw_button(self.screen, btn_sign, sign_label, self.font_sm,
                        point_in_rect((mx, my), btn_sign), color=sign_col)

        # ── slider action buttons ────────────────────────────────────────────
        btn_y = 60 + len(SERVOS) * (SLIDER_H + SLIDER_PAD) + 10

        btn_save_rest = pygame.Rect(SLIDER_X, btn_y, BTN_W, BTN_H)
        btn_load_rest = pygame.Rect(SLIDER_X + BTN_W + 8, btn_y, BTN_W, BTN_H)
        btn_all_mid = pygame.Rect(SLIDER_X + 2 * (BTN_W + 8), btn_y,
                                  BTN_W, BTN_H)

        self.buttons["save_rest"] = btn_save_rest
        self.buttons["load_rest"] = btn_load_rest
        self.buttons["all_mid"] = btn_all_mid

        draw_button(self.screen, btn_save_rest, "Save Rest", self.font_sm,
                    point_in_rect((mx, my), btn_save_rest), color=C_GREEN)
        draw_button(self.screen, btn_load_rest, "Load Rest", self.font_sm,
                    point_in_rect((mx, my), btn_load_rest))
        draw_button(self.screen, btn_all_mid, "All 135", self.font_sm,
                    point_in_rect((mx, my), btn_all_mid))

        # ── gait controls ─────────────────────────────────────────────────────
        gait_y = btn_y + BTN_H + 16
        draw_text(self.screen, "Gait", SLIDER_X, gait_y, self.font, C_ACCENT)
        gait_y += 20

        # Direction row
        draw_text(self.screen, f"Dir: {self.gait_direction:.0f}\u00b0",
                  SLIDER_X, gait_y + 4, self.font_sm, C_TEXT)
        bx = SLIDER_X + 80
        for key, label in [("gait_dir_down", "-"), ("gait_dir_up", "+")]:
            br = pygame.Rect(bx, gait_y, 26, 22)
            self.buttons[key] = br
            draw_button(self.screen, br, label, self.font_sm,
                        point_in_rect((mx, my), br))
            bx += 30

        # Stride
        draw_text(self.screen, f"Stride: {self.gait_stride:.0f}",
                  bx + 8, gait_y + 4, self.font_sm, C_TEXT)
        bx += 76
        for key, label in [("gait_str_down", "-"), ("gait_str_up", "+")]:
            br = pygame.Rect(bx, gait_y, 26, 22)
            self.buttons[key] = br
            draw_button(self.screen, br, label, self.font_sm,
                        point_in_rect((mx, my), br))
            bx += 30

        # Height
        draw_text(self.screen, f"H: {self.gait_height:.0f}",
                  bx + 8, gait_y + 4, self.font_sm, C_TEXT)
        bx += 48
        for key, label in [("gait_h_down", "-"), ("gait_h_up", "+")]:
            br = pygame.Rect(bx, gait_y, 26, 22)
            self.buttons[key] = br
            draw_button(self.screen, br, label, self.font_sm,
                        point_in_rect((mx, my), br))
            bx += 30

        # Second row: cycle time + generate button
        gait_y2 = gait_y + 28
        draw_text(self.screen, f"Cycle: {self.gait_cycle_time:.1f}s",
                  SLIDER_X, gait_y2 + 4, self.font_sm, C_TEXT)
        bx = SLIDER_X + 90
        for key, label in [("gait_cyc_down", "-"), ("gait_cyc_up", "+")]:
            br = pygame.Rect(bx, gait_y2, 26, 22)
            self.buttons[key] = br
            draw_button(self.screen, br, label, self.font_sm,
                        point_in_rect((mx, my), br))
            bx += 30

        # Generate button
        gen_r = pygame.Rect(bx + 12, gait_y2, 72, 22)
        self.buttons["gen_gait"] = gen_r
        draw_button(self.screen, gen_r, "Gen Gait", self.font_sm,
                    point_in_rect((mx, my), gen_r), color=C_ACCENT)

        # Third row: leg toggles + stop
        gait_y3 = gait_y2 + 28
        draw_text(self.screen, "Legs:", SLIDER_X, gait_y3 + 4,
                  self.font_sm, C_TEXT)
        bx = SLIDER_X + 44
        leg_colors = {"FR": C_LEG_FR, "FL": C_LEG_FL,
                      "BR": C_LEG_BR, "BL": C_LEG_BL}
        for leg_name in ["FR", "FL", "BR", "BL"]:
            btn_key = f"gait_leg_{leg_name}"
            br = pygame.Rect(bx, gait_y3, 32, 22)
            self.buttons[btn_key] = br
            active = self.gait_legs[leg_name]
            col = leg_colors[leg_name] if active else C_BTN
            draw_button(self.screen, br, leg_name, self.font_sm,
                        point_in_rect((mx, my), br), active=active,
                        color=col if active else None)
            bx += 36

        # Stop button
        stop_r = pygame.Rect(bx + 8, gait_y3, 48, 22)
        self.buttons["gait_stop"] = stop_r
        draw_button(self.screen, stop_r, "Stop", self.font_sm,
                    point_in_rect((mx, my), stop_r),
                    active=self.playing, color=C_RED if self.playing else None)

        # ── keyframe panel ───────────────────────────────────────────────────
        kf_x = KF_PANEL_X
        kf_w = W - KF_PANEL_X - 20

        # panel background
        panel_r = pygame.Rect(kf_x - 8, 36, kf_w + 16, H - 50)
        pygame.draw.rect(self.screen, C_PANEL, panel_r, border_radius=6)

        draw_text(self.screen, "Trajectory Keyframes", kf_x, 44,
                  self.font, C_ACCENT)

        # toolbar
        tb_y = 68
        btns = [
            ("add_kf",   "Add KF"),
            ("del_kf",   "Del KF"),
            ("up_kf",    "Up"),
            ("down_kf",  "Down"),
            ("play",     "Stop" if self.playing else "Play"),
            ("loop",     "Loop"),
            ("dur_down", "Dur -"),
            ("dur_up",   "Dur +"),
            ("export",   "Export"),
            ("import",   "Import"),
        ]
        bx = kf_x
        for key, label in btns:
            bw = self.font_sm.size(label)[0] + 20
            br = pygame.Rect(bx, tb_y, bw, 26)
            self.buttons[key] = br
            active = (key == "loop" and self.looping) or \
                     (key == "play" and self.playing)
            draw_button(self.screen, br, label, self.font_sm,
                        point_in_rect((mx, my), br), active)
            bx += bw + 4

        # keyframe list
        list_y = KF_LIST_Y
        list_bottom = H - 80
        kf_visible_h = list_bottom - list_y

        # header
        draw_text(self.screen, "  #   Duration   Label / Angles",
                  kf_x, list_y - 16, self.font_sm, C_TEXT_DIM)

        clip_r = pygame.Rect(kf_x - 4, list_y, kf_w + 8, kf_visible_h)
        self.screen.set_clip(clip_r)

        self.kf_rects: list[pygame.Rect] = []
        for i, kf in enumerate(self.keyframes):
            ky = list_y + i * KF_ITEM_H
            if ky > list_bottom:
                break
            kr = pygame.Rect(kf_x, ky, kf_w, KF_ITEM_H - 2)
            self.kf_rects.append(kr)

            is_sel = (i == self.kf_selected)
            is_play_target = (self.playing and i == self.play_idx)
            bg = C_KF_SEL if is_sel else C_KF_PLAY if is_play_target else C_KF_BG
            pygame.draw.rect(self.screen, bg, kr, border_radius=3)

            # index
            draw_text(self.screen, f"{i:3d}", kf_x + 4, ky + 4,
                      self.font_sm, C_TEXT)
            # duration
            draw_text(self.screen, f"{kf.duration:5.2f}s", kf_x + 40, ky + 4,
                      self.font_sm, C_YELLOW)
            # label
            label_text = kf.label or "—"
            draw_text(self.screen, label_text, kf_x + 100, ky + 4,
                      self.font_sm, C_TEXT)

            # mini angle preview
            preview_x = kf_x + 200
            for j, (_, ch) in enumerate(SERVOS):
                a = kf.angles.get(ch, NEUTRAL_DEG)
                t = a / MAX_DEG
                bar_w = 30
                bar_h = 6
                bx_pos = preview_x + j * (bar_w + 2)
                by_pos = ky + 20
                if bx_pos + bar_w > kf_x + kf_w:
                    break
                pygame.draw.rect(self.screen, C_SLIDER_BG,
                                 (bx_pos, by_pos, bar_w, bar_h),
                                 border_radius=1)
                fill = int(t * bar_w)
                if fill > 0:
                    col = LEG_COLORS.get(ch, C_SLIDER_FG)
                    pygame.draw.rect(self.screen, col,
                                     (bx_pos, by_pos, fill, bar_h),
                                     border_radius=1)

        self.screen.set_clip(None)

        # playback progress indicator
        if self.playing and self.keyframes:
            prog_y = list_bottom + 10
            draw_text(self.screen, f"Playing: KF {self.play_idx} "
                      f"({self.play_t:.0%})",
                      kf_x, prog_y, self.font_sm, C_GREEN)
            # progress bar
            bar_r = pygame.Rect(kf_x + 180, prog_y + 2, 200, 12)
            pygame.draw.rect(self.screen, C_SLIDER_BG, bar_r, border_radius=3)
            total_kf = len(self.keyframes)
            overall = (self.play_idx + self.play_t) / \
                total_kf if total_kf else 0
            fill = int(overall * bar_r.w)
            if fill > 0:
                pygame.draw.rect(self.screen, C_GREEN,
                                 (bar_r.x, bar_r.y, fill, bar_r.h),
                                 border_radius=3)

        # ── log pane ─────────────────────────────────────────────────────────
        log_y = H - 60
        pygame.draw.line(self.screen, C_HEADER, (10, log_y - 4),
                         (W - 10, log_y - 4))
        visible = self.log_lines[-3:]
        for j, line in enumerate(visible):
            col = C_GREEN if line.startswith("OK") else \
                C_RED if line.startswith("ERR") else C_TEXT_DIM
            draw_text(self.screen, line[:120], 12, log_y + j * 16,
                      self.font_sm, col)

        pygame.display.flip()

    # ── playback ─────────────────────────────────────────────────────────────

    def start_playback(self):
        if not self.keyframes:
            self._log("ERR no keyframes to play")
            return
        self.playing = True
        self.play_idx = 0
        self.play_t = 0.0
        self.play_start_angles = dict(self.angles)
        self.last_tick = time()
        enabled = [n for n in ["FR", "FL", "BR", "BL"] if self.gait_legs[n]]
        self._log(f"OK playing, enabled legs: {enabled}")

    def stop_playback(self):
        self.playing = False
        # Discard any queued serial writes so servos stop immediately
        self.conn.ser.reset_output_buffer()
        self.conn.ser.reset_input_buffer()

    def _enabled_channels(self) -> set[int]:
        """Return set of channels whose legs are currently enabled."""
        enabled: set[int] = set()
        for ch, leg in CHANNEL_TO_LEG.items():
            if self.gait_legs.get(leg, True):
                enabled.add(ch)
        return enabled

    def tick_playback(self):
        if not self.playing or not self.keyframes:
            return

        now = time()
        dt = now - self.last_tick
        self.last_tick = now

        # Compute which channels are allowed to be driven right now
        allowed = self._enabled_channels()

        kf = self.keyframes[self.play_idx]
        dur = max(0.01, kf.duration)
        self.play_t += dt / dur

        if self.play_t >= 1.0:
            # snap to target
            for ch, angle in kf.angles.items():
                if ch not in allowed:
                    continue
                self.angles[ch] = angle
                self.conn.servo(ch, angle)
            self.last_servo_write = now

            self.play_idx += 1
            if self.play_idx >= len(self.keyframes):
                if self.looping:
                    self.play_idx = 0
                else:
                    self.stop_playback()
                    self._log("OK playback finished")
                    return

            # set up next segment
            self.play_start_angles = dict(self.angles)
            self.play_t = 0.0
        else:
            # Throttle interpolation writes to ~20Hz to avoid saturating serial
            if now - self.last_servo_write < 0.05:
                return

            # interpolate — only channels in keyframe AND enabled
            t = smoothstep(self.play_t)
            for ch in kf.angles:
                if ch not in allowed:
                    continue
                start = self.play_start_angles.get(ch, self.angles[ch])
                end = kf.angles[ch]
                a = lerp(start, end, t)
                self.angles[ch] = round(a, 1)
                self.conn.servo(ch, self.angles[ch])
            self.last_servo_write = now

    # ── event handling ───────────────────────────────────────────────────────

    def handle_events(self):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return False

            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                self._on_click(ev.pos)

            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                self.dragging_slider = None

            elif ev.type == pygame.MOUSEMOTION:
                if self.dragging_slider is not None:
                    self._on_slider_drag(ev.pos)

            elif ev.type == pygame.KEYDOWN:
                self._on_key(ev)

        return True

    def _on_click(self, pos):
        mx, my = pos

        # check per-servo zero/sign buttons
        for _, ch in SERVOS:
            if ch in self.zero_btn_rects and \
               self.zero_btn_rects[ch].collidepoint(mx, my):
                # Set current raw angle as zero offset
                self.zeros[ch] = self.angles[ch]
                self.conn.zero_set(ch, self.angles[ch])
                self._log(f"OK zero ch{ch} = {self.angles[ch]:.1f}")
                return
            if ch in self.save_zero_btn_rects and \
               self.save_zero_btn_rects[ch].collidepoint(mx, my):
                # Persist zero offset to NVS
                self.conn.zero_save(ch)
                self._log(f"OK saved zero ch{ch} = {self.zeros[ch]:.1f}")
                return
            if ch in self.sign_btn_rects and \
               self.sign_btn_rects[ch].collidepoint(mx, my):
                # Toggle sign and persist
                self.signs[ch] = -1 if self.signs[ch] > 0 else 1
                self.conn.sign_set(ch, self.signs[ch])
                self.conn.sign_save(ch)
                self._log(f"OK sign ch{ch} = {self.signs[ch]:+d}")
                return

        # check slider clicks
        for i, (_, ch) in enumerate(SERVOS):
            r = self._slider_rect(i)
            if r.collidepoint(mx, my):
                self.dragging_slider = i
                self.angles[ch] = self._angle_from_slider(i, mx)
                self.conn.servo(ch, self.angles[ch])
                return

        # check buttons
        for key, rect in self.buttons.items():
            if rect.collidepoint(mx, my):
                self._on_button(key)
                return

        # check keyframe list clicks
        for i, kr in enumerate(getattr(self, "kf_rects", [])):
            if kr.collidepoint(mx, my):
                self.kf_selected = i
                return

    def _on_slider_drag(self, pos):
        if self.dragging_slider is None:
            return
        idx = self.dragging_slider
        _, ch = SERVOS[idx]
        self.angles[ch] = self._angle_from_slider(idx, pos[0])
        self.conn.servo(ch, self.angles[ch])

    def _on_button(self, key: str):
        if key == "save_rest":
            # Flush any pending serial data before issuing queries
            self.conn.ser.flush()
            sleep(0.1)
            self.conn.ser.reset_input_buffer()
            for _, ch in SERVOS:
                resp = self.conn.servo_save(ch)
                self._log(resp)
            self._log("OK all rest positions saved")

        elif key == "load_rest":
            # Flush any pending serial data before issuing queries
            self.conn.ser.flush()
            sleep(0.1)
            self.conn.ser.reset_input_buffer()
            # First read all rest positions (no interleaved writes)
            rest_angles: dict[int, float] = {}
            for _, ch in SERVOS:
                pos = self.conn.rest_get(ch)
                if pos is not None and 0 <= pos <= 270:
                    rest_angles[ch] = pos
            # Then apply them all
            for ch, pos in rest_angles.items():
                self.angles[ch] = pos
                self.conn.servo(ch, pos)
            self._log("OK loaded rest positions from NVS")

        elif key == "all_mid":
            for _, ch in SERVOS:
                self.angles[ch] = NEUTRAL_DEG
                self.conn.servo(ch, NEUTRAL_DEG)
            self._log("OK all servos to 135")

        elif key == "add_kf":
            idx = len(self.keyframes)
            label = f"KF {idx}"
            kf = Keyframe(dict(self.angles), duration=1.0, label=label)
            if self.kf_selected >= 0 and self.kf_selected < len(self.keyframes):
                self.keyframes.insert(self.kf_selected + 1, kf)
                self.kf_selected += 1
            else:
                self.keyframes.append(kf)
                self.kf_selected = len(self.keyframes) - 1
            self._log(f"OK added keyframe {self.kf_selected}")

        elif key == "del_kf":
            if 0 <= self.kf_selected < len(self.keyframes):
                self.keyframes.pop(self.kf_selected)
                if self.kf_selected >= len(self.keyframes):
                    self.kf_selected = len(self.keyframes) - 1
                self._log("OK deleted keyframe")

        elif key == "up_kf":
            i = self.kf_selected
            if i > 0:
                self.keyframes[i], self.keyframes[i-1] = \
                    self.keyframes[i-1], self.keyframes[i]
                self.kf_selected -= 1

        elif key == "down_kf":
            i = self.kf_selected
            if 0 <= i < len(self.keyframes) - 1:
                self.keyframes[i], self.keyframes[i+1] = \
                    self.keyframes[i+1], self.keyframes[i]
                self.kf_selected += 1

        elif key == "play":
            if self.playing:
                self.stop_playback()
                self._log("OK stopped")
            else:
                self.start_playback()

        elif key == "loop":
            self.looping = not self.looping
            self._log(f"OK loop {'on' if self.looping else 'off'}")

        elif key == "dur_down":
            if 0 <= self.kf_selected < len(self.keyframes):
                kf = self.keyframes[self.kf_selected]
                kf.duration = max(0.1, kf.duration - 0.25)
                self._log(f"OK duration = {kf.duration:.2f}s")

        elif key == "dur_up":
            if 0 <= self.kf_selected < len(self.keyframes):
                kf = self.keyframes[self.kf_selected]
                kf.duration = min(30.0, kf.duration + 0.25)
                self._log(f"OK duration = {kf.duration:.2f}s")

        elif key == "export":
            self._export_trajectory()

        elif key == "import":
            self._import_trajectory()

        elif key == "gait_dir_down":
            self.gait_direction = (self.gait_direction - 15) % 360
        elif key == "gait_dir_up":
            self.gait_direction = (self.gait_direction + 15) % 360
        elif key == "gait_str_down":
            self.gait_stride = max(5, self.gait_stride - 5)
        elif key == "gait_str_up":
            self.gait_stride = min(100, self.gait_stride + 5)
        elif key == "gait_h_down":
            self.gait_height = max(5, self.gait_height - 5)
        elif key == "gait_h_up":
            self.gait_height = min(80, self.gait_height + 5)
        elif key == "gait_cyc_down":
            self.gait_cycle_time = max(0.5, self.gait_cycle_time - 0.5)
        elif key == "gait_cyc_up":
            self.gait_cycle_time = min(20.0, self.gait_cycle_time + 0.5)
        elif key == "gen_gait":
            self._generate_gait()
        elif key == "gait_stop":
            self.stop_playback()
            self._log("OK stopped")
        elif key.startswith("gait_leg_"):
            leg_name = key[len("gait_leg_"):]
            self.gait_legs[leg_name] = not self.gait_legs[leg_name]
            state = "on" if self.gait_legs[leg_name] else "off"
            self._log(f"OK {leg_name} {state}")
            print(f"[DEBUG] gait_legs = {self.gait_legs}")
            print(f"[DEBUG] enabled channels = {sorted(self._enabled_channels())}")

    def _on_key(self, ev):
        if ev.key == pygame.K_SPACE:
            self._on_button("play")
        elif ev.key == pygame.K_l:
            self._on_button("loop")
        elif ev.key == pygame.K_a:
            self._on_button("add_kf")
        elif ev.key == pygame.K_DELETE or ev.key == pygame.K_BACKSPACE:
            self._on_button("del_kf")
        elif ev.key == pygame.K_UP and (ev.mod & pygame.KMOD_SHIFT):
            self._on_button("up_kf")
        elif ev.key == pygame.K_DOWN and (ev.mod & pygame.KMOD_SHIFT):
            self._on_button("down_kf")
        elif ev.key == pygame.K_LEFT and (ev.mod & pygame.KMOD_SHIFT):
            self._on_button("dur_down")
        elif ev.key == pygame.K_RIGHT and (ev.mod & pygame.KMOD_SHIFT):
            self._on_button("dur_up")
        elif ev.key == pygame.K_RETURN:
            # apply selected keyframe angles to sliders
            if 0 <= self.kf_selected < len(self.keyframes):
                kf = self.keyframes[self.kf_selected]
                for ch, angle in kf.angles.items():
                    if ch in self.angles:
                        self.angles[ch] = angle
                        self.conn.servo(ch, angle)
                self._log(f"OK applied KF {self.kf_selected}")

    def _export_trajectory(self):
        path = "trajectory.json"
        data = {
            "keyframes": [kf.to_dict() for kf in self.keyframes],
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        self._log(f"OK exported {len(self.keyframes)} keyframes to {path}")

    def _import_trajectory(self):
        path = "trajectory.json"
        if not os.path.exists(path):
            self._log(f"ERR {path} not found")
            return
        with open(path) as f:
            data = json.load(f)
        self.keyframes = [Keyframe.from_dict(d) for d in data["keyframes"]]
        self.kf_selected = 0 if self.keyframes else -1
        self._log(f"OK imported {len(self.keyframes)} keyframes from {path}")

    # ── main loop ────────────────────────────────────────────────────────────

    def run(self):
        running = True
        try:
            while running:
                running = self.handle_events()
                self.tick_playback()
                self.draw()
                self.clock.tick(60)
        finally:
            self.conn.close()
            pygame.quit()


if __name__ == "__main__":
    App().run()
