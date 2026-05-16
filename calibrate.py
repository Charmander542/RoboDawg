"""
RoboDawg servo calibration tool.

Controls
--------
  Tab / Shift-Tab   next / previous servo
  Right / Left      +1° / -1°
  Up / Down         +5° / -5°
  Shift+Right/Left  +0.5° / -0.5° (fine)
  t                 TRIM — save current angle as the zero point for this servo
  m                 SERVOSMID — send all servos to neutral (135°)
  r                 CALRESET — wipe NVS calibration back to firmware defaults
  s                 STATUS — dump firmware state to log pane
  q                 quit (does NOT stop servos)
Trim math
---------
The firmware's neutral is 135° (mid of 0..270° range).  After you dial a
servo to its physical zero with arrow keys, pressing 't' calculates

    offset_deg = current_angle - NEUTRAL_DEG

and sends  TRIM ch offset_deg  which the firmware converts to µs internally
and saves to NVS.  Future SERVO commands will then be offset so 135° hits
the real mechanical zero.
"""

import curses
import re
import subprocess
import sys
import os
from time import sleep
from math import floor

import serial


def get_devices() -> list[str]:
    output = subprocess.check_output(["pio", "device", "list"]).decode()
    return re.findall(r"^(\/.+)+", output, flags=re.MULTILINE)


PORT = os.environ.get('PORT', next(
    (dev for dev in get_devices() if 'Bluetooth' not in dev), None))
assert PORT, "not connected to any devices"

# ── connection ──────────────────────────────────────────────────────────────
BAUD = 115_200

# ── servo geometry ───────────────────────────────────────────────────────────
NEUTRAL_DEG = 135.0   # firmware neutral (mid of 0..270)
MIN_DEG = 0.0
MAX_DEG = 270.0

# channel map: (label, channel)
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

# ── helpers ──────────────────────────────────────────────────────────────────

_CAL_RE = re.compile(r"cal ch(\d+)\s+min=\d+\s+max=\d+\s+trim=(-?[\d.]+)")


def fetch_trims(ser: serial.Serial) -> dict[int, float]:
    """Send STATUS and return {channel: trim_deg}.  Raises RuntimeError if the
    cal lines are missing or unparseable."""
    ser.write(b"STATUS\n")
    sleep(0.15)
    raw = (ser.read_all() or b"").decode(errors="replace")
    trims: dict[int, float] = {}
    for m in _CAL_RE.finditer(raw):
        trims[int(m.group(1))] = float(m.group(2))
    if not trims:
        raise RuntimeError(
            f"Could not parse calibration from STATUS response:\n{raw!r}"
        )
    return trims


def send(ser: serial.Serial, msg: str) -> str:
    ser.write((msg.strip() + "\n").encode())
    sleep(0.05)
    resp = (ser.read_all() or b"").decode(errors="replace").strip()
    return resp


def cmd_servo(ser, ch: int, angle: float) -> str:
    angle = max(MIN_DEG, min(MAX_DEG, angle))
    return send(ser, f"SERVO {ch} {angle:.1f}")


def cmd_trim(ser, ch: int, angle: float) -> str:
    offset_deg = angle - NEUTRAL_DEG
    return send(ser, f"TRIM {ch} {offset_deg:.2f}")


# ── TUI ──────────────────────────────────────────────────────────────────────

def main(stdscr):
    ser = serial.Serial(PORT, BAUD, timeout=0.3)
    sleep(0.3)
    ser.read_all()   # flush startup banner

    # raises RuntimeError → printed before curses starts
    trims = fetch_trims(ser)

    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_CYAN)   # selected row
    curses.init_pair(2, curses.COLOR_GREEN,  -1)                 # OK response
    curses.init_pair(3, curses.COLOR_RED,    -1)                 # ERR response
    curses.init_pair(4, curses.COLOR_YELLOW, -1)                 # header

    # STATUS now returns trim in degrees; add directly to neutral.
    angles = {ch: NEUTRAL_DEG + trims.get(ch, 0.0) for _, ch in SERVOS}
    trimmed = {ch: trims.get(ch, 0.0) != 0.0 for _, ch in SERVOS}
    log_lines: list[str] = []
    sel = 0

    def add_log(line: str):
        log_lines.append(line)
        if len(log_lines) > 200:
            log_lines.pop(0)

    def draw():
        stdscr.erase()
        H, W = stdscr.getmaxyx()

        # ── header ──────────────────────────────────────────────────────────
        header = " RoboDawg Servo Calibrator "
        stdscr.addstr(0, max(0, (W - len(header)) // 2), header,
                      curses.color_pair(4) | curses.A_BOLD)

        keys = (" Tab/↑↓ navigate  ←/→ ±1°  Shift←/→ ±0.5°  ↑/↓ ±5°"
                "  t trim  m mid  r calreset  s status  q quit ")
        stdscr.addstr(1, 0, keys[:W-1], curses.color_pair(4))

        # ── servo table ─────────────────────────────────────────────────────
        col_w = max(26, W // 3)
        for i, (label, ch) in enumerate(SERVOS):
            row = 3 + i
            if row >= H - 4:
                break
            angle = angles[ch]
            trim_marker = "*" if trimmed[ch] else " "
            bar_filled = int((angle / MAX_DEG) * 20)
            bar = "[" + "#" * bar_filled + "-" * (20 - bar_filled) + "]"
            line = f"  ch{ch:02d} {label:<10s} {angle:6.1f}°  {bar} {trim_marker}"
            attr = curses.color_pair(1) | curses.A_BOLD if i == sel else 0
            stdscr.addstr(row, 0, line[:W-1], attr)

        # ── log pane ────────────────────────────────────────────────────────
        log_top = 3 + len(SERVOS) + 1
        if log_top < H - 1:
            stdscr.addstr(log_top - 1, 0, "─" * (W - 1), curses.color_pair(4))
            visible = log_lines[-(H - log_top - 1):]
            for j, line in enumerate(visible):
                r = log_top + j
                if r >= H - 1:
                    break
                attr = curses.color_pair(2) if line.startswith("OK") else \
                    curses.color_pair(3) if line.startswith("ERR") else 0
                stdscr.addstr(r, 0, line[:W-1], attr)

        stdscr.refresh()

    def do_servo(idx: int):
        label, ch = SERVOS[idx]
        resp = cmd_servo(ser, ch, angles[ch])
        add_log(resp or f"-> SERVO {ch} {angles[ch]:.1f}")

    def nudge(delta: float):
        _, ch = SERVOS[sel]
        angles[ch] = max(MIN_DEG, min(MAX_DEG, floor(angles[ch] + delta)))
        do_servo(sel)

    stdscr.nodelay(False)
    stdscr.keypad(True)

    # drive selected servo immediately on start
    do_servo(sel)
    draw()

    while True:
        key = stdscr.getch()

        if key in (ord('q'), ord('Q')):
            break

        elif key == ord('\t'):                       # Tab → next
            sel = (sel + 1) % len(SERVOS)
            do_servo(sel)

        elif key == curses.KEY_BTAB:                 # Shift-Tab → prev
            sel = (sel - 1) % len(SERVOS)
            do_servo(sel)

        elif key == curses.KEY_RIGHT:
            nudge(+1.0)

        elif key == curses.KEY_LEFT:
            nudge(-1.0)

        elif key == curses.KEY_UP:
            nudge(+5.0)

        elif key == curses.KEY_DOWN:
            nudge(-5.0)

        elif key == 0x221:    # Shift+Right (many terminals)
            nudge(+0.5)

        elif key == 0x220:    # Shift+Left
            nudge(-0.5)

        elif key in (ord('t'), ord('T')):
            _, ch = SERVOS[sel]
            resp = cmd_trim(ser, ch, angles[ch])
            trimmed[ch] = True
            add_log(resp or f"-> TRIM {ch} saved")

        elif key in (ord('m'), ord('M')):
            resp = send(ser, "SERVOSMID")
            for _, ch in SERVOS:
                angles[ch] = NEUTRAL_DEG
            add_log(resp or "-> SERVOSMID")

        elif key in (ord('r'), ord('R')):
            resp = send(ser, "CALRESET")
            add_log(resp or "-> CALRESET")
            try:
                fresh = fetch_trims(ser)
                for _, ch in SERVOS:
                    angles[ch] = NEUTRAL_DEG + fresh.get(ch, 0.0)
                    trimmed[ch] = fresh.get(ch, 0.0) != 0.0
            except RuntimeError as e:
                add_log(f"ERR {e}")

        elif key in (ord('s'), ord('S')):
            resp = send(ser, "STATUS")
            for line in resp.splitlines():
                add_log(line)

        draw()

    ser.close()


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except (serial.SerialException, RuntimeError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
