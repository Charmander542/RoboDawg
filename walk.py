import serial
from time import sleep, time
from math import acos, atan2, cos, degrees, radians, sin, sqrt
import subprocess
import re


def get_devices() -> list[str]:
    output = subprocess.check_output(["pio", "device", "list"]).decode()
    return re.findall(r"^(\/.+)+", output, flags=re.MULTILINE)


PORT = next((dev for dev in get_devices() if 'Bluetooth' not in dev), None)
assert PORT, "not connected to any devices"

BAUD = 115_200

ser = serial.Serial(PORT, BAUD, timeout=0.5)

"""
Servo control:
    - Each leg has two joints (j1, j2)
    - Each joint has angle range ()

"""

FR_HIP = 0
FR_JOINT_1 = 1
FR_JOINT_2 = 2

FL_HIP = 4
FL_JOINT_1 = 5
FL_JOINT_2 = 6

BR_HIP = 8
BR_JOINT_1 = 9
BR_JOINT_2 = 10

BL_HIP = 12
BL_JOINT_1 = 13
BL_JOINT_2 = 14

ANGLE_OFFSETS = {
    BR_JOINT_1: 10,
    BR_JOINT_2: 120,
}

# BR from 160 to 270

# mm
# SHIN_LENGTH = 125
# THIGH_LENGTH = 65

SHIN_LENGTH = 2
THIGH_LENGTH = 1


def servo(ch: int, angle_deg: float):
    angle_deg = max(0, min(270, angle_deg))
    assert 0 <= ch <= 15, ch
    ser.write(f"SERVO {ch} {angle_deg:.1f}\n".encode())


def repl():
    while True:
        line = input("READING LINE: ")
        ser.write(line.encode())
        ser.write(b'\n')
        sleep(0.1)
        print((ser.read_all() or b'').decode())


def stop():
    ser.write(f"STOP\n".encode())


def fk(joint1: float, joint2: float):
    assert radians(0) <= joint1 <= radians(120)
    assert radians(90) <= joint2 <= radians(180)
    return (
        SHIN_LENGTH * cos(joint1)
        + THIGH_LENGTH * cos(joint2),
        SHIN_LENGTH * sin(joint1)
        + THIGH_LENGTH * sin(joint2),
    )


def ik(x: float, y: float) -> list[tuple[float, float]]:
    """Inverse kinematics for 2 joint chain, with angle constraints.
    Returns list of valid (joint1, joint2) solutions in radians."""
    L1 = SHIN_LENGTH
    L2 = THIGH_LENGTH
    R = sqrt(x * x + y * y)
    phi = atan2(y, x)

    # x*cos(θ2) + y*sin(θ2) = (x²+y²+L2²-L1²) / (2*L2)
    # => R*cos(θ2-φ) = k
    k = (x * x + y * y + L2 * L2 - L1 * L1) / (2 * L2)
    if R == 0 or abs(k / R) > 1:
        return []

    offset = acos(k / R)
    solutions = []
    for theta2 in [phi + offset, phi - offset]:
        # θ1 = atan2(y - L2*sin(θ2), x - L2*cos(θ2))
        theta1 = atan2(y - L2 * sin(theta2), x - L2 * cos(theta2))

        # check constraints: joint1 in [0, 120°], joint2 in [90°, 180°]
        if radians(0) <= theta1 <= radians(120) and radians(90) <= theta2 <= radians(180):
            solutions.append((theta1, theta2))

    return solutions


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def move_to(x: float, y: float, joint1_ch: int, joint2_ch: int, duration: float = 0.5, steps: int = 20):
    """Smoothly move BR leg to target (x, y) position over duration seconds."""
    sols = ik(x, y)
    if not sols:
        print(f"No IK solution for ({x}, {y})")
        return
    target_j1, target_j2 = sols[0]

    # Drive interpolated steps
    start = time()
    for i in range(1, steps + 1):
        t = i / steps
        # Smooth step (ease in-out)
        t = t * t * (3 - 2 * t)
        j1 = lerp(move_to.last_j1, target_j1, t)
        j2 = lerp(move_to.last_j2, target_j2, t)

        servo(joint1_ch, degrees(j1))
        servo(joint2_ch, degrees(j2))

        # servo(FR_JOINT_1, 270 - degrees(j1))
        # servo(FR_JOINT_2, 270 - degrees(j2))

        # servo(FL_JOINT_1, degrees(j1))
        # servo(FL_JOINT_2, degrees(j2))

        # servo(BR_JOINT_1, 270 - degrees(j1))
        # servo(BR_JOINT_2, 270 - degrees(j2))

        # servo(BL_JOINT_1, degrees(j1))
        # servo(BL_JOINT_2, degrees(j2))

        sleep(duration / steps)

    move_to.last_j1 = target_j1
    move_to.last_j2 = target_j2


# Initialize move to lerp
init_j1 = radians(45)
init_j2 = radians(135)
move_to.last_j1 = init_j1
move_to.last_j2 = init_j2
servo(BR_JOINT_1, degrees(init_j1))
servo(BR_JOINT_2, degrees(init_j2))

try:
    while True:
        # SHIN_LENGTH = 2
        # THIGH_LENGTH = 1

        move_to(-1.5, 2, BR_JOINT_1, BR_JOINT_2, duration=2)
        print("AT POSE 1")

        move_to(1.2, 2.2, BR_JOINT_1, BR_JOINT_2, duration=2)
        print("AT POSE 2")

        # move_to(1.5, 1.5,  FL_JOINT_1, FL_JOINT_2, duration=0.3)
        # move_to(0.8, 1.5, FL_JOINT_1, FL_JOINT_2, duration=0.3)

        # move_to(FL_JOINT_1, FL_JOINT_2, *
        #         fk(radians(30), radians(100)), duration=0.5)
        # move_to(FL_JOINT_1, FL_JOINT_2, *
        #         fk(radians(90), radians(180)), duration=0.5)
        # move_to(FL_JOINT_1, FL_JOINT_2, *
        #         fk(radians(45), radians(135)), duration=0.5)

except KeyboardInterrupt:
    stop()
