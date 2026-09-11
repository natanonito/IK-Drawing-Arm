import math
import time
from machine import Pin, PWM


# Hardware bindings (ESP32 → PWM @ 50 Hz)


base_servo  = PWM(Pin(12))
base_servo.freq(50)

elbow_servo = PWM(Pin(13))
elbow_servo.freq(50)


# Mechanical alignment parameters
# (used to match math angles to real servos)

BASE_ZERO_SHIFT  = 0
BASE_SIGN        = 1

ELBOW_ZERO_SHIFT = 90
ELBOW_SIGN       = 1


# Servo command abstraction
# Input : angle in degrees (0–180)
# Output: PWM duty cycle


def write_servo_deg(pwm_obj, deg):
    deg = max(0, min(180, deg))

    pulse_min = 500
    pulse_max = 2500
    frame_us  = 20000   # 20 ms period → 50 Hz

    pulse_us = pulse_min + (deg / 180.0) * (pulse_max - pulse_min)

    duty_val = int((pulse_us / frame_us) * 1023)
    pwm_obj.duty(duty_val)


# 2-DOF planar arm inverse kinematics
# Returns all valid joint angle pairs


def solve_ik(xp, yp, link_a, link_b):
    dist_sq = xp * xp + yp * yp
    dist    = math.sqrt(dist_sq)

    # Reachability test
    if dist > (link_a + link_b) or dist < abs(link_a - link_b):
        return []

    cos_j2 = (dist_sq - link_a**2 - link_b**2) / (2 * link_a * link_b)
    cos_j2 = max(-1, min(1, cos_j2))

    joint2_a = math.acos(cos_j2)
    joint2_b = -joint2_a

    solutions = []

    for j2 in (joint2_a, joint2_b):
        a = link_a + link_b * math.cos(j2)
        b = link_b * math.sin(j2)

        j1 = math.atan2(yp, xp) - math.atan2(b, a)
        solutions.append((math.degrees(j1), math.degrees(j2)))

    return solutions


# Angle mapping utilities


def map_joint_to_servo(theta, zero, sign):
    return sign * theta + zero

def limit(v, lo=0, hi=180):
    return max(lo, min(hi, v))


# Cartesian move request
# (X, Y → joint angles → servo angles)


def goto_xy(x, y, link_a, link_b):
    ik_sets = solve_ik(x, y, link_a, link_b)

    if not ik_sets:
        print("Unreachable point:", x, y)
        return

    j1, j2 = ik_sets[0]   # select one configuration

    s_base  = limit(map_joint_to_servo(j1, BASE_ZERO_SHIFT, BASE_SIGN))
    s_elbow = limit(map_joint_to_servo(j2, ELBOW_ZERO_SHIFT, ELBOW_SIGN))

    write_servo_deg(base_servo, s_base)
    write_servo_deg(elbow_servo, s_elbow)


# Straight-line interpolation in task space


def trace_segment(xa, ya, xb, yb, link_a, link_b, steps=20):
    for i in range(steps + 1):
        r = i / steps

        xt = xa + (xb - xa) * r
        yt = ya + (yb - ya) * r

        goto_xy(xt, yt, link_a, link_b)
        time.sleep(0.05)


# Test routine: continuous square path


def run():
    arm_1 = 11
    arm_2 = 12

    A = (8, 8)
    B = (8, 14)
    C = (14, 14)
    D = (14, 8)

    print("Square motion started")

    try:
        while True:
            trace_segment(*A, *B, arm_1, arm_2)
            trace_segment(*B, *C, arm_1, arm_2)
            trace_segment(*C, *D, arm_1, arm_2)
            trace_segment(*D, *A, arm_1, arm_2)

    except KeyboardInterrupt:
        print("Motion halted")
        base_servo.duty(0)
        elbow_servo.duty(0)

# =================================================

if __name__ == "__main__":
    run()
