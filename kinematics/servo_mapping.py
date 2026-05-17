# Standing calibration from Arduino code
RH = 100
RK = 95
RA = 115

LH = 89
LK = 99
LA = 111


def clamp(angle, min_angle=0, max_angle=180):
    return max(min_angle, min(max_angle, angle))


def map_right_leg_ik_to_servo(hip_ik, knee_ik):
    """
    Convert IK angles to right-leg servo angles.
    Standing pose is used as the zero/reference pose.
    """

    hip_offset = abs(hip_ik) * 0.45
    knee_offset = abs(knee_ik) * 0.25

    # Right hip:
    # RH + value moves right leg forward
    right_hip_servo = RH + hip_offset

    # Right knee:
    # RK + value bends knee forward
    right_knee_servo = RK + knee_offset

    return {
        "right_hip": int(clamp(right_hip_servo)),
        "right_knee": int(clamp(right_knee_servo)),
        "right_ankle": RA
    }


def map_left_leg_ik_to_servo(hip_ik, knee_ik):
    """
    Convert IK angles to left-leg servo angles.
    Left hip moves forward when angle decreases.
    """

    hip_offset = abs(hip_ik) * 0.45
    knee_offset = abs(knee_ik) * 0.25

    # Left hip:
    # LH - value moves left leg forward
    left_hip_servo = LH - hip_offset

    # Left knee:
    # LK + value bends knee forward
    left_knee_servo = LK + knee_offset

    return {
        "left_hip": int(clamp(left_hip_servo)),
        "left_knee": int(clamp(left_knee_servo)),
        "left_ankle": LA
    }


if __name__ == "__main__":
    hip_ik = -15.245
    knee_ik = 83.62

    print("Right leg:", map_right_leg_ik_to_servo(hip_ik, knee_ik))
    print("Left leg:", map_left_leg_ik_to_servo(hip_ik, knee_ik))