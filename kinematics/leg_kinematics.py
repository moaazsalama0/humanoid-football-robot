import math


L1 = 6.0  # hip to knee in cm
L2 = 6.0  # knee to foot in cm


def forward_kinematics(theta1_deg, theta2_deg):
    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)

    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)

    return x, y


def inverse_kinematics(x, y, elbow_down=True):
    d = math.sqrt(x**2 + y**2)

    if d > L1 + L2:
        raise ValueError("Target is outside reachable workspace")

    if d < abs(L1 - L2):
        raise ValueError("Target is too close to reach")

    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(-1, min(1, cos_theta2))

    if elbow_down:
        theta2 = math.acos(cos_theta2)
    else:
        theta2 = -math.acos(cos_theta2)

    theta1 = math.atan2(y, x) - math.atan2(
        L2 * math.sin(theta2),
        L1 + L2 * math.cos(theta2)
    )

    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)

    return theta1_deg, theta2_deg


def validate_ik(x, y):
    theta1, theta2 = inverse_kinematics(x, y)
    x_fk, y_fk = forward_kinematics(theta1, theta2)

    error = math.sqrt((x - x_fk) ** 2 + (y - y_fk) ** 2)

    return {
        "target": (x, y),
        "angles": (theta1, theta2),
        "computed_position": (x_fk, y_fk),
        "error_cm": error
    }


if __name__ == "__main__":
    target_x = 4
    target_y = -8

    result = validate_ik(target_x, target_y)

    print("Target:", result["target"])
    print("Angles:", result["angles"])
    print("FK Position:", result["computed_position"])
    print("Error cm:", result["error_cm"])