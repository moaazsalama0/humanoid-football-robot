from leg_kinematics import inverse_kinematics
from servo_mapping import map_right_leg_ik_to_servo

target_x = 8
target_y = 4

hip_ik, knee_ik = inverse_kinematics(target_x, target_y)

servo_angles = map_right_leg_ik_to_servo(hip_ik, knee_ik)

print("IK Hip:", hip_ik)
print("IK Knee:", knee_ik)
print("Servo Angles:", servo_angles)