#!/usr/bin/env python3

import rospy
from scipy.spatial import distance
from geometry_msgs.msg import Pose2D
import numpy as np
from math import pi, sqrt, atan2, isclose, cos, sin, degrees, radians, exp

# Stand alone functions

# Simple trig
# All angles are in radians, -pi < angle < pi. 0 is at 3 o'clock, positive is counterclockwise

def calc_distance(pa: Pose2D, pb: Pose2D) -> float:
    return distance.euclidean([pa.x, pa.y], [pb.x, pb.y])

def invalid_pose(pose: Pose2D) -> bool:
    return pose.x == 0 and pose.y == 0 and pose.theta == 0

def invert_angle(angle: float) -> float:
    """Given a normalized angle, return a normalized angle pointing in the opposite direction"""
    if angle < 0:
        invert = pi + angle
    elif angle >= 0:
        invert = angle + pi
    invert = normalize_angle(invert)
    return invert

def normalize_angle(angle: float):
    """Convert an angle to a normalized angle. A normalized angle, for us, is
    in radians, -pi < angle < pi. 0 is at 3 o'clock, positive is counterclockwise"""
    angle_out = angle
    if angle_out < 0:
        while abs(angle_out) > pi:
            angle_out += pi
    elif angle_out > 0:
        while angle_out > pi:
            angle_out = -2 * pi + angle_out
    return angle_out

def poses_close(p1: Pose2D, p2: Pose2D) -> bool:
    """Returns true if two poses are close to each other and pointing in
    a similar direction (to be defined)"""
    d = calc_distance(p1, p2)
    print(f"p1: {pose_to_str(p1)} p2: {pose_to_str(p2)}, d:{d:1.2}")
    return d < 0.1 and isclose(p1.theta, p2.theta, abs_tol = 0.2)


def offset_point(point: Pose2D, distance: float) -> Pose2D:
    """Given a Pose, compute a new Pose which is @distance away from the first one
    in the direction of the theta of the pose"""
    new_point = Pose2D()
    new_point.theta = point.theta
    new_point.x = point.x + distance * cos(point.theta)
    new_point.y = point.y + distance * sin(point.theta)
    return new_point


# Logging and   debugging  functions
def info(msg: str):
    """Simple logging utility function"""
    #rospy.loginfo(msg)
    print(msg)

def pose_to_str(p: Pose2D):
    """Convert Pose fields to a string for debugging and logging"""
    return f"x:{p.x:1.2} y:{p.y:1.2} t:{p.theta:1.2}"

def turn_to_target(yaw, x0, y0, x1, y1):
    """ Current position is x0, y0. Current orientation is angle yaw, where 0 is North and positive angles go counterclockwise. 
    Target destination is x1, y1. Compute needed turn to point directly at x1, y1."""
    delta_x = x1 - x0
    delta_y = y1 - y0
# Calculate the angle between the current orientation and the target point using the arctangent function: θ = atan2(Δy, Δx).
    angle_to_target = atan2(delta_y, delta_x)
# Calculate the angle difference between the target angle and the current angle: Δθ = θ - A.
    turn_amount = angle_to_target - yaw
    return turn_amount

# Control functions
# Wait for the simulator to be ready. If simulator is not ready, then time will be stuck at zero

def wait_for_simulator():
    """ When running with Gazebo, this method will sleep until it detects that Gazebo is up """
    rate = rospy.Rate(5)
    while rospy.Time.now().to_sec() == 0:
        rate.sleep()

# Very simpl(istic) and easy to understand PID implementation
class PID:
    def __init__(self, min_val, max_val, kp, ki, kd):
        self.min_val = min_val
        self.max_val = max_val
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        self.derivative = error - self.prev_error
        if setpoint == 0 and self.prev_error == 0:
            self.integral = 0
        pid = self.kp * error + self.ki * self.integral + self.kd * self.derivative
        self.prev_error = error
        # r = np.clip(pid, self.min_val, self.max_val)
        # print(f"** PID {pid:1.2} {error:1.2} {r:1.2}")
        return np.clip(pid, self.min_val, self.max_val)

def sigmoid(error, k, scale):
    sig = 1 / (1 + exp(-k * error))
    return sig * scale

if __name__ == "__main__":
    print("test cases")
    assert invert_angle(0) == pi
    assert invert_angle(0.5) == -(pi - 0.5)
    assert invert_angle(-0.1) == pi - 0.1
    assert normalize_angle(0.0) == 0
    assert normalize_angle(-0.1) == -0.1
    assert normalize_angle(1.5) == 1.5
    assert normalize_angle(pi + 0.1) == -(pi - 0.1)
    
    A = Pose2D(-4,2,0.0)
    B = Pose2D(1,3,0.0)
    C = Pose2D(2,-1,0.0)
    D = Pose2D(-3,-1,0.0)
    assert isclose(calc_distance(A,B), 5.1, abs_tol=0.02)
    assert isclose(calc_distance(A,D), 3.162, abs_tol=0.02)
    assert isclose(calc_distance(D,B), 5.657, abs_tol=0.02)
    assert isclose(calc_distance(C,B), 4.123, abs_tol=0.02)
    assert calc_distance(C,B) == calc_distance(B, C)
    assert calc_distance(A,B) == calc_distance(B, A)
    assert calc_distance(A,D) == calc_distance(D,A)
    assert calc_distance(D,B) == calc_distance(B, D)
    assert calc_distance(A, A) == 0
    assert calc_distance(B, B) == 0
    assert calc_distance(C,C) == 0

    A = Pose2D(-4.0, 2.0, 1.0)
    pa = offset_point(A, 7.0)
    pb = Pose2D(-0.218, 7.89, 1.0)
    assert poses_close(pa, pb)

    C = Pose2D(3.0, -3.0, -1.0)
    pc = offset_point(C, 3.0)
    pd = Pose2D(4.621, -5.524, -1.0)
    assert poses_close(pc, pd)

    rotation_cases = [[0, 0, 0, 1, 0, 0],
                    [90, 0, 0, 1, 0, -90],
                    [180, 0, 0, -1, 0, 0],
                    [45, 0, 0, -1, 0, 135]]
    for case in rotation_cases:
        assert degrees(turn_to_target(radians(case[0]), case[1], case[2], case[3], case[4])) == case[5]
