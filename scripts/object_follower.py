#!/usr/bin/env python

import math
from dataclasses import dataclass
from typing import List, Generic, TypeVar

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


T = TypeVar('T')


@dataclass
class DataMessage(Generic[T]):
    data: T


@dataclass
class LaserScanType:
    header: str
    angle_min: float  # Start of the scan [rad]
    angle_max: float  # End of the scan [rad]
    angle_increment: float  # Resolution of the scan [rad]

    time_increment: float
    scan_time: float

    range_min: float  # minimum range value [m]
    range_max: float  # maximum range value [m]

    ranges: List[float]
    intensities: List[float]


NODE_NAME = "robot_control_node"
SIMULATION_SCAN_TOPIC = "base_scan"
REAL_SCAN_TOPIC = "scan"
SCAN_TOPIC =  REAL_SCAN_TOPIC
SAFE_REGION = 0.8  # 90 centimeters
frequency = 10
ANGULAR_SPEED = 0.7


# This class will receive the human orders to control a robot
class ObjectFollower:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # Publisher initialization
        print("Setting publisher...")
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        print("Publishers ok")
        print("Starting Node...")

        # Subscription stage
        rospy.Subscriber(SCAN_TOPIC, LaserScan, self.recognizer_cb)

        # Node setup
        rate = rospy.Rate(frequency)  # 1Hz
        print(f"Node initialized {frequency}hz")
        self.linear_x = 0
        self.angular_z = 0

        # Node program
        while not rospy.is_shutdown():
            rospy.loginfo(f"Working 1.8")
            self.__publish_vel(self.linear_x, self.angular_z)
            rate.sleep()

    # Callbacks
    def recognizer_cb(self, msg: LaserScanType) -> None:
        def filter_range(r: float) -> float:
            if r < msg.range_min or r > msg.range_max:
                return float('inf')
            return r

        # Ensure invalid values are removed
        filtered_ranges = [filter_range(r) for r in msg.ranges]

        # Calc robot values
        closest = min(filtered_ranges)
        index = filtered_ranges.index(closest)
        angle = self.__get_angle(index, msg.angle_min, msg.angle_increment)

        if closest == float('inf'):
            # Do not move the robot, no object is detected
            self.linear_x = 0
            self.angular_z = 0
            return

        if closest <= SAFE_REGION:
            self.linear_x = 0
            self.angular_z = self.__get_angular_speed(angle, ANGULAR_SPEED) if math.fabs(angle) > math.radians(5) else 0
        else:
            will_move = math.fabs(angle) <= math.radians(15) and self.linear_x == 0
            can_move = (math.fabs(angle) <= math.radians(30) and self.linear_x > 0) or will_move
            # The larger angle difference, the speed is lower
            # factor = (allowed_angle - min(math.fabs(angle), allowed_angle))/allowed_angle
            factor = ObjectFollower.invert_exponential(math.fabs(angle)) if can_move else 0
            self.angular_z = self.__get_angular_speed(angle, ANGULAR_SPEED)
            self.linear_x = 0.4 * factor

    def cleanup(self):
        self.linear_x = 0
        self.angular_z = 0
        self.__publish_vel(0, 0)

    # Movement functions
    @staticmethod
    def __get_angle(index: int, min_angle: float, angle_step: float) -> float:
        return min_angle + index*angle_step

    @staticmethod
    def __get_angular_speed(angle: float, speed: float = 0.3) -> float:
        positive_turn = math.pi + angle
        negative_turn = math.pi - angle
        rotate_to_right = positive_turn < negative_turn

        # Normalize the angle, when the difference is greater, then the factor is greater
        # Linear relation
        # factor = 1 - (max_angle_move - min(math.fabs(angle), max_angle_move))/max_angle_move
        factor = ObjectFollower.direct_exponential(math.fabs(angle))

        # Get speed
        target_speed = factor*speed
        if target_speed > 0:
            target_speed = max(0.15, target_speed)

        return -1*target_speed if rotate_to_right else target_speed

    @staticmethod
    def direct_exponential(delta: float):
        # if the difference is 0, then it is zero
        return 1 - math.exp(-delta) # The min delta is zero

    @staticmethod
    def invert_exponential(delta: float):
        # if the difference is 0, then it is one
        return 1 - ObjectFollower.direct_exponential(delta)

    # Helper to simplify the robot communication
    def __publish_vel(self, linear_x: float, angular_z: float):
        message = (
            Vector3(x=linear_x, y=0, z=0),  # Linear part
            Vector3(x=0, y=0, z=angular_z),  # Angular part
        )
        rospy.loginfo(f"ang: {angular_z}")
        self.velocity_publisher.publish(*message)


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    try:
        ObjectFollower()
    except Exception as e:
        print(e)
        rospy.logfatal(f"{NODE_NAME} died")
