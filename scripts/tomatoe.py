import numpy as np
import rospy
import requests
import time

from sensor_msgs.msg import Image
from nav2d_operator.msg import cmd

API_URL = "http://10.25.111.72:5000/classify"
PARAMS = {}


class Identify():
    """
        Class that will subscribe to the camera topic and will send a request trough an API running
        on a LocalHost to process the image and determine if there is a Tomato or not.
    """
    def __init__(self):
        rospy.on_shutdown(self.stop)
        self.mode = 0
        self.nav2d_publisher = rospy.Publisher('cmd', cmd, queue_size=2)
        self.subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self.process)
        rate = rospy.Rate(1)
        self.stop_rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.nav2d_publisher.publish(cmd(Velocity=0.8, Turn=0.0, Mode=self.mode))
            rate.sleep()

    def process(self, image):
        self.subscriber.unregister()
        img = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        # Send it to the API here
        PARAMS['Image'] = img.tolist()
        response = requests.get(API_URL, json=PARAMS)
        print(response.text)
        is_tomato = response.text == "Tomato"
        self.mode = 2 if is_tomato else 0
        self.nav2d_publisher.publish(cmd(Velocity=0.8, Turn=0.0, Mode=self.mode))
        self.subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self.process)

    def stop(self):
        self.nav2d_publisher.publish(cmd(Velocity=0.8, Turn=0.0, Mode=2))
        self.stop_rate.sleep()
        rospy.loginfo("Node Stopped")


if __name__ == "__main__":
    rospy.init_node("tomato_identification", anonymous=True)
    try:
        Identify()
    except Exception as e:
        rospy.logfatal("Couldn't Initialize the Tomato Identification Class")
        rospy.logfatal(e)