import rospy
import cv2

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

class controller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        img_sub = rospy.Subscriber('image', Image, self.callback_cam)

        self.img_list_pub = rospy.Publisher('img_list', Image, queue_size=1)
        pose_sub = rospy.Subscriber('pose_marker', Marker, self.callback_pose)

        self.visual_pub = rospy.Publisher('pose', Marker, queue_size=10)

    def callback_cam(self, data):
        self.img_list_pub.publish(data)

    def callback_pose(self, data):
        self.visual_pub.publish(data)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    c = controller()
    c.run()