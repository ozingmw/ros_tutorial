import rospy
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

class visual:
    def __init__(self) -> None:
        rospy.init_node('visual', anonymous=True)

        cam_sub = rospy.Subscriber('image', Image, self.callback_visual_cam)
        pose_sub = rospy.Subscriber('pose', Marker, self.callback_visual_pose)

        self.pose = None

    def callback_visual_cam(self, data):
        cv_bridge = CvBridge()
        cv_img = cv_bridge.imgmsg_to_cv2(data)

        if self.pose:
            for points in self.pose.points:
                x = points.x
                y = points.y
                z = points.z
                
                cv2.circle(cv_img, (int(cv_img.shape[1]*x), int(cv_img.shape[0]*y)), 5, (0, 255, 0), -1)

        cv2.imshow('img', cv_img)
        cv2.waitKey(1)

        rospy.loginfo(f'img frame_id: {data.header.frame_id}')

    def callback_visual_pose(self, data):
        self.pose = data

        rospy.loginfo(f'pose frame_id: {data.header.frame_id}')

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    v = visual()
    v.run()
