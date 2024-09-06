import rospy
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


class pose:
    def __init__(self) -> None:
        rospy.init_node('pose', anonymous=True)

        img_list_sub = rospy.Subscriber('img_list', Image, self.callback_img_list, queue_size=1, buff_size=2**32)
        self.pose_list_pub = rospy.Publisher('pose_marker', Marker, queue_size=10)

        base_options = python.BaseOptions(model_asset_path='./src/webcam/scripts/pose_landmarker.task')
        options = vision.PoseLandmarkerOptions(base_options=base_options,output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(options)

    def callback_img_list(self, data):
        cv_bridge = CvBridge()

        cv_img = cv_bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        pose_list = self.get_pose_list(cv_img)
        pose_marker = self.landmarker_to_marker(pose_list)

        pose_marker.header = data.header

        self.pose_list_pub.publish(pose_marker)

    def run(self):
        rospy.spin()

    def get_pose_list(self, cv_img):
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.array(cv_img))
        pose_landmarker_result = self.detector.detect(image)

        return pose_landmarker_result

    def landmarker_to_marker(self, landmarks):
        marker = Marker()
        landmarks = landmarks.pose_landmarks
        
        for idx in range(len(landmarks)):
            pose_landmarks = landmarks[idx]
            
            for lm in pose_landmarks:
                marker.points.append(Point(lm.x, lm.y, lm.z))

        return marker
            

if __name__ == '__main__':
    p = pose()
    p.run()