import cv2
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def cam():
    rospy.init_node('pub_cam', anonymous=True)
    img_pub = rospy.Publisher('image', Image, queue_size=10)

    web_cam = cv2.VideoCapture(0)

    cv_bridge = CvBridge()

    rate = rospy.Rate(60)

    frame_id = 0

    while not rospy.is_shutdown():
        status, frame = web_cam.read()

        ros_img = cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        header = Header()
        header.frame_id = str(frame_id)
        ros_img.header = header

        img_pub.publish(ros_img)

        rate.sleep()
        frame_id += 1
        
    web_cam.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    cam()