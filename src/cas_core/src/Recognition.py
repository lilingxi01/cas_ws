import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
from torchvision import transforms
import math

bridge = CvBridge()
depth_map = np.empty((0,))

# Define color image callback function
def color_image_callback(msg):
    global depth_map
    try:
        # Convert ROS depth image message to OpenCV image
        model = YOLO("yolov8n.pt")
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = model(cv_image)
        for box in results[0].boxes:
            x = int(box.xywh[0][0])
            y = int(box.xywh[0][1])
            depth_val = depth_map[y][x] + 0.18
            obj_angle = ((x/640)*87)-43.5
            object_coor = calculate_coordintes(depth_val, obj_angle)
            print("Object Coordinates")
            print(object_coor)

        # cv2.imshow("Image", results[0].plot())
        #cv2.imwrite("op.jpg", results[0].plot())
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(e)

def depth_image_callback(msg):
    global depth_map
    l = np.frombuffer(msg.data, dtype=np.uint8)
    depth_image = l.reshape(-1, 640, 4)
    depth_image_bytes = depth_image.view(dtype=np.uint8).reshape(depth_image.shape + (-1,))
    depth_map = np.frombuffer(depth_image_bytes, dtype=np.float32).reshape(depth_image.shape[:2])

def calculate_coordintes(depth_val, obj_angle):
    neg = False
    a = depth_val
    A = math.radians(90)
    B = math.radians(obj_angle)
    C = math.radians(90 - obj_angle)
    b = (a * math.sin(B)) / math.sin(A)
    c = (a * math.sin(C)) / math.sin(A)
    return (b, c)

def main():
    rospy.init_node('yolov8_pose_estimation')

    # Create a subscriber for the depth image
    rospy.Subscriber('/camera/depth/image_raw', Image, depth_image_callback)
    rospy.sleep(1)
    plt.imshow(depth_map, cmap='gray')
    plt.colorbar()
    plt.savefig('depth_image.png')
    rospy.Subscriber('/camera/color/image_raw', Image, color_image_callback)
    pose_publisher = rospy.Publisher('/object_poses', Pose, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()
