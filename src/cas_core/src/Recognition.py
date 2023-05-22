import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from geometry_msgs.msg import Pose
from torchvision import transforms

# Instantiate CvBridge
bridge = CvBridge()

# Define depth image callback function
def depth_image_callback(msg):
    try:
        # Convert ROS depth image message to OpenCV image
        model = YOLO("yolov8n.pt")
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # transform = transforms.Compose([
        #     transforms.ToPILImage(),
        #     transforms.Resize((256, 256)),
        #     transforms.ToTensor()
        # ])
        # image = transform(cv_image)
        # image = image.unsqueeze(0)
        results = model(cv_image)
        print(results)
        cv2.imshow("Image", results[0].plot())
        #cv2.imshow("Image", cv_image)
        cv2.waitKey(1)

        #model = YOLO("yolov8n-pose.pt")
#         model = YOLO('yolov8n-pose.yaml')  # build a new model from YAML
#         model = YOLO('yolov8n-pose.pt')  # load a pretrained model (recommended for training)
#         model = YOLO('yolov8n-pose.yaml').load('yolov8n-pose.pt')  # build from YAML and transfer weights
#
# # Train the model
#         model.train(data='coco8-pose.yaml', epochs=10, imgsz=640)
#         cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
#         results = model(cv_image)
#         print(results)
#         cv2.imshow("Image", results[0].plot())
#         cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(e)

def main():
    rospy.init_node('yolov8_pose_estimation')

    # Create a subscriber for the depth image
    rospy.Subscriber('/camera/color/image_raw', Image, depth_image_callback)


    # Create a publisher for the estimated poses
    pose_publisher = rospy.Publisher('/object_poses', Pose, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()
