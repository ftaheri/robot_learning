
import rclpy
from rclpy.node import Node
import os
from sensor_msgs.msg import Image
from cas726_interfaces.srv import DetectObjects
from cas726_interfaces.msg import BoundingBox

from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images


import torch
import numpy


 
from torchvision.io.image import read_image
from torchvision.models.detection import fasterrcnn_resnet50_fpn_v2, FasterRCNN_ResNet50_FPN_V2_Weights
from torchvision.utils import draw_bounding_boxes
from torchvision.transforms.functional import to_pil_image
import torchvision.transforms as transforms

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('object_detector')
        self.get_logger().info('received mess')

        print("Creating service")
        self.detect_obj_srv = self.create_service(
            DetectObjects,
            'object_detector/detect',
            self.detect_callback)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Step 1: Initialize model with the best available weights
        self.weights = FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT
        self.model = fasterrcnn_resnet50_fpn_v2(weights=self.weights, box_score_thresh=0.9)
        self.model.eval()
        # Step 2: Initialize the inference transforms
        self.preprocess = self.weights.transforms()
        print("Creating service --> done")
        

    def detect_callback(self, request, response):
        print("Processing image")

#########################################################################
        ######## comment for task1
        self.get_logger().info('Got image in frame: "%s"' % request.color.header.frame_id)
#########################################################################



        #TODO: implement this function.
        # 1. convert request image to a tensor. Go via the opencv bridge and then convert to tensor
#########################################################################
        ########## change for task1
        cv_image = self.br.imgmsg_to_cv2(request.color, desired_encoding="passthrough")
        pil_image = to_pil_image(cv_image)
        tensor_img = self.preprocess(pil_image)
        # cv_image = read_image("/workspace/images/image5.jpg")
        # tensor_img = self.preprocess(cv_image)
#########################################################################


        # 2. create a minibatch with just one image and call self.model to do inference
        minibatch = [tensor_img]
        prediction = self.model(minibatch) [0]

        labels = [self.weights.meta["categories"][i] for i in prediction["labels"]]
        print("prediction: ", prediction)
        print("labels: ",labels)
        print("bounding boxes: ", prediction["boxes"])
        print("scores: ",prediction["scores"])
        

#########################################################################
        ##########change for task1
        # box = draw_bounding_boxes(cv_image, boxes=prediction["boxes"],
        #                   labels=labels,
        #                   colors="red",
        #                   width=4,font="/workspace/images/FreeMonoBold.ttf" , font_size = 50)

        box = draw_bounding_boxes(tensor_img.to(torch.uint8), boxes=prediction["boxes"],
                          labels=labels,
                          colors="red",
                          width=4,font="/workspace/images/FreeMonoBold.ttf" , font_size = 50)
#########################################################################
      
      
      
        im = to_pil_image(box.detach())
        im.save("/workspace/images/imageFromScene.jpg")

        # 3. iterate through predictions and copy data into the response message
        for box, label, score in zip(prediction["boxes"], prediction["labels"], prediction["scores"]):
            if score < 0.9:  # skip low confidence detections
                continue
            bbox = BoundingBox()
            bbox.label = self.weights.meta["categories"][label]
            bbox.x_min = round(float(box[0]))
            
            bbox.y_min = round(float(box[1]))
            bbox.x_max = round(float(box[2]))
            bbox.y_max = round(float(box[3]))

#########################################################################
            ############ comment for task1
            response.detections.append(bbox)
#########################################################################


        print("Done")
        return response


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

#########################################################################
    #########uncomment for task1
    # image_subscriber.detect_callback(None,None)
#########################################################################



    rclpy.spin(image_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
