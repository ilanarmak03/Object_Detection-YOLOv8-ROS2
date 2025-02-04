import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        if not hasattr(self, 'yolo_model'):
            self.yolo_model = YOLO('yolov8n.pt')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image...')
        frame = self.bridge.imgmsg_to_cv2(msg)
        edges = cv2.Canny(frame, 100, 200)  # Perform edge detection
        cv2.imshow("Edges", edges)  # Display the edge-detected image
        #---------------------------------------------------
        # Perform inference
        results = self.yolo_model(frame)
        # Process results
        for result in results:
            for box in result.boxes:  # Bounding box predictions
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()  # Bounding box coordinates (moving the tensor to cpu first)
                conf = box.conf[0].cpu().numpy()           # Confidence score
                cls_id = int(box.cls[0].cpu().numpy())     # Class ID
                label = f"{self.yolo_model.names[cls_id]} {conf:.2f}"

                # Draw bounding box and label
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        #---------------------------------------------------
        # net = cv2.dnn.readNetFromCaffe('deploy.prototxt', 'mobilenet_iter_73000.caffemodel')
        # blob = cv2.dnn.blobFromImage(frame, 1, (300, 300), (104, 117, 123), False, crop=False)
        # net.setInput(blob)
        # detections = net.forward()

        # # Process detections (you can filter and draw bounding boxes on detected objects)
        # for i in range(detections.shape[2]):
        #     confidence = detections[0, 0, i, 2]
        #     if confidence > 0.5:  # Confidence threshold
        #         box = detections[0, 0, i, 3:7] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
        #         (x, y, x2, y2) = box.astype("int")
        #         cv2.rectangle(frame, (x, y), (x2, y2), (0, 255, 0), 2)
        # cv2.imshow("Object Detection", frame)
        #---------------------------------------------------
        cv2.imshow("YOLOv8 Object Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()