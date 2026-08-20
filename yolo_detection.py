import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')

        self.model = YOLO('/home/jaaziel/ros2_ws/src/puzzlebot_line_follower/puzzlebot_line_follower/best_2.pt')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, '/yolo', self.image_callback, 10)
        self.pub_detected = self.create_publisher(Bool, '/detected_signal', 10)
        self.pub_signal = self.create_publisher(String, '/signal', 10)
        self.pub_color = self.create_publisher(String, '/color', 10)
        self.pub_image = self.create_publisher(Image, '/detection_result', 10)

        self.traffic_sign_classes = ["stop", "go_straight", "turn_left", "turn_right", "give_way", "roadwork_ahead"]
        self.traffic_light_classes = ["red", "yellow", "green"]

        self.min_sign_area = 3300
        self.max_sign_area = 3600
        self.min_light_area = 700
        self.max_light_area = 2000  

        self.last_color = "unknown"
        self.timer = self.create_timer(0.1, self.publish_color)

        self.current_light_color = "unknown"
        self.start_detection_time = None
        self.color_confirmed = False

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (400, 300))

        results = self.model(frame)[0]

        detected_signal = False
        detected_class = ""
        new_color_detected = False

        for box in results.boxes:
            cls_id = int(box.cls[0])
            class_name = self.model.names[cls_id]
            conf = float(box.conf[0])
            if conf < 0.2:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            area = (x2 - x1) * (y2 - y1)
            print(f"[INFO] Clase detectada: {class_name}, Área: {area:.2f} px")

            roi = frame[y1:y2, x1:x2]

            
            if class_name in self.traffic_sign_classes:
                if area < self.min_sign_area or area > self.max_sign_area:
                    print(f"[INFO] Señal ignorada por área fuera de rango ({area:.2f} < {self.min_sign_area})")
                    continue

                if class_name in ["turn_left", "turn_right"]:
                    corrected_class = self.refine_turn_direction(roi)
                    if corrected_class:
                        class_name = corrected_class

                detected_signal = True
                detected_class = class_name

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'{class_name} {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            
            elif class_name in self.traffic_light_classes:
                if area < self.min_light_area or area > self.max_light_area:
                    print(f"[INFO] Semáforo ignorado por área fuera de rango ({area:.2f} < {self.min_light_area})")
                    continue

                current_time = time.time()
                if class_name == self.current_light_color:
                    
                    if self.start_detection_time is not None:
                        elapsed_time = current_time - self.start_detection_time
                        if elapsed_time >= 0.0 and not self.color_confirmed:
                            self.last_color = class_name
                            self.color_confirmed = True
                            
                else:
                    
                    self.current_light_color = class_name
                    self.start_detection_time = current_time
                    self.color_confirmed = False

                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f'{class_name} {conf:.2f}', (x1, y2 + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        
        self.pub_detected.publish(Bool(data=detected_signal))
        self.pub_signal.publish(String(data=detected_class if detected_signal else ""))
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

    def publish_color(self):
        
        self.pub_color.publish(String(data=self.last_color))

    def refine_turn_direction(self, roi):
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_contour = max(contours, key=cv2.contourArea, default=None)
        if largest_contour is None or cv2.contourArea(largest_contour) < 100:
            return None

        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        center_x = roi.shape[1] // 2
        return "turn_left" if cx < center_x else "turn_right"

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()