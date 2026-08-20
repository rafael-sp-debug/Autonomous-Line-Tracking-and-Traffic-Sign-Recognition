import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Int32  # AÑADIDO: para evitar error en Int32

class CameraNode(Node):
    def __init__(self):
        super().__init__('puzzelbot_camera')
        # self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.image = self.create_subscription(Image, '/yolo', self.obtener_imagen, 10)
        self.angle_pub = self.create_publisher(Int8, 'angle', 10)
        self.error_pub = self.create_publisher(Int8, 'error', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.capture_and_publish)
        self.cap = None
      
    def obtener_imagen(self, msg):
        self.cap = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.cap = cv2.flip(self.cap, 0)

    def capture_and_publish(self):
        if self.cap is None:
            self.get_logger().warn("Aún no se ha recibido ninguna imagen.")  #F
            return

        frame = self.cap.copy()
        height, width = frame.shape[:2]

        roi_start_y = int(height * 0.7)
        roi_start_x = int(width * 0.1)
        roi_end_x = int(width * 0.9)
        roi = frame[roi_start_y:height, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, Blackline = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        kernel = np.ones((3, 3), np.uint8)
        Blackline = cv2.morphologyEx(Blackline, cv2.MORPH_CLOSE, kernel, iterations=4)
        contours_blk, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        error = 0
        ang = 0

        if len(contours_blk) > 0:
            largest_contour = max(contours_blk, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:
                blackbox = cv2.minAreaRect(largest_contour)
                (x_min, y_min), (w_min, h_min), ang = blackbox

                # Corrección del ángulo
                if ang < -45:
                    ang = 90 + ang
                if w_min < h_min and ang > 0:
                    ang = (90 - ang) * -1
                if w_min > h_min and ang < 0:
                    ang = 90 + ang

                ang = int(ang)
                setpoint = roi.shape[1] // 2
                error = int(x_min - setpoint)

                box = cv2.boxPoints(blackbox)
                box = np.intp(box)
                cv2.drawContours(roi, [box], 0, (0, 0, 255), 3)  #F
                cv2.putText(roi, f"Angulo: {ang}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)  #F
                cv2.putText(roi, f"Error: {error}", (10, roi.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)  #F
                cv2.line(roi, (int(x_min), roi.shape[0]-50), (int(x_min), roi.shape[0]), (255, 0, 0), 3)  #F
                
                self.angle_pub.publish(Int32(data=ang))  #F
                self.error_pub.publish(Int32(data=error))  #F
                cv2.imshow("hola", roi)  #F
                
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self.get_logger().info("Cierre solicitado con 'q'.")  #F
                    rclpy.shutdown()  #F

    def destroy_node(self):
        cv2.destroyAllWindows()  #F
        super().destroy_node()  #F

def main(args=None):
    rclpy.init(args=args)  #F
    node = CameraNode()
    try:
        rclpy.spin(node)  #F
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido manualmente.")  #F
    finally:
        node.destroy_node()  
        rclpy.shutdown()  

if __name__ == '__main__':
    main()
