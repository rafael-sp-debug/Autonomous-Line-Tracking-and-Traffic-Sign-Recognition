import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32

class ClosedLoopControllerNode(Node):
    def __init__(self):
        super().__init__('final_challenge_controller')
        self.get_logger().info("Final controller is started lol")

        self.subscription = self.create_subscription(Int16, '/error', self.pid_callback, 1)
        self.angle_subscription = self.create_subscription(Int16, '/angle', self.angle_callback, 1)

        # Publicación
        self.pub_lin = self.create_publisher(Float32, '/linear', 1)
        self.pub_ang = self.create_publisher(Float32, '/angular', 1)
        #self.pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # Parámetros PID
        self.Kp = 0.5
        self.Ki = 0.0
        self.Kd = 0.0
        self.Ktheta = 0.7

        # Estado del controlador
        self.integral = 0.0
        self.previous_error = 0.0
        self.angle = 0.0
        self.error = 0.0
        self.previous_time = time.time()

        # Límites
        self.max_linear_speed = 0.14
        self.max_angular_speed = 0.1

        self.linear_velocity = 0.0
        self.angular_correction = 0.0

        self.create_timer(0.1, self.PID)

    def pid_callback(self,msg):
        self.error = msg.data

    def angle_callback(self,msg):
        self.angle = msg.data


    def PID(self):
        self.get_logger().info(f"erorr:{self.error}, anguo: {self.angle}")
        current_time = time.time()
        dt = current_time - self.previous_time
        if dt <= 0.0:
            return

        angle_rad = math.radians(self.angle)

        self.integral += self.error * dt
        derivative = (self.error - self.previous_error) / dt

        max_lin_speed = self.max_linear_speed
        max_ang_speed = self.max_angular_speed

        if abs(angle_rad) > 0.3 and abs(self.error) > 30:
            max_ang_speed *= 4.2
            max_lin_speed *= 1.5
        elif abs(angle_rad) < 0.3 and abs(self.error) < 100: # 0.3 #100 
            max_lin_speed *= 0.5

        output = -self.Kp * self.error + self.Ki * self.integral + self.Kd * derivative + self.Ktheta * angle_rad
        self.angular_correction = max(min(-output, max_ang_speed), -max_ang_speed)

        if abs(self.error) > 200:
            lin_velocity = 0.05
        else:
            lin_velocity = 0.07 if abs(angle_rad) > 0.3 else max_lin_speed

        if abs(self.error) < 35 and abs(angle_rad) < 0.3:
            self.error = 0.0

        self.linear_velocity = max(min(lin_velocity, max_lin_speed), -max_lin_speed)

        self.previous_error = self.error
        self.previous_time = current_time

        self.pub_lin.publish(Float32(data=self.linear_velocity))
        self.pub_ang.publish(Float32(data=self.angular_correction))

def main(args=None):
    rclpy.init(args=args)
    nodeh = ClosedLoopControllerNode()
    try:
        rclpy.spin(nodeh)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Node terminated by user!")

if __name__ == '__main__':
    main()