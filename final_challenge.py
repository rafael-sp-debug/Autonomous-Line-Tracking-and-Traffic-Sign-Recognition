import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32

class ClosedLoopControllerNode(Node):
    def __init__(self):
        super().__init__('final_challenge_controller')
        self.get_logger().info("Final controller started")

        # Suscripciones
        self.create_subscription(String, '/color', self.color_callback, 10)
        self.create_subscription(String, '/signal', self.signal_callback, 10)
        self.create_subscription(Float32, '/linear', self.linear_callback, 10)
        self.create_subscription(Float32, '/angular', self.angular_callback, 10)
        self.create_subscription(Bool, '/detected_signal', self.detected_signal_callback, 10)

        # Publicador
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables de estado
        self.current_color = "GREEN"
        self.current_signal = ""
        self.signal_detected = False
        self.linear_velocity = 0.0
        self.angular_correction = 0.0
        self.state = "advance"
        self.motion_start_time = None
        self.reduction_duration = 0.0
        self.pending_state = None

        # Timer principal
        self.create_timer(0.1, self.state_machine)

    # Callbacks
    def color_callback(self, msg):
        self.current_color = msg.data.upper()

    def signal_callback(self, msg):
        self.current_signal = msg.data

    def linear_callback(self, msg):
        self.linear_velocity = msg.data

    def angular_callback(self, msg):
        self.angular_correction = msg.data

    def detected_signal_callback(self, msg):
        self.signal_detected = msg.data

    # Acciones
    def publish_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def advance(self):
        self.publish_twist(self.linear_velocity, self.angular_correction)

    def stop(self):
        self.publish_twist(0.0, 0.0)

    def reduction(self):
        self.publish_twist(self.linear_velocity * 0.5, self.angular_correction * 0.5)

    def turn(self, direction):
        
        angular = 0.5 if direction == "left" else -0.5
        self.publish_twist(0.1, angular)

    def ahead(self):
        self.publish_twist(0.15, 0.0)

    def state_machine(self):
        current_time = time.time()

        # ───── PARADA POR SEÑAL DE TRÁFICO "STOP" ─────
        if self.current_signal == "stop" and self.signal_detected:
            self.motion_start_time = current_time
            if current_time - self.motion_start_time < 10:
                self.stop()
                self.get_logger().info("State: stop")
                return
            else:
                self.state = "stop"
                self.motion_start_time = None

        # ───── PARADA POR SEMÁFORO EN ROJO ─────
        if self.current_color == "RED":
            if self.state != "stop":
                self.pending_state = self.state
                self.state = "stop"
            self.stop()
            self.get_logger().info("State: STOP (Red Light)")
            return

        # ───── RETOMAR ESTADO PREVIO SI LUZ VERDE Y NO HAY "STOP" ─────
        if self.pending_state and self.current_color == "GREEN" and self.current_signal != "stop":
            self.state = self.pending_state
            self.pending_state = None
            self.motion_start_time = current_time

        # ───── BLOQUEO TEMPORAL PARA GIROS ─────
        if self.state in ["turn_left", "turn_right"] and self.motion_start_time:
            if current_time - self.motion_start_time < 3.0:
                self.turn("left" if self.state == "turn_left" else "right")
                self.get_logger().info(f"State: {self.state.upper()}")
                return
            else:
                self.state = "advance"
                self.motion_start_time = None

        elif self.state == "ahead" and self.motion_start_time:
            if current_time - self.motion_start_time < 4:
                self.ahead()
                self.get_logger().info("State: AHEAD")
                return
            else:
                self.state = "advance"
                self.motion_start_time = None

        # ───── LÓGICA DE TRANSICIÓN ─────
        if self.signal_detected:
            if self.current_signal == "turn_left":
                self.state = "turn_right"
                self.motion_start_time = current_time

            elif self.current_signal == "turn_right":
                self.state = "turn_right"
                self.motion_start_time = current_time

            elif self.current_signal == "go_straight":
                self.state = "ahead"
                self.motion_start_time = current_time

            elif self.current_signal == "give_way" or self.current_signal ==  "roadwork_ahead":
                self.state = "reduction"
                self.motion_start_time = current_time
                self.reduction_duration = 3

        elif self.current_color == "YELLOW":
            self.state = "advance"
            self.motion_start_time = current_time
            self.reduction_duration = 1

        elif self.current_color == "GREEN" and not self.signal_detected:
            self.state = "advance"

        elif self.current_color == "GREEN" and self.current_signal == "stop" and self.signal_detected:
            self.state = "stop"
            self.stop()

        # ───── LÓGICA DE EJECUCIÓN ─────
        if self.state == "advance":
            self.advance()
            self.get_logger().info("State: ADVANCE")

        elif self.state == "reduction":
            if self.motion_start_time and current_time - self.motion_start_time < self.reduction_duration:
                self.reduction()
                self.get_logger().info("State: REDUCTION")
            else:
                self.state = "advance"
                self.motion_start_time = None

def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()