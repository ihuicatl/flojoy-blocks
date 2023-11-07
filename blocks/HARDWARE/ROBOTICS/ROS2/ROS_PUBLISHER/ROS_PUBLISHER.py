from flojoy import flojoy, Scalar, TextBlob
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TurtleSimPublisher(Node):

    def __init__(self):
        super().__init__('flojoy_pub')
        self.publisher_ = self.create_publisher(String, 'flojoy_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.msg_blob = TextBlob("empty")

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        self.msg_blob = TextBlob(msg.data)

        if self.i > 10:
            self.destroy_node()
            rclpy.shutdown()

@flojoy
def ROS_PUBLISHER(
    x: Scalar,
) -> TextBlob:
    if not rclpy.ok():
        rclpy.init()

    turtle_pub = TurtleSimPublisher()

    rclpy.spin(turtle_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    # rclpy.shutdown()

    return turtle_pub.msg_blob or TextBlob( x.c * 3 )

