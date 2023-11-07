from flojoy import flojoy, Scalar, TextBlob
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Publisher(Node):

    # TODO: change topic name and node name to a block param
    # TODO: support other message types
    def __init__(self):
        super().__init__('flojoy_pub')
        self.publisher_ = self.create_publisher(String, 'flojoy_topic', 10)
        self.publisher_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def publish_msg(self, msg_str="no message"):
        msg = String()
        msg.data = msg_str
        self.publisher_.publish(msg)
        return TextBlob(msg.data)

    def publish_vel(self):
        msg = Twist()
        # msg.linear.x = 1.0
        # msg.linear.y = 2.0
        msg.angular.z = 6.0
        self.publisher_vel.publish(msg)
        return TextBlob(str([msg.linear.x,
                             msg.linear.y,
                             msg.linear.z,
                             msg.angular.x,
                             msg.angular.y,
                             msg.angular.z]))


@flojoy
def ROS_PUBLISHER(
    x: Scalar,
) -> TextBlob:
    if not rclpy.ok():
        rclpy.init()

    ros_pub = Publisher()
    ros_pub.publish_msg("tst tst")
    out = ros_pub.publish_vel()

    rclpy.spin_once(ros_pub, timeout_sec=1)

    ros_pub.destroy_node()
    rclpy.shutdown()

    return out or TextBlob( x.c * 3 )

