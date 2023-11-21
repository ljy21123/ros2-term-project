import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_term_project.GUI import GuiRun


class TimeSubscriber(Node):
    def __init__(self, g:GuiRun):
        super().__init__('time_subscriber')
        self.g = g
        g.run()
        self.subscription = self.create_subscription(
            String,
            '/PR001/driving_time',
            self.callback,
            10
        )
        self.subscription2 = self.create_subscription(
            String,
            '/PR002/driving_time',
            self.callback2,
            10
        )
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info('PR001 Received time: %s' % msg.data)
        self.g.update_gui_text(msg)

    def callback2(self, msg):
        self.get_logger().info('PR002 Received time: %s' % msg.data)
        self.g.update_gui_text2(msg)


def time_sub_run(args=None):
    rclpy.init(args=args)
    g = GuiRun()
    time_subscriber = TimeSubscriber(g)
    try:
        rclpy.spin(time_subscriber)
    except KeyboardInterrupt:
        pass
    time_subscriber.destroy_node()
    rclpy.shutdown()

