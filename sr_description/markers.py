import rclpy

import numpy as np
from asyncio            import Future
from rclpy.node         import Node
from visualization_msgs.msg import Marker
import keys


def build_piano_key(tstamp, px, py):
    # Build up a command message and publish.
    marker = Marker()
    marker.header.stamp = tstamp
    marker.header.frame_id = "world"
    marker.type = Marker.CUBE
    marker.scale.x = keys.SX
    marker.scale.y = keys.SY
    marker.scale.z = keys.SZ
    marker.pose.position.x = px
    marker.pose.position.y = py
    marker.pose.position.z = -marker.scale.z/2
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.lifetime = rclpy.duration.Duration(seconds=80000).to_msg()

    return marker

class MarkerNode(Node):
    # Initialization.
    def __init__(self, name, rate):
        # Initialize the node, naming it as specified
        super().__init__(name)


        # Add a publisher to send the joint commands.
        self.pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /visualization_marker subscriber...")
        while(not self.count_subscribers('/visualization_marker')):
            pass

        # Create a future object to signal when the trajectory ends,
        # i.e. no longer returns useful data.
        self.future = Future()

        # Create a timer to keep calculating/sending commands.
        self.starttime = self.get_clock().now()
        self.servotime = self.starttime
        self.timer     = self.create_timer(1/float(rate), self.update)

        self.t  = 0.0
        self.dt = self.timer.timer_period_ns * 1e-9
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))



    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()


    # Spin
    def spin(self):
        # Keep running (taking care of the timer callbacks and message
        # passing), until interrupted or the trajectory is complete
        # (as signaled by the future object).
        rclpy.spin_until_future_complete(self, self.future)

        # Report the reason for shutting down.
        if self.future.done():
            self.get_logger().info("Stopping: " + self.future.result())
        else:
            self.get_logger().info("Stopping: Interrupted")


    # Update - send a new marker command every time step.
    def update(self):
        now = self.get_clock().now()


        # Build up a command message and publish.
        for (i, key) in enumerate(keys.KEYS):
            marker = build_piano_key(now.to_msg(), key.bb.x, key.bb.z)
            marker.id = i-1
            self.pub.publish(marker)
        #for i in range(10):
        #   marker = build_piano_key(now.to_msg(), 1.0 + (0.025 + 0.001) * i, 1.0)
        #   marker.id = i-1
        #   self.pub.publish(marker)


# TODO: This is really inefficient. We don't need to be constantly re-sending the markers.
# But for now it gets the job done.
if __name__ == "__main__":
    rclpy.init(args=None)
    mnode = MarkerNode('marker_generator', 1)

    # Spin, until interrupted or the trajectory ends.
    mnode.spin()

    # Shutdown the node and ROS.
    mnode.shutdown()
    rclpy.shutdown()

