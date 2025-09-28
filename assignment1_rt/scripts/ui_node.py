#!/usr/bin/env python3
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from assignment1_rt.msg import TurtleInfo
import time

class TurtleUI(Node):
    def __init__(self):
        super().__init__('turtle_ui')

        # Publishers for turtle1 and turtle2
        self.pub_t1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_t2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Publisher for custom message
        self.info_pub = self.create_publisher(TurtleInfo, 'turtle_info', 10)

        # Spawn turtle2 first
        self.spawn_turtle2()

        # Now subscribe to turtle2 pose
        self.create_subscription(Pose, '/turtle2/pose', self.pose_callback, 10)

        # Publish TurtleInfo periodically
        self.create_timer(0.1, self.publish_turtle_info)

        self.turtle2_pose = None

    def spawn_turtle2(self):
        client = self.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        req = Spawn.Request()
        req.x = 5.0
        req.y = 5.0
        req.theta = 0.0
        req.name = 'turtle2'
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Spawned {future.result().name}')
        else:
            self.get_logger().error('Failed to spawn turtle2')

    def pose_callback(self, msg: Pose):
        self.turtle2_pose = msg

    def publish_turtle_info(self):
        if self.turtle2_pose is None:
            return
        info = TurtleInfo()
        info.name = 'turtle2'
        info.x = self.turtle2_pose.x
        info.y = self.turtle2_pose.y
        info.theta = self.turtle2_pose.theta
        self.info_pub.publish(info)

    def run_interface(self):
        while rclpy.ok():
            choice = input("Select turtle (1 or 2): ").strip()
            if choice not in ['1', '2']:
                print("Invalid choice")
                continue
            try:
                lin = float(input("Enter linear velocity: "))
                ang = float(input("Enter angular velocity: "))
            except ValueError:
                print("Invalid input")
                continue
            twist = Twist()
            twist.linear.x = lin
            twist.angular.z = ang
            pub = self.pub_t1 if choice == '1' else self.pub_t2
            start = time.time()
            while time.time() - start < 1.0:
                pub.publish(twist)
                time.sleep(0.1)
            pub.publish(Twist())
            print("Command finished, turtle stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleUI()

    # Run rclpy.spin in a separate thread so timers and subscriptions work
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_interface()  # Handles user input in the main thread
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

