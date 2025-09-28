#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn

class LawnMower(Node):
    def __init__(self):
        super().__init__('lawnmower_controller')

        # service clients: kill default turtle and spawn rpr_turtle
        self.kill_cli = self.create_client(Kill, 'kill')
        self.spawn_cli = self.create_client(Spawn, 'spawn')

        self.kill_req = Kill.Request()
        self.kill_req.name = 'turtle1'

        self.spawn_req = Spawn.Request()
        self.spawn_req.name = 'rpr_turtle'
        self.spawn_req.x = 2.0
        self.spawn_req.y = 1.0
        self.spawn_req.theta = 0.0

        # wait for services
        while not self.kill_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kill service...')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        # spawn sequence: kill then spawn
        f = self.kill_cli.call_async(self.kill_req)
        f.add_done_callback(self._after_kill)

        # publisher/subscriber created after spawn
        self.cmd_pub = None
        self.pose_sub = None

        # parameters for lawnmower
        self.x_min = 2.0
        self.x_max = 9.0
        self.row_step = 1.0            # vertical spacing between passes
        self.r = self.row_step / 2.0   # semicircle radius -> vertical step = 2*r = row_step
        self.turn_speed = 1.6          # angular speed during turn (rad/s)
        self.forward_speed = 2.0       # forward linear speed during horizontal pass
        self.top_limit = 10.0          # y coordinate to stop

        # state
        self.mode = 'idle'             # 'idle', 'forward', 'turn'
        self.dir_right = True          # True = moving right (increasing x), False = moving left
        self.turn_start_theta = None
        self.turn_sign = None          # +1 for CCW, -1 for CW
        self.turn_target = math.pi     # radians to rotate during semicircle

    def _after_kill(self, _):
        self.get_logger().info('turtle1 killed, spawning rpr_turtle...')
        f2 = self.spawn_cli.call_async(self.spawn_req)
        f2.add_done_callback(self._after_spawn)

    def _after_spawn(self, _):
        self.get_logger().info('rpr_turtle spawned at (%.2f, %.2f)' %
                               (self.spawn_req.x, self.spawn_req.y))
        # now create publisher and subscriber
        self.cmd_pub = self.create_publisher(Twist, '/rpr_turtle/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/rpr_turtle/pose',
                                                 self.pose_callback, 10)
        # start in forward mode
        self.mode = 'forward'

    # helper: normalize angle to [-pi, pi]
    @staticmethod
    def _normalize(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # compute how much angle (positive) has been turned from start to current
    def _turned_angle(self, start, current, sign):
        # diff = current - start normalized to [-pi,pi]
        diff = self._normalize(current - start)
        if sign == 1:
            # ccw desired: if diff < 0, add 2*pi to get positive rotation amount
            if diff < 0:
                diff += 2 * math.pi
        else:
            # cw desired (sign == -1): if diff > 0, subtract 2*pi to get negative rotation
            if diff > 0:
                diff -= 2 * math.pi
            diff = -diff  # make positive
        return abs(diff)

    def start_turn(self, pose):
        # start a semicircle turn that reverses the heading
        self.mode = 'turn'
        self.turn_start_theta = pose.theta
        # if moving right (heading ~ 0), we need CCW (+) semicircle to go north->west
        # if moving left (heading ~ pi), we need CW (-) semicircle to go north->east
        self.turn_sign = 1 if self.dir_right else -1
        self.get_logger().info(f'Starting turn sign={self.turn_sign} at theta={pose.theta:.2f}')

    def pose_callback(self, pose: Pose):
        if self.cmd_pub is None:
            return  # not ready yet

        twist = Twist()

        # stop when we reached the top
        if pose.y >= self.top_limit:
            self.get_logger().info('Top reached: stopping.')
            self.cmd_pub.publish(twist)
            return

        if self.mode == 'forward':
            # normal horizontal forward motion
            if self.dir_right:
                if pose.x < self.x_max - 0.05:
                    twist.linear.x = self.forward_speed
                    twist.angular.z = 0.0
                else:
                    # reached right boundary -> begin semicircle turn
                    self.start_turn(pose)
                    # set turning linear and angular for this callback (will continue in turn mode)
                    twist.linear.x = self.r * self.turn_speed
                    twist.angular.z = self.turn_sign * self.turn_speed
            else:
                # moving left
                if pose.x > self.x_min + 0.05:
                    twist.linear.x = self.forward_speed
                    twist.angular.z = 0.0
                else:
                    # reached left boundary -> begin semicircle turn
                    self.start_turn(pose)
                    twist.linear.x = self.r * self.turn_speed
                    twist.angular.z = self.turn_sign * self.turn_speed

        elif self.mode == 'turn':
            # compute how much we have turned so far (positive)
            turned = self._turned_angle(self.turn_start_theta, pose.theta, self.turn_sign)
            if turned < self.turn_target - 0.03:
                # keep turning along semicircle (linear = r * omega)
                twist.linear.x = self.r * self.turn_speed
                twist.angular.z = self.turn_sign * self.turn_speed
            else:
                # finished semicircle: flip direction and return to forward mode
                self.mode = 'forward'
                self.dir_right = not self.dir_right
                # after semicircle we have moved up approx 2*r (row_step)
                self.get_logger().info(f'Finished turn. New direction: {"right" if self.dir_right else "left"}')
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        else:
            # idle fallback
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)
        # small debug log (comment out if noisy)
        # self.get_logger().debug(f'pose: x={pose.x:.2f} y={pose.y:.2f} th={pose.theta:.2f} mode={self.mode}')

def main(args=None):
    rclpy.init(args=args)
    node = LawnMower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

