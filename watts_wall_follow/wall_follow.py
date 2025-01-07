# Wall following controller that maintains a distance of 1 meter from the wall on the right side of the vehicle
# Is enabled by pressing the 'A' button on the Xbox controller

import rclpy
import math
from rclpy.node import Node

from sensor_msgs.msg import Joy, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

MAX_STEERING_ANGLE = math.pi / 4
MAX_SPEED = 3.0 # Temporary value until we can get the actual max speed
NEGATIVE_Y_DIRECTION = -90 * math.pi/180 # Angle pointing to the right side of the vehicle in radians 

class wallFollowController(Node):
    def __init__(self):
        super().__init__('wall_follow_controller')
        self.subscription = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.subscription = self.create_subscription(LaserScan, "scan", self.laser_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, "vehicle_command_ackermann", 1)
        self.wall_follow_enabled = False
        self.last_error = 0.0
        self.integral = 0.0

    def joy_callback(self, msg):
        if msg.buttons[0] == True:
            self.wall_follow_enabled = True

        elif msg.buttons[0] == False:
            self.wall_follow_enabled = False

    def laser_callback(self, msg):
        # If wall following is not enabled, stop the car
        if self.wall_follow_enabled == False:
            ackermann_msg = AckermannDriveStamped()
            ackermann_msg.drive.steering_angle = 0.0
            ackermann_msg.drive.speed = 0.0
            self.publisher.publish(ackermann_msg)
            return

        # Create points from ranges
        for i, range in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if math.isclose(angle, NEGATIVE_Y_DIRECTION, abs_tol=0.005):
                d1 = range
                x1 = range * math.cos(angle)
                y2 = range * math.sin(angle)

            elif math.isclose(angle, NEGATIVE_Y_DIRECTION + msg.angle_increment, abs_tol=0.005):
                d2 = range
                x2 = range * math.cos(angle)
                y2 = range * math.sin(angle)

        d3 = math.sqrt(d1**2 + d2**2 - 2 * d1 * d2 * math.cos(msg.angle_increment))

        cos_angle = (x2 - x1) / d3
        d_wall = d1 * cos_angle

        error = d_wall - 1.0

        # non PID control
        Kp = 2.0  # Proportional gain
        Ki = 0.0  # Integral gain
        Kd = 0.6  # Derivative gain

        # Calculate the error terms
        proportional = error
        self.integral += error
        derivative = error - self.last_error

        # Calculate the control output
        steering_angle = Kp*proportional + Ki*self.integral + Kd*derivative

        # Limit the steering angle to max values
        steering_angle = max(min(steering_angle, MAX_STEERING_ANGLE), -MAX_STEERING_ANGLE)

        # Remember the last error for next time
        self.last_error = error

        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.drive.steering_angle = -steering_angle
        ackermann_msg.drive.speed = 1.0
        self.publisher.publish(ackermann_msg)

def main(args=None):
    rclpy.init(args=args)

    wall_follow_controller = wallFollowController()

    rclpy.spin(wall_follow_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
