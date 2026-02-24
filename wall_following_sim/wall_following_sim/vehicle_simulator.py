import math
import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped

D2R = math.pi / 180.0
R2D = 1.0 / D2R

class VehicleSimulator(Node):
    def __init__(self):
        super().__init__("vehicle_simulator")

        self.declare_parameter("starting_position", [0.0, 0.0, 0.0])
        self.declare_parameter("starting_ang_deg", 70.0)
        self.declare_parameter("L_wheelbase_m", 0.33)

        self.subscription_twist = self.create_subscription(
            Twist, "vehicle_command_twist", self.vehicle_command_twist_callback, 1
        )

        self.subscription_angle = self.create_subscription(
            AckermannDriveStamped, "vehicle_command_ackermann", self.vehicle_command_ackermann_callback, 1
        )

        self.publisher = self.create_publisher(PoseStamped, "vehicle_pose", 10)

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        self.timer = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)

        param_value = self.get_parameter("starting_position").value

        self.old_position = np.array([0.0, 0.0, 0.0])
        self.old_position[0] = param_value[0]
        self.old_position[1] = param_value[1]
        self.old_position[2] = param_value[2]

        self.old_heading_rad = D2R * self.get_parameter("starting_ang_deg").value
        self.L_wheelbase_m = self.get_parameter("L_wheelbase_m").value

        self.rad_of_curvature = 99999.0
        self.speed = 0.0
        self.cnt = 0

        self.steering_angle_rad = 0.0
        self.throttle_effort = 0.0

        self.input_type = 0  # 0 is undefined; 1 is twist input ; 2 is effort (angle) input

    def vehicle_command_twist_callback(self, msg):

        self.input_type = 1

        if math.isclose(msg.angular.z, 0.0, abs_tol=0.0001) or math.isclose(msg.linear.x, 0.0, abs_tol=0.001):
            self.rad_of_curvature = 999999.9
        else:
            self.rad_of_curvature = msg.linear.x / msg.angular.z

        self.speed = msg.linear.x
        self.cnt += 1

    def vehicle_command_ackermann_callback(self, msg):

        self.input_type = 2

        self.steering_angle_rad = msg.drive.steering_angle
        self.throttle_effort = (msg.drive.speed/6.35) *100
        self.cnt += 1

    def timer_callback(self):
        # PN - Is this here because the parameter might not be updated by the time a control is received?
        # CC - At the start, the initial 'old_heading_rad' must be  set to some value
        if self.cnt < 1:
            self.old_heading_rad = D2R * self.get_parameter("starting_ang_deg").value

        if self.input_type == 2:
            # get radius of curvature and speed from VehCmd message info
            if np.isclose(self.steering_angle_rad, 0.0, atol=1.0e-5):
                self.rad_of_curvature = 1.0e+5
            else:
                self.rad_of_curvature = self.L_wheelbase_m / math.tan(self.steering_angle_rad)
            self.speed = self.throttle_effort * 4.0 / 100.0  # max speed is 4 m/sec

        new_position, new_heading_rad = update_vehicle_pose(
            self.old_position, self.old_heading_rad, self.rad_of_curvature, self.speed / 10.0
        )

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "utm"
        msg.pose.position.x = new_position[0]
        msg.pose.position.y = new_position[1]
        msg.pose.position.z = new_position[2]
        msg.pose.orientation.w = math.cos(new_heading_rad / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(new_heading_rad / 2.0)

        self.publisher.publish(msg)

        # send out the tf2 for the vehicle pose
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = "utm"
        t.child_frame_id = "vehicle"

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        # Send the transformation
        self.br.sendTransform(t)

        self.old_position = new_position
        self.old_heading_rad = new_heading_rad


def main(args=None):
    rclpy.init(args=args)

    vehicle_simulator = VehicleSimulator()

    rclpy.spin(vehicle_simulator)

    vehicle_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


#########################################################################
def update_vehicle_pose(old_position, old_heading_rad, rad_of_curvature, speed):
# outputs - new_position, np.array of floats
#           new_heading_rad, rad_of_curvature, speed, float
  
  if (abs(rad_of_curvature) >= 150.0):
    Svec = np.array([math.cos(old_heading_rad), math.sin(old_heading_rad), 0.0])
    new_position = old_position + speed * Svec
    new_heading_rad = old_heading_rad
  else:
    rear_axle_dir_rad = old_heading_rad + 90.0*D2R
    center_pt = np.array([old_position[0] + rad_of_curvature * math.cos(rear_axle_dir_rad), \
                          old_position[1] + rad_of_curvature * math.sin(rear_axle_dir_rad), \
                          0.0])
    direction = np.sign(rad_of_curvature)    # positive is a left hand turn

    Svec = old_position - center_pt  # vector from center point to old vehicle position
    radius = np.linalg.norm(Svec)
    rotation_ang_rad = speed / radius  # speed represents dist to trave in this time step

    new_heading_rad = old_heading_rad + (direction * rotation_ang_rad)

    if (direction == 1):  # left turn
      temp = np.array([Svec[0] *math.cos(rotation_ang_rad) - Svec[1]*math.sin(rotation_ang_rad), \
                        Svec[0] *math.sin(rotation_ang_rad) + Svec[1]*math.cos(rotation_ang_rad), \
                        0.0])
      new_position = center_pt + temp
    elif (direction == -1):  # right turn
      temp = np.array([Svec[0] *math.cos(-rotation_ang_rad) - Svec[1]*math.sin(-rotation_ang_rad), \
                        Svec[0] *math.sin(-rotation_ang_rad) + Svec[1]*math.cos(-rotation_ang_rad), \
                        0.0])
      new_position = center_pt + temp

  return [new_position, new_heading_rad]

######################################################################### 