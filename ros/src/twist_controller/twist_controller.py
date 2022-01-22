import numpy as np
import rospy

from geometry_msgs.msg import TwistStamped
from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

from utilities import *


class Controller(object):

    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,
                 decel_limit, accel_limit, wheel_radius, wheel_base,
                 steer_ratio, max_lat_accel, max_steer_angle):

        # Initialize the velocity filter
        self.filter = LowPassFilter(tau=TAU, ts=TS)

        self.longitudinal_controller = PID(kp=KP, ki=KI, kd=KD,
                                           mn=decel_limit, mx=accel_limit)
        # TODO: Implement Stanley controller
        # self.lateral_controller =
        self.yaw_controller = YawController(wheel_base=wheel_base,
                                            steer_ratio=steer_ratio,
                                            min_speed=MIN_SPEED,
                                            max_lat_accel=max_lat_accel,
                                            max_steer_angle=max_steer_angle)

        self.total_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()  # Initial time in float seconds
        self.logger = Logger()

    def control(self, goal_twist, current_twist):
        """ Determine the throttle, brake and steer commands.

        Parameters
        ----------
        goal_twist: TwistStamped
        current_twist: TwistStamped
        """

        self.logger.reset()

        # Extract speeds
        goal_linear_speed = goal_twist.twist.linear.x
        goal_angular_speed = goal_twist.twist.angular.z
        current_linear_speed = self.filter.filt(current_twist.twist.linear.x)

        # Compute the time step
        current_time = rospy.get_time()
        time_step = current_time - self.last_time
        self.last_time = current_time

        # Perform the longitudinal control step
        speed_error = goal_linear_speed - current_linear_speed
        throttle = self.longitudinal_controller.step(speed_error, time_step)
        # Perform the lateral control step
        steering_angle = self.yaw_controller.get_steering(goal_linear_speed,
                                                          goal_angular_speed,
                                                          current_linear_speed)

        # If the goal velocity is zero and the ego vehicle is nearly stopped,
        # keep it at halt
        if (np.isclose(goal_linear_speed, 0.)
                and current_linear_speed < MIN_SPEED):
            throttle = 0.
            brake = 700.  # [Nm], required to keep the vehicle at halt
            self.logger.info("State: Full stop")
        elif throttle > 0 and speed_error > 0:
            brake = 0.
            self.logger.info("State: Accelerate")
        else:
            throttle = 0.
            deceleration = max(speed_error, self.decel_limit)
            brake = self.compute_torque(abs(deceleration))
            self.logger.info("State: Decelerate")

        self.logger.info("Speed (goal|current|error):        "
                         "%4.2f | %4.2f | %6.3f",
                         goal_linear_speed, current_linear_speed, speed_error)
        self.logger.info("Command (throttle|brake|steering): "
                         "%4.2f | %4.2f | %6.3f\n",
                         throttle, brake, steering_angle)
        return throttle, brake, steering_angle

    def reset(self):
        """ Reset the controller.

        The integral term of the longitudinal PID controller will be set to
        zero. This may be necessary if the drive by wire mode is deactivated.
        """
        self.logger.warn("Drive by wire is set of. Reset the PID controller")
        self.longitudinal_controller.reset()

    def compute_torque(self, acceleration):
        """ Compute the torque for the given acceleration. """
        return acceleration * self.total_mass * self.wheel_radius
