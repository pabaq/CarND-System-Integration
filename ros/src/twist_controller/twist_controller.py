import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from lowpass import LowPassFilter
from pid import PID
from stanley import Stanley
from yaw_controller import YawController

from utilities import *

# Parameters for the low pass filter
TAU = 0.5
TS = 0.02

# Parameters for the longitudinal controllers
KP_THROTTLE = 0.3  # PID gains for the throttle controller
KI_THROTTLE = 0.1
KD_THROTTLE = 0.0
THROTTLE_GAINS = KP_THROTTLE, KI_THROTTLE, KD_THROTTLE
KP_BRAKE = 0.2  # PID gains for the brake controller
KI_BRAKE = 0.0
KD_BRAKE = 0.1
BRAKE_GAINS = KP_BRAKE, KI_BRAKE, KD_BRAKE
GAS_DENSITY = 2.858  # [kg/gal]

# Parameters for the lateral controller
K = 10  # Stanley gains
K_SOFT = 0.1

# Arbitrary parameters
MIN_SPEED = 0.1  # [m/s]


class Controller(object):

    def __init__(self, vehicle_mass, fuel_capacity, accel_limit, decel_limit,
                 wheel_radius, wheel_base, max_steer_angle):
        """ Initialize the controller.

        Args:
            vehicle_mass: vehicle mass in [kg]
            fuel_capacity: fuel capocity in [gal]
            accel_limit: maximum longitudinal acceleration [m/s**2]
            decel_limit: maximum longitudinal deceleration [m/s**2]
            wheel_radius: wheel radius in [m]
            wheel_base: distance of front and rear axis in [m]
            max_steer_angle: maximum steering angle in [rad]
        """

        # Initialize the velocity filter
        self.filter = LowPassFilter(tau=TAU, ts=TS)
        # Initialize the longitudinal throtte and brake controllers
        self.throtte_controller = PID(*THROTTLE_GAINS, mn=0, mx=accel_limit)
        self.brake_controller = PID(*BRAKE_GAINS, mn=decel_limit, mx=0)
        # Initialize the lateral controller
        self.yaw_controller = Stanley(wheel_base=wheel_base,
                                      max_steering_angle=max_steer_angle,
                                      k=K, k_soft=K_SOFT)

        self.total_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()  # Initial time in float seconds
        self.logger = Logger()

    def control(self, current_pose, goal_twist, current_twist, tree):
        """ Determine the throttle, brake and steer commands.

        Parameters
        ----------
        current_pose: PoseStamped
        goal_twist: TwistStamped
        current_twist: TwistStamped
        tree: WaypointTree
        """

        self.logger.reset()

        # Compute the time step
        current_time = rospy.get_time()
        time_step = current_time - self.last_time
        self.last_time = current_time

        # Get the goal linear speed
        goal_speed = goal_twist.twist.linear.x
        # Use filter to remove noise from the vehicle velocity
        current_speed = self.filter.filt(current_twist.twist.linear.x)

        # Perform the lateral control step
        steering_angle = self.yaw_controller.step(current_pose,
                                                  current_speed,
                                                  tree)

        # Perform the longitudinal control step
        speed_error = goal_speed - current_speed
        # If the speed error is positive, accelerate
        if speed_error > 0:
            self.logger.info("State: Accelerate")
            throttle = self.throtte_controller.step(speed_error, time_step)
            self.brake_controller.reset()
            brake = 0.
        # If the speed error is negative, decelerate
        else:
            self.logger.info("State: Decelerate")
            deceleration = self.brake_controller.step(speed_error, time_step)
            brake = self.compute_torque(abs(deceleration))
            self.throtte_controller.reset()
            throttle = 0.

        # If the goal velocity is zero and the ego vehicle is nearly stopped,
        # keep it at halt
        if np.isclose(goal_speed, 0.) and current_speed < MIN_SPEED:
            self.logger.info("State: Full stop")
            throttle = 0.
            brake = 700.  # [Nm], required to keep the vehicle at halt

        self.logger.info("Speed (goal|current|error):        "
                         "%6.2f  %6.2f  %+6.3f",
                         goal_speed, current_speed, speed_error)
        self.logger.info("Command (throttle|brake|steering): "
                         "%6.2f  %6.2f  %+6.3f\n",
                         throttle, brake, steering_angle)
        return throttle, brake, steering_angle

    def reset(self):
        """ Reset the controller.

        The integral term of the longitudinal PID controller will be set to
        zero. This may be necessary if the drive by wire mode is deactivated.
        """
        self.logger.warn("Drive by wire is set of. Reset the PID controller")
        self.throtte_controller.reset()
        self.brake_controller.reset()

    def compute_torque(self, acceleration):
        """ Compute the torque for the given acceleration. """
        return acceleration * self.total_mass * self.wheel_radius
