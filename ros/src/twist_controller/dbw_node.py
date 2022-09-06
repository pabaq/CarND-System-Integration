#!/usr/bin/env python
import numpy as np

import rospy

from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane
from twist_controller import Controller

from utilities import *


# Overwrite max steering angle
MAX_STEER_ANGLE = np.deg2rad(30)  # [rad]


class DBWNode(object):
    def __init__(self):
        # Initialize the /dbw_node (drive by wire)
        rospy.init_node('dbw_node')

        # Subscribe to the required topics, which are
        # - the reference waypoints to be followed by the ego vehicle
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)
        # - the current pose of the ego vehicle
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        # - the status of the manual switch in the simulator
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        # - the current twist of the ego vehicle
        rospy.Subscriber('/current_velocity', TwistStamped,
                         self.current_velocity_cb)
        # - the goal twist for the ego vehicle
        rospy.Subscriber('/twist_cmd', TwistStamped, self.goal_twist_cb)

        # Initialize the attributes which will hold the callback messages
        self.tree = None  # type: WaypointTree | None
        self.current_pose = None  # type: PoseStamped | None
        self.dbw_enabled = None  # type: bool | None
        self.current_twist = None  # type: TwistStamped | None
        self.goal_twist = None  # type: TwistStamped | None

        # Initialize the /vehicle publishers, which will provide the steering,
        # throttle and brake commands
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Load parameters
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)  # [kg]
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)  # [gal]
        accel_limit = rospy.get_param('~accel_limit', 1.)  # [m/s**2]
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)  # [m]
        wheel_base = rospy.get_param('~wheel_base', 2.8498)  # [m]
        decel_limit = rospy.get_param('~decel_limit', -5)  # [m/s**2]
        max_steer_angle = MAX_STEER_ANGLE
        # max_steer_angle = rospy.get_param('~max_steer_angle', 8.)  # [deg]
        # max_steer_angle = np.deg2rad(max_steer_angle)  # [rad]
        # brake_deadband = rospy.get_param('~brake_deadband', .1)  # []
        # steer_ratio = rospy.get_param('~steer_ratio', 14.8)  # [-]
        # max_lat_accel = rospy.get_param('~max_lat_accel', 3.)  # [m/s**2]

        # Initialize the controller
        self.controller = Controller(vehicle_mass=vehicle_mass,
                                     fuel_capacity=fuel_capacity,
                                     accel_limit=accel_limit,
                                     decel_limit=decel_limit,
                                     wheel_radius=wheel_radius,
                                     wheel_base=wheel_base,
                                     max_steer_angle=max_steer_angle)

        # Initialize the node's logger
        self.logger = Logger()

        # Enter the publishing loop
        self.loop()

    @property
    def drive_by_wire_enabled(self):
        """ Flag indicating the controller's activity state. """
        enabled = self.dbw_enabled is True
        current_twist = self.current_twist is not None
        goal_twist = self.goal_twist is not None
        current_pose = self.current_pose is not None
        tree = self.tree is not None
        return all([enabled, current_twist, goal_twist, current_pose, tree])

    def loop(self):
        """ Publish throttle, brake and steering commands. """
        rate = rospy.Rate(PUBLISHING_FREQUENCY)  # Publishing rate
        while not rospy.is_shutdown():
            self.logger.reset()
            # The controller will only publish data if it is completely
            # initialized and the drive by wire mode is enabled
            if self.drive_by_wire_enabled:
                commands = self.controller.control(self.current_pose,
                                                   self.goal_twist,
                                                   self.current_twist,
                                                   self.tree)
                self.publish(*commands)
            rate.sleep()

    def base_waypoints_cb(self, reference_waypoints):
        """ Initialize the reference waypoints' WaypointTree.

        Args:
            reference_waypoints (Lane)
        """

        # Since the /waypoint_loader node keeps publishing the same reference
        # waypoints all the time, they need to be stored only once.
        if self.tree is None:
            self.tree = WaypointTree(reference_waypoints)

    def current_pose_cb(self, current_pose):
        """ Store the current pose of the ego vehicle.

        Args:
            current_pose (PoseStamped)
        """
        self.current_pose = current_pose

    def dbw_enabled_cb(self, dbw_enabled):
        """ Store the current status of the manual switch in the simulator.

        Args:
            dbw_enabled (Bool)
        """
        self.dbw_enabled = dbw_enabled.data
        # Reset the PID controller's terms if drive by wire is set off
        if not self.dbw_enabled:
            self.controller.reset()
        else:
            self.logger.warn("Drive by wire actived")
            # Initial time if drive by wire is set on
            self.controller.last_time = rospy.get_time()

    def current_velocity_cb(self, current_twist):
        """ Store the current velocity of the ego vehicle.

        Args:
            current_twist (TwistStamped)
        """
        self.current_twist = current_twist

    def goal_twist_cb(self, goal_twist):
        """ Store the goal velocity for the ego vehicle.

        Args:
            goal_twist (TwistStamped)
        """
        self.goal_twist = goal_twist

    def publish(self, throttle, brake, steer):
        """ Publish the throttle, brake and steering commands.

        Args:
            throttle (float): throttle value (should be between 0 and 1)
            brake (float): brake value in units of torque [Nm]
            steer (float): steering wheel angle
        """

        throttle_cmd = ThrottleCmd()
        throttle_cmd.enable = True
        throttle_cmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        throttle_cmd.pedal_cmd = throttle
        self.throttle_pub.publish(throttle_cmd)

        steering_cmd = SteeringCmd()
        steering_cmd.enable = True
        steering_cmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(steering_cmd)

        brake_cmd = BrakeCmd()
        brake_cmd.enable = True
        brake_cmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        brake_cmd.pedal_cmd = brake
        self.brake_pub.publish(brake_cmd)


if __name__ == '__main__':
    DBWNode()
