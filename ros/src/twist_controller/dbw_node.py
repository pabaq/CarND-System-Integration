#!/usr/bin/env python
import rospy

from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from twist_controller import Controller

from utilities import *

'''
You can build this node only after you have built (or partially built) the 
`waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear 
and angular velocities. You can subscribe to any other message that you find 
important or refer to the document for list of messages subscribed to by the 
reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` 
class is the status of `dbw_enabled`. While in the simulator, its enabled all 
the time, in the real car, that will not be the case. This may cause your PID 
controller to accumulate error because the car could temporarily be driven by a 
human instead of your controller.

We have provided two launch files with this node. Vehicle specific values 
(like vehicle_mass, wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and 
other utility classes. You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the 
various publishers that we have created in the `__init__` function.
'''

PUBLISHING_FREQUENCY = 50  # [Hz]


class DBWNode(object):
    def __init__(self):
        # Initialize the /dbw_node (drive by wire)
        rospy.init_node('dbw_node')

        # Load parameters
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)  # [kg]
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)  # [gal]
        brake_deadband = rospy.get_param('~brake_deadband', .1)  # []

        # TODO:
        # decel_limit = rospy.get_param('~decel_limit', -5)  # [m/s**2]
        decel_limit = MAX_DECEL

        accel_limit = rospy.get_param('~accel_limit', 1.)  # [m/s**2]
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)  # [m]
        wheel_base = rospy.get_param('~wheel_base', 2.8498)  # [m]
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)  # [-]
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)  # [m/s**2]
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)  # [deg]

        # Initialize the /vehicle publishers, which will provide the steering,
        # throttle and brake commands
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Initialize the controller
        self.controller = Controller(vehicle_mass=vehicle_mass,
                                     fuel_capacity=fuel_capacity,
                                     brake_deadband=brake_deadband,
                                     decel_limit=decel_limit,
                                     accel_limit=accel_limit,
                                     wheel_radius=wheel_radius,
                                     wheel_base=wheel_base,
                                     steer_ratio=steer_ratio,
                                     max_lat_accel=max_lat_accel,
                                     max_steer_angle=max_steer_angle)

        # Subscribe to the required topics:
        # - the status of the manual switch in the simulator
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        # - the current twist of the ego vehicle
        rospy.Subscriber('/current_velocity', TwistStamped,
                         self.current_velocity_cb)
        # - the goal twist for the ego vehicle
        rospy.Subscriber('/twist_cmd', TwistStamped, self.goal_twist_cb)

        # Initialize the attributes which will hold the callback messages
        self.dbw_enabled = None  # type: bool | None
        self.current_twist = None  # type: TwistStamped | None
        self.goal_twist = None  # type: TwistStamped | None

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
        return enabled and current_twist and goal_twist

    def loop(self):
        """ Publish throttle, brake and steering commands. """
        rate = rospy.Rate(PUBLISHING_FREQUENCY)  # Publishing rate
        while not rospy.is_shutdown():
            self.logger.reset()
            # The controller will only publish data if it is completely
            # initialized and the drive by wire mode is enabled
            if self.drive_by_wire_enabled:
                commands = self.controller.control(self.goal_twist,
                                                   self.current_twist)
                self.publish(*commands)
            rate.sleep()

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
