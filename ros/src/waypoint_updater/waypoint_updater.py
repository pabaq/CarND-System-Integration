#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint

from utilities import *

'''
This node will publish waypoints from the car's current position to some `x` 
distance ahead.

As mentioned in the doc, you should ideally first implement a version which does 
not care about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of 
traffic lights too.

Please note that our simulator also provides the exact location of traffic 
lights and their current status in `/vehicle/traffic_lights` message. You can 
use this message to build this node as well as to verify your TL classifier.
'''


class WaypointUpdater(object):
    """ Publish waypoints from car's current position to some distance ahead """

    def __init__(self):
        # Initialize the /waypoint_updater node
        rospy.init_node('waypoint_updater')
        # Subscribe to the required topics, which are
        # - the reference waypoints to be followed by the ego vehicle
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)
        # - the current pose of the ego vehicle
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        # - the reference waypoint index closest to upcoming red traffic light
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_waypoint_cb)
        # Current twist of the ego vehicle
        # rospy.Subscriber('/current_velocity', TwistStamped,
        #                  self.current_velocity_cb, queue_size=1)

        # Initialize the /final_waypoints publisher, which will provide the next
        # NUM_LOOKAHEAD reference waypoints in front of the ego vehicle at
        # PUBLISHING_FREQUENCY
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane,
                                                   queue_size=1)

        # WaypointTree instance used for some work in the upcoming methods
        self.tree = None  # type: WaypointTree | None
        # the ego vehicle's current pose
        self.current_pose = None  # type: PoseStamped | None
        # the reference waypoint index closest to upcoming red light's stopline
        self.stop_idx = UNKNOWN  # type: int
        # Initialize the node's logger
        self.logger = Logger()

        # Enter the publishing loop
        self.loop()

    def loop(self):
        """ Publish filtered reference waypoints to the /final_waypoints topic.

        The NUM_LOOKAHEAD reference waypoints in front of the ego vehicle's
        current position are published at PUBLISHING_FREQUENCY.
        """

        rate = rospy.Rate(PUBLISHING_FREQUENCY)  # Publishing rate
        while not rospy.is_shutdown():
            # Wait with publishing until the reference waypoints tree was
            # initialized and the ego vehicle's current pose was loaded in the
            # corresponding callbacks (base_waypoints_cb, current_pose_cb)
            if self.tree and self.current_pose:
                self.publish_waypoints()
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

    def traffic_waypoint_cb(self, waypoint_index):
        """ Store the waypoint index of the next red traffic light's stopline.

        Args:
            waypoint_index (Int32)
        """
        self.stop_idx = waypoint_index.data

    # Not required in this implementation
    def obstacle_cb(self, msg):
        pass

    def get_closest_reference_waypoint_idx(self):
        """ Get the closest waypoint index in front of the ego vehicle. """

        # Determine the closest reference waypoint to the ego vehicle
        ego_wp_idx = self.tree.get_closest_idx_from(self.current_pose)
        # Define vectors for the xy-coordinates of the closest waypoint, its
        # predecessor, and the ego vehicle's current position
        closest_xy = np.array(self.tree.xy[ego_wp_idx])
        previous_xy = np.array(self.tree.xy[ego_wp_idx - 1])
        ego_xy = np.array(get_position_from(self.current_pose)[:UNKNOWN])
        # Define a vector connecting the reference trajectory waypoints
        reference_trajectory_vector = closest_xy - previous_xy
        # Define a vector connecting the ego vehicle's current position with the
        # closest reference waypoint
        ego_vector = ego_xy - closest_xy
        # Check if the ego vehicle is ahead or behind the closest waypoint
        # by taking the dot product
        dot_product = np.dot(reference_trajectory_vector, ego_vector)
        # If the ego vehicle is ahead of the closest waypoint, take the next
        # waypoint on the reference trajectory
        if dot_product > 0:
            ego_wp_idx = (ego_wp_idx + 1) % self.tree.num_waypoints
        return ego_wp_idx

    def decelerate(self, ego_idx):
        """ Decelerate ego vehicle from its current position to the stopline.

        Args:
            ego_idx (int):
                index of closest reference waypoint in front of the ego vehicle
        """
        # Initialize the updated waypoint's list
        updated_waypoints = []
        # The waypoints' linear speed will be updated up to the lookahead dist
        lookahead_idx = ego_idx + NUM_LOOKAHEAD_PUB
        # The ego vehicle's linear speed shall be zero at the light's stopline
        stop_idx = self.stop_idx - NUM_STOPLINE_THRES

        if self.stop_idx != UNKNOWN:
            self.logger.info('Stopline (idx|pos): %19i | %7.2f %7.2f',
                             stop_idx, *self.tree.xy[stop_idx])

        if ego_idx < stop_idx:
            # Get the cumulated distances from the start to the stop waypoint
            dist_to_stop = self.tree.get_cumulated_dists_from(ego_idx, stop_idx)
            # Update the waypoints from the current position to the stopline
            for i, current_waypoint in enumerate(self.tree[ego_idx: stop_idx]):
                # Decrease the speed by a root function
                speed = np.sqrt(4 * abs(MAX_DECEL) * dist_to_stop[i])
                if speed < 1.:
                    speed = 0
                # Update the current waypoint's linear speed and store it
                updated_waypoint = set_linear_speed_of(current_waypoint, speed)
                updated_waypoints.append(updated_waypoint)
        else:
            stop_idx = ego_idx

        # Update the remaining waypoints
        for i, current_waypoint in enumerate(self.tree[stop_idx:lookahead_idx]):
            # Set linear speed of waypoints behind the traffic light to zero
            updated_waypoint = set_linear_speed_of(current_waypoint, 0.)
            updated_waypoints.append(updated_waypoint)
        return updated_waypoints

    def publish_waypoints(self):
        """ Publish next NUM_LOOKAHEAD reference waypoints to be followed. """

        self.logger.reset()
        # Index of the closest reference waypoint in front of the ego vehicle
        ego_idx = self.get_closest_reference_waypoint_idx()

        self.logger.info('Ego vehicle (idx|pos): %16i | %7.2f %7.2f',
                         ego_idx, *self.tree.xy[ego_idx])

        # Index of the waypoint in lookahead distance
        lookahead_idx = ego_idx + NUM_LOOKAHEAD_PUB
        # The waypoints are published using a Lane message
        lane = Lane()
        lane.header = self.tree.header
        # If there is no traffic light ahead publish the reference waypoints
        if self.stop_idx == UNKNOWN or self.stop_idx >= lookahead_idx:
            lane.waypoints = self.tree[ego_idx: lookahead_idx]
        else:
            lane.waypoints = self.decelerate(ego_idx)
        self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
